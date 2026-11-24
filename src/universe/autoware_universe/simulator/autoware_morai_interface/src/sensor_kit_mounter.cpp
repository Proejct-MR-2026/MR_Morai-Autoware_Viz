// Copyright 2026 The Autoware Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// 센서킷 YAML 파싱 + 프레임명 -> MORAI SensorType 매핑 + 장착

#include "autoware/morai_interface/sensor_kit_mounter.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace autoware::morai_interface
{

namespace
{
constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;

struct Pose
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double roll{0.0};
  double pitch{0.0};
  double yaw{0.0};
};

Pose read_pose(const YAML::Node & node)
{
  Pose p;
  p.x = node["x"].as<double>(0.0);
  p.y = node["y"].as<double>(0.0);
  p.z = node["z"].as<double>(0.0);
  p.roll = node["roll"].as<double>(0.0);
  p.pitch = node["pitch"].as<double>(0.0);
  p.yaw = node["yaw"].as<double>(0.0);
  return p;
}

/// Compose base_link -> kit -> sensor.
///
/// Only the kit's yaw is applied to the sensor's translation. Sensor kits are mounted
/// level in practice (roll and pitch are 0 in every kit in this workspace), and a full
/// rotation composition here would be a quaternion library's job, not a sensor mounter's.
/// If a kit ever ships a non-zero roll/pitch, the caller is warned rather than silently
/// given a wrong pose.
Pose compose(const Pose & kit, const Pose & sensor)
{
  const double c = std::cos(kit.yaw);
  const double s = std::sin(kit.yaw);

  Pose out;
  out.x = kit.x + c * sensor.x - s * sensor.y;
  out.y = kit.y + s * sensor.x + c * sensor.y;
  out.z = kit.z + sensor.z;
  out.roll = kit.roll + sensor.roll;
  out.pitch = kit.pitch + sensor.pitch;
  out.yaw = kit.yaw + sensor.yaw;
  return out;
}

bool contains(const std::string & haystack, const char * needle)
{
  return haystack.find(needle) != std::string::npos;
}
}  // namespace

std::vector<SensorKitEntry> load_sensor_kit(const std::string & sensor_model)
{
  const std::string share =
    ament_index_cpp::get_package_share_directory(sensor_model + "_description");

  const YAML::Node sensors = YAML::LoadFile(share + "/config/sensors_calibration.yaml");
  const YAML::Node kit_file = YAML::LoadFile(share + "/config/sensor_kit_calibration.yaml");

  // base_link -> sensor_kit_base_link. Absent or empty means the kit sits at the origin.
  Pose kit;
  if (sensors["base_link"] && sensors["base_link"]["sensor_kit_base_link"]) {
    kit = read_pose(sensors["base_link"]["sensor_kit_base_link"]);
  }

  const YAML::Node kit_root = kit_file["sensor_kit_base_link"];
  if (!kit_root) {
    throw std::runtime_error("sensor_kit_calibration.yaml has no sensor_kit_base_link");
  }

  std::vector<SensorKitEntry> entries;
  for (const auto & it : kit_root) {
    const Pose composed = compose(kit, read_pose(it.second));

    SensorKitEntry e;
    e.frame = it.first.as<std::string>();
    e.x = composed.x;
    e.y = composed.y;
    e.z = composed.z;
    e.roll = composed.roll;
    e.pitch = composed.pitch;
    e.yaw = composed.yaw;
    entries.push_back(e);
  }
  return entries;
}

morai_sim_api::sensor::SensorType to_morai_sensor_type(const std::string & frame)
{
  std::string f = frame;
  std::transform(f.begin(), f.end(), f.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  // Frame naming follows the Autoware sensor kit convention (velodyne_top, tamagawa/imu_link,
  // gnss_link, camera0/camera_link, ...), so substring matching is what fits.
  if (
    contains(f, "lidar") || contains(f, "velodyne") || contains(f, "ouster") ||
    contains(f, "livox")) {
    return morai_sim_api::sensor::SENSOR_TYPE_LIDAR3D;
  }
  if (contains(f, "imu")) {
    return morai_sim_api::sensor::SENSOR_TYPE_IMU;
  }
  if (contains(f, "gnss") || contains(f, "gps")) {
    return morai_sim_api::sensor::SENSOR_TYPE_GPS;
  }
  if (contains(f, "camera") || contains(f, "traffic_light")) {
    return morai_sim_api::sensor::SENSOR_TYPE_CAMERA;
  }
  if (contains(f, "radar")) {
    return morai_sim_api::sensor::SENSOR_TYPE_RADAR;
  }
  if (contains(f, "ultrasonic") || contains(f, "sonar")) {
    return morai_sim_api::sensor::SENSOR_TYPE_ULTRASONIC;
  }
  return morai_sim_api::sensor::SENSOR_TYPE_UNSPECIFIED;
}

size_t mount_sensor_kit(
  rclcpp::Node & node, morai_grpc_vendor::SensorClient & client,
  const std::vector<SensorKitEntry> & entries)
{
  size_t mounted = 0;

  for (const auto & e : entries) {
    const auto type = to_morai_sensor_type(e.frame);
    if (type == morai_sim_api::sensor::SENSOR_TYPE_UNSPECIFIED) {
      RCLCPP_WARN(
        node.get_logger(), "Sensor kit frame '%s' has no MORAI counterpart; skipping",
        e.frame.c_str());
      continue;
    }

    morai_grpc_vendor::SensorMount mount;
    mount.type = type;
    mount.label = e.frame;
    mount.x = e.x;
    mount.y = e.y;
    mount.z = e.z;
    // MORAI takes degrees (common/type.proto); the kit YAML is in radians.
    mount.roll = e.roll * kRadToDeg;
    mount.pitch = e.pitch * kRadToDeg;
    mount.yaw = e.yaw * kRadToDeg;

    std::string error;
    if (!client.add_sensor(mount, &error)) {
      RCLCPP_ERROR(node.get_logger(), "%s", error.c_str());
      continue;
    }

    RCLCPP_INFO(
      node.get_logger(), "Mounted %s at (%.2f, %.2f, %.2f) m, yaw %.1f deg", e.frame.c_str(),
      mount.x, mount.y, mount.z, mount.yaw);
    ++mounted;
  }
  return mounted;
}

}  // namespace autoware::morai_interface
