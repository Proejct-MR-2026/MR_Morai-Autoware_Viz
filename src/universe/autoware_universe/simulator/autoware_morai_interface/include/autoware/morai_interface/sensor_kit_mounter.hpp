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

// Autoware 센서킷 캘리브레이션을 읽어 MORAI 에 센서를 그대로 장착한다

#ifndef AUTOWARE__MORAI_INTERFACE__SENSOR_KIT_MOUNTER_HPP_
#define AUTOWARE__MORAI_INTERFACE__SENSOR_KIT_MOUNTER_HPP_

#include <morai_grpc_vendor/sensor_client.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

namespace autoware::morai_interface
{

/// One sensor frame read out of a sensor kit, still in Autoware conventions.
struct SensorKitEntry
{
  std::string frame;  //!< e.g. "lidar", "imu", "gnss_link", "camera0/camera_link"

  double x{0.0};  //!< [m], relative to base_link
  double y{0.0};
  double z{0.0};

  double roll{0.0};  //!< [rad]
  double pitch{0.0};
  double yaw{0.0};
};

/// Read `<sensor_model>_description`'s calibration YAMLs and compose base_link-relative
/// poses for every sensor frame.
///
/// Reads both files the sensor kit convention defines:
///   sensors_calibration.yaml     base_link -> sensor_kit_base_link
///   sensor_kit_calibration.yaml  sensor_kit_base_link -> each sensor
/// and composes them, so the result is what MORAI needs (poses relative to the vehicle).
///
/// Throws std::runtime_error if the package or either file cannot be read — the caller
/// treats that as "do not mount anything" rather than crashing the node.
std::vector<SensorKitEntry> load_sensor_kit(const std::string & sensor_model);

/// Map an Autoware sensor frame name to a MORAI SensorType.
///
/// Returns SENSOR_TYPE_UNSPECIFIED for frames with no MORAI counterpart; the caller
/// skips those rather than mounting something wrong.
morai_sim_api::sensor::SensorType to_morai_sensor_type(const std::string & frame);

/// Mount every recognised sensor of `sensor_model` onto the MORAI ego vehicle.
///
/// Idempotency is NOT guaranteed by MORAI: calling this twice mounts duplicates. The
/// caller checks GetSensorInfos first and skips if the vehicle already has sensors.
///
/// Returns the number actually mounted; logs each skip and failure through `node`.
size_t mount_sensor_kit(
  rclcpp::Node & node, morai_grpc_vendor::SensorClient & client,
  const std::vector<SensorKitEntry> & entries);

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__SENSOR_KIT_MOUNTER_HPP_
