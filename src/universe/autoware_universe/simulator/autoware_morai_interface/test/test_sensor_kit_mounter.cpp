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

// 센서킷 -> MORAI 매핑 테스트. 프레임 이름 규칙이 바뀌면 여기서 잡힌다

#include "autoware/morai_interface/sensor_kit_mounter.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::morai_interface
{
namespace
{

using morai_sim_api::sensor::SENSOR_TYPE_CAMERA;
using morai_sim_api::sensor::SENSOR_TYPE_GPS;
using morai_sim_api::sensor::SENSOR_TYPE_IMU;
using morai_sim_api::sensor::SENSOR_TYPE_LIDAR3D;
using morai_sim_api::sensor::SENSOR_TYPE_RADAR;
using morai_sim_api::sensor::SENSOR_TYPE_UNSPECIFIED;

TEST(SensorKitMounter, MapsMoraiKitFrames)
{
  // morai_sensor_kit_description 이 실제로 쓰는 세 프레임.
  EXPECT_EQ(to_morai_sensor_type("lidar"), SENSOR_TYPE_LIDAR3D);
  EXPECT_EQ(to_morai_sensor_type("imu"), SENSOR_TYPE_IMU);
  EXPECT_EQ(to_morai_sensor_type("gnss_link"), SENSOR_TYPE_GPS);
}

TEST(SensorKitMounter, MapsCommonAutowareFrameNames)
{
  // 다른 센서킷에서 흔한 이름들. 부분 문자열 매칭이라 벤더명도 잡힌다.
  EXPECT_EQ(to_morai_sensor_type("velodyne_top"), SENSOR_TYPE_LIDAR3D);
  EXPECT_EQ(to_morai_sensor_type("ouster_front"), SENSOR_TYPE_LIDAR3D);
  EXPECT_EQ(to_morai_sensor_type("livox_left"), SENSOR_TYPE_LIDAR3D);
  EXPECT_EQ(to_morai_sensor_type("tamagawa/imu_link"), SENSOR_TYPE_IMU);
  EXPECT_EQ(to_morai_sensor_type("gnss_antenna"), SENSOR_TYPE_GPS);
  EXPECT_EQ(to_morai_sensor_type("gps_link"), SENSOR_TYPE_GPS);
  EXPECT_EQ(to_morai_sensor_type("camera0/camera_link"), SENSOR_TYPE_CAMERA);
  EXPECT_EQ(to_morai_sensor_type("traffic_light_right_camera"), SENSOR_TYPE_CAMERA);
  EXPECT_EQ(to_morai_sensor_type("radar_front"), SENSOR_TYPE_RADAR);
}

TEST(SensorKitMounter, IsCaseInsensitive)
{
  EXPECT_EQ(to_morai_sensor_type("LiDAR"), SENSOR_TYPE_LIDAR3D);
  EXPECT_EQ(to_morai_sensor_type("IMU_LINK"), SENSOR_TYPE_IMU);
}

TEST(SensorKitMounter, UnknownFrameIsUnspecified)
{
  // 매핑 못 하는 프레임은 UNSPECIFIED 로 두고 호출자가 건너뛴다. 임의로 아무거나
  // 장착하는 것보다 안 붙이는 편이 낫다.
  EXPECT_EQ(to_morai_sensor_type("base_link"), SENSOR_TYPE_UNSPECIFIED);
  EXPECT_EQ(to_morai_sensor_type("some_unknown_thing"), SENSOR_TYPE_UNSPECIFIED);
}

TEST(SensorKitMounter, LidarMatchBeatsNothingElse)
{
  // "lidar" 와 "camera" 가 같이 들어간 이름은 없어야 정상이지만, 우선순위가 바뀌면
  // 조용히 잘못 장착되므로 현재 순서를 고정한다.
  EXPECT_EQ(to_morai_sensor_type("lidar_camera_rig"), SENSOR_TYPE_LIDAR3D);
}

TEST(SensorKitMounter, LoadsMoraiSensorKit)
{
  // 실제 morai_sensor_kit_description 을 읽는다. 캘리브레이션 값이 바뀌면 여기서 드러난다.
  std::vector<SensorKitEntry> entries;
  ASSERT_NO_THROW(entries = load_sensor_kit("morai_sensor_kit"));

  ASSERT_EQ(entries.size(), 3U) << "expected lidar, imu, gnss_link";

  bool saw_lidar = false;
  for (const auto & e : entries) {
    if (e.frame != "lidar") {
      continue;
    }
    saw_lidar = true;
    // sensors_calibration 의 base_link->kit 이 전부 0 이므로 kit 값이 그대로 나와야 한다.
    EXPECT_NEAR(e.x, 1.0, 1e-9);
    EXPECT_NEAR(e.y, 0.0, 1e-9);
    EXPECT_NEAR(e.z, 1.3, 1e-9);
    EXPECT_NEAR(e.yaw, 0.0, 1e-9);
  }
  EXPECT_TRUE(saw_lidar);
}

TEST(SensorKitMounter, MissingSensorKitThrows)
{
  // 없는 센서킷은 예외. 호출자가 잡아서 "센서 안 붙임" 으로 처리하고 차량 제어는 계속한다.
  EXPECT_THROW(load_sensor_kit("no_such_sensor_kit"), std::exception);
}

}  // namespace
}  // namespace autoware::morai_interface
