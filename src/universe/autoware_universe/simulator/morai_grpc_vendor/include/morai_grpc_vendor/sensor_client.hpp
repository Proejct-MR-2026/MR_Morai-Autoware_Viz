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

// MORAI SIM 의 gRPC Sensor 서비스를 감싸는 얇은 클라이언트 (grpcpp 를 헤더에 노출하지 않음)

#ifndef MORAI_GRPC_VENDOR__SENSOR_CLIENT_HPP_
#define MORAI_GRPC_VENDOR__SENSOR_CLIENT_HPP_

#include "morai_grpc_vendor/actor_client.hpp"

#include <morai/sensor/sensor_enum.pb.h>

#include <memory>
#include <string>
#include <vector>

namespace morai_grpc_vendor
{

/// One sensor to mount on the ego vehicle, in MORAI's own convention.
///
/// MORAI's Transform.location is ENU and rotation is (roll, pitch, yaw) in DEGREES,
/// counter-clockwise with east at zero (common/type.proto). Callers are responsible for
/// converting from whatever frame they hold; this struct is already MORAI-side.
struct SensorMount
{
  morai_sim_api::sensor::SensorType type{morai_sim_api::sensor::SENSOR_TYPE_UNSPECIFIED};

  double x{0.0};  //!< [m]
  double y{0.0};
  double z{0.0};

  double roll{0.0};  //!< [deg]
  double pitch{0.0};
  double yaw{0.0};

  /// Only for logging; MORAI assigns its own sensor id.
  std::string label;
};

/// Thin client for the MORAI `Sensor` gRPC service.
///
/// Shares the channel with ActorClient rather than opening a second one — MORAI serves
/// every service on the same port, and two channels would double the connection state
/// for no benefit.
///
/// NOTE this covers sensor SETUP only. Raw sensor data (LiDAR points, camera images,
/// IMU, GNSS) is not available over gRPC at all: Sensor.GetSensorData returns a
/// SensorData whose only variant is GroundTruth (sensor_data.proto). Those streams come
/// out of MORAI over ROS 2.
class SensorClient
{
public:
  /// Reuses the connection owned by `actor_client`, which must outlive this object.
  explicit SensorClient(ActorClient & actor_client);
  ~SensorClient();

  SensorClient(const SensorClient &) = delete;
  SensorClient & operator=(const SensorClient &) = delete;
  SensorClient(SensorClient &&) = delete;
  SensorClient & operator=(SensorClient &&) = delete;

  /// Mount one sensor on the ego vehicle.
  bool add_sensor(const SensorMount & mount, std::string * error);

  /// How many sensors are currently mounted on the ego vehicle.
  bool get_sensor_count(int * out, std::string * error);

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace morai_grpc_vendor

#endif  // MORAI_GRPC_VENDOR__SENSOR_CLIENT_HPP_
