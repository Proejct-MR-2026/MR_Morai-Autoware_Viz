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

// SensorClient 의 PIMPL 구현 — ActorClient 의 채널을 재사용한다

#include "morai_grpc_vendor/sensor_client.hpp"

#include <grpcpp/grpcpp.h>
#include <morai/common/enum.pb.h>
#include <morai/common/result.pb.h>
#include <morai/sensor/add_sensor_param.pb.h>
#include <morai/sensor/sensor.grpc.pb.h>
#include <morai/sensor/sensor_info.pb.h>

#include <chrono>
#include <memory>
#include <string>

namespace morai_grpc_vendor
{

namespace
{
/// AddSensor / GetSensorInfos are setup calls made at start-up, not on a control path,
/// so they get a longer deadline than ActorClient's 100 ms.
constexpr std::chrono::milliseconds kSetupDeadline{3000};

std::string describe(const grpc::Status & status)
{
  return "gRPC " + std::to_string(static_cast<int>(status.error_code())) + ": " +
         status.error_message();
}
}  // namespace

class SensorClient::Impl
{
public:
  explicit Impl(ActorClient & actor_client) : ego_info_(actor_client.ego_info())
  {
    // Reuses ActorClient's connection; MORAI serves every service on the same port.
    auto channel = std::static_pointer_cast<grpc::Channel>(actor_client.shared_channel());
    stub_ = morai_sim_api::sensor::Sensor::NewStub(channel);
  }

  bool add_sensor(const SensorMount & mount, std::string * error)
  {
    morai_sim_api::sensor::AddSensorParam req;
    *req.mutable_vehicle_id() = ego_info_;
    req.set_sensor_type(mount.type);

    auto * transform = req.mutable_transform();
    transform->mutable_location()->set_x(mount.x);
    transform->mutable_location()->set_y(mount.y);
    transform->mutable_location()->set_z(mount.z);
    transform->mutable_rotation()->set_x(mount.roll);
    transform->mutable_rotation()->set_y(mount.pitch);
    transform->mutable_rotation()->set_z(mount.yaw);

    grpc::ClientContext ctx;
    ctx.set_deadline(std::chrono::system_clock::now() + kSetupDeadline);

    morai_sim_api::Result result;
    const grpc::Status status = stub_->AddSensor(&ctx, req, &result);
    if (!status.ok()) {
      if (error) {
        *error = "AddSensor(" + mount.label + ") failed - " + describe(status);
      }
      return false;
    }
    // StatusCode is UNKNOWN_FAILURE=0 and SUCCESS=1; 0 is NOT success.
    if (result.status() != morai_sim_api::STATUS_CODE_SUCCESS) {
      if (error) {
        *error = "AddSensor(" + mount.label + ") rejected by simulator - " + result.description();
      }
      return false;
    }
    return true;
  }

  bool get_sensor_count(int * out, std::string * error)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(std::chrono::system_clock::now() + kSetupDeadline);

    morai_sim_api::sensor::SensorInfos res;
    const grpc::Status status = stub_->GetSensorInfos(&ctx, ego_info_, &res);
    if (!status.ok()) {
      if (error) {
        *error = "GetSensorInfos failed - " + describe(status);
      }
      return false;
    }
    *out = res.values_size();
    return true;
  }

private:
  morai_sim_api::ObjectInfo ego_info_;
  std::unique_ptr<morai_sim_api::sensor::Sensor::Stub> stub_;
};

// ============================================================================
// Forwarding shell
// ============================================================================

SensorClient::SensorClient(ActorClient & actor_client) : impl_(std::make_unique<Impl>(actor_client))
{
}

SensorClient::~SensorClient() = default;

bool SensorClient::add_sensor(const SensorMount & mount, std::string * error)
{
  return impl_->add_sensor(mount, error);
}

bool SensorClient::get_sensor_count(int * out, std::string * error)
{
  return impl_->get_sensor_count(out, error);
}

}  // namespace morai_grpc_vendor
