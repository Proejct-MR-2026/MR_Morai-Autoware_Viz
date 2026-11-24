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

// ActorClient 의 PIMPL 구현 — grpcpp 의존은 전부 이 파일 안에 갇힌다

#include "morai_grpc_vendor/actor_client.hpp"

#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/actor/actor_set.pb.h>
#include <morai/common/enum.pb.h>
#include <morai/common/object_info.pb.h>
#include <morai/common/result.pb.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>

namespace morai_grpc_vendor
{

namespace
{
/// MORAI streams no data on this service, but GetActorState carries a nested
/// LinkInfo/BoundingBox payload. 32 MB matches what MORAI's own Python SDK requests.
constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;

std::string describe(const grpc::Status & status)
{
  return "gRPC " + std::to_string(static_cast<int>(status.error_code())) + ": " +
         status.error_message();
}
}  // namespace

class ActorClient::Impl
{
public:
  explicit Impl(const ActorClientConfig & config) : config_(config)
  {
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);

    channel_ = grpc::CreateCustomChannel(
      config_.address + ":" + std::to_string(config_.port), grpc::InsecureChannelCredentials(),
      args);
    stub_ = morai_sim_api::actor::Actor::NewStub(channel_);

    // Prepared once and copied into every request. MORAI identifies the ego actor by
    // this triple, and it never changes for the lifetime of the client.
    ego_info_.mutable_id()->set_value(config_.ego_id);
    ego_info_.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
    ego_info_.set_client_key(config_.client_key);
  }

  bool wait_for_connected(std::chrono::milliseconds timeout)
  {
    return channel_->WaitForConnected(deadline_from(timeout));
  }

  // --- Autoware -> MORAI -----------------------------------------------------

  bool control_vehicle(morai_sim_api::actor::VehicleCtrlCmd cmd, std::string * error)
  {
    *cmd.mutable_actor_info() = ego_info_;
    return call_returning_result(
      [this](grpc::ClientContext * ctx, const auto & req, morai_sim_api::Result * res) {
        return stub_->ControlVehicle(ctx, req, res);
      },
      cmd, "ControlVehicle", error);
  }

  bool set_vehicle_gear(morai_sim_api::actor::GearMode gear, std::string * error)
  {
    morai_sim_api::actor::VehicleGear req;
    *req.mutable_actor_info() = ego_info_;
    req.set_gear(gear);
    return call_returning_result(
      [this](grpc::ClientContext * ctx, const auto & r, morai_sim_api::Result * res) {
        return stub_->SetVehicleGear(ctx, r, res);
      },
      req, "SetVehicleGear", error);
  }

  bool set_vehicle_tail_light(morai_sim_api::actor::VehicleTurnSignal signal, std::string * error)
  {
    morai_sim_api::actor::VehicleTailLight req;
    *req.mutable_actor_info() = ego_info_;
    req.set_signal(signal);
    return call_returning_result(
      [this](grpc::ClientContext * ctx, const auto & r, morai_sim_api::Result * res) {
        return stub_->SetVehicleTailLight(ctx, r, res);
      },
      req, "SetVehicleTailLight", error);
  }

  bool set_vehicle_control_mode(morai_sim_api::actor::VehicleControlMode mode, std::string * error)
  {
    morai_sim_api::actor::VehicleControlModeParam req;
    *req.mutable_actor_info() = ego_info_;
    req.set_mode(mode);
    return call_returning_result(
      [this](grpc::ClientContext * ctx, const auto & r, morai_sim_api::Result * res) {
        return stub_->SetVehicleControlMode(ctx, r, res);
      },
      req, "SetVehicleControlMode", error);
  }

  // --- MORAI -> Autoware -----------------------------------------------------

  bool get_actor_state(morai_sim_api::actor::ActorState * out, std::string * error)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(config_.deadline));
    const grpc::Status status = stub_->GetActorState(&ctx, ego_info_, out);
    return check_status(status, "GetActorState", error);
  }

  bool get_vehicle_control_mode(morai_sim_api::actor::VehicleControlMode * out, std::string * error)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(config_.deadline));
    morai_sim_api::actor::CurVehicleControlMode res;
    const grpc::Status status = stub_->GetVehicleControlMode(&ctx, ego_info_, &res);
    if (!check_status(status, "GetVehicleControlMode", error)) {
      return false;
    }
    *out = res.mode();
    return true;
  }

  std::shared_ptr<grpc::Channel> channel() const { return channel_; }
  const morai_sim_api::ObjectInfo & ego_info() const { return ego_info_; }

  bool get_vehicle_spec(morai_sim_api::actor::VehicleSpec * out, std::string * error)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(config_.deadline));
    const grpc::Status status = stub_->GetVehicleSpec(&ctx, ego_info_, out);
    return check_status(status, "GetVehicleSpec", error);
  }

private:
  static std::chrono::system_clock::time_point deadline_from(std::chrono::milliseconds timeout)
  {
    return std::chrono::system_clock::now() + timeout;
  }

  static bool check_status(const grpc::Status & status, const char * rpc_name, std::string * error)
  {
    if (status.ok()) {
      return true;
    }
    if (error) {
      *error = std::string(rpc_name) + " failed - " + describe(status);
    }
    return false;
  }

  /// Shared body for every RPC that returns morai_sim_api::Result.
  ///
  /// Two independent failure modes have to be checked: the transport (grpc::Status) and
  /// the application (Result.status). Note StatusCode is UNKNOWN_FAILURE=0 and
  /// SUCCESS=1 — treating 0 as success would invert every error path here.
  template <typename Invoke, typename Request>
  bool call_returning_result(
    Invoke && invoke, const Request & request, const char * rpc_name, std::string * error)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(config_.deadline));

    morai_sim_api::Result result;
    const grpc::Status status = invoke(&ctx, request, &result);
    if (!check_status(status, rpc_name, error)) {
      return false;
    }

    if (result.status() != morai_sim_api::STATUS_CODE_SUCCESS) {
      if (error) {
        *error = std::string(rpc_name) + " rejected by simulator - " + result.description();
      }
      return false;
    }
    return true;
  }

  ActorClientConfig config_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<morai_sim_api::actor::Actor::Stub> stub_;
  morai_sim_api::ObjectInfo ego_info_;
};

// ============================================================================
// Forwarding shell
// ============================================================================

ActorClient::ActorClient(const ActorClientConfig & config) : impl_(std::make_unique<Impl>(config))
{
}

ActorClient::~ActorClient() = default;

bool ActorClient::wait_for_connected(std::chrono::milliseconds timeout)
{
  return impl_->wait_for_connected(timeout);
}

bool ActorClient::control_vehicle(morai_sim_api::actor::VehicleCtrlCmd cmd, std::string * error)
{
  return impl_->control_vehicle(std::move(cmd), error);
}

bool ActorClient::set_vehicle_gear(morai_sim_api::actor::GearMode gear, std::string * error)
{
  return impl_->set_vehicle_gear(gear, error);
}

bool ActorClient::set_vehicle_tail_light(
  morai_sim_api::actor::VehicleTurnSignal signal, std::string * error)
{
  return impl_->set_vehicle_tail_light(signal, error);
}

bool ActorClient::set_vehicle_control_mode(
  morai_sim_api::actor::VehicleControlMode mode, std::string * error)
{
  return impl_->set_vehicle_control_mode(mode, error);
}

bool ActorClient::get_actor_state(morai_sim_api::actor::ActorState * out, std::string * error)
{
  return impl_->get_actor_state(out, error);
}

bool ActorClient::get_vehicle_control_mode(
  morai_sim_api::actor::VehicleControlMode * out, std::string * error)
{
  return impl_->get_vehicle_control_mode(out, error);
}

bool ActorClient::get_vehicle_spec(morai_sim_api::actor::VehicleSpec * out, std::string * error)
{
  return impl_->get_vehicle_spec(out, error);
}

std::shared_ptr<void> ActorClient::shared_channel() const
{
  return impl_->channel();
}

const morai_sim_api::ObjectInfo & ActorClient::ego_info() const
{
  return impl_->ego_info();
}

}  // namespace morai_grpc_vendor
