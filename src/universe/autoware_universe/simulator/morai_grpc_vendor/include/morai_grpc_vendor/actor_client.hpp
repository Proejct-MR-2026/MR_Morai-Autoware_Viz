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

// MORAI SIM 의 gRPC Actor 서비스를 감싸는 얇은 클라이언트 (grpcpp 를 헤더에 노출하지 않음)

#ifndef MORAI_GRPC_VENDOR__ACTOR_CLIENT_HPP_
#define MORAI_GRPC_VENDOR__ACTOR_CLIENT_HPP_

#include <morai/actor/actor_control.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/actor/actor_get.pb.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace morai_grpc_vendor
{

/// Connection and identity settings for a single MORAI ego vehicle.
struct ActorClientConfig
{
  std::string address{"127.0.0.1"};
  uint16_t port{7789};

  /// Goes into ObjectIdentifier.value. MORAI names the ego actor "Ego" by default.
  /// NOTE this is a string, not a numeric id.
  std::string ego_id{"Ego"};

  /// Goes into ObjectInfo.client_key. Any string; MORAI echoes it back for tracing.
  std::string client_key{"autoware_morai_interface"};

  /// Per-RPC deadline. Every call is unary and blocking, so this bounds how long a
  /// caller can be stalled by an unresponsive simulator.
  std::chrono::milliseconds deadline{100};
};

/// Thin client for the MORAI `Actor` gRPC service.
///
/// Deliberately NOT split by direction, unlike the bridge logic in
/// autoware_morai_interface. Upstream declares a single `service Actor`, so there is one
/// channel and one stub; splitting would duplicate connection state to satisfy a rule
/// that exists for the Autoware side. The two directions are separated by the comment
/// banners below.
///
/// This class owns the ObjectInfo and stamps `actor_info` into every request itself, so
/// callers never construct a MORAI identifier type. That keeps the vendor surface out of
/// the Autoware-facing package.
///
/// Every method reports failure as `false` plus a human-readable message through
/// `error`, rather than returning `grpc::Status` — exposing gRPC types here would drag
/// <grpcpp/...> into this header and defeat the PIMPL.
class ActorClient
{
public:
  explicit ActorClient(const ActorClientConfig & config);
  ~ActorClient();

  ActorClient(const ActorClient &) = delete;
  ActorClient & operator=(const ActorClient &) = delete;
  ActorClient(ActorClient &&) = delete;
  ActorClient & operator=(ActorClient &&) = delete;

  /// Block until the channel reaches READY, or `timeout` elapses.
  ///
  /// grpc::CreateChannel is lazy and never fails, so constructing this object proves
  /// nothing about the simulator being reachable. Callers that need a live connection
  /// must call this. (The Python bridge this replaces had a `while not is_connected()`
  /// loop that was a no-op for exactly this reason.)
  bool wait_for_connected(std::chrono::milliseconds timeout);

  // ===========================================================================
  // Autoware -> MORAI
  // ===========================================================================

  /// `cmd.actor_info` is overwritten by this client; leave it default-constructed.
  bool control_vehicle(morai_sim_api::actor::VehicleCtrlCmd cmd, std::string * error);
  bool set_vehicle_gear(morai_sim_api::actor::GearMode gear, std::string * error);
  bool set_vehicle_tail_light(morai_sim_api::actor::VehicleTurnSignal signal, std::string * error);
  bool set_vehicle_control_mode(morai_sim_api::actor::VehicleControlMode mode, std::string * error);

  // ===========================================================================
  // MORAI -> Autoware
  // ===========================================================================

  bool get_actor_state(morai_sim_api::actor::ActorState * out, std::string * error);
  bool get_vehicle_control_mode(
    morai_sim_api::actor::VehicleControlMode * out, std::string * error);
  bool get_vehicle_spec(morai_sim_api::actor::VehicleSpec * out, std::string * error);

  // ===========================================================================
  // Package internals
  // ===========================================================================

  /// Shared gRPC connection, for sibling clients in this package (e.g. SensorClient).
  ///
  /// Type-erased as void so that <grpcpp/...> stays out of this header; callers inside
  /// morai_grpc_vendor static_pointer_cast it back to grpc::Channel. MORAI serves every
  /// service on the same port, so opening a second channel would only duplicate
  /// connection state.
  std::shared_ptr<void> shared_channel() const;

  /// The ObjectInfo this client stamps into every request.
  const morai_sim_api::ObjectInfo & ego_info() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace morai_grpc_vendor

#endif  // MORAI_GRPC_VENDOR__ACTOR_CLIENT_HPP_
