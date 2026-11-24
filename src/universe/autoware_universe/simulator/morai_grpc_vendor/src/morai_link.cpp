// 신규 — 26.09.25 _ YSH _ MoraiLink PIMPL 구현 — grpcpp·생성 스텁 의존은 전부 이 파일 안에 갇힌다
//
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

#include "morai_grpc_vendor/morai_link.hpp"

#include "morai_link_convert.hpp"

#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/common/enum.pb.h>
#include <morai/simulation/simulation.grpc.pb.h>
#include <morai/simulation/simulation_enum.pb.h>
#include <morai/simulation/sync_mode.pb.h>
#include <morai/simulator/simulator.grpc.pb.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <thread>
#include <utility>

namespace morai_grpc_vendor
{

// 공개 열거형 사본이 proto 값과 어긋나면 빌드가 깨진다 (열거자 전부)
static_assert(static_cast<int>(GearMode::kM) == morai_sim_api::actor::GEAR_MODE_M, "gear");
static_assert(static_cast<int>(GearMode::kP) == morai_sim_api::actor::GEAR_MODE_P, "gear");
static_assert(static_cast<int>(GearMode::kR) == morai_sim_api::actor::GEAR_MODE_R, "gear");
static_assert(static_cast<int>(GearMode::kN) == morai_sim_api::actor::GEAR_MODE_N, "gear");
static_assert(static_cast<int>(GearMode::kD) == morai_sim_api::actor::GEAR_MODE_D, "gear");
static_assert(static_cast<int>(GearMode::kL) == morai_sim_api::actor::GEAR_MODE_L, "gear");
static_assert(
  static_cast<int>(LongCmdType::kUnspecified) == morai_sim_api::actor::LONG_CMD_TYPE_UNSPECIFIED,
  "lc");
static_assert(
  static_cast<int>(LongCmdType::kThrottle) == morai_sim_api::actor::LONG_CMD_TYPE_THROTTLE, "lc");
static_assert(
  static_cast<int>(LongCmdType::kSpeed) == morai_sim_api::actor::LONG_CMD_TYPE_SPEED, "lc");
static_assert(
  static_cast<int>(LongCmdType::kAcceleration) == morai_sim_api::actor::LONG_CMD_TYPE_ACCELERATION,
  "lc");
// 26.10.12 _ YSH _ kGhost 추가 (의사 VILS follow)
static_assert(static_cast<int>(LongCmdType::kGhost) == morai_sim_api::actor::LONG_CMD_TYPE_GHOST, "lc");
static_assert(
  static_cast<int>(ControlMode::kUnspecified) == morai_sim_api::actor::VEHICLE_CONTROL_UNSPECIFIED,
  "m");
static_assert(
  static_cast<int>(ControlMode::kKeyboard) == morai_sim_api::actor::VEHICLE_CONTROL_KEYBOARD, "m");
static_assert(
  static_cast<int>(ControlMode::kGameWheel) == morai_sim_api::actor::VEHICLE_CONTROL_GAME_WHEEL,
  "m");
static_assert(
  static_cast<int>(ControlMode::kAuto) == morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE, "m");
static_assert(
  static_cast<int>(ControlMode::kAutoLateral) ==
    morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LATERAL,
  "m");
static_assert(
  static_cast<int>(ControlMode::kAutoLongitudinal) ==
    morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LONGITUDINAL,
  "m");
static_assert(
  static_cast<int>(ControlMode::kCruise) == morai_sim_api::actor::VEHICLE_CONTROL_CRUISE_MODE,
  "m");
static_assert(
  static_cast<int>(ControlMode::kSynchronous) ==
    morai_sim_api::actor::VEHICLE_CONTROL_SYNCHRONOUS_MODE,
  "m");
static_assert(
  static_cast<int>(TurnSignal::kOff) == morai_sim_api::actor::VEHICLE_TURN_SIGNAL_UNSPECIFIED,
  "ts");
static_assert(
  static_cast<int>(TurnSignal::kLeft) == morai_sim_api::actor::VEHICLE_TURN_SIGNAL_LEFT, "ts");
static_assert(
  static_cast<int>(TurnSignal::kRight) == morai_sim_api::actor::VEHICLE_TURN_SIGNAL_RIGHT, "ts");
static_assert(
  static_cast<int>(TurnSignal::kAll) == morai_sim_api::actor::VEHICLE_TURN_SIGNAL_ALL, "ts");
static_assert(
  static_cast<int>(ObjectType::kUnspecified) == morai_sim_api::OBJECT_TYPE_UNSPECIFIED, "o");
static_assert(static_cast<int>(ObjectType::kVehicle) == morai_sim_api::OBJECT_TYPE_VEHICLE, "o");
static_assert(
  static_cast<int>(ObjectType::kPedestrian) == morai_sim_api::OBJECT_TYPE_PEDESTRIAN, "o");
static_assert(static_cast<int>(ObjectType::kObstacle) == morai_sim_api::OBJECT_TYPE_OBSTACLE, "o");
static_assert(
  static_cast<int>(ObjectType::kSpawnPoint) == morai_sim_api::OBJECT_TYPE_SPAWN_POINT, "o");
static_assert(static_cast<int>(ChannelState::kIdle) == GRPC_CHANNEL_IDLE, "cs");
static_assert(static_cast<int>(ChannelState::kConnecting) == GRPC_CHANNEL_CONNECTING, "cs");
static_assert(static_cast<int>(ChannelState::kReady) == GRPC_CHANNEL_READY, "cs");
static_assert(
  static_cast<int>(ChannelState::kTransientFailure) == GRPC_CHANNEL_TRANSIENT_FAILURE, "cs");
static_assert(static_cast<int>(ChannelState::kShutdown) == GRPC_CHANNEL_SHUTDOWN, "cs");
static_assert(static_cast<int>(grpc::StatusCode::DEADLINE_EXCEEDED) == 4, "Status helper");
static_assert(static_cast<int>(grpc::StatusCode::UNAVAILABLE) == 14, "Status helper");

std::string to_string(ChannelState s)
{
  switch (s) {
    case ChannelState::kIdle:
      return "IDLE";
    case ChannelState::kConnecting:
      return "CONNECTING";
    case ChannelState::kReady:
      return "READY";
    case ChannelState::kTransientFailure:
      return "TRANSIENT_FAILURE";
    case ChannelState::kShutdown:
      return "SHUTDOWN";
  }
  return "UNKNOWN(" + std::to_string(static_cast<int>(s)) + ")";
}

namespace
{

/// MORAI Python SDK 가 요청하는 값과 같다 (GetAllActorsState 는 actor 수에 비례해 커진다)
constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;

using SteadyClock = std::chrono::steady_clock;

int64_t system_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           SteadyClock::now().time_since_epoch())
    .count();
}

double ms_since(SteadyClock::time_point a)
{
  return std::chrono::duration<double, std::milli>(SteadyClock::now() - a).count();
}

/// 0 이하 deadline 은 매번 즉시 DEADLINE_EXCEEDED 가 되므로 1 ms 하한 (main.py 와 같음)
std::chrono::system_clock::time_point deadline_from(std::chrono::milliseconds d)
{
  return std::chrono::system_clock::now() + std::max(d, std::chrono::milliseconds(1));
}

std::string grpc_code_name(grpc::StatusCode c)
{
  switch (c) {
    case grpc::StatusCode::OK:
      return "OK";
    case grpc::StatusCode::CANCELLED:
      return "CANCELLED";
    case grpc::StatusCode::UNKNOWN:
      return "UNKNOWN";
    case grpc::StatusCode::INVALID_ARGUMENT:
      return "INVALID_ARGUMENT";
    case grpc::StatusCode::DEADLINE_EXCEEDED:
      return "DEADLINE_EXCEEDED";
    case grpc::StatusCode::NOT_FOUND:
      return "NOT_FOUND";
    case grpc::StatusCode::UNAVAILABLE:
      return "UNAVAILABLE";
    case grpc::StatusCode::UNIMPLEMENTED:
      return "UNIMPLEMENTED";
    case grpc::StatusCode::INTERNAL:
      return "INTERNAL";
    case grpc::StatusCode::RESOURCE_EXHAUSTED:
      return "RESOURCE_EXHAUSTED";
    default:
      return "CODE_" + std::to_string(static_cast<int>(c));
  }
}

/// 전송 계층 결과만 담은 Status (Result 는 apply_result 로 따로)
Status transport_status(const grpc::Status & st, double ms, const char * rpc_name)
{
  Status s;
  s.grpc_code = static_cast<int32_t>(st.error_code());
  s.ok = st.ok();
  s.rpc_ms = ms;
  if (!st.ok()) {
    s.message = std::string(rpc_name) + " failed - " + grpc_code_name(st.error_code());
    if (!st.error_message().empty()) {
      s.message += ": " + st.error_message().substr(0, 300);
    }
  }
  return s;
}

Status & status_of(Status & s)
{
  return s;
}

template <typename T>
Status & status_of(Reply<T> & r)
{
  return r.status;
}

/// 예외를 API 밖으로 내보내지 않는다. 메시지 대입이 다시 던지면 빈 메시지로 둔다
template <typename R, typename F>
R no_throw(const char * what, F && f) noexcept
{
  try {
    return f();
  } catch (const std::exception & e) {
    R r{};
    try {
      status_of(r).message = std::string(what) + " threw: " + e.what();
    } catch (...) {
    }
    return r;
  } catch (...) {
    R r{};
    try {
      status_of(r).message = std::string(what) + " threw a non-std exception";
    } catch (...) {
    }
    return r;
  }
}

}  // namespace

// =============================================================================
// Impl
// =============================================================================

class MoraiLink::Impl
{
public:
  explicit Impl(const MoraiLinkOptions & options) : opt_(options)
  {
    state_ch_ = make_channel("state");
    command_ch_ = make_channel("command");
    state_actor_ = morai_sim_api::actor::Actor::NewStub(state_ch_);
    state_simulation_ = morai_sim_api::simulation::Simulation::NewStub(state_ch_);
    // NOTE upstream 패키지명이 "simualtor"(sic)
    state_simulator_ = morai_sim_api::simualtor::Simulator::NewStub(state_ch_);
    command_actor_ = morai_sim_api::actor::Actor::NewStub(command_ch_);

    // main.py 와 같은 식별자. 수명 동안 바뀌지 않으므로 한 번만 만들어 요청마다 복사한다
    // 26.09.26 _ YSH _ 원시 기록(vehicle_ctrl_cmd_json)과 같은 함수로 만든다. 원문:
    // ego_.mutable_id()->set_value(opt_.ego_id);
    // ego_.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
    // ego_.set_client_key(opt_.client_key);
    ego_ = link_convert::ego_object_info(opt_.ego_id, opt_.client_key);

    filter_.set_client_key(opt_.filter_client_key);
    filter_.set_vehicle(opt_.include_vehicles);
    filter_.set_pedestrian(opt_.include_pedestrians);
    filter_.set_obstacle(opt_.include_obstacles);
  }

  const MoraiLinkOptions & options() const { return opt_; }

  Status wait_connected(std::chrono::milliseconds timeout)
  {
    const auto a = SteadyClock::now();
    const auto deadline = deadline_from(timeout);
    const bool state_ok = state_ch_->WaitForConnected(deadline);
    const bool command_ok = command_ch_->WaitForConnected(deadline);
    Status s;
    s.ok = state_ok && command_ok;
    s.grpc_code = s.ok ? 0 : static_cast<int32_t>(grpc::StatusCode::UNAVAILABLE);
    s.rpc_ms = ms_since(a);
    if (!s.ok) {
      s.message = "not connected to " + target() + " within " + std::to_string(timeout.count()) +
                  " ms (state " + to_string(channel_state(LinkChannel::kState, false)) +
                  ", command " + to_string(channel_state(LinkChannel::kCommand, false)) + ")";
    }
    return s;
  }

  ChannelState channel_state(LinkChannel which, bool try_to_connect) const
  {
    const auto & ch = which == LinkChannel::kState ? state_ch_ : command_ch_;
    return static_cast<ChannelState>(ch->GetState(try_to_connect));
  }

  // --- 상태 채널 --------------------------------------------------------------

  Reply<int64_t> wait_for_tick(std::chrono::milliseconds deadline)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(deadline));
    morai_sim_api::Empty req;
    morai_sim_api::Int64Value res;
    const auto a = SteadyClock::now();
    const grpc::Status st = state_simulation_->WaitForTick(&ctx, req, &res);
    Reply<int64_t> r;
    r.status = transport_status(st, ms_since(a), "WaitForTick");
    count_state(r.status);
    if (r.status.ok) {
      r.value = res.value();
    }
    return r;
  }

  Status get_snapshot(Snapshot * out)
  {
    if (out == nullptr) {
      return rejected("get_snapshot: out is null");
    }
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.state));
    morai_sim_api::actor::ActorStates res;
    const int64_t send_sys = system_ns();
    const int64_t send_steady = steady_ns();
    const auto a = SteadyClock::now();
    const grpc::Status st = state_actor_->GetAllActorsState(&ctx, filter_, &res);
    const double ms = ms_since(a);
    const int64_t recv_sys = system_ns();
    const int64_t recv_steady = steady_ns();
    Status s = transport_status(st, ms, "GetAllActorsState");
    count_state(s);
    if (!s.ok) {
      return s;
    }
    link_convert::to_snapshot(res, opt_.ego_id, out);
    out->send_system_ns = send_sys;
    out->send_steady_ns = send_steady;
    out->receive_system_ns = recv_sys;
    out->receive_steady_ns = recv_steady;
    out->rpc_ms = ms;
    out->ego.receive_system_ns = recv_sys;
    out->ego.receive_steady_ns = recv_steady;
    // 26.09.26 _ YSH _ 원시 기록: 받은 응답 전체를 다시 직렬화해 새 버퍼로 싣는다 (JSON 은 기록 스레드에서). 꺼져 있으면 비운다
    if (opt_.capture_raw) {
      auto raw = std::make_shared<std::string>();
      res.SerializeToString(raw.get());
      out->raw_actor_states = std::move(raw);
    } else {
      out->raw_actor_states.reset();
    }
    return s;
  }

  Reply<EgoSample> get_ego_state()
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.state));
    morai_sim_api::actor::ActorState res;
    const auto a = SteadyClock::now();
    const grpc::Status st = state_actor_->GetActorState(&ctx, ego_, &res);
    Reply<EgoSample> r;
    r.status = transport_status(st, ms_since(a), "GetActorState");
    count_state(r.status);
    if (!r.status.ok) {
      return r;
    }
    link_convert::to_ego_sample(res, &r.value);
    r.value.found = true;
    r.value.exact_match = res.actor_info().id().value() == opt_.ego_id;
    r.value.receive_system_ns = system_ns();
    r.value.receive_steady_ns = steady_ns();
    return r;
  }

  Reply<ControlMode> get_control_mode()
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.control_mode));
    morai_sim_api::actor::CurVehicleControlMode res;
    const auto a = SteadyClock::now();
    const grpc::Status st = state_actor_->GetVehicleControlMode(&ctx, ego_, &res);
    Reply<ControlMode> r;
    r.status = transport_status(st, ms_since(a), "GetVehicleControlMode");
    count_state(r.status);
    if (r.status.ok) {
      r.value = static_cast<ControlMode>(res.mode());
    }
    return r;
  }

  Reply<VehicleSpecInfo> get_vehicle_spec()
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.spec));
    morai_sim_api::actor::VehicleSpec res;
    const auto a = SteadyClock::now();
    const grpc::Status st = state_actor_->GetVehicleSpec(&ctx, ego_, &res);
    Reply<VehicleSpecInfo> r;
    r.status = transport_status(st, ms_since(a), "GetVehicleSpec");
    count_state(r.status);
    if (r.status.ok) {
      r.value = link_convert::to_spec(res);
    }
    return r;
  }

  Reply<SyncModeInfo> get_sync_mode()
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.sync_mode));
    morai_sim_api::Empty req;
    morai_sim_api::simulation::SyncMode res;
    const auto a = SteadyClock::now();
    const grpc::Status st = state_simulation_->GetSynchronousMode(&ctx, req, &res);
    Reply<SyncModeInfo> r;
    r.status = transport_status(st, ms_since(a), "GetSynchronousMode");
    count_state(r.status);
    if (r.status.ok) {
      r.value.type = static_cast<int32_t>(res.type());
      r.value.type_name = morai_sim_api::simulation::SyncModeType_IsValid(res.type())
                            ? morai_sim_api::simulation::SyncModeType_Name(res.type())
                            : "UNKNOWN(" + std::to_string(static_cast<int>(res.type())) + ")";
      r.value.tick_period_ms = res.tick_period();
    }
    return r;
  }

  Reply<ServerTimeSample> probe_server_time()
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(opt_.deadlines.time_probe));
    morai_sim_api::Empty req;
    morai_sim_api::Int64Value res;
    const int64_t send_sys = system_ns();
    const auto a = SteadyClock::now();
    const grpc::Status st = state_simulator_->GetTimestamp(&ctx, req, &res);
    const double ms = ms_since(a);
    const int64_t recv_sys = system_ns();
    Reply<ServerTimeSample> r;
    r.status = transport_status(st, ms, "GetTimestamp");
    count_state(r.status);
    if (!r.status.ok) {
      return r;
    }
    r.value.clock = ClockSample{res.value(), send_sys, recv_sys};
    r.value.rtt_ms = static_cast<double>(recv_sys - send_sys) / 1e6;
    // 프레임 시각이라 NTP 식 중간점이 아니라 하한 (server - recv) 만 낸다
    const double bound = clock_offset_lower_bound_ms({r.value.clock});
    r.value.offset_lower_bound_ms = std::isfinite(bound) ? bound : 0.0;
    return r;
  }

  // --- 명령 채널 --------------------------------------------------------------

  Status control_vehicle(const VehicleCommand & cmd, std::chrono::milliseconds deadline)
  {
    if (!command_is_finite(cmd)) {
      return rejected("ControlVehicle not sent: non-finite field in command");
    }
    morai_sim_api::actor::VehicleCtrlCmd req;
    link_convert::to_ctrl_cmd(cmd, ego_, &req);
    return call_result(
      deadline, "ControlVehicle",
      [&](grpc::ClientContext * ctx, morai_sim_api::Result * res) {
        return command_actor_->ControlVehicle(ctx, req, res);
      });
  }

  Status set_gear(GearMode gear, std::chrono::milliseconds verify_timeout)
  {
    morai_sim_api::actor::VehicleGear req;
    *req.mutable_actor_info() = ego_;
    req.set_gear(static_cast<morai_sim_api::actor::GearMode>(gear));
    Status s = call_result(
      opt_.deadlines.gear, "SetVehicleGear",
      [&](grpc::ClientContext * ctx, morai_sim_api::Result * res) {
        return command_actor_->SetVehicleGear(ctx, req, res);
      });
    std::string last = "-";
    return verify(std::move(s), verify_timeout, "gear " + to_string(gear), &last, [&]() {
      const Reply<EgoSample> e = get_ego_state();
      if (!e.ok()) {
        last = "read failed";
        return false;
      }
      last = e.value.vehicle_state.present ? to_string(e.value.vehicle_state.gear_mode)
                                           : "no vehicle_state";
      return e.value.vehicle_state.present && e.value.vehicle_state.gear_mode == gear;
    });
  }

  Status set_control_mode(ControlMode mode, std::chrono::milliseconds verify_timeout)
  {
    morai_sim_api::actor::VehicleControlModeParam req;
    *req.mutable_actor_info() = ego_;
    req.set_mode(static_cast<morai_sim_api::actor::VehicleControlMode>(mode));
    Status s = call_result(
      opt_.deadlines.set_mode, "SetVehicleControlMode",
      [&](grpc::ClientContext * ctx, morai_sim_api::Result * res) {
        return command_actor_->SetVehicleControlMode(ctx, req, res);
      });
    std::string last = "-";
    return verify(std::move(s), verify_timeout, "control mode " + to_string(mode), &last, [&]() {
      const Reply<ControlMode> m = get_control_mode();
      last = m.ok() ? to_string(m.value) : "read failed";
      return m.ok() && m.value == mode;
    });
  }

  Status set_transform(
    const TransformPose & pose, std::chrono::milliseconds verify_timeout, double tol_m,
    double tol_deg)
  {
    const double v[] = {pose.position.x,     pose.position.y,     pose.position.z,
                        pose.rotation_deg.x, pose.rotation_deg.y, pose.rotation_deg.z};
    if (!std::all_of(std::begin(v), std::end(v), [](double x) { return std::isfinite(x); })) {
      return rejected("SetTransform not sent: non-finite pose");
    }
    morai_sim_api::actor::SetTransformParam req;
    link_convert::to_set_transform(pose, ego_, &req);
    Status s = call_result(
      opt_.deadlines.transform, "SetTransform",
      [&](grpc::ClientContext * ctx, morai_sim_api::Result * res) {
        return command_actor_->SetTransform(ctx, req, res);
      });
    std::string last = "-";
    return verify(std::move(s), verify_timeout, "transform", &last, [&]() {
      const Reply<EgoSample> e = get_ego_state();
      if (!e.ok()) {
        last = "read failed";
        return false;
      }
      last = "(" + std::to_string(e.value.position.x) + ", " + std::to_string(e.value.position.y) +
             ", yaw " + std::to_string(e.value.rotation_deg.z) + ")";
      return link_convert::transform_matches(e.value, pose, tol_m, tol_deg);
    });
  }

  Status set_tail_light(TurnSignal signal)
  {
    morai_sim_api::actor::VehicleTailLight req;
    *req.mutable_actor_info() = ego_;
    req.set_signal(static_cast<morai_sim_api::actor::VehicleTurnSignal>(signal));
    return call_result(
      opt_.deadlines.tail_light, "SetVehicleTailLight",
      [&](grpc::ClientContext * ctx, morai_sim_api::Result * res) {
        return command_actor_->SetVehicleTailLight(ctx, req, res);
      });
  }

  LinkCounters counters() const
  {
    LinkCounters c;
    c.state_calls = state_calls_.load();
    c.state_failures = state_failures_.load();
    c.command_calls = command_calls_.load();
    c.command_failures = command_failures_.load();
    c.deadline_exceeded = deadline_exceeded_.load();
    c.rejected_inputs = rejected_inputs_.load();
    return c;
  }

private:
  std::string target() const { return opt_.address + ":" + std::to_string(opt_.port); }

  /// 역할마다 다른 채널 인자 + 로컬 subchannel pool → 두 채널이 TCP 연결을 공유하지 않는다
  std::shared_ptr<grpc::Channel> make_channel(const char * role) const
  {
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);
    args.SetInt(GRPC_ARG_USE_LOCAL_SUBCHANNEL_POOL, 1);
    args.SetString("morai_link.channel_role", role);
    const int min_ms = std::max(1, opt_.min_reconnect_backoff_ms);
    const int max_ms = std::max(min_ms, opt_.max_reconnect_backoff_ms);
    args.SetInt(GRPC_ARG_INITIAL_RECONNECT_BACKOFF_MS, min_ms);
    args.SetInt(GRPC_ARG_MIN_RECONNECT_BACKOFF_MS, min_ms);
    args.SetInt(GRPC_ARG_MAX_RECONNECT_BACKOFF_MS, max_ms);
    if (opt_.keepalive_time_ms > 0) {
      // 호출이 있을 때만 PING (PERMIT_WITHOUT_CALLS 0) — 서버 PING 정책을 덜 건드린다
      args.SetInt(GRPC_ARG_KEEPALIVE_TIME_MS, opt_.keepalive_time_ms);
      args.SetInt(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, std::max(1, opt_.keepalive_timeout_ms));
      args.SetInt(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 0);
    }
    return grpc::CreateCustomChannel(target(), grpc::InsecureChannelCredentials(), args);
  }

  Status rejected(const std::string & why)
  {
    ++rejected_inputs_;
    Status s;
    s.ok = false;
    s.grpc_code = -1;
    s.message = why;
    return s;
  }

  void count_state(const Status & s)
  {
    ++state_calls_;
    if (!s.ok) {
      ++state_failures_;
    }
    if (s.deadline_exceeded()) {
      ++deadline_exceeded_;
    }
  }

  /// Result 를 돌려주는 명령 RPC 공통부: 전송(grpc::Status) → 응용(Result.status) 순으로 본다
  template <typename Invoke>
  Status call_result(std::chrono::milliseconds deadline, const char * rpc_name, Invoke && invoke)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_from(deadline));
    morai_sim_api::Result result;
    const auto a = SteadyClock::now();
    const grpc::Status st = invoke(&ctx, &result);
    Status s = transport_status(st, ms_since(a), rpc_name);
    if (s.ok) {
      link_convert::apply_result(result, rpc_name, &s);
    }
    ++command_calls_;
    if (!s.ok) {
      ++command_failures_;
    }
    if (s.deadline_exceeded()) {
      ++deadline_exceeded_;
    }
    return s;
  }

  /// 되읽기 확인. 전송 OK(Result 무관) 또는 DEADLINE_EXCEEDED(적용 여부 모름)일 때만 읽는다.
  /// 확인하면 ok = 되읽기 일치 (MORAI Result 는 거짓일 수 있다)
  template <typename Check>
  Status verify(
    Status s, std::chrono::milliseconds timeout, const std::string & what, const std::string * last,
    Check && check)
  {
    if (timeout.count() <= 0) {
      return s;
    }
    if (s.grpc_code != 0 && !s.deadline_exceeded()) {
      return s;  // 보내지 못했다 (UNAVAILABLE 등) — 되읽을 이유가 없다
    }
    const auto end = SteadyClock::now() + timeout;
    bool matched = false;
    for (;;) {
      if (check()) {
        matched = true;
        break;
      }
      if (SteadyClock::now() >= end) {
        break;
      }
      std::this_thread::sleep_for(std::max(opt_.readback_poll, std::chrono::milliseconds(1)));
    }
    const bool result_said_ok = s.ok;
    s.readback_checked = true;
    s.readback_matched = matched;
    s.ok = matched;
    if (matched && !result_said_ok) {
      s.message = "readback confirms " + what + " although the call reported failure (" +
                  s.message + ")";
    } else if (!matched) {
      s.message = (s.message.empty() ? std::string() : s.message + "; ") + "readback: " + what +
                  " not observed within " + std::to_string(timeout.count()) + " ms (last " +
                  *last + ")";
    }
    return s;
  }

  MoraiLinkOptions opt_;
  std::shared_ptr<grpc::Channel> state_ch_;
  std::shared_ptr<grpc::Channel> command_ch_;
  std::unique_ptr<morai_sim_api::actor::Actor::Stub> state_actor_;
  std::unique_ptr<morai_sim_api::simulation::Simulation::Stub> state_simulation_;
  std::unique_ptr<morai_sim_api::simualtor::Simulator::Stub> state_simulator_;
  std::unique_ptr<morai_sim_api::actor::Actor::Stub> command_actor_;
  morai_sim_api::ObjectInfo ego_;
  morai_sim_api::actor::GetAllActorsFilter filter_;

  std::atomic<uint64_t> state_calls_{0};
  std::atomic<uint64_t> state_failures_{0};
  std::atomic<uint64_t> command_calls_{0};
  std::atomic<uint64_t> command_failures_{0};
  std::atomic<uint64_t> deadline_exceeded_{0};
  std::atomic<uint64_t> rejected_inputs_{0};
};

// =============================================================================
// Forwarding shell — 예외 차단 + Impl 생성 실패 방어
// =============================================================================

namespace
{
Status not_initialised()
{
  Status s;
  s.message = "MoraiLink not initialised (constructor failed)";
  return s;
}
}  // namespace

MoraiLink::MoraiLink(const MoraiLinkOptions & options)
{
  try {
    impl_ = std::make_unique<Impl>(options);
  } catch (...) {
    impl_.reset();  // 모든 메서드가 not_initialised 로 답한다
  }
}

MoraiLink::~MoraiLink() = default;

const MoraiLinkOptions & MoraiLink::options() const
{
  static const MoraiLinkOptions kEmpty{};
  return impl_ ? impl_->options() : kEmpty;
}

Status MoraiLink::wait_connected(std::chrono::milliseconds timeout)
{
  return no_throw<Status>("wait_connected", [&] {
    return impl_ ? impl_->wait_connected(timeout) : not_initialised();
  });
}

ChannelState MoraiLink::channel_state(LinkChannel channel, bool try_to_connect) const
{
  try {
    return impl_ ? impl_->channel_state(channel, try_to_connect) : ChannelState::kShutdown;
  } catch (...) {
    return ChannelState::kShutdown;
  }
}

Reply<int64_t> MoraiLink::wait_for_tick(std::chrono::milliseconds deadline)
{
  return no_throw<Reply<int64_t>>("wait_for_tick", [&] {
    return impl_ ? impl_->wait_for_tick(deadline) : Reply<int64_t>{not_initialised(), 0};
  });
}

Status MoraiLink::get_snapshot(Snapshot * out)
{
  return no_throw<Status>(
    "get_snapshot", [&] { return impl_ ? impl_->get_snapshot(out) : not_initialised(); });
}

Reply<Snapshot> MoraiLink::get_snapshot()
{
  return no_throw<Reply<Snapshot>>("get_snapshot", [&] {
    Reply<Snapshot> r;
    r.status = impl_ ? impl_->get_snapshot(&r.value) : not_initialised();
    return r;
  });
}

Reply<EgoSample> MoraiLink::get_ego_state()
{
  return no_throw<Reply<EgoSample>>("get_ego_state", [&] {
    return impl_ ? impl_->get_ego_state() : Reply<EgoSample>{not_initialised(), {}};
  });
}

Reply<ControlMode> MoraiLink::get_control_mode()
{
  return no_throw<Reply<ControlMode>>("get_control_mode", [&] {
    return impl_ ? impl_->get_control_mode()
                 : Reply<ControlMode>{not_initialised(), ControlMode::kUnspecified};
  });
}

Reply<VehicleSpecInfo> MoraiLink::get_vehicle_spec()
{
  return no_throw<Reply<VehicleSpecInfo>>("get_vehicle_spec", [&] {
    return impl_ ? impl_->get_vehicle_spec() : Reply<VehicleSpecInfo>{not_initialised(), {}};
  });
}

Reply<SyncModeInfo> MoraiLink::get_sync_mode()
{
  return no_throw<Reply<SyncModeInfo>>("get_sync_mode", [&] {
    return impl_ ? impl_->get_sync_mode() : Reply<SyncModeInfo>{not_initialised(), {}};
  });
}

Reply<ServerTimeSample> MoraiLink::probe_server_time()
{
  return no_throw<Reply<ServerTimeSample>>("probe_server_time", [&] {
    return impl_ ? impl_->probe_server_time() : Reply<ServerTimeSample>{not_initialised(), {}};
  });
}

Status MoraiLink::control_vehicle(const VehicleCommand & cmd)
{
  return no_throw<Status>("control_vehicle", [&] {
    return impl_ ? impl_->control_vehicle(cmd, impl_->options().deadlines.command) : not_initialised();
  });
}

Status MoraiLink::control_vehicle(const VehicleCommand & cmd, std::chrono::milliseconds deadline)
{
  return no_throw<Status>("control_vehicle", [&] {
    return impl_ ? impl_->control_vehicle(cmd, std::max(deadline, std::chrono::milliseconds(1)))
                 : not_initialised();
  });
}

Status MoraiLink::set_gear(GearMode gear, std::chrono::milliseconds verify_timeout)
{
  return no_throw<Status>("set_gear", [&] {
    return impl_ ? impl_->set_gear(gear, verify_timeout) : not_initialised();
  });
}

Status MoraiLink::set_control_mode(ControlMode mode, std::chrono::milliseconds verify_timeout)
{
  return no_throw<Status>("set_control_mode", [&] {
    return impl_ ? impl_->set_control_mode(mode, verify_timeout) : not_initialised();
  });
}

Status MoraiLink::set_transform(
  const TransformPose & pose, std::chrono::milliseconds verify_timeout, double tolerance_m,
  double tolerance_deg)
{
  return no_throw<Status>("set_transform", [&] {
    return impl_ ? impl_->set_transform(pose, verify_timeout, tolerance_m, tolerance_deg)
                 : not_initialised();
  });
}

Status MoraiLink::set_tail_light(TurnSignal signal)
{
  return no_throw<Status>(
    "set_tail_light", [&] { return impl_ ? impl_->set_tail_light(signal) : not_initialised(); });
}

LinkCounters MoraiLink::counters() const
{
  return impl_ ? impl_->counters() : LinkCounters{};
}

}  // namespace morai_grpc_vendor
