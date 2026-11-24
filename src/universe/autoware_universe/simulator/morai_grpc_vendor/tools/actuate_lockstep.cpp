// 신규 — 26.09.25 _ YSH _ morai_actuate --lockstep: MORAI 동기 모드에서 Tick 단위로 Ego 를 구동·측정 (ADS 가 MORAI 를 페이싱)
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

// morai_actuate --lockstep — the ADS paces MORAI. MORAI runs in SYNCHRONOUS mode and only
// advances when this tool calls Simulation.Tick, one command per physics step. Sim time may
// run slower than real time, but every measurement is in exact sim time.
//
// This file is the ONLY place in morai_actuate that calls SetSynchronousMode and Tick
// (test/test_actuate_scope.cpp): the real-time path (morai_actuate.cpp) never does, and
// WaitForTick is not used here at all (it returns 0 at once in synchronous mode).
// Allowed RPCs: SetSynchronousMode, Tick, ControlVehicle (steer always 0.0, ACCELERATION
// only), SetVehicleGear, SetVehicleControlMode (restore only, after leaving synchronous
// mode — restoring the original, or AUTO for a real-time brake when the Ego still moves
// after a failed Tick safe-stop), plus reads.
//
// Verified MORAI facts this relies on (README "lockstep mode"):
//   - SetSynchronousMode's Result says status 0 "Unknown error" even when applied: verify by
//     GetSynchronousMode + frozen timestamps + Tick behaviour, never by the Result.
//   - The gRPC server is UNAVAILABLE for ~1 s after switching to SYNCHRONOUS: retry.
//   - SYNCHRONOUS: world frozen until Tick; ActorStates.timestamp = sim time since entering
//     (µs, not epoch); the Ego's control mode is forced to 7 (SYNCHRONOUS_MODE) and restored
//     by the server on exit; ControlVehicle only works in mode 3 or 7.
//   - Queued ControlVehicle is applied once at the start of the next Tick (last one wins).
//   - Tick(n) blocks until the frames are simulated; a read after it sees the new frame.
//   - If this client dies in synchronous mode MORAI stays frozen: morai_sync_restore.
//
// Safety: same arming flag (plus --lockstep), countdown, envelope (speed, distance, reads,
// collision, steer echo; "frozen" counted per Tick), safe-stop by brake + Tick(1), and a
// RAII restore on every exit path incl. SIGINT/TERM/HUP/QUIT/TSTP: safe-stop -> original
// gear (only after a confirmed standstill) -> SetSynchronousMode(UNSPECIFIED) -> still
// moving: real-time brake at once -> verify real time -> original mode -> gear (only at
// standstill, only in verified real time) -> standstill check. Each step is isolated
// (an exception in one does not skip leaving synchronous mode).

#include "actuate_lockstep.hpp"

#include "actuate_analysis.hpp"
#include "actuate_options.hpp"
#include "lockstep_analysis.hpp"
#include "probe_json.hpp"
#include "probe_stats.hpp"
#include "sync_mode.hpp"

#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/actor/actor_control.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/actor/actor_get.pb.h>
#include <morai/actor/actor_set.pb.h>
#include <morai/common/enum.pb.h>
#include <morai/common/object_info.pb.h>
#include <morai/common/result.pb.h>
#include <morai/common/type.pb.h>
#include <morai/simulation/simulation.grpc.pb.h>
#include <morai/simulation/simulation_enum.pb.h>
#include <morai/simulation/sync_mode.pb.h>
#include <morai/simulator/simulator.grpc.pb.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::actuate
{

namespace
{

namespace ls = lockstep;
namespace sm = morai_grpc_vendor::sync_mode;
namespace pb_actor = morai_sim_api::actor;
namespace pb_sim = morai_sim_api::simulation;
using probe::grpc_code_name;
using probe::Json;
using probe::RawTable;
using Cell = RawTable::Cell;
using Clock = std::chrono::steady_clock;

static_assert(
  sm::kSyncUnspecified == static_cast<int>(pb_sim::SYNC_MODE_TYPE_UNSPECIFIED) &&
    sm::kSyncTimeManagement == static_cast<int>(pb_sim::SYNC_MODE_TYPE_TIME_MANAGEMENT) &&
    sm::kSyncSynchronous == static_cast<int>(pb_sim::SYNC_MODE_TYPE_SYNCHRONOUS),
  "sync_mode constants must match simulation_enum.proto");
static_assert(
  sm::kModeKeyboard == static_cast<int>(pb_actor::VEHICLE_CONTROL_KEYBOARD) &&
    sm::kModeAuto == static_cast<int>(pb_actor::VEHICLE_CONTROL_AUTO_MODE) &&
    sm::kModeSynchronous == static_cast<int>(pb_actor::VEHICLE_CONTROL_SYNCHRONOUS_MODE) &&
    kModeKeyboard == static_cast<int>(pb_actor::VEHICLE_CONTROL_KEYBOARD) &&
    kModeGameWheel == static_cast<int>(pb_actor::VEHICLE_CONTROL_GAME_WHEEL),
  "mode constants must match actor_enum.proto");

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;  // same as MORAI's Python SDK

// =============================================================================
// Signals — the handler only stores the number (same pattern as the real-time path)
// =============================================================================

static_assert(std::atomic<int>::is_always_lock_free, "signal flag must be lock-free");
std::atomic<int> g_signal{0};
std::atomic<int> g_signal_count{0};

void on_signal(int sig)
{
  g_signal.store(sig);
  g_signal_count.fetch_add(1);
}

void install_signal_handlers()
{
  struct sigaction sa
  {
  };
  sa.sa_handler = on_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);
  sigaction(SIGHUP, &sa, nullptr);
  // Ctrl-\ · Ctrl-Z 도 중단 → 복구. 멈춘 채 두면 MORAI 가 동기 모드로 얼어 있다
  sigaction(SIGQUIT, &sa, nullptr);
  sigaction(SIGTSTP, &sa, nullptr);
  std::signal(SIGPIPE, SIG_IGN);
}

bool stop_requested()
{
  return g_signal.load() != 0;
}

std::string signal_text()
{
  switch (g_signal.load()) {
    case SIGINT:
      return "SIGINT";
    case SIGTERM:
      return "SIGTERM";
    case SIGHUP:
      return "SIGHUP";
    case SIGQUIT:
      return "SIGQUIT";
    case SIGTSTP:
      return "SIGTSTP";
    default:
      return "signal " + std::to_string(g_signal.load());
  }
}

// =============================================================================
// Helpers
// =============================================================================

const Clock::time_point g_t0 = Clock::now();

double now_ms()
{
  return std::chrono::duration<double, std::milli>(Clock::now() - g_t0).count();
}

double unix_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::chrono::system_clock::time_point deadline_in(int ms)
{
  return std::chrono::system_clock::now() + std::chrono::milliseconds(ms);
}

void sleep_ms(double ms)
{
  std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(ms));
}

/// Sleeps in small slices; false as soon as a signal arrives.
bool interruptible_sleep(double ms)
{
  const double until = now_ms() + ms;
  while (now_ms() < until) {
    if (stop_requested()) {
      return false;
    }
    sleep_ms(10.0);
  }
  return !stop_requested();
}

std::string enum_name(const google::protobuf::EnumDescriptor * d, int v)
{
  const google::protobuf::EnumValueDescriptor * vd = d ? d->FindValueByNumber(v) : nullptr;
  return vd ? vd->name() : "UNKNOWN(" + std::to_string(v) + ")";
}

std::string mode_name(int m)
{
  return enum_name(pb_actor::VehicleControlMode_descriptor(), m);
}

std::string gear_name(int g)
{
  return enum_name(pb_actor::GearMode_descriptor(), g);
}

bool iequals(const std::string & a, const std::string & b)
{
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) ==
                  std::tolower(static_cast<unsigned char>(y));
         });
}

std::string f2(double v)
{
  char b[32];
  if (!std::isfinite(v)) {
    return "-";
  }
  std::snprintf(b, sizeof(b), "%.2f", v);
  return b;
}

std::string f3(double v)
{
  char b[32];
  if (!std::isfinite(v)) {
    return "-";
  }
  std::snprintf(b, sizeof(b), "%.3f", v);
  return b;
}

enum Phase : int {
  kPreflight = 0,
  kEnter,
  kSetup,
  kHold,
  kIdleProbe,
  kPreHold,
  kDrive,
  kBrake,
  kSettle,
  kRestore,
};

const char * phase_name(int p)
{
  static const char * k[] = {"preflight", "enter",    "setup", "hold",   "idle_probe",
                             "pre_hold",  "drive",    "brake", "settle", "restore"};
  return (p >= 0 && p <= kRestore) ? k[p] : "?";
}

grpc::ChannelArguments channel_args()
{
  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);
  args.SetInt(GRPC_ARG_USE_LOCAL_SUBCHANNEL_POOL, 1);
  // 동기 모드 전환 직후 MORAI gRPC 서버가 ~1 s 사라진다. 기본 재연결 백오프(1 s~)면 복귀를 늦게
  // 알아채므로 짧게 잡는다
  args.SetInt(GRPC_ARG_INITIAL_RECONNECT_BACKOFF_MS, 100);
  args.SetInt(GRPC_ARG_MIN_RECONNECT_BACKOFF_MS, 100);
  args.SetInt(GRPC_ARG_MAX_RECONNECT_BACKOFF_MS, 500);
  return args;
}

// =============================================================================
// The only place a VehicleCtrlCmd is built. Steering is ALWAYS 0 in this tool.
// =============================================================================

pb_actor::VehicleCtrlCmd make_cmd(const morai_sim_api::ObjectInfo & ego, double accel)
{
  // main.py _fill_longitudinal 기본 모드와 같다. frame 은 gRPC 경로에서 무시된다(서버 IL)
  pb_actor::VehicleCtrlCmd c;
  *c.mutable_actor_info() = ego;
  c.set_long_cmd_type(pb_actor::LONG_CMD_TYPE_ACCELERATION);
  c.set_acceleration(accel);
  c.set_throttle(0.0);
  c.set_brake(0.0);
  c.set_steer(0.0);
  c.set_velocity(0.0);
  c.set_frame(0);
  return c;
}

// =============================================================================
// Client — one channel, every call has a deadline
// =============================================================================

struct EgoState
{
  bool ok{false};
  std::string err;
  int64_t ts_us{0};
  uint64_t tick_count{0};
  double vx_kmh{0.0};
  double vy_kmh{0.0};
  double speed_mps{0.0};
  double planar_mps{0.0};
  double x{0.0};
  double y{0.0};
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double fwa{0.0};
  int gear{-1};
  int collisions{0};
};

struct CallResult
{
  bool rpc_ok{false};
  int grpc_code{-1};
  std::string grpc_msg;
  int status{-1};  ///< Result.status, -1 when no response
  std::string desc;
  double ms{0.0};
  bool success() const
  {
    return rpc_ok && status == static_cast<int>(morai_sim_api::STATUS_CODE_SUCCESS);
  }
  /// "Result.status=0 'Unknown error'" or "UNAVAILABLE 'failed to connect ...'".
  std::string brief() const
  {
    if (!rpc_ok) {
      return grpc_code_name(grpc_code) + (grpc_msg.empty() ? "" : " '" + grpc_msg + "'");
    }
    return "Result.status=" + std::to_string(status) + " '" + desc + "'";
  }
  std::string text(const std::string & name) const { return name + " " + brief(); }
  Json json() const
  {
    return Json::object()
      .set("grpc_code", grpc_code_name(grpc_code))
      .set("grpc_message", grpc_msg)
      .set("result_status", status)
      .set("result_description", desc)
      .set("rpc_ms", ms);
  }
};

struct TickRpc
{
  bool ok{false};
  int grpc_code{-1};
  std::string msg;
  uint64_t frame_count{0};
  int64_t elapsed_time{0};
  double wall_ms{0.0};
};

class Client
{
public:
  Client(const Options & o, const morai_sim_api::ObjectInfo & ego) : opt_(o), ego_(ego)
  {
    channel_ = grpc::CreateCustomChannel(
      opt_.host + ":" + std::to_string(opt_.port), grpc::InsecureChannelCredentials(),
      channel_args());
    actor_stub_ = pb_actor::Actor::NewStub(channel_);
    sim_stub_ = pb_sim::Simulation::NewStub(channel_);
    // NOTE upstream spells this package "simualtor" (sic).
    simr_stub_ = morai_sim_api::simualtor::Simulator::NewStub(channel_);
  }

  bool connect(int timeout_ms) { return channel_->WaitForConnected(deadline_in(timeout_ms)); }

  // --- reads -------------------------------------------------------------------
  bool get_version(std::string * out, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Empty req;
    morai_sim_api::StringValue res;
    const grpc::Status st = simr_stub_->GetSimulatorVersion(&ctx, req, &res);
    if (!st.ok()) {
      *err = "GetSimulatorVersion " + grpc_code_name(st.error_code());
      return false;
    }
    *out = res.value();
    return true;
  }

  /// GetSynchronousMode (type only; MORAI always reports tick_period 0).
  grpc::Status get_sync(int * type, int deadline_ms)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(deadline_ms));
    morai_sim_api::Empty req;
    pb_sim::SyncMode res;
    const grpc::Status st = sim_stub_->GetSynchronousMode(&ctx, req, &res);
    if (st.ok()) {
      *type = static_cast<int>(res.type());
    }
    return st;
  }

  bool get_mode(int * mode, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    pb_actor::CurVehicleControlMode res;
    const grpc::Status st = actor_stub_->GetVehicleControlMode(&ctx, ego_, &res);
    if (!st.ok()) {
      *err = "GetVehicleControlMode " + grpc_code_name(st.error_code());
      return false;
    }
    *mode = static_cast<int>(res.mode());
    return true;
  }

  EgoState read_state()
  {
    EgoState e;
    pb_actor::GetAllActorsFilter filter;
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);
    filter.set_pedestrian(false);
    filter.set_obstacle(false);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    pb_actor::ActorStates res;
    const grpc::Status st = actor_stub_->GetAllActorsState(&ctx, filter, &res);
    if (!st.ok()) {
      e.err = "GetAllActorsState " + grpc_code_name(st.error_code());
      return e;
    }
    const pb_actor::ActorState * s = nullptr;
    for (const auto & a : res.states()) {
      if (a.actor_info().id().value() == opt_.ego) {
        s = &a;
        break;
      }
    }
    if (!s) {
      for (const auto & a : res.states()) {
        if (iequals(a.actor_info().id().value(), opt_.ego)) {
          s = &a;
          break;
        }
      }
    }
    if (!s) {
      e.err = "Ego '" + opt_.ego + "' not in GetAllActorsState";
      return e;
    }
    e.ok = true;
    e.ts_us = res.timestamp();
    e.tick_count = res.sync_mode_tick_count();
    e.vx_kmh = s->velocity().x();
    e.vy_kmh = s->velocity().y();
    e.speed_mps = e.vx_kmh * kKmhToMps;
    e.planar_mps = std::hypot(e.vx_kmh, e.vy_kmh) * kKmhToMps;
    e.x = s->transform().location().x();
    e.y = s->transform().location().y();
    const auto & vs = s->vehicle_state();
    e.throttle = vs.throttle();
    e.brake = vs.brake();
    e.steer = vs.steer();
    e.fwa = vs.front_wheel_angle();
    e.gear = s->has_vehicle_state() ? static_cast<int>(vs.gear_mode()) : -1;
    e.collisions = vs.collision_objects_size();
    return e;
  }

  // --- writes (deadline; the caller decides whether the Result can be trusted) -------
  /// SetSynchronousMode. MORAI's Result is NOT trustworthy here (status 0 even when applied).
  CallResult set_sync(int type, int tick_period_ms)
  {
    pb_sim::SyncMode req;
    req.set_type(static_cast<pb_sim::SyncModeType>(type));
    req.set_tick_period(tick_period_ms);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(std::max(opt_.deadline_ms, 2000)));
    morai_sim_api::Result res;
    const double a = now_ms();
    const grpc::Status st = sim_stub_->SetSynchronousMode(&ctx, req, &res);
    return finish(st, res, a);
  }

  TickRpc tick(int frames)
  {
    TickRpc t;
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.tick_deadline_ms));
    morai_sim_api::Int32Value req;
    req.set_value(frames);
    pb_sim::SyncTimestamp res;
    const double a = now_ms();
    const grpc::Status st = sim_stub_->Tick(&ctx, req, &res);
    t.wall_ms = now_ms() - a;
    t.ok = st.ok();
    t.grpc_code = static_cast<int>(st.error_code());
    t.msg = st.error_message();
    t.frame_count = res.frame_count();
    t.elapsed_time = res.elapsed_time();
    return t;
  }

  CallResult control(double accel)
  {
    const pb_actor::VehicleCtrlCmd cmd = make_cmd(ego_, accel);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const double a = now_ms();
    const grpc::Status st = actor_stub_->ControlVehicle(&ctx, cmd, &res);
    return finish(st, res, a);
  }

  CallResult set_gear(int gear)
  {
    pb_actor::VehicleGear req;
    *req.mutable_actor_info() = ego_;
    req.set_gear(static_cast<pb_actor::GearMode>(gear));
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const double a = now_ms();
    const grpc::Status st = actor_stub_->SetVehicleGear(&ctx, req, &res);
    return finish(st, res, a);
  }

  /// Restore only (after leaving synchronous mode): the server forces 7 while synchronous.
  CallResult set_mode(int mode)
  {
    pb_actor::VehicleControlModeParam req;
    *req.mutable_actor_info() = ego_;
    req.set_mode(static_cast<pb_actor::VehicleControlMode>(mode));
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const double a = now_ms();
    const grpc::Status st = actor_stub_->SetVehicleControlMode(&ctx, req, &res);
    return finish(st, res, a);
  }

private:
  static CallResult finish(const grpc::Status & st, const morai_sim_api::Result & res, double t0)
  {
    CallResult c;
    c.ms = now_ms() - t0;
    c.rpc_ok = st.ok();
    c.grpc_code = static_cast<int>(st.error_code());
    c.grpc_msg = st.error_message();
    if (st.ok()) {
      c.status = static_cast<int>(res.status());
      c.desc = res.description();
    }
    return c;
  }

  const Options & opt_;
  morai_sim_api::ObjectInfo ego_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<pb_actor::Actor::Stub> actor_stub_;
  std::unique_ptr<pb_sim::Simulation::Stub> sim_stub_;
  std::unique_ptr<morai_sim_api::simualtor::Simulator::Stub> simr_stub_;
};

// =============================================================================
// Session
// =============================================================================

/// Brake Ticks of a safe-stop or a hold.
struct BrakeStats
{
  bool ok{false};  ///< safe-stop: stopped for 1 s sim; hold: >= 1 command accepted
  size_t ticks{0};
  size_t sends{0};
  size_t accepted{0};
  double sim_s{0.0};
  double took_ms{kNaN};
  std::string why;
  std::string last_error;

  std::string warning() const
  {
    if (sends > 0 && accepted == 0) {
      return "0/" + std::to_string(sends) + " brake commands accepted (" +
             (last_error.empty() ? std::string("?") : last_error) +
             ") - the Ego was NOT braked by this tool";
    }
    return "";
  }
  std::string counts() const
  {
    return std::to_string(accepted) + "/" + std::to_string(sends) + " brake commands accepted";
  }
  Json json() const
  {
    return Json::object()
      .set("ok", ok)
      .set("ticks", ticks)
      .set("sends", sends)
      .set("accepted", accepted)
      .set("sim_s", sim_s)
      .set("took_ms", took_ms)
      .set("why", why)
      .set("last_error", last_error)
      .set("warning", warning());
  }
};

struct RunRecord
{
  int index{0};
  std::string pattern;
  int repeat{0};
  size_t step_ref{0};  ///< index of the reference Tick (state at t0) in the step log
  size_t step_end{0};  ///< one past the last drive Tick
  int64_t t0_us{0};
  int64_t drive_end_us{0};
  size_t ticks{0};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string abort_reason;
  BrakeStats pre_hold;
  BrakeStats stop;
  BrakeStats settle;
  std::string brake_warning;
  double max_speed{0.0};
  double max_dist{0.0};
  ls::RunAnalysis analysis;
};

struct Preflight
{
  bool ok{false};
  std::string why;
  int mode{-1};
  int gear{-1};
  double x{0.0};
  double y{0.0};
  bool epoch{false};
  std::string version;
  Json j = Json::object();
};

struct ExitResult
{
  bool ok{false};
  int attempts{0};
  int final_type{-1};
  bool already{false};
  double took_s{0.0};
  std::string last_error;
};

class Session
{
public:
  Session(const Options & o, Client & c) : opt_(o), client_(c)
  {
    limits_.max_speed_mps = opt_.max_speed;
    limits_.max_distance_m = opt_.max_distance;
    limits_.max_stale_s = Safety::kMaxStaleS;
    limits_.max_frozen_s = std::numeric_limits<double>::infinity();  // per Tick instead
  }

  const std::vector<ls::Step> & steps() const { return steps_; }
  const std::vector<ls::TickClass> & classes() const { return classes_; }
  bool sync_touched() const { return sync_touched_; }
  bool sync_verified() const { return sync_verified_; }
  bool gear_touched() const { return gear_touched_; }
  bool drove() const { return drove_; }
  const ls::Step & last_good() const { return last_good_; }
  bool have_last_good() const { return have_last_good_; }

  // ---------------------------------------------------------------------------
  /// Read-only, in real time. Nothing is changed here.
  Preflight preflight()
  {
    Preflight p;
    std::string err;
    if (client_.get_version(&p.version, &err)) {
      std::printf("[preflight] MORAI version %s\n", p.version.c_str());
    } else {
      std::printf("[preflight] %s (continuing)\n", err.c_str());
    }
    p.j.set("simulator_version", p.version);

    int type = -1;
    const grpc::Status st = client_.get_sync(&type, opt_.deadline_ms);
    if (!st.ok()) {
      p.why = "GetSynchronousMode " + grpc_code_name(st.error_code()) +
              ": cannot confirm MORAI runs in real time, so it could not be restored";
      return p;
    }
    p.j.set("sync_type", sm::sync_type_name(type));
    std::printf("[preflight] sync mode %s\n", sm::sync_type_name(type).c_str());
    if (type != sm::kSyncUnspecified) {
      p.why = "MORAI is in " + sm::sync_type_name(type) +
              " already: another client may own it (or a previous run died). If nobody is "
              "using it, run morai_sync_restore first";
      return p;
    }

    // 0.6 s 실시간 관측: 시간이 흐르는지, 다른 클라이언트가 몰고 있지 않은지 (1b 와 같은 판정)
    EgoState last;
    const double t_first = now_ms();
    while (now_ms() - t_first < 2000.0) {
      last = client_.read_state();
      if (last.ok || stop_requested()) {
        break;
      }
      sleep_ms(10.0);
    }
    if (!last.ok) {
      p.why = "no Ego state within 2 s (" + last.err + ")";
      return p;
    }
    std::vector<sm::TimeSample> ts;
    std::vector<InputRecord> recs;
    int64_t prev_ts = last.ts_us;
    const double t0 = now_ms();
    ts.push_back({t0 / 1e3, last.ts_us});
    while (now_ms() - t0 < 600.0) {
      if (stop_requested()) {
        p.why = "interrupted (" + signal_text() + ")";
        return p;
      }
      const EgoState e = client_.read_state();
      if (e.ok) {
        last = e;
        if (e.ts_us != prev_ts) {
          prev_ts = e.ts_us;
          ts.push_back({now_ms() / 1e3, e.ts_us});
          recs.push_back({e.throttle, e.brake, e.steer, e.fwa});
        }
      }
      sleep_ms(1.0);
    }
    const size_t fresh = recs.size();
    const sm::RealtimeCheck rt = sm::judge_realtime(ts, unix_now_s(), 0.5);
    p.epoch = rt.epoch;
    const ForeignInputCheck foreign = check_foreign_input(recs, 0.2);
    p.j.set("updates_in_600ms", fresh)
      .set("rate", rt.rate)
      .set("epoch_timestamps", rt.epoch)
      .set("clock_offset_s", rt.offset_s)
      .set("active_pedal_records", foreign.pedal)
      .set("steer_echo_records", foreign.steer);
    if (fresh < 5) {
      p.why = "MORAI time is not advancing (" + std::to_string(fresh) +
              " updates in 0.6 s): paused?";
      return p;
    }
    if (!rt.epoch) {
      std::printf(
        "[preflight] WARNING: real-time timestamps are not Unix-epoch µs (%lld) - the restore "
        "check will not require epoch timestamps\n",
        static_cast<long long>(last.ts_us));
    }
    if (foreign.driving) {
      p.why = foreign.why;
      return p;
    }
    p.gear = last.gear;
    p.x = last.x;
    p.y = last.y;
    const double v = std::fabs(last.planar_mps);
    p.j.set("speed_mps", v).set("gear", gear_name(p.gear)).set("x", p.x).set("y", p.y);
    if (!(v < Safety::kStopSpeed)) {
      p.why = "Ego is moving (" + std::to_string(v) + " m/s); it must be at standstill";
      return p;
    }
    if (p.gear < 0) {
      p.why = "Ego has no vehicle_state (gear unknown, cannot restore it)";
      return p;
    }
    if (!client_.get_mode(&p.mode, &err)) {
      p.why = err + " (cannot record the original mode, so it could not be restored)";
      return p;
    }
    p.j.set("mode", mode_name(p.mode));
    std::printf(
      "[preflight] Ego at (%.2f, %.2f), %.3f m/s, gear %s, mode %s, %zu updates in 0.6 s "
      "(rate %s x wall)\n",
      p.x, p.y, v, gear_name(p.gear).c_str(), mode_name(p.mode).c_str(), fresh,
      f3(rt.rate).c_str());
    const std::string refusal = original_mode_refusal(p.mode, mode_name(p.mode), opt_.allow_any_mode);
    if (!refusal.empty()) {
      p.why = refusal;
      return p;
    }
    if (p.mode != kModeKeyboard && p.mode != kModeGameWheel) {
      std::printf(
        "[preflight] WARNING: taking over from %s (--allow-any-original-mode); make sure nothing "
        "else sends ControlVehicle\n",
        mode_name(p.mode).c_str());
    }
    p.ok = true;
    return p;
  }

  // ---------------------------------------------------------------------------
  /// One Tick: optional ControlVehicle (queued, applied at the Tick's start), Tick(frames),
  /// then the state read. Logged, classified against the previous Tick, envelope maxima.
  ls::Step step(int run, int phase, bool command, double accel, int frames)
  {
    ls::Step s;
    s.run = run;
    s.phase = phase;
    s.frames = frames;
    if (command) {
      const CallResult c = client_.control(accel);
      s.commanded = true;
      s.command_ok = c.success();
      s.accel = accel;
      if (c.success()) {
        consecutive_cmd_failures_ = 0;
      } else {
        ++consecutive_cmd_failures_;
        last_cmd_error_ = c.text("ControlVehicle");
      }
      drove_ = drove_ || accel > 0.0;
    }
    const TickRpc t = client_.tick(frames);
    s.tick_ok = t.ok;
    s.grpc_code = t.grpc_code;
    s.frame_count = t.frame_count;
    s.elapsed_time = t.elapsed_time;
    s.tick_wall_ms = t.wall_ms;
    if (!t.ok) {
      last_tick_error_ = "Tick " + grpc_code_name(t.grpc_code) + (t.msg.empty() ? "" : " '" + t.msg + "'");
    }
    EgoState e = client_.read_state();
    // Tick 은 스냅샷 갱신까지 막는다(실측). 그래도 프레임이 늘었는데 timestamp 가 그대로면 짧게 다시 읽는다
    if (t.ok && e.ok && have_prev_ && t.frame_count != prev_.frame_count && e.ts_us == prev_.ts_us) {
      for (int k = 0; k < 3 && e.ok && e.ts_us == prev_.ts_us; ++k) {
        sleep_ms(2.0);
        e = client_.read_state();
        ++s.read_retries;
      }
    }
    s.read_ok = e.ok;
    s.wall_ms = now_ms();
    if (e.ok) {
      s.ts_us = e.ts_us;
      s.tick_count = e.tick_count;
      s.speed_mps = e.speed_mps;
      s.planar_mps = e.planar_mps;
      s.x = e.x;
      s.y = e.y;
      s.throttle = e.throttle;
      s.brake = e.brake;
      s.steer = e.steer;
      s.fwa = e.fwa;
      s.gear = e.gear;
      s.collisions = e.collisions;
      last_read_ok_ms_ = s.wall_ms;
    } else {
      last_read_error_ = e.err;
    }
    ls::TickClass cls = ls::TickClass::kOk;
    if (!s.tick_ok) {
      cls = ls::TickClass::kFailed;
    } else if (!s.read_ok) {
      cls = ls::TickClass::kNoRead;
    } else if (have_prev_ && prev_.tick_ok && prev_.read_ok) {
      cls = ls::classify_tick(prev_, s, period_ms());
    } else if (have_last_good_) {
      // 앞 Tick 이 실패했거나 읽지 못했다: 마지막 정상 Tick 과 비교 — 그대로면 진행 안 함(실패·성공이 번갈아도 센다)
      cls = ls::classify_after_gap(last_good_, s, frames_since_good_ + frames, period_ms());
    }
    health_.update(cls);
    steps_.push_back(s);
    classes_.push_back(cls);
    prev_ = s;
    have_prev_ = true;
    if (s.tick_ok && s.read_ok) {
      last_good_ = s;
      have_last_good_ = true;
      frames_since_good_ = 0;
    } else {
      frames_since_good_ += frames;
    }
    if (armed_ && s.read_ok) {
      armed_max_speed_ = std::max(armed_max_speed_, std::fabs(s.planar_mps));
      armed_max_dist_ = std::max(armed_max_dist_, std::hypot(s.x - origin_x_, s.y - origin_y_));
      armed_collisions_ = std::max(armed_collisions_, static_cast<size_t>(s.collisions));
      armed_max_steer_ = std::max({armed_max_steer_, std::fabs(s.steer), std::fabs(s.fwa)});
    }
    return s;
  }

  /// Envelope + signal + Tick health + wall budget. "" = keep going.
  std::string check_abort(Clock::time_point start, double wall_budget_s) const
  {
    if (stop_requested()) {
      return signal_text();
    }
    const std::string h = ls::check_tick_health(health_);
    if (!h.empty()) {
      return "MORAI: " + h + (last_tick_error_.empty() ? "" : " (" + last_tick_error_ + ")");
    }
    EnvelopeState st;
    st.have_state = have_last_good_;
    st.speed_mps = armed_ ? armed_max_speed_ : std::fabs(last_good_.planar_mps);
    st.distance_m = armed_ ? armed_max_dist_ : 0.0;
    st.stale_s = (now_ms() - last_read_ok_ms_) / 1e3;
    st.frozen_s = 0.0;  // 동기 모드: 정지는 Tick 단위(TickHealth)로 본다
    st.collisions = armed_ ? armed_collisions_ : 0;
    st.steer_echo = armed_ ? armed_max_steer_ : 0.0;
    const std::string why = check_envelope(limits_, st);
    if (!why.empty()) {
      return "envelope: " + why +
             (st.stale_s > Safety::kMaxStaleS && !last_read_error_.empty() ? " (" + last_read_error_ + ")" : "");
    }
    if (std::chrono::duration<double>(Clock::now() - start).count() > wall_budget_s) {
      return "drive not finished after " + f2(wall_budget_s) + " s wall";
    }
    if (consecutive_cmd_failures_ >= ls::Limits::kMaxFailedCommands) {
      return "ControlVehicle failing: " + last_cmd_error_;
    }
    return "";
  }

  // ---------------------------------------------------------------------------
  /// Brake (-3.0 m/s^2, steer 0) + Tick(1) until |v| < 0.05 m/s for 1.0 s of SIM time, at
  /// most safe_stop_max_ticks(period) Ticks (2000, more for short periods) and `max_wall_s`.
  /// Ignores signals on purpose — it IS the abort path. Gives up when Ticks fail or stop
  /// advancing (MORAI gone / frozen, judged on this safe-stop's own Ticks; a timestamp jump
  /// stays fatal) — restore then leaves synchronous mode and brakes in real time.
  BrakeStats safe_stop(int run, int phase, double max_wall_s = ls::Limits::kSafeStopMaxWallS)
  {
    BrakeStats b;
    const double t_start = now_ms();
    int64_t still_since = -1;
    int64_t ts_start = have_last_good_ ? last_good_.ts_us : 0;
    int seen_signals = g_signal_count.load();
    const int max_ticks = ls::safe_stop_max_ticks(period_ms());
    // 이 안전 정지의 Tick 으로 새로 판정 (앞선 실패 뒤 MORAI 가 회복했으면 브레이크할 기회를 준다). 튐은 그대로 치명
    const bool jumped = health_.jumped;
    health_ = ls::TickHealth{};
    health_.jumped = jumped;
    for (int k = 0; k < max_ticks; ++k) {
      if (now_ms() - t_start > max_wall_s * 1e3) {
        b.why = "wall-time cap " + f2(max_wall_s) + " s reached after " + std::to_string(b.ticks) +
                " Ticks (|v|=" + f3(std::fabs(last_good_.planar_mps)) + " m/s)";
        break;
      }
      const ls::Step s = step(run, phase, true, Safety::kSafeStopAccel, 1);
      ++b.ticks;
      ++b.sends;
      if (s.command_ok) {
        ++b.accepted;
      } else {
        b.last_error = last_cmd_error_;
      }
      if (g_signal_count.load() != seen_signals) {
        seen_signals = g_signal_count.load();
        std::printf("[safe-stop] still braking by Ticks - restore follows, please wait\n");
        std::fflush(stdout);
      }
      const std::string h = ls::check_tick_health(health_);
      if (!h.empty()) {
        b.why = "MORAI: " + h + (last_tick_error_.empty() ? "" : " (" + last_tick_error_ + ")");
        break;
      }
      if (s.tick_ok && s.read_ok) {
        if (ts_start == 0) {
          ts_start = s.ts_us;
        }
        b.sim_s = static_cast<double>(s.ts_us - ts_start) / 1e6;
        if (std::fabs(s.planar_mps) < Safety::kStopSpeed) {
          if (still_since < 0) {
            still_since = s.ts_us;
          }
          if (static_cast<double>(s.ts_us - still_since) >= Safety::kStopHoldS * 1e6) {
            b.ok = true;
            break;
          }
        } else {
          still_since = -1;
        }
      } else if (now_ms() - last_read_ok_ms_ > 2000.0) {
        b.why = "state reads failing for > 2 s (" + last_read_error_ + ")";
        break;
      }
    }
    b.took_ms = now_ms() - t_start;
    if (!b.ok && b.why.empty()) {
      b.why = "not stopped after " + std::to_string(b.ticks) + " Ticks (|v|=" +
              f3(std::fabs(last_good_.planar_mps)) + " m/s, " + b.counts() + ")";
    }
    last_stop_ = b;
    return b;
  }

  /// Brake + Tick(1) for `sim_s` of sim time. Stops early on a signal (not in restore).
  BrakeStats hold(int run, int phase, double sim_s)
  {
    BrakeStats b;
    const double t_start = now_ms();
    const int n = std::max(1, static_cast<int>(std::lround(sim_s * 1000.0 / period_ms())));
    for (int k = 0; k < n && !stop_requested(); ++k) {
      const ls::Step s = step(run, phase, true, Safety::kSafeStopAccel, 1);
      ++b.ticks;
      ++b.sends;
      if (s.command_ok) {
        ++b.accepted;
      } else {
        b.last_error = last_cmd_error_;
      }
      if (!ls::check_tick_health(health_).empty()) {
        b.why = ls::check_tick_health(health_);
        break;
      }
    }
    b.took_ms = now_ms() - t_start;
    b.sim_s = static_cast<double>(b.ticks) * period_ms() / 1e3;
    b.ok = b.accepted > 0 && b.why.empty();
    return b;
  }

  // ---------------------------------------------------------------------------
  /// SetSynchronousMode(SYNCHRONOUS) and VERIFY it by behaviour (never by the Result).
  bool enter(Json * j, std::string * why)
  {
    if (stop_requested()) {
      *why = signal_text() + " before any change";
      return false;
    }
    const int period = opt_.tick_period_ms;
    // 1. 전환 (Result 는 적용돼도 status 0 'Unknown error' — 기록만 하고 무시)
    sync_touched_ = true;
    const CallResult r = client_.set_sync(sm::kSyncSynchronous, period);
    std::printf(
      "[enter] SetSynchronousMode(SYNCHRONOUS, %d ms) -> %s (not trusted - verifying by "
      "behaviour)\n",
      period, r.brief().c_str());
    j->set("set_sync", r.json());
    // 2. GetSynchronousMode 가 SYNCHRONOUS 가 될 때까지 (서버가 ~1 s UNAVAILABLE)
    const double t0 = now_ms();
    int type = -1;
    int fails = 0;
    std::string last_err;
    const std::vector<int> delays = sm::backoff_delays_ms(
      static_cast<int>(ls::Limits::kEnterRetryS * 1000.0), 50, 500);
    bool answered = false;
    for (size_t k = 0; k <= delays.size(); ++k) {
      const grpc::Status st = client_.get_sync(&type, 300);
      if (st.ok()) {
        answered = true;
        if (type == sm::kSyncSynchronous) {
          break;
        }
      } else {
        ++fails;
        last_err = grpc_code_name(st.error_code()) + " '" + st.error_message() + "'";
      }
      if (k == delays.size() || now_ms() - t0 > ls::Limits::kEnterRetryS * 1e3) {
        break;
      }
      sleep_ms(delays[k]);
    }
    const double took = (now_ms() - t0) / 1e3;
    std::printf(
      "[enter] GetSynchronousMode -> %s after %.2f s (%d failed attempts%s%s)\n",
      answered ? sm::sync_type_name(type).c_str() : "NO ANSWER", took, fails,
      last_err.empty() ? "" : ", last: ", last_err.c_str());
    j->set("get_sync", Json::object()
                         .set("type", answered ? sm::sync_type_name(type) : std::string("none"))
                         .set("took_s", took)
                         .set("failed_attempts", fails)
                         .set("last_error", last_err));
    if (!answered || type != sm::kSyncSynchronous) {
      *why = "MORAI did not report SYNCHRONOUS within " + f2(ls::Limits::kEnterRetryS) + " s (" +
             (answered ? sm::sync_type_name(type) : "no answer: " + last_err) + ")";
      return false;
    }
    sync_verified_ = true;
    // 3. Tick 전에는 세계가 멈춰 있어야 한다 (>= 0.3 s)
    std::vector<sm::TimeSample> samples;
    EgoState ref;
    const double tf = now_ms();
    while (now_ms() - tf < 2000.0) {
      const EgoState e = client_.read_state();
      if (e.ok) {
        ref = e;
        samples.push_back({now_ms() / 1e3, e.ts_us});
        if (samples.size() >= 3 && samples.back().wall_s - samples.front().wall_s >= ls::Limits::kFrozenCheckS) {
          break;
        }
      }
      sleep_ms(5.0);
    }
    const sm::FrozenCheck fc = sm::judge_frozen(samples, ls::Limits::kFrozenCheckS);
    std::printf(
      "[enter] world %s: timestamp %lld us (%s), %zu reads over %.2f s, %zu changes, "
      "sync_mode_tick_count %llu\n",
      fc.frozen ? "frozen" : "NOT FROZEN", static_cast<long long>(fc.last_us),
      sm::is_epoch_us(fc.last_us) ? "epoch?!" : "sim time since entry", fc.n, fc.span_s,
      fc.changes, static_cast<unsigned long long>(ref.tick_count));
    j->set("frozen", Json::object()
                       .set("frozen", fc.frozen)
                       .set("n", fc.n)
                       .set("span_s", fc.span_s)
                       .set("changes", fc.changes)
                       .set("timestamp_us", fc.last_us)
                       .set("tick_count", ref.tick_count)
                       .set("why", fc.why));
    if (!fc.frozen) {
      *why = "world not frozen after entering SYNCHRONOUS: " + fc.why;
      return false;
    }
    // 4. Tick(1) 두 번: frame_count +1, timestamp 정확히 +tick_period (±1 µs)
    prev_ = ls::Step{};
    prev_.tick_ok = true;
    prev_.read_ok = true;
    prev_.ts_us = ref.ts_us;
    prev_.frame_count = ref.tick_count;  // sync_mode_tick_count = frame_count (실측)
    have_prev_ = true;
    last_good_ = prev_;
    have_last_good_ = true;
    frames_since_good_ = 0;
    last_read_ok_ms_ = now_ms();
    const ls::Step s1 = step(-1, kEnter, false, 0.0, 1);
    const ls::Step s2 = step(-1, kEnter, false, 0.0, 1);
    const int64_t p_us = static_cast<int64_t>(period) * 1000;
    const int64_t d1 = s1.ts_us - ref.ts_us;
    const int64_t d2 = s2.ts_us - s1.ts_us;
    const int64_t df = static_cast<int64_t>(s2.frame_count) - static_cast<int64_t>(s1.frame_count);
    const bool tick_ok = s1.tick_ok && s1.read_ok && s2.tick_ok && s2.read_ok &&
                         std::llabs(d1 - p_us) <= ls::Limits::kDtToleranceUs &&
                         std::llabs(d2 - p_us) <= ls::Limits::kDtToleranceUs && df == 1;
    std::printf(
      "[enter] Tick(1) x2: frame_count %llu -> %llu (+%lld), timestamp +%lld us, +%lld us "
      "(expected +%lld), sync_mode_tick_count %llu, Tick %.1f / %.1f ms -> %s\n",
      static_cast<unsigned long long>(s1.frame_count),
      static_cast<unsigned long long>(s2.frame_count), static_cast<long long>(df),
      static_cast<long long>(d1), static_cast<long long>(d2), static_cast<long long>(p_us),
      static_cast<unsigned long long>(s2.tick_count), s1.tick_wall_ms, s2.tick_wall_ms,
      tick_ok ? "OK" : "FAILED");
    if (tick_ok && s2.tick_count != s2.frame_count) {
      std::printf(
        "[enter] note: sync_mode_tick_count %llu != SyncTimestamp.frame_count %llu\n",
        static_cast<unsigned long long>(s2.tick_count),
        static_cast<unsigned long long>(s2.frame_count));
    }
    j->set("tick_check", Json::object()
                           .set("ok", tick_ok)
                           .set("frame_1", s1.frame_count)
                           .set("frame_2", s2.frame_count)
                           .set("dt1_us", d1)
                           .set("dt2_us", d2)
                           .set("expected_us", p_us)
                           .set("tick_count_2", s2.tick_count)
                           .set("elapsed_time_1", s1.elapsed_time)
                           .set("elapsed_time_2", s2.elapsed_time)
                           .set("tick_ms_1", s1.tick_wall_ms)
                           .set("tick_ms_2", s2.tick_wall_ms)
                           .set("tick_error", last_tick_error_));
    if (!tick_ok) {
      *why = "Tick(1) did not advance frame_count by 1 and the timestamp by exactly " +
             std::to_string(period) + " ms" +
             (s1.tick_ok && s2.tick_ok ? "" : " (" + last_tick_error_ + ")");
      return false;
    }
    // 5. 제어 모드: 서버가 7(SYNCHRONOUS_MODE)로 강제한다. 3(AUTO)도 명령을 받는다. 그 밖이면 명령이 버려진다
    int mode = -1;
    std::string err;
    const bool mode_read = client_.get_mode(&mode, &err);
    std::printf(
      "[enter] control mode %s -> %s\n", mode_read ? mode_name(mode).c_str() : err.c_str(),
      mode_read && sm::mode_accepts_control_in_sync(mode) ? "OK (ControlVehicle is applied)"
                                                          : "NOT USABLE");
    j->set("mode", mode_read ? mode_name(mode) : err);
    if (!mode_read || !sm::mode_accepts_control_in_sync(mode)) {
      *why = "control mode is " + (mode_read ? mode_name(mode) : err) +
             " in SYNCHRONOUS (needs VEHICLE_CONTROL_SYNCHRONOUS_MODE or AUTO_MODE; MORAI would "
             "drop ControlVehicle silently)";
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  /// Gear D (read back after Ticks), idle probe (before any command), brake hold.
  bool setup(Json * j, IdleCalibration * idle, std::string * why)
  {
    const int drive = static_cast<int>(pb_actor::GEAR_MODE_D);
    if (stop_requested()) {
      *why = signal_text();
      return false;
    }
    gear_touched_ = true;
    const CallResult g = client_.set_gear(drive);
    int gear = -1;
    int after = 0;
    for (int k = 1; k <= ls::Limits::kGearReadbackTicks; ++k) {
      const ls::Step s = step(-1, kSetup, false, 0.0, 1);
      if (s.read_ok) {
        gear = s.gear;
      }
      if (gear == drive) {
        after = k;
        break;
      }
    }
    std::printf(
      "[setup] SetVehicleGear(D) -> %s; read back %s after %d Tick(s) %s\n",
      g.brief().c_str(), gear_name(gear).c_str(), after,
      gear == drive ? "- OK" : "- NOT D");
    j->set("gear_set", g.json()).set("gear", gear_name(gear)).set("gear_after_ticks", after);
    if (gear != drive) {
      *why = "gear did not read back as D within " + std::to_string(ls::Limits::kGearReadbackTicks) +
             " Ticks (got " + gear_name(gear) + ", " + g.text("SetVehicleGear") + ")";
      return false;
    }
    if (!g.success()) {
      std::printf("[setup] note: SetVehicleGear Result not SUCCESS, but the gear read back D\n");
    }
    // 명령 없는 Tick: 동기 모드의 유휴 레코드 실측. 이 세션의 첫 ControlVehicle 전이라 MORAI 가 붙들고 있을
    // 입력이 없다 (브레이크 유지 뒤에 재면 유지된 브레이크가 유휴로 잡힐 수 있다). 첫 Tick 은 기어 전환 직후라 버린다
    std::vector<std::pair<double, double>> recs;
    std::vector<InputRecord> inputs;
    double v_end = 0.0;
    const int n = static_cast<int>(std::lround(opt_.idle_probe_s * 1000.0 / period_ms()));
    for (int k = 0; k < n; ++k) {
      if (stop_requested()) {
        *why = signal_text();
        return false;
      }
      const ls::Step s = step(-1, kIdleProbe, false, 0.0, 1);
      if (!ls::check_tick_health(health_).empty()) {
        *why = "idle probe: " + ls::check_tick_health(health_);
        return false;
      }
      if (s.read_ok) {
        v_end = s.planar_mps;
        if (k > 0) {
          recs.emplace_back(s.throttle, s.brake);
          inputs.push_back({0.0, 0.0, s.steer, s.fwa});
        }
      }
    }
    const ForeignInputCheck foreign = check_foreign_input(inputs, 0.2);
    j->set("idle_probe_steer_echo_records", foreign.steer);
    if (foreign.steer > 0) {
      *why = "steer echo in " + std::to_string(foreign.steer) + "/" + std::to_string(foreign.n) +
             " records while this tool sent nothing: another client is commanding the Ego";
      return false;
    }
    IdleSignature fallback;
    fallback.throttle_max = opt_.idle_throttle_max;
    fallback.brake_center = opt_.idle_brake;
    fallback.brake_tol = opt_.idle_brake_tol;
    *idle = ls::calibrate_sync_idle(recs, fallback);
    std::printf(
      "[setup] idle probe (%d Ticks, before any command): record throttle %.4f brake %.4f -> "
      "signature brake %.4f+-%.4f (%s)%s, |v| %.3f m/s\n",
      n, idle->throttle_median, idle->brake_median, idle->signature.brake_center,
      idle->signature.brake_tol, idle->source.c_str(), idle->ambiguous ? " AMBIGUOUS" : "",
      std::fabs(v_end));
    for (const auto & w : idle->warnings) {
      std::printf("[setup] note: %s\n", w.c_str());
    }
    j->set("idle_probe_ticks", n).set("idle_probe_end_speed_mps", v_end);
    if (!(std::fabs(v_end) < Safety::kStopSpeed)) {
      const BrakeStats b = safe_stop(-1, kHold);
      if (!b.ok) {
        *why = "Ego rolls without a command and safe-stop failed: " + b.why;
        return false;
      }
    }
    const BrakeStats held = hold(-1, kHold, 0.5);
    j->set("hold", held.json());
    std::printf("[setup] brake hold %.2f s sim (%zu Ticks): %s\n", held.sim_s, held.ticks, held.counts().c_str());
    if (stop_requested()) {
      *why = signal_text();
      return false;
    }
    if (!held.ok) {
      *why = "brake hold failed: " + (held.why.empty() ? held.warning() : held.why);
      return false;
    }
    return true;
  }

  // ---------------------------------------------------------------------------
  RunRecord run_pattern(const LockstepPatternSpec & p, int repeat, int index)
  {
    RunRecord r;
    r.index = index;
    r.pattern = p.name;
    r.repeat = repeat;
    consecutive_cmd_failures_ = 0;
    if (!have_last_good_ || !(std::fabs(last_good_.planar_mps) < Safety::kStopSpeed)) {
      r.stop = safe_stop(index, kBrake);
      if (!r.stop.ok) {
        r.abort_reason = "not at standstill before the run: " + r.stop.why;
        return r;
      }
    }
    // 모든 런이 같은 상태(브레이크로 잡힌 정지)에서 출발
    r.pre_hold = hold(index, kPreHold, Safety::kPreDriveHoldS);
    if (stop_requested()) {
      r.abort_reason = signal_text();
      r.stop.ok = true;  // 아직 출발 전
      return r;
    }
    if (!r.pre_hold.ok) {
      r.abort_reason = "pre-drive hold failed: " +
                       (r.pre_hold.why.empty() ? r.pre_hold.warning() : r.pre_hold.why);
      return r;
    }
    r.step_ref = steps_.size() - 1;
    while (r.step_ref > 0 && !(steps_[r.step_ref].tick_ok && steps_[r.step_ref].read_ok)) {
      --r.step_ref;
    }
    r.t0_us = last_good_.ts_us;
    arm(&r.origin_x, &r.origin_y);
    r.abort_reason = drive(p, &r);
    r.step_end = steps_.size();
    r.drive_end_us = last_good_.ts_us;
    r.stop = safe_stop(index, kBrake);
    if (!r.stop.warning().empty()) {
      r.brake_warning = "safe-stop: " + r.stop.warning();
    }
    r.max_speed = armed_max_speed_;
    r.max_dist = armed_max_dist_;
    armed_ = false;
    if (r.stop.ok && r.abort_reason.empty() && r.brake_warning.empty() && opt_.settle_s > 0.0) {
      r.settle = hold(index, kSettle, opt_.settle_s);
      if (!r.settle.warning().empty() && !stop_requested()) {
        r.brake_warning = "settle: " + r.settle.warning();
      }
    }
    return r;
  }

  // ---------------------------------------------------------------------------
  /// SetSynchronousMode(UNSPECIFIED, 0) until GetSynchronousMode reads UNSPECIFIED, at most
  /// `total_s`. The Result is ignored (verification is by read-back).
  ExitResult exit_sync(double total_s)
  {
    ExitResult x;
    const double t0 = now_ms();
    const std::vector<int> delays = sm::backoff_delays_ms(static_cast<int>(total_s * 1000.0), 100, 1000);
    // 이미 실시간이면 보내지 않는다 (진입 자체가 적용되지 않았을 수 있다)
    int type = -1;
    if (client_.get_sync(&type, opt_.deadline_ms).ok() && type == sm::kSyncUnspecified) {
      x.ok = true;
      x.already = true;
      x.final_type = type;
      std::printf("[restore] GetSynchronousMode -> UNSPECIFIED already - SetSynchronousMode not sent\n");
      return x;
    }
    for (size_t k = 0;; ++k) {
      ++x.attempts;
      const CallResult r = client_.set_sync(sm::kSyncUnspecified, 0);
      const grpc::Status st = client_.get_sync(&type, opt_.deadline_ms);
      x.final_type = st.ok() ? type : -1;
      std::printf(
        "[restore] SetSynchronousMode(UNSPECIFIED, 0) #%d -> %s; GetSynchronousMode -> %s\n",
        x.attempts, r.brief().c_str(),
        st.ok() ? sm::sync_type_name(type).c_str() : grpc_code_name(st.error_code()).c_str());
      std::fflush(stdout);
      if (st.ok() && type == sm::kSyncUnspecified) {
        x.ok = true;
        break;
      }
      x.last_error = st.ok() ? sm::sync_type_name(type) : grpc_code_name(st.error_code());
      if (k >= delays.size() || now_ms() - t0 >= total_s * 1e3) {
        break;
      }
      sleep_ms(delays[k]);
    }
    x.took_s = (now_ms() - t0) / 1e3;
    return x;
  }

  /// Samples ActorStates.timestamp for `window_s` wall (after up to 2 s for the first
  /// read) and judges real time.
  sm::RealtimeCheck verify_realtime(double window_s, bool need_epoch)
  {
    std::vector<sm::TimeSample> s;
    const double t_first = now_ms();
    EgoState e;
    while (now_ms() - t_first < 2000.0) {
      e = client_.read_state();
      if (e.ok) {
        break;
      }
      sleep_ms(20.0);
    }
    double last_unix = unix_now_s();
    if (e.ok) {
      const double t0 = now_ms();
      s.push_back({t0 / 1e3, e.ts_us});
      while (now_ms() - t0 < window_s * 1e3) {
        sleep_ms(5.0);
        const EgoState x = client_.read_state();
        if (x.ok) {
          s.push_back({now_ms() / 1e3, x.ts_us});
          last_unix = unix_now_s();
        }
      }
    }
    sm::RealtimeCheck c = sm::judge_realtime(s, last_unix, 0.8 * window_s);
    if (!c.ok && !need_epoch && !c.epoch && c.why.find("epoch") != std::string::npos) {
      c.ok = true;  // 사전 점검에서도 epoch 가 아니었다
      c.why.clear();
    }
    return c;
  }

  bool fresh_gear(double since_ms, int * gear)
  {
    const double t = now_ms();
    const EgoState e = client_.read_state();
    if (!e.ok || !(t > since_ms)) {
      return false;
    }
    *gear = e.gear;
    return true;
  }

  const BrakeStats & last_stop() const { return last_stop_; }
  int period_ms() const { return opt_.tick_period_ms; }

private:
  void arm(double * ox, double * oy)
  {
    armed_ = true;
    origin_x_ = last_good_.x;
    origin_y_ = last_good_.y;
    armed_max_speed_ = std::fabs(last_good_.planar_mps);
    armed_max_dist_ = 0.0;
    armed_collisions_ = static_cast<size_t>(std::max(0, last_good_.collisions));
    armed_max_steer_ = 0.0;
    *ox = origin_x_;
    *oy = origin_y_;
  }

  int64_t drive_us() const { return std::llround(opt_.drive_s * 1e6); }

  /// Tick until drive_s of sim time: a command before every cmd_every-th Tick(frames).
  std::string drive(const LockstepPatternSpec & p, RunRecord * r)
  {
    const auto start = Clock::now();
    const double budget_s = 30.0 + opt_.drive_s * 1000.0 / period_ms() * 0.05;
    for (size_t i = 0;; ++i) {
      const std::string why = check_abort(start, budget_s);
      if (!why.empty()) {
        return why;
      }
      if (last_good_.ts_us - r->t0_us >= drive_us()) {
        return "";
      }
      const bool cmd = (i % static_cast<size_t>(p.cmd_every)) == 0;
      step(r->index, kDrive, cmd, opt_.accel, p.frames);
      ++r->ticks;
    }
  }

  const Options & opt_;
  Client & client_;
  EnvelopeLimits limits_;
  std::vector<ls::Step> steps_;
  std::vector<ls::TickClass> classes_;
  ls::TickHealth health_;
  ls::Step prev_;
  bool have_prev_{false};
  ls::Step last_good_;
  bool have_last_good_{false};
  int frames_since_good_{0};  ///< frames of the Ticks after last_good_ (failed or unread)
  double last_read_ok_ms_{0.0};
  std::string last_read_error_;
  std::string last_tick_error_;
  std::string last_cmd_error_;
  size_t consecutive_cmd_failures_{0};
  bool sync_touched_{false};
  bool sync_verified_{false};
  bool gear_touched_{false};
  bool drove_{false};
  bool armed_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double armed_max_speed_{0.0};
  double armed_max_dist_{0.0};
  size_t armed_collisions_{0};
  double armed_max_steer_{0.0};
  BrakeStats last_stop_;
};

// =============================================================================
// Restore guard — safe-stop by Ticks, ORIGINAL gear (only at a confirmed standstill), leave
// synchronous mode, still moving: real-time brake at once, verify real time, ORIGINAL mode,
// gear (only at standstill in verified real time), standstill check. Runs once, from
// restore() or from the destructor; every step is isolated in its own try/catch.
// =============================================================================

constexpr const char kFrozenBanner[] =
  "\n"
  "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
  "!! RESTORE FAILED: MORAI may be frozen in synchronous mode — run morai_sync_restore\n"
  "!!   ros2 run morai_grpc_vendor morai_sync_restore --host %s --port %d\n"
  "!!   (or in the MORAI UI: Time Manager -> leave synchronous mode, then check the Ego's\n"
  "!!    control mode and gear)\n"
  "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";

/// Several reads over a short wall window: stopped = >= 3 reads, every |v| < kStopSpeed.
struct StandstillCheck
{
  bool stopped{false};
  size_t reads{0};
  double max_speed{0.0};
  std::string err;
};

class RestoreGuard
{
public:
  RestoreGuard(Session & s, Client & c, const Options & o, int mode, int gear, bool need_epoch)
  : session_(s), client_(c), opt_(o), mode_(mode), gear_(gear), need_epoch_(need_epoch)
  {
  }
  ~RestoreGuard()
  {
    if (armed_ && !done_) {
      try {
        restore();
      } catch (...) {
        std::fprintf(stderr, "[restore] EXCEPTION during restore - check MORAI now\n");
        std::fprintf(stderr, kFrozenBanner, opt_.host.c_str(), opt_.port);
      }
    }
  }
  RestoreGuard(const RestoreGuard &) = delete;
  RestoreGuard & operator=(const RestoreGuard &) = delete;

  void arm() { armed_ = true; }
  bool ok() const { return !armed_ || ok_; }
  bool sync_ok() const { return !armed_ || sync_ok_; }

  Json restore()
  {
    if (!armed_) {
      return Json::object().set("needed", false);
    }
    if (done_) {
      return result_;
    }
    done_ = true;
    if (!session_.sync_touched()) {
      std::printf("\n[restore] nothing was changed (stopped before SetSynchronousMode) - nothing to restore\n");
      ok_ = sync_ok_ = true;
      result_ = Json::object().set("needed", false).set("ok", true);
      return result_;
    }
    std::printf(
      "\n[restore] safe-stop by Ticks -> gear %s (only at standstill) -> SetSynchronousMode(UNSPECIFIED) "
      "-> (still moving: real-time brake) -> verify real time -> mode %s -> gear -> standstill\n",
      gear_name(gear_).c_str(), mode_name(mode_).c_str());
    std::fflush(stdout);
    Json j = Json::object().set("needed", true);

    // 1. 안전 정지 (동기 모드이므로 브레이크 + Tick). 앞선 안전 정지가 실패했으면 짧게 — 곧 실시간으로 브레이크한다
    const bool stop_needed =
      session_.sync_verified() && (session_.gear_touched() || session_.drove());
    BrakeStats stop;
    guarded("1 safe-stop", [&] {
      if (stop_needed) {
        const BrakeStats & prev = session_.last_stop();
        const bool prev_failed = prev.ticks > 0 && !prev.ok;
        const double cap =
          prev_failed ? ls::Limits::kRestoreStopAfterFailureWallS : ls::Limits::kSafeStopMaxWallS;
        stop = session_.safe_stop(-1, kRestore, cap);
        std::printf(
          "[restore] 1 safe-stop %s (%.2f s sim, %zu Ticks, %.1f s wall, cap %.0f s), %s%s%s\n",
          stop.ok ? "OK" : "FAILED", stop.sim_s, stop.ticks, stop.took_ms / 1e3, cap,
          stop.counts().c_str(), stop.ok ? "" : ": ", stop.why.c_str());
        if (!stop.warning().empty()) {
          std::printf("[restore]   WARNING: %s\n", stop.warning().c_str());
        }
      } else {
        stop.ok = true;
        std::printf("[restore] 1 safe-stop not needed (never drove, gear untouched or sync not reached)\n");
      }
      j.set("safe_stop_needed", stop_needed).set("safe_stop", stop.json());
    });

    // 2. 원래 기어 — Tick 안전 정지가 정지를 확인했을 때만 (움직이는 채로 P 를 넣지 않는다; 아니면 6 에서)
    bool gear_ok_sync = !session_.gear_touched();
    guarded("2 gear", [&] {
      std::string text = "not changed";
      if (session_.gear_touched() && session_.sync_verified() && stop.ok) {
        CallResult g;
        for (int i = 0; i < 3; ++i) {
          g = client_.set_gear(gear_);
          if (g.rpc_ok) {
            break;
          }
        }
        int gear_now = -1;
        int after = 0;
        for (int k = 1; k <= ls::Limits::kGearReadbackTicks; ++k) {
          const ls::Step s = session_.step(-1, kRestore, false, 0.0, 1);
          if (s.read_ok) {
            gear_now = s.gear;
          }
          if (gear_now == gear_) {
            after = k;
            gear_ok_sync = true;
            break;
          }
          if (!s.tick_ok) {
            break;  // MORAI 불통: 실시간 복귀 뒤 다시 본다
          }
        }
        text = g.text("SetVehicleGear") + ", read back " + gear_name(gear_now) +
               (gear_ok_sync ? " after " + std::to_string(after) + " Tick(s)" : "");
        std::printf(
          "[restore] 2 gear %s: %s %s\n", gear_name(gear_).c_str(), text.c_str(),
          gear_ok_sync ? "- restored" : "- NOT YET (checked again in real time)");
      } else if (session_.gear_touched() && session_.sync_verified()) {
        text = "NOT sent in synchronous mode: the Tick safe-stop did not confirm a standstill";
        std::printf(
          "[restore] 2 gear %s: %s - set in step 6 only once the Ego is stopped in real time\n",
          gear_name(gear_).c_str(), text.c_str());
      } else {
        text = session_.gear_touched() ? "sync not verified - checked in real time" : "never changed";
        std::printf("[restore] 2 gear: %s\n", text.c_str());
      }
      j.set("gear_in_sync", text).set("gear_in_sync_ok", gear_ok_sync);
    });

    // 3. 동기 모드 해제
    ExitResult ex;
    guarded("3 synchronous mode", [&] {
      ex = session_.exit_sync(ls::Limits::kExitRetryS);
      std::printf(
        "[restore] 3 synchronous mode: %s%s\n",
        ex.ok ? "left - GetSynchronousMode reads UNSPECIFIED" : "STILL NOT UNSPECIFIED",
        ex.ok ? "" : (" after " + f2(ex.took_s) + " s, " + std::to_string(ex.attempts) + " attempts (" + ex.last_error + ")").c_str());
      j.set(
        "exit_sync", Json::object()
                       .set("ok", ex.ok)
                       .set("already", ex.already)
                       .set("attempts", ex.attempts)
                       .set("took_s", ex.took_s)
                       .set("final_type", ex.final_type >= 0 ? sm::sync_type_name(ex.final_type) : std::string("none"))
                       .set("last_error", ex.last_error));
    });

    // 3b. 실시간으로 풀리자마자 아직 움직이면(Tick 안전 정지 실패) 1 s 확인보다 브레이크가 먼저: 1b 방식 실시간 안전 정지
    //     (원래 모드가 KEYBOARD 면 ControlVehicle 이 무시되므로 AUTO 로 바꾼 뒤 브레이크, 모드는 5 에서 되돌린다)
    BrakeStats rt_stop;
    bool rt_stop_used = false;
    guarded("3b real-time brake", [&] {
      if (!ex.ok) {
        return;
      }
      EgoState e0;
      for (const double until = now_ms() + 2000.0; now_ms() < until;) {
        e0 = client_.read_state();
        if (e0.ok) {
          break;
        }
        sleep_ms(20.0);
      }
      const bool real_time_ts = e0.ok && (sm::is_epoch_us(e0.ts_us) || !need_epoch_);
      if (real_time_ts && !(std::fabs(e0.planar_mps) < Safety::kStopSpeed)) {
        rt_stop_used = true;
        std::printf(
          "[restore] 3b the Ego still moves at %.3f m/s in real time -> AUTO_MODE + brake %.1f "
          "m/s^2 at %.0f Hz until stopped (real-time safe-stop, before the real-time check)\n",
          std::fabs(e0.planar_mps), Safety::kSafeStopAccel, Safety::kSafeStopHz);
        rt_stop = realtime_brake();
        std::printf(
          "[restore] 3b real-time safe-stop %s (%.1f s), %s%s%s\n", rt_stop.ok ? "OK" : "FAILED",
          rt_stop.took_ms / 1e3, rt_stop.counts().c_str(), rt_stop.ok ? "" : ": ",
          rt_stop.why.c_str());
      }
    });

    // 4. 실시간 확인: timestamp 가 벽시계 속도로 흐르고 epoch 기반인지
    sm::RealtimeCheck rt;
    guarded("4 real time", [&] {
      rt = session_.verify_realtime(ls::Limits::kRealtimeCheckS, need_epoch_);
      std::printf(
        "[restore] 4 real time: %s - rate %s x wall, %zu changes in %.2f s, timestamp %s%s%s\n",
        rt.ok ? "OK" : "FAILED", f3(rt.rate).c_str(), rt.changes, rt.span_s,
        rt.epoch ? "epoch-based" : "NOT epoch-based",
        rt.epoch ? (" (clock offset " + f3(rt.offset_s) + " s)").c_str() : "",
        rt.ok ? "" : (": " + rt.why).c_str());
      j.set(
        "realtime", Json::object()
                      .set("ok", rt.ok)
                      .set("rate", rt.rate)
                      .set("changes", rt.changes)
                      .set("span_s", rt.span_s)
                      .set("epoch", rt.epoch)
                      .set("offset_s", rt.offset_s)
                      .set("why", rt.why));
    });

    // 4b. 3b 에서 세우지 못했거나(첫 읽기가 아직 실시간이 아니었음) 그 뒤 다시 움직이면: 실시간 안전 정지
    guarded("4b real-time brake", [&] {
      if (!sync_ok_now(ex, rt) || (rt_stop_used && rt_stop.ok)) {
        return;
      }
      const EgoState e1 = client_.read_state();
      if (e1.ok && !(std::fabs(e1.planar_mps) < Safety::kStopSpeed)) {
        rt_stop_used = true;
        std::printf(
          "[restore] 4b the Ego still moves at %.3f m/s -> AUTO_MODE + brake %.1f m/s^2 (real-time "
          "safe-stop)\n",
          std::fabs(e1.planar_mps), Safety::kSafeStopAccel);
        rt_stop = realtime_brake();
        std::printf(
          "[restore] 4b real-time safe-stop %s (%.1f s), %s%s%s\n", rt_stop.ok ? "OK" : "FAILED",
          rt_stop.took_ms / 1e3, rt_stop.counts().c_str(), rt_stop.ok ? "" : ": ",
          rt_stop.why.c_str());
      }
    });
    j.set("realtime_safe_stop_used", rt_stop_used).set("realtime_safe_stop", rt_stop.json());

    // 쓰기는 동기 모드가 아닐 때만: 실시간이 확인됐거나, 이 복구가 UNSPECIFIED 를 읽은 뒤 스스로 AUTO 로 바꿨을 때
    // (그건 되돌려야 한다). 아니면 읽고 보고만 한다 — 동기 모드에서 모드를 바꾸면 서버가 해제 때 되돌릴 값이 흐트러질 수 있다
    const bool mode_writes_ok = sync_ok_now(ex, rt) || (ex.ok && rt_stop_used);

    // 5. 제어 모드: 서버가 동기 모드 해제 때 되돌린다. 아니면 설정 후 되읽기
    bool mode_ok = false;
    guarded("5 mode", [&] {
      int mode_now = -1;
      std::string err;
      mode_ok = client_.get_mode(&mode_now, &err) && mode_now == mode_;
      std::string mode_text = "read " + (mode_now >= 0 ? mode_name(mode_now) : err);
      if (mode_ok) {
        mode_text += " (server restored it)";
      } else if (!mode_writes_ok) {
        mode_text += " -> SetVehicleControlMode NOT sent: MORAI is not verified back in real time "
                     "(left to morai_sync_restore / the MORAI UI)";
      } else {
        CallResult m;
        for (int i = 0; i < 3; ++i) {
          m = client_.set_mode(mode_);
          if (m.success()) {
            break;
          }
        }
        for (const double until = now_ms() + 1500.0; now_ms() < until;) {
          if (client_.get_mode(&mode_now, &err) && mode_now == mode_) {
            mode_ok = true;
            break;
          }
          sleep_ms(20.0);
        }
        mode_text += " -> " + m.text("SetVehicleControlMode(" + mode_name(mode_) + ")") +
                     ", read back " + (mode_now >= 0 ? mode_name(mode_now) : err);
      }
      std::printf(
        "[restore] 5 mode %s: %s %s\n", mode_name(mode_).c_str(), mode_text.c_str(),
        mode_ok ? "- restored" : "- NOT RESTORED");
      j.set("mode", mode_text).set("mode_ok", mode_ok).set("original_mode", mode_name(mode_));
    });

    // 6. 기어 (실시간에서 다시 확인). 설정은 실시간이 확인되고 새로 읽은 속도가 정지일 때만
    bool gear_ok = false;
    guarded("6 gear", [&] {
      const EgoState e = client_.read_state();
      int gear_now = e.ok ? e.gear : -1;
      gear_ok = gear_now == gear_;
      std::string gear_text = "read " + (e.ok ? gear_name(gear_now) : e.err);
      if (!gear_ok && session_.gear_touched() && e.ok) {
        if (!sync_ok_now(ex, rt)) {
          gear_text += " -> SetVehicleGear NOT sent: MORAI is not verified back in real time";
        } else if (!(std::fabs(e.planar_mps) < Safety::kStopSpeed)) {
          gear_text += " -> SetVehicleGear NOT sent: the Ego still moves at " +
                       f3(std::fabs(e.planar_mps)) + " m/s";
        } else {
          const double t_set = now_ms();
          const CallResult g = client_.set_gear(gear_);
          for (const double until = now_ms() + 1500.0; now_ms() < until;) {
            if (session_.fresh_gear(t_set, &gear_now) && gear_now == gear_) {
              gear_ok = true;
              break;
            }
            sleep_ms(20.0);
          }
          gear_text += " -> " + g.text("SetVehicleGear(" + gear_name(gear_) + ")") + " at " +
                       f3(std::fabs(e.planar_mps)) + " m/s, read back " + gear_name(gear_now);
        }
      }
      std::printf(
        "[restore] 6 gear %s: %s %s\n", gear_name(gear_).c_str(), gear_text.c_str(),
        gear_ok ? "- restored" : "- NOT RESTORED");
      j.set("gear", gear_text).set("gear_ok", gear_ok).set("original_gear", gear_name(gear_));
    });

    // 7. 정지 확인: 마지막으로 새로 읽은 속도로 판정 (Tick·실시간 안전 정지의 성패가 아니라 지금 상태)
    StandstillCheck fin;
    guarded("7 standstill", [&] {
      fin = standstill(0.5);
      std::printf(
        "[restore] 7 standstill: %s (%zu reads over 0.5 s, max |v| %.3f m/s%s%s)\n",
        fin.stopped ? "OK" : "NOT CONFIRMED", fin.reads, fin.max_speed,
        fin.err.empty() ? "" : ", ", fin.err.c_str());
      if (fin.reads > 0 && !(fin.max_speed < Safety::kStopSpeed)) {
        std::printf(
          "[restore] WARNING: the Ego still moves at %.3f m/s - stop it in MORAI\n", fin.max_speed);
      }
      j.set(
        "standstill", Json::object()
                        .set("stopped", fin.stopped)
                        .set("reads", fin.reads)
                        .set("max_speed_mps", fin.max_speed)
                        .set("error", fin.err));
      j.set("still_moving", fin.reads > 0 && !(fin.max_speed < Safety::kStopSpeed));
    });

    sync_ok_ = sync_ok_now(ex, rt);
    ok_ = sync_ok_ && mode_ok && gear_ok && fin.stopped && !exception_;
    try {
      j.set("sync_ok", sync_ok_).set("ok", ok_).set("exception", exception_);
    } catch (...) {
      exception_ = true;
    }
    if (!sync_ok_) {
      std::printf(kFrozenBanner, opt_.host.c_str(), opt_.port);
      std::fprintf(stderr, kFrozenBanner, opt_.host.c_str(), opt_.port);
    } else {
      std::printf(
        "[restore] %s\n",
        ok_ ? (stop.warning().empty() ? "COMPLETE - MORAI back in real time, mode and gear read back, Ego stopped"
                                      : "COMPLETE (brake WARNING above)")
            : "INCOMPLETE - MORAI runs in real time, but check the Ego in MORAI NOW (mode/gear/speed)");
    }
    std::fflush(stdout);
    result_ = j;
    return result_;
  }

private:
  static bool sync_ok_now(const ExitResult & ex, const sm::RealtimeCheck & rt) { return ex.ok && rt.ok; }

  /// Runs one restore step; an exception is reported and the next step still runs (leaving
  /// synchronous mode must never be skipped because an earlier step threw).
  template <typename F>
  void guarded(const char * name, F && f)
  {
    try {
      f();
    } catch (const std::exception & e) {
      exception_ = true;
      std::printf("[restore] %s: EXCEPTION %s - continuing with the next step\n", name, e.what());
      std::fflush(stdout);
    } catch (...) {
      exception_ = true;
      std::printf("[restore] %s: unknown EXCEPTION - continuing with the next step\n", name);
      std::fflush(stdout);
    }
  }

  StandstillCheck standstill(double window_s)
  {
    StandstillCheck c;
    bool all_below = true;
    const double t0 = now_ms();
    while (now_ms() - t0 < window_s * 1e3) {
      const EgoState e = client_.read_state();
      if (e.ok) {
        ++c.reads;
        const double v = std::fabs(e.planar_mps);
        c.max_speed = std::max(c.max_speed, v);
        all_below = all_below && v < Safety::kStopSpeed;
      } else {
        c.err = e.err;
      }
      sleep_ms(20.0);
    }
    c.stopped = c.reads >= 3 && all_below;
    return c;
  }

  /// Real time only: AUTO_MODE, then brake (-3.0 m/s^2, steer 0) at 100 Hz until |v| < 0.05 m/s
  /// for 1 s, at most 10 s — morai_actuate's real-time safe-stop. Gives up early when MORAI
  /// time does not advance for 1.5 s (paused: braking cannot act).
  BrakeStats realtime_brake()
  {
    BrakeStats b;
    const double t0 = now_ms();
    const CallResult m = client_.set_mode(static_cast<int>(pb_actor::VEHICLE_CONTROL_AUTO_MODE));
    if (!m.success()) {
      b.why = m.text("SetVehicleControlMode(AUTO_MODE)");
    }
    double still = kNaN;
    double next = now_ms();
    int64_t last_ts = -1;
    double last_ts_change = now_ms();
    while (now_ms() - t0 < Safety::kStopTimeoutS * 1e3) {
      const CallResult c = client_.control(Safety::kSafeStopAccel);
      ++b.sends;
      if (c.success()) {
        ++b.accepted;
      } else {
        b.last_error = c.text("ControlVehicle");
      }
      const EgoState e = client_.read_state();
      if (e.ok && e.ts_us != last_ts) {
        last_ts = e.ts_us;
        last_ts_change = now_ms();
      } else if (now_ms() - last_ts_change > 1500.0) {
        b.why = "no advancing MORAI state for 1.5 s (paused or unreachable) - braking cannot act";
        break;
      }
      if (e.ok && std::fabs(e.planar_mps) < Safety::kStopSpeed) {
        if (!std::isfinite(still)) {
          still = now_ms();
        }
        if (now_ms() - still >= Safety::kStopHoldS * 1e3) {
          b.ok = true;
          b.why.clear();
          break;
        }
      } else {
        still = kNaN;
      }
      next += 1000.0 / Safety::kSafeStopHz;
      const double wait = next - now_ms();
      if (wait > 0.0) {
        sleep_ms(wait);
      } else {
        next = now_ms();
      }
    }
    b.took_ms = now_ms() - t0;
    if (!b.ok && b.why.empty()) {
      b.why = "not stopped after " + f2(Safety::kStopTimeoutS) + " s (" + b.counts() + ")";
    }
    return b;
  }

  Session & session_;
  Client & client_;
  const Options & opt_;
  int mode_;
  int gear_;
  bool need_epoch_;
  bool armed_{false};
  bool done_{false};
  bool ok_{false};
  bool sync_ok_{false};
  bool exception_{false};
  Json result_;
};

// =============================================================================
// Output
// =============================================================================

ls::RunInput make_input(
  const RunRecord & r, const std::vector<ls::Step> & steps, const IdleSignature & idle,
  const Options & o)
{
  ls::RunInput in;
  in.pattern = r.pattern;
  const LockstepPatternSpec * p = find_lockstep_pattern(r.pattern);
  in.cmd_every = p ? p->cmd_every : 1;
  in.frames = p ? p->frames : 1;
  if (r.step_end > r.step_ref && r.step_end <= steps.size()) {
    in.steps.assign(
      steps.begin() + static_cast<std::ptrdiff_t>(r.step_ref),
      steps.begin() + static_cast<std::ptrdiff_t>(r.step_end));
  }
  in.drive_s = o.drive_s;
  in.fit_start_s = Safety::kFitStartS;
  in.commanded_accel = o.accel;
  in.period_ms = o.tick_period_ms;
  in.idle = idle;
  in.aborted = !r.abort_reason.empty() || r.step_end == 0;
  return in;
}

void print_run_line(const RunRecord & r)
{
  const auto & a = r.analysis;
  std::printf(
    "[run %2d] %-11s #%d: ratio %s (a=%s m/s^2, r2 %s), %zu Ticks (%zu cmd), a_cmd1 %s a_unc1 "
    "%s a_multi %s, %s s sim / %s s wall (%sx), dt ok %zu/%zu, vmax %.2f m/s, dmax %.2f m, "
    "stop %s s sim (%s)%s%s\n",
    r.index, r.pattern.c_str(), r.repeat + 1, f3(a.ratio).c_str(), f3(a.realised_accel).c_str(),
    f3(a.fit.r2).c_str(), a.ticks, a.commanded, f3(a.cmd1.accel()).c_str(),
    f3(a.unc1.accel()).c_str(), f3(a.multi.accel()).c_str(), f2(a.sim_s).c_str(),
    f2(a.wall_s).c_str(), f2(a.rtf).c_str(), a.dt.ok, a.dt.n, r.max_speed, r.max_dist,
    f2(r.stop.sim_s).c_str(), r.stop.counts().c_str(), r.abort_reason.empty() ? "" : " ABORTED: ",
    r.abort_reason.c_str());
  if (!r.brake_warning.empty()) {
    std::printf("[run %2d] WARNING: %s\n", r.index, r.brake_warning.c_str());
  }
  std::fflush(stdout);
}

Json run_json(const RunRecord & r)
{
  return Json::object()
    .set("index", r.index)
    .set("pattern", r.pattern)
    .set("repeat", r.repeat)
    .set("t0_us", r.t0_us)
    .set("drive_end_us", r.drive_end_us)
    .set("ticks", r.ticks)
    .set("origin_x", r.origin_x)
    .set("origin_y", r.origin_y)
    .set("aborted", !r.abort_reason.empty())
    .set("abort_reason", r.abort_reason)
    .set("pre_hold", r.pre_hold.json())
    .set("safe_stop", r.stop.json())
    .set("settle", r.settle.json())
    .set("brake_warning", r.brake_warning)
    .set("max_speed_mps", r.max_speed)
    .set("max_displacement_m", r.max_dist)
    .set("step_range", Json::array().push(r.step_ref).push(r.step_end))
    .set("analysis", ls::to_json(r.analysis));
}

Json raw_steps(const std::vector<ls::Step> & v, const std::vector<ls::TickClass> & cls)
{
  RawTable t(
    {"idx", "run", "phase", "commanded", "command_ok", "accel", "frames", "tick_ok", "grpc_code",
     "frame_count", "elapsed_time", "tick_wall_ms", "read_ok", "ts_us", "tick_count",
     "speed_mps", "planar_mps", "x", "y", "throttle", "brake", "steer", "front_wheel_angle",
     "gear", "collisions", "wall_ms", "read_retries", "tick_class"},
    10000000);
  for (size_t i = 0; i < v.size(); ++i) {
    const auto & s = v[i];
    t.add(
      {Cell::u(i), Cell::i(s.run), Cell::i(s.phase), Cell::i(s.commanded ? 1 : 0),
       Cell::i(s.command_ok ? 1 : 0), Cell::d(s.accel), Cell::i(s.frames),
       Cell::i(s.tick_ok ? 1 : 0), Cell::i(s.grpc_code), Cell::u(s.frame_count),
       Cell::i(s.elapsed_time), Cell::d(s.tick_wall_ms), Cell::i(s.read_ok ? 1 : 0),
       Cell::i(s.ts_us), Cell::u(s.tick_count), Cell::d(s.speed_mps), Cell::d(s.planar_mps),
       Cell::d(s.x), Cell::d(s.y), Cell::d(s.throttle), Cell::d(s.brake), Cell::d(s.steer),
       Cell::d(s.fwa), Cell::i(s.gear), Cell::i(s.collisions), Cell::d(s.wall_ms),
       Cell::i(s.read_retries), Cell::i(i < cls.size() ? static_cast<int>(cls[i]) : -1)});
  }
  return t.to_json();
}

Json names_json()
{
  Json ph = Json::array();
  for (int i = 0; i <= kRestore; ++i) {
    ph.push(phase_name(i));
  }
  Json tc = Json::array();
  for (auto c :
       {ls::TickClass::kOk, ls::TickClass::kFailed, ls::TickClass::kNoRead,
        ls::TickClass::kNotAdvanced, ls::TickClass::kFrameMismatch, ls::TickClass::kDtMismatch,
        ls::TickClass::kJumped}) {
    tc.push(ls::tick_class_name(c));
  }
  return Json::object().set("phase", std::move(ph)).set("tick_class", std::move(tc));
}

Json options_json(const Options & o)
{
  Json pats = Json::array();
  for (const auto & p : o.patterns) {
    pats.push(p);
  }
  return Json::object()
    .set("host", o.host)
    .set("port", o.port)
    .set("ego", o.ego)
    .set("client_key", o.client_key)
    .set("lockstep", true)
    .set("tick_period_ms", o.tick_period_ms)
    .set("tick_deadline_ms", o.tick_deadline_ms)
    .set("accel", o.accel)
    .set("drive_s", o.drive_s)
    .set("settle_s", o.settle_s)
    .set("repeats", o.repeats)
    .set("patterns", std::move(pats))
    .set("order", o.order)
    .set("allow_any_original_mode", o.allow_any_mode)
    .set("max_speed", o.max_speed)
    .set("max_distance", o.max_distance)
    .set("max_total_distance", o.max_total_distance)
    .set("deadline_ms", o.deadline_ms)
    .set("idle_probe_s", o.idle_probe_s)
    .set("idle_brake", o.idle_brake)
    .set("idle_brake_tol", o.idle_brake_tol)
    .set("idle_throttle_max", o.idle_throttle_max)
    .set("safe_stop_accel", Safety::kSafeStopAccel)
    .set("safe_stop_max_ticks", ls::safe_stop_max_ticks(o.tick_period_ms))
    .set("safe_stop_max_wall_s", ls::Limits::kSafeStopMaxWallS)
    .set("restore_stop_after_failure_wall_s", ls::Limits::kRestoreStopAfterFailureWallS)
    .set("health_window", ls::Limits::kHealthWindow)
    .set("max_bad_in_window", ls::Limits::kMaxBadInWindow)
    .set("stop_hold_sim_s", Safety::kStopHoldS)
    .set("fit_start_s", Safety::kFitStartS)
    .set("pre_drive_hold_s", Safety::kPreDriveHoldS)
    .set("max_failed_ticks", ls::Limits::kMaxFailedTicks)
    .set("max_frozen_ticks", ls::Limits::kMaxFrozenTicks)
    .set("dt_tolerance_us", ls::Limits::kDtToleranceUs)
    .set("enter_retry_s", ls::Limits::kEnterRetryS)
    .set("exit_retry_s", ls::Limits::kExitRetryS)
    .set("max_stale_s", Safety::kMaxStaleS);
}

bool write_json(const std::string & path, const Json & j)
{
  std::error_code ec;
  const std::filesystem::path p(path);
  if (p.has_parent_path()) {
    std::filesystem::create_directories(p.parent_path(), ec);
  }
  std::ofstream f(path);
  if (!f) {
    return false;
  }
  f << j.dump(2) << "\n";
  return static_cast<bool>(f);
}

}  // namespace

// =============================================================================

int run_lockstep(const Options & opt)
{
  install_signal_handlers();
  Json out = Json::object()
               .set("tool", "morai_actuate")
               .set("mode", "lockstep")
               .set("schema", 1)
               .set("started_at_unix_s", unix_now_s())
               .set("options", options_json(opt))
               .set("names", names_json());
  auto finish = [&](int code) {
    out.set("exit_code", code).set("finished_at_unix_s", unix_now_s());
    if (!opt.json_path.empty()) {
      if (!write_json(opt.json_path, out)) {
        std::fprintf(stderr, "morai_actuate: cannot write %s\n", opt.json_path.c_str());
        return code == 0 ? 3 : code;
      }
      std::printf("[info] JSON written to %s\n", opt.json_path.c_str());
    }
    return code;
  };

  morai_sim_api::ObjectInfo ego;
  ego.mutable_id()->set_value(opt.ego);
  ego.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);  // as main.py
  ego.set_client_key(opt.client_key);

  Client client(opt, ego);
  Session session(opt, client);

  std::printf(
    "morai_actuate --lockstep -> %s:%d ego '%s' (MOVES THE EGO, MORAI SYNCHRONOUS MODE)\n",
    opt.host.c_str(), opt.port, opt.ego.c_str());
  if (!client.connect(opt.connect_timeout_ms)) {
    std::fprintf(
      stderr, "morai_actuate: cannot connect to %s:%d within %d ms\n", opt.host.c_str(), opt.port,
      opt.connect_timeout_ms);
    out.set("connected", false);
    return finish(2);
  }
  out.set("connected", true);

  // ---- preflight (read-only, real time) -----------------------------------------
  const Preflight pf = session.preflight();
  out.set("preflight", pf.j);
  if (!pf.ok) {
    std::fprintf(stderr, "morai_actuate: preflight failed - %s. Nothing was changed.\n", pf.why.c_str());
    out.set("preflight_error", pf.why);
    return finish(stop_requested() ? 5 : 2);
  }

  // ---- banner + countdown -----------------------------------------------------------
  const double per_run = estimate_run_travel_m(opt.accel, opt.drive_s, Safety::kAssumedBrakeDecel);
  const std::vector<std::string> run_names = run_order(opt);
  std::string order;
  for (const auto & p : opt.patterns) {
    order += (order.empty() ? "" : " ") + p;
  }
  order += opt.order == "abba" && opt.repeats > 1 ? " (odd repeats reversed)" : "";
  std::printf(
    "======================================================================\n"
    " morai_actuate --lockstep - THIS MOVES THE EGO AND FREEZES MORAI INTO SYNCHRONOUS MODE\n"
    "   target    %s:%d  ego '%s'  MORAI %s\n"
    "   now       real time (UNSPECIFIED), mode %s, gear %s, standstill (%.2f, %.2f)\n"
    "   will set  SetSynchronousMode(SYNCHRONOUS, tick_period %d ms): MORAI only advances on\n"
    "             Tick (one command per Tick); the server forces the Ego's mode to\n"
    "             VEHICLE_CONTROL_SYNCHRONOUS_MODE; gear GEAR_MODE_D\n"
    "   restore   safe-stop by Ticks -> gear %s at standstill -> SetSynchronousMode(UNSPECIFIED)\n"
    "             -> verify real time -> mode %s -> gear, on every exit (read back)\n"
    "   runs      %zu = %zu patterns x %d, interleaved: %s\n"
    "   each run  brake hold %.1f s -> +%.2f m/s^2 for %.1f s MORAI time, steer 0 -> safe-stop\n"
    "             -> hold %.1f s (all in sim time)\n"
    "   travel    ~%.1f m straight ahead per run, ~%.0f m total (budget %.0f m)\n"
    "   envelope  |v| <= %.2f m/s, <= %.1f m from run start, reads fail <= %.1f s, <= %zu Ticks\n"
    "             without progress, no collision, no steer echo (nobody else commanding)\n"
    "   Ctrl-C    abort -> safe-stop (%.1f m/s^2 + Tick(1), <= %d Ticks / %.0f s) -> restore\n"
    "             (also SIGTERM/SIGHUP/Ctrl-\\/Ctrl-Z)\n"
    "   kill -9   MORAI STAYS FROZEN: run morai_sync_restore (or the MORAI UI Time Manager)\n"
    "======================================================================\n",
    opt.host.c_str(), opt.port, opt.ego.c_str(), pf.version.empty() ? "?" : pf.version.c_str(),
    mode_name(pf.mode).c_str(), gear_name(pf.gear).c_str(), pf.x, pf.y, opt.tick_period_ms,
    gear_name(pf.gear).c_str(), mode_name(pf.mode).c_str(), run_names.size(),
    opt.patterns.size(), opt.repeats, order.c_str(), Safety::kPreDriveHoldS, opt.accel,
    opt.drive_s, opt.settle_s, per_run, per_run * static_cast<double>(run_names.size()),
    opt.max_total_distance, opt.max_speed, opt.max_distance, Safety::kMaxStaleS,
    ls::Limits::kMaxFrozenTicks, Safety::kSafeStopAccel, ls::safe_stop_max_ticks(opt.tick_period_ms),
    ls::Limits::kSafeStopMaxWallS);
  if (opt.accel * opt.drive_s >= opt.max_speed) {
    std::printf(
      "WARNING: expected peak speed %.2f m/s >= --max-speed %.2f: runs will hit the envelope\n",
      opt.accel * opt.drive_s, opt.max_speed);
  }
  if (opt.countdown) {
    for (int i = 3; i > 0; --i) {
      std::printf("starting in %d ... (Ctrl-C to cancel)\n", i);
      if (!interruptible_sleep(1000.0)) {
        break;
      }
    }
  }
  if (stop_requested()) {
    std::printf("[info] %s before any change - nothing was changed\n", signal_text().c_str());
    return finish(5);
  }

  // ---- from here on every exit path restores ---------------------------------------
  RestoreGuard guard(session, client, opt, pf.mode, pf.gear, pf.epoch);
  guard.arm();
  int code = 0;
  std::string session_abort;
  bool budget_stop = false;
  IdleCalibration idle;
  std::vector<RunRecord> runs;
  Json enter_j = Json::object();
  Json setup_j = Json::object();
  try {
    std::string why;
    if (!session.enter(&enter_j, &why)) {
      session_abort = "enter: " + why;
      code = stop_requested() ? 5 : 4;
      std::fprintf(stderr, "morai_actuate: entering synchronous mode failed - %s\n", why.c_str());
    } else if (!session.setup(&setup_j, &idle, &why)) {
      session_abort = "setup: " + why;
      code = stop_requested() ? 5 : 4;
      std::fprintf(stderr, "morai_actuate: setup failed - %s\n", why.c_str());
    } else {
      const double ox = session.last_good().x;
      const double oy = session.last_good().y;
      std::map<std::string, int> repeat_of;
      for (size_t index = 0; index < run_names.size(); ++index) {
        const LockstepPatternSpec * p = find_lockstep_pattern(run_names[index]);
        if (stop_requested()) {
          session_abort = signal_text();
          break;
        }
        const double travelled = std::hypot(session.last_good().x - ox, session.last_good().y - oy);
        if (travelled + per_run > opt.max_total_distance) {
          std::printf(
            "[info] travel budget: %.1f m from the start + ~%.1f m > --max-total-distance %.0f; "
            "stopping here\n",
            travelled, per_run, opt.max_total_distance);
          budget_stop = true;
          break;
        }
        RunRecord r = session.run_pattern(*p, repeat_of[p->name]++, static_cast<int>(index));
        r.analysis = ls::analyze_run(make_input(r, session.steps(), idle.signature, opt));
        print_run_line(r);
        runs.push_back(r);
        if (!r.abort_reason.empty()) {
          session_abort = "run " + std::to_string(r.index) + " (" + r.pattern + "): " + r.abort_reason;
          break;
        }
        if (!r.stop.ok) {
          session_abort = "safe-stop failed: " + r.stop.why;
          break;
        }
        if (!r.brake_warning.empty()) {
          session_abort = "run " + std::to_string(r.index) + ": " + r.brake_warning;
          break;
        }
        // 명령이 아무 효과가 없으면(모드가 맞지 않아 서버가 조용히 버림) 남은 런을 돌리지 않는다
        if (runs.size() == 1 && r.max_speed < 0.05 * opt.accel * opt.drive_s) {
          session_abort = "the Ego did not move (vmax " + f3(r.max_speed) +
                          " m/s): MORAI ignores ControlVehicle in synchronous mode (control "
                          "mode? permission?)";
          break;
        }
      }
      if (!session_abort.empty()) {
        code = 5;
        std::fprintf(stderr, "morai_actuate: ABORTED - %s\n", session_abort.c_str());
      }
    }
  } catch (const std::exception & e) {
    session_abort = std::string("exception: ") + e.what();
    code = 5;
    std::fprintf(stderr, "morai_actuate: %s\n", session_abort.c_str());
  } catch (...) {
    session_abort = "unknown exception";
    code = 5;
    std::fprintf(stderr, "morai_actuate: %s\n", session_abort.c_str());
  }

  // restore() 는 단계마다 예외를 잡는다. 그래도 밖으로 나오면(예: 결과 JSON 할당 실패) 얼었을 수 있다고 크게 알린다
  Json restore = Json::object();
  try {
    restore = guard.restore();
  } catch (...) {
    std::printf(kFrozenBanner, opt.host.c_str(), opt.port);
    std::fprintf(stderr, "[restore] EXCEPTION escaped the restore - check MORAI now\n");
    std::fprintf(stderr, kFrozenBanner, opt.host.c_str(), opt.port);
    std::fflush(stdout);
    code = 6;
  }
  if (!guard.ok()) {
    code = 6;
  }

  // ---- analysis -------------------------------------------------------------------
  const std::vector<ls::Step> & steps = session.steps();
  const ls::DtReport session_dt = ls::check_dt(steps, opt.tick_period_ms);
  Json runs_j = Json::array();
  std::map<std::string, std::vector<const ls::RunAnalysis *>> by_pattern;
  std::vector<const ls::RunAnalysis *> all_runs;
  for (auto & r : runs) {
    runs_j.push(run_json(r));
    by_pattern[r.pattern].push_back(&r.analysis);
    if (!r.analysis.aborted) {
      all_runs.push_back(&r.analysis);
    }
  }
  std::vector<ls::PatternSummary> summaries;
  for (const auto & name : opt.patterns) {
    const auto it = by_pattern.find(name);
    if (it != by_pattern.end()) {
      summaries.push_back(ls::summarize_pattern(name, it->second));
    }
  }
  const ls::Interpretation interp =
    ls::interpret(summaries, opt.accel, idle.ambiguous, session_dt, all_runs);

  std::printf(
    "\n==== lockstep summary: tick_period %d ms, %zu Ticks in the session, idle brake %.4f (%s%s) "
    "====\n",
    opt.tick_period_ms, steps.size(), idle.signature.brake_center, idle.source.c_str(),
    idle.ambiguous ? ", AMBIGUOUS" : "");
  const ls::PatternSummary * ref = nullptr;
  for (const auto & p : summaries) {
    if (p.pattern == "sync1") {
      ref = &p;
    }
  }
  std::printf(
    "%-12s %-5s %-15s %6s %13s %8s %8s %8s %13s %9s %9s %8s\n", "pattern", "runs",
    "ratio mean+-se", "sd", "vs sync1", "a_cmd1", "a_unc1", "a_multi", "idle% cmd/unc",
    "sim/wall", "Tick p50", "dt viol");
  Json pats_j = Json::array();
  for (const auto & p : summaries) {
    char runs_txt[16];
    std::snprintf(runs_txt, sizeof(runs_txt), "%zu/%zu", p.valid, p.runs);
    const std::string ratio = f3(p.ratio_mean) + "+-" + f3(p.ratio_se);
    std::string vs = "-";
    if (ref && ref != &p) {
      const ls::RatioVs q = ls::ratio_vs(p, *ref);
      vs = f3(q.q) + "+-" + f3(q.se);
    }
    const std::string idle_txt =
      f2(100.0 * p.cmd1.idle_share()) + "/" + f2(100.0 * p.unc1.idle_share());
    std::printf(
      "%-12s %-5s %-15s %6s %13s %8s %8s %8s %13s %9s %9s %8zu\n", p.pattern.c_str(), runs_txt,
      ratio.c_str(), f3(p.ratio_sd).c_str(), vs.c_str(), f3(p.cmd1.accel()).c_str(),
      f3(p.unc1.accel()).c_str(), f3(p.multi.accel()).c_str(), idle_txt.c_str(),
      f2(p.rtf).c_str(), f2(p.tick_ms.p50).c_str(), p.dt_violations);
    pats_j.push(ls::to_json(p));
  }
  std::printf(
    "(ratio = LSQ speed slope over [%.1f s, drive end] / command, se = max(run spread, fit); vs "
    "sync1 = ratio / ratio(sync1): held 1, idle 1/cmd_every, Tick(3) persists 1 / first frame "
    "1/3; a_*: m/s^2 per Tick, cmd1/unc1 = Tick(1) with/without a command before it, multi = "
    "commanded Tick(n>1); sim/wall = lockstep speed)\n",
    Safety::kFitStartS);
  for (const auto & l : interp.lines) {
    std::printf("  %s\n", l.c_str());
  }
  std::printf("interpretation: %s\n", interp.line.c_str());
  if (!session_abort.empty()) {
    std::printf("session ABORTED: %s\n", session_abort.c_str());
  }
  std::printf(
    "restore: %s\n",
    guard.ok() ? "complete (real time verified, mode and gear read back, Ego stopped)"
               : (guard.sync_ok() ? "INCOMPLETE - check the Ego in MORAI"
                                  : "FAILED - MORAI may be frozen in synchronous mode — run "
                                    "morai_sync_restore"));

  out.set(
       "environment", Json::object()
                        .set("simulator_version", pf.version)
                        .set("original_mode", mode_name(pf.mode))
                        .set("original_gear", gear_name(pf.gear))
                        .set("original_position", Json::object().set("x", pf.x).set("y", pf.y))
                        .set("idle_calibration", to_json(idle)))
    .set("enter", enter_j)
    .set("setup", setup_j)
    .set("runs", std::move(runs_j))
    .set(
      "summary", Json::object()
                   .set("patterns", std::move(pats_j))
                   .set("session_dt", ls::to_json(session_dt))
                   .set("interpretation", ls::to_json(interp)))
    .set(
      "session", Json::object()
                   .set("aborted", !session_abort.empty())
                   .set("abort_reason", session_abort)
                   .set("travel_budget_stop", budget_stop)
                   .set("run_order", [&] {
                     Json a = Json::array();
                     for (const auto & n : run_names) {
                       a.push(n);
                     }
                     return a;
                   }())
                   .set("runs_completed", runs.size()))
    .set("restore", restore)
    .set("raw", Json::object().set("steps", raw_steps(steps, session.classes())));
  return finish(code);
}

}  // namespace morai_grpc_vendor::actuate
