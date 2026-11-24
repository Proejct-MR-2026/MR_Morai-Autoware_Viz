// 신규 — 26.09.25 _ YSH _ ControlVehicle 소비 단위(업데이트 vs 물리 스텝) 실측 CLI — Ego 를 실제로 움직인다
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

// morai_actuate — does MORAI consume ControlVehicle per update (~50/s) or per physics step
// (MORAI Hz)? Drives the Ego straight from standstill with +accel under several send
// patterns and measures what was realised.
//
// STATE-CHANGING, unlike morai_probe. It is a separate source file and executable ON
// PURPOSE: test/test_probe_readonly.cpp scans only tools/morai_probe.cpp, so the probe stays
// provably read-only. This file has its own allow-list in test/test_actuate_scope.cpp:
// ControlVehicle / AsyncControlVehicle (steer always 0.0), SetVehicleControlMode,
// SetVehicleGear, plus reads. Never add SetTransform, SetVelocity, Spawn*/Destroy*,
// SetSynchronousMode, Tick, Start/Stop/Pause/Resume or any other state-changing call.
//
// Safety (see README "morai_actuate"):
//   - refuses without --i-understand-this-moves-the-ego; banner + 3 s countdown
//   - read-only preflight: Ego present, |v| < 0.05, real-time mode, original mode KEYBOARD /
//     GAME_WHEEL (else --allow-any-original-mode), no pedal or steer input from anyone else;
//     records the ORIGINAL control mode and gear
//   - envelope per run: speed, displacement, failing reads, frozen MORAI time, collision,
//     a steer echo (this tool only sends 0.0, so a steer echo is another client)
//   - safe-stop: acceleration -3.0 m/s^2, steer 0, 100 Hz, until |v| < 0.05 m/s for 1 s;
//     accepted / rejected brake sends are counted and reported
//   - RestoreGuard (RAII) + signal flag: safe-stop, then ORIGINAL gear and mode, read back,
//     on every exit path once anything was changed. The signal handler only stores the
//     signal number (SIGINT, SIGTERM, SIGHUP, SIGQUIT, SIGTSTP).
//   - every state-changing RPC has a deadline and its Result.status is checked

#include "actuate_analysis.hpp"
// 26.09.25 _ YSH _ --lockstep 진입점. 동기 모드 코드(SetSynchronousMode·Tick)는 전부 actuate_lockstep.cpp 에 있고
// 이 파일(실시간 경로)은 여전히 부르지 않는다 — test_actuate_scope 가 파일별로 검증
#include "actuate_lockstep.hpp"
#include "actuate_options.hpp"
#include "probe_json.hpp"
#include "probe_stats.hpp"

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
#include <cinttypes>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

namespace act = morai_grpc_vendor::actuate;
namespace pb_actor = morai_sim_api::actor;
using morai_grpc_vendor::probe::grpc_code_name;
using morai_grpc_vendor::probe::Json;
using morai_grpc_vendor::probe::RawTable;
using morai_grpc_vendor::probe::summarize;
using act::Safety;
using Cell = RawTable::Cell;
using Clock = std::chrono::steady_clock;

static_assert(
  act::kModeKeyboard == static_cast<int>(pb_actor::VEHICLE_CONTROL_KEYBOARD) &&
    act::kModeGameWheel == static_cast<int>(pb_actor::VEHICLE_CONTROL_GAME_WHEEL),
  "actuate_analysis mode constants must match actor_enum.proto");

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;  // same as MORAI's Python SDK
/// Consecutive failed ControlVehicle calls that abort a run.
constexpr size_t kMaxConsecutiveSendFailures = 10;

// =============================================================================
// Signals — the handler only stores the number; every loop polls stop_requested()
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
  // Ctrl-\ (코어 덤프 종료)·Ctrl-Z (정지) 도 중단 → 안전 정지 → 복구. 정지된 채 AUTO/D 로 남으면
  // MORAI 는 명령을 유지하지 않으니 유휴 레코드(브레이크 ~0.003)로 굴러간다
  sigaction(SIGQUIT, &sa, nullptr);
  sigaction(SIGTSTP, &sa, nullptr);
  // 출력이 닫힌 파이프여도(| head 등) 주행 중 프로세스가 죽어 복구를 건너뛰지 않게
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

double to_ms(Clock::time_point t)
{
  return std::chrono::duration<double, std::milli>(t - g_t0).count();
}

double now_ms()
{
  return to_ms(Clock::now());
}

double unix_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::chrono::system_clock::time_point deadline_in(int ms)
{
  return std::chrono::system_clock::now() + std::chrono::milliseconds(ms);
}

Clock::duration period_of(double hz)
{
  return std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz));
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

/// Sleeps in small slices; false as soon as a signal arrives.
bool interruptible_sleep(double ms)
{
  const auto until = Clock::now() + std::chrono::duration_cast<Clock::duration>(
                                      std::chrono::duration<double, std::milli>(ms));
  while (Clock::now() < until) {
    if (stop_requested()) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return !stop_requested();
}

/// Fixed-rate pacing without catch-up bursts after a stall.
class Pacer
{
public:
  explicit Pacer(double hz)
  : period_(hz > 0.0 ? period_of(hz) : Clock::duration::zero()), next_(Clock::now())
  {
  }
  void wait()
  {
    if (period_ <= Clock::duration::zero()) {
      return;
    }
    std::this_thread::sleep_until(next_);
    next_ += period_;
    const auto now = Clock::now();
    if (now > next_ + period_) {
      next_ = now;
    }
  }

private:
  Clock::duration period_;
  Clock::time_point next_;
};

enum Phase : int {
  kPreflight = 0,
  kSetup,
  kHold,
  kIdleProbe,
  kDrive,
  kBrake,
  kSettle,
  kRestore,
  kPreHold,  ///< brake hold right before each drive
};

const char * phase_name(int p)
{
  static const char * k[] = {"preflight", "setup",  "hold",    "idle_probe", "drive",
                             "brake",     "settle", "restore", "pre_hold"};
  return (p >= 0 && p <= kPreHold) ? k[p] : "?";
}

int pattern_index(const std::string & name)
{
  const auto & k = act::known_patterns();
  for (size_t i = 0; i < k.size(); ++i) {
    if (name == k[i].name) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

grpc::ChannelArguments channel_args()
{
  grpc::ChannelArguments args;
  args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);
  // 채널마다 자기 TCP 연결 — 상태 읽기 루프가 명령 송신과 같은 연결에 줄 서지 않게
  args.SetInt(GRPC_ARG_USE_LOCAL_SUBCHANNEL_POOL, 1);
  return args;
}

// =============================================================================
// The only place a VehicleCtrlCmd is built. Steering is ALWAYS 0 in this tool.
// =============================================================================

pb_actor::VehicleCtrlCmd make_cmd(const morai_sim_api::ObjectInfo & ego, double accel)
{
  // main.py _fill_longitudinal 기본 모드와 같다: LONG_CMD_TYPE_ACCELERATION + acceleration, 페달 0
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
// State reader — own channel, tight GetAllActorsState loop, one row per DISTINCT timestamp
// =============================================================================

struct StateSample
{
  int64_t morai_us{0};
  uint64_t tick{0};
  double local_ms{0.0};  ///< midpoint of the poll that first saw this timestamp
  double poll_ms{0.0};
  double unix_s{0.0};
  uint32_t repeats_before{0};  ///< polls that returned the previous timestamp again
  int run{-1};
  int phase{kPreflight};
  double vx_kmh{0.0};
  double vy_kmh{0.0};
  double speed_mps{0.0};   ///< velocity.x km/h -> m/s (main.py)
  double planar_mps{0.0};  ///< hypot(vx, vy) km/h -> m/s
  double x{0.0};
  double y{0.0};
  double z{0.0};
  double yaw_deg{0.0};
  double ax{0.0};
  double ay{0.0};
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double fwa{0.0};
  int gear{-1};
  int collisions{0};
  int mode{-1};  ///< last GetVehicleControlMode (polled every 100 ms), -1 unknown
};

struct ReaderSnapshot
{
  bool have{false};
  StateSample latest;
  double last_ok_ms{-1e12};
  size_t count{0};
  size_t polls{0};
  size_t failures{0};
  size_t ego_missing{0};
  std::string last_error;
  bool armed{false};
  double armed_max_speed{0.0};
  double armed_max_dist{0.0};
  int armed_collisions{0};
  double armed_max_steer{0.0};  ///< max(|steer|, |fwa|) of records since arm()
  int mode{-1};
  bool dead{false};
};

class StateReader
{
public:
  StateReader(const act::Options & o, const morai_sim_api::ObjectInfo & ego) : opt_(o), ego_(ego)
  {
    channel_ = grpc::CreateCustomChannel(
      opt_.host + ":" + std::to_string(opt_.port), grpc::InsecureChannelCredentials(),
      channel_args());
    reader_stub_ = pb_actor::Actor::NewStub(channel_);
  }
  ~StateReader() { stop(); }
  StateReader(const StateReader &) = delete;
  StateReader & operator=(const StateReader &) = delete;

  bool connect(int timeout_ms) { return channel_->WaitForConnected(deadline_in(timeout_ms)); }

  void start()
  {
    stop_.store(false);
    thread_ = std::thread([this] {
      try {
        loop();
      } catch (const std::exception & e) {
        std::lock_guard<std::mutex> lk(mu_);
        dead_ = true;
        last_error_ = std::string("reader thread died: ") + e.what();
      } catch (...) {
        std::lock_guard<std::mutex> lk(mu_);
        dead_ = true;
        last_error_ = "reader thread died";
      }
    });
  }

  void stop()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  ReaderSnapshot snapshot() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    ReaderSnapshot s;
    s.have = have_;
    s.latest = latest_;
    s.last_ok_ms = last_ok_ms_;
    s.count = samples_.size();
    s.polls = polls_;
    s.failures = failures_;
    s.ego_missing = ego_missing_;
    s.last_error = last_error_;
    s.armed = armed_;
    s.armed_max_speed = armed_max_speed_;
    s.armed_max_dist = armed_max_dist_;
    s.armed_collisions = armed_collisions_;
    s.armed_max_steer = armed_max_steer_;
    s.mode = mode_;
    s.dead = dead_;
    return s;
  }

  int64_t latest_ts() const { return latest_ts_.load(); }
  size_t count() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return samples_.size();
  }
  void set_tag(int run, int phase)
  {
    run_.store(run);
    phase_.store(phase);
  }

  /// Start of a pattern run: origin = latest position; maxima restart from the latest state.
  void arm(double * ox, double * oy)
  {
    std::lock_guard<std::mutex> lk(mu_);
    armed_ = true;
    origin_x_ = latest_.x;
    origin_y_ = latest_.y;
    armed_max_speed_ = std::fabs(latest_.planar_mps);
    armed_max_dist_ = 0.0;
    armed_collisions_ = latest_.collisions;
    armed_max_steer_ = 0.0;  // arm() 뒤의 새 레코드만 본다
    *ox = origin_x_;
    *oy = origin_y_;
  }
  void disarm()
  {
    std::lock_guard<std::mutex> lk(mu_);
    armed_ = false;
  }

  std::vector<StateSample> all() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return samples_;
  }

  /// Polls until a fresh state is available.
  bool wait_for_state(double timeout_ms) const
  {
    const double until = now_ms() + timeout_ms;
    while (now_ms() < until) {
      const ReaderSnapshot s = snapshot();
      if (s.have && now_ms() - s.last_ok_ms < Safety::kMaxStaleS * 1e3) {
        return true;
      }
      if (stop_requested()) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
  }

private:
  const pb_actor::ActorState * find_ego(const pb_actor::ActorStates & res) const
  {
    for (const auto & s : res.states()) {
      if (s.actor_info().id().value() == opt_.ego) {
        return &s;
      }
    }
    for (const auto & s : res.states()) {
      if (iequals(s.actor_info().id().value(), opt_.ego)) {
        return &s;
      }
    }
    return nullptr;
  }

  void loop()
  {
    pb_actor::GetAllActorsFilter filter;
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);  // Ego 만 필요 — 보행자·장애물은 빼서 응답을 작게
    filter.set_pedestrian(false);
    filter.set_obstacle(false);
    Pacer pacer(opt_.reader_hz);
    double last_mode_ms = -1e12;
    uint32_t repeats = 0;
    while (!stop_.load()) {
      pacer.wait();
      grpc::ClientContext ctx;
      ctx.set_deadline(deadline_in(opt_.deadline_ms));
      pb_actor::ActorStates res;
      const auto a = Clock::now();
      const grpc::Status st = reader_stub_->GetAllActorsState(&ctx, filter, &res);
      const auto b = Clock::now();
      const double mid = 0.5 * (to_ms(a) + to_ms(b));
      if (!st.ok()) {
        {
          std::lock_guard<std::mutex> lk(mu_);
          ++failures_;
          last_error_ = "GetAllActorsState " + grpc_code_name(st.error_code());
        }
        if (st.error_code() != grpc::StatusCode::DEADLINE_EXCEEDED) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        continue;
      }
      const pb_actor::ActorState * e = find_ego(res);
      {
        std::lock_guard<std::mutex> lk(mu_);
        ++polls_;
        if (!e) {
          ++ego_missing_;
          last_error_ = "Ego '" + opt_.ego + "' not in GetAllActorsState";
        } else {
          last_ok_ms_ = mid;
          if (have_ && res.timestamp() == latest_.morai_us) {
            ++repeats;
          } else {
            StateSample s = build(res, *e, mid, to_ms(b) - to_ms(a));
            s.repeats_before = repeats;
            repeats = 0;
            samples_.push_back(s);
            latest_ = s;
            have_ = true;
            latest_ts_.store(s.morai_us);
            if (armed_) {
              armed_max_speed_ = std::max(armed_max_speed_, std::fabs(s.planar_mps));
              armed_max_dist_ =
                std::max(armed_max_dist_, std::hypot(s.x - origin_x_, s.y - origin_y_));
              armed_collisions_ = std::max(armed_collisions_, s.collisions);
              armed_max_steer_ =
                std::max({armed_max_steer_, std::fabs(s.steer), std::fabs(s.fwa)});
            }
          }
        }
      }
      if (mid - last_mode_ms >= 100.0) {  // 제어 모드는 100 ms 마다 (main.py 처럼 가끔만)
        last_mode_ms = mid;
        grpc::ClientContext mctx;
        mctx.set_deadline(deadline_in(opt_.deadline_ms));
        pb_actor::CurVehicleControlMode mres;
        if (reader_stub_->GetVehicleControlMode(&mctx, ego_, &mres).ok()) {
          std::lock_guard<std::mutex> lk(mu_);
          mode_ = static_cast<int>(mres.mode());
        }
      }
    }
  }

  StateSample build(
    const pb_actor::ActorStates & res, const pb_actor::ActorState & e, double mid,
    double poll_ms) const
  {
    StateSample s;
    s.morai_us = res.timestamp();
    s.tick = res.sync_mode_tick_count();
    s.local_ms = mid;
    s.poll_ms = poll_ms;
    s.unix_s = unix_now_s();
    s.run = run_.load();
    s.phase = phase_.load();
    s.vx_kmh = e.velocity().x();
    s.vy_kmh = e.velocity().y();
    s.speed_mps = s.vx_kmh * act::kKmhToMps;
    s.planar_mps = std::hypot(s.vx_kmh, s.vy_kmh) * act::kKmhToMps;
    s.x = e.transform().location().x();
    s.y = e.transform().location().y();
    s.z = e.transform().location().z();
    s.yaw_deg = e.transform().rotation().z();
    s.ax = e.acceleration().x();
    s.ay = e.acceleration().y();
    const auto & vs = e.vehicle_state();
    s.throttle = vs.throttle();
    s.brake = vs.brake();
    s.steer = vs.steer();
    s.fwa = vs.front_wheel_angle();
    s.gear = e.has_vehicle_state() ? static_cast<int>(vs.gear_mode()) : -1;
    s.collisions = vs.collision_objects_size();
    s.mode = mode_;
    return s;
  }

  const act::Options & opt_;
  morai_sim_api::ObjectInfo ego_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<pb_actor::Actor::Stub> reader_stub_;
  std::thread thread_;
  std::atomic<bool> stop_{false};
  std::atomic<int> run_{-1};
  std::atomic<int> phase_{kPreflight};
  std::atomic<int64_t> latest_ts_{0};

  mutable std::mutex mu_;
  std::vector<StateSample> samples_;
  StateSample latest_;
  bool have_{false};
  double last_ok_ms_{-1e12};
  size_t polls_{0};
  size_t failures_{0};
  size_t ego_missing_{0};
  std::string last_error_;
  bool armed_{false};
  double origin_x_{0.0};
  double origin_y_{0.0};
  double armed_max_speed_{0.0};
  double armed_max_dist_{0.0};
  int armed_collisions_{0};
  double armed_max_steer_{0.0};
  int mode_{-1};
  bool dead_{false};
};

// =============================================================================
// Send log
// =============================================================================

struct SendRecord
{
  size_t seq{0};
  int run{-1};
  int phase{kPreflight};
  int pattern{-1};
  double t_ms{0.0};
  double rpc_ms{kNaN};
  int grpc_code{-1};
  int result_status{-1};  ///< Result.status, -1 when no response
  double accel{0.0};
  int64_t prev_morai_us{0};
  int64_t wft_value{0};
  double wft_ms{kNaN};
  int burst_i{0};
  bool async{false};
  bool ok() const
  {
    return grpc_code == 0 && result_status == static_cast<int>(morai_sim_api::STATUS_CODE_SUCCESS);
  }
};

class SendLog
{
public:
  size_t add(SendRecord r)
  {
    std::lock_guard<std::mutex> lk(mu_);
    r.seq = v_.size();
    v_.push_back(r);
    return r.seq;
  }
  void complete(size_t i, double rpc_ms, int code, int status)
  {
    std::lock_guard<std::mutex> lk(mu_);
    if (i < v_.size()) {
      v_[i].rpc_ms = rpc_ms;
      v_[i].grpc_code = code;
      v_[i].result_status = status;
    }
  }
  std::vector<SendRecord> copy() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return v_;
  }
  size_t size() const
  {
    std::lock_guard<std::mutex> lk(mu_);
    return v_.size();
  }

private:
  mutable std::mutex mu_;
  std::vector<SendRecord> v_;
};

struct WftRecord
{
  int run{-1};
  double t_call_ms{0.0};
  double t_ret_ms{0.0};
  int grpc_code{0};
  int64_t value{0};
};

// =============================================================================
// Controller — control channel: ControlVehicle, Set*, WaitForTick, reads
// =============================================================================

class Controller
{
public:
  Controller(
    const act::Options & o, const morai_sim_api::ObjectInfo & ego, SendLog * log,
    const StateReader * reader)
  : opt_(o), ego_(ego), log_(log), reader_(reader)
  {
    channel_ = grpc::CreateCustomChannel(
      opt_.host + ":" + std::to_string(opt_.port), grpc::InsecureChannelCredentials(),
      channel_args());
    actor_stub_ = pb_actor::Actor::NewStub(channel_);
    sim_stub_ = morai_sim_api::simulation::Simulation::NewStub(channel_);
    // NOTE upstream spells this package "simualtor" (sic).
    simr_stub_ = morai_sim_api::simualtor::Simulator::NewStub(channel_);
  }
  ~Controller() { async_end(); }
  Controller(const Controller &) = delete;
  Controller & operator=(const Controller &) = delete;

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

  bool get_sync(morai_sim_api::simulation::SyncMode * out, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Empty req;
    const grpc::Status st = sim_stub_->GetSynchronousMode(&ctx, req, out);
    if (!st.ok()) {
      *err = "GetSynchronousMode " + grpc_code_name(st.error_code());
      return false;
    }
    return true;
  }

  bool get_spec(pb_actor::VehicleSpec * out, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    const grpc::Status st = actor_stub_->GetVehicleSpec(&ctx, ego_, out);
    if (!st.ok()) {
      *err = "GetVehicleSpec " + grpc_code_name(st.error_code());
      return false;
    }
    return true;
  }

  bool get_mode(int * mode, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    pb_actor::CurVehicleControlMode res;
    const grpc::Status st = actor_stub_->GetVehicleControlMode(&ctx, ego_, &res);
    if (!st.ok()) {
      *err = "GetVehicleControlMode " + grpc_code_name(st.error_code()) + " " + st.error_message();
      return false;
    }
    *mode = static_cast<int>(res.mode());
    return true;
  }

  bool wait_for_tick(int64_t * value, int * code)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.wft_deadline_ms));
    morai_sim_api::Empty req;
    morai_sim_api::Int64Value res;
    const grpc::Status st = sim_stub_->WaitForTick(&ctx, req, &res);
    *code = static_cast<int>(st.error_code());
    *value = res.value();
    return st.ok();
  }

  // --- writes: deadline + transport status + Result.status (SUCCESS = 1) ---------
  bool set_mode(int mode, std::string * err)
  {
    pb_actor::VehicleControlModeParam req;
    *req.mutable_actor_info() = ego_;
    req.set_mode(static_cast<pb_actor::VehicleControlMode>(mode));
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const grpc::Status st = actor_stub_->SetVehicleControlMode(&ctx, req, &res);
    return check("SetVehicleControlMode(" + mode_name(mode) + ")", st, res, err);
  }

  bool set_gear(int gear, std::string * err)
  {
    pb_actor::VehicleGear req;
    *req.mutable_actor_info() = ego_;
    req.set_gear(static_cast<pb_actor::GearMode>(gear));
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const grpc::Status st = actor_stub_->SetVehicleGear(&ctx, req, &res);
    return check("SetVehicleGear(" + gear_name(gear) + ")", st, res, err);
  }

  /// Blocking ControlVehicle. `m.prev_morai_us` may carry the WaitForTick value; the
  /// reader's latest timestamp is folded in right before the call.
  bool send(double accel, SendRecord m)
  {
    const pb_actor::VehicleCtrlCmd cmd = make_cmd(ego_, accel);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    m.accel = accel;
    m.prev_morai_us = std::max(m.prev_morai_us, reader_->latest_ts());
    const auto a = Clock::now();
    m.t_ms = to_ms(a);
    const grpc::Status st = actor_stub_->ControlVehicle(&ctx, cmd, &res);
    m.rpc_ms = to_ms(Clock::now()) - m.t_ms;
    m.grpc_code = static_cast<int>(st.error_code());
    m.result_status = st.ok() ? static_cast<int>(res.status()) : -1;
    log_->add(m);
    note_send(m.ok(), st, res);
    return m.ok();
  }

  // --- pipelined sends (free200) ------------------------------------------------
  void async_begin()
  {
    async_end();
    cq_ = std::make_unique<grpc::CompletionQueue>();
    inflight_.store(0);
    skipped_ = 0;
    drain_ = std::thread([this] {
      try {
        void * tag = nullptr;
        bool ok = false;
        while (cq_->Next(&tag, &ok)) {
          std::unique_ptr<AsyncCall> c(static_cast<AsyncCall *>(tag));
          const double ms = to_ms(Clock::now()) - c->t_ms;
          const int status = c->status.ok() ? static_cast<int>(c->res.status()) : -1;
          log_->complete(c->idx, ms, static_cast<int>(c->status.error_code()), status);
          note_send(c->status.ok() && status == morai_sim_api::STATUS_CODE_SUCCESS, c->status, c->res);
          inflight_.fetch_sub(1);
        }
      } catch (...) {
        // 완료 큐 드레인 실패 — 송신 실패로 집계되어 런이 중단된다
        consecutive_failures_.store(kMaxConsecutiveSendFailures);
      }
    });
  }

  /// false = skipped because --max-inflight sends are outstanding.
  bool async_send(double accel, SendRecord m)
  {
    if (!cq_ || inflight_.load() >= opt_.max_inflight) {
      ++skipped_;
      return false;
    }
    auto c = std::make_unique<AsyncCall>();
    c->req = make_cmd(ego_, accel);
    c->ctx.set_deadline(deadline_in(opt_.deadline_ms));
    m.accel = accel;
    m.async = true;
    m.prev_morai_us = std::max(m.prev_morai_us, reader_->latest_ts());
    m.t_ms = now_ms();
    c->t_ms = m.t_ms;
    c->idx = log_->add(m);
    inflight_.fetch_add(1);
    c->rpc = actor_stub_->AsyncControlVehicle(&c->ctx, c->req, cq_.get());
    AsyncCall * raw = c.release();  // 드레인 스레드가 태그로 받아 해제
    raw->rpc->Finish(&raw->res, &raw->status, raw);
    return true;
  }

  /// Waits for outstanding sends (bounded by the deadline), then stops the drain thread.
  void async_end()
  {
    if (!cq_) {
      return;
    }
    const auto until = Clock::now() + std::chrono::milliseconds(opt_.deadline_ms + 500);
    while (inflight_.load() > 0 && Clock::now() < until) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    cq_->Shutdown();
    if (drain_.joinable()) {
      drain_.join();
    }
    cq_.reset();
  }

  size_t async_skipped() const { return skipped_; }
  size_t consecutive_failures() const { return consecutive_failures_.load(); }
  void reset_failures() { consecutive_failures_.store(0); }
  std::string last_send_error() const
  {
    std::lock_guard<std::mutex> lk(err_mu_);
    return last_send_error_;
  }

private:
  struct AsyncCall
  {
    pb_actor::VehicleCtrlCmd req;
    grpc::ClientContext ctx;
    morai_sim_api::Result res;
    grpc::Status status;
    double t_ms{0.0};
    size_t idx{0};
    std::unique_ptr<grpc::ClientAsyncResponseReader<morai_sim_api::Result>> rpc;
  };

  static bool check(
    const std::string & name, const grpc::Status & st, const morai_sim_api::Result & res,
    std::string * err)
  {
    if (!st.ok()) {
      *err = name + " failed: " + grpc_code_name(st.error_code()) + " " + st.error_message();
      return false;
    }
    // StatusCode: UNKNOWN_FAILURE = 0, SUCCESS = 1 (0 을 성공으로 보면 오류가 전부 뒤집힌다)
    if (res.status() != morai_sim_api::STATUS_CODE_SUCCESS) {
      *err = name + " rejected: Result.status=" + std::to_string(static_cast<int>(res.status())) +
             " '" + res.description() + "'";
      return false;
    }
    return true;
  }

  void note_send(bool ok, const grpc::Status & st, const morai_sim_api::Result & res)
  {
    if (ok) {
      consecutive_failures_.store(0);
      return;
    }
    consecutive_failures_.fetch_add(1);
    std::lock_guard<std::mutex> lk(err_mu_);
    last_send_error_ = !st.ok() ? "ControlVehicle " + grpc_code_name(st.error_code())
                                : "ControlVehicle Result.status=" +
                                    std::to_string(static_cast<int>(res.status())) + " '" +
                                    res.description() + "'";
  }

  const act::Options & opt_;
  morai_sim_api::ObjectInfo ego_;
  SendLog * log_;
  const StateReader * reader_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<pb_actor::Actor::Stub> actor_stub_;
  std::unique_ptr<morai_sim_api::simulation::Simulation::Stub> sim_stub_;
  std::unique_ptr<morai_sim_api::simualtor::Simulator::Stub> simr_stub_;

  std::unique_ptr<grpc::CompletionQueue> cq_;
  std::thread drain_;
  std::atomic<int> inflight_{0};
  size_t skipped_{0};
  std::atomic<size_t> consecutive_failures_{0};
  mutable std::mutex err_mu_;
  std::string last_send_error_;
};

// =============================================================================
// Session — preflight, setup, runs, safe-stop
// =============================================================================

/// Brake sends of a safe-stop or a hold: failures are reported, not ignored.
struct BrakeStats
{
  bool ok{false};  ///< safe-stop: stopped (|v| < 0.05 for 1 s); hold: >= 1 send accepted
  double took_ms{kNaN};
  size_t sends{0};
  size_t accepted{0};
  bool stale_at_end{false};  ///< safe-stop: state reads were failing when it ended
  std::string why;           ///< safe-stop failure
  std::string last_error;    ///< last rejected / failed ControlVehicle

  /// "" or a warning: stopped although MORAI accepted no brake command.
  std::string warning() const
  {
    if (sends > 0 && accepted == 0) {
      return "0/" + std::to_string(sends) + " brake sends accepted (" +
             (last_error.empty() ? std::string("?") : last_error) +
             ") - the Ego was NOT braked by this tool";
    }
    return "";
  }
  std::string counts() const
  {
    return std::to_string(accepted) + "/" + std::to_string(sends) + " brake sends accepted";
  }
  Json json() const
  {
    return Json::object()
      .set("ok", ok)
      .set("took_ms", took_ms)
      .set("sends", sends)
      .set("accepted", accepted)
      .set("stale_at_end", stale_at_end)
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
  size_t sample_begin{0};
  size_t sample_end{0};
  size_t send_begin{0};
  size_t send_end{0};
  size_t wft_begin{0};
  size_t wft_end{0};
  int64_t t0_us{0};
  double t0_ms{kNaN};
  int64_t drive_end_us{0};
  double drive_end_ms{kNaN};
  double origin_x{0.0};
  double origin_y{0.0};
  std::string abort_reason;
  bool stop_ok{false};
  std::string stop_error;
  double stop_ms{kNaN};
  BrakeStats pre_hold;
  BrakeStats stop;
  BrakeStats settle;
  /// A brake phase in which MORAI accepted no ControlVehicle.
  std::string brake_warning;
  double max_speed{0.0};
  double max_dist{0.0};
  size_t async_skipped{0};
  size_t wft_failures{0};
  size_t ticks{0};
  act::RunAnalysis analysis;
};

struct Preflight
{
  bool ok{false};
  std::string why;
  int mode{-1};
  int gear{-1};
  double x{0.0};
  double y{0.0};
  std::string version;
  Json j = Json::object();
};

class Session
{
public:
  Session(const act::Options & o, Controller & c, StateReader & r, SendLog & l)
  : opt_(o), ctrl_(c), reader_(r), log_(l)
  {
    limits_.max_speed_mps = opt_.max_speed;
    limits_.max_distance_m = opt_.max_distance;
    limits_.max_stale_s = Safety::kMaxStaleS;
    limits_.max_frozen_s = Safety::kMaxFrozenS;
  }

  std::vector<WftRecord> & wft_log() { return wft_; }

  // ---------------------------------------------------------------------------
  /// Read-only. Nothing is changed here.
  Preflight preflight()
  {
    Preflight p;
    reader_.set_tag(-1, kPreflight);
    std::string err;
    if (ctrl_.get_version(&p.version, &err)) {
      std::printf("[preflight] MORAI version %s\n", p.version.c_str());
    } else {
      std::printf("[preflight] %s (continuing)\n", err.c_str());
    }
    p.j.set("simulator_version", p.version);

    morai_sim_api::simulation::SyncMode sync;
    if (ctrl_.get_sync(&sync, &err)) {
      const std::string t =
        enum_name(morai_sim_api::simulation::SyncModeType_descriptor(), sync.type());
      p.j.set("sync_type", t).set("sync_tick_period", sync.tick_period());
      std::printf("[preflight] sync mode %s tick_period %d\n", t.c_str(), sync.tick_period());
      if (sync.type() != morai_sim_api::simulation::SYNC_MODE_TYPE_UNSPECIFIED) {
        p.why = "MORAI is in " + t + "; this tool needs real-time (UNSPECIFIED) mode";
        return p;
      }
    } else {
      std::printf("[preflight] %s (continuing)\n", err.c_str());
      p.j.set("sync_error", err);
    }

    pb_actor::VehicleSpec spec;
    if (ctrl_.get_spec(&spec, &err)) {
      p.j.set(
        "spec", Json::object()
                  .set("sub_category", enum_name(pb_actor::VehicleClass_descriptor(), spec.sub_category()))
                  .set("length_m", static_cast<double>(spec.length()))
                  .set("wheel_base_m", static_cast<double>(spec.wheel_base()))
                  .set("max_steering_angle_deg", static_cast<double>(spec.max_steering_angle())));
      std::printf(
        "[preflight] spec wheelbase %.3f m, max steer %.1f deg\n", spec.wheel_base(),
        spec.max_steering_angle());
    } else {
      std::printf("[preflight] %s (continuing)\n", err.c_str());
    }

    if (!reader_.wait_for_state(2000.0)) {
      const ReaderSnapshot s = reader_.snapshot();
      p.why = "no Ego state within 2 s (" + (s.last_error.empty() ? "?" : s.last_error) + ")";
      return p;
    }
    // MORAI 시간이 흐르는지 (정지·동기 모드면 안 흐른다) + 다른 사람이 몰고 있지 않은지
    const size_t before = reader_.count();
    if (!interruptible_sleep(600.0)) {
      p.why = "interrupted (" + signal_text() + ")";
      return p;
    }
    const std::vector<StateSample> all = reader_.all();
    const size_t fresh = all.size() - std::min(before, all.size());
    std::vector<act::InputRecord> recs;
    for (size_t i = before; i < all.size(); ++i) {
      recs.push_back({all[i].throttle, all[i].brake, all[i].steer, all[i].fwa});
    }
    const act::ForeignInputCheck foreign = act::check_foreign_input(recs, 0.2);
    const ReaderSnapshot s = reader_.snapshot();
    p.j.set("updates_in_600ms", fresh)
      .set("active_pedal_records", foreign.pedal)
      .set("steer_echo_records", foreign.steer)
      .set("active_records", foreign.active);
    if (fresh < 5) {
      p.why = "MORAI time is not advancing (" + std::to_string(fresh) +
              " updates in 0.6 s): paused, or not in real-time mode";
      return p;
    }
    if (fresh > 600) {  // 1000/s 초과 — timestamp 가 폴링마다 바뀐다 (업데이트 단위가 아닐 수 있음)
      std::printf(
        "[preflight] WARNING: ActorStates.timestamp changed %zu times in 0.6 s - it may change "
        "on every poll rather than per update; per-update statistics would be meaningless\n",
        fresh);
    }
    if (foreign.driving) {
      p.why = foreign.why;
      return p;
    }
    p.gear = s.latest.gear;
    p.x = s.latest.x;
    p.y = s.latest.y;
    const double v = std::fabs(s.latest.planar_mps);
    p.j.set("speed_mps", v).set("gear", gear_name(p.gear)).set("x", p.x).set("y", p.y);
    if (!(v < Safety::kStopSpeed)) {
      p.why = "Ego is moving (" + std::to_string(v) + " m/s); it must be at standstill";
      return p;
    }
    if (p.gear < 0) {
      p.why = "Ego has no vehicle_state (gear unknown, cannot restore it)";
      return p;
    }
    if (!ctrl_.get_mode(&p.mode, &err)) {
      p.why = err + " (cannot record the original mode, so it could not be restored)";
      return p;
    }
    p.j.set("mode", mode_name(p.mode));
    std::printf(
      "[preflight] Ego at (%.2f, %.2f), %.3f m/s, gear %s, mode %s, %zu updates in 0.6 s\n", p.x,
      p.y, v, gear_name(p.gear).c_str(), mode_name(p.mode).c_str(), fresh);
    // AUTO 면 브릿지·Autoware 가 아직 보내고 있을 수 있다 (페달 0 ACCELERATION 명령은 유휴처럼 보인다)
    const std::string refusal =
      act::original_mode_refusal(p.mode, mode_name(p.mode), opt_.allow_any_mode);
    if (!refusal.empty()) {
      p.why = refusal;
      return p;
    }
    if (p.mode != act::kModeKeyboard && p.mode != act::kModeGameWheel) {
      std::printf(
        "[preflight] WARNING: taking over from %s (--allow-any-original-mode); make sure nothing "
        "else sends ControlVehicle\n",
        mode_name(p.mode).c_str());
    }

    bool any_locked = false;
    for (const auto & name : opt_.patterns) {
      const act::PatternSpec * ps = act::find_pattern(name);
      any_locked = any_locked || (ps && ps->locked);
    }
    if (any_locked) {
      size_t ok = 0;
      for (int i = 0; i < 3; ++i) {
        int64_t val = 0;
        int code = 0;
        ok += ctrl_.wait_for_tick(&val, &code) ? 1 : 0;
      }
      p.j.set("wait_for_tick_ok_of_3", ok);
      if (ok < 2) {
        p.why = "WaitForTick does not return within --wft-deadline-ms (" +
                std::to_string(ok) + "/3); locked patterns cannot run";
        return p;
      }
    }
    p.ok = true;
    return p;
  }

  // ---------------------------------------------------------------------------
  /// Envelope + signal + wall budget. "" = keep going.
  std::string check_abort(Clock::time_point start, double wall_budget_s)
  {
    if (stop_requested()) {
      return signal_text();
    }
    const ReaderSnapshot s = reader_.snapshot();
    if (s.dead) {
      return s.last_error;
    }
    const double now = now_ms();
    act::EnvelopeState st;
    st.have_state = s.have;
    st.speed_mps = s.armed ? s.armed_max_speed : std::fabs(s.latest.planar_mps);
    st.distance_m = s.armed ? s.armed_max_dist : 0.0;
    st.stale_s = (now - s.last_ok_ms) / 1e3;
    st.frozen_s = (now - s.latest.local_ms) / 1e3;
    st.collisions = static_cast<size_t>(s.armed ? s.armed_collisions : 0);
    st.steer_echo = s.armed ? s.armed_max_steer : 0.0;
    std::string why = act::check_envelope(limits_, st);
    if (!why.empty()) {
      return "envelope: " + why;
    }
    if (std::chrono::duration<double>(Clock::now() - start).count() > wall_budget_s) {
      return "drive not finished after " + std::to_string(wall_budget_s) +
             " s wall (MORAI time not advancing?)";
    }
    if (ctrl_.consecutive_failures() >= kMaxConsecutiveSendFailures) {
      return "ControlVehicle failing: " + ctrl_.last_send_error();
    }
    return "";
  }

  // ---------------------------------------------------------------------------
  /// Brake (acceleration -3.0, steer 0) at 100 Hz until |v| < 0.05 m/s for 1 s. Ignores
  /// signals on purpose — it IS the abort path. Fails after `timeout_s` (10 s). Every brake
  /// send is counted; the result remembers whether it ended with stale state.
  BrakeStats safe_stop(int run, int phase, double timeout_s = Safety::kStopTimeoutS)
  {
    reader_.set_tag(run, phase);
    ctrl_.reset_failures();
    BrakeStats b;
    const auto start = Clock::now();
    Pacer pacer(Safety::kSafeStopHz);
    double still_since = kNaN;
    int seen_signals = g_signal_count.load();
    while (true) {
      pacer.wait();
      SendRecord m;
      m.run = run;
      m.phase = phase;
      ++b.sends;
      if (ctrl_.send(Safety::kSafeStopAccel, m)) {
        ++b.accepted;
      } else {
        b.last_error = ctrl_.last_send_error();
      }
      if (g_signal_count.load() != seen_signals) {
        seen_signals = g_signal_count.load();
        std::printf("[safe-stop] still braking - restore follows, please wait\n");
        std::fflush(stdout);
      }
      const ReaderSnapshot s = reader_.snapshot();
      const double now = now_ms();
      const bool fresh = s.have && now - s.last_ok_ms <= Safety::kMaxStaleS * 1e3;
      b.stale_at_end = !fresh;
      if (fresh && std::fabs(s.latest.planar_mps) < Safety::kStopSpeed) {
        if (!std::isfinite(still_since)) {
          still_since = now;
        }
        if (now - still_since >= Safety::kStopHoldS * 1e3) {
          b.took_ms = to_ms(Clock::now()) - to_ms(start);
          b.ok = true;
          last_stop_ = b;
          return b;
        }
      } else {
        still_since = kNaN;
      }
      const double elapsed = std::chrono::duration<double>(Clock::now() - start).count();
      if (elapsed >= timeout_s) {
        b.took_ms = elapsed * 1e3;
        char buf[256];
        std::snprintf(
          buf, sizeof(buf), "not stopped after %.0f s (%s, |v|=%.3f m/s, %s%s%s)", timeout_s,
          fresh ? "state fresh" : "state STALE", std::fabs(s.latest.planar_mps),
          b.counts().c_str(), b.last_error.empty() ? "" : ", last error: ",
          b.last_error.c_str());
        b.why = buf;
        last_stop_ = b;
        return b;
      }
    }
  }

  /// Keeps braking for `seconds` (vehicle held). Stops early on a signal.
  BrakeStats hold(int run, int phase, double seconds)
  {
    reader_.set_tag(run, phase);
    Pacer pacer(Safety::kSafeStopHz);
    BrakeStats b;
    const auto start = Clock::now();
    const auto until = start + std::chrono::duration_cast<Clock::duration>(
                                 std::chrono::duration<double>(seconds));
    while (Clock::now() < until && !stop_requested()) {
      pacer.wait();
      SendRecord m;
      m.run = run;
      m.phase = phase;
      ++b.sends;
      if (ctrl_.send(Safety::kSafeStopAccel, m)) {
        ++b.accepted;
      } else {
        b.last_error = ctrl_.last_send_error();
      }
    }
    b.took_ms = to_ms(Clock::now()) - to_ms(start);
    b.ok = b.accepted > 0;
    return b;
  }

  /// Something may have been changed (set right before the first state-changing call).
  bool touched() const { return touched_; }
  /// The most recent safe-stop.
  const BrakeStats & last_stop() const { return last_stop_; }

  // ---------------------------------------------------------------------------
  /// AUTO_MODE + gear D, read back; brake hold (proves ControlVehicle is accepted); then
  /// a no-command window that measures MORAI's idle record (and shows whether another client
  /// starts commanding once the Ego is in AUTO).
  bool setup(Json * j, act::IdleCalibration * idle, std::string * why)
  {
    reader_.set_tag(-1, kSetup);
    std::string err;
    const int auto_mode = static_cast<int>(pb_actor::VEHICLE_CONTROL_AUTO_MODE);
    const int drive = static_cast<int>(pb_actor::GEAR_MODE_D);
    // 카운트다운 뒤 · 모드 되읽기 중에 온 신호: 더 바꾸지 않는다
    if (stop_requested()) {
      *why = signal_text() + " before any change";
      return false;
    }
    touched_ = true;
    if (!ctrl_.set_mode(auto_mode, &err)) {
      *why = err;
      return false;
    }
    int mode = -1;
    if (!poll_until(1500.0, [&] { return ctrl_.get_mode(&mode, &err) && mode == auto_mode; })) {
      *why = stop_requested() ? signal_text()
                              : "control mode did not read back as AUTO_MODE (got " +
                                  mode_name(mode) + ")";
      return false;
    }
    std::printf("[setup] mode -> %s (read back)\n", mode_name(mode).c_str());
    if (stop_requested()) {
      *why = signal_text();
      return false;
    }
    const double t_gear = now_ms();
    if (!ctrl_.set_gear(drive, &err)) {
      *why = err;
      return false;
    }
    int gear = -1;
    if (!poll_until(1500.0, [&] { return fresh_gear(t_gear, &gear) && gear == drive; })) {
      *why = stop_requested() ? signal_text()
                              : "gear did not read back as D (got " + gear_name(gear) + ")";
      return false;
    }
    std::printf("[setup] gear -> %s (read back)\n", gear_name(gear).c_str());
    j->set("mode", mode_name(mode)).set("gear", gear_name(gear));

    const BrakeStats held = hold(-1, kHold, 0.5);
    j->set("hold", held.json());
    if (stop_requested()) {
      *why = signal_text();
      return false;
    }
    if (held.accepted == 0) {
      *why = "ControlVehicle not accepted during the brake hold: " + held.last_error;
      return false;
    }

    // 명령 없는 창: MORAI 유휴 레코드 실측 (앞 100 ms 는 직전 브레이크 명령 잔여로 버린다)
    reader_.set_tag(-1, kIdleProbe);
    const double t_idle = now_ms();
    if (opt_.idle_probe_s > 0.0 && !interruptible_sleep(opt_.idle_probe_s * 1e3)) {
      *why = signal_text();
      return false;
    }
    std::vector<std::pair<double, double>> recs;
    std::vector<act::InputRecord> inputs;
    for (const auto & s : reader_.all()) {
      if (s.phase == kIdleProbe && s.local_ms >= t_idle + 100.0) {
        recs.emplace_back(s.throttle, s.brake);
        inputs.push_back({0.0, 0.0, s.steer, s.fwa});  // 페달은 유휴 판정용, 여기선 조향만
      }
    }
    // AUTO 로 바꾼 뒤 우리가 아무것도 안 보내는데 조향 에코가 보이면 다른 클라이언트가 보내고 있다
    const act::ForeignInputCheck foreign = act::check_foreign_input(inputs, 0.2);
    j->set("idle_probe_steer_echo_records", foreign.steer);
    if (foreign.steer > 0) {
      *why = "steer echo in " + std::to_string(foreign.steer) + "/" + std::to_string(foreign.n) +
             " records while this tool sent nothing: another client is commanding the Ego";
      return false;
    }
    act::IdleSignature fallback;
    fallback.throttle_max = opt_.idle_throttle_max;
    fallback.brake_center = opt_.idle_brake;
    fallback.brake_tol = opt_.idle_brake_tol;
    *idle = act::calibrate_idle(recs, fallback);
    std::printf(
      "[setup] idle record (no command, %zu updates): throttle %.4f brake %.4f -> signature "
      "brake %.4f+-%.4f (%s)%s\n",
      idle->n, idle->throttle_median, idle->brake_median, idle->signature.brake_center,
      idle->signature.brake_tol, idle->source.c_str(), idle->ambiguous ? " AMBIGUOUS" : "");
    for (const auto & w : idle->warnings) {
      std::printf("[setup] WARNING: %s\n", w.c_str());
    }
    const ReaderSnapshot s = reader_.snapshot();
    if (!(std::fabs(s.latest.planar_mps) < Safety::kStopSpeed)) {
      const BrakeStats b = safe_stop(-1, kHold);
      if (!b.ok) {
        *why = "Ego rolls without a command and safe-stop failed: " + b.why;
        return false;
      }
    }
    return true;
  }

  /// Gear from a state sample taken after `since_ms` (a stale sample proves nothing).
  bool fresh_gear(double since_ms, int * gear) const
  {
    const ReaderSnapshot s = reader_.snapshot();
    if (!s.have || !(s.latest.local_ms > since_ms)) {
      return false;
    }
    *gear = s.latest.gear;
    return true;
  }

  // ---------------------------------------------------------------------------
  RunRecord run_pattern(const act::PatternSpec & p, int repeat, int index)
  {
    RunRecord r;
    r.index = index;
    r.pattern = p.name;
    r.repeat = repeat;
    ctrl_.reset_failures();
    ReaderSnapshot s = reader_.snapshot();
    if (!s.have || !(std::fabs(s.latest.planar_mps) < Safety::kStopSpeed)) {
      const BrakeStats b = safe_stop(index, kBrake);
      if (!b.ok) {
        r.abort_reason = "not at standstill before the run: " + b.why;
        r.stop_error = b.why;
        r.stop = b;
        return r;
      }
    }
    // 모든 런이 같은 상태(브레이크로 잡힌 정지)에서 시작하게 — 첫 런은 명령 없는 유휴 창 뒤다
    r.pre_hold = hold(index, kPreHold, Safety::kPreDriveHoldS);
    if (stop_requested()) {
      r.abort_reason = signal_text();
      r.stop_ok = true;  // 아직 출발 전 (정지 상태)
      return r;
    }
    if (r.pre_hold.accepted == 0) {
      r.abort_reason = "brake not accepted during the pre-drive hold: " + r.pre_hold.last_error;
      r.brake_warning = r.pre_hold.warning();
      return r;
    }
    reader_.set_tag(index, kDrive);
    const size_t n = reader_.count();
    r.sample_begin = n ? n - 1 : 0;
    r.send_begin = log_.size();
    r.wft_begin = wft_.size();
    reader_.arm(&r.origin_x, &r.origin_y);
    ctrl_.reset_failures();

    r.abort_reason = p.locked ? drive_locked(p, &r) : drive_free(p, &r);
    r.drive_end_ms = now_ms();
    r.drive_end_us = reader_.latest_ts();

    r.stop = safe_stop(index, kBrake);
    r.stop_ok = r.stop.ok;
    r.stop_error = r.stop.why;
    r.stop_ms = r.stop.took_ms;
    if (!r.stop.warning().empty()) {
      r.brake_warning = "safe-stop: " + r.stop.warning();
    }
    s = reader_.snapshot();
    r.max_speed = s.armed_max_speed;
    r.max_dist = s.armed_max_dist;
    reader_.disarm();
    if (r.stop_ok && r.abort_reason.empty() && r.brake_warning.empty() && opt_.settle_s > 0.0) {
      r.settle = hold(index, kSettle, opt_.settle_s);
      if (!r.settle.warning().empty() && !stop_requested()) {
        r.brake_warning = "settle: " + r.settle.warning();
      }
    }
    r.sample_end = reader_.count();
    r.send_end = log_.size();
    r.wft_end = wft_.size();
    return r;
  }

private:
  template <typename F>
  bool poll_until(double timeout_ms, F && f)
  {
    const double until = now_ms() + timeout_ms;
    do {
      if (f()) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (now_ms() < until && !stop_requested());
    return f();
  }

  SendRecord meta(const RunRecord & r, int phase) const
  {
    SendRecord m;
    m.run = r.index;
    m.phase = phase;
    m.pattern = pattern_index(r.pattern);
    return m;
  }

  int64_t drive_us() const { return std::llround(opt_.drive_s * 1e6); }
  double wall_budget_s() const { return opt_.drive_s * 3.0 + 2.0; }

  /// After each WaitForTick return (every n-th), `burst` blocking sends.
  std::string drive_locked(const act::PatternSpec & p, RunRecord * r)
  {
    const auto start = Clock::now();
    bool started = false;
    size_t ticks = 0;
    while (true) {
      std::string why = check_abort(start, wall_budget_s());
      if (!why.empty()) {
        return why;
      }
      if (started && reader_.latest_ts() - r->t0_us >= drive_us()) {
        return "";
      }
      WftRecord w;
      w.run = r->index;
      w.t_call_ms = now_ms();
      int64_t v = 0;
      int code = 0;
      const bool ok = ctrl_.wait_for_tick(&v, &code);
      w.t_ret_ms = now_ms();
      w.grpc_code = code;
      w.value = v;
      wft_.push_back(w);
      if (!ok) {
        ++r->wft_failures;
        continue;
      }
      if (started && v - r->t0_us >= drive_us()) {
        return "";  // 방금 나온 업데이트가 이미 창 끝 — 더 보내지 않는다
      }
      ++ticks;
      r->ticks = ticks;
      if ((ticks - 1) % static_cast<size_t>(p.every_n) != 0) {
        continue;
      }
      for (int b = 0; b < p.burst; ++b) {
        SendRecord m = meta(*r, kDrive);
        m.wft_value = v;
        m.wft_ms = w.t_ret_ms;
        m.burst_i = b;
        m.prev_morai_us = v;
        if (!started) {
          r->t0_us = std::max(v, reader_.latest_ts());
          r->t0_ms = now_ms();
          started = true;
        }
        ctrl_.send(opt_.accel, m);
      }
    }
  }

  /// Fixed-rate timer; blocking sends, or pipelined async sends for `p.pipelined`.
  std::string drive_free(const act::PatternSpec & p, RunRecord * r)
  {
    const auto start = Clock::now();
    bool started = false;
    std::string why;
    if (p.pipelined) {
      ctrl_.async_begin();
    }
    Pacer pacer(p.hz);
    while (true) {
      why = check_abort(start, wall_budget_s());
      if (!why.empty()) {
        break;
      }
      if (started && reader_.latest_ts() - r->t0_us >= drive_us()) {
        break;
      }
      pacer.wait();
      SendRecord m = meta(*r, kDrive);
      const int64_t prev = reader_.latest_ts();
      if (p.pipelined) {
        if (ctrl_.async_send(opt_.accel, m) && !started) {
          r->t0_us = prev;
          r->t0_ms = now_ms();
          started = true;
        }
      } else {
        if (!started) {
          r->t0_us = prev;
          r->t0_ms = now_ms();
          started = true;
        }
        ctrl_.send(opt_.accel, m);
      }
    }
    if (p.pipelined) {
      ctrl_.async_end();
      r->async_skipped = ctrl_.async_skipped();
    }
    return why;
  }

  const act::Options & opt_;
  Controller & ctrl_;
  StateReader & reader_;
  SendLog & log_;
  act::EnvelopeLimits limits_;
  std::vector<WftRecord> wft_;
  bool touched_{false};
  BrakeStats last_stop_;
};

// =============================================================================
// Restore guard — safe-stop, ORIGINAL gear, ORIGINAL mode, read back. Runs once, from
// restore() or from the destructor (exception / early return).
// =============================================================================

class RestoreGuard
{
public:
  RestoreGuard(Session & s, Controller & c, int mode, int gear)
  : session_(s), ctrl_(c), mode_(mode), gear_(gear)
  {
  }
  ~RestoreGuard()
  {
    if (armed_ && !done_) {
      try {
        restore();
      } catch (...) {
        std::fprintf(stderr, "[restore] EXCEPTION during restore - check MORAI now\n");
      }
    }
  }
  RestoreGuard(const RestoreGuard &) = delete;
  RestoreGuard & operator=(const RestoreGuard &) = delete;

  void arm() { armed_ = true; }
  bool ok() const { return !armed_ || ok_; }

  Json restore()
  {
    if (!armed_) {
      return Json::object().set("needed", false);
    }
    if (done_) {
      return result_;
    }
    done_ = true;
    if (!session_.touched()) {
      // 준비(setup)가 상태를 바꾸기 전에 끝났다 — 되돌릴 것이 없고 아무것도 보내지 않는다
      std::printf("\n[restore] nothing was changed (stopped before setup) - nothing to restore\n");
      std::fflush(stdout);
      ok_ = true;
      result_ = Json::object().set("needed", false).set("ok", true).set(
        "reason", "stopped before the first state-changing call");
      return result_;
    }
    // 직전 안전 정지가 상태 끊김·브레이크 전부 거부로 실패했으면(MORAI 불통·거부) 10 s 를 또 쓰지 않고
    // 짧게 브레이크한 뒤 곧장 기어·모드를 시도한다
    const BrakeStats & prev = session_.last_stop();
    const bool prev_failed_dead =
      prev.sends > 0 && !prev.ok && (prev.stale_at_end || prev.accepted == 0);
    const double stop_timeout =
      prev_failed_dead ? Safety::kRestoreStopAfterFailureS : Safety::kStopTimeoutS;
    std::printf(
      "\n[restore] safe-stop%s, then gear %s and mode %s\n",
      prev_failed_dead
        ? (prev.accepted == 0 ? " (short: the previous safe-stop had no brake command accepted)"
                              : " (short: the previous safe-stop ended with state reads failing)")
        : "",
      gear_name(gear_).c_str(), mode_name(mode_).c_str());
    std::fflush(stdout);
    const BrakeStats stop = session_.safe_stop(-1, kRestore, stop_timeout);
    std::printf(
      "[restore] safe-stop %s (%.1f s), %s%s%s\n", stop.ok ? "OK" : "FAILED", stop.took_ms / 1e3,
      stop.counts().c_str(), stop.ok ? "" : ": ", stop.why.c_str());
    if (!stop.warning().empty()) {
      std::printf("[restore] WARNING: %s\n", stop.warning().c_str());
    }
    if (!stop.ok) {
      std::printf("[restore] WARNING: restoring gear/mode although the Ego may still move\n");
    }

    // 기어 먼저 (아직 AUTO 일 때), 그다음 모드. 되읽기는 설정 뒤에 받은 상태로만 인정한다
    std::string gear_err;
    bool gear_set = false;
    const double t_gear = now_ms();
    for (int i = 0; i < 3 && !gear_set; ++i) {
      gear_set = ctrl_.set_gear(gear_, &gear_err);
    }
    int gear_now = -1;
    bool gear_ok = false;
    for (double until = now_ms() + 1500.0; now_ms() < until;) {
      if (session_.fresh_gear(t_gear, &gear_now) && gear_now == gear_) {
        gear_ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::string mode_err;
    bool mode_set = false;
    for (int i = 0; i < 3 && !mode_set; ++i) {
      mode_set = ctrl_.set_mode(mode_, &mode_err);
    }
    int mode_now = -1;
    bool mode_ok = false;
    for (double until = now_ms() + 1500.0; now_ms() < until;) {
      std::string e;
      if (ctrl_.get_mode(&mode_now, &e) && mode_now == mode_) {
        mode_ok = true;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::printf(
      "[restore] gear %s: set %s, read back %s %s\n", gear_name(gear_).c_str(),
      gear_set ? "OK" : ("FAILED (" + gear_err + ")").c_str(), gear_name(gear_now).c_str(),
      gear_ok ? "- restored" : "- NOT RESTORED");
    std::printf(
      "[restore] mode %s: set %s, read back %s %s\n", mode_name(mode_).c_str(),
      mode_set ? "OK" : ("FAILED (" + mode_err + ")").c_str(), mode_name(mode_now).c_str(),
      mode_ok ? "- restored" : "- NOT RESTORED");
    ok_ = stop.ok && gear_ok && mode_ok;
    std::printf(
      "[restore] %s\n",
      ok_ ? (stop.warning().empty() ? "COMPLETE" : "COMPLETE (brake WARNING above)")
          : "INCOMPLETE - check the Ego in MORAI NOW (mode/gear)");
    std::fflush(stdout);
    result_ = Json::object()
                .set("needed", true)
                .set("ok", ok_)
                .set("safe_stop_ok", stop.ok)
                .set("safe_stop_ms", stop.took_ms)
                .set("safe_stop_error", stop.why)
                .set("safe_stop_timeout_s", stop_timeout)
                .set("safe_stop", stop.json())
                .set("safe_stop_warning", stop.warning())
                .set("previous_safe_stop", prev.json())
                .set("original_gear", gear_name(gear_))
                .set("gear_set_ok", gear_set)
                .set("gear_error", gear_err)
                .set("gear_read_back", gear_name(gear_now))
                .set("gear_restored", gear_ok)
                .set("original_mode", mode_name(mode_))
                .set("mode_set_ok", mode_set)
                .set("mode_error", mode_err)
                .set("mode_read_back", mode_name(mode_now))
                .set("mode_restored", mode_ok);
    return result_;
  }

private:
  Session & session_;
  Controller & ctrl_;
  int mode_;
  int gear_;
  bool armed_{false};
  bool done_{false};
  bool ok_{false};
  Json result_;
};

// =============================================================================
// Output
// =============================================================================

std::vector<act::UpdateSample> to_updates(const std::vector<StateSample> & s, size_t b, size_t e)
{
  std::vector<act::UpdateSample> u;
  for (size_t i = b; i < std::min(e, s.size()); ++i) {
    act::UpdateSample x;
    x.morai_us = s[i].morai_us;
    x.local_ms = s[i].local_ms;
    x.speed_mps = s[i].speed_mps;
    x.x = s[i].x;
    x.y = s[i].y;
    x.throttle = s[i].throttle;
    x.brake = s[i].brake;
    u.push_back(x);
  }
  return u;
}

act::StepEstimate step_from(const std::vector<StateSample> & s, double physics_hz)
{
  std::vector<int64_t> d;
  for (size_t i = 1; i < s.size(); ++i) {
    d.push_back(s[i].morai_us - s[i - 1].morai_us);
  }
  return act::estimate_physics_step(d, physics_hz > 0.0 ? 1e6 / physics_hz : 0.0);
}

act::RunInput make_input(
  const RunRecord & r, const std::vector<StateSample> & samples,
  const std::vector<SendRecord> & sends, double step_us, const act::IdleSignature & idle,
  const act::Options & o)
{
  act::RunInput in;
  in.updates = to_updates(samples, r.sample_begin, r.sample_end);
  for (size_t i = r.send_begin; i < std::min(r.send_end, sends.size()); ++i) {
    if (sends[i].phase == kDrive && sends[i].run == r.index) {
      in.sends.push_back(act::SendSample{sends[i].t_ms, sends[i].prev_morai_us, sends[i].ok()});
    }
  }
  in.t0_us = r.t0_us;
  in.drive_s = o.drive_s;
  // 중단된 런은 드라이브 루프가 끝난 시점(그 뒤는 브레이크)에서 창을 자른다
  if (r.drive_end_us > r.t0_us) {
    in.drive_s = std::min(o.drive_s, static_cast<double>(r.drive_end_us - r.t0_us) / 1e6);
  }
  in.fit_start_s = Safety::kFitStartS;
  in.commanded_accel = o.accel;
  in.step_us = step_us;
  in.idle = idle;
  in.aborted = !r.abort_reason.empty() || r.t0_us == 0;
  return in;
}

act::RunAnalysis analyze(
  const RunRecord & r, const std::vector<StateSample> & samples,
  const std::vector<SendRecord> & sends, double step_us, const act::IdleSignature & idle,
  const act::Options & o)
{
  return act::analyze_run(make_input(r, samples, sends, step_us, idle, o));
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

std::string f2(double v)
{
  char b[32];
  if (!std::isfinite(v)) {
    return "-";
  }
  std::snprintf(b, sizeof(b), "%.2f", v);
  return b;
}

void print_run_line(const RunRecord & r)
{
  const auto & a = r.analysis;
  std::printf(
    "[run %2d] %-11s #%d: ratio %s (a=%s m/s^2, r2 %s), idle %s%%, sends/upd %s, steps/upd %s, "
    "dv~k %s, vmax %.2f m/s, dmax %.2f m, stop %s s (%s)%s%s\n",
    r.index, r.pattern.c_str(), r.repeat + 1, f3(a.ratio).c_str(), f3(a.realised_accel).c_str(),
    f3(a.fit.r2).c_str(), f3(100.0 * a.idle_fraction).c_str(), f3(a.sends_per_update).c_str(),
    f3(a.steps_per_update).c_str(), f3(a.k_share).c_str(), r.max_speed, r.max_dist,
    f3(r.stop_ms / 1e3).c_str(), r.stop.counts().c_str(),
    r.abort_reason.empty() ? "" : " ABORTED: ", r.abort_reason.c_str());
  if (!r.brake_warning.empty()) {
    std::printf("[run %2d] WARNING: %s\n", r.index, r.brake_warning.c_str());
  }
  std::fflush(stdout);
}

Json send_stats(const std::vector<SendRecord> & sends, const RunRecord & r)
{
  std::vector<double> ms;
  size_t n = 0;
  size_t ok = 0;
  for (size_t i = r.send_begin; i < std::min(r.send_end, sends.size()); ++i) {
    if (sends[i].phase != kDrive || sends[i].run != r.index) {
      continue;
    }
    ++n;
    ok += sends[i].ok() ? 1 : 0;
    if (std::isfinite(sends[i].rpc_ms)) {
      ms.push_back(sends[i].rpc_ms);
    }
  }
  return Json::object()
    .set("drive_sends", n)
    .set("drive_sends_ok", ok)
    .set("rpc_ms", morai_grpc_vendor::probe::to_json(summarize(ms)));
}

Json run_json(const RunRecord & r, const std::vector<SendRecord> & sends)
{
  return Json::object()
    .set("index", r.index)
    .set("pattern", r.pattern)
    .set("repeat", r.repeat)
    .set("t0_us", r.t0_us)
    .set("t0_local_ms", r.t0_ms)
    .set("drive_end_us", r.drive_end_us)
    .set("drive_end_local_ms", r.drive_end_ms)
    .set("origin_x", r.origin_x)
    .set("origin_y", r.origin_y)
    .set("aborted", !r.abort_reason.empty())
    .set("abort_reason", r.abort_reason)
    .set("stop_ok", r.stop_ok)
    .set("stop_error", r.stop_error)
    .set("stop_ms", r.stop_ms)
    .set("pre_hold", r.pre_hold.json())
    .set("safe_stop", r.stop.json())
    .set("settle", r.settle.json())
    .set("brake_warning", r.brake_warning)
    .set("max_speed_mps", r.max_speed)
    .set("max_displacement_m", r.max_dist)
    .set("ticks", r.ticks)
    .set("wft_failures", r.wft_failures)
    .set("async_skipped_inflight", r.async_skipped)
    .set("sample_range", Json::array().push(r.sample_begin).push(r.sample_end))
    .set("send_range", Json::array().push(r.send_begin).push(r.send_end))
    .set("send_stats", send_stats(sends, r))
    .set("analysis", act::to_json(r.analysis));
}

/// (throttle, brake) value counts per phase, rounded to 1e-4 — what MORAI's records look
/// like with and without commands.
Json record_signatures(const std::vector<StateSample> & s)
{
  std::map<int, std::map<std::pair<long, long>, size_t>> m;
  for (const auto & x : s) {
    ++m[x.phase][{std::lround(x.throttle * 1e4), std::lround(x.brake * 1e4)}];
  }
  Json j = Json::object();
  for (const auto & ph : m) {
    std::vector<std::pair<size_t, std::pair<long, long>>> v;
    for (const auto & kv : ph.second) {
      v.emplace_back(kv.second, kv.first);
    }
    std::sort(v.begin(), v.end(), [](const auto & a, const auto & b) { return a.first > b.first; });
    Json arr = Json::array();
    for (size_t i = 0; i < v.size() && i < 8; ++i) {
      arr.push(Json::object()
                 .set("throttle", static_cast<double>(v[i].second.first) / 1e4)
                 .set("brake", static_cast<double>(v[i].second.second) / 1e4)
                 .set("count", v[i].first));
    }
    j.set(phase_name(ph.first), std::move(arr));
  }
  return j;
}

Json raw_samples(const std::vector<StateSample> & s)
{
  RawTable t(
    {"idx", "run", "phase", "morai_us", "tick", "local_ms", "poll_ms", "unix_s", "repeats_before",
     "vx_kmh", "vy_kmh", "speed_mps", "planar_mps", "x", "y", "z", "yaw_deg", "ax", "ay",
     "throttle", "brake", "steer", "front_wheel_angle", "gear", "collisions", "control_mode"},
    10000000);
  for (size_t i = 0; i < s.size(); ++i) {
    const auto & x = s[i];
    t.add(
      {Cell::u(i), Cell::i(x.run), Cell::i(x.phase), Cell::i(x.morai_us), Cell::u(x.tick),
       Cell::d(x.local_ms), Cell::d(x.poll_ms), Cell::d(x.unix_s), Cell::u(x.repeats_before),
       Cell::d(x.vx_kmh), Cell::d(x.vy_kmh), Cell::d(x.speed_mps), Cell::d(x.planar_mps),
       Cell::d(x.x), Cell::d(x.y), Cell::d(x.z), Cell::d(x.yaw_deg), Cell::d(x.ax), Cell::d(x.ay),
       Cell::d(x.throttle), Cell::d(x.brake), Cell::d(x.steer), Cell::d(x.fwa), Cell::i(x.gear),
       Cell::i(x.collisions), Cell::i(x.mode)});
  }
  return t.to_json();
}

Json raw_sends(const std::vector<SendRecord> & v)
{
  RawTable t(
    {"seq", "run", "phase", "pattern", "t_ms", "rpc_ms", "grpc_code", "result_status", "accel",
     "prev_morai_us", "wft_value", "wft_ms", "burst_i", "async"},
    10000000);
  for (const auto & x : v) {
    t.add(
      {Cell::u(x.seq), Cell::i(x.run), Cell::i(x.phase), Cell::i(x.pattern), Cell::d(x.t_ms),
       Cell::d(x.rpc_ms), Cell::i(x.grpc_code), Cell::i(x.result_status), Cell::d(x.accel),
       Cell::i(x.prev_morai_us), x.wft_value ? Cell::i(x.wft_value) : Cell::null(),
       Cell::d(x.wft_ms), Cell::i(x.burst_i), Cell::i(x.async ? 1 : 0)});
  }
  return t.to_json();
}

Json raw_wft(const std::vector<WftRecord> & v)
{
  RawTable t({"run", "t_call_ms", "t_ret_ms", "grpc_code", "value"}, 10000000);
  for (const auto & x : v) {
    t.add(
      {Cell::i(x.run), Cell::d(x.t_call_ms), Cell::d(x.t_ret_ms), Cell::i(x.grpc_code),
       x.grpc_code == 0 ? Cell::i(x.value) : Cell::null()});
  }
  return t.to_json();
}

Json names_json()
{
  Json j = Json::object();
  Json ph = Json::array();
  for (int i = 0; i <= kPreHold; ++i) {
    ph.push(phase_name(i));
  }
  j.set("phase", std::move(ph));
  Json p = Json::array();
  for (const auto & k : act::known_patterns()) {
    p.push(k.name);
  }
  j.set("pattern", std::move(p));
  return j;
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

Json options_json(const act::Options & o)
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
    .set("physics_hz_hint", o.physics_hz)
    .set("reader_hz", o.reader_hz)
    .set("deadline_ms", o.deadline_ms)
    .set("wft_deadline_ms", o.wft_deadline_ms)
    .set("max_inflight", o.max_inflight)
    .set("idle_probe_s", o.idle_probe_s)
    .set("idle_brake", o.idle_brake)
    .set("idle_brake_tol", o.idle_brake_tol)
    .set("idle_throttle_max", o.idle_throttle_max)
    .set("safe_stop_accel", Safety::kSafeStopAccel)
    .set("safe_stop_hz", Safety::kSafeStopHz)
    .set("fit_start_s", Safety::kFitStartS)
    .set("pre_drive_hold_s", Safety::kPreDriveHoldS)
    .set("restore_stop_after_failure_s", Safety::kRestoreStopAfterFailureS)
    .set("max_steer_echo", act::kSteerEchoTol)
    .set("max_stale_s", Safety::kMaxStaleS)
    .set("max_frozen_s", Safety::kMaxFrozenS);
}

}  // namespace

// =============================================================================

int main(int argc, char ** argv)
{
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  act::Options opt;
  std::string err;
  const std::vector<std::string> args =
    argc > 1 ? std::vector<std::string>(argv + 1, argv + argc) : std::vector<std::string>();
  if (
    !act::apply_env_defaults(std::getenv("MORAI_SIM_ADDRESS"), std::getenv("MORAI_SIM_PORT"), &opt, &err) ||
    !act::parse_args(args, &opt, &err)) {
    std::fprintf(stderr, "morai_actuate: %s\n\n", err.c_str());
    act::print_usage(stderr);
    return 1;
  }
  if (opt.help) {
    act::print_usage(stdout);
    return 0;
  }
  // 26.09.25 _ YSH _ --lockstep 이면 동기 모드 경로로 분기 (아래 실시간 경로는 그대로)
  if (opt.lockstep) {
    return act::run_lockstep(opt);
  }
  install_signal_handlers();

  Json out = Json::object()
               .set("tool", "morai_actuate")
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

  SendLog log;
  StateReader reader(opt, ego);
  Controller ctrl(opt, ego, &log, &reader);
  Session session(opt, ctrl, reader, log);

  std::printf(
    "morai_actuate -> %s:%d ego '%s' (MOVES THE EGO)\n", opt.host.c_str(), opt.port,
    opt.ego.c_str());
  if (!ctrl.connect(opt.connect_timeout_ms) || !reader.connect(opt.connect_timeout_ms)) {
    std::fprintf(
      stderr, "morai_actuate: cannot connect to %s:%d within %d ms\n", opt.host.c_str(), opt.port,
      opt.connect_timeout_ms);
    out.set("connected", false);
    return finish(2);
  }
  out.set("connected", true);
  reader.start();

  // ---- preflight (read-only) -----------------------------------------------------
  Preflight pf = session.preflight();
  out.set("preflight", pf.j);
  if (!pf.ok) {
    std::fprintf(stderr, "morai_actuate: preflight failed - %s. Nothing was changed.\n", pf.why.c_str());
    out.set("preflight_error", pf.why);
    reader.stop();
    out.set("raw", Json::object().set("samples", raw_samples(reader.all())));
    return finish(stop_requested() ? 5 : 2);
  }

  // ---- banner + countdown -------------------------------------------------------
  const double per_run =
    act::estimate_run_travel_m(opt.accel, opt.drive_s, Safety::kAssumedBrakeDecel);
  const std::vector<std::string> run_names = act::run_order(opt);
  const size_t n_runs = run_names.size();
  std::string order;
  for (const auto & p : opt.patterns) {
    order += (order.empty() ? "" : " ") + p;
  }
  order += opt.order == "abba" && opt.repeats > 1 ? " (odd repeats reversed)" : "";
  std::printf(
    "======================================================================\n"
    " morai_actuate - THIS MOVES THE EGO\n"
    "   target    %s:%d  ego '%s'  MORAI %s\n"
    "   now       mode %s, gear %s, at standstill (%.2f, %.2f)\n"
    "   will set  mode VEHICLE_CONTROL_AUTO_MODE, gear GEAR_MODE_D\n"
    "             (restored to %s / %s on every exit, read back)\n"
    "   runs      %zu = %zu patterns x %d, interleaved: %s\n"
    "   each run  brake hold %.1f s -> +%.2f m/s^2 for %.1f s MORAI time, steer 0 -> safe-stop\n"
    "             -> hold %.1f s\n"
    "   travel    ~%.1f m straight ahead per run, ~%.0f m total (budget %.0f m)\n"
    "   envelope  |v| <= %.2f m/s, <= %.1f m from run start, reads fail <= %.1f s, time frozen\n"
    "             <= %.1f s, no collision, no steer echo (nobody else commanding)\n"
    "   Ctrl-C    abort -> safe-stop (%.1f m/s^2 @ %.0f Hz) -> restore gear & mode\n"
    "             (also SIGTERM/SIGHUP/Ctrl-\\/Ctrl-Z; after kill -9 set KEYBOARD / P by hand)\n"
    "======================================================================\n",
    opt.host.c_str(), opt.port, opt.ego.c_str(), pf.version.empty() ? "?" : pf.version.c_str(),
    mode_name(pf.mode).c_str(), gear_name(pf.gear).c_str(), pf.x, pf.y, mode_name(pf.mode).c_str(),
    gear_name(pf.gear).c_str(), n_runs, opt.patterns.size(), opt.repeats, order.c_str(),
    Safety::kPreDriveHoldS, opt.accel, opt.drive_s, opt.settle_s, per_run,
    per_run * static_cast<double>(n_runs),
    opt.max_total_distance, opt.max_speed, opt.max_distance, Safety::kMaxStaleS,
    Safety::kMaxFrozenS, Safety::kSafeStopAccel, Safety::kSafeStopHz);
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
    reader.stop();
    return finish(5);
  }

  // ---- from here on every exit path restores -------------------------------------
  RestoreGuard guard(session, ctrl, pf.mode, pf.gear);
  guard.arm();
  int code = 0;
  std::string session_abort;
  bool budget_stop = false;
  act::IdleCalibration idle;
  std::vector<RunRecord> runs;
  Json setup_j = Json::object();
  try {
    std::string why;
    if (!session.setup(&setup_j, &idle, &why)) {
      session_abort = "setup: " + why;
      code = stop_requested() ? 5 : 4;
      std::fprintf(stderr, "morai_actuate: setup failed - %s\n", why.c_str());
    } else {
      const ReaderSnapshot s0 = reader.snapshot();
      const double ox = s0.latest.x;
      const double oy = s0.latest.y;
      std::map<std::string, int> repeat_of;
      for (size_t index = 0; index < run_names.size(); ++index) {
        const act::PatternSpec * p = act::find_pattern(run_names[index]);
        if (stop_requested()) {
          session_abort = signal_text();
          break;
        }
        const ReaderSnapshot s = reader.snapshot();
        const double travelled = std::hypot(s.latest.x - ox, s.latest.y - oy);
        if (travelled + per_run > opt.max_total_distance) {
          std::printf(
            "[info] travel budget: %.1f m from the start + ~%.1f m > --max-total-distance %.0f; "
            "stopping here\n",
            travelled, per_run, opt.max_total_distance);
          budget_stop = true;
          break;
        }
        RunRecord r = session.run_pattern(*p, repeat_of[p->name]++, static_cast<int>(index));
        const std::vector<StateSample> so_far = reader.all();
        const act::StepEstimate st = step_from(so_far, opt.physics_hz);
        r.analysis = analyze(r, so_far, log.copy(), st.step_us, idle.signature, opt);
        print_run_line(r);
        runs.push_back(r);
        if (!r.abort_reason.empty()) {
          session_abort = "run " + std::to_string(r.index) + " (" + r.pattern + "): " + r.abort_reason;
          break;
        }
        if (!r.stop_ok) {
          session_abort = "safe-stop failed: " + r.stop_error;
          break;
        }
        if (!r.brake_warning.empty()) {
          // 브레이크 명령이 먹지 않는데 다음 런을 출발시키지 않는다
          session_abort = "run " + std::to_string(r.index) + ": " + r.brake_warning;
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

  const Json restore = guard.restore();
  reader.stop();
  if (!guard.ok()) {
    code = 6;
  }

  // ---- analysis -------------------------------------------------------------------
  const std::vector<StateSample> samples = reader.all();
  const std::vector<SendRecord> sends = log.copy();
  const act::StepEstimate step = step_from(samples, opt.physics_hz);
  double update_rate = kNaN;
  if (samples.size() >= 2 && samples.back().local_ms > samples.front().local_ms) {
    update_rate = static_cast<double>(samples.size() - 1) /
                  ((samples.back().local_ms - samples.front().local_ms) / 1e3);
  }
  // 송신 귀속 지연: locked_half·free33 처럼 명령 없는 업데이트가 섞인 런에서 추정해 전 런에 적용
  std::vector<act::RunInput> inputs;
  for (const auto & r : runs) {
    inputs.push_back(make_input(r, samples, sends, step.step_us, idle.signature, opt));
  }
  const act::LagEstimate lag = act::estimate_lag(inputs);
  // WaitForTick 값 = ActorStates.timestamp 인지, 상태 읽기가 모든 업데이트를 봤는지
  std::vector<int64_t> wft_values;
  for (const auto & w : session.wft_log()) {
    if (w.grpc_code == 0) {
      wft_values.push_back(w.value);
    }
  }
  std::vector<int64_t> reader_ts;
  for (const auto & x : samples) {
    reader_ts.push_back(x.morai_us);
  }
  const act::WftMatch wft_match = act::match_wft_values(wft_values, reader_ts);

  Json runs_j = Json::array();
  std::map<std::string, std::vector<const act::RunAnalysis *>> by_pattern;
  for (size_t i = 0; i < runs.size(); ++i) {
    inputs[i].lag = lag.physics_lag;
    runs[i].analysis = act::analyze_run(inputs[i]);
    runs_j.push(run_json(runs[i], sends));
    by_pattern[runs[i].pattern].push_back(&runs[i].analysis);
  }
  std::vector<act::PatternSummary> summaries;
  for (const auto & name : opt.patterns) {
    const auto it = by_pattern.find(name);
    if (it != by_pattern.end()) {
      summaries.push_back(act::summarize_pattern(name, it->second));
    }
  }
  const act::Interpretation interp = act::interpret(summaries, idle.ambiguous);

  const ReaderSnapshot rs = reader.snapshot();
  std::printf(
    "\n==== summary: MORAI step %.3f ms (%.2f Hz, %s%s), updates %.2f/s, idle brake %.4f (%s%s) "
    "====\n",
    step.step_us / 1e3, 1e6 / step.step_us, step.method.c_str(), step.reliable ? "" : ", UNRELIABLE",
    update_rate, idle.signature.brake_center, idle.source.c_str(),
    idle.ambiguous ? ", AMBIGUOUS" : "");
  for (const auto & w : step.warnings) {
    std::printf("WARNING: %s\n", w.c_str());
  }
  if (update_rate > 1000.0) {
    std::printf(
      "WARNING: %.0f distinct timestamps/s - the timestamp seems to change on every poll, not "
      "per update; the per-update columns below are not meaningful\n",
      update_rate);
  }
  std::printf(
    "attribution: physics lag %d (%s), record lag %d (%s), from %zu partial-coverage runs; "
    "WaitForTick values found in the reader's timestamps: %s (%zu/%zu)\n",
    lag.physics_lag, lag.physics_decided ? "measured" : "assumed", lag.record_lag,
    lag.record_decided ? "measured" : "undecided", lag.runs_used,
    std::isfinite(wft_match.share) ? (f3(100.0 * wft_match.share) + "%").c_str() : "-",
    wft_match.found, wft_match.n);
  for (const auto & w : lag.warnings) {
    std::printf("WARNING: %s\n", w.c_str());
  }
  if (lag.physics_lag != 0) {
    std::printf(
      "(the [run] lines above used lag 0; the table below uses lag %d)\n", lag.physics_lag);
  }
  if (wft_match.n >= 20 && wft_match.share < 0.95) {
    std::printf(
      "WARNING: only %.1f%% of WaitForTick values equal an ActorStates.timestamp the reader saw: "
      "the clocks differ or the reader misses updates - locked-pattern attribution is suspect\n",
      100.0 * wft_match.share);
  }
  std::printf(
    "%-12s %-5s %-15s %7s %9s %9s %8s %-12s %14s %14s %6s\n", "pattern", "runs",
    "ratio mean+-sd", "idle%", "sends/upd", "steps/upd", "upd/step", "dv~k+-se",
    "a(k>=2)/a(k=1)", "a(0snd)/a(snd)", "stalls");
  Json pats_j = Json::array();
  for (const auto & p : summaries) {
    char runs_txt[16];
    std::snprintf(runs_txt, sizeof(runs_txt), "%zu/%zu", p.valid, p.runs);
    const std::string ratio = f3(p.ratio_mean) + "+-" + f3(p.ratio_sd);
    const std::string ks = f2(p.k_share) + "+-" + f2(p.k_share_se);
    std::printf(
      "%-12s %-5s %-15s %7s %9s %9s %8s %-12s %14s %14s %6zu\n", p.pattern.c_str(), runs_txt,
      ratio.c_str(), f3(100.0 * p.idle_mean).c_str(), f3(p.sends_per_update_mean).c_str(),
      f3(p.steps_per_update_mean).c_str(), f3(p.coverage_mean).c_str(), ks.c_str(),
      f3(p.multi_step_accel_ratio).c_str(), f3(p.zero_send_accel_ratio).c_str(), p.stalls);
    pats_j.push(act::to_json(p));
  }
  std::printf(
    "(dv~k: share of the per-update speed change that scales with the steps it advanced - 1 = "
    "per update, 0 = per step; +-1 SE. upd/step: updates / physics steps)\n");
  std::printf(
    "verdict: %s%s (hold: %s)\n", interp.verdict.c_str(),
    interp.verdict == "inconclusive" ? "" : (interp.confirmed ? ", confirmed" : ", physics only"),
    interp.hold.c_str());
  for (const auto & l : interp.lines) {
    std::printf("  %s\n", l.c_str());
  }
  if (!session_abort.empty()) {
    std::printf("session ABORTED: %s\n", session_abort.c_str());
  }
  std::printf(
    "restore: %s\n", guard.ok() ? "complete (mode and gear read back)" : "INCOMPLETE - check MORAI");

  out.set(
       "environment",
       Json::object()
         .set("simulator_version", pf.version)
         .set("original_mode", mode_name(pf.mode))
         .set("original_gear", gear_name(pf.gear))
         .set("original_position", Json::object().set("x", pf.x).set("y", pf.y))
         .set("physics_step", act::to_json(step))
         .set("update_rate_hz", update_rate)
         .set(
           "reader", Json::object()
                       .set("polls", rs.polls)
                       .set("distinct_updates", samples.size())
                       .set("failures", rs.failures)
                       .set("ego_missing", rs.ego_missing)
                       .set("last_error", rs.last_error))
         .set("idle_calibration", act::to_json(idle))
         .set("attribution_lag", act::to_json(lag))
         .set("wait_for_tick_match", act::to_json(wft_match))
         .set("record_signatures", record_signatures(samples)))
    .set("setup", setup_j)
    .set("runs", std::move(runs_j))
    .set(
      "summary",
      Json::object().set("patterns", std::move(pats_j)).set("interpretation", act::to_json(interp)))
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
    .set(
      "raw", Json::object()
               .set("samples", raw_samples(samples))
               .set("sends", raw_sends(sends))
               .set("wait_for_tick", raw_wft(session.wft_log())));
  return finish(code);
}
