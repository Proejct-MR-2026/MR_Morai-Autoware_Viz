// 신규 — 26.09.25 _ YSH _ 가짜 MORAI gRPC 서버 — C++ 엔진·노드 종단 시험 전용 (설치하지 않는다, 7789 거부)
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

// 흉내내는 것 (로컬 MORAI 실측, morai-time-facts):
//  - 업데이트 50 회/s (20 ms). ActorStates.timestamp = 시작 epoch µs + 20 ms × 틱. --skip-every N 이면 N 번째
//    업데이트마다 한 틱을 건너뛰어 timestamp 가 2 프레임(40 ms) 뛴다. 실제처럼 벽시계와 맞춰 간다
//  - WaitForTick: 업데이트마다 한 번 돌아온다. 값은 timestamp 와 다른 시계 (steady µs + 상수)
//  - GetAllActorsState: 움직이는 NPC 2 (차·보행자, 필터 준수) + 'Ego'(vehicle_state 포함) 한 번에
//  - Ego 동역학: 마지막 ControlVehicle 을 업데이트 사이에 유지해 적분한다. AUTO(3)·SYNCHRONOUS(7) + 기어 D/R
//    에서만 먹는다. 조향은 타이어각 라디안(±max 클램프), 자전거 모델(wheel_base 3.0)
//  - vehicle_state: front_wheel_angle = −조향/최대 (Unity 부호, 정규화), steer = 조향/최대
//  - angular_velocity: world rad/s, 축 (−z, x, −y) → yaw rate ω 는 raw.y = −ω (morai_coordinates.py)
//  - GetVehicleSpec (wheel_base 3.0, max_steering_angle 40 deg), Get/SetVehicleControlMode
//    (--lie-mode-result 면 Result 가 실패라고 거짓말 — 되읽기 확인 시험), SetVehicleGear, SetTransform
//    (30 ms 뒤 적용), SetVehicleTailLight, GetActorState(되읽기), GetSynchronousMode, GetTimestamp
//  - 모든 RPC 개수·마지막 시각, 업데이트당 ControlVehicle 개수 분포, JSONL 이벤트 로그(--events)
//  - 제어 포트(127.0.0.1, 한 줄 명령 → 한 줄 JSON): stats | freeze S | drop S | zero S | mode N | gear N | quit
//    freeze = 업데이트 멈춤(timestamp·상태 그대로, WaitForTick 안 돌아옴), drop = 서버를 닫고 S 초 뒤 같은
//    포트로 다시 연다(연결 끊김), zero = Ego 위치·자세·속도 전부 0 (daq_publisher ego_is_valid 무효)

#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/simulation/simulation.grpc.pb.h>
#include <morai/simulator/simulator.grpc.pb.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace
{

namespace A = morai_sim_api::actor;
using SteadyClock = std::chrono::steady_clock;

constexpr int kRealMoraiPort = 7789;
constexpr int64_t kTickUs = 20000;       // 50 업데이트/s
constexpr double kWheelBase = 3.0;       // [m]
constexpr double kMaxSteerDeg = 40.0;    // [deg]
constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxSteerRad = kMaxSteerDeg * kPi / 180.0;
constexpr int64_t kTransformDelayNs = 30LL * 1000000LL;  // SetTransform 은 조금 늦게 먹는다

int64_t steady_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(SteadyClock::now().time_since_epoch())
    .count();
}

int64_t system_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

std::string json_escape(const std::string & s)
{
  std::string out;
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

std::string num(double v)
{
  if (!std::isfinite(v)) {
    return "null";
  }
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.9g", v);
  return buf;
}

struct Options
{
  int port{17871};
  int control_port{0};  // 0 = port + 1
  std::string events_path;
  int skip_every{97};
  bool lie_mode_result{false};
  int initial_mode{A::VEHICLE_CONTROL_KEYBOARD};
};

struct Command
{
  bool present{false};
  int long_type{0};
  double throttle{0}, brake{0}, steer{0}, velocity{0}, acceleration{0};
};

struct PendingTransform
{
  int64_t apply_at_ns{0};
  double x{0}, y{0}, z{0}, yaw_deg{0};
};

/// 가짜 MORAI 의 모든 상태 (하나의 mutex)
struct World
{
  std::mutex m;
  std::condition_variable cv;

  // --- 시간 ---
  int64_t base_us{0};         // 틱 0 의 epoch µs
  int64_t start_steady_ns{0};
  int64_t tick{0};            // 20 ms 틱 (벽시계와 같이 간다)
  int64_t last_update_tick{0};
  uint64_t updates{0};        // 실제로 상태가 바뀐 업데이트 수
  int64_t ts_us{0};           // 마지막 업데이트의 timestamp
  int64_t freeze_until_ns{0};
  bool frozen{false};
  int64_t zero_until_ns{0};

  // --- Ego ---
  double x{100.0}, y{200.0}, z{0.3}, yaw_deg{90.0};
  double v{0.0};              // 종방향 [m/s] (후진이면 음수)
  double a{0.0};              // 적용한 가속 [m/s^2]
  double steer{0.0};          // 적용한 타이어각 [rad]
  double yaw_rate{0.0};       // [rad/s]
  int gear{A::GEAR_MODE_P};
  int mode{A::VEHICLE_CONTROL_KEYBOARD};
  int tail{0};
  Command cmd;
  PendingTransform pending;

  // --- NPC ---
  double npc_t{0.0};

  // --- 계수 ---
  std::map<std::string, uint64_t> rpc_count;
  std::map<std::string, int64_t> rpc_last_ns;
  uint64_t cmds_this_update{0};
  std::map<int, uint64_t> hist;   // 업데이트 사이 ControlVehicle 개수 → 업데이트 수
  uint64_t max_per_update{0};
  uint64_t nonfinite_cmds{0};
  std::set<std::string> peers;
  uint64_t transforms{0};
  double last_tf_x{0}, last_tf_y{0}, last_tf_z{0}, last_tf_yaw{0};
  uint64_t drops{0};
};

World W;
Options g_opt;
std::atomic<bool> g_quit{false};
std::atomic<uint64_t> g_generation{0};  // drop 때마다 올라간다 (막힌 WaitForTick 을 깨운다)

// --- 이벤트 로그 -----------------------------------------------------------------
std::mutex g_log_mutex;
FILE * g_log = nullptr;

void log_event(const std::string & body)
{
  if (g_log == nullptr) {
    return;
  }
  const std::lock_guard<std::mutex> lock(g_log_mutex);
  std::fprintf(g_log, "{\"t_ns\":%lld,%s}\n", static_cast<long long>(system_ns()), body.c_str());
}

void flush_log()
{
  if (g_log == nullptr) {
    return;
  }
  const std::lock_guard<std::mutex> lock(g_log_mutex);
  std::fflush(g_log);
}

/// W.m 을 잡은 채로 부른다
void count_rpc(const char * name, grpc::ServerContext * ctx)
{
  ++W.rpc_count[name];
  W.rpc_last_ns[name] = system_ns();
  if (ctx != nullptr) {
    W.peers.insert(ctx->peer());
  }
}

bool ego_zeroed(int64_t now)
{
  return now < W.zero_until_ns;
}

void set_vec(morai_sim_api::Vector3 * v, double x, double y, double z)
{
  v->set_x(x);
  v->set_y(y);
  v->set_z(z);
}

void fill_ego(A::ActorState * e, int64_t now)
{
  e->mutable_actor_info()->mutable_id()->set_value("Ego");
  e->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
  set_vec(e->mutable_bounding_box()->mutable_extent(), 1.06, 2.32, 0.75);
  auto * vs = e->mutable_vehicle_state();
  vs->set_gear_mode(static_cast<A::GearMode>(W.gear));
  if (ego_zeroed(now)) {
    // 미스폰·리셋: 위치·자세·속도 전부 0 (vehicle_state 는 남는다)
    return;
  }
  set_vec(e->mutable_transform()->mutable_location(), W.x, W.y, W.z);
  set_vec(e->mutable_transform()->mutable_rotation(), 0.0, 0.0, W.yaw_deg);
  set_vec(e->mutable_velocity(), W.v * 3.6, 0.0, 0.0);  // local, km/h
  set_vec(e->mutable_angular_velocity(), 0.0, -W.yaw_rate, 0.0);  // world rad/s, 축 (−z, x, −y)
  set_vec(e->mutable_acceleration(), W.a, 0.0, 0.0);
  const double yaw = W.yaw_deg * kPi / 180.0;
  set_vec(e->mutable_global_velocity(), W.v * std::cos(yaw) * 3.6, W.v * std::sin(yaw) * 3.6, 0.0);
  vs->set_throttle(static_cast<float>(W.a > 0.0 ? std::min(W.a / 3.0, 1.0) : 0.0));
  vs->set_brake(static_cast<float>(W.a < 0.0 ? std::min(-W.a / 4.5, 1.0) : 0.0));
  vs->set_steer(static_cast<float>(W.steer / kMaxSteerRad));
  vs->set_front_wheel_angle(static_cast<float>(-W.steer / kMaxSteerRad));  // Unity 부호
  // 26.10.07 _ YSH _ 충돌 상대 이름 (/daq/collision 시험): 1 s 마다 번갈아 — 홀수 초에는 NPC 차(actor)와 actor 가 아닌 벽
  if ((W.tick / 50) % 2 == 1) {
    vs->add_collision_objects("NPC_Car_1");
    vs->add_collision_objects("Wall_1");
  }
}

/// NPC: 차 1 대는 원을 돌고, 보행자 1 명은 y 로 오간다
void fill_npcs(A::ActorStates * r, const A::GetAllActorsFilter & filter)
{
  const double t = W.npc_t;
  if (filter.vehicle()) {
    auto * car = r->add_states();
    car->mutable_actor_info()->mutable_id()->set_value("NPC_Car_1");
    car->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
    const double radius = 15.0;
    const double speed = 5.0;
    const double w = speed / radius;
    const double th = w * t;
    set_vec(
      car->mutable_transform()->mutable_location(), 130.0 + radius * std::cos(th),
      200.0 + radius * std::sin(th), 0.3);
    set_vec(car->mutable_transform()->mutable_rotation(), 0.0, 0.0, (th + kPi / 2.0) * 180.0 / kPi);
    set_vec(car->mutable_velocity(), speed * 3.6, 0.0, 0.0);
    set_vec(car->mutable_angular_velocity(), 0.0, -w, 0.0);
    set_vec(car->mutable_bounding_box()->mutable_extent(), 1.06, 2.32, 0.75);
    car->mutable_vehicle_state()->set_gear_mode(A::GEAR_MODE_D);
  }
  if (filter.pedestrian()) {
    auto * ped = r->add_states();
    ped->mutable_actor_info()->mutable_id()->set_value("NPC_Ped_1");
    ped->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_PEDESTRIAN);
    const double y = 190.0 + 5.0 * std::sin(0.3 * t);
    const double vy = 5.0 * 0.3 * std::cos(0.3 * t);
    set_vec(ped->mutable_transform()->mutable_location(), 110.0, y, 0.0);
    set_vec(ped->mutable_transform()->mutable_rotation(), 0.0, 0.0, vy >= 0.0 ? 90.0 : -90.0);
    set_vec(ped->mutable_velocity(), std::abs(vy) * 3.6, 0.0, 0.0);
    set_vec(ped->mutable_bounding_box()->mutable_extent(), 0.3, 0.3, 0.9);
  }
}

// =============================================================================
// 업데이트 스레드
// =============================================================================

void integrate(double dt)
{
  const bool driving = (W.mode == A::VEHICLE_CONTROL_AUTO_MODE ||
                        W.mode == A::VEHICLE_CONTROL_SYNCHRONOUS_MODE) &&
                       (W.gear == A::GEAR_MODE_D || W.gear == A::GEAR_MODE_R) && W.cmd.present;
  double a = 0.0;
  if (driving) {
    switch (W.cmd.long_type) {
      case A::LONG_CMD_TYPE_ACCELERATION:
        a = W.cmd.acceleration;
        break;
      case A::LONG_CMD_TYPE_THROTTLE:
        a = 3.0 * W.cmd.throttle - 4.5 * W.cmd.brake;
        break;
      case A::LONG_CMD_TYPE_SPEED:
        a = std::max(-3.0, std::min(3.0, (W.cmd.velocity / 3.6 - std::abs(W.v)) / 0.5));
        break;
      default:
        a = 0.0;
        break;
    }
    W.steer = std::max(-kMaxSteerRad, std::min(kMaxSteerRad, W.cmd.steer));
  } else {
    a = std::abs(W.v) > 0.0 ? -1.0 : 0.0;  // 굴림 저항
  }
  const double dir = W.gear == A::GEAR_MODE_R ? -1.0 : 1.0;
  double speed = std::abs(W.v) + a * dt;
  speed = std::max(0.0, speed);  // 제동으로 뒤로 가지 않는다
  W.v = dir * speed;
  W.a = a;
  W.yaw_rate = W.v * std::tan(W.steer) / kWheelBase;
  W.yaw_deg += W.yaw_rate * dt * 180.0 / kPi;
  if (W.yaw_deg > 180.0) {
    W.yaw_deg -= 360.0;
  } else if (W.yaw_deg < -180.0) {
    W.yaw_deg += 360.0;
  }
  const double yaw = W.yaw_deg * kPi / 180.0;
  W.x += W.v * std::cos(yaw) * dt;
  W.y += W.v * std::sin(yaw) * dt;
}

void update_loop()
{
  auto next = SteadyClock::now();
  for (;;) {
    next += std::chrono::microseconds(kTickUs);
    std::this_thread::sleep_until(next);
    if (g_quit.load()) {
      break;
    }
    std::string event;
    {
      const std::lock_guard<std::mutex> lock(W.m);
      ++W.tick;
      const int64_t now = steady_ns();
      if (W.frozen) {
        if (now < W.freeze_until_ns) {
          continue;  // 멈춤: timestamp·상태 그대로, WaitForTick 도 돌아오지 않는다
        }
        W.frozen = false;
        W.last_update_tick = W.tick - 1;  // 멈춘 동안의 물리는 흐르지 않았다 (일시정지)
        log_event("\"ev\":\"resume\",\"tick\":" + std::to_string(W.tick));
      }
      // 가끔 한 틱 건너뛴다 → 다음 업데이트의 timestamp 가 2 프레임 뛴다 (렌더 프레임 누락)
      if (
        g_opt.skip_every > 0 && W.updates > 0 &&
        W.updates % static_cast<uint64_t>(g_opt.skip_every) == 0 && W.tick - W.last_update_tick == 1) {
        continue;
      }
      const double dt = static_cast<double>(W.tick - W.last_update_tick) * kTickUs / 1e6;
      W.last_update_tick = W.tick;
      if (W.pending.apply_at_ns > 0 && now >= W.pending.apply_at_ns) {
        W.x = W.pending.x;
        W.y = W.pending.y;
        W.z = W.pending.z;
        W.yaw_deg = W.pending.yaw_deg;
        W.v = 0.0;
        W.pending.apply_at_ns = 0;
      } else {
        integrate(dt);
      }
      W.npc_t += dt;
      W.ts_us = W.base_us + W.tick * kTickUs;
      const int bucket = static_cast<int>(std::min<uint64_t>(W.cmds_this_update, 3));
      if (W.updates > 0) {
        ++W.hist[bucket];
        W.max_per_update = std::max(W.max_per_update, W.cmds_this_update);
      }
      W.cmds_this_update = 0;
      ++W.updates;
      std::ostringstream os;
      os << "\"ev\":\"update\",\"u\":" << W.updates << ",\"ts\":" << W.ts_us
         << ",\"x\":" << num(W.x) << ",\"y\":" << num(W.y) << ",\"yaw\":" << num(W.yaw_deg)
         << ",\"v\":" << num(W.v) << ",\"a\":" << num(W.a) << ",\"steer\":" << num(W.steer)
         << ",\"mode\":" << W.mode << ",\"gear\":" << W.gear
         << ",\"zero\":" << (ego_zeroed(now) ? 1 : 0);
      event = os.str();
    }
    W.cv.notify_all();
    log_event(event);
  }
}

// =============================================================================
// 서비스
// =============================================================================

grpc::Status success(morai_sim_api::Result * r)
{
  r->set_status(morai_sim_api::STATUS_CODE_SUCCESS);
  return grpc::Status::OK;
}

class ActorService final : public A::Actor::Service
{
  grpc::Status GetAllActorsState(
    grpc::ServerContext * ctx, const A::GetAllActorsFilter * q, A::ActorStates * r) override
  {
    const std::lock_guard<std::mutex> lock(W.m);
    count_rpc("GetAllActorsState", ctx);
    r->set_timestamp(W.ts_us);
    r->set_sync_mode_tick_count(0);
    fill_npcs(r, *q);
    // Ego 는 목록 가운데 (첫 항목이라고 가정하지 않게)
    fill_ego(r->add_states(), steady_ns());
    if (r->states_size() >= 2) {
      r->mutable_states()->SwapElements(r->states_size() - 1, 1);
    }
    return grpc::Status::OK;
  }

  grpc::Status GetActorState(
    grpc::ServerContext * ctx, const morai_sim_api::ObjectInfo * q, A::ActorState * r) override
  {
    const std::lock_guard<std::mutex> lock(W.m);
    count_rpc("GetActorState", ctx);
    if (q->id().value() != "Ego") {
      return grpc::Status(grpc::StatusCode::UNKNOWN, "no such actor");
    }
    fill_ego(r, steady_ns());
    return grpc::Status::OK;
  }

  grpc::Status GetVehicleControlMode(
    grpc::ServerContext * ctx, const morai_sim_api::ObjectInfo *,
    A::CurVehicleControlMode * r) override
  {
    const std::lock_guard<std::mutex> lock(W.m);
    count_rpc("GetVehicleControlMode", ctx);
    r->set_mode(static_cast<A::VehicleControlMode>(W.mode));
    return grpc::Status::OK;
  }

  grpc::Status GetVehicleSpec(
    grpc::ServerContext * ctx, const morai_sim_api::ObjectInfo * q, A::VehicleSpec * r) override
  {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("GetVehicleSpec", ctx);
    }
    log_event("\"rpc\":\"GetVehicleSpec\",\"id\":\"" + json_escape(q->id().value()) + "\"");
    r->set_length(4.64F);
    r->set_width(2.12F);
    r->set_height(1.5F);
    r->set_wheel_base(static_cast<float>(kWheelBase));
    r->set_max_steering_angle(static_cast<float>(kMaxSteerDeg));
    return grpc::Status::OK;
  }

  grpc::Status ControlVehicle(
    grpc::ServerContext * ctx, const A::VehicleCtrlCmd * q, morai_sim_api::Result * r) override
  {
    std::ostringstream os;
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("ControlVehicle", ctx);
      ++W.cmds_this_update;
      const double f[] = {q->throttle(), q->brake(), q->steer(), q->velocity(), q->acceleration()};
      const bool finite = std::all_of(std::begin(f), std::end(f), [](double v) { return std::isfinite(v); });
      if (!finite) {
        ++W.nonfinite_cmds;
      } else {
        W.cmd.present = true;
        W.cmd.long_type = q->long_cmd_type();
        W.cmd.throttle = q->throttle();
        W.cmd.brake = q->brake();
        W.cmd.steer = q->steer();
        W.cmd.velocity = q->velocity();
        W.cmd.acceleration = q->acceleration();
      }
      os << "\"rpc\":\"ControlVehicle\",\"u\":" << W.updates << ",\"ts\":" << W.ts_us
         << ",\"type\":" << q->long_cmd_type() << ",\"accel\":" << num(q->acceleration())
         << ",\"steer\":" << num(q->steer()) << ",\"vel\":" << num(q->velocity())
         << ",\"thr\":" << num(q->throttle()) << ",\"brk\":" << num(q->brake())
         << ",\"finite\":" << (finite ? 1 : 0) << ",\"id\":\"" << json_escape(q->actor_info().id().value())
         << "\"";
    }
    log_event(os.str());
    return success(r);
  }

  grpc::Status SetVehicleGear(
    grpc::ServerContext * ctx, const A::VehicleGear * q, morai_sim_api::Result * r) override
  {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("SetVehicleGear", ctx);
      W.gear = q->gear();
    }
    log_event("\"rpc\":\"SetVehicleGear\",\"gear\":" + std::to_string(q->gear()));
    return success(r);
  }

  grpc::Status SetVehicleControlMode(
    grpc::ServerContext * ctx, const A::VehicleControlModeParam * q,
    morai_sim_api::Result * r) override
  {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("SetVehicleControlMode", ctx);
      W.mode = q->mode();
    }
    log_event(
      "\"rpc\":\"SetVehicleControlMode\",\"mode\":" + std::to_string(q->mode()) +
      ",\"result_lies\":" + (g_opt.lie_mode_result ? "1" : "0"));
    if (g_opt.lie_mode_result) {
      // 실제 MORAI 는 Result 가 거짓일 때가 있다 (SetSynchronousMode 실측) — 노드는 되읽기로 확인해야 한다
      r->set_status(morai_sim_api::STATUS_CODE_UNKNOWN_FAILURE);
      return grpc::Status::OK;
    }
    return success(r);
  }

  grpc::Status SetTransform(
    grpc::ServerContext * ctx, const A::SetTransformParam * q, morai_sim_api::Result * r) override
  {
    const auto & loc = q->transform().location();
    const auto & rot = q->transform().rotation();
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("SetTransform", ctx);
      W.pending.apply_at_ns = steady_ns() + kTransformDelayNs;
      W.pending.x = loc.x();
      W.pending.y = loc.y();
      W.pending.z = loc.z();
      W.pending.yaw_deg = rot.z();
      ++W.transforms;
      W.last_tf_x = loc.x();
      W.last_tf_y = loc.y();
      W.last_tf_z = loc.z();
      W.last_tf_yaw = rot.z();
    }
    std::ostringstream os;
    os << "\"rpc\":\"SetTransform\",\"x\":" << num(loc.x()) << ",\"y\":" << num(loc.y())
       << ",\"z\":" << num(loc.z()) << ",\"roll\":" << num(rot.x()) << ",\"pitch\":" << num(rot.y())
       << ",\"yaw\":" << num(rot.z()) << ",\"id\":\"" << json_escape(q->actor_info().id().value())
       << "\"";
    log_event(os.str());
    return success(r);
  }

  grpc::Status SetVehicleTailLight(
    grpc::ServerContext * ctx, const A::VehicleTailLight * q, morai_sim_api::Result * r) override
  {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      count_rpc("SetVehicleTailLight", ctx);
      W.tail = q->signal();
    }
    log_event("\"rpc\":\"SetVehicleTailLight\",\"signal\":" + std::to_string(q->signal()));
    return success(r);
  }
};

class SimulationService final : public morai_sim_api::simulation::Simulation::Service
{
  grpc::Status WaitForTick(
    grpc::ServerContext * ctx, const morai_sim_api::Empty *, morai_sim_api::Int64Value * o) override
  {
    const uint64_t generation = g_generation.load();
    std::unique_lock<std::mutex> lock(W.m);
    count_rpc("WaitForTick", ctx);
    const uint64_t u0 = W.updates;
    while (W.updates == u0) {
      if (ctx->IsCancelled() || g_quit.load() || g_generation.load() != generation) {
        return grpc::Status(grpc::StatusCode::CANCELLED, "cancelled");
      }
      W.cv.wait_for(lock, std::chrono::milliseconds(5));
    }
    // 다른 시계 (실측: WaitForTick 값은 ActorStates.timestamp 와 0% 일치) — 반복하지 않는다
    o->set_value(steady_ns() / 1000 + 7000000000000LL);
    return grpc::Status::OK;
  }

  grpc::Status GetSynchronousMode(
    grpc::ServerContext * ctx, const morai_sim_api::Empty *,
    morai_sim_api::simulation::SyncMode * r) override
  {
    const std::lock_guard<std::mutex> lock(W.m);
    count_rpc("GetSynchronousMode", ctx);
    r->set_tick_period(0);  // 실시간 모드
    return grpc::Status::OK;
  }
};

class SimulatorService final : public morai_sim_api::simualtor::Simulator::Service
{
  grpc::Status GetTimestamp(
    grpc::ServerContext * ctx, const morai_sim_api::Empty *, morai_sim_api::Int64Value * r) override
  {
    const std::lock_guard<std::mutex> lock(W.m);
    count_rpc("GetTimestamp", ctx);
    r->set_value(W.ts_us);  // 실측: 프레임 시각 (호출 시각이 아니다)
    return grpc::Status::OK;
  }
};

// =============================================================================
// 서버 수명 (drop = 닫고 S 초 뒤 다시 연다)
// =============================================================================

std::mutex g_srv_mutex;
std::condition_variable g_srv_cv;
double g_drop_s = 0.0;
bool g_serving = false;

std::unique_ptr<grpc::Server> build_server(
  ActorService * actor, SimulationService * simulation, SimulatorService * simulator)
{
  grpc::ServerBuilder b;
  const std::string addr = "127.0.0.1:" + std::to_string(g_opt.port);
  int bound = 0;
  b.AddListeningPort(addr, grpc::InsecureServerCredentials(), &bound);
  b.RegisterService(actor);
  b.RegisterService(simulation);
  b.RegisterService(simulator);
  auto server = b.BuildAndStart();
  if (!server || bound == 0) {
    return nullptr;
  }
  return server;
}

void serve_forever()
{
  for (;;) {
    ActorService actor;
    SimulationService simulation;
    SimulatorService simulator;
    std::unique_ptr<grpc::Server> server;
    for (int attempt = 0; attempt < 50 && !server; ++attempt) {
      server = build_server(&actor, &simulation, &simulator);
      if (!server) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
    }
    if (!server) {
      std::fprintf(stderr, "[fake] 127.0.0.1:%d 에 바인드하지 못했다\n", g_opt.port);
      g_quit = true;
      W.cv.notify_all();
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(g_srv_mutex);
      g_serving = true;
    }
    std::fprintf(stderr, "[fake] serving 127.0.0.1:%d\n", g_opt.port);
    log_event("\"ev\":\"serving\",\"port\":" + std::to_string(g_opt.port));
    double drop_s = 0.0;
    {
      std::unique_lock<std::mutex> lock(g_srv_mutex);
      g_srv_cv.wait(lock, [] { return g_quit.load() || g_drop_s > 0.0; });
      drop_s = g_drop_s;
      g_drop_s = 0.0;
      g_serving = false;
    }
    ++g_generation;  // 막힌 WaitForTick 을 깨운다
    W.cv.notify_all();
    server->Shutdown(std::chrono::system_clock::now() + std::chrono::milliseconds(100));
    server->Wait();
    server.reset();
    if (g_quit.load()) {
      return;
    }
    {
      const std::lock_guard<std::mutex> lock(W.m);
      ++W.drops;
      W.peers.clear();
    }
    std::fprintf(stderr, "[fake] dropped — reopening in %.2f s\n", drop_s);
    log_event("\"ev\":\"dropped\",\"reopen_s\":" + num(drop_s));
    const auto until = SteadyClock::now() + std::chrono::duration<double>(drop_s);
    while (SteadyClock::now() < until && !g_quit.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (g_quit.load()) {
      return;
    }
  }
}

// =============================================================================
// 제어 포트
// =============================================================================

std::string stats_json()
{
  std::ostringstream os;
  const std::lock_guard<std::mutex> lock(W.m);
  const int64_t now = steady_ns();
  bool serving = false;
  {
    const std::lock_guard<std::mutex> lk(g_srv_mutex);
    serving = g_serving;
  }
  os << "{\"ok\":true,\"t_ns\":" << system_ns() << ",\"base_us\":" << W.base_us
     << ",\"tick\":" << W.tick << ",\"updates\":" << W.updates << ",\"ts_us\":" << W.ts_us
     << ",\"frozen\":" << (W.frozen ? "true" : "false")
     << ",\"zero\":" << (ego_zeroed(now) ? "true" : "false")
     << ",\"serving\":" << (serving ? "true" : "false") << ",\"drops\":" << W.drops
     << ",\"ego\":{\"x\":" << num(W.x) << ",\"y\":" << num(W.y) << ",\"z\":" << num(W.z)
     << ",\"yaw_deg\":" << num(W.yaw_deg) << ",\"v\":" << num(W.v) << ",\"a\":" << num(W.a)
     << ",\"steer\":" << num(W.steer) << ",\"yaw_rate\":" << num(W.yaw_rate) << "}"
     << ",\"gear\":" << W.gear << ",\"mode\":" << W.mode << ",\"tail\":" << W.tail
     << ",\"cmd\":{\"present\":" << (W.cmd.present ? "true" : "false")
     << ",\"type\":" << W.cmd.long_type << ",\"accel\":" << num(W.cmd.acceleration)
     << ",\"steer\":" << num(W.cmd.steer) << ",\"vel\":" << num(W.cmd.velocity) << "}"
     << ",\"nonfinite_cmds\":" << W.nonfinite_cmds << ",\"max_per_update\":" << W.max_per_update
     << ",\"hist\":{\"0\":" << W.hist[0] << ",\"1\":" << W.hist[1] << ",\"2\":" << W.hist[2]
     << ",\"3+\":" << W.hist[3] << "},\"transforms\":" << W.transforms
     << ",\"last_transform\":{\"x\":" << num(W.last_tf_x) << ",\"y\":" << num(W.last_tf_y)
     << ",\"z\":" << num(W.last_tf_z) << ",\"yaw\":" << num(W.last_tf_yaw) << "}"
     << ",\"peers\":" << W.peers.size() << ",\"rpc\":{";
  bool first = true;
  for (const auto & kv : W.rpc_count) {
    os << (first ? "" : ",") << "\"" << kv.first << "\":{\"n\":" << kv.second
       << ",\"last_ns\":" << W.rpc_last_ns[kv.first] << "}";
    first = false;
  }
  os << "}}";
  return os.str();
}

std::string handle_command(const std::string & line)
{
  std::istringstream is(line);
  std::string cmd;
  is >> cmd;
  double arg = 0.0;
  const bool has_arg = static_cast<bool>(is >> arg);
  if (cmd == "stats") {
    flush_log();
    return stats_json();
  }
  if ((cmd == "freeze" || cmd == "drop" || cmd == "zero") && (!has_arg || !(arg > 0.0) || arg > 60.0)) {
    return "{\"ok\":false,\"error\":\"need seconds in (0, 60]\"}";
  }
  if (cmd == "freeze") {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      W.frozen = true;
      W.freeze_until_ns = steady_ns() + static_cast<int64_t>(arg * 1e9);
    }
    log_event("\"ev\":\"freeze\",\"s\":" + num(arg));
    return "{\"ok\":true,\"t_ns\":" + std::to_string(system_ns()) + "}";
  }
  if (cmd == "zero") {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      W.zero_until_ns = steady_ns() + static_cast<int64_t>(arg * 1e9);
    }
    log_event("\"ev\":\"zero\",\"s\":" + num(arg));
    return "{\"ok\":true,\"t_ns\":" + std::to_string(system_ns()) + "}";
  }
  if (cmd == "drop") {
    log_event("\"ev\":\"drop\",\"s\":" + num(arg));
    {
      const std::lock_guard<std::mutex> lock(g_srv_mutex);
      g_drop_s = arg;
    }
    g_srv_cv.notify_all();
    return "{\"ok\":true,\"t_ns\":" + std::to_string(system_ns()) + "}";
  }
  if ((cmd == "mode" || cmd == "gear") && has_arg) {
    {
      const std::lock_guard<std::mutex> lock(W.m);
      (cmd == "mode" ? W.mode : W.gear) = static_cast<int>(arg);
    }
    log_event("\"ev\":\"" + cmd + "\",\"value\":" + std::to_string(static_cast<int>(arg)));
    return "{\"ok\":true}";
  }
  if (cmd == "quit") {
    g_quit = true;
    g_srv_cv.notify_all();
    W.cv.notify_all();
    return "{\"ok\":true}";
  }
  return "{\"ok\":false,\"error\":\"unknown command\"}";
}

void control_loop(int listen_fd)
{
  while (!g_quit.load()) {
    const int fd = ::accept(listen_fd, nullptr, nullptr);
    if (fd < 0) {
      continue;
    }
    std::string buf;
    char chunk[512];
    for (;;) {
      const ssize_t n = ::read(fd, chunk, sizeof(chunk));
      if (n <= 0) {
        break;
      }
      buf.append(chunk, static_cast<size_t>(n));
      size_t pos = 0;
      while ((pos = buf.find('\n')) != std::string::npos) {
        const std::string line = buf.substr(0, pos);
        buf.erase(0, pos + 1);
        const std::string reply = handle_command(line) + "\n";
        if (::write(fd, reply.data(), reply.size()) < 0) {
          break;
        }
      }
      if (g_quit.load()) {
        break;
      }
    }
    ::close(fd);
  }
}

int open_control_port(int port)
{
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    return -1;
  }
  const int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<uint16_t>(port));
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0 || ::listen(fd, 4) != 0) {
    ::close(fd);
    return -1;
  }
  return fd;
}

void on_signal(int)
{
  g_quit = true;
}

int usage()
{
  std::fprintf(
    stderr,
    "usage: fake_morai_server [--port N (17871)] [--control-port N (port+1)] [--events PATH]\n"
    "                         [--skip-every N (97, 0=off)] [--lie-mode-result] [--initial-mode N (1)]\n"
    "  port 7789 (real MORAI) is refused.\n");
  return 2;
}

}  // namespace

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next_int = [&](int * out) {
      if (i + 1 >= argc) {
        return false;
      }
      char * end = nullptr;
      const long v = std::strtol(argv[++i], &end, 10);  // NOLINT(runtime/int)
      if (end == nullptr || *end != '\0') {
        return false;
      }
      *out = static_cast<int>(v);
      return true;
    };
    if (a == "--port") {
      if (!next_int(&g_opt.port)) {
        return usage();
      }
    } else if (a == "--control-port") {
      if (!next_int(&g_opt.control_port)) {
        return usage();
      }
    } else if (a == "--skip-every") {
      if (!next_int(&g_opt.skip_every)) {
        return usage();
      }
    } else if (a == "--initial-mode") {
      if (!next_int(&g_opt.initial_mode)) {
        return usage();
      }
    } else if (a == "--events" && i + 1 < argc) {
      g_opt.events_path = argv[++i];
    } else if (a == "--lie-mode-result") {
      g_opt.lie_mode_result = true;
    } else {
      return usage();
    }
  }
  if (g_opt.control_port == 0) {
    g_opt.control_port = g_opt.port + 1;
  }
  if (g_opt.port == kRealMoraiPort || g_opt.control_port == kRealMoraiPort) {
    std::fprintf(stderr, "[fake] refusing port %d (real MORAI)\n", kRealMoraiPort);
    return 2;
  }
  if (g_opt.port < 1 || g_opt.port > 65535 || g_opt.control_port < 1 || g_opt.control_port > 65535) {
    return usage();
  }
  if (!g_opt.events_path.empty()) {
    g_log = std::fopen(g_opt.events_path.c_str(), "w");
    if (g_log == nullptr) {
      std::fprintf(stderr, "[fake] cannot open %s\n", g_opt.events_path.c_str());
      return 3;
    }
  }
  std::signal(SIGINT, on_signal);
  std::signal(SIGTERM, on_signal);
  std::signal(SIGPIPE, SIG_IGN);

  const int control_fd = open_control_port(g_opt.control_port);
  if (control_fd < 0) {
    std::fprintf(stderr, "[fake] cannot bind control port %d\n", g_opt.control_port);
    return 3;
  }
  {
    const std::lock_guard<std::mutex> lock(W.m);
    W.mode = g_opt.initial_mode;
    W.start_steady_ns = steady_ns();
    W.base_us = (system_ns() / 1000000) * 1000;  // ms 로 맞춘 epoch µs
    W.ts_us = W.base_us;
  }
  std::fprintf(
    stderr, "[fake] grpc 127.0.0.1:%d control 127.0.0.1:%d skip_every %d lie_mode_result %d\n",
    g_opt.port, g_opt.control_port, g_opt.skip_every, g_opt.lie_mode_result ? 1 : 0);

  std::thread updater(update_loop);
  std::thread control(control_loop, control_fd);
  std::thread flusher([] {
    while (!g_quit.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      flush_log();
    }
  });
  std::thread quit_watch([] {
    while (!g_quit.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    g_srv_cv.notify_all();
    W.cv.notify_all();
  });

  serve_forever();

  g_quit = true;
  W.cv.notify_all();
  ::shutdown(control_fd, SHUT_RDWR);
  ::close(control_fd);
  updater.join();
  flusher.join();
  quit_watch.join();
  control.detach();  // accept 에 막혀 있을 수 있다 — 프로세스 종료로 정리
  flush_log();
  if (g_log != nullptr) {
    std::fclose(g_log);
  }
  std::fprintf(stderr, "[fake] bye\n");
  return 0;
}
