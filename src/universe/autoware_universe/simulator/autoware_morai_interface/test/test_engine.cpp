// 신규 — 26.09.25 _ YSH _ 엔진 루프 시험 (가짜 링크·가짜 시계, 네트워크·ROS executor 없음)
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

// 고정하는 것: 업데이트당 ControlVehicle 정확히 1 회 · 중복/무효 표본은 송신·발행 없음 · stale 감속(조향 유지) ·
// NaN 메시지는 버리고 마지막 유효 명령 유지 · forward 차단 · WaitForTick 폴백 · 링크 끊김→재연결→스펙 재조회 ·
// AUTO 재지정 · 발행 주기(기어·모드 2 회에 1 번, 객체 5 회에 1 번) · stamp 출처 · 시계 오프셋 · 종료 감속.
// 리뷰 반영: timestamp 0·멈춤(내용 판정) · 역행 빠른 새 기준 · 막지 않는 WaitForTick · 폴백 재확인 · 낡은 모드 보고 생략 ·
// 재연결 뒤 첫 조회로 AUTO 재지정 · Ego 복귀 스펙 재조회(재연결 아님) · 왕복 맞춤 deadline · 가드 차단 순간 감속.
// 26.09.25 _ YSH _ command_trigger=timer (main.py _CmdResender 조건): 업데이트율과 무관한 고정 주기 · 엔진 루프는 송신 안 함 ·
// 연결·스펙·첫 표본 전 무송신 · 링크 끊김에 헛돌지 않음 · stale 감속 · NaN · 정체 뒤 몰아 보내지 않음 · forward·가드 ·
// 런타임 전환 · 종료 감속 주기 · 조향 보정 속도.

#include "autoware/morai_interface/command_slot.hpp"
#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"
#include "autoware/morai_interface/engine.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>

// 26.09.25 _ YSH _ atomic·mutex·thread: 실제 스레드 명령 타이머 시험
#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{
namespace
{

namespace mgv = morai_grpc_vendor;

constexpr int64_t kMs = 1000000LL;
constexpr int64_t kEpochNs = 1758000000000000000LL;  // system_clock 기준점 (가짜)
constexpr int32_t kUnimplemented = 12;
constexpr int32_t kDeadline = 4;
constexpr int32_t kUnavailable = 14;
constexpr int32_t kGrpcUnknownForTest = 2;

mgv::Status ok_status(double rpc_ms = 0.2)
{
  mgv::Status s;
  s.ok = true;
  s.grpc_code = 0;
  s.result_status = 1;
  s.rpc_ms = rpc_ms;
  return s;
}

mgv::Status fail_status(int32_t code, const std::string & msg = "fail")
{
  mgv::Status s;
  s.ok = false;
  s.grpc_code = code;
  s.message = msg;
  s.rpc_ms = code == kDeadline ? 100.0 : 0.1;
  return s;
}

/// 스크립트로 움직이는 가짜 MORAI. wait_for_tick 한 번 = MORAI 업데이트 한 번 (시계 +20 ms)
class FakeLink : public ILink
{
public:
  explicit FakeLink(int64_t * now_ns) : now_ns_(now_ns) {}

  // --- 스크립트 ---
  std::deque<mgv::Status> wft_script;       ///< 비면 성공
  std::deque<std::pair<mgv::Status, mgv::Snapshot>> snapshots;  ///< 비면 UNAVAILABLE
  bool connected{true};
  mgv::Reply<mgv::VehicleSpecInfo> spec;
  mgv::Reply<mgv::ControlMode> mode;
  mgv::Status control_status = ok_status();
  mgv::Status set_mode_status = ok_status();
  int64_t tick_advance_ns{20 * kMs};  ///< 0 이면 막지 않고 바로 돌아오는 WaitForTick
  int64_t tick_value{0};              ///< WaitForTick 값 (호출마다 1 씩 — 실측: 반복 없음)

  // 26.09.25 _ YSH _ ControlVehicle 한 번이 걸리는 (가짜) 시간 — 앞에서부터 하나씩 쓴다 (비면 0). 명령 타이머 정체 시험용
  std::deque<int64_t> control_delays_ns;

  // --- 기록 ---
  std::vector<mgv::VehicleCommand> sent;
  // 26.09.25 _ YSH _ ControlVehicle 호출 시각 (가짜 시계) — 명령 타이머 주기 시험용
  std::vector<int64_t> sent_ns;
  std::vector<int64_t> sent_deadlines_ms;
  std::vector<int64_t> wft_deadlines_ms;
  std::vector<mgv::ControlMode> mode_sets;
  int spec_calls{0};
  int recreate_calls{0};
  int wft_calls{0};
  int snapshot_calls{0};
  int mode_queries{0};
  // 26.09.26 _ YSH _ get_control_mode 가 돌아오기 전에 부른다 (조회 도중 다른 스레드가 하는 일 흉내)
  std::function<void()> on_mode_query;

  mgv::Status wait_connected(std::chrono::milliseconds) override
  {
    return connected ? ok_status() : fail_status(kUnavailable);
  }
  mgv::Reply<int64_t> wait_for_tick(std::chrono::milliseconds deadline) override
  {
    ++wft_calls;
    wft_deadlines_ms.push_back(deadline.count());
    mgv::Reply<int64_t> r;
    if (!wft_script.empty()) {
      r.status = wft_script.front();
      wft_script.pop_front();
    } else {
      r.status = ok_status();
    }
    // 타임아웃이면 deadline 만큼, 아니면 다음 업데이트까지 기다린 셈
    *now_ns_ += r.status.deadline_exceeded() ? deadline.count() * kMs : tick_advance_ns;
    r.value = ++tick_value;
    return r;
  }
  mgv::Status get_snapshot(mgv::Snapshot * out) override
  {
    ++snapshot_calls;
    if (snapshots.empty()) {
      return fail_status(kUnavailable, "no more snapshots");
    }
    auto item = snapshots.front();
    snapshots.pop_front();
    if (item.first.ok) {
      *out = item.second;
    }
    return item.first;
  }
  mgv::Reply<mgv::ControlMode> get_control_mode() override
  {
    ++mode_queries;
    // 26.09.26 _ YSH _ 조회가 MORAI 를 기다리는 동안 다른 스레드가 하는 일을 흉내 낸다 (시험이 넣는다). 답은 그 일보다
    //   먼저 정해졌다 — MORAI 가 답한 뒤 다른 스레드가 모드를 바꾸고, 낡은 답이 나중에 도착하는 경우
    const auto reply = mode;
    if (on_mode_query) {
      on_mode_query();
    }
    return reply;
  }
  mgv::Reply<mgv::VehicleSpecInfo> get_vehicle_spec() override
  {
    ++spec_calls;
    return spec;
  }
  mgv::Status control_vehicle(const mgv::VehicleCommand & cmd, std::chrono::milliseconds deadline) override
  {
    sent.push_back(cmd);
    sent_deadlines_ms.push_back(deadline.count());
    // 26.09.25 _ YSH _ 호출 시각 기록 + 스크립트된 소요 시간만큼 시계를 민다
    sent_ns.push_back(*now_ns_);
    if (!control_delays_ns.empty()) {
      *now_ns_ += control_delays_ns.front();
      control_delays_ns.pop_front();
    }
    return control_status;
  }
  mgv::Status set_gear(mgv::GearMode, std::chrono::milliseconds) override { return ok_status(); }
  mgv::Status set_control_mode(mgv::ControlMode m, std::chrono::milliseconds) override
  {
    mode_sets.push_back(m);
    if (set_mode_status.ok) {
      mode.value = m;
    }
    return set_mode_status;
  }
  mgv::Status set_transform(const mgv::TransformPose &) override { return ok_status(); }
  mgv::Status set_tail_light(mgv::TurnSignal) override { return ok_status(); }
  void recreate() override { ++recreate_calls; }
  mgv::LinkCounters counters() const override { return {}; }
  std::string target() const override { return "fake:0"; }

private:
  int64_t * now_ns_;
};

class FakeSink : public EngineSink
{
public:
  std::vector<UpdateOutputs> outputs;
  std::vector<bool> connects;
  // 26.09.26 _ YSH _ DAQ 훅 기록 (update_index, stamp)
  std::vector<std::pair<uint64_t, builtin_interfaces::msg::Time>> samples;
  void publish(const UpdateOutputs & o) override { outputs.push_back(o); }
  void on_connected(bool reconnected) override { connects.push_back(reconnected); }
  // 26.11.10 _ YSH _ 외란 시험: DAQ 로 넘어가는 스냅샷도 적는다 (외란이 스냅샷을 고치지 않는지 본다)
  std::vector<mgv::Snapshot> sample_snapshots;
  void on_sample(
    // 원문: const mgv::Snapshot &, const builtin_interfaces::msg::Time & stamp, uint64_t update_index,
    const mgv::Snapshot & snapshot, const builtin_interfaces::msg::Time & stamp, uint64_t update_index,
    int64_t) override
  {
    samples.emplace_back(update_index, stamp);
    sample_snapshots.push_back(snapshot);  // 26.11.10 _ YSH _
  }
  // 26.09.26 _ YSH _ 원시 gRPC 기록 훅 기록 (보낸 명령·stale·RPC 직전 system_clock·결과)
  struct SentRecord
  {
    mgv::VehicleCommand cmd;
    bool stale{false};
    int64_t send_system_ns{0};
    mgv::Status status;
  };
  std::vector<SentRecord> sent;
  void on_command_sent(
    const mgv::VehicleCommand & cmd, bool stale, int64_t send_system_ns, const mgv::Status & status) override
  {
    sent.push_back(SentRecord{cmd, stale, send_system_ns, status});
  }
};

/// 유효한 Ego 가 있는 스냅샷
mgv::Snapshot make_snapshot(int64_t ts_us, double vx_kmh = 0.0, double fwa = 0.0)
{
  mgv::Snapshot s;
  s.timestamp_us = ts_us;
  s.ego.found = true;
  s.ego.exact_match = true;
  s.ego.valid = true;
  s.ego.id = "Ego";
  s.ego.position = mgv::Vec3{100.0, 200.0, 1.0};
  s.ego.rotation_deg = mgv::Vec3{0.0, 0.0, 45.0};
  s.ego.velocity_kmh = mgv::Vec3{vx_kmh, 0.0, 0.0};
  s.ego.vehicle_state.present = true;
  s.ego.vehicle_state.front_wheel_angle = fwa;
  s.ego.vehicle_state.gear_mode = mgv::GearMode::kD;
  // 수신 시각 = MORAI 시각 + 3 ms (같은 PC 에서 프레임 나이 3 ms 인 셈)
  s.receive_system_ns = ts_us * 1000 + 3 * kMs;
  mgv::ActorSample car;
  car.id = "Car_1";
  car.object_type = mgv::ObjectType::kVehicle;
  car.position = mgv::Vec3{110.0, 200.0, 1.0};
  car.bbox_extent = mgv::Vec3{1.0, 2.0, 0.7};
  s.actors.push_back(car);
  return s;
}

autoware_control_msgs::msg::Control make_control(double steer, double accel, double velocity = 0.0)
{
  autoware_control_msgs::msg::Control c;
  c.lateral.steering_tire_angle = static_cast<float>(steer);
  c.longitudinal.acceleration = static_cast<float>(accel);
  c.longitudinal.velocity = static_cast<float>(velocity);
  return c;
}

struct Rig
{
  int64_t now_ns{1000 * kMs};
  FakeLink link{&now_ns};
  CommandSlot slot;
  RuntimeParamsHandle params;
  StateCache cache;
  FakeSink sink;
  EngineConfig config;
  // 26.11.10 _ YSH _ 외란 손잡이 (with_disturbance 일 때만 엔진에 넘긴다. 엔진보다 먼저 선언 = 더 오래 산다)
  DisturbanceHandle disturbance;
  std::unique_ptr<Engine> engine;

  // 26.11.10 _ YSH _ 셋째 인자 with_disturbance (기본 false = 예전과 같은 엔진)
  // 원문: explicit Rig(const EngineConfig & cfg = EngineConfig{}, bool comp = false) : config(cfg)
  explicit Rig(const EngineConfig & cfg = EngineConfig{}, bool comp = false, bool with_disturbance = false)
  : config(cfg)
  {
    link.spec.status = ok_status();
    link.spec.value.max_steering_angle_deg = 40.0;
    link.spec.value.wheel_base_m = 3.0;
    link.mode.status = ok_status();
    link.mode.value = mgv::ControlMode::kAuto;
    RuntimeParams rp;
    rp.command.steer_comp.enable = comp;
    params.set(rp);
    EngineClock clock;
    clock.steady_ns = [this]() { return now_ns; };
    clock.system_ns = [this]() { return kEpochNs + now_ns; };
    clock.sleep_ns = [this](int64_t ns) { now_ns += ns; };
    engine = std::make_unique<Engine>(
      // 26.11.10 _ YSH _ 외란 손잡이 (끄면 nullptr)
      // 원문: link, slot, params, cache, sink, config, rclcpp::get_logger("test_engine"), clock);
      link, slot, params, cache, sink, config, rclcpp::get_logger("test_engine"), clock,
      with_disturbance ? &disturbance : nullptr);
  }

  /// 연결 단계를 끝낸다 (스펙·모드 조회)
  void connect()
  {
    engine->step();
    ASSERT_EQ(engine->phase(), Engine::Phase::kRunning);
  }

  void push(const mgv::Snapshot & s) { link.snapshots.emplace_back(ok_status(0.3), s); }

  void submit(const autoware_control_msgs::msg::Control & c)
  {
    ASSERT_EQ(submit_control(slot, c, params.get()->forward_control_to_sim, now_ns), SubmitResult::kAccepted);
  }

  void set_params(const std::function<void(RuntimeParams &)> & f)
  {
    RuntimeParams rp = *params.get();
    f(rp);
    params.set(rp);
  }

  // 26.09.25 _ YSH _ 명령 트리거 시험 도우미
  void set_trigger(CommandTrigger trigger, double hz = 100.0)
  {
    set_params([trigger, hz](RuntimeParams & p) {
      p.command_trigger = trigger;
      p.cmd_resend_hz = hz;
    });
  }

  int64_t ts_us{1000000};  ///< drive() 가 넣는 다음 MORAI timestamp

  /// 26.09.25 _ YSH _ 한 스레드에서 두 트리거를 번갈아 돌린다: MORAI 업데이트는 20 ms 마다 on_update(50/s),
  /// control_cmd 는 30 ms 마다(Autoware ~33 Hz, submit_every_ns 0 이면 안 넣는다), 그 사이는 timer_step
  /// (deadline 까지 가짜 시계를 민다). 반환: timer_step 호출 수
  int drive(int64_t duration_ns, int64_t submit_every_ns = 30 * kMs, double vx_kmh = 0.0)
  {
    const int64_t end = now_ns + duration_ns;
    int64_t next_update = now_ns;
    int64_t next_cmd = now_ns;
    int timer_calls = 0;
    while (now_ns < end) {
      if (submit_every_ns > 0 && now_ns >= next_cmd) {
        submit(make_control(0.1, 0.5));
        next_cmd += submit_every_ns;
      }
      if (now_ns >= next_update) {
        engine->on_update(make_snapshot(ts_us, vx_kmh), true);
        ts_us += 20000;
        next_update += 20 * kMs;
      }
      engine->timer_step();
      ++timer_calls;
    }
    return timer_calls;
  }

  /// 26.09.25 _ YSH _ 업데이트 없이 명령 타이머만 duration 동안 (반환: timer_step 호출 수)
  int run_timer(int64_t duration_ns)
  {
    const int64_t end = now_ns + duration_ns;
    int calls = 0;
    while (now_ns < end) {
      engine->timer_step();
      ++calls;
    }
    return calls;
  }
};

// ============================================================================
// 연결
// ============================================================================

TEST(Engine, ConnectsFetchesSpecThenRuns)
{
  Rig rig;
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  rig.connect();
  ASSERT_TRUE(rig.engine->spec().has_value());
  EXPECT_NEAR(rig.engine->spec()->max_steering_angle, 40.0 * M_PI / 180.0, 1e-12);
  EXPECT_EQ(rig.link.spec_calls, 1);
  EXPECT_EQ(rig.sink.connects, (std::vector<bool>{false}));
  EXPECT_EQ(rig.engine->control_mode(), mgv::ControlMode::kAuto);
}

TEST(Engine, WaitsForSpecWithoutDying)
{
  Rig rig;
  rig.link.spec.status = fail_status(2, "Exception was thrown by handler");  // UNKNOWN: Ego 없음
  rig.engine->step();
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  rig.link.spec.status = ok_status();
  for (int i = 0; i < 200 && rig.engine->phase() != Engine::Phase::kRunning; ++i) {
    rig.engine->step();
  }
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
}

// ============================================================================
// 업데이트당 정확히 한 번
// ============================================================================

TEST(Engine, SendsExactlyOneCommandPerNewUpdate)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.push(make_snapshot(1000000));
  rig.push(make_snapshot(1020000));
  rig.push(make_snapshot(1020000));  // 업데이트 전 — 같은 timestamp
  rig.push(make_snapshot(1040000));
  for (int i = 0; i < 4; ++i) {
    rig.engine->step();
  }
  EXPECT_EQ(rig.link.sent.size(), 3U);
  EXPECT_EQ(rig.sink.outputs.size(), 3U);
  EXPECT_EQ(rig.engine->stats().duplicates, 1U);
  EXPECT_EQ(rig.engine->stats().updates, 3U);
  EXPECT_EQ(rig.link.wft_calls, 4);
}

TEST(Engine, NoCommandBeforeTheFirstControlMessage)
{
  Rig rig;
  rig.connect();
  rig.push(make_snapshot(1000000));
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  rig.engine->step();
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_EQ(rig.sink.outputs.size(), 2U) << "status is still published";
}

TEST(Engine, InvalidEgoIsNeitherPublishedNorSent)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  auto bad = make_snapshot(1000000);
  bad.ego.valid = false;  // 전 필드 0 (미스폰·리셋)
  rig.push(bad);
  rig.engine->step();
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_TRUE(rig.sink.outputs.empty());
  EXPECT_EQ(rig.engine->stats().invalid_ego, 1U);
  EXPECT_EQ(rig.cache.latest(), nullptr);
}

TEST(Engine, SteerIsSentInRadiansAndBuiltOncePerMessage)
{
  Rig rig(EngineConfig{}, true);  // 조향 결손 보정 켬
  rig.connect();
  rig.submit(make_control(0.2, 0.5, 10.0));
  rig.push(make_snapshot(1000000, 12.6));  // 3.5 m/s → gain 0.8625
  rig.push(make_snapshot(1020000, 36.0));  // 10 m/s — 같은 메시지면 다시 만들지 않는다
  rig.engine->step();
  rig.engine->step();
  ASSERT_EQ(rig.link.sent.size(), 2U);
  EXPECT_NEAR(rig.link.sent[0].steer, static_cast<double>(0.2F) / 0.8625, 1e-12);
  EXPECT_DOUBLE_EQ(rig.link.sent[1].steer, rig.link.sent[0].steer);
  EXPECT_EQ(rig.link.sent[0].long_cmd_type, mgv::LongCmdType::kAcceleration);
  EXPECT_DOUBLE_EQ(rig.link.sent[0].acceleration, static_cast<double>(0.5F));
  EXPECT_DOUBLE_EQ(rig.link.sent[0].velocity, 36.0);  // 10 m/s × 3.6 (velocity_cmd_unit kph)

  rig.submit(make_control(0.2, 0.5, 10.0));  // 새 메시지 → 지금 속도(10 m/s)로 다시 만든다
  rig.push(make_snapshot(1040000, 36.0));
  rig.engine->step();
  ASSERT_EQ(rig.link.sent.size(), 3U);
  const double gain_10 = 0.807 + (10.0 - 8.0) / 3.0 * (0.796 - 0.807);
  EXPECT_NEAR(rig.link.sent[2].steer, static_cast<double>(0.2F) / gain_10, 1e-9);
}

TEST(Engine, StaleCommandBrakesAndKeepsLastSteer)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.15, 1.0, 5.0));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  rig.now_ns += 250 * kMs;  // Autoware 가 250 ms 조용하다 (> 0.2 s)
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  ASSERT_EQ(rig.link.sent.size(), 2U);
  const auto & stale = rig.link.sent[1];
  EXPECT_DOUBLE_EQ(stale.steer, rig.link.sent[0].steer);
  EXPECT_EQ(stale.long_cmd_type, mgv::LongCmdType::kAcceleration);
  EXPECT_DOUBLE_EQ(stale.acceleration, -2.4);
  EXPECT_DOUBLE_EQ(stale.velocity, 0.0);
  EXPECT_EQ(rig.engine->stats().stale_sends, 1U);
}

TEST(Engine, NonFiniteMessageIsDroppedAndLastValidCommandKept)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  const auto nan = make_control(std::nan(""), 0.5);
  EXPECT_EQ(submit_control(rig.slot, nan, true, rig.now_ns), SubmitResult::kRejectedNonFinite);
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_EQ(rig.link.sent.size(), 1U);
  EXPECT_NEAR(rig.link.sent[0].steer, 0.1, 1e-6);
  EXPECT_TRUE(std::isfinite(rig.link.sent[0].steer));
}

TEST(Engine, ForwardingOffClearsTheSlotAndSendsNothing)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.set_params([](RuntimeParams & p) { p.forward_control_to_sim = false; });
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_TRUE(rig.slot.empty());
  EXPECT_EQ(rig.sink.outputs.size(), 1U) << "observation continues";

  // 다시 켜도 새 control_cmd 가 올 때까지 보내지 않는다 (main.py 와 같음)
  rig.set_params([](RuntimeParams & p) { p.forward_control_to_sim = true; });
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_EQ(
    submit_control(rig.slot, make_control(0.1, 0.5), false, rig.now_ns), SubmitResult::kForwardingOff);
}

TEST(Engine, DuplicateWriterGuardBlocksCommands)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.engine->set_command_block(true);
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_EQ(rig.engine->stats().blocked_sends, 1U);
  rig.engine->set_command_block(false);
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_EQ(rig.link.sent.size(), 1U);
}

TEST(Engine, CountsSendOutcomes)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.link.control_status = fail_status(kDeadline);
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  rig.link.control_status = fail_status(0);  // 전송 OK, Result 실패
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  const auto s = rig.engine->stats();
  EXPECT_EQ(s.sends, 2U);
  EXPECT_EQ(s.send_timeouts, 1U);
  EXPECT_EQ(s.send_result_failures, 1U);
  EXPECT_EQ(s.send_ok, 0U);
}

// ============================================================================
// 발행
// ============================================================================

TEST(Engine, PublishCadence)
{
  Rig rig;
  rig.connect();
  for (int i = 0; i < 10; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * i));
  }
  for (int i = 0; i < 10; ++i) {
    rig.engine->step();
  }
  ASSERT_EQ(rig.sink.outputs.size(), 10U);
  int gear = 0;
  int mode = 0;
  int objects = 0;
  int gt = 0;
  for (const auto & o : rig.sink.outputs) {
    gear += o.gear ? 1 : 0;
    mode += o.control_mode ? 1 : 0;
    objects += o.objects ? 1 : 0;
    gt += o.has_ground_truth ? 1 : 0;
  }
  // 26.09.26 _ YSH _ CARLA·simple_planning_simulator 처럼 차량 리포트는 매 업데이트, objects 는 MORAI 시각 100 ms 마다,
  //   제어 모드 조회는 엔진 루프 밖 (연결 때 1 회뿐). 원문:
  // EXPECT_EQ(gear, 5);     // 2 업데이트에 1 번 (~25 Hz)
  // EXPECT_EQ(mode, 5);
  // EXPECT_EQ(objects, 2);  // 5 업데이트에 1 번 (~10 Hz)
  // EXPECT_EQ(gt, 10);      // kinematic_state·TF 는 매 업데이트
  // EXPECT_EQ(rig.link.mode_queries, 1 + 2);  // 연결 때 1 + 5 업데이트마다 1
  EXPECT_EQ(gear, 10);    // 매 업데이트
  EXPECT_EQ(mode, 10);
  EXPECT_EQ(objects, 2);  // 0 ms, 100 ms (20 ms 표본 10 개 = 180 ms)
  EXPECT_TRUE(rig.sink.outputs[0].objects.has_value());
  EXPECT_TRUE(rig.sink.outputs[5].objects.has_value());
  EXPECT_EQ(gt, 10);      // kinematic_state·TF 는 매 업데이트
  EXPECT_EQ(rig.link.mode_queries, 1) << "the engine loop must never block on GetVehicleControlMode";
}

// 26.09.26 _ YSH _ DAQ 훅: 받아들인(새·유효) 표본마다 정확히 한 번, 발행과 같은 stamp, 중복·역행·무효는 없음
TEST(Engine, OnSampleFollowsAcceptedUpdates)
{
  Rig rig;
  rig.connect();
  rig.push(make_snapshot(1000000));
  rig.push(make_snapshot(1000000));  // 중복 timestamp
  rig.push(make_snapshot(1020000));
  auto invalid = make_snapshot(1040000);
  invalid.ego.valid = false;
  rig.push(invalid);
  rig.push(make_snapshot(1060000));
  for (int i = 0; i < 5; ++i) {
    rig.engine->step();
  }
  ASSERT_EQ(rig.sink.outputs.size(), 3U);
  ASSERT_EQ(rig.sink.samples.size(), 3U);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(rig.sink.samples[i].second, rig.sink.outputs[i].stamp) << i;
    if (i > 0) {
      EXPECT_GT(rig.sink.samples[i].first, rig.sink.samples[i - 1].first) << "update_index is monotonic";
    }
  }
}

// 26.09.26 _ YSH _ MORAI 업데이트는 20 ms 가 기본이지만 1 ms 쌍·40 ms 구멍이 섞인다 (실측 20, 1, 19, 40 ms 꼴).
//   objects 는 업데이트 개수가 아니라 MORAI 시각으로 10 Hz 여야 한다 — 몰아 내지도, 주기가 밀리지도 않게
/// 26.09.26 _ YSH _ 표본 시각열을 한 Rig 에 넣고 objects 를 낸 표본 시각을 돌려준다
std::vector<int64_t> objects_stamps_for(const std::vector<int64_t> & stamps_us)
{
  Rig rig;
  rig.connect();
  for (const int64_t ts : stamps_us) {
    rig.push(make_snapshot(ts));
    rig.engine->step();
  }
  EXPECT_EQ(rig.sink.outputs.size(), stamps_us.size());
  std::vector<int64_t> out;
  for (size_t i = 0; i < rig.sink.outputs.size() && i < stamps_us.size(); ++i) {
    if (rig.sink.outputs[i].objects) {
      out.push_back(stamps_us[i]);
    }
  }
  return out;
}

std::vector<int64_t> stamps_from_steps(const std::vector<int64_t> & steps_us, size_t n)
{
  std::vector<int64_t> out;
  int64_t ts = 1000000;
  for (size_t i = 0; i < n; ++i) {
    out.push_back(ts);
    ts += steps_us[i % steps_us.size()];
  }
  return out;
}

// 26.09.26 _ YSH _ 리뷰 반영: 평균 업데이트율이 다른 여러 경우 — 업데이트 개수로 세던 옛 규칙(5 번에 1 번)은 10 ms(20 Hz)·
//   40 ms(5 Hz)·16.7 ms(12 Hz) 에서 틀리므로 MORAI 시각 스케줄만 전부 통과한다. 무작위 흔들림도 넣는다
TEST(Engine, ObjectsFollowMoraiTimeAtTenHz)
{
  std::mt19937 rng(7);
  const std::vector<int64_t> jitter_set = {20000, 1000, 19000, 40000, 21000, 18000};
  std::vector<int64_t> jitter;
  for (int i = 0; i < 2000; ++i) {
    jitter.push_back(jitter_set[rng() % jitter_set.size()]);
  }
  struct Case
  {
    const char * name;
    std::vector<int64_t> stamps;
    int64_t max_step_us;
  };
  const Case cases[] = {
    {"measured MORAI 20/1/19/40", stamps_from_steps({20000, 1000, 19000, 40000}, 400), 40000},
    {"10 ms (lockstep targetFPS 100)", stamps_from_steps({10000}, 800), 10000},
    {"40 ms (25 updates/s)", stamps_from_steps({40000}, 200), 40000},
    {"16.667 ms (60 updates/s)", stamps_from_steps({16667}, 480), 16667},
    {"random jitter", stamps_from_steps(jitter, 2000), 40000},
  };
  for (const auto & c : cases) {
    SCOPED_TRACE(c.name);
    const auto pub = objects_stamps_for(c.stamps);
    ASSERT_GE(pub.size(), 2U);
    const double span_s = static_cast<double>(c.stamps.back() - c.stamps.front()) / 1e6;
    EXPECT_NEAR(static_cast<double>(pub.size() - 1) / span_s, 10.0, 0.3);
    for (size_t i = 1; i < pub.size(); ++i) {
      const int64_t gap = pub[i] - pub[i - 1];
      EXPECT_GE(gap, Engine::kObjectsPeriodNs / 1000 - 2 * Engine::kObjectsSlackNs / 1000)
        << "never two object messages closer than 80 ms";
      EXPECT_LE(gap, Engine::kObjectsPeriodNs / 1000 + c.max_step_us) << "no long holes";
    }
  }
}

// 26.09.26 _ YSH _ 리뷰 반영: 한 번의 MORAI 정체(70 ms) 뒤 10 ms 표본 — 늦은 표본이 격자를 그대로 두면 40 ms 간격 쌍이 났다
TEST(Engine, ObjectsDoNotBurstAfterAStall)
{
  for (const int64_t hole_us : {55000, 60000, 70000}) {
    SCOPED_TRACE(hole_us);
    std::vector<int64_t> stamps = {1000000, 1020000, 1040000, 1060000, 1080000};
    int64_t ts = 1080000 + hole_us;
    for (int i = 0; i < 200; ++i) {
      stamps.push_back(ts);
      ts += 10000;
    }
    const auto pub = objects_stamps_for(stamps);
    ASSERT_GE(pub.size(), 3U);
    for (size_t i = 1; i < pub.size(); ++i) {
      EXPECT_GE(pub[i] - pub[i - 1], 80000);
    }
  }
}

// 26.09.26 _ YSH _ 리뷰 반영: 시각이 뒤로 재기준되면 objects 는 재기준 표본부터 다시 (옛 예정 시각까지 기다리지 않는다)
TEST(Engine, ObjectsRestartOnTheRebasedSample)
{
  Rig rig;
  rig.connect();
  // 1000, 1020, 1040 ms 다음 150 ms 뒤로 — 역행 두 번은 건너뛰고 세 번째가 새 기준
  const int64_t stamps[] = {1000000, 1020000, 1040000, 890000, 910000, 930000, 950000};
  for (const int64_t ts : stamps) {
    rig.push(make_snapshot(ts));
    rig.engine->step();
  }
  ASSERT_GE(rig.engine->stats().rebased, 1U);
  bool any_after_rebase = false;
  for (const auto & o : rig.sink.outputs) {
    const int64_t st_us =
      static_cast<int64_t>(o.stamp.sec) * 1000000 + static_cast<int64_t>(o.stamp.nanosec / 1000);
    if (st_us < 1000000 && o.objects) {
      any_after_rebase = true;
    }
  }
  EXPECT_TRUE(any_after_rebase) << "objects must restart on the rebased timeline, not wait ~0.3 s";
}

// 26.09.26 _ YSH _ 조회 도중 끊김(엔진 스레드가 모드를 지움)이 끼면 그 조회 결과는 버린다 — 끊기기 전 AUTO 가 되살아나면
//   재연결 뒤 AUTO 재지정을 건너뛸 수 있다
TEST(Engine, ModeQueryResultIsDroppedWhenTheLinkDropsDuringTheQuery)
{
  Rig rig;
  rig.connect();
  ASSERT_EQ(rig.engine->control_mode(), mgv::ControlMode::kAuto);
  bool fired = false;
  rig.link.on_mode_query = [&rig, &fired]() {
    if (fired) {
      return;
    }
    fired = true;
    rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
    rig.engine->step();  // 조회가 MORAI 를 기다리는 동안 엔진 스레드가 끊김을 본다
  };
  rig.engine->mode_query_step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  EXPECT_FALSE(rig.engine->control_mode().has_value()) << "stale pre-disconnect AUTO must not come back";
  EXPECT_EQ(rig.engine->stats().mode_query_failures, 0U) << "a dropped result is not a failure";
}

// 26.09.26 _ YSH _ 리뷰 반영: 연결 단계에서는 조회 스레드가 조회하지 않는다 (재연결 중 조회가 끼면 끊기기 전 값이 남는다)
TEST(Engine, ModeQueryThreadIsIdleWhileConnecting)
{
  Rig rig;
  rig.connect();
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();  // 끊김
  ASSERT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  const int queries = rig.link.mode_queries;
  rig.engine->mode_query_step();
  EXPECT_EQ(rig.link.mode_queries, queries) << "no GetVehicleControlMode while connecting";
  EXPECT_FALSE(rig.engine->control_mode().has_value());
}

// 26.09.26 _ YSH _ 리뷰 반영: 조회 도중 엔진 스레드가 AUTO 를 재지정하면(설정·되읽기 확인), 그 전에 MORAI 가 답한 KEYBOARD 는
//   버린다 — 안 버리면 ControlModeReport 가 AUTONOMOUS→MANUAL→AUTONOMOUS 로 흔들린다 (Autoware 는 MANUAL 을 개입으로 본다)
TEST(Engine, ModeQueryResultIsDroppedWhenAutoIsReassertedDuringTheQuery)
{
  Rig rig;
  rig.connect();
  rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();                                 // 끊김
  rig.link.mode.value = mgv::ControlMode::kKeyboard;  // MORAI 재기동
  rig.link.mode.status = fail_status(kDeadline);      // 재연결 순간 조회 실패 → 재지정 대기
  rig.engine->step();                                 // 재연결
  ASSERT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  ASSERT_TRUE(rig.link.mode_sets.empty());
  rig.link.mode.status = ok_status();
  rig.engine->mode_query_step();  // KEYBOARD 저장, seq +1
  ASSERT_EQ(rig.engine->control_mode(), mgv::ControlMode::kKeyboard);
  bool fired = false;
  rig.link.on_mode_query = [&rig, &fired]() {
    if (fired) {
      return;
    }
    fired = true;
    rig.push(make_snapshot(1000000));
    rig.engine->step();  // 새 seq → evaluate_reassert → AUTO 설정 → overwrite_control_mode
  };
  const uint64_t failures_before = rig.engine->stats().mode_query_failures;  // 재연결 순간 실패 1 회는 시험이 넣은 것
  rig.engine->mode_query_step();  // 설정 전에 답한 KEYBOARD — 버려야 한다
  ASSERT_EQ(rig.link.mode_sets.size(), 1U);
  EXPECT_EQ(rig.link.mode_sets[0], mgv::ControlMode::kAuto);
  EXPECT_EQ(rig.engine->control_mode(), mgv::ControlMode::kAuto)
    << "a stale KEYBOARD must not overwrite the confirmed AUTO";
  EXPECT_EQ(rig.engine->stats().mode_query_failures, failures_before) << "a dropped result is not a failure";
}

TEST(Engine, OutputsCarryMoraiSampleTimeAndConsistentPose)
{
  Rig rig;
  rig.connect();
  rig.push(make_snapshot(1758000000123456LL));
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  const auto & o = rig.sink.outputs[0];
  EXPECT_EQ(o.stamp.sec, 1758000000);
  EXPECT_EQ(o.stamp.nanosec, 123456000U);
  EXPECT_EQ(o.velocity.header.stamp, o.stamp);
  EXPECT_EQ(o.steering.stamp, o.stamp);
  EXPECT_EQ(o.odometry.header.stamp, o.stamp);
  EXPECT_EQ(o.transform.header.stamp, o.stamp);
  EXPECT_DOUBLE_EQ(o.transform.transform.translation.x, o.odometry.pose.pose.position.x);
  ASSERT_TRUE(o.objects.has_value());
  EXPECT_EQ(o.objects->header.stamp, o.stamp);
  EXPECT_EQ(o.objects->header.frame_id, "map");
  ASSERT_EQ(o.objects->objects.size(), 1U);
  EXPECT_EQ(o.objects->objects[0].object_id, morai_to_autoware::object_id_from_name("Car_1"));
  EXPECT_FALSE(o.clock.has_value()) << "publish_clock defaults to false";
}

TEST(Engine, ReceiveTimeSourceAndClockTopic)
{
  EngineConfig cfg;
  cfg.time_source = TimeSource::kReceive;
  cfg.publish_clock = true;
  Rig rig(cfg);
  rig.connect();
  const auto snap = make_snapshot(1758000000000000LL);
  rig.push(snap);
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  const auto & o = rig.sink.outputs[0];
  EXPECT_EQ(o.stamp, morai_to_autoware::to_time_msg(snap.receive_system_ns));
  ASSERT_TRUE(o.clock.has_value());
  // 리뷰 반영: /clock 은 메시지 stamp 와 같다 (use_sim_time 에서 stamp 와 /clock 이 어긋나지 않게)
  EXPECT_EQ(o.clock->clock, o.stamp);
}

TEST(Engine, ClockTopicIsMoraiTimeByDefault)
{
  EngineConfig cfg;
  cfg.publish_clock = true;
  Rig rig(cfg);
  rig.connect();
  const auto snap = make_snapshot(1758000000000000LL);
  rig.push(snap);
  rig.engine->step();
  ASSERT_TRUE(rig.sink.outputs.at(0).clock.has_value());
  EXPECT_EQ(rig.sink.outputs[0].clock->clock, morai_to_autoware::to_time_msg(snap.timestamp_us * 1000));
}

TEST(Engine, GroundTruthCanBeDisabled)
{
  EngineConfig cfg;
  cfg.publish_ground_truth = false;
  Rig rig(cfg);
  rig.connect();
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[0].has_ground_truth);
  EXPECT_FALSE(rig.sink.outputs[0].objects.has_value());
}

TEST(Engine, SteeringReportUndoesTheGainOfTheLastCommand)
{
  Rig rig(EngineConfig{}, true);
  rig.connect();
  rig.submit(make_control(0.2, 0.0));
  rig.push(make_snapshot(1000000, 18.0, -0.25));  // 5 m/s → gain 0.855
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  const double max_rad = 40.0 / (180.0 / M_PI);
  EXPECT_FLOAT_EQ(
    rig.sink.outputs[0].steering.steering_tire_angle, static_cast<float>(0.25 * max_rad * 0.855));
}

TEST(Engine, SteeringZeroDropoutIsHeld)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.2, 0.0));
  rig.push(make_snapshot(1000000, 18.0, -0.2));
  rig.push(make_snapshot(1020000, 18.0, 0.0));  // 유휴 레코드
  rig.engine->step();
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 2U);
  EXPECT_FLOAT_EQ(
    rig.sink.outputs[1].steering.steering_tire_angle,
    rig.sink.outputs[0].steering.steering_tire_angle);
}

// 26.10.01 _ YSH _ 외부 조향 필터 — 엔진을 통과한 조향 리포트로 확인 (끔 = 종전 값, 켬 = 직전 우리 값)
namespace
{
/// fwa 와 수신 시각을 정해 넣는다 (수신 시각은 시험 시계의 system 축 — 필터는 그 이전 300 ms 의 송신과 비교한다)
mgv::Snapshot make_steer_snapshot(int64_t ts_us, double fwa, int64_t receive_system_ns)
{
  mgv::Snapshot s = make_snapshot(ts_us, 18.0, fwa);
  s.receive_system_ns = receive_system_ns;
  return s;
}
}  // namespace

TEST(Engine, ForeignSteerFilterOffReportsTheMoraiValue)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.2, 0.0));
  rig.push(make_steer_snapshot(1000000, 0.0, kEpochNs + rig.now_ns));
  rig.engine->step();
  ASSERT_FALSE(rig.link.sent.empty());
  const double norm_rad = 40.0 * M_PI / 180.0;
  const double own_fwa = -rig.link.sent.back().steer / norm_rad;
  rig.now_ns += 20 * kMs;
  rig.push(make_steer_snapshot(1020000, own_fwa, kEpochNs + rig.now_ns));
  rig.engine->step();
  rig.now_ns += 20 * kMs;
  rig.push(make_steer_snapshot(1040000, 0.9, kEpochNs + rig.now_ns));  // 우리가 보낸 적 없는 값
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 3U);
  EXPECT_FLOAT_EQ(rig.sink.outputs[2].steering.steering_tire_angle, static_cast<float>(-0.9 * norm_rad))
    << "filter off (default): the foreign value is reported as before";
}

TEST(Engine, ForeignSteerFilterOnHoldsTheLastOwnValue)
{
  Rig rig;
  RuntimeParams rp;
  rp.command.steer_comp.enable = false;
  rp.steer_report_ignore_foreign = true;
  rig.params.set(rp);
  rig.connect();
  rig.submit(make_control(0.2, 0.0));
  rig.push(make_steer_snapshot(1000000, 0.0, kEpochNs + rig.now_ns));
  rig.engine->step();
  ASSERT_FALSE(rig.link.sent.empty());
  const double norm_rad = 40.0 * M_PI / 180.0;
  const double own_fwa = -rig.link.sent.back().steer / norm_rad;
  rig.now_ns += 20 * kMs;
  rig.push(make_steer_snapshot(1020000, own_fwa, kEpochNs + rig.now_ns));
  rig.engine->step();
  rig.now_ns += 20 * kMs;
  rig.push(make_steer_snapshot(1040000, 0.9, kEpochNs + rig.now_ns));  // 우리가 보낸 적 없는 값
  rig.engine->step();
  rig.now_ns += 20 * kMs;
  rig.push(make_steer_snapshot(1060000, own_fwa, kEpochNs + rig.now_ns));
  rig.engine->step();
  ASSERT_EQ(rig.sink.outputs.size(), 4U);
  const float own_report = rig.sink.outputs[1].steering.steering_tire_angle;
  EXPECT_NEAR(own_report, 0.2F, 1e-5F) << "our own value passes through unchanged";
  EXPECT_FLOAT_EQ(rig.sink.outputs[2].steering.steering_tire_angle, own_report)
    << "foreign frame: report holds the last own value";
  EXPECT_FLOAT_EQ(rig.sink.outputs[3].steering.steering_tire_angle, own_report);
}

TEST(Engine, CruiseModeIsReportedDisengaged)
{
  Rig rig;
  rig.link.mode.value = mgv::ControlMode::kCruise;
  rig.connect();
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_TRUE(rig.sink.outputs[0].control_mode.has_value());
  EXPECT_EQ(
    rig.sink.outputs[0].control_mode->mode, autoware_vehicle_msgs::msg::ControlModeReport::DISENGAGED);
}

TEST(Engine, ClockOffsetIsMoraiMinusReceive)
{
  Rig rig;
  rig.connect();
  for (int i = 0; i < 5; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * i));
    rig.engine->step();
  }
  const auto off = rig.engine->clock_offset_ms();
  EXPECT_EQ(off.count, 5U);
  EXPECT_NEAR(off.p50, -3.0, 1e-9);  // 수신이 MORAI 시각보다 3 ms 늦다
  const auto interval = rig.engine->update_interval_ms();
  EXPECT_NEAR(interval.p50, 20.0, 1e-9);
}

TEST(Engine, SampleAgeTracksTheLastAcceptedUpdate)
{
  Rig rig;
  rig.connect();
  EXPECT_FALSE(rig.engine->sample_age_s().has_value());
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  EXPECT_NEAR(*rig.engine->sample_age_s(), 0.0, 1e-9);
  rig.now_ns += 150 * kMs;
  EXPECT_NEAR(*rig.engine->sample_age_s(), 0.15, 1e-9);
}

// ============================================================================
// 트리거 폴백
// ============================================================================

TEST(Engine, UnimplementedWaitForTickFallsBackToPollingAndStillSendsOncePerTimestamp)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.link.wft_script.push_back(fail_status(kUnimplemented));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  EXPECT_TRUE(rig.engine->fallback_active());
  const int wft_before = rig.link.wft_calls;
  rig.push(make_snapshot(1000000));  // 100 Hz 폴링은 같은 업데이트를 두 번 본다
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  rig.engine->step();
  EXPECT_EQ(rig.link.wft_calls, wft_before) << "no WaitForTick while polling (probe every 10 s)";
  EXPECT_EQ(rig.link.sent.size(), 2U) << "one send per distinct MORAI timestamp";
}

// 26.09.25 _ YSH _ e2e 반영: deadline 초과는 업데이트가 왔는데 알리지 못했을 때만 실패로 센다 — 스냅샷이 새 업데이트
//   (timestamp 가 늘어난다)를 보이게 바꿨다. 원래는 같은 timestamp(멈춘 MORAI)로 폴백을 기대했다:
// TEST(Engine, RepeatedWaitForTickFailuresFallBack)
// {
//   ...
//   for (int i = 0; i < 3; ++i) {
//     rig.link.wft_script.push_back(fail_status(kDeadline));
//     rig.push(make_snapshot(1000000));  // MORAI 가 멈춰 있다 (동기 모드 등)
//   }
//   ...
// }
TEST(Engine, RepeatedWaitForTickFailuresFallBack)
{
  EngineConfig cfg;
  cfg.wft_fail_limit = 3;
  Rig rig(cfg);
  rig.connect();
  for (int i = 0; i < 3; ++i) {
    rig.link.wft_script.push_back(fail_status(kDeadline));
    rig.push(make_snapshot(1000000 + i * 20000));  // 업데이트는 오는데 WaitForTick 이 알리지 못한다
  }
  rig.engine->step();
  rig.engine->step();
  EXPECT_FALSE(rig.engine->fallback_active());
  rig.engine->step();
  EXPECT_TRUE(rig.engine->fallback_active());
}

// 26.09.25 _ YSH _ e2e 반영: 가짜 MORAI 1 s 멈춤에서 WaitForTick deadline 10 번으로 폴백해 10 s 동안 트리거를 잃었다
TEST(Engine, WaitForTickTimeoutsWhileMoraiIsPausedDoNotFallBack)
{
  EngineConfig cfg;
  cfg.wft_fail_limit = 3;
  Rig rig(cfg);
  rig.connect();
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  for (int i = 0; i < 20; ++i) {
    rig.link.wft_script.push_back(fail_status(kDeadline));
    rig.push(make_snapshot(1000000));  // 일시정지·정체: timestamp 그대로 → WaitForTick 은 맞게 기다렸다
    rig.engine->step();
  }
  EXPECT_FALSE(rig.engine->fallback_active());
  EXPECT_EQ(rig.engine->stats().wft_failures, 20U) << "still counted for diagnostics";
  // 다시 돌면 업데이트 트리거 그대로
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_FALSE(rig.engine->fallback_active());
  EXPECT_EQ(rig.engine->stats().updates, 2U);
}

TEST(Engine, FallbackReturnsToWaitForTickWhenItWorksAgain)
{
  Rig rig;
  rig.connect();
  rig.link.wft_script.push_back(fail_status(kUnimplemented));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_TRUE(rig.engine->fallback_active());
  rig.now_ns += 11LL * 1000 * kMs;  // 10 s 뒤 재확인
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_FALSE(rig.engine->fallback_active());
}

// ============================================================================
// 링크 끊김 · 재연결 · AUTO 재지정
// ============================================================================

TEST(Engine, LinkLossReconnectsAndRefetchesSpec)
{
  Rig rig;
  rig.connect();
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  EXPECT_EQ(rig.engine->stats().link_losses, 1U);
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  EXPECT_EQ(rig.link.spec_calls, 2) << "spec is re-fetched after every reconnect";
  EXPECT_EQ(rig.engine->stats().reconnects, 1U);
  EXPECT_EQ(rig.sink.connects, (std::vector<bool>{false, true}));
}

TEST(Engine, SilentDeadlinesRecreateTheChannel)
{
  EngineConfig cfg;
  cfg.link_fail_limit = 3;
  Rig rig(cfg);
  rig.connect();
  for (int i = 0; i < 3; ++i) {
    rig.link.snapshots.emplace_back(fail_status(kDeadline), mgv::Snapshot{});
  }
  rig.engine->step();
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  rig.engine->step();
  EXPECT_EQ(rig.link.recreate_calls, 1);
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
}

TEST(Engine, ReassertsAutoAfterReconnectWhenLastRequestWasAuto)
{
  Rig rig;
  rig.link.mode.value = mgv::ControlMode::kKeyboard;
  rig.connect();
  EXPECT_TRUE(rig.link.mode_sets.empty()) << "first connect never changes the mode by default";
  rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();  // 끊김 (MORAI 재기동 → KEYBOARD)
  rig.engine->step();  // 재연결
  ASSERT_EQ(rig.link.mode_sets.size(), 1U);
  EXPECT_EQ(rig.link.mode_sets[0], mgv::ControlMode::kAuto);
  EXPECT_EQ(rig.engine->control_mode(), mgv::ControlMode::kAuto);
}

// 26.09.25 _ YSH _ e2e 반영: 가짜 MORAI 서버 재기동 뒤 명령 채널이 옛 연결로 READY 에 남아 재지정이 'Socket closed' 로
//   한 번 실패하고 영영 다시 하지 않았다 → UNAVAILABLE 끊김은 채널을 새로 만들고, 전송 실패한 재지정은 다시 한다
TEST(Engine, UnavailableLinkLossRecreatesTheChannels)
{
  Rig rig;
  rig.connect();
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();
  rig.engine->step();
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  EXPECT_EQ(rig.link.recreate_calls, 1);
  EXPECT_EQ(rig.engine->stats().link_recreates, 1U);
}

TEST(Engine, ReassertIsRetriedWhenTheRpcNeverReachedMorai)
{
  Rig rig;
  rig.link.mode.value = mgv::ControlMode::kKeyboard;
  rig.connect();
  rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
  rig.link.set_mode_status = fail_status(kUnavailable, "Socket closed");
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();  // 끊김
  rig.engine->step();  // 재연결 → 재지정 1 회 (전송 실패)
  ASSERT_EQ(rig.link.mode_sets.size(), 1U);
  EXPECT_EQ(rig.engine->control_mode(), mgv::ControlMode::kKeyboard);
  rig.link.set_mode_status = ok_status();
  rig.push(make_snapshot(1000000));
  // 26.09.26 _ YSH _ 조회는 전용 스레드 — 조회가 없으면 업데이트가 와도 다시 판단하지 않는다
  rig.engine->step();
  EXPECT_EQ(rig.link.mode_sets.size(), 1U) << "retry waits for the next mode query";
  rig.engine->mode_query_step();
  rig.push(make_snapshot(1020000));
  rig.engine->step();  // 다음 모드 조회 → 다시 판단
  ASSERT_EQ(rig.link.mode_sets.size(), 2U);
  EXPECT_EQ(rig.link.mode_sets[1], mgv::ControlMode::kAuto);
  EXPECT_EQ(rig.engine->control_mode(), mgv::ControlMode::kAuto);
}

TEST(Engine, ReassertRetriesAreBoundedAndReadbackMismatchIsNotRetried)
{
  {
    Rig rig;
    rig.link.mode.value = mgv::ControlMode::kKeyboard;
    rig.connect();
    rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
    rig.link.set_mode_status = fail_status(kUnavailable, "Socket closed");
    rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
    rig.engine->step();
    rig.engine->step();
    for (int i = 0; i < 30; ++i) {
      rig.engine->mode_query_step();  // 26.09.26 _ YSH _ 조회 스레드 대신
      rig.push(make_snapshot(1000000 + i * 20000));
      rig.engine->step();
    }
    EXPECT_EQ(rig.link.mode_sets.size(), 3U) << "kReassertAttempts";
    EXPECT_GE(rig.link.mode_queries, 3) << "retries are driven by the mode query thread";
  }
  {
    Rig rig;
    rig.link.mode.value = mgv::ControlMode::kKeyboard;
    rig.connect();
    rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
    mgv::Status refused = fail_status(0, "readback: control mode AUTO not observed");
    refused.readback_checked = true;  // MORAI 는 받았지만 AUTO 가 되지 않았다
    rig.link.set_mode_status = refused;
    rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
    rig.engine->step();
    rig.engine->step();
    for (int i = 0; i < 30; ++i) {
      rig.engine->mode_query_step();  // 26.09.26 _ YSH _ 조회 스레드 대신
      rig.push(make_snapshot(1000000 + i * 20000));
      rig.engine->step();
    }
    EXPECT_EQ(rig.link.mode_sets.size(), 1U);
  }
}

TEST(Engine, NoReassertWhenManualWasRequestedOrDisabled)
{
  {
    Rig rig;
    rig.link.mode.value = mgv::ControlMode::kKeyboard;
    rig.connect();
    rig.engine->note_requested_mode(mgv::ControlMode::kKeyboard);  // Autoware 가 disengage 했다
    rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
    rig.engine->step();
    rig.engine->step();
    EXPECT_TRUE(rig.link.mode_sets.empty());
  }
  {
    EngineConfig cfg;
    cfg.auto_reassert_auto = false;
    Rig rig(cfg);
    rig.link.mode.value = mgv::ControlMode::kKeyboard;
    rig.connect();
    rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
    rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
    rig.engine->step();
    rig.engine->step();
    EXPECT_TRUE(rig.link.mode_sets.empty());
  }
}

TEST(Engine, EngageOnStartSetsAutoOnce)
{
  EngineConfig cfg;
  cfg.engage_on_start = true;
  Rig rig(cfg);
  rig.link.mode.value = mgv::ControlMode::kKeyboard;
  rig.connect();
  ASSERT_EQ(rig.link.mode_sets.size(), 1U);
  EXPECT_EQ(rig.link.mode_sets[0], mgv::ControlMode::kAuto);
  EXPECT_EQ(rig.engine->requested_mode(), mgv::ControlMode::kAuto);
}

TEST(Engine, EgoMissingForOneSecondRefetchesSpecWithoutReconnecting)
{
  Rig rig;
  rig.link.mode.value = mgv::ControlMode::kKeyboard;
  rig.connect();
  rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
  auto no_ego = make_snapshot(1000000);
  no_ego.ego.found = false;
  for (int i = 0; i < 60; ++i) {  // (20 ms WaitForTick + 5 ms) × 60 ≈ 1.5 s
    rig.push(no_ego);
    rig.engine->step();
  }
  EXPECT_EQ(rig.engine->phase(), Engine::Phase::kRunning) << "no detour through the connect phase";
  EXPECT_TRUE(rig.engine->spec_refresh_pending());
  EXPECT_TRUE(rig.sink.outputs.empty());
  EXPECT_EQ(rig.link.spec_calls, 1) << "spec needs the ego — not fetched while it is missing";

  // Ego 가 돌아오면 곧바로: 스펙 재조회 → 같은 업데이트에서 발행·송신 (모드 재지정·재연결 없음)
  // 26.09.26 _ YSH _ 리뷰 반영: 조회 스레드 대신 — Ego 복귀 업데이트가 새 KEYBOARD 조회로 AUTO 재지정을 실제로 판단하게
  //   (조회가 없으면 아래 mode_sets 검사는 재지정 판단 자체를 건너뛰어 공허하다)
  rig.engine->mode_query_step();
  rig.submit(make_control(0.1, 0.5));
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_EQ(rig.link.spec_calls, 2);
  EXPECT_FALSE(rig.engine->spec_refresh_pending());
  EXPECT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_EQ(rig.link.sent.size(), 1U);
  EXPECT_TRUE(rig.link.mode_sets.empty()) << "a scenario change is not a reconnect: no AUTO re-assert";
  EXPECT_EQ(rig.link.mode_queries, 2) << "the Ego-return update evaluated a fresh KEYBOARD query";
  rig.engine->mode_query_step();
  rig.push(make_snapshot(1040000));
  rig.engine->step();
  EXPECT_TRUE(rig.link.mode_sets.empty()) << "still no AUTO re-assert on the next query";
  EXPECT_EQ(rig.engine->stats().reconnects, 0U);
  EXPECT_EQ(rig.engine->stats().link_losses, 0U);
  EXPECT_EQ(rig.engine->stats().spec_refreshes, 1U);
  EXPECT_EQ(rig.sink.connects, (std::vector<bool>{false, true})) << "lamps are re-sent";
}

TEST(Engine, CommandsWaitForTheSpecRefreshButStatusDoesNot)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  auto no_ego = make_snapshot(1000000);
  no_ego.ego.found = false;
  for (int i = 0; i < 60; ++i) {
    rig.push(no_ego);
    rig.engine->step();
  }
  ASSERT_TRUE(rig.engine->spec_refresh_pending());
  rig.link.spec.status = fail_status(kGrpcUnknownForTest, "Ego 없음");
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_EQ(rig.sink.outputs.size(), 1U) << "status resumes immediately";
  EXPECT_TRUE(rig.link.sent.empty()) << "no command with a possibly wrong spec";
  rig.link.spec.status = ok_status();
  rig.push(make_snapshot(1040000));
  rig.engine->step();  // 재시도는 0.5 s 간격
  EXPECT_TRUE(rig.link.sent.empty());
  rig.now_ns += 500 * kMs;
  rig.submit(make_control(0.1, 0.5));
  rig.push(make_snapshot(1060000));
  rig.engine->step();
  EXPECT_EQ(rig.link.sent.size(), 1U);
}

TEST(Engine, ShutdownBrakeSendsStaleCommandForAShortWhile)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.12, 1.0));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_EQ(rig.link.sent.size(), 1U);
  rig.engine->shutdown_brake();
  ASSERT_GT(rig.link.sent.size(), 5U);  // 300 ms / 20 ms 업데이트
  for (size_t i = 1; i < rig.link.sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, -2.4);
    EXPECT_DOUBLE_EQ(rig.link.sent[i].steer, rig.link.sent[0].steer);
  }
}

TEST(Engine, NoShutdownBrakeWhenNothingWasBeingSent)
{
  Rig rig;
  rig.connect();
  rig.engine->shutdown_brake();
  EXPECT_TRUE(rig.link.sent.empty());
}

// ============================================================================
// 리뷰 반영 (26.09.25)
// ============================================================================

TEST(Engine, ZeroTimestampUsesContentGatingAndReceiveStamps)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  auto a = make_snapshot(0);
  a.receive_system_ns = kEpochNs + 7 * kMs;
  auto b = a;  // 같은 내용 (정지한 차) — WaitForTick 이 새 틱을 확인했으니 새 업데이트
  b.receive_system_ns = kEpochNs + 27 * kMs;
  auto c = b;
  c.ego.position.x += 0.1;
  c.receive_system_ns = kEpochNs + 47 * kMs;
  rig.push(a);
  rig.push(b);
  rig.push(c);
  for (int i = 0; i < 3; ++i) {
    rig.engine->step();
  }
  EXPECT_EQ(rig.engine->gate_mode(), GateMode::kContent);
  EXPECT_EQ(rig.engine->stats().content_mode_entries, 1U);
  EXPECT_EQ(rig.link.sent.size(), 3U) << "a stopped car must still get commands (confirmed ticks)";
  ASSERT_EQ(rig.sink.outputs.size(), 3U);
  EXPECT_EQ(rig.sink.outputs[0].stamp, morai_to_autoware::to_time_msg(a.receive_system_ns))
    << "never stamp epoch 0";
  EXPECT_EQ(rig.engine->clock_offset_ms().count, 0U) << "no clock offset from an unusable timestamp";
}

TEST(Engine, ContentGatingWithoutConfirmedTicksSkipsIdenticalSamples)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.link.wft_script.push_back(fail_status(kUnimplemented));  // 폴링 폴백 — 틱 확인 없음
  const auto a = make_snapshot(0);
  rig.push(a);
  rig.push(a);  // 100 Hz 폴링이 같은 업데이트를 두 번 본다
  auto b = a;
  b.ego.velocity_kmh.x = 1.0;
  rig.push(b);
  for (int i = 0; i < 3; ++i) {
    rig.engine->step();
  }
  EXPECT_TRUE(rig.engine->fallback_active());
  EXPECT_EQ(rig.link.sent.size(), 2U);
  EXPECT_EQ(rig.engine->stats().duplicates, 1U);
}

TEST(Engine, FrozenTimestampWithMovingEgoSwitchesToContentGating)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  for (int i = 0; i < 6; ++i) {
    auto s = make_snapshot(5000000, 36.0);
    s.ego.position.x = 100.0 + 0.2 * i;  // 차는 움직이는데 timestamp 는 그대로
    rig.push(s);
  }
  for (int i = 0; i < 6; ++i) {
    rig.engine->step();
  }
  EXPECT_EQ(rig.engine->gate_mode(), GateMode::kContent);
  // 1 번째(첫 표본) 받음, 2·3 번째 중복(내용 변화 1·2 회), 4 번째에 전환하며 받음, 5·6 번째 받음
  EXPECT_EQ(rig.link.sent.size(), 4U);
  EXPECT_EQ(rig.sink.outputs.size(), 4U);
}

TEST(Engine, PausedMoraiWithFrozenTimestampAndContentPublishesNothing)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  for (int i = 0; i < 20; ++i) {
    rig.push(make_snapshot(5000000));  // 일시정지: timestamp 도 내용도 그대로
  }
  for (int i = 0; i < 20; ++i) {
    rig.engine->step();
  }
  EXPECT_EQ(rig.engine->gate_mode(), GateMode::kTimestamp);
  EXPECT_EQ(rig.link.sent.size(), 1U);
  EXPECT_EQ(rig.sink.outputs.size(), 1U);
}

TEST(Engine, BackwardClockStepRebasesWithinThreeUpdates)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  for (const int64_t ts : {1000000, 1020000, 1040000}) {
    rig.push(make_snapshot(ts));
  }
  // MORAI 호스트 시계가 200 ms 뒤로 (chrony makestep 등) — 역행 3 표본째에 새 기준
  for (const int64_t ts : {860000, 880000, 900000, 920000}) {
    rig.push(make_snapshot(ts));
  }
  for (int i = 0; i < 7; ++i) {
    rig.engine->step();
  }
  const auto st = rig.engine->stats();
  EXPECT_EQ(st.backward, 2U);
  EXPECT_EQ(st.rebased, 1U);
  EXPECT_EQ(rig.link.sent.size(), 5U) << "commands pause for at most two updates (~40 ms), not 1 s";
}

TEST(Engine, LargeBackwardJumpRebasesImmediately)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.push(make_snapshot(5000000));
  rig.push(make_snapshot(1000000));  // 4 s 뒤로 (MORAI 재시작)
  rig.engine->step();
  rig.engine->step();
  EXPECT_EQ(rig.engine->stats().rebased, 1U);
  EXPECT_EQ(rig.link.sent.size(), 2U);
  EXPECT_EQ(rig.sink.outputs.size(), 2U);
}

TEST(Engine, NonBlockingWaitForTickFallsBackInsteadOfSpinning)
{
  Rig rig;
  rig.link.tick_advance_ns = 0;  // WaitForTick 이 막지 않고 바로 성공한다
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  for (int i = 0; i < 400; ++i) {
    rig.push(make_snapshot(1000000));  // MORAI 는 그대로 (새 업데이트 없음)
  }
  const int64_t start = rig.now_ns;
  while (rig.now_ns < start + 1000 * kMs && !rig.link.snapshots.empty()) {
    rig.engine->step();
  }
  EXPECT_TRUE(rig.engine->fallback_active());
  EXPECT_EQ(rig.engine->stats().wft_no_update, 10U);
  EXPECT_EQ(rig.link.sent.size(), 1U);
  // 폴백 뒤에는 fallback_hz(100 Hz) 로만 읽는다 — 헛돌면 1 s 에 수만 번
  EXPECT_LE(rig.link.snapshot_calls, 11 + 105);
}

TEST(Engine, FallbackProbeNeedsBlockingDistinctTicksAndUsesAShortDeadline)
{
  Rig rig;
  rig.connect();
  rig.link.wft_script.push_back(fail_status(kUnimplemented));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  ASSERT_TRUE(rig.engine->fallback_active());

  rig.link.tick_advance_ns = 0;  // 이번에는 막지 않고 성공 — 트리거가 아니다
  rig.now_ns += 11LL * 1000 * kMs;
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  EXPECT_TRUE(rig.engine->fallback_active());
  EXPECT_LE(rig.link.wft_deadlines_ms.back(), 40) << "probe must not stall the loop for 100 ms";

  rig.link.tick_advance_ns = 20 * kMs;
  rig.now_ns += 11LL * 1000 * kMs;
  rig.push(make_snapshot(1040000));
  rig.engine->step();
  EXPECT_FALSE(rig.engine->fallback_active());
}

TEST(Engine, ControlModeReportIsOmittedWhenTheQueryGoesStale)
{
  Rig rig;
  rig.connect();
  rig.link.mode.status = fail_status(kDeadline);
  for (int i = 0; i < 40; ++i) {  // 40 × 20 ms = 0.8 s, 조회는 모두 실패
    if (i % 5 == 0) {
      rig.engine->mode_query_step();  // 26.09.26 _ YSH _ 조회 스레드 대신 (~10 Hz)
    }
    rig.push(make_snapshot(1000000 + 20000 * i));
    rig.engine->step();
  }
  ASSERT_EQ(rig.sink.outputs.size(), 40U);
  EXPECT_TRUE(rig.sink.outputs[0].control_mode.has_value());
  ASSERT_TRUE(rig.sink.outputs[38].gear.has_value());
  EXPECT_FALSE(rig.sink.outputs[38].control_mode.has_value())
    << "a stale AUTONOMOUS must not hide a switch to KEYBOARD (main.py: failed query → no report)";
  EXPECT_GT(*rig.engine->control_mode_age_s(), Engine::kModeFreshS);
}

TEST(Engine, ReassertWaitsForTheFirstSuccessfulModeQueryAfterReconnect)
{
  Rig rig;
  rig.connect();  // MORAI AUTO
  rig.engine->note_requested_mode(mgv::ControlMode::kAuto);
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();  // 끊김
  EXPECT_FALSE(rig.engine->control_mode().has_value()) << "the pre-disconnect mode is forgotten";
  rig.link.mode.value = mgv::ControlMode::kKeyboard;  // MORAI 재기동 → KEYBOARD
  rig.link.mode.status = fail_status(kDeadline);      // 재연결 순간 조회 실패
  rig.engine->step();
  ASSERT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  EXPECT_TRUE(rig.link.mode_sets.empty());
  rig.link.mode.status = ok_status();
  rig.push(make_snapshot(1000000));
  // 26.09.26 _ YSH _ 조회 스레드의 첫 성공 조회가 KEYBOARD 를 보고, 다음 업데이트가 AUTO 재지정. 원문:
  // rig.engine->step();  // 첫 업데이트의 조회가 KEYBOARD 를 본다 → AUTO 재지정
  rig.engine->mode_query_step();
  rig.engine->step();
  ASSERT_EQ(rig.link.mode_sets.size(), 1U);
  EXPECT_EQ(rig.link.mode_sets[0], mgv::ControlMode::kAuto);
}

TEST(Engine, FailedEngageOnStartDoesNotLeaveAutoRequested)
{
  EngineConfig cfg;
  cfg.engage_on_start = true;
  Rig rig(cfg);
  rig.link.set_mode_status = fail_status(0, "rejected");
  rig.connect();
  EXPECT_FALSE(rig.engine->requested_mode().has_value());
}

TEST(Engine, CommandDeadlineFollowsTheMoraiRoundTrip)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  for (int i = 0; i < 52; ++i) {
    auto s = make_snapshot(1000000 + 20000 * i);
    s.rpc_ms = 20.0;  // 원격 MORAI: 왕복 20 ms
    rig.push(s);
    rig.submit(make_control(0.1, 0.5));
    rig.engine->step();
  }
  ASSERT_EQ(rig.link.sent_deadlines_ms.size(), 52U);
  EXPECT_EQ(rig.link.sent_deadlines_ms.front(), 50) << "local round trip: the configured 50 ms";
  EXPECT_EQ(rig.link.sent_deadlines_ms.back(), 80) << "main.py cmd_timeout_for_rtt: 4 x RTT";
  EXPECT_EQ(rig.engine->command_deadline_ms(), 80);

  Rig slow;
  slow.connect();
  for (int i = 0; i < 52; ++i) {
    auto s = make_snapshot(1000000 + 20000 * i);
    s.rpc_ms = 400.0;
    slow.push(s);
    slow.engine->step();
  }
  EXPECT_EQ(slow.engine->command_deadline_ms(), 1000) << "capped at 1 s";
}

TEST(Engine, GuardBlockWhileSendingBrakesBrieflyThenStops)
{
  Rig rig;
  rig.connect();
  for (int i = 0; i < 3; ++i) {
    rig.submit(make_control(0.12, 1.0));
    rig.push(make_snapshot(1000000 + 20000 * i));
    rig.engine->step();
  }
  ASSERT_EQ(rig.link.sent.size(), 3U);
  rig.engine->set_command_block(true);  // 다른 writer 가 나타났다
  for (int i = 3; i < 23; ++i) {
    rig.submit(make_control(0.12, 1.0));
    rig.push(make_snapshot(1000000 + 20000 * i));
    rig.engine->step();
  }
  const auto st = rig.engine->stats();
  EXPECT_EQ(st.block_brake_sends, 15U);  // 0.3 s / 20 ms
  EXPECT_EQ(st.blocked_sends, 5U);
  ASSERT_EQ(rig.link.sent.size(), 3U + 15U);
  for (size_t i = 3; i < rig.link.sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, -2.4) << "MORAI holds the last command: leave a brake";
    EXPECT_DOUBLE_EQ(rig.link.sent[i].steer, rig.link.sent[0].steer);
  }
}

// ============================================================================
// 26.09.26 _ YSH _ 원시 gRPC 기록 훅 (on_command_sent)
// ============================================================================

/// 두 명령이 필드마다 비트까지 같은가 (NaN 은 없다 — 슬롯이 걸렀다)
void expect_same_command(const mgv::VehicleCommand & a, const mgv::VehicleCommand & b, size_t i)
{
  EXPECT_EQ(a.long_cmd_type, b.long_cmd_type) << i;
  EXPECT_EQ(a.throttle, b.throttle) << i;
  EXPECT_EQ(a.brake, b.brake) << i;
  EXPECT_EQ(a.steer, b.steer) << i;
  EXPECT_EQ(a.velocity, b.velocity) << i;
  EXPECT_EQ(a.acceleration, b.acceleration) << i;
  EXPECT_EQ(a.frame, b.frame) << i;
}

TEST(EngineRawHook, OnCommandSentSeesExactlyThePayloadSentIncludingTheStaleSubstitute)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.15, 1.0, 5.0));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  rig.now_ns += 250 * kMs;  // Autoware 가 조용하다 → 감속 대체 명령
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  rig.link.control_status = fail_status(kDeadline, "slow");  // 결과도 그대로 넘어간다
  rig.submit(make_control(0.05, 0.3, 5.0));
  rig.push(make_snapshot(1040000));
  rig.engine->step();

  ASSERT_EQ(rig.link.sent.size(), 3U);
  ASSERT_EQ(rig.sink.sent.size(), rig.link.sent.size()) << "one hook call per ControlVehicle";
  for (size_t i = 0; i < rig.link.sent.size(); ++i) {
    expect_same_command(rig.sink.sent[i].cmd, rig.link.sent[i], i);
    // RPC 직전 system_clock (가짜 시계: 링크가 본 호출 시각 + epoch)
    EXPECT_EQ(rig.sink.sent[i].send_system_ns, kEpochNs + rig.link.sent_ns[i]) << i;
  }
  EXPECT_FALSE(rig.sink.sent[0].stale);
  EXPECT_TRUE(rig.sink.sent[1].stale);
  EXPECT_DOUBLE_EQ(rig.sink.sent[1].cmd.acceleration, -2.4) << "the substitute, not Autoware's 1.0";
  EXPECT_FALSE(rig.sink.sent[2].stale);
  EXPECT_TRUE(rig.sink.sent[0].status.ok);
  EXPECT_EQ(rig.sink.sent[0].status.grpc_code, 0);
  EXPECT_DOUBLE_EQ(rig.sink.sent[0].status.rpc_ms, 0.2);
  EXPECT_FALSE(rig.sink.sent[2].status.ok);
  EXPECT_EQ(rig.sink.sent[2].status.grpc_code, kDeadline);
}

TEST(EngineRawHook, GuardBrakeAndShutdownBrakeAreReportedAsStale)
{
  Rig rig;
  rig.connect();
  rig.submit(make_control(0.12, 1.0));
  rig.push(make_snapshot(1000000));
  rig.engine->step();
  rig.engine->set_command_block(true);  // 송신 중에 막힘 → 가드 감속
  rig.submit(make_control(0.12, 1.0));
  rig.push(make_snapshot(1020000));
  rig.engine->step();
  rig.engine->set_command_block(false);
  rig.submit(make_control(0.12, 1.0));
  rig.push(make_snapshot(1040000));
  rig.engine->step();
  const size_t before = rig.link.sent.size();
  ASSERT_EQ(before, 3U);
  rig.engine->shutdown_brake();
  ASSERT_GT(rig.link.sent.size(), before);
  ASSERT_EQ(rig.sink.sent.size(), rig.link.sent.size());
  EXPECT_FALSE(rig.sink.sent[0].stale);
  EXPECT_TRUE(rig.sink.sent[1].stale) << "guard brake";
  EXPECT_FALSE(rig.sink.sent[2].stale);
  for (size_t i = before; i < rig.link.sent.size(); ++i) {
    EXPECT_TRUE(rig.sink.sent[i].stale) << "shutdown brake " << i;
    expect_same_command(rig.sink.sent[i].cmd, rig.link.sent[i], i);
    EXPECT_DOUBLE_EQ(rig.sink.sent[i].cmd.acceleration, -2.4);
  }
}

TEST(EngineRawHook, TimerTriggerReportsEverySend)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer, 100.0);
  rig.drive(300 * kMs);
  ASSERT_GT(rig.link.sent.size(), 20U);
  ASSERT_EQ(rig.sink.sent.size(), rig.link.sent.size());
  for (size_t i = 0; i < rig.link.sent.size(); ++i) {
    expect_same_command(rig.sink.sent[i].cmd, rig.link.sent[i], i);
    EXPECT_EQ(rig.sink.sent[i].send_system_ns, kEpochNs + rig.link.sent_ns[i]) << i;
  }
}

// ============================================================================
// 26.09.25 _ YSH _ command_trigger=timer (main.py _CmdResender 조건: MORAI 50 Hz + 100 Hz 고정 주기 재송신)
// ============================================================================

TEST(CommandTrigger, ParsesAndValidates)
{
  EXPECT_EQ(parse_command_trigger("update"), CommandTrigger::kUpdate);
  EXPECT_EQ(parse_command_trigger("timer"), CommandTrigger::kTimer);
  EXPECT_FALSE(parse_command_trigger("Timer").has_value());
  EXPECT_FALSE(parse_command_trigger("").has_value());
  EXPECT_STREQ(to_string(CommandTrigger::kTimer), "timer");
  EXPECT_STREQ(to_string(CommandTrigger::kUpdate), "update");
  EXPECT_TRUE(cmd_resend_hz_is_valid(100.0));
  EXPECT_TRUE(cmd_resend_hz_is_valid(10.0));
  EXPECT_TRUE(cmd_resend_hz_is_valid(400.0));
  EXPECT_FALSE(cmd_resend_hz_is_valid(9.99));
  EXPECT_FALSE(cmd_resend_hz_is_valid(400.5));
  EXPECT_FALSE(cmd_resend_hz_is_valid(std::nan("")));
  EXPECT_FALSE(cmd_resend_hz_is_valid(INFINITY));
  EXPECT_EQ(RuntimeParams{}.command_trigger, CommandTrigger::kUpdate) << "default stays per-update";
  EXPECT_DOUBLE_EQ(RuntimeParams{}.cmd_resend_hz, 100.0) << "main.py default";
}

TEST(EngineTimer, SendsAtTheConfiguredRateIndependentOfTheUpdateRate)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer, 100.0);
  const auto before = rig.engine->stats();
  rig.drive(1000 * kMs);
  const auto s = rig.engine->stats();
  EXPECT_EQ(s.updates - before.updates, 50U) << "MORAI 50 updates/s";
  EXPECT_EQ(s.sends - before.sends, 100U) << "100 Hz, not once per update";
  ASSERT_EQ(rig.link.sent_ns.size(), 100U);
  for (size_t i = 1; i < rig.link.sent_ns.size(); ++i) {
    EXPECT_EQ(rig.link.sent_ns[i] - rig.link.sent_ns[i - 1], 10 * kMs)
      << "absolute 10 ms deadlines, i=" << i;
  }
  EXPECT_EQ(s.cmd_timer_late, 0U);
  EXPECT_EQ(rig.sink.outputs.size(), 50U) << "status is still published once per update";
  for (const auto d : rig.link.sent_deadlines_ms) {
    EXPECT_EQ(d, 50) << "same per-call deadline logic (max(setting, 4 x RTT))";
  }
  const auto gap = rig.engine->send_gap_ms();
  EXPECT_NEAR(gap.p50, 10.0, 1e-9);
  EXPECT_NEAR(gap.max, 10.0, 1e-9);

  // 200 Hz 도 업데이트와 무관하게
  Rig fast;
  fast.connect();
  fast.set_trigger(CommandTrigger::kTimer, 200.0);
  fast.drive(500 * kMs);
  EXPECT_EQ(fast.link.sent.size(), 100U);
  EXPECT_EQ(fast.engine->stats().updates, 25U);
}

TEST(EngineTimer, UpdateModeIsUnchangedUnderTheSameDrive)
{
  Rig rig;
  rig.connect();
  rig.drive(1000 * kMs);  // 기본 update — 타이머는 설정만 보고 잔다
  const auto s = rig.engine->stats();
  EXPECT_EQ(s.updates, 50U);
  EXPECT_EQ(s.sends, 50U) << "exactly one ControlVehicle per update";
  for (size_t i = 1; i < rig.link.sent_ns.size(); ++i) {
    EXPECT_EQ(rig.link.sent_ns[i] - rig.link.sent_ns[i - 1], 20 * kMs);
  }
  EXPECT_EQ(s.cmd_timer_late, 0U);
}

TEST(EngineTimer, EngineLoopDoesNotSendInTimerMode)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.submit(make_control(0.1, 0.5));
  for (int i = 0; i < 5; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * i));
    rig.engine->step();
  }
  EXPECT_TRUE(rig.link.sent.empty()) << "no double sending: only the command timer sends";
  EXPECT_EQ(rig.engine->stats().updates, 5U);
  EXPECT_EQ(rig.sink.outputs.size(), 5U);
  rig.engine->timer_step();
  EXPECT_EQ(rig.link.sent.size(), 1U);
}

TEST(EngineTimer, SendsNothingBeforeTheLinkTheSpecAndTheFirstValidSample)
{
  Rig rig;
  rig.set_trigger(CommandTrigger::kTimer);
  rig.submit(make_control(0.1, 0.5));
  rig.run_timer(100 * kMs);  // 연결 전
  EXPECT_TRUE(rig.link.sent.empty());
  rig.connect();
  rig.submit(make_control(0.1, 0.5));
  rig.run_timer(100 * kMs);  // 스펙은 있으나 받아들인 표본이 아직 없다
  EXPECT_TRUE(rig.link.sent.empty()) << "no sample yet: steer-comp speed unknown";
  auto bad = make_snapshot(1000000);
  bad.ego.valid = false;
  rig.engine->on_update(bad, true);
  rig.run_timer(50 * kMs);
  EXPECT_TRUE(rig.link.sent.empty()) << "an invalid ego is not a sample";
  rig.submit(make_control(0.1, 0.5));
  rig.engine->on_update(make_snapshot(1020000), true);
  rig.run_timer(50 * kMs);
  EXPECT_EQ(rig.link.sent.size(), 5U);
}

TEST(EngineTimer, LinkDownNeitherSendsNorSpins)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.drive(200 * kMs);
  const size_t sent_before = rig.link.sent.size();
  ASSERT_EQ(sent_before, 20U);
  rig.link.snapshots.emplace_back(fail_status(kUnavailable), mgv::Snapshot{});
  rig.engine->step();  // 끊김 → 연결 단계
  ASSERT_EQ(rig.engine->phase(), Engine::Phase::kConnecting);
  rig.submit(make_control(0.1, 0.5));  // Autoware 는 계속 보낸다
  const int calls = rig.run_timer(1000 * kMs);
  EXPECT_EQ(rig.link.sent.size(), sent_before) << "no ControlVehicle while the engine reconnects";
  EXPECT_LE(calls, 2 * 100 + 3) << "paced by its own deadlines, never a busy loop";
  // 다시 붙으면 곧바로 주기 송신 (새 표본을 기다리지 않는다 — 캐시는 끊기기 전 표본)
  rig.engine->step();
  ASSERT_EQ(rig.engine->phase(), Engine::Phase::kRunning);
  rig.submit(make_control(0.1, 0.5));
  rig.run_timer(100 * kMs);
  EXPECT_EQ(rig.link.sent.size(), sent_before + 10U);
}

TEST(EngineTimer, StaleCommandBrakesAndKeepsLastSteer)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000), true);
  rig.submit(make_control(0.15, 1.0, 5.0));  // 마지막 control_cmd (t = 0)
  rig.run_timer(400 * kMs);
  ASSERT_EQ(rig.link.sent.size(), 40U);
  // 0 ~ 200 ms 는 그대로 (나이 ≤ 0.2 s), 210 ms 부터 감속 (main.py: age > stale_timeout_s)
  for (size_t i = 0; i <= 20; ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, static_cast<double>(1.0F)) << i;
  }
  for (size_t i = 21; i < rig.link.sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, -2.4) << i;
    EXPECT_DOUBLE_EQ(rig.link.sent[i].velocity, 0.0);
    EXPECT_DOUBLE_EQ(rig.link.sent[i].steer, rig.link.sent[0].steer) << "steer kept";
  }
  EXPECT_EQ(rig.engine->stats().stale_sends, 19U);
  // Autoware 가 돌아오면 다음 틱에 곧바로 정상 명령
  rig.submit(make_control(0.15, 1.0, 5.0));
  rig.run_timer(10 * kMs);
  EXPECT_DOUBLE_EQ(rig.link.sent.back().acceleration, static_cast<double>(1.0F));
}

TEST(EngineTimer, NonFiniteMessageIsDroppedAndLastValidCommandKept)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000), true);
  rig.submit(make_control(0.1, 0.5));
  rig.run_timer(30 * kMs);
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(
      submit_control(rig.slot, make_control(0.1, std::nan("")), true, rig.now_ns),
      SubmitResult::kRejectedNonFinite);
    rig.run_timer(20 * kMs);
  }
  ASSERT_EQ(rig.link.sent.size(), 13U);
  for (const auto & c : rig.link.sent) {
    EXPECT_TRUE(std::isfinite(c.steer) && std::isfinite(c.acceleration));
    EXPECT_NEAR(c.steer, 0.1, 1e-6);
  }
  // 마지막 유효 명령이 0.2 s 넘게 낡으면 stale 감속 (NaN 이 나이를 되살리지 않는다)
  EXPECT_DOUBLE_EQ(rig.link.sent.back().acceleration, static_cast<double>(0.5F));
  rig.run_timer(100 * kMs);
  EXPECT_DOUBLE_EQ(rig.link.sent.back().acceleration, -2.4);
}

TEST(EngineTimer, NoBurstAfterAStall)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000), true);
  // 5 번째 ControlVehicle 이 55 ms 걸린다 (MORAI 정체) — 밀린 deadline 4 개를 몰아 보내면 안 된다
  rig.link.control_delays_ns = {0, 0, 0, 0, 55 * kMs};
  for (int i = 0; i < 10; ++i) {
    rig.submit(make_control(0.1, 0.5));
    rig.run_timer(10 * kMs);
  }
  const auto & t = rig.link.sent_ns;
  ASSERT_GE(t.size(), 8U);
  const int64_t t4 = t[4];
  EXPECT_EQ(t[5] - t4, 55 * kMs) << "exactly one send right after the stall";
  for (size_t i = 6; i < t.size(); ++i) {
    EXPECT_EQ(t[i] - t[i - 1], 10 * kMs) << "then the normal period from the reset base, i=" << i;
  }
  EXPECT_EQ(rig.engine->stats().cmd_timer_late, 1U);
  size_t in_window = 0;
  for (const auto x : t) {
    in_window += (x >= t4 && x < t4 + 100 * kMs) ? 1 : 0;
  }
  EXPECT_EQ(in_window, 6U) << "t4, +55, +65, +75, +85, +95 — not 5 catch-up sends at +55";
}

TEST(EngineTimer, ForwardingOffClearsTheSlotAndSendsNothing)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000), true);
  rig.submit(make_control(0.1, 0.5));
  rig.set_params([](RuntimeParams & p) { p.forward_control_to_sim = false; });
  rig.run_timer(100 * kMs);
  EXPECT_TRUE(rig.link.sent.empty());
  EXPECT_TRUE(rig.slot.empty());
  rig.set_params([](RuntimeParams & p) { p.forward_control_to_sim = true; });
  rig.run_timer(100 * kMs);
  EXPECT_TRUE(rig.link.sent.empty()) << "needs a new control_cmd (main.py)";
  rig.submit(make_control(0.1, 0.5));
  rig.run_timer(50 * kMs);
  EXPECT_EQ(rig.link.sent.size(), 5U);
}

TEST(EngineTimer, DuplicateWriterGuardBrakesBrieflyThenStops)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000, 0.0), true);
  for (int i = 0; i < 3; ++i) {
    rig.submit(make_control(0.12, 1.0));
    rig.run_timer(10 * kMs);
  }
  ASSERT_EQ(rig.link.sent.size(), 3U);
  rig.engine->set_command_block(true);
  for (int i = 0; i < 50; ++i) {
    rig.submit(make_control(0.12, 1.0));
    rig.run_timer(10 * kMs);
  }
  const auto st = rig.engine->stats();
  EXPECT_EQ(st.block_brake_sends, 30U) << "0.3 s at 100 Hz";
  EXPECT_EQ(st.blocked_sends, 20U);
  ASSERT_EQ(rig.link.sent.size(), 3U + 30U);
  for (size_t i = 3; i < rig.link.sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, -2.4);
    EXPECT_DOUBLE_EQ(rig.link.sent[i].steer, rig.link.sent[0].steer);
  }
}

TEST(EngineTimer, TriggerCanBeSwitchedAtRuntime)
{
  Rig rig;
  rig.connect();
  rig.drive(200 * kMs);  // update
  EXPECT_EQ(rig.link.sent.size(), 10U);
  rig.set_trigger(CommandTrigger::kTimer);
  rig.drive(200 * kMs);
  EXPECT_EQ(rig.link.sent.size(), 10U + 20U);
  rig.set_trigger(CommandTrigger::kUpdate);
  rig.drive(200 * kMs);
  EXPECT_EQ(rig.link.sent.size(), 10U + 20U + 10U);
  // 주기도 런타임에 (기준을 지금으로 다시 잡는다)
  rig.set_trigger(CommandTrigger::kTimer, 50.0);
  rig.drive(200 * kMs);
  EXPECT_EQ(rig.link.sent.size(), 10U + 20U + 10U + 10U);
  EXPECT_EQ(rig.engine->stats().updates, 40U);
  EXPECT_EQ(rig.engine->stats().cmd_timer_late, 0U);
}

TEST(EngineTimer, SteerCompUsesTheLatestSampleSpeedAndIsBuiltOncePerMessage)
{
  Rig rig(EngineConfig{}, true);  // 조향 결손 보정 켬
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000, 12.6), true);  // 3.5 m/s → gain 0.8625
  rig.submit(make_control(0.2, 0.5, 10.0));
  rig.run_timer(10 * kMs);
  ASSERT_EQ(rig.link.sent.size(), 1U);
  EXPECT_NEAR(rig.link.sent[0].steer, static_cast<double>(0.2F) / 0.8625, 1e-12);
  rig.engine->on_update(make_snapshot(1020000, 36.0), true);  // 10 m/s — 같은 메시지면 다시 만들지 않는다
  rig.run_timer(10 * kMs);
  ASSERT_EQ(rig.link.sent.size(), 2U);
  EXPECT_DOUBLE_EQ(rig.link.sent[1].steer, rig.link.sent[0].steer);
  rig.submit(make_control(0.2, 0.5, 10.0));  // 새 메시지 → 최신 표본 속도(10 m/s)로
  rig.run_timer(10 * kMs);
  const double gain_10 = 0.807 + (10.0 - 8.0) / 3.0 * (0.796 - 0.807);
  EXPECT_NEAR(rig.link.sent.back().steer, static_cast<double>(0.2F) / gain_10, 1e-9);
  // 조향 리포트 역보정은 타이머가 만든 명령의 배율을 쓴다
  rig.engine->on_update(make_snapshot(1040000, 36.0, -0.25), true);
  const double max_rad = 40.0 / (180.0 / M_PI);
  EXPECT_FLOAT_EQ(
    rig.sink.outputs.back().steering.steering_tire_angle, static_cast<float>(0.25 * max_rad * gain_10));
}

TEST(EngineTimer, ShutdownBrakeIsPacedAtTheTimerPeriod)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.drive(100 * kMs);
  const size_t before = rig.link.sent.size();
  const int wft_before = rig.link.wft_calls;
  rig.engine->shutdown_brake();
  const size_t n = rig.link.sent.size() - before;
  EXPECT_EQ(n, 30U) << "0.3 s at 100 Hz";
  EXPECT_EQ(rig.link.wft_calls, wft_before) << "not paced by MORAI updates in timer mode";
  for (size_t i = before; i < rig.link.sent.size(); ++i) {
    EXPECT_DOUBLE_EQ(rig.link.sent[i].acceleration, -2.4);
    EXPECT_DOUBLE_EQ(rig.link.sent[i].steer, rig.link.sent[0].steer);
    if (i > before) {
      EXPECT_EQ(rig.link.sent_ns[i] - rig.link.sent_ns[i - 1], 10 * kMs);
    }
  }
}

TEST(EngineTimer, SlowRoundTripsAreCountedLateNotQueued)
{
  Rig rig;
  rig.connect();
  rig.set_trigger(CommandTrigger::kTimer);
  rig.engine->on_update(make_snapshot(1000000), true);
  for (int i = 0; i < 20; ++i) {
    rig.link.control_delays_ns.push_back(15 * kMs);  // 매 호출 15 ms > 10 ms 주기
  }
  for (int i = 0; i < 10; ++i) {
    rig.submit(make_control(0.1, 0.5));
    rig.run_timer(30 * kMs);
  }
  const auto & t = rig.link.sent_ns;
  ASSERT_GE(t.size(), 15U);
  for (size_t i = 1; i < 15; ++i) {
    EXPECT_EQ(t[i] - t[i - 1], 15 * kMs) << "back-to-back at the RPC pace, never faster, i=" << i;
  }
  EXPECT_GE(rig.engine->stats().cmd_timer_late, 14U);
}

/// 26.09.25 _ YSH _ 실제 스레드·실제 시계용 가짜 MORAI: WaitForTick 은 20 ms 막히고(50 업데이트/s) 스냅샷은 매번 새 timestamp.
/// ControlVehicle 은 두 스레드(엔진·명령 타이머)가 부를 수 있어 잠근다
class ThreadedLink : public ILink
{
public:
  mgv::Status wait_connected(std::chrono::milliseconds) override { return ok_status(); }
  mgv::Reply<int64_t> wait_for_tick(std::chrono::milliseconds) override
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    mgv::Reply<int64_t> r;
    r.status = ok_status();
    r.value = ++tick_;
    return r;
  }
  mgv::Status get_snapshot(mgv::Snapshot * out) override
  {
    ts_us_ += 20000;
    *out = make_snapshot(ts_us_.load());
    return ok_status(0.3);
  }
  mgv::Reply<mgv::ControlMode> get_control_mode() override
  {
    mgv::Reply<mgv::ControlMode> r;
    r.status = ok_status();
    r.value = mgv::ControlMode::kAuto;
    return r;
  }
  mgv::Reply<mgv::VehicleSpecInfo> get_vehicle_spec() override
  {
    mgv::Reply<mgv::VehicleSpecInfo> r;
    r.status = ok_status();
    r.value.max_steering_angle_deg = 40.0;
    r.value.wheel_base_m = 3.0;
    return r;
  }
  mgv::Status control_vehicle(const mgv::VehicleCommand & cmd, std::chrono::milliseconds) override
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    sent.push_back(cmd);
    sent_ns.push_back(EngineClock::real().steady_ns());
    return ok_status();
  }
  mgv::Status set_gear(mgv::GearMode, std::chrono::milliseconds) override { return ok_status(); }
  mgv::Status set_control_mode(mgv::ControlMode, std::chrono::milliseconds) override { return ok_status(); }
  mgv::Status set_transform(const mgv::TransformPose &) override { return ok_status(); }
  mgv::Status set_tail_light(mgv::TurnSignal) override { return ok_status(); }
  void recreate() override {}
  mgv::LinkCounters counters() const override { return {}; }
  std::string target() const override { return "threaded-fake:0"; }

  std::mutex mutex_;
  std::vector<mgv::VehicleCommand> sent;
  std::vector<int64_t> sent_ns;

private:
  std::atomic<int64_t> tick_{0};
  std::atomic<int64_t> ts_us_{1000000};
};

TEST(EngineTimer, RealThreadsSendNearTheTargetRateAndStopCleanly)
{
  // 실제 시계 — 부하가 큰 PC 에서도 깨지지 않게 허용 폭을 넓게 잡는다 (정밀한 주기는 위 가짜 시계 시험이 고정한다)
  for (const auto trigger : {CommandTrigger::kTimer, CommandTrigger::kUpdate}) {
    ThreadedLink link;
    CommandSlot slot;
    RuntimeParamsHandle params;
    StateCache cache;
    FakeSink sink;
    RuntimeParams rp;
    rp.command_trigger = trigger;
    rp.cmd_resend_hz = 100.0;
    params.set(rp);
    Engine engine(link, slot, params, cache, sink, EngineConfig{}, rclcpp::get_logger("test_engine_threads"));
    engine.start();
    const auto real = EngineClock::real();
    const int64_t t0 = real.steady_ns();
    while (real.steady_ns() - t0 < 1300 * kMs) {
      submit_control(slot, make_control(0.1, 0.5), true, real.steady_ns());
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    const int64_t t_stop = real.steady_ns();
    const auto st = engine.stats();
    engine.stop();  // 두 스레드를 거두고 종료 감속
    ASSERT_FALSE(link.sent_ns.empty());
    const int64_t w0 = link.sent_ns.front() + 100 * kMs;
    size_t in_second = 0;
    size_t normal_after_stop = 0;
    size_t brakes = 0;
    for (size_t i = 0; i < link.sent.size(); ++i) {
      in_second += (link.sent_ns[i] >= w0 && link.sent_ns[i] < w0 + 1000 * kMs) ? 1 : 0;
      if (link.sent_ns[i] >= t_stop) {
        brakes += link.sent[i].acceleration == -2.4 ? 1 : 0;
        normal_after_stop += link.sent[i].acceleration != -2.4 ? 1 : 0;
      }
    }
    const bool timer = trigger == CommandTrigger::kTimer;
    if (timer) {
      EXPECT_GE(in_second, 80U) << "~100 per second with MORAI at 50 updates/s";
      EXPECT_LE(in_second, 105U) << "never faster than the target (no catch-up bursts)";
      EXPECT_GE(brakes, 20U) << "shutdown brake ~0.3 s at 100 Hz";
    } else {
      EXPECT_GE(in_second, 35U);
      EXPECT_LE(in_second, 52U) << "one per update";
      EXPECT_GE(brakes, 5U);
      EXPECT_LE(st.sends, st.updates + 1) << "one per update";
    }
    EXPECT_LE(normal_after_stop, 2U) << "only a send already in flight may land after stop()";
  }
}

// 26.09.26 _ YSH _ MORAI 는 GetVehicleControlMode 를 다음 프레임에야 돌려준다 (실측 최대 ~20 ms). 엔진 루프 안에서 조회하던
//   때는 5 업데이트마다 한 프레임을 놓쳐 41.5 Hz 였다 — 조회를 전용 스레드로 옮긴 뒤에는 업데이트를 하나도 잃지 않아야 한다
class SlowModeLink : public ThreadedLink
{
public:
  mgv::Reply<mgv::ControlMode> get_control_mode() override
  {
    ++mode_queries;
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return ThreadedLink::get_control_mode();
  }
  std::atomic<int> mode_queries{0};
};

TEST(EngineModeQuery, SlowModeQueriesDoNotCostUpdates)
{
  SlowModeLink link;
  CommandSlot slot;
  RuntimeParamsHandle params;
  StateCache cache;
  FakeSink sink;
  params.set(RuntimeParams{});
  Engine engine(link, slot, params, cache, sink, EngineConfig{}, rclcpp::get_logger("test_engine_mode"));
  engine.start();
  const auto real = EngineClock::real();
  const int64_t t_start = real.steady_ns();
  while (engine.phase() != Engine::Phase::kRunning && real.steady_ns() - t_start < 2000 * kMs) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_EQ(engine.phase(), Engine::Phase::kRunning);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  const auto s0 = engine.stats();
  const int q0 = link.mode_queries.load();
  const int64_t t0 = real.steady_ns();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  const auto s1 = engine.stats();
  const int q1 = link.mode_queries.load();
  const int64_t t1 = real.steady_ns();
  engine.stop();
  const double hz = static_cast<double>(s1.updates - s0.updates) / (static_cast<double>(t1 - t0) / 1e9);
  EXPECT_GE(hz, 45.0) << "one update per 20 ms tick (a query inside the loop caps this at ~41.7)";
  EXPECT_GE(q1 - q0, 7) << "~10 Hz mode queries on their own thread";
  EXPECT_LE(q1 - q0, 12);
  EXPECT_EQ(s1.mode_query_failures, 0U);
}

// ============================================================================
// 26.11.10 _ YSH _ 외란 주입 (엔진 수준) — 객체 프레임마다 ObjectDisturber 를 거친다. 손잡이가 없으면 예전 그대로,
//   있어도 걸린 것이 없으면 같은 메시지, 숨김 프레임도 같은 주기로 빈 목록, DAQ 로 가는 스냅샷은 건드리지 않는다.
//   난수·명령 규칙 자체는 test_object_disturbance.cpp 가 고정한다
// ============================================================================

constexpr uint64_t kDistSeed = 123456789012345ULL;

DisturbanceRequest dist_request(const std::string & trigger, uint64_t base_seed = 0)
{
  DisturbanceRequest r;
  r.trigger = trigger;
  r.run = "engine_test";
  r.base_seed = base_seed;
  r.source = "gtest";
  return r;
}

/// 20 ms 간격 표본 n 개를 넣고 돈다 (첫 표본 1.000 s)
void run_samples(Rig & rig, int n)
{
  for (int i = 0; i < n; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * static_cast<int64_t>(i)));
    rig.engine->step();
  }
}

std::vector<size_t> object_indices(const Rig & rig)
{
  std::vector<size_t> out;
  for (size_t i = 0; i < rig.sink.outputs.size(); ++i) {
    if (rig.sink.outputs[i].objects) {
      out.push_back(i);
    }
  }
  return out;
}

TEST(EngineDisturbance, NullHandleKeepsOldOutputs)
{
  Rig rig;
  rig.connect();
  run_samples(rig, 10);
  ASSERT_EQ(rig.sink.outputs.size(), 10U);
  for (const auto & o : rig.sink.outputs) {
    EXPECT_FALSE(o.disturbance.has_value());
  }
  EXPECT_EQ(object_indices(rig), (std::vector<size_t>{0, 5}));
  const auto & objects = *rig.sink.outputs[0].objects;
  ASSERT_EQ(objects.objects.size(), 1U);
  EXPECT_DOUBLE_EQ(objects.objects[0].kinematics.pose_with_covariance.pose.position.x, 110.0);
  EXPECT_DOUBLE_EQ(objects.objects[0].kinematics.pose_with_covariance.pose.position.y, 200.0);
}

TEST(EngineDisturbance, NothingActiveIsPassThrough)
{
  Rig plain;
  plain.connect();
  run_samples(plain, 10);

  Rig rig(EngineConfig{}, false, true);
  rig.connect();
  run_samples(rig, 10);

  ASSERT_EQ(rig.sink.outputs.size(), plain.sink.outputs.size());
  for (size_t i = 0; i < rig.sink.outputs.size(); ++i) {
    const auto & a = plain.sink.outputs[i];
    const auto & b = rig.sink.outputs[i];
    ASSERT_EQ(a.objects.has_value(), b.objects.has_value()) << i;
    // 외란 프레임은 objects 를 낸 업데이트에만 있다
    EXPECT_EQ(b.disturbance.has_value(), b.objects.has_value()) << i;
    if (a.objects) {
      EXPECT_TRUE(*a.objects == *b.objects) << "TrackedObjects must be identical when nothing is active, " << i;
      EXPECT_FALSE(b.disturbance->fault_flag);
      EXPECT_TRUE(b.disturbance->objects_visible);
      ASSERT_EQ(b.disturbance->sent_actors.size(), 1U);
      ASSERT_EQ(b.disturbance->objects.size(), 1U);
      EXPECT_EQ(b.disturbance->objects[0].name, "Car_1");
      EXPECT_EQ(b.disturbance->objects[0].dx_m, 0.0);
      EXPECT_EQ(b.disturbance->base_seed, 0U);
    }
  }
  EXPECT_EQ(object_indices(rig), (std::vector<size_t>{0, 5}));
  EXPECT_EQ(rig.engine->stats().objects_published, plain.engine->stats().objects_published);
}

TEST(EngineDisturbance, PositionShiftsTrackedObjectButNotTheDaqSnapshot)
{
  Rig rig(EngineConfig{}, false, true);
  DisturbanceController ctl(rig.disturbance, DisturbanceController::Options{});
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, position, 1, 0, 0, 0", kDistSeed)).ok);
  rig.connect();
  run_samples(rig, 6);
  ASSERT_EQ(rig.sink.outputs.size(), 6U);
  ASSERT_EQ(rig.sink.sample_snapshots.size(), 6U);
  for (const size_t i : {size_t{0}, size_t{5}}) {
    const auto & o = rig.sink.outputs[i];
    ASSERT_TRUE(o.objects.has_value());
    ASSERT_TRUE(o.disturbance.has_value());
    ASSERT_EQ(o.objects->objects.size(), 1U);
    // 자차 yaw 45° 에서 앞으로 1 m → (cos 45°, sin 45°)
    const auto & pose = o.objects->objects[0].kinematics.pose_with_covariance.pose;
    EXPECT_NEAR(pose.position.x, 110.70710678118655, 1e-9);
    EXPECT_NEAR(pose.position.y, 200.70710678118655, 1e-9);
    EXPECT_DOUBLE_EQ(pose.position.z, 1.0);
    EXPECT_EQ(o.objects->objects[0].object_id, morai_to_autoware::object_id_from_name("Car_1"));
    const auto & frame = *o.disturbance;
    EXPECT_TRUE(frame.position_on);
    EXPECT_TRUE(frame.fault_flag);
    EXPECT_EQ(frame.base_seed, kDistSeed);
    ASSERT_EQ(frame.objects.size(), 1U);
    EXPECT_EQ(frame.objects[0].seed, 0x4E6987CFFE11F9DBULL);
    EXPECT_EQ(frame.objects[0].dx_m, frame.sent_actors[0].position.x - 110.0);
    EXPECT_EQ(frame.objects[0].dy_m, frame.sent_actors[0].position.y - 200.0);
  }
  // DAQ 로 넘어간 스냅샷(GT)은 모든 표본에서 정확히 그대로다
  for (const auto & snapshot : rig.sink.sample_snapshots) {
    ASSERT_EQ(snapshot.actors.size(), 1U);
    EXPECT_EQ(snapshot.actors[0].position.x, 110.0);
    EXPECT_EQ(snapshot.actors[0].position.y, 200.0);
    EXPECT_EQ(snapshot.actors[0].position.z, 1.0);
  }
}

TEST(EngineDisturbance, HiddenFramesStillPublishAnEmptyListOnTheSameCadence)
{
  Rig plain;
  plain.connect();
  run_samples(plain, 50);

  Rig rig(EngineConfig{}, false, true);
  DisturbanceController ctl(rig.disturbance, DisturbanceController::Options{});
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, blackout, 0, 0", kDistSeed)).ok);
  rig.connect();
  run_samples(rig, 50);

  ASSERT_EQ(rig.sink.outputs.size(), 50U);
  const auto indices = object_indices(rig);
  EXPECT_EQ(indices, object_indices(plain)) << "hidden frames must not change the 10 Hz object cadence";
  EXPECT_EQ(indices.size(), 10U);
  for (const size_t i : indices) {
    const auto & o = rig.sink.outputs[i];
    EXPECT_TRUE(o.objects->objects.empty()) << i;
    EXPECT_EQ(o.objects->header.stamp, o.stamp);
    EXPECT_EQ(o.objects->header.frame_id, "map");
    ASSERT_TRUE(o.disturbance.has_value());
    EXPECT_FALSE(o.disturbance->objects_visible);
    EXPECT_TRUE(o.disturbance->blackout_on);
    EXPECT_TRUE(o.disturbance->sent_actors.empty());
    ASSERT_EQ(o.disturbance->objects.size(), 1U);  // GT 객체는 visible false 로 남는다
    EXPECT_FALSE(o.disturbance->objects[0].visible);
  }
  EXPECT_EQ(rig.engine->stats().objects_published, plain.engine->stats().objects_published);
  EXPECT_EQ(rig.engine->stats().objects_published, 10U);
  // 숨긴 동안에도 DAQ 스냅샷에는 객체가 있다
  for (const auto & snapshot : rig.sink.sample_snapshots) {
    EXPECT_EQ(snapshot.actors.size(), 1U);
  }
}

TEST(EngineDisturbance, FrameSeqAndPeriodFollowObjectFrames)
{
  Rig rig(EngineConfig{}, false, true);
  rig.connect();
  run_samples(rig, 30);
  ASSERT_EQ(rig.sink.outputs.size(), 30U);
  uint64_t expected_seq = 0;
  for (size_t i = 0; i < rig.sink.outputs.size(); ++i) {
    const auto & o = rig.sink.outputs[i];
    if (i % 5 != 0) {
      EXPECT_FALSE(o.objects.has_value()) << i;
      EXPECT_FALSE(o.disturbance.has_value()) << i;
      continue;
    }
    ASSERT_TRUE(o.disturbance.has_value()) << i;
    EXPECT_EQ(o.disturbance->frame_seq, expected_seq++) << i;
    EXPECT_DOUBLE_EQ(o.disturbance->frame_period_s, 0.1) << i;  // 20 ms 표본 격자 위 100 ms
  }
  EXPECT_EQ(expected_seq, 6U);
}

TEST(EngineDisturbance, CommandTakesEffectOnTheNextObjectFrame)
{
  Rig rig(EngineConfig{}, false, true);
  DisturbanceController ctl(rig.disturbance, DisturbanceController::Options{});
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, clear", kDistSeed)).ok);
  rig.connect();
  run_samples(rig, 2);  // 1.000 s (객체 프레임), 1.020 s
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, blackout, 0, 0")).ok);  // 두 객체 프레임 사이에 들어온 명령
  for (int i = 2; i < 11; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * static_cast<int64_t>(i)));
    rig.engine->step();
  }
  ASSERT_EQ(rig.sink.outputs.size(), 11U);
  EXPECT_EQ(object_indices(rig), (std::vector<size_t>{0, 5, 10}));
  // 명령 전 프레임은 그대로
  EXPECT_EQ(rig.sink.outputs[0].objects->objects.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[0].disturbance->blackout_on);
  EXPECT_EQ(rig.sink.outputs[0].disturbance->frame_seq, 0U);
  // 객체 프레임이 아닌 업데이트에는 아무 일도 없다
  for (size_t i = 1; i < 5; ++i) {
    EXPECT_FALSE(rig.sink.outputs[i].objects.has_value());
    EXPECT_FALSE(rig.sink.outputs[i].disturbance.has_value());
  }
  // 다음 객체 프레임부터 숨긴다
  EXPECT_TRUE(rig.sink.outputs[5].objects->objects.empty());
  EXPECT_TRUE(rig.sink.outputs[5].disturbance->blackout_on);
  EXPECT_EQ(rig.sink.outputs[5].disturbance->frame_seq, 1U);
  EXPECT_TRUE(rig.sink.outputs[10].objects->objects.empty());

  // 해제도 다음 객체 프레임에 먹는다
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, clear")).ok);
  for (int i = 11; i < 16; ++i) {
    rig.push(make_snapshot(1000000 + 20000 * static_cast<int64_t>(i)));
    rig.engine->step();
  }
  ASSERT_TRUE(rig.sink.outputs[15].objects.has_value());
  EXPECT_EQ(rig.sink.outputs[15].objects->objects.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[15].disturbance->fault_flag);
  EXPECT_EQ(rig.sink.outputs[15].disturbance->frame_seq, 3U);
}

// MORAI 시각이 뒤로 재기준돼도 (ObjectsRestartOnTheRebasedSample 의 시각열) blackout 은 프레임 수로 끝난다 —
//   역행한 프레임은 100 ms 로 센다
TEST(EngineDisturbance, RebaseKeepsBlackoutFinite)
{
  Rig rig(EngineConfig{}, false, true);
  DisturbanceController ctl(rig.disturbance, DisturbanceController::Options{});
  ASSERT_TRUE(ctl.handle(dist_request("disturbance, blackout, 0.3, 1", kDistSeed)).ok);
  rig.connect();
  std::vector<int64_t> stamps = {1000000, 1020000, 1040000, 890000, 910000, 930000, 950000};
  for (int64_t ts = 970000; ts <= 1500000; ts += 20000) {
    stamps.push_back(ts);
  }
  for (const int64_t ts : stamps) {
    rig.push(make_snapshot(ts));
    rig.engine->step();
  }
  ASSERT_GE(rig.engine->stats().rebased, 1U);
  std::vector<bool> visible;
  std::vector<double> periods;
  for (const auto & o : rig.sink.outputs) {
    if (o.disturbance) {
      visible.push_back(o.disturbance->objects_visible);
      periods.push_back(o.disturbance->frame_period_s);
      EXPECT_EQ(o.objects->objects.empty(), !o.disturbance->objects_visible);
    }
  }
  // 객체 프레임: 1.000 s (건 프레임), 재기준된 0.930 s (역행 → 100 ms), 1.030 s (200 ms), 1.130 s (300 ms → 복구)
  ASSERT_GE(visible.size(), 6U);
  EXPECT_EQ(std::vector<bool>(visible.begin(), visible.begin() + 5), (std::vector<bool>{false, false, false, true, true}));
  EXPECT_DOUBLE_EQ(periods[1], 0.1) << "a backward stamp counts as the nominal period";
  EXPECT_DOUBLE_EQ(periods[2], 0.1);
  for (size_t i = 3; i < visible.size(); ++i) {
    EXPECT_TRUE(visible[i]) << i;
  }
}

}  // namespace
}  // namespace autoware::morai_interface
