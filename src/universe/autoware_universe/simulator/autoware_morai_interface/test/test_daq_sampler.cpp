// 신규 — 26.09.26 _ YSH _ DAQ 샘플러 시험 (가짜 시계, 스레드 없이 step/tick) — 100 Hz ZOH·게이트·결손·명령 경과·요약
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

#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"
#include "autoware/morai_interface/daq_sampler.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>
// 26.09.26 _ YSH _ 원시 gRPC 기록 시험: 실제 ActorStates 바이트를 만든다 (vendor 가 설치한 메시지 헤더)
#include <morai/actor/actor_get.pb.h>
#include <morai/common/enum.pb.h>

#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace autoware::morai_interface
{
namespace
{

namespace mgv = morai_grpc_vendor;
constexpr int64_t kMs = 1000000LL;
constexpr int64_t kEpochNs = 1758000000000000000LL;

class FakeDaqSink : public DaqSink
{
public:
  std::vector<DaqOutputs> outputs;
  std::vector<DaqRunSummary> summaries;
  // 26.09.26 _ YSH _ 원시 기록 (publish_daq_raw 한 번 = 한 틱)
  std::vector<DaqRawOutputs> raws;
  void publish_daq(const DaqOutputs & o) override { outputs.push_back(o); }
  void on_daq_run_end(const DaqRunSummary & s) override { summaries.push_back(s); }
  void publish_daq_raw(const DaqRawOutputs & o) override { raws.push_back(o); }

  std::vector<std::string> raw_actor_states() const
  {
    std::vector<std::string> v;
    for (const auto & r : raws) {
      v.insert(v.end(), r.actor_states.begin(), r.actor_states.end());
    }
    return v;
  }
  std::vector<std::string> raw_commands() const
  {
    std::vector<std::string> v;
    for (const auto & r : raws) {
      v.insert(v.end(), r.vehicle_ctrl_cmd.begin(), r.vehicle_ctrl_cmd.end());
    }
    return v;
  }

  size_t states() const
  {
    size_t n = 0;
    for (const auto & o : outputs) {
      n += o.ego ? 1 : 0;  // 26.10.05 _ YSH _ 원문: n += o.state ? 1 : 0;
    }
    return n;
  }
  size_t cmds() const
  {
    size_t n = 0;
    for (const auto & o : outputs) {
      n += o.cmd ? 1 : 0;
    }
    return n;
  }
};

mgv::Snapshot make_snapshot(double vx_kmh = 36.0, size_t actors = 1)
{
  mgv::Snapshot s;
  s.ego.found = true;
  s.ego.valid = true;
  s.ego.position = mgv::Vec3{100.0, 50.0, 1.0};
  s.ego.rotation_deg = mgv::Vec3{0.0, 0.0, 90.0};
  s.ego.velocity_kmh = mgv::Vec3{vx_kmh, 0.0, 0.0};
  s.ego.acceleration_raw = mgv::Vec3{0.8, 0.1, 0.0};
  for (size_t i = 0; i < actors; ++i) {
    mgv::ActorSample a;
    a.id = "NPC_Car_" + std::to_string(i + 1);
    a.object_type = mgv::ObjectType::kVehicle;
    a.position = mgv::Vec3{120.0 + static_cast<double>(i), 50.0, 1.0};
    a.acceleration_raw = mgv::Vec3{0.3, 0.0, 0.0};
    a.bbox_extent = mgv::Vec3{1.0, 2.3, 0.7};
    s.actors.push_back(a);
  }
  return s;
}

autoware_control_msgs::msg::Control make_cmd(float accel, int32_t sec)
{
  autoware_control_msgs::msg::Control c;
  c.stamp.sec = sec;
  c.longitudinal.acceleration = accel;
  c.longitudinal.velocity = 5.0F;
  return c;
}

struct Rig
{
  int64_t now_ns{1000 * kMs};
  DaqFrameCache cache;
  DaqCommandLatch latch;
  RuntimeParamsHandle params;
  FakeDaqSink sink;
  bool link_up{true};
  uint64_t update_index{0};
  std::unique_ptr<DaqSampler> sampler;

  explicit Rig(DaqMode mode = DaqMode::kGated, DaqConfig cfg = DaqConfig{})
  {
    params.set(RuntimeParams{});
    EngineClock clock;
    clock.steady_ns = [this]() { return now_ns; };
    clock.system_ns = [this]() { return kEpochNs + now_ns; };
    clock.sleep_ns = [this](int64_t ns) { now_ns += ns; };
    sampler = std::make_unique<DaqSampler>(
      cache, latch, params, sink, cfg, mode, rclcpp::get_logger("test_daq"), clock,
      [this]() { return link_up; });
  }

  /// 엔진이 표본 하나를 받아들인 것처럼 (stamp = MORAI 시각 흉내)
  void feed(const mgv::Snapshot & s = make_snapshot())
  {
    auto f = std::make_shared<DaqFrame>();
    f->snapshot = s;
    f->stamp = morai_to_autoware::to_time_msg(kEpochNs + now_ns - 2 * kMs);
    f->update_index = update_index++;
    f->receive_steady_ns = now_ns;
    cache.store(f);
  }

  /// duration 동안: MORAI 업데이트는 update_every 마다, control_cmd 는 cmd_every 마다(0 = 안 넣음), 나머지는 step
  void run(int64_t duration_ns, int64_t update_every = 20 * kMs, int64_t cmd_every = 30 * kMs)
  {
    const int64_t end = now_ns + duration_ns;
    int64_t next_update = now_ns;
    int64_t next_cmd = now_ns;
    while (now_ns < end) {
      if (update_every > 0 && now_ns >= next_update) {
        feed();
        next_update += update_every;
      }
      if (cmd_every > 0 && now_ns >= next_cmd) {
        latch.submit(make_cmd(0.5F, static_cast<int32_t>(now_ns / 1000000000LL)), now_ns);
        next_cmd += cmd_every;
      }
      // tick 이면 시계는 그대로, 아니면 다음 DAQ deadline 까지 잔다 (업데이트·명령 주기는 10 ms 의 배수)
      sampler->step();
    }
  }
};

TEST(DaqSampler, ParsesRunStateMessages)
{
  const auto a = parse_daq_run_state("active scenario_1_1");
  ASSERT_TRUE(a.has_value());
  EXPECT_TRUE(a->first);
  EXPECT_EQ(a->second, "scenario_1_1");
  const auto b = parse_daq_run_state("  idle scenario_1_1 \n");
  ASSERT_TRUE(b.has_value());
  EXPECT_FALSE(b->first);
  EXPECT_EQ(b->second, "scenario_1_1");
  const auto c = parse_daq_run_state("idle");
  ASSERT_TRUE(c.has_value());
  EXPECT_FALSE(c->first);
  EXPECT_TRUE(c->second.empty());
  EXPECT_FALSE(parse_daq_run_state("active").has_value()) << "active needs a run name";
  EXPECT_FALSE(parse_daq_run_state("start r1").has_value());
  EXPECT_FALSE(parse_daq_run_state("").has_value());
  ASSERT_TRUE(parse_daq_mode("gated").has_value());
  EXPECT_TRUE(*parse_daq_mode("gated") == DaqMode::kGated);
  EXPECT_TRUE(*parse_daq_mode("always") == DaqMode::kAlways);
  EXPECT_FALSE(parse_daq_mode("on").has_value());
  EXPECT_TRUE(*parse_daq_stamp("tick") == DaqStamp::kTick);
  EXPECT_FALSE(daq_rate_is_valid(5.0));
  EXPECT_TRUE(daq_rate_is_valid(100.0));
}

TEST(DaqSampler, GatedPublishesNothingUntilARunStarts)
{
  Rig rig;
  rig.run(500 * kMs);
  EXPECT_TRUE(rig.sink.outputs.empty());
  EXPECT_FALSE(rig.sampler->publishing());
  EXPECT_EQ(rig.sampler->stats().gap_ticks, 0U);
}

TEST(DaqSampler, FiftyHzUpdatesArePublishedAtOneHundredHzWithTheSampleStamp)
{
  Rig rig;
  rig.sampler->set_run_state(true, "r1");
  rig.run(1000 * kMs);
  ASSERT_GE(rig.sink.states(), 99U);
  ASSERT_LE(rig.sink.states(), 101U);
  // 업데이트 하나가 두 번씩 (ZOH), 반복 행은 같은 stamp
  std::map<std::pair<int32_t, uint32_t>, int> per_stamp;
  for (const auto & o : rig.sink.outputs) {
    // 26.10.05 _ YSH _ MORAI 메시지 (원문: state·accel·objects 셋의 stamp 가 같다)
    ASSERT_TRUE(o.ego && o.objects);
    EXPECT_EQ(o.ego->header.stamp, o.objects->header.stamp);
    ++per_stamp[{o.ego->header.stamp.sec, o.ego->header.stamp.nanosec}];
  }
  EXPECT_NEAR(static_cast<double>(per_stamp.size()), 50.0, 1.0);
  for (const auto & kv : per_stamp) {
    EXPECT_LE(kv.second, 2) << "never more than 2 rows per 20 ms update";
  }
  const auto & o = rig.sink.outputs.back();
  // 26.10.05 _ YSH _ MORAI 메시지 (원문: state frame map·twist.x 10, accel frame base_link·x 0.8, 객체 가속도 x 0.3)
  EXPECT_EQ(o.ego->header.frame_id, "map");
  EXPECT_DOUBLE_EQ(o.ego->velocity.x, 10.0);
  EXPECT_DOUBLE_EQ(o.ego->acceleration.x, 0.8);
  EXPECT_DOUBLE_EQ(o.ego->acceleration.y, 0.1) << "MORAI axes (no lateral flip)";
  ASSERT_EQ(o.objects->npc_list.size(), 1U);
  EXPECT_EQ(o.objects->num_of_npcs, 1);
  EXPECT_DOUBLE_EQ(o.objects->npc_list[0].acceleration.x, 0.3);
}

TEST(DaqSampler, IrregularMoraiUpdatesStillGiveOneRowPerTenMs)
{
  Rig rig(DaqMode::kAlways);
  // MORAI 실측 꼴: 20, 1, 19, 40 ms
  const int64_t steps[] = {20 * kMs, 1 * kMs, 19 * kMs, 40 * kMs};
  const int64_t end = rig.now_ns + 2000 * kMs;
  int64_t next_update = rig.now_ns;
  int i = 0;
  while (rig.now_ns < end) {
    if (rig.now_ns >= next_update) {
      rig.feed();
      next_update += steps[i++ % 4];
    }
    rig.sampler->step();
  }
  EXPECT_NEAR(static_cast<double>(rig.sink.states()), 200.0, 2.0);
  EXPECT_EQ(rig.sampler->stats().gap_ticks, 0U) << "a 40 ms hole is well under max_sample_age_s";
}

TEST(DaqSampler, StaleSampleStopsStateTopicsAndCountsGaps)
{
  Rig rig;
  rig.sampler->set_run_state(true, "r1");
  rig.feed();
  rig.sampler->tick(rig.now_ns + 99 * kMs);
  ASSERT_EQ(rig.sink.states(), 1U) << "99 ms old is still fresh";
  rig.sampler->tick(rig.now_ns + 100 * kMs);
  EXPECT_EQ(rig.sink.states(), 1U) << "exactly max_sample_age_s is stale (daq_publisher: age < max)";
  rig.link_up = false;
  rig.sampler->tick(rig.now_ns + 150 * kMs);
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.gap_ticks, 2U);
  EXPECT_EQ(st.link_down_ticks, 1U);
  rig.now_ns += 200 * kMs;
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  EXPECT_EQ(rig.sink.states(), 2U) << "publishing resumes on the next sample";
}

TEST(DaqSampler, CommandIsAVerbatimCopyThatStopsWhenAutowareGoesQuiet)
{
  Rig rig;
  rig.sampler->set_run_state(true, "r1");
  rig.feed();
  auto cmd = make_cmd(std::nanf(""), 1234);  // NaN 도 그대로 (Autoware 가 낸 것)
  rig.latch.submit(cmd, rig.now_ns);
  rig.sampler->tick(rig.now_ns + 200 * kMs);
  ASSERT_EQ(rig.sink.cmds(), 1U) << "exactly cmd_stale_s old is still published";
  const auto & out = *rig.sink.outputs.back().cmd;
  EXPECT_EQ(out.stamp.sec, 1234) << "the original control_cmd stamp is kept";
  EXPECT_TRUE(std::isnan(out.longitudinal.acceleration));
  rig.sampler->tick(rig.now_ns + 201 * kMs);
  EXPECT_EQ(rig.sink.cmds(), 1U) << "no zero is invented after cmd_stale_s";
  rig.latch.submit(make_cmd(0.2F, 1235), rig.now_ns + 300 * kMs);
  rig.sampler->tick(rig.now_ns + 300 * kMs);
  EXPECT_EQ(rig.sink.cmds(), 2U);
}

// 26.10.06 _ YSH _ 명세 25 (gRPC): 보낸 명령이 ctrl 로 나가고, cmd_stale_s 넘게 새 송신이 없으면 멈춘다
TEST(DaqSampler, SentGrpcCommandIsPublishedAsCtrlAndStopsWhenStale)
{
  Rig rig(DaqMode::kAlways);
  rig.feed();
  mgv::VehicleCommand c;
  c.velocity = 36.0;
  c.acceleration = 0.5;
  rig.sampler->submit_sent_command(c, kEpochNs + 123);
  rig.sampler->tick(rig.now_ns);
  ASSERT_TRUE(rig.sink.outputs.back().ctrl.has_value());
  EXPECT_EQ(rig.sink.outputs.back().ctrl->velocity, 36.0);
  EXPECT_EQ(rig.sink.outputs.back().ctrl->acceleration, 0.5);
  EXPECT_EQ(rig.sink.outputs.back().ctrl->header.stamp, morai_to_autoware::to_time_msg(kEpochNs + 123));
  rig.feed();
  rig.now_ns += 250 * kMs;  // 새 송신 없이 0.25 s
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  EXPECT_FALSE(rig.sink.outputs.back().ctrl.has_value());
}

// 26.10.12 _ YSH _ 의사 VILS: ego_status 는 실차 함수 값 (MORAI 표본이 낡아도), 명세 25 는 Autoware 명령
TEST(DaqSampler, FollowModeTakesEgoFromTheRealCarAndCtrlFromTheCommand)
{
  DaqConfig cfg;
  cfg.ctrl_from_command = true;
  Rig rig(DaqMode::kAlways, cfg);
  bool real_ok = true;
  rig.sampler->set_ego_override([&]() -> std::optional<morai_ros2_msgs::msg::EgoVehicleStatus> {
    if (!real_ok) {
      return std::nullopt;
    }
    morai_ros2_msgs::msg::EgoVehicleStatus e;
    e.velocity.x = 7.0;
    e.header.stamp.sec = 77;
    return e;
  });
  rig.feed();  // MORAI Ego 는 36 km/h 지만 쓰지 않는다
  rig.latch.submit(make_cmd(0.4F, 5), rig.now_ns);
  mgv::VehicleCommand ghost;
  ghost.long_cmd_type = mgv::LongCmdType::kGhost;
  rig.sampler->submit_sent_command(ghost, kEpochNs);  // 고스트 명령은 명세 25 가 아니다
  rig.sampler->tick(rig.now_ns);
  const auto & o = rig.sink.outputs.back();
  ASSERT_TRUE(o.ego.has_value());
  EXPECT_EQ(o.ego->velocity.x, 7.0);
  EXPECT_EQ(o.ego->header.stamp.sec, 77) << "sample stamp = the real-car sample";
  EXPECT_TRUE(o.objects.has_value()) << "objects still come from MORAI";
  ASSERT_TRUE(o.ctrl.has_value());
  EXPECT_EQ(o.ctrl->longl_cmd_type, 3);
  EXPECT_NEAR(o.ctrl->acceleration, 0.4, 1e-6);
  EXPECT_EQ(o.ctrl->header.stamp.sec, 5);
  // MORAI 표본이 낡아도 실차 Ego 는 나간다 (객체는 멈춘다)
  rig.now_ns += 500 * kMs;
  rig.sampler->tick(rig.now_ns);
  EXPECT_TRUE(rig.sink.outputs.back().ego.has_value());
  EXPECT_FALSE(rig.sink.outputs.back().objects.has_value());
  // 실차 측위가 끊기면 ego 를 내지 않는다
  real_ok = false;
  rig.latch.submit(make_cmd(0.4F, 6), rig.now_ns);
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  EXPECT_FALSE(rig.sink.outputs.back().ego.has_value());
}

TEST(DaqSampler, CommandIsPublishedWhileTheStateIsStale)
{
  Rig rig(DaqMode::kAlways);
  rig.latch.submit(make_cmd(0.1F, 1), rig.now_ns);
  rig.sampler->tick(rig.now_ns);  // 표본 없음
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[0].ego.has_value());  // 26.10.05 _ YSH _ 원문: state
  EXPECT_TRUE(rig.sink.outputs[0].cmd.has_value());
}

TEST(DaqSampler, TickStampModeUsesThePublishTime)
{
  DaqConfig cfg;
  cfg.stamp = DaqStamp::kTick;
  Rig rig(DaqMode::kAlways, cfg);
  rig.feed();
  rig.latch.submit(make_cmd(0.1F, 7), rig.now_ns);
  rig.now_ns += 5 * kMs;  // 발행 시각 = 그 순간의 시스템 시계
  rig.sampler->tick(rig.now_ns);
  const auto expect = morai_to_autoware::to_time_msg(kEpochNs + rig.now_ns);
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_EQ(rig.sink.outputs[0].ego->header.stamp, expect);  // 26.10.05 _ YSH _ 원문: state
  EXPECT_EQ(rig.sink.outputs[0].cmd->stamp, expect);
}

TEST(DaqSampler, EmptyActorListStillPublishesAnEmptyObjectsMessage)
{
  Rig rig(DaqMode::kAlways);
  rig.feed(make_snapshot(36.0, 0));
  rig.sampler->tick(rig.now_ns);
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  ASSERT_TRUE(rig.sink.outputs[0].objects.has_value());
  // 26.10.05 _ YSH _ 원문: EXPECT_TRUE(rig.sink.outputs[0].objects->objects.empty());
  EXPECT_TRUE(rig.sink.outputs[0].objects->npc_list.empty());
  EXPECT_TRUE(rig.sink.outputs[0].objects->pedestrian_list.empty());
  EXPECT_TRUE(rig.sink.outputs[0].objects->obstacle_list.empty());
  EXPECT_EQ(rig.sink.outputs[0].objects->num_of_npcs, 0);
  EXPECT_EQ(rig.sink.outputs[0].objects->header.frame_id, "map");
}

TEST(DaqSampler, LateWakeUpDoesNotBurst)
{
  Rig rig(DaqMode::kAlways);
  rig.feed();
  rig.sampler->step();  // 첫 deadline = 지금 → tick
  ASSERT_EQ(rig.sink.states(), 1U);
  rig.now_ns += 55 * kMs;  // 55 ms 늦게 깨어남 (표본은 55 ms 낡음 — 아직 신선)
  rig.sampler->step();
  EXPECT_EQ(rig.sink.states(), 2U) << "one row, not five";
  EXPECT_EQ(rig.sampler->stats().late, 1U);
  const int64_t t_late = rig.now_ns;
  rig.sampler->step();  // 다음 deadline 은 늦은 행 + 한 주기 → 10 ms 자고 돌아온다 (바로 붙여 내지 않는다)
  EXPECT_EQ(rig.sink.states(), 2U);
  EXPECT_EQ(rig.now_ns, t_late + 10 * kMs);
  rig.sampler->step();
  EXPECT_EQ(rig.sink.states(), 3U);
}

// 26.10.05 _ YSH _ MORAI 메시지는 map_origin 을 쓰지 않는다 (MORAI world 그대로). 런타임 파라미터 반영은 steer_input_norm_deg 로 본다.
//   원래 시험 (MapOriginChangeTakesEffectWithoutANewSample): map_origin.x += 10 → state position.x += 10
TEST(DaqSampler, RuntimeParamChangeTakesEffectWithoutANewSample)
{
  Rig rig(DaqMode::kAlways);
  auto snap = make_snapshot();
  snap.ego.vehicle_state.present = true;
  snap.ego.vehicle_state.front_wheel_angle = 0.25;
  rig.feed(snap);
  rig.sampler->tick(rig.now_ns);
  const auto & first = *rig.sink.outputs.back().ego;
  const double x0 = first.position.x;
  EXPECT_DOUBLE_EQ(first.front_steer_angle, -10.0) << "-0.25 x 40 deg (left +)";
  RuntimeParams rp = *rig.params.get();
  rp.map_origin.x = 10.0;
  rp.steer_input_norm_deg = 20.0;
  rig.params.set(rp);
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  EXPECT_DOUBLE_EQ(rig.sink.outputs.back().ego->front_steer_angle, -5.0);
  EXPECT_DOUBLE_EQ(rig.sink.outputs.back().ego->position.x, x0) << "MORAI world, map_origin not applied";
}

TEST(DaqSampler, RunSummaryCountsTheRunAndSameRunRedeliveryIsIgnored)
{
  Rig rig;
  rig.sampler->set_run_state(true, "scenario_1_1");
  rig.run(1000 * kMs);
  rig.sampler->set_run_state(true, "scenario_1_1");  // latched 재수신 → 같은 런 이어 감
  rig.run(1000 * kMs);
  EXPECT_TRUE(rig.sink.summaries.empty());
  rig.sampler->set_run_state(false, "scenario_1_1");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  const auto & s = rig.sink.summaries[0];
  EXPECT_EQ(s.run, "scenario_1_1");
  EXPECT_EQ(s.end_reason, "idle");
  EXPECT_NEAR(s.duration_s, 2.0, 0.02);
  EXPECT_NEAR(static_cast<double>(s.state_published), 200.0, 2.0);
  EXPECT_NEAR(s.state_hz(), 100.0, 1.0);
  EXPECT_NEAR(static_cast<double>(s.updates), 100.0, 2.0);
  EXPECT_GT(s.cmd_published, 190U);
  EXPECT_EQ(s.gap_ticks, 0U);
  const std::string json = to_json(s);
  EXPECT_NE(json.find("\"run\": \"scenario_1_1\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"end_reason\": \"idle\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"state_hz\": "), std::string::npos) << json;
  // 끈 뒤에는 내지 않는다
  const size_t before = rig.sink.outputs.size();
  rig.run(300 * kMs);
  EXPECT_EQ(rig.sink.outputs.size(), before);
}

TEST(DaqSampler, NewRunReplacesAnUnfinishedOne)
{
  Rig rig;
  rig.sampler->set_run_state(true, "r1");
  rig.run(100 * kMs);
  rig.sampler->set_run_state(true, "r2");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].run, "r1");
  EXPECT_EQ(rig.sink.summaries[0].end_reason, "replaced");
  EXPECT_EQ(rig.sampler->stats().run, "r2");
}

// 26.09.26 _ YSH _ 리뷰 반영: 선점이 겹쳐 'active B' 뒤에 'idle A' 가 오면 B 는 계속된다. 이름 없는 idle 은 무엇이든 끝낸다
TEST(DaqSampler, LateIdleOfAPreviousRunDoesNotEndTheNextRun)
{
  Rig rig;
  rig.sampler->set_run_state(true, "A");
  rig.run(50 * kMs);
  rig.sampler->set_run_state(true, "B");  // A → replaced
  rig.sampler->set_run_state(false, "A");  // 늦게 온 A 의 idle
  EXPECT_TRUE(rig.sampler->publishing());
  EXPECT_EQ(rig.sampler->stats().run, "B");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].end_reason, "replaced");
  rig.sampler->set_run_state(false, "B");
  ASSERT_EQ(rig.sink.summaries.size(), 2U);
  EXPECT_EQ(rig.sink.summaries[1].run, "B");
  EXPECT_EQ(rig.sink.summaries[1].end_reason, "idle");
  rig.sampler->set_run_state(true, "C");
  rig.sampler->set_run_state(false, "");  // 이름 없는 idle
  EXPECT_FALSE(rig.sampler->publishing());
}

TEST(DaqSampler, ActiveAfterStopDoesNotReopenARun)
{
  Rig rig;
  rig.sampler->stop();
  rig.sampler->set_run_state(true, "late");
  EXPECT_FALSE(rig.sampler->stats().active);
}

TEST(DaqSampler, RunEndsItselfAfterTheTimeout)
{
  DaqConfig cfg;
  cfg.run_timeout_s = 1.0;
  Rig rig(DaqMode::kGated, cfg);
  rig.sampler->set_run_state(true, "orphan");
  rig.run(1500 * kMs);
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].end_reason, "timeout");
  EXPECT_FALSE(rig.sampler->stats().active);
  EXPECT_EQ(rig.sampler->stats().run_timeouts, 1U);
}

TEST(DaqSampler, GapsInsideARunAreCountedEvenBeforeTheFirstSample)
{
  Rig rig;
  rig.sampler->set_run_state(true, "r1");
  rig.run(100 * kMs, 0, 0);  // 표본·명령 없음
  const auto st = rig.sampler->stats();
  EXPECT_GE(st.gap_ticks, 9U);
  rig.sampler->set_run_state(false, "r1");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_GE(rig.sink.summaries[0].longest_gap_ticks, 9U);
}

TEST(DaqSampler, YieldingSuppressesAllOutput)
{
  Rig rig(DaqMode::kAlways);
  rig.sampler->set_yield(true);
  rig.run(200 * kMs);
  EXPECT_TRUE(rig.sink.outputs.empty());
  rig.sampler->set_yield(false);
  rig.run(100 * kMs);
  EXPECT_FALSE(rig.sink.outputs.empty());
}

TEST(DaqSampler, YieldInsideARunIsReportedInTheSummary)
{
  Rig rig;
  rig.sampler->yield_until_first_check();  // 노드 기동 직후 첫 writer 점검 전과 같다
  EXPECT_TRUE(rig.sampler->yield_pending_first_check());
  rig.sampler->set_run_state(true, "r1");
  rig.run(100 * kMs);
  rig.sampler->set_yield(false);
  rig.run(100 * kMs);
  rig.sampler->set_run_state(false, "r1");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].yield_ticks, 10U);
  EXPECT_EQ(rig.sink.summaries[0].state_published, 10U);
  EXPECT_NE(to_json(rig.sink.summaries[0]).find("\"yield_ticks\": 10"), std::string::npos);
}

// 26.10.07 _ YSH _ /daq/collision 은 ego_status 와 같은 틱·같은 stamp 로, 충돌이 없어도 빈 목록으로 나간다
TEST(DaqSampler, CollisionIsPublishedWithTheEgoStampAndCanBeTurnedOff)
{
  {
    Rig rig(DaqMode::kAlways);
    auto snap = make_snapshot();
    rig.feed(snap);
    rig.sampler->tick(rig.now_ns);
    ASSERT_EQ(rig.sink.outputs.size(), 1U);
    const auto & o = rig.sink.outputs[0];
    ASSERT_TRUE(o.ego && o.collision);
    EXPECT_EQ(o.collision->header.stamp, o.ego->header.stamp);
    EXPECT_TRUE(o.collision->collision_object.empty());
    snap.ego.vehicle_state.collision_objects = {"NPC_Car_1"};
    rig.now_ns += 10 * kMs;
    rig.feed(snap);
    rig.sampler->tick(rig.now_ns);
    ASSERT_EQ(rig.sink.outputs.size(), 2U);
    ASSERT_EQ(rig.sink.outputs[1].collision->collision_object.size(), 1U);
    EXPECT_EQ(rig.sink.outputs[1].collision->collision_object[0].name, "NPC_Car_1");
    EXPECT_EQ(rig.sink.outputs[1].collision->collision_object[0].type, 1);
  }
  DaqConfig cfg;
  cfg.publish_collision = false;
  Rig rig(DaqMode::kAlways, cfg);
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[0].collision.has_value());
}

TEST(DaqSampler, ObjectsCanBeTurnedOff)
{
  DaqConfig cfg;
  cfg.publish_objects = false;
  Rig rig(DaqMode::kAlways, cfg);
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  ASSERT_EQ(rig.sink.outputs.size(), 1U);
  EXPECT_FALSE(rig.sink.outputs[0].objects.has_value());
}

TEST(DaqSampler, RealThreadPublishesNearTheRateAndStopEndsTheRun)
{
  DaqFrameCache cache;
  DaqCommandLatch latch;
  RuntimeParamsHandle params;
  params.set(RuntimeParams{});
  FakeDaqSink sink;
  DaqSampler sampler(
    cache, latch, params, sink, DaqConfig{}, DaqMode::kGated, rclcpp::get_logger("test_daq_thread"));
  const auto real = EngineClock::real();
  sampler.set_run_state(true, "live");
  sampler.start();
  const int64_t t0 = real.steady_ns();
  while (real.steady_ns() - t0 < 500 * kMs) {
    auto f = std::make_shared<DaqFrame>();
    f->snapshot = make_snapshot();
    f->receive_steady_ns = real.steady_ns();
    cache.store(f);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  sampler.stop();  // 켜진 런 → shutdown 요약 (sink 는 스레드가 멈춘 뒤라 경쟁 없음)
  EXPECT_GE(sink.states(), 40U);
  EXPECT_LE(sink.states(), 55U);
  ASSERT_EQ(sink.summaries.size(), 1U);
  EXPECT_EQ(sink.summaries[0].end_reason, "shutdown");
}

// =============================================================================
// 26.09.26 _ YSH _ 원시 gRPC 기록 (daq.raw)
// =============================================================================

DaqConfig raw_config(size_t depth = 256)
{
  DaqConfig cfg;
  cfg.raw = true;
  cfg.raw_queue_depth = depth;
  cfg.raw_link.ego_id = "Ego";
  return cfg;
}

/// 실측 크기의 timestamp 와 Ego 하나가 든 ActorStates 응답 바이트 (링크가 Snapshot::raw_actor_states 에 싣는 것)
std::shared_ptr<const std::string> actor_states_bytes(int64_t timestamp_us, double x)
{
  morai_sim_api::actor::ActorStates s;
  s.set_timestamp(timestamp_us);
  auto * ego = s.add_states();
  ego->mutable_actor_info()->mutable_id()->set_value("Ego");
  ego->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
  ego->mutable_transform()->mutable_location()->set_x(x);
  ego->mutable_velocity()->set_x(36.0);
  auto out = std::make_shared<std::string>();
  s.SerializeToString(out.get());
  return out;
}

DaqRawActorStates raw_update(uint64_t update_index, double x = 0.1)
{
  DaqRawActorStates r;
  r.update_index = update_index;
  r.morai_timestamp_us = 1790000000000000 + static_cast<int64_t>(update_index) * 20000;
  r.rx_system_ns = r.morai_timestamp_us * 1000 + 3 * kMs;
  r.bytes = actor_states_bytes(r.morai_timestamp_us, x);
  return r;
}

DaqRawCommand raw_command(uint64_t seq, bool stale = false)
{
  DaqRawCommand c;
  c.cmd.long_cmd_type = mgv::LongCmdType::kAcceleration;
  c.cmd.steer = 0.1;
  c.cmd.acceleration = stale ? -2.4 : 0.5;
  c.cmd.velocity = 36.0;
  c.send_seq = seq;
  c.send_system_ns = kEpochNs + static_cast<int64_t>(seq) * 10 * kMs;
  c.stale = stale;
  c.ok = true;
  c.grpc_code = 0;
  c.rpc_ms = 0.25;
  c.result_status = 1;  // 26.09.26 _ YSH _ 리뷰 반영: MORAI Result STATUS_CODE_SUCCESS
  return c;
}

bool has(const std::string & json, const std::string & needle)
{
  return json.find(needle) != std::string::npos;
}

TEST(DaqSamplerRaw, TwoUpdatesInsideOneTickAreBothPublished)
{
  Rig rig(DaqMode::kGated, raw_config());
  rig.sampler->set_run_state(true, "r1");
  rig.feed();
  rig.sampler->tick(rig.now_ns);  // 첫 틱: 아직 원시 기록 없음 → publish_daq_raw 를 부르지 않는다
  EXPECT_TRUE(rig.sink.raws.empty());
  // 한 틱(10 ms) 사이에 MORAI 업데이트 둘 + 명령 하나 — ZOH 가 아니라 둘 다 나가야 한다
  rig.sampler->submit_raw_actor_states(raw_update(7, 0.1));
  rig.sampler->submit_raw_command(raw_command(1));
  rig.sampler->submit_raw_actor_states(raw_update(8, 0.2));
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  ASSERT_EQ(rig.sink.raws.size(), 1U) << "one publish_daq_raw per tick";
  const auto states = rig.sink.raw_actor_states();
  ASSERT_EQ(states.size(), 2U);
  EXPECT_TRUE(has(states[0], "\"update_index\":7,")) << states[0];
  EXPECT_TRUE(has(states[1], "\"update_index\":8,")) << states[1];
  // 봉투 (계약 morai_raw.actor_states/1) + 응답 전체의 proto3 JSON
  const std::string & j = states[0];
  EXPECT_EQ(j.rfind("{\"schema\":\"morai_raw.actor_states/1\",\"run\":\"r1\",", 0), 0U) << j;
  EXPECT_TRUE(has(j, "\"rx_system_ns\":1790000000143000000,")) << j;
  EXPECT_TRUE(has(j, "\"morai_timestamp_us\":1790000000140000,")) << j;
  EXPECT_TRUE(has(j, "\"actor_states\":{\"states\":[{\"actor_info\":{\"id\":{\"value\":\"Ego\"},\"object_type\":1"))
    << j;
  EXPECT_TRUE(has(j, "\"timestamp\":\"1790000000140000\"")) << j;  // int64 는 문자열
  EXPECT_TRUE(has(j, "\"location\":{\"x\":0.1,")) << j;
  EXPECT_EQ(j.back(), '}');
  EXPECT_EQ(j.find('\n'), std::string::npos);
  const auto cmds = rig.sink.raw_commands();
  ASSERT_EQ(cmds.size(), 1U);
  EXPECT_TRUE(has(cmds[0], "\"run\":\"r1\",\"send_seq\":1,")) << cmds[0];
  // 명세 25 원시 기록: 이 틱의 /daq/* 상태 행은 여전히 하나 (ZOH 는 그대로)
  EXPECT_EQ(rig.sink.states(), 2U);
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_published, 2U);
  EXPECT_EQ(st.tx_published, 1U);
  rig.sampler->set_run_state(false, "r1");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].raw_published, 2U);
  EXPECT_EQ(rig.sink.summaries[0].tx_published, 1U);
}

TEST(DaqSamplerRaw, IdleDropsAndNothingIsCarriedIntoTheNextRun)
{
  Rig rig(DaqMode::kGated, raw_config());
  // 런 전: 넣는 순간 버린다
  rig.sampler->submit_raw_actor_states(raw_update(1));
  rig.sampler->submit_raw_command(raw_command(1));
  rig.sampler->tick(rig.now_ns);
  EXPECT_TRUE(rig.sink.raws.empty());
  // 런 안에서 넣었지만 틱 전에 idle → 다음 틱이 버린다 (큐를 비운다)
  rig.sampler->set_run_state(true, "r1");
  rig.sampler->submit_raw_actor_states(raw_update(2));
  rig.sampler->submit_raw_command(raw_command(2));
  rig.sampler->set_run_state(false, "r1");
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  EXPECT_TRUE(rig.sink.raws.empty());
  auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_discarded, 2U);
  EXPECT_EQ(st.tx_discarded, 2U);
  EXPECT_EQ(st.raw_published + st.tx_published, 0U);
  EXPECT_EQ(st.raw_dropped + st.tx_dropped, 0U) << "gate discards are not overflow losses";
  // 다음 런: 앞에서 버린 것은 나오지 않고, 새로 넣은 것만
  rig.sampler->set_run_state(true, "r2");
  rig.sampler->tick(rig.now_ns + 20 * kMs);
  EXPECT_TRUE(rig.sink.raws.empty());
  rig.sampler->submit_raw_actor_states(raw_update(3));
  rig.sampler->tick(rig.now_ns + 30 * kMs);
  const auto states = rig.sink.raw_actor_states();
  ASSERT_EQ(states.size(), 1U);
  EXPECT_TRUE(has(states[0], "\"run\":\"r2\",\"update_index\":3,")) << states[0];
}

TEST(DaqSamplerRaw, QueueOverflowDropsTheOldestAndCountsIt)
{
  Rig rig(DaqMode::kGated, raw_config(3));
  rig.sampler->set_run_state(true, "r1");
  for (uint64_t i = 0; i < 5; ++i) {
    rig.sampler->submit_raw_actor_states(raw_update(i));
    rig.sampler->submit_raw_command(raw_command(i + 1));
  }
  rig.sampler->tick(rig.now_ns);
  const auto states = rig.sink.raw_actor_states();
  ASSERT_EQ(states.size(), 3U) << "capacity 3 — the two oldest were dropped";
  EXPECT_TRUE(has(states[0], "\"update_index\":2,"));
  EXPECT_TRUE(has(states[2], "\"update_index\":4,"));
  const auto cmds = rig.sink.raw_commands();
  ASSERT_EQ(cmds.size(), 3U);
  EXPECT_TRUE(has(cmds[0], "\"send_seq\":3,")) << cmds[0];
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_dropped, 2U);
  EXPECT_EQ(st.tx_dropped, 2U);
  EXPECT_EQ(st.raw_published, 3U);
  rig.sampler->set_run_state(false, "r1");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  const auto & s = rig.sink.summaries[0];
  EXPECT_EQ(s.raw_published, 3U);
  EXPECT_EQ(s.raw_dropped, 2U);
  EXPECT_EQ(s.tx_published, 3U);
  EXPECT_EQ(s.tx_dropped, 2U);
  const std::string json = to_json(s);
  EXPECT_TRUE(has(json, "\"raw_published\": 3, \"raw_dropped\": 2, \"tx_published\": 3, \"tx_dropped\": 2}"))
    << json;
  EXPECT_TRUE(has(json, "\"yield_ticks\": 0, ")) << "the old keys stay in place: " << json;
}

TEST(DaqSamplerRaw, CommandsArePublishedOnlyWhilePublishing)
{
  Rig rig(DaqMode::kGated, raw_config());
  uint64_t seq = 0;
  // 100 Hz 명령 타이머 흉내: 틱마다 송신 하나. 앞 100 ms idle → 100 ms 런 → 100 ms idle
  auto run_ticks = [&](int n) {
    for (int i = 0; i < n; ++i) {
      rig.sampler->submit_raw_command(raw_command(++seq));
      rig.sampler->step();
      rig.sampler->step();  // deadline 까지 잔다
    }
  };
  run_ticks(10);
  EXPECT_TRUE(rig.sink.raw_commands().empty());
  rig.sampler->set_run_state(true, "r1");
  run_ticks(10);
  rig.sampler->set_run_state(false, "r1");
  run_ticks(10);
  const auto cmds = rig.sink.raw_commands();
  ASSERT_EQ(cmds.size(), 10U);
  EXPECT_TRUE(has(cmds.front(), "\"send_seq\":11,")) << cmds.front();
  EXPECT_TRUE(has(cmds.back(), "\"send_seq\":20,")) << cmds.back();
  for (const auto & c : cmds) {
    EXPECT_TRUE(has(c, "\"run\":\"r1\"")) << c;
  }
  EXPECT_EQ(rig.sampler->stats().tx_discarded, 20U);
  // always 모드: 런이 없으면 run 은 ""
  rig.sampler->set_mode(DaqMode::kAlways);
  rig.sampler->submit_raw_command(raw_command(++seq));
  rig.sampler->tick(rig.now_ns);
  ASSERT_EQ(rig.sink.raw_commands().size(), 11U);
  EXPECT_TRUE(has(rig.sink.raw_commands().back(), "\"run\":\"\",\"send_seq\":31,"));
  // 양보 중이면 always 여도 버린다
  rig.sampler->set_yield(true);
  rig.sampler->submit_raw_command(raw_command(++seq));
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  EXPECT_EQ(rig.sink.raw_commands().size(), 11U);
}

TEST(DaqSamplerRaw, CommandJsonCarriesTheSendResult)
{
  mgv::MoraiLinkOptions link;
  link.ego_id = "Ego";
  auto c = raw_command(42, true);
  const std::string j = daq_raw_vehicle_ctrl_cmd_json(c, "run \"q\"", link);
  EXPECT_EQ(
    j.rfind(
      "{\"schema\":\"morai_raw.vehicle_ctrl_cmd/1\",\"run\":\"run \\\"q\\\"\",\"send_seq\":42,"
      "\"send_system_ns\":1758000000420000000,\"stale\":true,\"ok\":true,\"grpc_code\":0,\"rpc_ms\":0.2500,"
      "\"result_status\":1,\"vehicle_ctrl_cmd\":{\"actor_info\":{\"id\":{\"value\":\"Ego\"},\"object_type\":1,\"client_key\":\"\"},"
      "\"long_cmd_type\":3,",
      0),
    0U)
    << j;
  EXPECT_TRUE(has(j, "\"acceleration\":-2.4,")) << j;
  EXPECT_TRUE(has(j, "\"velocity\":36,")) << j;
  // 보내지 않은 명령 (비유한 값, grpc_code -1): rpc_ms 는 null, 값은 그대로
  c.ok = false;
  c.grpc_code = -1;
  c.rpc_ms.reset();
  c.result_status = -1;
  c.cmd.acceleration = std::nan("");
  const std::string n = daq_raw_vehicle_ctrl_cmd_json(c, "", link);
  EXPECT_TRUE(has(n, "\"ok\":false,\"grpc_code\":-1,\"rpc_ms\":null,\"result_status\":null,")) << n;
  EXPECT_TRUE(has(n, "\"acceleration\":\"NaN\"")) << n;
  // 바이트 없음 → actor_states null
  DaqRawActorStates empty;
  empty.update_index = 3;
  EXPECT_TRUE(has(daq_raw_actor_states_json(empty, "r"), "\"actor_states\":null}"));
}

TEST(DaqSamplerRaw, ARecordKeepsTheRunItWasProducedIn)
{
  Rig rig(DaqMode::kGated, raw_config());
  rig.sampler->set_run_state(true, "A");
  rig.sampler->submit_raw_actor_states(raw_update(1));
  rig.sampler->set_run_state(true, "B");  // A → replaced, 틱 전
  rig.sampler->submit_raw_actor_states(raw_update(2));
  rig.sampler->tick(rig.now_ns);
  const auto states = rig.sink.raw_actor_states();
  ASSERT_EQ(states.size(), 2U);
  EXPECT_TRUE(has(states[0], "\"run\":\"A\",\"update_index\":1,")) << states[0];
  EXPECT_TRUE(has(states[1], "\"run\":\"B\",\"update_index\":2,")) << states[1];
  rig.sampler->set_run_state(false, "B");
  ASSERT_EQ(rig.sink.summaries.size(), 2U);
  EXPECT_EQ(rig.sink.summaries[1].run, "B");
  EXPECT_EQ(rig.sink.summaries[1].raw_published, 1U) << "A's tail is not counted in B";
  // 26.09.26 _ YSH _ 리뷰 반영: A 이름으로 낸 꼬리는 A 요약에 센다 (요약 수 = 그 런 이름으로 낸 수)
  EXPECT_EQ(rig.sink.summaries[0].run, "A");
  EXPECT_EQ(rig.sink.summaries[0].end_reason, "replaced");
  EXPECT_EQ(rig.sink.summaries[0].raw_published, 1U);
}

TEST(DaqSamplerRaw, StopPublishesWhatWasQueuedAfterTheLastTick)
{
  Rig rig(DaqMode::kGated, raw_config());
  rig.sampler->set_run_state(true, "r1");
  rig.sampler->tick(rig.now_ns);
  rig.sampler->submit_raw_actor_states(raw_update(9));
  rig.sampler->submit_raw_command(raw_command(9));
  rig.sampler->stop();
  EXPECT_EQ(rig.sink.raw_actor_states().size(), 1U);
  EXPECT_EQ(rig.sink.raw_commands().size(), 1U);
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].end_reason, "shutdown");
  EXPECT_EQ(rig.sink.summaries[0].raw_published, 1U);
  EXPECT_EQ(rig.sink.summaries[0].tx_published, 1U);
}

// 26.09.26 _ YSH _ 리뷰 반영: always 모드는 stop 뒤에도 게이트가 열려 있다. 노드는 DAQ → 엔진(종료 감속) 순으로 멈추고,
//   멤버 소멸 순서상 pub_daq_raw_* 가 daq_sampler_ 보다 먼저 사라진다. 첫 stop 뒤의 submit 은 버리고, 두 번째 stop
//   (~DaqSampler) 은 sink 를 부르지 않아야 한다
TEST(DaqSamplerRaw, StopClosesTheRawPathEvenInAlwaysMode)
{
  Rig rig(DaqMode::kAlways, raw_config());
  rig.sampler->set_yield(false);
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  rig.sampler->submit_raw_actor_states(raw_update(1));  // 마지막 틱 뒤 — 첫 stop 이 낸다
  rig.sampler->stop();
  ASSERT_EQ(rig.sink.raws.size(), 1U);
  EXPECT_EQ(rig.sink.raw_actor_states().size(), 1U);
  EXPECT_TRUE(rig.sampler->publishing()) << "always mode keeps the /daq gate open after stop";
  // engine_->stop(): 마지막 업데이트와 종료 감속 ~0.3 s (100 Hz)
  rig.sampler->submit_raw_actor_states(raw_update(2));
  for (uint64_t i = 0; i < 30; ++i) {
    rig.sampler->submit_raw_command(raw_command(i + 1, true));
  }
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_discarded, 1U);
  EXPECT_EQ(st.tx_discarded, 30U);
  EXPECT_EQ(st.raw_published, 1U);
  EXPECT_EQ(st.tx_published, 0U);
  rig.sampler->stop();  // 소멸자 본문의 두 번째 stop
  rig.sampler.reset();  // ~DaqSampler 의 stop
  EXPECT_EQ(rig.sink.raws.size(), 1U) << "no publish_daq_raw after the first stop";
  EXPECT_TRUE(rig.sink.summaries.empty()) << "always mode without a run has no summary";
}

TEST(DaqSamplerRaw, StartAfterStopReopensTheRawPath)
{
  // 실제 시계 (가짜 시계로 스레드를 돌리면 now_ns 를 두 스레드가 만진다). sink 는 stop 으로 스레드를 거둔 뒤에만 읽는다
  DaqFrameCache cache;
  DaqCommandLatch latch;
  RuntimeParamsHandle params;
  params.set(RuntimeParams{});
  FakeDaqSink sink;
  DaqSampler sampler(
    cache, latch, params, sink, raw_config(), DaqMode::kAlways, rclcpp::get_logger("test_daq_raw_restart"));
  sampler.stop();
  sampler.submit_raw_command(raw_command(1));
  EXPECT_EQ(sampler.stats().tx_discarded, 1U) << "closed by the first stop";
  sampler.start();
  sampler.submit_raw_command(raw_command(2));
  std::this_thread::sleep_for(std::chrono::milliseconds(30));
  sampler.stop();
  const auto cmds = sink.raw_commands();
  ASSERT_EQ(cmds.size(), 1U) << "start() reopens the raw path";
  EXPECT_TRUE(has(cmds[0], "\"send_seq\":2,")) << cmds[0];
  sampler.submit_raw_command(raw_command(3));
  EXPECT_EQ(sampler.stats().tx_discarded, 2U) << "closed again by the second stop";
  EXPECT_EQ(sink.raw_commands().size(), 1U);
}

// 26.09.26 _ YSH _ 리뷰 반영: A → B (replaced) 가 한 틱 안에 오고 B 가 틱 전에 idle 이 되어도, A 의 꼬리는 A 요약에 세고
//   A 이름으로 낸다. B 안에서 넣은 것은 idle 이라 버린다 — 이름별로 낸 수 = 그 런 요약 수
TEST(DaqSamplerRaw, ReplacedRunTailIsCountedInItsOwnSummaryAndPublished)
{
  Rig rig(DaqMode::kGated, raw_config());
  rig.sampler->set_run_state(true, "A");
  rig.sampler->tick(rig.now_ns);
  rig.sampler->submit_raw_actor_states(raw_update(1));
  rig.sampler->submit_raw_command(raw_command(1));
  rig.sampler->submit_raw_actor_states(raw_update(2));
  rig.sampler->set_run_state(true, "B");
  ASSERT_EQ(rig.sink.summaries.size(), 1U);
  EXPECT_EQ(rig.sink.summaries[0].run, "A");
  EXPECT_EQ(rig.sink.summaries[0].raw_published, 2U) << "counted when A ends, before the tick publishes it";
  EXPECT_EQ(rig.sink.summaries[0].tx_published, 1U);
  rig.sampler->submit_raw_actor_states(raw_update(3));
  rig.sampler->set_run_state(false, "B");  // 틱 전에 idle — 게이트가 닫힌다
  ASSERT_EQ(rig.sink.summaries.size(), 2U);
  EXPECT_EQ(rig.sink.summaries[1].raw_published, 0U);
  EXPECT_FALSE(rig.sampler->publishing());
  rig.sampler->tick(rig.now_ns + 10 * kMs);
  const auto states = rig.sink.raw_actor_states();
  ASSERT_EQ(states.size(), 2U) << "A's tail goes out even though the gate is closed now";
  EXPECT_TRUE(has(states[0], "\"run\":\"A\",\"update_index\":1,")) << states[0];
  EXPECT_TRUE(has(states[1], "\"run\":\"A\",\"update_index\":2,")) << states[1];
  const auto cmds = rig.sink.raw_commands();
  ASSERT_EQ(cmds.size(), 1U);
  EXPECT_TRUE(has(cmds[0], "\"run\":\"A\",\"send_seq\":1,")) << cmds[0];
  EXPECT_TRUE(rig.sink.outputs.empty()) << "the /daq/* gate itself stays closed";
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_published, 2U);
  EXPECT_EQ(st.tx_published, 1U);
  EXPECT_EQ(st.raw_discarded, 1U) << "B's update was dropped at the idle";
  // 다음 틱에는 더 없다 (꼬리는 한 번만)
  rig.sampler->tick(rig.now_ns + 20 * kMs);
  EXPECT_EQ(rig.sink.raw_actor_states().size(), 2U);
}

// 26.09.26 _ YSH _ 리뷰 반영: 링크 Status.ok 는 MORAI Result 거부도 false 로 접는다. 계약의 ok 는 전송 OK (grpc_code 0),
//   거부 여부는 result_status 로 — "안 닿음"과 "MORAI 가 거부"를 가를 수 있어야 한다
TEST(DaqSamplerRaw, CommandRecordSeparatesTransportFromMoraiResult)
{
  mgv::MoraiLinkOptions link;
  link.ego_id = "Ego";
  mgv::VehicleCommand cmd;
  cmd.long_cmd_type = mgv::LongCmdType::kAcceleration;
  cmd.acceleration = 0.7;
  cmd.steer = -0.05;
  // MORAI 가 받았지만 거부 (gRPC OK, Result UNKNOWN_FAILURE = 0) — 링크는 ok=false 로 준다
  mgv::Status rejected;
  rejected.ok = false;
  rejected.grpc_code = 0;
  rejected.result_status = 0;
  rejected.rpc_ms = 1.5;
  const DaqRawCommand r = make_daq_raw_command(cmd, false, kEpochNs + 7, rejected, 5);
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.grpc_code, 0);
  EXPECT_EQ(r.result_status, 0);
  ASSERT_TRUE(r.rpc_ms.has_value());
  EXPECT_DOUBLE_EQ(*r.rpc_ms, 1.5);
  EXPECT_EQ(r.send_seq, 5U);
  EXPECT_EQ(r.send_system_ns, kEpochNs + 7);
  EXPECT_FALSE(r.stale);
  EXPECT_DOUBLE_EQ(r.cmd.acceleration, 0.7);
  EXPECT_DOUBLE_EQ(r.cmd.steer, -0.05);
  EXPECT_TRUE(has(
    daq_raw_vehicle_ctrl_cmd_json(r, "r", link),
    "\"ok\":true,\"grpc_code\":0,\"rpc_ms\":1.5000,\"result_status\":0,\"vehicle_ctrl_cmd\":"));
  // 성공
  mgv::Status success;
  success.ok = true;
  success.grpc_code = 0;
  success.result_status = 1;
  success.rpc_ms = 0.3;
  const DaqRawCommand s = make_daq_raw_command(cmd, true, kEpochNs, success, 6);
  EXPECT_TRUE(s.ok);
  EXPECT_EQ(s.result_status, 1);
  EXPECT_TRUE(s.stale);
  // DEADLINE_EXCEEDED — 응답 없음 (적용 여부 모름): ok false, result_status null, rpc_ms 는 잰 값
  mgv::Status deadline;
  deadline.ok = false;
  deadline.grpc_code = 4;
  deadline.rpc_ms = 20.0;
  const DaqRawCommand d = make_daq_raw_command(cmd, false, kEpochNs, deadline, 7);
  EXPECT_FALSE(d.ok);
  EXPECT_EQ(d.result_status, -1);
  EXPECT_TRUE(has(
    daq_raw_vehicle_ctrl_cmd_json(d, "r", link),
    "\"ok\":false,\"grpc_code\":4,\"rpc_ms\":20.0000,\"result_status\":null,"));
  // 보내지 않음 (비유한 명령): rpc_ms·result_status 모두 null
  mgv::Status not_sent;
  const DaqRawCommand n = make_daq_raw_command(cmd, false, kEpochNs, not_sent, 8);
  EXPECT_FALSE(n.ok);
  EXPECT_FALSE(n.rpc_ms.has_value());
  EXPECT_TRUE(has(
    daq_raw_vehicle_ctrl_cmd_json(n, "r", link),
    "\"ok\":false,\"grpc_code\":-1,\"rpc_ms\":null,\"result_status\":null,"));
}

TEST(DaqSamplerRaw, OffByDefault)
{
  EXPECT_FALSE(DaqConfig{}.raw);
  EXPECT_EQ(DaqConfig{}.raw_queue_depth, 256U);
  Rig rig(DaqMode::kAlways);
  rig.sampler->submit_raw_actor_states(raw_update(1));
  rig.sampler->submit_raw_command(raw_command(1));
  rig.feed();
  rig.sampler->tick(rig.now_ns);
  EXPECT_TRUE(rig.sink.raws.empty());
  const auto st = rig.sampler->stats();
  EXPECT_EQ(st.raw_published + st.raw_discarded + st.tx_published + st.tx_discarded, 0U);
  EXPECT_EQ(rig.sink.states(), 1U);
}

}  // namespace
}  // namespace autoware::morai_interface
