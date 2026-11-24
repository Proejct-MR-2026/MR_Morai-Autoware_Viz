// 신규 — 26.09.25 _ YSH _ MoraiLink 순수부 gtest (proto<->구조체 변환, Result 판정, 시계 오프셋, 실패 경로)
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
#include "morai_grpc_vendor/morai_link_types.hpp"
#include "morai_link_convert.hpp"

#include <gtest/gtest.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/common/enum.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace
{

namespace mv = morai_grpc_vendor;
namespace lc = morai_grpc_vendor::link_convert;
using morai_sim_api::actor::ActorState;
using morai_sim_api::actor::ActorStates;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
/// 2026-09 의 Unix epoch µs (실측 ActorStates.timestamp 와 같은 크기)
constexpr int64_t kEpochUs = 1790000000123456;

void set_vec(morai_sim_api::Vector3 * v, double x, double y, double z)
{
  v->set_x(x);
  v->set_y(y);
  v->set_z(z);
}

ActorState * add_actor(ActorStates * s, const std::string & id, morai_sim_api::ObjectType type)
{
  ActorState * a = s->add_states();
  a->mutable_actor_info()->mutable_id()->set_value(id);
  a->mutable_actor_info()->set_object_type(type);
  set_vec(a->mutable_transform()->mutable_location(), 10.0, 20.0, 0.5);
  set_vec(a->mutable_transform()->mutable_rotation(), 0.0, 0.0, 90.0);
  return a;
}

ActorStates three_actors()
{
  ActorStates s;
  s.set_timestamp(kEpochUs);
  s.set_sync_mode_tick_count(0);
  ActorState * npc = add_actor(&s, "Npc_1", morai_sim_api::OBJECT_TYPE_VEHICLE);
  set_vec(npc->mutable_bounding_box()->mutable_extent(), 1.06, 2.32, 0.75);
  set_vec(npc->mutable_velocity(), 36.0, -3.6, 0.0);

  ActorState * ego = add_actor(&s, "Ego", morai_sim_api::OBJECT_TYPE_VEHICLE);
  set_vec(ego->mutable_transform()->mutable_location(), 100.5, -200.25, 1.5);
  set_vec(ego->mutable_transform()->mutable_rotation(), 1.0, -2.0, 179.0);
  set_vec(ego->mutable_velocity(), 72.0, -1.8, 0.1);             // km/h 그대로 남아야 한다
  set_vec(ego->mutable_angular_velocity(), 0.01, -0.3, 0.02);    // world (-z, x, -y) 원값
  set_vec(ego->mutable_acceleration(), 0.8, 0.0, 0.0);
  set_vec(ego->mutable_global_velocity(), -20.0, 0.5, 0.0);
  auto * vs = ego->mutable_vehicle_state();
  vs->set_throttle(0.25F);
  vs->set_brake(0.003F);
  vs->set_steer(-0.4F);
  vs->set_front_wheel_angle(0.5F);  // Unity 부호 그대로 (반전은 AMI)
  vs->set_gear_mode(morai_sim_api::actor::GEAR_MODE_D);
  vs->add_collision_objects("Npc_9");
  vs->add_collision_objects("Wall");

  ActorState * ped = add_actor(&s, "Ped_1", morai_sim_api::OBJECT_TYPE_PEDESTRIAN);
  set_vec(ped->mutable_bounding_box()->mutable_extent(), 0.3, 0.3, 0.9);
  set_vec(ped->mutable_acceleration(), 0.1, 0.2, 0.3);
  return s;
}

// =============================================================================
// proto -> 구조체
// =============================================================================

TEST(MoraiLinkConvert, SnapshotSplitsEgoFromActorsAndKeepsRawUnits)
{
  mv::Snapshot out;
  lc::to_snapshot(three_actors(), "Ego", &out);

  EXPECT_EQ(out.timestamp_us, kEpochUs);
  EXPECT_EQ(out.sync_mode_tick_count, 0u);
  ASSERT_TRUE(out.ego.found);
  EXPECT_TRUE(out.ego.exact_match);
  EXPECT_TRUE(out.ego.valid);
  EXPECT_TRUE(out.ego.has_timestamp);
  EXPECT_EQ(out.ego.timestamp_us, kEpochUs);
  EXPECT_EQ(out.ego.id, "Ego");
  EXPECT_EQ(out.ego.object_type, mv::ObjectType::kVehicle);

  // 값 변환 없음: km/h, deg, world 각속도 원값, Unity 부호 조향
  EXPECT_DOUBLE_EQ(out.ego.position.x, 100.5);
  EXPECT_DOUBLE_EQ(out.ego.position.y, -200.25);
  EXPECT_DOUBLE_EQ(out.ego.rotation_deg.x, 1.0);
  EXPECT_DOUBLE_EQ(out.ego.rotation_deg.y, -2.0);
  EXPECT_DOUBLE_EQ(out.ego.rotation_deg.z, 179.0);
  EXPECT_DOUBLE_EQ(out.ego.velocity_kmh.x, 72.0);
  EXPECT_DOUBLE_EQ(out.ego.velocity_kmh.y, -1.8);
  EXPECT_DOUBLE_EQ(out.ego.angular_velocity_raw.y, -0.3);
  EXPECT_DOUBLE_EQ(out.ego.acceleration_raw.x, 0.8);
  EXPECT_DOUBLE_EQ(out.ego.global_velocity_raw.x, -20.0);
  const auto & vs = out.ego.vehicle_state;
  ASSERT_TRUE(vs.present);
  EXPECT_NEAR(vs.throttle, 0.25, 1e-7);
  EXPECT_NEAR(vs.brake, 0.003, 1e-7);
  EXPECT_NEAR(vs.steer, -0.4, 1e-7);
  EXPECT_NEAR(vs.front_wheel_angle, 0.5, 1e-7);
  EXPECT_EQ(vs.gear_mode, mv::GearMode::kD);
  EXPECT_EQ(vs.collision_count, 2u);
  ASSERT_EQ(vs.collision_objects.size(), 2u);
  EXPECT_EQ(vs.collision_objects[0], "Npc_9");

  ASSERT_EQ(out.actors.size(), 2u);  // Ego 제외, 순서 유지
  EXPECT_EQ(out.actors[0].id, "Npc_1");
  EXPECT_EQ(out.actors[0].object_type, mv::ObjectType::kVehicle);
  EXPECT_DOUBLE_EQ(out.actors[0].velocity_kmh.x, 36.0);
  // extent 는 반치수 x=반폭 y=반길이 그대로 (맞바꿈·2 배는 AMI)
  EXPECT_DOUBLE_EQ(out.actors[0].bbox_extent.x, 1.06);
  EXPECT_DOUBLE_EQ(out.actors[0].bbox_extent.y, 2.32);
  EXPECT_DOUBLE_EQ(out.actors[0].bbox_extent.z, 0.75);
  EXPECT_FALSE(out.actors[0].has_vehicle_state);
  EXPECT_EQ(out.actors[1].id, "Ped_1");
  EXPECT_EQ(out.actors[1].object_type, mv::ObjectType::kPedestrian);
  EXPECT_DOUBLE_EQ(out.actors[1].acceleration_raw.z, 0.3);
  EXPECT_DOUBLE_EQ(out.actors[1].rotation_deg.z, 90.0);
}

TEST(MoraiLinkConvert, EgoFallsBackToCaseInsensitiveMatch)
{
  ActorStates s;
  s.set_timestamp(kEpochUs);
  add_actor(&s, "EGO", morai_sim_api::OBJECT_TYPE_VEHICLE);
  add_actor(&s, "Npc_1", morai_sim_api::OBJECT_TYPE_VEHICLE);
  mv::Snapshot out;
  lc::to_snapshot(s, "Ego", &out);
  ASSERT_TRUE(out.ego.found);
  EXPECT_FALSE(out.ego.exact_match);
  EXPECT_EQ(out.ego.id, "EGO");
  // 대체 매치로 ego 가 된 항목도 actors 에서 뺀다 — 남기면 자차가 base_link 에 장애물로 발행된다
  ASSERT_EQ(out.actors.size(), 1u);
  EXPECT_EQ(out.actors[0].id, "Npc_1");
}

TEST(MoraiLinkConvert, EveryExactEgoIsDroppedFromActorsLikeMainPy)
{
  ActorStates s;
  s.set_timestamp(kEpochUs);
  ActorState * first = add_actor(&s, "Ego", morai_sim_api::OBJECT_TYPE_VEHICLE);
  set_vec(first->mutable_transform()->mutable_location(), 1.0, 2.0, 3.0);
  add_actor(&s, "ego", morai_sim_api::OBJECT_TYPE_VEHICLE);  // 대소문자만 같은 NPC
  add_actor(&s, "Ego", morai_sim_api::OBJECT_TYPE_VEHICLE);  // 중복 Ego
  add_actor(&s, "Npc_1", morai_sim_api::OBJECT_TYPE_VEHICLE);
  mv::Snapshot out;
  lc::to_snapshot(s, "Ego", &out);
  ASSERT_TRUE(out.ego.found);
  EXPECT_TRUE(out.ego.exact_match);
  EXPECT_DOUBLE_EQ(out.ego.position.x, 1.0);  // 정확 일치 첫 항목
  ASSERT_EQ(out.actors.size(), 2u);
  EXPECT_EQ(out.actors[0].id, "ego");
  EXPECT_EQ(out.actors[1].id, "Npc_1");
}

TEST(MoraiLinkConvert, MissingEgoLeavesEveryActorAndClearsStaleEgo)
{
  mv::Snapshot out;
  lc::to_snapshot(three_actors(), "Ego", &out);
  ASSERT_TRUE(out.ego.found);

  // 같은 버퍼 재사용: 다음 업데이트에 Ego 가 없으면 이전 Ego 가 남으면 안 된다
  ActorStates s;
  s.set_timestamp(kEpochUs + 20000);
  add_actor(&s, "Npc_1", morai_sim_api::OBJECT_TYPE_VEHICLE);
  lc::to_snapshot(s, "Ego", &out);
  EXPECT_FALSE(out.ego.found);
  EXPECT_FALSE(out.ego.has_timestamp);
  EXPECT_FALSE(out.ego.vehicle_state.present);
  EXPECT_EQ(out.timestamp_us, kEpochUs + 20000);
  ASSERT_EQ(out.actors.size(), 1u);
  EXPECT_EQ(out.actors[0].id, "Npc_1");
  EXPECT_EQ(out.actors[0].bbox_extent.x, 0.0);  // 이전 Npc_1 의 extent 가 남지 않는다
}

TEST(MoraiLinkConvert, EgoValidityFollowsDaqAllZeroRule)
{
  const mv::Vec3 zero{};
  EXPECT_FALSE(mv::ego_is_valid(zero, zero, zero));                   // 미스폰·리셋 기본값
  EXPECT_TRUE(mv::ego_is_valid(zero, zero, mv::Vec3{0.1, 0.0, 0.0}));  // 원점 주행
  EXPECT_TRUE(mv::ego_is_valid(mv::Vec3{1.0, 0.0, 0.0}, zero, zero));  // 정지 차량
  EXPECT_TRUE(mv::ego_is_valid(zero, mv::Vec3{0.0, 0.0, 90.0}, zero));

  ActorStates s;
  ActorState * e = s.add_states();
  e->mutable_actor_info()->mutable_id()->set_value("Ego");
  mv::Snapshot out;
  lc::to_snapshot(s, "Ego", &out);
  EXPECT_TRUE(out.ego.found);
  EXPECT_FALSE(out.ego.valid);
}

TEST(MoraiLinkConvert, SpecKeepsDegrees)
{
  morai_sim_api::actor::VehicleSpec in;
  in.set_max_steering_angle(40.0F);
  in.set_wheel_base(3.0F);
  in.set_length(4.64F);
  const mv::VehicleSpecInfo s = lc::to_spec(in);
  EXPECT_NEAR(s.max_steering_angle_deg, 40.0, 1e-6);
  EXPECT_NEAR(s.wheel_base_m, 3.0, 1e-6);
  EXPECT_NEAR(s.length_m, 4.64, 1e-6);
}

// =============================================================================
// 구조체 -> proto
// =============================================================================

TEST(MoraiLinkConvert, ControlCommandCarriesEveryFieldUnchanged)
{
  morai_sim_api::ObjectInfo ego;
  ego.mutable_id()->set_value("Ego");
  ego.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);

  mv::VehicleCommand c;
  c.long_cmd_type = mv::LongCmdType::kAcceleration;
  c.throttle = 0.2;
  c.brake = 0.1;
  c.steer = -0.415;  // 타이어각 rad 그대로 — 정규화·부호 반전 없음 (정책은 AMI)
  c.velocity = 36.0;
  c.acceleration = -1.5;
  c.frame = 7;

  morai_sim_api::actor::VehicleCtrlCmd out;
  out.mutable_actor_info()->mutable_id()->set_value("stale");
  lc::to_ctrl_cmd(c, ego, &out);
  EXPECT_EQ(out.actor_info().id().value(), "Ego");
  EXPECT_EQ(out.actor_info().object_type(), morai_sim_api::OBJECT_TYPE_VEHICLE);
  EXPECT_EQ(out.actor_info().client_key(), "");
  EXPECT_EQ(out.long_cmd_type(), morai_sim_api::actor::LONG_CMD_TYPE_ACCELERATION);
  EXPECT_DOUBLE_EQ(out.throttle(), 0.2);
  EXPECT_DOUBLE_EQ(out.brake(), 0.1);
  EXPECT_DOUBLE_EQ(out.steer(), -0.415);
  EXPECT_DOUBLE_EQ(out.velocity(), 36.0);
  EXPECT_DOUBLE_EQ(out.acceleration(), -1.5);
  EXPECT_EQ(out.frame(), 7u);

  c.long_cmd_type = mv::LongCmdType::kSpeed;
  lc::to_ctrl_cmd(c, ego, &out);
  EXPECT_EQ(out.long_cmd_type(), morai_sim_api::actor::LONG_CMD_TYPE_SPEED);
}

TEST(MoraiLinkConvert, SetTransformKeepsMoraiConvention)
{
  morai_sim_api::ObjectInfo ego;
  ego.mutable_id()->set_value("Ego");
  mv::TransformPose p;
  p.position = {1.0, 2.0, 3.0};
  p.rotation_deg = {0.0, 0.0, -135.0};
  morai_sim_api::actor::SetTransformParam out;
  lc::to_set_transform(p, ego, &out);
  EXPECT_EQ(out.actor_info().id().value(), "Ego");
  EXPECT_DOUBLE_EQ(out.transform().location().x(), 1.0);
  EXPECT_DOUBLE_EQ(out.transform().location().z(), 3.0);
  EXPECT_DOUBLE_EQ(out.transform().rotation().z(), -135.0);
}

TEST(MoraiLinkConvert, CommandFinitenessGuard)
{
  mv::VehicleCommand c;
  EXPECT_TRUE(mv::command_is_finite(c));
  c.steer = kNaN;
  EXPECT_FALSE(mv::command_is_finite(c));
  c.steer = 0.0;
  c.acceleration = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(mv::command_is_finite(c));
}

// =============================================================================
// Result 판정·되읽기 일치
// =============================================================================

TEST(MoraiLinkConvert, ResultStatusZeroIsFailureNotSuccess)
{
  morai_sim_api::Result r;
  r.set_status(morai_sim_api::STATUS_CODE_SUCCESS);
  mv::Status s;
  s.ok = true;
  s.grpc_code = 0;
  lc::apply_result(r, "ControlVehicle", &s);
  EXPECT_TRUE(s.ok);
  EXPECT_EQ(s.result_status, 1);
  EXPECT_TRUE(s.message.empty());

  r.set_status(morai_sim_api::STATUS_CODE_UNKNOWN_FAILURE);  // = 0
  r.set_description("Unknown error");
  mv::Status f;
  f.ok = true;
  f.grpc_code = 0;
  lc::apply_result(r, "SetVehicleGear", &f);
  EXPECT_FALSE(f.ok);
  EXPECT_EQ(f.result_status, 0);
  EXPECT_NE(f.message.find("SetVehicleGear"), std::string::npos);
  EXPECT_NE(f.message.find("Unknown error"), std::string::npos);
}

TEST(MoraiLinkConvert, TransformReadbackMatchWrapsYaw)
{
  mv::EgoSample e;
  e.found = true;
  e.valid = true;
  e.position = {10.0, 20.0, 0.3};
  e.rotation_deg = {0.0, 0.0, -179.5};
  mv::TransformPose t;
  t.position = {10.2, 19.9, 0.0};  // z 는 보지 않는다 (MORAI 가 지면에 맞춘다)
  t.rotation_deg = {0.0, 0.0, 179.5};
  EXPECT_TRUE(lc::transform_matches(e, t, 0.5, 2.0));  // yaw 차 1 도 (±180 래핑)
  t.rotation_deg.z = 170.0;
  EXPECT_FALSE(lc::transform_matches(e, t, 0.5, 2.0));
  t.rotation_deg.z = 179.5;
  t.position.x = 11.0;
  EXPECT_FALSE(lc::transform_matches(e, t, 0.5, 2.0));
  t.position.x = 10.0;
  e.found = false;
  EXPECT_FALSE(lc::transform_matches(e, t, 0.5, 2.0));
}

// =============================================================================
// 이름·시계 오프셋
// =============================================================================

TEST(MoraiLinkTypes, NamesAndUnknownValues)
{
  EXPECT_EQ(mv::to_string(mv::ControlMode::kAuto), "AUTO");
  EXPECT_EQ(mv::to_string(mv::ControlMode::kKeyboard), "KEYBOARD");
  EXPECT_EQ(mv::to_string(static_cast<mv::ControlMode>(42)), "UNKNOWN(42)");
  EXPECT_EQ(mv::to_string(mv::GearMode::kD), "D");
  EXPECT_EQ(mv::to_string(mv::GearMode::kP), "P");
  EXPECT_EQ(mv::to_string(mv::LongCmdType::kAcceleration), "ACCELERATION");
  EXPECT_EQ(mv::to_string(mv::TurnSignal::kOff), "OFF");
  EXPECT_EQ(mv::to_string(mv::ObjectType::kObstacle), "OBSTACLE");
  EXPECT_EQ(mv::to_string(mv::ChannelState::kTransientFailure), "TRANSIENT_FAILURE");
  // 정수 값이 proto 와 같다 (static_assert 가 빌드 시 고정 — 여기서는 main.py 매핑 값 확인)
  EXPECT_EQ(static_cast<int>(mv::GearMode::kP), 1);
  EXPECT_EQ(static_cast<int>(mv::GearMode::kD), 4);
  EXPECT_EQ(static_cast<int>(mv::ControlMode::kAuto), 3);
  EXPECT_EQ(static_cast<int>(mv::ControlMode::kSynchronous), 7);
}

/// MORAI 프레임 시계 흉내: 프레임이 period_us 마다, timestamp 이후 publish_delay_us 뒤에 읽힌다.
/// 로컬은 poll_us 마다 폴링하고 왕복 rtt_us (편도 절반). offset_us = MORAI 시계 - 로컬 시계.
/// distinct_only 면 서로 다른 프레임의 첫 응답만 (monitor 와 같음), 아니면 모든 폴링
std::vector<mv::ClockSample> frame_clock_polls(
  int64_t offset_us, int64_t period_us, int64_t publish_delay_us, int64_t poll_us, int64_t rtt_us,
  int64_t span_us, bool distinct_only = true)
{
  std::vector<mv::ClockSample> out;
  int64_t last_ts = -1;
  const int64_t t0 = 1000000;  // 시작 1 s 뒤부터 — 음수 나눗셈(0 쪽 절삭)을 피한다
  for (int64_t local = t0; local < t0 + span_us; local += poll_us) {
    const int64_t served_server = local + rtt_us / 2 + offset_us;  // 서버가 요청을 처리한 시각
    // 처리 시점에 이미 발행된 가장 최근 프레임
    const int64_t ts = ((served_server - publish_delay_us) / period_us) * period_us;
    if (distinct_only && ts == last_ts) {
      continue;
    }
    last_ts = ts;
    const int64_t send_ns = (kEpochUs + local) * 1000;
    out.push_back(mv::ClockSample{kEpochUs + ts, send_ns, send_ns + rtt_us * 1000});
  }
  return out;
}

TEST(MoraiLinkTypes, FrameClockBoundPlusSamePcBaselineRecoversOffset)
{
  // 60 fps 프레임, 발행 지연 2 ms, 1 kHz 폴링, 왕복 0.2 ms, 10 s
  const int64_t period = 16667;
  const int64_t delay = 2000;
  const auto local = frame_clock_polls(0, period, delay, 1000, 200, 10000000);
  const auto remote = frame_clock_polls(3000, period, delay, 1000, 200, 10000000);  // MORAI +3 ms
  ASSERT_GT(local.size(), 500u);

  // 하한은 참 오프셋 이하이고, 최소 프레임 나이(발행 지연 + 편도 + 폴링 간격 이내)만큼 낮다
  const double base = mv::clock_offset_lower_bound_ms(local);
  EXPECT_LE(base, 0.0);
  EXPECT_NEAR(base, -2.1, 1.0);
  const double bound = mv::clock_offset_lower_bound_ms(remote);
  EXPECT_LE(bound, 3.0);
  // 같은 PC 기준선을 빼면 참 오프셋에 폴링 간격 이내로 붙는다
  EXPECT_NEAR(mv::clock_offset_from_baseline_ms(bound, base), 3.0, 1.0);
  EXPECT_TRUE(std::isnan(mv::clock_offset_from_baseline_ms(bound, kNaN)));
  EXPECT_TRUE(std::isnan(mv::clock_offset_from_baseline_ms(kNaN, base)));

  // 반례: 아무 때나 읽은 표본(GetTimestamp 1 s 마다 같은)에 NTP 식 (server - 왕복 중간점) 중앙값을
  // 내면 참값 0 인데 발행 지연 + 평균 프레임 나이(~반 주기)만큼 음수로 틀린다
  const auto any_time = frame_clock_polls(0, period, delay, 1000, 200, 10000000, false);
  std::vector<double> ntp;
  for (const auto & c : any_time) {
    ntp.push_back(
      static_cast<double>(c.server_us * 1000 - (c.local_send_ns + c.local_recv_ns) / 2) / 1e6);
  }
  std::sort(ntp.begin(), ntp.end());
  EXPECT_LT(ntp[ntp.size() / 2], -8.0);
}

TEST(MoraiLinkTypes, SnapshotLowerBoundUsesLatestFrame)
{
  // 스냅샷 timestamp 는 프레임 시각 → server - recv 는 offset 이하. 최댓값이 하한
  const int64_t recv = kEpochUs * 1000;
  const std::vector<mv::ClockSample> v = {
    mv::ClockSample{kEpochUs - 15000, recv - 1000000, recv},  // 15 ms 된 프레임
    mv::ClockSample{kEpochUs - 1000, recv - 1000000, recv},   // 1 ms 된 프레임
    mv::ClockSample{0, recv, recv}};                          // 무효 → 무시
  EXPECT_NEAR(mv::clock_offset_lower_bound_ms(v), -1.0, 1e-9);
  EXPECT_TRUE(std::isnan(mv::clock_offset_lower_bound_ms({})));
  // 수신 시각이 없는 표본은 버린다 (0 이면 하한이 epoch 크기로 튄다)
  EXPECT_TRUE(std::isnan(mv::clock_offset_lower_bound_ms({mv::ClockSample{kEpochUs, 0, 0}})));
}

// =============================================================================
// 연결되지 않은 MoraiLink — 실패 경로가 예외 없이 Status 로 끝나는가
// =============================================================================

mv::MoraiLinkOptions unreachable()
{
  mv::MoraiLinkOptions o;
  o.address = "127.0.0.1";
  o.port = 1;  // MORAI(7789)가 아니다. 닫힌 포트 — 연결 거부로 곧바로 실패한다
  o.deadlines.state = std::chrono::milliseconds(100);
  o.deadlines.command = std::chrono::milliseconds(100);
  o.deadlines.gear = std::chrono::milliseconds(100);
  o.deadlines.set_mode = std::chrono::milliseconds(100);
  o.deadlines.transform = std::chrono::milliseconds(100);
  o.deadlines.control_mode = std::chrono::milliseconds(100);
  o.deadlines.time_probe = std::chrono::milliseconds(100);
  return o;
}

TEST(MoraiLinkOffline, FailuresAreStatusesNotExceptions)
{
  mv::MoraiLink link(unreachable());
  EXPECT_EQ(link.options().port, 1);
  EXPECT_EQ(link.options().ego_id, "Ego");

  const mv::Status c = link.wait_connected(std::chrono::milliseconds(200));
  EXPECT_FALSE(c.ok);
  EXPECT_FALSE(c.message.empty());
  EXPECT_NE(link.channel_state(mv::LinkChannel::kState), mv::ChannelState::kReady);
  EXPECT_NE(link.channel_state(mv::LinkChannel::kCommand), mv::ChannelState::kReady);

  mv::Snapshot snap;
  snap.timestamp_us = 42;
  const mv::Status s = link.get_snapshot(&snap);
  EXPECT_FALSE(s.ok);
  EXPECT_GE(s.grpc_code, 1);
  EXPECT_EQ(snap.timestamp_us, 42);  // 실패하면 out 을 건드리지 않는다
  EXPECT_FALSE(link.get_snapshot().ok());
  EXPECT_FALSE(link.get_ego_state().ok());
  EXPECT_FALSE(link.get_control_mode().ok());
  EXPECT_FALSE(link.probe_server_time().ok());
  EXPECT_FALSE(link.get_snapshot(nullptr).ok);

  mv::VehicleCommand cmd;
  EXPECT_FALSE(link.control_vehicle(cmd).ok);  // 보냈지만 닿지 않음
  cmd.steer = kNaN;
  const mv::Status nan = link.control_vehicle(cmd);
  EXPECT_FALSE(nan.ok);
  EXPECT_EQ(nan.grpc_code, -1);  // 보내지 않았다
  EXPECT_EQ(nan.rpc_ms, 0.0);

  // 닿지 않은 전송 실패(UNAVAILABLE)는 되읽기를 하지 않는다
  const mv::Status g = link.set_gear(mv::GearMode::kD, std::chrono::milliseconds(300));
  EXPECT_FALSE(g.ok);
  if (!g.deadline_exceeded()) {
    EXPECT_FALSE(g.readback_checked);
  }
  mv::TransformPose bad;
  bad.position.x = kNaN;
  const mv::Status t = link.set_transform(bad);
  EXPECT_FALSE(t.ok);
  EXPECT_EQ(t.grpc_code, -1);
  EXPECT_FALSE(link.set_tail_light(mv::TurnSignal::kOff).ok);

  const mv::LinkCounters n = link.counters();
  EXPECT_EQ(n.rejected_inputs, 3u);  // nullptr 스냅샷 + NaN 명령 + NaN 자세
  EXPECT_GE(n.state_calls, 5u);
  EXPECT_EQ(n.state_failures, n.state_calls);
  EXPECT_GE(n.command_calls, 3u);
  EXPECT_EQ(n.command_failures, n.command_calls);
}

}  // namespace
