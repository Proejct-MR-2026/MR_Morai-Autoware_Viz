// 신규 — 26.11.10 _ YSH _ 외란 주입 (MRDIST-1) 단위 시험 — 골든 값·트리거 문법·명령 제어기·프레임 적용·메시지 변환.
//   골든 값은 design_261110/10 §4 의 것을 그대로 쓴다 (Python 기준 구현과 같은 글자). 정수·u01 은 정확히,
//   가우스는 1e-12, 회전한 위치는 1e-9, dx/dy 는 (보낸 값 − GT) 와 정확히 같아야 한다
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
#include "autoware/morai_interface/object_disturbance.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{
namespace
{
namespace mgv = morai_grpc_vendor;

constexpr uint64_t kBase = 123456789012345ULL;
constexpr int64_t kMs = 1000000LL;

// 설계 문서 §4 의 숨김 무늬 ('.' 보임, 'H' 숨김), 건 프레임부터 60 프레임
const char kPattern075At0[] = "..........H..........HHH..H................HHH..............";
const char kPattern075At7[] = "...H..........HHH..H................HHH.....................";
const char kPattern050At0[] = "...H.H....H..H.H....H.HH..H.HHHH.HH..HH...H..H..H...H.HH.HH.";

mgv::ActorSample make_actor(const std::string & name, double x, double y)
{
  mgv::ActorSample a;
  a.id = name;
  a.object_type = mgv::ObjectType::kVehicle;
  a.client_key = "key_" + name;
  a.position = {x, y, 1.0};
  a.rotation_deg = {0.5, -0.25, 30.0};
  a.velocity_kmh = {36.0, 0.1, -0.2};
  a.angular_velocity_raw = {0.01, 0.02, 0.03};
  a.acceleration_raw = {0.3, 0.2, 0.1};
  a.global_velocity_raw = {31.0, 18.0, 0.0};
  a.bbox_extent = {1.0, 2.3, 0.75};
  a.has_vehicle_state = true;
  a.vehicle_state.present = true;
  a.vehicle_state.throttle = 0.25;
  a.vehicle_state.gear_mode = mgv::GearMode::kD;
  return a;
}

/// 자차 yaw [deg] 와 actor 목록으로 스냅샷 하나
mgv::Snapshot make_snapshot(double ego_yaw_deg, std::vector<mgv::ActorSample> actors)
{
  mgv::Snapshot s;
  s.ego.found = true;
  s.ego.valid = true;
  s.ego.id = "Ego";
  s.ego.position = {100.0, 200.0, 1.0};
  s.ego.rotation_deg = {0.0, 0.0, ego_yaw_deg};
  s.actors = std::move(actors);
  return s;
}

/// 시험 test_engine.cpp 의 make_snapshot 과 같은 배치: 자차 yaw 45°, Car_1 이 (110, 200)
mgv::Snapshot car1_snapshot()
{
  return make_snapshot(45.0, {make_actor("Car_1", 110.0, 200.0)});
}

void expect_vec_eq(const mgv::Vec3 & a, const mgv::Vec3 & b)
{
  EXPECT_EQ(a.x, b.x);
  EXPECT_EQ(a.y, b.y);
  EXPECT_EQ(a.z, b.z);
}

/// 위치·상자 중심만 빼고 전부 같은가
void expect_actor_eq_except_xy(const mgv::ActorSample & a, const mgv::ActorSample & b)
{
  EXPECT_EQ(a.id, b.id);
  EXPECT_EQ(a.object_type, b.object_type);
  EXPECT_EQ(a.client_key, b.client_key);
  EXPECT_EQ(a.position.z, b.position.z);
  expect_vec_eq(a.rotation_deg, b.rotation_deg);
  expect_vec_eq(a.velocity_kmh, b.velocity_kmh);
  expect_vec_eq(a.angular_velocity_raw, b.angular_velocity_raw);
  expect_vec_eq(a.acceleration_raw, b.acceleration_raw);
  expect_vec_eq(a.global_velocity_raw, b.global_velocity_raw);
  expect_vec_eq(a.bbox_extent, b.bbox_extent);
  EXPECT_EQ(a.bbox_location.z, b.bbox_location.z);
  expect_vec_eq(a.bbox_rotation, b.bbox_rotation);
  EXPECT_EQ(a.has_vehicle_state, b.has_vehicle_state);
  EXPECT_EQ(a.vehicle_state.present, b.vehicle_state.present);
  EXPECT_EQ(a.vehicle_state.throttle, b.vehicle_state.throttle);
  EXPECT_EQ(a.vehicle_state.brake, b.vehicle_state.brake);
  EXPECT_EQ(a.vehicle_state.steer, b.vehicle_state.steer);
  EXPECT_EQ(a.vehicle_state.front_wheel_angle, b.vehicle_state.front_wheel_angle);
  EXPECT_EQ(a.vehicle_state.gear_mode, b.vehicle_state.gear_mode);
  EXPECT_EQ(a.vehicle_state.collision_objects, b.vehicle_state.collision_objects);
}

void expect_actor_eq(const mgv::ActorSample & a, const mgv::ActorSample & b)
{
  expect_actor_eq_except_xy(a, b);
  expect_vec_eq(a.position, b.position);
  expect_vec_eq(a.bbox_location, b.bbox_location);
}

DisturbanceRequest make_request(
  const std::string & trigger, const std::string & run = "", uint64_t base_seed = 0)
{
  DisturbanceRequest r;
  r.trigger = trigger;
  r.run = run;
  r.base_seed = base_seed;
  r.source = "gtest";
  return r;
}

DisturbanceController::Options options_with_seed(uint64_t seed)
{
  DisturbanceController::Options o;
  o.seed_source = [seed]() { return seed; };
  return o;
}

bool starts_with(const std::string & text, const std::string & prefix)
{
  return text.compare(0, prefix.size(), prefix) == 0;
}

bool has_token(const std::string & message, const std::string & token)
{
  return (" " + message + " ").find(" " + token + " ") != std::string::npos;
}

/// 제어기 + 손잡이 + 적용기를 엔진이 쓰는 방식 그대로 묶은 것 (프레임마다 stamp 를 dt 만큼 민다)
struct Bench
{
  DisturbanceHandle handle;
  DisturbanceController ctl;
  ObjectDisturber disturber;
  int64_t stamp_ns{1000000000LL};

  explicit Bench(DisturbanceController::Options options = options_with_seed(777))
  : ctl(handle, std::move(options))
  {
  }

  DisturbanceReply send(
    const std::string & trigger, const std::string & run = "", uint64_t base_seed = 0)
  {
    return ctl.handle(make_request(trigger, run, base_seed));
  }

  DisturbanceFrame step(const mgv::Snapshot & snapshot, int64_t dt_ms = 100)
  {
    stamp_ns += dt_ms * kMs;
    return disturber.apply(handle, snapshot, stamp_ns);
  }

  std::string pattern(const mgv::Snapshot & snapshot, int frames)
  {
    std::string out;
    for (int i = 0; i < frames; ++i) {
      out += step(snapshot).objects_visible ? '.' : 'H';
    }
    return out;
  }
};
}  // namespace

// =============================================================================
// MRDIST-1 골든 값
// =============================================================================

TEST(Mrdist, GoldenIntegers)
{
  EXPECT_EQ(mrdist::mix64(1), 0x5692161D100B05E5ULL);
  EXPECT_EQ(mrdist::mix64(0x0123456789ABCDEFULL), 0xB2C058E4EBB5112CULL);
  // SplitMix64 시드 0 의 첫·둘째 출력
  EXPECT_EQ(mrdist::stream(0, 1), 0xE220A8397B1DCDAFULL);
  EXPECT_EQ(mrdist::stream(0, 2), 0x6E789E6AA1B965F4ULL);

  EXPECT_EQ(mrdist::object_seed(kBase, "NPC_Car_1"), 0x8DF6BF7B5DBCDFCBULL);
  EXPECT_EQ(mrdist::object_seed(kBase, "NPC_Ped_1"), 0x11C916F768E8BDE8ULL);
  EXPECT_EQ(mrdist::object_seed(kBase, "Car_1"), 0x4E6987CFFE11F9DBULL);
  EXPECT_EQ(mrdist::dropout_seed(kBase), 0xFCC081FFA48A068BULL);

  // 꼬리표는 ASCII 글자를 big-endian 으로 읽은 값
  const auto tag = [](const char * text) {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) {
      v = (v << 8) | static_cast<uint64_t>(static_cast<unsigned char>(text[i]));
    }
    return v;
  };
  EXPECT_EQ(mrdist::kTagObject, tag("OBJSEED1"));
  EXPECT_EQ(mrdist::kTagDropout, tag("DROPOUT1"));
  EXPECT_STREQ(mrdist::kAlgorithm, "MRDIST-1");
  EXPECT_EQ(mrdist::kMaxBaseSeed, 999999999999999ULL);
}

TEST(Mrdist, GoldenUniformsAreExact)
{
  const uint64_t car = mrdist::object_seed(kBase, "NPC_Car_1");
  EXPECT_EQ(mrdist::u01(car, 0), 0.47180387289432024);
  EXPECT_EQ(mrdist::u01(car, 1), 0.3467998336145032);

  const uint64_t drop = mrdist::dropout_seed(kBase);
  EXPECT_EQ(mrdist::u01(drop, 0), 0.6494517364983766);
  EXPECT_EQ(mrdist::u01(drop, 1), 0.7364053681583135);
  EXPECT_EQ(mrdist::u01(drop, 2), 0.9036469442413301);
  EXPECT_EQ(mrdist::u01(drop, 3), 0.2304097804570514);

  // 0 과 1 은 나오지 않는다 (ln(0) 을 막는다)
  for (uint64_t seed : {uint64_t{0}, car, drop, ~uint64_t{0}}) {
    for (uint64_t c = 0; c < 20000; ++c) {
      const double u = mrdist::u01(seed, c);
      ASSERT_GT(u, 0.0);
      ASSERT_LT(u, 1.0);
    }
  }
}

TEST(Mrdist, GoldenGaussians)
{
  const uint64_t car = mrdist::object_seed(kBase, "NPC_Car_1");
  const struct
  {
    uint64_t k;
    double lon;
    double lat;
  } golden[] = {
    {0, -0.7003756082673686, 1.0059114344229763},
    {1, 1.240929661537579, -2.6310726412698884},
    {2, -0.3890819506789319, 0.4217375882632737},
    {1000, -2.6649254703441576, -0.6826765099462565},
  };
  for (const auto & g : golden) {
    const auto z = mrdist::gauss2(car, g.k);
    EXPECT_NEAR(z.lon, g.lon, 1e-12) << "k=" << g.k;
    EXPECT_NEAR(z.lat, g.lat, 1e-12) << "k=" << g.k;
  }
}

TEST(Mrdist, DropoutProbabilities)
{
  auto p = mrdist::dropout_prob(0.75, 0.5);
  EXPECT_EQ(p.p_hide, 0.06666666666666667);
  EXPECT_EQ(p.p_show, 0.2);

  p = mrdist::dropout_prob(0.95, 0.1);
  EXPECT_EQ(p.p_hide, 0.05263157894736847);
  EXPECT_EQ(p.p_show, 1.0);

  p = mrdist::dropout_prob(0.5, 0.2);
  EXPECT_EQ(p.p_hide, 0.5);
  EXPECT_EQ(p.p_show, 0.5);

  // 0.1 s 보다 짧은 평균 끊김은 0.1 s 로 본다
  const auto a = mrdist::dropout_prob(0.75, 0.03);
  const auto b = mrdist::dropout_prob(0.75, 0.1);
  EXPECT_EQ(a.p_hide, b.p_hide);
  EXPECT_EQ(a.p_show, b.p_show);
  EXPECT_EQ(a.p_show, 1.0);

  // 가장자리는 나눗셈 없이 막는다 (사슬은 이 값에서 뽑지 않는다)
  p = mrdist::dropout_prob(0.0, 0.5);
  EXPECT_EQ(p.p_hide, 1.0);
  EXPECT_EQ(p.p_show, 0.0);
  p = mrdist::dropout_prob(1.0, 0.5);
  EXPECT_EQ(p.p_hide, 0.0);
  EXPECT_EQ(p.p_show, 1.0);
}

// =============================================================================
// 트리거 문법
// =============================================================================

TEST(DisturbanceParser, AcceptsCanonicalGrammar)
{
  {  // 빈 문자열·공백만 = 상태 조회
    for (const char * text : {"", "   ", " \t "}) {
      const auto r = parse_disturbance_trigger(text);
      EXPECT_TRUE(r.ok) << "'" << text << "'";
      EXPECT_EQ(r.cmd.kind, DisturbanceKind::kNone);
      EXPECT_FALSE(r.cmd.clear);
    }
  }
  {  // 전체 해제
    for (const char * text : {"disturbance, clear", "disturbance,clear", "  disturbance\t,\tclear  "}) {
      const auto r = parse_disturbance_trigger(text);
      EXPECT_TRUE(r.ok) << text;
      EXPECT_EQ(r.cmd.kind, DisturbanceKind::kClearAll);
      EXPECT_TRUE(r.cmd.clear);
    }
  }
  {  // 종류별 해제
    const std::pair<const char *, DisturbanceKind> cases[] = {
      {"disturbance, position, clear", DisturbanceKind::kPosition},
      {"disturbance, dropout, clear", DisturbanceKind::kDropout},
      {"disturbance,blackout,clear", DisturbanceKind::kBlackout},
    };
    for (const auto & c : cases) {
      const auto r = parse_disturbance_trigger(c.first);
      EXPECT_TRUE(r.ok) << c.first;
      EXPECT_EQ(r.cmd.kind, c.second);
      EXPECT_TRUE(r.cmd.clear);
    }
  }
  {  // position: 값 4 개
    const auto r = parse_disturbance_trigger("disturbance, position, 1.4, 0.4, -0.5, 0.2");
    ASSERT_TRUE(r.ok) << r.code;
    EXPECT_EQ(r.cmd.kind, DisturbanceKind::kPosition);
    EXPECT_FALSE(r.cmd.clear);
    EXPECT_EQ(r.cmd.lon_offset_m, 1.4);
    EXPECT_EQ(r.cmd.lon_std_m, 0.4);
    EXPECT_EQ(r.cmd.lat_offset_m, -0.5);
    EXPECT_EQ(r.cmd.lat_std_m, 0.2);

    const auto tight = parse_disturbance_trigger("disturbance,position,-100,100,1e2,0");
    ASSERT_TRUE(tight.ok) << tight.code;
    EXPECT_EQ(tight.cmd.lon_offset_m, -100.0);
    EXPECT_EQ(tight.cmd.lon_std_m, 100.0);
    EXPECT_EQ(tight.cmd.lat_offset_m, 100.0);
    EXPECT_EQ(tight.cmd.lat_std_m, 0.0);

    // 끄는 값도 문법으로는 정상이다 (-0 은 0 으로 읽는다)
    const auto zeros = parse_disturbance_trigger("disturbance, position, 0, -0, 0.0, 0");
    ASSERT_TRUE(zeros.ok) << zeros.code;
    EXPECT_FALSE(std::signbit(zeros.cmd.lon_std_m));
    EXPECT_TRUE(is_off_equivalent(zeros.cmd));
    EXPECT_FALSE(is_off_equivalent(r.cmd));
  }
  {  // dropout: 값 2 개
    const auto r = parse_disturbance_trigger("disturbance, dropout, 0.75, 0.5");
    ASSERT_TRUE(r.ok) << r.code;
    EXPECT_EQ(r.cmd.kind, DisturbanceKind::kDropout);
    EXPECT_EQ(r.cmd.keep_prob, 0.75);
    EXPECT_EQ(r.cmd.mean_gap_s, 0.5);
    EXPECT_FALSE(is_off_equivalent(r.cmd));

    // keep 0 (전부 숨김)·keep 1 (끔) 에서는 gap 이 0 이어도 된다
    const auto always = parse_disturbance_trigger("disturbance, dropout, 0, 0");
    ASSERT_TRUE(always.ok) << always.code;
    EXPECT_FALSE(is_off_equivalent(always.cmd));
    const auto off = parse_disturbance_trigger("disturbance, dropout, 1, 0");
    ASSERT_TRUE(off.ok) << off.code;
    EXPECT_TRUE(is_off_equivalent(off.cmd));
    EXPECT_TRUE(parse_disturbance_trigger("disturbance, dropout, 0.5, 600").ok);
  }
  {  // blackout: 값 2 개. recover 는 1·0·1.0·0.0·true·false
    const std::pair<const char *, bool> cases[] = {
      {"disturbance, blackout, 0.5, 1", true},      {"disturbance, blackout, 0.5, 0", false},
      {"disturbance, blackout, 0.5, 1.0", true},    {"disturbance, blackout, 0.5, 0.0", false},
      {"disturbance, blackout, 0.5, true", true},   {"disturbance, blackout, 0.5, false", false},
    };
    for (const auto & c : cases) {
      const auto r = parse_disturbance_trigger(c.first);
      ASSERT_TRUE(r.ok) << c.first << " " << r.code;
      EXPECT_EQ(r.cmd.kind, DisturbanceKind::kBlackout);
      EXPECT_FALSE(r.cmd.clear);
      EXPECT_EQ(r.cmd.duration_s, 0.5);
      EXPECT_EQ(r.cmd.recover, c.second) << c.first;
    }
    // 끝없는 끊김 (정상 명령) 과 끄는 값
    const auto forever = parse_disturbance_trigger("disturbance, blackout, 0, 0");
    ASSERT_TRUE(forever.ok) << forever.code;
    EXPECT_FALSE(forever.cmd.recover);
    EXPECT_FALSE(is_off_equivalent(forever.cmd));
    const auto off = parse_disturbance_trigger("disturbance, blackout, 0, 1");
    ASSERT_TRUE(off.ok) << off.code;
    EXPECT_TRUE(is_off_equivalent(off.cmd));
    EXPECT_TRUE(parse_disturbance_trigger("disturbance, blackout, 3600, 1").ok);
  }
}

TEST(DisturbanceParser, RejectsMalformed)
{
  const std::pair<std::string, const char *> cases[] = {
    // 머리말
    {"disturb, clear", "BAD_PREFIX"},
    {"Disturbance, clear", "BAD_PREFIX"},
    {"DISTURBANCE, dropout, 0.75, 0.5", "BAD_PREFIX"},
    {"clear", "BAD_PREFIX"},
    {", clear", "BAD_PREFIX"},
    {"disturbance clear", "BAD_PREFIX"},
    // 종류
    {"disturbance, jitter, 1", "UNKNOWN_KIND"},
    {"disturbance, Position, 1, 0, 0, 0", "UNKNOWN_KIND"},
    {"disturbance, , clear", "UNKNOWN_KIND"},
    {"disturbance, heading, 1, 0.5", "UNSUPPORTED_KIND"},
    {"disturbance, mount, 1", "UNSUPPORTED_KIND"},
    // 값 개수
    {"disturbance", "WRONG_ARITY"},
    {"disturbance, clear, 1", "WRONG_ARITY"},
    {"disturbance, position, 1, 0, 0", "WRONG_ARITY"},
    {"disturbance, position, 1, 0, 0, 0, 0", "WRONG_ARITY"},
    {"disturbance, position", "WRONG_ARITY"},
    {"disturbance, dropout, 0.75", "WRONG_ARITY"},
    {"disturbance, dropout, 0.75, 0.5,", "WRONG_ARITY"},
    {"disturbance, blackout, 1", "WRONG_ARITY"},
    {"disturbance, blackout, Clear", "WRONG_ARITY"},
    // 숫자
    {"disturbance, dropout, , 0.5", "BAD_NUMBER"},
    {"disturbance, dropout, nan, 0.5", "BAD_NUMBER"},
    {"disturbance, dropout, 0.5, inf", "BAD_NUMBER"},
    {"disturbance, position, -inf, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, position, +1, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, position, 0x10, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, position, 1.4m, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, position, 1 2, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, position, 1e999, 0, 0, 0", "BAD_NUMBER"},
    {"disturbance, blackout, 0.5, yes", "BAD_NUMBER"},
    {"disturbance, blackout, true, 1", "BAD_NUMBER"},
    // 범위
    {"disturbance, dropout, 1.5, 1", "OUT_OF_RANGE"},
    {"disturbance, dropout, -0.1, 1", "OUT_OF_RANGE"},
    {"disturbance, dropout, 0.75, 0", "OUT_OF_RANGE"},
    {"disturbance, dropout, 0.75, 601", "OUT_OF_RANGE"},
    {"disturbance, dropout, 0, -1", "OUT_OF_RANGE"},
    {"disturbance, position, 0, -0.1, 0, 0", "OUT_OF_RANGE"},
    {"disturbance, position, 0, 0, 0, 100.5", "OUT_OF_RANGE"},
    {"disturbance, position, 100.5, 0, 0, 0", "OUT_OF_RANGE"},
    {"disturbance, position, 0, 0, -101, 0", "OUT_OF_RANGE"},
    {"disturbance, blackout, 0.5, 2", "OUT_OF_RANGE"},
    {"disturbance, blackout, -1, 1", "OUT_OF_RANGE"},
    {"disturbance, blackout, 3601, 1", "OUT_OF_RANGE"},
    // 길이
    {"disturbance, clear" + std::string(300, ' '), "TOO_LONG"},
    {std::string(257, ' '), "TOO_LONG"},
  };
  for (const auto & c : cases) {
    const auto r = parse_disturbance_trigger(c.first);
    EXPECT_FALSE(r.ok) << c.first;
    EXPECT_EQ(r.code, c.second) << c.first;
    EXPECT_FALSE(r.detail.empty()) << c.first;
  }
  // 256 자까지는 받는다
  EXPECT_TRUE(parse_disturbance_trigger("disturbance, clear" + std::string(238, ' ')).ok);
}

// =============================================================================
// 명령 제어기
// =============================================================================

TEST(DisturbanceController, EpochStartClearsAndResets)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "run_1", 1001).ok);
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run_1").ok);
  const auto before = b.handle.get();
  ASSERT_TRUE(before->dropout.on);
  EXPECT_EQ(before->epoch_id, 1u);

  const auto reply = b.send("disturbance, clear", "run_2", 2002);
  EXPECT_TRUE(reply.ok);
  EXPECT_TRUE(has_token(reply.message, "EPOCH")) << reply.message;
  EXPECT_EQ(reply.run, "run_2");
  EXPECT_EQ(reply.base_seed, 2002u);
  const auto after = b.handle.get();
  EXPECT_EQ(after->epoch_id, before->epoch_id + 1);
  EXPECT_EQ(after->base_seed, 2002u);
  EXPECT_EQ(after->run, "run_2");
  EXPECT_FALSE(after->position.on);
  EXPECT_FALSE(after->dropout.on);
  EXPECT_FALSE(after->blackout.on);
  // 명령 번호는 구간이 바뀌어도 이어 센다
  EXPECT_GT(after->command_seq, before->command_seq);

  // 시드만 넣은 호출(문장 없음)도 새 구간을 연다
  const auto only = b.send("", "run_3", 3003);
  EXPECT_TRUE(only.ok);
  EXPECT_TRUE(has_token(only.message, "EPOCH")) << only.message;
  EXPECT_EQ(b.handle.get()->epoch_id, after->epoch_id + 1);
  EXPECT_EQ(b.handle.get()->command_seq, after->command_seq);

  // 범위 밖 시드는 거절하고 아무것도 바꾸지 않는다
  const auto kept = b.handle.get();
  const auto bad = b.send("disturbance, clear", "run_4", mrdist::kMaxBaseSeed + 1);
  EXPECT_FALSE(bad.ok);
  EXPECT_TRUE(starts_with(bad.message, "BAD_SEED:")) << bad.message;
  EXPECT_EQ(b.handle.get(), kept);
  EXPECT_EQ(bad.run, "run_3");
  EXPECT_EQ(bad.base_seed, 3003u);
  EXPECT_TRUE(b.send("disturbance, clear", "run_4", mrdist::kMaxBaseSeed).ok);
}

TEST(DisturbanceController, AutoSeedsWhenNoEpoch)
{
  Bench b(options_with_seed(42));
  // 해제·상태 조회는 시드를 만들지 않는다
  EXPECT_TRUE(b.send("disturbance, clear").ok);
  EXPECT_TRUE(b.send("disturbance, dropout, clear", "run_x").ok);
  const auto status = b.send("");
  EXPECT_TRUE(status.ok);
  EXPECT_TRUE(has_token(status.message, "STATUS")) << status.message;
  EXPECT_EQ(status.base_seed, 0u);
  EXPECT_EQ(b.handle.get()->epoch_id, 0u);
  EXPECT_EQ(b.handle.get()->base_seed, 0u);

  const auto reply = b.send("disturbance, dropout, 0.75, 0.5", "run_auto");
  EXPECT_TRUE(reply.ok);
  EXPECT_TRUE(starts_with(reply.message, "OK AUTO_SEED")) << reply.message;
  EXPECT_EQ(reply.base_seed, 42u);
  EXPECT_EQ(reply.run, "run_auto");
  const auto s = b.handle.get();
  EXPECT_EQ(s->epoch_id, 1u);
  EXPECT_EQ(s->base_seed, 42u);
  EXPECT_EQ(s->run, "run_auto");
  EXPECT_TRUE(s->dropout.on);

  // 구간이 있으면 다시 뽑지 않는다
  const auto again = b.send("disturbance, position, 1, 0, 0, 0", "run_auto");
  EXPECT_TRUE(again.ok);
  EXPECT_FALSE(has_token(again.message, "AUTO_SEED")) << again.message;
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);

  // 시드 원천이 범위 밖 값을 줘도 1 ~ 999999999999999 로 들어온다
  for (uint64_t raw : {uint64_t{0}, mrdist::kMaxBaseSeed + 1, ~uint64_t{0}}) {
    Bench c(options_with_seed(raw));
    const auto r = c.send("disturbance, blackout, 0, 0");
    ASSERT_TRUE(r.ok);
    EXPECT_GE(r.base_seed, 1u);
    EXPECT_LE(r.base_seed, mrdist::kMaxBaseSeed);
  }
  // 기본 원천 (random_device)
  DisturbanceHandle handle;
  DisturbanceController ctl(handle, DisturbanceController::Options{});
  const auto r = ctl.handle(make_request("disturbance, blackout, 0, 0"));
  ASSERT_TRUE(r.ok);
  EXPECT_GE(r.base_seed, 1u);
  EXPECT_LE(r.base_seed, mrdist::kMaxBaseSeed);
}

TEST(DisturbanceController, RejectsRunMismatch)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  const auto before = b.handle.get();

  // 다른 run 표시를 단 켜는 명령 (시드 없음) 만 거절한다
  const auto reply = b.send("disturbance, dropout, 0.75, 0.5", "B");
  EXPECT_FALSE(reply.ok);
  EXPECT_TRUE(starts_with(reply.message, "RUN_MISMATCH:")) << reply.message;
  EXPECT_EQ(b.handle.get(), before);
  EXPECT_EQ(reply.run, "A");
  EXPECT_EQ(reply.base_seed, 1001u);
  EXPECT_EQ(b.ctl.counters().rejected, 1u);
  EXPECT_EQ(b.ctl.counters().last_reject, reply.message);

  // 상태 조회는 run 을 보지 않는다
  const auto status = b.send("", "B");
  EXPECT_TRUE(status.ok);
  EXPECT_EQ(b.handle.get(), before);

  // 표시가 없거나 같으면 받는다
  EXPECT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "").ok);
  EXPECT_TRUE(b.handle.get()->dropout.on);
  EXPECT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "A").ok);
  EXPECT_TRUE(b.handle.get()->position.on);
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);

  // 구간의 run 이 비어 있으면 (수동 사용) 어떤 표시도 거절하지 않는다
  Bench manual;
  ASSERT_TRUE(manual.send("disturbance, clear", "", 5).ok);
  EXPECT_TRUE(manual.send("disturbance, blackout, 0, 0", "B").ok);
}

TEST(DisturbanceController, ClearWithForeignRunTagIsAccepted)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  ASSERT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "A").ok);
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "A").ok);
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "A").ok);

  // 종류별 해제
  auto r = b.send("disturbance, dropout, clear", "B");
  EXPECT_TRUE(r.ok) << r.message;
  EXPECT_FALSE(b.handle.get()->dropout.on);
  EXPECT_TRUE(b.handle.get()->position.on);
  // 끄는 값
  r = b.send("disturbance, position, 0, 0, 0, 0", "B");
  EXPECT_TRUE(r.ok) << r.message;
  EXPECT_TRUE(has_token(r.message, "POSITION_OFF")) << r.message;
  EXPECT_FALSE(b.handle.get()->position.on);
  EXPECT_TRUE(b.handle.get()->blackout.on);
  // 전체 해제
  r = b.send("disturbance, clear", "B");
  EXPECT_TRUE(r.ok) << r.message;
  EXPECT_FALSE(b.handle.get()->blackout.on);
  // 구간은 그대로
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);
  EXPECT_EQ(b.handle.get()->run, "A");
  EXPECT_EQ(b.ctl.counters().rejected, 0u);
}

TEST(DisturbanceController, SameRunAndSeedKeepsEpoch)
{
  Bench b;
  const auto snap = car1_snapshot();
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  EXPECT_EQ(b.step(snap).frame_seq, 0u);
  EXPECT_EQ(b.step(snap).frame_seq, 1u);

  // 같은 (run, 시드) 를 실은 명령은 구간을 다시 열지 않고 문장만 적용한다
  auto r = b.send("disturbance, dropout, 0.75, 0.5", "A", 1001);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(has_token(r.message, "EPOCH_KEPT")) << r.message;
  EXPECT_FALSE(has_token(r.message, "EPOCH")) << r.message;
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);
  EXPECT_TRUE(b.handle.get()->dropout.on);
  auto f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 2u);
  EXPECT_TRUE(f.dropout_on);

  // 시작 호출(해제 + 시드)을 한 번 더 보내도 구간·카운터는 이어진다 (문장은 그대로 해제)
  r = b.send("disturbance, clear", "A", 1001);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(has_token(r.message, "EPOCH_KEPT")) << r.message;
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);
  EXPECT_FALSE(b.handle.get()->dropout.on);
  EXPECT_EQ(b.step(snap).frame_seq, 3u);

  // 문장 없이 같은 (run, 시드): 아무것도 바뀌지 않는다
  const auto kept = b.handle.get();
  r = b.send("", "A", 1001);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(has_token(r.message, "EPOCH_KEPT")) << r.message;
  EXPECT_EQ(b.handle.get(), kept);

  // run 이나 시드가 다르면 새 구간 — 전부 끈 뒤 문장을 적용하고 frame_seq 는 0 부터
  ASSERT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "A").ok);
  r = b.send("disturbance, dropout, 0.75, 0.5", "A", 1002);
  EXPECT_TRUE(r.ok);
  EXPECT_TRUE(has_token(r.message, "EPOCH")) << r.message;
  EXPECT_EQ(b.handle.get()->epoch_id, 2u);
  EXPECT_FALSE(b.handle.get()->position.on);
  EXPECT_TRUE(b.handle.get()->dropout.on);
  EXPECT_EQ(b.step(snap).frame_seq, 0u);
  r = b.send("disturbance, clear", "B", 1002);
  EXPECT_TRUE(has_token(r.message, "EPOCH")) << r.message;
  EXPECT_EQ(b.handle.get()->epoch_id, 3u);
  EXPECT_EQ(b.step(snap).frame_seq, 0u);
}

TEST(DisturbanceController, RejectsOnInRealCarButAllowsClear)
{
  DisturbanceController::Options real = options_with_seed(7);
  real.real_car = true;
  Bench b(real);
  for (const char * on :
    {"disturbance, position, 1, 0, 0, 0", "disturbance, dropout, 0.75, 0.5",
      "disturbance, blackout, 1, 1", "disturbance, blackout, 0, 0", "disturbance, dropout, 0, 0"}) {
    const auto r = b.send(on, "run", 0);
    EXPECT_FALSE(r.ok) << on;
    EXPECT_TRUE(starts_with(r.message, "REAL_CAR:")) << r.message;
    // 시드를 실어도 켜는 명령은 거절하고 구간도 열지 않는다
    const auto seeded = b.send(on, "run", 99);
    EXPECT_FALSE(seeded.ok) << on;
    EXPECT_TRUE(starts_with(seeded.message, "REAL_CAR:")) << seeded.message;
  }
  EXPECT_EQ(b.handle.get()->epoch_id, 0u);
  for (const char * pass :
    {"disturbance, clear", "disturbance, position, clear", "disturbance, dropout, clear",
      "disturbance, blackout, clear", "disturbance, position, 0, 0, 0, 0",
      "disturbance, dropout, 1, 0.5", "disturbance, blackout, 0, 1", ""}) {
    const auto r = b.send(pass, "run", 0);
    EXPECT_TRUE(r.ok) << pass << " -> " << r.message;
  }
  // 시작 호출(해제 + 시드)도 통과한다
  EXPECT_TRUE(b.send("disturbance, clear", "run", 1234).ok);

  DisturbanceController::Options no_gt = options_with_seed(7);
  no_gt.ground_truth = false;
  Bench c(no_gt);
  const auto r = c.send("disturbance, dropout, 0.75, 0.5");
  EXPECT_FALSE(r.ok);
  EXPECT_TRUE(starts_with(r.message, "NO_GROUND_TRUTH:")) << r.message;
  EXPECT_TRUE(c.send("disturbance, clear").ok);
  EXPECT_TRUE(c.send("").ok);
}

TEST(DisturbanceController, KindClearOnlyClearsThatKind)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  ASSERT_TRUE(b.send("  disturbance, position, 1.4, 0.4, -0.5, 0.2  ", "A").ok);
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "A").ok);
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "A").ok);

  ASSERT_TRUE(b.send("disturbance, dropout, clear", "A").ok);
  auto s = b.handle.get();
  EXPECT_FALSE(s->dropout.on);
  EXPECT_EQ(s->dropout.keep_prob, 1.0);
  EXPECT_TRUE(s->position.on);
  EXPECT_EQ(s->position.trigger, "disturbance, position, 1.4, 0.4, -0.5, 0.2");  // 바깥 공백만 뗀 원문
  EXPECT_EQ(s->position.lon_offset_m, 1.4);
  EXPECT_EQ(s->position.lat_std_m, 0.2);
  EXPECT_TRUE(s->blackout.on);
  EXPECT_EQ(s->blackout.trigger, "disturbance, blackout, 0, 0");

  ASSERT_TRUE(b.send("disturbance, position, clear", "A").ok);
  s = b.handle.get();
  EXPECT_FALSE(s->position.on);
  EXPECT_TRUE(s->blackout.on);
  ASSERT_TRUE(b.send("disturbance, blackout, clear", "A").ok);
  EXPECT_FALSE(b.handle.get()->blackout.on);

  // 새 position 명령은 네 값을 통째로 바꾼다
  ASSERT_TRUE(b.send("disturbance, position, 1, 0.1, 2, 0.2", "A").ok);
  ASSERT_TRUE(b.send("disturbance, position, 3, 0, 0, 0", "A").ok);
  s = b.handle.get();
  EXPECT_EQ(s->position.lon_offset_m, 3.0);
  EXPECT_EQ(s->position.lon_std_m, 0.0);
  EXPECT_EQ(s->position.lat_offset_m, 0.0);
  EXPECT_EQ(s->position.lat_std_m, 0.0);
  EXPECT_EQ(s->position.trigger, "disturbance, position, 3, 0, 0, 0");
}

TEST(DisturbanceController, OffEquivalentValuesTurnKindOff)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  const auto all_on = [&b]() {
    ASSERT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "A").ok);
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "A").ok);
    ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "A").ok);
  };

  all_on();
  auto r = b.send("disturbance, position, 0, 0, 0, 0", "A");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.message, "OK POSITION_OFF");
  auto s = b.handle.get();
  EXPECT_FALSE(s->position.on);
  EXPECT_TRUE(s->dropout.on);
  EXPECT_TRUE(s->blackout.on);

  all_on();
  r = b.send("disturbance, dropout, 1, 0.5", "A");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.message, "OK DROPOUT_OFF");
  s = b.handle.get();
  EXPECT_TRUE(s->position.on);
  EXPECT_FALSE(s->dropout.on);
  EXPECT_TRUE(s->blackout.on);

  all_on();
  r = b.send("disturbance, blackout, 0, 1", "A");
  EXPECT_TRUE(r.ok);
  EXPECT_EQ(r.message, "OK BLACKOUT_OFF");
  s = b.handle.get();
  EXPECT_TRUE(s->position.on);
  EXPECT_TRUE(s->dropout.on);
  EXPECT_FALSE(s->blackout.on);

  // 끝없는 끊김·전부 숨김은 켜는 명령이다. SATURATED 는 0 < keep < 1 에서만 붙는다
  r = b.send("disturbance, blackout, 0, 0", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_TRUE(b.handle.get()->blackout.on);
  r = b.send("disturbance, dropout, 0, 0", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_TRUE(b.handle.get()->dropout.on);
  r = b.send("disturbance, dropout, 0.75, 0.05", "A");  // 끊김 평균 < 0.1 s
  EXPECT_EQ(r.message, "OK SATURATED");
  r = b.send("disturbance, dropout, 0.01, 0.5", "A");  // 보이는 구간 평균 < 0.1 s
  EXPECT_EQ(r.message, "OK SATURATED");
  // 협력사 예시 값은 포화가 아니다
  for (const char * row :
    {"disturbance, dropout, 0.75, 0.5", "disturbance, dropout, 0.75, 1",
      "disturbance, dropout, 0.75, 2", "disturbance, dropout, 0.75, 1.5"}) {
    EXPECT_EQ(b.send(row, "A").message, "OK") << row;
  }
}

TEST(DisturbanceController, RejectIsAtomic)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "A").ok);
  const auto before = b.handle.get();
  const auto accepted = b.ctl.counters().accepted;

  // 시드를 같이 실은 거절 호출은 구간을 열지 않는다
  const std::pair<const char *, const char *> cases[] = {
    {"disturbance, dropout, 1.5, 1", "OUT_OF_RANGE"},
    {"disturbance, mount, 1", "UNSUPPORTED_KIND"},
    {"disturbance, position, 1, 2, 3", "WRONG_ARITY"},
    {"disturbance, blackout, x, 1", "BAD_NUMBER"},
    {"nonsense", "BAD_PREFIX"},
  };
  uint64_t rejected = 0;
  for (const auto & c : cases) {
    const auto r = b.send(c.first, "B", 2002);
    EXPECT_FALSE(r.ok) << c.first;
    EXPECT_TRUE(starts_with(r.message, std::string(c.second) + ": ")) << r.message;
    EXPECT_EQ(b.handle.get(), before) << c.first;
    EXPECT_EQ(r.run, "A");
    EXPECT_EQ(r.base_seed, 1001u);
    // 읽지 못한 문장은 기본값 명령으로 답한다
    EXPECT_EQ(r.command.kind, DisturbanceKind::kNone);
    EXPECT_FALSE(r.command.clear);
    EXPECT_EQ(r.command.keep_prob, 0.0);
    EXPECT_EQ(b.ctl.counters().rejected, ++rejected);
  }
  const auto too_long = b.send(std::string(300, 'x'), "B", 2002);
  EXPECT_TRUE(starts_with(too_long.message, "TOO_LONG:")) << too_long.message;
  EXPECT_EQ(b.handle.get(), before);
  EXPECT_EQ(b.ctl.counters().accepted, accepted);
  EXPECT_TRUE(b.handle.get()->dropout.on);
  EXPECT_EQ(b.handle.get()->epoch_id, 1u);
}

TEST(DisturbanceController, ClearOnRunEndSkipsReplacedForeignRun)
{
  for (const char * reason : {"idle", "timeout", "shutdown"}) {
    Bench b;
    ASSERT_TRUE(b.send("disturbance, clear", "run_1", 1001).ok);
    ASSERT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "run_1").ok);
    ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run_1").ok);
    const auto before = b.handle.get();
    EXPECT_TRUE(b.ctl.clear_on_run_end("run_1", reason)) << reason;
    const auto s = b.handle.get();
    EXPECT_FALSE(s->position.on);
    EXPECT_FALSE(s->dropout.on);
    EXPECT_FALSE(s->blackout.on);
    EXPECT_EQ(s->epoch_id, before->epoch_id);
    EXPECT_EQ(s->base_seed, 1001u);
    EXPECT_EQ(s->run, "run_1");
    EXPECT_EQ(b.ctl.counters().auto_clears, 1u);
    // 걸린 것이 없으면 false, 설정도 그대로
    EXPECT_FALSE(b.ctl.clear_on_run_end("run_1", reason));
    EXPECT_EQ(b.handle.get(), s);
  }
  {  // 다음 런이 이미 구간을 연 뒤 앞 런이 replaced 로 끝났다 — 건드리지 않는다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run_2", 2002).ok);
    const auto before = b.handle.get();
    EXPECT_FALSE(b.ctl.clear_on_run_end("run_1", "replaced"));
    EXPECT_EQ(b.handle.get(), before);
    // 같은 run 이 replaced 로 끝나면 끈다
    EXPECT_TRUE(b.ctl.clear_on_run_end("run_2", "replaced"));
    EXPECT_FALSE(b.handle.get()->dropout.on);
    EXPECT_EQ(b.handle.get()->epoch_id, before->epoch_id);
  }
  {  // 구간의 run 이 비어 있으면 (수동) replaced 여도 끈다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5").ok);
    EXPECT_TRUE(b.ctl.clear_on_run_end("run_1", "replaced"));
    EXPECT_FALSE(b.handle.get()->dropout.on);
  }
  {  // 다른 run 이라도 replaced 가 아니면 끈다 (이름이 어긋나도 외란이 남지 않게)
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run_2", 2002).ok);
    EXPECT_TRUE(b.ctl.clear_on_run_end("something_else", "idle"));
    EXPECT_FALSE(b.handle.get()->dropout.on);
  }
}

TEST(DisturbanceController, SecondEpochSameCommandOrderStillStartsBlackout)
{
  const auto snap = car1_snapshot();
  {  // 제어기를 거친 두 런: 같은 순서(해제, blackout 0.5 1) — 둘째 런도 정확히 5 프레임 숨긴다
    Bench b;
    for (int run = 1; run <= 2; ++run) {
      const std::string tag = "run_" + std::to_string(run);
      ASSERT_TRUE(b.send("disturbance, clear", tag, 1000 + static_cast<uint64_t>(run)).ok);
      b.step(snap);
      b.step(snap);
      ASSERT_TRUE(b.send("disturbance, blackout, 0.5, 1", tag).ok);
      EXPECT_EQ(b.pattern(snap, 8), "HHHHH...") << tag;
    }
    // 둘째 런의 keep 0 dropout 도 건 프레임부터 숨긴다
    ASSERT_TRUE(b.send("disturbance, clear", "run_3", 1003).ok);
    b.step(snap);
    ASSERT_TRUE(b.send("disturbance, dropout, 0, 0.5", "run_3").ok);
    EXPECT_EQ(b.pattern(snap, 3), "HHH");
    ASSERT_TRUE(b.send("disturbance, clear", "run_4", 1004).ok);
    b.step(snap);
    ASSERT_TRUE(b.send("disturbance, dropout, 0, 0.5", "run_4").ok);
    EXPECT_EQ(b.pattern(snap, 3), "HHH");
  }
  {  // 시드 호출과 켜는 문장이 한 요청에 같이 와도 건다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, blackout, 0.5, 1", "run_1", 1001).ok);
    EXPECT_EQ(b.pattern(snap, 7), "HHHHH..");
    ASSERT_TRUE(b.send("disturbance, blackout, 0.5, 1", "run_2", 1002).ok);
    EXPECT_EQ(b.pattern(snap, 7), "HHHHH..");
  }
  {  // 적용기만: 구간마다 cmd_seq 가 같은 값으로 되풀이돼도 (epoch_id, cmd_seq) 로 새 명령을 가린다
    ObjectDisturber d;
    DisturbanceSettings s;
    s.base_seed = kBase;
    s.command_seq = 2;
    s.blackout.on = true;
    s.blackout.duration_s = 0.5;
    s.blackout.recover = true;
    s.blackout.trigger = "disturbance, blackout, 0.5, 1";
    s.blackout.cmd_seq = 2;
    s.dropout.on = true;
    s.dropout.keep_prob = 0.0;
    s.dropout.cmd_seq = 1;
    int64_t t = 0;
    for (uint64_t epoch = 1; epoch <= 3; ++epoch) {
      s.epoch_id = epoch;
      for (int i = 0; i < 7; ++i) {
        t += 100 * kMs;
        const auto f = d.apply(s, snap, t);
        EXPECT_EQ(f.frame_seq, static_cast<uint64_t>(i));
        EXPECT_EQ(f.blackout_on, i < 5) << "epoch " << epoch << " frame " << i;
        EXPECT_FALSE(f.objects_visible);  // keep 0 dropout 은 계속 숨긴다
      }
    }
  }
}

TEST(DisturbanceController, IdenticalResendKeepsStatePosition)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "A").ok);
  const auto before = b.handle.get();

  // 값이 같으면 (글자가 달라도) 그대로 둔다
  for (const char * same :
    {"disturbance, position, 1.4, 0.4, -0.5, 0.2", "disturbance,position,1.40,0.4,-0.5,2e-1"}) {
    const auto r = b.send(same, "A");
    EXPECT_TRUE(r.ok);
    EXPECT_EQ(r.message, "OK UNCHANGED") << same;
    EXPECT_EQ(b.handle.get(), before);
  }
  // 시드를 같이 실은 다시 보냄 (리그가 늘 이렇게 보낸다)
  auto r = b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "A", 1001);
  EXPECT_EQ(r.message, "OK EPOCH_KEPT UNCHANGED");
  EXPECT_EQ(b.handle.get(), before);

  // 값이 다르면 새 명령
  r = b.send("disturbance, position, 1.4, 0.4, -0.5, 0.3", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_NE(b.handle.get(), before);
  EXPECT_GT(b.handle.get()->position.cmd_seq, before->position.cmd_seq);
  // 해제 뒤에는 같은 값도 새 명령
  ASSERT_TRUE(b.send("disturbance, position, clear", "A").ok);
  r = b.send("disturbance, position, 1.4, 0.4, -0.5, 0.3", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_TRUE(b.handle.get()->position.on);
}

TEST(DisturbanceController, IdenticalResendKeepsStateDropout)
{
  const auto snap = car1_snapshot();
  Bench b;
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "A", kBase).ok);
  std::string seen = b.pattern(snap, 22);  // k = 0..21 — k = 21 은 숨김 구간(21~23) 의 첫 프레임
  ASSERT_EQ(seen.back(), 'H');
  const auto before = b.handle.get();

  // 숨김 구간 한가운데에서 같은 명령을 다시 보낸다 — 사슬을 다시 시작하지 않는다 (다시 시작하면 다음 프레임이 보인다)
  auto r = b.send("disturbance, dropout, 0.75, 0.5", "A", kBase);
  EXPECT_EQ(r.message, "OK EPOCH_KEPT UNCHANGED");
  EXPECT_EQ(b.handle.get(), before);
  r = b.send("disturbance, dropout, 0.750, 5e-1", "A");
  EXPECT_EQ(r.message, "OK UNCHANGED");
  EXPECT_EQ(b.handle.get(), before);
  seen += b.pattern(snap, 38);
  EXPECT_EQ(seen, kPattern075At0);

  // 값이 다르면 사슬을 새로 시작한다 (건 프레임은 보인다)
  r = b.send("disturbance, dropout, 0.5, 0.2", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_GT(b.handle.get()->dropout.cmd_seq, before->dropout.cmd_seq);
  EXPECT_TRUE(b.step(snap).objects_visible);
  // 해제 뒤 같은 값은 새 명령
  ASSERT_TRUE(b.send("disturbance, dropout, clear", "A").ok);
  r = b.send("disturbance, dropout, 0.5, 0.2", "A");
  EXPECT_EQ(r.message, "OK");
}

TEST(DisturbanceController, IdenticalResendKeepsStateBlackout)
{
  const auto snap = car1_snapshot();
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "A", 1001).ok);
  // 엔진이 아직 프레임을 내기 전에 다시 와도 그대로
  ASSERT_TRUE(b.send("disturbance, blackout, 0.5, 1", "A").ok);
  const auto before = b.handle.get();
  EXPECT_EQ(b.send("disturbance, blackout, 0.5, 1", "A").message, "OK UNCHANGED");
  EXPECT_EQ(b.handle.get(), before);

  EXPECT_EQ(b.pattern(snap, 3), "HHH");
  // 진행 중에 같은 명령 — 시간을 다시 재지 않는다 (다시 재면 5 프레임을 더 숨긴다)
  auto r = b.send("disturbance, blackout, 0.50, true", "A", 1001);
  EXPECT_EQ(r.message, "OK EPOCH_KEPT UNCHANGED");
  EXPECT_EQ(b.handle.get(), before);
  EXPECT_EQ(b.pattern(snap, 4), "HH..");

  // 자동 복구 뒤에는 같은 명령이 새로 건다
  r = b.send("disturbance, blackout, 0.5, 1", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_GT(b.handle.get()->blackout.cmd_seq, before->blackout.cmd_seq);
  EXPECT_EQ(b.pattern(snap, 7), "HHHHH..");

  // 진행 중에 값이 다르면 새로 건다
  ASSERT_TRUE(b.send("disturbance, blackout, 0.5, 1", "A").ok);
  EXPECT_EQ(b.pattern(snap, 2), "HH");
  r = b.send("disturbance, blackout, 0.3, 1", "A");
  EXPECT_EQ(r.message, "OK");
  EXPECT_EQ(b.pattern(snap, 5), "HHH..");

  // 끝없는 끊김은 해제 전까지 진행 중이다
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "A").ok);
  EXPECT_EQ(b.pattern(snap, 3), "HHH");
  EXPECT_EQ(b.send("disturbance, blackout, 0, 0", "A").message, "OK UNCHANGED");
  ASSERT_TRUE(b.send("disturbance, blackout, clear", "A").ok);
  EXPECT_EQ(b.send("disturbance, blackout, 0, 0", "A").message, "OK");

  // 복구된 blackout 만 남아 있으면 런 종료 해제는 "걸려 있던 것 없음" 으로 답하되 설정은 정리한다
  ASSERT_TRUE(b.send("disturbance, clear", "A").ok);
  ASSERT_TRUE(b.send("disturbance, blackout, 0.2, 1", "A").ok);
  EXPECT_EQ(b.pattern(snap, 3), "HH.");
  EXPECT_TRUE(b.handle.get()->blackout.on);
  EXPECT_FALSE(b.ctl.clear_on_run_end("A", "idle"));
  EXPECT_FALSE(b.handle.get()->blackout.on);
}

// =============================================================================
// 프레임 적용
// =============================================================================

TEST(ObjectDisturber, PassThroughWhenNothingActive)
{
  auto second = make_actor("NPC_Ped_1", 95.0, 210.0);
  second.object_type = mgv::ObjectType::kPedestrian;
  second.bbox_location = {95.1, 210.2, 0.9};
  const auto snap = make_snapshot(45.0, {make_actor("Car_1", 110.0, 200.0), second});

  Bench b;
  for (uint64_t k = 0; k < 3; ++k) {
    const auto f = b.step(snap);
    EXPECT_EQ(f.frame_seq, k);
    EXPECT_EQ(f.frame_period_s, 0.1);
    EXPECT_FALSE(f.fault_flag);
    EXPECT_FALSE(f.position_on);
    EXPECT_FALSE(f.dropout_on);
    EXPECT_FALSE(f.blackout_on);
    EXPECT_TRUE(f.objects_visible);
    EXPECT_TRUE(f.active_triggers.empty());
    EXPECT_EQ(f.lon_offset_m, 0.0);
    EXPECT_EQ(f.lon_std_m, 0.0);
    EXPECT_EQ(f.lat_offset_m, 0.0);
    EXPECT_EQ(f.lat_std_m, 0.0);
    EXPECT_EQ(f.keep_prob, 1.0);
    EXPECT_EQ(f.mean_gap_s, 0.0);
    EXPECT_EQ(f.blackout_duration_s, 0.0);
    EXPECT_FALSE(f.blackout_recover);
    // 시드 구간이 없으면 시드는 전부 0
    EXPECT_EQ(f.base_seed, 0u);
    EXPECT_EQ(f.dropout_seed, 0u);
    EXPECT_TRUE(f.event.empty());

    ASSERT_EQ(f.sent_actors.size(), snap.actors.size());
    ASSERT_EQ(f.objects.size(), snap.actors.size());
    for (size_t i = 0; i < snap.actors.size(); ++i) {
      expect_actor_eq(f.sent_actors[i], snap.actors[i]);
      const auto & rec = f.objects[i];
      EXPECT_EQ(rec.name, snap.actors[i].id);
      EXPECT_EQ(rec.unique_id, morai_to_autoware::object_unique_id_from_name(snap.actors[i].id));
      EXPECT_EQ(rec.seed, 0u);
      EXPECT_TRUE(rec.visible);
      EXPECT_EQ(rec.noise_lon_m, 0.0);
      EXPECT_EQ(rec.noise_lat_m, 0.0);
      EXPECT_EQ(rec.dx_m, 0.0);
      EXPECT_EQ(rec.dy_m, 0.0);
    }
  }

  // 시드 구간만 열어도 (외란 없음) 그대로 지나간다 — 시드는 채워진다
  ASSERT_TRUE(b.send("disturbance, clear", "run", kBase).ok);
  const auto f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 0u);
  EXPECT_FALSE(f.fault_flag);
  EXPECT_EQ(f.base_seed, kBase);
  EXPECT_EQ(f.dropout_seed, 0xFCC081FFA48A068BULL);
  ASSERT_EQ(f.objects.size(), 2u);
  EXPECT_EQ(f.objects[0].seed, 0x4E6987CFFE11F9DBULL);
  EXPECT_EQ(f.objects[1].seed, 0x11C916F768E8BDE8ULL);
  EXPECT_FALSE(f.event.empty());  // 구간 시작은 한 줄 남긴다
  for (size_t i = 0; i < snap.actors.size(); ++i) {
    expect_actor_eq(f.sent_actors[i], snap.actors[i]);
    EXPECT_EQ(f.objects[i].dx_m, 0.0);
    EXPECT_EQ(f.objects[i].dy_m, 0.0);
  }
  EXPECT_TRUE(b.step(snap).event.empty());
}

TEST(ObjectDisturber, BiasRotatesEgoFrameToWorld)
{
  {  // 자차 yaw 45°, 앞으로 1 m → world (cos 45°, sin 45°)
    const auto snap = car1_snapshot();
    Bench b;
    ASSERT_TRUE(b.send("disturbance, position, 1, 0, 0, 0", "run", kBase).ok);
    const auto f = b.step(snap);
    ASSERT_EQ(f.sent_actors.size(), 1u);
    const auto & sent = f.sent_actors[0];
    EXPECT_NEAR(sent.position.x - 110.0, 0.7071067811865476, 1e-12);
    EXPECT_NEAR(sent.position.y - 200.0, 0.7071067811865475, 1e-12);
    EXPECT_NEAR(sent.position.x, 110.70710678118655, 1e-9);
    EXPECT_NEAR(sent.position.y, 200.70710678118655, 1e-9);
    // 위치 x·y 말고는 GT 그대로
    expect_actor_eq_except_xy(sent, snap.actors[0]);
    EXPECT_TRUE(f.position_on);
    EXPECT_TRUE(f.fault_flag);
    EXPECT_TRUE(f.objects_visible);
    EXPECT_EQ(f.lon_offset_m, 1.0);
    EXPECT_EQ(f.active_triggers, std::vector<std::string>{"disturbance, position, 1, 0, 0, 0"});
    // 표준편차 0 이면 잡음 칸은 정확히 +0.0
    EXPECT_EQ(f.objects[0].noise_lon_m, 0.0);
    EXPECT_EQ(f.objects[0].noise_lat_m, 0.0);
    EXPECT_FALSE(std::signbit(f.objects[0].noise_lon_m));
    EXPECT_FALSE(std::signbit(f.objects[0].noise_lat_m));
  }
  {  // 자차 yaw 90° (북쪽을 본다), 왼쪽으로 1 m → 서쪽 (-1, 0)
    const auto snap = make_snapshot(90.0, {make_actor("Car_1", 110.0, 200.0)});
    Bench b;
    ASSERT_TRUE(b.send("disturbance, position, 0, 0, 1, 0", "run", kBase).ok);
    const auto f = b.step(snap);
    ASSERT_EQ(f.sent_actors.size(), 1u);
    EXPECT_NEAR(f.sent_actors[0].position.x - 110.0, -1.0, 1e-12);
    EXPECT_NEAR(f.sent_actors[0].position.y - 200.0, 0.0, 1e-12);
    expect_actor_eq_except_xy(f.sent_actors[0], snap.actors[0]);
  }
  {  // 자차 yaw 0°, 앞 2 m·왼 3 m → (+2, +3)
    const auto snap = make_snapshot(0.0, {make_actor("Car_1", 110.0, 200.0)});
    Bench b;
    ASSERT_TRUE(b.send("disturbance, position, 2, 0, 3, 0", "run", kBase).ok);
    const auto f = b.step(snap);
    EXPECT_EQ(f.sent_actors[0].position.x, 112.0);
    EXPECT_EQ(f.sent_actors[0].position.y, 203.0);
    EXPECT_EQ(f.objects[0].dx_m, 2.0);
    EXPECT_EQ(f.objects[0].dy_m, 3.0);
  }
}

// 26.11.11 _ YSH _ 신규 — 자차 yaw 가 NaN 이면 그 프레임은 옮기지 않고(GT 그대로·유한), 다음 정상 프레임부터 다시 옮긴다
TEST(ObjectDisturber, NonFiniteEgoYawSkipsShift)
{
  const auto bad = make_snapshot(std::nan(""), {make_actor("Car_1", 110.0, 200.0)});
  const auto good = make_snapshot(0.0, {make_actor("Car_1", 110.0, 200.0)});
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 2, 0, 3, 0", "run", kBase).ok);
  const auto f0 = b.step(good);
  EXPECT_EQ(f0.sent_actors[0].position.x, 112.0);
  const auto f1 = b.step(bad);
  ASSERT_EQ(f1.sent_actors.size(), 1u);
  EXPECT_TRUE(f1.position_on);
  EXPECT_EQ(f1.sent_actors[0].position.x, 110.0);
  EXPECT_EQ(f1.sent_actors[0].position.y, 200.0);
  EXPECT_EQ(f1.objects[0].dx_m, 0.0);
  EXPECT_EQ(f1.objects[0].dy_m, 0.0);
  EXPECT_NE(f1.event.find("yaw"), std::string::npos);
  const auto f2 = b.step(bad);
  EXPECT_TRUE(f2.event.empty());  // 같은 상태가 이어지면 다시 적지 않는다
  const auto f3 = b.step(good);
  EXPECT_EQ(f3.sent_actors[0].position.x, 112.0);
  EXPECT_EQ(f3.sent_actors[0].position.y, 203.0);
  EXPECT_NE(f3.event.find("yaw"), std::string::npos);  // 회복도 한 줄
}

// 26.11.11 _ YSH _ 신규 — 길이 제한은 UTF-8 글자 중간에서 끊지 않는다
TEST(DisturbanceText, Utf8PrefixKeepsWholeCharacters)
{
  EXPECT_EQ(utf8_prefix("abcdef", 10), "abcdef");
  EXPECT_EQ(utf8_prefix("abcdef", 3), "abc");
  EXPECT_EQ(utf8_prefix("abcdef", 0), "");
  const std::string ko = "\xea\xb0\x80\xeb\x82\x98";  // 가나 (3 + 3 바이트)
  EXPECT_EQ(utf8_prefix(ko, 6), ko);
  EXPECT_EQ(utf8_prefix(ko, 5), "\xea\xb0\x80");
  EXPECT_EQ(utf8_prefix(ko, 4), "\xea\xb0\x80");
  EXPECT_EQ(utf8_prefix(ko, 3), "\xea\xb0\x80");
  EXPECT_EQ(utf8_prefix(ko, 2), "");
  EXPECT_EQ(utf8_prefix("a" + ko, 3), "a");
}

TEST(ObjectDisturber, BboxCentreMovesWithPosition)
{
  auto with_centre = make_actor("Car_1", 110.0, 200.0);
  with_centre.bbox_location = {111.3, 200.4, 0.8};
  auto z_only_centre = make_actor("Car_2", 120.0, 190.0);
  z_only_centre.bbox_location = {0.0, 0.0, 0.8};  // 한 성분만 0 이 아니어도 "있다" 로 본다 (to_tracked_object 와 같은 판정)
  const auto without_centre = make_actor("Car_3", 130.0, 180.0);
  const auto snap = make_snapshot(45.0, {with_centre, z_only_centre, without_centre});

  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "run", kBase).ok);
  const auto f = b.step(snap);
  ASSERT_EQ(f.sent_actors.size(), 3u);

  const auto wx0 = f.sent_actors[0].position.x - 110.0;
  const auto wy0 = f.sent_actors[0].position.y - 200.0;
  EXPECT_NE(wx0, 0.0);
  EXPECT_NEAR(f.sent_actors[0].bbox_location.x - 111.3, wx0, 1e-12);
  EXPECT_NEAR(f.sent_actors[0].bbox_location.y - 200.4, wy0, 1e-12);
  EXPECT_EQ(f.sent_actors[0].bbox_location.z, 0.8);

  const auto wx1 = f.sent_actors[1].position.x - 120.0;
  const auto wy1 = f.sent_actors[1].position.y - 190.0;
  EXPECT_NEAR(f.sent_actors[1].bbox_location.x, wx1, 1e-12);
  EXPECT_NEAR(f.sent_actors[1].bbox_location.y, wy1, 1e-12);
  EXPECT_EQ(f.sent_actors[1].bbox_location.z, 0.8);

  // 상자 중심이 없으면(전부 0) 정확히 0 으로 남는다 — to_tracked_object 가 계속 position 을 쓴다
  EXPECT_NE(f.sent_actors[2].position.x, 130.0);
  EXPECT_EQ(f.sent_actors[2].bbox_location.x, 0.0);
  EXPECT_EQ(f.sent_actors[2].bbox_location.y, 0.0);
  EXPECT_EQ(f.sent_actors[2].bbox_location.z, 0.0);

  // TrackedObject pose = 옮긴 상자 중심 (있으면) / 옮긴 position (없으면)
  const MapOrigin origin;
  const auto obj0 = morai_to_autoware::to_tracked_object(f.sent_actors[0], origin);
  EXPECT_NEAR(obj0.kinematics.pose_with_covariance.pose.position.x, 111.3 + wx0, 1e-9);
  EXPECT_NEAR(obj0.kinematics.pose_with_covariance.pose.position.y, 200.4 + wy0, 1e-9);
  const auto obj2 = morai_to_autoware::to_tracked_object(f.sent_actors[2], origin);
  EXPECT_NEAR(obj2.kinematics.pose_with_covariance.pose.position.x, f.sent_actors[2].position.x, 1e-9);
  EXPECT_NEAR(obj2.kinematics.pose_with_covariance.pose.position.y, f.sent_actors[2].position.y, 1e-9);
}

TEST(ObjectDisturber, NoiseMatchesGoldenPerObjectPerFrame)
{
  const auto snap = car1_snapshot();
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "run", kBase).ok);
  const auto f = b.step(snap);
  ASSERT_EQ(f.frame_seq, 0u);
  ASSERT_EQ(f.objects.size(), 1u);
  ASSERT_EQ(f.sent_actors.size(), 1u);
  const auto & rec = f.objects[0];
  EXPECT_EQ(rec.seed, 0x4E6987CFFE11F9DBULL);
  EXPECT_NEAR(rec.noise_lon_m, -0.5327034947241036, 1e-12);
  EXPECT_NEAR(rec.noise_lat_m, 0.10049929485672632, 1e-12);
  EXPECT_NEAR(f.sent_actors[0].position.x, 110.89576089787559, 1e-9);
  EXPECT_NEAR(f.sent_actors[0].position.y, 200.33078158248438, 1e-9);
  EXPECT_NEAR(rec.dx_m, 0.89576089787559, 1e-9);
  EXPECT_NEAR(rec.dy_m, 0.33078158248437717, 1e-9);
  EXPECT_EQ(rec.dx_m, f.sent_actors[0].position.x - 110.0);
  EXPECT_EQ(rec.dy_m, f.sent_actors[0].position.y - 200.0);
  EXPECT_EQ(f.lon_offset_m, 1.4);
  EXPECT_EQ(f.lon_std_m, 0.4);
  EXPECT_EQ(f.lat_offset_m, -0.5);
  EXPECT_EQ(f.lat_std_m, 0.2);

  // 뒤 프레임도 식 그대로: noise = std × gauss2(object_seed, k)
  for (uint64_t k = 1; k < 20; ++k) {
    const auto g = b.step(snap);
    ASSERT_EQ(g.frame_seq, k);
    const auto z = mrdist::gauss2(0x4E6987CFFE11F9DBULL, k);
    EXPECT_NEAR(g.objects[0].noise_lon_m, 0.4 * z.lon, 1e-12);
    EXPECT_NEAR(g.objects[0].noise_lat_m, 0.2 * z.lat, 1e-12);
    const double d_lon = 1.4 + 0.4 * z.lon;
    const double d_lat = -0.5 + 0.2 * z.lat;
    const double c = std::cos(morai_to_autoware::radians(45.0));
    const double s = std::sin(morai_to_autoware::radians(45.0));
    EXPECT_NEAR(g.objects[0].dx_m, d_lon * c - d_lat * s, 1e-9);
    EXPECT_NEAR(g.objects[0].dy_m, d_lon * s + d_lat * c, 1e-9);
  }

  // 표준편차 0 인 축은 잡음이 정확히 +0.0
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0, -0.5, 0.2", "run").ok);
  auto g = b.step(snap);
  EXPECT_EQ(g.objects[0].noise_lon_m, 0.0);
  EXPECT_FALSE(std::signbit(g.objects[0].noise_lon_m));
  EXPECT_NE(g.objects[0].noise_lat_m, 0.0);
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0", "run").ok);
  g = b.step(snap);
  EXPECT_NE(g.objects[0].noise_lon_m, 0.0);
  EXPECT_EQ(g.objects[0].noise_lat_m, 0.0);
  EXPECT_FALSE(std::signbit(g.objects[0].noise_lat_m));
}

TEST(ObjectDisturber, ObjectsAreIndependentAndStable)
{
  const auto car = make_actor("NPC_Car_1", 110.0, 200.0);
  const auto ped = make_actor("NPC_Ped_1", 95.0, 210.0);
  const auto extra = make_actor("NPC_Car_2", 130.0, 180.0);
  const auto two = make_snapshot(30.0, {car, ped});
  const auto three_front = make_snapshot(30.0, {extra, car, ped});
  const auto one = make_snapshot(30.0, {ped});
  const char * trigger = "disturbance, position, 0.5, 0.4, -0.3, 0.2";

  Bench a;
  Bench b;
  Bench c;
  Bench replay;
  for (Bench * bench : {&a, &b, &c, &replay}) {
    ASSERT_TRUE(bench->send(trigger, "run", kBase).ok);
  }
  for (int i = 0; i < 30; ++i) {
    const auto fa = a.step(two);
    const auto fb = b.step(three_front);  // 셋째 actor 가 앞에 끼어든다
    const auto fc = c.step(one);          // 첫 actor 가 사라진다
    const auto fr = replay.step(two);
    ASSERT_EQ(fa.objects.size(), 2u);
    ASSERT_EQ(fb.objects.size(), 3u);
    ASSERT_EQ(fc.objects.size(), 1u);

    // 두 객체는 시드도 잡음도 다르다
    EXPECT_EQ(fa.objects[0].seed, 0x8DF6BF7B5DBCDFCBULL);
    EXPECT_EQ(fa.objects[1].seed, 0x11C916F768E8BDE8ULL);
    EXPECT_NE(fa.objects[0].noise_lon_m, fa.objects[1].noise_lon_m);
    EXPECT_NE(fa.objects[0].noise_lat_m, fa.objects[1].noise_lat_m);

    // 다른 객체가 끼거나 빠져도 같은 프레임 번호의 값은 그대로 (이름으로만 정해진다)
    EXPECT_EQ(fb.objects[1].name, "NPC_Car_1");
    EXPECT_EQ(fb.objects[1].noise_lon_m, fa.objects[0].noise_lon_m);
    EXPECT_EQ(fb.objects[1].noise_lat_m, fa.objects[0].noise_lat_m);
    EXPECT_EQ(fb.objects[1].dx_m, fa.objects[0].dx_m);
    EXPECT_EQ(fb.objects[2].noise_lon_m, fa.objects[1].noise_lon_m);
    EXPECT_EQ(fb.objects[2].dy_m, fa.objects[1].dy_m);
    EXPECT_EQ(fc.objects[0].noise_lon_m, fa.objects[1].noise_lon_m);
    EXPECT_EQ(fc.objects[0].noise_lat_m, fa.objects[1].noise_lat_m);

    // 같은 시드로 다시 돌리면 프레임이 똑같다
    ASSERT_EQ(fr.objects.size(), 2u);
    for (size_t j = 0; j < 2; ++j) {
      EXPECT_EQ(fr.objects[j].seed, fa.objects[j].seed);
      EXPECT_EQ(fr.objects[j].noise_lon_m, fa.objects[j].noise_lon_m);
      EXPECT_EQ(fr.objects[j].noise_lat_m, fa.objects[j].noise_lat_m);
      EXPECT_EQ(fr.objects[j].dx_m, fa.objects[j].dx_m);
      EXPECT_EQ(fr.objects[j].dy_m, fa.objects[j].dy_m);
      EXPECT_EQ(fr.sent_actors[j].position.x, fa.sent_actors[j].position.x);
      EXPECT_EQ(fr.sent_actors[j].position.y, fa.sent_actors[j].position.y);
    }
  }

  // 시드가 다르면 값도 다르다
  Bench other;
  ASSERT_TRUE(other.send(trigger, "run", kBase + 1).ok);
  Bench same;
  ASSERT_TRUE(same.send(trigger, "run", kBase).ok);
  const auto fo = other.step(two);
  const auto fs = same.step(two);
  EXPECT_NE(fo.objects[0].seed, fs.objects[0].seed);
  EXPECT_NE(fo.objects[0].noise_lon_m, fs.objects[0].noise_lon_m);
}

TEST(ObjectDisturber, DxDyEqualSentMinusGtExactly)
{
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "run", kBase).ok);
  for (int i = 0; i < 200; ++i) {
    // 프레임마다 GT 와 자차 yaw 를 바꾼다 (자릿수가 다른 좌표를 섞는다)
    auto car = make_actor("NPC_Car_1", 110.0 + 0.37 * i, 200.0 - 0.11 * i);
    auto ped = make_actor("NPC_Ped_1", 300123.456 + 0.013 * i, 4100456.789 + 0.029 * i);
    ped.bbox_location = {ped.position.x + 0.1, ped.position.y - 0.2, 0.9};
    const auto snap = make_snapshot(-170.0 + 1.7 * i, {car, ped});
    const auto f = b.step(snap);
    ASSERT_EQ(f.sent_actors.size(), 2u);
    for (size_t j = 0; j < 2; ++j) {
      EXPECT_TRUE(f.objects[j].dx_m == f.sent_actors[j].position.x - snap.actors[j].position.x);
      EXPECT_TRUE(f.objects[j].dy_m == f.sent_actors[j].position.y - snap.actors[j].position.y);
      EXPECT_NE(f.objects[j].dx_m, 0.0);
    }
  }
}

TEST(ObjectDisturber, DropoutPatternGolden)
{
  const auto snap = car1_snapshot();
  ASSERT_EQ(std::string(kPattern075At0).size(), 60u);
  ASSERT_EQ(std::string(kPattern075At7).size(), 60u);
  ASSERT_EQ(std::string(kPattern050At0).size(), 60u);
  {  // keep 0.75, gap 0.5, k = 0 에 건다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run", kBase).ok);
    std::string seen;
    for (int i = 0; i < 60; ++i) {
      const auto f = b.step(snap);
      EXPECT_EQ(f.frame_seq, static_cast<uint64_t>(i));
      EXPECT_TRUE(f.dropout_on);
      EXPECT_TRUE(f.fault_flag);  // 보이는 프레임에도 걸려 있는 동안은 true
      EXPECT_EQ(f.keep_prob, 0.75);
      EXPECT_EQ(f.mean_gap_s, 0.5);
      EXPECT_EQ(f.dropout_seed, 0xFCC081FFA48A068BULL);
      EXPECT_EQ(f.active_triggers, std::vector<std::string>{"disturbance, dropout, 0.75, 0.5"});
      EXPECT_EQ(f.sent_actors.empty(), !f.objects_visible);
      seen += f.objects_visible ? '.' : 'H';
    }
    EXPECT_EQ(seen, kPattern075At0);
    EXPECT_EQ(seen[0], '.');  // 건 프레임은 보인다
  }
  {  // 같은 값을 k = 7 에 건다 — 뽑는 값은 프레임 번호로 정해진다 (명령별 카운터가 아니다)
    Bench b;
    ASSERT_TRUE(b.send("disturbance, clear", "run", kBase).ok);
    EXPECT_EQ(b.pattern(snap, 7), ".......");
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run").ok);
    const auto on_frame = b.step(snap);
    EXPECT_EQ(on_frame.frame_seq, 7u);
    EXPECT_TRUE(on_frame.objects_visible);
    EXPECT_EQ("." + b.pattern(snap, 59), kPattern075At7);
  }
  {  // keep 0.5, gap 0.2
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0.5, 0.2", "run", kBase).ok);
    EXPECT_EQ(b.pattern(snap, 60), kPattern050At0);
  }
  {  // keep 0: 건 프레임부터 전부 숨긴다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 0, 0.5", "run", kBase).ok);
    for (int i = 0; i < 40; ++i) {
      const auto f = b.step(snap);
      EXPECT_FALSE(f.objects_visible);
      EXPECT_TRUE(f.dropout_on);
      EXPECT_EQ(f.keep_prob, 0.0);
    }
  }
  {  // keep 1 은 끄는 값: 숨기지 않고 dropout_on 도 false
    Bench b;
    ASSERT_TRUE(b.send("disturbance, dropout, 1, 0.5", "run", kBase).ok);
    for (int i = 0; i < 40; ++i) {
      const auto f = b.step(snap);
      EXPECT_TRUE(f.objects_visible);
      EXPECT_FALSE(f.dropout_on);
      EXPECT_FALSE(f.fault_flag);
      EXPECT_EQ(f.keep_prob, 1.0);
    }
  }
}

TEST(ObjectDisturber, DropoutLongRunStatistics)
{
  const auto snap = make_snapshot(0.0, {});
  Bench b;
  ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run", kBase).ok);
  int hidden = 0;
  int outages = 0;
  bool was_hidden = false;
  for (int i = 0; i < 20000; ++i) {
    const bool now_hidden = !b.step(snap).objects_visible;
    if (now_hidden) {
      ++hidden;
      if (!was_hidden) {
        ++outages;
      }
    }
    was_hidden = now_hidden;
  }
  // 결정적이다: 보이는 비율 0.741, 평균 끊김 4.89 프레임
  EXPECT_EQ(hidden, 5173);
  EXPECT_EQ(outages, 1058);
}

TEST(ObjectDisturber, BlackoutHidesForDurationThenRecovers)
{
  const auto snap = car1_snapshot();
  const std::vector<std::string> trigger{"disturbance, blackout, 0.5, 1"};
  Bench b;
  ASSERT_TRUE(b.send("disturbance, clear", "run", kBase).ok);
  EXPECT_TRUE(b.step(snap).objects_visible);

  // 100 ms 간격: 정확히 5 프레임
  ASSERT_TRUE(b.send(trigger[0], "run").ok);
  for (int i = 0; i < 5; ++i) {
    const auto f = b.step(snap);
    EXPECT_FALSE(f.objects_visible) << i;
    EXPECT_TRUE(f.blackout_on);
    EXPECT_TRUE(f.fault_flag);
    EXPECT_EQ(f.active_triggers, trigger);
    EXPECT_EQ(f.blackout_duration_s, 0.5);
    EXPECT_TRUE(f.blackout_recover);
    EXPECT_TRUE(f.sent_actors.empty());
    EXPECT_EQ(f.event.empty(), i != 0);  // 시작 프레임만 한 줄
  }
  auto f = b.step(snap);
  EXPECT_TRUE(f.objects_visible);
  EXPECT_FALSE(f.blackout_on);
  EXPECT_FALSE(f.fault_flag);
  EXPECT_TRUE(f.active_triggers.empty());
  EXPECT_EQ(f.blackout_duration_s, 0.0);
  EXPECT_FALSE(f.blackout_recover);
  EXPECT_EQ(f.sent_actors.size(), 1u);
  EXPECT_FALSE(f.event.empty());  // 복구도 한 줄
  EXPECT_EQ(b.disturber.blackout_recovered_seq(), b.handle.get()->blackout.cmd_seq);
  EXPECT_EQ(b.handle.blackout_recovered_seq(), b.handle.get()->blackout.cmd_seq);
  for (int i = 0; i < 5; ++i) {
    f = b.step(snap);
    EXPECT_TRUE(f.objects_visible);
    EXPECT_TRUE(f.event.empty());
  }

  // 간격이 들쭉날쭉해도 (80, 120, 100, 140, 100 ms) 5 프레임: 경과 0, 80, 200, 300, 440 → 540 에서 복구
  ASSERT_TRUE(b.send(trigger[0], "run").ok);
  EXPECT_FALSE(b.step(snap, 100).objects_visible);
  for (int64_t dt : {80, 120, 100, 140}) {
    EXPECT_FALSE(b.step(snap, dt).objects_visible) << dt;
  }
  EXPECT_TRUE(b.step(snap, 100).objects_visible);

  // 진행 중에 다른 값의 blackout 이 오면 시간을 처음부터 다시 잰다
  ASSERT_TRUE(b.send(trigger[0], "run").ok);
  EXPECT_EQ(b.pattern(snap, 3), "HHH");
  ASSERT_TRUE(b.send("disturbance, blackout, 0.4, 1", "run").ok);
  EXPECT_EQ(b.pattern(snap, 6), "HHHH..");

  // 반올림: 0.25 s 는 100 ms 격자에서 3 프레임 (경과 0, 100, 200 → 300 ≥ 250)
  ASSERT_TRUE(b.send("disturbance, blackout, 0.25, 1", "run").ok);
  EXPECT_EQ(b.pattern(snap, 5), "HHH..");
}

TEST(ObjectDisturber, BlackoutWithoutRecoverStaysUntilClear)
{
  const auto snap = car1_snapshot();
  {
    Bench b;
    // 밑에 dropout 사슬을 같이 건다 (k = 0)
    ASSERT_TRUE(b.send("disturbance, dropout, 0.75, 0.5", "run", kBase).ok);
    ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run").ok);
    for (int i = 0; i < 30; ++i) {
      const auto f = b.step(snap);
      EXPECT_FALSE(f.objects_visible);
      EXPECT_TRUE(f.blackout_on);
      EXPECT_FALSE(f.blackout_recover);
      EXPECT_EQ(f.blackout_duration_s, 0.0);
      EXPECT_EQ(
        f.active_triggers, (std::vector<std::string>{
                             "disturbance, dropout, 0.75, 0.5", "disturbance, blackout, 0, 0"}));
    }
    // blackout 만 풀면 그동안 계속 돌던 dropout 사슬이 그 자리에서 이어진다 (k = 30 ~ 59)
    ASSERT_TRUE(b.send("disturbance, blackout, clear", "run").ok);
    EXPECT_EQ(b.pattern(snap, 30), std::string(kPattern075At0).substr(30));
  }
  {
    Bench b;
    ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run", kBase).ok);
    for (int i = 0; i < 100; ++i) {
      EXPECT_FALSE(b.step(snap, 137).objects_visible);
    }
    ASSERT_TRUE(b.send("disturbance, clear", "run").ok);
    const auto f = b.step(snap);
    EXPECT_TRUE(f.objects_visible);
    EXPECT_FALSE(f.blackout_on);
    EXPECT_FALSE(f.fault_flag);
    EXPECT_TRUE(f.active_triggers.empty());
  }
  {  // duration 이 있어도 recover 0 이면 끝나지 않는다
    Bench b;
    ASSERT_TRUE(b.send("disturbance, blackout, 0.3, 0", "run", kBase).ok);
    for (int i = 0; i < 50; ++i) {
      const auto f = b.step(snap);
      EXPECT_FALSE(f.objects_visible);
      EXPECT_EQ(f.blackout_duration_s, 0.3);
    }
  }
}

TEST(ObjectDisturber, TimeAnomalyCountsNominalPeriod)
{
  const auto snap = car1_snapshot();
  const int64_t sec = 1000 * kMs;
  {
    ObjectDisturber d;
    const DisturbanceSettings off;
    EXPECT_EQ(d.apply(off, snap, 10 * sec).frame_period_s, 0.1);             // 첫 프레임
    EXPECT_DOUBLE_EQ(d.apply(off, snap, 10 * sec + 80 * kMs).frame_period_s, 0.08);
    EXPECT_DOUBLE_EQ(d.apply(off, snap, 10 * sec + 220 * kMs).frame_period_s, 0.14);
    EXPECT_EQ(d.apply(off, snap, 9 * sec).frame_period_s, 0.1);              // 역행
    EXPECT_EQ(d.apply(off, snap, 9 * sec).frame_period_s, 0.1);              // 같은 stamp
    EXPECT_EQ(d.apply(off, snap, 19 * sec).frame_period_s, 0.1);             // 10 s 건너뜀
    EXPECT_DOUBLE_EQ(d.apply(off, snap, 20 * sec).frame_period_s, 1.0);      // 딱 1 s 는 그대로
    EXPECT_EQ(d.apply(off, snap, 21 * sec + 1).frame_period_s, 0.1);         // 1 s 초과
    EXPECT_EQ(d.apply(off, snap, 21 * sec + 1 + 100 * kMs).frame_period_s, 0.1);
  }
  {  // 이상한 stamp 가 이어져도 0.3 s blackout 은 세 프레임 뒤 끝난다 (프레임당 100 ms 로 센다)
    ObjectDisturber d;
    DisturbanceSettings s;
    s.epoch_id = 1;
    s.base_seed = kBase;
    s.blackout.on = true;
    s.blackout.duration_s = 0.3;
    s.blackout.recover = true;
    s.blackout.cmd_seq = 1;
    EXPECT_FALSE(d.apply(s, snap, 100 * sec).objects_visible);  // 건 프레임
    EXPECT_FALSE(d.apply(s, snap, 90 * sec).objects_visible);   // 역행 → +100 ms
    EXPECT_FALSE(d.apply(s, snap, 90 * sec).objects_visible);   // 반복 → +100 ms
    const auto f = d.apply(s, snap, 200 * sec);                 // 건너뜀 → +100 ms = 300 ms
    EXPECT_TRUE(f.objects_visible);
    EXPECT_FALSE(f.blackout_on);
    EXPECT_EQ(d.blackout_recovered_seq(), 1u);
  }
  {  // 정상 간격이면 실제 stamp 차를 더한다: 250 ms 간격 두 번이면 0.5 s
    ObjectDisturber d;
    DisturbanceSettings s;
    s.epoch_id = 1;
    s.blackout.on = true;
    s.blackout.duration_s = 0.5;
    s.blackout.recover = true;
    s.blackout.cmd_seq = 1;
    EXPECT_FALSE(d.apply(s, snap, 100 * sec).objects_visible);
    EXPECT_FALSE(d.apply(s, snap, 100 * sec + 250 * kMs).objects_visible);
    EXPECT_TRUE(d.apply(s, snap, 100 * sec + 500 * kMs).objects_visible);
  }
}

TEST(ObjectDisturber, EpochResetsFrameSeqAndState)
{
  const auto snap = car1_snapshot();
  Bench b;
  // 시드 구간 전에는 노드 기동부터 센다
  EXPECT_EQ(b.step(snap).frame_seq, 0u);
  EXPECT_EQ(b.step(snap).frame_seq, 1u);
  EXPECT_EQ(b.step(snap).frame_seq, 2u);

  ASSERT_TRUE(b.send("disturbance, clear", "run_1", kBase).ok);
  auto f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 0u);
  EXPECT_EQ(f.base_seed, kBase);
  EXPECT_FALSE(f.event.empty());
  EXPECT_DOUBLE_EQ(f.frame_period_s, 0.1);  // 구간 첫 프레임도 직전 객체 프레임과의 간격
  EXPECT_EQ(b.step(snap).frame_seq, 1u);

  // 명령·해제는 카운터를 되돌리지 않는다
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run_1").ok);
  ASSERT_TRUE(b.send("disturbance, dropout, 0, 0.5", "run_1").ok);
  f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 2u);
  EXPECT_FALSE(f.objects_visible);
  EXPECT_TRUE(f.blackout_on);
  EXPECT_TRUE(f.dropout_on);
  EXPECT_EQ(b.step(snap).frame_seq, 3u);
  EXPECT_TRUE(b.ctl.clear_on_run_end("run_1", "idle"));
  f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 4u);  // 런 종료 해제도 구간·카운터를 그대로 둔다
  EXPECT_TRUE(f.objects_visible);
  EXPECT_EQ(f.base_seed, kBase);

  // 새 구간: 카운터 0, 사슬·blackout 상태도 지운다
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run_1").ok);
  EXPECT_FALSE(b.step(snap).objects_visible);
  ASSERT_TRUE(b.send("disturbance, clear", "run_2", kBase + 1).ok);
  f = b.step(snap);
  EXPECT_EQ(f.frame_seq, 0u);
  EXPECT_EQ(f.base_seed, kBase + 1);
  EXPECT_TRUE(f.objects_visible);
  EXPECT_FALSE(f.blackout_on);
  EXPECT_FALSE(f.dropout_on);
  EXPECT_FALSE(f.fault_flag);
  EXPECT_TRUE(f.active_triggers.empty());
  EXPECT_EQ(b.step(snap).frame_seq, 1u);
}

TEST(ObjectDisturber, HiddenFrameListsObjectsAsNotVisible)
{
  const auto snap =
    make_snapshot(45.0, {make_actor("NPC_Car_1", 110.0, 200.0), make_actor("NPC_Ped_1", 95.0, 210.0)});
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1.4, 0.4, -0.5, 0.2", "run", kBase).ok);
  ASSERT_TRUE(b.send("disturbance, blackout, 0, 0", "run").ok);
  const auto f = b.step(snap);
  EXPECT_FALSE(f.objects_visible);
  EXPECT_TRUE(f.sent_actors.empty());
  EXPECT_TRUE(f.position_on);  // 위치 외란은 걸려 있지만 보낸 것이 없다
  ASSERT_EQ(f.objects.size(), 2u);
  EXPECT_EQ(f.objects[0].name, "NPC_Car_1");
  EXPECT_EQ(f.objects[0].seed, 0x8DF6BF7B5DBCDFCBULL);
  EXPECT_EQ(f.objects[0].unique_id, morai_to_autoware::object_unique_id_from_name("NPC_Car_1"));
  EXPECT_EQ(f.objects[1].name, "NPC_Ped_1");
  EXPECT_EQ(f.objects[1].seed, 0x11C916F768E8BDE8ULL);
  for (const auto & rec : f.objects) {
    EXPECT_FALSE(rec.visible);
    EXPECT_EQ(rec.noise_lon_m, 0.0);
    EXPECT_EQ(rec.noise_lat_m, 0.0);
    EXPECT_EQ(rec.dx_m, 0.0);
    EXPECT_EQ(rec.dy_m, 0.0);
  }
  // 풀리면 같은 프레임 번호의 값이 그대로 나온다 (숨긴 프레임도 번호를 쓴다)
  ASSERT_TRUE(b.send("disturbance, blackout, clear", "run").ok);
  const auto g = b.step(snap);
  ASSERT_EQ(g.frame_seq, 1u);
  ASSERT_EQ(g.sent_actors.size(), 2u);
  EXPECT_TRUE(g.objects[0].visible);
  EXPECT_NEAR(g.objects[0].noise_lon_m, 0.4 * 1.240929661537579, 1e-12);
  EXPECT_NEAR(g.objects[0].noise_lat_m, 0.2 * -2.6310726412698884, 1e-12);
}

TEST(ObjectDisturber, ZeroActorsStillProduceAFrame)
{
  const auto empty = make_snapshot(10.0, {});
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1, 0.1, 0, 0.1", "run", kBase).ok);
  for (uint64_t k = 0; k < 5; ++k) {
    const auto f = b.step(empty);
    EXPECT_EQ(f.frame_seq, k);
    EXPECT_TRUE(f.objects.empty());
    EXPECT_TRUE(f.sent_actors.empty());
    EXPECT_TRUE(f.objects_visible);  // 숨긴 것이 아니다 — GT 객체 수와 무관
    EXPECT_TRUE(f.fault_flag);
  }
  // 객체가 나타나면 그 프레임 번호의 값을 쓴다
  const auto f = b.step(make_snapshot(10.0, {make_actor("NPC_Car_1", 1.0, 2.0)}));
  ASSERT_EQ(f.frame_seq, 5u);
  const auto z = mrdist::gauss2(0x8DF6BF7B5DBCDFCBULL, 5);
  EXPECT_NEAR(f.objects[0].noise_lon_m, 0.1 * z.lon, 1e-12);
}

// =============================================================================
// 메시지 변환
// =============================================================================

// 26.11.11 _ YSH _ 신규 — /daq/object_disturbance_status: 객체 한 칸에 외란 전(GT)·후·변화가 같은 프레임으로 맞물린다
TEST(DisturbanceMsgs, PairedListCarriesBeforeAfterAndChain)
{
  auto ped = make_actor("NPC_Ped_1", 95.0, 210.0);
  ped.object_type = mgv::ObjectType::kPedestrian;
  const auto snap = make_snapshot(30.0, {make_actor("NPC_Car_1", 110.0, 200.0), ped});
  Bench b;
  ASSERT_TRUE(b.send("disturbance, position, 1.5, 0.4, -0.5, 0.2", "run", kBase).ok);
  const auto f = b.step(snap);
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1700000000;
  stamp.nanosec = 5u;
  const auto msg = to_object_disturbance_status_list(f, stamp, 1.0);
  EXPECT_EQ(msg.header.stamp, stamp);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.frame_seq, f.frame_seq);
  EXPECT_EQ(msg.base_seed, kBase);
  EXPECT_EQ(msg.ego_x, 100.0);
  EXPECT_EQ(msg.ego_y, 200.0);
  EXPECT_EQ(msg.ego_yaw_deg, 30.0);
  ASSERT_EQ(msg.num_of_npcs, 1);
  ASSERT_EQ(msg.num_of_pedestrian, 1);
  EXPECT_EQ(msg.num_of_obstacle, 0);
  const double psi = 30.0 * M_PI / 180.0;
  for (const auto * o : {&msg.npc_list[0], &msg.pedestrian_list[0]}) {
    const auto & gt = (o->name == "NPC_Car_1") ? snap.actors[0] : snap.actors[1];
    const auto gt_msg = morai_to_autoware::to_daq_object_status(gt, 1.0);
    // 외란 전 = GT 그대로 (/daq/object_status 와 같은 계산)
    EXPECT_EQ(o->before, gt_msg);
    // 외란 후 = 위치 x·y 만 다르고 나머지는 같다
    EXPECT_EQ(o->unique_id, gt_msg.unique_id);
    EXPECT_EQ(o->type, gt_msg.type);
    EXPECT_EQ(o->heading, gt_msg.heading);
    EXPECT_EQ(o->velocity, gt_msg.velocity);
    EXPECT_EQ(o->size, gt_msg.size);
    EXPECT_EQ(o->position.z, gt_msg.position.z);
    EXPECT_TRUE(o->visible);
    // 변화의 사슬: z → noise → d → 회전 → dx·dy → position
    EXPECT_NE(o->seed, 0u);
    EXPECT_EQ(o->noise_lon_m, 0.4 * o->z_lon);
    EXPECT_EQ(o->noise_lat_m, 0.2 * o->z_lat);
    EXPECT_EQ(o->d_lon_m, 1.5 + o->noise_lon_m);
    EXPECT_EQ(o->d_lat_m, -0.5 + o->noise_lat_m);
    EXPECT_NEAR(o->dx_m, o->d_lon_m * std::cos(psi) - o->d_lat_m * std::sin(psi), 1e-12);
    EXPECT_NEAR(o->dy_m, o->d_lon_m * std::sin(psi) + o->d_lat_m * std::cos(psi), 1e-12);
    EXPECT_EQ(o->position.x - o->before.position.x, o->dx_m);
    EXPECT_EQ(o->position.y - o->before.position.y, o->dy_m);
  }
  EXPECT_NE(msg.npc_list[0].seed, msg.pedestrian_list[0].seed);  // 객체마다 시드가 다르다

  // 숨김 프레임 — 객체는 그대로 실리고(외란 전·시드), 외란 후 값은 NaN, 변화는 0
  Bench h;
  ASSERT_TRUE(h.send("disturbance, blackout, 0, 0", "run", kBase).ok);
  const auto hf = h.step(snap);
  const auto hidden = to_object_disturbance_status_list(hf, stamp, 1.0);
  ASSERT_EQ(hidden.num_of_npcs, 1);
  const auto & o = hidden.npc_list[0];
  EXPECT_FALSE(o.visible);
  EXPECT_EQ(o.before, morai_to_autoware::to_daq_object_status(snap.actors[0], 1.0));
  EXPECT_EQ(o.name, "NPC_Car_1");
  EXPECT_EQ(o.size, o.before.size);
  EXPECT_TRUE(std::isnan(o.position.x) && std::isnan(o.position.y) && std::isnan(o.heading));
  EXPECT_TRUE(std::isnan(o.velocity.x) && std::isnan(o.acceleration.x));
  EXPECT_EQ(o.dx_m, 0.0);
  EXPECT_EQ(o.z_lon, 0.0);
  EXPECT_NE(o.seed, 0u);
}

TEST(DisturbanceMsgs, StatusCarriesEveryField)
{
  DisturbanceFrame frame;
  frame.frame_seq = 1234;
  frame.frame_period_s = 0.12;
  frame.fault_flag = true;
  frame.position_on = true;
  frame.dropout_on = true;
  frame.blackout_on = true;
  frame.objects_visible = false;
  frame.active_triggers = {"p", "d", "b"};
  frame.lon_offset_m = 1.5;
  frame.lon_std_m = 0.25;
  frame.lat_offset_m = -0.75;
  frame.lat_std_m = 0.125;
  frame.keep_prob = 0.625;
  frame.mean_gap_s = 2.5;
  frame.blackout_duration_s = 3.5;
  frame.blackout_recover = true;
  frame.base_seed = kBase;
  frame.dropout_seed = 0xFCC081FFA48A068BULL;
  ObjectDisturbanceRecord rec;
  rec.unique_id = -123456;
  rec.name = "NPC_Car_1";
  rec.seed = 0x8DF6BF7B5DBCDFCBULL;
  rec.visible = false;
  rec.noise_lon_m = 0.5;
  rec.noise_lat_m = -0.25;
  rec.dx_m = 1.25;
  rec.dy_m = -2.5;
  frame.objects = {rec, ObjectDisturbanceRecord{}};
  frame.sent_actors = {make_actor("NPC_Car_1", 1.0, 2.0)};
  frame.event = "x";

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1700000000;
  stamp.nanosec = 123456789u;
  const auto msg = to_disturbance_status_msg(frame, stamp);
  EXPECT_EQ(msg.header.stamp, stamp);
  EXPECT_EQ(msg.header.frame_id, "map");
  EXPECT_EQ(msg.frame_seq, 1234u);
  EXPECT_EQ(msg.frame_period_s, 0.12);
  EXPECT_TRUE(msg.fault_flag);
  EXPECT_TRUE(msg.position_on);
  EXPECT_TRUE(msg.dropout_on);
  EXPECT_TRUE(msg.blackout_on);
  EXPECT_FALSE(msg.objects_visible);
  EXPECT_EQ(msg.active_triggers, (std::vector<std::string>{"p", "d", "b"}));
  EXPECT_EQ(msg.lon_offset_m, 1.5);
  EXPECT_EQ(msg.lon_std_m, 0.25);
  EXPECT_EQ(msg.lat_offset_m, -0.75);
  EXPECT_EQ(msg.lat_std_m, 0.125);
  EXPECT_EQ(msg.keep_prob, 0.625);
  EXPECT_EQ(msg.mean_gap_s, 2.5);
  EXPECT_EQ(msg.blackout_duration_s, 3.5);
  EXPECT_TRUE(msg.blackout_recover);
  EXPECT_EQ(msg.base_seed, kBase);
  EXPECT_EQ(msg.dropout_seed, 0xFCC081FFA48A068BULL);
  EXPECT_EQ(msg.seed_algorithm, "MRDIST-1");
  ASSERT_EQ(msg.objects.size(), 2u);
  EXPECT_EQ(msg.objects[0].unique_id, -123456);
  EXPECT_EQ(msg.objects[0].name, "NPC_Car_1");
  EXPECT_EQ(msg.objects[0].seed, 0x8DF6BF7B5DBCDFCBULL);
  EXPECT_FALSE(msg.objects[0].visible);
  EXPECT_EQ(msg.objects[0].noise_lon_m, 0.5);
  EXPECT_EQ(msg.objects[0].noise_lat_m, -0.25);
  EXPECT_EQ(msg.objects[0].dx_m, 1.25);
  EXPECT_EQ(msg.objects[0].dy_m, -2.5);
  EXPECT_TRUE(msg.objects[1].visible);
  EXPECT_EQ(msg.objects[1].name, "");

  // 기본 프레임 (외란 없음) 의 꺼짐 값
  const auto idle = to_disturbance_status_msg(DisturbanceFrame{}, stamp);
  EXPECT_FALSE(idle.fault_flag);
  EXPECT_TRUE(idle.objects_visible);
  EXPECT_EQ(idle.keep_prob, 1.0);
  EXPECT_EQ(idle.mean_gap_s, 0.0);
  EXPECT_EQ(idle.frame_period_s, 0.1);
  EXPECT_EQ(idle.seed_algorithm, "MRDIST-1");
  EXPECT_TRUE(idle.active_triggers.empty());
  EXPECT_TRUE(idle.objects.empty());

  // 명령 여섯 꼴의 type·clear
  const struct
  {
    const char * trigger;
    const char * type;
    bool clear;
  } forms[] = {
    {"", "", false},
    {"disturbance, clear", "clear", true},
    {"disturbance, position, clear", "position", true},
    {"disturbance, position, 1.4, 0.4, -0.5, 0.2", "position", false},
    {"disturbance, dropout, 0.75, 0.5", "dropout", false},
    {"disturbance, blackout, 2.5, 1", "blackout", false},
    {"disturbance, dropout, clear", "dropout", true},
    {"disturbance, blackout, clear", "blackout", true},
  };
  for (const auto & form : forms) {
    const auto parsed = parse_disturbance_trigger(form.trigger);
    ASSERT_TRUE(parsed.ok) << form.trigger;
    const auto cmd = to_disturbance_command_msg(parsed.cmd, form.trigger);
    EXPECT_EQ(cmd.trigger, form.trigger);
    EXPECT_EQ(cmd.type, form.type) << form.trigger;
    EXPECT_EQ(cmd.clear, form.clear) << form.trigger;
    EXPECT_EQ(cmd.angle_offset_deg, 0.0);
    EXPECT_EQ(cmd.angle_std_deg, 0.0);
  }
  const auto pos = to_disturbance_command_msg(
    parse_disturbance_trigger("disturbance, position, 1.4, 0.4, -0.5, 0.2").cmd, "t");
  EXPECT_EQ(pos.lon_offset_m, 1.4);
  EXPECT_EQ(pos.lon_std_m, 0.4);
  EXPECT_EQ(pos.lat_offset_m, -0.5);
  EXPECT_EQ(pos.lat_std_m, 0.2);
  const auto drop = to_disturbance_command_msg(
    parse_disturbance_trigger("disturbance, dropout, 0.75, 0.5").cmd, "t");
  EXPECT_EQ(drop.keep_prob, 0.75);
  EXPECT_EQ(drop.mean_gap_s, 0.5);
  const auto black = to_disturbance_command_msg(
    parse_disturbance_trigger("disturbance, blackout, 2.5, 1").cmd, "t");
  EXPECT_EQ(black.duration_s, 2.5);
  EXPECT_TRUE(black.recover);
  // 읽지 못한 문장(거절)에는 기본값 명령이 간다
  const auto rejected = to_disturbance_command_msg(ParsedTrigger{}, "bad text");
  EXPECT_EQ(rejected.trigger, "bad text");
  EXPECT_EQ(rejected.type, "");
  EXPECT_FALSE(rejected.clear);
  EXPECT_EQ(rejected.keep_prob, 0.0);
  EXPECT_FALSE(rejected.recover);
}

}  // namespace autoware::morai_interface
