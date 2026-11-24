// 신규 — 26.09.25 _ YSH _ 표본 판정(timestamp 증가·중복·역행·무효 Ego)·구간 통계·조향 유지 필터·명령 슬롯 시험
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

#include "autoware/morai_interface/command_slot.hpp"
#include "autoware/morai_interface/sample_gate.hpp"
#include "autoware/morai_interface/steering_report_hold.hpp"

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::morai_interface
{
namespace
{

constexpr int64_t kSec = 1000000000LL;
constexpr int64_t kMsForTest = 1000000LL;

morai_grpc_vendor::Snapshot snap(int64_t ts_us, bool found = true, bool valid = true)
{
  morai_grpc_vendor::Snapshot s;
  s.timestamp_us = ts_us;
  s.ego.found = found;
  s.ego.valid = valid;
  return s;
}

TEST(SampleGate, AcceptsIncreasingTimestampsAndSkipsDuplicates)
{
  SampleGate gate;
  EXPECT_EQ(gate.check(snap(1000), 0), SampleVerdict::kNew);
  EXPECT_FALSE(gate.last_step_us().has_value());
  EXPECT_EQ(gate.check(snap(1000), 1), SampleVerdict::kDuplicate);
  EXPECT_EQ(gate.check(snap(21000), 2), SampleVerdict::kNew);
  ASSERT_TRUE(gate.last_step_us().has_value());
  EXPECT_EQ(*gate.last_step_us(), 20000);
}

TEST(SampleGate, InvalidOrMissingEgoIsNeverAccepted)
{
  SampleGate gate;
  EXPECT_EQ(gate.check(snap(1000, false), 0), SampleVerdict::kNoEgo);
  EXPECT_EQ(gate.check(snap(1000, true, false), 0), SampleVerdict::kInvalidEgo);
  EXPECT_FALSE(gate.has_last()) << "an invalid sample must not become the timestamp baseline";
}

// 26.09.25 _ YSH _ 리뷰 반영: 역행은 1 s 가 아니라 서로 다른 역행 표본 3 개(또는 0.5 s 넘는 점프)에 새 기준 —
//   1 s 동안 Autoware 명령이 MORAI 로 가지 않던 문제 (MORAI 는 마지막 명령을 유지한다)
TEST(SampleGate, BackwardIsSkippedThenRebasedAfterThreeDistinctSamples)
{
  SampleGate gate;
  EXPECT_EQ(gate.check(snap(5000000), 0), SampleVerdict::kNew);
  EXPECT_EQ(gate.check(snap(4800000), 10), SampleVerdict::kBackward);
  EXPECT_EQ(gate.check(snap(4800000), 15), SampleVerdict::kDuplicate) << "same backward update polled twice";
  EXPECT_EQ(gate.check(snap(4820000), 20), SampleVerdict::kBackward);
  EXPECT_EQ(gate.check(snap(4840000), 30), SampleVerdict::kRebased);
  EXPECT_EQ(gate.last_timestamp_us(), 4840000);
  EXPECT_FALSE(gate.last_step_us().has_value());
  EXPECT_EQ(gate.check(snap(4860000), 40), SampleVerdict::kNew);
}

TEST(SampleGate, LargeBackwardJumpRebasesAtOnce)
{
  SampleGate gate;
  gate.check(snap(5000000), 0);
  EXPECT_EQ(gate.check(snap(4000000), 10), SampleVerdict::kRebased);  // 1 s 뒤로 (MORAI 재시작)
}

TEST(SampleGate, SingleBackwardBlipDoesNotRebase)
{
  SampleGate gate;
  gate.check(snap(5000), 0);
  EXPECT_EQ(gate.check(snap(4000), 10), SampleVerdict::kBackward);
  EXPECT_EQ(gate.check(snap(6000), 20), SampleVerdict::kNew);  // 역행 연속이 초기화된다
  EXPECT_EQ(gate.check(snap(4000), 2 * kSec), SampleVerdict::kBackward);
}

// 26.09.25 _ YSH _ 리뷰 반영: timestamp 를 채우지 않는(0·멈춘) 빌드에서 첫 표본 뒤 영원히 멈추지 않게
morai_grpc_vendor::Snapshot at(int64_t ts_us, double x)
{
  auto s = snap(ts_us);
  s.ego.position.x = x;
  return s;
}

TEST(SampleGate, ZeroTimestampSwitchesToContentMode)
{
  SampleGate gate;
  EXPECT_EQ(gate.check(at(0, 1.0), 0), SampleVerdict::kNew);
  EXPECT_EQ(gate.mode(), GateMode::kContent);
  ASSERT_TRUE(gate.take_mode_change().has_value());
  EXPECT_FALSE(gate.take_mode_change().has_value()) << "reported once";
  EXPECT_EQ(gate.check(at(0, 1.0), 5 * kMsForTest), SampleVerdict::kDuplicate) << "same content, no tick";
  EXPECT_EQ(gate.check(at(0, 1.1), 10 * kMsForTest), SampleVerdict::kNew) << "content changed";
}

TEST(SampleGate, ContentModeTakesSameContentOnlyOnConfirmedTicksAndNotTooOften)
{
  SampleGate gate;
  gate.check(at(0, 1.0), 0);
  EXPECT_EQ(gate.check(at(0, 1.0), 10 * kMsForTest, true), SampleVerdict::kDuplicate)
    << "< 15 ms since the last accepted sample";
  EXPECT_EQ(gate.check(at(0, 1.0), 20 * kMsForTest, true), SampleVerdict::kNew);
  ASSERT_TRUE(gate.last_step_us().has_value());
  EXPECT_EQ(*gate.last_step_us(), 20000);
  EXPECT_EQ(gate.check(at(0, 1.0), 60 * kMsForTest, false), SampleVerdict::kDuplicate)
    << "polling without a confirmed tick: a paused MORAI is not republished";
}

TEST(SampleGate, FrozenTimestampWithChangingContentSwitchesToContentMode)
{
  SampleGate gate;
  EXPECT_EQ(gate.check(at(5000, 1.0), 0), SampleVerdict::kNew);
  EXPECT_EQ(gate.check(at(5000, 1.1), 1), SampleVerdict::kDuplicate);
  EXPECT_EQ(gate.check(at(5000, 1.2), 2), SampleVerdict::kDuplicate);
  EXPECT_EQ(gate.mode(), GateMode::kTimestamp);
  EXPECT_EQ(gate.check(at(5000, 1.3), 3), SampleVerdict::kNew);
  EXPECT_EQ(gate.mode(), GateMode::kContent);
}

TEST(SampleGate, FrozenTimestampWithStaticContentIsAPauseNotABrokenClock)
{
  SampleGate gate;
  gate.check(at(5000, 1.0), 0);
  for (int i = 1; i < 50; ++i) {
    EXPECT_EQ(gate.check(at(5000, 1.0), i), SampleVerdict::kDuplicate);
  }
  EXPECT_EQ(gate.mode(), GateMode::kTimestamp);
}

TEST(SampleGate, ContentModeReturnsToTimestampsWhenTheyAdvance)
{
  SampleGate gate;
  gate.check(at(0, 1.0), 0);
  ASSERT_EQ(gate.mode(), GateMode::kContent);
  gate.take_mode_change();
  for (int i = 1; i <= 6; ++i) {
    EXPECT_EQ(gate.check(at(1000000 + 20000 * i, 1.0 + 0.1 * i), i * 20 * kMsForTest), SampleVerdict::kNew);
  }
  EXPECT_EQ(gate.mode(), GateMode::kTimestamp);
  EXPECT_TRUE(gate.take_mode_change().has_value());
  EXPECT_EQ(gate.check(at(1120000, 9.0), 200 * kMsForTest), SampleVerdict::kDuplicate)
    << "back on timestamps: same timestamp is a duplicate";
}

TEST(RollingWindow, PercentilesAndWrap)
{
  RollingWindow w(5);
  EXPECT_EQ(w.summary().count, 0U);
  for (int i = 1; i <= 7; ++i) {
    w.add(static_cast<double>(i));  // 3..7 이 남는다
  }
  w.add(std::nan(""));  // 비유한은 버린다
  const auto s = w.summary();
  EXPECT_EQ(s.count, 5U);
  EXPECT_DOUBLE_EQ(s.min, 3.0);
  EXPECT_DOUBLE_EQ(s.max, 7.0);
  EXPECT_DOUBLE_EQ(s.p50, 5.0);
}

// ============================================================================
// 26.09.25 _ YSH _ steer_report_hold (main.py publish_steering_report 와 같은 규칙)
// ============================================================================

TEST(SteeringReportHold, ZeroIsHeldOnlyWhileCommandingSteer)
{
  SteeringReportHold hold;
  EXPECT_DOUBLE_EQ(hold.apply(-0.3, 0.2, true, true), -0.3);
  EXPECT_DOUBLE_EQ(hold.apply(0.0, 0.2, true, true), -0.3) << "idle-record zero must be held";
  EXPECT_EQ(hold.hold_count(), 1);
  EXPECT_DOUBLE_EQ(hold.apply(0.0, 0.0005, true, true), 0.0) << "tiny last command: a real zero";
  EXPECT_DOUBLE_EQ(hold.last_good(), 0.0);
}

TEST(SteeringReportHold, DisabledPassesZeroThrough)
{
  SteeringReportHold hold;
  hold.apply(-0.3, 0.2, false, true);
  EXPECT_DOUBLE_EQ(hold.apply(0.0, 0.2, false, true), 0.0);
}

TEST(SteeringReportHold, LogsDropoutRatioEvery500MovingSamples)
{
  SteeringReportHold hold;
  for (int i = 0; i < 499; ++i) {
    hold.apply(i % 4 == 0 ? 0.0 : -0.1, 0.2, true, true);
    EXPECT_FALSE(hold.take_log().has_value());
  }
  hold.apply(-0.1, 0.2, true, false);  // 정지 표본은 세지 않는다
  EXPECT_FALSE(hold.take_log().has_value());
  hold.apply(0.0, 0.2, true, true);
  const auto log = hold.take_log();
  ASSERT_TRUE(log.has_value());
  EXPECT_NE(log->find("126/500"), std::string::npos) << *log;
  EXPECT_EQ(hold.report_count(), 0);
}

// ============================================================================
// 26.10.01 _ YSH _ 외부 조향 필터 (steer_report_ignore_foreign)
// ============================================================================

TEST(SentSteerHistory, MatchesOnlySendsInsideTheWindowBeforeTheFrame)
{
  SentSteerHistory h;
  const int64_t ms = 1'000'000;
  EXPECT_FALSE(h.matches(0.1, 1000 * ms).has_value()) << "nothing sent yet: cannot judge";
  h.add(1000 * ms, 0.10);
  h.add(1010 * ms, 0.11);
  EXPECT_EQ(h.matches(0.10, 1020 * ms), std::optional<bool>(true));
  EXPECT_EQ(h.matches(0.11 + 1e-7, 1020 * ms), std::optional<bool>(true)) << "within 0.0001 deg";
  EXPECT_EQ(h.matches(0.105, 1020 * ms), std::optional<bool>(false)) << "between two sends is not ours";
  EXPECT_EQ(h.matches(0.11, 1005 * ms), std::optional<bool>(false)) << "a send after the frame does not count";
  EXPECT_EQ(h.matches(0.10, 1300 * ms), std::optional<bool>(true)) << "exactly 300 ms old still counts";
  EXPECT_EQ(h.matches(0.10, 1305 * ms), std::optional<bool>(false)) << "older than 300 ms, newer one present";
  EXPECT_FALSE(h.matches(0.10, 1400 * ms).has_value()) << "no send in the window: cannot judge";
}

TEST(SentSteerHistory, RingKeepsTheNewestEntries)
{
  SentSteerHistory h;
  const int64_t ms = 1'000'000;
  for (int i = 0; i < 200; ++i) {
    h.add(i * 10 * ms, 0.001 * i);
  }
  EXPECT_EQ(h.matches(0.199, 1990 * ms), std::optional<bool>(true));
  EXPECT_EQ(h.matches(0.170, 1990 * ms), std::optional<bool>(true)) << "290 ms old";
  EXPECT_EQ(h.matches(0.100, 1990 * ms), std::optional<bool>(false)) << "990 ms old";
}

TEST(ForeignSteerFilter, DisabledPassesEverythingThrough)
{
  ForeignSteerFilter f;
  EXPECT_DOUBLE_EQ(f.apply(-0.10, true, false, true), -0.10);
  EXPECT_DOUBLE_EQ(f.apply(0.90, false, false, true), 0.90) << "off = today's behaviour";
  EXPECT_DOUBLE_EQ(f.apply(0.90, std::nullopt, false, true), 0.90);
  EXPECT_FALSE(f.take_log().has_value());
}

TEST(ForeignSteerFilter, ForeignFrameIsReplacedByTheLastOwnValue)
{
  ForeignSteerFilter f;
  EXPECT_DOUBLE_EQ(f.apply(-0.10, true, true, true), -0.10);
  EXPECT_DOUBLE_EQ(f.apply(0.92, false, true, true), -0.10) << "foreign frame: hold the last own value";
  EXPECT_DOUBLE_EQ(f.apply(0.93, false, true, true), -0.10);
  EXPECT_DOUBLE_EQ(f.apply(-0.12, true, true, true), -0.12) << "own value again";
  EXPECT_DOUBLE_EQ(f.apply(0.5, false, true, true), -0.12);
  EXPECT_EQ(f.held_count(), 3);
}

TEST(ForeignSteerFilter, UnjudgeableAndFirstFramesPassThrough)
{
  ForeignSteerFilter f;
  EXPECT_DOUBLE_EQ(f.apply(0.5, false, true, true), 0.5) << "no own value seen yet: nothing to hold";
  EXPECT_DOUBLE_EQ(f.apply(-0.1, true, true, true), -0.1);
  EXPECT_DOUBLE_EQ(f.apply(0.7, std::nullopt, true, true), 0.7) << "nothing sent recently: pass";
}

TEST(ForeignSteerFilter, OwnValueSeenWhileDisabledIsUsedRightAfterEnabling)
{
  ForeignSteerFilter f;
  f.apply(-0.20, true, false, true);
  EXPECT_DOUBLE_EQ(f.apply(0.9, false, true, true), -0.20);
}

TEST(ForeignSteerFilter, StopsHoldingAfterMaxHoldFramesAndResumes)
{
  ForeignSteerFilter f;
  f.apply(-0.10, true, true, true);
  for (int i = 0; i < ForeignSteerFilter::kMaxHold; ++i) {
    EXPECT_DOUBLE_EQ(f.apply(0.9, false, true, true), -0.10) << i;
  }
  EXPECT_DOUBLE_EQ(f.apply(0.9, false, true, true), 0.9) << "streak too long: report the MORAI value";
  const auto log = f.take_log();
  ASSERT_TRUE(log.has_value());
  EXPECT_FALSE(f.tripped());
  EXPECT_DOUBLE_EQ(f.apply(0.9, false, true, true), 0.9);
  EXPECT_DOUBLE_EQ(f.apply(-0.11, true, true, true), -0.11);
  EXPECT_DOUBLE_EQ(f.apply(0.9, false, true, true), -0.11) << "own value came back: filter resumes";
}

TEST(ForeignSteerFilter, LogsHeldRatioEvery500MovingSamples)
{
  ForeignSteerFilter f;
  for (int i = 0; i < 499; ++i) {
    f.apply(i % 5 == 1 ? 0.9 : -0.1, i % 5 != 1, true, true);
  }
  f.apply(0.9, false, true, false);  // 정지 표본은 세지 않는다
  EXPECT_FALSE(f.take_log().has_value());
  f.apply(-0.1, true, true, true);
  const auto log = f.take_log();
  ASSERT_TRUE(log.has_value());
  EXPECT_NE(log->find("100/500"), std::string::npos) << *log;
  EXPECT_FALSE(f.tripped());
  EXPECT_EQ(f.judged_count(), 0);
}

TEST(ForeignSteerFilter, TripsItselfWhenAlmostNothingMatchesAndRetriesAfterToggle)
{
  ForeignSteerFilter f;
  f.apply(-0.1, true, true, true);
  // 기준각이 안 맞으면 거의 모든 프레임이 "외부" 로 보인다 (직진 0 만 우연히 맞는다)
  for (int i = 0; i < 499; ++i) {
    f.apply(i % 20 == 0 ? -0.1 : 0.3, i % 20 == 0, true, true);
  }
  EXPECT_TRUE(f.tripped());
  ASSERT_TRUE(f.take_log().has_value());
  EXPECT_DOUBLE_EQ(f.apply(0.3, false, true, true), 0.3) << "tripped: pass through";
  f.apply(0.3, false, false, true);  // 끈다
  EXPECT_FALSE(f.tripped());
  f.apply(-0.1, true, true, true);   // 다시 켠다
  EXPECT_DOUBLE_EQ(f.apply(0.3, false, true, true), -0.1);
}

TEST(CommandSlot, KeepsOnlyTheLatestWithIncreasingSeq)
{
  CommandSlot slot;
  EXPECT_TRUE(slot.empty());
  autoware_control_msgs::msg::Control a;
  a.lateral.steering_tire_angle = 0.1F;
  slot.submit(a, 100);
  autoware_control_msgs::msg::Control b;
  b.lateral.steering_tire_angle = 0.2F;
  slot.submit(b, 200);
  const auto e = slot.latest();
  ASSERT_TRUE(e.has_value());
  EXPECT_FLOAT_EQ(e->msg.lateral.steering_tire_angle, 0.2F);
  EXPECT_EQ(e->receive_steady_ns, 200);
  EXPECT_EQ(e->seq, 2U);
  slot.clear();
  EXPECT_FALSE(slot.latest().has_value());
}

}  // namespace
}  // namespace autoware::morai_interface
