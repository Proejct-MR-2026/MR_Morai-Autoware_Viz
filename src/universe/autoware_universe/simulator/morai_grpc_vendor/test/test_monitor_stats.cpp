// 신규 — 26.09.25 _ YSH _ morai_probe monitor 집계부 gtest (합성 업데이트, 네트워크 없음)
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

#include "monitor_stats.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <vector>

namespace
{

using morai_grpc_vendor::probe::looks_oscillating;
using morai_grpc_vendor::probe::MonitorEvent;
using morai_grpc_vendor::probe::MonitorSample;
using morai_grpc_vendor::probe::MonitorStats;
using morai_grpc_vendor::probe::MonitorThresholds;
using morai_grpc_vendor::probe::MonitorWindow;

constexpr int64_t kT0Us = 1790000000000000;

/// i 번째 업데이트: 벽시계·MORAI 모두 20 ms 간격 (실측 ~50 upd/s)
MonitorSample sample(int i, double throttle = 0.3, double brake = 0.0, double steer = 0.1)
{
  MonitorSample s;
  s.t_s = 0.02 * i;
  s.timestamp_us = kT0Us + 20000LL * i;
  s.recv_minus_ts_ms = 1.0;
  s.speed_mps = 5.0;
  s.throttle = throttle;
  s.brake = brake;
  s.steer = steer;
  s.front_wheel_angle = -steer;  // Unity 부호 — 집계는 |값| 만 본다
  s.gear = 4;
  return s;
}

TEST(MonitorStats, SmoothDrivingRaisesNothing)
{
  MonitorStats m;
  for (int i = 0; i < 500; ++i) {
    m.add_poll(true);
    m.add_update(sample(i));
  }
  const MonitorWindow w = m.take_window(10.0);
  EXPECT_EQ(w.updates, 500u);
  EXPECT_NEAR(w.updates_per_s, 50.0, 1e-9);
  EXPECT_EQ(w.polls, 500u);
  EXPECT_EQ(w.throttle_toggles, 0u);
  EXPECT_EQ(w.brake_engagements, 0u);
  EXPECT_EQ(w.brake_on_updates, 0u);
  EXPECT_EQ(w.idle_records, 0u);
  EXPECT_EQ(w.steer_zero_drops, 0u);
  EXPECT_EQ(w.fwa_zero_drops, 0u);
  EXPECT_EQ(w.ts_gaps, 0u);
  EXPECT_EQ(w.wall_gaps, 0u);
  EXPECT_NEAR(w.max_ts_gap_ms, 20.0, 1e-9);
  EXPECT_EQ(w.gear_changes, 0u);
  EXPECT_EQ(w.last_gear, 4);
  EXPECT_DOUBLE_EQ(w.speed_min_mps, 5.0);
  EXPECT_NEAR(w.recv_minus_ts_ms.p50, 1.0, 1e-12);
  EXPECT_FALSE(looks_oscillating(w, m.thresholds()));
  EXPECT_TRUE(m.events().empty());
}

TEST(MonitorStats, FloodOscillationShowsAsTogglesAndBrake)
{
  // 실측 진동(비동기 200 Hz @ targetFPS 60): throttle 0<->1, 그 사이 브레이크 개입
  MonitorStats m;
  for (int i = 0; i < 100; ++i) {
    const bool on = i % 2 == 0;
    m.add_update(sample(i, on ? 1.0 : 0.0, on ? 0.0 : 0.6));
  }
  const MonitorWindow w = m.take_window(2.0);
  EXPECT_EQ(w.throttle_toggles, 99u);
  EXPECT_EQ(w.brake_engagements, 50u);
  EXPECT_EQ(w.brake_on_updates, 50u);
  EXPECT_NEAR(w.brake_on_share(), 0.5, 1e-12);
  EXPECT_TRUE(looks_oscillating(w, m.thresholds()));

  // 정상 가감속 전환 (2 s 에 한 번)은 진동이 아니다
  MonitorStats n;
  for (int i = 0; i < 500; ++i) {
    const bool accel = (i / 100) % 2 == 0;
    n.add_update(sample(i, accel ? 0.3 : 0.0, accel ? 0.0 : 0.2));
  }
  const MonitorWindow v = n.take_window(10.0);
  EXPECT_EQ(v.throttle_toggles, 4u);
  EXPECT_FALSE(looks_oscillating(v, n.thresholds()));
}

TEST(MonitorStats, ThrottleHysteresisIgnoresCruiseDither)
{
  // 등속 주행(가속 명령 ~0): MORAI 가속 제어기가 0 근처를 오간다 — 전환이 아니다
  MonitorStats m;
  for (int i = 0; i < 500; ++i) {
    m.add_update(sample(i, i % 2 == 0 ? 0.015 : 0.0, 0.0));
  }
  const MonitorWindow w = m.take_window(10.0);
  EXPECT_EQ(w.throttle_toggles, 0u);
  EXPECT_FALSE(looks_oscillating(w, m.thresholds()));

  // 밟은 뒤 두 임계 사이(0.001 < x <= 0.02)로 내려와도 뗀 것이 아니다. 0 까지 가야 뗌
  MonitorStats n;
  int i = 0;
  for (const double thr : {0.3, 0.01, 0.3, 0.01, 0.0, 0.01, 0.3}) {
    n.add_update(sample(i++, thr, 0.0));
  }
  const MonitorWindow v = n.take_window(1.0);
  EXPECT_EQ(v.throttle_toggles, 2u);  // 0.01 -> 0.0 (뗌), 0.01 -> 0.3 (밟음)
}

TEST(MonitorStats, ClockBoundIsNegatedMinimumAge)
{
  MonitorStats m;
  const double ages[] = {12.0, 3.5, 7.25, 20.0};
  int i = 0;
  for (const double a : ages) {
    MonitorSample s = sample(i++);
    s.recv_minus_ts_ms = a;
    m.add_update(s);
  }
  const MonitorWindow w = m.take_window(1.0);
  EXPECT_DOUBLE_EQ(w.recv_minus_ts_ms.min, 3.5);
  EXPECT_DOUBLE_EQ(w.clock_bound_ms, -3.5);  // MORAI - 로컬 >= -3.5 ms
  EXPECT_TRUE(std::isnan(m.take_window(2.0).clock_bound_ms));  // 업데이트 없는 창
}

TEST(MonitorStats, IdleRecordIsNotBraking)
{
  MonitorStats m;
  m.add_update(sample(0, 0.0, 0.003, 0.0));  // MORAI 유휴 레코드
  m.add_update(sample(1, 0.0, 0.0035, 0.0));
  m.add_update(sample(2, 0.0, 0.0, 0.0));    // 페달 0 명령 — 유휴 아님 (brake 0)
  m.add_update(sample(3, 0.2, 0.003, 0.0));  // throttle 있음 — 유휴 아님
  const MonitorWindow w = m.take_window(0.1);
  EXPECT_EQ(w.idle_records, 2u);
  EXPECT_EQ(w.brake_on_updates, 0u);
  EXPECT_EQ(w.brake_engagements, 0u);
}

TEST(MonitorStats, SteerZeroDropsNeedActiveSteeringOnBothSides)
{
  MonitorStats m;
  int i = 0;
  auto add = [&](double steer) { m.add_update(sample(i++, 0.3, 0.0, steer)); };
  // 1) 한 업데이트 0 → 떨어짐
  add(0.2);
  add(0.0);
  add(0.2);
  // 2) 두 업데이트 0, 부호가 바뀌어도 → 떨어짐 (길이 2)
  add(0.0);
  add(0.0);
  add(-0.2);
  // 3) 6 업데이트 0 (> max_drop_run 5) → 직진 구간으로 보고 세지 않는다
  for (int k = 0; k < 6; ++k) {
    add(0.0);
  }
  add(0.2);
  // 4) 앞이 조향 중이 아니면 (|steer| <= 1e-3) 세지 않는다
  add(0.0005);
  add(0.0);
  add(0.2);
  // 5) 뒤가 조향 중이 아니면 세지 않는다
  add(0.0);
  add(0.0005);
  const MonitorWindow w = m.take_window(1.0);
  EXPECT_EQ(w.steer_zero_drops, 2u);
  EXPECT_EQ(w.fwa_zero_drops, 2u);  // front_wheel_angle = -steer 라 같은 패턴
  EXPECT_EQ(w.longest_drop_run, 2u);
}

TEST(MonitorStats, DropSpanningWindowsCountsWhereItEnds)
{
  MonitorStats m;
  m.add_update(sample(0, 0.3, 0.0, 0.2));
  m.add_update(sample(1, 0.3, 0.0, 0.0));
  const MonitorWindow first = m.take_window(0.03);
  EXPECT_EQ(first.steer_zero_drops, 0u);
  m.add_update(sample(2, 0.3, 0.0, 0.2));
  const MonitorWindow second = m.take_window(0.05);
  EXPECT_EQ(second.steer_zero_drops, 1u);
  EXPECT_EQ(m.total(0.05).steer_zero_drops, 1u);
  EXPECT_EQ(m.total(0.05).updates, 3u);
  EXPECT_EQ(second.updates, 1u);
}

TEST(MonitorStats, GapsAndBackwardsTimestamps)
{
  MonitorStats m;
  MonitorSample s = sample(0);
  m.add_update(s);
  s = sample(1);
  m.add_update(s);
  // MORAI 정체 뒤 따라잡기: timestamp +60 ms, 벽시계 +50 ms
  s.timestamp_us += 60000;
  s.t_s += 0.05;
  m.add_update(s);
  // 시계 역행 (MORAI 재시작) — gap 이 아니라 사건
  MonitorSample back = s;
  back.timestamp_us -= 1000000;
  back.t_s += 0.02;
  m.add_update(back);
  const MonitorWindow w = m.take_window(1.0);
  EXPECT_EQ(w.ts_gaps, 1u);
  EXPECT_NEAR(w.max_ts_gap_ms, 60.0, 1e-9);
  EXPECT_EQ(w.wall_gaps, 1u);
  EXPECT_NEAR(w.max_wall_gap_ms, 50.0, 1e-6);
  EXPECT_EQ(w.ts_backwards, 1u);
  ASSERT_EQ(m.events().size(), 1u);
  EXPECT_EQ(m.events()[0].kind, MonitorEvent::Kind::kTimestampBackwards);
}

TEST(MonitorStats, ControlModeFlipsAreEvents)
{
  MonitorStats m;
  m.add_mode(0.0, true, 3);
  m.add_mode(0.5, true, 3);
  m.add_mode(1.0, true, 1);    // AUTO -> KEYBOARD
  m.add_mode(1.5, false, 0);   // 조회 실패는 변화가 아니다
  m.add_mode(2.0, true, 1);
  const MonitorWindow w = m.take_window(2.0);
  EXPECT_EQ(w.mode_polls, 5u);
  EXPECT_EQ(w.mode_poll_failures, 1u);
  EXPECT_EQ(w.mode_changes, 1u);
  EXPECT_EQ(w.last_mode, 1);
  const std::vector<MonitorEvent> ev = m.drain_new_events();
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_EQ(ev[0].kind, MonitorEvent::Kind::kModeChange);
  EXPECT_EQ(ev[0].from, 3);
  EXPECT_EQ(ev[0].to, 1);
  EXPECT_DOUBLE_EQ(ev[0].t_s, 1.0);
  EXPECT_TRUE(m.drain_new_events().empty());  // 이미 꺼낸 사건은 다시 나오지 않는다
  // 다음 창은 변화가 없어도 현재 모드를 이어 받는다
  const MonitorWindow next = m.take_window(4.0);
  EXPECT_EQ(next.last_mode, 1);
  EXPECT_EQ(next.mode_changes, 0u);
}

TEST(MonitorStats, GearChangesAndCollisions)
{
  MonitorStats m;
  const size_t collisions[] = {0, 2, 2, 0, 1};
  for (int i = 0; i < 5; ++i) {
    MonitorSample s = sample(i);
    s.gear = i < 3 ? 4 : 1;  // D -> P
    s.collisions = collisions[i];
    m.add_update(s);
  }
  const MonitorWindow w = m.take_window(0.1);
  EXPECT_EQ(w.gear_changes, 1u);
  EXPECT_EQ(w.last_gear, 1);
  EXPECT_EQ(w.collision_updates, 3u);
  EXPECT_EQ(w.collision_events, 2u);
  size_t gear_events = 0;
  size_t coll_events = 0;
  for (const auto & e : m.events()) {
    gear_events += e.kind == MonitorEvent::Kind::kGearChange ? 1 : 0;
    coll_events += e.kind == MonitorEvent::Kind::kCollision ? 1 : 0;
  }
  EXPECT_EQ(gear_events, 1u);
  EXPECT_EQ(coll_events, 2u);
}

TEST(MonitorStats, MissingEgoDoesNotFakeToggles)
{
  MonitorStats m;
  m.add_update(sample(0, 0.3));
  MonitorSample lost = sample(1, 0.0, 0.0, 0.0);
  lost.ego_found = false;
  m.add_update(lost);
  m.add_update(sample(2, 0.3));  // Ego 가 돌아왔다 — 사이의 0 은 차량 값이 아니다
  MonitorSample invalid = sample(3, 0.3);
  invalid.ego_valid = false;
  m.add_update(invalid);
  const MonitorWindow w = m.take_window(0.1);
  EXPECT_EQ(w.updates, 4u);
  EXPECT_EQ(w.ego_missing, 1u);
  EXPECT_EQ(w.ego_invalid, 1u);
  EXPECT_EQ(w.throttle_toggles, 0u);
  EXPECT_EQ(w.steer_zero_drops, 0u);
  EXPECT_EQ(w.ts_gaps, 0u);
  ASSERT_EQ(m.events().size(), 2u);
  EXPECT_EQ(m.events()[0].kind, MonitorEvent::Kind::kEgoLost);
  EXPECT_EQ(m.events()[1].kind, MonitorEvent::Kind::kEgoBack);
}

TEST(MonitorStats, EmptyWindowIsNaNNotZero)
{
  MonitorStats m;
  m.add_poll(false);
  const MonitorWindow w = m.take_window(10.0);
  EXPECT_EQ(w.updates, 0u);
  EXPECT_EQ(w.rpc_failures, 1u);
  EXPECT_NEAR(w.updates_per_s, 0.0, 1e-12);
  EXPECT_TRUE(std::isnan(w.brake_on_share()));
  EXPECT_TRUE(std::isnan(w.speed_min_mps));
  EXPECT_EQ(w.recv_minus_ts_ms.n, 0u);
  EXPECT_TRUE(std::isnan(w.clock_bound_ms));
  EXPECT_FALSE(looks_oscillating(w, MonitorThresholds{}));
  EXPECT_FALSE(to_json(w).dump().empty());
}

}  // namespace
