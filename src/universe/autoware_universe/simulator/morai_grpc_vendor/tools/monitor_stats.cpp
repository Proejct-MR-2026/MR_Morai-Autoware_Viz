// 신규 — 26.09.25 _ YSH _ monitor_stats.hpp 구현 (업데이트별 진동·0 떨어짐·모드 변화 집계)
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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::probe
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// 한 업데이트가 창·전체에 똑같이 더할 값
struct Delta
{
  bool ego_missing{false};
  bool ego_invalid{false};
  bool ts_backwards{false};
  bool ts_gap{false};
  double ts_gap_ms{kNaN};
  bool wall_gap{false};
  double wall_gap_ms{kNaN};
  bool vs{false};  ///< vehicle_state 기반 항목을 셌다 (Ego 있음)
  bool toggle{false};
  bool engagement{false};
  bool braking{false};
  bool idle{false};
  size_t steer_drop{0};
  size_t fwa_drop{0};
  bool gear_change{false};
  bool collision{false};
  bool collision_event{false};
};
}  // namespace

std::string to_string(MonitorEvent::Kind k)
{
  switch (k) {
    case MonitorEvent::Kind::kModeChange:
      return "control_mode_change";
    case MonitorEvent::Kind::kGearChange:
      return "gear_change";
    case MonitorEvent::Kind::kCollision:
      return "collision";
    case MonitorEvent::Kind::kTimestampBackwards:
      return "timestamp_backwards";
    case MonitorEvent::Kind::kEgoLost:
      return "ego_lost";
    case MonitorEvent::Kind::kEgoBack:
      return "ego_back";
  }
  return "unknown";
}

double MonitorWindow::brake_on_share() const
{
  const size_t n = updates >= ego_missing ? updates - ego_missing : 0;
  return n > 0 ? static_cast<double>(brake_on_updates) / static_cast<double>(n) : kNaN;
}

bool looks_oscillating(const MonitorWindow & w, const MonitorThresholds & th)
{
  const double span = w.t_end_s - w.t_begin_s;
  return span > 0.0 &&
         static_cast<double>(w.throttle_toggles) / span >= th.oscillation_toggles_per_s;
}

Json to_json(const MonitorWindow & w)
{
  return Json::object()
    .set("t_begin_s", w.t_begin_s)
    .set("t_end_s", w.t_end_s)
    .set("updates", w.updates)
    .set("updates_per_s", w.updates_per_s)
    .set("polls", w.polls)
    .set("rpc_failures", w.rpc_failures)
    .set("ts_gaps", w.ts_gaps)
    .set("wall_gaps", w.wall_gaps)
    .set("max_ts_gap_ms", w.max_ts_gap_ms)
    .set("max_wall_gap_ms", w.max_wall_gap_ms)
    .set("ts_backwards", w.ts_backwards)
    .set("throttle_toggles", w.throttle_toggles)
    .set("brake_engagements", w.brake_engagements)
    .set("brake_on_updates", w.brake_on_updates)
    .set("brake_on_share", w.brake_on_share())
    .set("idle_records", w.idle_records)
    .set("steer_zero_drops", w.steer_zero_drops)
    .set("fwa_zero_drops", w.fwa_zero_drops)
    .set("longest_drop_run", w.longest_drop_run)
    .set("mode_polls", w.mode_polls)
    .set("mode_poll_failures", w.mode_poll_failures)
    .set("mode_changes", w.mode_changes)
    .set("last_mode", w.last_mode)
    .set("gear_changes", w.gear_changes)
    .set("last_gear", w.last_gear)
    .set("collision_updates", w.collision_updates)
    .set("collision_events", w.collision_events)
    .set("ego_missing", w.ego_missing)
    .set("ego_invalid", w.ego_invalid)
    .set("speed_min_mps", w.speed_min_mps)
    .set("speed_max_mps", w.speed_max_mps)
    .set("speed_last_mps", w.speed_last_mps)
    .set("recv_minus_ts_ms", to_json(w.recv_minus_ts_ms))
    .set("wall_gap_ms", to_json(w.wall_gap_ms))
    .set("clock_offset_lower_bound_ms", w.clock_bound_ms);
}

Json to_json(const MonitorEvent & e)
{
  return Json::object()
    .set("t_s", e.t_s)
    .set("kind", to_string(e.kind))
    .set("from", e.from)
    .set("to", e.to);
}

MonitorStats::MonitorStats(MonitorThresholds th, double t_start_s) : th_(th)
{
  win_.w.t_begin_s = t_start_s;
  all_.w.t_begin_s = t_start_s;
}

size_t MonitorStats::step_drop(ZeroDrop * z, double v) const
{
  const double a = std::fabs(v);
  if (a <= th_.steer_zero) {
    if (z->armed) {
      ++z->run;  // 조향 중에 시작한 0 구간만 잰다
    }
    return 0;
  }
  size_t ended = 0;
  if (z->run > 0 && z->armed && z->run <= th_.max_drop_run && a > th_.steer_active) {
    ended = z->run;
  }
  z->run = 0;
  z->armed = a > th_.steer_active;
  return ended;
}

void MonitorStats::event(double t, MonitorEvent::Kind k, int64_t from, int64_t to)
{
  events_.push_back(MonitorEvent{t, k, from, to});
}

void MonitorStats::add_update(const MonitorSample & s)
{
  Delta d;

  // --- 시간 간격 (Ego 유무와 무관) ---
  if (have_prev_) {
    const double dts_ms = static_cast<double>(s.timestamp_us - prev_.timestamp_us) / 1e3;
    if (dts_ms < 0.0) {
      d.ts_backwards = true;  // MORAI 재시작·시나리오 리셋
      event(s.t_s, MonitorEvent::Kind::kTimestampBackwards, prev_.timestamp_us, s.timestamp_us);
    } else {
      d.ts_gap_ms = dts_ms;
      d.ts_gap = dts_ms > th_.gap_ms;
    }
    d.wall_gap_ms = (s.t_s - prev_.t_s) * 1e3;
    d.wall_gap = d.wall_gap_ms > th_.gap_ms;
  }

  // --- Ego 상태 ---
  if (!s.ego_found) {
    d.ego_missing = true;
    if (!ego_lost_) {
      ego_lost_ = true;
      event(s.t_s, MonitorEvent::Kind::kEgoLost, 0, 0);
    }
  } else {
    if (ego_lost_) {
      ego_lost_ = false;
      event(s.t_s, MonitorEvent::Kind::kEgoBack, 0, 0);
    }
    d.vs = true;
    d.ego_invalid = !s.ego_valid;
    const bool had_vs = have_prev_ && prev_.ego_found;

    // 히스테리시스: 두 임계 사이 값은 직전 상태를 유지한다
    bool pressed = prev_pressed_;
    if (s.throttle > th_.throttle_on) {
      pressed = true;
    } else if (s.throttle <= th_.throttle_off) {
      pressed = false;
    }
    d.toggle = had_vs && pressed != prev_pressed_;
    prev_pressed_ = pressed;

    d.braking = s.brake > th_.brake_on;
    d.engagement = had_vs && d.braking && !prev_braking_;
    prev_braking_ = d.braking;

    d.idle = std::fabs(s.throttle) <= th_.idle_throttle_max &&
             std::fabs(s.brake - th_.idle_brake_center) <= th_.idle_brake_tol;

    d.steer_drop = step_drop(&steer_drop_, s.steer);
    d.fwa_drop = step_drop(&fwa_drop_, s.front_wheel_angle);

    if (had_vs && s.gear != prev_.gear) {
      d.gear_change = true;
      event(s.t_s, MonitorEvent::Kind::kGearChange, prev_.gear, s.gear);
    }
    d.collision = s.collisions > 0;
    const bool prev_collision = had_vs && prev_.collisions > 0;
    if (d.collision && !prev_collision) {
      d.collision_event = true;
      event(
        s.t_s, MonitorEvent::Kind::kCollision, static_cast<int64_t>(had_vs ? prev_.collisions : 0),
        static_cast<int64_t>(s.collisions));
    }
  }

  for (Acc * a : {&win_, &all_}) {
    MonitorWindow & w = a->w;
    ++w.updates;
    w.ego_missing += d.ego_missing ? 1 : 0;
    w.ts_backwards += d.ts_backwards ? 1 : 0;
    if (std::isfinite(d.ts_gap_ms)) {
      w.ts_gaps += d.ts_gap ? 1 : 0;
      w.max_ts_gap_ms = std::max(w.max_ts_gap_ms, d.ts_gap_ms);
    }
    if (std::isfinite(d.wall_gap_ms)) {
      w.wall_gaps += d.wall_gap ? 1 : 0;
      w.max_wall_gap_ms = std::max(w.max_wall_gap_ms, d.wall_gap_ms);
      a->gap_ms.push_back(d.wall_gap_ms);
    }
    if (std::isfinite(s.recv_minus_ts_ms)) {
      a->age_ms.push_back(s.recv_minus_ts_ms);
    }
    if (!d.vs) {
      continue;
    }
    w.ego_invalid += d.ego_invalid ? 1 : 0;
    w.throttle_toggles += d.toggle ? 1 : 0;
    w.brake_engagements += d.engagement ? 1 : 0;
    w.brake_on_updates += d.braking ? 1 : 0;
    w.idle_records += d.idle ? 1 : 0;
    w.steer_zero_drops += d.steer_drop > 0 ? 1 : 0;
    w.fwa_zero_drops += d.fwa_drop > 0 ? 1 : 0;
    w.longest_drop_run = std::max({w.longest_drop_run, d.steer_drop, d.fwa_drop});
    w.gear_changes += d.gear_change ? 1 : 0;
    w.last_gear = s.gear;
    w.collision_updates += d.collision ? 1 : 0;
    w.collision_events += d.collision_event ? 1 : 0;
    if (std::isfinite(s.speed_mps)) {
      if (!a->speed_seen) {
        w.speed_min_mps = w.speed_max_mps = s.speed_mps;
        a->speed_seen = true;
      } else {
        w.speed_min_mps = std::min(w.speed_min_mps, s.speed_mps);
        w.speed_max_mps = std::max(w.speed_max_mps, s.speed_mps);
      }
      w.speed_last_mps = s.speed_mps;
    }
  }

  // Ego 가 없는 업데이트는 vehicle_state 비교 기준을 바꾸지 않는다 — prev_ 의 차량 값은 유지
  if (s.ego_found || !have_prev_) {
    prev_ = s;
  } else {
    prev_.t_s = s.t_s;
    prev_.timestamp_us = s.timestamp_us;
    prev_.ego_found = false;
  }
  have_prev_ = true;
}

void MonitorStats::add_poll(bool ok)
{
  for (Acc * a : {&win_, &all_}) {
    (ok ? a->w.polls : a->w.rpc_failures) += 1;
  }
}

void MonitorStats::add_mode(double t_s, bool ok, int mode)
{
  bool changed = false;
  if (ok) {
    changed = have_mode_ && mode != mode_;
    if (changed) {
      event(t_s, MonitorEvent::Kind::kModeChange, mode_, mode);
    }
    mode_ = mode;
    have_mode_ = true;
  }
  for (Acc * a : {&win_, &all_}) {
    ++a->w.mode_polls;
    a->w.mode_poll_failures += ok ? 0 : 1;
    a->w.mode_changes += changed ? 1 : 0;
    if (ok) {
      a->w.last_mode = mode;
    }
  }
}

MonitorWindow MonitorStats::finish(const Acc & a, double now_s)
{
  MonitorWindow w = a.w;
  w.t_end_s = now_s;
  const double span = now_s - w.t_begin_s;
  w.updates_per_s = span > 0.0 ? static_cast<double>(w.updates) / span : kNaN;
  w.recv_minus_ts_ms = summarize(a.age_ms);
  w.wall_gap_ms = summarize(a.gap_ms);
  w.clock_bound_ms = w.recv_minus_ts_ms.n > 0 ? -w.recv_minus_ts_ms.min : kNaN;
  if (!a.speed_seen) {
    w.speed_min_mps = w.speed_max_mps = w.speed_last_mps = kNaN;
  }
  return w;
}

MonitorWindow MonitorStats::take_window(double now_s)
{
  MonitorWindow w = finish(win_, now_s);
  const int mode = win_.w.last_mode;
  const int gear = win_.w.last_gear;
  win_ = Acc{};
  win_.w.t_begin_s = now_s;
  win_.w.last_mode = mode;  // 변화 없는 창도 현재 모드·기어를 보이게 이어 받는다
  win_.w.last_gear = gear;
  return w;
}

MonitorWindow MonitorStats::total(double now_s) const
{
  return finish(all_, now_s);
}

std::vector<MonitorEvent> MonitorStats::drain_new_events()
{
  std::vector<MonitorEvent> out(
    events_.begin() + static_cast<std::ptrdiff_t>(drained_), events_.end());
  drained_ = events_.size();
  return out;
}

}  // namespace morai_grpc_vendor::probe
