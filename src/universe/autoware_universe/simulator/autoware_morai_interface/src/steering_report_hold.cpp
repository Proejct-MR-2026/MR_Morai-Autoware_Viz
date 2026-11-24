// 신규 — 26.09.25 _ YSH _ SteeringReportHold 구현 (main.py 와 한 줄씩 같은 규칙)
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

#include "autoware/morai_interface/steering_report_hold.hpp"

#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::morai_interface
{

double SteeringReportHold::apply(
  double front_wheel_angle, double last_steer_cmd, bool enabled, bool moving)
{
  double fwa = front_wheel_angle;
  if (enabled && fwa == 0.0 && std::abs(last_steer_cmd) > 1e-3) {
    fwa = last_good_;
    if (moving) {
      ++hold_count_;
    }
  } else {
    last_good_ = fwa;
  }
  report_count_ += moving ? 1 : 0;
  if (report_count_ >= kLogEvery) {
    if (hold_count_ > 0) {
      char buf[160];
      std::snprintf(
        buf, sizeof(buf), "조향 리포트 0 드롭아웃 %d/%d (%.0f%%) — 직전값 유지 (주행 중 기준)",
        hold_count_, report_count_,
        100.0 * static_cast<double>(hold_count_) / static_cast<double>(report_count_));
      pending_log_ = std::string(buf);
    }
    report_count_ = 0;
    hold_count_ = 0;
  }
  return fwa;
}

std::optional<std::string> SteeringReportHold::take_log()
{
  auto out = pending_log_;
  pending_log_.reset();
  return out;
}

// ============================================================================
// 26.10.01 _ YSH _ 신규 — 외부 조향 필터 (steering_report_hold.hpp 머리말)
// ============================================================================

void SentSteerHistory::add(int64_t system_ns, double steer_rad)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  ring_[head_] = Entry{system_ns, steer_rad};
  head_ = (head_ + 1) % kSize;
  if (count_ < kSize) {
    ++count_;
  }
}

std::optional<bool> SentSteerHistory::matches(double applied_rad, int64_t rx_system_ns) const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  bool any = false;
  for (size_t i = 0; i < count_; ++i) {
    const Entry & e = ring_[i];
    if (e.system_ns > rx_system_ns || e.system_ns < rx_system_ns - kWindowNs) {
      continue;
    }
    any = true;
    if (std::abs(e.steer_rad - applied_rad) <= kExactRad) {
      return true;
    }
  }
  if (!any) {
    return std::nullopt;
  }
  return false;
}

double ForeignSteerFilter::apply(
  double front_wheel_angle, std::optional<bool> ours, bool enabled, bool moving)
{
  const double fwa = front_wheel_angle;
  // 껐다 켜면 자기 차단·통계를 처음부터 (파라미터를 다시 켜는 것이 재시도 방법이다)
  if (enabled != was_enabled_) {
    was_enabled_ = enabled;
    tripped_ = false;
    streak_ = 0;
    streak_logged_ = false;
    judged_count_ = own_count_ = held_count_ = 0;
  }
  // 우리 값은 꺼져 있어도 기억해 둔다 — 주행 중에 켜면 그 프레임부터 유지할 값이 있다
  if (ours.has_value() && *ours) {
    last_own_ = fwa;
    has_own_ = true;
  }
  if (!enabled) {
    return fwa;
  }
  if (!ours.has_value()) {
    streak_ = 0;          // 최근에 보낸 것이 없다 (수동 주행·연결 직후) — 판정하지 않고 통과
    streak_logged_ = false;
    return fwa;
  }

  double out = fwa;
  bool held = false;
  if (*ours) {
    streak_ = 0;
    streak_logged_ = false;
  } else {
    ++streak_;
    if (!tripped_ && has_own_) {
      if (streak_ <= kMaxHold) {
        out = last_own_;
        held = true;
      } else if (!streak_logged_) {
        streak_logged_ = true;
        char buf[512];  // 한글은 글자당 3 바이트
        std::snprintf(
          buf, sizeof(buf),
          "외부 조향이 %d 프레임 넘게 이어진다 — 리포트를 다시 MORAI 값으로 낸다 (우리 값이 돌아오면 필터 재개)",
          kMaxHold);
        pending_log_ = std::string(buf);
      }
    }
  }

  if (moving) {
    ++judged_count_;
    own_count_ += *ours ? 1 : 0;
    held_count_ += held ? 1 : 0;
    if (judged_count_ >= kLogEvery) {
      const double own_ratio = static_cast<double>(own_count_) / static_cast<double>(judged_count_);
      char buf[512];
      if (!tripped_ && own_ratio < kTripOwnRatio) {
        tripped_ = true;
        std::snprintf(
          buf, sizeof(buf),
          "외부 조향 필터를 스스로 끈다 — 우리 값과 일치한 프레임이 %d/%d (%.0f%%) 뿐이다. "
          "MORAI 조향 정규화 기준(steer_input_norm_deg)이 이 차량과 안 맞는 것 같다 "
          "(steer_report_ignore_foreign 을 껐다 켜면 다시 시도)",
          own_count_, judged_count_, 100.0 * own_ratio);
        pending_log_ = std::string(buf);
      } else if (held_count_ > 0) {
        std::snprintf(
          buf, sizeof(buf), "외부 조향 %d/%d (%.0f%%) — 직전 우리 값 유지 (주행 중 기준)", held_count_,
          judged_count_,
          100.0 * static_cast<double>(held_count_) / static_cast<double>(judged_count_));
        pending_log_ = std::string(buf);
      }
      judged_count_ = own_count_ = held_count_ = 0;
    }
  }
  return out;
}

std::optional<std::string> ForeignSteerFilter::take_log()
{
  auto out = pending_log_;
  pending_log_.reset();
  return out;
}

}  // namespace autoware::morai_interface
