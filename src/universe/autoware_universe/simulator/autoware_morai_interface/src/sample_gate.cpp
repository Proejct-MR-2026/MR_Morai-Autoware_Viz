// 신규 — 26.09.25 _ YSH _ SampleGate / RollingWindow 구현
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

#include "autoware/morai_interface/sample_gate.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{

const char * to_string(SampleVerdict v)
{
  switch (v) {
    case SampleVerdict::kNew:
      return "new";
    case SampleVerdict::kRebased:
      return "rebased";
    case SampleVerdict::kDuplicate:
      return "duplicate";
    case SampleVerdict::kBackward:
      return "backward";
    case SampleVerdict::kNoEgo:
      return "no_ego";
    case SampleVerdict::kInvalidEgo:
      return "invalid_ego";
  }
  return "?";
}

const char * to_string(GateMode m)
{
  return m == GateMode::kContent ? "content" : "timestamp";
}

namespace
{
bool same_vec(const morai_grpc_vendor::Vec3 & a, const morai_grpc_vendor::Vec3 & b)
{
  return a.x == b.x && a.y == b.y && a.z == b.z;
}
}  // namespace

bool same_ego_content(const morai_grpc_vendor::EgoSample & a, const morai_grpc_vendor::EgoSample & b)
{
  const auto & va = a.vehicle_state;
  const auto & vb = b.vehicle_state;
  return same_vec(a.position, b.position) && same_vec(a.rotation_deg, b.rotation_deg) &&
         same_vec(a.velocity_kmh, b.velocity_kmh) &&
         same_vec(a.angular_velocity_raw, b.angular_velocity_raw) &&
         same_vec(a.acceleration_raw, b.acceleration_raw) && va.present == vb.present &&
         va.throttle == vb.throttle && va.brake == vb.brake && va.steer == vb.steer &&
         va.front_wheel_angle == vb.front_wheel_angle && va.gear_mode == vb.gear_mode;
}

SampleVerdict SampleGate::check(
  const morai_grpc_vendor::Snapshot & snapshot, int64_t now_steady_ns, bool tick_confirmed)
{
  if (!snapshot.ego.found) {
    return SampleVerdict::kNoEgo;
  }
  if (!snapshot.ego.valid) {
    return SampleVerdict::kInvalidEgo;
  }
  const int64_t ts = snapshot.timestamp_us;
  const SampleVerdict v = mode_ == GateMode::kTimestamp
                            ? check_timestamp(snapshot.ego, ts, now_steady_ns, tick_confirmed)
                            : check_content(snapshot.ego, ts, now_steady_ns, tick_confirmed);
  last_seen_ = snapshot.ego;
  has_seen_ = true;
  return v;
}

void SampleGate::accept_timestamp(int64_t ts)
{
  last_ts_us_ = ts;
  has_last_ = true;
  backward_streak_ = 0;
  frozen_changes_ = 0;
}

SampleVerdict SampleGate::check_timestamp(
  const morai_grpc_vendor::EgoSample & ego, int64_t ts, int64_t now, bool tick_confirmed)
{
  if (ts <= 0) {
    // 실시간 모드의 timestamp 는 epoch us 라 0 일 수 없다 — 채우지 않는 빌드
    enter_content("ActorStates.timestamp 가 0");
    return check_content(ego, ts, now, tick_confirmed);
  }
  if (!has_last_) {
    accept_timestamp(ts);
    last_step_us_.reset();
    return SampleVerdict::kNew;
  }
  if (ts == last_ts_us_) {
    // 같은 프레임이면 내용도 같다. 같은 timestamp 로 내용이 거듭 바뀌면 timestamp 가 멈춘 것이다
    if (has_seen_ && !same_ego_content(ego, last_seen_)) {
      ++frozen_changes_;
    }
    if (frozen_changes_ >= config_.frozen_content_changes) {
      enter_content(
        "ActorStates.timestamp 가 그대로인데 Ego 내용이 " + std::to_string(frozen_changes_) + " 번 바뀌었다");
      return check_content(ego, ts, now, tick_confirmed);
    }
    return SampleVerdict::kDuplicate;
  }
  if (ts < last_ts_us_) {
    if (backward_streak_ > 0 && ts == last_backward_ts_us_) {
      return SampleVerdict::kDuplicate;  // 같은 역행 업데이트를 두 번 읽었다 (폴링)
    }
    last_backward_ts_us_ = ts;
    ++backward_streak_;
    if (last_ts_us_ - ts > config_.rebase_jump_us || backward_streak_ >= config_.rebase_after_backward) {
      accept_timestamp(ts);
      last_step_us_.reset();
      return SampleVerdict::kRebased;
    }
    return SampleVerdict::kBackward;
  }
  last_step_us_ = ts - last_ts_us_;
  accept_timestamp(ts);
  return SampleVerdict::kNew;
}

SampleVerdict SampleGate::check_content(
  const morai_grpc_vendor::EgoSample & ego, int64_t ts, int64_t now, bool tick_confirmed)
{
  const bool changed = !content_has_last_ || !same_ego_content(ego, last_accepted_);
  const bool period_ok = !content_has_last_ || now - last_accept_ns_ >= config_.content_min_period_ns;
  if (!changed && !(tick_confirmed && period_ok)) {
    return SampleVerdict::kDuplicate;
  }
  if (content_has_last_) {
    last_step_us_ = (now - last_accept_ns_) / 1000;
  } else {
    last_step_us_.reset();
  }
  // timestamp 가 되살아나면 (0 으로 시작해 늘기 시작하는 빌드 등) 연속 N 번 늘 때 timestamp 모드로 복귀
  if (ts > 0 && content_has_last_ && ts > content_last_ts_us_) {
    ++timestamp_advancing_;
  } else {
    timestamp_advancing_ = 0;
  }
  content_last_ts_us_ = ts;
  content_has_last_ = true;
  last_accepted_ = ego;
  last_accept_ns_ = now;
  has_last_ = true;
  if (timestamp_advancing_ >= config_.timestamp_recover_after) {
    mode_ = GateMode::kTimestamp;
    last_ts_us_ = ts;
    backward_streak_ = 0;
    frozen_changes_ = 0;
    timestamp_advancing_ = 0;
    mode_change_ = "ActorStates.timestamp 가 다시 늘어난다 — timestamp 판정으로 복귀";
  }
  return SampleVerdict::kNew;
}

void SampleGate::enter_content(const std::string & why)
{
  mode_ = GateMode::kContent;
  content_has_last_ = false;
  timestamp_advancing_ = 0;
  frozen_changes_ = 0;
  backward_streak_ = 0;
  mode_change_ = why;
}

std::optional<std::string> SampleGate::take_mode_change()
{
  auto out = std::move(mode_change_);
  mode_change_.reset();
  return out;
}

RollingWindow::RollingWindow(size_t capacity) : capacity_(std::max<size_t>(capacity, 1))
{
  buffer_.reserve(capacity_);
}

void RollingWindow::add(double value)
{
  if (!std::isfinite(value)) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_.size() < capacity_) {
    buffer_.push_back(value);
    next_ = buffer_.size() % capacity_;
    return;
  }
  buffer_[next_] = value;
  next_ = (next_ + 1) % capacity_;
}

RollingWindow::Summary RollingWindow::summary() const
{
  std::vector<double> copy;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    copy = buffer_;
  }
  Summary s;
  s.count = copy.size();
  if (copy.empty()) {
    return s;
  }
  std::sort(copy.begin(), copy.end());
  s.min = copy.front();
  s.max = copy.back();
  s.p01 = percentile_sorted(copy, 0.01);
  s.p50 = percentile_sorted(copy, 0.50);
  s.p99 = percentile_sorted(copy, 0.99);
  return s;
}

void RollingWindow::clear()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  buffer_.clear();
  next_ = 0;
}

double percentile_sorted(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) {
    return 0.0;
  }
  const double clamped = std::min(std::max(p, 0.0), 1.0);
  const auto idx = static_cast<size_t>(std::lround(clamped * static_cast<double>(sorted.size() - 1)));
  return sorted[std::min(idx, sorted.size() - 1)];
}

}  // namespace autoware::morai_interface
