// 신규 — 26.10.13 _ YSH _ VILS 차량 PC 고스트(ego_mode localize): kinematic_state twist 를 실차 속도·위치 헤딩 변화율로
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

#include "autoware/morai_interface/localize_twist.hpp"

#include <cmath>

namespace autoware::morai_interface
{
namespace
{
// 이만큼 표본이 비면(재연결 등) 창을 새로 채운다 — 긴 공백을 가로지르는 평균 변화율을 내지 않게
constexpr int64_t kGapResetNs = 1'000'000'000;
// 연속 표본 사이 위치가 이만큼 튀면(차량 PC 가 고스트를 옮김) 창을 새로 채운다. 25 ms 표본이면 200 m/s
constexpr double kJumpResetM = 5.0;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

const char * to_string(LocalizeSpeedSource source)
{
  switch (source) {
    case LocalizeSpeedSource::kVehicle:
      return "vehicle";
    case LocalizeSpeedSource::kPose:
      return "pose";
    default:
      return "none";
  }
}

LocalizeTwist::LocalizeTwist(const LocalizeTwistConfig & config) : config_(config)
{
  if (!(config_.window_s > 0.0)) {
    config_.window_s = 0.2;
  }
  if (!(config_.velocity_max_age_s > 0.0)) {
    config_.velocity_max_age_s = 0.2;
  }
}

void LocalizeTwist::set_vehicle_speed(double speed_mps, int64_t now_ns)
{
  if (!std::isfinite(speed_mps)) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  vehicle_speed_ = speed_mps;
  vehicle_ns_ = now_ns;
}

void LocalizeTwist::apply(nav_msgs::msg::Odometry & odom, int64_t sample_ns, int64_t now_ns)
{
  const auto & p = odom.pose.pose.position;
  const PoseSample cur{sample_ns, p.x, p.y, yaw_of(odom.pose.pose.orientation)};

  if (!history_.empty()) {
    const auto & last = history_.back();
    const bool backwards = cur.ns <= last.ns;
    const bool gap = cur.ns - last.ns > kGapResetNs;
    const bool jump = std::hypot(cur.x - last.x, cur.y - last.y) > kJumpResetM;
    if (backwards || gap || jump) {
      history_.clear();
    }
  }
  history_.push_back(cur);
  // 맨 앞 = 창 길이 이상 지난 표본 중 가장 새 것 (그래야 앞뒤 간격이 창 이상)
  const auto window_ns = static_cast<int64_t>(config_.window_s * 1e9);
  while (history_.size() > 2 && cur.ns - history_[1].ns >= window_ns) {
    history_.pop_front();
  }

  std::optional<double> yaw_rate;
  std::optional<double> pose_speed;
  if (history_.size() >= 2) {
    const auto & a = history_.front();
    const double dt = static_cast<double>(cur.ns - a.ns) * 1e-9;
    if (dt >= 0.5 * config_.window_s) {
      const double dyaw = std::remainder(cur.yaw - a.yaw, 2.0 * M_PI);
      yaw_rate = dyaw / dt;
      // 진행 방향으로 투영 — 후진이면 음수 (VelocityReport 와 같은 부호)
      const double mid_yaw = a.yaw + 0.5 * dyaw;
      pose_speed = ((cur.x - a.x) * std::cos(mid_yaw) + (cur.y - a.y) * std::sin(mid_yaw)) / dt;
    }
  }

  const std::lock_guard<std::mutex> lock(mutex_);
  const auto max_age_ns = static_cast<int64_t>(config_.velocity_max_age_s * 1e9);
  const bool vehicle_fresh = vehicle_speed_ && now_ns - vehicle_ns_ <= max_age_ns;
  double speed = 0.0;
  if (vehicle_fresh) {
    speed = *vehicle_speed_;
    stats_.last_source = LocalizeSpeedSource::kVehicle;
    ++stats_.vehicle_used;
  } else if (pose_speed) {
    speed = *pose_speed;
    stats_.last_source = LocalizeSpeedSource::kPose;
    ++stats_.pose_fallback;
  } else {
    stats_.last_source = LocalizeSpeedSource::kNone;
    ++stats_.no_speed;
  }
  stats_.vehicle_age_s = vehicle_speed_ ? static_cast<double>(now_ns - vehicle_ns_) * 1e-9 : -1.0;
  stats_.last_speed_mps = speed;
  stats_.last_yaw_rate_rps = yaw_rate.value_or(0.0);

  auto & tw = odom.twist.twist;
  tw.linear.x = speed;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.x = 0.0;
  tw.angular.y = 0.0;
  tw.angular.z = yaw_rate.value_or(0.0);
}

LocalizeTwistStats LocalizeTwist::stats() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return stats_;
}

}  // namespace autoware::morai_interface
