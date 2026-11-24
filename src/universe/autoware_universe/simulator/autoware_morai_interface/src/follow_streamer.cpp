// 신규 — 26.10.10 _ YSH _ 의사 VILS follow 모드 (헤더 설명 참고)
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

#include "autoware/morai_interface/follow_streamer.hpp"

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{
namespace
{
constexpr size_t kRpcWindow = 200;

double yaw_of(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double quantile(std::vector<double> v, double q)
{
  if (v.empty()) {
    return 0.0;
  }
  const size_t k = std::min(v.size() - 1, static_cast<size_t>(q * static_cast<double>(v.size() - 1) + 0.5));
  std::nth_element(v.begin(), v.begin() + static_cast<std::ptrdiff_t>(k), v.end());
  return v[k];
}
}  // namespace

std::optional<FollowZSource> parse_follow_z_source(const std::string & text)
{
  if (text == "morai_current") {
    return FollowZSource::kMoraiCurrent;
  }
  if (text == "map") {
    return FollowZSource::kMap;
  }
  if (text == "fixed") {
    return FollowZSource::kFixed;
  }
  return std::nullopt;
}

const char * to_string(FollowZSource z)
{
  switch (z) {
    case FollowZSource::kMoraiCurrent:
      return "morai_current";
    case FollowZSource::kMap:
      return "map";
    case FollowZSource::kFixed:
      return "fixed";
  }
  return "?";
}

FollowStreamer::FollowStreamer(
  FollowConfig config, SendFn send, CurrentZFn current_z, MapOriginFn map_origin, NowFn now_ns)
: config_(config), send_(std::move(send)), current_z_(std::move(current_z)), map_origin_(std::move(map_origin)),
  now_ns_(std::move(now_ns))
{
}

FollowStreamer::~FollowStreamer() { stop(); }

void FollowStreamer::submit(const nav_msgs::msg::Odometry & pose, int64_t receive_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  latest_ = pose;
  latest_ns_ = receive_ns;
}

std::optional<morai_grpc_vendor::TransformPose> FollowStreamer::target_for(
  const nav_msgs::msg::Odometry & odom, double age_s) const
{
  geometry_msgs::msg::Pose pose = odom.pose.pose;
  // lead_s + 나이만큼 차체 속도·요레이트로 앞으로 (받은 뒤 흐른 시간도 메운다)
  const double dt = std::max(0.0, config_.lead_s + age_s);
  if (dt > 0.0) {
    const double vx = odom.twist.twist.linear.x;
    const double vy = odom.twist.twist.linear.y;
    const double wz = odom.twist.twist.angular.z;
    const double yaw = yaw_of(pose.orientation);
    const double ym = yaw + 0.5 * wz * dt;
    pose.position.x += dt * (vx * std::cos(ym) - vy * std::sin(ym));
    pose.position.y += dt * (vx * std::sin(ym) + vy * std::cos(ym));
    const double ny = yaw + wz * dt;
    pose.orientation.x = 0.0;
    pose.orientation.y = 0.0;
    pose.orientation.z = std::sin(ny / 2.0);
    pose.orientation.w = std::cos(ny / 2.0);
  }
  std::optional<double> current_z;
  switch (config_.z_source) {
    case FollowZSource::kMoraiCurrent:
      // from_map_pose 는 |map z| > 0.01 이면 map z 를 쓴다 — 지금 MORAI z 를 쓰려면 map z 를 0 으로
      pose.position.z = 0.0;
      current_z = current_z_ ? current_z_() : std::nullopt;
      break;
    case FollowZSource::kMap:
      current_z = current_z_ ? current_z_() : std::nullopt;
      break;
    case FollowZSource::kFixed:
      pose.position.z = config_.fixed_z;
      if (std::fabs(pose.position.z) <= 0.01) {
        pose.position.z = 0.0;
        current_z = config_.fixed_z;
      }
      break;
  }
  return autoware_to_morai::from_map_pose(pose, map_origin_ ? map_origin_() : MapOrigin{}, current_z);
}

bool FollowStreamer::step()
{
  std::optional<nav_msgs::msg::Odometry> pose;
  int64_t pose_ns = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pose = latest_;
    pose_ns = latest_ns_;
  }
  const int64_t now = now_ns_();
  if (!pose) {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.no_pose;
    stats_.last_skip_ns = now;  // 26.10.13 _ YSH _
    return false;
  }
  const double age_s = static_cast<double>(now - pose_ns) * 1e-9;
  if (age_s > config_.max_pose_age_s) {
    const std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.stale_skipped;
    stats_.last_skip_ns = now;  // 26.10.13 _ YSH _
    return false;
  }
  // 26.10.12 _ YSH _ 고스트 명령을 위치보다 먼저 (MORAI 가 물리 없이 위치를 따른다). 실패해도 위치는 보낸다
  // 26.10.13 _ YSH _ 고스트를 목표 외삽 앞으로 옮겼다 — 종전에는 외삽한 뒤 고스트 RPC(p50 ~7 ms, 상한 50 ms)를 기다려
  //   그 시간만큼 낡은 위치가 MORAI 로 갔다 (25 m/s · 50 ms = 1.25 m). 원문 순서: target_for → ghost_ → send_
  if (config_.ghost && ghost_) {
    const auto gst = ghost_(pose->twist.twist.linear.x);
    const std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.ghost_sent;
    if (!(gst.ok || gst.grpc_code == 0)) {
      ++stats_.ghost_failed;
      stats_.ghost_last_error = gst.message;
    } else if (!gst.ok) {
      // 26.10.13 _ YSH _ 전송은 됐는데 MORAI 가 거부 (예: Ego 가 AUTO 가 아니다) — 고스트가 안 먹는 상태
      ++stats_.ghost_rejected;
      stats_.ghost_last_error = gst.message;
    }
  }
  // 26.10.13 _ YSH _ 나이를 고스트 RPC 뒤에 다시 잰다 (위 주석). 원문: const auto target = target_for(*pose, age_s);
  const double send_age_s = static_cast<double>(now_ns_() - pose_ns) * 1e-9;
  const auto target = target_for(*pose, send_age_s);
  if (!target) {
    return false;
  }
  const int64_t t0 = now_ns_();
  const auto st = send_(*target);
  const double ms = static_cast<double>(now_ns_() - t0) * 1e-6;
  const std::lock_guard<std::mutex> lock(mutex_);
  ++stats_.sent;
  stats_.last_target = target;
  stats_.last_attempt_ns = t0;  // 26.10.13 _ YSH _
  rpc_ms_.push_back(ms);
  if (rpc_ms_.size() > kRpcWindow) {
    rpc_ms_.pop_front();
  }
  stats_.rpc_max_ms = std::max(stats_.rpc_max_ms, ms);
  // 보내졌으면 성공으로 센다 (MORAI Result 가 거짓일 수 있다 — 실제 위치는 노드가 스냅샷으로 따로 잰다)
  if (st.ok || st.grpc_code == 0) {
    ++stats_.ok;
    stats_.last_ok_ns = now_ns_();
    if (!st.ok) {
      ++stats_.result_rejected;  // 26.10.13 _ YSH _ 전송은 됐고 Result 만 거부 — 실제 위치는 진단이 오차로 본다
      stats_.last_error = st.message;
    }
  } else {
    ++stats_.failed;
    stats_.last_error = st.message;
  }
  return true;
}

void FollowStreamer::start()
{
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread([this]() { loop(); });
}

void FollowStreamer::stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void FollowStreamer::loop()
{
  const auto period = std::chrono::nanoseconds(static_cast<int64_t>(1e9 / std::max(1.0, config_.rate_hz)));
  auto next = std::chrono::steady_clock::now();
  while (running_) {
    step();
    next += period;
    const auto now = std::chrono::steady_clock::now();
    if (next < now) {
      next = now;  // 늦으면 몰아서 따라잡지 않는다 — 다음 틱에 최신 값 하나
    }
    std::unique_lock<std::mutex> lk(wake_mutex_);
    wake_cv_.wait_until(lk, next, [this]() { return !running_; });
  }
}

FollowStats FollowStreamer::stats() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  FollowStats s = stats_;
  const std::vector<double> v(rpc_ms_.begin(), rpc_ms_.end());
  s.rpc_p50_ms = quantile(v, 0.5);
  s.rpc_p99_ms = quantile(v, 0.99);
  return s;
}

}  // namespace autoware::morai_interface
