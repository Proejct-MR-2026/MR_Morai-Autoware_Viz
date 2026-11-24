// 신규 — 26.10.10 _ YSH _ 의사 VILS follow 모드: 실차 위치(Autoware map)를 MORAI Ego 에 SetTransform 으로 계속 옮긴다.
//   최신 위치 하나만 잡아 두고(쌓지 않는다) rate_hz 마다 한 번 보낸다. 한 번에 한 호출만 — 늦으면 다음 틱에 최신 값.
//   ControlVehicle 은 보내지 않는다 (follow 모드에서 노드가 명령 송신을 잠근다). gRPC 만 쓴다.
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

#ifndef AUTOWARE__MORAI_INTERFACE__FOLLOW_STREAMER_HPP_
#define AUTOWARE__MORAI_INTERFACE__FOLLOW_STREAMER_HPP_

#include "autoware/morai_interface/map_origin.hpp"

#include <morai_grpc_vendor/morai_link_types.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace autoware::morai_interface
{

enum class FollowZSource { kMoraiCurrent, kMap, kFixed };
std::optional<FollowZSource> parse_follow_z_source(const std::string & text);
const char * to_string(FollowZSource z);

struct FollowConfig
{
  double rate_hz{20.0};          ///< SetTransform 주기 상한 (벤치 결과로 정한다)
  double max_pose_age_s{0.3};    ///< 이보다 낡은 위치는 보내지 않는다 (실차 측위가 끊기면 MORAI Ego 도 멈춘다)
  double lead_s{0.0};            ///< 위치를 이만큼 앞으로 외삽해 보낸다 (RPC 지연 보상)
  FollowZSource z_source{FollowZSource::kMoraiCurrent};
  double fixed_z{0.0};
  double fail_timeout_s{0.5};    ///< 성공한 SetTransform 이 이만큼 없으면 진단 ERROR
  // 26.10.12 _ YSH _ 고스트: 매 틱 SetTransform 직전에 ControlVehicle(GHOST, 실차 속도)을 보낸다 (10-12 실 MORAI 벤치 방식 c)
  bool ghost{true};
};

struct FollowStats
{
  uint64_t sent{0};
  uint64_t ok{0};
  uint64_t failed{0};
  uint64_t stale_skipped{0};   ///< 위치가 낡아 보내지 않은 틱
  uint64_t no_pose{0};         ///< 위치를 아직 못 받은 틱
  double rpc_p50_ms{0.0};
  double rpc_p99_ms{0.0};
  double rpc_max_ms{0.0};
  int64_t last_ok_ns{0};
  std::string last_error;
  // 26.10.12 _ YSH _ 고스트 명령 (ControlVehicle GHOST)
  uint64_t ghost_sent{0};
  uint64_t ghost_failed{0};
  std::string ghost_last_error;
  std::optional<morai_grpc_vendor::TransformPose> last_target;
  // 26.10.13 _ YSH _ 전송은 됐지만 MORAI Result 가 거부한 수 (grpc OK · ok=false). 생존 판정에는 넣지 않는다
  uint64_t result_rejected{0};
  uint64_t ghost_rejected{0};
  // 26.10.13 _ YSH _ 마지막으로 SetTransform 을 시도한 시각 · 실차 측위가 없거나 낡아 건너뛴 시각 (진단이 원인을 가른다)
  int64_t last_attempt_ns{0};
  int64_t last_skip_ns{0};
};

class FollowStreamer
{
public:
  using SendFn = std::function<morai_grpc_vendor::Status(const morai_grpc_vendor::TransformPose &)>;
  using CurrentZFn = std::function<std::optional<double>()>;
  using MapOriginFn = std::function<MapOrigin()>;
  using NowFn = std::function<int64_t()>;
  /// 26.10.12 _ YSH _ 고스트 명령 송신 (인자 = 실차 속도 [m/s], 차체 앞 +)
  using GhostFn = std::function<morai_grpc_vendor::Status(double speed_mps)>;

  FollowStreamer(FollowConfig config, SendFn send, CurrentZFn current_z, MapOriginFn map_origin, NowFn now_ns);
  ~FollowStreamer();
  FollowStreamer(const FollowStreamer &) = delete;
  FollowStreamer & operator=(const FollowStreamer &) = delete;

  /// 26.10.12 _ YSH _ config.ghost 일 때 매 틱 SetTransform 직전에 부른다. start() 전에 한 번 준다
  void set_ghost(GhostFn ghost) { ghost_ = std::move(ghost); }
  /// 최신 위치 (map, base_link). 콜백 스레드에서 부른다 — 복사만 한다
  void submit(const nav_msgs::msg::Odometry & pose, int64_t receive_ns);
  /// 한 틱: 위치가 있고 낡지 않았으면 MORAI 좌표로 바꿔 보낸다(블로킹). 보냈으면 true
  bool step();
  /// 전용 스레드로 rate_hz 마다 step()
  void start();
  void stop();

  FollowStats stats() const;
  const FollowConfig & config() const { return config_; }
  /// 보낼 MORAI 목표 (시험·진단용, 보내지 않는다)
  std::optional<morai_grpc_vendor::TransformPose> target_for(
    const nav_msgs::msg::Odometry & pose, double age_s) const;

private:
  void loop();

  FollowConfig config_;
  SendFn send_;
  CurrentZFn current_z_;
  MapOriginFn map_origin_;
  NowFn now_ns_;
  GhostFn ghost_;

  mutable std::mutex mutex_;
  std::optional<nav_msgs::msg::Odometry> latest_;
  int64_t latest_ns_{0};
  FollowStats stats_;
  std::deque<double> rpc_ms_;  ///< 최근 RPC 시간 (분위수용)

  std::thread thread_;
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::atomic<bool> running_{false};
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__FOLLOW_STREAMER_HPP_
