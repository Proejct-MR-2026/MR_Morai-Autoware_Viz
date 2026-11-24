// 신규 — 26.10.13 _ YSH _ VILS 차량 PC 고스트(ego_mode localize): /localization/kinematic_state 의 twist 를 채운다.
//   차량 PC 는 MORAI Ego 를 위치로만 옮겨서 MORAI 가 보고하는 Ego 속도가 늘 0 이다. 그대로 내면 Autoware 의 종방향
//   PID 는 "서 있다" 로 보고 계속 가속하고(10-13 리프트 bag: 목표 1.5 m/s 에 바퀴 50 km/h), MPC 는 v = 0 모델로
//   조향해 진동한다. 그래서
//     linear.x  = 실차 바퀴 속도 (/vehicle/status/velocity_status — K-CAN). 낡으면 위치 변화로 잰 속도로 대신한다
//     angular.z = 위치의 헤딩 변화율 (K-CAN 요레이트 부호가 실차 미확인이라 위치에서 잰다 — 부호가 구조적으로 맞다)
//     linear.y·angular.x·angular.y = 0
//   ROS 노드·실행기와 무관한 순수 로직 (가짜 시계로 시험한다).
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

#ifndef AUTOWARE__MORAI_INTERFACE__LOCALIZE_TWIST_HPP_
#define AUTOWARE__MORAI_INTERFACE__LOCALIZE_TWIST_HPP_

#include <nav_msgs/msg/odometry.hpp>

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::morai_interface
{

struct LocalizeTwistConfig
{
  double velocity_max_age_s{0.2};  ///< 실차 속도가 이보다 낡으면 위치 변화 속도로 대신한다
  double window_s{0.2};            ///< 위치·헤딩 변화율을 재는 창 (MORAI 표본 25 ms, 같은 위치가 반복되는 표본이 섞인다)
};

/// 어느 값으로 linear.x 를 채웠는가 (진단용)
enum class LocalizeSpeedSource { kVehicle, kPose, kNone };

const char * to_string(LocalizeSpeedSource source);

struct LocalizeTwistStats
{
  LocalizeSpeedSource last_source{LocalizeSpeedSource::kNone};
  uint64_t vehicle_used{0};
  uint64_t pose_fallback{0};  ///< 실차 속도가 없거나 낡아 위치 변화 속도를 쓴 표본
  uint64_t no_speed{0};       ///< 둘 다 없어 0 을 낸 표본 (기동 직후 창이 차기 전)
  double vehicle_age_s{-1.0};  ///< 마지막 apply 때 실차 속도 나이 (-1 = 받은 적 없음)
  double last_speed_mps{0.0};
  double last_yaw_rate_rps{0.0};
};

class LocalizeTwist
{
public:
  explicit LocalizeTwist(const LocalizeTwistConfig & config);

  /// 실차 속도 [m/s] (VelocityReport.longitudinal_velocity, 후진은 음수). 구독 콜백 스레드에서 부른다.
  void set_vehicle_speed(double speed_mps, int64_t now_ns);

  /// odom 의 twist 를 바꿔 쓴다. pose 는 건드리지 않는다. sample_ns = 이 표본의 시각 (stamp),
  /// now_ns = 실차 속도 나이를 재는 시계 (set_vehicle_speed 와 같은 시계). 엔진 스레드 한 곳에서만 부른다.
  void apply(nav_msgs::msg::Odometry & odom, int64_t sample_ns, int64_t now_ns);

  LocalizeTwistStats stats() const;

private:
  struct PoseSample
  {
    int64_t ns;
    double x;
    double y;
    double yaw;
  };

  LocalizeTwistConfig config_;
  std::deque<PoseSample> history_;  // apply 스레드 전용

  mutable std::mutex mutex_;  // 아래 둘
  std::optional<double> vehicle_speed_;
  int64_t vehicle_ns_{0};
  LocalizeTwistStats stats_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__LOCALIZE_TWIST_HPP_
