// 신규 — 26.10.13 _ YSH _ VILS 차량 PC 고스트(ego_mode localize) twist 채우기 (가짜 시계)
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

#include <gtest/gtest.h>

#include <cmath>

namespace autoware::morai_interface
{
namespace
{
constexpr int64_t kMs = 1000000;

nav_msgs::msg::Odometry ghost(double x, double y, double yaw)
{
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.orientation.z = std::sin(yaw / 2.0);
  o.pose.pose.orientation.w = std::cos(yaw / 2.0);
  // MORAI 가 위치만 옮긴 고스트라 보고하는 속도는 0 (10-13 bag 그대로)
  return o;
}

// 직선 v [m/s], 회전 w [rad/s] 로 25 ms 마다 움직이는 고스트를 t_end 까지 흘려 마지막 결과를 돌려준다.
// repeat_every > 0 이면 그 간격마다 직전 표본을 한 번 더 보낸다 — 차는 계속 움직이고 MORAI 표본만 늦은 것
// (bag 의 반복 표본 16 % 흉내)
nav_msgs::msg::Odometry run(
  LocalizeTwist & lt, double v, double w, double yaw0, double t_end_s, int repeat_every = 0,
  int64_t t0_ns = 0)
{
  double x = 0.0, y = 0.0, yaw = yaw0;
  nav_msgs::msg::Odometry out = ghost(x, y, yaw);
  int i = 0;
  for (int64_t t = t0_ns; t <= t0_ns + static_cast<int64_t>(t_end_s * 1e9); t += 25 * kMs, ++i) {
    x += v * 0.025 * std::cos(yaw);
    y += v * 0.025 * std::sin(yaw);
    yaw += w * 0.025;
    const bool repeat = repeat_every > 0 && i % repeat_every == repeat_every - 1;
    auto sample = repeat ? out : ghost(x, y, yaw);
    lt.apply(sample, t, t);
    out = sample;
  }
  return out;
}
}  // namespace

TEST(LocalizeTwist, UsesFreshVehicleSpeed)
{
  LocalizeTwist lt({0.2, 0.2});
  lt.set_vehicle_speed(3.3, 0);
  auto o = ghost(1.0, 2.0, 0.3);
  lt.apply(o, 10 * kMs, 50 * kMs);
  EXPECT_DOUBLE_EQ(o.twist.twist.linear.x, 3.3);
  EXPECT_EQ(lt.stats().last_source, LocalizeSpeedSource::kVehicle);
  // 위치는 그대로
  EXPECT_DOUBLE_EQ(o.pose.pose.position.x, 1.0);
  EXPECT_DOUBLE_EQ(o.pose.pose.position.y, 2.0);
}

TEST(LocalizeTwist, StaleVehicleSpeedFallsBackToPoseSpeed)
{
  LocalizeTwist lt({0.2, 0.2});
  lt.set_vehicle_speed(9.9, 0);  // 아래 시각에서는 0.2 s 를 한참 넘는다
  const auto o = run(lt, 5.0, 0.0, 0.5, 1.0, 0, 2'000 * kMs);
  EXPECT_NEAR(o.twist.twist.linear.x, 5.0, 0.05);
  EXPECT_EQ(lt.stats().last_source, LocalizeSpeedSource::kPose);
  EXPECT_GT(lt.stats().pose_fallback, 0U);
}

TEST(LocalizeTwist, PoseSpeedIsNegativeWhenReversing)
{
  LocalizeTwist lt({0.2, 0.2});
  const auto o = run(lt, -2.0, 0.0, 1.0, 1.0);
  EXPECT_NEAR(o.twist.twist.linear.x, -2.0, 0.05);
}

TEST(LocalizeTwist, YawRateFromPoseWithRepeatedSamples)
{
  LocalizeTwist lt({0.2, 0.2});
  const auto left = run(lt, 4.0, 0.15, 0.0, 2.0, 6);
  // 반복 표본 때문에 창 안의 실제 회전이 조금 줄어든다 — 부호와 크기가 맞으면 된다
  EXPECT_NEAR(left.twist.twist.angular.z, 0.15, 0.03);
  LocalizeTwist lt2({0.2, 0.2});
  const auto right = run(lt2, 4.0, -0.15, 0.0, 2.0);
  EXPECT_NEAR(right.twist.twist.angular.z, -0.15, 0.01);
}

TEST(LocalizeTwist, YawRateAcrossPiWrap)
{
  LocalizeTwist lt({0.2, 0.2});
  // π 바로 아래에서 시작해 좌회전 → 헤딩이 -π 쪽으로 넘어간다
  const auto o = run(lt, 3.0, 0.3, M_PI - 0.05, 1.0);
  EXPECT_NEAR(o.twist.twist.angular.z, 0.3, 0.01);
}

TEST(LocalizeTwist, OtherTwistComponentsAreZero)
{
  LocalizeTwist lt({0.2, 0.2});
  auto o = ghost(0.0, 0.0, 0.0);
  o.twist.twist.linear.y = 7.0;
  o.twist.twist.angular.x = 1.0;
  lt.apply(o, 0, 0);
  EXPECT_DOUBLE_EQ(o.twist.twist.linear.y, 0.0);
  EXPECT_DOUBLE_EQ(o.twist.twist.angular.x, 0.0);
  EXPECT_DOUBLE_EQ(o.twist.twist.angular.y, 0.0);
}

TEST(LocalizeTwist, NothingKnownYieldsZeroAndNone)
{
  LocalizeTwist lt({0.2, 0.2});
  auto o = ghost(0.0, 0.0, 0.0);
  lt.apply(o, 0, 0);
  EXPECT_DOUBLE_EQ(o.twist.twist.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(o.twist.twist.angular.z, 0.0);
  EXPECT_EQ(lt.stats().last_source, LocalizeSpeedSource::kNone);
  EXPECT_DOUBLE_EQ(lt.stats().vehicle_age_s, -1.0);
}

TEST(LocalizeTwist, JumpResetsWindow)
{
  LocalizeTwist lt({0.2, 0.2});
  run(lt, 5.0, 0.2, 0.0, 1.0);
  // 차량 PC 가 고스트를 50 m 옮겼다 — 이 표본에서 속도·요레이트를 만들지 않는다
  auto o = ghost(50.0, 0.0, 1.0);
  lt.apply(o, 1'025 * kMs, 1'025 * kMs);
  EXPECT_DOUBLE_EQ(o.twist.twist.angular.z, 0.0);
  EXPECT_EQ(lt.stats().last_source, LocalizeSpeedSource::kNone);
}

TEST(LocalizeTwist, LongGapResetsWindow)
{
  LocalizeTwist lt({0.2, 0.2});
  run(lt, 5.0, 0.2, 0.0, 1.0);
  auto o = ghost(5.5, 0.5, 0.3);
  lt.apply(o, 5'000 * kMs, 5'000 * kMs);  // 4 s 공백
  EXPECT_DOUBLE_EQ(o.twist.twist.angular.z, 0.0);
  EXPECT_EQ(lt.stats().last_source, LocalizeSpeedSource::kNone);
}

TEST(LocalizeTwist, VehicleSpeedWinsOverPoseSpeed)
{
  LocalizeTwist lt({0.2, 0.2});
  double x = 0.0;
  nav_msgs::msg::Odometry o;
  for (int64_t t = 0; t <= 1'000 * kMs; t += 25 * kMs) {
    x += 0.1;  // 위치로는 4 m/s
    lt.set_vehicle_speed(4.2, t);
    o = ghost(x, 0.0, 0.0);
    lt.apply(o, t, t);
  }
  EXPECT_DOUBLE_EQ(o.twist.twist.linear.x, 4.2);
  EXPECT_EQ(lt.stats().pose_fallback, 0U);
}

}  // namespace autoware::morai_interface
