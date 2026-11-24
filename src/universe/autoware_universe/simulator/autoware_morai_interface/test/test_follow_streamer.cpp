// 신규 — 26.10.10 _ YSH _ 의사 VILS follow 모드 스트리머 (가짜 송신·가짜 시계)
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <string>  // 26.10.12 _ YSH _
#include <thread>
#include <vector>

namespace autoware::morai_interface
{
namespace
{
namespace mgv = morai_grpc_vendor;
constexpr int64_t kMs = 1000000;

nav_msgs::msg::Odometry pose(double x, double y, double yaw, double vx = 0.0, double z = 0.0)
{
  nav_msgs::msg::Odometry o;
  o.header.frame_id = "map";
  o.pose.pose.position.x = x;
  o.pose.pose.position.y = y;
  o.pose.pose.position.z = z;
  o.pose.pose.orientation.z = std::sin(yaw / 2.0);
  o.pose.pose.orientation.w = std::cos(yaw / 2.0);
  o.twist.twist.linear.x = vx;
  return o;
}

struct Rig
{
  int64_t now{1000 * kMs};
  std::vector<mgv::TransformPose> sent;
  bool fail{false};
  std::optional<double> morai_z{2.5};
  FollowStreamer s;
  explicit Rig(FollowConfig c = FollowConfig{})
  : s(
      c,
      [this](const mgv::TransformPose & t) {
        sent.push_back(t);
        now += 5 * kMs;  // RPC 5 ms
        mgv::Status st;
        st.ok = !fail;
        st.grpc_code = fail ? 14 : 0;
        st.message = fail ? "unavailable" : "";
        return st;
      },
      [this]() { return morai_z; }, []() { return MapOrigin{}; }, [this]() { return now; })
  {
  }
};
}  // namespace

TEST(FollowStreamer, SendsNothingWithoutAFreshPose)
{
  Rig r;
  EXPECT_FALSE(r.s.step());
  EXPECT_EQ(r.s.stats().no_pose, 1U);
  r.s.submit(pose(1, 2, 0), r.now);
  r.now += 400 * kMs;  // max_pose_age_s 0.3 넘김 — 실차 측위가 끊기면 MORAI Ego 도 멈춘다
  EXPECT_FALSE(r.s.step());
  EXPECT_EQ(r.s.stats().stale_skipped, 1U);
  EXPECT_TRUE(r.sent.empty());
}

TEST(FollowStreamer, SendsOnlyTheLatestPoseInMoraiFrameWithCurrentZ)
{
  Rig r;
  r.s.submit(pose(1, 2, 0), r.now);
  r.s.submit(pose(10, 20, M_PI / 2, 0.0, 0.6), r.now);  // z 0.6 은 morai_current 에서 무시된다
  EXPECT_TRUE(r.s.step());
  ASSERT_EQ(r.sent.size(), 1U);
  EXPECT_DOUBLE_EQ(r.sent[0].position.x, 10.0);
  EXPECT_DOUBLE_EQ(r.sent[0].position.y, 20.0);
  EXPECT_DOUBLE_EQ(r.sent[0].position.z, 2.5) << "z from MORAI current";
  EXPECT_NEAR(r.sent[0].rotation_deg.z, 90.0, 1e-9);
  const auto st = r.s.stats();
  EXPECT_EQ(st.sent, 1U);
  EXPECT_EQ(st.ok, 1U);
  EXPECT_NEAR(st.rpc_p50_ms, 5.0, 1e-9);
}

TEST(FollowStreamer, ExtrapolatesByAgePlusLead)
{
  FollowConfig c;
  c.lead_s = 0.1;
  Rig r(c);
  r.s.submit(pose(0, 0, 0, 10.0), r.now);
  r.now += 50 * kMs;  // 받은 뒤 50 ms
  EXPECT_TRUE(r.s.step());
  EXPECT_NEAR(r.sent[0].position.x, 10.0 * 0.15, 1e-9) << "vx * (age 0.05 + lead 0.1)";
}

TEST(FollowStreamer, MapAndFixedZSources)
{
  FollowConfig c;
  c.z_source = FollowZSource::kMap;
  Rig r(c);
  r.s.submit(pose(0, 0, 0, 0.0, 0.6), r.now);
  r.s.step();
  EXPECT_DOUBLE_EQ(r.sent[0].position.z, 0.6);
  FollowConfig f;
  f.z_source = FollowZSource::kFixed;
  f.fixed_z = 0.0;
  Rig q(f);
  q.s.submit(pose(0, 0, 0, 0.0, 0.6), q.now);
  q.s.step();
  EXPECT_DOUBLE_EQ(q.sent[0].position.z, 0.0);
  EXPECT_FALSE(parse_follow_z_source("ground").has_value());
}

TEST(FollowStreamer, CountsFailures)
{
  Rig r;
  r.fail = true;
  r.s.submit(pose(0, 0, 0), r.now);
  EXPECT_TRUE(r.s.step());
  const auto st = r.s.stats();
  EXPECT_EQ(st.failed, 1U);
  EXPECT_EQ(st.ok, 0U);
  EXPECT_EQ(st.last_ok_ns, 0);
  EXPECT_EQ(st.last_error, "unavailable");
}

// 26.10.12 _ YSH _ 고스트: GHOST 명령이 위치보다 먼저, 실차 속도로. 꺼져 있으면 보내지 않는다. 실패해도 위치는 보낸다
TEST(FollowStreamer, GhostCommandPrecedesEachTransformWithTheRealSpeed)
{
  Rig r;
  std::vector<std::string> order;
  std::vector<double> speeds;
  bool ghost_fail = false;
  r.s.set_ghost([&](double v) {
    order.push_back("ghost");
    speeds.push_back(v);
    mgv::Status st;
    st.ok = !ghost_fail;
    st.grpc_code = ghost_fail ? 9 : 0;
    st.message = ghost_fail ? "not auto" : "";
    return st;
  });
  r.s.submit(pose(0, 0, 0, 7.5), r.now);
  ASSERT_TRUE(r.s.step());
  ghost_fail = true;
  r.s.submit(pose(1, 0, 0, 8.0), r.now);
  ASSERT_TRUE(r.s.step());
  ASSERT_EQ(speeds.size(), 2U);
  EXPECT_DOUBLE_EQ(speeds[0], 7.5);
  EXPECT_EQ(r.sent.size(), 2U) << "a failed ghost command does not block the transform";
  const auto st = r.s.stats();
  EXPECT_EQ(st.ghost_sent, 2U);
  EXPECT_EQ(st.ghost_failed, 1U);
  EXPECT_EQ(st.ghost_last_error, "not auto");

  FollowConfig off;
  off.ghost = false;
  Rig q(off);
  int calls = 0;
  q.s.set_ghost([&](double) {
    ++calls;
    return mgv::Status{};
  });
  q.s.submit(pose(0, 0, 0), q.now);
  q.s.step();
  EXPECT_EQ(calls, 0);
  EXPECT_EQ(q.sent.size(), 1U);
}

// 26.10.13 _ YSH _ 고스트 RPC 시간만큼 낡지 않게: 목표는 고스트 응답 뒤의 나이로 외삽한다
TEST(FollowStreamer, ExtrapolatesAfterTheGhostCall)
{
  Rig r;
  r.s.set_ghost([&](double) {
    r.now += 40 * kMs;  // 고스트 RPC 40 ms
    mgv::Status st;
    st.ok = true;
    st.grpc_code = 0;
    return st;
  });
  r.s.submit(pose(0, 0, 0, 10.0), r.now);
  ASSERT_TRUE(r.s.step());
  EXPECT_NEAR(r.sent[0].position.x, 10.0 * 0.04, 1e-9) << "age includes the 40 ms ghost RPC";
}

// 26.10.13 _ YSH _ grpc OK 인데 Result 가 거부: 생존(ok·last_ok)은 유지하고 거부 수만 따로 센다 (고스트도 같다)
TEST(FollowStreamer, CountsResultRejectionsSeparately)
{
  FollowConfig c;
  Rig r(c);
  mgv::Status rejected;
  rejected.ok = false;
  rejected.grpc_code = 0;
  rejected.message = "result false";
  r.s.set_ghost([&](double) { return rejected; });
  FollowStreamer s(
    c, [&](const mgv::TransformPose &) { return rejected; }, []() { return std::optional<double>{}; },
    []() { return MapOrigin{}; }, [&]() { return r.now; });
  s.set_ghost([&](double) { return rejected; });
  s.submit(pose(0, 0, 0), r.now);
  ASSERT_TRUE(s.step());
  const auto st = s.stats();
  EXPECT_EQ(st.ok, 1U);
  EXPECT_EQ(st.result_rejected, 1U);
  EXPECT_EQ(st.ghost_rejected, 1U);
  EXPECT_EQ(st.ghost_failed, 0U);
  EXPECT_GT(st.last_attempt_ns, 0);
}

// 26.10.13 _ YSH _ 측위가 없거나 낡아 건너뛴 시각을 남긴다 (진단이 MORAI 탓과 가른다)
TEST(FollowStreamer, RecordsWhenItSkipsForMissingOrStalePose)
{
  Rig r;
  EXPECT_FALSE(r.s.step());
  EXPECT_EQ(r.s.stats().last_skip_ns, r.now);
  r.s.submit(pose(0, 0, 0), r.now);
  r.now += 2000 * kMs;
  EXPECT_FALSE(r.s.step());
  const auto st = r.s.stats();
  EXPECT_EQ(st.last_skip_ns, r.now);
  EXPECT_EQ(st.last_attempt_ns, 0);
}

TEST(FollowStreamer, RealThreadKeepsTheRateAndNeverQueues)
{
  FollowConfig c;
  c.rate_hz = 50.0;
  std::atomic<int> calls{0};
  FollowStreamer s(
    c,
    [&calls](const mgv::TransformPose &) {
      ++calls;
      mgv::Status st;
      st.ok = true;
      st.grpc_code = 0;
      return st;
    },
    []() { return std::optional<double>(0.0); }, []() { return MapOrigin{}; },
    []() {
      return static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
    });
  const auto t0 = std::chrono::steady_clock::now();
  // 위치를 100 Hz 로 넣는다 — 스트리머는 50 Hz 로 최신 것만 보낸다
  s.start();
  for (int i = 0; i < 40; ++i) {
    s.submit(
      pose(i, 0, 0),
      static_cast<int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()));
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  s.stop();
  const double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  EXPECT_NEAR(calls.load() / dt, 50.0, 8.0);
}

}  // namespace autoware::morai_interface
