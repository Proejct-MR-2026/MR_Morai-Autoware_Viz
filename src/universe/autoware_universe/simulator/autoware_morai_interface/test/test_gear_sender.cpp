// 신규 — 26.09.25 _ YSH _ 기어 송신기 시험 (main.py _GearSender 의미: 변경 시 송신·재시도·재동기·차단·epoch)
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

#include "autoware/morai_interface/gear_sender.hpp"

#include <rclcpp/logger.hpp>

#include <gtest/gtest.h>

#include <optional>
#include <vector>

namespace autoware::morai_interface
{
namespace
{

struct Harness
{
  double now{100.0};
  std::vector<int> sent;
  bool fail{false};
  bool timeout{false};
  std::optional<int> actual;
  std::function<void()> during_send;
  GearSender sender;

  Harness()
  : sender(
      [this](int gear) {
        sent.push_back(gear);
        if (during_send) {
          during_send();
        }
        morai_grpc_vendor::Status s;
        if (timeout) {
          s.grpc_code = 4;  // DEADLINE_EXCEEDED
          s.ok = false;
        } else {
          s.grpc_code = 0;
          s.ok = !fail;
          s.message = fail ? "readback mismatch" : "";
        }
        return s;
      },
      [this]() { return actual; }, GearSenderConfig{1.0, 0.2}, rclcpp::get_logger("test_gear"), true,
      [this]() { return now; })
  {
  }
};

TEST(GearSender, SendsOnlyOnChange)
{
  Harness h;
  h.sender.submit(4);
  h.sender.tick();
  h.sender.tick();
  EXPECT_EQ(h.sent, (std::vector<int>{4}));
  EXPECT_EQ(h.sender.last_ok(), 4);
  h.sender.submit(4);
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U) << "same gear must not be resent";
  h.sender.submit(1);
  h.sender.tick();
  EXPECT_EQ(h.sent, (std::vector<int>{4, 1}));
}

TEST(GearSender, SameGearFailureRetriesAfterRetryPeriodNewGearImmediately)
{
  Harness h;
  h.fail = true;
  h.sender.submit(4);
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U);
  EXPECT_FALSE(h.sender.last_ok().has_value());
  h.now += 0.1;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U) << "retry_s (0.2 s) not elapsed";
  h.now += 0.15;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 2U);
  h.sender.submit(2);  // 새 목표는 곧바로
  h.sender.tick();
  EXPECT_EQ(h.sent.back(), 2);
  const auto c = h.sender.counters();
  EXPECT_EQ(c.errors, 3U);
  EXPECT_EQ(c.ok, 0U);
}

TEST(GearSender, TimeoutsAreCountedSeparately)
{
  Harness h;
  h.timeout = true;
  h.sender.submit(4);
  h.sender.tick();
  EXPECT_EQ(h.sender.counters().timeouts, 1U);
  EXPECT_EQ(h.sender.counters().errors, 0U);
}

TEST(GearSender, ResyncsWhenMoraiGearDiffers)
{
  Harness h;
  h.sender.submit(4);
  h.sender.tick();
  ASSERT_EQ(h.sent.size(), 1U);
  h.actual = 1;  // 누가 MORAI 키보드로 P 로 바꿨다
  h.now += 0.5;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U) << "resync_s (1 s) not elapsed";
  h.now += 0.6;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 2U);
  EXPECT_EQ(h.sent.back(), 4);
  EXPECT_EQ(h.sender.counters().resyncs, 1U);
  h.actual = 4;
  h.now += 2.0;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 2U) << "in sync — nothing to send";
}

TEST(GearSender, NoResyncForManualOrLowGear)
{
  Harness h;
  h.sender.submit(5);  // L
  h.sender.tick();
  h.actual = 4;
  h.now += 5.0;
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U);
}

TEST(GearSender, ForwardingOffClearsTargetUntilNextSubmit)
{
  Harness h;
  h.sender.submit(4);
  h.sender.tick();
  h.sender.set_forwarding(false);
  EXPECT_FALSE(h.sender.target().has_value());
  EXPECT_FALSE(h.sender.last_ok().has_value());
  h.actual = 1;
  h.now += 5.0;
  h.sender.tick();
  h.sender.set_forwarding(true);
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 1U) << "re-enabling alone must not send";
  h.sender.submit(4);
  h.sender.tick();
  EXPECT_EQ(h.sent.size(), 2U) << "after forwarding off, the same gear is sent again";
}

TEST(GearSender, SuccessDuringForwardingOffIsDiscarded)
{
  Harness h;
  h.during_send = [&h]() { h.sender.set_forwarding(false); };  // 송신 중 차단 (epoch 증가)
  h.sender.submit(4);
  h.sender.tick();
  EXPECT_FALSE(h.sender.last_ok().has_value());
}

TEST(GearSender, NamesMatchMainPy)
{
  EXPECT_EQ(GearSender::name(4), "D(4)");
  EXPECT_EQ(GearSender::name(1), "P(1)");
  EXPECT_EQ(GearSender::name(std::nullopt), "?");
}

}  // namespace
}  // namespace autoware::morai_interface
