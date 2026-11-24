// 신규 — 26.09.25 _ YSH _ 동기 모드 순수 헬퍼·morai_sync_restore 옵션·종료 코드 gtest (네트워크 없음)
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

#include "sync_mode.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <numeric>
#include <string>
#include <vector>

namespace
{
namespace sm = morai_grpc_vendor::sync_mode;

constexpr int64_t kEpochUs = 1758729600LL * 1000000LL;  // 2025-09-24T16:00:00Z

TEST(SyncMode, EffectiveTickPeriodMirrorsTheServer)
{
  EXPECT_EQ(sm::effective_tick_period_ms(0), 10);  // 0 -> 10
  EXPECT_EQ(sm::effective_tick_period_ms(1), 1);
  EXPECT_EQ(sm::effective_tick_period_ms(10), 10);
  EXPECT_EQ(sm::effective_tick_period_ms(100), 100);
  EXPECT_EQ(sm::effective_tick_period_ms(101), 20);  // out of 1..100 -> 20
  EXPECT_EQ(sm::effective_tick_period_ms(-5), 20);
}

TEST(SyncMode, ControlIsAcceptedOnlyInAutoOrSynchronousMode)
{
  EXPECT_TRUE(sm::mode_accepts_control_in_sync(3));
  EXPECT_TRUE(sm::mode_accepts_control_in_sync(7));
  for (int m : {0, 1, 2, 4, 5, 6, 8}) {
    EXPECT_FALSE(sm::mode_accepts_control_in_sync(m)) << m;
  }
}

TEST(SyncMode, NamesMatchTheProto)
{
  EXPECT_EQ(sm::sync_type_name(0), "SYNC_MODE_TYPE_UNSPECIFIED");
  EXPECT_EQ(sm::sync_type_name(1), "SYNC_MODE_TYPE_TIME_MANAGEMENT");
  EXPECT_EQ(sm::sync_type_name(2), "SYNC_MODE_TYPE_SYNCHRONOUS");
  EXPECT_EQ(sm::sync_type_name(9), "UNKNOWN(9)");
}

TEST(SyncMode, BackoffDoublesCapsAndStaysWithinTheBudget)
{
  const std::vector<int> d = sm::backoff_delays_ms(5000, 50, 500);
  ASSERT_GE(d.size(), 5u);
  EXPECT_EQ(d[0], 50);
  EXPECT_EQ(d[1], 100);
  EXPECT_EQ(d[2], 200);
  EXPECT_EQ(d[3], 400);
  EXPECT_EQ(d[4], 500);  // capped
  const int sum = std::accumulate(d.begin(), d.end(), 0);
  EXPECT_LE(sum, 5000);
  EXPECT_GT(sum, 5000 - 500);  // uses the budget
  for (int x : d) {
    EXPECT_LE(x, 500);
  }
  EXPECT_TRUE(sm::backoff_delays_ms(40, 50, 500).empty());
  EXPECT_TRUE(sm::backoff_delays_ms(0, 50, 500).empty());
  EXPECT_TRUE(sm::backoff_delays_ms(1000, 0, 500).empty());
  EXPECT_TRUE(sm::backoff_delays_ms(1000, 50, -1).empty());
  const std::vector<int> ten = sm::backoff_delays_ms(10000, 100, 1000);
  EXPECT_LE(std::accumulate(ten.begin(), ten.end(), 0), 10000);
  EXPECT_GE(ten.size(), 10u);
}

TEST(SyncMode, EpochMicrosecondsVersusSimTime)
{
  EXPECT_TRUE(sm::is_epoch_us(kEpochUs));
  EXPECT_FALSE(sm::is_epoch_us(0));
  EXPECT_FALSE(sm::is_epoch_us(10000));         // 10 ms of sim time after entering sync mode
  EXPECT_FALSE(sm::is_epoch_us(3600000000LL));  // 1 h of sim time
  EXPECT_FALSE(sm::is_epoch_us(kEpochUs * 1000));  // ns, not µs
  EXPECT_FALSE(sm::is_epoch_us(-kEpochUs));
}

std::vector<sm::TimeSample> samples(double span_s, int n, int64_t ts0, double rate)
{
  std::vector<sm::TimeSample> v;
  for (int i = 0; i < n; ++i) {
    const double t = span_s * i / (n - 1);
    // 16.67 ms steps like real-time MORAI
    const int64_t steps = static_cast<int64_t>(std::floor(t * rate / 0.016667));
    v.push_back({t, ts0 + steps * 16667});
  }
  return v;
}

TEST(SyncMode, FrozenNeedsThreeSamplesOverTheSpanWithoutChange)
{
  std::vector<sm::TimeSample> f = {{0.0, 0}, {0.1, 0}, {0.2, 0}, {0.31, 0}};
  const sm::FrozenCheck ok = sm::judge_frozen(f, 0.3);
  EXPECT_TRUE(ok.frozen) << ok.why;
  EXPECT_EQ(ok.changes, 0u);
  f[2].ts_us = 10000;
  const sm::FrozenCheck moved = sm::judge_frozen(f, 0.3);
  EXPECT_FALSE(moved.frozen);
  EXPECT_NE(moved.why.find("advancing without Tick"), std::string::npos) << moved.why;
  EXPECT_FALSE(sm::judge_frozen({{0.0, 0}, {0.1, 0}, {0.2, 0}}, 0.3).frozen);  // too short
  EXPECT_FALSE(sm::judge_frozen({{0.0, 0}, {0.5, 0}}, 0.3).frozen);            // too few
  EXPECT_FALSE(sm::judge_frozen({}, 0.3).frozen);
}

TEST(SyncMode, RealtimeNeedsWallRateAndEpochTimestamps)
{
  const sm::RealtimeCheck ok =
    sm::judge_realtime(samples(1.0, 200, kEpochUs, 1.0), kEpochUs / 1e6 + 1.0, 0.8);
  EXPECT_TRUE(ok.ok) << ok.why;
  EXPECT_NEAR(ok.rate, 1.0, 0.03);
  EXPECT_TRUE(ok.epoch);
  EXPECT_NEAR(ok.offset_s, 0.0, 0.05);

  // 여전히 동기 모드: timestamp 가 멈춰 있다
  std::vector<sm::TimeSample> frozen;
  for (int i = 0; i < 50; ++i) {
    frozen.push_back({i * 0.02, 123450000});
  }
  const sm::RealtimeCheck f = sm::judge_realtime(frozen, 0.0, 0.8);
  EXPECT_FALSE(f.ok);
  EXPECT_NE(f.why.find("not advancing"), std::string::npos) << f.why;

  // 흐르지만 sim 시간(epoch 아님): 동기 모드 진입 뒤 시간
  const sm::RealtimeCheck sim = sm::judge_realtime(samples(1.0, 200, 5000000, 1.0), 0.0, 0.8);
  EXPECT_FALSE(sim.ok);
  EXPECT_FALSE(sim.epoch);
  EXPECT_NE(sim.why.find("epoch"), std::string::npos) << sim.why;

  // 너무 느리거나 빠르다 (TIME_MANAGEMENT 배속 등)
  EXPECT_FALSE(sm::judge_realtime(samples(1.0, 200, kEpochUs, 0.3), 0.0, 0.8).ok);
  EXPECT_FALSE(sm::judge_realtime(samples(1.0, 200, kEpochUs, 3.0), 0.0, 0.8).ok);
  // 창이 짧다 / 표본이 없다 (읽기 실패)
  EXPECT_FALSE(sm::judge_realtime(samples(0.3, 50, kEpochUs, 1.0), 0.0, 0.8).ok);
  const sm::RealtimeCheck none = sm::judge_realtime({}, 0.0, 0.8);
  EXPECT_FALSE(none.ok);
  EXPECT_NE(none.why.find("no timestamp"), std::string::npos);
}

TEST(SyncRestore, ExitCodesFollowTheFirstFailingCheck)
{
  sm::RestoreOutcome o;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreUnreachable);
  o.reachable = true;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreStillSync);
  o.unspecified = true;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreNotAdvancing);
  o.realtime_ok = true;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreOk);
  o.keyboard_requested = true;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreModeFailed);
  o.keyboard_ok = true;
  EXPECT_EQ(sm::restore_exit_code(o), sm::kRestoreOk);
  // 문서화된 값 고정 (README "morai_sync_restore")
  EXPECT_EQ(sm::kRestoreOk, 0);
  EXPECT_EQ(sm::kRestoreUsage, 1);
  EXPECT_EQ(sm::kRestoreUnreachable, 2);
  EXPECT_EQ(sm::kRestoreStillSync, 3);
  EXPECT_EQ(sm::kRestoreNotAdvancing, 4);
  EXPECT_EQ(sm::kRestoreModeFailed, 5);
}

bool parse(const std::vector<std::string> & a, sm::RestoreOptions * o, std::string * err = nullptr)
{
  std::string e;
  const bool ok = sm::parse_restore_args(a, o, &e);
  if (err) {
    *err = e;
  }
  return ok;
}

TEST(SyncRestore, NoArmingFlagAndSaneDefaults)
{
  sm::RestoreOptions o;
  ASSERT_TRUE(parse({}, &o));
  EXPECT_EQ(o.host, "127.0.0.1");
  EXPECT_EQ(o.port, 7789);
  EXPECT_EQ(o.ego, "Ego");
  EXPECT_FALSE(o.keyboard);
  EXPECT_DOUBLE_EQ(o.timeout_s, 10.0);
  EXPECT_DOUBLE_EQ(o.verify_s, 1.0);
  EXPECT_EQ(o.deadline_ms, 500);
}

TEST(SyncRestore, AcceptsValidValues)
{
  sm::RestoreOptions o;
  ASSERT_TRUE(parse(
    {"--host", "10.10.10.1", "--port=17850", "--ego", "Ego2", "--client-key", "k", "--keyboard",
     "--timeout-s", "20", "--verify-s", "0.5", "--deadline-ms", "250", "--connect-timeout-ms",
     "100"},
    &o));
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_EQ(o.port, 17850);
  EXPECT_EQ(o.ego, "Ego2");
  EXPECT_EQ(o.client_key, "k");
  EXPECT_TRUE(o.keyboard);
  EXPECT_DOUBLE_EQ(o.timeout_s, 20.0);
  EXPECT_DOUBLE_EQ(o.verify_s, 0.5);
  EXPECT_EQ(o.deadline_ms, 250);
  EXPECT_EQ(o.connect_timeout_ms, 100);
}

TEST(SyncRestore, RejectsBadValues)
{
  const std::vector<std::vector<std::string>> bad = {
    {"--timeout-s", "0"},     {"--timeout-s", "nan"},  {"--timeout-s", "121"},
    {"--verify-s", "0.1"},    {"--verify-s", "11"},    {"--deadline-ms", "5"},
    {"--deadline-ms", "1.5"}, {"--port", "0"},         {"--port", "70000"},
    {"--host", ""},           {"--ego", ""},           {"--connect-timeout-ms", "0"}};
  for (const auto & b : bad) {
    sm::RestoreOptions o;
    std::string err;
    EXPECT_FALSE(parse(b, &o, &err)) << b[0] << " " << b[1];
    EXPECT_NE(err.find(b[0]), std::string::npos) << err;
  }
  sm::RestoreOptions o;
  EXPECT_FALSE(parse({"--bogus"}, &o));
  EXPECT_FALSE(parse({"now"}, &o));
  EXPECT_FALSE(parse({"--keyboard=1"}, &o));
  EXPECT_FALSE(parse({"--timeout-s"}, &o));
  EXPECT_TRUE(parse({"--help"}, &o));
  EXPECT_TRUE(o.help);
}

TEST(SyncRestore, EnvDefaults)
{
  sm::RestoreOptions o;
  std::string err;
  EXPECT_TRUE(sm::apply_restore_env_defaults("10.10.10.1", "7790", &o, &err));
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_EQ(o.port, 7790);
  EXPECT_FALSE(sm::apply_restore_env_defaults(nullptr, "x", &o, &err));
  EXPECT_TRUE(sm::apply_restore_env_defaults(nullptr, nullptr, &o, &err));
}

}  // namespace
