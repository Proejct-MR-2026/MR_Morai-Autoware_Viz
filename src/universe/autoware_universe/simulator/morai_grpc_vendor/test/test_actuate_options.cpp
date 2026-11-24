// 신규 — 26.09.25 _ YSH _ morai_actuate 옵션 파서 gtest (확인 플래그 없으면 거부, 잘못된 값 거부)
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

#include "actuate_options.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
namespace act = morai_grpc_vendor::actuate;

const std::string kFlag = act::kConfirmFlag;

bool parse(std::vector<std::string> args, act::Options * o, std::string * err = nullptr)
{
  std::string e;
  const bool ok = act::parse_args(args, o, &e);
  if (err) {
    *err = e;
  }
  return ok;
}

TEST(ActuateOptions, RefusesWithoutTheConfirmationFlag)
{
  act::Options o;
  std::string err;
  EXPECT_FALSE(parse({}, &o, &err));
  EXPECT_NE(err.find(kFlag), std::string::npos) << err;
  EXPECT_FALSE(parse({"--host", "127.0.0.1", "--port", "17789", "--no-countdown"}, &o, &err));
  EXPECT_NE(err.find("refusing"), std::string::npos) << err;
  // a near miss is not the flag
  EXPECT_FALSE(parse({"--i-understand"}, &o, &err));
  EXPECT_FALSE(parse({kFlag + "=yes"}, &o, &err));
}

TEST(ActuateOptions, HelpNeedsNoFlag)
{
  act::Options o;
  EXPECT_TRUE(parse({"--help"}, &o));
  EXPECT_TRUE(o.help);
}

TEST(ActuateOptions, DefaultsMatchTheSpec)
{
  act::Options o;
  ASSERT_TRUE(parse({kFlag}, &o));
  EXPECT_TRUE(o.confirmed);
  EXPECT_DOUBLE_EQ(o.accel, 0.8);
  EXPECT_DOUBLE_EQ(o.drive_s, 2.0);
  EXPECT_DOUBLE_EQ(o.settle_s, 1.0);
  EXPECT_EQ(o.repeats, 2);
  EXPECT_DOUBLE_EQ(o.max_speed, 3.0);
  EXPECT_DOUBLE_EQ(o.max_distance, 15.0);
  EXPECT_TRUE(o.countdown);
  EXPECT_EQ(o.port, 7789);
  EXPECT_EQ(o.ego, "Ego");
  const std::vector<std::string> def = {"locked1", "locked_half", "burst4",
                                        "free33",  "free100",     "free200"};
  EXPECT_EQ(o.patterns, def);
  EXPECT_DOUBLE_EQ(act::Safety::kSafeStopAccel, -3.0);
  EXPECT_GE(act::Safety::kSafeStopHz, 50.0);
  EXPECT_DOUBLE_EQ(act::Safety::kStopSpeed, 0.05);
  EXPECT_DOUBLE_EQ(act::Safety::kStopHoldS, 1.0);
  EXPECT_DOUBLE_EQ(act::Safety::kStopTimeoutS, 10.0);
  EXPECT_DOUBLE_EQ(act::Safety::kMaxStaleS, 0.5);
  EXPECT_GT(act::Safety::kPreDriveHoldS, 0.0);
  EXPECT_LT(act::Safety::kRestoreStopAfterFailureS, act::Safety::kStopTimeoutS);
}

TEST(ActuateOptions, AcceptsValidValues)
{
  act::Options o;
  ASSERT_TRUE(parse(
    {"--host",        "10.10.10.1",  "--port",           "17789",          kFlag,
     "--accel=1.2",   "--drive-s",   "3",                "--settle-s",     "0",
     "--repeats",     "3",           "--patterns",       "free200,locked1", "--max-speed",
     "2.5",           "--max-distance=20", "--max-total-distance", "100", "--physics-hz",
     "200",           "--reader-hz", "1000",             "--deadline-ms",  "50",
     "--wft-deadline-ms", "300",     "--max-inflight",   "4",              "--idle-probe-s",
     "0",             "--idle-brake", "0.004",           "--no-countdown", "--json",
     "/tmp/x.json"},
    &o));
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_EQ(o.port, 17789);
  EXPECT_DOUBLE_EQ(o.accel, 1.2);
  EXPECT_DOUBLE_EQ(o.drive_s, 3.0);
  EXPECT_DOUBLE_EQ(o.settle_s, 0.0);
  EXPECT_EQ(o.repeats, 3);
  EXPECT_EQ(o.patterns, (std::vector<std::string>{"free200", "locked1"}));
  EXPECT_DOUBLE_EQ(o.max_speed, 2.5);
  EXPECT_DOUBLE_EQ(o.max_distance, 20.0);
  EXPECT_DOUBLE_EQ(o.max_total_distance, 100.0);
  EXPECT_DOUBLE_EQ(o.physics_hz, 200.0);
  EXPECT_DOUBLE_EQ(o.reader_hz, 1000.0);
  EXPECT_EQ(o.deadline_ms, 50);
  EXPECT_EQ(o.wft_deadline_ms, 300);
  EXPECT_EQ(o.max_inflight, 4);
  EXPECT_DOUBLE_EQ(o.idle_probe_s, 0.0);
  EXPECT_DOUBLE_EQ(o.idle_brake, 0.004);
  EXPECT_FALSE(o.countdown);
  EXPECT_EQ(o.json_path, "/tmp/x.json");
}

TEST(ActuateOptions, RejectsBadNumbers)
{
  const std::vector<std::vector<std::string>> bad = {
    {"--accel", "nan"},         {"--accel", "inf"},          {"--accel", "-1"},
    {"--accel", "0"},           {"--accel", "2.5"},          {"--accel", "abc"},
    {"--accel", "0.8x"},        {"--accel", ""},             {"--accel", " 0.8"},
    {"--drive-s", "0.5"},       {"--drive-s", "11"},         {"--settle-s", "-1"},
    {"--max-speed", "0"},       {"--max-speed", "-1"},       {"--max-speed", "11"},
    {"--max-distance", "0"},    {"--max-distance", "1e9"},   {"--max-total-distance", "0"},
    {"--repeats", "0"},         {"--repeats", "1.5"},        {"--repeats", "-1"},
    {"--repeats", "21"},        {"--port", "0"},             {"--port", "70000"},
    {"--deadline-ms", "0"},     {"--wft-deadline-ms", "5"},  {"--max-inflight", "0"},
    {"--max-inflight", "65"},   {"--physics-hz", "0.5"},     {"--physics-hz", "-60"},
    {"--reader-hz", "5"},       {"--idle-brake", "-0.1"},    {"--idle-probe-s", "6"},
    {"--connect-timeout-ms", "0"}};
  for (const auto & b : bad) {
    act::Options o;
    std::vector<std::string> args = b;
    args.push_back(kFlag);
    std::string err;
    EXPECT_FALSE(parse(args, &o, &err)) << b[0] << " " << b[1];
    EXPECT_NE(err.find(b[0]), std::string::npos) << err;
  }
}

TEST(ActuateOptions, OrderAndOriginalModeOverride)
{
  act::Options o;
  ASSERT_TRUE(parse({kFlag}, &o));
  EXPECT_EQ(o.order, "abba");
  EXPECT_FALSE(o.allow_any_mode);  // AUTO etc. refused unless asked
  ASSERT_TRUE(parse({kFlag, "--order", "fixed", "--allow-any-original-mode"}, &o));
  EXPECT_EQ(o.order, "fixed");
  EXPECT_TRUE(o.allow_any_mode);
  act::Options b;
  EXPECT_FALSE(parse({kFlag, "--order", "random"}, &b));
  EXPECT_FALSE(parse({kFlag, "--order", ""}, &b));
  EXPECT_FALSE(parse({kFlag, "--allow-any-original-mode=1"}, &b));
}

TEST(ActuateOptions, RunOrderCounterbalancesDrift)
{
  act::Options o;
  o.patterns = {"a", "b", "c"};
  o.repeats = 3;
  EXPECT_EQ(
    act::run_order(o),
    (std::vector<std::string>{"a", "b", "c", "c", "b", "a", "a", "b", "c"}));
  o.order = "fixed";
  EXPECT_EQ(
    act::run_order(o),
    (std::vector<std::string>{"a", "b", "c", "a", "b", "c", "a", "b", "c"}));
  o.repeats = 1;
  o.order = "abba";
  EXPECT_EQ(act::run_order(o), (std::vector<std::string>{"a", "b", "c"}));
}

TEST(ActuateOptions, RejectsBadPatternLists)
{
  for (const char * p : {"", "locked1,bogus", "locked1,locked1", "locked1,", ",free33",
                         "LOCKED1"}) {
    act::Options o;
    EXPECT_FALSE(parse({kFlag, "--patterns", std::string(p)}, &o)) << "'" << p << "'";
  }
}

TEST(ActuateOptions, RejectsUnknownAndStrayArguments)
{
  act::Options o;
  EXPECT_FALSE(parse({kFlag, "--bogus"}, &o));
  EXPECT_FALSE(parse({kFlag, "all"}, &o));
  EXPECT_FALSE(parse({kFlag, "--accel"}, &o));  // missing value
  EXPECT_FALSE(parse({kFlag, "--no-countdown=1"}, &o));
}

TEST(ActuateOptions, EnvDefaults)
{
  act::Options o;
  std::string err;
  EXPECT_TRUE(act::apply_env_defaults("10.10.10.1", "7790", &o, &err));
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_EQ(o.port, 7790);
  EXPECT_FALSE(act::apply_env_defaults(nullptr, "77x", &o, &err));
  EXPECT_TRUE(act::apply_env_defaults(nullptr, nullptr, &o, &err));
}

TEST(ActuatePatterns, TableMatchesTheSpec)
{
  const auto & k = act::known_patterns();
  ASSERT_EQ(k.size(), 6u);
  const act::PatternSpec * l1 = act::find_pattern("locked1");
  const act::PatternSpec * lh = act::find_pattern("locked_half");
  const act::PatternSpec * b4 = act::find_pattern("burst4");
  const act::PatternSpec * f33 = act::find_pattern("free33");
  const act::PatternSpec * f100 = act::find_pattern("free100");
  const act::PatternSpec * f200 = act::find_pattern("free200");
  ASSERT_TRUE(l1 && lh && b4 && f33 && f100 && f200);
  EXPECT_TRUE(l1->locked);
  EXPECT_EQ(l1->every_n, 1);
  EXPECT_EQ(l1->burst, 1);
  EXPECT_EQ(lh->every_n, 2);
  EXPECT_EQ(b4->burst, 4);
  EXPECT_FALSE(f33->locked);
  EXPECT_NEAR(f33->hz, 33.33, 0.01);
  EXPECT_DOUBLE_EQ(f100->hz, 100.0);
  EXPECT_FALSE(f100->pipelined);
  EXPECT_DOUBLE_EQ(f200->hz, 200.0);
  EXPECT_TRUE(f200->pipelined);
  EXPECT_EQ(act::find_pattern("nope"), nullptr);
}

// =============================================================================
// 26.09.25 _ YSH _ --lockstep 옵션: 확인 플래그와 둘 다 필요, tick_period 1..100, 모드별 패턴·전용 옵션
// =============================================================================

TEST(ActuateLockstepOptions, NeedsTheConfirmationFlagToo)
{
  act::Options o;
  std::string err;
  EXPECT_FALSE(parse({"--lockstep"}, &o, &err));
  EXPECT_NE(err.find(kFlag), std::string::npos) << err;
  EXPECT_FALSE(parse({"--lockstep", "--tick-period-ms", "10", "--no-countdown"}, &o, &err));
  EXPECT_NE(err.find("refusing"), std::string::npos) << err;
  EXPECT_FALSE(parse({kFlag + "", "--lockstep=1"}, &o, &err));  // a flag takes no value
}

TEST(ActuateLockstepOptions, Defaults)
{
  act::Options o;
  ASSERT_TRUE(parse({kFlag, "--lockstep"}, &o));
  EXPECT_TRUE(o.lockstep);
  EXPECT_EQ(o.tick_period_ms, 10);
  EXPECT_EQ(o.tick_deadline_ms, 1000);
  EXPECT_EQ(
    o.patterns,
    (std::vector<std::string>{"sync1", "sync_drop2", "sync_tick3", "sync_tick3x"}));
  EXPECT_EQ(o.repeats, 2);
  EXPECT_EQ(o.order, "abba");
  EXPECT_DOUBLE_EQ(o.accel, 0.8);
  EXPECT_DOUBLE_EQ(o.drive_s, 2.0);
  // 실시간 모드 기본값은 그대로
  act::Options rt;
  ASSERT_TRUE(parse({kFlag}, &rt));
  EXPECT_FALSE(rt.lockstep);
  EXPECT_EQ(rt.patterns.size(), 6u);
}

TEST(ActuateLockstepOptions, TickPeriodIsValidated)
{
  for (const char * ok : {"1", "10", "20", "100"}) {
    act::Options o;
    ASSERT_TRUE(parse({kFlag, "--lockstep", "--tick-period-ms", ok}, &o)) << ok;
    EXPECT_EQ(o.tick_period_ms, std::stoi(ok));
  }
  act::Options o;
  ASSERT_TRUE(parse({kFlag, "--lockstep", "--tick-period-ms=5", "--tick-deadline-ms", "250"}, &o));
  EXPECT_EQ(o.tick_period_ms, 5);
  EXPECT_EQ(o.tick_deadline_ms, 250);
  // 0 은 서버가 10 으로, 1..100 밖은 20 으로 바꾼다 — 조용히 다른 주기로 도는 대신 거부
  for (const char * bad : {"0", "101", "-1", "1.5", "abc", "", " 10", "1e1", "1000"}) {
    act::Options b;
    std::string err;
    EXPECT_FALSE(parse({kFlag, "--lockstep", "--tick-period-ms", bad}, &b, &err)) << bad;
    EXPECT_NE(err.find("--tick-period-ms"), std::string::npos) << err;
  }
  for (const char * bad : {"0", "49", "10001", "x"}) {
    act::Options b;
    std::string err;
    EXPECT_FALSE(parse({kFlag, "--lockstep", "--tick-deadline-ms", bad}, &b, &err)) << bad;
    EXPECT_NE(err.find("--tick-deadline-ms"), std::string::npos) << err;
  }
}

TEST(ActuateLockstepOptions, LockstepOnlyOptionsNeedLockstep)
{
  for (const char * opt : {"--tick-period-ms", "--tick-deadline-ms"}) {
    act::Options o;
    std::string err;
    EXPECT_FALSE(parse({kFlag, opt, "100"}, &o, &err)) << opt;
    EXPECT_NE(err.find("--lockstep"), std::string::npos) << err;
  }
}

TEST(ActuateLockstepOptions, RealTimeOnlyOptionsAreRefused)
{
  for (const auto & a : std::vector<std::vector<std::string>>{
         {"--physics-hz", "60"},
         {"--reader-hz", "1000"},
         {"--wft-deadline-ms", "300"},
         {"--max-inflight", "4"}}) {
    act::Options o;
    std::string err;
    std::vector<std::string> args = {kFlag, "--lockstep"};
    args.insert(args.end(), a.begin(), a.end());
    EXPECT_FALSE(parse(args, &o, &err)) << a[0];
    EXPECT_NE(err.find(a[0]), std::string::npos) << err;
    EXPECT_NE(err.find("--lockstep"), std::string::npos) << err;
  }
}

TEST(ActuateLockstepOptions, PatternsAreModeSpecific)
{
  act::Options o;
  std::string err;
  EXPECT_FALSE(parse({kFlag, "--patterns", "sync1"}, &o, &err));
  EXPECT_NE(err.find("--patterns"), std::string::npos) << err;
  EXPECT_NE(err.find("--lockstep"), std::string::npos) << err;
  EXPECT_FALSE(parse({kFlag, "--lockstep", "--patterns", "locked1"}, &o, &err));
  EXPECT_NE(err.find("--patterns"), std::string::npos) << err;
  EXPECT_FALSE(parse({kFlag, "--lockstep", "--patterns", "sync1,locked1"}, &o, &err));
  EXPECT_FALSE(parse({kFlag, "--lockstep", "--patterns", "sync1,sync1"}, &o, &err));
  EXPECT_FALSE(parse({kFlag, "--lockstep", "--patterns", "sync2"}, &o, &err));
  // 순서와 무관 (--patterns 가 --lockstep 앞에 와도)
  act::Options a;
  ASSERT_TRUE(parse({"--patterns", "sync_tick3,sync1", kFlag, "--lockstep"}, &a));
  EXPECT_EQ(a.patterns, (std::vector<std::string>{"sync_tick3", "sync1"}));
  a.repeats = 2;
  EXPECT_EQ(
    act::run_order(a), (std::vector<std::string>{"sync_tick3", "sync1", "sync1", "sync_tick3"}));
}

TEST(ActuateLockstepPatterns, TableMatchesTheSpec)
{
  const auto & k = act::known_lockstep_patterns();
  ASSERT_EQ(k.size(), 4u);
  const act::LockstepPatternSpec * s1 = act::find_lockstep_pattern("sync1");
  const act::LockstepPatternSpec * d2 = act::find_lockstep_pattern("sync_drop2");
  const act::LockstepPatternSpec * t3 = act::find_lockstep_pattern("sync_tick3");
  const act::LockstepPatternSpec * t3x = act::find_lockstep_pattern("sync_tick3x");
  ASSERT_TRUE(s1 && d2 && t3 && t3x);
  EXPECT_EQ(s1->cmd_every, 1);  // 매 스텝 명령 → Tick(1)
  EXPECT_EQ(s1->frames, 1);
  EXPECT_EQ(d2->cmd_every, 2);  // 짝수 스텝만 명령
  EXPECT_EQ(d2->frames, 1);
  EXPECT_EQ(t3->cmd_every, 1);  // 명령 한 번 → Tick(3)
  EXPECT_EQ(t3->frames, 3);
  EXPECT_EQ(t3x->cmd_every, 3);  // 명령 한 번 → Tick(1) x3
  EXPECT_EQ(t3x->frames, 1);
  EXPECT_EQ(act::find_lockstep_pattern("locked1"), nullptr);
  EXPECT_EQ(act::find_pattern("sync1"), nullptr);  // 두 표는 섞이지 않는다
  EXPECT_EQ(act::default_lockstep_patterns().size(), 4u);
}

}  // namespace
