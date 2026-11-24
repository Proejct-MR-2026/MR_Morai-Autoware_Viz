// 신규 — 26.09.24 _ YSH _ morai_probe 옵션 파서 gtest (기본 측정 창 길이, 잘못된 값 거부)
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

#include "probe_options.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{
using morai_grpc_vendor::probe::apply_env_defaults;
using morai_grpc_vendor::probe::Options;
using morai_grpc_vendor::probe::parse_args;

bool parse(const std::vector<std::string> & args, Options * o, std::string * err = nullptr)
{
  std::string e;
  const bool ok = parse_args(args, o, &e);
  if (err) {
    *err = e;
  }
  return ok;
}

TEST(ProbeOptions, DefaultWindowsAreLongEnoughForTheTail)
{
  // Stalls were seen roughly every 20 s: a default run must cover several of them.
  Options o;
  ASSERT_TRUE(parse({}, &o));
  EXPECT_EQ(o.command, "all");
  EXPECT_GE(o.rtt_seconds, 60.0);
  EXPECT_GE(o.seconds, 60.0);
  EXPECT_EQ(o.n, 0u);  // rtt is bounded by time, not by a call count
  EXPECT_DOUBLE_EQ(o.rate_hz, 200.0);
  EXPECT_DOUBLE_EQ(o.poll_hz, 0.0);
  EXPECT_GE(o.max_raw_rows, 200000u);
}

TEST(ProbeOptions, AcceptsValidValues)
{
  Options o;
  ASSERT_TRUE(parse(
    {"frames",     "--seconds",
     "120",        "--rtt-seconds=90.5",
     "--n",        "5000",
     "--wft-n=10", "--rate",
     "0",          "--poll-hz",
     "1000",       "--deadline-ms",
     "500",        "--connect-timeout-ms",
     "100",        "--max-raw-rows",
     "1000",       "--port",
     "17789",      "--host",
     "10.10.10.1", "--no-raw"},
    &o));
  EXPECT_EQ(o.command, "frames");
  EXPECT_DOUBLE_EQ(o.seconds, 120.0);
  EXPECT_DOUBLE_EQ(o.rtt_seconds, 90.5);
  EXPECT_EQ(o.n, 5000u);
  EXPECT_EQ(o.wft_n, 10u);
  EXPECT_DOUBLE_EQ(o.rate_hz, 0.0);
  EXPECT_DOUBLE_EQ(o.poll_hz, 1000.0);
  EXPECT_EQ(o.deadline_ms, 500);
  EXPECT_EQ(o.connect_timeout_ms, 100);
  EXPECT_EQ(o.max_raw_rows, 1000u);
  EXPECT_EQ(o.port, 17789);
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_FALSE(o.raw);
}

TEST(ProbeOptions, RejectsValuesThatWouldOverflowOrAbort)
{
  // Each of these used to pass and then abort (length_error), overflow a duration_cast or
  // an int conversion, or silently record nothing.
  const std::vector<std::vector<std::string>> bad = {
    {"--n", "-1"},
    {"--wft-n", "-1"},
    {"--n", "0"},
    {"--n", "12abc"},
    {"--n", "1e3"},
    {"--n", "99999999999"},
    {"--wft-n", " 5"},
    {"--seconds", "inf"},
    {"--seconds", "nan"},
    {"--seconds", "1e10"},
    {"--seconds", "0"},
    {"--seconds", "-3"},
    {"--seconds", "10s"},
    {"--rtt-seconds", "inf"},
    {"--rtt-seconds", "3601"},
    {"--rate", "nan"},
    {"--rate", "-1"},
    {"--rate", "1e-300"},
    {"--rate", "0.05"},
    {"--rate", "1e6"},
    {"--poll-hz", "nan"},
    {"--poll-hz", "inf"},
    {"--deadline-ms", "0"},
    {"--deadline-ms", "-5"},
    {"--deadline-ms", "1e3"},
    {"--connect-timeout-ms", "0"},
    {"--port", "0"},
    {"--port", "70000"},
    {"--port", "80x"},
    {"--max-raw-rows", "0"},
    {"--max-raw-rows", "-1"},
    {"--seconds"},
    {"--bogus"},
    {"rtt", "frames"},
    {"nosuch"}};
  for (const auto & args : bad) {
    Options o;
    std::string err;
    std::string joined;
    for (const auto & a : args) {
      joined += a + " ";
    }
    EXPECT_FALSE(parse(args, &o, &err)) << "accepted: " << joined;
    EXPECT_FALSE(err.empty()) << joined;
  }
}

TEST(ProbeOptions, HelpShortCircuits)
{
  Options o;
  ASSERT_TRUE(parse({"--n", "5", "-h", "--n", "-1"}, &o));
  EXPECT_TRUE(o.help);
}

TEST(ProbeOptions, EnvironmentDefaults)
{
  Options o;
  std::string err;
  EXPECT_TRUE(apply_env_defaults("10.10.10.1", "7790", &o, &err));
  EXPECT_EQ(o.host, "10.10.10.1");
  EXPECT_EQ(o.port, 7790);

  Options d;
  EXPECT_TRUE(apply_env_defaults(nullptr, "", &d, &err));
  EXPECT_EQ(d.host, "127.0.0.1");
  EXPECT_EQ(d.port, 7789);

  Options b;
  EXPECT_FALSE(apply_env_defaults(nullptr, "port", &b, &err));
  EXPECT_FALSE(err.empty());
}

// 26.09.25 _ YSH _ monitor 서브커맨드 옵션 — 기본값(Ctrl-C 까지·1000 Hz), 명시값 우선, monitor 전용 옵션 거부
TEST(ProbeOptions, MonitorDefaultsRunUntilCtrlCAt1kHz)
{
  Options o;
  ASSERT_TRUE(parse({"monitor"}, &o));
  EXPECT_EQ(o.command, "monitor");
  EXPECT_DOUBLE_EQ(o.seconds, 0.0);  // 0 = Ctrl-C 까지
  EXPECT_DOUBLE_EQ(o.poll_hz, 1000.0);
  EXPECT_EQ(o.trigger, "poll");
  EXPECT_TRUE(o.csv_path.empty());

  // 다른 서브커맨드 기본값은 그대로
  Options f;
  ASSERT_TRUE(parse({"frames"}, &f));
  EXPECT_DOUBLE_EQ(f.seconds, 60.0);
  EXPECT_DOUBLE_EQ(f.poll_hz, 0.0);
}

TEST(ProbeOptions, MonitorExplicitValuesWin)
{
  Options o;
  ASSERT_TRUE(parse(
    {"--seconds", "7200", "monitor", "--poll-hz", "0", "--csv", "/tmp/m.csv", "--trigger=wft"},
    &o));
  EXPECT_DOUBLE_EQ(o.seconds, 7200.0);  // monitor 는 3600 초과 허용
  EXPECT_DOUBLE_EQ(o.poll_hz, 0.0);     // 명시한 tight loop 가 기본 1000 Hz 를 이긴다
  EXPECT_EQ(o.csv_path, "/tmp/m.csv");
  EXPECT_EQ(o.trigger, "wft");

  Options z;
  ASSERT_TRUE(parse({"monitor", "--seconds", "0"}, &z));
  EXPECT_DOUBLE_EQ(z.seconds, 0.0);
}

TEST(ProbeOptions, MonitorOnlyOptionsAreRejectedElsewhere)
{
  const std::vector<std::vector<std::string>> bad = {
    {"frames", "--seconds", "0"},     // 0 = Ctrl-C 까지는 monitor 만
    {"frames", "--seconds", "4000"},  // 3600 초과는 monitor 만
    {"frames", "--csv", "/tmp/x.csv"},
    {"rtt", "--trigger", "wft"},
    {"--trigger", "poll2", "monitor"},
    {"monitor", "--trigger", "tick"},
    {"monitor", "--seconds", "-1"},
    {"monitor", "--seconds", "90000"},
    {"monitor", "--seconds", "nan"},
    {"monitor", "--csv"},
    {"monitor", "--csv", ""}};
  for (const auto & args : bad) {
    Options o;
    std::string err;
    std::string joined;
    for (const auto & a : args) {
      joined += a + " ";
    }
    EXPECT_FALSE(parse(args, &o, &err)) << "accepted: " << joined;
    EXPECT_FALSE(err.empty()) << joined;
  }
}

}  // namespace
