// 신규 — 26.09.25 _ YSH _ morai_actuate --lockstep 순수 계산부 gtest (합성 동기 모드 MORAI, 네트워크 없음)
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
#include "lockstep_analysis.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <random>
#include <string>
#include <vector>

namespace
{
namespace act = morai_grpc_vendor::actuate;
namespace ls = morai_grpc_vendor::actuate::lockstep;

// =============================================================================
// Synthetic synchronous-mode MORAI: a queued command is applied at the start of the next
// Tick (optionally one Tick late); what happens on an uncommanded Tick (hold / idle) and on
// frames 2..n of a Tick(n) (persist / first frame only) is configurable — exactly the two
// unknowns the lockstep patterns measure. `tau_s` adds a first-order actuator / powertrain
// lag between the input and the realised acceleration (1 ms sub-steps).
// =============================================================================

struct Sim
{
  int period_ms{10};
  double eff{0.9};             ///< realised / commanded acceleration
  double idle_accel{-0.05};    ///< m/s^2 without input (idle record, rolling resistance)
  bool hold{false};            ///< an uncommanded Tick keeps the last input
  bool persist{false};         ///< frames 2..n of a Tick(n) keep the input
  int lag{0};                  ///< the command acts this many Ticks late
  bool ignore{false};          ///< commands silently dropped (wrong control mode)
  double noise{0.0};           ///< reported speed noise, m/s
  double tick_wall_ms{16.5};   ///< per frame
  double tau_s{0.0};           ///< first-order lag of the realised acceleration (0 = none)
};

struct Pattern
{
  const char * name;
  int cmd_every;
  int frames;
};

std::vector<ls::Step> simulate(
  const Sim & sim, const Pattern & p, double accel, double drive_s, std::mt19937 * rng,
  bool reject_some = false)
{
  std::normal_distribution<double> nz(0.0, 1.0);
  std::vector<ls::Step> out;
  ls::Step ref;
  ref.tick_ok = ref.read_ok = true;
  ref.frame_count = 100;
  ref.ts_us = 1000000;
  ref.tick_count = 100;
  ref.brake = 0.6;  // 브레이크 유지 뒤
  ref.wall_ms = 5000.0;
  out.push_back(ref);
  double v = 0.0;
  double a_real = 0.0;  // 지연된 실제 가속도
  bool input_valid = false;
  double input = 0.0;
  std::deque<bool> queue;  // commanded per Tick, for the lag
  uint64_t frame = ref.frame_count;
  int64_t ts = ref.ts_us;
  double wall = ref.wall_ms;
  const double dt = sim.period_ms / 1000.0;
  for (size_t i = 0; static_cast<double>(ts - ref.ts_us) / 1e6 < drive_s; ++i) {
    ls::Step s;
    s.frames = p.frames;
    s.commanded = (i % static_cast<size_t>(p.cmd_every)) == 0;
    s.command_ok = s.commanded && !(reject_some && i % 10 == 5);
    s.accel = s.commanded ? accel : 0.0;
    queue.push_back(s.command_ok && !sim.ignore);
    bool fresh = false;
    if (static_cast<int>(queue.size()) > sim.lag) {
      fresh = queue.front();
      queue.pop_front();
    }
    if (fresh) {
      input = accel;
      input_valid = true;
    }
    bool last_driven = false;
    for (int f = 0; f < p.frames; ++f) {
      bool driven = f == 0 ? (fresh || (sim.hold && input_valid)) : false;
      if (f > 0) {
        const bool first = fresh || (sim.hold && input_valid);
        driven = sim.persist ? first : false;
      }
      const double a = driven ? input * sim.eff : sim.idle_accel;
      if (sim.tau_s > 0.0) {
        const int sub = std::max(1, sim.period_ms);
        const double h = dt / sub;
        for (int k = 0; k < sub; ++k) {
          a_real += (a - a_real) * std::min(1.0, h / sim.tau_s);
          v = std::max(0.0, v + a_real * h);
        }
      } else {
        v = std::max(0.0, v + a * dt);
      }
      last_driven = driven;
    }
    if (!sim.hold) {
      input_valid = false;
    }
    frame += static_cast<uint64_t>(p.frames);
    ts += static_cast<int64_t>(p.frames) * sim.period_ms * 1000;
    wall += sim.tick_wall_ms * p.frames + 0.3;
    s.tick_ok = s.read_ok = true;
    s.frame_count = frame;
    s.tick_count = frame;
    s.ts_us = ts;
    s.tick_wall_ms = sim.tick_wall_ms * p.frames;
    s.speed_mps = v + (sim.noise > 0.0 ? sim.noise * nz(*rng) : 0.0);
    s.planar_mps = std::fabs(s.speed_mps);
    s.throttle = last_driven ? std::max(0.0, input / 3.0) : 0.0;
    s.brake = last_driven ? std::max(0.0, -input / 4.5) : 0.003;
    s.wall_ms = wall;
    out.push_back(s);
  }
  return out;
}

const std::vector<Pattern> kPatterns = {
  {"sync1", 1, 1}, {"sync_drop2", 2, 1}, {"sync_tick3", 1, 3}, {"sync_tick3x", 3, 1}};

struct Session
{
  std::vector<ls::RunAnalysis> runs;
  std::vector<ls::PatternSummary> summaries;
  ls::Interpretation interp;
};

Session run_session(const Sim & sim, int repeats = 2, double accel = 0.8, double drive_s = 2.0)
{
  std::mt19937 rng(42);
  Session s;
  s.runs.reserve(kPatterns.size() * static_cast<size_t>(repeats));
  std::vector<ls::Step> log;
  for (int r = 0; r < repeats; ++r) {
    for (const auto & p : kPatterns) {
      ls::RunInput in;
      in.pattern = p.name;
      in.cmd_every = p.cmd_every;
      in.frames = p.frames;
      in.steps = simulate(sim, p, accel, drive_s, &rng);
      // 세션 로그는 이어진다: 이 런의 기준 Tick 은 앞 런 마지막 Tick 의 다음 프레임
      if (!log.empty()) {
        const int64_t dts = log.back().ts_us + sim.period_ms * 1000 - in.steps.front().ts_us;
        const int64_t dfr =
          static_cast<int64_t>(log.back().frame_count) + 1 - static_cast<int64_t>(in.steps.front().frame_count);
        for (auto & st : in.steps) {
          st.ts_us += dts;
          st.frame_count = static_cast<uint64_t>(static_cast<int64_t>(st.frame_count) + dfr);
        }
      }
      in.drive_s = drive_s;
      in.fit_start_s = 0.5;
      in.commanded_accel = accel;
      in.period_ms = sim.period_ms;
      s.runs.push_back(ls::analyze_run(in));
      log.insert(log.end(), in.steps.begin(), in.steps.end());
    }
  }
  for (const auto & p : kPatterns) {
    std::vector<const ls::RunAnalysis *> v;
    for (const auto & r : s.runs) {
      if (r.pattern == p.name) {
        v.push_back(&r);
      }
    }
    s.summaries.push_back(ls::summarize_pattern(p.name, v));
  }
  std::vector<const ls::RunAnalysis *> all;
  for (const auto & r : s.runs) {
    all.push_back(&r);
  }
  s.interp = ls::interpret(s.summaries, accel, false, ls::check_dt(log, sim.period_ms), all);
  return s;
}

const ls::HoldVerdict & hold_of(const ls::Interpretation & i, const std::string & p)
{
  for (const auto & h : i.hold) {
    if (h.pattern == p) {
      return h;
    }
  }
  static ls::HoldVerdict none;
  return none;
}

// =============================================================================
// The four protocol possibilities
// =============================================================================

TEST(LockstepVerdicts, IdleUncommandedAndFirstFrameOnly)
{
  Sim sim;  // hold false, persist false
  const Session s = run_session(sim);
  EXPECT_NEAR(s.interp.sync1_ratio, 0.9, 0.01);
  EXPECT_EQ(hold_of(s.interp, "sync_drop2").verdict, "idle");
  EXPECT_EQ(hold_of(s.interp, "sync_tick3x").verdict, "idle");
  EXPECT_EQ(s.interp.hold_overall, "idle");
  EXPECT_NEAR(hold_of(s.interp, "sync_drop2").share, -0.05 / 0.72, 0.02);
  EXPECT_EQ(s.interp.multi.verdict, "first_frame_only");
  EXPECT_NEAR(s.interp.multi.m, (0.72 - 2 * 0.05) / 3 / 0.72, 0.02);
  EXPECT_NE(s.interp.multi.note.find("behaves like"), std::string::npos) << s.interp.multi.note;
  EXPECT_EQ(s.interp.idle_records.vote, "idle_record");
  EXPECT_NE(s.interp.line.find("idle"), std::string::npos) << s.interp.line;
  EXPECT_NE(s.interp.line.find("only the first frame"), std::string::npos) << s.interp.line;
  EXPECT_EQ(s.interp.dt.violations, 0u);
  EXPECT_GT(s.interp.dt.n, 500u);
}

TEST(LockstepVerdicts, HeldUncommandedAndPersistingInput)
{
  Sim sim;
  sim.hold = true;
  sim.persist = true;
  const Session s = run_session(sim);
  EXPECT_EQ(hold_of(s.interp, "sync_drop2").verdict, "held");
  EXPECT_EQ(hold_of(s.interp, "sync_tick3x").verdict, "held");
  EXPECT_NEAR(hold_of(s.interp, "sync_drop2").share, 1.0, 0.02);
  EXPECT_EQ(s.interp.hold_overall, "held");
  EXPECT_EQ(s.interp.multi.verdict, "persists");
  EXPECT_NEAR(s.interp.multi.m, 1.0, 0.02);
  EXPECT_EQ(s.interp.idle_records.vote, "held_record");
  EXPECT_NE(s.interp.line.find("held"), std::string::npos);
  EXPECT_NE(s.interp.line.find("persists"), std::string::npos);
}

TEST(LockstepVerdicts, IdleUncommandedButPersistingWithinTickN)
{
  Sim sim;
  sim.persist = true;
  const Session s = run_session(sim);
  EXPECT_EQ(s.interp.hold_overall, "idle");
  EXPECT_EQ(s.interp.multi.verdict, "persists");
  // Tick(3) 은 입력을 유지하지만 명령 하나 + Tick(1) x3 은 아니다
  EXPECT_NEAR(s.interp.multi.m_chain, 0.287, 0.03);
  EXPECT_NE(s.interp.multi.note.find("DIFFERS"), std::string::npos) << s.interp.multi.note;
}

TEST(LockstepVerdicts, HeldUncommandedButFirstFrameOnlyWithinTickN)
{
  Sim sim;
  sim.hold = true;
  const Session s = run_session(sim);
  EXPECT_EQ(s.interp.hold_overall, "held");
  EXPECT_EQ(s.interp.multi.verdict, "first_frame_only");
  EXPECT_NE(s.interp.multi.note.find("DIFFERS"), std::string::npos) << s.interp.multi.note;
}

TEST(LockstepVerdicts, LateCommandIsNotMistakenForHold)
{
  Sim sim;
  sim.lag = 1;  // 명령이 한 Tick 늦게 먹는다 (다음 Tick 시작 시 적용되지 않음)
  const Session s = run_session(sim);
  EXPECT_EQ(hold_of(s.interp, "sync_drop2").verdict, "late");
  EXPECT_NE(hold_of(s.interp, "sync_drop2").note.find("one Tick late"), std::string::npos);
  EXPECT_NE(s.interp.hold_overall, "held");
}

TEST(LockstepVerdicts, IgnoredCommandsAreInconclusiveNotIdle)
{
  Sim sim;
  sim.ignore = true;  // 제어 모드가 3/7 이 아니면 MORAI 가 조용히 버린다
  const Session s = run_session(sim);
  EXPECT_EQ(hold_of(s.interp, "sync_drop2").verdict, "inconclusive");
  EXPECT_NE(hold_of(s.interp, "sync_drop2").note.find("no effect"), std::string::npos);
  EXPECT_EQ(s.interp.hold_overall, "unknown");
  EXPECT_NE(s.interp.multi.verdict, "first_frame_only");
  EXPECT_NE(s.interp.multi.verdict, "persists");
}

TEST(LockstepVerdicts, SpeedNoiseNeverFlipsTheVerdict)
{
  // 속도 잡음 σ 가 Tick 당 속도 변화(7 mm/s)보다 크면 판정은 미정이어야지 반대가 되면 안 된다
  for (double noise : {0.005, 0.02, 0.05}) {
    Sim idle;
    idle.noise = noise;
    const Session a = run_session(idle, 2);
    for (const auto & h : a.interp.hold) {
      EXPECT_NE(h.verdict, "held") << noise << " " << h.pattern << " " << h.share << "+-" << h.share_se;
    }
    EXPECT_NE(a.interp.multi.verdict, "persists") << noise;
    Sim held;
    held.noise = noise;
    held.hold = true;
    held.persist = true;
    const Session b = run_session(held, 2);
    for (const auto & h : b.interp.hold) {
      EXPECT_NE(h.verdict, "idle") << noise << " " << h.pattern << " " << h.share << "+-" << h.share_se;
    }
    EXPECT_NE(b.interp.multi.verdict, "first_frame_only") << noise;
  }
  // 작은 잡음에서는 여전히 결정한다
  Sim small;
  small.noise = 0.001;
  EXPECT_EQ(run_session(small).interp.hold_overall, "idle");
}

TEST(LockstepThroughput, SimOverWallAndTickPercentiles)
{
  Sim sim;  // 10 ms per frame, 16.5 ms wall per frame + 0.3 ms read
  const Session s = run_session(sim);
  EXPECT_NEAR(s.interp.speed.rtf, 10.0 / 16.8, 0.02);
  EXPECT_NEAR(s.interp.speed.tick1_ms.p50, 16.5, 1e-9);
  EXPECT_NEAR(s.interp.speed.tickn_ms.p50, 49.5, 1e-9);
  EXPECT_NE(s.interp.line.find("real time"), std::string::npos);
  for (const auto & p : s.summaries) {
    EXPECT_NEAR(p.rtf, p.pattern == "sync_tick3" ? 30.0 / 49.8 : 10.0 / 16.8, 0.02) << p.pattern;
  }
}

TEST(LockstepAnalysis, RunGroupsAndRatio)
{
  Sim sim;
  std::mt19937 rng(1);
  ls::RunInput in;
  in.pattern = "sync_drop2";
  in.cmd_every = 2;
  in.steps = simulate(sim, {"sync_drop2", 2, 1}, 0.8, 2.0, &rng);
  in.commanded_accel = 0.8;
  const ls::RunAnalysis a = ls::analyze_run(in);
  EXPECT_EQ(a.ticks, 200u);
  EXPECT_EQ(a.frames, 200u);
  EXPECT_EQ(a.commanded, 100u);
  EXPECT_NEAR(a.sim_s, 2.0, 1e-9);
  EXPECT_EQ(a.cmd1.n_drive, 100u);
  EXPECT_EQ(a.unc1.n_drive, 100u);
  EXPECT_EQ(a.cmd1.n_fit(), 75u);  // Ticks starting at t >= 0.5 s
  EXPECT_NEAR(a.cmd1.accel(), 0.72, 1e-6);
  EXPECT_NEAR(a.unc1.accel(), -0.05, 1e-6);
  EXPECT_NEAR(a.ratio, (0.72 - 0.05) / 2 / 0.8, 0.01);
  EXPECT_EQ(a.dt.violations, 0u);
  EXPECT_DOUBLE_EQ(a.cmd1.idle_share(), 0.0);
  EXPECT_DOUBLE_EQ(a.unc1.idle_share(), 1.0);
}

TEST(LockstepAnalysis, RejectedCommandsAndFailedTicksAreExcluded)
{
  Sim sim;
  std::mt19937 rng(1);
  ls::RunInput in;
  in.pattern = "sync1";
  in.steps = simulate(sim, {"sync1", 1, 1}, 0.8, 2.0, &rng, true);
  in.steps[50].tick_ok = false;  // Tick RPC 실패
  in.steps[80].read_ok = false;  // 읽기 실패
  in.commanded_accel = 0.8;
  const ls::RunAnalysis a = ls::analyze_run(in);
  EXPECT_EQ(a.command_failed, 20u);
  EXPECT_GE(a.skipped, 4u);  // 실패 Tick 두 개와 그 다음 Tick 들
  EXPECT_EQ(a.cmd1.n_drive + a.command_failed + a.skipped, a.ticks);
  EXPECT_FALSE(a.warnings.empty());
}

TEST(LockstepAnalysis, AJumpToEpochIsExcludedNotTakenAsSimTime)
{
  // 누가 UI 에서 동기 모드를 끄면 마지막 Tick 의 timestamp 가 epoch 로 튄다 — sim 시간·기울기에 넣지 않는다
  Sim sim;
  std::mt19937 rng(1);
  ls::RunInput in;
  in.pattern = "sync1";
  in.steps = simulate(sim, {"sync1", 1, 1}, 0.8, 2.0, &rng);
  in.steps.back().ts_us = 1758729600LL * 1000000LL;
  in.commanded_accel = 0.8;
  const ls::RunAnalysis a = ls::analyze_run(in);
  EXPECT_NEAR(a.sim_s, 1.99, 1e-9);
  EXPECT_GE(a.skipped, 1u);
  EXPECT_NEAR(a.ratio, 0.9, 0.01);
  EXPECT_EQ(a.dt.counts.at("jumped"), 1u);
}

TEST(LockstepAnalysis, SummaryPoolsOnlyValidRuns)
{
  ls::RunAnalysis ok;
  ok.pattern = "sync1";
  ok.fit.n = 100;
  ok.ratio = 0.9;
  ok.sim_s = 2.0;
  ok.wall_s = 3.3;
  ok.cmd1.fit = {{0.0072, 0.01}, {0.0072, 0.01}, {0.0072, 0.01}};
  ls::RunAnalysis aborted = ok;
  aborted.aborted = true;
  aborted.ratio = 0.1;
  ls::RunAnalysis short_run = ok;
  short_run.fit.n = 3;
  short_run.ratio = 0.2;
  const ls::PatternSummary p = ls::summarize_pattern("sync1", {&ok, &aborted, &short_run});
  EXPECT_EQ(p.runs, 3u);
  EXPECT_EQ(p.valid, 1u);
  EXPECT_EQ(p.aborted, 1u);
  EXPECT_DOUBLE_EQ(p.ratio_mean, 0.9);
  EXPECT_EQ(p.cmd1.n_fit(), 3u);
  EXPECT_NEAR(p.rtf, 2.0 / 3.3, 1e-12);
}

// =============================================================================
// Tick classification (a) and Tick health (envelope)
// =============================================================================

ls::Step tick_step(uint64_t frame, int64_t ts, int frames = 1)
{
  ls::Step s;
  s.tick_ok = s.read_ok = true;
  s.frame_count = frame;
  s.ts_us = ts;
  s.frames = frames;
  return s;
}

TEST(LockstepTicks, Classification)
{
  const ls::Step p = tick_step(10, 100000);
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 110000), 10), ls::TickClass::kOk);
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 110001), 10), ls::TickClass::kOk);  // ±1 µs
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 110002), 10), ls::TickClass::kDtMismatch);
  EXPECT_EQ(ls::classify_tick(p, tick_step(13, 130000, 3), 10), ls::TickClass::kOk);
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 130000, 3), 10), ls::TickClass::kFrameMismatch);
  EXPECT_EQ(ls::classify_tick(p, tick_step(10, 100000), 10), ls::TickClass::kNotAdvanced);
  // 바쁨·권한 없음 Tick 이 기본 SyncTimestamp(frame_count 0)를 돌려줘도 sim 시간이 그대로면 진행 안 함
  EXPECT_EQ(ls::classify_tick(p, tick_step(0, 100000), 10), ls::TickClass::kNotAdvanced);
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 100000), 10), ls::TickClass::kNotAdvanced);
  EXPECT_EQ(ls::classify_tick(p, tick_step(10, 110000), 10), ls::TickClass::kFrameMismatch);
  EXPECT_EQ(ls::classify_tick(p, tick_step(12, 120000), 10), ls::TickClass::kFrameMismatch);
  // 누군가 UI 에서 동기 모드를 끄면 timestamp 가 epoch 로 튄다
  EXPECT_EQ(
    ls::classify_tick(p, tick_step(11, 1758729600LL * 1000000LL), 10), ls::TickClass::kJumped);
  EXPECT_EQ(ls::classify_tick(p, tick_step(11, 50000), 10), ls::TickClass::kDtMismatch);
  ls::Step failed = tick_step(11, 110000);
  failed.tick_ok = false;
  EXPECT_EQ(ls::classify_tick(p, failed, 10), ls::TickClass::kFailed);
  ls::Step noread = tick_step(11, 110000);
  noread.read_ok = false;
  EXPECT_EQ(ls::classify_tick(p, noread, 10), ls::TickClass::kNoRead);
  EXPECT_STREQ(ls::tick_class_name(ls::TickClass::kNotAdvanced), "not_advanced");
}

TEST(LockstepTicks, DtReportCountsViolations)
{
  std::vector<ls::Step> v = {
    tick_step(1, 10000), tick_step(2, 20000), tick_step(5, 50000, 3), tick_step(6, 60002),
    tick_step(6, 60002), tick_step(7, 70002)};
  const ls::DtReport r = ls::check_dt(v, 10);
  EXPECT_EQ(r.n, 5u);
  EXPECT_EQ(r.ok, 3u);
  EXPECT_EQ(r.violations, 2u);
  EXPECT_EQ(r.counts.at("dt_mismatch"), 1u);
  EXPECT_EQ(r.counts.at("not_advanced"), 1u);
  ASSERT_EQ(r.examples.size(), 2u);
  EXPECT_NE(r.examples[0].find("dt_mismatch"), std::string::npos) << r.examples[0];
  // 실패한 Tick 뒤의 Tick 은 판정하지 않는다
  v[1].tick_ok = false;
  EXPECT_EQ(ls::check_dt(v, 10).n, 4u);
}

TEST(LockstepTicks, HealthAbortsOnConsecutiveFailuresOrFrozenTicks)
{
  using C = ls::TickClass;
  ls::TickHealth h;
  h.update(C::kFailed);
  h.update(C::kFailed);
  EXPECT_EQ(ls::check_tick_health(h), "");
  for (int i = 0; i < 4; ++i) {
    h.update(C::kOk);  // 복구되면 다시 센다 (창도 비워질 만큼)
  }
  h.update(C::kFailed);
  h.update(C::kFailed);
  EXPECT_EQ(ls::check_tick_health(h), "");
  h.update(C::kFailed);
  EXPECT_NE(ls::check_tick_health(h).find("Tick RPCs failed"), std::string::npos);

  ls::TickHealth f;
  f.update(C::kNotAdvanced);
  f.update(C::kNotAdvanced);
  f.update(C::kDtMismatch);  // 진행은 했다
  f.update(C::kOk);
  f.update(C::kOk);
  f.update(C::kOk);
  f.update(C::kNotAdvanced);
  f.update(C::kNotAdvanced);
  EXPECT_EQ(ls::check_tick_health(f), "");
  f.update(C::kNotAdvanced);
  EXPECT_NE(ls::check_tick_health(f).find("did not advance"), std::string::npos);

  ls::TickHealth j;
  j.update(ls::TickClass::kJumped);
  j.update(ls::TickClass::kOk);  // 한 번 튀면 계속 중단
  EXPECT_NE(ls::check_tick_health(j).find("left synchronous mode"), std::string::npos);
  EXPECT_EQ(ls::Limits::kSafeStopMaxTicks, 2000);
  EXPECT_EQ(ls::Limits::kDtToleranceUs, 1);
}

TEST(LockstepTicks, IntermittentTroubleAbortsThroughTheWindow)
{
  // 실패·성공이 번갈아 오면 연속 카운터는 매번 0 으로 돌아간다 — 창(최근 5 Tick 중 3)이 잡는다
  using C = ls::TickClass;
  ls::TickHealth h;
  h.update(C::kFailed);
  h.update(C::kOk);
  h.update(C::kFailed);
  h.update(C::kOk);
  EXPECT_EQ(ls::check_tick_health(h), "");
  EXPECT_EQ(h.window_bad(), 2u);
  h.update(C::kFailed);
  EXPECT_EQ(h.consecutive_failed, 1u);
  EXPECT_NE(ls::check_tick_health(h).find("3 of the last 5"), std::string::npos)
    << ls::check_tick_health(h);
  ls::TickHealth g;  // 실패와 진행 안 함이 섞여도
  g.update(C::kFailed);
  g.update(C::kNotAdvanced);
  g.update(C::kOk);
  g.update(C::kFailed);
  EXPECT_NE(ls::check_tick_health(g), "");
  ls::TickHealth r;  // 읽기 실패는 창에 넣지 않는다 (벽시계 stale 규칙이 따로 본다)
  for (int i = 0; i < 5; ++i) {
    r.update(C::kNoRead);
  }
  EXPECT_EQ(ls::check_tick_health(r), "");
}

TEST(LockstepTicks, AfterAGapTheLastGoodTickIsTheReference)
{
  const ls::Step good = tick_step(10, 100000);
  // 실패한 Tick 두 개 뒤: timestamp 그대로면 진행 안 함, frames_since 안에서 움직였으면 정상
  EXPECT_EQ(ls::classify_after_gap(good, tick_step(10, 100000), 3, 10), ls::TickClass::kNotAdvanced);
  EXPECT_EQ(ls::classify_after_gap(good, tick_step(0, 100000), 3, 10), ls::TickClass::kNotAdvanced);
  EXPECT_EQ(ls::classify_after_gap(good, tick_step(11, 110000), 3, 10), ls::TickClass::kOk);
  EXPECT_EQ(ls::classify_after_gap(good, tick_step(13, 130000), 3, 10), ls::TickClass::kOk);
  EXPECT_EQ(
    ls::classify_after_gap(good, tick_step(11, 1758729600LL * 1000000LL), 3, 10),
    ls::TickClass::kJumped);
  EXPECT_EQ(ls::classify_after_gap(good, tick_step(11, 50000), 3, 10), ls::TickClass::kJumped);
  ls::Step failed = tick_step(11, 110000);
  failed.tick_ok = false;
  EXPECT_EQ(ls::classify_after_gap(good, failed, 3, 10), ls::TickClass::kFailed);
}

TEST(LockstepTicks, SafeStopBudgetCoversShortTickPeriods)
{
  EXPECT_EQ(ls::safe_stop_max_ticks(10), 2000);  // 스펙 기본값
  EXPECT_EQ(ls::safe_stop_max_ticks(20), 2000);
  EXPECT_EQ(ls::safe_stop_max_ticks(100), 2000);
  EXPECT_EQ(ls::safe_stop_max_ticks(1), 20000);  // 1 ms: 2000 Tick 은 sim 2 s 뿐
  EXPECT_EQ(ls::safe_stop_max_ticks(5), 4000);
  EXPECT_EQ(ls::safe_stop_max_ticks(0), 2000);
  // 모든 허용 주기에서: --max-speed 10 m/s 에서 ~2.7 m/s^2 로 서고 1 s 유지할 sim 시간이 된다
  for (int p = 1; p <= 100; ++p) {
    EXPECT_GE(ls::safe_stop_max_ticks(p) * p / 1000.0, 10.0 / 2.7 + 1.0 + 1.0) << p;
  }
  // 벽시계 상한은 그 최악의 정상 정지(1 ms, 10 m/s, Tick 당 ~16.5 ms)보다 넉넉하다
  EXPECT_GT(ls::Limits::kSafeStopMaxWallS, (10.0 / 2.7 + 1.0) * 1000.0 * 0.0165);
  EXPECT_LT(ls::Limits::kRestoreStopAfterFailureWallS, ls::Limits::kSafeStopMaxWallS);
}

// =============================================================================
// Verdict helpers on edge cases
// =============================================================================

ls::StepGroup group(double accel, size_t n, double noise = 0.0, unsigned seed = 3)
{
  std::mt19937 rng(seed);
  std::normal_distribution<double> nz(0.0, 1.0);
  ls::StepGroup g;
  for (size_t i = 0; i < n; ++i) {
    g.fit.emplace_back(accel * 0.01 + noise * nz(rng), 0.01);
  }
  g.n_drive = n;
  return g;
}

TEST(LockstepVerdictHelpers, HoldNeedsEnoughTicks)
{
  const ls::HoldVerdict h = ls::judge_hold("sync_drop2", group(0.72, 9), group(0.0, 50), 0.8);
  EXPECT_EQ(h.verdict, "n/a");
  EXPECT_NE(h.note.find(">= 10"), std::string::npos);
  EXPECT_EQ(ls::judge_hold("x", group(0.72, 30), group(0.70, 30), 0.8).verdict, "held");
  EXPECT_EQ(ls::judge_hold("x", group(0.72, 30), group(0.01, 30), 0.8).verdict, "idle");
  EXPECT_EQ(ls::judge_hold("x", group(0.72, 30), group(0.40, 30), 0.8).verdict, "inconclusive");
}

TEST(LockstepVerdictHelpers, CombineHold)
{
  ls::HoldVerdict a;
  ls::HoldVerdict b;
  a.verdict = "idle";
  b.verdict = "inconclusive";
  EXPECT_EQ(ls::combine_hold({a, b}), "idle");
  b.verdict = "held";
  EXPECT_EQ(ls::combine_hold({a, b}), "inconsistent");
  a.verdict = "n/a";
  EXPECT_EQ(ls::combine_hold({a, b}), "held");
  EXPECT_EQ(ls::combine_hold({}), "unknown");
}

TEST(LockstepVerdictHelpers, MultiFrameBandsAndReference)
{
  // 1/3 과 1 사이 경계: persists >= 0.833, first_frame_only <= 0.567
  EXPECT_EQ(
    ls::judge_multi_frame(group(0.72, 100), "sync1", group(0.72, 50), nullptr, 3).verdict,
    "persists");
  EXPECT_EQ(
    ls::judge_multi_frame(group(0.72, 100), "sync1", group(0.24, 50), nullptr, 3).verdict,
    "first_frame_only");
  EXPECT_EQ(
    ls::judge_multi_frame(group(0.72, 100), "sync1", group(0.50, 50), nullptr, 3).verdict,
    "inconclusive");
  EXPECT_EQ(
    ls::judge_multi_frame(group(0.72, 100), "sync1", group(0.72, 4), nullptr, 3).verdict, "n/a");
  EXPECT_EQ(
    ls::judge_multi_frame(group(0.0, 100), "sync1", group(0.72, 50), nullptr, 3).verdict,
    "inconclusive");
}

TEST(LockstepVerdictHelpers, IdleRecordPositiveControl)
{
  ls::StepGroup cmd;
  cmd.n_drive = 100;
  cmd.idle_drive = 0;
  ls::StepGroup unc;
  unc.n_drive = 100;
  unc.idle_drive = 95;
  EXPECT_EQ(ls::check_idle_records(cmd, unc, false).vote, "idle_record");
  unc.idle_drive = 3;
  EXPECT_EQ(ls::check_idle_records(cmd, unc, false).vote, "held_record");
  unc.idle_drive = 30;
  EXPECT_EQ(ls::check_idle_records(cmd, unc, false).vote, "neutral");
  // 명령 받은 Tick 의 레코드도 유휴로 보이면 레코드는 아무것도 증명하지 못한다
  cmd.idle_drive = 50;
  const ls::IdleRecordCheck c = ls::check_idle_records(cmd, unc, false);
  EXPECT_EQ(c.vote, "unavailable");
  EXPECT_NE(c.note.find("positive control"), std::string::npos);
  cmd.idle_drive = 0;
  EXPECT_EQ(ls::check_idle_records(cmd, unc, true).vote, "unavailable");
  unc.n_drive = 5;
  EXPECT_EQ(ls::check_idle_records(cmd, unc, false).vote, "unavailable");
}

TEST(LockstepVerdictHelpers, JsonCarriesTheVerdicts)
{
  Sim sim;
  const Session s = run_session(sim, 1);
  const std::string j = ls::to_json(s.interp).dump(0);
  EXPECT_NE(j.find("\"hold_overall\""), std::string::npos);
  EXPECT_NE(j.find("\"multi_frame\""), std::string::npos);
  EXPECT_NE(j.find("\"throughput\""), std::string::npos);
  EXPECT_NE(ls::to_json(s.runs.front()).dump(0).find("\"cmd1\""), std::string::npos);
  EXPECT_NE(ls::to_json(s.summaries.front()).dump(0).find("\"rtf\""), std::string::npos);
  EXPECT_NE(j.find("\"ratio_verdict\""), std::string::npos);
  EXPECT_NE(j.find("\"share_verdict\""), std::string::npos);
  EXPECT_NE(j.find("\"multi_records\""), std::string::npos);
  EXPECT_NE(ls::to_json(s.summaries.front()).dump(0).find("\"ratio_se\""), std::string::npos);
}

// =============================================================================
// Actuator / powertrain lag: the per-Tick share reads ~1 even when MORAI drops the input;
// the pattern-level ratios must not be fooled
// =============================================================================

TEST(LockstepLag, DroppedInputIsStillIdleWithActuatorLag)
{
  for (double tau : {0.03, 0.1}) {
    Sim sim;  // 명령 없는 Tick 은 유휴, Tick(3) 은 첫 프레임만
    sim.tau_s = tau;
    const Session s = run_session(sim);
    EXPECT_EQ(s.interp.hold_overall, "idle") << tau << " " << s.interp.line;
    for (const auto & h : s.interp.hold) {
      EXPECT_EQ(h.verdict, "idle") << tau << " " << h.pattern << " " << h.note;
      EXPECT_EQ(h.ratio_verdict, "idle") << tau << " " << h.pattern;
      EXPECT_NEAR(h.q, h.pattern == "sync_drop2" ? 0.465 : 0.287, 0.03) << tau << " " << h.pattern;
    }
    // 지연이 Tick 단위 share 를 "유지" 쪽으로 끌어올린다 — 그래서 share 만으로는 판정하지 않는다
    const ls::HoldVerdict & d2 = hold_of(s.interp, "sync_drop2");
    EXPECT_GT(d2.share, 0.75) << tau;
    if (d2.share_verdict == "held") {
      EXPECT_NE(d2.note.find("lags the input"), std::string::npos) << d2.note;
    }
    EXPECT_EQ(s.interp.multi.verdict, "first_frame_only") << tau << " " << s.interp.multi.note;
    EXPECT_EQ(s.interp.multi.method, "pattern ratios");
    EXPECT_NE(s.interp.line.find("idle"), std::string::npos) << s.interp.line;
    EXPECT_EQ(s.interp.line.find("held"), std::string::npos) << s.interp.line;
  }
}

TEST(LockstepLag, HeldInputIsStillHeldWithActuatorLag)
{
  for (double tau : {0.03, 0.1}) {
    Sim sim;
    sim.hold = true;
    sim.persist = true;
    sim.tau_s = tau;
    const Session s = run_session(sim);
    EXPECT_EQ(s.interp.hold_overall, "held") << tau << " " << s.interp.line;
    EXPECT_EQ(s.interp.multi.verdict, "persists") << tau;
    EXPECT_NEAR(s.interp.multi.m, 1.0, 0.03) << tau;
  }
}

// =============================================================================
// Pattern-level ratios and the combination with the per-Tick share
// =============================================================================

ls::PatternSummary summary(const char * name, double ratio, double se, size_t valid = 2)
{
  ls::PatternSummary p;
  p.pattern = name;
  p.runs = p.valid = valid;
  p.ratio_mean = ratio;
  p.ratio_se = se;
  return p;
}

TEST(LockstepPatternLevel, RatioSeUsesTheLargerOfRunSpreadAndFit)
{
  ls::RunAnalysis a;
  a.pattern = "sync1";
  a.fit.n = 100;
  a.ratio = 0.90;
  a.ratio_se = 0.001;
  ls::RunAnalysis b = a;
  b.ratio = 0.94;
  const ls::PatternSummary p = ls::summarize_pattern("sync1", {&a, &b});
  EXPECT_NEAR(p.ratio_se, 0.02, 1e-9);  // sd 0.0283 / sqrt(2) > 0.001 / sqrt(2)
  b.ratio = 0.90;
  b.ratio_se = 0.02;
  const ls::PatternSummary q = ls::summarize_pattern("sync1", {&a, &b});
  EXPECT_NEAR(q.ratio_se, std::sqrt((0.001 * 0.001 + 0.02 * 0.02) / 2.0) / std::sqrt(2.0), 1e-9);
  // 런 하나: 산포가 없으니 최소제곱 SE
  const ls::PatternSummary one = ls::summarize_pattern("sync1", {&a});
  EXPECT_NEAR(one.ratio_se, 0.001, 1e-12);
  const ls::RatioVs r = ls::ratio_vs(summary("sync_drop2", 0.45, 0.01), summary("sync1", 0.9, 0.01));
  EXPECT_NEAR(r.q, 0.5, 1e-12);
  EXPECT_NEAR(r.se, std::sqrt(0.01 * 0.01 + 0.25 * 0.01 * 0.01) / 0.9, 1e-12);
  const ls::PatternSummary good = summary("sync1", 0.9, 0.01);
  const ls::PatternSummary still = summary("sync1", 0.01, 0.001);  // 거의 안 움직임
  const ls::PatternSummary noisy = summary("sync1", 0.5, 0.3);     // 2 SE 안 됨
  EXPECT_TRUE(ls::reference_usable(&good));
  EXPECT_FALSE(ls::reference_usable(&still));
  EXPECT_FALSE(ls::reference_usable(&noisy));
  EXPECT_FALSE(ls::reference_usable(nullptr));
}

TEST(LockstepPatternLevel, ShareAndRatioCombine)
{
  const ls::PatternSummary s1 = summary("sync1", 0.9, 0.01);
  // 비율은 유지, Tick 단위는 유휴: 지연으로 설명되지 않는다 -> conflict (held 가 아니다)
  ls::PatternSummary d2 = summary("sync_drop2", 0.9, 0.01);
  d2.cmd1 = group(0.72, 60);
  d2.unc1 = group(0.0, 60);
  const ls::HoldVerdict c = ls::judge_hold_pattern(d2, 2, &s1, 0.8);
  EXPECT_EQ(c.ratio_verdict, "held");
  EXPECT_EQ(c.share_verdict, "idle");
  EXPECT_EQ(c.verdict, "conflict");
  EXPECT_EQ(ls::combine_hold({c}), "conflict");
  // 비율은 유휴, Tick 단위는 유지: 지연으로 설명된다 -> idle
  d2.ratio_mean = 0.42;
  d2.unc1 = group(0.70, 60);
  const ls::HoldVerdict l = ls::judge_hold_pattern(d2, 2, &s1, 0.8);
  EXPECT_EQ(l.share_verdict, "held");
  EXPECT_EQ(l.verdict, "idle");
  EXPECT_NE(l.note.find("lags"), std::string::npos) << l.note;
  // 기준 sync1 없음: Tick 단위 유지는 믿지 않고, 유휴는 믿는다
  const ls::HoldVerdict nh = ls::judge_hold_pattern(d2, 2, nullptr, 0.8);
  EXPECT_EQ(nh.ratio_verdict, "n/a");
  EXPECT_EQ(nh.verdict, "inconclusive");
  EXPECT_NE(nh.note.find("measure sync1"), std::string::npos) << nh.note;
  d2.unc1 = group(0.0, 60);
  EXPECT_EQ(ls::judge_hold_pattern(d2, 2, nullptr, 0.8).verdict, "idle");
  // 비율이 띠 안 (예: 유휴에서 크게 굴러감): inconclusive
  d2.ratio_mean = 0.70;
  d2.unc1 = group(0.5, 60);
  EXPECT_EQ(ls::judge_hold_pattern(d2, 2, &s1, 0.8).verdict, "inconclusive");
  // 둘 다 유지
  d2.ratio_mean = 0.89;
  d2.unc1 = group(0.71, 60);
  EXPECT_EQ(ls::judge_hold_pattern(d2, 2, &s1, 0.8).verdict, "held");
  // 패턴 수준이 유지로 정했으면 잡음에 약한 Tick 단위 진단(late)은 판정도 설명도 되지 않는다
  d2.cmd1 = group(0.0, 60);
  d2.unc1 = group(0.7, 60);
  const ls::HoldVerdict nl = ls::judge_hold_pattern(d2, 2, &s1, 0.8);
  EXPECT_EQ(nl.share_verdict, "late");
  EXPECT_EQ(nl.verdict, "held");
  EXPECT_NE(nl.note.find("not used"), std::string::npos) << nl.note;
  EXPECT_EQ(nl.note.find("one Tick late"), std::string::npos) << nl.note;
  // sync_tick3x 의 유휴 예측은 1/3
  const ls::HoldVerdict t = ls::judge_hold_pattern(summary("sync_tick3x", 0.26, 0.01), 3, &s1, 0.8);
  EXPECT_NEAR(t.q_idle, 1.0 / 3.0, 1e-12);
  EXPECT_EQ(t.ratio_verdict, "idle");
}

TEST(LockstepPatternLevel, MultiFrameFromPatternRatios)
{
  const ls::PatternSummary s1 = summary("sync1", 0.9, 0.01);
  const ls::PatternSummary t3x = summary("sync_tick3x", 0.26, 0.01);
  const ls::MultiFrameVerdict a =
    ls::judge_multi_frame_pattern(summary("sync_tick3", 0.26, 0.01), s1, &t3x, 3, 0.8);
  EXPECT_EQ(a.verdict, "first_frame_only");
  EXPECT_EQ(a.method, "pattern ratios");
  EXPECT_NE(a.note.find("behaves like"), std::string::npos) << a.note;
  const ls::MultiFrameVerdict b =
    ls::judge_multi_frame_pattern(summary("sync_tick3", 0.9, 0.01), s1, &t3x, 3, 0.8);
  EXPECT_EQ(b.verdict, "persists");
  EXPECT_NE(b.note.find("DIFFERS"), std::string::npos) << b.note;
  EXPECT_EQ(
    ls::judge_multi_frame_pattern(summary("sync_tick3", 0.6, 0.01), s1, nullptr, 3, 0.8).verdict,
    "inconclusive");  // m 0.667: between first frame only (<= 0.567) and persists (>= 0.833)
  EXPECT_EQ(
    ls::judge_multi_frame_pattern(summary("sync_tick3", 0.5, 0.01), summary("sync1", 0.0, 0.0), nullptr, 3, 0.8)
      .verdict,
    "inconclusive");
  EXPECT_EQ(
    ls::judge_multi_frame_pattern(summary("sync_tick3", 0.5, 0.01, 0), s1, nullptr, 3, 0.8).verdict,
    "n/a");
}

TEST(LockstepPatternLevel, WithoutSync1TheMultiFrameFallsBackToPerTick)
{
  Sim sim;
  std::mt19937 rng(5);
  std::vector<ls::RunAnalysis> runs;
  for (const auto & p : kPatterns) {
    if (std::string(p.name) == "sync1") {
      continue;
    }
    ls::RunInput in;
    in.pattern = p.name;
    in.cmd_every = p.cmd_every;
    in.frames = p.frames;
    in.steps = simulate(sim, p, 0.8, 2.0, &rng);
    in.commanded_accel = 0.8;
    runs.push_back(ls::analyze_run(in));
  }
  std::vector<ls::PatternSummary> sums;
  for (const auto & r : runs) {
    sums.push_back(ls::summarize_pattern(r.pattern, {&r}));
  }
  const ls::Interpretation i = ls::interpret(sums, 0.8, false, ls::DtReport{}, {});
  EXPECT_EQ(i.multi.method, "per-Tick");
  EXPECT_NE(i.multi.a1_source.find("sync1 not measured"), std::string::npos);
  EXPECT_EQ(hold_of(i, "sync_drop2").ratio_verdict, "n/a");
  EXPECT_EQ(hold_of(i, "sync_drop2").verdict, "idle");  // Tick 단위 유휴는 지연으로 생기지 않는다
}

// =============================================================================
// Idle records: synchronous-mode calibration and the Tick(n) record vote
// =============================================================================

TEST(LockstepRecords, SyncIdleCalibrationAcceptsAnyBrakeBeforeTheFirstCommand)
{
  act::IdleSignature fb;  // 0.003 +- tol
  std::vector<std::pair<double, double>> brake1(20, {0.0, 1.0});
  const act::IdleCalibration c = ls::calibrate_sync_idle(brake1, fb);
  EXPECT_EQ(c.source, "measured");
  EXPECT_DOUBLE_EQ(c.signature.brake_center, 1.0);
  EXPECT_FALSE(c.ambiguous);
  // 실시간 규칙(0..0.05)은 그대로
  std::vector<std::pair<double, double>> small(20, {0.0, 0.004});
  EXPECT_DOUBLE_EQ(ls::calibrate_sync_idle(small, fb).signature.brake_center, 0.004);
  // 스로틀이 있으면 유휴가 아니다 -> 기본값
  std::vector<std::pair<double, double>> driven(20, {0.3, 0.5});
  EXPECT_EQ(ls::calibrate_sync_idle(driven, fb).source, "default");
  // 브레이크 0 은 여전히 구분 불가
  std::vector<std::pair<double, double>> zero(20, {0.0, 0.0});
  EXPECT_TRUE(ls::calibrate_sync_idle(zero, fb).ambiguous);
  // 너무 적으면 기본값
  std::vector<std::pair<double, double>> few(3, {0.0, 1.0});
  EXPECT_EQ(ls::calibrate_sync_idle(few, fb).source, "default");
}

TEST(LockstepRecords, TickNRecordVote)
{
  ls::StepGroup cmd;
  cmd.n_drive = 100;
  ls::StepGroup multi;
  multi.n_drive = 60;
  multi.idle_drive = 60;
  EXPECT_EQ(ls::check_multi_records(cmd, multi, false).vote, "first_frame_record");
  multi.idle_drive = 2;
  EXPECT_EQ(ls::check_multi_records(cmd, multi, false).vote, "persist_record");
  multi.idle_drive = 20;
  EXPECT_EQ(ls::check_multi_records(cmd, multi, false).vote, "neutral");
  EXPECT_EQ(ls::check_multi_records(cmd, multi, true).vote, "unavailable");
  cmd.idle_drive = 40;  // 양성 대조 실패
  EXPECT_EQ(ls::check_multi_records(cmd, multi, false).vote, "unavailable");
  // 합성 세션: 첫 프레임만이면 Tick(3) 뒤 레코드는 유휴, 유지면 명령
  EXPECT_EQ(run_session(Sim{}).interp.multi_records.vote, "first_frame_record");
  Sim persist;
  persist.persist = true;
  EXPECT_EQ(run_session(persist).interp.multi_records.vote, "persist_record");
}

}  // namespace
