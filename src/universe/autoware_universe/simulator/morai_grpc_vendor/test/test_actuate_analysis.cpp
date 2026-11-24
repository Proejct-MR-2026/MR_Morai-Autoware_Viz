// 신규 — 26.09.25 _ YSH _ morai_actuate 순수 계산부 gtest (합성 데이터, 네트워크 없음)
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

#include "actuate_analysis.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <vector>

namespace
{
namespace act = morai_grpc_vendor::actuate;

// =============================================================================
// Synthetic MORAI: a physics step, updates that advance k steps, and a consumption model
// =============================================================================

enum class Consume { kPerUpdate, kPerStep };

/// What vehicle_state shows. main.py only says it echoes the command and falls back to the
/// idle record in a frame without one; per step or per frame is unknown, so all three are
/// tested.
enum class Record {
  kLastStepInput,  ///< the input of the update's last physics step
  kArrivalEcho,    ///< non-idle iff a command arrived for this update (whatever physics did)
  kAlwaysIdle,     ///< a consumed zero-pedal command also echoes brake 0.003
};

struct SimSpec
{
  double step_us{16666.67};
  /// Steps per update, cycled (1,1,1,1,2 = 20% two-step updates, as measured at 60 Hz).
  std::vector<int> k_cycle{1, 1, 1, 1, 2};
  /// Random steps per update (value, probability); used instead of k_cycle when not empty.
  std::vector<std::pair<int, double>> k_dist;
  /// Sends before each update, cycled (1 = locked1, {1,0} = locked_half, 4 = burst4).
  std::vector<int> sends_cycle{1};
  Consume consume{Consume::kPerUpdate};
  Record record{Record::kLastStepInput};
  /// Physics uses the command sent `lag` updates earlier (record lag follows `record`).
  int lag{0};
  /// MORAI holds the last command (the measured MORAI does not; tested as the contrast).
  bool hold{false};
  double sigma_v{0.0};  ///< Gaussian speed noise, m/s
  unsigned seed{1};
  double accel{0.8};
  double drive_s{2.0};
  double idle_decel{0.0};  ///< speed change per idle step (coasting)
};

act::RunInput simulate(const SimSpec & sp)
{
  std::mt19937 rng(sp.seed);
  std::normal_distribution<double> noise(0.0, 1.0);
  std::uniform_real_distribution<double> u01(0.0, 1.0);
  act::RunInput in;
  in.drive_s = sp.drive_s;
  in.fit_start_s = 0.5;
  in.commanded_accel = sp.accel;
  in.step_us = sp.step_us;
  const int64_t t_start = 1790000000000000;  // Unix-epoch µs, as MORAI
  double t_us = static_cast<double>(t_start);
  double v = 0.0;
  double x = 0.0;
  in.updates.push_back({t_start, 0.0, 0.0, 0.0, 0.0, 0.0, 0.003});
  in.t0_us = t_start;
  const double dt = sp.step_us / 1e6;
  std::vector<int> sent;
  bool holding = false;
  size_t i = 0;
  while (t_us - static_cast<double>(t_start) < (sp.drive_s + 0.3) * 1e6) {
    int k = sp.k_cycle[i % sp.k_cycle.size()];
    if (!sp.k_dist.empty()) {
      const double r = u01(rng);
      double c = 0.0;
      k = sp.k_dist.back().first;
      for (const auto & kp : sp.k_dist) {
        c += kp.second;
        if (r < c) {
          k = kp.first;
          break;
        }
      }
    }
    const int s = sp.sends_cycle[i % sp.sends_cycle.size()];
    sent.push_back(s);
    const int64_t prev_ts = in.updates.back().morai_us;
    for (int b = 0; b < s; ++b) {
      in.sends.push_back({in.updates.back().local_ms + 0.1, prev_ts, true});
    }
    const long src = static_cast<long>(sent.size()) - 1 - sp.lag;
    const bool cmd = src >= 0 && sent[static_cast<size_t>(src)] > 0;
    holding = holding || cmd;
    const bool have = sp.hold ? holding : cmd;
    const double v0 = v;
    bool last_step_driven = false;
    for (int step = 0; step < k; ++step) {
      const bool driven =
        have && (sp.consume == Consume::kPerUpdate || step == 0);  // per step: one step only
      v += driven ? sp.accel * dt : -sp.idle_decel * dt;
      last_step_driven = driven;
    }
    t_us += k * sp.step_us;
    x += 0.5 * (v0 + v) * k * dt;
    act::UpdateSample u;
    u.morai_us = static_cast<int64_t>(std::llround(t_us));
    u.local_ms = (t_us - static_cast<double>(t_start)) / 1e3 + 0.05;
    u.speed_mps = v + sp.sigma_v * noise(rng);
    u.x = x;
    u.y = 0.0;
    const bool rec = sp.record == Record::kLastStepInput ? last_step_driven
                     : sp.record == Record::kArrivalEcho ? s > 0
                                                         : false;
    u.throttle = rec ? sp.accel / 3.0 : 0.0;  // 레코드는 마지막 스텝의 입력 (기본)
    u.brake = rec ? 0.0 : 0.003;
    in.updates.push_back(u);
    ++i;
  }
  return in;
}

// =============================================================================
// Fit / stats
// =============================================================================

TEST(ActuateFit, ExactLine)
{
  const act::LinearFit f = act::fit_line({0.0, 1.0, 2.0, 3.0}, {1.0, 3.0, 5.0, 7.0});
  EXPECT_EQ(f.n, 4u);
  EXPECT_NEAR(f.slope, 2.0, 1e-12);
  EXPECT_NEAR(f.intercept, 1.0, 1e-12);
  EXPECT_NEAR(f.r2, 1.0, 1e-12);
}

TEST(ActuateFit, NoisyLineAndDegenerateInputs)
{
  const act::LinearFit f = act::fit_line({0, 1, 2, 3, 4}, {0.1, 0.9, 2.1, 2.9, 4.1});
  EXPECT_NEAR(f.slope, 1.0, 0.05);
  EXPECT_LT(f.r2, 1.0);
  EXPECT_GT(f.r2, 0.98);
  EXPECT_TRUE(std::isnan(act::fit_line({1.0}, {2.0}).slope));
  EXPECT_TRUE(std::isnan(act::fit_line({1.0, 1.0, 1.0}, {1.0, 2.0, 3.0}).slope));
  const act::LinearFit flat = act::fit_line({0, 1, 2}, {5, 5, 5});
  EXPECT_NEAR(flat.slope, 0.0, 1e-12);
  EXPECT_TRUE(std::isnan(flat.r2));
}

TEST(ActuateFit, MeanAndSampleSd)
{
  EXPECT_DOUBLE_EQ(act::mean_of({1, 2, 3}), 2.0);
  EXPECT_DOUBLE_EQ(act::sd_of({1, 2, 3}), 1.0);
  EXPECT_TRUE(std::isnan(act::sd_of({1})));
  EXPECT_TRUE(std::isnan(act::mean_of({})));
}

// =============================================================================
// Physics step
// =============================================================================

std::vector<int64_t> jittered(const std::vector<std::pair<int64_t, int>> & base_count)
{
  std::vector<int64_t> d;
  int j = 0;
  for (const auto & bc : base_count) {
    for (int i = 0; i < bc.second; ++i) {
      d.push_back(bc.first + (j++ % 11) - 5);  // ±5 µs, as MORAI
    }
  }
  return d;
}

TEST(ActuateStep, SixtyHzWithTwoStepCatchUps)
{
  // 60 Hz step, ~50 updates/s: 80% one step, 20% two, a rare 6-step stall.
  auto d = jittered({{16667, 800}, {33333, 200}});
  d.push_back(100000);
  const act::StepEstimate e = act::estimate_physics_step(d, 0.0);
  EXPECT_TRUE(e.reliable);
  EXPECT_EQ(e.method, "fundamental");
  EXPECT_EQ(e.divisor, 1);
  EXPECT_NEAR(e.step_us, 16666.7, 3.0);
  EXPECT_EQ(e.clusters.size(), 2u);
}

TEST(ActuateStep, TwoHundredHzIsNotTheMode)
{
  // 200 Hz, ~45 updates/s: every update advances 4 or 5 steps; the mode is 4 steps.
  const act::StepEstimate e = act::estimate_physics_step(jittered({{20000, 700}, {25000, 300}}), 0);
  EXPECT_TRUE(e.reliable);
  EXPECT_EQ(e.divisor, 4);
  EXPECT_NEAR(e.step_us, 5000.0, 2.0);
}

TEST(ActuateStep, SingleClusterIsAmbiguousUntilHinted)
{
  const auto d = jittered({{20000, 500}});
  const act::StepEstimate e = act::estimate_physics_step(d, 0.0);
  EXPECT_FALSE(e.reliable);
  EXPECT_NEAR(e.step_us, 20000.0, 2.0);
  EXPECT_FALSE(e.warnings.empty());

  const act::StepEstimate h = act::estimate_physics_step(d, 5000.0);
  EXPECT_TRUE(h.reliable);
  EXPECT_EQ(h.method, "hint_divides_updates");
  EXPECT_NEAR(h.step_us, 5000.0, 1.0);
}

TEST(ActuateStep, HintAgreesOrIsUsedOnMismatch)
{
  const auto d = jittered({{16667, 800}, {33333, 200}});
  const act::StepEstimate ok = act::estimate_physics_step(d, 1e6 / 60.0);
  EXPECT_TRUE(ok.reliable);
  EXPECT_EQ(ok.method, "fundamental");

  const act::StepEstimate bad = act::estimate_physics_step(d, 1e6 / 47.0);
  EXPECT_FALSE(bad.reliable);
  EXPECT_EQ(bad.method, "hint");
  EXPECT_NEAR(bad.step_us, 1e6 / 47.0, 1e-6);
}

TEST(ActuateStep, InsufficientData)
{
  const act::StepEstimate e = act::estimate_physics_step({16667, 16667, -5, 0}, 0.0);
  EXPECT_TRUE(std::isnan(e.step_us));
  EXPECT_EQ(e.method, "insufficient");
  const act::StepEstimate h = act::estimate_physics_step({16667}, 10000.0);
  EXPECT_DOUBLE_EQ(h.step_us, 10000.0);
  EXPECT_EQ(h.method, "hint");
}

TEST(ActuateStep, StepsFor)
{
  EXPECT_EQ(act::steps_for(16660, 16666.7), 1);
  EXPECT_EQ(act::steps_for(33340, 16666.7), 2);
  EXPECT_EQ(act::steps_for(100000, 16666.7), 6);
  EXPECT_EQ(act::steps_for(3000, 16666.7), 1);  // never 0
  EXPECT_EQ(act::steps_for(20000, 0.0), 1);     // no step known
}

// =============================================================================
// Idle record
// =============================================================================

TEST(ActuateIdle, MainPyIdleRecord)
{
  const act::IdleSignature sig;  // throttle 0, brake 0.003 ± 0.0015
  EXPECT_TRUE(act::looks_idle(sig, 0.0, 0.003));
  EXPECT_TRUE(act::looks_idle(sig, 0.0, 0.0031));
  EXPECT_FALSE(act::looks_idle(sig, 0.0, 0.0));   // zero-pedal echo
  EXPECT_FALSE(act::looks_idle(sig, 0.27, 0.0));  // driven
  EXPECT_FALSE(act::looks_idle(sig, 0.0, 0.66));  // braking
}

TEST(ActuateIdle, CalibrationUsesTheMeasuredIdleBrake)
{
  const act::IdleSignature def;
  std::vector<std::pair<double, double>> recs(20, {0.0, 0.005});
  const act::IdleCalibration c = act::calibrate_idle(recs, def);
  EXPECT_EQ(c.source, "measured");
  EXPECT_FALSE(c.ambiguous);
  EXPECT_NEAR(c.signature.brake_center, 0.005, 1e-12);
  EXPECT_FALSE(c.warnings.empty());  // differs from main.py's 0.003
}

TEST(ActuateIdle, ZeroBrakeIdleIsAmbiguous)
{
  const act::IdleCalibration c =
    act::calibrate_idle(std::vector<std::pair<double, double>>(20, {0.0, 0.0}), {});
  EXPECT_TRUE(c.ambiguous);
  EXPECT_EQ(c.source, "default");
}

TEST(ActuateIdle, TooFewOrActiveRecordsKeepDefaults)
{
  EXPECT_EQ(act::calibrate_idle({{0.0, 0.003}}, {}).source, "default");
  const act::IdleCalibration c =
    act::calibrate_idle(std::vector<std::pair<double, double>>(20, {0.3, 0.0}), {});
  EXPECT_EQ(c.source, "default");
  EXPECT_FALSE(c.warnings.empty());
  EXPECT_FALSE(c.ambiguous);  // default 0.003 is distinguishable from 0
}

// =============================================================================
// Per run
// =============================================================================

TEST(ActuateRun, PerUpdateLocked1)
{
  SimSpec sp;
  const act::RunAnalysis a = act::analyze_run(simulate(sp));
  EXPECT_NEAR(a.ratio, 1.0, 0.03);
  EXPECT_GT(a.fit.r2, 0.99);
  EXPECT_DOUBLE_EQ(a.idle_fraction, 0.0);
  EXPECT_NEAR(a.sends_per_update, 1.0, 1e-9);
  EXPECT_NEAR(a.steps_per_update, 1.2, 0.02);
  EXPECT_NEAR(a.update_step_coverage, 1.0 / 1.2, 0.02);
  EXPECT_NEAR(a.multi_step_accel_ratio, 1.0, 0.02);
  EXPECT_NEAR(a.k_share, 1.0, 0.02);
  EXPECT_DOUBLE_EQ(a.single.idle_share(), 0.0);
  EXPECT_DOUBLE_EQ(a.multi.idle_share(), 0.0);
  EXPECT_NEAR(a.distance_ratio, 1.0, 1e-4);  // 정수 µs 반올림 오차
  EXPECT_EQ(a.stalls, 0u);
  EXPECT_EQ(a.steps_hist.at(1) + a.steps_hist.at(2), a.updates_drive);
}

TEST(ActuateRun, PerStepLocked1DrivesOneStepPerUpdate)
{
  SimSpec sp;
  sp.consume = Consume::kPerStep;
  const act::RunAnalysis a = act::analyze_run(simulate(sp));
  EXPECT_NEAR(a.ratio, a.update_step_coverage, 0.03);  // ~0.83
  EXPECT_NEAR(a.multi_step_accel_ratio, 0.5, 0.03);
  EXPECT_NEAR(a.k_share, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(a.single.idle_share(), 0.0);
  EXPECT_DOUBLE_EQ(a.multi.idle_share(), 1.0);  // last step of a 2-step update is idle
  EXPECT_NEAR(a.idle_fraction, 0.2, 0.03);
}

TEST(ActuateRun, TwoHundredHzHasNoSingleStepUpdatesButKShareStillDecides)
{
  SimSpec sp;
  sp.step_us = 5000.0;
  sp.k_cycle = {4, 4, 5, 3, 4, 4, 5};  // ~50 updates/s at 200 Hz
  const act::RunAnalysis up = act::analyze_run(simulate(sp));
  EXPECT_EQ(up.single.n_fit, 0u);
  EXPECT_TRUE(std::isnan(up.multi_step_accel_ratio));
  EXPECT_NEAR(up.k_share, 1.0, 0.02);
  EXPECT_NEAR(up.ratio, 1.0, 0.03);

  sp.consume = Consume::kPerStep;
  const act::RunAnalysis st = act::analyze_run(simulate(sp));
  EXPECT_NEAR(st.k_share, 0.0, 0.02);
  EXPECT_NEAR(st.ratio, st.update_step_coverage, 0.03);  // ~0.25
  EXPECT_DOUBLE_EQ(st.multi.idle_share(), 1.0);
}

TEST(ActuateRun, KDvRegression)
{
  act::KDvStat k;
  for (int i = 0; i < 10; ++i) {
    k.add(1, 0.01);
    k.add(2, 0.02);
  }
  EXPECT_NEAR(k.slope(), 0.01, 1e-12);
  EXPECT_NEAR(k.intercept(), 0.0, 1e-12);
  EXPECT_NEAR(k.k_share(), 1.0, 1e-9);
  EXPECT_TRUE(k.spread_ok(5));
  EXPECT_FALSE(k.spread_ok(11));
  act::KDvStat flat;
  for (int i = 0; i < 10; ++i) {
    flat.add(4, 0.01);
  }
  EXPECT_TRUE(std::isnan(flat.slope()));  // no spread in k
  EXPECT_FALSE(flat.spread_ok(1));
  act::KDvStat m = k;
  m.merge(k);
  EXPECT_EQ(m.n, 40u);
  EXPECT_EQ(m.k_counts.at(2), 20u);
}

TEST(ActuateRun, KDvStandardError)
{
  act::KDvStat exact;
  for (int i = 0; i < 20; ++i) {
    exact.add(1 + i % 2, 0.01 * (1 + i % 2));
  }
  EXPECT_NEAR(exact.slope_se(), 0.0, 1e-9);
  EXPECT_NEAR(exact.k_share_se(), 0.0, 1e-6);

  // Δv = 0.004 k + N(0, 0.001), k in {3, 4, 5}: SE(slope) = sigma / sqrt(Sxx)
  std::mt19937 rng(3);
  std::normal_distribution<double> nz(0.0, 0.001);
  act::KDvStat k;
  for (int i = 0; i < 3000; ++i) {
    const int kk = 3 + i % 3;
    k.add(kk, 0.004 * kk + nz(rng));
  }
  const double sxx = 3000.0 * 2.0 / 3.0;  // var of {3,4,5} = 2/3
  EXPECT_NEAR(k.slope(), 0.004, 5e-5);
  EXPECT_NEAR(k.slope_se(), 0.001 / std::sqrt(sxx), 0.1 * 0.001 / std::sqrt(sxx));
  EXPECT_NEAR(k.k_share_se(), k.slope_se() * 4.0 / (0.016), 0.02 * k.k_share_se());

  act::KDvStat two;
  two.add(1, 0.1);
  two.add(2, 0.2);
  EXPECT_TRUE(std::isnan(two.slope_se()));  // n < 3
}

TEST(ActuateRun, LockedHalfShowsNoHold)
{
  SimSpec sp;
  sp.sends_cycle = {1, 0};
  sp.k_cycle = {1, 1, 1, 2, 1};  // odd length: both send groups see 1- and 2-step updates
  const act::RunAnalysis a = act::analyze_run(simulate(sp));
  EXPECT_NEAR(a.sends_per_update, 0.5, 0.03);
  EXPECT_NEAR(a.idle_fraction, 0.5, 0.03);
  EXPECT_DOUBLE_EQ(a.zero_send.idle_share(), 1.0);
  EXPECT_DOUBLE_EQ(a.some_send.idle_share(), 0.0);
  EXPECT_NEAR(a.zero_send_accel_ratio, 0.0, 1e-9);
  EXPECT_NEAR(a.ratio, 0.5, 0.06);
  EXPECT_EQ(a.sends_hist.at(0), a.zero_send.n_drive);
}

TEST(ActuateRun, Burst4CountsEverySend)
{
  SimSpec sp;
  sp.sends_cycle = {4};
  const act::RunAnalysis a = act::analyze_run(simulate(sp));
  EXPECT_NEAR(a.sends_per_update, 4.0, 1e-9);
  EXPECT_EQ(a.sends_hist.at(4), a.updates_drive);
  EXPECT_NEAR(a.ratio, 1.0, 0.03);
}

TEST(ActuateRun, SendAttributionIsInMoraiTime)
{
  // updates at 100, 200, 300 (+t0 = 100). A send whose newest known update is 200 belongs to
  // the update at 300, never to 200 — even if its local time says otherwise.
  act::RunInput in;
  in.t0_us = 100;
  in.drive_s = 1.0;
  in.step_us = 100;
  in.commanded_accel = 1.0;
  in.updates = {{100, 0.0, 0, 0, 0, 0, 0}, {200, 1.0, 0, 0, 0, 0, 0}, {300, 2.0, 0, 0, 0, 0, 0}};
  in.sends = {{0.5, 200, true}, {0.6, 200, true}, {0.7, 100, true}, {0.8, 150, false}};
  const act::RunAnalysis a = act::analyze_run(in);
  EXPECT_EQ(a.updates_drive, 2u);
  EXPECT_EQ(a.sends_hist.at(1), 1u);  // update 200: the send after 100
  EXPECT_EQ(a.sends_hist.at(2), 1u);  // update 300: two sends after 200
  EXPECT_EQ(a.sends_failed, 1u);
  EXPECT_EQ(a.sends_in_drive, 3u);
}

TEST(ActuateRun, WindowAndStalls)
{
  SimSpec sp;
  sp.k_cycle = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 6};
  const act::RunInput in = simulate(sp);
  const act::RunAnalysis a = act::analyze_run(in);
  // updates after drive_s (the simulation runs 0.3 s longer) are excluded
  size_t within = 0;
  for (size_t i = 1; i < in.updates.size(); ++i) {
    const double t = static_cast<double>(in.updates[i].morai_us - in.t0_us) / 1e6;
    within += (t > 0.0 && t <= sp.drive_s) ? 1 : 0;
  }
  EXPECT_EQ(a.updates_drive, within);
  EXPECT_GE(a.stalls, 3u);  // one 6-step gap per 20 updates
  EXPECT_NEAR(a.max_gap_ms, 100.0, 0.1);
  EXPECT_EQ(a.steps_hist.count(6), 1u);
}

TEST(ActuateRun, DistanceRatioFlagsAWrongSpeedUnit)
{
  act::RunInput in = simulate(SimSpec{});
  for (auto & u : in.updates) {
    u.speed_mps *= 3.6;  // as if km/h were not converted
  }
  const act::RunAnalysis a = act::analyze_run(in);
  EXPECT_NEAR(a.distance_ratio, 1.0 / 3.6, 0.01);
  EXPECT_FALSE(a.warnings.empty());
}

// =============================================================================
// Summary / interpretation
// =============================================================================

std::vector<act::PatternSummary> run_all(
  Consume c, int repeats = 2, std::vector<int> k_cycle = {1, 1, 1, 2, 1, 1, 2}, double step_us = 16666.67)
{
  struct P
  {
    const char * name;
    std::vector<int> sends;
  };
  const std::vector<P> ps = {
    {"locked1", {1}}, {"locked_half", {1, 0}}, {"burst4", {4}}, {"free33", {1, 1, 0}}};
  std::vector<act::PatternSummary> out;
  std::vector<act::RunAnalysis> store;
  store.reserve(ps.size() * static_cast<size_t>(repeats));
  for (const auto & p : ps) {
    std::vector<const act::RunAnalysis *> runs;
    for (int r = 0; r < repeats; ++r) {
      SimSpec sp;
      sp.consume = c;
      sp.sends_cycle = p.sends;
      sp.k_cycle = k_cycle;
      sp.step_us = step_us;
      store.push_back(act::analyze_run(simulate(sp)));
      runs.push_back(&store.back());
    }
    out.push_back(act::summarize_pattern(p.name, runs));
  }
  return out;
}

/// A whole morai_actuate session on synthetic MORAI: the six default patterns x repeats,
/// attribution lag estimated from the runs (as the tool does), then summaries.
struct Session
{
  act::LagEstimate lag;
  std::vector<act::PatternSummary> summaries;
  act::Interpretation interp;
};

Session run_session(SimSpec base, bool idle_ambiguous, int repeats = 2, bool estimate = true)
{
  struct P
  {
    const char * name;
    std::vector<int> sends;
  };
  const std::vector<P> ps = {{"locked1", {1}},       {"locked_half", {1, 0}}, {"burst4", {4}},
                             {"free33", {1, 1, 0}},  {"free100", {2}},        {"free200", {4}}};
  std::vector<act::RunInput> ins;
  const unsigned seed0 = base.seed * 1000u;
  for (const auto & p : ps) {
    for (int r = 0; r < repeats; ++r) {
      SimSpec sp = base;
      sp.sends_cycle = p.sends;
      sp.seed = seed0 + static_cast<unsigned>(ins.size());
      ins.push_back(simulate(sp));
    }
  }
  Session out;
  out.lag = act::estimate_lag(ins);
  std::vector<act::RunAnalysis> store;
  store.reserve(ins.size());
  for (auto & in : ins) {
    in.lag = estimate ? out.lag.physics_lag : 0;
    store.push_back(act::analyze_run(in));
  }
  for (size_t i = 0; i < ps.size(); ++i) {
    std::vector<const act::RunAnalysis *> runs;
    for (int r = 0; r < repeats; ++r) {
      runs.push_back(&store[i * static_cast<size_t>(repeats) + static_cast<size_t>(r)]);
    }
    out.summaries.push_back(act::summarize_pattern(ps[i].name, runs));
  }
  out.interp = act::interpret(out.summaries, idle_ambiguous);
  return out;
}

bool has_line(const act::Interpretation & i, const std::string & needle)
{
  for (const auto & l : i.lines) {
    if (l.find(needle) != std::string::npos) {
      return true;
    }
  }
  return false;
}

TEST(ActuateSummary, RatioMeanAndSd)
{
  act::RunAnalysis a;
  a.fit.n = 10;
  a.ratio = 0.9;
  act::RunAnalysis b = a;
  b.ratio = 1.1;
  act::RunAnalysis ab = a;
  ab.aborted = true;
  const act::PatternSummary p = act::summarize_pattern("locked1", {&a, &b, &ab});
  EXPECT_EQ(p.runs, 3u);
  EXPECT_EQ(p.valid, 2u);
  EXPECT_EQ(p.aborted, 1u);
  EXPECT_NEAR(p.ratio_mean, 1.0, 1e-12);
  EXPECT_NEAR(p.ratio_sd, std::sqrt(0.02), 1e-12);
}

TEST(ActuateInterpret, PerUpdate)
{
  const auto s = run_all(Consume::kPerUpdate);
  const act::Interpretation i = act::interpret(s, false);
  EXPECT_EQ(i.verdict, "per_update");
  EXPECT_EQ(i.hold, "no_hold");
  EXPECT_NEAR(i.multi_step_accel_ratio, 1.0, 0.05);
  // locked1 >> free33 hint appears
  bool hint = false;
  for (const auto & l : i.lines) {
    hint = hint || l.find("free33") != std::string::npos;
  }
  EXPECT_TRUE(hint);
}

TEST(ActuateInterpret, PerStep)
{
  const auto s = run_all(Consume::kPerStep);
  const act::Interpretation i = act::interpret(s, false);
  EXPECT_EQ(i.verdict, "per_step");
  EXPECT_NEAR(i.multi_step_accel_ratio, 0.5, 0.05);
  EXPECT_GT(i.idle_share_multi - i.idle_share_single, 0.9);
}

TEST(ActuateInterpret, TwoHundredHz)
{
  const std::vector<int> k = {4, 4, 5, 3, 4, 4, 5};
  EXPECT_EQ(act::interpret(run_all(Consume::kPerUpdate, 2, k, 5000.0), false).verdict, "per_update");
  const act::Interpretation st = act::interpret(run_all(Consume::kPerStep, 2, k, 5000.0), false);
  EXPECT_EQ(st.verdict, "per_step");
  EXPECT_EQ(st.n_single_fit, 0u);
}

TEST(ActuateInterpret, AmbiguousIdleStillDecidesFromAcceleration)
{
  const act::Interpretation i = act::interpret(run_all(Consume::kPerStep), true);
  EXPECT_EQ(i.verdict, "per_step");
}

TEST(ActuateInterpret, NoMultiStepUpdatesIsInconclusive)
{
  SimSpec sp;
  sp.k_cycle = {1};
  const act::RunAnalysis a = act::analyze_run(simulate(sp));
  const act::Interpretation i = act::interpret({act::summarize_pattern("locked1", {&a})}, false);
  EXPECT_EQ(i.verdict, "inconclusive");
}

TEST(ActuateInterpret, ConflictingEvidenceIsInconclusive)
{
  // Δv scales with k (per update) but 90% of commanded multi-step records are idle (per step)
  act::PatternSummary p;
  p.pattern = "locked1";
  p.single = {100, 0, 100, 100 * 0.8 * 0.0167, 100 * 0.0167};
  p.multi = {50, 45, 50, 50 * 0.8 * 0.0333, 50 * 0.0333};
  for (int i = 0; i < 20; ++i) {
    p.kdv.add(1, 0.8 * 0.0167);
    p.kdv.add(2, 0.8 * 0.0333);
  }
  const act::Interpretation i = act::interpret({p}, false);
  EXPECT_NEAR(i.k_share, 1.0, 0.02);
  EXPECT_EQ(i.verdict, "inconclusive");
}

// --- noise, record semantics, lag: never the wrong model --------------------------------

const std::vector<std::pair<int, double>> kK200 = {{3, 0.1}, {4, 0.8}, {5, 0.1}};

TEST(ActuateRobust, SpeedNoiseAt200HzNeverGivesTheWrongModel)
{
  // Under per-step consumption mean Δv is ~a·step, so dv~k amplifies speed noise; without
  // the standard error this voted per_update in 9-27% of sessions.
  for (double sigma : {0.005, 0.01}) {
    int right_step = 0;
    int right_update = 0;
    const int trials = 40;
    for (int t = 0; t < trials; ++t) {
      SimSpec sp;
      sp.step_us = 5000.0;
      sp.k_dist = kK200;
      sp.sigma_v = sigma;
      sp.idle_decel = 0.05;
      sp.seed = static_cast<unsigned>(100 + t);
      sp.consume = Consume::kPerStep;
      const Session st = run_session(sp, true);
      EXPECT_NE(st.interp.verdict, "per_update") << "sigma " << sigma << " trial " << t;
      right_step += st.interp.verdict == "per_step" ? 1 : 0;
      sp.consume = Consume::kPerUpdate;
      const Session up = run_session(sp, true);
      EXPECT_NE(up.interp.verdict, "per_step") << "sigma " << sigma << " trial " << t;
      right_update += up.interp.verdict == "per_update" ? 1 : 0;
      EXPECT_FALSE(std::isnan(up.interp.k_share_se));
    }
    if (sigma == 0.005) {
      EXPECT_GE(right_update, trials / 2);  // still decides when the evidence is there
    }
    RecordProperty("right_step_" + std::to_string(sigma), right_step);
  }
  // noiseless: decided
  SimSpec sp;
  sp.step_us = 5000.0;
  sp.k_dist = kK200;
  sp.consume = Consume::kPerStep;
  EXPECT_EQ(run_session(sp, true).interp.verdict, "per_step");
}

TEST(ActuateRobust, UniformKWithArrivalEchoIsInconclusive)
{
  // 200 Hz, every update 4 steps, per-step physics, record = "a command arrived": the old
  // idle vote alone said per_update.
  SimSpec sp;
  sp.step_us = 5000.0;
  sp.k_cycle = {4};
  sp.consume = Consume::kPerStep;
  sp.record = Record::kArrivalEcho;
  const Session s = run_session(sp, false);
  EXPECT_EQ(s.interp.verdict, "inconclusive");
  EXPECT_TRUE(s.interp.k_uniform);
  EXPECT_EQ(s.interp.idle_vote, "unavailable");  // no k=1 updates: no positive control
  EXPECT_TRUE(has_line(s.interp, "k=4"));
  EXPECT_TRUE(has_line(s.interp, "compare with the 60 Hz run"));
  // locked1 ratio ~ updates/steps under per step: the cross-Hz comparison has what it needs
  EXPECT_NEAR(s.summaries[0].coverage_mean, 0.25, 0.01);
  EXPECT_NEAR(s.summaries[0].ratio_mean, 0.25, 0.03);
}

TEST(ActuateRobust, IdleVoteNeedsAPositiveControl)
{
  // A consumed zero-pedal command echoes brake 0.003 too: every record looks idle. The k=1
  // control fails, so the idle vote is unavailable and cannot force per_step.
  for (Consume c : {Consume::kPerUpdate, Consume::kPerStep}) {
    SimSpec sp;
    sp.consume = c;
    sp.record = Record::kAlwaysIdle;
    const Session s = run_session(sp, false);
    EXPECT_EQ(s.interp.idle_vote, "unavailable");
    EXPECT_NE(s.interp.idle_vote_note.find("positive control failed"), std::string::npos);
    EXPECT_EQ(s.interp.verdict, c == Consume::kPerUpdate ? "per_update" : "per_step");
    EXPECT_FALSE(s.interp.confirmed);
  }
  // 60 Hz, record = arrival echo, per-step physics: control passes, the idle vote says
  // per_update, physics says per_step -> inconclusive, never per_update.
  SimSpec sp;
  sp.consume = Consume::kPerStep;
  sp.record = Record::kArrivalEcho;
  const Session s = run_session(sp, false);
  EXPECT_EQ(s.interp.idle_vote, "per_update");
  EXPECT_EQ(s.interp.k_vote, "per_step");
  EXPECT_EQ(s.interp.verdict, "inconclusive");
  // record = last-step input: both agree
  sp.record = Record::kLastStepInput;
  const Session ok = run_session(sp, false);
  EXPECT_EQ(ok.interp.verdict, "per_step");
  EXPECT_TRUE(ok.interp.confirmed);
}

TEST(ActuateLag, OneUpdateLagIsMeasuredAndHoldIsNotMisread)
{
  for (Record rec : {Record::kLastStepInput, Record::kArrivalEcho}) {
    SimSpec sp;
    sp.lag = 1;
    sp.record = rec;
    sp.idle_decel = 0.05;
    const Session s = run_session(sp, false);
    EXPECT_TRUE(s.lag.physics_decided);
    EXPECT_EQ(s.lag.physics_lag, 1);
    EXPECT_EQ(s.interp.verdict, "per_update");
    EXPECT_EQ(s.interp.hold, "no_hold");
    if (rec == Record::kArrivalEcho) {
      // vehicle_state shows the command one update before it acts
      EXPECT_TRUE(s.lag.record_decided);
      EXPECT_EQ(s.lag.record_lag, 0);
      bool warned = false;
      for (const auto & w : s.lag.warnings) {
        warned = warned || w.find("record lag 0 != physics lag 1") != std::string::npos;
      }
      EXPECT_TRUE(warned);
    } else {
      EXPECT_EQ(s.lag.record_lag, 1);
    }
    // the failure mode it fixes: attributing with lag 0 reads "MORAI holds the command"
    EXPECT_EQ(run_session(sp, false, 2, false).interp.hold, "holds");
  }
  // with speed noise the lag is still found or left undecided, never a wrong one
  for (int t = 0; t < 10; ++t) {
    SimSpec sp;
    sp.lag = 1;
    sp.sigma_v = 0.01;
    sp.seed = static_cast<unsigned>(7 + t);
    const Session s = run_session(sp, false);
    EXPECT_TRUE(!s.lag.physics_decided || s.lag.physics_lag == 1);
    EXPECT_NE(s.interp.hold, "holds");
    EXPECT_NE(s.interp.verdict, "per_step");
  }
}

TEST(ActuateLag, ZeroLagAndHoldingMorai)
{
  SimSpec sp;
  const Session s = run_session(sp, false);
  EXPECT_TRUE(s.lag.physics_decided);
  EXPECT_EQ(s.lag.physics_lag, 0);
  EXPECT_EQ(s.lag.runs_used, 4u);  // locked_half x2, free33 x2
  EXPECT_EQ(s.interp.hold, "no_hold");

  // MORAI that holds the last command: no lag separates commanded from other updates
  sp.hold = true;
  const Session h = run_session(sp, false);
  EXPECT_FALSE(h.lag.physics_decided);
  EXPECT_EQ(h.lag.physics_lag, 0);
  EXPECT_EQ(h.interp.hold, "holds");
}

TEST(ActuateLag, NoPartialCoverageRunsMeansNoEstimate)
{
  std::vector<act::RunInput> ins = {simulate(SimSpec{})};  // locked1 only
  const act::LagEstimate e = act::estimate_lag(ins);
  EXPECT_EQ(e.runs_used, 0u);
  EXPECT_FALSE(e.physics_decided);
  EXPECT_EQ(e.physics_lag, 0);
  EXPECT_FALSE(e.warnings.empty());
}

TEST(ActuateLag, WaitForTickValuesMatchReaderTimestamps)
{
  const std::vector<int64_t> ts = {100, 200, 300, 400};
  const act::WftMatch all = act::match_wft_values({200, 300, 400}, ts);
  EXPECT_EQ(all.n, 3u);
  EXPECT_DOUBLE_EQ(all.share, 1.0);
  const act::WftMatch part = act::match_wft_values({50, 150, 200, 250, 500}, ts);
  EXPECT_EQ(part.out_of_range, 2u);  // 50, 500
  EXPECT_EQ(part.n, 3u);
  EXPECT_EQ(part.found, 1u);
  EXPECT_NEAR(part.share, 1.0 / 3.0, 1e-12);
  EXPECT_TRUE(std::isnan(act::match_wft_values({}, ts).share));
}

// =============================================================================
// Preflight
// =============================================================================

TEST(ActuatePreflight, OnlyHumanModesAreTakenOverByDefault)
{
  EXPECT_EQ(act::original_mode_refusal(act::kModeKeyboard, "KEYBOARD", false), "");
  EXPECT_EQ(act::original_mode_refusal(act::kModeGameWheel, "GAME_WHEEL", false), "");
  for (int m : {0, 3, 4, 5, 6, 7}) {
    EXPECT_NE(act::original_mode_refusal(m, "M", false), "") << m;
    EXPECT_EQ(act::original_mode_refusal(m, "M", true), "") << m;
  }
  const std::string why = act::original_mode_refusal(3, "VEHICLE_CONTROL_AUTO_MODE", false);
  EXPECT_NE(why.find("--allow-any-original-mode"), std::string::npos);
}

TEST(ActuatePreflight, ZeroPedalBridgeIsSeenThroughItsSteerEcho)
{
  // main.py sends ACCELERATION with throttle = brake = 0; MORAI echoes the steer.
  const std::vector<act::InputRecord> bridge(30, {0.0, 0.0, 0.3, 0.3});
  const act::ForeignInputCheck b = act::check_foreign_input(bridge, 0.2);
  EXPECT_TRUE(b.driving);
  EXPECT_EQ(b.steer, 30u);
  EXPECT_EQ(b.pedal, 0u);
  EXPECT_NE(b.why.find("steer echo"), std::string::npos);

  const std::vector<act::InputRecord> idle(30, {0.0, 0.0, 0.0, 0.0});
  EXPECT_FALSE(act::check_foreign_input(idle, 0.2).driving);
  std::vector<act::InputRecord> pedals(30, {0.3, 0.0, 0.0, 0.0});
  EXPECT_TRUE(act::check_foreign_input(pedals, 0.2).driving);
  std::vector<act::InputRecord> few = idle;
  few[0] = {0.0, 0.0, 0.0, 0.2};  // one record in 30 < 20%
  EXPECT_FALSE(act::check_foreign_input(few, 0.2).driving);
  EXPECT_FALSE(act::check_foreign_input({}, 0.2).driving);
  EXPECT_TRUE(act::steer_echo(0.0, -0.01));
  EXPECT_FALSE(act::steer_echo(0.0005, -0.0005));
  EXPECT_TRUE(act::steer_echo(std::nan(""), 0.0));
}

// =============================================================================
// Envelope
// =============================================================================

TEST(ActuateEnvelope, Limits)
{
  act::EnvelopeLimits l;  // 3 m/s, 15 m, 0.5 s stale, 1 s frozen
  act::EnvelopeState s;
  EXPECT_NE(act::check_envelope(l, s), "");  // no state yet
  s.have_state = true;
  s.speed_mps = 1.6;
  s.distance_m = 2.0;
  s.stale_s = 0.01;
  s.frozen_s = 0.1;
  EXPECT_EQ(act::check_envelope(l, s), "");
  auto bad = [&](auto mutate) {
    act::EnvelopeState t = s;
    mutate(t);
    return act::check_envelope(l, t);
  };
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.speed_mps = 3.01; }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.distance_m = 15.1; }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.stale_s = 0.51; }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.frozen_s = 1.2; }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.collisions = 1; }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.speed_mps = std::nan(""); }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.distance_m = std::nan(""); }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.stale_s = std::nan(""); }), "");
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.steer_echo = 0.3; }), "");  // someone else
  EXPECT_NE(bad([](act::EnvelopeState & t) { t.steer_echo = std::nan(""); }), "");
  EXPECT_EQ(bad([](act::EnvelopeState & t) { t.steer_echo = 1e-4; }), "");
  EXPECT_EQ(bad([](act::EnvelopeState & t) { t.speed_mps = 3.0; }), "");  // inclusive
}

TEST(ActuateEnvelope, TravelEstimate)
{
  // 0.8 m/s^2 for 2 s: 1.6 m accelerating + 1.6^2 / (2 x 1.0) braking.
  EXPECT_NEAR(act::estimate_run_travel_m(0.8, 2.0, 1.0), 1.6 + 1.28, 1e-12);
}

TEST(ActuateUnits, KmhToMpsIsMainPys)
{
  EXPECT_DOUBLE_EQ(act::kKmhToMps, 1.0 / 3.6);
}

}  // namespace
