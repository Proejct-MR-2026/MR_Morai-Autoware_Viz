// 신규 — 26.09.24 _ YSH _ morai_probe 순수 헬퍼 gtest (네트워크 없음)
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

#include "probe_json.hpp"
#include "probe_stats.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace
{
using morai_grpc_vendor::probe::analyze_frames;
using morai_grpc_vendor::probe::checked_sub;
using morai_grpc_vendor::probe::classify_time_base;
using morai_grpc_vendor::probe::classify_wait_for_tick;
using morai_grpc_vendor::probe::count_over;
using morai_grpc_vendor::probe::FrameAnalysis;
using morai_grpc_vendor::probe::grpc_code_name;
using morai_grpc_vendor::probe::infer_unit;
using morai_grpc_vendor::probe::Json;
using morai_grpc_vendor::probe::percentile_sorted;
using morai_grpc_vendor::probe::summarize;
using morai_grpc_vendor::probe::TimedSample;
using morai_grpc_vendor::probe::WaitForTickObservation;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

/// A simulated MORAI frame: from `wall_s` on, polls see `value` / `tick`.
struct Frame
{
  double wall_s;
  int64_t value;
  uint64_t tick;
};

/// Poll `frames` every `poll_s` over [0, end_s): each poll returns the latest frame whose
/// wall time is <= the poll time (what a tight GetAllActorsState loop observes).
std::vector<TimedSample> poll(const std::vector<Frame> & frames, double poll_s, double end_s)
{
  std::vector<TimedSample> out;
  size_t f = 0;
  const auto n = static_cast<size_t>(std::llround(end_s / poll_s));
  for (size_t k = 0; k < n; ++k) {
    const double t = static_cast<double>(k) * poll_s;
    while (f + 1 < frames.size() && frames[f + 1].wall_s <= t + 1e-12) {
      ++f;
    }
    out.push_back(TimedSample{t, frames[f].value, frames[f].tick});
  }
  return out;
}

/// Uniform frames: period `period_s`, timestamp step `step_raw`, starting at `base`.
std::vector<Frame> uniform_frames(double period_s, int64_t step_raw, int64_t base, double end_s)
{
  std::vector<Frame> fr;
  const auto n = static_cast<size_t>(std::llround(end_s / period_s)) + 1;
  for (size_t i = 0; i < n; ++i) {
    fr.push_back(
      Frame{
        static_cast<double>(i) * period_s, base + static_cast<int64_t>(i) * step_raw,
        static_cast<uint64_t>(i)});
  }
  return fr;
}

// =============================================================================
// Statistics
// =============================================================================

TEST(ProbeStats, PercentileLinearInterpolation)
{
  const std::vector<double> v{1, 2, 3, 4, 5};
  EXPECT_DOUBLE_EQ(percentile_sorted(v, 0), 1.0);
  EXPECT_DOUBLE_EQ(percentile_sorted(v, 50), 3.0);
  EXPECT_DOUBLE_EQ(percentile_sorted(v, 90), 4.6);  // numpy.percentile(..., 90) == 4.6
  EXPECT_DOUBLE_EQ(percentile_sorted(v, 100), 5.0);
  EXPECT_DOUBLE_EQ(percentile_sorted({7.0}, 99), 7.0);
  EXPECT_TRUE(std::isnan(percentile_sorted({}, 50)));
}

TEST(ProbeStats, SummarizeSortsAndHandlesEmpty)
{
  const auto s = summarize({5.0, 1.0, 3.0, 2.0, 4.0});
  EXPECT_EQ(s.n, 5u);
  EXPECT_DOUBLE_EQ(s.mean, 3.0);
  EXPECT_DOUBLE_EQ(s.min, 1.0);
  EXPECT_DOUBLE_EQ(s.p50, 3.0);
  EXPECT_DOUBLE_EQ(s.max, 5.0);

  const auto e = summarize({});
  EXPECT_EQ(e.n, 0u);
  EXPECT_TRUE(std::isnan(e.p50));
  EXPECT_TRUE(std::isnan(e.max));
}

TEST(ProbeStats, TailOf035msRttWithA50msStall)
{
  // The field observation: 0.35 ms mean RTT hides 50-69 ms stalls. p99 of 1000 calls with
  // 3 stalls must stay small, but max and the >45 ms count must expose them.
  std::vector<double> v(997, 0.35);
  v.push_back(50.0);
  v.push_back(62.0);
  v.push_back(69.0);
  const auto s = summarize(v);
  EXPECT_NEAR(s.p50, 0.35, 1e-12);
  EXPECT_LT(s.p99, 1.0);
  EXPECT_DOUBLE_EQ(s.max, 69.0);
  EXPECT_EQ(count_over(v, 20.0), 3u);
  EXPECT_EQ(count_over(v, 45.0), 3u);
  EXPECT_EQ(count_over(v, 60.0), 2u);
}

TEST(ProbeStats, GrpcCodeNames)
{
  EXPECT_EQ(grpc_code_name(0), "OK");
  EXPECT_EQ(grpc_code_name(2), "UNKNOWN");
  EXPECT_EQ(grpc_code_name(4), "DEADLINE_EXCEEDED");
  EXPECT_EQ(grpc_code_name(12), "UNIMPLEMENTED");
  EXPECT_EQ(grpc_code_name(14), "UNAVAILABLE");
  EXPECT_EQ(grpc_code_name(16), "UNAUTHENTICATED");
  EXPECT_EQ(grpc_code_name(99), "CODE_99");
  EXPECT_EQ(grpc_code_name(-1), "CODE_-1");
}

// =============================================================================
// Unit inference
// =============================================================================

TEST(ProbeUnit, InfersDecadeAndRateRatio)
{
  auto g = infer_unit(1000.0);
  EXPECT_EQ(g.name, "ms");
  EXPECT_DOUBLE_EQ(g.per_second, 1e3);
  EXPECT_DOUBLE_EQ(g.rate_ratio, 1.0);
  EXPECT_TRUE(g.plausible);

  g = infer_unit(800.0);  // ms timestamps, simulator at 0.8x real time
  EXPECT_EQ(g.name, "ms");
  EXPECT_NEAR(g.rate_ratio, 0.8, 1e-12);
  EXPECT_TRUE(g.plausible);

  EXPECT_EQ(infer_unit(1.0).name, "s");
  EXPECT_EQ(infer_unit(0.98e6).name, "us");
  EXPECT_EQ(infer_unit(1.01e7).name, "100ns");
  EXPECT_EQ(infer_unit(1e9).name, "ns");

  g = infer_unit(300.0);  // closest to ms, but 0.3x is not a finding
  EXPECT_EQ(g.name, "ms");
  EXPECT_FALSE(g.plausible);
}

TEST(ProbeUnit, RejectsNonPositiveAndNaN)
{
  for (const double x : {0.0, -5.0, kNaN, std::numeric_limits<double>::infinity()}) {
    const auto g = infer_unit(x);
    EXPECT_EQ(g.name, "unknown");
    EXPECT_FALSE(g.plausible);
    EXPECT_TRUE(std::isnan(g.per_second));
  }
}

TEST(ProbeUnit, ClassifiesTimeBase)
{
  const double now = 1790000000.0;  // 2026-09
  EXPECT_EQ(classify_time_base(static_cast<int64_t>(now * 1e3) + 1234, 1e3, now), "unix_epoch");
  EXPECT_EQ(
    classify_time_base(static_cast<int64_t>(now) * INT64_C(1000000000), 1e9, now), "unix_epoch");
  EXPECT_EQ(classify_time_base(123456, 1e3, now), "elapsed");  // 123 s since sim start
  EXPECT_EQ(classify_time_base(-5, 1e3, now), "unknown");
  EXPECT_EQ(classify_time_base(123456, kNaN, now), "unknown");
}

TEST(ProbeUnit, ClassifiesDotNetEpoch)
{
  // MORAI is Unity/C#: DateTime.Ticks counts 100 ns from 0001-01-01.
  const double now = 1790000000.0;
  const auto ticks = static_cast<int64_t>((now + 62135596800.0) * 1e7);  // ~6.39e17
  EXPECT_EQ(infer_unit(1e7).name, "100ns");
  EXPECT_EQ(classify_time_base(ticks, 1e7, now), "dotnet_epoch");
  // DateTime.Now is local time (KST = UTC+9): still inside the 2-day window.
  EXPECT_EQ(classify_time_base(ticks + INT64_C(9) * 3600 * 10000000, 1e7, now), "dotnet_epoch");
  // Ticks converted to ms keep the 0001-01-01 origin.
  EXPECT_EQ(
    classify_time_base(static_cast<int64_t>((now + 62135596800.0) * 1e3), 1e3, now),
    "dotnet_epoch");
  // A Unix-epoch value is not misread as .NET.
  EXPECT_EQ(classify_time_base(static_cast<int64_t>(now * 1e7), 1e7, now), "unix_epoch");
}

TEST(ProbeStats, CheckedSubDetectsOverflow)
{
  int64_t d = 42;
  EXPECT_TRUE(checked_sub(5, 7, &d));
  EXPECT_EQ(d, -2);
  EXPECT_FALSE(checked_sub(std::numeric_limits<int64_t>::min(), 1, &d));
  EXPECT_FALSE(checked_sub(std::numeric_limits<int64_t>::max(), -5, &d));
  EXPECT_EQ(d, -2);  // untouched on overflow
}

// =============================================================================
// Frame-period estimation
// =============================================================================

TEST(ProbeFrames, HundredHzMillisecondsWithDuplicates)
{
  // 100 Hz frames, ms timestamps, polled every 0.4 ms (~0.35 ms RTT) for 3 s.
  const auto s = poll(uniform_frames(0.010, 10, 5000, 3.1), 0.0004, 3.0);
  const FrameAnalysis a = analyze_frames(s);

  EXPECT_EQ(a.samples, s.size());
  EXPECT_EQ(a.monotonic_violations, 0u);
  EXPECT_NEAR(static_cast<double>(a.changes), 299.0, 1.0);
  EXPECT_NEAR(a.duplicate_ratio, 1.0 - 300.0 / 7500.0, 0.01);
  EXPECT_EQ(a.dvalue_mode_raw, 10);
  EXPECT_DOUBLE_EQ(a.dvalue_mode_share, 1.0);
  EXPECT_DOUBLE_EQ(a.dvalue_raw.p50, 10.0);
  EXPECT_DOUBLE_EQ(a.frame_period_ms_assuming_ms, 10.0);

  EXPECT_EQ(a.unit.name, "ms");
  EXPECT_TRUE(a.unit.plausible);
  EXPECT_NEAR(a.unit.rate_ratio, 1.0, 0.01);
  EXPECT_NEAR(a.frame_period_s, 0.010, 1e-9);
  EXPECT_NEAR(a.frame_rate_hz, 100.0, 1e-6);

  EXPECT_NEAR(a.change_interval_ms.p50, 10.0, 0.5);
  EXPECT_EQ(a.change_intervals_over_20ms, 0u);
  EXPECT_EQ(a.change_intervals_over_45ms, 0u);
  EXPECT_NEAR(a.changes_per_wall_s, 100.0, 1.0);
  EXPECT_NEAR(a.changes_per_second_bucket.p50, 100.0, 1.0);

  EXPECT_TRUE(a.tick_present);
  EXPECT_DOUBLE_EQ(a.dtick_per_change.p50, 1.0);
  EXPECT_EQ(a.tick_changes_without_value_change, 0u);

  // Sampling supports the value: ~25 polls per frame, one step size.
  EXPECT_NEAR(a.poll_interval_ms.p50, 0.4, 1e-6);
  EXPECT_FALSE(a.undersampled);
  EXPECT_FALSE(a.poll_limited);
  EXPECT_FALSE(a.resolution_limited);
  EXPECT_EQ(a.dvalue_gcd_raw, 10);
  EXPECT_TRUE(a.deltas_consistent_with_mode);
  EXPECT_FALSE(a.deltas_are_multiples);
  EXPECT_TRUE(a.frame_period_reliable);
  EXPECT_TRUE(a.warnings.empty());
  EXPECT_NEAR(a.sim_step_s, 0.010, 1e-12);
  EXPECT_NEAR(a.wall_frame_rate_hz, 100.0, 0.5);
  EXPECT_EQ(a.missed_frame_events, 0u);
  EXPECT_EQ(a.multi_frame_changes, 0u);
  ASSERT_EQ(a.dvalue_hist_top.size(), 1u);
  EXPECT_EQ(a.dvalue_hist_top.front().dvalue, 10);
}

TEST(ProbeFrames, FiftyMillisecondStallIsReportedNotAveragedAway)
{
  // 100 Hz, but the frame due at 1.00 s arrives at 1.05 s, and sim time only advances one
  // frame across the stall (the loop paused). Median period must stay 10 ms; the stall
  // must appear as a >45 ms change interval with a +10 timestamp step.
  std::vector<Frame> fr;
  int64_t v = 0;
  uint64_t tick = 0;
  for (double t = 0.0; t < 0.995; t += 0.010) {
    fr.push_back(Frame{t, v, tick});
    v += 10;
    ++tick;
  }
  for (double t = 1.050; t < 2.1; t += 0.010) {
    fr.push_back(Frame{t, v, tick});
    v += 10;
    ++tick;
  }
  const FrameAnalysis a = analyze_frames(poll(fr, 0.0004, 2.0));

  EXPECT_DOUBLE_EQ(a.dvalue_raw.p50, 10.0);
  EXPECT_NEAR(a.frame_rate_hz, 100.0, 1e-6);  // sim-time period, unaffected by the stall
  EXPECT_EQ(a.change_intervals_over_45ms, 1u);
  ASSERT_FALSE(a.largest_gaps.empty());
  EXPECT_NEAR(a.largest_gaps.front().gap_ms, 60.0, 0.5);  // 0.99 -> 1.05
  EXPECT_EQ(a.largest_gaps.front().dvalue, 10);
  EXPECT_EQ(a.largest_gaps.front().dtick, 1);
  EXPECT_NEAR(a.largest_gaps.front().wall_s, 1.05, 0.001);
  // Sim time lost 50 ms over ~2 s of wall time.
  EXPECT_LT(a.unit.rate_ratio, 1.0);
  EXPECT_TRUE(a.unit.plausible);
  EXPECT_EQ(a.change_interval_ms.max, a.largest_gaps.front().gap_ms);
  // Relative counting: one missed-frame event (a wall stall, sim advanced one frame).
  EXPECT_TRUE(a.frame_period_reliable);
  EXPECT_EQ(a.wall_stall_events, 1u);
  EXPECT_EQ(a.multi_frame_changes, 0u);
  EXPECT_EQ(a.missed_frame_events, 1u);
  EXPECT_EQ(a.largest_gaps.front().frames, 1);
  EXPECT_NEAR(a.max_gap_in_frames, 6.0, 0.5);  // 60 ms at ~10 ms per frame
}

TEST(ProbeFrames, NonMonotonicSampleIsCountedAndExcludedFromDeltas)
{
  const std::vector<TimedSample> s{{0.000, 100, 0}, {0.010, 110, 0},
                                   {0.011, 110, 0}, {0.020, 105, 0},  // goes backwards
                                   {0.030, 120, 0}, {0.040, 130, 0}};
  const FrameAnalysis a = analyze_frames(s);
  EXPECT_EQ(a.monotonic_violations, 1u);
  EXPECT_EQ(a.duplicates, 1u);
  EXPECT_EQ(a.changes, 4u);
  EXPECT_EQ(a.dvalue_raw.n, 3u);  // +10, +15, +10 ; the -5 is not a period
  EXPECT_DOUBLE_EQ(a.dvalue_raw.min, 10.0);
  EXPECT_DOUBLE_EQ(a.dvalue_raw.max, 15.0);
  EXPECT_EQ(a.dvalue_mode_raw, 10);
  EXPECT_FALSE(a.tick_present);
}

TEST(ProbeFrames, MicrosecondTimestampsAreNotMistakenForMilliseconds)
{
  // Same 100 Hz clock, but in microseconds. "Assuming ms" would read 10 s frames.
  const auto s = poll(uniform_frames(0.010, 10000, 0, 2.1), 0.0004, 2.0);
  const FrameAnalysis a = analyze_frames(s);
  EXPECT_EQ(a.unit.name, "us");
  EXPECT_TRUE(a.unit.plausible);
  EXPECT_DOUBLE_EQ(a.frame_period_ms_assuming_ms, 10000.0);
  EXPECT_NEAR(a.frame_period_s, 0.010, 1e-9);
  EXPECT_NEAR(a.frame_rate_hz, 100.0, 1e-6);
}

TEST(ProbeFrames, TwoHundredHzAndSlowPollingSeeMultiplesOfThePeriod)
{
  // 200 Hz frames polled every 12 ms: changes skip frames, so the advance per change is
  // 10 or 15 ms, never 5. The median gap (10 ms, "100 Hz") must NOT come out as a
  // plausible frame period; the gcd must expose the 5 ms step.
  const auto s = poll(uniform_frames(0.005, 5, 0, 3.1), 0.012, 3.0);
  const FrameAnalysis a = analyze_frames(s);
  EXPECT_EQ(a.unit.name, "ms");
  EXPECT_NEAR(a.unit.rate_ratio, 1.0, 0.02);
  EXPECT_GE(a.dvalue_raw.min, 10.0);
  EXPECT_EQ(a.duplicates, 0u);

  EXPECT_TRUE(a.undersampled);
  EXPECT_TRUE(a.poll_limited);
  EXPECT_EQ(a.dvalue_mode_raw, 10);
  EXPECT_EQ(a.dvalue_gcd_raw, 5);
  EXPECT_TRUE(a.deltas_are_multiples);
  EXPECT_NEAR(a.gcd_step_s, 0.005, 1e-12);
  EXPECT_FALSE(a.frame_period_reliable);
  EXPECT_TRUE(std::isnan(a.frame_period_s));
  EXPECT_TRUE(std::isnan(a.frame_rate_hz));
  EXPECT_GE(a.warnings.size(), 3u);
}

TEST(ProbeFrames, PollingAtExactlyTwoFramesIsFlaggedNotBelieved)
{
  // 200 Hz polled every 10 ms: every delta is 10, gcd == mode, so nothing in the deltas
  // betrays the aliasing. Only the sampling checks can: 1 poll per change.
  const auto s = poll(uniform_frames(0.005, 5, 0, 3.1), 0.010, 3.0);
  const FrameAnalysis a = analyze_frames(s);
  EXPECT_EQ(a.dvalue_mode_raw, 10);
  EXPECT_EQ(a.dvalue_gcd_raw, 10);
  EXPECT_FALSE(a.deltas_are_multiples);
  EXPECT_NEAR(a.frame_rate_hz_unchecked, 100.0, 1e-6);  // the aliased, wrong answer
  EXPECT_TRUE(a.undersampled);
  EXPECT_TRUE(a.poll_limited);
  EXPECT_FALSE(a.frame_period_reliable);
  EXPECT_TRUE(std::isnan(a.frame_rate_hz));
}

TEST(ProbeFrames, RenderRateStampsOf200HzPhysicsExposeTheGcd)
{
  // 200 Hz physics, but the timestamp is refreshed once per 60 Hz render frame: sim time
  // advances 15, 15, 20 ms per update. Polled fast (no aliasing), yet the mode (15 ms,
  // "66.7 Hz") is not the physics step.
  std::vector<Frame> fr;
  for (int64_t i = 0; i <= 200; ++i) {
    fr.push_back(Frame{static_cast<double>(i) / 60.0, 5 * (i * 10 / 3), 0});
  }
  const FrameAnalysis a = analyze_frames(poll(fr, 0.0004, 3.0));
  EXPECT_FALSE(a.undersampled);
  EXPECT_FALSE(a.poll_limited);
  EXPECT_EQ(a.dvalue_mode_raw, 15);
  EXPECT_EQ(a.dvalue_gcd_raw, 5);
  EXPECT_FALSE(a.deltas_consistent_with_mode);
  EXPECT_TRUE(a.deltas_are_multiples);
  EXPECT_EQ(a.unit.name, "ms");
  EXPECT_NEAR(a.gcd_step_s, 0.005, 1e-12);       // physics-step candidate: 200 Hz
  EXPECT_NEAR(a.changes_per_wall_s, 60.0, 0.5);  // observed updates
  EXPECT_NEAR(a.frame_rate_hz_unchecked, 60.0, 0.5);
  EXPECT_FALSE(a.frame_period_reliable);
  EXPECT_TRUE(std::isnan(a.frame_rate_hz));
}

TEST(ProbeFrames, ContinuouslyAdvancingClockIsNotAFramePeriod)
{
  // (a) Wall-clock microseconds polled every 0.4 ms: every poll sees a new value, the
  //     "period" is the polling period.
  std::vector<TimedSample> us;
  for (int k = 0; k < 2500; ++k) {
    const double t = k * 0.0004;
    us.push_back(TimedSample{t, std::llround(t * 1e6), 0});
  }
  const FrameAnalysis a = analyze_frames(us);
  EXPECT_EQ(a.unit.name, "us");
  EXPECT_TRUE(a.unit.plausible);
  EXPECT_TRUE(a.poll_limited);
  EXPECT_TRUE(a.undersampled);
  EXPECT_FALSE(a.frame_period_reliable);
  EXPECT_TRUE(std::isnan(a.frame_rate_hz));

  // (b) Wall-clock milliseconds polled every 0.4 ms: changes every 1 ms look like a
  //     1000 Hz frame clock. The 1-raw-unit step is the tell.
  std::vector<TimedSample> ms;
  for (int k = 0; k < 5000; ++k) {
    const double t = k * 0.0004;
    ms.push_back(TimedSample{t, static_cast<int64_t>(std::floor(t * 1e3 + 1e-9)), 0});
  }
  const FrameAnalysis b = analyze_frames(ms);
  EXPECT_EQ(b.unit.name, "ms");
  EXPECT_EQ(b.dvalue_mode_raw, 1);
  EXPECT_FALSE(b.poll_limited);
  EXPECT_FALSE(b.undersampled);
  EXPECT_TRUE(b.resolution_limited);
  EXPECT_FALSE(b.frame_period_reliable);
  EXPECT_TRUE(std::isnan(b.frame_rate_hz));
}

TEST(ProbeFrames, FiftyHzRigWithNonIntegerStep)
{
  // The older rig: 49.9 Hz. Sim step 20.04 ms, integer-ms timestamps -> deltas 20 and 21.
  // U1 must report 20.04 ms / 49.9 Hz, not "20 ms (50 Hz)", and the 21s must not be
  // mistaken for a finer step (gcd 1).
  std::vector<Frame> fr;
  for (int64_t i = 0; i <= 520; ++i) {
    fr.push_back(Frame{static_cast<double>(i) * 0.02004, std::llround(i * 20.04), 0});
  }
  // 0.37 ms polls: observed intervals are 54 or 55 polls (19.98 / 20.35 ms), no ties at 20.
  const FrameAnalysis a = analyze_frames(poll(fr, 0.00037, 10.0));
  EXPECT_EQ(a.dvalue_mode_raw, 20);
  EXPECT_EQ(a.dvalue_gcd_raw, 1);
  EXPECT_TRUE(a.deltas_consistent_with_mode);
  EXPECT_FALSE(a.deltas_are_multiples);
  EXPECT_FALSE(a.resolution_limited);
  EXPECT_TRUE(a.frame_period_reliable);
  EXPECT_NEAR(a.sim_step_s * 1e3, 20.04, 0.01);
  EXPECT_NEAR(a.frame_period_s * 1e3, 20.04, 0.01);
  EXPECT_NEAR(a.wall_frame_rate_hz, 49.90, 0.03);
  EXPECT_NEAR(a.changes_per_wall_s, 49.90, 0.03);
  // Absolute 20 ms thresholds flag most normal frames; relative counting flags none.
  EXPECT_GT(a.change_intervals_over_20ms, 40u);
  EXPECT_EQ(a.wall_stall_events, 0u);
  EXPECT_EQ(a.missed_frame_events, 0u);
}

TEST(ProbeFrames, TwentyMsStepAtSlightlyBelowRealTime)
{
  // Sim step exactly 20 ms, simulator at 0.998x real time: 1/sim_step = 50 Hz, but frames
  // arrive at 49.9 Hz of wall time. Both must be visible.
  std::vector<Frame> fr;
  for (int64_t i = 0; i <= 520; ++i) {
    fr.push_back(Frame{static_cast<double>(i) * 0.020 / 0.998, 20 * i, 0});
  }
  const FrameAnalysis a = analyze_frames(poll(fr, 0.0004, 10.0));
  EXPECT_TRUE(a.frame_period_reliable);
  EXPECT_NEAR(a.sim_step_s, 0.020, 1e-12);
  EXPECT_NEAR(a.frame_rate_hz, 50.0, 1e-9);
  EXPECT_NEAR(a.wall_frame_rate_hz, 49.90, 0.03);
  EXPECT_NEAR(a.unit.rate_ratio, 0.998, 0.001);
}

TEST(ProbeFrames, HitchesAt200HzAreCountedRelativeToThePeriod)
{
  // 200 Hz. A 10 ms hitch (one frame, late) and a 15 ms catch-up (sim +3 frames at once).
  // Neither crosses the absolute 20/45 ms thresholds; both are missed-frame events.
  std::vector<Frame> fr;
  double t = 0.0;
  int64_t v = 0;
  for (int i = 0; i <= 420; ++i) {
    fr.push_back(Frame{t, v, 0});
    if (i == 100) {
      t += 0.010;
      v += 5;
    } else if (i == 200) {
      t += 0.015;
      v += 15;
    } else {
      t += 0.005;
      v += 5;
    }
  }
  const FrameAnalysis a = analyze_frames(poll(fr, 0.0004, 2.0));
  EXPECT_TRUE(a.frame_period_reliable);
  EXPECT_EQ(a.dvalue_mode_raw, 5);
  EXPECT_NEAR(a.sim_step_s, 0.005, 1e-12);
  EXPECT_EQ(a.change_intervals_over_20ms, 0u);
  EXPECT_EQ(a.change_intervals_over_45ms, 0u);
  EXPECT_EQ(a.wall_stall_events, 2u);
  EXPECT_EQ(a.multi_frame_changes, 1u);
  EXPECT_EQ(a.missed_frame_events, 2u);
  EXPECT_EQ(a.max_frames_per_change, 3);
  ASSERT_EQ(a.frames_per_change_hist.count(3), 1u);
  EXPECT_EQ(a.frames_per_change_hist.at(3), 1u);
  EXPECT_NEAR(a.max_gap_in_frames, 3.0, 0.2);
  EXPECT_NEAR(a.missed_frame_events_per_min, 60.0, 0.5);  // 2 events in ~2 s
}

TEST(ProbeFrames, OverflowingTimestampDeltasAreCountedNotComputed)
{
  // Pathological server values: a negative sentinel next to INT64_MAX, then INT64_MIN.
  const std::vector<TimedSample> s{
    {0.000, -5, 0},
    {0.010, std::numeric_limits<int64_t>::max(), 0},
    {0.020, std::numeric_limits<int64_t>::min(), 0},
    {0.030, std::numeric_limits<int64_t>::min() + 10, 0}};
  const FrameAnalysis a = analyze_frames(s);
  EXPECT_EQ(a.changes, 3u);
  EXPECT_EQ(a.delta_overflows, 2u);
  EXPECT_EQ(a.monotonic_violations, 1u);
  EXPECT_EQ(a.dvalue_raw.n, 1u);
  EXPECT_DOUBLE_EQ(a.dvalue_raw.min, 10.0);
  EXPECT_TRUE(std::isfinite(a.raw_per_wall_s));
  EXPECT_FALSE(a.frame_period_reliable);
}

TEST(ProbeFrames, EmptyAndConstantInputs)
{
  const FrameAnalysis e = analyze_frames({});
  EXPECT_EQ(e.samples, 0u);
  EXPECT_EQ(e.unit.name, "unknown");
  EXPECT_TRUE(std::isnan(e.frame_rate_hz));

  // Timestamp never moves (e.g. field not populated): no period, all duplicates.
  std::vector<TimedSample> c;
  for (int i = 0; i < 100; ++i) {
    c.push_back(TimedSample{i * 0.001, 0, 0});
  }
  const FrameAnalysis z = analyze_frames(c);
  EXPECT_EQ(z.changes, 0u);
  EXPECT_EQ(z.zero_values, 100u);
  EXPECT_DOUBLE_EQ(z.duplicate_ratio, 1.0);
  EXPECT_EQ(z.unit.name, "unknown");
  EXPECT_TRUE(std::isnan(z.frame_period_s));
}

// =============================================================================
// WaitForTick classification
// =============================================================================

TEST(ProbeWaitForTick, Classification)
{
  WaitForTickObservation o;
  o.calls = 200;
  o.ok = 200;
  o.deadline_ms = 200;
  o.rtt_ms_p50 = 0.35;
  o.frame_period_ms = 10.0;

  o.call_ms_p50 = 9.6;
  o.interval_ms_p50 = 10.0;
  EXPECT_EQ(classify_wait_for_tick(o), "per_frame");

  o.interval_ms_p50 = 20.0;
  o.call_ms_p50 = 19.6;
  EXPECT_EQ(classify_wait_for_tick(o), "multi_frame");

  o.interval_ms_p50 = 5.0;
  o.call_ms_p50 = 4.6;
  EXPECT_EQ(classify_wait_for_tick(o), "sub_frame");

  o.call_ms_p50 = 0.4;  // ~RTT: does not wait for anything
  o.interval_ms_p50 = 0.4;
  EXPECT_EQ(classify_wait_for_tick(o), "returns_immediately");

  o.call_ms_p50 = 9.6;
  o.interval_ms_p50 = 10.0;
  o.frame_period_ms = kNaN;
  EXPECT_EQ(classify_wait_for_tick(o), "blocks_period_unknown");

  // Deadline shorter than the tick wait: OK and DEADLINE_EXCEEDED alternate, so no
  // interval exists. Must not be mistaken for "multi_frame".
  o.frame_period_ms = 10.0;
  o.call_ms_p50 = 4.7;
  o.interval_ms_p50 = kNaN;
  o.deadline_exceeded = 100;
  o.ok = 100;
  EXPECT_EQ(classify_wait_for_tick(o), "blocks_interval_unknown");
  o.deadline_exceeded = 0;
  o.ok = 200;

  o.ok = 0;
  o.deadline_exceeded = 10;
  o.calls = 10;
  EXPECT_EQ(classify_wait_for_tick(o), "no_ok_deadline");

  o.deadline_exceeded = 0;
  EXPECT_EQ(classify_wait_for_tick(o), "no_ok_error");

  o.calls = 0;
  EXPECT_EQ(classify_wait_for_tick(o), "no_calls");
}

// =============================================================================
// JSON
// =============================================================================

TEST(ProbeJson, EscapingNumbersAndNesting)
{
  Json j = Json::object()
             .set("s", "a\"b\\c\nd\x01")
             .set("nan", kNaN)
             .set("i64", INT64_C(1790000000123456789))
             .set("u64", UINT64_MAX)
             .set("d", 0.35)
             .set("b", true)
             .set("arr", Json::array().push(1).push(2.5).push(Json()))
             .set("raw", Json::raw("{\"x\":1}"));
  const std::string out = j.dump(0);
  EXPECT_EQ(
    out,
    "{\"s\":\"a\\\"b\\\\c\\nd\\u0001\",\"nan\":null,\"i64\":1790000000123456789,"
    "\"u64\":18446744073709551615,\"d\":0.35,\"b\":true,\"arr\":[1,2.5,null],"
    "\"raw\":{\"x\":1}}");

  // set() overwrites an existing key instead of duplicating it.
  j.set("b", false);
  EXPECT_NE(j.dump(0).find("\"b\":false"), std::string::npos);
  EXPECT_EQ(j.dump(0).find("\"b\":true"), std::string::npos);
  ASSERT_NE(j.find("d"), nullptr);
  EXPECT_DOUBLE_EQ(j.find("d")->as_double(), 0.35);
  EXPECT_EQ(j.find("missing"), nullptr);
}

TEST(ProbeJson, RawTableFormatsRowsAndCapsThem)
{
  using Cell = morai_grpc_vendor::probe::RawTable::Cell;
  morai_grpc_vendor::probe::RawTable t({"t_ms", "ts", "v"}, 2);
  t.add({Cell::d(0.5), Cell::i(INT64_C(1790000000123456789)), Cell::null()});
  t.add({Cell::d(kNaN), Cell::u(UINT64_MAX), Cell::i(-3)});
  t.add({Cell::d(1.0), Cell::i(1), Cell::i(2)});  // over the cap: counted, not stored
  EXPECT_EQ(t.rows(), 2u);
  EXPECT_EQ(t.dropped(), 1u);
  EXPECT_EQ(
    t.to_json().dump(0),
    "{\"columns\":[\"t_ms\",\"ts\",\"v\"],\"rows\":[[0.5,1790000000123456789,null],"
    "[null,18446744073709551615,-3]],\"dropped_rows\":1}");
}

}  // namespace
