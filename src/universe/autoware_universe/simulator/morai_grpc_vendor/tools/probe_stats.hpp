// 신규 — 26.09.24 _ YSH _ morai_probe 순수 계산부 (통계·프레임 주기·단위 추정)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_STATS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_STATS_HPP_

#include "probe_json.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace morai_grpc_vendor::probe
{

// =============================================================================
// Descriptive statistics
// =============================================================================

/// Percentile of an ascending-sorted sample, linear interpolation between closest ranks
/// (numpy's default "linear" / Hyndman-Fan type 7). p in [0, 100]. NaN if empty.
double percentile_sorted(const std::vector<double> & sorted, double p);

struct Summary
{
  size_t n{0};
  double mean{0.0};
  double min{0.0};
  double p50{0.0};
  double p90{0.0};
  double p99{0.0};
  double max{0.0};
};

/// All fields NaN (n = 0) for an empty input.
Summary summarize(std::vector<double> values);

size_t count_over(const std::vector<double> & values, double threshold);

Json to_json(const Summary & s);

/// grpc::StatusCode integer -> canonical name ("OK", "DEADLINE_EXCEEDED", ...).
/// Kept here, free of <grpcpp>, so it is unit-testable.
std::string grpc_code_name(int code);

/// a - b without signed-overflow UB. False (and *out untouched) on overflow. Server
/// timestamps are untrusted int64 of unknown unit and origin (a negative sentinel next to
/// an epoch-ns value overflows a plain subtraction).
bool checked_sub(int64_t a, int64_t b, int64_t * out);

// =============================================================================
// Timestamp unit inference
// =============================================================================

struct UnitGuess
{
  /// "s", "ms", "us", "100ns" (.NET DateTime ticks — MORAI is Unity/C#), "ns" or
  /// "unknown".
  std::string name{"unknown"};
  /// Raw units per second of the chosen unit; NaN if unknown.
  double per_second{0.0};
  /// Observed sim-time / wall-time ratio under that unit (1.0 = real time).
  double rate_ratio{0.0};
  /// rate_ratio in [0.5, 2.0]. Outside that the unit is a guess, not a finding.
  bool plausible{false};
};

/// Pick the unit whose scale is closest (in log10) to the observed raw-units per
/// wall-second. A simulator running at 0.8x real time with ms timestamps yields 800 raw/s
/// -> "ms", rate_ratio 0.8.
UnitGuess infer_unit(double raw_per_wall_second);

/// Seconds between 0001-01-01 (.NET DateTime epoch) and 1970-01-01.
constexpr double kDotNetEpochToUnixS = 62135596800.0;

/// "unix_epoch" if raw/per_second is within 2 days of now_unix_s, "dotnet_epoch" if it is
/// within 2 days of now counted from 0001-01-01 (DateTime.Ticks and derivatives; 2 days
/// also absorbs DateTime.Now's local-time offset), "elapsed" if it is a small
/// non-negative duration (< 30 days), else "unknown".
std::string classify_time_base(int64_t raw, double per_second, double now_unix_s);

// =============================================================================
// Frame-period estimation from polled timestamps
// =============================================================================

/// One polled response.
struct TimedSample
{
  double wall_s{0.0};  ///< steady clock, seconds (any origin); midpoint of the RPC
  int64_t value{0};    ///< raw timestamp as returned by MORAI
  uint64_t tick{0};    ///< ActorStates.sync_mode_tick_count (0 where not applicable)
};

/// A wall-clock gap between two successive timestamp CHANGES.
struct GapEvent
{
  double wall_s{0.0};  ///< wall time of the change that ended the gap
  double gap_ms{0.0};  ///< wall time since the previous change
  int64_t dvalue{0};   ///< timestamp advance across the gap (raw units; 0 if it overflowed)
  int64_t dtick{0};    ///< tick advance across the gap
  int64_t frames{0};   ///< round(dvalue / delta mode); 0 for a non-positive advance
};

struct DeltaCount
{
  int64_t dvalue{0};
  size_t count{0};
};

struct FrameAnalysis
{
  size_t samples{0};
  /// Number of timestamp changes (sample value != previous sample value).
  size_t changes{0};
  /// Samples whose value equals the previous sample's value.
  size_t duplicates{0};
  double duplicate_ratio{0.0};
  /// Samples whose value is LOWER than the previous sample's value.
  size_t monotonic_violations{0};
  /// Changes whose raw difference does not fit in int64 (excluded from the deltas).
  size_t delta_overflows{0};
  size_t zero_values{0};
  /// First to last sample: the observation window every per-minute rate refers to.
  double wall_span_s{0.0};
  /// Wall time between successive polls.
  Summary poll_interval_ms;

  /// Positive advances between successive distinct timestamps, raw units.
  Summary dvalue_raw;
  int64_t dvalue_mode_raw{0};
  double dvalue_mode_share{0.0};
  /// Delta histogram, most frequent first, at most 10 entries.
  std::vector<DeltaCount> dvalue_hist_top;
  /// GCD of the deltas that make up >= 1% of the positive deltas.
  int64_t dvalue_gcd_raw{0};
  /// Every such delta is within quantisation (max(1 raw, 1%)) of k x mode.
  bool deltas_consistent_with_mode{false};
  /// !deltas_consistent_with_mode: the frequent deltas share a step finer than the mode
  /// (e.g. 15/20 ms -> gcd 5 ms). Either the timestamp is refreshed less often than the
  /// physics step, or polling skipped frames. The gcd is then a physics-step candidate.
  bool deltas_are_multiples{false};
  /// dvalue_gcd_raw converted with `unit`, when deltas_are_multiples; NaN otherwise.
  double gcd_step_s{0.0};

  /// Frames per change, k = max(1, round(dvalue / mode)), over changes after the first.
  Summary frames_per_change;
  std::map<int64_t, size_t> frames_per_change_hist;
  int64_t max_frames_per_change{0};
  size_t multi_frame_changes{0};  ///< k >= 2

  /// Wall time between successive changes, ms. This is where MORAI stalls show up.
  Summary change_interval_ms;
  /// Absolute thresholds; secondary, not comparable across Hz settings.
  size_t change_intervals_over_20ms{0};
  size_t change_intervals_over_45ms{0};
  /// Relative to the measured wall period: a change whose wall gap exceeds 1.5 periods.
  size_t wall_stall_events{0};
  /// Changes with a wall stall OR more than one frame of sim-time advance.
  size_t missed_frame_events{0};
  double missed_frame_events_per_min{0.0};
  double wall_stall_events_per_min{0.0};
  /// max over changes of max(k, gap_ms / wall_period_ms).
  double max_gap_in_frames{0.0};
  std::vector<GapEvent> largest_gaps;  ///< descending by gap_ms

  /// (changes - 1) / (w_last_change - w_first_change): observed update rate.
  double changes_per_wall_s{0.0};
  /// Changes per whole wall-clock second bucket (partial last bucket dropped).
  Summary changes_per_second_bucket;

  /// (v_last_change - v_first_change) / (w_last_change - w_first_change).
  double raw_per_wall_s{0.0};
  UnitGuess unit;

  /// Sim time per frame: sum(dvalue) / sum(k), raw units and seconds.
  double sim_step_raw{0.0};
  double sim_step_s{0.0};
  /// Wall-clock frame rate: sum(k) / (w_last_change - w_first_change). In real-time mode
  /// this, not 1/sim_step, is how often a command must arrive.
  double wall_frame_rate_hz{0.0};
  double wall_period_ms{0.0};

  /// Fewer than 2 polls per change (duplicate_ratio < 0.5): the delta mode may be a
  /// multiple of the true step.
  bool undersampled{false};
  /// Median change interval <= 1.5 x median poll interval: every poll sees a new value,
  /// so the "period" may be the polling period (or a continuously advancing clock).
  bool poll_limited{false};
  /// The delta mode is 1 raw unit: a continuously advancing clock cannot be ruled out.
  bool resolution_limited{false};
  /// None of the above, unit plausible, >= 3 changes, deltas consistent with the mode.
  bool frame_period_reliable{false};
  std::vector<std::string> warnings;

  /// sim_step_s / 1/sim_step_s when frame_period_reliable, else NaN. Never hand an
  /// unchecked value to a consumer.
  double frame_period_s{0.0};
  double frame_rate_hz{0.0};
  /// The same values regardless of reliability (for inspection only).
  double frame_period_s_unchecked{0.0};
  double frame_rate_hz_unchecked{0.0};
  /// median(dvalue_raw) read as milliseconds, regardless of the inferred unit.
  double frame_period_ms_assuming_ms{0.0};

  bool tick_present{false};
  /// Tick advance per timestamp change.
  Summary dtick_per_change;
  /// Tick changed while the timestamp did not.
  size_t tick_changes_without_value_change{0};
};

/// Samples are taken in polling order; they need not be uniformly spaced.
FrameAnalysis analyze_frames(const std::vector<TimedSample> & samples, size_t top_gaps = 10);

Json to_json(const FrameAnalysis & a);

// =============================================================================
// WaitForTick behaviour classification
// =============================================================================

struct WaitForTickObservation
{
  size_t calls{0};
  size_t ok{0};
  size_t deadline_exceeded{0};
  double deadline_ms{0.0};
  /// Median duration of a successful call, ms.
  double call_ms_p50{0.0};
  /// Median wall interval between successive successful returns, ms.
  double interval_ms_p50{0.0};
  /// Median plain unary RTT (e.g. GetActorState), ms. NaN if unknown.
  double rtt_ms_p50{0.0};
  /// Frame period from another measurement, ms. NaN if unknown.
  double frame_period_ms{0.0};
};

/// One of:
///   "no_ok_deadline"     every call hit the deadline — blocks longer than the deadline
///                        (plausibly only ticks in synchronous mode)
///   "no_ok_error"        no call succeeded, for another reason
///   "returns_immediately" successful calls take ~RTT; not a frame wait
///   "per_frame"          returns once per frame (interval ~ frame period)
///   "multi_frame"        blocks, but returns less often than every frame
///   "sub_frame"          blocks, but returns more often than the frame period
///   "blocks_interval_unknown" blocks, but no two successive calls succeeded
///   "blocks_period_unknown" blocks, frame period unknown
///   "no_calls"
std::string classify_wait_for_tick(const WaitForTickObservation & o);

}  // namespace morai_grpc_vendor::probe

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_STATS_HPP_
