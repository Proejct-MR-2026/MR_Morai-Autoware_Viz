// 신규 — 26.09.24 _ YSH _ probe_stats.hpp 구현 (gRPC·protobuf 비의존)
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

#include "probe_stats.hpp"

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <map>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::probe
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Summary nan_summary()
{
  Summary s;
  s.n = 0;
  s.mean = s.min = s.p50 = s.p90 = s.p99 = s.max = kNaN;
  return s;
}

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));

std::string format(const char * fmt, ...)
{
  char buf[512];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return buf;
}

/// k = max(1, round(d / mode)) for a positive advance d, clamped so that llround never
/// sees a value outside long long.
int64_t frames_for(int64_t d, int64_t mode)
{
  const double r = static_cast<double>(d) / static_cast<double>(mode);
  if (!(r < 1e15)) {
    return static_cast<int64_t>(1e15);
  }
  return std::max<int64_t>(1, static_cast<int64_t>(std::llround(r)));
}

/// A change: the sample at `idx` differs from the sample before it.
struct Change
{
  size_t idx{0};
  bool positive{false};  ///< d computed without overflow and > 0
  int64_t d{0};          ///< 0 if the subtraction overflowed
  int64_t dtick{0};
};
}  // namespace

// =============================================================================
// Descriptive statistics
// =============================================================================

double percentile_sorted(const std::vector<double> & sorted, double p)
{
  if (sorted.empty()) {
    return kNaN;
  }
  if (sorted.size() == 1) {
    return sorted.front();
  }
  const double clamped = std::min(100.0, std::max(0.0, p));
  const double rank = clamped / 100.0 * static_cast<double>(sorted.size() - 1);
  const auto lo = static_cast<size_t>(std::floor(rank));
  const size_t hi = std::min(lo + 1, sorted.size() - 1);
  const double frac = rank - static_cast<double>(lo);
  return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
}

Summary summarize(std::vector<double> values)
{
  if (values.empty()) {
    return nan_summary();
  }
  std::sort(values.begin(), values.end());
  Summary s;
  s.n = values.size();
  double sum = 0.0;
  for (const double v : values) {
    sum += v;
  }
  s.mean = sum / static_cast<double>(values.size());
  s.min = values.front();
  s.max = values.back();
  s.p50 = percentile_sorted(values, 50.0);
  s.p90 = percentile_sorted(values, 90.0);
  s.p99 = percentile_sorted(values, 99.0);
  return s;
}

size_t count_over(const std::vector<double> & values, double threshold)
{
  return static_cast<size_t>(
    std::count_if(values.begin(), values.end(), [threshold](double v) { return v > threshold; }));
}

Json to_json(const Summary & s)
{
  return Json::object()
    .set("n", s.n)
    .set("mean", s.mean)
    .set("min", s.min)
    .set("p50", s.p50)
    .set("p90", s.p90)
    .set("p99", s.p99)
    .set("max", s.max);
}

std::string grpc_code_name(int code)
{
  static const std::array<const char *, 17> kNames = {
    "OK",        "CANCELLED",      "UNKNOWN",           "INVALID_ARGUMENT",   "DEADLINE_EXCEEDED",
    "NOT_FOUND", "ALREADY_EXISTS", "PERMISSION_DENIED", "RESOURCE_EXHAUSTED", "FAILED_PRECONDITION",
    "ABORTED",   "OUT_OF_RANGE",   "UNIMPLEMENTED",     "INTERNAL",           "UNAVAILABLE",
    "DATA_LOSS", "UNAUTHENTICATED"};
  if (code >= 0 && static_cast<size_t>(code) < kNames.size()) {
    return kNames[static_cast<size_t>(code)];
  }
  return "CODE_" + std::to_string(code);
}

bool checked_sub(int64_t a, int64_t b, int64_t * out)
{
  int64_t r = 0;
  if (__builtin_sub_overflow(a, b, &r)) {
    return false;
  }
  *out = r;
  return true;
}

// =============================================================================
// Timestamp unit inference
// =============================================================================

UnitGuess infer_unit(double raw_per_wall_second)
{
  UnitGuess g;
  if (!std::isfinite(raw_per_wall_second) || raw_per_wall_second <= 0.0) {
    g.name = "unknown";
    g.per_second = kNaN;
    g.rate_ratio = kNaN;
    g.plausible = false;
    return g;
  }
  static const std::array<std::pair<const char *, double>, 5> kUnits = {
    {{"s", 1.0}, {"ms", 1e3}, {"us", 1e6}, {"100ns", 1e7}, {"ns", 1e9}}};
  const double lx = std::log10(raw_per_wall_second);
  double best = std::numeric_limits<double>::infinity();
  for (const auto & u : kUnits) {
    const double d = std::fabs(lx - std::log10(u.second));
    if (d < best) {
      best = d;
      g.name = u.first;
      g.per_second = u.second;
    }
  }
  g.rate_ratio = raw_per_wall_second / g.per_second;
  g.plausible = g.rate_ratio >= 0.5 && g.rate_ratio <= 2.0;
  return g;
}

std::string classify_time_base(int64_t raw, double per_second, double now_unix_s)
{
  if (!std::isfinite(per_second) || per_second <= 0.0) {
    return "unknown";
  }
  const double secs = static_cast<double>(raw) / per_second;
  constexpr double kDay = 86400.0;
  if (std::fabs(secs - now_unix_s) < 2.0 * kDay) {
    return "unix_epoch";
  }
  if (std::fabs(secs - kDotNetEpochToUnixS - now_unix_s) < 2.0 * kDay) {
    return "dotnet_epoch";
  }
  if (secs >= 0.0 && secs < 30.0 * kDay) {
    return "elapsed";
  }
  return "unknown";
}

// =============================================================================
// Frame-period estimation
// =============================================================================

FrameAnalysis analyze_frames(const std::vector<TimedSample> & samples, size_t top_gaps)
{
  FrameAnalysis a;
  a.samples = samples.size();
  a.poll_interval_ms = nan_summary();
  a.dvalue_raw = nan_summary();
  a.frames_per_change = nan_summary();
  a.change_interval_ms = nan_summary();
  a.changes_per_second_bucket = nan_summary();
  a.dtick_per_change = nan_summary();
  a.raw_per_wall_s = kNaN;
  a.unit = infer_unit(kNaN);
  a.gcd_step_s = kNaN;
  a.sim_step_raw = a.sim_step_s = kNaN;
  a.wall_frame_rate_hz = a.wall_period_ms = kNaN;
  a.frame_period_s = a.frame_rate_hz = kNaN;
  a.frame_period_s_unchecked = a.frame_rate_hz_unchecked = kNaN;
  a.frame_period_ms_assuming_ms = kNaN;
  a.duplicate_ratio = kNaN;
  a.changes_per_wall_s = kNaN;
  a.dvalue_mode_share = kNaN;
  a.missed_frame_events_per_min = a.wall_stall_events_per_min = kNaN;
  a.max_gap_in_frames = kNaN;
  if (samples.empty()) {
    a.warnings.emplace_back("no samples");
    return a;
  }

  a.wall_span_s = samples.back().wall_s - samples.front().wall_s;

  // ---- Pass 1: every transition between successive samples.
  std::vector<Change> changes;
  std::vector<double> dvalues;
  std::vector<double> dticks;
  std::vector<double> polls_ms;
  std::map<int64_t, size_t> dvalue_hist;

  for (size_t i = 0; i < samples.size(); ++i) {
    const TimedSample & cur = samples[i];
    if (cur.value == 0) {
      ++a.zero_values;
    }
    if (cur.tick != 0) {
      a.tick_present = true;
    }
    if (i == 0) {
      continue;
    }
    const TimedSample & prev = samples[i - 1];
    polls_ms.push_back((cur.wall_s - prev.wall_s) * 1e3);
    // Unsigned wrap is well defined; the int64 reading of it is the signed advance.
    const auto dtick = static_cast<int64_t>(cur.tick - prev.tick);
    if (cur.value == prev.value) {
      ++a.duplicates;
      if (cur.tick != prev.tick) {
        ++a.tick_changes_without_value_change;
      }
      continue;
    }

    ++a.changes;
    Change c;
    c.idx = i;
    c.dtick = dtick;
    if (cur.value < prev.value) {
      ++a.monotonic_violations;
    }
    int64_t d = 0;
    if (!checked_sub(cur.value, prev.value, &d)) {
      ++a.delta_overflows;
    } else {
      c.d = d;
      if (d > 0) {
        c.positive = true;
        dvalues.push_back(static_cast<double>(d));
        ++dvalue_hist[d];
        dticks.push_back(static_cast<double>(dtick));
      }
    }
    changes.push_back(c);
  }

  a.duplicate_ratio =
    samples.size() > 1 ? static_cast<double>(a.duplicates) / static_cast<double>(samples.size() - 1)
                       : kNaN;
  a.poll_interval_ms = summarize(polls_ms);
  a.dvalue_raw = summarize(dvalues);
  a.dtick_per_change = summarize(dticks);

  // ---- Delta structure: mode, histogram, gcd of the frequent deltas.
  if (!dvalue_hist.empty()) {
    size_t best = 0;
    for (const auto & kv : dvalue_hist) {  // ascending keys: ties resolve to the smaller
      if (kv.second > best) {
        best = kv.second;
        a.dvalue_mode_raw = kv.first;
      }
    }
    a.dvalue_mode_share = static_cast<double>(best) / static_cast<double>(dvalues.size());

    for (const auto & kv : dvalue_hist) {
      a.dvalue_hist_top.push_back(DeltaCount{kv.first, kv.second});
    }
    std::stable_sort(
      a.dvalue_hist_top.begin(), a.dvalue_hist_top.end(),
      [](const DeltaCount & l, const DeltaCount & r) { return l.count > r.count; });
    if (a.dvalue_hist_top.size() > 10) {
      a.dvalue_hist_top.resize(10);
    }

    // "Frequent" = at least 1% of the positive deltas (at least one sample).
    const size_t min_count = std::max<size_t>(1, (dvalues.size() + 99) / 100);
    const auto mode = static_cast<double>(a.dvalue_mode_raw);
    int64_t g = 0;
    bool consistent = true;
    for (const auto & kv : dvalue_hist) {
      if (kv.second < min_count) {
        continue;
      }
      g = std::gcd(g, kv.first);
      // Within quantisation of an integer multiple of the mode? Integer timestamps of a
      // non-integer step (20.04 ms as 20/21) differ by 1 raw; allow 1% for float jitter.
      const auto k = static_cast<double>(frames_for(kv.first, a.dvalue_mode_raw));
      const double tol = std::max(1.0, 0.01 * k * mode);
      if (std::fabs(static_cast<double>(kv.first) - k * mode) > tol) {
        consistent = false;
      }
    }
    a.dvalue_gcd_raw = g;
    a.deltas_consistent_with_mode = consistent;
    a.deltas_are_multiples = !consistent && g > 0 && g < a.dvalue_mode_raw;
  }

  // ---- Pass 2: intervals between successive changes (the first change opens the window).
  std::vector<double> intervals_ms;
  std::vector<double> ks;
  std::vector<GapEvent> gaps;
  double sum_d = 0.0;
  double sum_k = 0.0;
  for (size_t j = 1; j < changes.size(); ++j) {
    const Change & c = changes[j];
    const TimedSample & cur = samples[c.idx];
    const TimedSample & last = samples[changes[j - 1].idx];
    GapEvent g;
    g.wall_s = cur.wall_s;
    g.gap_ms = (cur.wall_s - last.wall_s) * 1e3;
    g.dvalue = c.d;
    g.dtick = c.dtick;
    if (c.positive && a.dvalue_mode_raw > 0) {
      g.frames = frames_for(c.d, a.dvalue_mode_raw);
      sum_d += static_cast<double>(c.d);
      sum_k += static_cast<double>(g.frames);
      ks.push_back(static_cast<double>(g.frames));
      ++a.frames_per_change_hist[g.frames];
      a.max_frames_per_change = std::max(a.max_frames_per_change, g.frames);
      if (g.frames >= 2) {
        ++a.multi_frame_changes;
      }
    }
    intervals_ms.push_back(g.gap_ms);
    gaps.push_back(g);
  }
  a.frames_per_change = summarize(ks);
  a.change_interval_ms = summarize(intervals_ms);
  a.change_intervals_over_20ms = count_over(intervals_ms, 20.0);
  a.change_intervals_over_45ms = count_over(intervals_ms, 45.0);

  // ---- Rates over the longest well-defined window: first change to last change (both
  // are moments where the value had just updated). Fall back to first/last sample.
  if (changes.size() >= 2) {
    const TimedSample & f = samples[changes.front().idx];
    const TimedSample & l = samples[changes.back().idx];
    const double dw = l.wall_s - f.wall_s;
    if (dw > 0.0) {
      a.changes_per_wall_s = static_cast<double>(changes.size() - 1) / dw;
      // Doubles, not int64: the raw values are untrusted and may be far apart.
      a.raw_per_wall_s = (static_cast<double>(l.value) - static_cast<double>(f.value)) / dw;
      if (sum_k > 0.0) {
        a.wall_frame_rate_hz = sum_k / dw;
        a.wall_period_ms = 1e3 / a.wall_frame_rate_hz;
        a.sim_step_raw = sum_d / sum_k;
      }
    }
  } else if (a.wall_span_s > 0.0) {
    a.raw_per_wall_s =
      (static_cast<double>(samples.back().value) - static_cast<double>(samples.front().value)) /
      a.wall_span_s;
  }
  a.unit = infer_unit(a.raw_per_wall_s);
  if (std::isfinite(a.unit.per_second) && a.unit.per_second > 0.0) {
    a.sim_step_s = a.sim_step_raw / a.unit.per_second;
    if (a.deltas_are_multiples) {
      a.gcd_step_s = static_cast<double>(a.dvalue_gcd_raw) / a.unit.per_second;
    }
  }
  if (std::isfinite(a.sim_step_s) && a.sim_step_s > 0.0) {
    a.frame_period_s_unchecked = a.sim_step_s;
    a.frame_rate_hz_unchecked = 1.0 / a.sim_step_s;
  }
  if (a.dvalue_raw.n > 0) {
    a.frame_period_ms_assuming_ms = a.dvalue_raw.p50;
  }

  // ---- Stalls relative to the measured wall period, comparable across Hz settings.
  if (std::isfinite(a.wall_period_ms) && a.wall_period_ms > 0.0) {
    const double threshold_ms = 1.5 * a.wall_period_ms;
    a.max_gap_in_frames = 0.0;
    for (const GapEvent & g : gaps) {
      const bool stall = g.gap_ms > threshold_ms;
      a.wall_stall_events += stall ? 1 : 0;
      a.missed_frame_events += (stall || g.frames >= 2) ? 1 : 0;
      a.max_gap_in_frames = std::max(
        a.max_gap_in_frames, std::max(static_cast<double>(g.frames), g.gap_ms / a.wall_period_ms));
    }
    if (a.wall_span_s > 0.0) {
      const double minutes = a.wall_span_s / 60.0;
      a.missed_frame_events_per_min = static_cast<double>(a.missed_frame_events) / minutes;
      a.wall_stall_events_per_min = static_cast<double>(a.wall_stall_events) / minutes;
    }
  }

  std::sort(gaps.begin(), gaps.end(), [](const GapEvent & l, const GapEvent & r) {
    return l.gap_ms > r.gap_ms;
  });
  if (gaps.size() > top_gaps) {
    gaps.resize(top_gaps);
  }
  a.largest_gaps = std::move(gaps);

  if (a.wall_span_s > 0.0) {
    const auto buckets = static_cast<size_t>(std::floor(a.wall_span_s));
    if (buckets > 0) {
      std::vector<double> counts(buckets, 0.0);
      const double origin = samples.front().wall_s;
      for (const Change & c : changes) {
        const auto b = static_cast<size_t>(std::floor(samples[c.idx].wall_s - origin));
        if (b < buckets) {
          counts[b] += 1.0;
        }
      }
      a.changes_per_second_bucket = summarize(counts);
    }
  }

  // ---- Can the sampling support a frame period at all?
  a.undersampled = a.changes > 0 && a.duplicate_ratio < 0.5;
  a.poll_limited = a.change_interval_ms.n > 0 && a.poll_interval_ms.n > 0 &&
                   a.change_interval_ms.p50 <= 1.5 * a.poll_interval_ms.p50;
  a.resolution_limited = a.dvalue_raw.n > 0 && a.dvalue_mode_raw == 1;

  if (changes.size() < 3) {
    a.warnings.push_back(format("only %zu timestamp change(s): no frame period", changes.size()));
  }
  if (a.dvalue_raw.n > 0 && !a.unit.plausible) {
    a.warnings.push_back(format(
      "unit '%s' is a guess: sim/wall ratio %.4g is outside 0.5..2", a.unit.name.c_str(),
      a.unit.rate_ratio));
  }
  if (a.undersampled) {
    a.warnings.push_back(format(
      "undersampled: %.2f polls per change (< 2); the delta mode may be a multiple of the true "
      "step - poll faster",
      1.0 / std::max(1e-9, 1.0 - a.duplicate_ratio)));
  }
  if (a.poll_limited) {
    a.warnings.push_back(format(
      "poll-limited: median change interval %.4g ms <= 1.5 x median poll interval %.4g ms; the "
      "'period' may be the polling period or a continuously advancing clock",
      a.change_interval_ms.p50, a.poll_interval_ms.p50));
  }
  if (a.resolution_limited) {
    a.warnings.emplace_back(
      "delta mode is 1 raw unit: a continuously advancing clock cannot be ruled out");
  }
  if (a.deltas_are_multiples) {
    a.warnings.push_back(format(
      "deltas are multiples of %" PRId64 " raw (mode %" PRId64
      "): timestamp refreshed every few physics steps, or polling skipped frames; "
      "physics-step candidate %.4g ms",
      a.dvalue_gcd_raw, a.dvalue_mode_raw, a.gcd_step_s * 1e3));
  }
  if (a.monotonic_violations > 0) {
    a.warnings.push_back(format(
      "%zu monotonic violation(s): timestamp went backwards (scenario reset?)",
      a.monotonic_violations));
  }
  if (a.delta_overflows > 0) {
    a.warnings.push_back(
      format("%zu timestamp difference(s) overflowed int64 (sentinel values?)", a.delta_overflows));
  }

  a.frame_period_reliable = changes.size() >= 3 && a.dvalue_raw.n > 0 && a.unit.plausible &&
                            std::isfinite(a.frame_period_s_unchecked) && !a.undersampled &&
                            !a.poll_limited && !a.resolution_limited &&
                            a.deltas_consistent_with_mode;
  if (a.frame_period_reliable) {
    a.frame_period_s = a.frame_period_s_unchecked;
    a.frame_rate_hz = a.frame_rate_hz_unchecked;
  }
  return a;
}

Json to_json(const FrameAnalysis & a)
{
  Json gaps = Json::array();
  for (const auto & g : a.largest_gaps) {
    gaps.push(
      Json::object()
        .set("wall_s", g.wall_s)
        .set("gap_ms", g.gap_ms)
        .set("dvalue_raw", g.dvalue)
        .set("dtick", g.dtick)
        .set("frames", g.frames));
  }
  Json hist = Json::array();  // [[dvalue, count], ...] most frequent first
  for (const auto & h : a.dvalue_hist_top) {
    hist.push(Json::array().push(h.dvalue).push(h.count));
  }
  Json khist = Json::array();  // [[frames, count], ...] ascending
  for (const auto & kv : a.frames_per_change_hist) {
    khist.push(Json::array().push(kv.first).push(kv.second));
  }
  Json warnings = Json::array();
  for (const auto & w : a.warnings) {
    warnings.push(w);
  }
  return Json::object()
    .set("samples", a.samples)
    .set("changes", a.changes)
    .set("duplicates", a.duplicates)
    .set("duplicate_ratio", a.duplicate_ratio)
    .set("monotonic_violations", a.monotonic_violations)
    .set("delta_overflows", a.delta_overflows)
    .set("zero_values", a.zero_values)
    .set("wall_span_s", a.wall_span_s)
    .set("poll_interval_ms", to_json(a.poll_interval_ms))
    .set("dvalue_raw", to_json(a.dvalue_raw))
    .set("dvalue_mode_raw", a.dvalue_mode_raw)
    .set("dvalue_mode_share", a.dvalue_mode_share)
    .set("dvalue_hist_top", std::move(hist))
    .set("dvalue_gcd_raw", a.dvalue_gcd_raw)
    .set("deltas_consistent_with_mode", a.deltas_consistent_with_mode)
    .set("deltas_are_multiples", a.deltas_are_multiples)
    .set("gcd_step_ms", a.gcd_step_s * 1e3)
    .set("frames_per_change", to_json(a.frames_per_change))
    .set("frames_per_change_hist", std::move(khist))
    .set("max_frames_per_change", a.max_frames_per_change)
    .set("multi_frame_changes", a.multi_frame_changes)
    .set("change_interval_ms", to_json(a.change_interval_ms))
    .set("wall_stall_events", a.wall_stall_events)
    .set("missed_frame_events", a.missed_frame_events)
    .set("missed_frame_events_per_min", a.missed_frame_events_per_min)
    .set("wall_stall_events_per_min", a.wall_stall_events_per_min)
    .set("max_gap_in_frames", a.max_gap_in_frames)
    .set("change_intervals_over_20ms", a.change_intervals_over_20ms)
    .set("change_intervals_over_45ms", a.change_intervals_over_45ms)
    .set("largest_gaps", std::move(gaps))
    .set("changes_per_wall_s", a.changes_per_wall_s)
    .set("changes_per_second_bucket", to_json(a.changes_per_second_bucket))
    .set("raw_per_wall_s", a.raw_per_wall_s)
    .set(
      "unit", Json::object()
                .set("name", a.unit.name)
                .set("per_second", a.unit.per_second)
                .set("rate_ratio", a.unit.rate_ratio)
                .set("plausible", a.unit.plausible))
    .set("sim_step_raw", a.sim_step_raw)
    .set("sim_step_ms", a.sim_step_s * 1e3)
    .set("wall_frame_rate_hz", a.wall_frame_rate_hz)
    .set("wall_period_ms", a.wall_period_ms)
    .set("undersampled", a.undersampled)
    .set("poll_limited", a.poll_limited)
    .set("resolution_limited", a.resolution_limited)
    .set("frame_period_reliable", a.frame_period_reliable)
    .set("warnings", std::move(warnings))
    .set("frame_period_s", a.frame_period_s)
    .set("frame_rate_hz", a.frame_rate_hz)
    .set("frame_period_s_unchecked", a.frame_period_s_unchecked)
    .set("frame_rate_hz_unchecked", a.frame_rate_hz_unchecked)
    .set("frame_period_ms_assuming_ms", a.frame_period_ms_assuming_ms)
    .set("tick_present", a.tick_present)
    .set("dtick_per_change", to_json(a.dtick_per_change))
    .set("tick_changes_without_value_change", a.tick_changes_without_value_change);
}

// =============================================================================
// WaitForTick
// =============================================================================

std::string classify_wait_for_tick(const WaitForTickObservation & o)
{
  if (o.calls == 0) {
    return "no_calls";
  }
  if (o.ok == 0) {
    return o.deadline_exceeded * 2 >= o.calls ? "no_ok_deadline" : "no_ok_error";
  }
  // "Blocking" = a successful call takes clearly longer than a plain unary round trip.
  const double immediate_ms = std::isfinite(o.rtt_ms_p50) ? std::max(1.0, 3.0 * o.rtt_ms_p50) : 1.0;
  if (!(o.call_ms_p50 > immediate_ms)) {
    return "returns_immediately";
  }
  if (!std::isfinite(o.interval_ms_p50)) {
    // No two successive successful returns (e.g. deadline shorter than the tick wait).
    return "blocks_interval_unknown";
  }
  if (!std::isfinite(o.frame_period_ms) || o.frame_period_ms <= 0.0) {
    return "blocks_period_unknown";
  }
  const double r = o.interval_ms_p50 / o.frame_period_ms;
  if (r < 0.65) {
    return "sub_frame";
  }
  if (r <= 1.5) {
    return "per_frame";
  }
  return "multi_frame";
}

}  // namespace morai_grpc_vendor::probe
