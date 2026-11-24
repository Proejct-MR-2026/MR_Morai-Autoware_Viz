// 신규 — 26.09.25 _ YSH _ lockstep_analysis.hpp 구현 (gRPC·protobuf 비의존)
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

#include "lockstep_analysis.hpp"

#include "actuate_options.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::actuate::lockstep
{

namespace
{
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

std::string format(const char * fmt, ...) __attribute__((format(printf, 1, 2)));

std::string format(const char * fmt, ...)
{
  char buf[1024];
  va_list ap;
  va_start(ap, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  return buf;
}

std::string f2(double v)
{
  return std::isfinite(v) ? format("%.2f", v) : std::string("-");
}

std::string f3(double v)
{
  return std::isfinite(v) ? format("%.3f", v) : std::string("-");
}

Json strings_json(const std::vector<std::string> & v)
{
  Json j = Json::array();
  for (const auto & s : v) {
    j.push(s);
  }
  return j;
}

/// Ratio a / b and its delta-method standard error (a, b independent).
void ratio_se(double a, double a_se, double b, double b_se, double * r, double * r_se)
{
  *r = (std::isfinite(a) && std::isfinite(b) && b != 0.0) ? a / b : kNaN;
  *r_se = kNaN;
  if (std::isfinite(*r) && std::isfinite(a_se) && std::isfinite(b_se)) {
    *r_se = std::sqrt(a_se * a_se + (*r) * (*r) * b_se * b_se) / std::fabs(b);
  }
}

/// Band votes shared with morai_actuate: hypotheses at `h0` (low) and 1 (high).
///   high if v >= h0 + 0.75 (1 - h0) and v - k se > h0 + 0.35 (1 - h0)
///   low  if v <= h0 + 0.35 (1 - h0) and v + k se < h0 + 0.75 (1 - h0)
int band_vote(double v, double se, double h0, double k = 2.0)
{
  const double hi = h0 + 0.75 * (1.0 - h0);
  const double lo = h0 + 0.35 * (1.0 - h0);
  if (!std::isfinite(v) || !std::isfinite(se)) {
    return 0;
  }
  if (v >= hi && v - k * se > lo) {
    return 1;
  }
  if (v <= lo && v + k * se < hi) {
    return -1;
  }
  return 0;
}

/// Standard error of an LSQ slope from its residuals; NaN when n < 3 or x has no spread.
double slope_se_of(const std::vector<double> & x, const std::vector<double> & y, const LinearFit & f)
{
  const size_t n = x.size();
  if (n < 3 || !std::isfinite(f.slope) || !std::isfinite(f.intercept)) {
    return kNaN;
  }
  double mx = 0.0;
  for (double v : x) {
    mx += v;
  }
  mx /= static_cast<double>(n);
  double sxx = 0.0;
  double ss = 0.0;
  for (size_t i = 0; i < n; ++i) {
    sxx += (x[i] - mx) * (x[i] - mx);
    const double r = y[i] - (f.intercept + f.slope * x[i]);
    ss += r * r;
  }
  if (!(sxx > 0.0)) {
    return kNaN;
  }
  return std::sqrt(ss / static_cast<double>(n - 2) / sxx);
}

}  // namespace

// =============================================================================
// Tick classification
// =============================================================================

const char * tick_class_name(TickClass c)
{
  switch (c) {
    case TickClass::kOk:
      return "ok";
    case TickClass::kFailed:
      return "tick_failed";
    case TickClass::kNoRead:
      return "no_read";
    case TickClass::kNotAdvanced:
      return "not_advanced";
    case TickClass::kFrameMismatch:
      return "frame_mismatch";
    case TickClass::kDtMismatch:
      return "dt_mismatch";
    case TickClass::kJumped:
      return "jumped";
  }
  return "?";
}

TickClass classify_tick(const Step & prev, const Step & cur, int period_ms)
{
  if (!cur.tick_ok) {
    return TickClass::kFailed;
  }
  if (!cur.read_ok || !prev.read_ok) {
    return TickClass::kNoRead;
  }
  const int64_t df = static_cast<int64_t>(cur.frame_count) - static_cast<int64_t>(prev.frame_count);
  const int64_t dt = cur.ts_us - prev.ts_us;
  const int64_t expected = static_cast<int64_t>(cur.frames) * period_ms * 1000;
  // sim 시간이 기준: timestamp 가 그대로면 frame_count 가 무엇이든(바쁨·권한 없음이면 기본값 0 일 수 있다) 진행 안 함
  if (dt == 0) {
    return TickClass::kNotAdvanced;
  }
  if (std::llabs(dt - expected) > Limits::kJumpUs) {
    return TickClass::kJumped;
  }
  if (df != cur.frames) {
    return TickClass::kFrameMismatch;
  }
  if (std::llabs(dt - expected) > Limits::kDtToleranceUs) {
    return TickClass::kDtMismatch;
  }
  return TickClass::kOk;
}

TickClass classify_after_gap(const Step & last_good, const Step & cur, int frames_since, int period_ms)
{
  if (!cur.tick_ok) {
    return TickClass::kFailed;
  }
  if (!cur.read_ok || !last_good.read_ok) {
    return TickClass::kNoRead;
  }
  const int64_t dt = cur.ts_us - last_good.ts_us;
  if (dt == 0) {
    return TickClass::kNotAdvanced;
  }
  const int64_t max_us = static_cast<int64_t>(std::max(frames_since, cur.frames)) * period_ms * 1000;
  if (dt < 0 || dt > max_us + Limits::kJumpUs) {
    return TickClass::kJumped;
  }
  return TickClass::kOk;  // 실패한 Tick 이 진행했는지 모르므로 dt 를 정확히 따질 수 없다
}

int safe_stop_max_ticks(int period_ms)
{
  if (period_ms < 1) {
    return Limits::kSafeStopMaxTicks;
  }
  const auto by_sim = static_cast<int>(std::ceil(Limits::kSafeStopMaxSimS * 1000.0 / period_ms));
  return std::max(Limits::kSafeStopMaxTicks, by_sim);
}

DtReport check_dt(const std::vector<Step> & steps, int period_ms)
{
  DtReport r;
  for (size_t i = 1; i < steps.size(); ++i) {
    const Step & p = steps[i - 1];
    const Step & c = steps[i];
    if (!p.tick_ok || !p.read_ok) {
      continue;  // 앞 Tick 을 모르면 판정하지 않는다
    }
    const TickClass cls = classify_tick(p, c, period_ms);
    ++r.n;
    ++r.counts[tick_class_name(cls)];
    if (cls == TickClass::kOk) {
      ++r.ok;
      continue;
    }
    ++r.violations;
    if (r.examples.size() < 5) {
      r.examples.push_back(format(
        "Tick #%zu (run %d, Tick(%d)): %s - frame_count %llu -> %llu, timestamp %+lld us "
        "(expected %+lld)",
        i, c.run, c.frames, tick_class_name(cls), static_cast<unsigned long long>(p.frame_count),
        static_cast<unsigned long long>(c.frame_count), static_cast<long long>(c.ts_us - p.ts_us),
        static_cast<long long>(c.frames) * period_ms * 1000));
    }
  }
  return r;
}

void TickHealth::update(TickClass c)
{
  consecutive_failed = c == TickClass::kFailed ? consecutive_failed + 1 : 0;
  consecutive_not_advanced = c == TickClass::kNotAdvanced ? consecutive_not_advanced + 1 : 0;
  jumped = jumped || c == TickClass::kJumped;
  const bool bad = c == TickClass::kFailed || c == TickClass::kNotAdvanced;
  const unsigned mask = (1u << Limits::kHealthWindow) - 1u;
  recent = ((recent << 1) | (bad ? 1u : 0u)) & mask;
  ++seen;
}

size_t TickHealth::window_bad() const
{
  size_t n = 0;
  for (unsigned r = recent; r != 0; r >>= 1) {
    n += r & 1u;
  }
  return n;
}

std::string check_tick_health(const TickHealth & h)
{
  if (h.jumped) {
    return "the timestamp jumped by > 1 s on a Tick: MORAI left synchronous mode (UI?)";
  }
  if (h.consecutive_failed >= Limits::kMaxFailedTicks) {
    return format("%zu consecutive Tick RPCs failed (MORAI gone?)", h.consecutive_failed);
  }
  if (h.consecutive_not_advanced >= Limits::kMaxFrozenTicks) {
    return format(
      "%zu consecutive Ticks did not advance the timestamp (MORAI busy, paused or no Tick "
      "permission)",
      h.consecutive_not_advanced);
  }
  if (h.window_bad() >= Limits::kMaxBadInWindow) {
    return format(
      "%zu of the last %zu Ticks failed or did not advance the timestamp (MORAI unstable)",
      h.window_bad(), std::min(h.seen, Limits::kHealthWindow));
  }
  return "";
}

// =============================================================================
// Groups and runs
// =============================================================================

double StepGroup::accel() const
{
  double sdv = 0.0;
  double sdt = 0.0;
  for (const auto & x : fit) {
    sdv += x.first;
    sdt += x.second;
  }
  return (fit.empty() || !(sdt > 0.0)) ? kNaN : sdv / sdt;
}

double StepGroup::accel_se() const
{
  const double a = accel();
  if (fit.size() < 3 || !std::isfinite(a)) {
    return kNaN;
  }
  double sdt = 0.0;
  double ss = 0.0;
  for (const auto & x : fit) {
    sdt += x.second;
    const double r = x.first - a * x.second;
    ss += r * r;
  }
  const auto n = static_cast<double>(fit.size());
  return std::sqrt(ss / (n - 1.0) * n) / sdt;
}

double StepGroup::idle_share() const
{
  return n_drive ? static_cast<double>(idle_drive) / static_cast<double>(n_drive) : kNaN;
}

void StepGroup::merge(const StepGroup & o)
{
  n_drive += o.n_drive;
  idle_drive += o.idle_drive;
  fit.insert(fit.end(), o.fit.begin(), o.fit.end());
}

RunAnalysis analyze_run(const RunInput & in)
{
  RunAnalysis a;
  a.pattern = in.pattern;
  a.aborted = in.aborted;
  a.fit.slope = a.fit.intercept = a.fit.r2 = kNaN;
  a.realised_accel = a.ratio = a.rtf = kNaN;
  if (in.steps.size() < 2 || !in.steps.front().read_ok) {
    a.warnings.push_back("no drive Ticks with a usable reference state");
    return a;
  }
  const Step & ref = in.steps.front();
  const int64_t t0 = ref.ts_us;
  a.ticks = in.steps.size() - 1;
  std::vector<double> fx;
  std::vector<double> fy;
  const Step * prev = &ref;
  bool prev_usable = ref.tick_ok && ref.read_ok;
  int64_t last_ts = t0;
  for (size_t i = 1; i < in.steps.size(); ++i) {
    const Step & s = in.steps[i];
    a.frames += static_cast<size_t>(std::max(0, s.frames));
    if (s.commanded) {
      ++a.commanded;
      a.command_failed += s.command_ok ? 0 : 1;
    }
    if (s.tick_ok) {
      (s.frames == 1 ? a.tick1_wall_ms : a.tickn_wall_ms).push_back(s.tick_wall_ms);
    }
    const bool usable = s.tick_ok && s.read_ok;
    // 1 s 넘게 튄 Tick(동기 모드가 풀림)은 분석에서 뺀다 — epoch 로 튄 timestamp 가 sim 시간·기울기를 망친다
    const int64_t expected_us = static_cast<int64_t>(std::max(1, s.frames)) * in.period_ms * 1000;
    const bool jumped =
      usable && prev_usable && std::llabs((s.ts_us - prev->ts_us) - expected_us) > Limits::kJumpUs;
    if (!usable || !prev_usable || s.ts_us <= prev->ts_us || jumped) {
      ++a.skipped;
      prev = &s;
      prev_usable = usable;
      continue;
    }
    const double dt = static_cast<double>(s.ts_us - prev->ts_us) / 1e6;
    const double t = static_cast<double>(s.ts_us - t0) / 1e6;
    const double t_prev = static_cast<double>(prev->ts_us - t0) / 1e6;
    const double dv = s.speed_mps - prev->speed_mps;
    const bool idle = looks_idle(in.idle, s.throttle, s.brake);
    StepGroup * g = nullptr;
    if (s.commanded && !s.command_ok) {
      g = nullptr;  // 거부된 명령: 명령 받은 것도 안 받은 것도 아니다
    } else if (s.commanded) {
      g = s.frames == 1 ? &a.cmd1 : &a.multi;
    } else if (s.frames == 1) {
      g = &a.unc1;
    }
    if (g) {
      ++g->n_drive;
      g->idle_drive += idle ? 1 : 0;
    }
    ++a.all.n_drive;
    a.all.idle_drive += idle ? 1 : 0;
    if (t_prev >= in.fit_start_s - 1e-9) {
      if (g) {
        g->fit.emplace_back(dv, dt);
      }
      a.all.fit.emplace_back(dv, dt);
    }
    if (t >= in.fit_start_s - 1e-9) {
      fx.push_back(t);
      fy.push_back(s.speed_mps);
    }
    last_ts = s.ts_us;
    prev = &s;
    prev_usable = true;
  }
  a.sim_s = static_cast<double>(last_ts - t0) / 1e6;
  a.wall_s = (in.steps.back().wall_ms - ref.wall_ms) / 1e3;
  a.rtf = a.wall_s > 0.0 ? a.sim_s / a.wall_s : kNaN;
  a.fit = fit_line(fx, fy);
  a.slope_se = slope_se_of(fx, fy, a.fit);
  a.realised_accel = a.fit.slope;
  a.ratio = (std::isfinite(a.fit.slope) && in.commanded_accel != 0.0)
              ? a.fit.slope / in.commanded_accel
              : kNaN;
  a.ratio_se = (std::isfinite(a.slope_se) && in.commanded_accel != 0.0)
                 ? a.slope_se / std::fabs(in.commanded_accel)
                 : kNaN;
  a.dt = check_dt(in.steps, in.period_ms);
  if (a.fit.n < 5) {
    a.warnings.push_back(format("only %zu Ticks in the fit window", a.fit.n));
  }
  if (a.command_failed > 0) {
    a.warnings.push_back(format(
      "%zu of %zu ControlVehicle calls were rejected (excluded from the groups)",
      a.command_failed, a.commanded));
  }
  if (a.skipped > 0) {
    a.warnings.push_back(format("%zu Ticks without a usable Tick + read pair", a.skipped));
  }
  if (a.dt.violations > 0) {
    a.warnings.push_back(format(
      "%zu of %zu Ticks did not advance exactly frames x %d ms", a.dt.violations, a.dt.n,
      in.period_ms));
  }
  return a;
}

PatternSummary summarize_pattern(
  const std::string & pattern, const std::vector<const RunAnalysis *> & runs)
{
  PatternSummary p;
  p.pattern = pattern;
  p.runs = runs.size();
  std::vector<double> ticks;
  double within_ss = 0.0;
  size_t within_n = 0;
  for (const RunAnalysis * r : runs) {
    if (r->aborted) {
      ++p.aborted;
      continue;
    }
    if (r->fit.n < 5) {
      continue;
    }
    ++p.valid;
    p.ratios.push_back(r->ratio);
    if (std::isfinite(r->ratio_se)) {
      within_ss += r->ratio_se * r->ratio_se;
      ++within_n;
    }
    p.cmd1.merge(r->cmd1);
    p.unc1.merge(r->unc1);
    p.multi.merge(r->multi);
    p.all.merge(r->all);
    p.sim_s += r->sim_s;
    p.wall_s += r->wall_s;
    ticks.insert(ticks.end(), r->tick1_wall_ms.begin(), r->tick1_wall_ms.end());
    ticks.insert(ticks.end(), r->tickn_wall_ms.begin(), r->tickn_wall_ms.end());
    p.dt_n += r->dt.n;
    p.dt_violations += r->dt.violations;
  }
  p.ratio_mean = mean_of(p.ratios);
  p.ratio_sd = sd_of(p.ratios);
  // 평균의 SE: 런 간 산포(2 런 이상)와 런 안 최소제곱 SE 중 큰 쪽 — 런이 둘뿐이면 산포 추정이 약하다
  const double n = static_cast<double>(p.valid);
  const double between = (p.valid >= 2 && std::isfinite(p.ratio_sd)) ? p.ratio_sd / std::sqrt(n) : kNaN;
  const double within = within_n > 0 ? std::sqrt(within_ss / static_cast<double>(within_n)) / std::sqrt(n) : kNaN;
  if (std::isfinite(between) && std::isfinite(within)) {
    p.ratio_se = std::max(between, within);
  } else {
    p.ratio_se = std::isfinite(between) ? between : within;
  }
  p.rtf = p.wall_s > 0.0 ? p.sim_s / p.wall_s : kNaN;
  p.tick_ms = probe::summarize(ticks);
  return p;
}

// =============================================================================
// Verdicts
// =============================================================================

HoldVerdict judge_hold(
  const std::string & pattern, const StepGroup & cmd, const StepGroup & unc,
  double commanded_accel)
{
  HoldVerdict h;
  h.pattern = pattern;
  h.q = h.q_se = h.q_idle = kNaN;
  h.n_cmd = cmd.n_fit();
  h.n_unc = unc.n_fit();
  h.a_cmd = cmd.accel();
  h.a_cmd_se = cmd.accel_se();
  h.a_unc = unc.accel();
  h.a_unc_se = unc.accel_se();
  ratio_se(h.a_unc, h.a_unc_se, h.a_cmd, h.a_cmd_se, &h.share, &h.share_se);
  auto done = [&](const std::string & v) {
    h.verdict = h.share_verdict = v;
    return h;
  };
  if (h.n_cmd < 10 || h.n_unc < 10) {
    h.note = format(
      "needs >= 10 commanded and >= 10 uncommanded Tick(1) in the fit window (have %zu / %zu)",
      h.n_cmd, h.n_unc);
    return done("n/a");
  }
  const double tiny = 0.05 * std::fabs(commanded_accel);
  const bool cmd_moves = std::isfinite(h.a_cmd) && std::isfinite(h.a_cmd_se) &&
                         h.a_cmd > 2.0 * h.a_cmd_se && h.a_cmd > tiny;
  if (!cmd_moves) {
    const bool unc_moves = std::isfinite(h.a_unc) && std::isfinite(h.a_unc_se) &&
                           h.a_unc > 2.0 * h.a_unc_se && h.a_unc > tiny;
    if (unc_moves) {
      h.note = format(
        "commanded Ticks do not accelerate (%s m/s^2) but uncommanded ones do (%s m/s^2): the "
        "command acts one Tick late",
        f3(h.a_cmd).c_str(), f3(h.a_unc).c_str());
      return done("late");
    }
    h.note = format(
      "commanded Ticks do not accelerate (%s +- %s m/s^2): the commands had no effect "
      "(control mode? gear?)",
      f3(h.a_cmd).c_str(), f3(h.a_cmd_se).c_str());
    return done("inconclusive");
  }
  // 3 SE: 이웃한 dv 가 속도 표본을 공유해 a_cmd·a_unc 가 음의 상관 — 독립 가정 SE 는 1.2~1.5 배 작다
  const int v = band_vote(h.share, h.share_se, 0.0, 3.0);
  if (v == 0) {
    h.note = format(
      "share %s +- %s is inside the undecided band (held >= 0.75, idle <= 0.35, 3 SE clear)",
      f2(h.share).c_str(), f2(h.share_se).c_str());
  } else if (v > 0 && h.share - 3.0 * h.share_se > 1.2) {
    h.note = "uncommanded Ticks accelerate MORE than commanded ones - partly late commands?";
  }
  return done(v > 0 ? "held" : (v < 0 ? "idle" : "inconclusive"));
}

RatioVs ratio_vs(const PatternSummary & p, const PatternSummary & ref)
{
  RatioVs r;
  r.q = r.se = kNaN;
  if (p.valid == 0 || ref.valid == 0) {
    return r;
  }
  ratio_se(p.ratio_mean, p.ratio_se, ref.ratio_mean, ref.ratio_se, &r.q, &r.se);
  return r;
}

bool reference_usable(const PatternSummary * s1)
{
  return s1 && s1->valid > 0 && std::isfinite(s1->ratio_mean) && std::isfinite(s1->ratio_se) &&
         s1->ratio_mean > 2.0 * s1->ratio_se && s1->ratio_mean > 0.05;
}

HoldVerdict judge_hold_pattern(
  const PatternSummary & p, int cmd_every, const PatternSummary * sync1, double commanded_accel)
{
  HoldVerdict h = judge_hold(p.pattern, p.cmd1, p.unc1, commanded_accel);
  const std::string share_note = h.note;
  h.q_idle = 1.0 / static_cast<double>(std::max(1, cmd_every));
  std::string ratio_note;
  if (cmd_every < 2) {
    ratio_note = "the pattern commands every Tick - nothing to compare";
  } else if (!sync1) {
    ratio_note = "sync1 not measured - no pattern-level reference";
  } else if (!reference_usable(sync1)) {
    ratio_note = format(
      "sync1 did not accelerate clearly (ratio %s +- %s) - no pattern-level reference",
      f3(sync1->ratio_mean).c_str(), f3(sync1->ratio_se).c_str());
  } else if (p.valid == 0) {
    ratio_note = "no valid run of " + p.pattern;
  } else {
    const RatioVs r = ratio_vs(p, *sync1);
    h.q = r.q;
    h.q_se = r.se;
    const int v = band_vote(h.q, h.q_se, h.q_idle);
    h.ratio_verdict = v > 0 ? "held" : (v < 0 ? "idle" : "inconclusive");
    if (v == 0) {
      ratio_note = format(
        "q %s +- %s is inside the undecided band (held >= %.3f, idle <= %.3f, 2 SE clear)",
        f3(h.q).c_str(), f3(h.q_se).c_str(), h.q_idle + 0.75 * (1.0 - h.q_idle),
        h.q_idle + 0.35 * (1.0 - h.q_idle));
    }
  }
  const std::string & rv = h.ratio_verdict;
  const std::string & sv = h.share_verdict;
  std::string note;
  if (sv == "late" && rv != "held") {
    h.verdict = "late";
    note = share_note;
  } else if (rv == "held" && sv == "idle") {
    // 지연은 share 를 1 쪽으로만 끌어당긴다 — 패턴 수준 유지 + Tick 단위 유휴는 설명되지 않는다
    h.verdict = "conflict";
    note = format(
      "pattern-level q %s says held, but uncommanded Ticks do not accelerate (share %s) - no "
      "actuator lag explains that; check the raw steps",
      f2(h.q).c_str(), f2(h.share).c_str());
  } else if (rv == "held" || rv == "idle") {
    h.verdict = rv;
    if (rv == "idle" && sv == "held") {
      note = format(
        "the per-Tick share %s looks held: the realised acceleration lags the input over several "
        "Ticks and smooths commanded and uncommanded Ticks alike; the pattern-level q is immune "
        "to that",
        f2(h.share).c_str());
    } else if (sv != rv) {
      // Tick 단위 dv 는 속도 잡음에 약하다 — 패턴 수준이 정했으면 그 진단(late·no effect 등)은 판정에 쓰지 않는다
      note = "the per-Tick share is " + sv + " (" + f2(h.share) + " +- " + f2(h.share_se) +
             ") - not used: the pattern-level q decides (per-Tick dv are sensitive to speed noise)";
    }
  } else if (rv == "n/a") {
    // 기준(sync1)이 없으면 Tick 단위만: 유휴는 지연으로 생기지 않으므로 믿고, 유지는 지연으로도 생기므로 미정
    if (sv == "idle") {
      h.verdict = "idle";
      note = "per-Tick share only (" + ratio_note + ")";
    } else if (sv == "held") {
      h.verdict = "inconclusive";
      note = "the per-Tick share looks held, but actuator lag makes a dropped input look held "
             "too (" + ratio_note + "): measure sync1 as the reference";
    } else {
      h.verdict = sv;
      note = share_note.empty() ? ratio_note : share_note;
    }
  } else {
    h.verdict = "inconclusive";
    note = ratio_note;
  }
  if (note.empty() && h.verdict == sv) {
    note = share_note;
  }
  h.note = note;
  return h;
}

std::string combine_hold(const std::vector<HoldVerdict> & v)
{
  for (const auto & h : v) {
    if (h.verdict == "conflict") {
      return "conflict";
    }
  }
  std::string out;
  for (const auto & h : v) {
    if (h.verdict != "held" && h.verdict != "idle" && h.verdict != "late") {
      continue;
    }
    if (out.empty()) {
      out = h.verdict;
    } else if (out != h.verdict) {
      return "inconsistent";
    }
  }
  return out.empty() ? "unknown" : out;
}

namespace
{
std::string chain_note(const MultiFrameVerdict & m)
{
  if (!std::isfinite(m.m_chain)) {
    return "";
  }
  // 2 SE, 최소 0.05 (잡음 없는 합성 데이터에서도 부동소수 차이로 "다르다" 가 나오지 않게)
  const double tol = std::max(
    0.05, 2.0 * std::sqrt(
                  (std::isfinite(m.m_se) ? m.m_se * m.m_se : 0.0) +
                  (std::isfinite(m.m_chain_se) ? m.m_chain_se * m.m_chain_se : 0.0)));
  return format(
    "; one command + Tick(1) x%d: %s +- %s -> Tick(%d) %s Tick(1) x%d", m.frames,
    f2(m.m_chain).c_str(), f2(m.m_chain_se).c_str(), m.frames,
    std::fabs(m.m - m.m_chain) <= tol ? "behaves like" : "DIFFERS from", m.frames);
}
}  // namespace

MultiFrameVerdict judge_multi_frame(
  const StepGroup & single_cmd, const std::string & a1_source, const StepGroup & multi,
  const StepGroup * chain, int frames)
{
  MultiFrameVerdict m;
  m.frames = frames;
  m.method = "per-Tick";
  m.a1_source = a1_source;
  m.n1 = single_cmd.n_fit();
  m.nn = multi.n_fit();
  m.a1 = single_cmd.accel();
  m.a1_se = single_cmd.accel_se();
  m.an = multi.accel();
  m.an_se = multi.accel_se();
  ratio_se(m.an, m.an_se, m.a1, m.a1_se, &m.m, &m.m_se);
  m.m_chain = m.m_chain_se = kNaN;
  if (chain && chain->n_fit() >= 10) {
    ratio_se(chain->accel(), chain->accel_se(), m.a1, m.a1_se, &m.m_chain, &m.m_chain_se);
  }
  if (frames < 2 || m.n1 < 10 || m.nn < 5) {
    m.verdict = "n/a";
    m.note = format(
      "needs >= 10 commanded Tick(1) and >= 5 Tick(%d) in the fit window (have %zu / %zu)",
      frames, m.n1, m.nn);
    return m;
  }
  if (!(std::isfinite(m.a1_se) && m.a1 > 2.0 * m.a1_se && m.a1 > 0.0)) {
    m.verdict = "inconclusive";
    m.note = "commanded Tick(1) did not accelerate clearly - no reference";
    return m;
  }
  const double base = 1.0 / static_cast<double>(frames);
  const int v = band_vote(m.m, m.m_se, base);
  m.verdict = v > 0 ? "persists" : (v < 0 ? "first_frame_only" : "inconclusive");
  m.note = format(
    "a Tick(%d) accelerates %s +- %s x a commanded Tick(1) (persisting input 1, first frame "
    "only %.2f)",
    frames, f2(m.m).c_str(), f2(m.m_se).c_str(), base) + chain_note(m);
  return m;
}

MultiFrameVerdict judge_multi_frame_pattern(
  const PatternSummary & tickn, const PatternSummary & sync1, const PatternSummary * chain,
  int frames, double commanded_accel)
{
  MultiFrameVerdict m;
  m.frames = frames;
  m.method = "pattern ratios";
  m.a1_source = "sync1 (LSQ ratio over runs)";
  m.n1 = sync1.valid;
  m.nn = tickn.valid;
  const double c = std::fabs(commanded_accel);
  m.a1 = sync1.ratio_mean * c;
  m.a1_se = sync1.ratio_se * c;
  m.an = tickn.ratio_mean * c;
  m.an_se = tickn.ratio_se * c;
  const RatioVs r = ratio_vs(tickn, sync1);
  m.m = r.q;
  m.m_se = r.se;
  m.m_chain = m.m_chain_se = kNaN;
  if (chain && chain->valid > 0) {
    const RatioVs rc = ratio_vs(*chain, sync1);
    m.m_chain = rc.q;
    m.m_chain_se = rc.se;
  }
  if (frames < 2 || m.n1 == 0 || m.nn == 0) {
    m.verdict = "n/a";
    m.note = format(
      "needs a valid sync1 and a valid Tick(%d) run (have %zu / %zu)", frames, m.n1, m.nn);
    return m;
  }
  if (!reference_usable(&sync1)) {
    m.verdict = "inconclusive";
    m.note = "sync1 did not accelerate clearly - no reference";
    return m;
  }
  const double base = 1.0 / static_cast<double>(frames);
  const int v = band_vote(m.m, m.m_se, base);
  m.verdict = v > 0 ? "persists" : (v < 0 ? "first_frame_only" : "inconclusive");
  m.note = format(
    "ratio(Tick(%d)) / ratio(sync1) = %s +- %s (persisting input 1, first frame only %.2f)",
    frames, f2(m.m).c_str(), f2(m.m_se).c_str(), base) + chain_note(m);
  return m;
}

IdleRecordCheck check_idle_records(
  const StepGroup & cmd, const StepGroup & unc, bool idle_ambiguous)
{
  IdleRecordCheck c;
  c.n_cmd = cmd.n_drive;
  c.idle_cmd = cmd.idle_drive;
  c.n_unc = unc.n_drive;
  c.idle_unc = unc.idle_drive;
  c.share_cmd = cmd.idle_share();
  c.share_unc = unc.idle_share();
  if (idle_ambiguous) {
    c.note = "the idle signature is ambiguous (brake 0): records cannot tell idle from a command";
    return c;
  }
  if (c.n_cmd < 10 || c.n_unc < 10) {
    c.note = format("too few records (%zu commanded, %zu uncommanded Tick(1))", c.n_cmd, c.n_unc);
    return c;
  }
  if (!(c.share_cmd <= 0.1)) {
    c.note = format(
      "positive control failed: %.0f%% of commanded Tick(1) records look idle - the record "
      "does not show a consumed command",
      100.0 * c.share_cmd);
    return c;
  }
  if (c.share_unc >= 0.5) {
    c.vote = "idle_record";
  } else if (c.share_unc <= 0.1) {
    c.vote = "held_record";
  } else {
    c.vote = "neutral";
  }
  c.note = format(
    "uncommanded Tick(1) records idle %.0f%%, commanded %.0f%%", 100.0 * c.share_unc,
    100.0 * c.share_cmd);
  return c;
}

MultiRecordCheck check_multi_records(
  const StepGroup & cmd, const StepGroup & multi, bool idle_ambiguous)
{
  MultiRecordCheck c;
  c.n_multi = multi.n_drive;
  c.idle_multi = multi.idle_drive;
  c.share_multi = multi.idle_share();
  if (idle_ambiguous) {
    c.note = "the idle signature is ambiguous (brake 0)";
    return c;
  }
  if (cmd.n_drive < 10 || c.n_multi < 5) {
    c.note = format("too few records (%zu commanded Tick(1), %zu Tick(n))", cmd.n_drive, c.n_multi);
    return c;
  }
  if (!(cmd.idle_share() <= 0.1)) {
    c.note = format(
      "positive control failed: %.0f%% of commanded Tick(1) records look idle",
      100.0 * cmd.idle_share());
    return c;
  }
  if (c.share_multi >= 0.5) {
    c.vote = "first_frame_record";
  } else if (c.share_multi <= 0.1) {
    c.vote = "persist_record";
  } else {
    c.vote = "neutral";
  }
  c.note = format(
    "records read after a commanded Tick(n) (its last frame) idle %.0f%%", 100.0 * c.share_multi);
  return c;
}

IdleCalibration calibrate_sync_idle(
  const std::vector<std::pair<double, double>> & records, const IdleSignature & fallback)
{
  IdleCalibration c = calibrate_idle(records, fallback);
  if (c.source == "measured" || c.ambiguous || c.n < 5 ||
      !(std::fabs(c.throttle_median) <= fallback.throttle_max)) {
    return c;
  }
  // 이 세션의 첫 ControlVehicle 전에 잰 레코드라 유지된 명령일 수 없다 — 0.05 초과 브레이크도 유휴 레코드로 받는다
  if (c.brake_median > 0.05 && c.brake_median <= 1.0) {
    c.signature.brake_center = c.brake_median;
    c.source = "measured";
    c.warnings.clear();
    c.warnings.push_back(format(
      "synchronous-mode idle record has brake %.4f (outside the real-time 0..0.05 window); "
      "taken as the idle signature because the probe ran before any command",
      c.brake_median));
  }
  return c;
}

Throughput throughput(const std::vector<const RunAnalysis *> & runs)
{
  Throughput t;
  std::vector<double> t1;
  std::vector<double> tn;
  for (const RunAnalysis * r : runs) {
    if (!(r->wall_s > 0.0)) {
      continue;
    }
    t.sim_s += r->sim_s;
    t.wall_s += r->wall_s;
    t1.insert(t1.end(), r->tick1_wall_ms.begin(), r->tick1_wall_ms.end());
    tn.insert(tn.end(), r->tickn_wall_ms.begin(), r->tickn_wall_ms.end());
  }
  t.rtf = t.wall_s > 0.0 ? t.sim_s / t.wall_s : kNaN;
  t.tick1_ms = probe::summarize(t1);
  t.tickn_ms = probe::summarize(tn);
  return t;
}

Interpretation interpret(
  const std::vector<PatternSummary> & patterns, double commanded_accel, bool idle_ambiguous,
  const DtReport & session_dt, const std::vector<const RunAnalysis *> & runs)
{
  Interpretation i;
  i.sync1_ratio = kNaN;
  auto find = [&](const char * name) -> const PatternSummary * {
    for (const auto & p : patterns) {
      if (p.pattern == name) {
        return &p;
      }
    }
    return nullptr;
  };
  auto cmd_every = [](const std::string & name) {
    const LockstepPatternSpec * spec = find_lockstep_pattern(name);
    return spec ? spec->cmd_every : 1;
  };
  const PatternSummary * s1 = find("sync1");
  const PatternSummary * d2 = find("sync_drop2");
  const PatternSummary * t3 = find("sync_tick3");
  const PatternSummary * t3x = find("sync_tick3x");
  if (s1) {
    i.sync1_ratio = s1->ratio_mean;
  }
  // (c) 유지 판정: 패턴 수준 ratio / ratio(sync1) 이 1차(지연에 속지 않음), Tick 단위 share 는 늦음·충돌 검출
  for (const PatternSummary * p : {d2, t3x}) {
    if (p) {
      i.hold.push_back(judge_hold_pattern(*p, cmd_every(p->pattern), s1, commanded_accel));
    }
  }
  i.hold_overall = combine_hold(i.hold);
  // (d) Tick(3) 판정: ratio(sync_tick3) / ratio(sync1); sync1 이 없으면 Tick 단위(모든 패턴의 명령 Tick(1))
  const LockstepPatternSpec * spec3 = find_lockstep_pattern("sync_tick3");
  const int frames3 = spec3 ? spec3->frames : 3;
  if (t3 && s1) {
    i.multi = judge_multi_frame_pattern(*t3, *s1, t3x, frames3, commanded_accel);
  } else {
    StepGroup single;
    for (const auto & p : patterns) {
      single.merge(p.cmd1);
    }
    i.multi = judge_multi_frame(
      single, "commanded Tick(1) of all patterns (sync1 not measured)",
      t3 ? t3->multi : StepGroup{}, t3x ? &t3x->all : nullptr, frames3);
  }
  // (f) 레코드: 명령 받은 Tick(1) 전부 vs 안 받은 Tick(1) 전부, 그리고 Tick(n) 뒤 레코드
  StepGroup cmd_all;
  StepGroup unc_all;
  StepGroup multi_all;
  for (const auto & p : patterns) {
    cmd_all.merge(p.cmd1);
    unc_all.merge(p.unc1);
    multi_all.merge(p.multi);
  }
  i.idle_records = check_idle_records(cmd_all, unc_all, idle_ambiguous);
  i.multi_records = check_multi_records(cmd_all, multi_all, idle_ambiguous);
  // (e) 처리량, (a) dt
  i.speed = throughput(runs);
  i.dt = session_dt;

  auto hold_text = [](const std::string & v) -> std::string {
    if (v == "held") return "held (MORAI keeps the last input)";
    if (v == "idle") return "idle (input dropped - command every Tick)";
    if (v == "late") return "command acts one Tick late";
    if (v == "inconsistent") return "inconsistent between patterns";
    if (v == "conflict") return "CONFLICT between pattern-level and per-Tick evidence (see hold lines)";
    return "undecided";
  };
  auto multi_text = [](const std::string & v) -> std::string {
    if (v == "persists") return "input persists for all frames";
    if (v == "first_frame_only") return "only the first frame is driven";
    if (v == "inconclusive") return "undecided";
    return "not measured";
  };
  std::string line = s1 && std::isfinite(i.sync1_ratio)
                       ? "sync1 ratio ≈ " + f2(i.sync1_ratio)
                       : std::string("sync1 not measured");
  line += "; uncommanded steps: " + hold_text(i.hold_overall);
  line += "; Tick(" + std::to_string(frames3) + "): " + multi_text(i.multi.verdict);
  line += "; lockstep speed ≈ " + f2(i.speed.rtf) + "× real time";
  line += format("; dt exact in %zu/%zu Ticks", i.dt.ok, i.dt.n);
  i.line = line;

  for (const auto & h : i.hold) {
    i.lines.push_back(format(
      "hold %-11s: %-12s q %s +- %s (ratio vs sync1: held 1, idle %.2f) [%s]; per-Tick share %s "
      "+- %s (a_unc %s / a_cmd %s m/s^2, n %zu/%zu) [%s]%s%s",
      h.pattern.c_str(), h.verdict.c_str(), f3(h.q).c_str(), f3(h.q_se).c_str(), h.q_idle,
      h.ratio_verdict.c_str(), f2(h.share).c_str(), f2(h.share_se).c_str(), f3(h.a_unc).c_str(),
      f3(h.a_cmd).c_str(), h.n_unc, h.n_cmd, h.share_verdict.c_str(),
      h.note.empty() ? "" : " - ", h.note.c_str()));
  }
  i.lines.push_back(format(
    "Tick(%d)          : %-16s %s (%s; reference: %s)", frames3, i.multi.verdict.c_str(),
    i.multi.note.c_str(), i.multi.method.c_str(), i.multi.a1_source.c_str()));
  i.lines.push_back(format(
    "idle record      : %-12s %s", i.idle_records.vote.c_str(), i.idle_records.note.c_str()));
  i.lines.push_back(format(
    "Tick(n) record   : %-18s %s", i.multi_records.vote.c_str(), i.multi_records.note.c_str()));
  if (i.idle_records.vote == "idle_record" && i.hold_overall == "held") {
    i.lines.push_back("WARNING: physics says held but the record looks idle on uncommanded Ticks");
  } else if (i.idle_records.vote == "held_record" && i.hold_overall == "idle") {
    i.lines.push_back("WARNING: physics says idle but the record shows the held command");
  }
  if (i.multi_records.vote == "first_frame_record" && i.multi.verdict == "persists") {
    i.lines.push_back("WARNING: physics says Tick(n) input persists but the record after it looks idle");
  } else if (i.multi_records.vote == "persist_record" && i.multi.verdict == "first_frame_only") {
    i.lines.push_back(
      "WARNING: physics says only the first frame is driven but the record after Tick(n) shows "
      "the command");
  }
  i.lines.push_back(format(
    "lockstep speed   : %s x real time (%.2f s sim in %.2f s wall); Tick(1) RPC p50 %s p90 %s "
    "max %s ms (n %zu)%s",
    f2(i.speed.rtf).c_str(), i.speed.sim_s, i.speed.wall_s, f2(i.speed.tick1_ms.p50).c_str(),
    f2(i.speed.tick1_ms.p90).c_str(), f2(i.speed.tick1_ms.max).c_str(), i.speed.tick1_ms.n,
    i.speed.tickn_ms.n ? format(
                           "; Tick(n>1) p50 %s ms (n %zu)", f2(i.speed.tickn_ms.p50).c_str(),
                           i.speed.tickn_ms.n)
                           .c_str()
                       : ""));
  i.lines.push_back(format(
    "dt               : %zu/%zu Ticks advanced exactly frames x tick_period (+-%lld us)%s",
    i.dt.ok, i.dt.n, static_cast<long long>(Limits::kDtToleranceUs),
    i.dt.violations ? " - VIOLATIONS below" : ""));
  for (const auto & e : i.dt.examples) {
    i.lines.push_back("  " + e);
  }
  return i;
}

// =============================================================================
// JSON
// =============================================================================

Json to_json(const DtReport & d)
{
  Json c = Json::object();
  for (const auto & kv : d.counts) {
    c.set(kv.first, kv.second);
  }
  return Json::object()
    .set("n", d.n)
    .set("ok", d.ok)
    .set("violations", d.violations)
    .set("counts", std::move(c))
    .set("examples", strings_json(d.examples));
}

Json to_json(const StepGroup & g)
{
  return Json::object()
    .set("n_drive", g.n_drive)
    .set("idle_drive", g.idle_drive)
    .set("idle_share", g.idle_share())
    .set("n_fit", g.n_fit())
    .set("accel", g.accel())
    .set("accel_se", g.accel_se());
}

Json to_json(const RunAnalysis & a)
{
  return Json::object()
    .set("pattern", a.pattern)
    .set("aborted", a.aborted)
    .set("ticks", a.ticks)
    .set("frames", a.frames)
    .set("commanded", a.commanded)
    .set("command_failed", a.command_failed)
    .set("skipped", a.skipped)
    .set("sim_s", a.sim_s)
    .set("wall_s", a.wall_s)
    .set("rtf", a.rtf)
    .set("fit", to_json(a.fit))
    .set("slope_se", a.slope_se)
    .set("realised_accel", a.realised_accel)
    .set("ratio", a.ratio)
    .set("ratio_se", a.ratio_se)
    .set("cmd1", to_json(a.cmd1))
    .set("unc1", to_json(a.unc1))
    .set("multi", to_json(a.multi))
    .set("all", to_json(a.all))
    .set("tick1_wall_ms", probe::to_json(probe::summarize(a.tick1_wall_ms)))
    .set("tickn_wall_ms", probe::to_json(probe::summarize(a.tickn_wall_ms)))
    .set("dt", to_json(a.dt))
    .set("warnings", strings_json(a.warnings));
}

Json to_json(const PatternSummary & p)
{
  Json r = Json::array();
  for (double x : p.ratios) {
    r.push(x);
  }
  return Json::object()
    .set("pattern", p.pattern)
    .set("runs", p.runs)
    .set("valid", p.valid)
    .set("aborted", p.aborted)
    .set("ratios", std::move(r))
    .set("ratio_mean", p.ratio_mean)
    .set("ratio_sd", p.ratio_sd)
    .set("ratio_se", p.ratio_se)
    .set("cmd1", to_json(p.cmd1))
    .set("unc1", to_json(p.unc1))
    .set("multi", to_json(p.multi))
    .set("all", to_json(p.all))
    .set("sim_s", p.sim_s)
    .set("wall_s", p.wall_s)
    .set("rtf", p.rtf)
    .set("tick_ms", probe::to_json(p.tick_ms))
    .set("dt_n", p.dt_n)
    .set("dt_violations", p.dt_violations);
}

Json to_json(const HoldVerdict & h)
{
  return Json::object()
    .set("pattern", h.pattern)
    .set("verdict", h.verdict)
    .set("ratio_verdict", h.ratio_verdict)
    .set("q", h.q)
    .set("q_se", h.q_se)
    .set("q_idle", h.q_idle)
    .set("share_verdict", h.share_verdict)
    .set("share", h.share)
    .set("share_se", h.share_se)
    .set("a_cmd", h.a_cmd)
    .set("a_cmd_se", h.a_cmd_se)
    .set("a_unc", h.a_unc)
    .set("a_unc_se", h.a_unc_se)
    .set("n_cmd", h.n_cmd)
    .set("n_unc", h.n_unc)
    .set("note", h.note);
}

Json to_json(const MultiFrameVerdict & m)
{
  return Json::object()
    .set("verdict", m.verdict)
    .set("frames", m.frames)
    .set("method", m.method)
    .set("a1_source", m.a1_source)
    .set("a1", m.a1)
    .set("a1_se", m.a1_se)
    .set("an", m.an)
    .set("an_se", m.an_se)
    .set("m", m.m)
    .set("m_se", m.m_se)
    .set("m_chain", m.m_chain)
    .set("m_chain_se", m.m_chain_se)
    .set("n1", m.n1)
    .set("nn", m.nn)
    .set("note", m.note);
}

Json to_json(const IdleRecordCheck & c)
{
  return Json::object()
    .set("vote", c.vote)
    .set("n_cmd", c.n_cmd)
    .set("idle_cmd", c.idle_cmd)
    .set("share_cmd", c.share_cmd)
    .set("n_unc", c.n_unc)
    .set("idle_unc", c.idle_unc)
    .set("share_unc", c.share_unc)
    .set("note", c.note);
}

Json to_json(const MultiRecordCheck & c)
{
  return Json::object()
    .set("vote", c.vote)
    .set("n_multi", c.n_multi)
    .set("idle_multi", c.idle_multi)
    .set("share_multi", c.share_multi)
    .set("note", c.note);
}

Json to_json(const Throughput & t)
{
  return Json::object()
    .set("sim_s", t.sim_s)
    .set("wall_s", t.wall_s)
    .set("rtf", t.rtf)
    .set("tick1_ms", probe::to_json(t.tick1_ms))
    .set("tickn_ms", probe::to_json(t.tickn_ms));
}

Json to_json(const Interpretation & i)
{
  Json h = Json::array();
  for (const auto & x : i.hold) {
    h.push(to_json(x));
  }
  return Json::object()
    .set("line", i.line)
    .set("sync1_ratio", i.sync1_ratio)
    .set("hold", std::move(h))
    .set("hold_overall", i.hold_overall)
    .set("multi_frame", to_json(i.multi))
    .set("idle_records", to_json(i.idle_records))
    .set("multi_records", to_json(i.multi_records))
    .set("throughput", to_json(i.speed))
    .set("dt", to_json(i.dt))
    .set("lines", strings_json(i.lines));
}

}  // namespace morai_grpc_vendor::actuate::lockstep
