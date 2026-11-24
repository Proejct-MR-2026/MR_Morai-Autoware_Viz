// 신규 — 26.09.25 _ YSH _ actuate_analysis.hpp 구현 (gRPC·protobuf 비의존)
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

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <limits>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::actuate
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

double median_of(std::vector<double> v)
{
  if (v.empty()) {
    return kNaN;
  }
  std::sort(v.begin(), v.end());
  const size_t m = v.size() / 2;
  return v.size() % 2 ? v[m] : 0.5 * (v[m - 1] + v[m]);
}

double ratio_or_nan(double a, double b)
{
  return (std::isfinite(a) && std::isfinite(b) && b != 0.0) ? a / b : kNaN;
}

Json hist_json(const std::map<int64_t, size_t> & h)
{
  Json j = Json::object();
  for (const auto & kv : h) {
    j.set(std::to_string(kv.first), kv.second);
  }
  return j;
}

Json strings_json(const std::vector<std::string> & v)
{
  Json j = Json::array();
  for (const auto & s : v) {
    j.push(s);
  }
  return j;
}

std::string fmt2(double v)
{
  return std::isfinite(v) ? format("%.2f", v) : std::string("-");
}

}  // namespace

// =============================================================================
// Least squares
// =============================================================================

LinearFit fit_line(const std::vector<double> & x, const std::vector<double> & y)
{
  LinearFit f;
  f.n = std::min(x.size(), y.size());
  f.slope = f.intercept = f.r2 = kNaN;
  if (f.n < 2) {
    return f;
  }
  double mx = 0.0;
  double my = 0.0;
  for (size_t i = 0; i < f.n; ++i) {
    mx += x[i];
    my += y[i];
  }
  mx /= static_cast<double>(f.n);
  my /= static_cast<double>(f.n);
  double sxx = 0.0;
  double sxy = 0.0;
  double syy = 0.0;
  for (size_t i = 0; i < f.n; ++i) {
    sxx += (x[i] - mx) * (x[i] - mx);
    sxy += (x[i] - mx) * (y[i] - my);
    syy += (y[i] - my) * (y[i] - my);
  }
  if (!(sxx > 0.0)) {
    return f;
  }
  f.slope = sxy / sxx;
  f.intercept = my - f.slope * mx;
  if (syy > 0.0) {
    double ss_res = 0.0;
    for (size_t i = 0; i < f.n; ++i) {
      const double e = y[i] - (f.intercept + f.slope * x[i]);
      ss_res += e * e;
    }
    f.r2 = 1.0 - ss_res / syy;
  }
  return f;
}

double mean_of(const std::vector<double> & v)
{
  if (v.empty()) {
    return kNaN;
  }
  double s = 0.0;
  for (double x : v) {
    s += x;
  }
  return s / static_cast<double>(v.size());
}

double sd_of(const std::vector<double> & v)
{
  if (v.size() < 2) {
    return kNaN;
  }
  const double m = mean_of(v);
  double s = 0.0;
  for (double x : v) {
    s += (x - m) * (x - m);
  }
  return std::sqrt(s / static_cast<double>(v.size() - 1));
}

// =============================================================================
// Physics step
// =============================================================================

int64_t steps_for(int64_t delta_us, double step_us)
{
  if (!(step_us > 0.0) || delta_us <= 0) {
    return 1;
  }
  const double r = static_cast<double>(delta_us) / step_us;
  if (!(r < 1e9)) {
    return static_cast<int64_t>(1e9);
  }
  return std::max<int64_t>(1, static_cast<int64_t>(std::llround(r)));
}

StepEstimate estimate_physics_step(const std::vector<int64_t> & deltas_us, double hint_step_us)
{
  StepEstimate e;
  e.step_us = kNaN;
  e.hint_step_us = hint_step_us > 0.0 ? hint_step_us : kNaN;

  std::vector<int64_t> d;
  d.reserve(deltas_us.size());
  for (int64_t v : deltas_us) {
    if (v > 0) {
      d.push_back(v);
    }
  }
  e.n_deltas = d.size();
  std::sort(d.begin(), d.end());

  // 정렬된 델타를 인접 허용오차(max(30 µs, 2%))로 묶는다
  std::vector<std::vector<int64_t>> groups;
  for (int64_t v : d) {
    if (!groups.empty()) {
      const double first = static_cast<double>(groups.back().front());
      if (static_cast<double>(v) - first <= std::max(30.0, 0.02 * first)) {
        groups.back().push_back(v);
        continue;
      }
    }
    groups.push_back({v});
  }
  const double min_count = std::max(2.0, 0.02 * static_cast<double>(d.size()));
  for (const auto & g : groups) {
    if (static_cast<double>(g.size()) >= min_count) {
      std::vector<double> gv(g.begin(), g.end());
      e.clusters.push_back(DeltaCluster{median_of(gv), g.size()});
    }
  }

  double est = kNaN;
  if (d.size() >= 5 && !e.clusters.empty()) {
    const double c_min = e.clusters.front().center_us;
    for (int m = 1; m <= 12; ++m) {
      const double s = c_min / m;
      if (s < 250.0) {
        break;
      }
      const bool ok = std::all_of(e.clusters.begin(), e.clusters.end(), [&](const DeltaCluster & c) {
        const double r = c.center_us / s;
        return std::fabs(r - std::round(r)) <= 0.06;
      });
      if (ok) {
        double num = 0.0;
        double den = 0.0;
        for (const auto & c : e.clusters) {
          num += c.center_us * static_cast<double>(c.count);
          den += std::round(c.center_us / s) * static_cast<double>(c.count);
        }
        est = den > 0.0 ? num / den : s;
        e.divisor = m;
        break;
      }
    }
  }

  if (std::isfinite(est)) {
    e.step_us = est;
    e.method = "fundamental";
    // 빈번한 군집이 하나뿐이면 모든 업데이트가 같은 스텝 수만큼 진행한 것 — 약수를 배제할 수 없다
    e.reliable = e.clusters.size() >= 2;
    if (!e.reliable) {
      e.warnings.push_back(format(
        "every frequent delta is %.0f us: each update may advance k > 1 steps; pass "
        "--physics-hz with the MORAI Hz setting",
        est));
    }
  } else {
    e.warnings.push_back(
      format("cannot estimate the physics step from %zu deltas", e.n_deltas));
  }

  if (hint_step_us > 0.0) {
    if (!std::isfinite(est)) {
      e.step_us = hint_step_us;
      e.method = "hint";
      e.reliable = false;
    } else {
      const double r = est / hint_step_us;
      const double k = std::round(r);
      if (std::fabs(r - 1.0) <= 0.05) {
        e.step_us = est;
        e.method = "fundamental";
        e.reliable = true;
      } else if (k >= 2.0 && std::fabs(r - k) <= 0.06 * k) {
        e.step_us = est / k;
        e.method = "hint_divides_updates";
        e.reliable = true;
        e.warnings.push_back(format(
          "estimated %.0f us is %.0f x the --physics-hz step: updates always advance >= %.0f "
          "steps",
          est, k, k));
      } else {
        e.warnings.push_back(format(
          "estimated step %.0f us does not match --physics-hz step %.0f us; using the hint",
          est, hint_step_us));
        e.step_us = hint_step_us;
        e.method = "hint";
        e.reliable = false;
      }
    }
  }
  return e;
}

// =============================================================================
// Idle record
// =============================================================================

bool looks_idle(const IdleSignature & sig, double throttle, double brake)
{
  return std::fabs(throttle) <= sig.throttle_max &&
         std::fabs(brake - sig.brake_center) <= sig.brake_tol;
}

IdleCalibration calibrate_idle(
  const std::vector<std::pair<double, double>> & records, const IdleSignature & fallback)
{
  IdleCalibration c;
  c.signature = fallback;
  c.n = records.size();
  std::vector<double> t;
  std::vector<double> b;
  for (const auto & r : records) {
    t.push_back(r.first);
    b.push_back(r.second);
  }
  c.throttle_median = median_of(t);
  c.brake_median = median_of(b);
  if (c.n < 5) {
    c.warnings.push_back(format(
      "only %zu no-command records; idle signature = defaults (brake %.4f)", c.n,
      fallback.brake_center));
  } else if (!(std::fabs(c.throttle_median) <= fallback.throttle_max)) {
    c.warnings.push_back(format(
      "no-command records have throttle %.4f (not idle?); idle signature = defaults",
      c.throttle_median));
  } else if (c.brake_median > 0.0 && c.brake_median <= 0.05) {
    c.signature.brake_center = c.brake_median;
    c.source = "measured";
    if (std::fabs(c.brake_median - fallback.brake_center) > fallback.brake_tol) {
      c.warnings.push_back(format(
        "measured idle brake %.4f differs from the default %.4f; using the measurement",
        c.brake_median, fallback.brake_center));
    }
  } else if (c.brake_median == 0.0) {
    c.ambiguous = true;
    c.warnings.push_back(
      "no-command records have brake 0: indistinguishable from a zero-pedal command echo; "
      "idle fractions are only meaningful if driven records show throttle > 0");
  } else {
    c.warnings.push_back(format(
      "no-command records have brake %.4f (outside 0..0.05); idle signature = defaults",
      c.brake_median));
  }
  // 기본값 그대로 쓰는데 그 값이 0 이면 역시 구분 불가
  if (c.source == "default" && c.signature.brake_center - c.signature.brake_tol <= 0.0) {
    c.ambiguous = true;
  }
  return c;
}

// =============================================================================
// Per run
// =============================================================================

double GroupStat::idle_share() const
{
  return n_drive ? static_cast<double>(idle_drive) / static_cast<double>(n_drive) : kNaN;
}

double GroupStat::accel() const
{
  return (n_fit && dt_fit > 0.0) ? dv_fit / dt_fit : kNaN;
}

void GroupStat::merge(const GroupStat & o)
{
  n_drive += o.n_drive;
  idle_drive += o.idle_drive;
  n_fit += o.n_fit;
  dv_fit += o.dv_fit;
  dt_fit += o.dt_fit;
}

void KDvStat::add(int64_t k, double dv)
{
  const auto kd = static_cast<double>(k);
  ++n;
  sk += kd;
  skk += kd * kd;
  sdv += dv;
  sdv2 += dv * dv;
  skdv += kd * dv;
  ++k_counts[k];
}

void KDvStat::merge(const KDvStat & o)
{
  n += o.n;
  sk += o.sk;
  skk += o.skk;
  sdv += o.sdv;
  sdv2 += o.sdv2;
  skdv += o.skdv;
  for (const auto & kv : o.k_counts) {
    k_counts[kv.first] += kv.second;
  }
}

double KDvStat::slope() const
{
  const auto nd = static_cast<double>(n);
  const double den = nd * skk - sk * sk;
  if (n < 2 || !(den > 1e-9 * nd * nd)) {
    return kNaN;
  }
  return (nd * skdv - sk * sdv) / den;
}

double KDvStat::intercept() const
{
  const double b = slope();
  return std::isfinite(b) ? (sdv - b * sk) / static_cast<double>(n) : kNaN;
}

double KDvStat::slope_se() const
{
  const auto nd = static_cast<double>(n);
  if (n < 3 || !std::isfinite(slope())) {
    return kNaN;
  }
  const double sxx = skk - sk * sk / nd;
  const double sxy = skdv - sk * sdv / nd;
  const double syy = sdv2 - sdv * sdv / nd;
  if (!(sxx > 0.0)) {
    return kNaN;
  }
  // 잔차 제곱합 (합으로 계산해 생기는 음수 반올림 오차는 0 으로)
  const double ssr = std::max(0.0, syy - sxy * sxy / sxx);
  return std::sqrt(ssr / (nd - 2.0) / sxx);
}

double KDvStat::k_share() const
{
  const double b = slope();
  if (!std::isfinite(b) || n == 0) {
    return kNaN;
  }
  const double mean_dv = sdv / static_cast<double>(n);
  const double mean_k = sk / static_cast<double>(n);
  return mean_dv > 0.0 ? b * mean_k / mean_dv : kNaN;
}

double KDvStat::k_share_se() const
{
  const double se = slope_se();
  if (!std::isfinite(se) || n == 0) {
    return kNaN;
  }
  const double mean_dv = sdv / static_cast<double>(n);
  const double mean_k = sk / static_cast<double>(n);
  return mean_dv > 0.0 ? se * mean_k / mean_dv : kNaN;
}

bool KDvStat::spread_ok(size_t min_each) const
{
  size_t groups = 0;
  for (const auto & kv : k_counts) {
    groups += kv.second >= min_each ? 1 : 0;
  }
  return groups >= 2;
}

namespace
{

/// Sends credited per update: the (1 + lag)-th update after prev_morai_us (MORAI time).
std::vector<size_t> credit_sends(
  const RunInput & in, int lag, size_t * failed, size_t * unattributed)
{
  const auto & u = in.updates;
  std::vector<size_t> per(u.size(), 0);
  const auto extra = static_cast<size_t>(std::max(0, lag));
  for (const auto & s : in.sends) {
    if (!s.ok) {
      if (failed) {
        ++*failed;
      }
      continue;
    }
    const auto it = std::upper_bound(
      u.begin(), u.end(), s.prev_morai_us,
      [](int64_t v, const UpdateSample & x) { return v < x.morai_us; });
    const size_t idx = static_cast<size_t>(it - u.begin()) + extra;
    if (idx >= u.size()) {
      if (unattributed) {
        ++*unattributed;
      }
      continue;
    }
    ++per[idx];
  }
  return per;
}

/// Calls f(i, in_fit, dt_s, dv) for every update i with MORAI time in (0, drive_s] after t0.
template <typename F>
size_t for_each_drive_update(const RunInput & in, F && f)
{
  const auto & u = in.updates;
  size_t non_monotonic = 0;
  for (size_t i = 1; i < u.size(); ++i) {
    const int64_t d = u[i].morai_us - u[i - 1].morai_us;
    if (d <= 0) {
      ++non_monotonic;
      continue;
    }
    const double t = static_cast<double>(u[i].morai_us - in.t0_us) / 1e6;
    if (!(t > 0.0 && t <= in.drive_s)) {
      continue;
    }
    f(i, t >= in.fit_start_s, static_cast<double>(d) / 1e6, u[i].speed_mps - u[i - 1].speed_mps);
  }
  return non_monotonic;
}

}  // namespace

RunAnalysis analyze_run(const RunInput & in)
{
  RunAnalysis a;
  a.aborted = in.aborted;
  a.lag = in.lag;
  a.commanded_accel = in.commanded_accel;
  a.step_us = in.step_us;
  const auto & u = in.updates;
  a.updates_total = u.size();

  // 송신 귀속: prev_morai_us 보다 뒤인 (1 + lag) 번째 업데이트 (MORAI 시간 기준)
  const std::vector<size_t> sends_per =
    credit_sends(in, in.lag, &a.sends_failed, &a.sends_unattributed);

  std::vector<double> fx;
  std::vector<double> fy;
  std::vector<double> gaps_ms;
  double path_m = 0.0;
  double integ_m = 0.0;
  size_t non_monotonic = 0;
  const double drive_s = in.drive_s;

  for (size_t i = 1; i < u.size(); ++i) {
    const int64_t d = u[i].morai_us - u[i - 1].morai_us;
    if (d <= 0) {
      ++non_monotonic;
      continue;
    }
    const double t = static_cast<double>(u[i].morai_us - in.t0_us) / 1e6;
    if (!(t > 0.0 && t <= drive_s)) {
      continue;
    }
    const bool in_fit = t >= in.fit_start_s;
    const int64_t k = steps_for(d, in.step_us);
    const bool idle = looks_idle(in.idle, u[i].throttle, u[i].brake);
    const size_t s = sends_per[i];
    const double dt = static_cast<double>(d) / 1e6;
    const double dv = u[i].speed_mps - u[i - 1].speed_mps;

    ++a.updates_drive;
    a.idle_drive += idle ? 1 : 0;
    ++a.steps_hist[k];
    a.steps_total += static_cast<size_t>(k);
    ++a.sends_hist[static_cast<int64_t>(std::min<size_t>(s, 5))];
    a.sends_in_drive += s;
    gaps_ms.push_back(dt * 1e3);

    GroupStat * step_group = s >= 1 ? (k == 1 ? &a.single : &a.multi) : nullptr;
    GroupStat & send_group = s == 0 ? a.zero_send : a.some_send;
    if (step_group) {
      ++step_group->n_drive;
      step_group->idle_drive += idle ? 1 : 0;
    }
    ++send_group.n_drive;
    send_group.idle_drive += idle ? 1 : 0;

    if (in_fit) {
      ++a.updates_fit;
      a.idle_fit += idle ? 1 : 0;
      fx.push_back(t);
      fy.push_back(u[i].speed_mps);
      if (step_group) {
        ++step_group->n_fit;
        step_group->dv_fit += dv;
        step_group->dt_fit += dt;
        a.kdv.add(k, dv);
      }
      ++send_group.n_fit;
      send_group.dv_fit += dv;
      send_group.dt_fit += dt;
      path_m += std::hypot(u[i].x - u[i - 1].x, u[i].y - u[i - 1].y);
      integ_m += 0.5 * (std::fabs(u[i].speed_mps) + std::fabs(u[i - 1].speed_mps)) * dt;
    }
  }

  a.fit = fit_line(fx, fy);
  a.realised_accel = a.fit.slope;
  a.ratio = ratio_or_nan(a.realised_accel, in.commanded_accel);
  a.idle_fraction =
    a.updates_drive ? static_cast<double>(a.idle_drive) / static_cast<double>(a.updates_drive) : kNaN;
  a.idle_fraction_fit =
    a.updates_fit ? static_cast<double>(a.idle_fit) / static_cast<double>(a.updates_fit) : kNaN;
  a.steps_per_update = a.updates_drive ? static_cast<double>(a.steps_total) /
                                           static_cast<double>(a.updates_drive)
                                       : kNaN;
  a.update_step_coverage = a.steps_total ? static_cast<double>(a.updates_drive) /
                                             static_cast<double>(a.steps_total)
                                         : kNaN;
  a.sends_per_update = a.updates_drive ? static_cast<double>(a.sends_in_drive) /
                                           static_cast<double>(a.updates_drive)
                                       : kNaN;
  a.multi_step_accel_ratio = ratio_or_nan(a.multi.accel(), a.single.accel());
  a.zero_send_accel_ratio = ratio_or_nan(a.zero_send.accel(), a.some_send.accel());
  a.k_share = a.kdv.k_share();
  a.median_gap_ms = median_of(gaps_ms);
  a.max_gap_ms = gaps_ms.empty() ? kNaN : *std::max_element(gaps_ms.begin(), gaps_ms.end());
  if (std::isfinite(a.median_gap_ms)) {
    a.stalls = static_cast<size_t>(std::count_if(
      gaps_ms.begin(), gaps_ms.end(), [&](double g) { return g > 2.5 * a.median_gap_ms; }));
  }
  a.distance_ratio = integ_m > 0.05 ? path_m / integ_m : kNaN;

  if (a.fit.n < 5) {
    a.warnings.push_back(format("only %zu updates in the fit window", a.fit.n));
  }
  if (non_monotonic) {
    a.warnings.push_back(format("%zu non-increasing timestamps skipped", non_monotonic));
  }
  if (a.sends_unattributed) {
    a.warnings.push_back(format("%zu sends after the last update", a.sends_unattributed));
  }
  if (std::isfinite(a.distance_ratio) && std::fabs(a.distance_ratio - 1.0) > 0.2) {
    a.warnings.push_back(format(
      "path length / integral of speed = %.3f (expected ~1): check the km/h -> m/s conversion",
      a.distance_ratio));
  }
  if (!(in.step_us > 0.0)) {
    a.warnings.push_back("no physics step: steps per update assume 1");
  }
  return a;
}

// =============================================================================
// Attribution lag / clock identity
// =============================================================================

namespace
{

/// a = sum dv / sum dt and its standard error from the per-update residuals.
void group_accel(const std::vector<std::pair<double, double>> & g, double * a, double * se)
{
  double sdv = 0.0;
  double sdt = 0.0;
  for (const auto & x : g) {
    sdv += x.first;
    sdt += x.second;
  }
  *a = (g.empty() || !(sdt > 0.0)) ? kNaN : sdv / sdt;
  *se = kNaN;
  if (g.size() < 3 || !std::isfinite(*a)) {
    return;
  }
  double ss = 0.0;
  for (const auto & x : g) {
    const double r = x.first - *a * x.second;
    ss += r * r;
  }
  const auto n = static_cast<double>(g.size());
  *se = std::sqrt(ss / (n - 1.0) * n) / sdt;
}

double share_se(double p1, size_t n1, double p2, size_t n2)
{
  if (!n1 || !n2 || !std::isfinite(p1) || !std::isfinite(p2)) {
    return kNaN;
  }
  // 0/1 비율에서 SE 가 0 이 되지 않게 (n+2) 보정
  const double q1 = (p1 * static_cast<double>(n1) + 1.0) / (static_cast<double>(n1) + 2.0);
  const double q2 = (p2 * static_cast<double>(n2) + 1.0) / (static_cast<double>(n2) + 2.0);
  return std::sqrt(
    q1 * (1.0 - q1) / static_cast<double>(n1) + q2 * (1.0 - q2) / static_cast<double>(n2));
}

/// Smallest lag whose value is within 2 SE of the best; *decided when it clears the bars.
int pick_lag(
  const std::vector<LagCandidate> & c, double LagCandidate::*value, double LagCandidate::*se,
  bool (*usable)(const LagCandidate &), double min_value, bool * decided)
{
  *decided = false;
  const LagCandidate * best = nullptr;
  for (const auto & x : c) {
    if (usable(x) && std::isfinite(x.*value) && (!best || x.*value > best->*value)) {
      best = &x;
    }
  }
  if (!best) {
    return 0;
  }
  const LagCandidate * pick = best;
  for (const auto & x : c) {
    if (!usable(x) || !std::isfinite(x.*value)) {
      continue;
    }
    const double tol = 2.0 * std::max(std::isfinite(x.*se) ? x.*se : 0.0,
                                      std::isfinite(best->*se) ? best->*se : 0.0);
    if (x.*value >= best->*value - tol) {
      pick = &x;  // 오름차순이라 처음 걸린 것이 가장 작은 지연
      break;
    }
  }
  const double se_pick = std::isfinite(pick->*se) ? pick->*se : 0.0;
  *decided = pick->*value >= std::max(min_value, 3.0 * se_pick);
  return pick->lag;
}

}  // namespace

LagEstimate estimate_lag(const std::vector<RunInput> & runs, int max_lag)
{
  LagEstimate e;
  max_lag = std::clamp(max_lag, 0, 4);
  std::vector<const RunInput *> use;
  double a_cmd = kNaN;
  for (const auto & in : runs) {
    if (in.aborted) {
      continue;
    }
    const std::vector<size_t> per = credit_sends(in, 0, nullptr, nullptr);
    size_t n = 0;
    size_t zero = 0;
    for_each_drive_update(in, [&](size_t i, bool, double, double) {
      ++n;
      zero += per[i] == 0 ? 1 : 0;
    });
    const double z = n ? static_cast<double>(zero) / static_cast<double>(n) : kNaN;
    if (n < 20 || !(z >= 0.2 && z <= 0.8)) {
      continue;
    }
    use.push_back(&in);
    if (!std::isfinite(a_cmd)) {
      a_cmd = std::fabs(in.commanded_accel);
    }
  }
  e.runs_used = use.size();
  if (use.empty()) {
    e.warnings.push_back(
      "no run with 20-80% command-less updates (locked_half / free33): attribution lag not "
      "estimated, 0 assumed");
    return e;
  }

  for (int lag = 0; lag <= max_lag; ++lag) {
    LagCandidate c;
    c.lag = lag;
    std::vector<std::pair<double, double>> gc;
    std::vector<std::pair<double, double>> go;
    size_t ni_c = 0;
    size_t ni_o = 0;
    for (const RunInput * in : use) {
      const std::vector<size_t> per = credit_sends(*in, lag, nullptr, nullptr);
      for_each_drive_update(*in, [&](size_t i, bool in_fit, double dt, double dv) {
        const bool credited = per[i] > 0;
        const bool nonidle = !looks_idle(in->idle, in->updates[i].throttle, in->updates[i].brake);
        ++(credited ? c.n_credited_drive : c.n_other_drive);
        (credited ? ni_c : ni_o) += nonidle ? 1 : 0;
        if (in_fit) {
          (credited ? gc : go).emplace_back(dv, dt);
        }
      });
    }
    c.n_credited = gc.size();
    c.n_other = go.size();
    double se_c = kNaN;
    double se_o = kNaN;
    group_accel(gc, &c.accel_credited, &se_c);
    group_accel(go, &c.accel_other, &se_o);
    c.accel_contrast = c.accel_credited - c.accel_other;
    c.accel_contrast_se = std::hypot(se_c, se_o);
    c.nonidle_credited = c.n_credited_drive ? static_cast<double>(ni_c) /
                                                static_cast<double>(c.n_credited_drive)
                                            : kNaN;
    c.nonidle_other =
      c.n_other_drive ? static_cast<double>(ni_o) / static_cast<double>(c.n_other_drive) : kNaN;
    c.record_contrast = c.nonidle_credited - c.nonidle_other;
    c.record_contrast_se =
      share_se(c.nonidle_credited, c.n_credited_drive, c.nonidle_other, c.n_other_drive);
    e.candidates.push_back(c);
  }

  e.physics_lag = pick_lag(
    e.candidates, &LagCandidate::accel_contrast, &LagCandidate::accel_contrast_se,
    [](const LagCandidate & c) { return c.n_credited >= 20 && c.n_other >= 20; },
    0.05 * (std::isfinite(a_cmd) ? a_cmd : 0.0), &e.physics_decided);
  e.record_lag = pick_lag(
    e.candidates, &LagCandidate::record_contrast, &LagCandidate::record_contrast_se,
    [](const LagCandidate & c) { return c.n_credited_drive >= 20 && c.n_other_drive >= 20; },
    0.2, &e.record_decided);
  if (!e.physics_decided) {
    e.physics_lag = 0;
    e.warnings.push_back(
      "no lag separates commanded from command-less updates in acceleration (MORAI holds the "
      "command, or too little data): 0 assumed");
  } else if (e.physics_lag != 0) {
    e.warnings.push_back(format(
      "sends act %d update(s) later than the default attribution: analysis uses lag %d",
      e.physics_lag, e.physics_lag));
  }
  if (e.physics_decided && e.record_decided && e.record_lag != e.physics_lag) {
    e.warnings.push_back(format(
      "record lag %d != physics lag %d: vehicle_state shows a command at a different update "
      "than the one it drives; idle-record evidence is grouped by the physics lag",
      e.record_lag, e.physics_lag));
  }
  return e;
}

WftMatch match_wft_values(const std::vector<int64_t> & wft_values, std::vector<int64_t> reader_ts)
{
  WftMatch m;
  std::sort(reader_ts.begin(), reader_ts.end());
  for (int64_t v : wft_values) {
    if (reader_ts.empty() || v < reader_ts.front() || v > reader_ts.back()) {
      ++m.out_of_range;
      continue;
    }
    ++m.n;
    m.found += std::binary_search(reader_ts.begin(), reader_ts.end(), v) ? 1 : 0;
  }
  m.share = m.n ? static_cast<double>(m.found) / static_cast<double>(m.n) : kNaN;
  return m;
}

// =============================================================================
// JSON
// =============================================================================

Json to_json(const LinearFit & f)
{
  return Json::object()
    .set("n", f.n)
    .set("slope", f.slope)
    .set("intercept", f.intercept)
    .set("r2", f.r2);
}

Json to_json(const StepEstimate & s)
{
  Json cl = Json::array();
  for (const auto & c : s.clusters) {
    cl.push(Json::object().set("center_us", c.center_us).set("count", c.count));
  }
  return Json::object()
    .set("step_us", s.step_us)
    .set("step_ms", s.step_us / 1e3)
    .set("physics_hz", 1e6 / s.step_us)
    .set("reliable", s.reliable)
    .set("method", s.method)
    .set("divisor", s.divisor)
    .set("n_deltas", s.n_deltas)
    .set("hint_step_us", s.hint_step_us)
    .set("clusters", std::move(cl))
    .set("warnings", strings_json(s.warnings));
}

Json to_json(const IdleCalibration & c)
{
  return Json::object()
    .set("n", c.n)
    .set("throttle_median", c.throttle_median)
    .set("brake_median", c.brake_median)
    .set("source", c.source)
    .set("ambiguous", c.ambiguous)
    .set(
      "signature", Json::object()
                     .set("throttle_max", c.signature.throttle_max)
                     .set("brake_center", c.signature.brake_center)
                     .set("brake_tol", c.signature.brake_tol))
    .set("warnings", strings_json(c.warnings));
}

Json to_json(const GroupStat & g)
{
  return Json::object()
    .set("n_drive", g.n_drive)
    .set("idle_drive", g.idle_drive)
    .set("idle_share", g.idle_share())
    .set("n_fit", g.n_fit)
    .set("dv_fit", g.dv_fit)
    .set("dt_fit", g.dt_fit)
    .set("accel", g.accel());
}

Json to_json(const KDvStat & k)
{
  return Json::object()
    .set("n", k.n)
    .set("slope_mps_per_step", k.slope())
    .set("slope_se", k.slope_se())
    .set("intercept_mps", k.intercept())
    .set("k_share", k.k_share())
    .set("k_share_se", k.k_share_se())
    .set("k_counts", hist_json(k.k_counts));
}

Json to_json(const LagEstimate & e)
{
  Json c = Json::array();
  for (const auto & x : e.candidates) {
    c.push(Json::object()
             .set("lag", x.lag)
             .set("n_credited_fit", x.n_credited)
             .set("n_other_fit", x.n_other)
             .set("accel_credited", x.accel_credited)
             .set("accel_other", x.accel_other)
             .set("accel_contrast", x.accel_contrast)
             .set("accel_contrast_se", x.accel_contrast_se)
             .set("n_credited_drive", x.n_credited_drive)
             .set("n_other_drive", x.n_other_drive)
             .set("nonidle_credited", x.nonidle_credited)
             .set("nonidle_other", x.nonidle_other)
             .set("record_contrast", x.record_contrast)
             .set("record_contrast_se", x.record_contrast_se));
  }
  return Json::object()
    .set("runs_used", e.runs_used)
    .set("physics_lag", e.physics_lag)
    .set("physics_decided", e.physics_decided)
    .set("record_lag", e.record_lag)
    .set("record_decided", e.record_decided)
    .set("candidates", std::move(c))
    .set("warnings", strings_json(e.warnings));
}

Json to_json(const WftMatch & m)
{
  return Json::object()
    .set("n", m.n)
    .set("found", m.found)
    .set("out_of_range", m.out_of_range)
    .set("share", m.share);
}

Json to_json(const RunAnalysis & a)
{
  return Json::object()
    .set("aborted", a.aborted)
    .set("attribution_lag", a.lag)
    .set("commanded_accel", a.commanded_accel)
    .set("step_us", a.step_us)
    .set("updates_total", a.updates_total)
    .set("updates_drive", a.updates_drive)
    .set("updates_fit", a.updates_fit)
    .set("fit", to_json(a.fit))
    .set("realised_accel", a.realised_accel)
    .set("ratio", a.ratio)
    .set("idle_drive", a.idle_drive)
    .set("idle_fraction", a.idle_fraction)
    .set("idle_fit", a.idle_fit)
    .set("idle_fraction_fit", a.idle_fraction_fit)
    .set("steps_hist", hist_json(a.steps_hist))
    .set("steps_total", a.steps_total)
    .set("steps_per_update", a.steps_per_update)
    .set("update_step_coverage", a.update_step_coverage)
    .set("sends_hist", hist_json(a.sends_hist))
    .set("sends_in_drive", a.sends_in_drive)
    .set("sends_failed", a.sends_failed)
    .set("sends_unattributed", a.sends_unattributed)
    .set("sends_per_update", a.sends_per_update)
    .set("single_step_sent", to_json(a.single))
    .set("multi_step_sent", to_json(a.multi))
    .set("zero_send", to_json(a.zero_send))
    .set("some_send", to_json(a.some_send))
    .set("multi_step_accel_ratio", a.multi_step_accel_ratio)
    .set("zero_send_accel_ratio", a.zero_send_accel_ratio)
    .set("dv_vs_k", to_json(a.kdv))
    .set("k_share", a.k_share)
    .set("median_gap_ms", a.median_gap_ms)
    .set("max_gap_ms", a.max_gap_ms)
    .set("stalls", a.stalls)
    .set("distance_ratio", a.distance_ratio)
    .set("warnings", strings_json(a.warnings));
}

// =============================================================================
// Pattern summary / interpretation
// =============================================================================

PatternSummary summarize_pattern(
  const std::string & pattern, const std::vector<const RunAnalysis *> & runs)
{
  PatternSummary p;
  p.pattern = pattern;
  std::vector<double> idle;
  std::vector<double> spu;
  std::vector<double> steps;
  std::vector<double> cov;
  for (const RunAnalysis * r : runs) {
    if (!r) {
      continue;
    }
    ++p.runs;
    if (r->aborted) {
      ++p.aborted;
      continue;
    }
    if (r->fit.n < 5 || !std::isfinite(r->ratio)) {
      continue;
    }
    ++p.valid;
    p.ratios.push_back(r->ratio);
    if (std::isfinite(r->idle_fraction)) idle.push_back(r->idle_fraction);
    if (std::isfinite(r->sends_per_update)) spu.push_back(r->sends_per_update);
    if (std::isfinite(r->steps_per_update)) steps.push_back(r->steps_per_update);
    if (std::isfinite(r->update_step_coverage)) cov.push_back(r->update_step_coverage);
    p.stalls += r->stalls;
    p.single.merge(r->single);
    p.multi.merge(r->multi);
    p.zero_send.merge(r->zero_send);
    p.some_send.merge(r->some_send);
    p.kdv.merge(r->kdv);
  }
  p.ratio_mean = mean_of(p.ratios);
  p.ratio_sd = sd_of(p.ratios);
  p.idle_mean = mean_of(idle);
  p.sends_per_update_mean = mean_of(spu);
  p.steps_per_update_mean = mean_of(steps);
  p.coverage_mean = mean_of(cov);
  p.multi_step_accel_ratio = ratio_or_nan(p.multi.accel(), p.single.accel());
  p.zero_send_accel_ratio = ratio_or_nan(p.zero_send.accel(), p.some_send.accel());
  p.k_share = p.kdv.k_share();
  p.k_share_se = p.kdv.k_share_se();
  return p;
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
    .set("idle_mean", p.idle_mean)
    .set("sends_per_update_mean", p.sends_per_update_mean)
    .set("steps_per_update_mean", p.steps_per_update_mean)
    .set("update_step_coverage_mean", p.coverage_mean)
    .set("stalls", p.stalls)
    .set("single_step_sent", to_json(p.single))
    .set("multi_step_sent", to_json(p.multi))
    .set("zero_send", to_json(p.zero_send))
    .set("some_send", to_json(p.some_send))
    .set("multi_step_accel_ratio", p.multi_step_accel_ratio)
    .set("zero_send_accel_ratio", p.zero_send_accel_ratio)
    .set("dv_vs_k", to_json(p.kdv))
    .set("k_share", p.k_share)
    .set("k_share_se", p.k_share_se);
}

Interpretation interpret(const std::vector<PatternSummary> & patterns, bool idle_ambiguous)
{
  Interpretation it;
  GroupStat single;
  GroupStat multi;
  GroupStat zero;
  GroupStat some;
  KDvStat kdv;
  std::map<std::string, const PatternSummary *> by_name;
  for (const auto & p : patterns) {
    single.merge(p.single);
    multi.merge(p.multi);
    zero.merge(p.zero_send);
    some.merge(p.some_send);
    kdv.merge(p.kdv);
    by_name[p.pattern] = &p;
  }
  it.multi_step_accel_ratio = ratio_or_nan(multi.accel(), single.accel());
  it.idle_share_single = single.idle_share();
  it.idle_share_multi = multi.idle_share();
  it.zero_send_accel_ratio = ratio_or_nan(zero.accel(), some.accel());
  it.k_share = kdv.k_share();
  it.k_share_se = kdv.k_share_se();
  it.n_kdv = kdv.n;
  it.n_single_fit = single.n_fit;
  it.n_single_drive = single.n_drive;
  it.n_multi_fit = multi.n_fit;
  it.n_multi_drive = multi.n_drive;

  std::string ratios;
  for (const auto & p : patterns) {
    ratios += (ratios.empty() ? "" : ", ") + p.pattern + " " + fmt2(p.ratio_mean);
  }
  it.lines.push_back("realised/commanded: " + (ratios.empty() ? std::string("-") : ratios));

  std::string ks;
  for (const auto & kv : kdv.k_counts) {
    ks += (ks.empty() ? "" : " ") + format("k%lld:%zu", static_cast<long long>(kv.first), kv.second);
  }

  // 1) 물리 표: 명령을 받은 업데이트의 Δv 가 진행 스텝 수 k 에 비례하는가 — 판정에 필수.
  //    표준오차를 넘는 차이만 표로 친다 (200 Hz 스텝당이면 평균 Δv 가 작아 잡음이 커진다)
  const double share = it.k_share;
  const double se = it.k_share_se;
  const bool k_enough = kdv.n >= 20 && kdv.spread_ok(5);
  it.k_uniform = kdv.n >= 20 && !kdv.spread_ok(5);
  if (k_enough && std::isfinite(share) && std::isfinite(se)) {
    if (share >= 0.75 && share - 2.0 * se > 0.35) {
      it.k_vote = "per_update";
    } else if (share <= 0.35 && share + 2.0 * se < 0.75) {
      it.k_vote = "per_step";
    }
    it.lines.push_back(format(
      "dv~k share = %.2f+-%.2f (1 SE) over %zu commanded updates (%s): per update ~1 (every "
      "step driven), per step ~0 (one step driven) -> %s",
      share, se, kdv.n, ks.c_str(),
      it.k_vote == "none" ? "no vote (within 2 SE of the other model's range)"
                          : ("vote " + it.k_vote).c_str()));
  } else {
    it.lines.push_back(format(
      "dv~k share: not decidable (%zu commanded updates, k %s) - need >= 2 k values x 5",
      kdv.n, ks.empty() ? "-" : ks.c_str()));
  }
  if (single.n_fit >= 10 && multi.n_fit >= 10 && std::isfinite(it.multi_step_accel_ratio)) {
    it.lines.push_back(format(
      "a(k>=2)/a(k=1) = %.2f (%zu/%zu updates; per update ~1, per step ~1/k)",
      it.multi_step_accel_ratio, multi.n_fit, single.n_fit));
  }

  // 2) 유휴 레코드 표: 확인 또는 강등만. 양성 대조 — 명령을 받은 k=1 업데이트의 레코드가
  //    유휴로 보이지 않아야(<= 10%) 레코드가 소비된 명령을 보여 준다고 믿을 수 있다
  if (idle_ambiguous) {
    it.idle_vote_note = "idle signature ambiguous (idle brake 0)";
  } else if (single.n_drive < 10) {
    it.idle_vote_note = format(
      "no positive control: %zu commanded k=1 updates (< 10; e.g. 200 Hz has none)",
      single.n_drive);
  } else if (!(it.idle_share_single <= 0.1)) {
    it.idle_vote_note = format(
      "positive control failed: %.1f%% of %zu commanded k=1 records look idle (> 10%%) - the "
      "record does not reliably show a consumed command",
      100.0 * it.idle_share_single, single.n_drive);
  } else if (multi.n_drive < 10 || !std::isfinite(it.idle_share_multi)) {
    it.idle_vote_note = format("%zu commanded k>=2 updates (< 10)", multi.n_drive);
  } else if (it.idle_share_multi <= 0.1) {
    it.idle_vote = "per_update";
  } else if (it.idle_share_multi >= 0.5) {
    it.idle_vote = "per_step";
  } else {
    it.idle_vote = "neutral";
  }
  if (it.idle_vote == "unavailable") {
    it.lines.push_back("idle record vote unavailable: " + it.idle_vote_note);
  } else {
    it.lines.push_back(format(
      "idle record share of commanded updates: k>=2 %.1f%% (%zu), k=1 %.1f%% (%zu, control) "
      "-> %s",
      100.0 * it.idle_share_multi, multi.n_drive, 100.0 * it.idle_share_single, single.n_drive,
      it.idle_vote.c_str()));
  }

  if (it.k_vote == "none") {
    it.verdict = "inconclusive";
  } else if (
    (it.idle_vote == "per_update" || it.idle_vote == "per_step") && it.idle_vote != it.k_vote) {
    it.verdict = "inconclusive";
    it.lines.push_back(
      "physics (" + it.k_vote + ") and idle records (" + it.idle_vote + ") disagree");
  } else {
    it.verdict = it.k_vote;
    it.confirmed = it.idle_vote == it.k_vote;
  }

  // 유지(hold) 여부: 명령이 없던 업데이트의 가속 / 명령이 있던 업데이트의 가속
  const double z = it.zero_send_accel_ratio;
  if (zero.n_fit >= 10 && some.n_fit >= 10 && std::isfinite(z)) {
    it.hold = z <= 0.3 ? "no_hold" : (z >= 0.7 ? "holds" : "unknown");
    it.lines.push_back(format(
      "a(no send)/a(>=1 send) = %.2f over %zu/%zu updates -> %s", z, zero.n_fit, some.n_fit,
      it.hold == "no_hold" ? "no hold (an update without a command is not driven)"
                           : (it.hold == "holds" ? "MORAI holds the last command" : "unclear")));
  }

  auto get = [&](const char * name) -> const PatternSummary * {
    const auto f = by_name.find(name);
    return f == by_name.end() ? nullptr : f->second;
  };
  auto ratio_of = [&](const char * name) {
    const PatternSummary * p = get(name);
    return p ? p->ratio_mean : kNaN;
  };
  const double l1 = ratio_of("locked1");
  const double b4 = ratio_of("burst4");
  const double f33 = ratio_of("free33");
  if (it.verdict == "per_update") {
    it.lines.push_back(std::string(
      "=> consumed per UPDATE: every physics step of an update uses that update's command; "
      "one command per update is enough and a higher MORAI Hz does not create empty steps") +
      (it.confirmed ? " (confirmed by idle records)"
                    : " (physics only; idle records do not confirm)"));
    if (std::isfinite(l1) && std::isfinite(f33) && l1 > f33 + 0.1) {
      it.lines.push_back(format(
        "   locked1 %.2f >> free33 %.2f: updates without a command are lost (send >= 1 per "
        "update: frame-lock or >= 2x the update rate)",
        l1, f33));
    }
  } else if (it.verdict == "per_step") {
    it.lines.push_back(std::string(
      "=> consumed per PHYSICS STEP without hold: an update that advances k steps drives only "
      "one of them; a higher MORAI Hz leaves more empty steps whatever the send rate") +
      (it.confirmed ? " (confirmed by idle records)"
                    : " (physics only; idle records do not confirm)"));
    if (std::isfinite(l1) && std::isfinite(b4) && b4 > l1 + 0.1) {
      it.lines.push_back(format(
        "   burst4 %.2f > locked1 %.2f: MORAI queues commands per step — sending k per update "
        "fills the steps",
        b4, l1));
    }
  } else {
    if (it.k_uniform) {
      const int64_t k = std::max_element(
                          kdv.k_counts.begin(), kdv.k_counts.end(),
                          [](const auto & a, const auto & b) { return a.second < b.second; })
                          ->first;
      it.lines.push_back(format(
        "=> inconclusive at this Hz: (nearly) every commanded update advanced k=%lld steps, so "
        "dv~k cannot separate the models",
        static_cast<long long>(k)));
    } else if (it.k_vote == "none" && k_enough) {
      it.lines.push_back(
        "=> inconclusive: dv~k is within noise of both models (see +-SE above)");
    } else if (it.k_vote == "none") {
      it.lines.push_back("=> inconclusive: too few commanded updates for dv~k");
    } else {
      it.lines.push_back("=> inconclusive: physics and idle-record evidence disagree");
    }
    const PatternSummary * p1 = get("locked1");
    const double cov = p1 ? p1->coverage_mean : kNaN;
    it.lines.push_back(format(
      "   compare with the 60 Hz run: locked1 ratio here %s, updates/steps here %s. Per update "
      "keeps locked1's ratio (~ the 60 Hz one); per step scales it by updates/steps "
      "(ratio_60Hz x %s / updates_per_step_60Hz, the latter ~0.83)",
      fmt2(l1).c_str(), fmt2(cov).c_str(), fmt2(cov).c_str()));
  }
  return it;
}

Json to_json(const Interpretation & i)
{
  return Json::object()
    .set("verdict", i.verdict)
    .set("confirmed", i.confirmed)
    .set("hold", i.hold)
    .set("k_vote", i.k_vote)
    .set("k_uniform", i.k_uniform)
    .set("idle_vote", i.idle_vote)
    .set("idle_vote_note", i.idle_vote_note)
    .set("multi_step_accel_ratio", i.multi_step_accel_ratio)
    .set("idle_share_single", i.idle_share_single)
    .set("idle_share_multi", i.idle_share_multi)
    .set("zero_send_accel_ratio", i.zero_send_accel_ratio)
    .set("k_share", i.k_share)
    .set("k_share_se", i.k_share_se)
    .set("n_kdv", i.n_kdv)
    .set("n_single_fit", i.n_single_fit)
    .set("n_single_drive", i.n_single_drive)
    .set("n_multi_fit", i.n_multi_fit)
    .set("n_multi_drive", i.n_multi_drive)
    .set("lines", strings_json(i.lines));
}

// =============================================================================
// Envelope
// =============================================================================

bool steer_echo(double steer, double front_wheel_angle)
{
  return !(std::fabs(steer) <= kSteerEchoTol) || !(std::fabs(front_wheel_angle) <= kSteerEchoTol);
}

std::string check_envelope(const EnvelopeLimits & l, const EnvelopeState & s)
{
  if (!s.have_state) {
    return "no Ego state yet";
  }
  if (!(s.stale_s <= l.max_stale_s)) {
    return format("state reads failing for %.2f s (> %.2f s)", s.stale_s, l.max_stale_s);
  }
  if (!(s.frozen_s <= l.max_frozen_s)) {
    return format("MORAI time frozen for %.2f s (> %.2f s)", s.frozen_s, l.max_frozen_s);
  }
  if (!(s.speed_mps <= l.max_speed_mps)) {
    return format("speed %.3f m/s > --max-speed %.3f", s.speed_mps, l.max_speed_mps);
  }
  if (!(s.distance_m <= l.max_distance_m)) {
    return format(
      "displacement %.2f m from the pattern start > --max-distance %.2f", s.distance_m,
      l.max_distance_m);
  }
  if (s.collisions > 0) {
    return format("MORAI reports %zu collision object(s)", s.collisions);
  }
  if (!(s.steer_echo <= l.max_steer_echo)) {
    return format(
      "vehicle_state steer echo %.3f although this tool sends steer 0: another client is "
      "commanding the Ego",
      s.steer_echo);
  }
  return "";
}

double estimate_run_travel_m(double accel, double drive_s, double brake_decel)
{
  const double v = accel * drive_s;
  const double b = brake_decel > 0.0 ? brake_decel : 1.0;
  return 0.5 * accel * drive_s * drive_s + v * v / (2.0 * b);
}

// =============================================================================
// Preflight
// =============================================================================

std::string original_mode_refusal(int mode, const std::string & mode_name, bool allow_any)
{
  if (mode == kModeKeyboard || mode == kModeGameWheel || allow_any) {
    return "";
  }
  return "the Ego's control mode is " + mode_name +
         ", not KEYBOARD / GAME_WHEEL: a bridge or Autoware may be commanding it (their "
         "zero-pedal ACCELERATION commands look idle). Stop them, switch MORAI to KEYBOARD, or "
         "pass --allow-any-original-mode if you are sure nobody else sends";
}

ForeignInputCheck check_foreign_input(const std::vector<InputRecord> & records, double share_limit)
{
  ForeignInputCheck c;
  c.n = records.size();
  for (const auto & r : records) {
    const bool pedal = r.throttle > 0.01 || r.brake > 0.05;
    const bool steer = steer_echo(r.steer, r.fwa);
    c.pedal += pedal ? 1 : 0;
    c.steer += steer ? 1 : 0;
    c.active += (pedal || steer) ? 1 : 0;
  }
  c.active_share = c.n ? static_cast<double>(c.active) / static_cast<double>(c.n) : kNaN;
  c.driving = c.n > 0 && c.active_share >= share_limit;
  if (c.driving) {
    c.why = format(
      "someone else seems to be driving the Ego (%zu/%zu records with input: %zu pedal, %zu "
      "steer echo) - stop the bridge / Autoware / keyboard",
      c.active, c.n, c.pedal, c.steer);
  }
  return c;
}

}  // namespace morai_grpc_vendor::actuate
