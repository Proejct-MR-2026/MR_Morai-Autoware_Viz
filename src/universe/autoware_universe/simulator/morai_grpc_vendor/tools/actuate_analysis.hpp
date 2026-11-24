// 신규 — 26.09.25 _ YSH _ morai_actuate 순수 계산부 (가속 적합·유휴 레코드·스텝/업데이트·송신/업데이트·판정, gRPC 비의존)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_ANALYSIS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_ANALYSIS_HPP_

#include "probe_json.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::actuate
{

using probe::Json;

/// main.py KMPH_TO_MPS: ActorState.velocity is km/h.
constexpr double kKmhToMps = 1.0 / 3.6;

// =============================================================================
// Least squares
// =============================================================================

struct LinearFit
{
  size_t n{0};
  double slope{0.0};      ///< NaN if n < 2 or x has no spread
  double intercept{0.0};  ///< NaN likewise
  double r2{0.0};         ///< NaN if y has no spread
};

LinearFit fit_line(const std::vector<double> & x, const std::vector<double> & y);

/// Mean / sample standard deviation (n-1). NaN when undefined.
double mean_of(const std::vector<double> & v);
double sd_of(const std::vector<double> & v);

// =============================================================================
// Physics step from ActorStates.timestamp deltas
// =============================================================================

struct DeltaCluster
{
  double center_us{0.0};
  size_t count{0};
};

struct StepEstimate
{
  /// Physics step, µs. NaN if nothing could be estimated.
  double step_us{0.0};
  /// false: too few deltas, or every frequent delta is the same multiple (a divisor cannot
  /// be ruled out) and no --physics-hz hint confirmed it.
  bool reliable{false};
  /// "fundamental" (largest step every frequent delta cluster is a multiple of),
  /// "hint", "hint_divides_updates", "insufficient".
  std::string method{"insufficient"};
  /// Divisor applied to the smallest frequent cluster (1 = the smallest cluster is one step).
  int divisor{0};
  size_t n_deltas{0};
  double hint_step_us{0.0};  ///< NaN when no hint
  /// Frequent clusters (>= 2% of the deltas), ascending.
  std::vector<DeltaCluster> clusters;
  std::vector<std::string> warnings;
};

/// `deltas_us`: differences between successive DISTINCT timestamps. `hint_step_us` > 0
/// is the step implied by the MORAI Hz the user configured (--physics-hz); 0 = none.
///
/// Not simply the mode: at 200 Hz with ~50 updates/s nearly every update advances 4 steps,
/// so the mode is 4 steps. Instead the frequent deltas are clustered (tolerance
/// max(30 µs, 2%)) and the largest s = c_min / m (m = 1..12) of which every frequent
/// cluster is an integer multiple (within 6% of a step) is taken, then refined by
/// sum(center * count) / sum(multiple * count).
StepEstimate estimate_physics_step(const std::vector<int64_t> & deltas_us, double hint_step_us);

/// max(1, round(delta / step)).
int64_t steps_for(int64_t delta_us, double step_us);

// =============================================================================
// Idle input record
// =============================================================================

/// main.py: a frame that received no ControlVehicle reports MORAI's idle record
/// (throttle 0, brake ~0.003, steer 0). A record is idle when
/// throttle <= throttle_max and |brake - brake_center| <= brake_tol.
struct IdleSignature
{
  double throttle_max{1e-3};
  double brake_center{0.003};
  double brake_tol{0.0015};
};

bool looks_idle(const IdleSignature & sig, double throttle, double brake);

struct IdleCalibration
{
  size_t n{0};
  double throttle_median{0.0};
  double brake_median{0.0};
  /// "measured" (median of the no-command window) or "default" (the --idle-* options).
  std::string source{"default"};
  IdleSignature signature;
  /// The idle record's brake is 0: a consumed ACCELERATION command whose throttle/brake
  /// fields are 0 would look identical if MORAI echoes the raw fields. Idle fractions are
  /// then only meaningful if consumed records show throttle > 0.
  bool ambiguous{false};
  std::vector<std::string> warnings;
};

/// `records`: (throttle, brake) of every distinct update seen while NOTHING was sent.
IdleCalibration calibrate_idle(
  const std::vector<std::pair<double, double>> & records, const IdleSignature & fallback);

// =============================================================================
// Per-run analysis
// =============================================================================

struct UpdateSample
{
  int64_t morai_us{0};  ///< ActorStates.timestamp (µs)
  double local_ms{0.0};
  double speed_mps{0.0};  ///< signed longitudinal (velocity.x km/h -> m/s)
  double x{0.0};
  double y{0.0};
  double throttle{0.0};
  double brake{0.0};
};

struct SendSample
{
  double local_ms{0.0};
  /// Newest MORAI timestamp known to precede the send (reader latest, or the WaitForTick
  /// value it followed). The send is attributed to the first update after it — in MORAI
  /// time, so a frame-locked send is never attributed to the update it followed.
  int64_t prev_morai_us{0};
  bool ok{false};
};

/// Updates grouped by a property: counts over the drive window, velocity change over the
/// fit window.
struct GroupStat
{
  size_t n_drive{0};
  size_t idle_drive{0};
  size_t n_fit{0};
  double dv_fit{0.0};  ///< sum of speed changes, m/s
  double dt_fit{0.0};  ///< sum of MORAI time, s

  double idle_share() const;  ///< NaN if n_drive == 0
  double accel() const;       ///< dv_fit / dt_fit, NaN if n_fit == 0
  void merge(const GroupStat & o);
};

/// Least squares of Δv on k over commanded updates (fit window). Per-update consumption:
/// Δv = a·k·dt (all of Δv scales with k). Per-step consumption without hold: Δv = a·dt +
/// idle·(k-1)·dt (Δv does not scale with k). Works at any MORAI Hz as long as k varies —
/// at 200 Hz there are no 1-step updates, so a(k>=2)/a(k=1) alone cannot decide. The share
/// carries a standard error (speed noise at 200 Hz can swing it by +-0.3).
struct KDvStat
{
  size_t n{0};
  double sk{0.0};
  double skk{0.0};
  double sdv{0.0};
  double sdv2{0.0};
  double skdv{0.0};
  std::map<int64_t, size_t> k_counts;

  void add(int64_t k, double dv);
  void merge(const KDvStat & o);
  double slope() const;      ///< m/s per step; NaN without spread in k
  double intercept() const;  ///< m/s
  /// Standard error of slope(): sqrt(SSR / (n-2) / Sxx). NaN when n < 3 or no spread.
  double slope_se() const;
  /// slope·mean(k) / mean(Δv): ~1 per update, ~0 per step. NaN when undefined.
  double k_share() const;
  /// slope_se()·mean(k) / |mean(Δv)|. At 200 Hz mean(Δv) is small under per-step
  /// consumption, so speed noise is amplified here — votes must respect it.
  double k_share_se() const;
  /// >= 2 distinct k values with >= `min_each` updates each.
  bool spread_ok(size_t min_each) const;
};

struct RunInput
{
  /// Distinct updates, ascending MORAI time. Must include the last update at or before t0.
  std::vector<UpdateSample> updates;
  /// Drive-phase sends only.
  std::vector<SendSample> sends;
  int64_t t0_us{0};  ///< newest MORAI timestamp before the first drive send
  double drive_s{2.0};
  double fit_start_s{0.5};
  double commanded_accel{0.8};
  double step_us{0.0};
  IdleSignature idle;
  bool aborted{false};
  /// Attribution lag: a send is credited to the (1 + lag)-th update after prev_morai_us.
  /// 0 = the next update (see estimate_lag()).
  int lag{0};
};

struct RunAnalysis
{
  bool aborted{false};
  int lag{0};
  double commanded_accel{0.0};
  double step_us{0.0};
  size_t updates_total{0};
  /// Updates with MORAI time t in (0, drive_s] after t0 / in [fit_start, drive_s].
  size_t updates_drive{0};
  size_t updates_fit{0};

  // (1) realised acceleration
  LinearFit fit;
  double realised_accel{0.0};
  double ratio{0.0};

  // (2) idle records
  size_t idle_drive{0};
  double idle_fraction{0.0};
  size_t idle_fit{0};
  double idle_fraction_fit{0.0};

  // (3) steps per update
  std::map<int64_t, size_t> steps_hist;
  size_t steps_total{0};
  double steps_per_update{0.0};
  /// updates / steps: the share of physics steps that could carry a command if exactly
  /// one step per update consumed one.
  double update_step_coverage{0.0};

  // (4) sends per update (key 5 = 5 or more)
  std::map<int64_t, size_t> sends_hist;
  size_t sends_in_drive{0};
  size_t sends_failed{0};
  size_t sends_unattributed{0};
  double sends_per_update{0.0};

  // (5) per-step vs per-update evidence. single/multi: updates that received >= 1 send
  // and advanced 1 / >= 2 steps. zero_send/some_send: updates with 0 / >= 1 sends.
  GroupStat single;
  GroupStat multi;
  GroupStat zero_send;
  GroupStat some_send;
  /// multi.accel() / single.accel(): ~1 = the command acts on every step of an update
  /// (per update), ~1/k = only one step per update is driven (per physics step, no hold).
  double multi_step_accel_ratio{0.0};
  /// zero_send.accel() / some_send.accel(): ~0 = no hold, ~1 = MORAI holds the command.
  double zero_send_accel_ratio{0.0};
  /// Δv vs k over commanded updates in the fit window, and its k_share().
  KDvStat kdv;
  double k_share{0.0};

  // stalls
  double median_gap_ms{0.0};
  double max_gap_ms{0.0};
  size_t stalls{0};  ///< MORAI-time gap > 2.5 x median gap

  /// Path length from positions / integral of |speed| dt over the fit window. ~1 confirms
  /// the km/h -> m/s conversion; ~3.6 or ~0.28 means the unit is wrong.
  double distance_ratio{0.0};

  std::vector<std::string> warnings;
};

RunAnalysis analyze_run(const RunInput & in);

// =============================================================================
// Attribution lag and clock identity
// =============================================================================

struct LagCandidate
{
  int lag{0};
  /// Fit window: updates credited with >= 1 send under this lag vs the rest.
  size_t n_credited{0};
  size_t n_other{0};
  double accel_credited{0.0};  ///< sum dv / sum dt, m/s^2
  double accel_other{0.0};
  double accel_contrast{0.0};  ///< credited - other
  double accel_contrast_se{0.0};
  /// Drive window: share of records that are NOT idle.
  size_t n_credited_drive{0};
  size_t n_other_drive{0};
  double nonidle_credited{0.0};
  double nonidle_other{0.0};
  double record_contrast{0.0};
  double record_contrast_se{0.0};
};

struct LagEstimate
{
  size_t runs_used{0};
  std::vector<LagCandidate> candidates;
  /// Lag at which credited updates accelerate most (the one analyze_run should use).
  int physics_lag{0};
  bool physics_decided{false};
  /// Lag at which credited updates show non-idle records most.
  int record_lag{0};
  bool record_decided{false};
  std::vector<std::string> warnings;
};

/// Uses runs where 20..80% of the drive-window updates get no send under lag 0 (locked_half,
/// free33). For each lag 0..max_lag, contrasts credited vs other updates in acceleration
/// (physics lag) and in non-idle record share (record lag). The smallest lag within 2 SE of
/// the best contrast is taken; it is "decided" when that contrast is >= 3 SE (and >= 5% of
/// the command for acceleration, >= 0.2 for the record share). Undecided -> 0.
LagEstimate estimate_lag(const std::vector<RunInput> & runs, int max_lag = 2);

struct WftMatch
{
  size_t n{0};             ///< WaitForTick values inside the reader's timestamp range
  size_t found{0};         ///< ... equal to one of the reader's timestamps
  size_t out_of_range{0};  ///< outside the reader's range (not judged)
  double share{0.0};       ///< found / n, NaN if n == 0
};

/// Locked-pattern attribution assumes WaitForTick returns ActorStates.timestamp values and
/// that the reader saw every update. share ~1 confirms both.
WftMatch match_wft_values(const std::vector<int64_t> & wft_values, std::vector<int64_t> reader_ts);

Json to_json(const LagEstimate & e);
Json to_json(const WftMatch & m);

Json to_json(const LinearFit & f);
Json to_json(const StepEstimate & s);
Json to_json(const IdleCalibration & c);
Json to_json(const GroupStat & g);
Json to_json(const KDvStat & k);
Json to_json(const RunAnalysis & a);

// =============================================================================
// Pattern summary and interpretation
// =============================================================================

struct PatternSummary
{
  std::string pattern;
  size_t runs{0};
  size_t valid{0};  ///< not aborted and >= 5 points in the fit
  size_t aborted{0};
  std::vector<double> ratios;
  double ratio_mean{0.0};
  double ratio_sd{0.0};
  double idle_mean{0.0};
  double sends_per_update_mean{0.0};
  double steps_per_update_mean{0.0};
  /// Mean updates / steps (share of steps one command per update could drive).
  double coverage_mean{0.0};
  size_t stalls{0};
  GroupStat single;
  GroupStat multi;
  GroupStat zero_send;
  GroupStat some_send;
  double multi_step_accel_ratio{0.0};
  double zero_send_accel_ratio{0.0};
  KDvStat kdv;
  double k_share{0.0};
  double k_share_se{0.0};
};

PatternSummary summarize_pattern(
  const std::string & pattern, const std::vector<const RunAnalysis *> & runs);

Json to_json(const PatternSummary & p);

struct Interpretation
{
  /// "per_update", "per_step", "inconclusive".
  std::string verdict{"inconclusive"};
  /// The idle-record vote agreed with the physics vote.
  bool confirmed{false};
  /// "no_hold", "holds", "unknown".
  std::string hold{"unknown"};
  /// Physics vote from Δv ~ k: "per_update", "per_step", "none".
  std::string k_vote{"none"};
  /// Every commanded update advanced (nearly) the same k: Δv ~ k cannot decide.
  bool k_uniform{false};
  /// Idle-record vote: "per_update", "per_step", "neutral", "unavailable".
  std::string idle_vote{"unavailable"};
  std::string idle_vote_note;
  double multi_step_accel_ratio{0.0};
  double idle_share_single{0.0};
  double idle_share_multi{0.0};
  double zero_send_accel_ratio{0.0};
  double k_share{0.0};
  double k_share_se{0.0};
  size_t n_kdv{0};
  size_t n_single_fit{0};
  size_t n_single_drive{0};
  size_t n_multi_fit{0};
  size_t n_multi_drive{0};
  std::vector<std::string> lines;
};

/// Pools every pattern's groups.
///   Physics vote (REQUIRED for any verdict): Δv ~ k share over commanded updates (>= 20,
///   k spread >= 2 values x 5) with its standard error:
///     per update if share >= 0.75 and share - 2 SE > 0.35,
///     per step   if share <= 0.35 and share + 2 SE < 0.75, else no vote.
///   Idle-record vote (can only confirm, or downgrade to inconclusive): needs a positive
///   control — >= 10 commanded k=1 updates whose records are idle <= 10% (so the record
///   does show a consumed command); unavailable without k=1 updates (200 Hz) or when the
///   idle signature is ambiguous. Then idle share of commanded k>=2 updates <= 0.1 ->
///   per update, >= 0.5 -> per step.
///   Physics vote contradicted by the idle vote -> inconclusive.
/// Uniform k -> inconclusive with the cross-Hz instruction (compare locked1 with 60 Hz).
Interpretation interpret(const std::vector<PatternSummary> & patterns, bool idle_ambiguous);

Json to_json(const Interpretation & i);

// =============================================================================
// Safety envelope
// =============================================================================

/// vehicle_state.steer / front_wheel_angle echo the last command (main.py: fwa == steer,
/// normalised [-1, 1]); this tool only ever sends 0.0, so a larger echo is someone else.
constexpr double kSteerEchoTol = 1e-3;

bool steer_echo(double steer, double front_wheel_angle);

struct EnvelopeLimits
{
  double max_speed_mps{3.0};
  double max_distance_m{15.0};
  /// Any state read failing (or Ego missing) for longer than this.
  double max_stale_s{0.5};
  /// MORAI time not advancing for longer than this (stalls were <= ~0.11 s).
  double max_frozen_s{1.0};
  /// max(|steer|, |front_wheel_angle|) of a new record.
  double max_steer_echo{kSteerEchoTol};
};

struct EnvelopeState
{
  bool have_state{false};
  double speed_mps{0.0};   ///< max planar speed since the pattern started
  double distance_m{0.0};  ///< max planar displacement from the pattern's start
  double stale_s{0.0};
  double frozen_s{0.0};
  size_t collisions{0};
  double steer_echo{0.0};  ///< max steer echo of records since the pattern started
};

/// "" inside the envelope, else the reason. NaN speed/distance is a violation.
std::string check_envelope(const EnvelopeLimits & l, const EnvelopeState & s);

/// Rough straight-line travel of one run: 0.5 a T^2 + (a T)^2 / (2 b).
double estimate_run_travel_m(double accel, double drive_s, double brake_decel);

// =============================================================================
// Preflight: is anyone else driving the Ego?
// =============================================================================

/// VehicleControlMode values (actor_enum.proto); morai_actuate static_asserts they match.
constexpr int kModeKeyboard = 1;
constexpr int kModeGameWheel = 2;

/// "" when the Ego may be taken over from `mode` (KEYBOARD / GAME_WHEEL: a human's modes),
/// else the reason. AUTO_* (a bridge or Autoware may be sending), UNSPECIFIED and the rest
/// need `allow_any` (--allow-any-original-mode).
std::string original_mode_refusal(int mode, const std::string & mode_name, bool allow_any);

struct InputRecord
{
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double fwa{0.0};
};

struct ForeignInputCheck
{
  size_t n{0};
  size_t pedal{0};   ///< throttle > 0.01 or brake > 0.05
  size_t steer{0};   ///< steer_echo()
  size_t active{0};  ///< either
  double active_share{0.0};
  bool driving{false};
  std::string why;
};

/// Records seen while this tool sent nothing. A bridge sending LONG_CMD_TYPE_ACCELERATION
/// with zero pedals looks idle on the pedals, so a steer echo counts as well.
ForeignInputCheck check_foreign_input(const std::vector<InputRecord> & records, double share_limit);

}  // namespace morai_grpc_vendor::actuate

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_ANALYSIS_HPP_
