// 신규 — 26.09.25 _ YSH _ morai_actuate --lockstep 순수 계산부 (Tick 검증·유지/유휴 판정·Tick(n) 판정·처리량, gRPC 비의존)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__LOCKSTEP_ANALYSIS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__LOCKSTEP_ANALYSIS_HPP_

#include "actuate_analysis.hpp"
#include "probe_json.hpp"
#include "probe_stats.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace morai_grpc_vendor::actuate::lockstep
{

// 동기 모드에서는 Tick 을 불러야만 시간이 흐른다. 그래서 "정지(frozen)" 는 벽시계가 아니라
// Tick 단위로 판정하고, 명령 귀속은 "Tick 직전에 보낸 명령 → 그 Tick" 으로 확정된다
// (큐에 쌓인 ControlVehicle 은 다음 Tick 시작 시 한 번 적용, 여러 개면 마지막 것).

/// Fixed lockstep safety constants (not options on purpose).
struct Limits
{
  /// Safe-stop = brake command + Tick(1), until |v| < 0.05 m/s for 1 s of SIM time: at least
  /// this many Ticks (safe_stop_max_ticks() raises it for short tick periods).
  static constexpr int kSafeStopMaxTicks = 2000;
  /// ... and at least this much sim time (1 ms Ticks: 2000 Ticks = 2 s sim, too short to stop
  /// from --max-speed 10 m/s at ~2.7 m/s^2 realised and then hold 1 s).
  static constexpr double kSafeStopMaxSimS = 20.0;
  /// Wall-time cap of one safe-stop by Ticks, so a MORAI whose Ticks crawl (each up to
  /// --tick-deadline-ms) cannot keep the restore from leaving synchronous mode. A real stop
  /// from 10 m/s at 1 ms Ticks needs ~4.7 s sim = ~4700 Ticks = ~78 s at ~16.5 ms per Tick.
  static constexpr double kSafeStopMaxWallS = 120.0;
  /// Restore's safe-stop by Ticks when the session's last safe-stop already failed: short,
  /// then leave synchronous mode and brake in real time.
  static constexpr double kRestoreStopAfterFailureWallS = 10.0;
  /// Consecutive failing Tick RPCs -> abort (MORAI gone).
  static constexpr size_t kMaxFailedTicks = 3;
  /// Consecutive Ticks that did not advance the timestamp -> abort (MORAI busy / paused / no
  /// permission: Tick returns at once, nothing advances).
  static constexpr size_t kMaxFrozenTicks = 3;
  /// Intermittent trouble: this many failed or not-advancing Ticks among the last
  /// kHealthWindow -> abort (fail, ok, fail, ok, ... never trips the consecutive counters).
  static constexpr size_t kHealthWindow = 5;
  static constexpr size_t kMaxBadInWindow = 3;
  /// A Tick(n) must advance ActorStates.timestamp by n x tick_period within this.
  static constexpr int64_t kDtToleranceUs = 1;
  /// A timestamp step this far from the expected one: MORAI left synchronous mode.
  static constexpr int64_t kJumpUs = 1000000;
  /// GetSynchronousMode retry budget after entering SYNCHRONOUS (server UNAVAILABLE ~1 s).
  static constexpr double kEnterRetryS = 5.0;
  /// SetSynchronousMode(UNSPECIFIED) retry budget on restore.
  static constexpr double kExitRetryS = 10.0;
  /// The world must stay frozen this long (wall) right after entering, before any Tick.
  static constexpr double kFrozenCheckS = 0.3;
  /// After restore, time must advance at ~wall rate over this window.
  static constexpr double kRealtimeCheckS = 1.0;
  /// Gear commands are applied at the start of the next Tick: read back within this many.
  static constexpr int kGearReadbackTicks = 5;
  /// Consecutive failing ControlVehicle calls that abort a run.
  static constexpr size_t kMaxFailedCommands = 10;
};

// =============================================================================
// One Tick
// =============================================================================

/// One Tick RPC: the command sent before it (if any), the Tick result and the state read
/// right after it (a read after Tick sees the new frame).
struct Step
{
  int run{-1};
  int phase{0};
  bool commanded{false};   ///< a ControlVehicle was sent after the previous Tick
  bool command_ok{false};  ///< ... and it returned Result.status SUCCESS
  double accel{0.0};       ///< the command (m/s^2)
  int frames{1};           ///< Tick(frames)
  bool tick_ok{false};
  int grpc_code{-1};
  uint64_t frame_count{0};   ///< SyncTimestamp.frame_count
  int64_t elapsed_time{0};   ///< SyncTimestamp.elapsed_time (NOT sim time)
  double tick_wall_ms{0.0};  ///< Tick RPC wall time
  bool read_ok{false};
  int64_t ts_us{0};         ///< ActorStates.timestamp after the Tick
  uint64_t tick_count{0};   ///< ActorStates.sync_mode_tick_count
  double speed_mps{0.0};    ///< velocity.x km/h -> m/s (main.py)
  double planar_mps{0.0};   ///< hypot(vx, vy) km/h -> m/s
  double x{0.0};
  double y{0.0};
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double fwa{0.0};
  int gear{-1};
  int collisions{0};
  double wall_ms{0.0};  ///< local steady time when the read returned
  int read_retries{0};  ///< re-reads because the snapshot still showed the old frame
};

enum class TickClass {
  kOk,             ///< frame_count +frames, timestamp +frames x period (+-1 µs)
  kFailed,         ///< the Tick RPC failed
  kNoRead,         ///< the state read after it failed
  kNotAdvanced,    ///< the timestamp did not move (busy / paused / no permission)
  kFrameMismatch,  ///< frame_count moved by something else than `frames`
  kDtMismatch,     ///< timestamp moved by something else than frames x period
  kJumped,         ///< timestamp moved by > 1 s off: MORAI left synchronous mode?
};

const char * tick_class_name(TickClass c);

/// Judges `cur` against the immediately preceding Tick `prev` (both need tick_ok and
/// read_ok to be judged beyond kFailed / kNoRead). Sim time is the ground truth: an unchanged
/// timestamp is kNotAdvanced whatever frame_count says (a busy / no-permission Tick may
/// return a default SyncTimestamp with frame_count 0).
TickClass classify_tick(const Step & prev, const Step & cur, int period_ms);

/// `cur` follows one or more failed Ticks / reads: judged against the last good Tick.
/// `frames_since` = frames requested by every Tick after `last_good` up to and including
/// `cur` (failed Ticks may or may not have advanced). Unchanged timestamp -> kNotAdvanced;
/// a step outside [0, frames_since x period] by more than 1 s -> kJumped; else kOk (the exact
/// dt cannot be checked across a gap).
TickClass classify_after_gap(const Step & last_good, const Step & cur, int frames_since, int period_ms);

/// Safe-stop Tick budget for a tick period: max(kSafeStopMaxTicks, kSafeStopMaxSimS / period).
int safe_stop_max_ticks(int period_ms);

struct DtReport
{
  size_t n{0};          ///< consecutive Tick pairs judged
  size_t ok{0};
  size_t violations{0}; ///< judged pairs that are not kOk
  std::map<std::string, size_t> counts;
  std::vector<std::string> examples;  ///< first 5 violations
};

/// (a) Every consecutive pair of Ticks in `steps` (the session log, in order).
DtReport check_dt(const std::vector<Step> & steps, int period_ms);

/// Tick health for the envelope: MORAI time only advances on Tick, so "frozen" is counted
/// in Ticks, not in wall time. Consecutive counters plus a sliding window, so an
/// intermittent fail / ok / fail / ok pattern also aborts.
struct TickHealth
{
  size_t consecutive_failed{0};
  size_t consecutive_not_advanced{0};
  bool jumped{false};
  unsigned recent{0};  ///< bit i = the i-th last Tick failed or did not advance
  size_t seen{0};
  void update(TickClass c);
  /// Failed or not-advancing Ticks among the last kHealthWindow.
  size_t window_bad() const;
};

/// "" = keep going, else the abort reason.
std::string check_tick_health(const TickHealth & h);

// =============================================================================
// Per run
// =============================================================================

/// Ticks pooled by a property: record counts over the drive window, (dv, dt) over the fit
/// window.
struct StepGroup
{
  size_t n_drive{0};
  size_t idle_drive{0};
  std::vector<std::pair<double, double>> fit;  ///< (dv m/s, dt s)

  size_t n_fit() const { return fit.size(); }
  double accel() const;      ///< sum dv / sum dt; NaN when empty
  double accel_se() const;   ///< from the per-Tick residuals; NaN when n < 3
  double idle_share() const; ///< NaN when n_drive == 0
  void merge(const StepGroup & o);
};

struct RunInput
{
  std::string pattern;
  int cmd_every{1};
  int frames{1};
  /// steps[0]: the last Tick before the drive (its state is t0); steps[1..]: drive Ticks.
  std::vector<Step> steps;
  double drive_s{2.0};
  double fit_start_s{0.5};
  double commanded_accel{0.8};
  int period_ms{10};
  IdleSignature idle;
  bool aborted{false};
};

struct RunAnalysis
{
  std::string pattern;
  bool aborted{false};
  size_t ticks{0};
  size_t frames{0};
  size_t commanded{0};
  size_t command_failed{0};
  size_t skipped{0};  ///< Ticks without a usable Tick+read pair
  double sim_s{0.0};
  double wall_s{0.0};
  double rtf{0.0};  ///< sim_s / wall_s (lockstep speed vs real time)
  LinearFit fit;    ///< speed vs sim time, Ticks that end at t >= fit_start
  double slope_se{0.0};  ///< LSQ standard error of fit.slope (m/s^2), NaN if n < 3
  double realised_accel{0.0};
  double ratio{0.0};
  double ratio_se{0.0};  ///< slope_se / commanded
  StepGroup cmd1;   ///< commanded Tick(1)
  StepGroup unc1;   ///< uncommanded Tick(1)
  StepGroup multi;  ///< commanded Tick(n > 1)
  StepGroup all;    ///< every Tick (the pattern's cycle average)
  std::vector<double> tick1_wall_ms;  ///< Tick(1) RPC wall times
  std::vector<double> tickn_wall_ms;  ///< Tick(n > 1) RPC wall times
  DtReport dt;
  std::vector<std::string> warnings;
};

RunAnalysis analyze_run(const RunInput & in);

struct PatternSummary
{
  std::string pattern;
  size_t runs{0};
  size_t valid{0};  ///< not aborted and >= 5 points in the speed fit
  size_t aborted{0};
  std::vector<double> ratios;
  double ratio_mean{0.0};
  double ratio_sd{0.0};
  /// SE of ratio_mean: the larger of the run-to-run spread (sd / sqrt(valid), needs 2 runs)
  /// and the within-run LSQ slope SE (rms / sqrt(valid)). NaN when neither is defined.
  double ratio_se{0.0};
  StepGroup cmd1;
  StepGroup unc1;
  StepGroup multi;
  StepGroup all;
  double sim_s{0.0};
  double wall_s{0.0};
  double rtf{0.0};
  probe::Summary tick_ms;
  size_t dt_n{0};
  size_t dt_violations{0};
};

/// Pools the valid (not aborted) runs.
PatternSummary summarize_pattern(
  const std::string & pattern, const std::vector<const RunAnalysis *> & runs);

// =============================================================================
// Verdicts
// =============================================================================

/// Pattern-level ratio q = ratio(p) / ratio(ref) and its SE (runs are independent, so the
/// delta method holds). NaN when either pattern has no valid run.
struct RatioVs
{
  double q{0.0};
  double se{0.0};
};

RatioVs ratio_vs(const PatternSummary & p, const PatternSummary & ref);

/// sync1 is a usable reference: >= 1 valid run and ratio_mean clearly > 0 (> 2 SE, > 0.05).
bool reference_usable(const PatternSummary * sync1);

/// (c) Does MORAI keep the last input on a Tick without a new command?
///
/// PRIMARY (lag-proof): pattern-level q = ratio(pattern) / ratio(sync1). The pattern commands
/// one Tick in `cmd_every`; if MORAI holds the input every Tick is driven (q ~ 1), if it drops
/// it only the commanded Ticks are (q ~ 1/cmd_every, a bit less with rolling resistance).
///   held if q >= h0 + 0.75 (1 - h0) and q - 2 SE > h0 + 0.35 (1 - h0); idle if
///   q <= h0 + 0.35 (1 - h0) and q + 2 SE < h0 + 0.75 (1 - h0), h0 = 1 / cmd_every.
/// An actuator / powertrain lag of a few Ticks smooths the realised acceleration over
/// neighbouring Ticks, but not its average over the drive: q cannot be fooled by it.
///
/// SECONDARY (per Tick): share = a_unc / a_cmd, uncommanded vs commanded Tick(1) of the same
/// pattern. Lag pulls the share towards 1 (a dropped input still looks held), never towards
/// 0. Used for: "late" (commanded Ticks do not accelerate but the uncommanded ones do: the
/// command acts one Tick late), "conflict" (q says held but the share says idle, which lag
/// cannot explain), and as a fallback without sync1 (then only "idle" is trusted). Its
/// delta-method SE treats a_cmd and a_unc as independent although neighbouring dv share a
/// speed sample (true SD 1.2-1.5x larger, Monte Carlo), so its band needs 3 SE clear.
///   held if share >= 0.75 and share - 3 SE > 0.35; idle if share <= 0.35 and share + 3 SE <
///   0.75. Needs >= 10 Ticks in each group and a_cmd clearly > 0 (> 2 SE and > 5% of the
///   command).
struct HoldVerdict
{
  std::string pattern;
  /// final: "held", "idle", "late", "conflict", "inconclusive", "n/a"
  std::string verdict{"n/a"};
  // pattern level
  std::string ratio_verdict{"n/a"};  ///< "held", "idle", "inconclusive", "n/a"
  double q{0.0};
  double q_se{0.0};
  double q_idle{0.0};  ///< h0 = 1 / cmd_every
  // per Tick
  std::string share_verdict{"n/a"};  ///< "held", "idle", "late", "inconclusive", "n/a"
  double share{0.0};
  double share_se{0.0};
  double a_cmd{0.0};
  double a_cmd_se{0.0};
  double a_unc{0.0};
  double a_unc_se{0.0};
  size_t n_cmd{0};
  size_t n_unc{0};
  std::string note;
};

/// The per-Tick share alone (verdict == share_verdict).
HoldVerdict judge_hold(
  const std::string & pattern, const StepGroup & cmd, const StepGroup & unc,
  double commanded_accel);

/// The final (c) verdict for one pattern: the pattern-level q against `sync1` (nullptr or not
/// usable: per-Tick fallback), combined with the per-Tick share as described above.
HoldVerdict judge_hold_pattern(
  const PatternSummary & p, int cmd_every, const PatternSummary * sync1, double commanded_accel);

/// "conflict" if any pattern is in conflict; else "held" / "idle" / "late" when every decided
/// verdict agrees, "inconsistent" when they disagree, "unknown" when none is decided.
std::string combine_hold(const std::vector<HoldVerdict> & v);

/// (d) Does a Tick(n) keep driving with the input for all n frames? m = a(Tick(n)) /
/// a(commanded Tick(1)). Persisting input: m ~ 1; only the first frame driven: m ~ 1/n.
/// The band maps the hold band onto [1/n, 1]: persists if m >= 1/n + 0.75 (1 - 1/n) and
/// m - 2 SE > 1/n + 0.35 (1 - 1/n); first_frame_only if m <= 1/n + 0.35 (1 - 1/n) and
/// m + 2 SE < 1/n + 0.75 (1 - 1/n). `chain` (sync_tick3x: one command, Tick(1) x n) is
/// compared the same way.
///   judge_multi_frame_pattern: m = ratio(sync_tick3) / ratio(sync1) (LSQ slopes, SE from
///   the run spread and the fit; lag-proof). judge_multi_frame: the per-Tick fallback when
///   sync1 was not measured.
struct MultiFrameVerdict
{
  std::string verdict{"n/a"};  ///< "persists", "first_frame_only", "inconclusive", "n/a"
  int frames{3};
  std::string method{"per-Tick"};  ///< "pattern ratios" or "per-Tick"
  std::string a1_source;
  double a1{0.0};
  double a1_se{0.0};
  double an{0.0};
  double an_se{0.0};
  double m{0.0};
  double m_se{0.0};
  double m_chain{0.0};
  double m_chain_se{0.0};
  size_t n1{0};
  size_t nn{0};
  std::string note;
};

MultiFrameVerdict judge_multi_frame(
  const StepGroup & single_cmd, const std::string & a1_source, const StepGroup & multi,
  const StepGroup * chain, int frames);

MultiFrameVerdict judge_multi_frame_pattern(
  const PatternSummary & tickn, const PatternSummary & sync1, const PatternSummary * chain,
  int frames, double commanded_accel);

/// (f) What vehicle_state shows on uncommanded Ticks. Positive control: commanded Tick(1)
/// records must look idle <= 10% (the record shows a consumed command at all).
struct IdleRecordCheck
{
  size_t n_cmd{0};
  size_t idle_cmd{0};
  size_t n_unc{0};
  size_t idle_unc{0};
  double share_cmd{0.0};
  double share_unc{0.0};
  std::string vote{"unavailable"};  ///< "idle_record", "held_record", "neutral", "unavailable"
  std::string note;
};

IdleRecordCheck check_idle_records(
  const StepGroup & cmd, const StepGroup & unc, bool idle_ambiguous);

/// (f) for (d): the record read after a commanded Tick(n) shows frame n's input. Idle there
/// = only the first frame was driven; not idle = the input persisted. Same positive control
/// (commanded Tick(1) records idle <= 10%).
struct MultiRecordCheck
{
  size_t n_multi{0};
  size_t idle_multi{0};
  double share_multi{0.0};
  std::string vote{"unavailable"};  ///< "first_frame_record", "persist_record", "neutral", "unavailable"
  std::string note;
};

MultiRecordCheck check_multi_records(
  const StepGroup & cmd, const StepGroup & multi, bool idle_ambiguous);

/// The synchronous-mode idle signature. The probe runs right after SetVehicleGear(D), before
/// this session's first ControlVehicle, so there is no input MORAI could be holding: any
/// no-command brake level up to 1.0 (MORAI reports brake 1.0 in real-time AUTO without a
/// command) is taken as the idle record. calibrate_idle() (real time) only accepts 0..0.05.
IdleCalibration calibrate_sync_idle(
  const std::vector<std::pair<double, double>> & records, const IdleSignature & fallback);

/// (e) Lockstep throughput over the drive windows.
struct Throughput
{
  double sim_s{0.0};
  double wall_s{0.0};
  double rtf{0.0};  ///< sim_s / wall_s: 1 = real time, < 1 slower
  probe::Summary tick1_ms;  ///< Tick(1) RPC wall
  probe::Summary tickn_ms;  ///< Tick(n > 1) RPC wall
};

Throughput throughput(const std::vector<const RunAnalysis *> & runs);

struct Interpretation
{
  double sync1_ratio{0.0};
  std::vector<HoldVerdict> hold;
  std::string hold_overall{"unknown"};
  MultiFrameVerdict multi;
  IdleRecordCheck idle_records;
  MultiRecordCheck multi_records;
  Throughput speed;
  DtReport dt;
  /// The one-line summary.
  std::string line;
  std::vector<std::string> lines;
};

/// Pools the pattern summaries (by name: sync1, sync_drop2, sync_tick3, sync_tick3x).
/// `session_dt` is check_dt() over every Tick of the session.
Interpretation interpret(
  const std::vector<PatternSummary> & patterns, double commanded_accel, bool idle_ambiguous,
  const DtReport & session_dt, const std::vector<const RunAnalysis *> & runs);

Json to_json(const DtReport & d);
Json to_json(const StepGroup & g);
Json to_json(const RunAnalysis & a);
Json to_json(const PatternSummary & p);
Json to_json(const HoldVerdict & h);
Json to_json(const MultiFrameVerdict & m);
Json to_json(const IdleRecordCheck & c);
Json to_json(const MultiRecordCheck & c);
Json to_json(const Throughput & t);
Json to_json(const Interpretation & i);

}  // namespace morai_grpc_vendor::actuate::lockstep

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__LOCKSTEP_ANALYSIS_HPP_
