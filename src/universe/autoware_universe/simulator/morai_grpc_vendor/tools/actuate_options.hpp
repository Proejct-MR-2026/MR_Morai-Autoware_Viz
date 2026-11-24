// 신규 — 26.09.25 _ YSH _ morai_actuate CLI 옵션·패턴 표 (gRPC 비의존, 확인 플래그 없으면 거부)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_OPTIONS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_OPTIONS_HPP_

#include <cstdio>
#include <string>
#include <vector>

namespace morai_grpc_vendor::actuate
{

/// The flag without which nothing runs.
constexpr const char * kConfirmFlag = "--i-understand-this-moves-the-ego";

/// Fixed safety constants (not options on purpose).
struct Safety
{
  static constexpr double kSafeStopAccel = -3.0;  ///< m/s^2, LONG_CMD_TYPE_ACCELERATION
  static constexpr double kSafeStopHz = 100.0;    ///< >= 50 Hz required; 2x the update rate
  static constexpr double kStopSpeed = 0.05;      ///< m/s
  static constexpr double kStopHoldS = 1.0;       ///< below kStopSpeed this long = stopped
  static constexpr double kStopTimeoutS = 10.0;
  static constexpr double kMaxStaleS = 0.5;       ///< state reads failing this long = abort
  static constexpr double kMaxFrozenS = 1.0;      ///< MORAI time not advancing this long
  static constexpr double kFitStartS = 0.5;       ///< fit window [0.5 s, drive_s]
  /// Brake hold right before every drive, so every run starts from the same brake-held
  /// standstill (the first one would otherwise follow the no-command idle probe).
  static constexpr double kPreDriveHoldS = 0.5;
  /// Restore's safe-stop when the previous safe-stop already failed with stale state or no
  /// accepted brake command (MORAI unreachable): short, then straight to gear and mode.
  static constexpr double kRestoreStopAfterFailureS = 2.0;
  /// Travel estimate assumes this realised deceleration (0.29 x 3.0 was measured at 33 Hz).
  static constexpr double kAssumedBrakeDecel = 1.0;
};

/// How a pattern sends the drive command.
struct PatternSpec
{
  const char * name;
  bool locked;     ///< after WaitForTick returns (true) or on a fixed timer (false)
  int every_n;     ///< locked: send after every n-th WaitForTick return
  int burst;       ///< locked: back-to-back sends per trigger
  double hz;       ///< free: timer rate
  bool pipelined;  ///< free: asynchronous sends, at most --max-inflight outstanding
  const char * what;
};

const std::vector<PatternSpec> & known_patterns();
const PatternSpec * find_pattern(const std::string & name);

// 26.09.25 _ YSH _ --lockstep (MORAI 동기 모드) 패턴 표 — 실시간 표(known_patterns)와 섞지 않는다
/// A lockstep pattern: ControlVehicle before every cmd_every-th Tick, each Tick is Tick(frames).
struct LockstepPatternSpec
{
  const char * name;
  int cmd_every;  ///< 1 = a command before every Tick; 2 = every 2nd; 3 = every 3rd
  int frames;     ///< frames per Tick RPC: Tick(frames)
  const char * what;
};

const std::vector<LockstepPatternSpec> & known_lockstep_patterns();
const LockstepPatternSpec * find_lockstep_pattern(const std::string & name);

struct Options
{
  bool confirmed{false};
  std::string host{"127.0.0.1"};
  int port{7789};
  std::string ego{"Ego"};
  std::string client_key;  // main.py leaves it empty
  double accel{0.8};
  double drive_s{2.0};
  double settle_s{1.0};
  int repeats{2};
  std::vector<std::string> patterns{"locked1", "locked_half", "burst4",
                                    "free33",  "free100",     "free200"};
  /// "abba": A..F, then F..A (odd repeats reversed) so a linear drift cancels; "fixed":
  /// A..F every repeat.
  std::string order{"abba"};
  /// Take over from AUTO_*/UNSPECIFIED/other modes too (default: KEYBOARD / GAME_WHEEL only).
  bool allow_any_mode{false};
  double max_speed{3.0};
  double max_distance{15.0};
  double max_total_distance{60.0};
  double physics_hz{0.0};  ///< hint, 0 = estimate only
  double reader_hz{0.0};   ///< 0 = tight loop
  int deadline_ms{100};
  int wft_deadline_ms{250};
  int connect_timeout_ms{3000};
  int max_inflight{8};
  double idle_probe_s{0.5};
  double idle_brake{0.003};
  double idle_brake_tol{0.0015};
  double idle_throttle_max{1e-3};
  bool countdown{true};
  std::string json_path;
  bool help{false};
  // 26.09.25 _ YSH _ --lockstep: MORAI 동기 모드에서 Tick 단위 구동 (actuate_lockstep.cpp). 확인 플래그와 둘 다 필요
  bool lockstep{false};
  int tick_period_ms{10};     ///< SyncMode.tick_period, 1..100 (MORAI's own range)
  int tick_deadline_ms{1000}; ///< deadline of one Tick RPC
};

void print_usage(std::FILE * f);

/// MORAI_SIM_ADDRESS / MORAI_SIM_PORT, applied before the command line.
bool apply_env_defaults(
  const char * env_host, const char * env_port, Options * o, std::string * err);

// 26.09.25 _ YSH _ --lockstep 규칙 설명 추가 (패턴 표·전용 옵션 교차 검증)
// /// `args` excludes argv[0]. Refuses (false) without kConfirmFlag unless --help. Numbers must
// /// be consumed completely and lie in range; --patterns must name known, distinct patterns.
/// `args` excludes argv[0]. Refuses (false) without kConfirmFlag unless --help. Numbers must
/// be consumed completely and lie in range; --patterns must name known, distinct patterns.
/// With --lockstep the patterns come from known_lockstep_patterns() (default: all four),
/// --tick-period-ms / --tick-deadline-ms are accepted, and the real-time-only options
/// (--physics-hz, --reader-hz, --wft-deadline-ms, --max-inflight) are refused; without it
/// the reverse holds.
bool parse_args(const std::vector<std::string> & args, Options * o, std::string * err);

// 26.09.25 _ YSH _ lockstep 기본 패턴 목록 (표 순서)
std::vector<std::string> default_lockstep_patterns();

/// Run order: repeats x patterns, odd repeats reversed for "abba".
std::vector<std::string> run_order(const Options & o);

}  // namespace morai_grpc_vendor::actuate

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__ACTUATE_OPTIONS_HPP_
