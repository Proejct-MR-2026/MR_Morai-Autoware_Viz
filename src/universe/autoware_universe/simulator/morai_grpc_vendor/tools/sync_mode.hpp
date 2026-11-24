// 신규 — 26.09.25 _ YSH _ MORAI 동기 모드 진입·복구 판정 순수 헬퍼 + morai_sync_restore 옵션 (gRPC 비의존, lockstep·restore 공용)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__SYNC_MODE_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__SYNC_MODE_HPP_

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace morai_grpc_vendor::sync_mode
{

// 실측·서버 IL 분석으로 확인된 MORAI 동기 모드 사실 (build R1.260701.H3, README "lockstep mode")
//   - SetSynchronousMode 의 Result 는 적용돼도 status 0 'Unknown error' → Result 를 믿지 않고
//     GetSynchronousMode·timestamp 동작으로 확인한다
//   - SYNCHRONOUS 로 바꾼 직후 ~1 s 동안 gRPC 서버가 UNAVAILABLE → 백오프 재시도
//   - 동기 모드의 ActorStates.timestamp 는 진입 후 sim 시간(µs, epoch 아님), 실시간 모드는 epoch µs

/// SyncModeType (simulation_enum.proto). The gRPC tools static_assert these against protobuf.
constexpr int kSyncUnspecified = 0;
constexpr int kSyncTimeManagement = 1;
constexpr int kSyncSynchronous = 2;

/// VehicleControlMode values the sync-mode logic needs (actor_enum.proto).
constexpr int kModeKeyboard = 1;
constexpr int kModeAuto = 3;
constexpr int kModeSynchronous = 7;

std::string sync_type_name(int type);

/// The tick period MORAI actually uses for a requested SyncMode.tick_period (server IL):
/// 0 -> 10 ms, outside 1..100 -> 20 ms, else as requested.
int effective_tick_period_ms(int requested_ms);

/// In SYNCHRONOUS mode MORAI applies ControlVehicle only when the Ego's control mode is
/// AUTO (3) or SYNCHRONOUS_MODE (7); otherwise it drops it silently (server log only).
bool mode_accepts_control_in_sync(int mode);

/// Sleeps between retries while the gRPC server is UNAVAILABLE after a mode switch:
/// first_ms, doubling, capped at max_ms, as long as the running sum stays <= total_ms.
/// Empty when total_ms < first_ms. Non-positive arguments give an empty schedule.
std::vector<int> backoff_delays_ms(int total_ms, int first_ms, int max_ms);

/// Plausible Unix-epoch microseconds (2017-01-01 .. 2100-01-01). Real-time MORAI reports
/// its host wall clock in epoch µs; SYNCHRONOUS mode reports sim time since entering it.
bool is_epoch_us(int64_t ts_us);

struct TimeSample
{
  double wall_s{0.0};  ///< local steady clock, s
  int64_t ts_us{0};    ///< ActorStates.timestamp
};

struct FrozenCheck
{
  size_t n{0};
  double span_s{0.0};
  size_t changes{0};
  int64_t first_us{0};
  int64_t last_us{0};
  bool frozen{false};
  std::string why;  ///< "" when frozen
};

/// Frozen = at least 3 samples spanning >= min_span_s wall and no timestamp change.
FrozenCheck judge_frozen(const std::vector<TimeSample> & samples, double min_span_s);

struct RealtimeCheck
{
  size_t n{0};
  size_t changes{0};
  double span_s{0.0};
  double rate{0.0};      ///< (last ts - first ts) / wall span; NaN if undefined
  bool epoch{false};     ///< the last timestamp is epoch µs
  double offset_s{0.0};  ///< last ts - local Unix time (informational; two PCs may differ)
  bool ok{false};
  std::string why;  ///< "" when ok
};

/// Real time = >= 3 timestamp changes over >= min_span_s wall, rate in [rate_lo, rate_hi],
/// epoch-based timestamps. `local_unix_s_at_last` is the local Unix time of the last sample.
RealtimeCheck judge_realtime(
  const std::vector<TimeSample> & samples, double local_unix_s_at_last, double min_span_s,
  double rate_lo = 0.5, double rate_hi = 1.5);

// =============================================================================
// morai_sync_restore
// =============================================================================

/// Exit codes of morai_sync_restore (README "morai_sync_restore").
enum RestoreExit : int {
  kRestoreOk = 0,            ///< real time (UNSPECIFIED) and time advances at ~wall rate
  kRestoreUsage = 1,         ///< bad command line
  kRestoreUnreachable = 2,   ///< GetSynchronousMode never answered within --timeout-s
  kRestoreStillSync = 3,     ///< still not UNSPECIFIED after --timeout-s of retries
  kRestoreNotAdvancing = 4,  ///< UNSPECIFIED, but time does not advance at ~wall rate
  kRestoreModeFailed = 5,    ///< --keyboard: the Ego's mode did not read back KEYBOARD
};

struct RestoreOutcome
{
  bool reachable{false};     ///< GetSynchronousMode answered at least once
  bool unspecified{false};   ///< it reads UNSPECIFIED at the end
  bool realtime_ok{false};   ///< judge_realtime() ok
  bool keyboard_requested{false};
  bool keyboard_ok{false};   ///< the mode read back KEYBOARD
};

/// First failing check in the order above; kRestoreOk when all pass.
int restore_exit_code(const RestoreOutcome & o);

struct RestoreOptions
{
  std::string host{"127.0.0.1"};
  int port{7789};
  std::string ego{"Ego"};
  std::string client_key;
  bool keyboard{false};        ///< also set the Ego's control mode to KEYBOARD
  double timeout_s{10.0};      ///< GetSynchronousMode / SetSynchronousMode retry budget
  double verify_s{1.0};        ///< real-time verification window (wall)
  int deadline_ms{500};        ///< per-RPC deadline
  int connect_timeout_ms{3000};
  bool help{false};
};

void print_restore_usage(std::FILE * f);

/// MORAI_SIM_ADDRESS / MORAI_SIM_PORT, applied before the command line.
bool apply_restore_env_defaults(
  const char * env_host, const char * env_port, RestoreOptions * o, std::string * err);

/// `args` excludes argv[0]. No arming flag: this tool only restores.
bool parse_restore_args(
  const std::vector<std::string> & args, RestoreOptions * o, std::string * err);

}  // namespace morai_grpc_vendor::sync_mode

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__SYNC_MODE_HPP_
