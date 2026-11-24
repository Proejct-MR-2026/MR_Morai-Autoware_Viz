// 신규 — 26.09.24 _ YSH _ morai_probe CLI 옵션 파서 (gRPC 비의존, gtest 로 입력 검증)
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

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_OPTIONS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_OPTIONS_HPP_

#include <cstddef>
#include <cstdio>
#include <string>
#include <vector>

namespace morai_grpc_vendor::probe
{

/// Accepted ranges. Everything outside is rejected by parse_args() so that no value can
/// reach an allocation, a float->int conversion or a chrono duration_cast that overflows.
struct OptionLimits
{
  static constexpr double kMaxSeconds = 3600.0;
  // 26.09.25 _ YSH _ monitor 는 주행 한 판 내내 돌 수 있어 창 상한을 따로 둔다 (0 = Ctrl-C 까지)
  static constexpr double kMaxMonitorSeconds = 86400.0;
  static constexpr double kMinRateHz = 0.1;  ///< rate / poll-hz: 0 (= unpaced) or >= this
  static constexpr double kMaxRateHz = 100000.0;
  static constexpr size_t kMaxCalls = 10000000;
  static constexpr int kMaxDeadlineMs = 60000;
  static constexpr int kMaxConnectTimeoutMs = 600000;
  static constexpr size_t kMaxRawRows = 10000000;
};

struct Options
{
  std::string command{"all"};
  std::string host{"127.0.0.1"};
  int port{7789};
  std::string ego{"Ego"};
  std::string client_key;  // main.py leaves it empty; so do we
  /// frames / simtime / waitfortick window. 60 s: MORAI stalls were seen roughly every
  /// 20 s, so a 10 s window expects fewer than one.
  double seconds{60.0};
  /// rtt window. The call count follows from it (rate_hz * rtt_seconds when paced).
  double rtt_seconds{60.0};
  /// rtt: optional hard cap on calls; 0 = bounded by rtt_seconds only.
  size_t n{0};
  size_t wft_n{200};
  double rate_hz{200.0};
  double poll_hz{0.0};
  int deadline_ms{200};
  int connect_timeout_ms{3000};
  /// Per-section cap on stored raw rows; rows beyond it are counted and warned about.
  size_t max_raw_rows{200000};
  std::string json_path;
  bool raw{true};
  bool help{false};
  // 26.09.25 _ YSH _ monitor 서브커맨드 옵션. 명시하지 않으면 monitor 는 --seconds 0(Ctrl-C 까지)·
  //   --poll-hz 1000 을 쓴다 — 다른 서브커맨드 기본값(60 s, tight loop)을 바꾸지 않으려고 명시 여부를 기록한다
  std::string csv_path;         ///< monitor: 업데이트마다 한 행 CSV (진행 중에도 기록)
  std::string trigger{"poll"};  ///< monitor: poll = GetAllActorsState 폴링, wft = WaitForTick 트리거
  bool seconds_set{false};
  bool poll_hz_set{false};
};

void print_usage(std::FILE * f);

/// MORAI_SIM_ADDRESS / MORAI_SIM_PORT, applied before the command line. A malformed port
/// is an error, not a silent 0.
bool apply_env_defaults(
  const char * env_host, const char * env_port, Options * o, std::string * err);

/// `args` excludes argv[0]. Every numeric value must be consumed completely ("12abc",
/// "-1" for a count, "nan", "inf" and out-of-range values are rejected). `--help` sets
/// o->help and returns true without validating the rest.
// 26.09.25 _ YSH _ monitor: --seconds 0 허용(Ctrl-C 까지, 상한 86400), --csv·--trigger 는 monitor 전용
bool parse_args(const std::vector<std::string> & args, Options * o, std::string * err);

}  // namespace morai_grpc_vendor::probe

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__PROBE_OPTIONS_HPP_
