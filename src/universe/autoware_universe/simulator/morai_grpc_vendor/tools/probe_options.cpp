// 신규 — 26.09.24 _ YSH _ probe_options.hpp 구현 (엄격한 숫자 파싱·범위 검증)
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

#include "probe_options.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace morai_grpc_vendor::probe
{

namespace
{

bool all_digits(const std::string & s)
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
  });
}

/// Unsigned decimal only: no sign, no whitespace, no exponent, no trailing junk.
bool parse_count(const std::string & s, size_t max, size_t * out)
{
  if (!all_digits(s) || s.size() > 12) {
    return false;
  }
  const auto v = std::stoull(s);  // cannot throw: <= 12 digits
  if (v > max) {
    return false;
  }
  *out = static_cast<size_t>(v);
  return true;
}

bool parse_int(const std::string & s, int min, int max, int * out)
{
  if (!all_digits(s) || s.size() > 9) {
    return false;
  }
  const int v = std::stoi(s);  // cannot throw: <= 9 digits
  if (v < min || v > max) {
    return false;
  }
  *out = v;
  return true;
}

/// Finite decimal, fully consumed.
bool parse_finite(const std::string & s, double * out)
{
  if (s.empty() || std::isspace(static_cast<unsigned char>(s.front()))) {
    return false;
  }
  try {
    size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size() || !std::isfinite(v)) {
      return false;
    }
    *out = v;
    return true;
  } catch (const std::exception &) {
    return false;  // invalid_argument, or out_of_range (e.g. 1e400)
  }
}

bool rate_ok(double hz)
{
  return hz == 0.0 || (hz >= OptionLimits::kMinRateHz && hz <= OptionLimits::kMaxRateHz);
}

}  // namespace

void print_usage(std::FILE * f)
{
  std::fprintf(
    f,
    // 26.09.25 _ YSH _ monitor 서브커맨드·--csv·--trigger 안내 추가. 원래 첫 줄:
    // "usage: morai_probe [options] <rtt|frames|sync|simtime|waitfortick|spec|mode|all>\n"
    "usage: morai_probe [options] <rtt|frames|sync|simtime|waitfortick|spec|mode|all|monitor>\n"
    "\n"
    "Read-only MORAI SIM measurement. Issues only Get*/WaitForTick/GetTimestamp RPCs.\n"
    "\n"
    "  monitor                watch what MORAI applies while something else drives: per\n"
    "                         distinct update, Ego vehicle_state + speed; control mode every\n"
    "                         0.5 s; a summary line every 10 s (Ctrl-C ends it)\n"
    "\n"
    "  --host H               default $MORAI_SIM_ADDRESS, else 127.0.0.1\n"
    "  --port P               default $MORAI_SIM_PORT, else 7789\n"
    "  --ego NAME             ego actor id (default \"Ego\", as main.py)\n"
    "  --client-key K         GetAllActorsFilter.client_key (default empty, as main.py)\n"
    "  --seconds S            window for frames/simtime/waitfortick (default 60, max 3600)\n"
    "                         monitor: 0 = until Ctrl-C (default), max 86400\n"
    "  --rtt-seconds S        rtt: window (default 60, max 3600)\n"
    "  --n N                  rtt: optional cap on GetActorState calls (default: none)\n"
    "  --rate HZ              rtt: pacing, 0 = back-to-back (default 200, = main.py poll)\n"
    "  --poll-hz HZ           frames/simtime: pacing, 0 = tight loop (default 0)\n"
    "                         monitor: default 1000 (every update is ~20 ms apart)\n"
    "  --trigger poll|wft     monitor: poll GetAllActorsState, or read it after each\n"
    "                         WaitForTick (default poll; keep poll while the engine runs)\n"
    "  --csv PATH             monitor: one row per distinct update, written as it runs\n"
    "  --wft-n N              waitfortick: max calls (default 200)\n"
    "  --deadline-ms MS       per-RPC deadline (default 200)\n"
    "  --connect-timeout-ms MS  channel connect deadline (default 3000)\n"
    "  --max-raw-rows N       per-section cap on stored raw rows (default 200000)\n"
    "  --json PATH            write all results (incl. raw samples) as JSON\n"
    "  --no-raw               omit raw per-sample arrays from the JSON\n"
    "  -h, --help\n"
    "\n"
    "HZ: 0 or 0.1..100000. Run it from a shell whose LAST sourced setup.bash is this\n"
    "workspace's install/setup.bash (see README 'morai_probe').\n"
    "\n"
    // 26.09.25 _ YSH _ monitor 의 --csv 실패도 3. 원래 줄:
    // "Exit: 0 ok, 1 usage, 2 cannot connect, 3 cannot write --json.\n");
    "Exit: 0 ok, 1 usage, 2 cannot connect, 3 cannot write --json / --csv (checked before\n"
    "connecting; missing parent directories are created).\n");
}

bool apply_env_defaults(
  const char * env_host, const char * env_port, Options * o, std::string * err)
{
  if (env_host && *env_host) {
    o->host = env_host;
  }
  if (env_port && *env_port) {
    if (!parse_int(env_port, 1, 65535, &o->port)) {
      *err = std::string("MORAI_SIM_PORT is not a port number: '") + env_port + "'";
      return false;
    }
  }
  return true;
}

bool parse_args(const std::vector<std::string> & args, Options * o, std::string * err)
{
  bool have_command = false;
  for (size_t i = 0; i < args.size(); ++i) {
    std::string a = args[i];
    std::string val;
    bool has_inline = false;
    if (a.rfind("--", 0) == 0) {
      const auto eq = a.find('=');
      if (eq != std::string::npos) {
        val = a.substr(eq + 1);
        a = a.substr(0, eq);
        has_inline = true;
      }
    }
    auto need = [&]() -> bool {
      if (has_inline) {
        return true;
      }
      if (i + 1 >= args.size()) {
        *err = "missing value for " + a;
        return false;
      }
      val = args[++i];
      return true;
    };
    auto bad = [&](const char * why) {
      *err = "bad value for " + a + ": '" + val + "' (" + why + ")";
      return false;
    };

    if (a == "-h" || a == "--help") {
      o->help = true;
      return true;
    } else if (a == "--host") {
      if (!need()) return false;
      o->host = val;
    } else if (a == "--port") {
      if (!need()) return false;
      if (!parse_int(val, 1, 65535, &o->port)) return bad("1..65535");
    } else if (a == "--ego") {
      if (!need()) return false;
      o->ego = val;
    } else if (a == "--client-key") {
      if (!need()) return false;
      o->client_key = val;
    } else if (a == "--seconds" || a == "--rtt-seconds") {
      if (!need()) return false;
      double v = 0.0;
      // 26.09.25 _ YSH _ --seconds 는 monitor 용으로 0 과 86400 까지 받고, 서브커맨드별 범위는 아래
      //   명령 확인 뒤에 다시 본다 (명령이 옵션 뒤에 올 수 있다). --rtt-seconds 는 그대로
      // if (!parse_finite(val, &v) || !(v > 0.0) || v > OptionLimits::kMaxSeconds) {
      //   return bad("finite, > 0, <= 3600");
      // }
      // (a == "--seconds" ? o->seconds : o->rtt_seconds) = v;
      if (a == "--seconds") {
        if (!parse_finite(val, &v) || !(v >= 0.0) || v > OptionLimits::kMaxMonitorSeconds) {
          return bad("finite, >= 0, <= 86400 (0 and > 3600 only for monitor)");
        }
        o->seconds = v;
        o->seconds_set = true;
      } else {
        if (!parse_finite(val, &v) || !(v > 0.0) || v > OptionLimits::kMaxSeconds) {
          return bad("finite, > 0, <= 3600");
        }
        o->rtt_seconds = v;
      }
    } else if (a == "--n" || a == "--wft-n") {
      if (!need()) return false;
      size_t v = 0;
      if (!parse_count(val, OptionLimits::kMaxCalls, &v) || v < 1) {
        return bad("integer 1..10000000");
      }
      (a == "--n" ? o->n : o->wft_n) = v;
    } else if (a == "--rate" || a == "--poll-hz") {
      if (!need()) return false;
      double v = 0.0;
      if (!parse_finite(val, &v) || !rate_ok(v)) {
        return bad("0, or 0.1..100000");
      }
      (a == "--rate" ? o->rate_hz : o->poll_hz) = v;
      // 26.09.25 _ YSH _ monitor 기본 폴링(1000 Hz)을 명시값이 이기게 기록
      if (a == "--poll-hz") {
        o->poll_hz_set = true;
      }
    } else if (a == "--deadline-ms") {
      if (!need()) return false;
      if (!parse_int(val, 1, OptionLimits::kMaxDeadlineMs, &o->deadline_ms)) {
        return bad("1..60000");
      }
    } else if (a == "--connect-timeout-ms") {
      if (!need()) return false;
      if (!parse_int(val, 1, OptionLimits::kMaxConnectTimeoutMs, &o->connect_timeout_ms)) {
        return bad("1..600000");
      }
    } else if (a == "--max-raw-rows") {
      if (!need()) return false;
      size_t v = 0;
      if (!parse_count(val, OptionLimits::kMaxRawRows, &v) || v < 1) {
        return bad("integer 1..10000000");
      }
      o->max_raw_rows = v;
    } else if (a == "--json") {
      if (!need()) return false;
      o->json_path = val;
    } else if (a == "--no-raw") {
      o->raw = false;
    } else if (a == "--csv") {  // 26.09.25 _ YSH _ monitor 전용 옵션 (--csv, --trigger)
      if (!need()) return false;
      if (val.empty()) return bad("path");
      o->csv_path = val;
    } else if (a == "--trigger") {
      if (!need()) return false;
      if (val != "poll" && val != "wft") return bad("poll or wft");
      o->trigger = val;
    } else if (!a.empty() && a[0] == '-') {
      *err = "unknown option " + a;
      return false;
    } else if (!have_command) {
      o->command = a;
      have_command = true;
    } else {
      *err = "unexpected argument " + a;
      return false;
    }
  }

  // 26.09.25 _ YSH _ monitor 추가
  // static const char * kCommands[] = {"rtt",         "frames", "sync", "simtime",
  //                                    "waitfortick", "spec",   "mode", "all"};
  static const char * kCommands[] = {"rtt",  "frames", "sync", "simtime", "waitfortick",
                                     "spec", "mode",   "all",  "monitor"};
  if (std::none_of(std::begin(kCommands), std::end(kCommands), [&](const char * c) {
        return o->command == c;
      })) {
    *err = "unknown subcommand '" + o->command + "'";
    return false;
  }
  // 26.09.25 _ YSH _ 서브커맨드별 확인 — --seconds 0·3600 초과는 monitor 만, --csv·--trigger 도 monitor 만.
  //   monitor 가 --seconds·--poll-hz 를 명시하지 않았으면 Ctrl-C 까지·1000 Hz 로 채운다
  if (o->command == "monitor") {
    if (!o->seconds_set) {
      o->seconds = 0.0;
    }
    if (!o->poll_hz_set) {
      o->poll_hz = 1000.0;
    }
  } else {
    if (!(o->seconds > 0.0) || o->seconds > OptionLimits::kMaxSeconds) {
      *err = "bad value for --seconds: must be > 0 and <= 3600 for '" + o->command +
             "' (0 = until Ctrl-C only for monitor)";
      return false;
    }
    if (!o->csv_path.empty()) {
      *err = "--csv is only for monitor";
      return false;
    }
    if (o->trigger != "poll") {
      *err = "--trigger is only for monitor";
      return false;
    }
  }
  return true;
}

}  // namespace morai_grpc_vendor::probe
