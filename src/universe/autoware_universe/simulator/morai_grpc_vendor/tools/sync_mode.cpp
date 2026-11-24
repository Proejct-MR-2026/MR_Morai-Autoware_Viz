// 신규 — 26.09.25 _ YSH _ sync_mode.hpp 구현 (gRPC·protobuf 비의존)
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

#include "sync_mode.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

namespace morai_grpc_vendor::sync_mode
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

bool all_digits(const std::string & s)
{
  return !s.empty() && std::all_of(s.begin(), s.end(), [](char c) {
    return std::isdigit(static_cast<unsigned char>(c)) != 0;
  });
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

bool parse_range(const std::string & s, double lo, double hi, double * out)
{
  if (s.empty() || std::isspace(static_cast<unsigned char>(s.front()))) {
    return false;
  }
  try {
    size_t pos = 0;
    const double v = std::stod(s, &pos);
    if (pos != s.size() || !std::isfinite(v) || v < lo || v > hi) {
      return false;
    }
    *out = v;
    return true;
  } catch (const std::exception &) {
    return false;
  }
}

std::string fmt(const char * f, double a, double b = 0.0, double c = 0.0)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf), f, a, b, c);
  return buf;
}

}  // namespace

std::string sync_type_name(int type)
{
  switch (type) {
    case kSyncUnspecified:
      return "SYNC_MODE_TYPE_UNSPECIFIED";
    case kSyncTimeManagement:
      return "SYNC_MODE_TYPE_TIME_MANAGEMENT";
    case kSyncSynchronous:
      return "SYNC_MODE_TYPE_SYNCHRONOUS";
    default:
      return "UNKNOWN(" + std::to_string(type) + ")";
  }
}

int effective_tick_period_ms(int requested_ms)
{
  if (requested_ms == 0) {
    return 10;
  }
  if (requested_ms < 1 || requested_ms > 100) {
    return 20;
  }
  return requested_ms;
}

bool mode_accepts_control_in_sync(int mode)
{
  return mode == kModeAuto || mode == kModeSynchronous;
}

std::vector<int> backoff_delays_ms(int total_ms, int first_ms, int max_ms)
{
  std::vector<int> out;
  if (total_ms <= 0 || first_ms <= 0 || max_ms <= 0) {
    return out;
  }
  long sum = 0;
  long d = first_ms;
  while (sum + std::min<long>(d, max_ms) <= total_ms) {
    const long step = std::min<long>(d, max_ms);
    out.push_back(static_cast<int>(step));
    sum += step;
    d = std::min<long>(d * 2, max_ms);
    if (out.size() > 100000) {
      break;  // 방어: 비정상 입력
    }
  }
  return out;
}

bool is_epoch_us(int64_t ts_us)
{
  // 2017-01-01T00:00:00Z = 1483228800 s, 2100-01-01T00:00:00Z = 4102444800 s
  return ts_us >= 1483228800LL * 1000000LL && ts_us < 4102444800LL * 1000000LL;
}

FrozenCheck judge_frozen(const std::vector<TimeSample> & s, double min_span_s)
{
  FrozenCheck c;
  c.n = s.size();
  if (s.empty()) {
    c.why = "no timestamp samples";
    return c;
  }
  c.first_us = s.front().ts_us;
  c.last_us = s.back().ts_us;
  c.span_s = s.back().wall_s - s.front().wall_s;
  for (size_t i = 1; i < s.size(); ++i) {
    c.changes += s[i].ts_us != s[i - 1].ts_us ? 1 : 0;
  }
  if (c.n < 3) {
    c.why = "only " + std::to_string(c.n) + " samples";
  } else if (!(c.span_s >= min_span_s)) {
    c.why = fmt("samples span only %.3f s (< %.3f s)", c.span_s, min_span_s);
  } else if (c.changes > 0) {
    c.why = "timestamp changed " + std::to_string(c.changes) + " times in " +
            fmt("%.2f s (last - first = %.0f us): MORAI time is advancing without Tick",
                c.span_s, static_cast<double>(c.last_us - c.first_us));
  } else {
    c.frozen = true;
  }
  return c;
}

RealtimeCheck judge_realtime(
  const std::vector<TimeSample> & s, double local_unix_s_at_last, double min_span_s,
  double rate_lo, double rate_hi)
{
  RealtimeCheck c;
  c.n = s.size();
  c.rate = kNaN;
  c.offset_s = kNaN;
  if (s.empty()) {
    c.why = "no timestamp samples (state reads failing)";
    return c;
  }
  for (size_t i = 1; i < s.size(); ++i) {
    c.changes += s[i].ts_us != s[i - 1].ts_us ? 1 : 0;
  }
  c.span_s = s.back().wall_s - s.front().wall_s;
  if (c.span_s > 0.0) {
    c.rate = static_cast<double>(s.back().ts_us - s.front().ts_us) / 1e6 / c.span_s;
  }
  c.epoch = is_epoch_us(s.back().ts_us);
  if (c.epoch) {
    c.offset_s = static_cast<double>(s.back().ts_us) / 1e6 - local_unix_s_at_last;
  }
  if (!(c.span_s >= min_span_s)) {
    c.why = fmt("samples span only %.3f s (< %.3f s)", c.span_s, min_span_s);
  } else if (c.changes < 3) {
    c.why = "timestamp changed only " + std::to_string(c.changes) + " times in " +
            fmt("%.2f s: MORAI time is not advancing (still synchronous, or paused)", c.span_s);
  } else if (!(c.rate >= rate_lo && c.rate <= rate_hi)) {
    c.why = fmt("MORAI time advances at %.3f x wall rate (expected %.2f..%.2f)", c.rate, rate_lo,
                rate_hi);
  } else if (!c.epoch) {
    c.why = "timestamp " + std::to_string(s.back().ts_us) +
            " us is not Unix-epoch microseconds (sim time since a synchronous-mode entry?)";
  } else {
    c.ok = true;
  }
  return c;
}

int restore_exit_code(const RestoreOutcome & o)
{
  if (!o.reachable) {
    return kRestoreUnreachable;
  }
  if (!o.unspecified) {
    return kRestoreStillSync;
  }
  if (!o.realtime_ok) {
    return kRestoreNotAdvancing;
  }
  if (o.keyboard_requested && !o.keyboard_ok) {
    return kRestoreModeFailed;
  }
  return kRestoreOk;
}

void print_restore_usage(std::FILE * f)
{
  std::fprintf(
    f,
    "usage: morai_sync_restore [options]\n"
    "\n"
    "Emergency restore after a lockstep run (morai_actuate --lockstep) died and left MORAI\n"
    "frozen in SYNCHRONOUS mode. Idempotent: if MORAI already runs in real time it changes\n"
    "nothing. Never drives the Ego (no ControlVehicle, no Tick, no gear change).\n"
    "\n"
    "  1. GetSynchronousMode (retried while the server is UNAVAILABLE)\n"
    "  2. if not UNSPECIFIED: SetSynchronousMode(UNSPECIFIED, 0), retried up to --timeout-s,\n"
    "     its Result ignored (MORAI's Result is unreliable), until it reads UNSPECIFIED\n"
    "  3. verify ActorStates.timestamp advances at ~wall rate and is Unix-epoch based again\n"
    "  4. print the Ego's control mode and gear; --keyboard also sets KEYBOARD (read back)\n"
    "\n"
    "  --host H                default $MORAI_SIM_ADDRESS, else 127.0.0.1\n"
    "  --port P                default $MORAI_SIM_PORT, else 7789\n"
    "  --ego NAME              ego actor id (default \"Ego\")\n"
    "  --client-key K          ObjectInfo / filter client_key (default empty)\n"
    "  --keyboard              also set the Ego's control mode to KEYBOARD and read it back\n"
    "  --timeout-s S           retry budget for steps 1 and 2 (default 10, 1..120)\n"
    "  --verify-s S            real-time verification window (default 1.0, 0.3..10)\n"
    "  --deadline-ms MS        per-RPC deadline (default 500, 10..10000)\n"
    "  --connect-timeout-ms MS (default 3000)\n"
    "  -h, --help\n"
    "\n"
    "Exit: 0 real time and advancing (restored or already fine), 1 usage,\n"
    "2 MORAI unreachable, 3 still not UNSPECIFIED (use the MORAI UI Time Manager),\n"
    "4 UNSPECIFIED but time not advancing at ~wall rate (paused? check the MORAI UI),\n"
    "5 --keyboard: mode did not read back KEYBOARD.\n");
}

bool apply_restore_env_defaults(
  const char * env_host, const char * env_port, RestoreOptions * o, std::string * err)
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

bool parse_restore_args(
  const std::vector<std::string> & args, RestoreOptions * o, std::string * err)
{
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
      if (val.empty()) return bad("empty");
      o->host = val;
    } else if (a == "--port") {
      if (!need()) return false;
      if (!parse_int(val, 1, 65535, &o->port)) return bad("1..65535");
    } else if (a == "--ego") {
      if (!need()) return false;
      if (val.empty()) return bad("empty");
      o->ego = val;
    } else if (a == "--client-key") {
      if (!need()) return false;
      o->client_key = val;
    } else if (a == "--keyboard") {
      if (has_inline) {
        *err = a + " takes no value";
        return false;
      }
      o->keyboard = true;
    } else if (a == "--timeout-s") {
      if (!need()) return false;
      if (!parse_range(val, 1.0, 120.0, &o->timeout_s)) return bad("1..120 s");
    } else if (a == "--verify-s") {
      if (!need()) return false;
      if (!parse_range(val, 0.3, 10.0, &o->verify_s)) return bad("0.3..10 s");
    } else if (a == "--deadline-ms") {
      if (!need()) return false;
      if (!parse_int(val, 10, 10000, &o->deadline_ms)) return bad("10..10000");
    } else if (a == "--connect-timeout-ms") {
      if (!need()) return false;
      if (!parse_int(val, 1, 600000, &o->connect_timeout_ms)) return bad("1..600000");
    } else if (!a.empty() && a[0] == '-') {
      *err = "unknown option " + a;
      return false;
    } else {
      *err = "unexpected argument " + a;
      return false;
    }
  }
  return true;
}

}  // namespace morai_grpc_vendor::sync_mode
