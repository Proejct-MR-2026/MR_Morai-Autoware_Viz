// 신규 — 26.09.25 _ YSH _ actuate_options.hpp 구현 (엄격한 숫자 파싱·범위 검증·패턴 이름 검증)
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

#include "actuate_options.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <set>
#include <string>
#include <vector>

namespace morai_grpc_vendor::actuate
{

namespace
{

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

/// Finite decimal, fully consumed, within [lo, hi].
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

std::vector<std::string> split_csv(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',') {
      out.push_back(cur);
      cur.clear();
    } else {
      cur += c;
    }
  }
  out.push_back(cur);
  return out;
}

}  // namespace

const std::vector<PatternSpec> & known_patterns()
{
  static const std::vector<PatternSpec> k = {
    {"locked1", true, 1, 1, 0.0, false, "1 ControlVehicle right after each WaitForTick return"},
    {"locked_half", true, 2, 1, 0.0, false, "1 ControlVehicle after every 2nd WaitForTick"},
    {"burst4", true, 1, 4, 0.0, false, "4 back-to-back ControlVehicle after each WaitForTick"},
    {"free33", false, 0, 1, 100.0 / 3.0, false, "33.3 Hz timer, blocking (Autoware one-shot)"},
    {"free100", false, 0, 1, 100.0, false, "100 Hz timer, blocking (main.py resender)"},
    {"free200", false, 0, 1, 200.0, true, "200 Hz timer, pipelined async (capped in-flight)"},
  };
  return k;
}

const PatternSpec * find_pattern(const std::string & name)
{
  for (const auto & p : known_patterns()) {
    if (name == p.name) {
      return &p;
    }
  }
  return nullptr;
}

// 26.09.25 _ YSH _ lockstep 패턴 표 — 모두 정지에서 +accel, 조향 0. Tick(1) 은 1 프레임(= tick_period)
const std::vector<LockstepPatternSpec> & known_lockstep_patterns()
{
  static const std::vector<LockstepPatternSpec> k = {
    {"sync1", 1, 1, "every step: ControlVehicle -> Tick(1) -> read"},
    {"sync_drop2", 2, 1, "ControlVehicle before even steps only; odd steps Tick(1) without one"},
    {"sync_tick3", 1, 3, "ControlVehicle once -> Tick(3) -> read (does the input persist?)"},
    {"sync_tick3x", 3, 1, "ControlVehicle once -> Tick(1) x3, no command on the 2nd/3rd"},
  };
  return k;
}

const LockstepPatternSpec * find_lockstep_pattern(const std::string & name)
{
  for (const auto & p : known_lockstep_patterns()) {
    if (name == p.name) {
      return &p;
    }
  }
  return nullptr;
}

std::vector<std::string> default_lockstep_patterns()
{
  std::vector<std::string> out;
  for (const auto & p : known_lockstep_patterns()) {
    out.push_back(p.name);
  }
  return out;
}

void print_usage(std::FILE * f)
{
  std::fprintf(
    f,
    "usage: morai_actuate %s [options]\n"
    "\n"
    "MOVES THE EGO. Drives it straight (steer 0) from standstill with +accel for --drive-s,\n"
    "brakes, and repeats that for several ControlVehicle send patterns, to find out whether\n"
    "MORAI consumes ControlVehicle per update or per physics step. Switches the Ego to\n"
    "AUTO_MODE and gear D, and restores the ORIGINAL mode and gear on every exit path\n"
    "(normal end, Ctrl-C, SIGTERM, error). Put the Ego on a long, empty, straight road.\n"
    "\n"
    "  --host H                  default $MORAI_SIM_ADDRESS, else 127.0.0.1\n"
    "  --port P                  default $MORAI_SIM_PORT, else 7789\n"
    "  --ego NAME                ego actor id (default \"Ego\", as main.py)\n"
    "  --client-key K            ObjectInfo / filter client_key (default empty, as main.py)\n"
    "  --accel A                 drive command, m/s^2 (default 0.8, 0.1..2.0)\n"
    "  --drive-s S               drive time in MORAI time (default 2.0, 1..10)\n"
    "  --settle-s S              brake hold after each stop (default 1.0, 0..10)\n"
    "  --repeats N               runs per pattern, interleaved (default 2, 1..20)\n"
    "  --patterns a,b,c          subset/order of: locked1,locked_half,burst4,free33,free100,\n"
    "                            free200 (default: all six)\n"
    "  --order abba|fixed        abba: A B C, C B A, ... (drift cancels; default)\n"
    "                            fixed: A B C, A B C, ...\n"
    "  --allow-any-original-mode also take over an Ego in AUTO / UNSPECIFIED / other modes\n"
    "                            (default: only KEYBOARD / GAME_WHEEL; AUTO may mean a bridge\n"
    "                            or Autoware is still sending)\n"
    "  --max-speed V             envelope, m/s (default 3.0, 0.1..10)\n"
    "  --max-distance D          envelope per pattern run, m (default 15, 1..100)\n"
    "  --max-total-distance D    travel budget for the session, m (default 60, 1..500)\n"
    "  --physics-hz HZ           MORAI Hz you set (hint for the step estimate; 0 = none)\n"
    "  --reader-hz HZ            state reader pacing, 0 = tight loop (default 0)\n"
    "  --deadline-ms MS          per-RPC deadline (default 100, 1..5000)\n"
    "  --wft-deadline-ms MS      WaitForTick deadline (default 250, 10..5000)\n"
    "  --connect-timeout-ms MS   (default 3000)\n"
    "  --max-inflight N          free200 outstanding async sends (default 8, 1..64)\n"
    "  --idle-probe-s S          no-command window to measure the idle record (default 0.5)\n"
    "  --idle-brake B            idle record brake (default 0.003, main.py)\n"
    "  --idle-brake-tol T        (default 0.0015)\n"
    "  --idle-throttle-max T     (default 0.001)\n"
    "  --no-countdown            skip the 3 s countdown\n"
    "  --json PATH               write everything (raw samples, sends, analysis) as JSON\n"
    "  -h, --help\n"
    "\n"
    // 26.09.25 _ YSH _ --lockstep 사용법 추가
    "LOCKSTEP (--lockstep, needs %s too): MORAI SYNCHRONOUS mode, the tool paces MORAI\n"
    "with Simulation.Tick - one command per physics step, sim time may run slower than real\n"
    "time. Refuses unless MORAI is in real-time (UNSPECIFIED) mode. On every exit: safe-stop\n"
    "by Ticks -> original gear (only once stopped) -> SetSynchronousMode(UNSPECIFIED) ->\n"
    "still moving: real-time brake -> verify real time -> original control mode -> gear ->\n"
    "standstill check. If that fails: exit 6, run morai_sync_restore.\n"
    "  --lockstep                use MORAI synchronous mode (patterns below)\n"
    "  --tick-period-ms MS       SyncMode.tick_period = sim time per frame (default 10, 1..100)\n"
    "  --tick-deadline-ms MS     deadline of one Tick RPC (default 1000, 50..10000)\n"
    "  --patterns a,b            subset/order of: sync1,sync_drop2,sync_tick3,sync_tick3x\n"
    "                            (default: all four). --drive-s / --settle-s / --idle-probe-s\n"
    "                            are MORAI (sim) seconds. --physics-hz, --reader-hz,\n"
    "                            --wft-deadline-ms and --max-inflight do not apply.\n"
    "\n"
    "Envelope (abort run -> safe-stop -> restore): speed > --max-speed, displacement from the\n"
    "run start > --max-distance, state reads failing > 0.5 s, MORAI time frozen > 1 s,\n"
    "collision, a steer echo (someone else commanding). Safe-stop: acceleration -3.0 m/s^2,\n"
    "steer 0, at 100 Hz until |v| < 0.05 m/s for 1 s (timeout 10 s). SIGINT, SIGTERM, SIGHUP,\n"
    "SIGQUIT (Ctrl-\\) and SIGTSTP (Ctrl-Z) all abort and restore. If the tool is killed\n"
    "(kill -9) or crashes, set the Ego to KEYBOARD / P in the MORAI UI by hand.\n"
    "\n"
    "Exit: 0 ok, 1 usage/refused, 2 connect or preflight failed (nothing changed),\n"
    "3 cannot write --json, 4 setup failed (restored), 5 aborted (restored),\n"
    // 26.09.25 _ YSH _ lockstep 복구 실패 안내 추가 (확인 플래그 인자도 하나 더)
    // "6 RESTORE FAILED - check MORAI now.\n",
    // kConfirmFlag);
    "6 RESTORE FAILED - check MORAI now (lockstep: MORAI may be frozen in synchronous mode\n"
    "  - run morai_sync_restore).\n",
    kConfirmFlag, kConfirmFlag);
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
  // 26.09.25 _ YSH _ 모드 교차 검증용 — 명시된 패턴 목록·실시간 전용·lockstep 전용 옵션 기록
  bool patterns_given = false;
  std::vector<std::string> realtime_only;
  std::vector<std::string> lockstep_only;
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
    auto num = [&](double lo, double hi, double * out, const char * why) -> bool {
      if (!need()) return false;
      if (!parse_range(val, lo, hi, out)) return bad(why);
      return true;
    };
    auto flag = [&]() -> bool {
      if (has_inline) {
        *err = a + " takes no value";
        return false;
      }
      return true;
    };

    if (a == "-h" || a == "--help") {
      o->help = true;
      return true;
    } else if (a == kConfirmFlag) {
      if (!flag()) return false;
      o->confirmed = true;
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
    } else if (a == "--accel") {
      if (!num(0.1, 2.0, &o->accel, "0.1..2.0 m/s^2")) return false;
    } else if (a == "--drive-s") {
      if (!num(1.0, 10.0, &o->drive_s, "1..10 s")) return false;
    } else if (a == "--settle-s") {
      if (!num(0.0, 10.0, &o->settle_s, "0..10 s")) return false;
    } else if (a == "--repeats") {
      if (!need()) return false;
      if (!parse_int(val, 1, 20, &o->repeats)) return bad("integer 1..20");
    } else if (a == "--patterns") {
      if (!need()) return false;
      std::vector<std::string> list = split_csv(val);
      std::set<std::string> seen;
      for (const auto & p : list) {
        if (p.empty()) return bad("empty pattern name");
        // 26.09.25 _ YSH _ lockstep 패턴 이름도 받고, 모드와 맞는지는 끝에서 검사
        // if (!find_pattern(p)) return bad("unknown pattern");
        if (!find_pattern(p) && !find_lockstep_pattern(p)) return bad("unknown pattern");
        if (!seen.insert(p).second) return bad("duplicate pattern");
      }
      o->patterns = list;
      // 26.09.25 _ YSH _ 명시 여부 (lockstep 기본 목록으로 바꿀지 결정)
      patterns_given = true;
    } else if (a == "--order") {
      if (!need()) return false;
      if (val != "abba" && val != "fixed") return bad("abba or fixed");
      o->order = val;
    } else if (a == "--allow-any-original-mode") {
      if (!flag()) return false;
      o->allow_any_mode = true;
    } else if (a == "--max-speed") {
      if (!num(0.1, 10.0, &o->max_speed, "0.1..10 m/s")) return false;
    } else if (a == "--max-distance") {
      if (!num(1.0, 100.0, &o->max_distance, "1..100 m")) return false;
    } else if (a == "--max-total-distance") {
      if (!num(1.0, 500.0, &o->max_total_distance, "1..500 m")) return false;
    } else if (a == "--physics-hz") {
      // 26.09.25 _ YSH _ 실시간 전용 옵션 기록 (--lockstep 과 함께 주면 거부)
      realtime_only.push_back(a);
      if (!num(0.0, 1000.0, &o->physics_hz, "0, or 1..1000")) return false;
      if (o->physics_hz != 0.0 && o->physics_hz < 1.0) return bad("0, or 1..1000");
    } else if (a == "--reader-hz") {
      // 26.09.25 _ YSH _ 실시간 전용 옵션 기록
      realtime_only.push_back(a);
      if (!num(0.0, 100000.0, &o->reader_hz, "0, or 10..100000")) return false;
      if (o->reader_hz != 0.0 && o->reader_hz < 10.0) return bad("0, or 10..100000");
    } else if (a == "--deadline-ms") {
      if (!need()) return false;
      if (!parse_int(val, 1, 5000, &o->deadline_ms)) return bad("1..5000");
    } else if (a == "--wft-deadline-ms") {
      // 26.09.25 _ YSH _ 실시간 전용 옵션 기록 (동기 모드의 WaitForTick 은 즉시 0 을 돌려준다)
      realtime_only.push_back(a);
      if (!need()) return false;
      if (!parse_int(val, 10, 5000, &o->wft_deadline_ms)) return bad("10..5000");
    } else if (a == "--connect-timeout-ms") {
      if (!need()) return false;
      if (!parse_int(val, 1, 600000, &o->connect_timeout_ms)) return bad("1..600000");
    } else if (a == "--max-inflight") {
      // 26.09.25 _ YSH _ 실시간 전용 옵션 기록
      realtime_only.push_back(a);
      if (!need()) return false;
      if (!parse_int(val, 1, 64, &o->max_inflight)) return bad("1..64");
    } else if (a == "--idle-probe-s") {
      if (!num(0.0, 5.0, &o->idle_probe_s, "0..5 s")) return false;
    } else if (a == "--idle-brake") {
      if (!num(0.0, 1.0, &o->idle_brake, "0..1")) return false;
    } else if (a == "--idle-brake-tol") {
      if (!num(0.0, 1.0, &o->idle_brake_tol, "0..1")) return false;
    } else if (a == "--idle-throttle-max") {
      if (!num(0.0, 1.0, &o->idle_throttle_max, "0..1")) return false;
    } else if (a == "--no-countdown") {
      if (!flag()) return false;
      o->countdown = false;
    } else if (a == "--json") {
      if (!need()) return false;
      if (val.empty()) return bad("empty");
      o->json_path = val;
      // 26.09.25 _ YSH _ lockstep 옵션 (MORAI 범위 1..100 ms 밖의 tick_period 는 서버가 20 ms 로 바꾸므로 거부)
    } else if (a == "--lockstep") {
      if (!flag()) return false;
      o->lockstep = true;
    } else if (a == "--tick-period-ms") {
      lockstep_only.push_back(a);
      if (!need()) return false;
      if (!parse_int(val, 1, 100, &o->tick_period_ms)) return bad("integer 1..100 ms");
    } else if (a == "--tick-deadline-ms") {
      lockstep_only.push_back(a);
      if (!need()) return false;
      if (!parse_int(val, 50, 10000, &o->tick_deadline_ms)) return bad("50..10000");
    } else if (!a.empty() && a[0] == '-') {
      *err = "unknown option " + a;
      return false;
    } else {
      *err = "unexpected argument " + a;
      return false;
    }
  }
  if (!o->confirmed) {
    *err = std::string("refusing to run: this tool MOVES THE EGO. Add ") + kConfirmFlag +
           " if the Ego is on a long, empty, straight road and nobody else is driving it";
    return false;
  }
  // 26.09.25 _ YSH _ 실시간/lockstep 모드 교차 검증 (패턴 표·전용 옵션)
  if (o->lockstep) {
    if (!realtime_only.empty()) {
      *err = realtime_only.front() + " does not apply with --lockstep (real-time mode only)";
      return false;
    }
    if (!patterns_given) {
      o->patterns = default_lockstep_patterns();
    }
    for (const auto & p : o->patterns) {
      if (!find_lockstep_pattern(p)) {
        *err = "bad value for --patterns: '" + p +
               "' is a real-time pattern; with --lockstep use sync1,sync_drop2,sync_tick3,"
               "sync_tick3x";
        return false;
      }
    }
  } else {
    if (!lockstep_only.empty()) {
      *err = lockstep_only.front() + " needs --lockstep";
      return false;
    }
    for (const auto & p : o->patterns) {
      if (!find_pattern(p)) {
        *err = "bad value for --patterns: '" + p + "' is a lockstep pattern; add --lockstep";
        return false;
      }
    }
  }
  return true;
}

std::vector<std::string> run_order(const Options & o)
{
  std::vector<std::string> out;
  for (int rep = 0; rep < o.repeats; ++rep) {
    std::vector<std::string> block = o.patterns;
    if (o.order == "abba" && rep % 2 == 1) {
      std::reverse(block.begin(), block.end());
    }
    out.insert(out.end(), block.begin(), block.end());
  }
  return out;
}

}  // namespace morai_grpc_vendor::actuate
