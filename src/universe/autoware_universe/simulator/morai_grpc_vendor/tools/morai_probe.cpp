// 신규 — 26.09.24 _ YSH _ MORAI 시간 동기 사전 측정용 읽기 전용 CLI (Stage 1)
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

// morai_probe — measure what MORAI SIM actually provides for time synchronisation.
//
// STRICTLY READ-ONLY. The only RPCs this file may issue are listed in kAllowedRpcs in
// test/test_probe_readonly.cpp, which scans this source for `<name>_stub_->Method(` and
// fails on anything else. Never add Set*, Control*, Tick, Start/Stop/Pause/Resume,
// SetSynchronousMode, Spawn*/Destroy* or any other state-changing call here.

// 26.09.25 _ YSH _ monitor 서브커맨드: 순수 집계부(monitor_stats), MoraiLink 와 같은 proto->구조체 변환
//   (morai_link_convert — 변환만 쓰고 MoraiLink 객체는 만들지 않는다, 쓰기 RPC 가 있으므로)
#include "monitor_stats.hpp"
#include "morai_grpc_vendor/morai_link_types.hpp"
#include "morai_link_convert.hpp"
#include "probe_json.hpp"
#include "probe_options.hpp"
#include "probe_stats.hpp"

#include <google/protobuf/descriptor.h>
#include <google/protobuf/util/json_util.h>
#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/actor/actor_get.pb.h>
#include <morai/common/enum.pb.h>
#include <morai/common/object_info.pb.h>
#include <morai/common/type.pb.h>
#include <morai/simulation/simulation.grpc.pb.h>
#include <morai/simulation/simulation_enum.pb.h>
#include <morai/simulation/sync_mode.pb.h>
#include <morai/simulator/simulator.grpc.pb.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cinttypes>
#include <cmath>
// 26.09.25 _ YSH _ monitor 의 Ctrl-C 처리
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
// 26.09.25 _ YSH _ --json·--csv 부모 디렉터리 생성
#include <filesystem>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <string>
// 26.09.25 _ YSH _ --json·--csv 부모 디렉터리 생성 (std::error_code)
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

namespace
{

using morai_grpc_vendor::probe::analyze_frames;
using morai_grpc_vendor::probe::classify_time_base;
using morai_grpc_vendor::probe::classify_wait_for_tick;
using morai_grpc_vendor::probe::count_over;
using morai_grpc_vendor::probe::FrameAnalysis;
using morai_grpc_vendor::probe::grpc_code_name;
using morai_grpc_vendor::probe::Json;
using morai_grpc_vendor::probe::Options;
using morai_grpc_vendor::probe::RawTable;
using morai_grpc_vendor::probe::summarize;
using morai_grpc_vendor::probe::Summary;
using morai_grpc_vendor::probe::TimedSample;
using morai_grpc_vendor::probe::to_json;
using morai_grpc_vendor::probe::WaitForTickObservation;

// 26.09.25 _ YSH _ monitor 가 쓰는 공개 자료형 (grpc 비의존)
using morai_grpc_vendor::ControlMode;
using morai_grpc_vendor::GearMode;

using Clock = std::chrono::steady_clock;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;  // same as MORAI's Python SDK
/// rtt / frames / simtime give up after this many DEADLINE_EXCEEDED in a row: MORAI
/// accepts TCP but does not answer (hung main thread, scenario load).
constexpr size_t kMaxConsecutiveDeadlines = 10;
using Cell = RawTable::Cell;

// =============================================================================
// Helpers
// =============================================================================

std::string enum_name(const google::protobuf::EnumDescriptor * d, int v)
{
  const google::protobuf::EnumValueDescriptor * vd = d ? d->FindValueByNumber(v) : nullptr;
  return vd ? vd->name() : "UNKNOWN(" + std::to_string(v) + ")";
}

Json proto_json(const google::protobuf::Message & m)
{
  google::protobuf::util::JsonPrintOptions opts;
  opts.always_print_primitive_fields = true;
  opts.preserve_proto_field_names = true;
  std::string out;
  if (!google::protobuf::util::MessageToJsonString(m, &out, opts).ok()) {
    return Json();
  }
  return Json::raw(out);
}

Json codes_json(const std::map<std::string, size_t> & codes)
{
  Json j = Json::object();
  for (const auto & kv : codes) {
    j.set(kv.first, kv.second);
  }
  return j;
}

std::string codes_text(const std::map<std::string, size_t> & codes)
{
  std::string s;
  for (const auto & kv : codes) {
    if (!s.empty()) {
      s += " ";
    }
    s += kv.first + "=" + std::to_string(kv.second);
  }
  return s.empty() ? "-" : s;
}

double unix_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

// 26.09.25 _ YSH _ monitor: 수신 system 시각(Unix epoch ns) — MORAI timestamp(µs) 와 정수로 뺀다
int64_t unix_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

// 26.09.25 _ YSH _ monitor: 핸들러는 플래그만 세운다 (루프가 deadline 안에 빠져나와 CSV·JSON 을 마저 쓴다)
volatile std::sig_atomic_t g_monitor_stop = 0;
void on_monitor_signal(int)
{
  g_monitor_stop = 1;
}

double ms_between(Clock::time_point a, Clock::time_point b)
{
  return std::chrono::duration<double, std::milli>(b - a).count();
}

/// Options bound seconds to <= 3600, so this cannot overflow Clock::duration.
Clock::time_point after(Clock::time_point t, double seconds)
{
  return t + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(seconds));
}

bool iequals(const std::string & a, const std::string & b)
{
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) ==
                  std::tolower(static_cast<unsigned char>(y));
         });
}

std::string pace_text(double hz, const char * unpaced)
{
  if (!(hz > 0.0)) {
    return unpaced;
  }
  char buf[48];
  std::snprintf(buf, sizeof(buf), "@%g Hz", hz);
  return buf;
}

// 26.09.25 _ YSH _ monitor 시작 줄의 창 길이 ("13 s", "7200 s")
std::string seconds_text(double s)
{
  char buf[48];
  std::snprintf(buf, sizeof(buf), "%g s", s);
  return buf;
}

double per_min(size_t count, double span_s)
{
  return span_s > 0.0 ? static_cast<double>(count) / (span_s / 60.0) : kNaN;
}

void warn_if_dropped(const char * tag, const RawTable & t)
{
  if (t.dropped() > 0) {
    std::printf(
      "[%s] WARNING: raw row cap hit - %zu rows stored, %zu dropped (raise --max-raw-rows)\n", tag,
      t.rows(), t.dropped());
  }
}

/// Paces a loop at `hz` (0 = no pacing). After a stall it resumes instead of bursting to
/// catch up, so a slow simulator is not hammered with back-to-back calls. Options bound
/// hz to 0 or [0.1, 1e5], so the period cannot overflow.
class Pacer
{
public:
  explicit Pacer(double hz)
  : period_(
      hz > 0.0
        ? std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz))
        : Clock::duration::zero()),
    next_(Clock::now())
  {
  }

  void wait()
  {
    if (period_ <= Clock::duration::zero()) {
      return;
    }
    std::this_thread::sleep_until(next_);
    next_ += period_;
    const auto now = Clock::now();
    if (now > next_ + period_) {
      next_ = now;
    }
  }

private:
  Clock::duration period_;
  Clock::time_point next_;
};

/// Raw rows are stored only when the observed state changes. Polls that repeat the last
/// stored state are counted here and summarised in the next stored row (count, first and
/// last poll time), which keeps everything a frame or stall re-analysis needs at a
/// fraction of the size of one row per poll.
class RepeatRun
{
public:
  void add(double t_ms)
  {
    if (count_ == 0) {
      first_ = t_ms;
    }
    last_ = t_ms;
    ++count_;
  }
  Cell count() const { return Cell::u(count_); }
  Cell first() const { return count_ ? Cell::d(first_) : Cell::null(); }
  Cell last() const { return count_ ? Cell::d(last_) : Cell::null(); }
  void clear() { count_ = 0; }
  Json to_json() const
  {
    return Json::object()
      .set("count", count_)
      .set("first_t_ms", count_ ? Json(first_) : Json())
      .set("last_t_ms", count_ ? Json(last_) : Json());
  }

private:
  uint64_t count_{0};
  double first_{0.0};
  double last_{0.0};
};

void print_frame_analysis(const char * tag, const FrameAnalysis & a)
{
  std::printf(
    "[%s] samples=%zu over %.1f s, poll p50=%.3f ms | changes=%zu dup_ratio=%.3f "
    "monotonic_violations=%zu overflows=%zu zero_values=%zu\n",
    tag, a.samples, a.wall_span_s, a.poll_interval_ms.p50, a.changes, a.duplicate_ratio,
    a.monotonic_violations, a.delta_overflows, a.zero_values);
  std::string top;
  for (const auto & h : a.dvalue_hist_top) {
    top += (top.empty() ? "" : " ") + std::to_string(h.dvalue) + ":" + std::to_string(h.count);
  }
  std::printf(
    "[%s] d(timestamp) between distinct: p50=%.6g min=%.6g max=%.6g mode=%" PRId64
    " (%.0f%%) gcd=%" PRId64 " raw units; top %s\n",
    tag, a.dvalue_raw.p50, a.dvalue_raw.min, a.dvalue_raw.max, a.dvalue_mode_raw,
    a.dvalue_mode_share * 100.0, a.dvalue_gcd_raw, top.empty() ? "-" : top.c_str());
  std::printf(
    "[%s] raw/wall-s=%.6g -> unit=%s rate_ratio=%.4f %s | sim step=%.5g ms/frame, wall frame "
    "rate=%.3f Hz, observed updates=%.3f/s; assuming ms: %.4g ms\n",
    tag, a.raw_per_wall_s, a.unit.name.c_str(), a.unit.rate_ratio,
    a.unit.plausible ? "(plausible)" : "(IMPLAUSIBLE - check)", a.sim_step_s * 1e3,
    a.wall_frame_rate_hz, a.changes_per_wall_s, a.frame_period_ms_assuming_ms);
  if (a.frame_period_reliable) {
    std::printf(
      "[%s] frame period: %.5g ms (%.3f Hz) - reliable\n", tag, a.frame_period_s * 1e3,
      a.frame_rate_hz);
  } else {
    std::printf("[%s] frame period: UNRELIABLE (see WARNING lines)\n", tag);
  }
  if (a.deltas_are_multiples) {
    std::printf(
      "[%s] physics-step candidate (delta gcd): %.5g ms (%.3f Hz)\n", tag, a.gcd_step_s * 1e3,
      1.0 / a.gcd_step_s);
  }
  std::string khist;
  size_t shown = 0;
  for (const auto & kv : a.frames_per_change_hist) {
    if (shown++ >= 8) {
      khist += " ...";
      break;
    }
    khist +=
      (khist.empty() ? "" : " ") + std::to_string(kv.first) + ":" + std::to_string(kv.second);
  }
  std::printf(
    "[%s] missed-frame events=%zu (%.2f/min over %.1f s): wall gap > 1.5 x %.4g ms=%zu, "
    ">1 frame per change=%zu; max gap %.2f frames; frames/change %s\n",
    tag, a.missed_frame_events, a.missed_frame_events_per_min, a.wall_span_s, a.wall_period_ms,
    a.wall_stall_events, a.multi_frame_changes, a.max_gap_in_frames,
    khist.empty() ? "-" : khist.c_str());
  std::printf(
    "[%s] wall between changes p50=%.2f p90=%.2f p99=%.2f max=%.2f ms (absolute: >20ms=%zu "
    ">45ms=%zu) | changes per 1 s bucket min=%.0f p50=%.0f max=%.0f\n",
    tag, a.change_interval_ms.p50, a.change_interval_ms.p90, a.change_interval_ms.p99,
    a.change_interval_ms.max, a.change_intervals_over_20ms, a.change_intervals_over_45ms,
    a.changes_per_second_bucket.min, a.changes_per_second_bucket.p50,
    a.changes_per_second_bucket.max);
  if (a.tick_present) {
    std::printf(
      "[%s] sync_mode_tick_count: d(tick)/change p50=%.3g min=%.3g max=%.3g; tick moved w/o "
      "timestamp=%zu\n",
      tag, a.dtick_per_change.p50, a.dtick_per_change.min, a.dtick_per_change.max,
      a.tick_changes_without_value_change);
  }
  for (const auto & w : a.warnings) {
    std::printf("[%s] WARNING: %s\n", tag, w.c_str());
  }
  if (!(std::isfinite(a.wall_period_ms) && a.wall_period_ms > 0.0)) {
    return;
  }
  for (const auto & g : a.largest_gaps) {  // descending: stop at the first non-stall
    if (!(g.gap_ms > 1.5 * a.wall_period_ms)) {
      break;
    }
    std::printf(
      "[%s]   stall at t=%.3f s: %.1f ms wall (%.2f frames), timestamp +%" PRId64 " raw (%" PRId64
      " frames), tick +%" PRId64 "\n",
      tag, g.wall_s, g.gap_ms, g.gap_ms / a.wall_period_ms, g.dvalue, g.frames, g.dtick);
  }
}

// =============================================================================
// Probe
// =============================================================================

class Probe
{
public:
  explicit Probe(const Options & o) : opt_(o)
  {
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);
    channel_ = grpc::CreateCustomChannel(
      opt_.host + ":" + std::to_string(opt_.port), grpc::InsecureChannelCredentials(), args);
    actor_stub_ = morai_sim_api::actor::Actor::NewStub(channel_);
    simulation_stub_ = morai_sim_api::simulation::Simulation::NewStub(channel_);
    // NOTE upstream spells this package "simualtor" (sic).
    simulator_stub_ = morai_sim_api::simualtor::Simulator::NewStub(channel_);

    ego_.mutable_id()->set_value(opt_.ego);
    ego_.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);  // as main.py
  }

  /// grpc::CreateChannel is lazy; this is the only proof the simulator is reachable.
  bool connect(double * connect_ms)
  {
    const auto a = Clock::now();
    const bool ok = channel_->WaitForConnected(
      std::chrono::system_clock::now() + std::chrono::milliseconds(opt_.connect_timeout_ms));
    *connect_ms = ms_between(a, Clock::now());
    return ok;
  }

  Json simulator_version()
  {
    grpc::ClientContext ctx;
    set_deadline(&ctx);
    morai_sim_api::Empty req;
    morai_sim_api::StringValue res;
    const grpc::Status st = simulator_stub_->GetSimulatorVersion(&ctx, req, &res);
    if (!st.ok()) {
      std::printf("[info] GetSimulatorVersion: %s\n", grpc_code_name(st.error_code()).c_str());
      return Json();
    }
    std::printf("[info] simulator version: %s\n", res.value().c_str());
    return Json(res.value());
  }

  // ---------------------------------------------------------------------------
  Json run_sync()
  {
    grpc::ClientContext ctx;
    set_deadline(&ctx);
    morai_sim_api::Empty req;
    morai_sim_api::simulation::SyncMode res;
    const auto a = Clock::now();
    const grpc::Status st = simulation_stub_->GetSynchronousMode(&ctx, req, &res);
    const double ms = ms_between(a, Clock::now());

    Json j = Json::object().set("status", grpc_code_name(st.error_code())).set("call_ms", ms);
    if (!st.ok()) {
      j.set("error", st.error_message());
      std::printf(
        "[sync] GetSynchronousMode failed: %s %s\n", grpc_code_name(st.error_code()).c_str(),
        st.error_message().c_str());
      return j;
    }
    const std::string type =
      enum_name(morai_sim_api::simulation::SyncModeType_descriptor(), res.type());
    sync_tick_period_ms_ = res.tick_period();
    sync_type_ = type;
    j.set("type", static_cast<int>(res.type()))
      .set("type_name", type)
      .set("tick_period_ms", res.tick_period())
      .set("response", proto_json(res));
    std::printf(
      "[sync] type=%s(%d) tick_period=%d (proto: msec, range 1..100)\n", type.c_str(),
      static_cast<int>(res.type()), res.tick_period());
    return j;
  }

  // ---------------------------------------------------------------------------
  Json run_spec()
  {
    grpc::ClientContext ctx;
    set_deadline(&ctx);
    morai_sim_api::actor::VehicleSpec res;
    const grpc::Status st = actor_stub_->GetVehicleSpec(&ctx, ego_, &res);
    Json j = Json::object().set("status", grpc_code_name(st.error_code()));
    if (!st.ok()) {
      j.set("error", st.error_message());
      std::printf(
        "[spec] GetVehicleSpec(%s) failed: %s %s\n", opt_.ego.c_str(),
        grpc_code_name(st.error_code()).c_str(), st.error_message().c_str());
      return j;
    }
    constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
    const std::string cls =
      enum_name(morai_sim_api::actor::VehicleClass_descriptor(), res.sub_category());
    spec_max_steer_deg_ = res.max_steering_angle();
    j.set("sub_category", cls)
      .set("length_m", static_cast<double>(res.length()))
      .set("width_m", static_cast<double>(res.width()))
      .set("height_m", static_cast<double>(res.height()))
      .set("wheel_base_m", static_cast<double>(res.wheel_base()))
      .set("front_overhang_m", static_cast<double>(res.front_overhang()))
      .set("rear_overhang_m", static_cast<double>(res.rear_overhang()))
      .set("max_steering_angle_deg", static_cast<double>(res.max_steering_angle()))
      .set("max_steering_angle_rad", static_cast<double>(res.max_steering_angle()) * kDegToRad)
      .set("wheel_radius_m", static_cast<double>(res.wheel_radius()))
      .set("response", proto_json(res));
    std::printf(
      "[spec] %s L=%.3f W=%.3f H=%.3f wheelbase=%.3f overhang f/r=%.3f/%.3f m, "
      "max_steer=%.3f deg (%.4f rad), wheel_r=%.3f m, dynamics_levels=%d\n",
      cls.c_str(), res.length(), res.width(), res.height(), res.wheel_base(), res.front_overhang(),
      res.rear_overhang(), res.max_steering_angle(), res.max_steering_angle() * kDegToRad,
      res.wheel_radius(), res.dynamics_level_size());
    return j;
  }

  // ---------------------------------------------------------------------------
  Json run_mode()
  {
    grpc::ClientContext ctx;
    set_deadline(&ctx);
    morai_sim_api::actor::CurVehicleControlMode res;
    const grpc::Status st = actor_stub_->GetVehicleControlMode(&ctx, ego_, &res);
    Json j = Json::object().set("status", grpc_code_name(st.error_code()));
    if (!st.ok()) {
      j.set("error", st.error_message());
      std::printf(
        "[mode] GetVehicleControlMode(%s) failed: %s %s\n", opt_.ego.c_str(),
        grpc_code_name(st.error_code()).c_str(), st.error_message().c_str());
      return j;
    }
    const std::string name =
      enum_name(morai_sim_api::actor::VehicleControlMode_descriptor(), res.mode());
    control_mode_ = name;
    j.set("mode", static_cast<int>(res.mode())).set("mode_name", name);
    std::printf("[mode] %s(%d)\n", name.c_str(), static_cast<int>(res.mode()));
    return j;
  }

  // ---------------------------------------------------------------------------
  /// Duration-bound (--rtt-seconds): the tail (50-69 ms stalls every ~20 s) needs minutes,
  /// not a few seconds, to show up.
  Json run_rtt()
  {
    std::map<std::string, size_t> codes;
    std::vector<double> ok_ms;
    std::vector<double> all_ms;  // every call; a DEADLINE_EXCEEDED call counts as the deadline
    Json slow = Json::array();
    size_t slow_count = 0;
    constexpr size_t kMaxSlowCalls = 200;
    RawTable raw({"t_ms", "rtt_ms", "grpc_code"}, opt_.max_raw_rows);
    Json first_ok;
    size_t calls = 0;
    size_t timeouts = 0;
    size_t consecutive_deadline = 0;
    bool stopped_early = false;
    const auto deadline_ms = static_cast<double>(opt_.deadline_ms);

    const auto start = Clock::now();
    const auto end = after(start, opt_.rtt_seconds);
    Pacer pacer(opt_.rate_hz);
    while (Clock::now() < end && (opt_.n == 0 || calls < opt_.n)) {
      pacer.wait();
      grpc::ClientContext ctx;
      set_deadline(&ctx);
      morai_sim_api::actor::ActorState res;
      const auto a = Clock::now();
      const grpc::Status st = actor_stub_->GetActorState(&ctx, ego_, &res);
      const auto b = Clock::now();
      const double ms = ms_between(a, b);
      const std::string code = grpc_code_name(st.error_code());
      const bool timeout = st.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED;
      ++calls;
      ++codes[code];
      all_ms.push_back(timeout ? std::min(ms, deadline_ms) : ms);
      if (st.ok()) {
        ok_ms.push_back(ms);
        if (first_ok.type() == Json::Type::kNull) {
          first_ok = proto_json(res);
        }
      }
      if (ms > 20.0 && slow_count < kMaxSlowCalls) {
        ++slow_count;
        slow.push(
          Json::object()
            .set("i", calls - 1)
            .set("t_s", ms_between(start, a) / 1e3)
            .set("ms", ms)
            .set("status", code));
      }
      if (opt_.raw) {
        raw.add({Cell::d(ms_between(start, a)), Cell::d(ms), Cell::i(st.error_code())});
      }
      if (timeout) {
        ++timeouts;
        if (++consecutive_deadline >= kMaxConsecutiveDeadlines) {
          stopped_early = true;
          break;
        }
      } else {
        consecutive_deadline = 0;
        if (!st.ok()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(1));  // no hot error loop
        }
      }
    }
    const double span_s = ms_between(start, Clock::now()) / 1e3;

    const Summary s = summarize(ok_ms);
    const Summary all = summarize(all_ms);
    rtt_ = s;
    rtt_all_ = all;
    rtt_timeouts_ = timeouts;
    rtt_span_s_ = span_s;
    const size_t over20 = count_over(all_ms, 20.0);
    const size_t over45 = count_over(all_ms, 45.0);
    rtt_over45_ = over45;

    Json j = Json::object()
               .set("rpc", "Actor.GetActorState")
               .set("rtt_seconds", opt_.rtt_seconds)
               .set("max_calls", opt_.n)
               .set("calls", calls)
               .set("rate_hz", opt_.rate_hz)
               .set("span_s", span_s)
               .set("stopped_early", stopped_early)
               .set("codes", codes_json(codes))
               .set("ok_ms", to_json(s))
               .set("all_calls_ms", to_json(all))
               .set("timeouts", timeouts)
               .set("all_calls_max_is_timeout", timeouts > 0 && all.max >= deadline_ms)
               .set("over_20ms_all_calls", over20)
               .set("over_45ms_all_calls", over45)
               .set("over_20ms_per_min", per_min(over20, span_s))
               .set("over_45ms_per_min", per_min(over45, span_s))
               .set("slow_calls", std::move(slow))
               .set("first_ok_response", std::move(first_ok));
    if (opt_.raw) {
      j.set("raw", raw.to_json());
    }
    std::printf(
      "[rtt] GetActorState %s over %.2f s: %zu calls, %s%s\n",
      pace_text(opt_.rate_hz, "back-to-back").c_str(), span_s, calls, codes_text(codes).c_str(),
      stopped_early ? " (stopped after 10 consecutive DEADLINE_EXCEEDED)" : "");
    std::printf(
      "[rtt] ok: p50=%.3f p90=%.3f p99=%.3f max=%.3f mean=%.3f ms | all calls: p99=%.3f max=%.3f%s "
      "ms, timeouts=%zu | >20ms=%zu (%.2f/min) >45ms=%zu (%.2f/min)\n",
      s.p50, s.p90, s.p99, s.max, s.mean, all.p99, all.max,
      timeouts > 0 && all.max >= deadline_ms ? " (timeout)" : "", timeouts, over20,
      per_min(over20, span_s), over45, per_min(over45, span_s));
    warn_if_dropped("rtt", raw);
    return j;
  }

  // ---------------------------------------------------------------------------
  Json run_frames()
  {
    morai_sim_api::actor::GetAllActorsFilter filter;  // same filter as main.py
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);
    filter.set_pedestrian(true);
    filter.set_obstacle(true);

    struct Key
    {
      int64_t ts;
      uint64_t tick;
      size_t n_states;
      bool ego;
      bool vs;
      bool operator==(const Key & o) const
      {
        return ts == o.ts && tick == o.tick && n_states == o.n_states && ego == o.ego && vs == o.vs;
      }
    };

    std::map<std::string, size_t> codes;
    std::vector<TimedSample> samples;
    std::vector<double> call_ms;
    RawTable raw(
      {"t_ms", "grpc_code", "timestamp", "tick", "n_states", "ego_found",
       "ego_vehicle_state_non_default", "call_ms", "repeats_before", "repeat_first_t_ms",
       "repeat_last_t_ms"},
      opt_.max_raw_rows);
    std::optional<Key> last_key;
    RepeatRun repeats;
    Json first_names = Json::array();
    Json ego_example;
    size_t ego_exact = 0;
    size_t ego_ci = 0;
    size_t ego_has_vs = 0;
    size_t ego_vs_nondefault = 0;
    size_t min_states = std::numeric_limits<size_t>::max();
    size_t max_states = 0;
    std::string ego_type;
    size_t consecutive_deadline = 0;
    bool stopped_early = false;

    const auto start = Clock::now();
    const auto end = after(start, opt_.seconds);
    Pacer pacer(opt_.poll_hz);
    while (Clock::now() < end) {
      pacer.wait();
      grpc::ClientContext ctx;
      set_deadline(&ctx);
      morai_sim_api::actor::ActorStates res;
      const auto a = Clock::now();
      const grpc::Status st = actor_stub_->GetAllActorsState(&ctx, filter, &res);
      const auto b = Clock::now();
      ++codes[grpc_code_name(st.error_code())];
      const double ms = ms_between(a, b);
      const double mid_s = (ms_between(start, a) + ms / 2.0) / 1e3;
      if (!st.ok()) {
        if (opt_.raw) {  // a gap in the rows must be attributable to a failed call
          raw.add(
            {Cell::d(mid_s * 1e3), Cell::i(st.error_code()), Cell::null(), Cell::null(),
             Cell::null(), Cell::null(), Cell::null(), Cell::d(ms), Cell::null(), Cell::null(),
             Cell::null()});
        }
        if (st.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
          if (++consecutive_deadline >= kMaxConsecutiveDeadlines) {
            stopped_early = true;
            break;
          }
        } else {
          consecutive_deadline = 0;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        continue;
      }
      consecutive_deadline = 0;
      call_ms.push_back(ms);

      const morai_sim_api::actor::ActorState * ego = nullptr;
      bool exact = false;
      for (const auto & s : res.states()) {
        if (s.actor_info().id().value() == opt_.ego) {
          ego = &s;
          exact = true;
          break;
        }
      }
      if (!ego) {
        for (const auto & s : res.states()) {
          if (iequals(s.actor_info().id().value(), opt_.ego)) {
            ego = &s;
            break;
          }
        }
      }
      const bool has_vs = ego && ego->has_vehicle_state();
      const bool vs_nondefault = has_vs && ego->vehicle_state().ByteSizeLong() > 0;
      if (ego) {
        ++(exact ? ego_exact : ego_ci);
        ego_has_vs += has_vs ? 1 : 0;
        ego_vs_nondefault += vs_nondefault ? 1 : 0;
        if (ego_example.type() == Json::Type::kNull) {
          ego_example = proto_json(*ego);
          ego_type =
            enum_name(morai_sim_api::ObjectType_descriptor(), ego->actor_info().object_type());
        }
      }
      if (samples.empty()) {  // first successful snapshot: which ids does MORAI use?
        int listed = 0;
        for (const auto & s : res.states()) {
          if (listed++ >= 50) {
            break;  // keep the list short on crowded scenarios
          }
          first_names.push(s.actor_info().id().value());
        }
      }
      const auto n_states = static_cast<size_t>(res.states_size());
      min_states = std::min(min_states, n_states);
      max_states = std::max(max_states, n_states);
      samples.push_back(TimedSample{mid_s, res.timestamp(), res.sync_mode_tick_count()});
      if (opt_.raw) {
        const Key key{
          res.timestamp(), res.sync_mode_tick_count(), n_states, ego != nullptr, vs_nondefault};
        if (last_key && *last_key == key) {
          repeats.add(mid_s * 1e3);
        } else {
          raw.add(
            {Cell::d(mid_s * 1e3), Cell::i(0), Cell::i(res.timestamp()),
             Cell::u(res.sync_mode_tick_count()), Cell::u(n_states), Cell::i(ego ? 1 : 0),
             Cell::i(vs_nondefault ? 1 : 0), Cell::d(ms), repeats.count(), repeats.first(),
             repeats.last()});
          repeats.clear();
          last_key = key;
        }
      }
    }

    const FrameAnalysis fa = analyze_frames(samples);
    frames_ = fa;
    frames_done_ = true;
    const size_t ok = samples.size();
    // Only a verified period may steer another verdict (WaitForTick). Wall period, since
    // WaitForTick returns are wall-clock events.
    frame_period_ms_ = fa.frame_period_reliable ? fa.wall_period_ms : kNaN;
    ego_in_all_ = ok > 0 && (ego_exact + ego_ci) == ok;
    ego_vs_in_all_ = ok > 0 && ego_vs_nondefault == ok;
    const std::string time_base =
      samples.empty() ? "unknown"
                      : classify_time_base(samples.back().value, fa.unit.per_second, unix_now_s());
    frames_time_base_ = time_base;

    Json j = Json::object()
               .set("rpc", "Actor.GetAllActorsState")
               .set("filter", proto_json(filter))
               .set("seconds", opt_.seconds)
               .set("poll_hz", opt_.poll_hz)
               .set("stopped_early", stopped_early)
               .set("codes", codes_json(codes))
               .set("call_ms", to_json(summarize(call_ms)))
               .set("states_min", ok ? min_states : 0)
               .set("states_max", max_states)
               .set(
                 "ego", Json::object()
                          .set("name", opt_.ego)
                          .set("found_exact", ego_exact)
                          .set("found_case_insensitive_only", ego_ci)
                          .set("has_vehicle_state", ego_has_vs)
                          .set("vehicle_state_non_default", ego_vs_nondefault)
                          .set("object_type", ego_type)
                          .set("example", std::move(ego_example)))
               .set("first_response_actor_ids", std::move(first_names))
               .set("first_timestamp_raw", samples.empty() ? Json() : Json(samples.front().value))
               .set("last_timestamp_raw", samples.empty() ? Json() : Json(samples.back().value))
               .set("time_base", time_base)
               .set("analysis", to_json(fa));
    if (opt_.raw) {
      j.set(
         "raw_mode",
         "on_change: one row per change of (timestamp, tick, n_states, ego) or "
         "failed call; repeats_before = identical polls since the previous row")
        .set("raw", raw.to_json())
        .set("trailing_repeats", repeats.to_json());
    }

    const Summary cs = summarize(call_ms);
    std::printf(
      "[frames] GetAllActorsState %s %.1f s: %s%s | call p50=%.3f max=%.3f ms | states "
      "%zu..%zu\n",
      pace_text(opt_.poll_hz, "tight loop").c_str(), opt_.seconds, codes_text(codes).c_str(),
      stopped_early ? " (stopped after 10 consecutive DEADLINE_EXCEEDED)" : "", cs.p50, cs.max,
      ok ? min_states : 0, max_states);
    std::printf(
      "[frames] ego '%s': found %zu/%zu (exact %zu), vehicle_state present %zu, non-default %zu, "
      "type %s\n",
      opt_.ego.c_str(), ego_exact + ego_ci, ok, ego_exact, ego_has_vs, ego_vs_nondefault,
      ego_type.empty() ? "-" : ego_type.c_str());
    print_frame_analysis("frames", fa);
    std::printf(
      "[frames] time base: %s (first=%" PRId64 " last=%" PRId64 " raw)\n", time_base.c_str(),
      samples.empty() ? int64_t{0} : samples.front().value,
      samples.empty() ? int64_t{0} : samples.back().value);
    warn_if_dropped("frames", raw);
    return j;
  }

  // ---------------------------------------------------------------------------
  Json run_simtime()
  {
    std::map<std::string, size_t> codes;
    std::map<std::string, size_t> actor_codes;
    std::vector<TimedSample> samples;
    std::vector<double> call_ms;
    std::vector<double> offsets;
    std::vector<std::pair<int64_t, int64_t>> pairs;  // (sim, actor)
    RawTable raw(
      {"t_ms", "grpc_code", "sim_timestamp", "call_ms", "actor_grpc_code", "actor_timestamp",
       "repeats_before", "repeat_first_t_ms", "repeat_last_t_ms"},
      opt_.max_raw_rows);
    std::optional<int64_t> last_value;
    RepeatRun repeats;
    morai_sim_api::actor::GetAllActorsFilter filter;
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);
    size_t consecutive_deadline = 0;
    bool stopped_early = false;

    const auto start = Clock::now();
    const auto end = after(start, opt_.seconds);
    size_t iter = 0;
    Pacer pacer(opt_.poll_hz);
    while (Clock::now() < end) {
      pacer.wait();
      grpc::ClientContext ctx;
      set_deadline(&ctx);
      morai_sim_api::Empty req;
      morai_sim_api::Int64Value res;
      const auto a = Clock::now();
      const grpc::Status st = simulator_stub_->GetTimestamp(&ctx, req, &res);
      const auto b = Clock::now();
      ++codes[grpc_code_name(st.error_code())];
      const double ms = ms_between(a, b);
      const double mid_s = (ms_between(start, a) + ms / 2.0) / 1e3;
      if (!st.ok()) {
        if (opt_.raw) {
          raw.add(
            {Cell::d(mid_s * 1e3), Cell::i(st.error_code()), Cell::null(), Cell::d(ms),
             Cell::null(), Cell::null(), Cell::null(), Cell::null(), Cell::null()});
        }
        if (st.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
          if (++consecutive_deadline >= kMaxConsecutiveDeadlines) {
            stopped_early = true;
            break;
          }
        } else {
          consecutive_deadline = 0;
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        continue;
      }
      consecutive_deadline = 0;
      call_ms.push_back(ms);
      samples.push_back(TimedSample{mid_s, res.value(), 0});

      int64_t actor_ts = 0;
      bool actor_tried = false;
      bool have_actor = false;
      int actor_code = 0;
      // Every 10th sample, read ActorStates.timestamp right after, to compare clocks.
      if (iter++ % 10 == 0) {
        grpc::ClientContext actx;
        set_deadline(&actx);
        morai_sim_api::actor::ActorStates ares;
        const grpc::Status ast = actor_stub_->GetAllActorsState(&actx, filter, &ares);
        ++actor_codes[grpc_code_name(ast.error_code())];
        actor_tried = true;
        actor_code = ast.error_code();
        if (ast.ok()) {
          have_actor = true;
          actor_ts = ares.timestamp();
          pairs.emplace_back(res.value(), ares.timestamp());
          // Doubles, not int64: the two clocks may have unrelated origins.
          offsets.push_back(
            static_cast<double>(ares.timestamp()) - static_cast<double>(res.value()));
        }
      }
      if (opt_.raw) {
        if (!actor_tried && last_value && *last_value == res.value()) {
          repeats.add(mid_s * 1e3);
        } else {
          raw.add(
            {Cell::d(mid_s * 1e3), Cell::i(0), Cell::i(res.value()), Cell::d(ms),
             actor_tried ? Cell::i(actor_code) : Cell::null(),
             have_actor ? Cell::i(actor_ts) : Cell::null(), repeats.count(), repeats.first(),
             repeats.last()});
          repeats.clear();
          last_value = res.value();
        }
      }
    }

    const FrameAnalysis fa = analyze_frames(samples);
    simtime_ = fa;
    simtime_done_ = true;
    const std::string time_base =
      samples.empty() ? "unknown"
                      : classify_time_base(samples.back().value, fa.unit.per_second, unix_now_s());
    double actor_over_sim = kNaN;
    if (pairs.size() >= 2 && pairs.back().first != pairs.front().first) {
      actor_over_sim =
        (static_cast<double>(pairs.back().second) - static_cast<double>(pairs.front().second)) /
        (static_cast<double>(pairs.back().first) - static_cast<double>(pairs.front().first));
    }
    simtime_actor_ratio_ = actor_over_sim;
    const Summary off = summarize(offsets);

    Json j = Json::object()
               .set("rpc", "Simulator.GetTimestamp")
               .set("seconds", opt_.seconds)
               .set("poll_hz", opt_.poll_hz)
               .set("stopped_early", stopped_early)
               .set("codes", codes_json(codes))
               .set("call_ms", to_json(summarize(call_ms)))
               .set("time_base", time_base)
               .set("analysis", to_json(fa))
               .set(
                 "vs_actor_states_timestamp", Json::object()
                                                .set("codes", codes_json(actor_codes))
                                                .set("pairs", pairs.size())
                                                .set("offset_actor_minus_sim_raw", to_json(off))
                                                .set("rate_actor_over_sim", actor_over_sim));
    if (opt_.raw) {
      j.set(
         "raw_mode",
         "on_change: one row per sim_timestamp change, actor pair or failed "
         "call; repeats_before = identical polls since the previous row")
        .set("raw", raw.to_json())
        .set("trailing_repeats", repeats.to_json());
    }
    std::printf(
      "[simtime] GetTimestamp %s %.1f s: %s%s, call p50=%.3f ms\n",
      pace_text(opt_.poll_hz, "tight loop").c_str(), opt_.seconds, codes_text(codes).c_str(),
      stopped_early ? " (stopped after 10 consecutive DEADLINE_EXCEEDED)" : "",
      summarize(call_ms).p50);
    print_frame_analysis("simtime", fa);
    std::printf(
      "[simtime] time base: %s | vs ActorStates.timestamp (%zu pairs): offset actor-sim "
      "p50=%.6g min=%.6g max=%.6g raw, rate actor/sim=%.6g\n",
      time_base.c_str(), pairs.size(), off.p50, off.min, off.max, actor_over_sim);
    warn_if_dropped("simtime", raw);
    return j;
  }

  // ---------------------------------------------------------------------------
  Json run_waitfortick()
  {
    // Baseline unary RTT when `rtt` was not run: cheap GetTimestamp calls.
    double rtt_ref = rtt_.n ? rtt_.p50 : kNaN;
    std::string rtt_ref_src = rtt_.n ? "rtt.GetActorState" : "";
    if (!rtt_.n) {
      std::vector<double> base;
      for (int i = 0; i < 20; ++i) {
        grpc::ClientContext ctx;
        set_deadline(&ctx);
        morai_sim_api::Empty req;
        morai_sim_api::Int64Value res;
        const auto a = Clock::now();
        if (simulator_stub_->GetTimestamp(&ctx, req, &res).ok()) {
          base.push_back(ms_between(a, Clock::now()));
        }
      }
      rtt_ref = summarize(base).p50;
      rtt_ref_src = "GetTimestamp x20";
    }

    std::map<std::string, size_t> codes;
    std::vector<double> call_ms_ok;
    std::vector<double> intervals;
    std::vector<TimedSample> samples;
    RawTable raw({"t_return_ms", "call_ms", "grpc_code", "value"}, opt_.max_raw_rows);
    size_t calls = 0;
    size_t ok = 0;
    size_t deadline_exceeded = 0;
    size_t consecutive_deadline = 0;
    bool stopped_early = false;
    bool prev_ok = false;
    Clock::time_point prev_return{};

    const auto start = Clock::now();
    const auto end = after(start, opt_.seconds);
    while (calls < opt_.wft_n && Clock::now() < end) {
      grpc::ClientContext ctx;
      set_deadline(&ctx);  // never hang: WaitForTick may block until the next tick forever
      morai_sim_api::Empty req;
      morai_sim_api::Int64Value res;
      const auto a = Clock::now();
      const grpc::Status st = simulation_stub_->WaitForTick(&ctx, req, &res);
      const auto b = Clock::now();
      ++calls;
      const double ms = ms_between(a, b);
      ++codes[grpc_code_name(st.error_code())];
      if (opt_.raw) {
        raw.add(
          {Cell::d(ms_between(start, b)), Cell::d(ms), Cell::i(st.error_code()),
           st.ok() ? Cell::i(res.value()) : Cell::null()});
      }
      if (st.ok()) {
        ++ok;
        consecutive_deadline = 0;
        call_ms_ok.push_back(ms);
        if (prev_ok) {
          intervals.push_back(ms_between(prev_return, b));
        }
        samples.push_back(TimedSample{ms_between(start, b) / 1e3, res.value(), 0});
        prev_ok = true;
        prev_return = b;
        continue;
      }
      prev_ok = false;
      if (st.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
        ++deadline_exceeded;
        if (++consecutive_deadline >= kMaxConsecutiveDeadlines) {
          stopped_early = true;
          break;
        }
      } else {
        consecutive_deadline = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }

    const FrameAnalysis fa = analyze_frames(samples);
    const Summary cs = summarize(call_ms_ok);
    const Summary is = summarize(intervals);
    double frame_ref = frame_period_ms_;
    std::string frame_ref_src = std::isfinite(frame_ref) ? "frames (verified wall period)" : "";
    if (!std::isfinite(frame_ref) && fa.dvalue_raw.n > 0) {
      frame_ref = fa.dvalue_raw.p50;  // proto: "response : timestamp (msec)"
      frame_ref_src = "WaitForTick value delta, assuming msec";
    }

    WaitForTickObservation obs;
    obs.calls = calls;
    obs.ok = ok;
    obs.deadline_exceeded = deadline_exceeded;
    obs.deadline_ms = opt_.deadline_ms;
    obs.call_ms_p50 = cs.p50;
    obs.interval_ms_p50 = is.p50;
    obs.rtt_ms_p50 = rtt_ref;
    obs.frame_period_ms = frame_ref;
    const std::string verdict = classify_wait_for_tick(obs);
    wft_verdict_ = verdict;

    Json j = Json::object()
               .set("rpc", "Simulation.WaitForTick")
               .set("deadline_ms", opt_.deadline_ms)
               .set("calls", calls)
               .set("stopped_early_after_10_deadlines", stopped_early)
               .set("codes", codes_json(codes))
               .set("ok_call_ms", to_json(cs))
               .set("interval_between_ok_returns_ms", to_json(is))
               .set("value_analysis", to_json(fa))
               .set("rtt_reference_ms", rtt_ref)
               .set("rtt_reference_source", rtt_ref_src)
               .set("frame_period_reference_ms", frame_ref)
               .set("frame_period_reference_source", frame_ref_src)
               .set("verdict", verdict);
    if (opt_.raw) {
      j.set("raw", raw.to_json());
    }
    std::printf(
      "[waitfortick] %zu calls (deadline %d ms)%s: %s\n", calls, opt_.deadline_ms,
      stopped_early ? ", stopped after 10 consecutive deadlines" : "", codes_text(codes).c_str());
    std::printf(
      "[waitfortick] ok call p50=%.3f p90=%.3f max=%.3f ms | interval between returns p50=%.3f "
      "p90=%.3f max=%.3f ms\n",
      cs.p50, cs.p90, cs.max, is.p50, is.p90, is.max);
    if (ok > 0 && deadline_exceeded > 0) {
      std::printf(
        "[waitfortick] note: both OK and DEADLINE_EXCEEDED — --deadline-ms %d is probably "
        "shorter than one tick wait; rerun with a larger deadline\n",
        opt_.deadline_ms);
    }
    std::printf(
      "[waitfortick] value: d p50=%.6g min=%.6g max=%.6g, repeats=%zu, unit=%s | ref rtt=%.3f ms "
      "(%s), ref frame=%.4g ms (%s) -> verdict=%s\n",
      fa.dvalue_raw.p50, fa.dvalue_raw.min, fa.dvalue_raw.max, fa.duplicates, fa.unit.name.c_str(),
      rtt_ref, rtt_ref_src.c_str(), frame_ref, frame_ref_src.empty() ? "-" : frame_ref_src.c_str(),
      verdict.c_str());
    warn_if_dropped("waitfortick", raw);
    return j;
  }

  // ---------------------------------------------------------------------------
  // 26.09.25 _ YSH _ 신규 — monitor: 다른 무엇(C++ 노드·main.py·키보드)이 모는 동안 MORAI 가 실제로
  //   적용한 값을 서로 다른 업데이트마다 기록하고 진동·0 떨어짐·모드 뒤집힘을 센다. 읽기 전용이다 —
  //   GetAllActorsState(매 폴링) · GetVehicleControlMode(0.5 s) · WaitForTick(--trigger wft).
  //   시계는 GetTimestamp 를 쓰지 않는다: 그것도 프레임 시각이라 스냅샷 timestamp 와 같은 정보다.
  //   서로 다른 업데이트의 첫 응답이 가장 빠듯하므로 창마다 max(ts - recv) 를 오프셋 하한으로 낸다.
  Json run_monitor()
  {
    using morai_grpc_vendor::probe::looks_oscillating;
    using morai_grpc_vendor::probe::MonitorEvent;
    using morai_grpc_vendor::probe::MonitorSample;
    using morai_grpc_vendor::probe::MonitorStats;
    using morai_grpc_vendor::probe::MonitorThresholds;
    using morai_grpc_vendor::probe::MonitorWindow;
    constexpr double kModePeriodS = 0.5;
    constexpr double kSummaryPeriodS = 10.0;
    constexpr double kKmhToMps = 1.0 / 3.6;  // main.py KMPH_TO_MPS

    morai_sim_api::actor::GetAllActorsFilter filter;  // main.py 와 같은 필터
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);
    filter.set_pedestrian(true);
    filter.set_obstacle(true);

    const MonitorThresholds th{};
    MonitorStats stats(th, 0.0);
    const bool wft = opt_.trigger == "wft";
    const bool forever = !(opt_.seconds > 0.0);

    Json j = Json::object()
               .set("rpc", wft ? "Simulation.WaitForTick -> Actor.GetAllActorsState"
                               : "Actor.GetAllActorsState (polled)")
               .set("trigger", opt_.trigger)
               .set("poll_hz", wft ? Json() : Json(opt_.poll_hz))
               .set("seconds", opt_.seconds)
               .set("until_ctrl_c", forever)
               .set(
                 "thresholds", Json::object()
                                 .set("gap_ms", th.gap_ms)
                                 .set("throttle_on", th.throttle_on)
                                 .set("throttle_off", th.throttle_off)
                                 .set("brake_on", th.brake_on)
                                 .set("idle_throttle_max", th.idle_throttle_max)
                                 .set("idle_brake_center", th.idle_brake_center)
                                 .set("idle_brake_tol", th.idle_brake_tol)
                                 .set("steer_active", th.steer_active)
                                 .set("steer_zero", th.steer_zero)
                                 .set("max_drop_run", th.max_drop_run)
                                 .set("oscillation_toggles_per_s", th.oscillation_toggles_per_s));

    static const char * kColumns[] = {
      "t_s",      "recv_unix_s", "morai_ts_us", "dts_ms",   "recv_minus_ts_ms", "rpc_ms",
      "polls",    "ego_found",   "ego_valid",   "speed_mps", "accel_x_raw",     "throttle",
      "brake",    "steer",       "front_wheel_angle", "gear", "control_mode",   "collisions",
      "n_actors", "x",           "y",           "yaw_deg"};
    std::FILE * csv = nullptr;
    if (!opt_.csv_path.empty()) {
      csv = std::fopen(opt_.csv_path.c_str(), "w");
      if (!csv) {
        std::fprintf(stderr, "[monitor] cannot open --csv %s\n", opt_.csv_path.c_str());
        io_error_ = true;
        return j.set("error", "cannot open csv " + opt_.csv_path);
      }
      for (size_t c = 0; c < sizeof(kColumns) / sizeof(kColumns[0]); ++c) {
        std::fprintf(csv, "%s%s", c ? "," : "", kColumns[c]);
      }
      std::fprintf(csv, "\n");
    }
    RawTable raw(
      std::vector<std::string>(std::begin(kColumns), std::end(kColumns)), opt_.max_raw_rows);

    std::printf(
      "[monitor] %s, %s, control mode every %.1f s, summary every %.0f s%s%s — read-only\n",
      wft ? "WaitForTick-triggered" : pace_text(opt_.poll_hz, "tight loop").c_str(),
      forever ? "until Ctrl-C" : seconds_text(opt_.seconds).c_str(), kModePeriodS,
      kSummaryPeriodS, csv ? ", csv " : "", csv ? opt_.csv_path.c_str() : "");
    if (wft) {
      // 엔진도 WaitForTick 을 트리거로 쓴다. 대기자가 둘일 때 MORAI 가 각자에게 매 업데이트를 주는지는
      // 아직 실측 전이다 — 엔진이 도는 동안에는 기본 --trigger poll 을 쓴다
      std::printf(
        "[monitor] WARNING: --trigger wft adds a second WaitForTick waiter next to the engine's; "
        "whether MORAI serves both on every update is unmeasured. Use the default --trigger poll "
        "while the engine is running.\n");
    }

    g_monitor_stop = 0;
    const auto old_int = std::signal(SIGINT, on_monitor_signal);
    const auto old_term = std::signal(SIGTERM, on_monitor_signal);
    const auto old_hup = std::signal(SIGHUP, on_monitor_signal);

    const auto start = Clock::now();
    const auto end = forever ? Clock::time_point::max() : after(start, opt_.seconds);
    Pacer pacer(wft ? 0.0 : opt_.poll_hz);
    auto next_mode = start;
    auto next_summary = after(start, kSummaryPeriodS);
    morai_grpc_vendor::Snapshot snap;
    std::optional<int64_t> last_ts;
    uint64_t polls_since = 0;
    int current_mode = -1;
    size_t consecutive_fail = 0;
    size_t fail_streaks_reported = 0;
    size_t csv_rows = 0;
    size_t wft_calls = 0;
    size_t wft_failures = 0;
    size_t wft_stale = 0;
    std::map<std::string, size_t> codes;
    Json windows = Json::array();
    Json events = Json::array();
    std::vector<std::string> last_collisions;
    size_t osc_windows = 0;

    auto t_of = [&](Clock::time_point t) { return ms_between(start, t) / 1e3; };
    auto print_events = [&]() {
      for (const MonitorEvent & e : stats.drain_new_events()) {
        events.push(to_json(e));
        std::string what;
        switch (e.kind) {
          case MonitorEvent::Kind::kModeChange:
            what = "control mode " +
                   morai_grpc_vendor::to_string(static_cast<ControlMode>(e.from)) + "(" +
                   std::to_string(e.from) + ") -> " +
                   morai_grpc_vendor::to_string(static_cast<ControlMode>(e.to)) + "(" +
                   std::to_string(e.to) + ")";
            break;
          case MonitorEvent::Kind::kGearChange:
            what = "gear " + morai_grpc_vendor::to_string(static_cast<GearMode>(e.from)) + " -> " +
                   morai_grpc_vendor::to_string(static_cast<GearMode>(e.to));
            break;
          case MonitorEvent::Kind::kCollision: {
            what = "COLLISION (" + std::to_string(e.to) + " object(s)):";
            size_t k = 0;
            for (const auto & name : last_collisions) {
              if (k++ >= 5) {
                what += " ...";
                break;
              }
              what += " '" + name + "'";
            }
            break;
          }
          case MonitorEvent::Kind::kTimestampBackwards:
            what = "MORAI timestamp went BACKWARDS " + std::to_string(e.from) + " -> " +
                   std::to_string(e.to) + " us (MORAI restarted / scenario reset?)";
            break;
          case MonitorEvent::Kind::kEgoLost:
            what = "Ego '" + opt_.ego + "' missing from GetAllActorsState";
            break;
          case MonitorEvent::Kind::kEgoBack:
            what = "Ego '" + opt_.ego + "' back";
            break;
        }
        std::printf("[monitor] t=%8.3f s  %s\n", e.t_s, what.c_str());
      }
    };
    auto print_window = [&](const MonitorWindow & w, const char * tag) {
      char clk[48] = "-";
      if (std::isfinite(w.clock_bound_ms)) {
        std::snprintf(clk, sizeof(clk), "%+.2f ms", w.clock_bound_ms);
      }
      const std::string mode =
        w.last_mode < 0 ? "?"
                        : morai_grpc_vendor::to_string(static_cast<ControlMode>(w.last_mode)) +
                            "(" + std::to_string(w.last_mode) + ")";
      const std::string gear =
        w.last_gear < 0 ? "?" : morai_grpc_vendor::to_string(static_cast<GearMode>(w.last_gear));
      std::printf(
        "[%s] t=%7.1f s | %5.1f upd/s (polls %zu, fail %zu) | gaps>%.0fms ts %zu wall %zu (max "
        "%.0f/%.0f ms) | thr 0<->+ %zu | brake on %zu (%.0f%%) | idle %zu | 0-drop steer %zu fwa "
        "%zu | mode %s chg %zu | gear %s chg %zu | coll %zu | v %.1f..%.1f m/s | recv-ts min %.2f "
        "p50 %.1f max %.1f ms | clk>= %s%s\n",
        tag, w.t_end_s, w.updates_per_s, w.polls, w.rpc_failures, th.gap_ms, w.ts_gaps,
        w.wall_gaps, w.max_ts_gap_ms, w.max_wall_gap_ms, w.throttle_toggles, w.brake_engagements,
        w.brake_on_share() * 100.0, w.idle_records, w.steer_zero_drops, w.fwa_zero_drops,
        mode.c_str(), w.mode_changes, gear.c_str(), w.gear_changes, w.collision_events,
        w.speed_min_mps, w.speed_max_mps, w.recv_minus_ts_ms.min, w.recv_minus_ts_ms.p50,
        w.recv_minus_ts_ms.max, clk,
        looks_oscillating(w, th) ? " | OSCILLATION?" : "");
      std::fflush(stdout);
    };

    while (!g_monitor_stop && Clock::now() < end) {
      // 제어 모드(0.5 s)를 트리거 대기 전에 — 첫 행부터 모드가 채워진다
      const auto tick_now = Clock::now();
      if (tick_now >= next_mode) {
        grpc::ClientContext mctx;
        set_deadline(&mctx);
        morai_sim_api::actor::CurVehicleControlMode mres;
        const grpc::Status mst = actor_stub_->GetVehicleControlMode(&mctx, ego_, &mres);
        if (mst.ok()) {
          current_mode = static_cast<int>(mres.mode());
        }
        stats.add_mode(t_of(Clock::now()), mst.ok(), static_cast<int>(mres.mode()));
        next_mode = after(next_mode, kModePeriodS);
        if (next_mode < tick_now) {
          next_mode = after(tick_now, kModePeriodS);
        }
      }
      if (wft) {
        grpc::ClientContext wctx;
        set_deadline(&wctx);  // 한 업데이트(~20 ms)보다 긴 --deadline-ms (기본 200)
        morai_sim_api::Empty wreq;
        morai_sim_api::Int64Value wres;
        ++wft_calls;
        const grpc::Status wst = simulation_stub_->WaitForTick(&wctx, wreq, &wres);
        if (!wst.ok()) {
          ++wft_failures;
          if (wst.error_code() != grpc::StatusCode::DEADLINE_EXCEEDED) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
        }
      } else {
        pacer.wait();
      }

      grpc::ClientContext ctx;
      set_deadline(&ctx);
      morai_sim_api::actor::ActorStates res;
      const auto a = Clock::now();
      const grpc::Status st = actor_stub_->GetAllActorsState(&ctx, filter, &res);
      const auto b = Clock::now();
      const int64_t recv_sys_ns = unix_now_ns();
      ++codes[grpc_code_name(st.error_code())];
      stats.add_poll(st.ok());
      if (!st.ok()) {
        if (++consecutive_fail % 50 == 0 && fail_streaks_reported < 20) {
          ++fail_streaks_reported;
          std::printf(
            "[monitor] t=%8.3f s  %zu consecutive GetAllActorsState failures (%s) - still "
            "watching\n",
            t_of(b), consecutive_fail, grpc_code_name(st.error_code()).c_str());
        }
        // 멈춘 MORAI 를 두드리지 않는다: 연속 실패가 쌓이면 100 ms, 아니면 1 ms 쉰다
        std::this_thread::sleep_for(std::chrono::milliseconds(consecutive_fail >= 10 ? 100 : 1));
      } else {
        consecutive_fail = 0;
        ++polls_since;
        if (last_ts && *last_ts == res.timestamp()) {
          wft_stale += wft ? 1 : 0;  // WaitForTick 이 돌아왔는데 스냅샷은 그대로
        } else {
          morai_grpc_vendor::link_convert::to_snapshot(res, opt_.ego, &snap);
          const auto & ego = snap.ego;
          const auto & vs = ego.vehicle_state;
          MonitorSample s;
          s.t_s = t_of(b);
          s.timestamp_us = snap.timestamp_us;
          s.recv_minus_ts_ms =
            static_cast<double>(recv_sys_ns - snap.timestamp_us * 1000) / 1e6;
          s.ego_found = ego.found;
          s.ego_valid = ego.valid;
          s.speed_mps = ego.velocity_kmh.x * kKmhToMps;
          s.throttle = vs.throttle;
          s.brake = vs.brake;
          s.steer = vs.steer;
          s.front_wheel_angle = vs.front_wheel_angle;
          s.gear = vs.present ? static_cast<int>(vs.gear_mode) : -1;
          s.collisions = vs.collision_count;
          if (vs.collision_count > 0) {
            last_collisions = vs.collision_objects;
          }
          const double dts_ms =
            last_ts ? static_cast<double>(snap.timestamp_us - *last_ts) / 1e3 : kNaN;
          stats.add_update(s);

          const double recv_unix_s = static_cast<double>(recv_sys_ns) / 1e9;
          const double rpc = ms_between(a, b);
          if (csv) {
            std::fprintf(
              csv,
              "%.4f,%.6f,%" PRId64 ",%s,%.3f,%.3f,%" PRIu64
              ",%d,%d,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%d,%d,%zu,%zu,%.3f,%.3f,%.3f\n",
              s.t_s, recv_unix_s, snap.timestamp_us,
              std::isfinite(dts_ms) ? std::to_string(dts_ms).c_str() : "", s.recv_minus_ts_ms,
              rpc, polls_since, s.ego_found ? 1 : 0, s.ego_valid ? 1 : 0, s.speed_mps,
              ego.acceleration_raw.x, s.throttle, s.brake, s.steer, s.front_wheel_angle, s.gear,
              current_mode, s.collisions, snap.actors.size(), ego.position.x, ego.position.y,
              ego.rotation_deg.z);
            ++csv_rows;
          }
          if (opt_.raw) {
            raw.add(
              {Cell::d(s.t_s), Cell::d(recv_unix_s), Cell::i(snap.timestamp_us), Cell::d(dts_ms),
               Cell::d(s.recv_minus_ts_ms), Cell::d(rpc), Cell::u(polls_since),
               Cell::i(s.ego_found ? 1 : 0), Cell::i(s.ego_valid ? 1 : 0), Cell::d(s.speed_mps),
               Cell::d(ego.acceleration_raw.x), Cell::d(s.throttle), Cell::d(s.brake),
               Cell::d(s.steer), Cell::d(s.front_wheel_angle), Cell::i(s.gear),
               Cell::i(current_mode), Cell::u(s.collisions), Cell::u(snap.actors.size()),
               Cell::d(ego.position.x), Cell::d(ego.position.y), Cell::d(ego.rotation_deg.z)});
          }
          last_ts = snap.timestamp_us;
          polls_since = 0;
        }
      }

      const auto now = Clock::now();
      print_events();
      if (now >= next_summary) {
        const MonitorWindow w = stats.take_window(t_of(now));
        print_window(w, "monitor");
        osc_windows += looks_oscillating(w, th) ? 1 : 0;
        windows.push(to_json(w).set("oscillation_suspected", looks_oscillating(w, th)));
        if (csv) {
          std::fflush(csv);
        }
        next_summary = after(next_summary, kSummaryPeriodS);
        if (next_summary < now) {
          next_summary = after(now, kSummaryPeriodS);
        }
      }
    }
    const bool by_signal = g_monitor_stop != 0;
    std::signal(SIGINT, old_int);
    std::signal(SIGTERM, old_term);
    std::signal(SIGHUP, old_hup);

    const double t_end = t_of(Clock::now());
    print_events();
    const MonitorWindow last = stats.take_window(t_end);
    if (last.updates > 0 || last.rpc_failures > 0) {
      osc_windows += looks_oscillating(last, th) ? 1 : 0;
      print_window(last, "monitor");
      windows.push(to_json(last)
                     .set("oscillation_suspected", looks_oscillating(last, th))
                     .set("partial", true));
    }
    const MonitorWindow total = stats.total(t_end);
    std::printf("\n==== monitor total (%s) ====\n", by_signal ? "stopped by signal" : "window over");
    print_window(total, "total");
    std::printf(
      "[total] %zu events, oscillation-suspected windows %zu, longest 0-drop run %zu upd, "
      "codes %s%s\n",
      stats.events().size(), osc_windows, total.longest_drop_run, codes_text(codes).c_str(),
      wft ? (", WaitForTick calls " + std::to_string(wft_calls) + " fail " +
             std::to_string(wft_failures) + " stale snapshot " + std::to_string(wft_stale))
              .c_str()
          : "");

    if (csv) {
      if (std::fclose(csv) != 0) {
        std::fprintf(stderr, "[monitor] error closing --csv %s\n", opt_.csv_path.c_str());
        io_error_ = true;
      } else {
        std::printf("[info] CSV (%zu rows) written to %s\n", csv_rows, opt_.csv_path.c_str());
      }
    }

    j.set("stopped_by_signal", by_signal)
      .set("span_s", t_end)
      .set("codes", codes_json(codes))
      .set("windows", std::move(windows))
      .set("total", to_json(total).set("oscillation_suspected", looks_oscillating(total, th)))
      .set("oscillation_suspected_windows", osc_windows)
      .set("events", std::move(events))
      .set(
        "clock",
        Json::object()
          .set("offset_lower_bound_ms", total.clock_bound_ms)
          .set("samples", total.recv_minus_ts_ms.n)
          .set(
            "method",
            "max(ActorStates.timestamp - local receive) over distinct updates = MORAI - local "
            "lower bound; MORAI has no call-time clock (GetTimestamp is the frame timestamp)")
          .set(
            "use",
            "same-PC value is the baseline (true offset 0); remote offset ~= remote value - "
            "baseline, measured the same way; cross-check chronyc tracking on both PCs"))
      .set("csv_path", opt_.csv_path.empty() ? Json() : Json(opt_.csv_path))
      .set("csv_rows", csv_rows);
    if (wft) {
      j.set(
        "wait_for_tick",
        Json::object().set("calls", wft_calls).set("failures", wft_failures).set("stale", wft_stale));
    }
    if (opt_.raw) {
      j.set("raw_mode", "one row per distinct ActorStates.timestamp; polls = calls since previous")
        .set("raw", raw.to_json());
      warn_if_dropped("monitor", raw);
    }
    return j;
  }

  // ---------------------------------------------------------------------------
  /// Compact cross-check of the open unknowns, after `all`.
  Json summary()
  {
    Json j = Json::object();
    std::printf("\n==== summary ====\n");
    if (frames_done_) {
      const FrameAnalysis & f = frames_;
      std::printf(
        "U1  sim step %.5g ms/frame, wall frame rate %.3f Hz -> %s "
        "(ActorStates.timestamp, %.1f s); sync.tick_period=%s\n",
        f.sim_step_s * 1e3, f.wall_frame_rate_hz,
        f.frame_period_reliable ? "reliable" : "UNRELIABLE, see warnings", f.wall_span_s,
        sync_tick_period_ms_ >= 0 ? (std::to_string(sync_tick_period_ms_) + " ms").c_str() : "-");
      if (f.deltas_are_multiples) {
        std::printf(
          "    physics-step candidate (delta gcd) %.5g ms (%.3f Hz); observed updates %.3f/s\n",
          f.gcd_step_s * 1e3, 1.0 / f.gcd_step_s, f.changes_per_wall_s);
      }
      std::printf(
        "    missed-frame events %zu (%.2f/min over %.1f s), max gap %.2f frames; absolute "
        ">45ms=%zu\n",
        f.missed_frame_events, f.missed_frame_events_per_min, f.wall_span_s, f.max_gap_in_frames,
        f.change_intervals_over_45ms);
      for (const auto & w : f.warnings) {
        std::printf("    WARNING: %s\n", w.c_str());
      }
      std::printf(
        "U4  ego in every GetAllActorsState: %s; vehicle_state non-default in every one: %s\n",
        ego_in_all_ ? "yes" : "NO", ego_vs_in_all_ ? "yes" : "NO");
      std::printf(
        "U17 ActorStates.timestamp unit=%s (rate %.4f, %s), base=%s, monotonic violations=%zu, "
        "resolution(min d)=%.6g raw; tick_count %s\n",
        f.unit.name.c_str(), f.unit.rate_ratio, f.unit.plausible ? "plausible" : "implausible",
        frames_time_base_.c_str(), f.monotonic_violations, f.dvalue_raw.min,
        f.tick_present ? "moves" : "always 0");
      Json warnings = Json::array();
      for (const auto & w : f.warnings) {
        warnings.push(w);
      }
      j.set("U1_reliable", f.frame_period_reliable)
        .set("U1_frame_period_ms", f.frame_period_s * 1e3)  // null unless reliable
        .set("U1_frame_rate_hz", f.frame_rate_hz)           // null unless reliable
        .set("U1_sim_step_ms", f.sim_step_s * 1e3)
        .set("U1_wall_frame_rate_hz", f.wall_frame_rate_hz)
        .set("U1_observed_updates_per_s", f.changes_per_wall_s)
        .set("U1_delta_mode_raw", f.dvalue_mode_raw)
        .set("U1_delta_gcd_raw", f.dvalue_gcd_raw)
        .set("U1_deltas_are_multiples", f.deltas_are_multiples)
        .set("U1_gcd_step_ms", f.gcd_step_s * 1e3)
        .set("U1_warnings", std::move(warnings))
        .set("U1_observation_s", f.wall_span_s)
        .set("missed_frame_events", f.missed_frame_events)
        .set("missed_frame_events_per_min", f.missed_frame_events_per_min)
        .set("wall_stall_events_per_min", f.wall_stall_events_per_min)
        .set("max_gap_in_frames", f.max_gap_in_frames)
        .set("U4_ego_in_every_snapshot", ego_in_all_)
        .set("U4_ego_vehicle_state_in_every_snapshot", ego_vs_in_all_)
        .set("U17_unit", f.unit.name)
        .set("U17_rate_ratio", f.unit.rate_ratio)
        .set("U17_time_base", frames_time_base_)
        .set("U17_monotonic_violations", f.monotonic_violations)
        .set("U17_tick_present", f.tick_present)
        .set("stalls_over_45ms_in_frames", f.change_intervals_over_45ms);
    }
    if (!sync_type_.empty()) {
      std::printf("U7  sync mode: %s, tick_period=%d\n", sync_type_.c_str(), sync_tick_period_ms_);
      j.set("U7_sync_type", sync_type_).set("U7_tick_period", sync_tick_period_ms_);
    }
    if (!wft_verdict_.empty()) {
      std::printf("Uw  WaitForTick: %s\n", wft_verdict_.c_str());
      j.set("Uw_wait_for_tick", wft_verdict_);
    }
    if (simtime_done_) {
      std::printf(
        "Ut  GetTimestamp unit=%s rate sim/wall=%.4f (%s); actor/sim rate=%.6g\n",
        simtime_.unit.name.c_str(), simtime_.unit.rate_ratio,
        simtime_.unit.plausible ? "plausible" : "implausible", simtime_actor_ratio_);
      j.set("Ut_unit", simtime_.unit.name)
        .set("Ut_rate_ratio", simtime_.unit.rate_ratio)
        .set("Ut_actor_over_sim_rate", simtime_actor_ratio_);
    }
    if (rtt_all_.n) {
      const bool max_is_timeout =
        rtt_timeouts_ > 0 && rtt_all_.max >= static_cast<double>(opt_.deadline_ms);
      std::printf(
        "U5  RTT ok p50=%.3f p90=%.3f p99=%.3f max=%.3f ms | all %zu calls p99=%.3f max=%.3f%s ms, "
        "timeouts=%zu | >45ms=%zu (%.2f/min over %.1f s)\n",
        rtt_.p50, rtt_.p90, rtt_.p99, rtt_.max, rtt_all_.n, rtt_all_.p99, rtt_all_.max,
        max_is_timeout ? " (timeout)" : "", rtt_timeouts_, rtt_over45_,
        per_min(rtt_over45_, rtt_span_s_), rtt_span_s_);
      j.set("U5_rtt_ms", to_json(rtt_))
        .set("U5_all_calls_ms", to_json(rtt_all_))
        .set("U5_timeouts", rtt_timeouts_)
        .set("U5_over_45ms", rtt_over45_)
        .set("U5_over_45ms_per_min", per_min(rtt_over45_, rtt_span_s_))
        .set("U5_window_s", rtt_span_s_);
    }
    if (std::isfinite(spec_max_steer_deg_) || !control_mode_.empty()) {
      std::printf(
        "U12 max_steering_angle=%.3f deg, control mode=%s\n", spec_max_steer_deg_,
        control_mode_.empty() ? "-" : control_mode_.c_str());
      j.set("U12_max_steering_angle_deg", spec_max_steer_deg_)
        .set("U12_control_mode", control_mode_);
    }
    return j;
  }

  // 26.09.25 _ YSH _ monitor 의 --csv 열기·닫기 실패 (main 이 종료 코드 3 으로 돌려준다)
  bool io_error() const { return io_error_; }

private:
  void set_deadline(grpc::ClientContext * ctx) const
  {
    ctx->set_deadline(
      std::chrono::system_clock::now() + std::chrono::milliseconds(opt_.deadline_ms));
  }

  const Options & opt_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<morai_sim_api::actor::Actor::Stub> actor_stub_;
  std::unique_ptr<morai_sim_api::simulation::Simulation::Stub> simulation_stub_;
  std::unique_ptr<morai_sim_api::simualtor::Simulator::Stub> simulator_stub_;
  morai_sim_api::ObjectInfo ego_;

  // Cross-subcommand results (for `all` and the summary).
  Summary rtt_{};
  Summary rtt_all_{};
  size_t rtt_timeouts_{0};
  double rtt_span_s_{0.0};
  size_t rtt_over45_{0};
  double frame_period_ms_{kNaN};
  FrameAnalysis frames_{};
  bool frames_done_{false};
  bool ego_in_all_{false};
  bool ego_vs_in_all_{false};
  std::string frames_time_base_;
  FrameAnalysis simtime_{};
  bool simtime_done_{false};
  double simtime_actor_ratio_{kNaN};
  int sync_tick_period_ms_{-1};
  std::string sync_type_;
  std::string wft_verdict_;
  double spec_max_steer_deg_{kNaN};
  std::string control_mode_;
  // 26.09.25 _ YSH _ monitor --csv 실패 표시
  bool io_error_{false};
};

bool write_json(const std::string & path, const Json & j)
{
  std::ofstream f(path);
  if (!f) {
    return false;
  }
  f << j.dump(2) << "\n";
  return static_cast<bool>(f);
}

// 26.09.25 _ YSH _ 출력 경로를 측정 전에 확인한다. monitor 는 Ctrl-C 까지 몇 시간이고 돌 수 있어 끝에서야
//   --json 실패를 알면 한 판을 통째로 잃는다. 없는 부모 디렉터리는 만들고, 쓰기로 열어 본 뒤 새로 만든
//   빈 파일은 지운다 (기존 파일은 내용을 건드리지 않는다)
bool prepare_output(const std::string & path, std::string * err)
{
  if (path.empty()) {
    return true;
  }
  std::error_code ec;
  const std::filesystem::path p(path);
  if (p.has_parent_path() && !std::filesystem::exists(p.parent_path(), ec)) {
    std::filesystem::create_directories(p.parent_path(), ec);
    if (ec) {
      *err = "cannot create directory " + p.parent_path().string() + ": " + ec.message();
      return false;
    }
  }
  const bool existed = std::filesystem::exists(p, ec);
  {
    std::ofstream f(path, std::ios::app);
    if (!f) {
      *err = "cannot open " + path + " for writing";
      return false;
    }
  }
  if (!existed) {
    std::filesystem::remove(p, ec);
  }
  return true;
}

}  // namespace

int main(int argc, char ** argv)
{
  Options opt;
  std::string err;
  const std::vector<std::string> args =
    argc > 1 ? std::vector<std::string>(argv + 1, argv + argc) : std::vector<std::string>();
  if (
    !morai_grpc_vendor::probe::apply_env_defaults(
      std::getenv("MORAI_SIM_ADDRESS"), std::getenv("MORAI_SIM_PORT"), &opt, &err) ||
    !morai_grpc_vendor::probe::parse_args(args, &opt, &err)) {
    std::fprintf(stderr, "morai_probe: %s\n\n", err.c_str());
    morai_grpc_vendor::probe::print_usage(stderr);
    return 1;
  }
  if (opt.help) {
    morai_grpc_vendor::probe::print_usage(stdout);
    return 0;
  }

  Json out = Json::object()
               .set("tool", "morai_probe")
               .set("schema", 2)
               .set("started_at_unix_s", unix_now_s())
               .set("command", opt.command)
               .set("host", opt.host)
               .set("port", opt.port)
               .set("ego", opt.ego)
               .set("client_key", opt.client_key)
               .set("deadline_ms", opt.deadline_ms)
               .set("seconds", opt.seconds)
               .set("rtt_seconds", opt.rtt_seconds)
               .set("rtt_max_calls", opt.n)
               .set("rtt_rate_hz", opt.rate_hz)
               .set("poll_hz", opt.poll_hz)
               // 26.09.25 _ YSH _ monitor 옵션 기록
               .set("trigger", opt.trigger)
               .set("csv", opt.csv_path)
               .set("max_raw_rows", opt.max_raw_rows);

  // 26.09.25 _ YSH _ --json·--csv 를 연결·측정 전에 확인 (실패하면 종료 코드 3, 측정하지 않는다)
  for (const std::string * path : {&opt.json_path, &opt.csv_path}) {
    std::string perr;
    if (!prepare_output(*path, &perr)) {
      std::fprintf(stderr, "morai_probe: %s\n", perr.c_str());
      return 3;
    }
  }

  std::printf(
    "morai_probe %s -> %s:%d ego='%s' deadline=%d ms (read-only)\n", opt.command.c_str(),
    opt.host.c_str(), opt.port, opt.ego.c_str(), opt.deadline_ms);

  Probe probe(opt);
  double connect_ms = 0.0;
  const bool connected = probe.connect(&connect_ms);
  out.set("connected", connected).set("connect_ms", connect_ms);
  if (!connected) {
    std::fprintf(
      stderr, "morai_probe: cannot connect to %s:%d within %d ms\n", opt.host.c_str(), opt.port,
      opt.connect_timeout_ms);
    if (!opt.json_path.empty() && !write_json(opt.json_path, out)) {
      std::fprintf(stderr, "morai_probe: cannot write %s\n", opt.json_path.c_str());
    }
    return 2;
  }
  std::printf("[info] connected in %.1f ms\n", connect_ms);
  out.set("simulator_version", probe.simulator_version());

  Json results = Json::object();
  const std::string & c = opt.command;
  const bool all = c == "all";
  if (all || c == "sync") results.set("sync", probe.run_sync());
  if (all || c == "spec") results.set("spec", probe.run_spec());
  if (all || c == "mode") results.set("mode", probe.run_mode());
  if (all || c == "rtt") results.set("rtt", probe.run_rtt());
  if (all || c == "frames") results.set("frames", probe.run_frames());
  if (all || c == "simtime") results.set("simtime", probe.run_simtime());
  if (all || c == "waitfortick") results.set("waitfortick", probe.run_waitfortick());
  // 26.09.25 _ YSH _ monitor 는 all 에 넣지 않는다 (다른 것이 모는 동안 Ctrl-C 까지 도는 관찰 도구)
  if (c == "monitor") results.set("monitor", probe.run_monitor());
  out.set("results", std::move(results));
  if (all) {
    out.set("summary", probe.summary());
  }

  if (!opt.json_path.empty()) {
    if (!write_json(opt.json_path, out)) {
      std::fprintf(stderr, "morai_probe: cannot write %s\n", opt.json_path.c_str());
      return 3;
    }
    std::printf("[info] JSON written to %s\n", opt.json_path.c_str());
  }
  // 26.09.25 _ YSH _ monitor --csv 실패도 3
  // return 0;
  return probe.io_error() ? 3 : 0;
}
