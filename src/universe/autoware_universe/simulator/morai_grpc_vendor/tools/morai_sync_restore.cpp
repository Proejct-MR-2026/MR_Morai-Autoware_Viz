// 신규 — 26.09.25 _ YSH _ MORAI 동기 모드 비상 복구 CLI morai_sync_restore (멱등, Ego 를 움직이지 않음)
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

// morai_sync_restore — emergency restore after a lockstep client (morai_actuate --lockstep,
// or anything else) died and left MORAI frozen in SYNCHRONOUS mode: MORAI does nothing on
// its own when the pacing client disappears.
//
//   1. GetSynchronousMode (retried while the server is UNAVAILABLE, up to --timeout-s)
//   2. if not UNSPECIFIED: SetSynchronousMode(UNSPECIFIED, 0), retried up to --timeout-s, its
//      Result ignored (MORAI's SetSynchronousMode Result is unreliable), until
//      GetSynchronousMode reads UNSPECIFIED
//   3. verify ActorStates.timestamp advances at ~wall rate and is Unix-epoch based again
//   4. print the Ego's control mode and gear; --keyboard also sets KEYBOARD and reads it back
//
// Idempotent, no arming flag: it only restores. It never sends ControlVehicle, Tick,
// SetVehicleGear or anything else that moves the Ego (test/test_actuate_scope.cpp).

#include "probe_stats.hpp"
#include "sync_mode.hpp"

#include <grpcpp/grpcpp.h>
#include <morai/actor/actor.grpc.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/actor/actor_get.pb.h>
#include <morai/actor/actor_set.pb.h>
#include <morai/common/enum.pb.h>
#include <morai/common/object_info.pb.h>
#include <morai/common/result.pb.h>
#include <morai/common/type.pb.h>
#include <morai/simulation/simulation.grpc.pb.h>
#include <morai/simulation/simulation_enum.pb.h>
#include <morai/simulation/sync_mode.pb.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace
{

namespace sm = morai_grpc_vendor::sync_mode;
namespace pb_actor = morai_sim_api::actor;
namespace pb_sim = morai_sim_api::simulation;
using morai_grpc_vendor::probe::grpc_code_name;
using Clock = std::chrono::steady_clock;

static_assert(
  sm::kSyncUnspecified == static_cast<int>(pb_sim::SYNC_MODE_TYPE_UNSPECIFIED) &&
    sm::kSyncSynchronous == static_cast<int>(pb_sim::SYNC_MODE_TYPE_SYNCHRONOUS) &&
    sm::kModeKeyboard == static_cast<int>(pb_actor::VEHICLE_CONTROL_KEYBOARD),
  "sync_mode constants must match the protos");

constexpr int kMaxReceiveMessageLength = 32 * 1024 * 1024;

const Clock::time_point g_t0 = Clock::now();

double now_s()
{
  return std::chrono::duration<double>(Clock::now() - g_t0).count();
}

double unix_now_s()
{
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

std::chrono::system_clock::time_point deadline_in(int ms)
{
  return std::chrono::system_clock::now() + std::chrono::milliseconds(ms);
}

void sleep_ms(int ms)
{
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

std::string enum_name(const google::protobuf::EnumDescriptor * d, int v)
{
  const google::protobuf::EnumValueDescriptor * vd = d ? d->FindValueByNumber(v) : nullptr;
  return vd ? vd->name() : "UNKNOWN(" + std::to_string(v) + ")";
}

bool iequals(const std::string & a, const std::string & b)
{
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) ==
                  std::tolower(static_cast<unsigned char>(y));
         });
}

struct Ego
{
  bool ts_ok{false};  ///< GetAllActorsState answered: ts_us is valid (with or without the Ego)
  bool ok{false};     ///< ... and the Ego was found: speed / gear are valid
  std::string err;
  int64_t ts_us{0};
  double speed_mps{0.0};
  int gear{-1};
};

class Restorer
{
public:
  explicit Restorer(const sm::RestoreOptions & o) : opt_(o)
  {
    grpc::ChannelArguments args;
    args.SetMaxReceiveMessageSize(kMaxReceiveMessageLength);
    // 동기 모드 전환 직후 서버가 잠시 사라질 수 있다 — 재연결을 빨리 시도
    args.SetInt(GRPC_ARG_INITIAL_RECONNECT_BACKOFF_MS, 100);
    args.SetInt(GRPC_ARG_MIN_RECONNECT_BACKOFF_MS, 100);
    args.SetInt(GRPC_ARG_MAX_RECONNECT_BACKOFF_MS, 500);
    channel_ = grpc::CreateCustomChannel(
      opt_.host + ":" + std::to_string(opt_.port), grpc::InsecureChannelCredentials(), args);
    sim_stub_ = pb_sim::Simulation::NewStub(channel_);
    actor_stub_ = pb_actor::Actor::NewStub(channel_);
    ego_.mutable_id()->set_value(opt_.ego);
    ego_.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
    ego_.set_client_key(opt_.client_key);
  }

  bool connect() { return channel_->WaitForConnected(deadline_in(opt_.connect_timeout_ms)); }

  grpc::Status get_sync(int * type)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Empty req;
    pb_sim::SyncMode res;
    const grpc::Status st = sim_stub_->GetSynchronousMode(&ctx, req, &res);
    if (st.ok()) {
      *type = static_cast<int>(res.type());
    }
    return st;
  }

  /// The only state change of this tool besides --keyboard: back to real time.
  std::string set_unspecified()
  {
    pb_sim::SyncMode req;
    req.set_type(pb_sim::SYNC_MODE_TYPE_UNSPECIFIED);
    req.set_tick_period(0);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(std::max(opt_.deadline_ms, 2000)));
    morai_sim_api::Result res;
    const grpc::Status st = sim_stub_->SetSynchronousMode(&ctx, req, &res);
    if (!st.ok()) {
      return grpc_code_name(st.error_code()) + " '" + st.error_message() + "'";
    }
    return "Result.status=" + std::to_string(static_cast<int>(res.status())) + " '" +
           res.description() + "'";
  }

  bool get_mode(int * mode, std::string * err)
  {
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    pb_actor::CurVehicleControlMode res;
    const grpc::Status st = actor_stub_->GetVehicleControlMode(&ctx, ego_, &res);
    if (!st.ok()) {
      *err = "GetVehicleControlMode " + grpc_code_name(st.error_code());
      return false;
    }
    *mode = static_cast<int>(res.mode());
    return true;
  }

  /// --keyboard only.
  std::string set_keyboard()
  {
    pb_actor::VehicleControlModeParam req;
    *req.mutable_actor_info() = ego_;
    req.set_mode(pb_actor::VEHICLE_CONTROL_KEYBOARD);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    morai_sim_api::Result res;
    const grpc::Status st = actor_stub_->SetVehicleControlMode(&ctx, req, &res);
    if (!st.ok()) {
      return grpc_code_name(st.error_code());
    }
    return "Result.status=" + std::to_string(static_cast<int>(res.status())) + " '" +
           res.description() + "'";
  }

  Ego read_ego()
  {
    Ego e;
    pb_actor::GetAllActorsFilter filter;
    filter.set_client_key(opt_.client_key);
    filter.set_vehicle(true);
    filter.set_pedestrian(false);
    filter.set_obstacle(false);
    grpc::ClientContext ctx;
    ctx.set_deadline(deadline_in(opt_.deadline_ms));
    pb_actor::ActorStates res;
    const grpc::Status st = actor_stub_->GetAllActorsState(&ctx, filter, &res);
    if (!st.ok()) {
      e.err = "GetAllActorsState " + grpc_code_name(st.error_code());
      return e;
    }
    e.ts_ok = true;
    e.ts_us = res.timestamp();
    for (const auto & a : res.states()) {
      if (a.actor_info().id().value() == opt_.ego || iequals(a.actor_info().id().value(), opt_.ego)) {
        e.ok = true;
        e.speed_mps = std::hypot(a.velocity().x(), a.velocity().y()) / 3.6;
        e.gear = a.has_vehicle_state() ? static_cast<int>(a.vehicle_state().gear_mode()) : -1;
        return e;
      }
    }
    e.err = "Ego '" + opt_.ego + "' not in GetAllActorsState";
    e.ok = false;
    return e;
  }

private:
  const sm::RestoreOptions & opt_;
  morai_sim_api::ObjectInfo ego_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<pb_sim::Simulation::Stub> sim_stub_;
  std::unique_ptr<pb_actor::Actor::Stub> actor_stub_;
};

}  // namespace

int main(int argc, char ** argv)
{
  std::setvbuf(stdout, nullptr, _IOLBF, 0);
  sm::RestoreOptions opt;
  std::string err;
  const std::vector<std::string> args =
    argc > 1 ? std::vector<std::string>(argv + 1, argv + argc) : std::vector<std::string>();
  if (
    !sm::apply_restore_env_defaults(
      std::getenv("MORAI_SIM_ADDRESS"), std::getenv("MORAI_SIM_PORT"), &opt, &err) ||
    !sm::parse_restore_args(args, &opt, &err)) {
    std::fprintf(stderr, "morai_sync_restore: %s\n\n", err.c_str());
    sm::print_restore_usage(stderr);
    return sm::kRestoreUsage;
  }
  if (opt.help) {
    sm::print_restore_usage(stdout);
    return sm::kRestoreOk;
  }
  std::printf(
    "morai_sync_restore -> %s:%d ego '%s' (restores real-time mode; never drives the Ego)\n",
    opt.host.c_str(), opt.port, opt.ego.c_str());
  Restorer r(opt);
  sm::RestoreOutcome outcome;
  outcome.keyboard_requested = opt.keyboard;
  std::vector<std::string> did;
  if (!r.connect()) {
    std::printf("[0] not connected within %d ms - retrying GetSynchronousMode anyway\n", opt.connect_timeout_ms);
  }

  // 1. 현재 모드 (서버가 재시작 중이면 UNAVAILABLE — 재시도)
  const std::vector<int> delays =
    sm::backoff_delays_ms(static_cast<int>(opt.timeout_s * 1000.0), 100, 1000);
  int type = -1;
  int attempts = 0;
  std::string last_err;
  const double t1 = now_s();
  for (size_t k = 0;; ++k) {
    ++attempts;
    const grpc::Status st = r.get_sync(&type);
    if (st.ok()) {
      outcome.reachable = true;
      break;
    }
    last_err = grpc_code_name(st.error_code()) + " '" + st.error_message() + "'";
    if (k >= delays.size() || now_s() - t1 >= opt.timeout_s) {
      break;
    }
    sleep_ms(delays[k]);
  }
  if (!outcome.reachable) {
    std::printf(
      "[1] GetSynchronousMode: NO ANSWER after %d attempts in %.1f s (%s)\n"
      "morai_sync_restore: MORAI unreachable at %s:%d - nothing was changed. Is MORAI running? "
      "Right host/port?\n",
      attempts, now_s() - t1, last_err.c_str(), opt.host.c_str(), opt.port);
    return sm::restore_exit_code(outcome);
  }
  std::printf(
    "[1] GetSynchronousMode -> %s (attempt %d)\n", sm::sync_type_name(type).c_str(), attempts);

  // 2. UNSPECIFIED 가 아니면 되돌린다 (Result 는 무시, 되읽기로 확인)
  if (type == sm::kSyncUnspecified) {
    outcome.unspecified = true;
    std::printf("[2] already UNSPECIFIED (real time) - SetSynchronousMode NOT sent\n");
  } else {
    const double t2 = now_s();
    int sets = 0;
    for (size_t k = 0;; ++k) {
      ++sets;
      const std::string res = r.set_unspecified();
      const grpc::Status st = r.get_sync(&type);
      std::printf(
        "[2] SetSynchronousMode(UNSPECIFIED, 0) #%d -> %s (ignored); GetSynchronousMode -> %s\n",
        sets, res.c_str(),
        st.ok() ? sm::sync_type_name(type).c_str() : grpc_code_name(st.error_code()).c_str());
      if (st.ok() && type == sm::kSyncUnspecified) {
        outcome.unspecified = true;
        break;
      }
      if (k >= delays.size() || now_s() - t2 >= opt.timeout_s) {
        break;
      }
      sleep_ms(delays[k]);
    }
    did.push_back("SetSynchronousMode(UNSPECIFIED, 0) x" + std::to_string(sets));
    if (!outcome.unspecified) {
      std::printf(
        "morai_sync_restore: MORAI still reports %s after %.1f s - use the MORAI UI (Time "
        "Manager) to leave synchronous mode\n",
        sm::sync_type_name(type).c_str(), now_s() - t2);
      std::printf("did: %s\n", did.back().c_str());
      return sm::restore_exit_code(outcome);
    }
  }

  // 3. 실시간 확인 — ActorStates.timestamp 는 Ego 를 찾든 못 찾든 응답에 있다 (Ego 문제는 4 에서만 보고)
  std::vector<sm::TimeSample> samples;
  Ego e;
  const double t3 = now_s();
  while (now_s() - t3 < 2.0) {
    e = r.read_ego();
    if (e.ts_ok) {
      break;
    }
    sleep_ms(20);
  }
  double last_unix = unix_now_s();
  if (e.ts_ok) {
    const double t0 = now_s();
    samples.push_back({t0, e.ts_us});
    while (now_s() - t0 < opt.verify_s) {
      sleep_ms(5);
      const Ego x = r.read_ego();
      if (x.ts_ok) {
        samples.push_back({now_s(), x.ts_us});
        last_unix = unix_now_s();
        e = x;
      }
    }
  }
  const sm::RealtimeCheck rt = sm::judge_realtime(samples, last_unix, 0.8 * opt.verify_s);
  outcome.realtime_ok = rt.ok;
  char rate[32] = "-";
  if (std::isfinite(rt.rate)) {
    std::snprintf(rate, sizeof(rate), "%.3f", rt.rate);
  }
  const std::string ts_txt =
    samples.empty() ? std::string("none (GetAllActorsState failing: ") + e.err + ")"
                    : std::to_string(e.ts_us) + (rt.epoch ? " (epoch)" : " (NOT epoch)");
  std::printf(
    "[3] real time: %s - rate %s x wall, %zu changes in %.2f s, timestamp %s%s%s\n",
    rt.ok ? "OK" : "FAILED", rate, rt.changes, rt.span_s, ts_txt.c_str(), rt.ok ? "" : ": ",
    rt.ok ? "" : rt.why.c_str());

  // 4. Ego 모드·기어 (보고), --keyboard 면 KEYBOARD 로 설정 후 되읽기
  int mode = -1;
  const bool mode_read = r.get_mode(&mode, &err);
  const Ego g = r.read_ego();
  std::printf(
    "[4] Ego control mode %s, gear %s, |v| %s\n",
    mode_read ? enum_name(pb_actor::VehicleControlMode_descriptor(), mode).c_str() : err.c_str(),
    g.ok ? enum_name(pb_actor::GearMode_descriptor(), g.gear).c_str() : "-",
    g.ok ? (std::to_string(g.speed_mps) + " m/s").c_str() : "-");
  if (!g.ok) {
    std::printf(
      "[4] note: %s - check --ego (the real-time check above does not need the Ego)\n",
      g.err.c_str());
  }
  if (g.ok && !(g.speed_mps < 0.05)) {
    // 동기 모드에서 달리던 중 클라이언트가 죽었으면 실시간으로 풀리는 순간 다시 굴러간다 — 이 도구는 브레이크하지 않는다
    std::printf(
      "[4] WARNING: the Ego is moving at %.3f m/s (it was frozen mid-drive) - this tool does not "
      "brake: stop it in the MORAI UI\n",
      g.speed_mps);
  }
  if (opt.keyboard) {
    const std::string res = r.set_keyboard();
    did.push_back("SetVehicleControlMode(KEYBOARD)");
    int now = -1;
    for (int i = 0; i < 75; ++i) {
      if (r.get_mode(&now, &err) && now == sm::kModeKeyboard) {
        outcome.keyboard_ok = true;
        break;
      }
      sleep_ms(20);
    }
    std::printf(
      "[4] --keyboard: SetVehicleControlMode(KEYBOARD) -> %s; read back %s %s\n", res.c_str(),
      now >= 0 ? enum_name(pb_actor::VehicleControlMode_descriptor(), now).c_str() : err.c_str(),
      outcome.keyboard_ok ? "- OK" : "- NOT KEYBOARD");
  } else if (mode_read && mode == 7) {
    std::printf(
      "[4] note: mode is still VEHICLE_CONTROL_SYNCHRONOUS_MODE - rerun with --keyboard, or set "
      "it in the MORAI UI\n");
  }

  const int code = sm::restore_exit_code(outcome);
  std::string did_txt;
  for (const auto & d : did) {
    did_txt += (did_txt.empty() ? "" : ", ") + d;
  }
  std::printf(
    "did: %s\nresult: %s (exit %d)\n", did_txt.empty() ? "nothing (read-only checks)" : did_txt.c_str(),
    code == sm::kRestoreOk ? "MORAI runs in real time" :
    code == sm::kRestoreNotAdvancing ? "UNSPECIFIED but time is not advancing at wall rate - paused? check the MORAI UI" :
    code == sm::kRestoreModeFailed ? "real time, but the mode did not read back KEYBOARD" : "FAILED",
    code);
  return code;
}
