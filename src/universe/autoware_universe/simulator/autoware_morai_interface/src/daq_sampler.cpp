// 신규 — 26.09.26 _ YSH _ DAQ 샘플러 구현 — 스냅샷 ZOH 100 Hz 발행 + cue_server 녹화 구간 게이트
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

#include "autoware/morai_interface/daq_sampler.hpp"

#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

// 26.09.26 _ YSH _ 원시 gRPC 기록 — proto3 JSON 변환 (grpc·protobuf 비노출 헤더)
#include <morai_grpc_vendor/morai_link_raw.hpp>
#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <iterator>  // 26.09.26 _ YSH _ 리뷰 반영 (append_all)
#include <string>
#include <utility>

namespace autoware::morai_interface
{

namespace
{
unsigned long ul(uint64_t v)  // NOLINT(runtime/int)
{
  return static_cast<unsigned long>(v);  // NOLINT(runtime/int)
}

std::string json_escape(const std::string & s)
{
  std::string out;
  out.reserve(s.size());
  for (const char c : s) {
    if (c == '"' || c == '\\') {
      out.push_back('\\');
      out.push_back(c);
    } else if (static_cast<unsigned char>(c) < 0x20) {
      char buf[8];
      std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
      out += buf;
    } else {
      out.push_back(c);
    }
  }
  return out;
}

std::string trim(const std::string & s)
{
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) {
    return std::string();
  }
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

// 26.09.26 _ YSH _ 원시 기록 큐: 넘치면 가장 오래된 것을 버리고 그 칸의 런 번호를 돌려준다 (없으면 nullopt)
template <typename Entry>
std::optional<uint64_t> push_bounded(std::deque<Entry> * q, Entry && e, size_t capacity)
{
  std::optional<uint64_t> evicted;
  if (q->size() >= capacity) {
    evicted = q->front().run_serial;
    q->pop_front();
  }
  q->push_back(std::move(e));
  return evicted;
}

// 26.09.26 _ YSH _ 리뷰 반영: src 를 dst 뒤에 옮기고 비운다 (dst 가 비었으면 swap — 꼬리가 없는 보통 틱은 복사 없음)
template <typename Entry>
void append_all(std::deque<Entry> * dst, std::deque<Entry> * src)
{
  if (dst->empty()) {
    dst->swap(*src);
    return;
  }
  std::move(src->begin(), src->end(), std::back_inserter(*dst));
  src->clear();
}
}  // namespace

// =============================================================================
// 작은 부품
// =============================================================================

void DaqCommandLatch::submit(const autoware_control_msgs::msg::Control & msg, int64_t receive_steady_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  Entry e;
  e.msg = msg;
  e.receive_steady_ns = receive_steady_ns;
  e.seq = ++seq_;
  entry_ = std::move(e);
}

std::optional<DaqCommandLatch::Entry> DaqCommandLatch::latest() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return entry_;
}

uint64_t DaqCommandLatch::count() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return seq_;
}

std::optional<DaqMode> parse_daq_mode(const std::string & text)
{
  if (text == "gated") {
    return DaqMode::kGated;
  }
  if (text == "always") {
    return DaqMode::kAlways;
  }
  return std::nullopt;
}

const char * to_string(DaqMode mode)
{
  return mode == DaqMode::kAlways ? "always" : "gated";
}

std::optional<DaqStamp> parse_daq_stamp(const std::string & text)
{
  if (text == "sample") {
    return DaqStamp::kSample;
  }
  if (text == "tick") {
    return DaqStamp::kTick;
  }
  return std::nullopt;
}

const char * to_string(DaqStamp stamp)
{
  return stamp == DaqStamp::kTick ? "tick" : "sample";
}

std::optional<std::pair<bool, std::string>> parse_daq_run_state(const std::string & data)
{
  const std::string s = trim(data);
  const auto sp = s.find_first_of(" \t");
  const std::string word = sp == std::string::npos ? s : s.substr(0, sp);
  const std::string run = sp == std::string::npos ? std::string() : trim(s.substr(sp + 1));
  if (word == "active") {
    if (run.empty()) {
      return std::nullopt;  // 런 이름 없이 켜면 같은 런의 재수신인지 가를 수 없다
    }
    return std::make_pair(true, run);
  }
  if (word == "idle") {
    return std::make_pair(false, run);
  }
  return std::nullopt;
}

bool daq_rate_is_valid(double hz)
{
  return std::isfinite(hz) && hz >= kDaqRateHzMin && hz <= kDaqRateHzMax;
}

// 26.09.26 _ YSH _ 원시 gRPC 기록 수 4 개를 뒤에 더한다 (앞 키는 그대로 — cue_server 가 run JSON 'daq' 로 옮긴다). 원문:
// std::string to_json(const DaqRunSummary & s)
// {
//   char buf[512];
//   std::snprintf(
//     buf, sizeof(buf),
//     "\"duration_s\": %.3f, \"ticks\": %lu, \"state_published\": %lu, \"state_hz\": %.2f, "
//     "\"cmd_published\": %lu, \"gap_ticks\": %lu, \"link_down_ticks\": %lu, \"longest_gap_ticks\": %lu, "
//     "\"late\": %lu, \"updates\": %lu, \"yield_ticks\": %lu",
//     s.duration_s, ul(s.ticks), ul(s.state_published), s.state_hz(), ul(s.cmd_published),
//     ul(s.gap_ticks), ul(s.link_down_ticks), ul(s.longest_gap_ticks), ul(s.late), ul(s.updates),
//     ul(s.yield_ticks));
//   return "{\"run\": \"" + json_escape(s.run) + "\", \"end_reason\": \"" + json_escape(s.end_reason) +
//          "\", " + buf + "}";
// }
std::string to_json(const DaqRunSummary & s)
{
  char buf[768];
  std::snprintf(
    buf, sizeof(buf),
    "\"duration_s\": %.3f, \"ticks\": %lu, \"state_published\": %lu, \"state_hz\": %.2f, "
    "\"cmd_published\": %lu, \"gap_ticks\": %lu, \"link_down_ticks\": %lu, \"longest_gap_ticks\": %lu, "
    "\"late\": %lu, \"updates\": %lu, \"yield_ticks\": %lu, "
    "\"raw_published\": %lu, \"raw_dropped\": %lu, \"tx_published\": %lu, \"tx_dropped\": %lu",
    s.duration_s, ul(s.ticks), ul(s.state_published), s.state_hz(), ul(s.cmd_published),
    ul(s.gap_ticks), ul(s.link_down_ticks), ul(s.longest_gap_ticks), ul(s.late), ul(s.updates),
    ul(s.yield_ticks), ul(s.raw_published), ul(s.raw_dropped), ul(s.tx_published),
    ul(s.tx_dropped));
  return "{\"run\": \"" + json_escape(s.run) + "\", \"end_reason\": \"" + json_escape(s.end_reason) +
         "\", " + buf + "}";
}

// 26.09.26 _ YSH _ 원시 gRPC 기록 JSON (계약 morai_raw.*/1). 정수는 그대로 (std::to_string — 반올림 없음), 메시지 본문은
//   vendor 의 proto3 JSON (int64 는 문자열, double 은 되읽으면 같은 값). 한 줄, 끝 줄바꿈 없음
std::string daq_raw_actor_states_json(const DaqRawActorStates & item, const std::string & run)
{
  std::string body = item.bytes ? morai_grpc_vendor::actor_states_json(*item.bytes) : std::string();
  if (body.empty()) {
    body = "null";
  }
  std::string out;
  out.reserve(body.size() + 200);
  out += "{\"schema\":\"morai_raw.actor_states/1\",\"run\":\"";
  out += json_escape(run);
  out += "\",\"update_index\":";
  out += std::to_string(item.update_index);
  out += ",\"rx_system_ns\":";
  out += std::to_string(item.rx_system_ns);
  out += ",\"morai_timestamp_us\":";
  out += std::to_string(item.morai_timestamp_us);
  out += ",\"actor_states\":";
  out += body;
  out += "}";
  return out;
}

// 26.09.26 _ YSH _ 리뷰 반영: 링크 Status → 원시 기록 한 칸 (ok 는 계약대로 전송 OK, MORAI Result 는 따로)
DaqRawCommand make_daq_raw_command(
  const morai_grpc_vendor::VehicleCommand & cmd, bool stale, int64_t send_system_ns,
  const morai_grpc_vendor::Status & status, uint64_t send_seq)
{
  DaqRawCommand tx;
  tx.cmd = cmd;
  tx.send_seq = send_seq;
  tx.send_system_ns = send_system_ns;
  tx.stale = stale;
  tx.ok = status.grpc_code == 0;
  tx.grpc_code = status.grpc_code;
  if (status.grpc_code >= 0) {
    tx.rpc_ms = status.rpc_ms;  // 보내지 않았으면(-1) 잰 것이 없다 → JSON null
  }
  tx.result_status = status.result_status;
  return tx;
}

std::string daq_raw_vehicle_ctrl_cmd_json(
  const DaqRawCommand & item, const std::string & run,
  const morai_grpc_vendor::MoraiLinkOptions & link)
{
  std::string body = morai_grpc_vendor::vehicle_ctrl_cmd_json(item.cmd, link);
  if (body.empty()) {
    body = "null";
  }
  std::string rpc_ms = "null";
  if (item.rpc_ms && std::isfinite(*item.rpc_ms)) {
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%.4f", *item.rpc_ms);
    rpc_ms = buf;
  }
  std::string out;
  out.reserve(body.size() + 250);
  out += "{\"schema\":\"morai_raw.vehicle_ctrl_cmd/1\",\"run\":\"";
  out += json_escape(run);
  out += "\",\"send_seq\":";
  out += std::to_string(item.send_seq);
  out += ",\"send_system_ns\":";
  out += std::to_string(item.send_system_ns);
  out += ",\"stale\":";
  out += item.stale ? "true" : "false";
  out += ",\"ok\":";
  out += item.ok ? "true" : "false";
  out += ",\"grpc_code\":";
  out += std::to_string(item.grpc_code);
  out += ",\"rpc_ms\":";
  out += rpc_ms;
  // 26.09.26 _ YSH _ 리뷰 반영: 응답 Result.status (gRPC 가 준 값) — 계약 키 뒤에 더한다. 응답이 없으면 null
  out += ",\"result_status\":";
  out += item.result_status >= 0 ? std::to_string(item.result_status) : std::string("null");
  out += ",\"vehicle_ctrl_cmd\":";
  out += body;
  out += "}";
  return out;
}

// =============================================================================
// DaqSampler
// =============================================================================

DaqSampler::DaqSampler(
  const DaqFrameCache & cache, const DaqCommandLatch & cmd_latch, const RuntimeParamsHandle & params,
  DaqSink & sink, const DaqConfig & config, DaqMode mode, const rclcpp::Logger & logger,
  EngineClock clock, std::function<bool()> link_up)
: cache_(cache),
  cmd_latch_(cmd_latch),
  params_(params),
  sink_(sink),
  config_(config),
  logger_(logger),
  clock_(std::move(clock)),
  link_up_(std::move(link_up)),
  mode_(mode)
{
  const EngineClock real = EngineClock::real();
  if (!clock_.steady_ns) {
    clock_.steady_ns = real.steady_ns;
  }
  if (!clock_.system_ns) {
    clock_.system_ns = real.system_ns;
  }
  const double hz = daq_rate_is_valid(config_.rate_hz) ? config_.rate_hz : 100.0;
  period_ns_ = std::max<int64_t>(static_cast<int64_t>(std::llround(1e9 / hz)), 1);
  max_age_ns_ = static_cast<int64_t>(std::max(config_.max_sample_age_s, 0.0) * 1e9);
  cmd_stale_ns_ = static_cast<int64_t>(std::max(config_.cmd_stale_s, 0.0) * 1e9);
  run_timeout_ns_ = static_cast<int64_t>(std::max(config_.run_timeout_s, 0.0) * 1e9);
  totals_.mode = mode;
  raw_capacity_ = std::max<size_t>(config_.raw_queue_depth, 1);  // 26.09.26 _ YSH _
}

DaqSampler::~DaqSampler()
{
  stop();
}

void DaqSampler::start()
{
  if (thread_.joinable()) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_ = false;
  }
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    raw_closed_ = false;  // 26.09.26 _ YSH _ 리뷰 반영: 다시 시작하면 원시 기록도 다시 받는다
  }
  thread_ = std::thread([this]() { run_loop(); });
}

void DaqSampler::stop()
{
  {
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_ = true;
  }
  stop_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
  // 26.09.26 _ YSH _ 원시 기록: 마지막 틱 뒤에 쌓인 것 (발행 중에 넣은 것) 을 런을 닫기 전에 낸다 — 요약 수와 맞게.
  //   스레드를 거둔 뒤라 DAQ 스레드와 겹치지 않는다.
  //   리뷰 반영: 첫 stop 만 낸다. 그 뒤로는 submit 이 버리므로 큐가 비어 있고, 두 번째 stop (~DaqSampler — 노드의
  //   publisher 가 이미 지워진 뒤) 은 sink 를 부르지 않는다
  RawBatch rest;
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    if (!raw_closed_) {
      rest = take_raw_locked(!yield_ && (mode_ == DaqMode::kAlways || run_.active));
      raw_closed_ = true;
    }
  }
  if (!rest.empty()) {
    publish_raw(rest);
  }
  std::optional<DaqRunSummary> ended;
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    if (run_.active) {
      end_run_locked("shutdown", clock_.steady_ns(), &ended);
    }
  }
  if (ended) {
    finish_run(*ended);
  }
}

bool DaqSampler::stop_requested() const
{
  const std::lock_guard<std::mutex> lock(stop_mutex_);
  return stop_requested_;
}

void DaqSampler::sleep_until_ns(int64_t deadline_ns)
{
  const int64_t ns = deadline_ns - clock_.steady_ns();
  if (ns <= 0) {
    return;
  }
  if (clock_.sleep_ns) {
    clock_.sleep_ns(ns);
    return;
  }
  std::unique_lock<std::mutex> lock(stop_mutex_);
  stop_cv_.wait_for(lock, std::chrono::nanoseconds(ns), [this]() { return stop_requested_; });
}

void DaqSampler::run_loop()
{
  while (!stop_requested()) {
    step();
  }
}

void DaqSampler::step()
{
  int64_t now = clock_.steady_ns();
  if (next_ns_ == 0) {
    next_ns_ = now;
  }
  // Engine::timer_step 처럼 deadline 전이면 자고 돌아와 다시 본다 (그사이 stop 이 올 수 있다)
  if (now < next_ns_) {
    sleep_until_ns(next_ns_);
    return;
  }
  try {
    tick(now);
  } catch (const std::exception & e) {
    // 스레드가 죽으면 DAQ 가 조용히 멈춘다 — 삼키고 계속한다 (운전 경로와 무관)
    RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 5000, "DAQ 틱 예외: %s", e.what());
  }
  next_ns_ += period_ns_;
  now = clock_.steady_ns();
  if (next_ns_ < now) {
    // 뒤처짐 → 몰아서 따라잡지 않는다. 명령 타이머(main.py)는 기준을 '지금'으로 잡아 곧바로 한 번 더 보내지만,
    //   DAQ 는 늦은 행 다음을 한 주기 뒤로 둔다 (행 두 개가 붙어 나가지 않게)
    next_ns_ = now + period_ns_;
    const std::lock_guard<std::mutex> lock(run_mutex_);
    ++totals_.late;
    if (run_.active) {
      ++run_.summary.late;
    }
  }
}

void DaqSampler::build_state(const DaqFrame & frame, const RuntimeParams & params)
{
  const auto & snap = frame.snapshot;
  // 26.10.05 _ YSH _ MORAI 공식 메시지로 (MORAI world·MORAI 축 그대로 — map_origin 을 쓰지 않는다)
  built_ego_ = morai_to_autoware::to_daq_ego_status(
    snap.ego, config_.accel_scale, params.steer_input_norm_deg);
  built_objects_ = config_.publish_objects
                     ? morai_to_autoware::to_daq_object_status_list(snap.actors, config_.accel_scale)
                     : morai_to_autoware::to_daq_object_status_list({}, config_.accel_scale);
  // 26.10.07 _ YSH _ Ego 충돌 상대 (이름 → actor 의 ObjectStatus)
  built_collision_ = morai_to_autoware::to_daq_collision_data(snap.ego, snap.actors, config_.accel_scale);
  // 원문: built_state_ = morai_to_autoware::to_daq_vehicle_state(snap.ego, params.map_origin);
  // 원문: built_accel_ = morai_to_autoware::to_daq_acceleration(snap.ego, config_.accel_scale);
  // 원문: built_objects_ = autoware_perception_msgs::msg::TrackedObjects();
  // 원문: built_objects_.header.frame_id = "map";
  // 원문: if (config_.publish_objects) {
  // 원문:   built_objects_.objects.reserve(snap.actors.size());
  // 원문:   for (const auto & actor : snap.actors) {
  // 원문:     built_objects_.objects.push_back(
  // 원문:       morai_to_autoware::to_daq_tracked_object(actor, params.map_origin, config_.accel_scale));
  // 원문:   }
  // 원문: }
}

void DaqSampler::tick(int64_t now)
{
  std::optional<DaqRunSummary> ended;
  bool publishing = false;
  bool active = false;
  RawBatch raw;  // 26.09.26 _ YSH _ 이 틱에 낼 원시 기록 (게이트 판정과 같은 락 안에서 꺼낸다)
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    if (run_.active && run_timeout_ns_ > 0 && now - run_.start_ns > run_timeout_ns_) {
      ++totals_.run_timeouts;
      end_run_locked("timeout", now, &ended);
    }
    active = run_.active;
    publishing = !yield_ && (mode_ == DaqMode::kAlways || active);
    if (active && yield_) {
      ++run_.summary.yield_ticks;  // 켜진 런인데 양보 중 — 요약에서 결손 원인을 가를 수 있게
    }
    // 26.09.26 _ YSH _ 발행 중이 아니면 쌓인 원시 기록을 버린다 (다음 런으로 넘어가지 않게)
    raw = take_raw_locked(publishing);
  }
  if (ended) {
    finish_run(*ended);
  }
  if (!publishing) {
    // 26.09.26 _ YSH _ 리뷰 반영: replaced 로 끝난 앞 런의 꼬리 (이미 그 런 요약에 셌다) 는 게이트가 닫혀도 낸다
    if (!raw.empty()) {
      publish_raw(raw);
    }
    maybe_log(now);
    return;
  }

  DaqOutputs out;
  const auto frame = cache_.latest();
  // daq_publisher sample_is_fresh: age < max_age (같으면 낡음)
  const bool fresh = frame && (now - frame->receive_steady_ns) < max_age_ns_;
  if (frame) {
    frame_seen_ = true;
  }
  if (fresh) {
    const auto params = params_.get();
    if (!has_built_ || frame->update_index != built_update_index_ || params != built_params_) {
      // 새 MORAI 업데이트이거나 map_origin 등이 바뀌었을 때만 다시 만든다 (그 밖에는 ZOH 로 다시 낸다)
      build_state(*frame, *params);
      built_update_index_ = frame->update_index;
      built_params_ = params;
      has_built_ = true;
    }
    const builtin_interfaces::msg::Time stamp = config_.stamp == DaqStamp::kTick
                                                  ? morai_to_autoware::to_time_msg(clock_.system_ns())
                                                  : frame->stamp;
    // 26.10.05 _ YSH _ ego_status (원문: out.state = built_state_; …stamp; out.accel = built_accel_; …stamp;)
    // 26.10.12 _ YSH _ VILS 면 아래에서 실차 값으로. 원문: out.ego = built_ego_; out.ego->header.stamp = stamp;
    if (!ego_override_) {
      out.ego = built_ego_;
      out.ego->header.stamp = stamp;
    }
    if (config_.publish_objects) {
      out.objects = built_objects_;
      out.objects->header.stamp = stamp;
    }
    if (config_.publish_collision) {  // 26.10.07 _ YSH _
      out.collision = built_collision_;
      out.collision->header.stamp = stamp;
    }
  }
  // 26.10.12 _ YSH _ 의사 VILS: Ego 는 실차 (MORAI 표본 신선도와 무관 — 실차 값이 끊기면 함수가 nullopt)
  if (ego_override_) {
    out.ego = ego_override_();
    if (out.ego && config_.stamp == DaqStamp::kTick) {
      out.ego->header.stamp = morai_to_autoware::to_time_msg(clock_.system_ns());
    }
  }
  // 명세 25: 최신 control_cmd 원본. 새 명령이 cmd_stale_s 넘게 없으면 내지 않는다 (이전 런 값이 남지 않게)
  if (const auto entry = cmd_latch_.latest()) {
    if (now - entry->receive_steady_ns <= cmd_stale_ns_) {
      out.cmd = entry->msg;
      if (config_.stamp == DaqStamp::kTick) {
        out.cmd->stamp = morai_to_autoware::to_time_msg(clock_.system_ns());
      }
    }
  }
  // 26.10.12 _ YSH _ 의사 VILS: 명세 25 = 실차에 보낸 명령 (Autoware control_cmd). MORAI 에는 명령이 가지 않는다
  if (config_.ctrl_from_command) {
    if (out.cmd) {
      out.ctrl = morai_to_autoware::to_daq_ctrl_cmd_from_control(*out.cmd);
    }
  } else
  // 26.10.06 _ YSH _ 명세 25 (gRPC): MORAI 에 마지막으로 보낸 명령. cmd_stale_s 넘게 새 송신이 없으면 내지 않는다
  {
    std::optional<SentCommand> sc;
    {
      const std::lock_guard<std::mutex> lock(sent_mutex_);
      sc = sent_;
    }
    if (sc && now - sc->steady_ns <= cmd_stale_ns_) {
      out.ctrl = morai_to_autoware::to_daq_ctrl_cmd(sc->cmd, params_.get()->steer_input_norm_deg);
      out.ctrl->header.stamp = morai_to_autoware::to_time_msg(
        config_.stamp == DaqStamp::kTick ? clock_.system_ns() : sc->system_ns);
    }
  }
  // 26.10.06 _ YSH _ 원문: if (out.ego || out.cmd) {  // 26.10.05 _ YSH _ 원문: if (out.state || out.cmd) {
  if (out.ego || out.cmd || out.ctrl) {
    sink_.publish_daq(out);
  }
  // 26.09.26 _ YSH _ 원시 gRPC 기록: 지난 틱 뒤에 받아들인 업데이트·보낸 명령 전부 (한 틱에 둘 이상이어도 전부, ZOH 아님)
  if (!raw.empty()) {
    publish_raw(raw);
  }

  const bool link_down = link_up_ ? !link_up_() : false;
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    ++totals_.ticks;
    if (out.cmd) {
      ++totals_.cmd_published;
    }
    if (fresh) {
      ++totals_.state_published;
    } else if (frame_seen_ || run_.active) {
      // 첫 표본 전(기동 직후)의 always 모드는 결손으로 세지 않는다. 켜진 런 안에서는 센다
      ++totals_.gap_ticks;
      if (link_down) {
        ++totals_.link_down_ticks;
      }
    }
    if (run_.active) {
      auto & s = run_.summary;
      ++s.ticks;
      if (out.cmd) {
        ++s.cmd_published;
      }
      if (fresh) {
        ++s.state_published;
        run_.current_gap = 0;
        if (!run_.has_update || frame->update_index != run_.last_update_index) {
          ++s.updates;
          run_.last_update_index = frame->update_index;
          run_.has_update = true;
        }
      } else {
        ++s.gap_ticks;
        if (link_down) {
          ++s.link_down_ticks;
        }
        ++run_.current_gap;
        s.longest_gap_ticks = std::max(s.longest_gap_ticks, run_.current_gap);
      }
    }
  }
  maybe_log(now);
}

// 26.09.26 _ YSH _ 원시 gRPC 기록 ---------------------------------------------------------------------------------

void DaqSampler::submit_raw_actor_states(DaqRawActorStates item)
{
  if (!config_.raw) {
    return;
  }
  const std::lock_guard<std::mutex> lock(run_mutex_);
  // 26.09.26 _ YSH _ 리뷰 반영: stop() 뒤에도 버린다 (raw_closed_ — always 모드는 게이트가 열린 채다)
  if (raw_closed_ || yield_ || !(mode_ == DaqMode::kAlways || run_.active)) {
    ++totals_.raw_discarded;
    return;
  }
  RawEntry<DaqRawActorStates> e{
    std::move(item), run_.active ? run_.run : std::string(), run_.active ? run_serial_ : 0};
  if (const auto evicted = push_bounded(&raw_queue_, std::move(e), raw_capacity_)) {
    ++totals_.raw_dropped;
    if (run_.active && *evicted == run_serial_) {
      ++run_.summary.raw_dropped;
    }
  }
}

// 26.10.06 _ YSH _ 신규
void DaqSampler::submit_sent_command(const morai_grpc_vendor::VehicleCommand & cmd, int64_t send_system_ns)
{
  const std::lock_guard<std::mutex> lock(sent_mutex_);
  sent_ = SentCommand{cmd, clock_.steady_ns(), send_system_ns};
}

void DaqSampler::submit_raw_command(DaqRawCommand item)
{
  if (!config_.raw) {
    return;
  }
  const std::lock_guard<std::mutex> lock(run_mutex_);
  // 26.09.26 _ YSH _ 리뷰 반영: stop() 뒤에도 버린다 (엔진 종료 감속 — DAQ 가 엔진보다 먼저 멈춘다)
  if (raw_closed_ || yield_ || !(mode_ == DaqMode::kAlways || run_.active)) {
    ++totals_.tx_discarded;
    return;
  }
  RawEntry<DaqRawCommand> e{
    std::move(item), run_.active ? run_.run : std::string(), run_.active ? run_serial_ : 0};
  if (const auto evicted = push_bounded(&tx_queue_, std::move(e), raw_capacity_)) {
    ++totals_.tx_dropped;
    if (run_.active && *evicted == run_serial_) {
      ++run_.summary.tx_dropped;
    }
  }
}

DaqSampler::RawBatch DaqSampler::take_raw_locked(bool publishing)
{
  RawBatch batch;
  // 26.09.26 _ YSH _ 리뷰 반영: replaced 로 끝난 런의 꼬리를 앞에 (넣은 순서 유지). 옮길 때 이미 셌다
  batch.raw.swap(raw_carry_.raw);
  batch.tx.swap(raw_carry_.tx);
  if (!publishing) {
    totals_.raw_discarded += raw_queue_.size();
    totals_.tx_discarded += tx_queue_.size();
    raw_queue_.clear();
    tx_queue_.clear();
    return batch;
  }
  // 꺼낸 것은 반드시 낸다 — 수는 여기서 (게이트와 같은 락 안) 센다. 내고 나서 세면 그사이 온 idle 이 런을 닫아
  //   낸 것이 요약에서 빠질 수 있다. 요약은 이 런 이름으로 넣은 것만 (앞 런의 꼬리는 carry_raw_locked 가 앞 런에 셌다)
  totals_.raw_published += raw_queue_.size();
  totals_.tx_published += tx_queue_.size();
  if (run_.active) {
    for (const auto & e : raw_queue_) {
      if (e.run_serial == run_serial_) {
        ++run_.summary.raw_published;
      }
    }
    for (const auto & e : tx_queue_) {
      if (e.run_serial == run_serial_) {
        ++run_.summary.tx_published;
      }
    }
  }
  append_all(&batch.raw, &raw_queue_);
  append_all(&batch.tx, &tx_queue_);
  return batch;
}

// 26.09.26 _ YSH _ 리뷰 반영: A → B (replaced) 가 한 틱 안에 오면 A 의 큐 칸이 A 요약에 안 세이고 A 이름으로 나갔다.
//   끝나기 직전에 꺼내 A 요약에 세고 (take_raw_locked — run_serial_ 이 아직 A), 꼬리로 붙잡아 둔다
void DaqSampler::carry_raw_locked()
{
  raw_carry_ = take_raw_locked(!yield_);
}

void DaqSampler::publish_raw(const RawBatch & batch)
{
  // JSON 변환은 여기 (DAQ 스레드, 락 밖) — 엔진·명령 스레드는 복사만 했다
  DaqRawOutputs out;
  out.actor_states.reserve(batch.raw.size());
  for (const auto & e : batch.raw) {
    out.actor_states.push_back(daq_raw_actor_states_json(e.item, e.run));
  }
  out.vehicle_ctrl_cmd.reserve(batch.tx.size());
  for (const auto & e : batch.tx) {
    out.vehicle_ctrl_cmd.push_back(daq_raw_vehicle_ctrl_cmd_json(e.item, e.run, config_.raw_link));
  }
  sink_.publish_daq_raw(out);
}

void DaqSampler::maybe_log(int64_t now)
{
  if (log_window_start_ns_ == 0) {
    log_window_start_ns_ = now;
    log_window_prev_ = stats();
    return;
  }
  if (now - log_window_start_ns_ < kLogPeriodNs) {
    return;
  }
  const DaqStats cur = stats();
  const DaqStats & p = log_window_prev_;
  const double dt = static_cast<double>(now - log_window_start_ns_) / 1e9;
  const uint64_t state = cur.state_published - p.state_published;
  const uint64_t gaps = cur.gap_ticks - p.gap_ticks;
  // 켜지지 않은 창(gated·idle)은 조용히 넘긴다 — 로그가 런 경계만 보이게
  if (cur.ticks != p.ticks || cur.active != p.active) {
    RCLCPP_INFO(
      logger_,
      "DAQ 발행 %.1fs: 상태 %lu (%.1f Hz) 명령 %lu 결손 %lu (끊김 %lu) 지연 %lu · mode %s · 런 %s%s",
      dt, ul(state), static_cast<double>(state) / dt, ul(cur.cmd_published - p.cmd_published), ul(gaps),
      ul(cur.link_down_ticks - p.link_down_ticks), ul(cur.late - p.late), to_string(cur.mode),
      cur.active ? cur.run.c_str() : "없음", cur.yielding ? " · 다른 /daq writer 에 양보 중" : "");
    // 26.09.26 _ YSH _ 원시 gRPC 기록은 따로 한 줄 (위 줄 형식은 그대로 둔다)
    if (config_.raw) {
      const uint64_t raw = cur.raw_published - p.raw_published;
      const uint64_t tx = cur.tx_published - p.tx_published;
      RCLCPP_INFO(
        logger_, "DAQ raw %.1fs: actor_states %lu (%.1f/s) · vehicle_ctrl_cmd %lu (%.1f/s) · 큐 넘침 %lu/%lu",
        dt, ul(raw), static_cast<double>(raw) / dt, ul(tx), static_cast<double>(tx) / dt,
        ul(cur.raw_dropped - p.raw_dropped), ul(cur.tx_dropped - p.tx_dropped));
    }
  }
  log_window_prev_ = cur;
  log_window_start_ns_ = now;
}

void DaqSampler::end_run_locked(
  const std::string & reason, int64_t now, std::optional<DaqRunSummary> * ended)
{
  DaqRunSummary s = run_.summary;
  s.run = run_.run;
  s.end_reason = reason;
  s.duration_s = static_cast<double>(now - run_.start_ns) / 1e9;
  run_ = RunState();
  ++totals_.runs_ended;
  *ended = std::move(s);
}

void DaqSampler::finish_run(const DaqRunSummary & s)
{
  RCLCPP_INFO(
    logger_,
    "DAQ 런 종료 %s (%s): %.1f s · 상태 %lu (%.1f Hz) · 명령 %lu · 결손 %lu 틱 (최장 %lu, 끊김 %lu) · 지연 %lu · "
    "MORAI 업데이트 %lu · 양보 %lu 틱",
    s.run.c_str(), s.end_reason.c_str(), s.duration_s, ul(s.state_published), s.state_hz(),
    ul(s.cmd_published), ul(s.gap_ticks), ul(s.longest_gap_ticks), ul(s.link_down_ticks), ul(s.late),
    ul(s.updates), ul(s.yield_ticks));
  // 26.09.26 _ YSH _ 원시 gRPC 기록 수는 따로 한 줄 (위 줄 형식은 그대로 둔다)
  if (config_.raw) {
    RCLCPP_INFO(
      logger_, "DAQ 런 종료 %s raw: actor_states %lu (넘침 %lu) · vehicle_ctrl_cmd %lu (넘침 %lu)",
      s.run.c_str(), ul(s.raw_published), ul(s.raw_dropped), ul(s.tx_published), ul(s.tx_dropped));
  }
  sink_.on_daq_run_end(s);
}

void DaqSampler::set_mode(DaqMode mode)
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  if (mode != mode_) {
    RCLCPP_INFO(logger_, "DAQ mode %s → %s", to_string(mode_), to_string(mode));
  }
  mode_ = mode;
  totals_.mode = mode;
}

DaqMode DaqSampler::mode() const
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  return mode_;
}

void DaqSampler::set_run_state(bool active, const std::string & run)
{
  // 26.09.26 _ YSH _ 리뷰 반영: 종료 중(pre-shutdown 에서 stop 한 뒤)에 온 active 로 스레드 없는 런을 다시 열지 않는다
  if (stop_requested()) {
    return;
  }
  const int64_t now = clock_.steady_ns();
  std::optional<DaqRunSummary> ended;
  std::string stale_idle_active;  // 이름이 다른 idle 을 무시했으면 켜진 런 이름 (로그는 락 밖에서)
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    if (active) {
      if (run_.active && run_.run == run) {
        return;  // latched 재수신 (구독이 다시 붙음) — 같은 런을 이어 간다
      }
      if (run_.active) {
        carry_raw_locked();  // 26.09.26 _ YSH _ 리뷰 반영: 앞 런의 원시 기록 꼬리를 그 런 요약에 센다
        end_run_locked("replaced", now, &ended);
      }
      run_.active = true;
      run_.run = run;
      run_.start_ns = now;
      ++run_serial_;  // 26.09.26 _ YSH _ 원시 기록 큐 칸이 어느 런 것인지 가른다
      ++totals_.runs_started;
    } else if (run_.active) {
      // 26.09.26 _ YSH _ 리뷰 반영: 이름이 같을 때만 (이름 없는 idle 은 무엇이든). 원문:
      // end_run_locked("idle", now, &ended);
      if (run.empty() || run == run_.run) {
        end_run_locked("idle", now, &ended);
      } else {
        stale_idle_active = run_.run;
      }
    }
  }
  if (!stale_idle_active.empty()) {
    RCLCPP_WARN(
      logger_, "idle %s 무시 — 켜진 런은 %s (앞 런의 늦은 idle)", run.c_str(), stale_idle_active.c_str());
  }
  if (ended) {
    finish_run(*ended);
  }
  if (active) {
    RCLCPP_INFO(
      logger_, "DAQ 런 시작 %s — /daq/* %g Hz (mode %s)", run.c_str(), config_.rate_hz,
      to_string(mode()));
  }
}

// 26.09.26 _ YSH _ 리뷰 반영: 첫 판정 전 양보(yield_until_first_check)는 경고 없이, 판정 뒤에는 실제 변화만 알린다. 원문:
// void DaqSampler::set_yield(bool yield)
// {
//   const std::lock_guard<std::mutex> lock(run_mutex_);
//   if (yield != yield_) {
//     if (yield) {
//       RCLCPP_WARN(logger_, "다른 노드가 이미 /daq 를 낸다 (daq_publisher.py?) — 이 노드는 DAQ 를 내지 않는다");
//     } else {
//       RCLCPP_INFO(logger_, "다른 /daq writer 가 사라졌다 — DAQ 발행 재개");
//     }
//   }
//   yield_ = yield;
// }
void DaqSampler::set_yield(bool yield)
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  const bool first = !yield_checked_;
  yield_checked_ = true;
  if (yield && (first || !yield_)) {
    RCLCPP_WARN(logger_, "다른 노드가 이미 /daq 를 낸다 (daq_publisher.py?) — 이 노드는 DAQ 를 내지 않는다");
  } else if (!yield && yield_ && !first) {
    RCLCPP_INFO(logger_, "다른 /daq writer 가 사라졌다 — DAQ 발행 재개");
  }
  yield_ = yield;
}

void DaqSampler::yield_until_first_check()
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  yield_ = true;
  yield_checked_ = false;
}

bool DaqSampler::yield_pending_first_check() const
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  return yield_ && !yield_checked_;
}

bool DaqSampler::publishing() const
{
  const std::lock_guard<std::mutex> lock(run_mutex_);
  return !yield_ && (mode_ == DaqMode::kAlways || run_.active);
}

DaqStats DaqSampler::stats() const
{
  const int64_t now = clock_.steady_ns();
  DaqStats s;
  {
    const std::lock_guard<std::mutex> lock(run_mutex_);
    s = totals_;
    s.mode = mode_;
    s.active = run_.active;
    s.yielding = yield_;
    s.publishing = !yield_ && (mode_ == DaqMode::kAlways || run_.active);
    s.run = run_.run;
    s.run_elapsed_s = run_.active ? static_cast<double>(now - run_.start_ns) / 1e9 : 0.0;
  }
  if (const auto frame = cache_.latest()) {
    s.frame_age_s = static_cast<double>(now - frame->receive_steady_ns) / 1e9;
  }
  if (const auto entry = cmd_latch_.latest()) {
    s.cmd_age_s = static_cast<double>(now - entry->receive_steady_ns) / 1e9;
  }
  return s;
}

}  // namespace autoware::morai_interface
