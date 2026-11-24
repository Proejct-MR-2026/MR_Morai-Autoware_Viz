// 신규 — 26.09.25 _ YSH _ Engine 구현 — 트리거(WaitForTick/폴백 폴링) + on_update 본체 + 연결 상태 기계
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

#include "autoware/morai_interface/engine.hpp"

#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <utility>
#include <vector>  // 26.11.10 _ YSH _ 외란 주입 (객체 프레임의 actor 목록)

namespace autoware::morai_interface
{

namespace mgv = morai_grpc_vendor;

namespace
{
constexpr int64_t kMs = 1000000LL;
constexpr int64_t kSec = 1000000000LL;

constexpr std::chrono::milliseconds kConnectTimeout{500};
constexpr int64_t kConnectRetryNs = 200 * kMs;  ///< wait_connected 가 이미 500 ms 기다린 뒤
constexpr int kRecreateEvery = 5;               ///< 연결 시도 이만큼 실패할 때마다 채널을 새로
constexpr int64_t kWftProbePeriodNs = 10 * kSec;
/// 폴백 중 WaitForTick 재확인: 짧은 deadline 으로 연속 kWftProbeTicks 번, 서로 다른 값, 실제로 기다렸어야 복귀
constexpr std::chrono::milliseconds kWftProbeDeadline{40};
constexpr int kWftProbeTicks = 3;
constexpr int64_t kWftProbeMinElapsedNs = 5 * kMs;
constexpr int64_t kWftNoUpdateSleepNs = 1 * kMs;  ///< WaitForTick 성공 + 새 업데이트 없음 → 헛돌지 않게
constexpr int64_t kWftNoEgoSleepNs = 5 * kMs;     ///< WaitForTick 성공 + Ego 없음 → 헛돌지 않게
constexpr int64_t kEgoLostRefreshNs = 1 * kSec;   ///< Ego 가 이만큼 없으면 돌아올 때 스펙을 다시 읽는다
constexpr int64_t kSpecRefreshRetryNs = 500 * kMs;
// 26.09.25 _ YSH _ e2e 반영: 재연결 뒤 AUTO 재지정이 전송에서 실패하면 모드 조회마다 다시 (합쳐 이만큼)
constexpr int kReassertAttempts = 3;
constexpr int64_t kStatsPeriodNs = 10 * kSec;
constexpr int64_t kGapThresholdUs = 45000;
constexpr int64_t kShutdownBrakeNs = 300 * kMs;
constexpr int64_t kShutdownActiveNs = 1 * kSec;
constexpr uint64_t kDeadlineRefreshEvery = 50;  ///< 명령 deadline 을 스냅샷 왕복 p50 으로 다시 정하는 주기 (업데이트)
constexpr size_t kDeadlineMinSamples = 10;
constexpr double kCmdTimeoutFactor = 4.0;       ///< main.py CMD_TIMEOUT_FACTOR
constexpr int64_t kCmdTimeoutCapMs = 1000;      ///< main.py CMD_TIMEOUT_CAP_S
constexpr int32_t kGrpcUnknown = 2;
constexpr int32_t kGrpcUnimplemented = 12;

/// main.py spec_backoff_s: min(0.5 * 1.5^attempt, 5.0) [s]
int64_t spec_backoff_ns(int attempt)
{
  const double s = std::min(0.5 * std::pow(1.5, std::max(attempt, 0)), 5.0);
  return static_cast<int64_t>(s * 1e9);
}

unsigned long ul(uint64_t v)  // NOLINT(runtime/int)
{
  return static_cast<unsigned long>(v);  // NOLINT(runtime/int)
}

bool accepted(SampleVerdict v)
{
  return v == SampleVerdict::kNew || v == SampleVerdict::kRebased;
}

std::string mode_name(int mode)
{
  return mode < 0 ? std::string("?") : mgv::to_string(static_cast<mgv::ControlMode>(mode));
}
}  // namespace

std::optional<TimeSource> parse_time_source(const std::string & text)
{
  if (text == "morai") {
    return TimeSource::kMorai;
  }
  if (text == "receive") {
    return TimeSource::kReceive;
  }
  return std::nullopt;
}

const char * to_string(TimeSource source)
{
  return source == TimeSource::kReceive ? "receive" : "morai";
}

// 26.09.25 _ YSH _ command_trigger 파라미터 (update | timer)
std::optional<CommandTrigger> parse_command_trigger(const std::string & text)
{
  if (text == "update") {
    return CommandTrigger::kUpdate;
  }
  if (text == "timer") {
    return CommandTrigger::kTimer;
  }
  return std::nullopt;
}

const char * to_string(CommandTrigger trigger)
{
  return trigger == CommandTrigger::kTimer ? "timer" : "update";
}

bool cmd_resend_hz_is_valid(double hz)
{
  return std::isfinite(hz) && hz >= kCmdResendHzMin && hz <= kCmdResendHzMax;
}

int64_t cmd_timeout_for_rtt_ms(int64_t base_ms, double rtt_ms)
{
  // main.py cmd_timeout_for_rtt: min(max(설정, 왕복 × 4), max(1 s, 설정))
  const double want = std::max(static_cast<double>(base_ms), rtt_ms * kCmdTimeoutFactor);
  const double cap = static_cast<double>(std::max(kCmdTimeoutCapMs, base_ms));
  if (!std::isfinite(want)) {
    return base_ms;
  }
  return static_cast<int64_t>(std::ceil(std::min(want, cap)));
}

EngineClock EngineClock::real()
{
  EngineClock c;
  c.steady_ns = []() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::steady_clock::now().time_since_epoch())
                                  .count());
  };
  c.system_ns = []() {
    return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                  std::chrono::system_clock::now().time_since_epoch())
                                  .count());
  };
  return c;
}

// =============================================================================
// 통계 (원자 카운터)
// =============================================================================

struct Engine::AtomicStats
{
  std::atomic<uint64_t> snapshots{0};
  std::atomic<uint64_t> updates{0};
  std::atomic<uint64_t> duplicates{0};
  std::atomic<uint64_t> backward{0};
  std::atomic<uint64_t> rebased{0};
  std::atomic<uint64_t> no_ego{0};
  std::atomic<uint64_t> invalid_ego{0};
  std::atomic<uint64_t> gaps_over_45ms{0};
  std::atomic<uint64_t> snapshot_failures{0};
  std::atomic<uint64_t> wft_failures{0};
  std::atomic<uint64_t> link_losses{0};
  std::atomic<uint64_t> reconnects{0};
  std::atomic<uint64_t> link_recreates{0};
  std::atomic<uint64_t> sends{0};
  std::atomic<uint64_t> send_ok{0};
  std::atomic<uint64_t> send_timeouts{0};
  std::atomic<uint64_t> send_result_failures{0};
  std::atomic<uint64_t> send_errors{0};
  std::atomic<uint64_t> stale_sends{0};
  std::atomic<uint64_t> blocked_sends{0};
  std::atomic<uint64_t> block_brake_sends{0};
  std::atomic<uint64_t> mode_query_failures{0};
  std::atomic<uint64_t> objects_published{0};
  std::atomic<uint64_t> wft_no_update{0};
  std::atomic<uint64_t> spec_refreshes{0};
  std::atomic<uint64_t> content_mode_entries{0};
  // 26.09.25 _ YSH _ timer 트리거 지연재설정 (main.py late)
  std::atomic<uint64_t> cmd_timer_late{0};

  EngineStats load() const
  {
    EngineStats s;
    s.snapshots = snapshots.load();
    s.updates = updates.load();
    s.duplicates = duplicates.load();
    s.backward = backward.load();
    s.rebased = rebased.load();
    s.no_ego = no_ego.load();
    s.invalid_ego = invalid_ego.load();
    s.gaps_over_45ms = gaps_over_45ms.load();
    s.snapshot_failures = snapshot_failures.load();
    s.wft_failures = wft_failures.load();
    s.link_losses = link_losses.load();
    s.reconnects = reconnects.load();
    s.link_recreates = link_recreates.load();
    s.sends = sends.load();
    s.send_ok = send_ok.load();
    s.send_timeouts = send_timeouts.load();
    s.send_result_failures = send_result_failures.load();
    s.send_errors = send_errors.load();
    s.stale_sends = stale_sends.load();
    s.blocked_sends = blocked_sends.load();
    s.block_brake_sends = block_brake_sends.load();
    s.mode_query_failures = mode_query_failures.load();
    s.objects_published = objects_published.load();
    s.wft_no_update = wft_no_update.load();
    s.spec_refreshes = spec_refreshes.load();
    s.content_mode_entries = content_mode_entries.load();
    // 26.09.25 _ YSH _ timer 트리거 지연재설정
    s.cmd_timer_late = cmd_timer_late.load();
    return s;
  }
};

// =============================================================================
// 수명
// =============================================================================

Engine::Engine(
  ILink & link, CommandSlot & slot, const RuntimeParamsHandle & params, StateCache & cache,
  // 26.11.10 _ YSH _ 외란 손잡이 인자 추가 (nullptr = 외란 없음)
  // 원문: EngineSink & sink, const EngineConfig & config, const rclcpp::Logger & logger, EngineClock clock)
  EngineSink & sink, const EngineConfig & config, const rclcpp::Logger & logger, EngineClock clock,
  const DisturbanceHandle * disturbance)
: link_(link),
  slot_(slot),
  params_(params),
  cache_(cache),
  sink_(sink),
  config_(config),
  logger_(logger),
  clock_(std::move(clock)),
  gate_(config.gate),
  disturbance_(disturbance),  // 26.11.10 _ YSH _
  stats_(std::make_unique<AtomicStats>())
{
  command_deadline_ms_ = std::max<int64_t>(config_.command_deadline.count(), 1);
  const EngineClock real = EngineClock::real();
  if (!clock_.steady_ns) {
    clock_.steady_ns = real.steady_ns;
  }
  if (!clock_.system_ns) {
    clock_.system_ns = real.system_ns;
  }
}

Engine::~Engine()
{
  stop();
}

// 26.09.25 _ YSH _ 명령 타이머 스레드를 같이 띄우고 같이 거둔다 (종료 감속은 두 스레드가 다 멈춘 뒤 한 번)
// void Engine::start()
// {
//   if (thread_.joinable()) {
//     return;
//   }
//   {
//     const std::lock_guard<std::mutex> lock(stop_mutex_);
//     stop_requested_ = false;
//   }
//   thread_ = std::thread([this]() { run(); });
// }
//
// void Engine::stop()
// {
//   {
//     const std::lock_guard<std::mutex> lock(stop_mutex_);
//     if (stop_requested_ && !thread_.joinable()) {
//       return;
//     }
//     stop_requested_ = true;
//   }
//   stop_cv_.notify_all();
//   if (thread_.joinable()) {
//     thread_.join();
//     shutdown_brake();
//   }
// }
// 26.09.26 _ YSH _ 제어 모드 조회 스레드도 같이 띄우고 같이 거둔다. 원문 (명령 타이머까지):
// void Engine::start()
// {
//   if (thread_.joinable() || timer_thread_.joinable()) {
//     return;
//   }
//   {
//     const std::lock_guard<std::mutex> lock(stop_mutex_);
//     stop_requested_ = false;
//   }
//   thread_ = std::thread([this]() { run(); });
//   timer_thread_ = std::thread([this]() { run_command_timer(); });
// }
//
// void Engine::stop()
// {
//   {
//     const std::lock_guard<std::mutex> lock(stop_mutex_);
//     if (stop_requested_ && !thread_.joinable() && !timer_thread_.joinable()) {
//       return;
//     }
//     stop_requested_ = true;
//   }
//   stop_cv_.notify_all();
//   const bool was_running = thread_.joinable() || timer_thread_.joinable();
//   if (thread_.joinable()) {
//     thread_.join();
//   }
//   if (timer_thread_.joinable()) {
//     timer_thread_.join();
//   }
//   if (was_running) {
//     shutdown_brake();
//   }
// }
void Engine::start()
{
  if (thread_.joinable() || timer_thread_.joinable() || mode_thread_.joinable()) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    stop_requested_ = false;
  }
  thread_ = std::thread([this]() { run(); });
  timer_thread_ = std::thread([this]() { run_command_timer(); });
  mode_thread_ = std::thread([this]() { run_mode_query(); });
}

void Engine::stop()
{
  {
    const std::lock_guard<std::mutex> lock(stop_mutex_);
    if (
      stop_requested_ && !thread_.joinable() && !timer_thread_.joinable() &&
      !mode_thread_.joinable()) {
      return;
    }
    stop_requested_ = true;
  }
  stop_cv_.notify_all();
  const bool was_running = thread_.joinable() || timer_thread_.joinable();
  if (thread_.joinable()) {
    thread_.join();
  }
  if (timer_thread_.joinable()) {
    timer_thread_.join();
  }
  if (mode_thread_.joinable()) {
    mode_thread_.join();
  }
  if (was_running) {
    shutdown_brake();
  }
}

bool Engine::stop_requested() const
{
  const std::lock_guard<std::mutex> lock(stop_mutex_);
  return stop_requested_;
}

void Engine::sleep_ns(int64_t ns)
{
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

// 26.09.25 _ YSH _ 명령 타이머의 절대 deadline 대기 (main.py _stop.wait(delay) 와 같이 stop() 으로 깨운다).
// 26.09.25 _ YSH _ 리뷰 반영: deadline 을 std::chrono::steady_clock 절대 시각으로 바꾸지 않고 남은 시간만큼 상대 대기한다
//   (main.py 도 delay = next − now 로 wait). clock_.steady_ns 가 실제 steady_clock 이 아닌 주입 시계인데 sleep_ns 는 비어
//   있으면, 절대 시각 변환은 엉뚱한 시각까지 자거나(아주 길게) 바로 돌아와 타이머 스레드가 헛돌았다. 주기 계산은 여전히 절대
//   deadline(timer_next_ns_ += 주기)이라 오차가 쌓이지 않는다. 원문:
// void Engine::sleep_until_ns(int64_t deadline_ns)
// {
//   if (clock_.sleep_ns) {
//     sleep_ns(deadline_ns - clock_.steady_ns());
//     return;
//   }
//   const std::chrono::steady_clock::time_point until{std::chrono::nanoseconds(deadline_ns)};
//   std::unique_lock<std::mutex> lock(stop_mutex_);
//   stop_cv_.wait_until(lock, until, [this]() { return stop_requested_; });
// }
void Engine::sleep_until_ns(int64_t deadline_ns)
{
  // sleep_ns: 주입 대기가 있으면 그것, 없으면 stop() 으로 깨울 수 있는 상대 대기 (0 이하면 바로 돌아온다)
  sleep_ns(deadline_ns - clock_.steady_ns());
}

// 26.09.25 _ YSH _ stop() 뒤(종료 감속)에도 실제로 자는 대기 — sleep_ns 는 stop 이면 바로 돌아온다
void Engine::plain_sleep_ns(int64_t ns)
{
  if (ns <= 0) {
    return;
  }
  if (clock_.sleep_ns) {
    clock_.sleep_ns(ns);
    return;
  }
  std::this_thread::sleep_for(std::chrono::nanoseconds(ns));
}

void Engine::run()
{
  while (!stop_requested()) {
    try {
      step();
    } catch (const std::exception & e) {
      // 스레드가 죽으면 송신·발행이 조용히 멈춘다 — 삼키고 계속한다
      RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 5000, "엔진 루프 예외: %s", e.what());
      sleep_ns(100 * kMs);
    }
  }
}

void Engine::step()
{
  if (phase_.load() == Phase::kConnecting) {
    connect_step();
  } else {
    run_step();
  }
}

// =============================================================================
// 연결 (연결 확인 → 스펙 → 제어 모드 → 실행)
// =============================================================================

void Engine::connect_step()
{
  const int64_t now = clock_.steady_ns();
  if (now < next_connect_ns_) {
    sleep_ns(std::min<int64_t>(next_connect_ns_ - now, 50 * kMs));
    return;
  }
  ++connect_attempts_;
  if (recreate_pending_ || (connect_attempts_ > 1 && connect_attempts_ % kRecreateEvery == 0)) {
    link_.recreate();
    recreate_pending_ = false;
    ++stats_->link_recreates;
    RCLCPP_WARN(logger_, "MORAI gRPC 채널을 새로 만든다 (%s)", link_.target().c_str());
  }

  const mgv::Status connected = link_.wait_connected(kConnectTimeout);
  if (!connected.ok) {
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 5000, "MORAI 연결 대기 (%s, 시도 %d): %s", link_.target().c_str(),
      connect_attempts_, connected.message.c_str());
    next_connect_ns_ = clock_.steady_ns() + kConnectRetryNs;
    return;
  }

  // 26.09.25 _ YSH _ main.py get_max_steering_angle 과 같이 죽지 않고 기다린다 (백오프 0.75 s ~ 5 s)
  const auto spec = link_.get_vehicle_spec();
  if (!spec.ok() || !(spec.value.max_steering_angle_deg > 0.0)) {
    ++spec_attempts_;
    std::string why;
    if (!spec.ok()) {
      why = spec.status.unavailable() ? "MORAI 에 닿지 않는다"
            : spec.status.grpc_code == kGrpcUnknown
              ? "MORAI 는 응답하나 Ego 가 없다(시나리오 교체 중?)"
              : spec.status.message;
    } else {
      why = "max_steering_angle=" + std::to_string(spec.value.max_steering_angle_deg) +
            " deg (Ego 미스폰?)";
    }
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 10000, "Ego 스펙 대기 (재시도 %d회): %s — 죽지 않고 기다린다",
      spec_attempts_, why.c_str());
    next_connect_ns_ = clock_.steady_ns() + spec_backoff_ns(spec_attempts_);
    return;
  }

  EgoVehicleSpec ego_spec;
  ego_spec.max_steering_angle =
    morai_to_autoware::max_steering_angle_rad(spec.value.max_steering_angle_deg);
  ego_spec.wheel_base = spec.value.wheel_base_m;
  {
    const std::lock_guard<std::mutex> lock(spec_mutex_);
    spec_ = ego_spec;
  }
  RCLCPP_INFO(
    logger_, "MORAI Ego 스펙: max_steering_angle %.4f rad (%.2f deg), wheel_base %.3f m%s",
    ego_spec.max_steering_angle, spec.value.max_steering_angle_deg, ego_spec.wheel_base,
    spec_attempts_ > 0 ? " (스펙 대기 뒤 확보)" : "");
  spec_refresh_pending_ = false;
  // 첫 명령 deadline 은 스펙 호출 왕복으로 (스냅샷 왕복 창이 차면 그것으로 바꾼다)
  snapshot_rtt_ms_.clear();
  refresh_command_deadline(spec.status.rpc_ms);

  const bool reconnected = ever_connected_;
  query_control_mode();
  if (!reconnected) {
    handle_engage_on_start();
  } else if (config_.auto_reassert_auto) {
    // 조회가 실패했으면 다음 조회 성공 때 판단한다 (재연결 직후 MORAI 가 아직 응답을 못 할 수 있다)
    reassert_pending_ = true;
    reassert_attempts_ = 0;  // 26.09.25 _ YSH _ e2e 반영: 전송 실패 재시도 횟수
    evaluate_reassert();
  }
  // 26.09.26 _ YSH _ 방금 조회로 판단했다 — 재시도는 조회 스레드의 다음 성공 조회 때
  reassert_seen_seq_ = mode_query_seq_.load();

  ever_connected_ = true;
  connect_attempts_ = 0;
  spec_attempts_ = 0;
  link_fail_streak_ = 0;
  deadline_streak_ = 0;
  wft_fail_streak_ = 0;
  wft_dup_streak_ = 0;
  has_last_wft_value_ = false;
  fallback_ = false;  // 트리거는 WaitForTick 부터 다시 본다
  next_poll_ns_ = 0;
  if (reconnected) {
    ++stats_->reconnects;
  }
  phase_ = Phase::kRunning;
  // 26.09.25 _ YSH _ 명령 트리거에 따라 ControlVehicle 설명이 다르다 (timer = 고정 주기, 업데이트 루프는 송신 안 함)
  // RCLCPP_INFO(
  //   logger_, "MORAI %s (%s) — MORAI 업데이트마다 스냅샷 1 회 · ControlVehicle 1 회 (트리거 WaitForTick)",
  //   reconnected ? "재연결" : "연결", link_.target().c_str());
  const auto rp = params_.get();
  if (rp->command_trigger == CommandTrigger::kTimer) {
    RCLCPP_INFO(
      logger_,
      "MORAI %s (%s) — MORAI 업데이트마다 스냅샷 1 회 (트리거 WaitForTick) · ControlVehicle 은 명령 타이머 %g Hz "
      "(command_trigger=timer)",
      reconnected ? "재연결" : "연결", link_.target().c_str(), rp->cmd_resend_hz);
  } else {
    RCLCPP_INFO(
      logger_, "MORAI %s (%s) — MORAI 업데이트마다 스냅샷 1 회 · ControlVehicle 1 회 (트리거 WaitForTick)",
      reconnected ? "재연결" : "연결", link_.target().c_str());
  }
  sink_.on_connected(reconnected);
}

void Engine::handle_engage_on_start()
{
  if (!config_.engage_on_start || !params_.get()->forward_control_to_sim) {
    return;  // 사람이 시뮬 키보드로 모는 중이면 모드를 건드리지 않는다
  }
  const auto previous = requested_mode();
  note_requested_mode(mgv::ControlMode::kAuto);
  const auto st = link_.set_control_mode(mgv::ControlMode::kAuto, config_.control_mode_verify);
  if (st.ok) {
    // 26.09.26 _ YSH _ 조회 스레드와 같이 쓴다 — 세대를 올려 설정 전에 시작한 조회가 덮어쓰지 않게. 원문:
    // control_mode_ = static_cast<int>(mgv::ControlMode::kAuto);
    // mode_ok_steady_ns_ = clock_.steady_ns();
    overwrite_control_mode(static_cast<int>(mgv::ControlMode::kAuto), clock_.steady_ns());
    RCLCPP_INFO(logger_, "engage_on_start: MORAI 제어 모드 → AUTO");
  } else {
    restore_requested_mode(previous);
    RCLCPP_WARN(logger_, "engage_on_start: AUTO 설정 실패: %s", st.message.c_str());
  }
}

void Engine::evaluate_reassert()
{
  if (!reassert_pending_) {
    return;
  }
  const auto current = control_mode();
  if (!current) {
    return;  // 재연결 뒤 아직 조회 성공 전 — 기다린다
  }
  reassert_pending_ = false;
  if (!params_.get()->forward_control_to_sim) {
    return;  // 사람이 시뮬 키보드로 모는 중 — 모드를 건드리지 않는다
  }
  const auto requested = requested_mode();
  if (!requested || *requested != mgv::ControlMode::kAuto) {
    return;
  }
  if (*current != mgv::ControlMode::kKeyboard) {
    return;
  }
  RCLCPP_WARN(
    logger_, "재연결 뒤 MORAI 제어 모드가 KEYBOARD — 마지막 요청(AUTO)으로 재지정 (auto_reassert_auto)");
  const auto st = link_.set_control_mode(mgv::ControlMode::kAuto, config_.control_mode_verify);
  if (st.ok) {
    // 26.09.26 _ YSH _ 조회 스레드와 같이 쓴다 (위 engage_on_start 와 같은 이유). 원문:
    // control_mode_ = static_cast<int>(mgv::ControlMode::kAuto);
    // mode_ok_steady_ns_ = clock_.steady_ns();
    overwrite_control_mode(static_cast<int>(mgv::ControlMode::kAuto), clock_.steady_ns());
    RCLCPP_INFO(logger_, "AUTO 재지정 확인 (되읽기)");
  } else if (!st.readback_checked && ++reassert_attempts_ < kReassertAttempts) {
    // 26.09.25 _ YSH _ e2e 반영: MORAI 에 닿지도 못한 실패(되읽기 전 — 재연결 직후 전송 오류)는 다음 모드 조회(~100 ms)
    //   때 다시 판단한다. 전에는 한 번 실패로 재지정을 영영 잃었다. MORAI 가 받고도 AUTO 가 안 된 것(되읽기 불일치)은
    //   되풀이하지 않는다 (되읽기 500 ms 가 엔진 루프를 막는다)
    reassert_pending_ = true;
    RCLCPP_WARN(
      logger_, "AUTO 재지정 전송 실패 (%d/%d, 다음 모드 조회 때 다시): %s", reassert_attempts_,
      kReassertAttempts, st.message.c_str());
  } else {
    RCLCPP_ERROR(logger_, "AUTO 재지정 실패: %s", st.message.c_str());
  }
}

void Engine::enter_connecting(const std::string & why, bool recreate)
{
  if (phase_.load() == Phase::kConnecting) {
    return;
  }
  phase_ = Phase::kConnecting;
  ++stats_->link_losses;
  recreate_pending_ = recreate;
  next_connect_ns_ = 0;
  connect_attempts_ = 0;
  spec_attempts_ = 0;
  link_fail_streak_ = 0;
  deadline_streak_ = 0;
  wft_fail_streak_ = 0;
  wft_dup_streak_ = 0;
  last_send_ns_ = 0;
  // 끊기기 전 모드로 AUTO 재지정을 판단하지 않게 지운다 (재연결 뒤 첫 조회 성공이 채운다)
  // 26.09.26 _ YSH _ 조회 스레드가 끊기기 전에 시작한 조회를 뒤늦게 저장하지 않게 세대를 올리며 지운다. 원문:
  // control_mode_ = -1;
  // mode_ok_steady_ns_ = 0;
  overwrite_control_mode(-1, 0);
  reassert_pending_ = false;
  RCLCPP_WARN(logger_, "MORAI 링크 재연결로 전환: %s", why.c_str());
}

void Engine::enter_fallback(const std::string & why)
{
  if (fallback_.load()) {
    return;
  }
  fallback_ = true;
  next_poll_ns_ = 0;
  wft_dup_streak_ = 0;
  last_wft_probe_ns_ = clock_.steady_ns();
  RCLCPP_WARN(
    logger_,
    "WaitForTick 을 트리거로 쓸 수 없다 (%s) — %.0f Hz 폴링으로 폴백 (새 timestamp 마다 송신 1 회는 "
    "그대로)",
    why.c_str(), config_.fallback_hz);
}

// =============================================================================
// 트리거
// =============================================================================

bool Engine::probe_wait_for_tick()
{
  // 짧은 deadline 으로 연속 몇 번 — 모두 성공, 값이 매번 다르고(실측: 반복 없음), 실제로 기다렸어야 한다.
  // 막지 않고 바로 돌아오는 WaitForTick 은 트리거가 아니다
  const auto deadline = std::min(config_.wait_for_tick_deadline, kWftProbeDeadline);
  const int64_t t0 = clock_.steady_ns();
  bool has_prev = false;
  int64_t prev = 0;
  for (int i = 0; i < kWftProbeTicks; ++i) {
    const auto r = link_.wait_for_tick(deadline);
    if (!r.ok() || (has_prev && r.value == prev)) {
      return false;
    }
    prev = r.value;
    has_prev = true;
  }
  if (clock_.steady_ns() - t0 < kWftProbeMinElapsedNs) {
    return false;
  }
  last_wft_value_ = prev;
  has_last_wft_value_ = true;
  return true;
}

void Engine::run_step()
{
  bool wft_tried = false;
  bool wft_ok = false;
  bool wft_new_value = false;
  // 26.09.25 _ YSH _ e2e 반영: deadline 초과를 따로 기억한다 (아래 — 업데이트가 없었으면 실패로 세지 않는다)
  bool wft_timed_out = false;
  if (!fallback_.load()) {
    const auto r = link_.wait_for_tick(config_.wait_for_tick_deadline);
    wft_tried = true;
    wft_ok = r.ok();
    wft_timed_out = r.status.deadline_exceeded();
    if (wft_ok) {
      wft_new_value = !has_last_wft_value_ || r.value != last_wft_value_;
      last_wft_value_ = r.value;
      has_last_wft_value_ = true;
    } else {
      ++stats_->wft_failures;
      if (r.status.grpc_code == kGrpcUnimplemented) {
        enter_fallback("UNIMPLEMENTED");
      }
    }
  } else {
    const int64_t period = static_cast<int64_t>(1e9 / std::max(config_.fallback_hz, 1.0));
    const int64_t now0 = clock_.steady_ns();
    if (next_poll_ns_ == 0) {
      next_poll_ns_ = now0;
    }
    if (now0 < next_poll_ns_) {
      sleep_ns(next_poll_ns_ - now0);
    }
    next_poll_ns_ += period;
    const int64_t after = clock_.steady_ns();
    if (next_poll_ns_ < after) {
      next_poll_ns_ = after;  // 뒤처지면 몰아서 따라잡지 않는다
    }
    if (after - last_wft_probe_ns_ >= kWftProbePeriodNs) {
      last_wft_probe_ns_ = after;
      if (probe_wait_for_tick()) {
        fallback_ = false;
        wft_fail_streak_ = 0;
        wft_dup_streak_ = 0;
        RCLCPP_INFO(logger_, "WaitForTick 이 다시 돌아온다 — 업데이트 트리거로 복귀");
      }
    }
  }
  if (stop_requested()) {
    return;
  }

  const mgv::Status st = link_.get_snapshot(&snapshot_buf_);
  if (!st.ok) {
    on_snapshot_failure(st);
    return;
  }
  ++stats_->snapshots;
  link_fail_streak_ = 0;
  deadline_streak_ = 0;
  snapshot_rtt_ms_.add(snapshot_buf_.rpc_ms);

  // 26.09.25 _ YSH _ e2e(가짜 MORAI 1 s 멈춤) 반영: 판정을 먼저 하고 WaitForTick 실패를 센다. deadline 초과인데 스냅샷도
  //   새 업데이트가 아니면(일시정지·정체·아무도 Tick 하지 않는 동기 모드) WaitForTick 은 맞게 기다린 것이다 — 세지 않는다.
  //   전에는 이것도 세어 1 s 넘는 일시정지 한 번에 폴링 폴백으로 넘어가 10 s 동안 업데이트 트리거를 잃었다.
  //   업데이트가 왔는데 WaitForTick 이 알리지 못했거나(deadline + 새 표본) deadline 이 아닌 오류만 센다
  // if (wft_tried) {
  //   if (wft_ok) {
  //     wft_fail_streak_ = 0;
  //   } else if (!fallback_.load() && ++wft_fail_streak_ >= config_.wft_fail_limit) {
  //     // 스냅샷은 되는데 WaitForTick 만 계속 실패 (동기 모드·미지원 빌드)
  //     enter_fallback("연속 " + std::to_string(wft_fail_streak_) + " 회 실패");
  //   }
  // }
  // const SampleVerdict verdict = on_update(snapshot_buf_, wft_ok && wft_new_value);
  const SampleVerdict verdict = on_update(snapshot_buf_, wft_ok && wft_new_value);
  if (wft_tried) {
    const bool missed_update = !wft_timed_out || accepted(verdict);
    if (wft_ok) {
      wft_fail_streak_ = 0;
    } else if (
      missed_update && !fallback_.load() && ++wft_fail_streak_ >= config_.wft_fail_limit) {
      // 스냅샷은 새 업데이트를 보이는데 WaitForTick 은 계속 실패 (미지원 빌드·동기 모드에서 안 도는 빌드)
      enter_fallback("업데이트가 오는데 연속 " + std::to_string(wft_fail_streak_) + " 회 실패");
    }
  }

  if (!wft_tried || !wft_ok || fallback_.load()) {
    return;
  }
  // WaitForTick 이 돌아왔는데 새 업데이트가 없다 — 막지 않는 빌드면 루프가 RTT 속도로 헛돈다
  if (!wft_new_value || verdict == SampleVerdict::kDuplicate) {
    ++stats_->wft_no_update;
    if (++wft_dup_streak_ >= config_.wft_duplicate_limit) {
      enter_fallback(
        "성공했는데 새 업데이트가 없는 일이 연속 " + std::to_string(wft_dup_streak_) +
        " 회 (막지 않고 돌아오는 빌드·동기 모드?)");
    } else {
      sleep_ns(kWftNoUpdateSleepNs);
    }
    return;
  }
  if (verdict == SampleVerdict::kNoEgo || verdict == SampleVerdict::kInvalidEgo) {
    sleep_ns(kWftNoEgoSleepNs);  // Ego 가 없으면 중복을 가를 수 없다 — 속도만 묶는다
    return;
  }
  wft_dup_streak_ = 0;
}

void Engine::on_snapshot_failure(const mgv::Status & status)
{
  ++stats_->snapshot_failures;
  ++link_fail_streak_;
  if (status.deadline_exceeded()) {
    ++deadline_streak_;
  }
  RCLCPP_WARN_THROTTLE(
    logger_, throttle_clock_, 5000, "GetAllActorsState 실패 (연속 %d): %s — 발행·송신 건너뜀",
    link_fail_streak_, status.message.c_str());
  if (status.unavailable() || link_fail_streak_ >= config_.link_fail_limit) {
    // 연결은 됐는데 deadline 만 이어지면 TCP 가 소리 없이 죽은 것일 수 있다 → 채널 재생성
    // 26.09.25 _ YSH _ e2e(가짜 MORAI 서버 재기동) 반영: UNAVAILABLE 로 끊겨도 채널을 새로 만든다. 상태 채널만 끊김을 보고
    //   명령 채널은 옛 연결로 READY 에 남아 있어(실측), 재연결 직후 첫 명령 RPC(AUTO 재지정)가 'Socket closed' 로 실패했다
    // const bool recreate = deadline_streak_ >= config_.link_fail_limit;
    const bool recreate = status.unavailable() || deadline_streak_ >= config_.link_fail_limit;
    enter_connecting("스냅샷 실패: " + status.message, recreate);
    return;
  }
  if (status.rpc_ms < 1.0) {
    sleep_ns(5 * kMs);  // 즉시 실패가 이어져도 바쁜 루프가 되지 않게
  }
}

// =============================================================================
// 루프 본체
// =============================================================================

SampleVerdict Engine::on_update(const mgv::Snapshot & snapshot, bool tick_confirmed)
{
  const int64_t now = clock_.steady_ns();
  const SampleVerdict verdict = gate_.check(snapshot, now, tick_confirmed);
  if (const auto change = gate_.take_mode_change()) {
    gate_mode_ = static_cast<int>(gate_.mode());
    objects_next_due_ns_ = 0;  // 26.09.26 _ YSH _ stamp 가 MORAI 시각 ↔ 수신 시각으로 바뀌었다 — objects 스케줄을 새로
    if (gate_.mode() == GateMode::kContent) {
      ++stats_->content_mode_entries;
      RCLCPP_ERROR(
        logger_,
        "MORAI timestamp 를 쓸 수 없다 (%s) — Ego 내용 변화·트리거로 업데이트를 가르고 stamp 는 수신 시각을 쓴다 "
        "(time_source 무시). 이 MORAI 빌드를 `morai_probe all` 로 확인할 것",
        change->c_str());
    } else {
      RCLCPP_WARN(logger_, "%s", change->c_str());
    }
  }
  switch (verdict) {
    case SampleVerdict::kDuplicate:
      ++stats_->duplicates;
      break;
    case SampleVerdict::kBackward:
      ++stats_->backward;
      RCLCPP_WARN_THROTTLE(
        logger_, throttle_clock_, 2000,
        "MORAI timestamp 역행 (%ld < %ld us) — 건너뜀 (%d 표본 이어지거나 %.1f s 넘게 뒤로 가면 새 기준)",
        static_cast<long>(snapshot.timestamp_us),      // NOLINT(runtime/int)
        static_cast<long>(gate_.last_timestamp_us()),  // NOLINT(runtime/int)
        config_.gate.rebase_after_backward, static_cast<double>(config_.gate.rebase_jump_us) / 1e6);
      break;
    case SampleVerdict::kRebased:
      ++stats_->rebased;
      // 26.09.26 _ YSH _ 시각이 새 기준으로 뒤로 갔다 — objects 스케줄을 이 표본부터 (안 지우면 옛 예정 시각까지 최대 ~0.3 s 안 나간다)
      objects_next_due_ns_ = 0;
      RCLCPP_WARN(
        logger_, "MORAI timestamp 역행 — 새 기준(%ld us)으로 받아들인다 (MORAI 재시작·호스트 시계 계단?)",
        static_cast<long>(snapshot.timestamp_us));  // NOLINT(runtime/int)
      break;
    case SampleVerdict::kNoEgo:
      ++stats_->no_ego;
      break;
    case SampleVerdict::kInvalidEgo:
      ++stats_->invalid_ego;
      break;
    case SampleVerdict::kNew:
      break;
  }

  if (verdict == SampleVerdict::kNoEgo || verdict == SampleVerdict::kInvalidEgo) {
    if (!ego_missing_since_ns_) {
      ego_missing_since_ns_ = now;
    }
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 2000, "MORAI Ego %s — 발행·송신 건너뜀",
      verdict == SampleVerdict::kNoEgo ? "없음" : "전 필드 0 (미스폰·리셋 의심)");
    if (!spec_refresh_pending_.load() && now - *ego_missing_since_ns_ >= kEgoLostRefreshNs) {
      // 시나리오 교체로 다른 차가 나올 수 있다 → 돌아오면 스펙을 다시 읽는다. 연결 단계로는 가지 않는다
      // (스냅샷을 계속 읽어 Ego 가 보이는 즉시 발행을 재개하고, 모드 재지정도 하지 않는다)
      spec_refresh_pending_ = true;
      next_spec_refresh_ns_ = 0;
      RCLCPP_WARN(logger_, "Ego 가 1 s 넘게 없다 — 다시 보이면 스펙을 새로 읽고 그때까지 명령은 보류");
    }
    maybe_log_stats(now);
    return verdict;
  }
  ego_missing_since_ns_.reset();
  if (!accepted(verdict)) {
    maybe_log_stats(now);
    return verdict;
  }
  if (spec_refresh_pending_.load()) {
    try_refresh_spec(now);
  }

  if (verdict == SampleVerdict::kNew && gate_.last_step_us()) {
    const int64_t step_us = *gate_.last_step_us();
    update_interval_ms_.add(static_cast<double>(step_us) / 1000.0);
    if (step_us > kGapThresholdUs) {
      ++stats_->gaps_over_45ms;
    }
  }

  // 1) StateCache
  auto cached = std::make_shared<CachedState>();
  cached->ego = snapshot.ego;
  cached->timestamp_us = snapshot.timestamp_us;
  cached->receive_system_ns = snapshot.receive_system_ns;
  cached->receive_steady_ns = now;
  cached->update_index = update_index_;
  cache_.store(std::move(cached));
  last_valid_steady_ns_ = now;
  last_actor_count_ = snapshot.actors.size();
  ++stats_->updates;
  if (
    gate_.mode() == GateMode::kTimestamp && snapshot.receive_system_ns > 0 &&
    snapshot.timestamp_us > 0) {
    clock_offset_ms_.add(
      static_cast<double>(snapshot.timestamp_us * 1000 - snapshot.receive_system_ns) / 1e6);
  }

  const auto params = params_.get();
  // 2) ControlVehicle 정확히 1 회 (업데이트가 트리거)
  // 26.09.25 _ YSH _ command_trigger=timer 면 명령 타이머 스레드가 보낸다 — 여기서는 보내지 않는다 (이중 송신 금지)
  // send_command(snapshot, *params, now);
  if (params->command_trigger == CommandTrigger::kUpdate) {
    send_command(
      std::abs(snapshot.ego.velocity_kmh.x) * morai_to_autoware::kKmphToMps, *params, now);
  }
  // 3) 발행
  // 26.09.26 _ YSH _ DAQ 훅: 발행한 stamp 를 그대로 넘긴다 (DAQ stamp 가 kinematic_state 와 같게). 원문:
  // sink_.publish(build_outputs(snapshot, *params, now));
  const UpdateOutputs outputs = build_outputs(snapshot, *params, now);
  sink_.publish(outputs);
  sink_.on_sample(snapshot, outputs.stamp, update_index_, now);

  // 26.09.26 _ YSH _ 조회는 전용 스레드(run_mode_query)가 한다 — 이 동기 조회가 MORAI 다음 프레임까지 막혀 다음 업데이트를
  //   놓쳤다 (5 업데이트마다 1 개, 41.5 Hz). 여기서는 새 조회 결과가 있을 때만 AUTO 재지정을 판단한다. 원문:
  // if (update_index_ % kModeQueryEvery == 0) {
  //   query_control_mode();
  //   evaluate_reassert();
  // }
  if (const uint64_t seq = mode_query_seq_.load(); seq != reassert_seen_seq_) {
    reassert_seen_seq_ = seq;
    evaluate_reassert();
  }
  if (update_index_ % kDeadlineRefreshEvery == 0) {
    const auto rtt = snapshot_rtt_ms_.summary();
    if (rtt.count >= kDeadlineMinSamples) {
      refresh_command_deadline(rtt.p50);
    }
  }
  ++update_index_;
  maybe_log_stats(now);
  return verdict;
}

void Engine::try_refresh_spec(int64_t now)
{
  if (now < next_spec_refresh_ns_) {
    return;
  }
  next_spec_refresh_ns_ = now + kSpecRefreshRetryNs;
  const auto spec = link_.get_vehicle_spec();
  if (!spec.ok() || !(spec.value.max_steering_angle_deg > 0.0)) {
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 5000, "Ego 는 돌아왔지만 스펙 재조회 실패 (%s) — 명령 보류, 0.5 s 뒤 재시도",
      spec.ok() ? "max_steering_angle <= 0" : spec.status.message.c_str());
    return;
  }
  EgoVehicleSpec ego_spec;
  ego_spec.max_steering_angle =
    morai_to_autoware::max_steering_angle_rad(spec.value.max_steering_angle_deg);
  ego_spec.wheel_base = spec.value.wheel_base_m;
  {
    const std::lock_guard<std::mutex> lock(spec_mutex_);
    spec_ = ego_spec;
  }
  spec_refresh_pending_ = false;
  ++stats_->spec_refreshes;
  RCLCPP_INFO(
    logger_, "Ego 복귀 — 스펙 재조회: max_steering_angle %.4f rad (%.2f deg), wheel_base %.3f m",
    ego_spec.max_steering_angle, spec.value.max_steering_angle_deg, ego_spec.wheel_base);
  // 시나리오가 바뀌었을 수 있다 — 램프 재전송·센서 장착 (제어 모드는 건드리지 않는다)
  sink_.on_connected(true);
}

void Engine::refresh_command_deadline(double rtt_ms)
{
  const int64_t base = std::max<int64_t>(config_.command_deadline.count(), 1);
  const int64_t next = cmd_timeout_for_rtt_ms(base, std::max(rtt_ms, 0.0));
  const int64_t prev = command_deadline_ms_.exchange(next);
  const bool remote = rtt_ms > kRemoteRttMs;
  if (remote && !remote_rtt_warned_) {
    RCLCPP_WARN(
      logger_,
      "MORAI 왕복 p50 %.2f ms (> %.0f ms, 원격·느린 망) — ControlVehicle deadline %ld ms (설정 %ld ms, main.py "
      "cmd_timeout_for_rtt). 업데이트당 스냅샷·송신이 순차라 루프가 ~2.5×왕복보다 빨리 돌 수 없다",
      rtt_ms, kRemoteRttMs, static_cast<long>(next), static_cast<long>(base));  // NOLINT(runtime/int)
  } else if (next != prev) {
    RCLCPP_INFO(
      logger_, "ControlVehicle deadline %ld ms (MORAI 왕복 p50 %.2f ms, 설정 %ld ms)",
      static_cast<long>(next), rtt_ms, static_cast<long>(base));  // NOLINT(runtime/int)
  }
  remote_rtt_warned_ = remote;
}

// 26.09.26 _ YSH _ 원시 gRPC 기록: RPC 직전 system_clock 을 적고, 끝나면 보낸 명령·결과를 sink 에 넘긴다. 원문:
// mgv::Status Engine::transmit(const mgv::VehicleCommand & cmd)
// {
//   ++stats_->sends;
//   const mgv::Status st =
//     link_.control_vehicle(cmd, std::chrono::milliseconds(command_deadline_ms_.load()));
mgv::Status Engine::transmit(const mgv::VehicleCommand & cmd, bool stale)
{
  ++stats_->sends;
  const int64_t send_system_ns = clock_.system_ns();
  // 26.10.01 _ YSH _ 외부 조향 필터가 "우리가 보낸 값인가" 를 판정할 수 있게 RPC 전에 적어 둔다
  //   (응답을 기다리는 사이에 이 값이 적용된 프레임이 먼저 올 수 있다)
  sent_steer_.add(send_system_ns, cmd.steer);
  const mgv::Status st =
    link_.control_vehicle(cmd, std::chrono::milliseconds(command_deadline_ms_.load()));
  sink_.on_command_sent(cmd, stale, send_system_ns, st);
  if (st.grpc_code >= 0) {
    send_rtt_ms_.add(st.rpc_ms);
  }
  if (st.ok) {
    ++stats_->send_ok;
  } else if (st.deadline_exceeded()) {
    ++stats_->send_timeouts;
  } else if (st.grpc_code == 0) {
    ++stats_->send_result_failures;
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 10000, "ControlVehicle 응답 실패: %s", st.message.c_str());
  } else {
    ++stats_->send_errors;
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 10000, "ControlVehicle 실패: %s", st.message.c_str());
  }
  return st;
}

// 26.09.25 _ YSH _ 스냅샷 대신 속도 [m/s] 를 받는다 — update 는 이 업데이트 표본, timer 는 StateCache 최신 표본의 속도.
//   두 트리거가 이 한 경로로 만든다 (조향 라디안·보정·stale·NaN 은 슬롯에서 이미 걸렀다·forward·가드 감속 전부 같다).
//   cmd_mutex_ 를 잡는다 — 트리거를 런타임에 바꾸는 순간에만 두 스레드가 겹칠 수 있다
// void Engine::send_command(const mgv::Snapshot & snapshot, const RuntimeParams & params, int64_t now)
void Engine::send_command(double speed_mps, const RuntimeParams & params, int64_t now)
{
  const std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
  if (!params.forward_control_to_sim) {
    // main.py set_forwarding(False): 슬롯을 비운다 — 다시 켜도 새 control_cmd 가 와야 보낸다
    if (!slot_.empty()) {
      slot_.clear();
    }
    last_send_ns_ = 0;
    return;
  }
  const bool blocked = command_blocked_.load();
  if (blocked && !was_blocked_ && has_built_ && last_normal_send_ns_ > 0 &&
      now - last_normal_send_ns_ <= kShutdownActiveNs) {
    // 송신 중에 막혔다 — 그냥 멈추면 MORAI 가 마지막(가속일 수 있는) 명령을 유지한다
    block_brake_until_ns_ = now + kShutdownBrakeNs;
    RCLCPP_ERROR(
      logger_, "중복 writer — 감속 대체 명령을 %.1f s 보내고 ControlVehicle 을 멈춘다",
      static_cast<double>(kShutdownBrakeNs) / 1e9);
  }
  was_blocked_ = blocked;
  if (blocked) {
    last_send_ns_ = 0;
    if (now < block_brake_until_ns_) {
      ++stats_->block_brake_sends;
      // 26.09.26 _ YSH _ 가드 감속도 감속 대체 명령이다 (stale = true). 원문:
      // transmit(autoware_to_morai::make_stale_command(
      //   built_cmd_, autoware_to_morai::sanitize_stale_accel(config_.cmd_stale_accel),
      //   params.command.longitudinal));
      transmit(
        autoware_to_morai::make_stale_command(
          built_cmd_, autoware_to_morai::sanitize_stale_accel(config_.cmd_stale_accel),
          params.command.longitudinal),
        true);
      return;
    }
    ++stats_->blocked_sends;
    return;
  }
  block_brake_until_ns_ = 0;
  const auto spec = this->spec();
  const auto entry = slot_.latest();
  if (!spec || !entry || spec_refresh_pending_.load()) {
    last_send_ns_ = 0;  // 공백 구간은 최대 간격 계산에서 뺀다
    return;
  }

  // 새 메시지면 한 번만 만든다 (main.py 는 콜백에서 만들어 재송신기가 같은 객체를 다시 보냈다)
  if (!has_built_ || entry->seq != built_seq_) {
    // 26.09.25 _ YSH _ 속도는 인자로 받는다 (호출한 트리거가 고른 표본)
    // const double speed_mps = std::abs(snapshot.ego.velocity_kmh.x) * morai_to_autoware::kKmphToMps;
    const auto built =
      autoware_to_morai::build_command(entry->msg, speed_mps, spec->max_steering_angle, params.command);
    if (built.comp_table_invalid) {
      RCLCPP_WARN_THROTTLE(
        logger_, throttle_clock_, 10000,
        "steer_comp_speeds_mps / steer_comp_gains 가 잘못됐다(길이 불일치·오름차순 아님·배율 0.5~1.5 밖) — "
        "보정 안 함");
    }
    if (built.velocity_forced_zero) {
      RCLCPP_WARN_THROTTLE(
        logger_, throttle_clock_, 10000,
        "velocity 모드: 속도 단위가 off이거나 목표속도가 유한하지 않음 → 목표속도 0 송신");
    }
    // 모드가 바뀔 때만, 그리고 10 초에 한 번만 (main.py 와 같은 규칙)
    if (
      built.longitudinal_desc != long_mode_logged_ &&
      (long_mode_log_ns_ < 0 || now - long_mode_log_ns_ >= 10 * kSec)) {
      long_mode_logged_ = built.longitudinal_desc;
      long_mode_log_ns_ = now;
      RCLCPP_INFO(logger_, "종방향 명령 모드: %s", built.longitudinal_desc.c_str());
    }
    built_cmd_ = built.cmd;
    steer_comp_gain_ = built.steer_comp_gain;
    last_steer_cmd_ = built.cmd.steer;
    built_seq_ = entry->seq;
    has_built_ = true;
    // 26.09.25 _ YSH _ 통계 창은 엔진 스레드가 읽고 지운다 → window_mutex_
    // window_max_velocity_ = std::max(window_max_velocity_, std::abs(built.cmd.velocity));
    const std::lock_guard<std::mutex> window_lock(window_mutex_);
    window_max_velocity_ = std::max(window_max_velocity_, std::abs(built.cmd.velocity));
  }

  const int64_t age_ns = now - entry->receive_steady_ns;
  const bool stale = age_ns > static_cast<int64_t>(config_.cmd_stale_timeout_s * 1e9);
  const mgv::VehicleCommand payload =
    stale ? autoware_to_morai::make_stale_command(
              built_cmd_, autoware_to_morai::sanitize_stale_accel(config_.cmd_stale_accel),
              params.command.longitudinal)
          : built_cmd_;
  if (stale) {
    ++stats_->stale_sends;
    RCLCPP_WARN_THROTTLE(
      logger_, throttle_clock_, 2000,
      "control_cmd %.0f ms 끊김 (> %.0f ms) — 감속 대체 명령 송신 중 (조향은 마지막 값)",
      static_cast<double>(age_ns) / 1e6, config_.cmd_stale_timeout_s * 1000.0);
  }
  // 26.09.25 _ YSH _ 간격은 10 초 로그 창(window_mutex_)과 진단용 최근 500 개(send_gap_ms_) 둘에 남긴다
  // if (last_send_ns_ > 0) {
  //   window_max_gap_ms_ =
  //     std::max(window_max_gap_ms_, static_cast<double>(now - last_send_ns_) / 1e6);
  // }
  // last_send_ns_ = now;
  const int64_t prev_send_ns = last_send_ns_.exchange(now);
  if (prev_send_ns > 0) {
    const double gap_ms = static_cast<double>(now - prev_send_ns) / 1e6;
    send_gap_ms_.add(gap_ms);
    const std::lock_guard<std::mutex> window_lock(window_mutex_);
    window_max_gap_ms_ = std::max(window_max_gap_ms_, gap_ms);
  }

  // 26.09.26 _ YSH _ stale 을 원시 기록 훅에 넘긴다. 원문:
  // if (transmit(payload).ok && !stale) {
  if (transmit(payload, stale).ok && !stale) {
    last_normal_send_ns_ = now;
  }
}

// =============================================================================
// 26.09.25 _ YSH _ 명령 타이머 (command_trigger=timer) — main.py _CmdResender 와 같은 조건
// =============================================================================

void Engine::command_tick(const RuntimeParams & params, int64_t now)
{
  // 연결 단계(재연결·스펙 대기)면 보내지 않는다 — 막힌 링크에 100 Hz 로 두드리지 않고, 엔진의 재연결 백오프를 따른다.
  // 첫 유효 표본·스펙 전에도 보내지 않는다 (조향 보정 속도·최대 조향각을 모른다)
  const auto state = cache_.latest();
  if (phase_.load() != Phase::kRunning || !state || !spec()) {
    last_send_ns_ = 0;  // 공백 구간은 최대 간격 계산에서 뺀다
    return;
  }
  // 속도는 가장 최근에 받아들인 표본 (main.py 는 control_cmd 콜백 때 최신 Ego 속도로 만들었다)
  send_command(std::abs(state->ego.velocity_kmh.x) * morai_to_autoware::kKmphToMps, params, now);
}

void Engine::timer_step()
{
  const auto params = params_.get();
  if (params->command_trigger != CommandTrigger::kTimer) {
    if (timer_active_) {
      timer_active_ = false;
      RCLCPP_INFO(logger_, "command_trigger → update: 명령 타이머 송신을 멈춘다 (MORAI 업데이트당 1 회로)");
    }
    sleep_ns(kTimerIdlePollNs);
    return;
  }
  // on-set 콜백이 범위(10~400)를 지키지만 한 번 더 막는다
  const double hz = cmd_resend_hz_is_valid(params->cmd_resend_hz) ? params->cmd_resend_hz
                                                                   : kCmdResendHzDefault;
  const int64_t period = std::max<int64_t>(static_cast<int64_t>(std::llround(1e9 / hz)), 1);
  int64_t now = clock_.steady_ns();
  if (!timer_active_ || period != timer_period_ns_) {
    RCLCPP_INFO(
      logger_,
      "command_trigger=timer: 최신 명령을 %g Hz 고정 주기로 동기 송신 (main.py _CmdResender 방식 — 절대 deadline, 늦으면 "
      "몰아 보내지 않고 기준 재설정). 업데이트 루프는 송신하지 않는다",
      hz);
    timer_active_ = true;
    timer_period_ns_ = period;
    timer_next_ns_ = now;
    last_send_ns_ = 0;  // 트리거가 바뀐 틈은 간격 계산에서 뺀다
  }
  // main.py _run: delay = next − now 가 양수면 기다리고 다시 본다 (그사이 설정·stop 이 바뀌었을 수 있다)
  if (now < timer_next_ns_) {
    sleep_until_ns(timer_next_ns_);
    return;
  }
  try {
    command_tick(*params, now);
  } catch (const std::exception & e) {
    // 스레드가 죽으면 송신이 조용히 멈춘다 — 삼키고 계속한다 (main.py 와 같음)
    RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 5000, "명령 타이머 예외: %s", e.what());
  }
  timer_next_ns_ += timer_period_ns_;
  now = clock_.steady_ns();
  if (timer_next_ns_ < now) {
    // 뒤처짐 → 몰아서 따라잡지 않고 기준을 지금으로 (main.py late += 1)
    timer_next_ns_ = now;
    ++stats_->cmd_timer_late;
  }
}

void Engine::run_command_timer()
{
  while (!stop_requested()) {
    try {
      timer_step();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 5000, "명령 타이머 루프 예외: %s", e.what());
      sleep_ns(100 * kMs);
    }
  }
}

UpdateOutputs Engine::build_outputs(
  const mgv::Snapshot & snapshot, const RuntimeParams & params, int64_t now)
{
  using morai_to_autoware::kKmphToMps;
  const auto & ego = snapshot.ego;

  UpdateOutputs out;
  // timestamp 를 쓸 수 없는 빌드(내용 판정)면 time_source 와 무관하게 수신 시각
  const bool morai_time =
    config_.time_source == TimeSource::kMorai && gate_.mode() == GateMode::kTimestamp;
  const int64_t stamp_ns = morai_time ? snapshot.timestamp_us * 1000 : snapshot.receive_system_ns;
  out.stamp = morai_to_autoware::to_time_msg(stamp_ns);

  out.velocity = morai_to_autoware::to_velocity_report(ego);
  out.velocity.header.stamp = out.stamp;

  const bool moving = std::abs(ego.velocity_kmh.x) * kKmphToMps > 0.5;
  // 26.10.01 _ YSH _ 외부 조향 필터를 0 드롭아웃 유지 앞에 둔다. 우리가 보낸 값이면 −front_wheel_angle × 기준각이
  //   최근 송신 조향과 같다(09-29 실측 1e-6° 이내). 아니면 직전 우리 값으로 대신한다. 끄면 값이 그대로 지나간다. 원문:
  // const double fwa = steer_hold_.apply(
  //   ego.vehicle_state.front_wheel_angle, last_steer_cmd_, params.steer_report_hold, moving);
  const double applied_rad =
    -ego.vehicle_state.front_wheel_angle * params.steer_input_norm_deg * (M_PI / 180.0);
  const double fwa_in = foreign_filter_.apply(
    ego.vehicle_state.front_wheel_angle,
    sent_steer_.matches(applied_rad, snapshot.receive_system_ns), params.steer_report_ignore_foreign,
    moving);
  if (const auto log = foreign_filter_.take_log()) {
    if (foreign_filter_.tripped()) {
      RCLCPP_ERROR(logger_, "%s", log->c_str());
    } else {
      RCLCPP_WARN(logger_, "%s", log->c_str());
    }
  }
  const double fwa =
    steer_hold_.apply(fwa_in, last_steer_cmd_, params.steer_report_hold, moving);
  if (const auto log = steer_hold_.take_log()) {
    RCLCPP_WARN(logger_, "%s", log->c_str());
  }
  out.steering =
    morai_to_autoware::to_steering_report(fwa, spec().value_or(EgoVehicleSpec{}), steer_comp_gain_);
  out.steering.stamp = out.stamp;

  if (config_.publish_ground_truth) {
    out.has_ground_truth = true;
    out.odometry = morai_to_autoware::to_odometry(ego, params.map_origin);
    out.odometry.header.stamp = out.stamp;
    out.transform = morai_to_autoware::to_map_to_base_link(out.odometry.pose.pose, out.stamp);
  }

  // 26.09.26 _ YSH _ CARLA·simple_planning_simulator 처럼 차량 리포트는 한 표본에서 한 묶음 — 매 업데이트, 같은 stamp. 원문:
  // if (update_index_ % kGearModeEvery == 0) {
  {
    autoware_vehicle_msgs::msg::GearReport gear;
    gear.stamp = out.stamp;
    gear.report = morai_to_autoware::to_gear_report(ego.vehicle_state.gear_mode);
    out.gear = gear;

    // 조회가 kModeFreshS 넘게 실패하면 내지 않는다 (main.py: 조회 실패 → None → 보고 중단). 낡은
    // AUTONOMOUS 가 KEYBOARD 전환을 가리지 않게
    const int64_t mode_ok = mode_ok_steady_ns_.load();
    const auto mode = control_mode();
    if (mode && mode_ok > 0 && now - mode_ok <= static_cast<int64_t>(kModeFreshS * 1e9)) {
      autoware_vehicle_msgs::msg::ControlModeReport report;
      report.stamp = out.stamp;
      report.mode = morai_to_autoware::to_control_mode_report(*mode);
      out.control_mode = report;
      if (*mode == mgv::ControlMode::kCruise) {
        RCLCPP_WARN_THROTTLE(
          logger_, throttle_clock_, 10000,
          "MORAI 제어 모드 CRUISE — MORAI 가 스스로 운전 중 (제2 제어원). DISENGAGED(5) 로 보고");
      }
    }
  }

  // 26.09.26 _ YSH _ TrackedObjects 는 업데이트 개수가 아니라 MORAI 시각(stamp) 100 ms 스케줄 — MORAI 업데이트가 1 ms 쌍·
  //   40 ms 구멍으로 고르지 않거나 업데이트 수가 바뀌어도 10 Hz (CARLA 에서 perception 이 10 Hz 라이다로 내는 주기).
  //   예정 시각보다 반 프레임(kObjectsSlackNs) 이르게 온 표본부터 예정 것으로 낸다. 다음 예정은 격자 +100 ms 인데, 반 프레임
  //   넘게 늦게 온 표본(구멍 뒤)이면 그만큼 격자를 민다 — 다음 발행이 이번보다 최소 80 ms(주기 − 2 × 반 프레임) 뒤라 몰아
  //   내지 않는다. 평균 ~10 Hz(모의: 9.86~10.01 Hz), 간격 80 ms ~ 100 ms + MORAI 업데이트 간격. 반 주기 넘게 늦었거나(긴 정체)
  //   첫 표본·크게 역행한 시각이면 그 표본 기준으로 다시 잡는다 (on_update 가 시각 재기준·판정 방식 전환 때 0 으로 지운다). 원문:
  // if (config_.publish_ground_truth && update_index_ % kObjectsEvery == 0) {
  bool objects_due = false;
  if (config_.publish_ground_truth) {
    const int64_t due = objects_next_due_ns_;
    if (
      due == 0 || stamp_ns > due + kObjectsPeriodNs / 2 || stamp_ns < due - 2 * kObjectsPeriodNs) {
      objects_due = true;
      objects_next_due_ns_ = stamp_ns + kObjectsPeriodNs;
    } else if (stamp_ns >= due - kObjectsSlackNs) {
      objects_due = true;
      objects_next_due_ns_ =
        std::max(due + kObjectsPeriodNs, stamp_ns + kObjectsPeriodNs - kObjectsSlackNs);
    }
  }
  if (objects_due) {
    // 액터 목록은 Ego 와 같은 GetAllActorsState 에서 왔다 — 이 표본이 새것이면 목록도 새것이다
    autoware_perception_msgs::msg::TrackedObjects objects;
    objects.header.stamp = out.stamp;
    objects.header.frame_id = "map";
    // 26.11.10 _ YSH _ 외란 주입: 손잡이가 있으면 이 프레임에 Autoware 로 줄 actor 목록을 ObjectDisturber 가 만든다.
    //   스냅샷은 고치지 않는다 (GT DAQ 는 그대로). 숨김 프레임은 빈 목록이고, 그래도 out.objects 는 같은 주기로 낸다
    //   (하류 객체 주기 감시가 끊기지 않게). 손잡이가 없으면 예전 그대로 snapshot.actors 를 쓴다
    // 원문: objects.objects.reserve(snapshot.actors.size());
    // 원문: for (const auto & actor : snapshot.actors) {
    const std::vector<mgv::ActorSample> * actors = &snapshot.actors;
    if (disturbance_ != nullptr) {
      out.disturbance = disturber_.apply(*disturbance_, snapshot, stamp_ns);  // 설정은 프레임마다 한 번 읽는다
      actors = &out.disturbance->sent_actors;
      if (!out.disturbance->event.empty()) {
        RCLCPP_INFO(logger_, "외란: %s", out.disturbance->event.c_str());
      }
    }
    objects.objects.reserve(actors->size());
    for (const auto & actor : *actors) {
      // Snapshot.actors 는 이미 ego 로 고른 항목·id 가 정확히 ego_id("Ego") 인 항목을 뺐다 (main.py 와 같음)
      objects.objects.push_back(morai_to_autoware::to_tracked_object(actor, params.map_origin));
    }
    out.objects = std::move(objects);
    ++stats_->objects_published;
  }

  if (config_.publish_clock) {
    // /clock 은 메시지 stamp 와 같은 시각 (time_source·내용 판정을 따른다 — 둘이 어긋나지 않게)
    rosgraph_msgs::msg::Clock clock;
    clock.clock = out.stamp;
    out.clock = clock;
  }
  return out;
}

// 26.09.26 _ YSH _ 조회 스레드·엔진 스레드(연결 단계) 둘 다 부른다. RPC 는 락 밖, 저장은 mode_mutex_ 아래 — 조회 도중
//   엔진 스레드가 모드를 바꿨으면(끊김으로 지움·AUTO 설정) 이 결과는 낡았으니 버린다 (실패로도 세지 않는다). 원문:
// void Engine::query_control_mode()
// {
//   const auto r = link_.get_control_mode();
//   if (!r.ok()) {
//     ++stats_->mode_query_failures;
//     return;
//   }
//   mode_ok_steady_ns_ = clock_.steady_ns();
//   const int value = static_cast<int>(r.value);
//   const int prev = control_mode_.exchange(value);
//   if (prev != value) {
//     RCLCPP_INFO(
//       logger_, "MORAI 제어 모드 %s → %s", mode_name(prev).c_str(), mode_name(value).c_str());
//   }
// }
void Engine::query_control_mode(bool only_while_running)
{
  uint64_t epoch = 0;
  {
    const std::lock_guard<std::mutex> lock(mode_mutex_);
    // 26.09.26 _ YSH _ 리뷰 반영: 실행 중 확인과 세대 읽기를 한 락 안에서. enter_connecting 은 phase_ 를 먼저 바꾸고 이
    //   락 아래 세대를 올리므로, 새 세대를 본 조회는 반드시 kConnecting 도 본다 (락 밖에서 확인하면 그 사이 끊김이 끼어
    //   재연결 중에 조회하고 저장할 수 있었다). 연결 단계(connect_step)는 false 로 부른다
    if (only_while_running && phase_.load() != Phase::kRunning) {
      return;
    }
    epoch = mode_epoch_;
  }
  const auto r = link_.get_control_mode();
  int prev = -1;
  int value = -1;
  {
    const std::lock_guard<std::mutex> lock(mode_mutex_);
    if (mode_epoch_ != epoch) {
      return;
    }
    if (!r.ok()) {
      ++stats_->mode_query_failures;
      return;
    }
    mode_ok_steady_ns_ = clock_.steady_ns();
    value = static_cast<int>(r.value);
    prev = control_mode_.exchange(value);
    ++mode_query_seq_;
  }
  if (prev != value) {
    RCLCPP_INFO(
      logger_, "MORAI 제어 모드 %s → %s", mode_name(prev).c_str(), mode_name(value).c_str());
  }
}

// 26.09.26 _ YSH _ 엔진 스레드가 아는 모드를 직접 바꾼다 (끊김 → 모름, AUTO 설정 성공 → AUTO)
void Engine::overwrite_control_mode(int value, int64_t ok_steady_ns)
{
  const std::lock_guard<std::mutex> lock(mode_mutex_);
  ++mode_epoch_;
  control_mode_ = value;
  mode_ok_steady_ns_ = ok_steady_ns;
}

// 26.09.26 _ YSH _ 조회 한 번 (실행 중일 때만). 연결 단계는 connect_step 이 직접 조회한다
void Engine::mode_query_step()
{
  query_control_mode(/*only_while_running=*/true);
}

// 26.09.26 _ YSH _ 제어 모드 조회 전용 스레드 — 10 Hz 절대 deadline (늦으면 기준 재설정, 몰아 조회하지 않는다).
//   GetVehicleControlMode 는 MORAI 다음 프레임에야 돌아오므로(최대 ~20 ms) 엔진 루프에서 떼어 둔다
void Engine::run_mode_query()
{
  int64_t next = clock_.steady_ns();
  while (!stop_requested()) {
    try {
      mode_query_step();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(logger_, throttle_clock_, 5000, "제어 모드 조회 루프 예외: %s", e.what());
    }
    next += kModeQueryPeriodNs;
    const int64_t now = clock_.steady_ns();
    if (next < now) {
      next = now;
    }
    sleep_until_ns(next);
  }
}

void Engine::maybe_log_stats(int64_t now)
{
  if (stats_window_start_ns_ == 0) {
    stats_window_start_ns_ = now;
    stats_window_prev_ = stats();
    return;
  }
  const double dt = static_cast<double>(now - stats_window_start_ns_) / 1e9;
  if (now - stats_window_start_ns_ < kStatsPeriodNs) {
    return;
  }
  const EngineStats cur = stats();
  const EngineStats & p = stats_window_prev_;
  const uint64_t sends = cur.sends - p.sends;
  // main.py _CmdResender 10 초 통계 줄과 같은 항목·순서 (지연재설정은 프레임 고정이라 항상 0)
  // 26.09.25 _ YSH _ 명령 트리거 반영: timer 면 '목표 <hz> Hz'(main.py 와 같은 표기)·지연재설정 = 타이머 late, 끝에 트리거·
  //   업데이트당 송신 수. 창 최댓값은 타이머 스레드도 쓰므로 window_mutex_ 로 읽고 지운다. 원문:
  // RCLCPP_INFO(
  //   logger_,
  //   "제어 재송신 %.1fs: 송신 %lu (%.1f Hz, 목표 MORAI 업데이트당 1회) 응답성공 %lu 타임아웃 %lu 오류 %lu "
  //   "MORAI응답실패 %lu stale %lu 지연재설정 0 최대간격 %.1f ms · 명세25 velocity 최대 %.1f%s",
  //   dt, ul(sends), static_cast<double>(sends) / dt, ul(cur.send_ok - p.send_ok),
  //   ul(cur.send_timeouts - p.send_timeouts), ul(cur.send_errors - p.send_errors),
  //   ul(cur.send_result_failures - p.send_result_failures), ul(cur.stale_sends - p.stale_sends),
  //   window_max_gap_ms_, window_max_velocity_, command_blocked_.load() ? " · 중복 writer 로 차단 중" : "");
  double max_gap_ms = 0.0;
  double max_velocity = 0.0;
  {
    const std::lock_guard<std::mutex> window_lock(window_mutex_);
    max_gap_ms = window_max_gap_ms_;
    max_velocity = window_max_velocity_;
    window_max_gap_ms_ = 0.0;
    window_max_velocity_ = 0.0;
  }
  const auto rp = params_.get();
  const bool timer = rp->command_trigger == CommandTrigger::kTimer;
  char target[64];
  if (timer) {
    std::snprintf(target, sizeof(target), "%g Hz", rp->cmd_resend_hz);
  } else {
    std::snprintf(target, sizeof(target), "MORAI 업데이트당 1회");
  }
  const uint64_t window_updates = cur.updates - p.updates;
  RCLCPP_INFO(
    logger_,
    "제어 재송신 %.1fs: 송신 %lu (%.1f Hz, 목표 %s) 응답성공 %lu 타임아웃 %lu 오류 %lu "
    "MORAI응답실패 %lu stale %lu 지연재설정 %lu 최대간격 %.1f ms · 명세25 velocity 최대 %.1f · 명령 트리거 %s "
    "(업데이트당 %.2f)%s",
    dt, ul(sends), static_cast<double>(sends) / dt, target, ul(cur.send_ok - p.send_ok),
    ul(cur.send_timeouts - p.send_timeouts), ul(cur.send_errors - p.send_errors),
    ul(cur.send_result_failures - p.send_result_failures), ul(cur.stale_sends - p.stale_sends),
    ul(cur.cmd_timer_late - p.cmd_timer_late), max_gap_ms, max_velocity,
    to_string(rp->command_trigger),
    window_updates > 0 ? static_cast<double>(sends) / static_cast<double>(window_updates) : 0.0,
    command_blocked_.load() ? " · 중복 writer 로 차단 중" : "");
  const auto snap_rtt = snapshot_rtt_ms_.summary();
  const auto send_rtt = send_rtt_ms_.summary();
  const auto offset = clock_offset_ms_.summary();
  const uint64_t updates = cur.updates - p.updates;
  RCLCPP_INFO(
    logger_,
    "MORAI 갱신 %.1fs: 새 표본 %lu (%.1f Hz) 중복 %lu 역행 %lu Ego없음/무효 %lu/%lu 간격>45ms %lu · 스냅샷 "
    "RTT p50 %.2f p99 %.2f ms · 송신 RTT p50 %.2f p99 %.2f ms (deadline %ld ms) · MORAI−수신 p50 %.1f ms · "
    "트리거 %s · 판정 %s",
    dt, ul(updates), static_cast<double>(updates) / dt, ul(cur.duplicates - p.duplicates),
    ul(cur.backward - p.backward), ul(cur.no_ego - p.no_ego), ul(cur.invalid_ego - p.invalid_ego),
    ul(cur.gaps_over_45ms - p.gaps_over_45ms), snap_rtt.p50, snap_rtt.p99, send_rtt.p50,
    send_rtt.p99, static_cast<long>(command_deadline_ms_.load()),  // NOLINT(runtime/int)
    offset.p50, fallback_.load() ? "폴링 폴백" : "WaitForTick",
    gate_.mode() == GateMode::kTimestamp ? "timestamp" : "내용(timestamp 못 씀, stamp=수신 시각)");
  stats_window_prev_ = cur;
  stats_window_start_ns_ = now;
  // 26.09.25 _ YSH _ 창 최댓값은 위에서 window_mutex_ 아래 읽으며 지웠다
  // window_max_gap_ms_ = 0.0;
  // window_max_velocity_ = 0.0;
}

void Engine::shutdown_brake()
{
  // 26.09.25 _ YSH _ 송신 상태는 cmd_mutex_ 아래 (두 스레드가 멈춘 뒤라 다툼은 없지만 규칙대로)
  const std::lock_guard<std::mutex> cmd_lock(cmd_mutex_);
  if (phase_.load() != Phase::kRunning || !has_built_) {
    return;
  }
  const auto params = params_.get();
  if (!params->forward_control_to_sim || command_blocked_.load()) {
    return;
  }
  const int64_t now = clock_.steady_ns();
  if (last_normal_send_ns_ == 0 || now - last_normal_send_ns_ > kShutdownActiveNs) {
    return;  // 최근에 명령을 보내고 있지 않았다
  }
  const mgv::VehicleCommand brake = autoware_to_morai::make_stale_command(
    built_cmd_, autoware_to_morai::sanitize_stale_accel(config_.cmd_stale_accel),
    params->command.longitudinal);
  // 26.09.25 _ YSH _ timer 트리거면 감속도 같은 고정 주기로 (업데이트당이 아니라 cmd_resend_hz)
  const bool timer = params->command_trigger == CommandTrigger::kTimer;
  const double hz = cmd_resend_hz_is_valid(params->cmd_resend_hz) ? params->cmd_resend_hz
                                                                   : kCmdResendHzDefault;
  const int64_t timer_period_ns = static_cast<int64_t>(std::llround(1e9 / hz));
  int sent = 0;
  const int64_t end = now + kShutdownBrakeNs;
  // 26.09.25 _ YSH _ timer 면 절대 deadline 으로 주기를 맞춘다 (늦으면 기준 재설정, 몰아 보내지 않음)
  int64_t next = now;
  while (clock_.steady_ns() < end) {
    // const auto tick = link_.wait_for_tick(config_.wait_for_tick_deadline);
    // if (!tick.ok() && !tick.status.deadline_exceeded()) {
    //   // WaitForTick 이 안 되면 폴백 주기로 보낸다
    //   std::this_thread::sleep_for(std::chrono::nanoseconds(
    //     static_cast<int64_t>(1e9 / std::max(config_.fallback_hz, 1.0))));
    // }
    if (timer) {
      if (next >= end) {
        break;  // 0.3 s 창 끝 deadline 은 보내지 않는다 (100 Hz 면 정확히 30 회)
      }
      plain_sleep_ns(next - clock_.steady_ns());
      next += timer_period_ns;
      const int64_t after = clock_.steady_ns();
      if (next < after) {
        next = after;
      }
    } else {
      const auto tick = link_.wait_for_tick(config_.wait_for_tick_deadline);
      if (!tick.ok() && !tick.status.deadline_exceeded()) {
        // WaitForTick 이 안 되면 폴백 주기로 보낸다
        plain_sleep_ns(static_cast<int64_t>(1e9 / std::max(config_.fallback_hz, 1.0)));
      }
    }
    // 26.09.26 _ YSH _ 종료 감속도 원시 기록 훅에 넘긴다 (stale = true, 송신 통계에는 전처럼 넣지 않는다). 노드는 DAQ 를
    //   엔진보다 먼저 멈추므로 (shutdown 요약이 컨텍스트가 닫히기 전에 나가게) 토픽에는 나가지 않고 discarded 로 센다. 원문:
    // if (link_.control_vehicle(brake, std::chrono::milliseconds(command_deadline_ms_.load())).ok) {
    //   ++sent;
    // }
    const int64_t send_system_ns = clock_.system_ns();
    const mgv::Status st =
      link_.control_vehicle(brake, std::chrono::milliseconds(command_deadline_ms_.load()));
    sink_.on_command_sent(brake, true, send_system_ns, st);
    if (st.ok) {
      ++sent;
    }
  }
  // 26.09.25 _ YSH _ 트리거를 같이 적는다
  // RCLCPP_INFO(
  //   logger_, "종료 — 감속 대체 명령 %d 회 송신 (accel %.1f m/s^2, 조향은 마지막 값)", sent,
  //   brake.acceleration);
  RCLCPP_INFO(
    logger_, "종료 — 감속 대체 명령 %d 회 송신 (accel %.1f m/s^2, 조향은 마지막 값, 명령 트리거 %s)", sent,
    brake.acceleration, to_string(params->command_trigger));
}

// =============================================================================
// 조회
// =============================================================================

void Engine::note_requested_mode(mgv::ControlMode mode)
{
  requested_mode_ = static_cast<int>(mode);
}

void Engine::restore_requested_mode(std::optional<mgv::ControlMode> mode)
{
  requested_mode_ = mode ? static_cast<int>(*mode) : -1;
}

void Engine::set_command_block(bool blocked)
{
  command_blocked_ = blocked;
}

std::optional<EgoVehicleSpec> Engine::spec() const
{
  const std::lock_guard<std::mutex> lock(spec_mutex_);
  return spec_;
}

std::optional<mgv::ControlMode> Engine::control_mode() const
{
  const int v = control_mode_.load();
  if (v < 0) {
    return std::nullopt;
  }
  return static_cast<mgv::ControlMode>(v);
}

std::optional<double> Engine::control_mode_age_s() const
{
  const int64_t last = mode_ok_steady_ns_.load();
  if (last == 0) {
    return std::nullopt;
  }
  return static_cast<double>(clock_.steady_ns() - last) / 1e9;
}

std::optional<mgv::ControlMode> Engine::requested_mode() const
{
  const int v = requested_mode_.load();
  if (v < 0) {
    return std::nullopt;
  }
  return static_cast<mgv::ControlMode>(v);
}

EngineStats Engine::stats() const
{
  return stats_->load();
}

std::optional<double> Engine::sample_age_s() const
{
  const int64_t last = last_valid_steady_ns_.load();
  if (last == 0) {
    return std::nullopt;
  }
  return static_cast<double>(clock_.steady_ns() - last) / 1e9;
}

std::optional<double> Engine::command_age_s() const
{
  const auto entry = slot_.latest();
  if (!entry) {
    return std::nullopt;
  }
  return static_cast<double>(clock_.steady_ns() - entry->receive_steady_ns) / 1e9;
}

}  // namespace autoware::morai_interface
