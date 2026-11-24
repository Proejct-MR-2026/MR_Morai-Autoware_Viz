// 신규 — 26.09.25 _ YSH _ GearSender 구현 (main.py _GearSender 와 같은 순서·같은 로그)
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

#include "autoware/morai_interface/gear_sender.hpp"

#include <rclcpp/logging.hpp>

#include <algorithm>
#include <chrono>
#include <exception>
#include <string>
#include <utility>

namespace autoware::morai_interface
{

namespace
{
double steady_seconds()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

GearSender::GearSender(
  SendFn send, ReadFn read, const GearSenderConfig & config, const rclcpp::Logger & logger,
  bool forwarding, ClockFn clock)
: send_fn_(std::move(send)),
  read_fn_(std::move(read)),
  config_(config),
  logger_(logger),
  clock_(clock ? std::move(clock) : ClockFn(steady_seconds)),
  forwarding_(forwarding)
{
  config_.resync_s = std::max(config_.resync_s, 0.0);
  config_.retry_s = std::max(config_.retry_s, 0.0);
}

GearSender::~GearSender()
{
  stop();
}

void GearSender::start()
{
  if (thread_.joinable()) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(wake_mutex_);
    stop_ = false;
  }
  thread_ = std::thread([this]() { run(); });
}

void GearSender::stop()
{
  {
    const std::lock_guard<std::mutex> lock(wake_mutex_);
    stop_ = true;
    wake_ = true;
  }
  wake_cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void GearSender::submit(int gear)
{
  bool changed = false;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    changed = !target_ || *target_ != gear || !forwarding_;
    forwarding_ = true;
    target_ = gear;
  }
  if (changed) {
    {
      const std::lock_guard<std::mutex> lock(wake_mutex_);
      wake_ = true;
    }
    wake_cv_.notify_all();
  }
}

void GearSender::set_forwarding(bool on)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (!on) {
    ++epoch_;
    target_.reset();
    last_ok_.reset();
  }
  forwarding_ = on;
}

void GearSender::run()
{
  for (;;) {
    {
      std::unique_lock<std::mutex> lock(wake_mutex_);
      wake_cv_.wait_for(
        lock, std::chrono::duration<double>(kPollS), [this]() { return wake_ || stop_; });
      if (stop_) {
        return;
      }
      wake_ = false;
    }
    try {
      tick();
    } catch (const std::exception & e) {
      // 스레드가 죽으면 기어가 조용히 안 바뀌므로 무엇이든 삼킨다 (main.py 와 같음)
      RCLCPP_ERROR(logger_, "기어 송신 루프 예외: %s", e.what());
    }
  }
}

bool GearSender::is_resync_gear(int gear)
{
  // MORAI GearMode P/R/N/D (M=0·L=5 는 재동기 안 함)
  return gear == 1 || gear == 2 || gear == 3 || gear == 4;
}

void GearSender::tick()
{
  std::optional<int> target;
  std::optional<int> last_ok;
  uint64_t epoch = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!forwarding_) {
      return;
    }
    target = target_;
    last_ok = last_ok_;
    epoch = epoch_;
  }
  if (!target) {
    return;
  }
  const double now = clock_();
  if (target != last_ok) {
    if (fail_gear_ == target && now - last_try_t_ < config_.retry_s) {
      return;  // 같은 기어 실패 직후 — retry_s 기다림
    }
    send(*target, last_ok, epoch, false);
    return;
  }
  if (!is_resync_gear(*target) || now - last_try_t_ < config_.resync_s) {
    return;
  }
  std::optional<int> actual;
  try {
    actual = read_fn_ ? read_fn_() : std::nullopt;
  } catch (...) {
    actual.reset();
  }
  if (!actual || *actual == *target) {
    return;
  }
  uint64_t resyncs = 0;
  {
    const std::lock_guard<std::mutex> lock(counters_mutex_);
    resyncs = ++counters_.resyncs;
  }
  if (now - resync_warn_t_ >= kWarnPeriodS) {
    resync_warn_t_ = now;
    RCLCPP_WARN(
      logger_, "MORAI 기어 %s ≠ 목표 %s — 재송신 (누적 재동기 %lu)", name(actual).c_str(),
      name(target).c_str(), static_cast<unsigned long>(resyncs));  // NOLINT(runtime/int)
  }
  send(*target, actual, epoch, true);
}

void GearSender::send(int gear, std::optional<int> prev, uint64_t epoch, bool resync)
{
  {
    const std::lock_guard<std::mutex> lock(counters_mutex_);
    ++counters_.sends;
  }
  const double t0 = clock_();
  morai_grpc_vendor::Status status;
  try {
    status = send_fn_(gear);
  } catch (const std::exception & e) {
    status.ok = false;
    status.message = e.what();
  }
  const double now = clock_();
  last_try_t_ = now;
  if (!status.ok) {
    fail_gear_ = gear;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (epoch_ == epoch) {
        last_ok_.reset();  // 타임아웃이면 적용됐는지 모른다
      }
    }
    std::string what;
    Counters c;
    {
      const std::lock_guard<std::mutex> lock(counters_mutex_);
      if (status.deadline_exceeded()) {
        ++counters_.timeouts;
        what = "타임아웃";
      } else {
        ++counters_.errors;
        what = status.message.substr(0, 160);
      }
      c = counters_;
    }
    if (now - fail_warn_t_ >= kWarnPeriodS) {
      fail_warn_t_ = now;
      RCLCPP_WARN(
        logger_, "기어 송신 %s 실패: %s — %g s 뒤 재시도 (누적 타임아웃 %lu 오류 %lu)",
        name(gear).c_str(), what.c_str(), config_.retry_s,
        static_cast<unsigned long>(c.timeouts),  // NOLINT(runtime/int)
        static_cast<unsigned long>(c.errors));   // NOLINT(runtime/int)
    }
    return;
  }
  fail_gear_.reset();
  {
    const std::lock_guard<std::mutex> lock(counters_mutex_);
    ++counters_.ok;
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (epoch_ == epoch) {
      last_ok_ = gear;
    }
  }
  if (!resync) {
    RCLCPP_INFO(
      logger_, "기어 송신 %s→%s rtt %.0f ms", name(prev).c_str(), name(gear).c_str(),
      (now - t0) * 1000.0);
  }
}

GearSender::Counters GearSender::counters() const
{
  const std::lock_guard<std::mutex> lock(counters_mutex_);
  return counters_;
}

std::optional<int> GearSender::target() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return target_;
}

std::optional<int> GearSender::last_ok() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return last_ok_;
}

std::string GearSender::name(std::optional<int> gear)
{
  if (!gear) {
    return "?";
  }
  static const char * kNames[] = {"M", "P", "R", "N", "D", "L"};
  const char * n = (*gear >= 0 && *gear <= 5) ? kNames[*gear] : "?";
  return std::string(n) + "(" + std::to_string(*gear) + ")";
}

}  // namespace autoware::morai_interface
