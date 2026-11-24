// 신규 — 26.09.25 _ YSH _ JobWorker 구현
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

#include "autoware/morai_interface/job_worker.hpp"

#include <rclcpp/logging.hpp>

#include <exception>
#include <utility>

namespace autoware::morai_interface
{

JobWorker::JobWorker(const rclcpp::Logger & logger, size_t max_queue)
: logger_(logger), max_queue_(max_queue == 0 ? 1 : max_queue)
{
}

JobWorker::~JobWorker()
{
  stop();
}

void JobWorker::start()
{
  if (thread_.joinable()) {
    return;
  }
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_ = false;
  }
  thread_ = std::thread([this]() { run(); });
}

void JobWorker::stop()
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    queue_.clear();
  }
  cv_.notify_all();
  if (thread_.joinable()) {
    thread_.join();
  }
}

bool JobWorker::post(std::function<void()> job)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (stop_ || queue_.size() >= max_queue_) {
      return false;
    }
    queue_.push_back(std::move(job));
  }
  cv_.notify_one();
  return true;
}

size_t JobWorker::pending() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return queue_.size();
}

void JobWorker::run()
{
  for (;;) {
    std::function<void()> job;
    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait(lock, [this]() { return stop_ || !queue_.empty(); });
      if (stop_) {
        return;
      }
      job = std::move(queue_.front());
      queue_.pop_front();
    }
    try {
      job();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "작업 스레드 예외: %s", e.what());
    } catch (...) {
      RCLCPP_ERROR(logger_, "작업 스레드 예외 (std::exception 아님)");
    }
  }
}

}  // namespace autoware::morai_interface
