// 신규 — 26.09.25 _ YSH _ 보조 작업 스레드 (모드 요청·텔레포트·램프·센서 장착 — executor 를 막지 않는다)
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

#ifndef AUTOWARE__MORAI_INTERFACE__JOB_WORKER_HPP_
#define AUTOWARE__MORAI_INTERFACE__JOB_WORKER_HPP_

#include <rclcpp/logger.hpp>

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>

namespace autoware::morai_interface
{

/// 한 스레드가 넣은 순서대로 작업을 돈다. blocking RPC(최대 deadline + 되읽기)가 들어가는 일을
/// executor 콜백에서 떼어낸다. 큐가 max_queue 를 넘으면 post 가 false 를 돌려준다 (MORAI 가 멈췄을 때
/// 쌓이지 않게). 예외는 삼키고 로그만 남긴다.
class JobWorker
{
public:
  explicit JobWorker(const rclcpp::Logger & logger, size_t max_queue = 64);
  ~JobWorker();

  JobWorker(const JobWorker &) = delete;
  JobWorker & operator=(const JobWorker &) = delete;

  void start();
  /// 남은 작업은 버린다
  void stop();
  bool post(std::function<void()> job);
  size_t pending() const;

private:
  void run();

  rclcpp::Logger logger_;
  size_t max_queue_;
  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::deque<std::function<void()>> queue_;
  bool stop_{false};
  std::thread thread_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__JOB_WORKER_HPP_
