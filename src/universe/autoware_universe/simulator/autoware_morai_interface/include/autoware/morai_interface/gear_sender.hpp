// 신규 — 26.09.25 _ YSH _ 기어 송신기 (main.py _GearSender 이식 — 변경 시 송신, 0.2 s 재시도, 1 s 재동기)
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

#ifndef AUTOWARE__MORAI_INTERFACE__GEAR_SENDER_HPP_
#define AUTOWARE__MORAI_INTERFACE__GEAR_SENDER_HPP_

#include <morai_grpc_vendor/morai_link_types.hpp>
#include <rclcpp/logger.hpp>

#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace autoware::morai_interface
{

struct GearSenderConfig
{
  double resync_s{1.0};  ///< main.py gear_resync_s — MORAI 실제 기어 불일치 재송신 간격
  double retry_s{0.2};   ///< main.py gear_retry_s — 같은 기어 실패 뒤 재시도 간격
};

/// 목표 기어를 바뀔 때만, 그리고 MORAI 실제 기어가 다를 때만 전용 스레드에서 보낸다 (main.py 와 같은 의미).
///
/// 슬롯 : 목표 기어 1 칸. executor 콜백이 submit 으로 넣고, 목표가 바뀐 경우에만 스레드를 깨운다.
/// 송신 : 목표 != 마지막 성공 기어면 보낸다. 같은 기어로 직전 시도가 실패했으면 retry_s 뒤에만 다시,
///        새 목표는 곧바로. 실패(타임아웃 포함)면 적용 여부를 모르므로 마지막 성공 기어를 비운다.
/// 재동기: 목표 == 마지막 성공이고 P/R/N/D 이며 마지막 시도 뒤 resync_s 가 지났는데 read()(MORAI 실제
///        기어, 엔진 StateCache)가 목표와 다르면 다시 보낸다 (키보드·맵 재로드로 바뀐 경우).
/// 차단 : set_forwarding(false) 면 목표와 마지막 성공을 비운다 → 송신·재동기 없음. 다시 켜도 다음
///        submit 전까지는 보내지 않는다.
/// main.py 와 다른 점: 성공 판정이 "예외 없음" 이 아니라 Result + 되읽기 일치다 (send 함수가 정한다).
///
/// tick() 은 한 번의 판단이다 (시험은 스레드 없이 tick 을 직접 부른다).
class GearSender
{
public:
  /// gear 는 MORAI GearMode 값 (0~5). ok = 적용 확인
  using SendFn = std::function<morai_grpc_vendor::Status(int gear)>;
  /// MORAI 실제 기어 (모르면 nullopt)
  using ReadFn = std::function<std::optional<int>()>;
  /// 단조 시계 [s]
  using ClockFn = std::function<double()>;

  static constexpr double kPollS = 0.05;        ///< 깨우지 않아도 이 주기로 재시도·재동기를 본다
  static constexpr double kWarnPeriodS = 10.0;  ///< 재동기·실패 경고 스로틀

  GearSender(
    SendFn send, ReadFn read, const GearSenderConfig & config, const rclcpp::Logger & logger,
    bool forwarding = true, ClockFn clock = ClockFn());
  ~GearSender();

  GearSender(const GearSender &) = delete;
  GearSender & operator=(const GearSender &) = delete;

  void start();
  void stop();

  void submit(int gear);
  void set_forwarding(bool on);
  void tick();

  struct Counters
  {
    uint64_t sends{0};
    uint64_t ok{0};
    uint64_t timeouts{0};
    uint64_t errors{0};
    uint64_t resyncs{0};
  };
  Counters counters() const;
  std::optional<int> target() const;
  std::optional<int> last_ok() const;

  /// 로그용 "D(4)" 형식 (main.py _name)
  static std::string name(std::optional<int> gear);

private:
  void run();
  void send(int gear, std::optional<int> prev, uint64_t epoch, bool resync);
  static bool is_resync_gear(int gear);

  SendFn send_fn_;
  ReadFn read_fn_;
  GearSenderConfig config_;
  rclcpp::Logger logger_;
  ClockFn clock_;

  mutable std::mutex mutex_;  // 아래 넷
  std::optional<int> target_;
  bool forwarding_;
  std::optional<int> last_ok_;
  uint64_t epoch_{0};

  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  bool wake_{false};
  bool stop_{false};
  std::thread thread_;

  // 스레드(tick)만 쓰는 상태
  double last_try_t_{-1e9};
  std::optional<int> fail_gear_;
  double resync_warn_t_{-1e9};
  double fail_warn_t_{-1e9};

  mutable std::mutex counters_mutex_;
  Counters counters_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__GEAR_SENDER_HPP_
