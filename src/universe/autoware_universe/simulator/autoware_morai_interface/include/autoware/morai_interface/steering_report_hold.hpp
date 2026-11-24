// 신규 — 26.09.25 _ YSH _ 조향 리포트 0 드롭아웃 유지 필터 (main.py publish_steering_report 의 steer_report_hold)
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

#ifndef AUTOWARE__MORAI_INTERFACE__STEERING_REPORT_HOLD_HPP_
#define AUTOWARE__MORAI_INTERFACE__STEERING_REPORT_HOLD_HPP_

// 26.10.01 _ YSH _ 외부 조향 필터(SentSteerHistory·ForeignSteerFilter) 추가로 <array>·<cstdint>·<mutex>. 원문:
// #include <optional>
// #include <string>
#include <array>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>

namespace autoware::morai_interface
{

/// MORAI 는 명령이 없는 프레임에 front_wheel_angle = 0.0 인 유휴 레코드를 끼워 넣는다 (main.py 실측
/// 35~39%, forward_control_to_sim=false 여도 남음). 그대로 내보내면 조향 피드백이 실제값과 0 을 오가
/// 횡방향 제어기가 진동한다. 0 이 왔는데 마지막으로 보낸 조향이 1e-3 rad 를 넘으면 직전 정상값을 쓴다.
///
/// 이 빌드에서는 명령이 업데이트 사이에 유지돼(실측) 0 레코드가 드물 것으로 보지만, 규칙은 main.py 와
/// 똑같이 둔다 — 주행 표본 500 개마다 유지 비율을 WARN 으로 남겨 실제로 몇 번 걸렸는지 보인다.
class SteeringReportHold
{
public:
  /// front_wheel_angle 원값 -> 발행에 쓸 값. moving = |v| > 0.5 m/s (통계만 주행 중 기준)
  double apply(double front_wheel_angle, double last_steer_cmd, bool enabled, bool moving);

  /// 주행 표본 500 개가 찼고 그동안 유지한 적이 있으면 main.py 와 같은 WARN 문구를 한 번 돌려준다
  std::optional<std::string> take_log();

  int hold_count() const { return hold_count_; }
  int report_count() const { return report_count_; }
  double last_good() const { return last_good_; }

  static constexpr int kLogEvery = 500;

private:
  double last_good_{0.0};
  int report_count_{0};
  int hold_count_{0};
  std::optional<std::string> pending_log_;
};

// ============================================================================
// 26.10.01 _ YSH _ 신규 — 외부 조향 필터 (steer_report_ignore_foreign, 기본 끔)
//
// 09-29 시나리오 44런 실측: MORAI 가 Ego 에 적용했다고 알려준 조향의 11.5 % 가 우리가 보낸 적 없는 값이었다
// (MORAI 시나리오가 도는 동안만, MGeo 차선 추종 조향과 일치). front_wheel_angle 은 입력 steer 와 모든 프레임에서
// 같아서, 그 프레임의 조향 리포트가 그대로 튄다. 이 필터는 그런 프레임의 리포트를 직전 우리 값으로 대신한다.
// 없애는 것은 Autoware 가 튄 리포트에 반응하는 몫뿐이다 — 그 프레임에 차가 MORAI 안에서 꺾이는 것은 그대로다.
// 판정 규칙은 Morai_Autoware_Bridge/scripts/daq/input_source.py 와 같다.
// ============================================================================

/// 최근에 MORAI 로 보낸 조향 [rad]. 송신 스레드(엔진 루프·명령 타이머)가 넣고 발행(엔진 스레드)이 읽는다
class SentSteerHistory
{
public:
  /// ControlVehicle 직전에 부른다. system_ns = RPC 직전 system_clock (raw 기록의 send_system_ns 와 같은 값)
  void add(int64_t system_ns, double steer_rad);

  /// applied_rad 가 [rx_system_ns - kWindowNs, rx_system_ns] 의 송신 중 하나와 kExactRad 안으로 같으면 true.
  /// 그 창에 송신이 하나도 없으면 nullopt (판정 불가 — 호출자는 통과시킨다)
  std::optional<bool> matches(double applied_rad, int64_t rx_system_ns) const;

  static constexpr size_t kSize = 64;                  ///< 300 ms × 100 Hz = 30 개의 두 배
  static constexpr int64_t kWindowNs = 300'000'000;    ///< 우리 송신을 찾는 창
  static constexpr double kExactRad = 1.7453292519943296e-6;  ///< 0.0001° — 같은 값으로 볼 허용 오차

private:
  struct Entry
  {
    int64_t system_ns{0};
    double steer_rad{0.0};
  };
  mutable std::mutex mutex_;
  std::array<Entry, kSize> ring_{};
  size_t head_{0};
  size_t count_{0};
};

/// 외부 값이 온 프레임의 front_wheel_angle 을 직전 우리 값으로 대신한다 (엔진 스레드만 쓴다)
class ForeignSteerFilter
{
public:
  /// front_wheel_angle 원값 -> 발행에 쓸 값.
  ///   ours: SentSteerHistory::matches 결과 (nullopt = 판정 불가 → 통과)
  ///   enabled = steer_report_ignore_foreign, moving = |v| > 0.5 m/s (통계·자기 차단은 주행 중 기준)
  double apply(double front_wheel_angle, std::optional<bool> ours, bool enabled, bool moving);

  /// 남길 로그가 있으면 한 번 돌려준다 (유지 비율 · 상한 초과 · 자기 차단)
  std::optional<std::string> take_log();

  bool tripped() const { return tripped_; }
  int streak() const { return streak_; }
  int held_count() const { return held_count_; }
  int judged_count() const { return judged_count_; }

  static constexpr int kLogEvery = 500;     ///< 주행 판정 표본 이만큼마다 통계·자기 차단 판단
  static constexpr int kMaxHold = 25;       ///< 연속 유지 상한 [프레임] (09-29 최장 19 프레임, 50 Hz 에서 0.5 s)
  static constexpr double kTripOwnRatio = 0.2;  ///< 우리 값 일치가 이 비율 아래면 정규화 기준이 안 맞는 것 → 스스로 끈다

private:
  double last_own_{0.0};
  bool has_own_{false};
  int streak_{0};
  bool streak_logged_{false};
  bool tripped_{false};
  bool was_enabled_{false};
  int judged_count_{0};
  int own_count_{0};
  int held_count_{0};
  std::optional<std::string> pending_log_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__STEERING_REPORT_HOLD_HPP_
