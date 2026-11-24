// 신규 — 26.09.25 _ YSH _ morai_probe monitor 의 순수 집계부 (gRPC 비의존, gtest 로 검증)
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

// Autoware 가 C++ 노드로 MORAI 를 모는 동안 MORAI 가 실제로 적용한 값(vehicle_state)을 서로 다른
// 업데이트마다 받아 진동·0 떨어짐·모드 뒤집힘을 센다. 판정 기준은 실측에서 왔다.
//   - 유휴 레코드: throttle 0, brake ~0.003 (main.py, morai_actuate IdleSignature 와 같은 값)
//   - 진동: 비동기 200 Hz 송신에서 throttle 0<->1 왕복, brake 20~43% 개입 (morai_actuate 실측)
//   - 조향 0 떨어짐: 명령이 못 들어간 프레임의 steer/front_wheel_angle = 0 (main.py 조향 홀드 필터)

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__MONITOR_STATS_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__MONITOR_STATS_HPP_

#include "probe_json.hpp"
#include "probe_stats.hpp"

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace morai_grpc_vendor::probe
{

struct MonitorThresholds
{
  double gap_ms{45.0};             ///< 업데이트 간격이 이보다 길면 gap
  /// throttle 히스테리시스: > throttle_on 이면 밟음, <= throttle_off 면 뗌, 그 사이는 직전 상태 유지.
  /// 등속 주행(가속 명령 ~0)에서 MORAI 가속 제어기가 0 근처를 오가는 잔떨림을 전환으로 세지 않는다
  double throttle_on{0.02};
  double throttle_off{1e-3};
  double brake_on{0.01};           ///< brake > 이 값 = 개입 (유휴 레코드 0.003 은 아래)
  double idle_throttle_max{1e-3};  ///< 유휴 레코드: throttle <= 이 값
  double idle_brake_center{0.003};
  double idle_brake_tol{0.0015};   ///< |brake - center| <= tol
  double steer_active{1e-3};       ///< |steer| > 이 값 = 조향 중
  double steer_zero{1e-9};         ///< |steer| <= 이 값 = 0 (float 0 그대로)
  size_t max_drop_run{5};          ///< 이 길이(업데이트 수) 이하의 0 구간만 "떨어짐" 으로 센다
  /// throttle 밟음<->뗌 전환이 초당 이 이상이면 진동 의심 — 판정이 아니라 힌트다 (실측 진동은
  /// 업데이트마다 0<->1 로 뒤집혔다 ~25/s, 정상 주행의 가감속 전환은 몇 초에 한 번. 등속 주행에서의
  /// 오탐률은 아직 실측 전이다)
  double oscillation_toggles_per_s{2.0};
};

/// 서로 다른 업데이트(ActorStates.timestamp 변화) 하나
struct MonitorSample
{
  double t_s{0.0};                ///< 로컬 steady, 시작부터 [s] (응답 도착)
  int64_t timestamp_us{0};        ///< MORAI ActorStates.timestamp
  double recv_minus_ts_ms{0.0};   ///< 로컬 수신 system 시각 - timestamp (프레임 나이 + 시계 차)
  bool ego_found{true};
  bool ego_valid{true};
  double speed_mps{0.0};          ///< velocity.x km/h / 3.6 (main.py 와 같음)
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double front_wheel_angle{0.0};
  int gear{-1};
  size_t collisions{0};
};

/// 즉시 출력·JSON 용 사건
struct MonitorEvent
{
  enum class Kind { kModeChange, kGearChange, kCollision, kTimestampBackwards, kEgoLost, kEgoBack };
  double t_s{0.0};
  Kind kind{Kind::kModeChange};
  int64_t from{0};
  int64_t to{0};
};
std::string to_string(MonitorEvent::Kind k);

/// 한 창(또는 전체) 집계
struct MonitorWindow
{
  double t_begin_s{0.0};
  double t_end_s{0.0};
  size_t updates{0};
  double updates_per_s{0.0};  ///< updates / (t_end - t_begin)
  size_t polls{0};            ///< 스냅샷 호출 성공 수 (중복 포함)
  size_t rpc_failures{0};

  size_t ts_gaps{0};    ///< MORAI timestamp 간격 > gap_ms
  size_t wall_gaps{0};  ///< 수신 간격 > gap_ms
  double max_ts_gap_ms{0.0};
  double max_wall_gap_ms{0.0};
  size_t ts_backwards{0};

  size_t throttle_toggles{0};   ///< throttle 밟음 <-> 뗌 전환 (히스테리시스 적용)
  size_t brake_engagements{0};  ///< brake 미개입 -> 개입 전환
  size_t brake_on_updates{0};
  size_t idle_records{0};
  size_t steer_zero_drops{0};
  size_t fwa_zero_drops{0};
  size_t longest_drop_run{0};

  size_t mode_polls{0};
  size_t mode_poll_failures{0};
  size_t mode_changes{0};
  int last_mode{-1};
  size_t gear_changes{0};
  int last_gear{-1};

  size_t collision_updates{0};
  size_t collision_events{0};
  size_t ego_missing{0};
  size_t ego_invalid{0};

  double speed_min_mps{0.0};
  double speed_max_mps{0.0};
  double speed_last_mps{0.0};
  Summary recv_minus_ts_ms;  ///< 같은 PC 면 프레임 나이, 원격이면 나이 - (MORAI - 로컬) 시계 차
  Summary wall_gap_ms;
  /// -min(recv - ts) = max(ts - recv): MORAI - 로컬 오프셋의 하한 [ms] (업데이트가 없으면 NaN).
  /// timestamp 는 프레임 시각이라 참 오프셋보다 최소 프레임 나이만큼 낮다 — 같은 PC 에서 잰 값이
  /// 기준선이고, 원격 오프셋 ~= 원격 값 - 기준선 (morai_link_types.hpp "시계 오프셋")
  double clock_bound_ms{0.0};

  double brake_on_share() const;  ///< brake_on_updates / updates (0 이면 NaN)
};

/// 창 안 throttle 전환 빈도가 oscillation_toggles_per_s 이상 (힌트)
bool looks_oscillating(const MonitorWindow & w, const MonitorThresholds & th);

Json to_json(const MonitorWindow & w);
Json to_json(const MonitorEvent & e);

/// 업데이트를 넣으면 창·전체를 함께 센다. 상태 기계(직전 값, 0 구간)는 창을 넘어 이어진다.
class MonitorStats
{
public:
  explicit MonitorStats(MonitorThresholds th = {}, double t_start_s = 0.0);

  /// 서로 다른 업데이트 하나
  void add_update(const MonitorSample & s);
  /// 스냅샷 호출 결과 (ok 면 중복이어도 poll 로 센다)
  void add_poll(bool ok);
  /// 제어 모드 조회 결과 (~0.5 s 마다)
  void add_mode(double t_s, bool ok, int mode);

  /// 현재 창을 닫아 돌려주고 새 창을 now_s 부터 연다
  MonitorWindow take_window(double now_s);
  /// 지금까지 전체 (창을 닫지 않는다)
  MonitorWindow total(double now_s) const;

  /// 아직 출력하지 않은 사건을 꺼낸다 (전체 목록은 events())
  std::vector<MonitorEvent> drain_new_events();
  const std::vector<MonitorEvent> & events() const { return events_; }

  const MonitorThresholds & thresholds() const { return th_; }

private:
  struct Acc
  {
    MonitorWindow w;
    std::vector<double> age_ms;
    std::vector<double> gap_ms;
    bool speed_seen{false};
  };
  struct ZeroDrop
  {
    bool armed{false};  ///< 0 구간 앞 마지막 비0 값이 조향 중이었다
    size_t run{0};
  };

  void each(Acc * a, const MonitorSample & s);
  /// 0 떨어짐이 끝났으면 그 구간 길이, 아니면 0
  size_t step_drop(ZeroDrop * z, double v) const;
  static MonitorWindow finish(const Acc & a, double now_s);
  void event(double t, MonitorEvent::Kind k, int64_t from, int64_t to);

  MonitorThresholds th_;
  Acc win_;
  Acc all_;

  bool have_prev_{false};
  MonitorSample prev_{};
  bool prev_pressed_{false};
  bool prev_braking_{false};
  ZeroDrop steer_drop_;
  ZeroDrop fwa_drop_;
  bool have_mode_{false};
  int mode_{-1};
  bool ego_lost_{false};

  std::vector<MonitorEvent> events_;
  size_t drained_{0};
};

}  // namespace morai_grpc_vendor::probe

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__TOOLS__MONITOR_STATS_HPP_
