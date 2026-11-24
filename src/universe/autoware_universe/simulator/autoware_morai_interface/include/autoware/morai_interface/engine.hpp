// 신규 — 26.09.25 _ YSH _ 프레임 고정 엔진 — MORAI 업데이트마다 스냅샷 1 회, 명령 1 회, 발행 1 회
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

// 설계 근거 (실측, morai-time-facts):
//  - MORAI 상태는 targetFPS 와 무관하게 초당 ~50 번 바뀐다. GetAllActorsState 한 번이면 Ego+NPC+timestamp.
//  - WaitForTick 은 실시간 모드에서도 업데이트마다 한 번 돌아온다 (값은 다른 시계 → 트리거로만).
//  - 비동기 200 Hz 로 몰아 보내면(렌더 프레임당 3 개 이상) MORAI 가속 제어기가 진동했다 → 업데이트당
//    ControlVehicle 을 정확히 한 번, 업데이트가 트리거한다.
// 루프 본체 on_update(snapshot) 는 트리거와 떨어져 있다 — 이후 lockstep(Tick) 드라이버가 트리거만
// 바꿔 끼우면 된다. 엔진은 ILink·EngineSink 에만 의존해 네트워크·ROS 없이 gtest 로 돈다.
// 26.09.25 _ YSH _ A/B 용 명령 트리거 추가 (command_trigger). 위 "업데이트당 1 회"(update, 기본) 외에
//   timer = main.py _CmdResender 와 같은 조건 — 전용 명령 스레드가 최신 명령을 cmd_resend_hz(기본 100 Hz) 고정 주기로
//   동기 송신한다 (절대 deadline, 늦으면 몰아 보내지 않고 기준을 지금으로). 명령을 만드는 경로(send_command)는 같고,
//   timer 에서는 업데이트 루프가 보내지 않는다. 현장 폐루프에서 C++ 노드(update)일 때 MORAI 적용 throttle 이 가속 중
//   0↔on 을 초당 ~4 번 오갔고, 알려진 정상 조건은 MORAI 기본 ~50 Hz + main.py 100 Hz 였다 — 둘을 같은 노드에서 가른다.

#ifndef AUTOWARE__MORAI_INTERFACE__ENGINE_HPP_
#define AUTOWARE__MORAI_INTERFACE__ENGINE_HPP_

#include "autoware/morai_interface/command_slot.hpp"
#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"
#include "autoware/morai_interface/ego_vehicle_spec.hpp"
#include "autoware/morai_interface/link_interface.hpp"
#include "autoware/morai_interface/map_origin.hpp"
// 26.11.10 _ YSH _ 외란 주입 (객체 프레임마다 ObjectDisturber 가 Autoware 로 줄 목록을 만든다)
#include "autoware/morai_interface/object_disturbance.hpp"
#include "autoware/morai_interface/sample_gate.hpp"
#include "autoware/morai_interface/steering_report_hold.hpp"

#include <morai_grpc_vendor/morai_link_types.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace autoware::morai_interface
{

/// 발행 stamp 의 출처
enum class TimeSource {
  kMorai,    ///< ActorStates.timestamp (MORAI 샘플 시각, 실시간 모드에선 MORAI 호스트 epoch)
  kReceive,  ///< 응답 도착 시각 (이 PC system_clock)
};
std::optional<TimeSource> parse_time_source(const std::string & text);
const char * to_string(TimeSource source);

// 26.09.25 _ YSH _ ControlVehicle 을 무엇이 트리거하나 (A/B: 업데이트당 1 회 vs main.py 식 고정 주기)
enum class CommandTrigger {
  kUpdate,  ///< MORAI 업데이트(받아들인 새 표본)마다 엔진 루프가 1 회 (기본)
  kTimer,   ///< 전용 명령 스레드가 cmd_resend_hz 고정 주기로 최신 명령을 동기 송신 (main.py _CmdResender)
};
std::optional<CommandTrigger> parse_command_trigger(const std::string & text);
const char * to_string(CommandTrigger trigger);

// 26.09.25 _ YSH _ cmd_resend_hz (main.py 와 같은 이름·기본값) 유효 범위
inline constexpr double kCmdResendHzDefault = 100.0;
inline constexpr double kCmdResendHzMin = 10.0;
inline constexpr double kCmdResendHzMax = 400.0;
/// 유효(유한, 10~400)하면 true
bool cmd_resend_hz_is_valid(double hz);

/// 기동 시 한 번 읽는 값 (런타임에 바꾸지 않는다)
struct EngineConfig
{
  std::chrono::milliseconds wait_for_tick_deadline{100};
  /// ControlVehicle deadline 의 하한 (morai.command_deadline_ms). 실제 값은 max(이 값, 4 × 스냅샷 왕복
  /// p50), 상한 max(1 s, 이 값) — main.py cmd_timeout_for_rtt
  std::chrono::milliseconds command_deadline{50};
  double fallback_hz{100.0};       ///< WaitForTick 이 안 될 때 폴링+송신 주기
  double max_sample_age_s{0.1};    ///< 이보다 오래 새 표본이 없으면 진단 ERROR
  TimeSource time_source{TimeSource::kMorai};
  bool publish_clock{false};
  bool publish_ground_truth{true};
  bool auto_reassert_auto{true};
  bool engage_on_start{false};
  double cmd_stale_timeout_s{0.2};  ///< main.py cmd_stale_timeout_s
  double cmd_stale_accel{-2.4};     ///< main.py cmd_stale_accel (음수만)
  double clock_offset_warn_ms{20.0};
  std::chrono::milliseconds control_mode_verify{500};
  // 26.09.25 _ YSH _ e2e 반영: deadline 초과는 그사이 새 업데이트가 왔을 때만 센다 (멈춘 MORAI 는 폴백 사유가 아니다)
  // int wft_fail_limit{10};   ///< 연속 WaitForTick 실패(스냅샷은 성공) 이만큼이면 폴링 폴백
  int wft_fail_limit{10};   ///< 연속 WaitForTick 실패(스냅샷은 새 업데이트) 이만큼이면 폴링 폴백
  /// WaitForTick 이 성공했는데 새 업데이트가 없는(같은 값·중복 스냅샷) 일이 연속 이만큼이면 WaitForTick 이
  /// 트리거 노릇을 못 하는 것으로 보고 폴링 폴백 (막지 않고 바로 돌아오는 빌드에서 루프가 헛돌지 않게).
  /// WaitForTick 은 렌더 프레임마다, 상태는 ~50/s 로 바뀌므로(실측) targetFPS 200 이면 새 업데이트 사이에 중복이
  /// ~3 번 낀다 — 여유를 두고 10
  int wft_duplicate_limit{10};
  int link_fail_limit{5};   ///< 연속 스냅샷 실패 이만큼이면 링크 끊김으로 본다
  SampleGateConfig gate;    ///< 역행 새 기준·멈춘 timestamp 판정
};

/// 런타임에 바뀌는 값 (on-set 콜백이 통째로 바꿔 끼운다). main.py 가 콜백마다 읽던 것들
struct RuntimeParams
{
  bool forward_control_to_sim{true};
  autoware_to_morai::CommandParams command;
  bool steer_report_hold{true};
  // 26.10.01 _ YSH _ 외부 조향 필터 (런타임 — 같은 주행 중 A/B). 기본 끔 = 종전과 같은 리포트
  bool steer_report_ignore_foreign{false};
  /// MORAI vehicle_state.steer·front_wheel_angle 1.0 이 뜻하는 조향각 [deg]. 09-29 실측 40° (스펙 보고값 32.48° 와 무관)
  double steer_input_norm_deg{40.0};
  MapOrigin map_origin;
  bool teleport_on_initialpose{true};
  // 26.09.25 _ YSH _ 명령 트리거 (런타임 전환 — 같은 주행 중 A/B). timer 일 때만 cmd_resend_hz 를 쓴다
  CommandTrigger command_trigger{CommandTrigger::kUpdate};
  double cmd_resend_hz{kCmdResendHzDefault};
};

/// RuntimeParams 를 원자적으로 바꿔 끼우는 손잡이 (읽는 쪽은 한 업데이트 동안 같은 사본을 쓴다)
class RuntimeParamsHandle
{
public:
  RuntimeParamsHandle() : params_(std::make_shared<const RuntimeParams>()) {}
  std::shared_ptr<const RuntimeParams> get() const { return std::atomic_load(&params_); }
  void set(const RuntimeParams & params)
  {
    std::atomic_store(&params_, std::make_shared<const RuntimeParams>(params));
  }

private:
  std::shared_ptr<const RuntimeParams> params_;
};

/// 마지막으로 받아들인(새·유효) 표본. 텔레포트 z·되읽기, 측위 초기화, 기어 재동기가 읽는다
struct CachedState
{
  morai_grpc_vendor::EgoSample ego;
  int64_t timestamp_us{0};
  int64_t receive_system_ns{0};
  int64_t receive_steady_ns{0};  ///< 엔진 시계 기준
  uint64_t update_index{0};
};

class StateCache
{
public:
  void store(std::shared_ptr<const CachedState> state) { std::atomic_store(&state_, std::move(state)); }
  std::shared_ptr<const CachedState> latest() const { return std::atomic_load(&state_); }

private:
  std::shared_ptr<const CachedState> state_;
};

/// 한 업데이트에서 발행할 것 전부 (엔진이 만들고 sink 가 발행한다 — 시험은 sink 로 받아 본다)
struct UpdateOutputs
{
  builtin_interfaces::msg::Time stamp;
  autoware_vehicle_msgs::msg::VelocityReport velocity;
  autoware_vehicle_msgs::msg::SteeringReport steering;
  bool has_ground_truth{false};
  nav_msgs::msg::Odometry odometry;
  geometry_msgs::msg::TransformStamped transform;
  std::optional<autoware_vehicle_msgs::msg::GearReport> gear;
  std::optional<autoware_vehicle_msgs::msg::ControlModeReport> control_mode;
  std::optional<autoware_perception_msgs::msg::TrackedObjects> objects;
  std::optional<rosgraph_msgs::msg::Clock> clock;
  // 26.11.10 _ YSH _ 외란 프레임 — objects 를 낸 업데이트에만, 엔진에 외란 손잡이가 있을 때만 (objects 는 이 프레임의
  //   sent_actors 로 만든 것). 노드가 /daq/disturbance·/daq/object_status_disturbed 로 낸다
  std::optional<DisturbanceFrame> disturbance;
};

/// 엔진 출력 (노드가 ROS 로 발행한다)
class EngineSink
{
public:
  virtual ~EngineSink() = default;
  /// 엔진 스레드에서 업데이트마다 한 번
  virtual void publish(const UpdateOutputs & outputs) = 0;
  /// 연결·스펙 확보 직후 (reconnected = 처음이 아님). 램프 재전송·센서 장착 같은 일을 건다
  virtual void on_connected(bool reconnected) = 0;
  // 26.09.26 _ YSH _ 신규 — DAQ 훅. 엔진 스레드, publish() 직후, 받아들인(새·유효) 표본마다 한 번. stamp 는 그
  //   표본의 발행 stamp (time_source·내용 판정을 따른다). 기본은 아무것도 하지 않는다 (시험 sink 는 그대로)
  virtual void on_sample(
    const morai_grpc_vendor::Snapshot & /*snapshot*/, const builtin_interfaces::msg::Time & /*stamp*/,
    uint64_t /*update_index*/, int64_t /*receive_steady_ns*/)
  {
  }
  // 26.09.26 _ YSH _ 신규 — 원시 gRPC 기록 훅. ControlVehicle 을 부른 뒤마다 한 번 (엔진 스레드 또는 명령 타이머 스레드,
  //   cmd_mutex_ 아래라 송신 순서대로 온다). cmd = 실제로 링크에 넘긴 명령 (stale 이면 감속 대체 명령), send_system_ns =
  //   RPC 직전 system_clock, status = 그 RPC 결과 (grpc_code -1 = 보내지 않음 — 비유한 명령). 받는 쪽은 복사만 하고
  //   곧바로 돌아온다 (송신 주기를 늦추지 않게). 기본은 아무것도 하지 않는다
  virtual void on_command_sent(
    const morai_grpc_vendor::VehicleCommand & /*cmd*/, bool /*stale*/, int64_t /*send_system_ns*/,
    const morai_grpc_vendor::Status & /*status*/)
  {
  }
};

/// 시계 주입 (시험은 가짜 시계로 결정적으로 돈다)
struct EngineClock
{
  std::function<int64_t()> steady_ns;
  std::function<int64_t()> system_ns;
  /// 비어 있으면 엔진이 stop() 으로 깨울 수 있는 대기를 쓴다
  std::function<void(int64_t ns)> sleep_ns;
  static EngineClock real();
};

/// 누적 카운터 스냅샷 (진단·통계용)
struct EngineStats
{
  uint64_t snapshots{0};           ///< 성공한 GetAllActorsState
  uint64_t updates{0};             ///< 받아들인 새 표본 (= 발행한 업데이트)
  uint64_t duplicates{0};          ///< timestamp 같음 → 건너뜀
  uint64_t backward{0};            ///< timestamp 역행 → 건너뜀
  uint64_t rebased{0};
  uint64_t no_ego{0};
  uint64_t invalid_ego{0};
  uint64_t gaps_over_45ms{0};      ///< 이웃한 새 표본 사이 MORAI 시간 > 45 ms
  uint64_t snapshot_failures{0};
  uint64_t wft_failures{0};
  uint64_t link_losses{0};
  uint64_t reconnects{0};          ///< 끊긴 뒤 다시 연결·스펙 확보까지 마친 횟수
  uint64_t link_recreates{0};
  uint64_t sends{0};               ///< ControlVehicle 시도
  uint64_t send_ok{0};
  uint64_t send_timeouts{0};
  uint64_t send_result_failures{0};  ///< 전송은 됐지만 MORAI Result 가 성공이 아님
  uint64_t send_errors{0};           ///< 그 밖의 전송 실패
  uint64_t stale_sends{0};         ///< cmd_stale_timeout_s 넘어 감속 대체 명령을 보낸 횟수
  uint64_t blocked_sends{0};       ///< 중복 writer 가드로 보내지 않은 업데이트
  uint64_t block_brake_sends{0};   ///< 가드가 막히는 순간 보낸 감속 대체 명령 (sends 에도 포함)
  uint64_t mode_query_failures{0};
  uint64_t objects_published{0};
  uint64_t wft_no_update{0};       ///< WaitForTick 은 성공했는데 새 업데이트가 없었다 (같은 값·중복 스냅샷)
  uint64_t spec_refreshes{0};      ///< Ego 가 1 s 넘게 없다가 돌아와 스펙을 다시 읽은 횟수 (재연결 아님)
  uint64_t content_mode_entries{0};  ///< timestamp 를 못 써 내용 판정으로 바꾼 횟수
  // 26.09.25 _ YSH _ timer 트리거: 송신이 늦어 다음 deadline 을 넘긴 횟수 (main.py '지연재설정' — 몰아 보내지 않고 기준 재설정)
  uint64_t cmd_timer_late{0};
};

class Engine
{
public:
  enum class Phase { kConnecting, kRunning };

  // 26.09.26 _ YSH _ 발행 주기를 CARLA·simple_planning_simulator 방식으로 — 차량 리포트 전부(gear·control_mode 포함)를
  //   MORAI 업데이트마다 같은 stamp 로 내고, TrackedObjects 만 MORAI 시각 100 ms 스케줄(10 Hz). 제어 모드 조회는 엔진 루프
  //   밖 전용 스레드 10 Hz — 엔진 루프 안 동기 조회가 MORAI 다음 프레임까지 막혀 6번째 업데이트마다 놓쳤다(41.5 Hz). 원문:
  // static constexpr uint64_t kGearModeEvery = 2;   ///< GearReport·ControlModeReport (~25 Hz)
  // static constexpr uint64_t kObjectsEvery = 5;    ///< TrackedObjects (~10 Hz)
  // static constexpr uint64_t kModeQueryEvery = 5;  ///< GetVehicleControlMode (~10 Hz)
  static constexpr int64_t kObjectsPeriodNs = 100000000LL;   ///< TrackedObjects 주기 (MORAI 시각, 10 Hz)
  static constexpr int64_t kObjectsSlackNs = 10000000LL;     ///< 이만큼 이르게 온 표본도 그 주기의 것으로 친다 (반 프레임)
  static constexpr int64_t kModeQueryPeriodNs = 100000000LL;  ///< GetVehicleControlMode 전용 스레드 주기 (10 Hz)
  static constexpr double kModeFreshS = 0.5;      ///< 조회가 이보다 낡으면 ControlModeReport 생략 (main.py 는 실패 즉시 생략)
  static constexpr double kRemoteRttMs = 5.0;     ///< 스냅샷 왕복 p50 이 이보다 크면 원격으로 보고 WARN (main.py 와 같은 기준)

  Engine(
    ILink & link, CommandSlot & slot, const RuntimeParamsHandle & params, StateCache & cache,
    EngineSink & sink, const EngineConfig & config, const rclcpp::Logger & logger,
    // 26.11.10 _ YSH _ 외란 손잡이를 끝 인자로 받는다 (nullptr = 외란 없음, 예전과 같은 경로). 엔진보다 오래 살아야 한다
    // 원문: EngineClock clock = EngineClock::real());
    EngineClock clock = EngineClock::real(), const DisturbanceHandle * disturbance = nullptr);
  ~Engine();

  Engine(const Engine &) = delete;
  Engine & operator=(const Engine &) = delete;

  /// 엔진 스레드 시작 / 정지 (정지 시 최근 송신 중이었으면 ~0.3 s 감속 명령을 보낸다)
  // 26.09.25 _ YSH _ 명령 타이머 스레드도 같이 시작·정지한다 (command_trigger=update 면 20 ms 마다 설정만 본다)
  void start();
  void stop();

  /// 트리거 한 번 + 업데이트 한 번 (연결 중이면 연결 시도 한 번). 스레드 없이 시험할 때 쓴다
  void step();

  // 26.09.25 _ YSH _ 명령 타이머 루프 한 바퀴 (스레드 없이 시험할 때 쓴다). deadline 전이면 deadline 까지 자고 돌아오고,
  //   deadline 이면 최신 명령을 한 번 보낸 뒤 다음 deadline = 이전 + 주기 (늦었으면 지금, cmd_timer_late +1).
  //   command_trigger 가 update 면 아무것도 보내지 않고 kTimerIdlePollNs 만큼 잔다
  void timer_step();
  static constexpr int64_t kTimerIdlePollNs = 20000000LL;  ///< update 모드에서 설정 확인 주기 (20 ms)

  // 26.09.26 _ YSH _ 제어 모드 조회 한 번 (스레드 없이 시험할 때 쓴다). 실행 중일 때만 GetVehicleControlMode 1 회,
  //   대기는 하지 않는다. 전용 스레드는 이것을 kModeQueryPeriodNs 마다 부른다
  void mode_query_step();

  /// 루프 본체: 스냅샷 판정 → StateCache → ControlVehicle 1 회 → 발행. 트리거와 무관하다.
  /// tick_confirmed: 트리거가 새 MORAI 업데이트를 확인했다 (WaitForTick 성공, lockstep Tick) — timestamp 를
  /// 쓸 수 없는 빌드(내용 판정)에서 정지한 차의 업데이트를 가르는 데만 쓴다. 판정을 돌려준다
  SampleVerdict on_update(const morai_grpc_vendor::Snapshot & snapshot, bool tick_confirmed = false);

  /// 정지 직전 감속 명령 (start/stop 을 쓰지 않는 시험용으로도 공개)
  void shutdown_brake();

  // --- 노드가 알려 주는 것 ---
  /// control_mode_request 로 MORAI 에 요청한 모드 (AUTO 재지정 판단에 쓴다). RPC **전에** 기록한다 —
  /// 그래야 되읽기를 기다리는 동안 재연결이 끼어도 disengage 를 AUTO 로 되돌리지 않는다
  void note_requested_mode(morai_grpc_vendor::ControlMode mode);
  /// AUTO 요청이 실패했을 때 이전 기록으로 되돌린다 (nullopt = 요청한 적 없음)
  void restore_requested_mode(std::optional<morai_grpc_vendor::ControlMode> mode);
  /// 중복 writer 가드 (true 면 ControlVehicle 을 보내지 않는다). 송신 중에 막히면 먼저 ~0.3 s 감속 대체
  /// 명령을 보낸다 — MORAI 는 마지막 명령을 유지하므로(실측) 가속 명령이 남지 않게
  void set_command_block(bool blocked);

  // --- 상태 조회 (스레드 안전) ---
  Phase phase() const { return phase_.load(); }
  bool fallback_active() const { return fallback_.load(); }
  bool command_blocked() const { return command_blocked_.load(); }
  std::optional<EgoVehicleSpec> spec() const;
  /// 마지막으로 안 MORAI 제어 모드 (재연결 중이면 nullopt)
  std::optional<morai_grpc_vendor::ControlMode> control_mode() const;
  /// 마지막 제어 모드 조회 성공 이후 경과 [s]. kModeFreshS 보다 낡으면 ControlModeReport 를 내지 않는다
  std::optional<double> control_mode_age_s() const;
  std::optional<morai_grpc_vendor::ControlMode> requested_mode() const;
  EngineStats stats() const;
  /// 마지막 새 표본 이후 경과 [s] (한 번도 없으면 nullopt)
  std::optional<double> sample_age_s() const;
  size_t last_actor_count() const { return last_actor_count_.load(); }
  /// 표본 판정 방식 (timestamp / 내용 — timestamp 를 쓸 수 없는 빌드)
  GateMode gate_mode() const { return static_cast<GateMode>(gate_mode_.load()); }
  /// 지금 쓰는 ControlVehicle deadline [ms]
  int64_t command_deadline_ms() const { return command_deadline_ms_.load(); }
  /// 스펙 재조회 대기 중 (Ego 가 1 s 넘게 없다가 돌아왔고 아직 스펙을 못 읽었다 — 명령 보류)
  bool spec_refresh_pending() const { return spec_refresh_pending_.load(); }
  /// 마지막 명령 메시지 나이 [s] (슬롯이 비었으면 nullopt)
  std::optional<double> command_age_s() const;
  const EngineConfig & config() const { return config_; }

  // 26.09.25 _ YSH _ 지금 명령 트리거 (런타임 파라미터)와 이웃한 ControlVehicle 사이 간격 [ms] (최근 500 개, 공백 구간 제외)
  CommandTrigger command_trigger() const { return params_.get()->command_trigger; }
  RollingWindow::Summary send_gap_ms() const { return send_gap_ms_.summary(); }
  RollingWindow::Summary send_rtt_ms() const { return send_rtt_ms_.summary(); }
  RollingWindow::Summary snapshot_rtt_ms() const { return snapshot_rtt_ms_.summary(); }
  /// MORAI timestamp − 로컬 수신 시각 [ms] (같은 PC 면 −프레임 나이, 두 PC 면 + 시계 오프셋)
  RollingWindow::Summary clock_offset_ms() const { return clock_offset_ms_.summary(); }
  /// 이웃한 새 표본 사이 MORAI 시간 [ms]
  RollingWindow::Summary update_interval_ms() const { return update_interval_ms_.summary(); }

private:
  void run();
  void connect_step();
  void run_step();
  void on_snapshot_failure(const morai_grpc_vendor::Status & status);
  void enter_connecting(const std::string & why, bool recreate);
  void enter_fallback(const std::string & why);
  bool probe_wait_for_tick();
  void handle_engage_on_start();
  void evaluate_reassert();
  void try_refresh_spec(int64_t now);
  void refresh_command_deadline(double rtt_ms);
  // 26.09.25 _ YSH _ 스냅샷 대신 속도만 받는다 — update(업데이트 표본)·timer(StateCache 최신 표본) 두 트리거가 같은 경로로 만든다.
  //   cmd_mutex_ 를 잡고 돈다 (트리거 전환 순간 두 스레드가 겹쳐도 한 번에 하나)
  // void send_command(
  //   const morai_grpc_vendor::Snapshot & snapshot, const RuntimeParams & params, int64_t now);
  void send_command(double speed_mps, const RuntimeParams & params, int64_t now);
  // 26.09.25 _ YSH _ timer 트리거 한 번: 연결·스펙·첫 유효 표본이 있을 때만 send_command
  void command_tick(const RuntimeParams & params, int64_t now);
  void run_command_timer();
  // 26.09.26 _ YSH _ stale = 감속 대체 명령을 보낸다 (원시 기록 훅에 넘긴다). 원문:
  // morai_grpc_vendor::Status transmit(const morai_grpc_vendor::VehicleCommand & cmd);
  morai_grpc_vendor::Status transmit(const morai_grpc_vendor::VehicleCommand & cmd, bool stale);
  UpdateOutputs build_outputs(
    const morai_grpc_vendor::Snapshot & snapshot, const RuntimeParams & params, int64_t now);
  // 26.09.26 _ YSH _ only_while_running: 조회 스레드용 — 실행 중이 아니면 조회하지 않는다 (확인은 세대 읽기와 같은 락 안). 원문:
  // void query_control_mode();
  void query_control_mode(bool only_while_running = false);
  // 26.09.26 _ YSH _ 제어 모드 조회 전용 스레드 루프 / 엔진 스레드가 아는 모드를 직접 바꿀 때 (재연결·AUTO 설정) —
  //   mode_mutex_ 아래 세대를 올려 그 전에 시작한 조회 결과를 버리게 한다. value -1 = 모름
  void run_mode_query();
  void overwrite_control_mode(int value, int64_t ok_steady_ns);
  void maybe_log_stats(int64_t now);
  void sleep_ns(int64_t ns);
  // 26.09.25 _ YSH _ 절대 deadline 까지 (stop() 으로 깨운다) / stop 뒤에도 자는 대기 (종료 감속 주기)
  void sleep_until_ns(int64_t deadline_ns);
  void plain_sleep_ns(int64_t ns);
  bool stop_requested() const;

  ILink & link_;
  CommandSlot & slot_;
  const RuntimeParamsHandle & params_;
  StateCache & cache_;
  EngineSink & sink_;
  const EngineConfig config_;
  rclcpp::Logger logger_;
  EngineClock clock_;
  mutable rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // --- 스레드 ---
  std::thread thread_;
  // 26.09.25 _ YSH _ 명령 타이머 스레드 (command_trigger=timer 일 때만 송신)
  std::thread timer_thread_;
  // 26.09.26 _ YSH _ 제어 모드 조회 스레드 (엔진 루프가 조회 RPC 에 막히지 않게)
  std::thread mode_thread_;
  mutable std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stop_requested_{false};

  // --- 연결 ---
  std::atomic<Phase> phase_{Phase::kConnecting};
  bool ever_connected_{false};
  bool recreate_pending_{false};
  int connect_attempts_{0};
  int spec_attempts_{0};
  int64_t next_connect_ns_{0};
  int link_fail_streak_{0};
  int deadline_streak_{0};
  mutable std::mutex spec_mutex_;
  std::optional<EgoVehicleSpec> spec_;

  // --- 트리거 ---
  std::atomic<bool> fallback_{false};
  int wft_fail_streak_{0};
  int wft_dup_streak_{0};         ///< WaitForTick 성공 + 새 업데이트 없음 연속 횟수
  bool has_last_wft_value_{false};
  int64_t last_wft_value_{0};
  int64_t next_poll_ns_{0};
  int64_t last_wft_probe_ns_{0};
  morai_grpc_vendor::Snapshot snapshot_buf_;

  // --- 표본 ---
  SampleGate gate_;
  std::atomic<int> gate_mode_{0};
  std::optional<int64_t> ego_missing_since_ns_;
  /// Ego 가 1 s 넘게 없었다 → 돌아오면 연결 단계로 가지 않고 스펙만 다시 읽는다 (그동안 명령 보류)
  std::atomic<bool> spec_refresh_pending_{false};
  int64_t next_spec_refresh_ns_{0};
  uint64_t update_index_{0};
  // 26.09.26 _ YSH _ 다음 TrackedObjects 를 낼 MORAI 시각 [ns] (0 = 아직 없음, 엔진 스레드만)
  int64_t objects_next_due_ns_{0};
  // 26.11.10 _ YSH _ 외란 주입: 설정 손잡이 (노드 것, nullptr = 끔) · 프레임 적용기 (엔진 스레드만)
  const DisturbanceHandle * disturbance_{nullptr};
  ObjectDisturber disturber_;
  std::atomic<int64_t> last_valid_steady_ns_{0};
  std::atomic<size_t> last_actor_count_{0};
  SteeringReportHold steer_hold_;
  // 26.10.01 _ YSH _ 외부 조향 필터: 최근 송신 조향(송신 스레드가 쓴다, 자체 mutex) · 필터(엔진 스레드만)
  SentSteerHistory sent_steer_;
  ForeignSteerFilter foreign_filter_;

  // --- 명령 ---
  // 26.09.25 _ YSH _ 명령 트리거가 둘(엔진 루프·타이머 스레드)이 되어 송신 상태를 나눈다:
  //   cmd_mutex_ 아래 = send_command·shutdown_brake 만 쓰는 것 (was_blocked_ ~ last_normal_send_ns_).
  //   발행(build_outputs)이 읽는 steer_comp_gain_·last_steer_cmd_, 재연결이 지우는 last_send_ns_ 는 원자.
  //   10 초 통계 창(window_*)은 window_mutex_ (RPC 를 잡은 채 기다리지 않게 cmd_mutex_ 와 따로)
  // bool was_blocked_{false};
  // int64_t block_brake_until_ns_{0};
  // std::atomic<int64_t> command_deadline_ms_{50};
  // bool remote_rtt_warned_{false};
  // bool has_built_{false};
  // uint64_t built_seq_{0};
  // morai_grpc_vendor::VehicleCommand built_cmd_;
  // double steer_comp_gain_{1.0};  ///< 마지막으로 만든 명령의 배율 (조향 리포트 역보정)
  // double last_steer_cmd_{0.0};   ///< 마지막으로 만든 명령의 조향 [rad] (0 드롭아웃 유지 판단)
  // std::string long_mode_logged_;
  // int64_t long_mode_log_ns_{-1};
  // int64_t last_send_ns_{0};
  // int64_t last_normal_send_ns_{0};
  // double window_max_gap_ms_{0.0};
  // double window_max_velocity_{0.0};
  std::atomic<bool> command_blocked_{false};
  std::atomic<int64_t> command_deadline_ms_{50};
  bool remote_rtt_warned_{false};  ///< 엔진 스레드만
  std::mutex cmd_mutex_;
  bool was_blocked_{false};
  int64_t block_brake_until_ns_{0};
  bool has_built_{false};
  uint64_t built_seq_{0};
  morai_grpc_vendor::VehicleCommand built_cmd_;
  std::string long_mode_logged_;
  int64_t long_mode_log_ns_{-1};
  int64_t last_normal_send_ns_{0};
  std::atomic<double> steer_comp_gain_{1.0};  ///< 마지막으로 만든 명령의 배율 (조향 리포트 역보정)
  std::atomic<double> last_steer_cmd_{0.0};   ///< 마지막으로 만든 명령의 조향 [rad] (0 드롭아웃 유지 판단)
  std::atomic<int64_t> last_send_ns_{0};      ///< 0 = 공백 (간격 계산에서 뺀다)
  std::mutex window_mutex_;
  double window_max_gap_ms_{0.0};
  double window_max_velocity_{0.0};
  RollingWindow send_gap_ms_{500};
  // 26.09.25 _ YSH _ 명령 타이머 상태 (타이머 스레드만 — 시험은 timer_step 을 한 스레드에서 부른다)
  bool timer_active_{false};
  int64_t timer_next_ns_{0};
  int64_t timer_period_ns_{0};

  // --- 제어 모드 ---
  std::atomic<int> control_mode_{-1};    ///< -1 = 모름 (재연결 때 지운다)
  std::atomic<int64_t> mode_ok_steady_ns_{0};  ///< 마지막 조회 성공 (엔진 시계)
  std::atomic<int> requested_mode_{-1};  ///< -1 = 요청한 적 없음
  bool reassert_pending_{false};         ///< 재연결 뒤 첫 조회 성공 때 AUTO 재지정을 판단한다
  // 26.09.25 _ YSH _ e2e 반영: 재지정 RPC 가 MORAI 에 닿지 못한 횟수 (kReassertAttempts 까지 모드 조회마다 재시도)
  int reassert_attempts_{0};
  // 26.09.26 _ YSH _ 조회 스레드와 엔진 스레드가 모드를 같이 쓴다. mode_epoch_ 는 엔진 스레드가 모드를 직접 바꿀 때마다
  //   올린다 — 조회는 시작할 때 읽은 세대가 저장 순간에도 같을 때만 저장한다 (끊김 전 값이 되살아나지 않게).
  //   mode_query_seq_ = 저장한 조회 성공 수 — 엔진 스레드는 이것이 바뀔 때만 AUTO 재지정을 판단한다 ("다음 조회 때")
  std::mutex mode_mutex_;
  uint64_t mode_epoch_{0};
  std::atomic<uint64_t> mode_query_seq_{0};
  uint64_t reassert_seen_seq_{0};  ///< 엔진 스레드만

  // --- 통계 ---
  struct AtomicStats;
  std::unique_ptr<AtomicStats> stats_;
  RollingWindow send_rtt_ms_{500};
  RollingWindow snapshot_rtt_ms_{500};
  RollingWindow clock_offset_ms_{500};
  RollingWindow update_interval_ms_{500};
  int64_t stats_window_start_ns_{0};
  EngineStats stats_window_prev_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__ENGINE_HPP_
