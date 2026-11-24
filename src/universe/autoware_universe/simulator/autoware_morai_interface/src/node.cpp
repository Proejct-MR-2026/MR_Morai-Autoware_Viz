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
// 26.11.11 _ YSH _ 외란 메시지를 morai_ros2_msgs(외란 추가판)로 옮겼다 — 이 파일의 mr_disturbance_msgs 를 morai_ros2_msgs 로 바꿨다 (이름만, 주석 속 옛 이름·원문은 그대로)

// MoraiInterfaceNode 구현 — 파라미터 선언, gRPC 클라이언트 생성, 초기화 재시도 타이머
// 26.09.25 _ YSH _ 전면 교체: 파라미터(전부 코드 기본값, main.py 이름) · 엔진/기어/작업 스레드 기동 ·
//   ROS 입출력(토픽·QoS 는 main.py 와 같음) · 지연 응답 서비스 · 측위 초기화 · 중복 writer 가드 · 진단.
//   옛 on_initialize()(연결 재시도 타이머)와 mount_sensors_once() 의 역할은 엔진 연결 단계와 작업 스레드로
//   옮겼다. 원본: git show 94cdb2b:<이 경로>

#include "autoware/morai_interface/node.hpp"

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"
#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"
#include "autoware/morai_interface/sensor_kit_mounter.hpp"

#include <morai_grpc_vendor/actor_client.hpp>
#include <morai_grpc_vendor/sensor_client.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
// 26.09.25 _ YSH _ snprintf (기동 로그의 명령 트리거 설명)
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{

namespace mgv = morai_grpc_vendor;

namespace
{
constexpr int64_t kMs = 1000000LL;
constexpr int64_t kSec = 1000000000LL;

/// main.py 와 같은 토픽 이름 (절대 이름 — remap 없이 main.py 를 그대로 대체한다)
constexpr char kTopicVelocity[] = "/vehicle/status/velocity_status";
constexpr char kTopicSteering[] = "/vehicle/status/steering_status";
constexpr char kTopicGear[] = "/vehicle/status/gear_status";
constexpr char kTopicControlMode[] = "/vehicle/status/control_mode";
constexpr char kTopicTurnIndicators[] = "/vehicle/status/turn_indicators_status";
constexpr char kTopicHazardLights[] = "/vehicle/status/hazard_lights_status";
constexpr char kTopicKinematicState[] = "/localization/kinematic_state";
constexpr char kTopicObjects[] = "/perception/object_recognition/tracking/objects";
constexpr char kTopicControlCmd[] = "/control/command/control_cmd";
constexpr char kTopicGearCmd[] = "/control/command/gear_cmd";
constexpr char kTopicTurnIndicatorsCmd[] = "/control/command/turn_indicators_cmd";
constexpr char kTopicHazardLightsCmd[] = "/control/command/hazard_lights_cmd";
constexpr char kTopicInitialPose[] = "/initialpose";
constexpr char kServiceControlModeRequest[] = "/control/control_mode_request";
constexpr char kServiceInitializeLocalization[] = "/localization/initialize";
// 26.10.12 _ YSH _ 의사 VILS 2단계: follow 모드에서 실차 값을 받는 토픽 (DAQ Ego·등화)
constexpr char kTopicKcanImu[] = "/vehicle/status/kcan_imu";
constexpr char kTopicActuationStatus[] = "/vehicle/status/actuation_status";
constexpr char kTopicLocalizationAccel[] = "/localization/acceleration";
constexpr int64_t kRealCarPoseMaxAgeNs = 300 * kMs;   // 실차 측위가 이보다 낡으면 ego_status 를 내지 않는다
constexpr int64_t kRealCarInputMaxAgeNs = 500 * kMs;  // 그 밖의 실차 값은 이보다 낡으면 그 칸을 대체·0

/// main.py create_publisher/create_subscription 의 depth
constexpr size_t kQueueDepth = 10;

constexpr std::chrono::milliseconds kGearVerify{300};
// 26.10.12 _ YSH _ VILS 고스트 ControlVehicle deadline (follow 20 Hz 한 틱 안)
constexpr std::chrono::milliseconds kGhostDeadline{50};
constexpr int64_t kTeleportVerifyNs = 1 * kSec;
constexpr double kTeleportToleranceM = 0.5;
constexpr int64_t kLocalizationResponseTimeoutNs = 10 * kSec;
constexpr double kLocalizationMaxSampleAgeS = 1.0;
// 26.09.25 _ YSH _ 리뷰 반영: 가드 주기 2 s → 1 s (첫 판정 전에는 ControlVehicle 을 막아 둔다)
constexpr std::chrono::seconds kGuardPeriod{1};
/// |MORAI timestamp − 수신 시각| 이 이보다 크면 시계 어긋남이 아니라 epoch 가 아닌 시각이다 (진단 ERROR)
constexpr double kClockImplausibleMs = 3600.0 * 1000.0;

/// main.py 에만 있던 파라미터 — 이 노드는 선언하지 않는다 (-p 로 줘도 효과 없음 → 기동 때 WARN)
struct LegacyParam
{
  const char * name;
  const char * note;
};
// 26.09.25 _ YSH _ cmd_resend_hz 는 이제 선언한다 (command_trigger=timer 의 주기) — 목록에서 뺐다. cmd_send_mode 설명 갱신
constexpr LegacyParam kLegacyParams[] = {
  // {"cmd_resend_hz", "송신은 MORAI 업데이트당 1 회 (재송신 스레드 없음)"},
  {"cmd_rpc_timeout_s", "morai.command_deadline_ms (ms, 왕복에 맞춰 자동으로 늘어난다)"},
  // {"cmd_send_mode", "항상 동기 1 회 (업데이트 트리거)"},
  {"cmd_send_mode",
   "항상 동기 (command_trigger: update = MORAI 업데이트당 1 회, timer = cmd_resend_hz 고정 주기)"},
  {"gear_async", "기어는 항상 GearSender 스레드"},
  {"gear_rpc_timeout_s", "morai.gear_deadline_ms (ms)"},
  {"ego_spec_wait_s", "스펙은 죽지 않고 무한히 기다린다 (백오프 0.75~5 s)"},
};

unsigned long ul(uint64_t v)  // NOLINT(runtime/int)
{
  return static_cast<unsigned long>(v);  // NOLINT(runtime/int)
}

std::string env_or(const char * name, const std::string & fallback)
{
  const char * v = std::getenv(name);
  return (v != nullptr && *v != '\0') ? std::string(v) : fallback;
}

int64_t env_port_or(const char * name, int64_t fallback)
{
  const char * v = std::getenv(name);
  if (v == nullptr || *v == '\0') {
    return fallback;
  }
  char * end = nullptr;
  const long parsed = std::strtol(v, &end, 10);  // NOLINT(runtime/int)
  if (end == v || *end != '\0' || parsed < 1 || parsed > 65535) {
    return fallback;
  }
  return parsed;
}

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & text, bool read_only)
{
  rcl_interfaces::msg::ParameterDescriptor d;
  d.description = text;
  d.read_only = read_only;
  return d;
}

// 26.09.25 _ YSH _ 정수·실수 둘 다 받는 숫자 파라미터 (cmd_resend_hz:=100 · `ros2 param set ... 100` 은 정수로 온다 —
//   double 로만 선언하면 기동 때 타입 예외로 노드가 죽는다). 숫자가 아니면 nullopt
std::optional<double> as_number(const rclcpp::ParameterValue & value)
{
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
    return value.get<double>();
  }
  if (value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
    return static_cast<double>(value.get<int64_t>());
  }
  return std::nullopt;
}

EngineStats diff(const EngineStats & a, const EngineStats & b)
{
  EngineStats d;
  d.snapshots = a.snapshots - b.snapshots;
  d.updates = a.updates - b.updates;
  d.duplicates = a.duplicates - b.duplicates;
  d.backward = a.backward - b.backward;
  d.rebased = a.rebased - b.rebased;
  d.no_ego = a.no_ego - b.no_ego;
  d.invalid_ego = a.invalid_ego - b.invalid_ego;
  d.gaps_over_45ms = a.gaps_over_45ms - b.gaps_over_45ms;
  d.snapshot_failures = a.snapshot_failures - b.snapshot_failures;
  d.wft_failures = a.wft_failures - b.wft_failures;
  d.link_losses = a.link_losses - b.link_losses;
  d.reconnects = a.reconnects - b.reconnects;
  d.link_recreates = a.link_recreates - b.link_recreates;
  d.sends = a.sends - b.sends;
  d.send_ok = a.send_ok - b.send_ok;
  d.send_timeouts = a.send_timeouts - b.send_timeouts;
  d.send_result_failures = a.send_result_failures - b.send_result_failures;
  d.send_errors = a.send_errors - b.send_errors;
  d.stale_sends = a.stale_sends - b.stale_sends;
  d.blocked_sends = a.blocked_sends - b.blocked_sends;
  d.block_brake_sends = a.block_brake_sends - b.block_brake_sends;
  d.mode_query_failures = a.mode_query_failures - b.mode_query_failures;
  d.objects_published = a.objects_published - b.objects_published;
  d.wft_no_update = a.wft_no_update - b.wft_no_update;
  d.spec_refreshes = a.spec_refreshes - b.spec_refreshes;
  d.content_mode_entries = a.content_mode_entries - b.content_mode_entries;
  // 26.09.25 _ YSH _ timer 트리거 지연재설정
  d.cmd_timer_late = a.cmd_timer_late - b.cmd_timer_late;
  return d;
}


}  // namespace

// =============================================================================
// 생성 / 소멸
// =============================================================================

MoraiInterfaceNode::MoraiInterfaceNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("morai_interface", options)
{
  // 26.09.25 _ YSH _ 모든 파라미터를 코드 기본값으로 선언 (yaml 없이도 기동)
  declare_parameters();
  runtime_params_.set(runtime_params_value_);

  link_ = std::make_unique<MoraiLinkClient>(link_options_);

  // --- 출력 (main.py 와 같은 토픽·depth 10) ---
  const rclcpp::QoS qos{kQueueDepth};
  // 26.10.10 _ YSH _ follow 모드면 Ego 상태는 /morai/ego/* 로 (실차 인터페이스가 /vehicle/status/* 를, GNSS 노드가
  //   /localization/kinematic_state 를 낸다). TF 는 내지 않는다. 객체는 그대로 /perception/... 로 낸다. 원문은 kTopic* 그대로
  pub_velocity_ = create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(ego_topic(kTopicVelocity), qos);
  pub_steering_ = create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(ego_topic(kTopicSteering), qos);
  pub_gear_ = create_publisher<autoware_vehicle_msgs::msg::GearReport>(ego_topic(kTopicGear), qos);
  pub_control_mode_ =
    create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(ego_topic(kTopicControlMode), qos);
  pub_turn_indicators_ =
    create_publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>(ego_topic(kTopicTurnIndicators), qos);
  pub_hazard_lights_ =
    create_publisher<autoware_vehicle_msgs::msg::HazardLightsReport>(ego_topic(kTopicHazardLights), qos);
  if (engine_config_.publish_ground_truth) {
    pub_kinematic_state_ = create_publisher<nav_msgs::msg::Odometry>(ego_topic(kTopicKinematicState), qos);
    pub_objects_ =
      create_publisher<autoware_perception_msgs::msg::TrackedObjects>(kTopicObjects, qos);
    if (!follow_mode_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }
  }
  if (engine_config_.publish_clock) {
    pub_clock_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", rclcpp::ClockQoS());
  }
  // 26.10.13 _ YSH _ localize: twist 를 실차 속도·위치 헤딩 변화율로 채운다 (엔진이 첫 표본을 내기 전에 만든다).
  //   자기 차량 상태는 /morai/ego/* 로 나가므로 이 구독은 실차 인터페이스(K-CAN) 값만 받는다
  if (localize_mode_ && localize_twist_from_vehicle_ && engine_config_.publish_ground_truth) {
    localize_twist_ = std::make_unique<LocalizeTwist>(localize_twist_config_);
    sub_localize_velocity_ = create_subscription<autoware_vehicle_msgs::msg::VelocityReport>(
      kTopicVelocity, qos, [this](const autoware_vehicle_msgs::msg::VelocityReport::ConstSharedPtr m) {
        localize_twist_->set_vehicle_speed(m->longitudinal_velocity, steady_now_ns());
      });
  }

  // 26.11.10 _ YSH _ 외란 주입: 명령 제어기 (엔진보다 먼저 만든다 — 엔진이 손잡이를 읽는다). 첫 객체 프레임 전의 상태
  //   조회 답은 "외란 없음" 기본 프레임으로 채워 둔다 (메시지 기본값은 objects_visible false 라 숨김으로 읽힌다)
  if (disturbance_enable_) {
    disturbance_ctl_ = std::make_unique<DisturbanceController>(
      disturbance_handle_,
      DisturbanceController::Options{real_car_vehicle_, engine_config_.publish_ground_truth, {}});
    last_disturbance_status_ = to_disturbance_status_msg(DisturbanceFrame{}, builtin_interfaces::msg::Time{});
  }

  // --- 엔진·기어·작업 스레드 ---
  EngineSink & sink = *this;  // private 기반이라 변환은 클래스 안에서
  engine_ = std::make_unique<Engine>(
    *link_, command_slot_, runtime_params_, state_cache_, sink, engine_config_,
    // 26.11.10 _ YSH _ 외란 손잡이를 넘긴다 (disturbance.enable + GT 발행 + gt 모드일 때만, 아니면 nullptr = 예전 경로)
    // 원문: get_logger().get_child("engine"));
    get_logger().get_child("engine"), EngineClock::real(),
    disturbance_engine_ ? &disturbance_handle_ : nullptr);
  gear_sender_ = std::make_unique<GearSender>(
    [this](int gear) { return link_->set_gear(static_cast<mgv::GearMode>(gear), kGearVerify); },
    [this]() -> std::optional<int> {
      // main.py read_fn: MORAI 실제 기어 = 최신 표본의 vehicle_state.gear_mode
      const auto state = state_cache_.latest();
      if (!state || !state->ego.vehicle_state.present) {
        return std::nullopt;
      }
      return static_cast<int>(state->ego.vehicle_state.gear_mode);
    },
    gear_config_, get_logger().get_child("gear"), runtime_params_value_.forward_control_to_sim);
  worker_ = std::make_unique<JobWorker>(get_logger().get_child("worker"));
  mode_worker_ = std::make_unique<JobWorker>(get_logger().get_child("mode"));
  // 26.09.25 _ YSH _ 리뷰 반영: 첫 가드 판정(1 s, DDS 탐색 뒤) 전에는 ControlVehicle 을 막아 둔다 — main.py 가 이미
  //   떠 있으면 판정 전 몇 백 ms 동안 두 writer 가 번갈아 명령하지 않게
  engine_->set_command_block(duplicate_guard_enabled_);

  // 26.09.26 _ YSH _ DAQ — 퍼블리셔·샘플러·게이트 구독은 publish_daq 일 때만 (cue_server preflight 가 publisher 수를
  //   센다). 게이트 /daq/run_state 는 latched(transient_local) — 런 도중 이 노드가 재시작돼도 active 를 곧바로 다시 받는다
  if (publish_daq_) {
    // 26.10.05 _ YSH _ 명세 토픽을 MORAI 공식 메시지로 (이름도 바꾼다 — 같은 이름에 타입만 바꾸면 옛 구독·옛 bag 판독이 조용히 어긋난다)
    pub_daq_ego_ = create_publisher<morai_ros2_msgs::msg::EgoVehicleStatus>(
      daq_prefix_ + "/ego_status", qos);
    // 26.11.12 _ YSH _ 계산 토픽 (KATECH 요청 — CSV 에만 있던 world 판 자차 속도·가속도를 bag 에도). 같은 틱·같은 stamp·같은 게이트
    pub_daq_ego_vel_world_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      daq_prefix_ + "/calc/ego_velocity_world", qos);
    pub_daq_ego_acc_world_ = create_publisher<geometry_msgs::msg::Vector3Stamped>(
      daq_prefix_ + "/calc/ego_accel_world", qos);
    if (daq_config_.publish_objects) {
      pub_daq_objects_ = create_publisher<morai_ros2_msgs::msg::ObjectStatusList>(
        daq_prefix_ + "/object_status", qos);
    }
    // 26.11.10 _ YSH _ 외란 DAQ — 객체 프레임마다 한 건 (약 10 Hz), 같은 QoS. 엔진이 외란 프레임을 낼 때만 만든다
    //   (실차 모드·GT 미발행이면 발행자도 없다). 목록 토픽은 daq.publish_objects 를 따른다
    if (disturbance_engine_) {
      pub_daq_disturbance_ = create_publisher<morai_ros2_msgs::msg::DisturbanceStatus>(
        daq_prefix_ + "/disturbance", qos);
      if (daq_config_.publish_objects) {
        pub_daq_objects_disturbed_ = create_publisher<morai_ros2_msgs::msg::ObjectStatusList>(
          daq_prefix_ + "/object_status_disturbed", qos);
        // 26.11.11 _ YSH _ /daq/object_status 의 외란 짝 (외란 전·후·변화·시드를 객체 한 칸에)
        pub_daq_object_disturbance_ = create_publisher<morai_ros2_msgs::msg::ObjectDisturbanceStatusList>(
          daq_prefix_ + "/object_disturbance_status", qos);
      }
    }
    // 원문: pub_daq_state_ = create_publisher<nav_msgs::msg::Odometry>(daq_prefix_ + "/vehicle_state", qos);
    // 원문: pub_daq_accel_ = create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
    // 원문:   daq_prefix_ + "/acceleration", qos);
    // 원문: if (daq_config_.publish_objects) {
    // 원문:   pub_daq_objects_ = create_publisher<autoware_perception_msgs::msg::TrackedObjects>(
    // 원문:     daq_prefix_ + "/track_objects", qos);
    // 원문: }
    pub_daq_cmd_ = create_publisher<Control>(daq_prefix_ + "/vehicle_cmd", qos);
    // 26.10.06 _ YSH _ 명세 25 를 gRPC 송신값(MORAI CtrlCmd)으로. /vehicle_cmd(Autoware 원본)는 참고용으로 남긴다
    pub_daq_ctrl_ = create_publisher<morai_ros2_msgs::msg::CtrlCmd>(daq_prefix_ + "/ctrl_cmd", qos);
    // 26.10.07 _ YSH _ Ego 충돌 상대 — gRPC vehicle_state.collision_objects 를 MORAI CollisionData 로
    if (daq_config_.publish_collision) {
      pub_daq_collision_ = create_publisher<morai_ros2_msgs::msg::CollisionData>(daq_prefix_ + "/collision", qos);
    }
    const rclcpp::QoS latched = rclcpp::QoS(1).reliable().transient_local();
    pub_daq_run_summary_ =
      create_publisher<std_msgs::msg::String>(daq_prefix_ + "/run_summary", latched);
    // 26.09.26 _ YSH _ 원시 gRPC 기록 (daq.raw). RELIABLE·VOLATILE, depth 는 큐 깊이 이상 (최소 64) — DAQ 스레드가 늦게
    //   깨어 한 틱에 몰아 내도 bag 이 받기 전에 덮이지 않게
    if (daq_config_.raw) {
      const rclcpp::QoS raw_qos =
        rclcpp::QoS(std::max<size_t>(64, daq_config_.raw_queue_depth)).reliable();
      pub_daq_raw_actor_states_ =
        create_publisher<std_msgs::msg::String>(daq_prefix_ + "/raw/actor_states", raw_qos);
      pub_daq_raw_ctrl_cmd_ =
        create_publisher<std_msgs::msg::String>(daq_prefix_ + "/raw/vehicle_ctrl_cmd", raw_qos);
    }
    DaqSink & daq_sink = *this;
    // 26.10.12 _ YSH _ 의사 VILS: 명세 25 는 실차에 보낸 명령 (MORAI 로는 명령이 가지 않는다)
    // 26.10.13 _ YSH _ localize 도 명령은 실차로 간다. 원문: daq_config_.ctrl_from_command = follow_mode_;
    daq_config_.ctrl_from_command = real_car_vehicle_;
    daq_sampler_ = std::make_unique<DaqSampler>(
      daq_cache_, daq_cmd_latch_, runtime_params_, daq_sink, daq_config_, daq_mode_initial_,
      get_logger().get_child("daq"), EngineClock::real(),
      [this]() { return engine_ && engine_->phase() == Engine::Phase::kRunning; });
    // 첫 가드 판정 전에는 양보해 둔다 — daq_publisher.py 가 이미 떠 있으면 두 writer 가 섞이지 않게
    // 26.09.26 _ YSH _ 리뷰 반영: 조용히 (거짓 '다른 writer' 경고를 기동마다 내지 않게). 원문:
    // daq_sampler_->set_yield(true);
    daq_sampler_->yield_until_first_check();
    // 26.10.12 _ YSH _ 의사 VILS: ego_status 는 실차 값 (MORAI Ego 는 순간이동만 해서 속도·페달·조향이 없다)
    if (follow_mode_) {
      daq_sampler_->set_ego_override([this]() { return build_real_car_daq_ego(); });
    }
    // 26.09.26 _ YSH _ 리뷰 반영: 구독 쪽은 depth 10 — active/idle 이 몰려 와도 합쳐지지 않게 (쓰는 쪽은 depth 1 이라
    //   늦게 붙은 구독자는 여전히 최신 상태 하나만 받는다). 원문:
    // sub_daq_run_state_ = create_subscription<std_msgs::msg::String>(
    //   daq_prefix_ + "/run_state", latched,
    //   [this](const std_msgs::msg::String::ConstSharedPtr msg) { on_daq_run_state(msg); });
    sub_daq_run_state_ = create_subscription<std_msgs::msg::String>(
      daq_prefix_ + "/run_state", rclcpp::QoS(10).reliable().transient_local(),
      [this](const std_msgs::msg::String::ConstSharedPtr msg) { on_daq_run_state(msg); });
    // 26.09.26 _ YSH _ 리뷰 반영: rclcpp 는 Ctrl-C 에 컨텍스트를 먼저 닫고 그 뒤에 노드를 지운다 — 소멸자에서 멈추면
    //   'shutdown' 요약 발행이 조용히 버려진다. 컨텍스트가 닫히기 직전에 멈춘다 (stop 은 여러 번 불러도 된다)
    daq_pre_shutdown_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
      [this]() {
        if (daq_sampler_) {
          daq_sampler_->stop();
        }
      });
  }

  // --- 입력 (콜백은 RPC 를 부르지 않는다) ---
  sub_control_ = create_subscription<Control>(
    kTopicControlCmd, qos, [this](const Control::ConstSharedPtr msg) { on_control_cmd(msg); });
  sub_gear_ = create_subscription<GearCommand>(
    kTopicGearCmd, qos, [this](const GearCommand::ConstSharedPtr msg) { on_gear_cmd(msg); });
  sub_turn_indicators_ = create_subscription<TurnIndicatorsCommand>(
    kTopicTurnIndicatorsCmd, qos,
    [this](const TurnIndicatorsCommand::ConstSharedPtr msg) { on_turn_indicators_cmd(msg); });
  sub_hazard_lights_ = create_subscription<HazardLightsCommand>(
    kTopicHazardLightsCmd, qos,
    [this](const HazardLightsCommand::ConstSharedPtr msg) { on_hazard_lights_cmd(msg); });
  // 26.10.10 _ YSH _ follow 모드에서는 /initialpose(텔레포트)·control_mode_request(실차 인터페이스가 받는다)를 만들지 않는다
  // 26.10.13 _ YSH _ localize 도 만들지 않는다. follow 송출기(아래 else)는 follow 만. 원문: if (!follow_mode_) { … } else {
  if (!real_car_vehicle_) {
    sub_initialpose_ = create_subscription<PoseWithCovarianceStamped>(
      kTopicInitialPose, qos,
      [this](const PoseWithCovarianceStamped::ConstSharedPtr msg) { on_initialpose(msg); });

    // 지연 응답: 콜백은 작업 스레드에 넘기고, 작업 스레드가 RPC·되읽기 뒤 응답한다
    srv_control_mode_ = create_service<ControlModeCommand>(
      kServiceControlModeRequest,
      [this](
        std::shared_ptr<rclcpp::Service<ControlModeCommand>> service,
        std::shared_ptr<rmw_request_id_t> header,
        std::shared_ptr<ControlModeCommand::Request> request) {
        on_control_mode_request(service, header, request);
      });
  } else if (follow_mode_) {
    // 실차 위치 → MORAI Ego (SetTransform). 링크가 아직 안 붙었으면 실패로 세고 다음 틱에 다시 보낸다
    follow_ = std::make_unique<FollowStreamer>(
      follow_config_, [this](const morai_grpc_vendor::TransformPose & t) { return link_->set_transform(t); },
      [this]() -> std::optional<double> {
        const auto state = state_cache_.latest();
        if (!state) {
          return std::nullopt;
        }
        return state->ego.position.z;
      },
      [this]() { return runtime_params_.get()->map_origin; }, [this]() { return steady_now_ns(); });
    // 26.10.12 _ YSH _ 고스트 명령: 명령 채널 잠금(forward_control_to_sim=false)과 별개로 GHOST 하나만 이 경로로 보낸다
    follow_->set_ghost([this](double speed_mps) {
      morai_grpc_vendor::VehicleCommand cmd;
      cmd.long_cmd_type = morai_grpc_vendor::LongCmdType::kGhost;
      cmd.velocity = speed_mps * 3.6;  // VehicleCtrlCmd.velocity 는 km/h (gt yaml velocity_cmd_unit kph)
      return link_->control_vehicle(cmd, kGhostDeadline);
    });
    // 26.10.12 _ YSH _ 실차 측위는 DAQ Ego 에도 쓴다. 원문:
    // sub_follow_pose_ = create_subscription<nav_msgs::msg::Odometry>(
    //   follow_input_topic_, rclcpp::QoS(10),
    //   [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) { follow_->submit(*msg, steady_now_ns()); });
    sub_follow_pose_ = create_subscription<nav_msgs::msg::Odometry>(
      follow_input_topic_, rclcpp::QoS(10), [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        const int64_t now = steady_now_ns();
        follow_->submit(*msg, now);
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.pose = *msg;
        real_car_.pose_ns = now;
      });
    // 26.10.12 _ YSH _ 의사 VILS 2단계: 실차 상태 (K-CAN 인터페이스) — DAQ Ego 와 MORAI Ego 등화에 쓴다
    using autoware_vehicle_msgs::msg::HazardLightsReport;
    using autoware_vehicle_msgs::msg::SteeringReport;
    using autoware_vehicle_msgs::msg::TurnIndicatorsReport;
    using autoware_vehicle_msgs::msg::VelocityReport;
    real_car_subs_.push_back(create_subscription<VelocityReport>(
      kTopicVelocity, qos, [this](const VelocityReport::ConstSharedPtr m) {
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.velocity = *m;
        real_car_.velocity_ns = steady_now_ns();
      }));
    real_car_subs_.push_back(create_subscription<SteeringReport>(
      kTopicSteering, qos, [this](const SteeringReport::ConstSharedPtr m) {
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.steering = *m;
        real_car_.steering_ns = steady_now_ns();
      }));
    real_car_subs_.push_back(create_subscription<sensor_msgs::msg::Imu>(
      kTopicKcanImu, qos, [this](const sensor_msgs::msg::Imu::ConstSharedPtr m) {
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.kcan_imu = *m;
        real_car_.kcan_imu_ns = steady_now_ns();
      }));
    real_car_subs_.push_back(create_subscription<tier4_vehicle_msgs::msg::ActuationStatusStamped>(
      kTopicActuationStatus, qos, [this](const tier4_vehicle_msgs::msg::ActuationStatusStamped::ConstSharedPtr m) {
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.actuation = *m;
        real_car_.actuation_ns = steady_now_ns();
      }));
    real_car_subs_.push_back(create_subscription<geometry_msgs::msg::AccelWithCovarianceStamped>(
      kTopicLocalizationAccel, qos, [this](const geometry_msgs::msg::AccelWithCovarianceStamped::ConstSharedPtr m) {
        const std::lock_guard<std::mutex> lock(real_car_mutex_);
        real_car_.accel = *m;
        real_car_.accel_ns = steady_now_ns();
      }));
    // MORAI Ego 등화 = 실차 등화 상태 (Report 값은 Command 와 번호가 같다: DISABLE 1 · LEFT 2 · RIGHT 3 / ENABLE 2)
    real_car_subs_.push_back(create_subscription<TurnIndicatorsReport>(
      kTopicTurnIndicators, qos, [this](const TurnIndicatorsReport::ConstSharedPtr m) {
        lamp_cache_.set_turn_indicators(m->report);
        post_lamp_sync(false);
      }));
    real_car_subs_.push_back(create_subscription<HazardLightsReport>(
      kTopicHazardLights, qos, [this](const HazardLightsReport::ConstSharedPtr m) {
        lamp_cache_.set_hazard_lights(m->report);
        post_lamp_sync(false);
      }));
    follow_->start();
  }

  // 26.11.10 _ YSH _ 외란 명령 서비스 (launch 노드 이름 morai_bridge → /morai_bridge/set_disturbance). 기본 콜백 그룹 —
  //   콜백은 설정만 바꿔 끼우고 바로 답한다 (RPC·대기 없음). 실차 모드에도 있고, 거기서는 켜는 명령을 REAL_CAR 로 거절한다
  if (disturbance_enable_) {
    srv_disturbance_ = create_service<SetDisturbance>(
      "~/set_disturbance",
      [this](
        const std::shared_ptr<SetDisturbance::Request> request,
        const std::shared_ptr<SetDisturbance::Response> response) {
        on_set_disturbance(request, response);
      });
  }

  lamp_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { on_lamp_timer(); });
  // 26.09.25 _ YSH _ 리뷰 반영: 2 s → 1 s
  guard_timer_ = create_wall_timer(kGuardPeriod, [this]() { on_guard_timer(); });
  // 26.10.10 _ YSH _ follow 모드에서는 측위 초기화를 GNSS 노드가 한다. 원문: if (engine_config_.publish_ground_truth) {
  if (engine_config_.publish_ground_truth && !follow_mode_) {
    init_localization_client_ = create_client<InitializeLocalization>(kServiceInitializeLocalization);
    localization_init_timer_ =
      create_wall_timer(std::chrono::seconds(1), [this]() { on_localization_init_timer(); });
  }

  // --- 진단 ---
  updater_ = std::make_unique<diagnostic_updater::Updater>(this);
  updater_->setHardwareID("morai@" + link_->target());
  updater_->add("link", this, &MoraiInterfaceNode::diagnose_link);
  updater_->add("update", this, &MoraiInterfaceNode::diagnose_update);
  updater_->add("command", this, &MoraiInterfaceNode::diagnose_command);
  updater_->add("clock", this, &MoraiInterfaceNode::diagnose_clock);
  updater_->add("ground_truth", this, &MoraiInterfaceNode::diagnose_ground_truth);
  if (publish_daq_) {
    updater_->add("daq", this, &MoraiInterfaceNode::diagnose_daq);  // 26.09.26 _ YSH _
  }
  if (follow_mode_) {
    updater_->add("follow", this, &MoraiInterfaceNode::diagnose_follow);  // 26.10.10 _ YSH _
  }

  on_set_handle_ = add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      return on_set_parameters(parameters);
    });

  const auto & rp = runtime_params_value_;
  // 26.09.25 _ YSH _ 명령 트리거를 같이 적는다 (A/B 로그를 가르는 첫 줄). 원문:
  // RCLCPP_INFO(
  //   get_logger(),
  //   "MORAI 인터페이스(C++ 엔진) %s ego '%s' · time_source %s · publish_ground_truth %s · publish_clock %s · "
  //   "forward_control_to_sim %s · steer_comp %s · long_cmd_mode %s · velocity_cmd_scale %g · "
  //   "stale %.0f ms → %.1f m/s^2 · 폴백 %.0f Hz",
  //   link_->target().c_str(), link_options_.ego_id.c_str(), to_string(engine_config_.time_source),
  //   engine_config_.publish_ground_truth ? "true" : "false",
  //   engine_config_.publish_clock ? "true" : "false", rp.forward_control_to_sim ? "true" : "false",
  //   rp.command.steer_comp.enable ? "on" : "off",
  //   autoware_to_morai::to_string(rp.command.longitudinal.mode).c_str(),
  //   rp.command.longitudinal.velocity_cmd_scale, engine_config_.cmd_stale_timeout_s * 1000.0,
  //   engine_config_.cmd_stale_accel, engine_config_.fallback_hz);
  char trigger_desc[96];
  if (rp.command_trigger == CommandTrigger::kTimer) {
    std::snprintf(
      trigger_desc, sizeof(trigger_desc), "timer (%g Hz 고정 주기, main.py 조건)", rp.cmd_resend_hz);
  } else {
    std::snprintf(trigger_desc, sizeof(trigger_desc), "update (MORAI 업데이트당 1 회)");
  }
  RCLCPP_INFO(
    get_logger(),
    "MORAI 인터페이스(C++ 엔진) %s ego '%s' · time_source %s · publish_ground_truth %s · publish_clock %s · "
    "forward_control_to_sim %s · steer_comp %s · long_cmd_mode %s · velocity_cmd_scale %g · "
    "stale %.0f ms → %.1f m/s^2 · 폴백 %.0f Hz · command_trigger %s",
    link_->target().c_str(), link_options_.ego_id.c_str(), to_string(engine_config_.time_source),
    engine_config_.publish_ground_truth ? "true" : "false",
    engine_config_.publish_clock ? "true" : "false", rp.forward_control_to_sim ? "true" : "false",
    rp.command.steer_comp.enable ? "on" : "off",
    autoware_to_morai::to_string(rp.command.longitudinal.mode).c_str(),
    rp.command.longitudinal.velocity_cmd_scale, engine_config_.cmd_stale_timeout_s * 1000.0,
    engine_config_.cmd_stale_accel, engine_config_.fallback_hz, trigger_desc);

  // 26.09.26 _ YSH _ DAQ 설정 한 줄 (위 줄은 그대로 둔다 — A/B 로그 파서가 읽는다)
  if (publish_daq_) {
    RCLCPP_INFO(
      get_logger(),
      // 26.10.05 _ YSH _ 원문: "DAQ: %s/{vehicle_state,acceleration,track_objects%s,vehicle_cmd} %g Hz · mode %s (gated = %s/run_state "
      // 26.10.06 _ YSH _ ctrl_cmd 추가. 원문: "DAQ: %s/{ego_status,object_status%s,vehicle_cmd} %g Hz · mode %s (gated = %s/run_state "
      // 26.10.07 _ YSH _ collision 추가. 원문: "DAQ: %s/{ego_status,object_status%s,ctrl_cmd,vehicle_cmd} %g Hz · mode %s (gated = %s/run_state "
      "DAQ: %s/{ego_status,object_status%s,ctrl_cmd,vehicle_cmd,collision%s} %g Hz · mode %s (gated = %s/run_state "
      "active 동안만) · stamp %s · 표본 나이 < %.0f ms · 명령 stale %.0f ms · 자동 종료 %.0f s",
      daq_prefix_.c_str(), daq_config_.publish_objects ? "" : "(끔)",
      daq_config_.publish_collision ? "" : "(끔)", daq_config_.rate_hz,
      to_string(daq_mode_initial_), daq_prefix_.c_str(), to_string(daq_config_.stamp),
      daq_config_.max_sample_age_s * 1000.0, daq_config_.cmd_stale_s * 1000.0, daq_config_.run_timeout_s);
  }
  // 26.10.05 _ YSH _ 명세 10~11(roll·pitch)은 MORAI 메시지에 칸이 없어 raw/actor_states 에서 읽는다 — 꺼져 있으면 CSV 가 빈다
  if (publish_daq_ && !daq_config_.raw) {
    RCLCPP_WARN(
      get_logger(),
      "daq.raw=false — 명세 10~11(roll·pitch)은 %s/raw/actor_states 에서만 나온다. 이대로 녹화하면 daq_all.csv 의 roll·pitch 가 빈다",
      daq_prefix_.c_str());
  }
  // 26.09.26 _ YSH _ 원시 gRPC 기록 설정은 새 줄로 (위 DAQ 줄은 그대로 둔다 — 파서가 읽는다)
  if (publish_daq_ && daq_config_.raw) {
    RCLCPP_INFO(
      get_logger(),
      "DAQ raw: %s/raw/{actor_states,vehicle_ctrl_cmd} std_msgs/String proto3 JSON · 받아들인 MORAI 업데이트마다·"
      "ControlVehicle 마다 (ZOH 아님) · /daq/* 와 같은 게이트 · 큐 %zu (넘치면 가장 오래된 것부터 버리고 센다)",
      daq_prefix_.c_str(), daq_config_.raw_queue_depth);
  }
  // 26.11.10 _ YSH _ 외란 주입 설정 한 줄 (새 줄 — 위 줄들은 파서가 읽는다)
  if (disturbance_enable_) {
    const char * engine_desc = "켬 (객체 프레임마다)";
    if (!disturbance_engine_) {
      engine_desc = real_car_vehicle_ ? "끔 (실차 모드 — 켜는 명령은 REAL_CAR 로 거절)" :
                    "끔 (publish_ground_truth false — 켜는 명령은 NO_GROUND_TRUTH 로 거절)";
    }
    const char * daq_desc = "없음";
    if (pub_daq_disturbance_) {
      // 26.11.11 _ YSH _ object_disturbance_status 추가. 원문:
      // daq_desc = pub_daq_objects_disturbed_ ? "disturbance·object_status_disturbed (게이트가 열린 동안)" :
      daq_desc = pub_daq_objects_disturbed_ ?
                   "disturbance·object_status_disturbed·object_disturbance_status (게이트가 열린 동안)" :
                   "disturbance (게이트가 열린 동안)";
    }
    RCLCPP_INFO(
      get_logger(), "외란 주입(MRDIST-1): 서비스 %s/set_disturbance · 엔진 적용 %s · DAQ %s",
      get_fully_qualified_name(), engine_desc, daq_desc);
    // 26.11.11 _ YSH _ DAQ 를 안 내면 외란 기록이 bag 에 없고, 런 경계(cue_server 의 clear)가 없을 수 있다 — 알린다
    if (disturbance_engine_ && !publish_daq_) {
      RCLCPP_WARN(
        get_logger(),
        "외란 주입: publish_daq=false — /daq/disturbance 기록이 없다. 런이 끝나도 외란은 clear 명령이 올 때까지 걸려 있다");
    }
  }

  // 생성자는 막히지 않는다 — 연결·스펙은 엔진 스레드가 재시도한다
  worker_->start();
  mode_worker_->start();
  gear_sender_->start();
  engine_->start();
  if (daq_sampler_) {
    daq_sampler_->start();  // 26.09.26 _ YSH _
  }
}

MoraiInterfaceNode::~MoraiInterfaceNode()
{
  // 26.09.26 _ YSH _ DAQ 먼저 (켜진 런이 있으면 'shutdown' 요약을 낸다 — Ctrl-C 면 pre-shutdown 콜백이 이미 멈췄다.
  //   컨테이너 unload 처럼 컨텍스트가 살아 있는 경로는 여기서). 콜백을 먼저 떼어 이미 지워진 this 를 부르지 않게
  if (daq_sampler_) {
    get_node_base_interface()->get_context()->remove_pre_shutdown_callback(daq_pre_shutdown_);
    daq_sampler_->stop();
  }
  // 26.10.10 _ YSH _ follow 스트리머는 링크를 쓰므로 엔진보다 먼저 멈춘다
  if (follow_) {
    follow_->stop();
  }
  // 엔진 먼저 (최근 송신 중이었으면 ~0.3 s 감속 명령 뒤 종료), 그다음 보조 스레드
  if (engine_) {
    engine_->stop();
  }
  if (gear_sender_) {
    gear_sender_->stop();
  }
  if (mode_worker_) {
    mode_worker_->stop();
  }
  if (worker_) {
    worker_->stop();
  }
  // 26.09.26 _ YSH _ 리뷰 반영: DAQ 를 publisher 보다 먼저 지운다. 멤버는 선언 역순으로 지워져 pub_daq_* 가 daq_sampler_
  //   보다 먼저 사라진다 — ~DaqSampler 가 sink(this) 를 부르면 지워진 publisher 를 쓴다. 모든 스레드를 거둔 뒤라 안전하다
  daq_sampler_.reset();
}

int64_t MoraiInterfaceNode::steady_now_ns() const
{
  return static_cast<int64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                std::chrono::steady_clock::now().time_since_epoch())
                                .count());
}

// =============================================================================
// 파라미터
// =============================================================================

void MoraiInterfaceNode::declare_parameters()
{
  auto & rp = runtime_params_value_;

  // --- main.py 이름, 런타임 변경 가능 (on-set 콜백) ---
  rp.forward_control_to_sim = declare_parameter<bool>(
    "forward_control_to_sim", true,
    describe("false 면 제어·기어·모드 요청을 MORAI 로 보내지 않는다 (시뮬 키보드 수동 주행)", false));
  rp.command.steer_cmd_sign = declare_parameter<double>(
    "steer_cmd_sign", 1.0,
    describe("제어 명령 조향 부호. 리포트 쪽 -1 과 일부러 비대칭 (context-notes '절대 통일하지 말 것')", false));
  rp.command.steer_comp.enable = declare_parameter<bool>(
    "steer_comp_enable", true, describe("MORAI 조향 결손 보정 (속도별 배율로 나눠 보낸다)", false));
  rp.command.steer_comp.speeds_mps = declare_parameter<std::vector<double>>(
    "steer_comp_speeds_mps", std::vector<double>{2.0, 5.0, 8.0, 11.0},
    describe("보정 표 속도 [m/s], 엄격 오름차순", false));
  rp.command.steer_comp.gains = declare_parameter<std::vector<double>>(
    "steer_comp_gains", std::vector<double>{0.870, 0.855, 0.807, 0.796},
    describe("보정 표 배율 (0.5~1.5, 속도와 길이 같음)", false));
  rp.command.longitudinal.send_pedal_with_accel = declare_parameter<bool>(
    "send_pedal_with_accel", false,
    describe("acceleration 모드에서 throttle/brake 페달값을 같이 채운다", false));
  rp.command.longitudinal.pedal_accel_max = declare_parameter<double>(
    "pedal_accel_max", 3.0, describe("throttle = clip(a / 이 값, 0, 1) [m/s^2]", false));
  rp.command.longitudinal.pedal_brake_max = declare_parameter<double>(
    "pedal_brake_max", 4.5, describe("brake = clip(-a / 이 값, 0, 1) [m/s^2]", false));
  const std::string long_mode = declare_parameter<std::string>(
    "long_cmd_mode", "acceleration",
    describe("acceleration | throttle | velocity (main.py _fill_longitudinal)", false));
  if (const auto m = autoware_to_morai::parse_long_cmd_mode(long_mode)) {
    rp.command.longitudinal.mode = *m;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "long_cmd_mode='%s' 는 모르는 값 → 'acceleration' 으로 동작 (acceleration|throttle|velocity 만 유효)",
      long_mode.c_str());
    rp.command.longitudinal.mode = autoware_to_morai::LongCmdMode::kAcceleration;
  }
  rp.steer_report_hold = declare_parameter<bool>(
    "steer_report_hold", true, describe("조향 리포트 0 드롭아웃이면 직전값 유지", false));
  // 26.10.01 _ YSH _ 외부 조향 필터 (기본 끔). 켜는 법: run_rig.sh filter on | STEER_FILTER=1 run_rig.sh
  rp.steer_report_ignore_foreign = declare_parameter<bool>(
    "steer_report_ignore_foreign", false,
    describe("우리가 보내지 않은 조향이 적용된 프레임의 조향 리포트를 직전 우리 값으로 대신한다", false));
  rp.steer_input_norm_deg = declare_parameter<double>(
    "steer_input_norm_deg", 40.0,
    describe("MORAI front_wheel_angle 1.0 의 조향각 [deg] (외부 조향 판정용, 실측 40)", false));
  if (!std::isfinite(rp.steer_input_norm_deg) || rp.steer_input_norm_deg <= 0.0) {
    RCLCPP_WARN(get_logger(), "steer_input_norm_deg 가 양수가 아니다 → 40 으로 동작");
    rp.steer_input_norm_deg = 40.0;
  }
  if (rp.steer_report_ignore_foreign) {
    RCLCPP_INFO(get_logger(), "steer_report_ignore_foreign true — 외부 조향 필터 켬 (기준각 %g°)", rp.steer_input_norm_deg);
  }
  rp.map_origin.x = declare_parameter<double>(
    "map_origin_offset_x", 0.0, describe("map = R(yaw) * morai + offset [m]", false));
  rp.map_origin.y =
    declare_parameter<double>("map_origin_offset_y", 0.0, describe("[m]", false));
  rp.map_origin.z =
    declare_parameter<double>("map_origin_offset_z", 0.0, describe("[m]", false));
  rp.map_origin.yaw_deg =
    declare_parameter<double>("map_origin_yaw_offset_deg", 0.0, describe("[deg]", false));
  rp.teleport_on_initialpose = declare_parameter<bool>(
    "teleport_on_initialpose", true,
    describe("/initialpose(RViz 2D Pose Estimate) 로 MORAI Ego 를 옮긴다", false));
  // 26.09.25 _ YSH _ A/B: ControlVehicle 트리거 (update = MORAI 업데이트당 1 회 / timer = main.py 식 고정 주기).
  //   런타임 전환 가능 — 같은 주행 중에 `ros2 param set /morai_bridge command_trigger timer` 로 가른다
  const std::string trigger = declare_parameter<std::string>(
    "command_trigger", "update",
    describe(
      "ControlVehicle 트리거: update (MORAI 업데이트당 1 회, 기본) | timer (최신 명령을 cmd_resend_hz 고정 주기로 "
      "동기 송신 — main.py _CmdResender 조건)",
      false));
  if (const auto t = parse_command_trigger(trigger)) {
    rp.command_trigger = *t;
  } else {
    // 26.09.25 _ YSH _ 리뷰 반영: 파라미터 서버에도 실제로 도는 값을 써 둔다 — 전에는 대체만 하고 서버에 틀린 값('Timer')이
    //   남아 `ros2 param get /morai_bridge command_trigger` 가 돌지 않는 모드를 보였다 (A/B 중 오타를 timer 로 착각).
    //   on-set 콜백은 아직 등록 전이라 이 set 은 콜백을 거치지 않는다. 원문:
    // RCLCPP_WARN(
    //   get_logger(), "command_trigger='%s' 는 모르는 값 → 'update' 로 동작 (update|timer 만 유효)",
    //   trigger.c_str());
    RCLCPP_WARN(
      get_logger(),
      "command_trigger='%s' 는 모르는 값 → 'update' 로 동작 (update|timer 만 유효, 파라미터도 update 로 바꿔 둔다)",
      trigger.c_str());
    rp.command_trigger = CommandTrigger::kUpdate;
    set_parameter(rclcpp::Parameter("command_trigger", std::string(to_string(rp.command_trigger))));
  }
  auto resend_desc = describe(
    "command_trigger=timer 일 때 ControlVehicle 주기 [Hz] (main.py 와 같은 이름·기본 100, 10~400, 정수도 받는다). "
    "update 면 쓰지 않는다",
    false);
  resend_desc.dynamic_typing = true;
  const auto resend = as_number(
    declare_parameter("cmd_resend_hz", rclcpp::ParameterValue(kCmdResendHzDefault), resend_desc));
  rp.cmd_resend_hz = resend.value_or(std::nan(""));
  if (!cmd_resend_hz_is_valid(rp.cmd_resend_hz)) {
    // 26.09.25 _ YSH _ 리뷰 반영: 대체값을 파라미터 서버에도 쓴다 (위 command_trigger 와 같은 이유). 원문:
    // RCLCPP_WARN(
    //   get_logger(), "cmd_resend_hz=%g 은 %g~%g Hz 의 숫자가 아니다 → %g Hz", rp.cmd_resend_hz, kCmdResendHzMin,
    //   kCmdResendHzMax, kCmdResendHzDefault);
    RCLCPP_WARN(
      get_logger(), "cmd_resend_hz=%g 은 %g~%g Hz 의 숫자가 아니다 → %g Hz (파라미터도 %g 로 바꿔 둔다)",
      rp.cmd_resend_hz, kCmdResendHzMin, kCmdResendHzMax, kCmdResendHzDefault, kCmdResendHzDefault);
    rp.cmd_resend_hz = kCmdResendHzDefault;
    set_parameter(rclcpp::Parameter("cmd_resend_hz", rp.cmd_resend_hz));
  }

  // --- main.py 이름, 기동 시 1 회 (main.py 도 재시작해야 반영) ---
  engine_config_.cmd_stale_timeout_s = declare_parameter<double>(
    "cmd_stale_timeout_s", 0.2,
    describe("control_cmd 가 이보다 오래되면 감속 대체 명령 [s]", true));
  const double stale_accel = declare_parameter<double>(
    "cmd_stale_accel", -2.4, describe("stale 일 때 보낼 종방향 가속도 [m/s^2], 음수만", true));
  bool replaced = false;
  engine_config_.cmd_stale_accel = autoware_to_morai::sanitize_stale_accel(stale_accel, &replaced);
  if (replaced) {
    RCLCPP_WARN(
      get_logger(), "cmd_stale_accel=%g 은 감속이 아니다 → -2.4 m/s^2 로 대체 (음수만 유효)",
      stale_accel);
  }
  if (!(engine_config_.cmd_stale_timeout_s > 0.0)) {
    RCLCPP_WARN(
      get_logger(), "cmd_stale_timeout_s=%g 은 양수가 아니다 → 0.2 s", engine_config_.cmd_stale_timeout_s);
    engine_config_.cmd_stale_timeout_s = 0.2;
  }
  const std::string unit = declare_parameter<std::string>(
    "velocity_cmd_unit", "kph",
    describe("VehicleCtrlCmd.velocity 단위: kph (m/s × 3.6) | mps | off", true));
  if (const auto scale = autoware_to_morai::parse_velocity_cmd_unit(unit)) {
    rp.command.longitudinal.velocity_cmd_scale = *scale;
  } else {
    RCLCPP_WARN(get_logger(), "velocity_cmd_unit=%s 은 kph|mps|off 가 아니다 → kph 로 진행", unit.c_str());
    rp.command.longitudinal.velocity_cmd_scale = 3.6;
  }
  gear_config_.resync_s = declare_parameter<double>(
    "gear_resync_s", 1.0, describe("MORAI 기어 불일치 재송신 간격 [s]", true));
  gear_config_.retry_s = declare_parameter<double>(
    "gear_retry_s", 0.2, describe("기어 송신 실패 뒤 같은 기어 재시도 간격 [s]", true));

  // --- 신규 (기동 시 1 회) ---
  // 26.09.27 _ YSH _ 기본값을 원래 컴퓨터의 MORAI PC(10.10.10.1)로 — 리그는 주소를 넘기지 않는다. 원문:
  // link_options_.address = declare_parameter<std::string>(
  //   "morai.address", env_or("MORAI_SIM_ADDRESS", "127.0.0.1"),
  //   describe("MORAI PC 주소 (기본: 환경변수 MORAI_SIM_ADDRESS, 없으면 127.0.0.1)", true));
  link_options_.address = declare_parameter<std::string>(
    "morai.address", env_or("MORAI_SIM_ADDRESS", "10.10.10.1"),
    describe("MORAI PC 주소 (기본: 환경변수 MORAI_SIM_ADDRESS, 없으면 10.10.10.1)", true));
  const int64_t port = declare_parameter<int64_t>(
    "morai.port", env_port_or("MORAI_SIM_PORT", 7789),
    describe("MORAI gRPC 포트 (기본: 환경변수 MORAI_SIM_PORT, 없으면 7789)", true));
  if (port < 1 || port > 65535) {
    RCLCPP_WARN(get_logger(), "morai.port=%ld 범위 밖 → 7789", static_cast<long>(port));  // NOLINT
    link_options_.port = 7789;
  } else {
    link_options_.port = static_cast<uint16_t>(port);
  }
  link_options_.ego_id = declare_parameter<std::string>(
    "morai.ego_name", "Ego", describe("ObjectInfo.id.value (문자열, MORAI 기본 'Ego')", true));

  auto deadline = [this](const std::string & name, int64_t def, const std::string & text) {
    const int64_t v = declare_parameter<int64_t>(name, def, describe(text + " [ms]", true));
    return std::chrono::milliseconds(std::max<int64_t>(v, 1));
  };
  engine_config_.wait_for_tick_deadline =
    deadline("morai.wait_for_tick_deadline_ms", 100, "WaitForTick deadline (업데이트 트리거)");
  link_options_.deadlines.state =
    deadline("morai.state_deadline_ms", 100, "GetAllActorsState·GetVehicleControlMode deadline");
  link_options_.deadlines.control_mode = link_options_.deadlines.state;
  // 26.09.25 _ YSH _ 리뷰 반영: 하한으로 쓰고 실제 값은 max(이 값, 4 × MORAI 왕복 p50), 상한 1 s (main.py cmd_timeout_for_rtt)
  link_options_.deadlines.command = deadline(
    "morai.command_deadline_ms", 50,
    "ControlVehicle deadline 하한 (main.py cmd_rpc_timeout_s). 실제 값은 max(이 값, 4 × 왕복 p50), 상한 1 s");
  engine_config_.command_deadline = link_options_.deadlines.command;
  link_options_.deadlines.gear =
    deadline("morai.gear_deadline_ms", 200, "SetVehicleGear deadline (main.py gear_rpc_timeout_s)");
  link_options_.deadlines.set_mode =
    deadline("morai.control_mode_deadline_ms", 200, "SetVehicleControlMode deadline");
  link_options_.deadlines.transform =
    deadline("morai.transform_deadline_ms", 1000, "SetTransform deadline (/initialpose)");
  link_options_.deadlines.tail_light =
    deadline("morai.tail_light_deadline_ms", 200, "SetVehicleTailLight deadline");
  link_options_.deadlines.spec =
    deadline("morai.spec_deadline_ms", 3000, "GetVehicleSpec deadline");
  link_options_.keepalive_time_ms = static_cast<int32_t>(declare_parameter<int64_t>(
    "morai.keepalive_time_ms", 0,
    describe("gRPC keepalive [ms], 0 = 끔 (MORAI PING 정책 확인 전까지 끔)", true)));

  const std::string time_source = declare_parameter<std::string>(
    "time_source", "morai",
    describe("발행 stamp: morai (ActorStates.timestamp) | receive (이 PC 수신 시각)", true));
  if (const auto ts = parse_time_source(time_source)) {
    engine_config_.time_source = *ts;
  } else {
    RCLCPP_WARN(get_logger(), "time_source=%s 은 morai|receive 가 아니다 → morai", time_source.c_str());
  }
  engine_config_.publish_clock = declare_parameter<bool>(
    "publish_clock", false, describe("/clock 에 메시지 stamp(기본 MORAI 샘플 시각)를 업데이트마다 발행", true));
  // 26.09.25 _ YSH _ 리뷰 반영: /clock 은 메시지 stamp 와 같은 시각을 싣는다 — receive 면 수신 시각
  if (engine_config_.publish_clock && engine_config_.time_source == TimeSource::kReceive) {
    RCLCPP_WARN(
      get_logger(),
      "publish_clock=true + time_source=receive: /clock 은 MORAI 시각이 아니라 이 PC 수신 시각을 싣는다 "
      "(메시지 stamp 와 맞추려고). MORAI 시각이 필요하면 time_source=morai");
  }
  engine_config_.max_sample_age_s = declare_parameter<double>(
    "max_sample_age_s", 0.1, describe("새 표본이 이보다 오래 없으면 진단 ERROR [s]", true));
  // 26.09.25 _ YSH _ 리뷰 반영: 예약 — 액터는 Ego 와 같은 GetAllActorsState 에서 와 따로 낡을 수 없다. 새 표본이
  //   없으면 TrackedObjects 도 나가지 않는다 (max_sample_age_s). 액터를 따로 조회하게 되면 쓴다
  (void)declare_parameter<double>(
    "actor_hold_s", 0.5,
    describe(
      "예약(지금은 효과 없음): 액터가 Ego 와 같은 호출에서 와서 따로 낡지 않는다 — 새 표본이 없으면 "
      "TrackedObjects 도 멈춘다",
      true));
  engine_config_.fallback_hz = declare_parameter<double>(
    "fallback_hz", 100.0, describe("WaitForTick 이 안 될 때 폴링+송신 주기 [Hz]", true));
  if (!(engine_config_.fallback_hz > 0.0)) {
    RCLCPP_WARN(get_logger(), "fallback_hz=%g → 100", engine_config_.fallback_hz);
    engine_config_.fallback_hz = 100.0;
  }
  engine_config_.auto_reassert_auto = declare_parameter<bool>(
    "auto_reassert_auto", true,
    describe("재연결 뒤 MORAI 가 KEYBOARD 면, 마지막 요청이 AUTO 였을 때 AUTO 로 재지정", true));
  engine_config_.engage_on_start = declare_parameter<bool>(
    "engage_on_start", false, describe("첫 연결 때 MORAI 를 AUTO 로 둔다", true));
  engine_config_.publish_ground_truth = declare_parameter<bool>(
    "publish_ground_truth", true,
    describe("kinematic_state·map→base_link TF·TrackedObjects·측위 초기화 (GT 모드)", true));
  engine_config_.clock_offset_warn_ms = declare_parameter<double>(
    "clock_offset_warn_ms", 20.0,
    describe("|MORAI timestamp − 수신 시각| 중앙값이 이보다 크면 진단 WARN (chrony 확인) [ms]", true));
  sensor_model_ = declare_parameter<std::string>(
    "sensor_model", "",
    describe("비우면 센서를 건드리지 않는다. 값이 있으면 그 센서킷대로 MORAI 에 장착", true));
  // 26.09.25 _ YSH _ 리뷰 반영: 가드를 끌 수 있게 (bag 재생·relay 가 velocity_status 를 낼 때). 꺼도 감지·진단은 한다
  duplicate_guard_enabled_ = declare_parameter<bool>(
    "duplicate_writer_guard", true,
    describe(
      "다른 publisher 가 /vehicle/status/velocity_status 를 내면 ControlVehicle 을 막는다 (false = 진단만)",
      true));

  // 26.09.26 _ YSH _ DAQ (ViLS 명세 25 항목 100 Hz, daq_publisher.py 대체). publish_daq 만 켜면 된다 —
  //   daq.mode gated 면 cue_server 의 /daq/run_state(active/idle) 사이에서만 낸다
  publish_daq_ = declare_parameter<bool>(
    "publish_daq", false,
    // 26.10.05 _ YSH _ 원문: describe("/daq/{vehicle_state,acceleration,track_objects,vehicle_cmd} 발행 (daq_publisher.py 대체)", true));
    describe("/daq/{ego_status,object_status,vehicle_cmd} 발행 (MORAI 메시지, daq_publisher.py 대체)", true));
  const std::string daq_mode = declare_parameter<std::string>(
    "daq.mode", "gated",
    describe(
      "(런타임) gated = /daq/run_state 가 active 인 동안만 (cue_server 녹화 구간) | always = 늘", false));
  if (const auto m = parse_daq_mode(daq_mode)) {
    daq_mode_initial_ = *m;
  } else {
    RCLCPP_WARN(get_logger(), "daq.mode=%s 은 gated|always 가 아니다 → gated", daq_mode.c_str());
  }
  daq_config_.rate_hz = declare_parameter<double>(
    "daq.rate_hz", 100.0, describe("DAQ 발행 주기 [Hz] (명세 100, 10~400)", true));
  if (!daq_rate_is_valid(daq_config_.rate_hz)) {
    RCLCPP_WARN(get_logger(), "daq.rate_hz=%g 는 10~400 밖 → 100", daq_config_.rate_hz);
    daq_config_.rate_hz = 100.0;
  }
  daq_config_.max_sample_age_s = declare_parameter<double>(
    "daq.max_sample_age_s", 0.1,
    describe("MORAI 표본이 이만큼 낡으면 상태 토픽을 멈추고 결손으로 센다 [s] (daq_publisher 와 같음)", true));
  daq_config_.cmd_stale_s = declare_parameter<double>(
    "daq.cmd_stale_s", 0.2,
    describe("새 control_cmd 가 이만큼 없으면 /daq/vehicle_cmd 를 멈춘다 [s] (0 을 지어내지 않음)", true));
  daq_config_.accel_scale = declare_parameter<double>(
    "daq.accel_scale", 1.0, describe("MORAI acceleration 배율 (daq_publisher --accel-scale)", true));
  daq_config_.publish_objects = declare_parameter<bool>(
    // 26.10.05 _ YSH _ 원문: "daq.publish_objects", true, describe("/daq/track_objects 발행", true));
    "daq.publish_objects", true, describe("/daq/object_status 발행", true));
  // 26.10.07 _ YSH _ 신규
  daq_config_.publish_collision = declare_parameter<bool>(
    "daq.publish_collision", true,
    describe("/daq/collision 발행 (gRPC Ego collision_objects → morai_ros2_msgs/CollisionData)", true));
  const std::string daq_stamp = declare_parameter<std::string>(
    "daq.stamp", "sample",
    describe(
      "sample = MORAI 표본 시각(반복 행은 같은 stamp)·control_cmd 원래 stamp | tick = 발행 시각 (daq_publisher 방식)",
      true));
  if (const auto st = parse_daq_stamp(daq_stamp)) {
    daq_config_.stamp = *st;
  } else {
    RCLCPP_WARN(get_logger(), "daq.stamp=%s 은 sample|tick 이 아니다 → sample", daq_stamp.c_str());
  }
  daq_config_.run_timeout_s = declare_parameter<double>(
    "daq.run_timeout_s", 1200.0,
    describe("active 가 이보다 길면 자동 종료 [s] (cue_server 사망 대비, 0 = 끔)", true));
  daq_prefix_ = declare_parameter<std::string>(
    "daq.topic_prefix", "/daq", describe("DAQ 토픽 접두 (daq/spec.py 와 같아야 한다)", true));
  // 26.09.26 _ YSH _ 원시 gRPC 기록 — gRPC 가 준 값(ActorStates 응답 전체)과 보낸 VehicleCtrlCmd 를 그대로 JSON 으로.
  //   기존 /daq/* 는 그대로 두고 두 토픽을 더한다. publish_daq 가 꺼져 있으면 쓰지 않는다
  daq_config_.raw = declare_parameter<bool>(
    "daq.raw", false,
    describe(
      "<daq.topic_prefix>/raw/{actor_states,vehicle_ctrl_cmd} 발행 — 받아들인 MORAI 업데이트마다 GetAllActorsState 응답 "
      "전체, ControlVehicle 마다 보낸 요청 (std_msgs/String proto3 JSON, /daq/* 와 같은 게이트). publish_daq 필요",
      true));
  const int64_t raw_depth = declare_parameter<int64_t>(
    "daq.raw_queue_depth", 256,
    describe("원시 기록 큐 깊이 (종류마다, 엔진·명령 스레드 → DAQ 스레드). 넘치면 가장 오래된 것을 버리고 센다", true));
  if (raw_depth < 1 || raw_depth > 100000) {
    RCLCPP_WARN(
      get_logger(), "daq.raw_queue_depth=%ld 는 1~100000 밖 → 256", static_cast<long>(raw_depth));  // NOLINT
    daq_config_.raw_queue_depth = 256;
  } else {
    daq_config_.raw_queue_depth = static_cast<size_t>(raw_depth);
  }
  if (daq_config_.raw && !publish_daq_) {
    RCLCPP_WARN(get_logger(), "daq.raw=true 인데 publish_daq=false — 원시 기록을 내지 않는다");
    daq_config_.raw = false;
  }
  // 링크가 받은 응답을 바이트로 싣게 하고(엔진 스레드는 복사만), 보낸 요청은 같은 ObjectInfo 로 다시 만든다
  link_options_.capture_raw = daq_config_.raw;
  daq_config_.raw_link = link_options_;

  // 26.10.10 _ YSH _ 의사 VILS: ego_mode follow — 실차 측위를 MORAI Ego 에 옮긴다 (기동 때 한 번 읽는다)
  // 26.10.13 _ YSH _ localize 를 더했다 (VILS 차량 PC 고스트 — MORAI Ego 가 측위, 실차가 차량). 원문:
  // const std::string ego_mode = declare_parameter<std::string>(
  //   "ego_mode", "gt",
  //   describe(
  //     "gt = MORAI Ego 를 Autoware 차량으로 (지금까지) | follow = 실차 측위(follow.input_topic)를 MORAI Ego 에 "
  //     "SetTransform 으로 옮긴다 (의사 VILS — 명령·텔레포트·TF·차량 상태를 내지 않는다)",
  //     true));
  // if (ego_mode != "gt" && ego_mode != "follow") {
  //   throw std::invalid_argument("ego_mode=" + ego_mode + " — gt | follow");
  // }
  const std::string ego_mode = declare_parameter<std::string>(
    "ego_mode", "gt",
    describe(
      "gt = MORAI Ego 를 Autoware 차량으로 (지금까지) | follow = 실차 측위(follow.input_topic)를 MORAI Ego 에 "
      "SetTransform 으로 옮긴다 (의사 VILS — 명령·텔레포트·TF·차량 상태를 내지 않는다) | localize = MORAI Ego 를 "
      "측위로만 낸다 (VILS 차량 PC 고스트 — kinematic_state·TF·측위 초기화만, 차량 상태는 /morai/ego/*, MORAI 로 쓰지 않는다)",
      true));
  if (ego_mode != "gt" && ego_mode != "follow" && ego_mode != "localize") {
    throw std::invalid_argument("ego_mode=" + ego_mode + " — gt | follow | localize");
  }
  ego_mode_name_ = ego_mode;
  follow_mode_ = ego_mode == "follow";
  localize_mode_ = ego_mode == "localize";
  real_car_vehicle_ = follow_mode_ || localize_mode_;
  // 26.11.13 _ YSH _ 13_ 시각화 전용: MORAI Ego 를 사람이 조종할 때 등화 동기화(SetVehicleTailLight)도 끈다. 기본 true = 예전과 같다
  lamp_sync_enable_ = declare_parameter<bool>(
    "lamp_sync", true,
    describe("false 면 MORAI 로 방향지시등·비상등(SetVehicleTailLight)을 보내지 않는다 (연결 때 강제 재전송 포함)", true));
  // 26.11.10 _ YSH _ 외란 주입. 코드 기본 false = 서비스·토픽·엔진 경로가 전부 예전과 같다 (11_ GT yaml 이 켠다).
  //   실차 모드(follow·localize)는 GT yaml 위에 덧씌워 뜨므로 서비스는 생기지만, 엔진에는 손잡이를 주지 않고
  //   켜는 명령은 REAL_CAR 로 거절한다
  disturbance_enable_ = declare_parameter<bool>(
    "disturbance.enable", false,
    describe(
      // 26.11.11 _ YSH _ object_disturbance_status 추가. 원문:
      // "외란 주입(MRDIST-1): 서비스 ~/set_disturbance 와 <daq.topic_prefix>/{disturbance,object_status_disturbed}. "
      "외란 주입(MRDIST-1): 서비스 ~/set_disturbance 와 "
      "<daq.topic_prefix>/{disturbance,object_status_disturbed,object_disturbance_status}. "
      "Autoware 로 가는 GT 객체에만 위치 편차·잡음, 드롭아웃, 완전 끊김을 건다 (GT DAQ 는 그대로)",
      true));
  disturbance_engine_ =
    disturbance_enable_ && engine_config_.publish_ground_truth && !real_car_vehicle_;
  // 26.10.13 _ YSH _ localize: kinematic_state twist. 차량 PC 고스트는 위치만 옮겨 MORAI 속도가 늘 0 이다 — 그대로 내면
  //   종방향 PID 가 계속 가속하고(10-13 리프트 bag 바퀴 50 km/h) MPC 가 v = 0 모델로 조향해 진동한다
  const std::string twist_source = declare_parameter<std::string>(
    "localize.twist_source", "vehicle",
    describe(
      "localize 전용. vehicle = 실차 바퀴 속도(/vehicle/status/velocity_status)·위치 헤딩 변화율 | morai = MORAI Ego "
      "보고값 그대로 (차량 PC 가 고스트에 속도를 넣어 줄 때)",
      true));
  if (twist_source != "vehicle" && twist_source != "morai") {
    throw std::invalid_argument("localize.twist_source=" + twist_source + " — vehicle | morai");
  }
  localize_twist_from_vehicle_ = twist_source == "vehicle";
  localize_twist_config_.velocity_max_age_s = declare_parameter<double>(
    "localize.velocity_max_age_s", 0.2,
    describe("실차 속도가 이보다 낡으면 위치 변화로 잰 속도로 대신한다 (진단 WARN) [s]", true));
  localize_twist_config_.window_s = declare_parameter<double>(
    "localize.window_s", 0.2, describe("위치·헤딩 변화율을 재는 창 [s]", true));
  // VILS: follow.rate_hz · max_pose_age_s · lead_s · z_source · fail_timeout_s 는 벤치·시험장에서 맞춘다
  //       (morai_interface.follow.param.yaml 의 "## VILS" 값이 이 기본값을 덮는다)
  follow_config_.rate_hz = declare_parameter<double>("follow.rate_hz", 20.0, describe("SetTransform 주기 [Hz]", true));
  follow_config_.max_pose_age_s =
    declare_parameter<double>("follow.max_pose_age_s", 0.3, describe("이보다 낡은 실차 위치는 보내지 않는다 [s]", true));
  follow_config_.lead_s =
    declare_parameter<double>("follow.lead_s", 0.0, describe("위치를 이만큼 앞으로 외삽해 보낸다 (RPC 지연 보상) [s]", true));
  const std::string z_src = declare_parameter<std::string>(
    "follow.z_source", "morai_current", describe("morai_current | map | fixed", true));
  if (const auto z = parse_follow_z_source(z_src)) {
    follow_config_.z_source = *z;
  } else {
    throw std::invalid_argument("follow.z_source=" + z_src + " — morai_current | map | fixed");
  }
  follow_config_.fixed_z = declare_parameter<double>("follow.fixed_z", 0.0, describe("[m]", true));
  follow_config_.fail_timeout_s = declare_parameter<double>(
    "follow.fail_timeout_s", 0.5, describe("성공한 SetTransform 이 이만큼 없으면 진단 ERROR [s]", true));
  // 26.10.12 _ YSH _ VILS: 고스트 명령 (10-12 실 MORAI 벤치 방식 c — 물리 없이 SetTransform 을 따른다)
  follow_config_.ghost = declare_parameter<bool>(
    "follow.ghost", true,
    describe("매 틱 ControlVehicle(GHOST, 실차 속도 km/h) 뒤 SetTransform. 연결 때 제어 모드 AUTO", true));
  // 26.10.12 _ YSH _ VILS: DAQ brake(0~1) = K-CAN 브레이크 압력 / 이 값 (K-CAN factor 0.1/1 미확정 — 실측 뒤 맞춘다)
  follow_daq_brake_full_bar_ = declare_parameter<double>(
    "follow.daq_brake_full_bar", 100.0, describe("DAQ brake 1.0 에 해당하는 브레이크 압력 [bar]", true));
  follow_input_topic_ = declare_parameter<std::string>(
    "follow.input_topic", "/localization/kinematic_state", describe("실차 측위 (nav_msgs/Odometry, map)", true));
  // 26.10.13 _ YSH _ 잠금은 localize 도 같다 (실차가 차량). 원문: if (follow_mode_) { … 잠금 … rate_hz … WARN }
  if (real_car_vehicle_) {
    // 잠금: MORAI 로 명령·기어·모드를 보내지 않고, 텔레포트·자동 AUTO 를 하지 않는다 (on_set_parameters 가 다시 켜는 것을 막는다)
    runtime_params_value_.forward_control_to_sim = false;
    runtime_params_value_.teleport_on_initialpose = false;
    engine_config_.engage_on_start = false;
    engine_config_.auto_reassert_auto = false;
    // 파라미터 값도 맞춘다 (ros2 param get·mission_server 가 읽는다). on-set 콜백은 아직 없다. engage_on_start·
    //   auto_reassert_auto 는 read-only 라 값은 남고 안에서만 끈다 (위 로그)
    set_parameter(rclcpp::Parameter("forward_control_to_sim", false));
    set_parameter(rclcpp::Parameter("teleport_on_initialpose", false));
  }
  if (localize_mode_) {
    RCLCPP_WARN(
      get_logger(),
      "ego_mode=localize (VILS 차량 PC 고스트): MORAI Ego → /localization/kinematic_state·TF·측위 초기화. 명령·기어·모드·"
      "텔레포트·등화·control_mode_request 끔 (MORAI 로 쓰지 않는다), Ego 상태는 /morai/ego/*");
  }
  if (follow_mode_) {
    if (!(follow_config_.rate_hz > 0.0)) {
      follow_config_.rate_hz = 20.0;
    }
    RCLCPP_WARN(
      get_logger(),
      "ego_mode=follow (의사 VILS): %s → MORAI Ego SetTransform %.0f Hz (z %s, lead %.2f s). 명령 송신·텔레포트·TF·"
      "control_mode_request·engage_on_start·auto_reassert_auto 끔, Ego 상태는 /morai/ego/* 로",
      follow_input_topic_.c_str(), follow_config_.rate_hz, to_string(follow_config_.z_source), follow_config_.lead_s);
  }

  // 26.09.25 _ YSH _ 리뷰 반영: main.py 에만 있던 파라미터를 -p 로 넘기면 조용히 무시되므로 알려 준다
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  for (const auto & legacy : kLegacyParams) {
    if (overrides.count(legacy.name) > 0) {
      RCLCPP_WARN(
        get_logger(), "main.py 파라미터 '%s' 는 이 노드에서 효과가 없다 → %s", legacy.name, legacy.note);
    }
  }
}

rcl_interfaces::msg::SetParametersResult MoraiInterfaceNode::on_set_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  const std::lock_guard<std::mutex> lock(params_mutex_);
  RuntimeParams next = runtime_params_value_;
  std::optional<DaqMode> next_daq_mode;  // 26.09.26 _ YSH _ daq.mode (RuntimeParams 밖 — 성공 뒤 샘플러에 적용)
  auto finite = [&result](const rclcpp::Parameter & p) {
    const double v = p.as_double();
    if (!std::isfinite(v)) {
      result.successful = false;
      result.reason = p.get_name() + " must be finite";
    }
    return v;
  };
  try {
    for (const auto & p : parameters) {
      const std::string & name = p.get_name();
      if (name == "forward_control_to_sim") {
        next.forward_control_to_sim = p.as_bool();
        // 26.10.10 _ YSH _ follow 모드는 명령 송신을 켤 수 없다 (SetTransform 과 싸운다)
        // 26.10.13 _ YSH _ localize 도 (차량 PC 고스트와 싸운다). 원문: if (follow_mode_ && next.forward_control_to_sim) {
        //   result.reason = "ego_mode=follow 에서는 forward_control_to_sim 을 켤 수 없다";
        if (real_car_vehicle_ && next.forward_control_to_sim) {
          result.successful = false;
          result.reason = "ego_mode=" + ego_mode_name_ + " 에서는 forward_control_to_sim 을 켤 수 없다";
        }
      } else if (name == "steer_cmd_sign") {
        next.command.steer_cmd_sign = finite(p);
      } else if (name == "steer_comp_enable") {
        next.command.steer_comp.enable = p.as_bool();
      } else if (name == "steer_comp_speeds_mps") {
        next.command.steer_comp.speeds_mps = p.as_double_array();
      } else if (name == "steer_comp_gains") {
        next.command.steer_comp.gains = p.as_double_array();
      } else if (name == "send_pedal_with_accel") {
        next.command.longitudinal.send_pedal_with_accel = p.as_bool();
      } else if (name == "pedal_accel_max") {
        next.command.longitudinal.pedal_accel_max = finite(p);
      } else if (name == "pedal_brake_max") {
        next.command.longitudinal.pedal_brake_max = finite(p);
      } else if (name == "long_cmd_mode") {
        const auto m = autoware_to_morai::parse_long_cmd_mode(p.as_string());
        if (!m) {
          result.successful = false;
          result.reason = "long_cmd_mode must be acceleration | throttle | velocity";
        } else {
          next.command.longitudinal.mode = *m;
        }
      } else if (name == "steer_report_hold") {
        next.steer_report_hold = p.as_bool();
      } else if (name == "steer_report_ignore_foreign") {
        // 26.10.01 _ YSH _ 외부 조향 필터 (주행 중 A/B)
        next.steer_report_ignore_foreign = p.as_bool();
      } else if (name == "steer_input_norm_deg") {
        const double v = finite(p);
        if (result.successful && v <= 0.0) {
          result.successful = false;
          result.reason = "steer_input_norm_deg must be positive";
        } else {
          next.steer_input_norm_deg = v;
        }
      } else if (name == "map_origin_offset_x") {
        next.map_origin.x = finite(p);
      } else if (name == "map_origin_offset_y") {
        next.map_origin.y = finite(p);
      } else if (name == "map_origin_offset_z") {
        next.map_origin.z = finite(p);
      } else if (name == "map_origin_yaw_offset_deg") {
        next.map_origin.yaw_deg = finite(p);
      } else if (name == "teleport_on_initialpose") {
        next.teleport_on_initialpose = p.as_bool();
        // 26.10.13 _ YSH _ localize 도. 원문: if (follow_mode_ && next.teleport_on_initialpose) {  // 26.10.10 _ YSH _
        //   result.reason = "ego_mode=follow 에서는 teleport_on_initialpose 를 켤 수 없다";
        if (real_car_vehicle_ && next.teleport_on_initialpose) {
          result.successful = false;
          result.reason = "ego_mode=" + ego_mode_name_ + " 에서는 teleport_on_initialpose 를 켤 수 없다";
        }
      } else if (name == "command_trigger") {
        // 26.09.25 _ YSH _ 명령 트리거·주기 (엔진 루프는 업데이트마다, 명령 타이머는 틱마다 읽는다)
        const auto t = parse_command_trigger(p.as_string());
        if (!t) {
          result.successful = false;
          result.reason = "command_trigger must be update | timer";
        } else {
          next.command_trigger = *t;
        }
      } else if (name == "cmd_resend_hz") {
        const auto hz = as_number(p.get_parameter_value());
        if (!hz || !cmd_resend_hz_is_valid(*hz)) {
          result.successful = false;
          result.reason = "cmd_resend_hz must be a number within [10, 400] Hz";
        } else {
          next.cmd_resend_hz = *hz;
        }
      } else if (name == "daq.mode") {
        // 26.09.26 _ YSH _ DAQ 게이트 (수동 주행·record_run 은 always)
        const auto m = parse_daq_mode(p.as_string());
        if (!m) {
          result.successful = false;
          result.reason = "daq.mode must be gated | always";
        } else {
          next_daq_mode = *m;
        }
      }
      if (!result.successful) {
        return result;
      }
    }
  } catch (const std::exception & e) {
    result.successful = false;
    result.reason = e.what();
    return result;
  }
  if (next.forward_control_to_sim != runtime_params_value_.forward_control_to_sim) {
    RCLCPP_INFO(
      get_logger(), "forward_control_to_sim → %s",
      next.forward_control_to_sim ? "true (Autoware 가 운전)" : "false (MORAI 키보드 수동 주행)");
  }
  // 26.10.01 _ YSH _ 외부 조향 필터 전환 기록 (A/B 시각을 launch 로그에 남긴다)
  if (next.steer_report_ignore_foreign != runtime_params_value_.steer_report_ignore_foreign) {
    RCLCPP_INFO(
      get_logger(), "steer_report_ignore_foreign → %s",
      next.steer_report_ignore_foreign ? "true (외부 조향 프레임은 직전 우리 값으로 리포트)"
                                       : "false (MORAI 값 그대로 리포트)");
  }
  // 26.09.25 _ YSH _ A/B 전환 기록 (전환 시각을 로그로 남겨 bag·MORAI 기록과 맞춘다)
  if (
    next.command_trigger != runtime_params_value_.command_trigger ||
    next.cmd_resend_hz != runtime_params_value_.cmd_resend_hz) {
    if (next.command_trigger == CommandTrigger::kTimer) {
      RCLCPP_INFO(
        get_logger(), "command_trigger → timer (%g Hz 고정 주기, main.py 조건)", next.cmd_resend_hz);
    } else {
      RCLCPP_INFO(
        get_logger(), "command_trigger → update (MORAI 업데이트당 1 회, cmd_resend_hz %g 는 쓰지 않는다)",
        next.cmd_resend_hz);
    }
  }
  runtime_params_value_ = next;
  runtime_params_.set(next);
  if (next_daq_mode && daq_sampler_) {
    daq_sampler_->set_mode(*next_daq_mode);
  }
  return result;
}

// =============================================================================
// EngineSink
// =============================================================================

void MoraiInterfaceNode::publish(const UpdateOutputs & outputs)
{
  pub_velocity_->publish(outputs.velocity);
  pub_steering_->publish(outputs.steering);
  if (outputs.has_ground_truth && pub_kinematic_state_) {
    // 26.10.13 _ YSH _ localize: twist 를 실차 속도·위치 헤딩 변화율로 바꿔 낸다 (pose·TF 는 그대로). 원문:
    // pub_kinematic_state_->publish(outputs.odometry);
    if (localize_twist_) {
      auto odometry = outputs.odometry;
      localize_twist_->apply(odometry, rclcpp::Time(odometry.header.stamp).nanoseconds(), steady_now_ns());
      pub_kinematic_state_->publish(odometry);
    } else {
      pub_kinematic_state_->publish(outputs.odometry);
    }
    // 26.10.10 _ YSH _ follow 모드는 TF 를 내지 않는다 (GNSS 노드가 낸다). 원문: tf_broadcaster_->sendTransform(outputs.transform);
    if (tf_broadcaster_) {
      tf_broadcaster_->sendTransform(outputs.transform);
    }
  }
  if (outputs.gear) {
    pub_gear_->publish(*outputs.gear);
  }
  if (outputs.control_mode) {
    pub_control_mode_->publish(*outputs.control_mode);
  }
  // 26.09.26 _ YSH _ 램프 리포트도 다른 차량 리포트와 한 묶음 — MORAI 업데이트마다, 같은 stamp (CARLA·
  //   simple_planning_simulator 와 같음). 값은 전처럼 마지막 명령의 에코 (MORAI read-back 이 없다). lamp_cache_ 는 락 보호라
  //   엔진 스레드에서 읽어도 된다. 전에는 on_lamp_timer 가 10 Hz·stamp=now() 로 냈다
  {
    const auto lamps = lamp_cache_.get();
    autoware_vehicle_msgs::msg::TurnIndicatorsReport turn;
    turn.stamp = outputs.stamp;
    turn.report = lamps.turn_indicators;
    pub_turn_indicators_->publish(turn);
    autoware_vehicle_msgs::msg::HazardLightsReport hazard;
    hazard.stamp = outputs.stamp;
    hazard.report = lamps.hazard_lights;
    pub_hazard_lights_->publish(hazard);
  }
  if (outputs.objects && pub_objects_) {
    pub_objects_->publish(*outputs.objects);
  }
  // 26.11.10 _ YSH _ 외란 프레임 → DAQ. 위 TrackedObjects 와 같은 프레임·같은 stamp 다 (daq.stamp 와 무관). 다른 /daq
  //   토픽과 같은 게이트(녹화 구간)에서만 낸다 — publishing() 은 샘플러 run_mutex_ 만 잠깐 잡는다 (엔진 스레드는
  //   submit_raw_actor_states 에서 이미 업데이트마다 잡는다). 마지막 상태는 게이트와 무관하게 둔다 (서비스 응답용)
  if (outputs.disturbance) {
    auto status = to_disturbance_status_msg(*outputs.disturbance, outputs.stamp);
    if (pub_daq_disturbance_ && daq_sampler_ && daq_sampler_->publishing()) {
      if (pub_daq_objects_disturbed_) {
        // Autoware 에 실제로 준 목록 (숨김 프레임이면 빈 목록). 위치는 MORAI 기준점(transform.location) 그대로
        auto list = morai_to_autoware::to_daq_object_status_list(
          outputs.disturbance->sent_actors, daq_config_.accel_scale);
        list.header.stamp = outputs.stamp;
        pub_daq_objects_disturbed_->publish(list);
      }
      // 26.11.11 _ YSH _ 외란 전·후 짝 (숨김 프레임에도 GT 객체 전부, visible false)
      if (pub_daq_object_disturbance_) {
        pub_daq_object_disturbance_->publish(
          to_object_disturbance_status_list(*outputs.disturbance, outputs.stamp, daq_config_.accel_scale));
      }
      pub_daq_disturbance_->publish(status);
    }
    const std::lock_guard<std::mutex> lock(disturbance_status_mutex_);
    last_disturbance_status_ = std::move(status);
  }
  if (outputs.clock && pub_clock_) {
    pub_clock_->publish(*outputs.clock);
  }
}

// 26.11.10 _ YSH _ 신규 — 외란 명령 (executor). 제어기가 읽고 검사해 설정을 통째로 바꿔 끼운다. 여기서는 기다리지 않는다 —
//   적용은 다음 객체 프레임(최대 약 0.14 s 뒤)이고 applied_time 은 받아들인 시각이지 그 프레임의 stamp 가 아니다.
//   status 는 마지막 객체 프레임이라 이 명령이 반영되기 전일 수 있다 (run·base_seed 는 지금 값)
void MoraiInterfaceNode::on_set_disturbance(
  const std::shared_ptr<SetDisturbance::Request> request,
  const std::shared_ptr<SetDisturbance::Response> response)
{
  DisturbanceRequest req;
  req.trigger = request->trigger;
  // 26.11.11 _ YSH _ run·source 는 128 바이트까지 — 시드 구간 run 은 /daq/disturbance 매 프레임(10 Hz)에 실린다.
  //   trigger 는 파서가 256 자 넘으면 TOO_LONG 으로 거절한다. 원문:
  // req.run = request->run;
  // req.source = request->source;
  constexpr size_t kMaxTagBytes = 128;
  req.run = utf8_prefix(request->run, kMaxTagBytes);
  req.source = utf8_prefix(request->source, kMaxTagBytes);
  req.base_seed = request->base_seed;
  req.seq = request->seq;
  const DisturbanceReply reply = disturbance_ctl_->handle(req);

  response->ok = reply.ok;
  response->message = reply.message;
  response->command = to_disturbance_command_msg(reply.command, request->trigger);
  response->run = reply.run;
  response->base_seed = reply.base_seed;
  if (reply.ok) {
    response->applied_time = morai_to_autoware::to_time_msg(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch())
        .count());
  }
  {
    const std::lock_guard<std::mutex> lock(disturbance_status_mutex_);
    response->status = last_disturbance_status_;
  }

  // 호출마다 한 줄 (문장은 길이 제한 전 것일 수 있어 잘라 적는다)
  // 26.11.11 _ YSH _ source·run 은 자른 값(req)을 적고, 응답 문장도 400 자까지. 원문의 인자 줄:
  //   request->trigger.c_str(), reply.message.c_str(), request->source.c_str(), ul(request->seq),
  //   request->run.c_str(), ul(request->base_seed), reply.run.c_str(), ul(reply.base_seed));
  if (reply.ok) {
    RCLCPP_INFO(
      get_logger(), "외란 명령 '%.300s' → %.400s (source '%s' seq %lu, 요청 run '%s' seed %lu → 구간 run '%s' seed %lu)",
      request->trigger.c_str(), reply.message.c_str(), req.source.c_str(), ul(request->seq),
      req.run.c_str(), ul(request->base_seed), reply.run.c_str(), ul(reply.base_seed));
  } else {
    RCLCPP_WARN(
      get_logger(),
      "외란 명령 거절 '%.300s' → %.400s (source '%s' seq %lu, 요청 run '%s' seed %lu, 구간 run '%s' seed %lu) — 바뀐 것 없음",
      request->trigger.c_str(), reply.message.c_str(), req.source.c_str(), ul(request->seq),
      req.run.c_str(), ul(request->base_seed), reply.run.c_str(), ul(reply.base_seed));
  }
}

// 26.09.26 _ YSH _ DAQ: 엔진이 받아들인 표본을 복사해 둔다 (엔진이 스냅샷 버퍼를 다시 쓰므로 복사). 켜지지 않은 동안도
//   복사한다 — 런이 켜지는 첫 틱부터 표본이 있어 시작에 결손이 생기지 않게 (Ego + NPC 몇 개라 수 µs)
void MoraiInterfaceNode::on_sample(
  const morai_grpc_vendor::Snapshot & snapshot, const builtin_interfaces::msg::Time & stamp,
  uint64_t update_index, int64_t receive_steady_ns)
{
  if (!daq_sampler_) {
    return;
  }
  auto frame = std::make_shared<DaqFrame>();
  frame->snapshot = snapshot;
  frame->stamp = stamp;
  frame->update_index = update_index;
  frame->receive_steady_ns = receive_steady_ns;
  daq_cache_.store(std::move(frame));
  // 26.09.26 _ YSH _ 원시 gRPC 기록: 받은 응답 바이트(호출마다 새 버퍼 — 포인터만 복사)를 큐에. 게이트·JSON 은 DAQ 쪽
  if (daq_config_.raw && snapshot.raw_actor_states) {
    DaqRawActorStates raw;
    raw.bytes = snapshot.raw_actor_states;
    raw.update_index = update_index;
    raw.rx_system_ns = snapshot.receive_system_ns;
    raw.morai_timestamp_us = snapshot.timestamp_us;
    daq_sampler_->submit_raw_actor_states(std::move(raw));
  }
}

// 26.09.26 _ YSH _ 원시 gRPC 기록: 보낸 ControlVehicle (엔진 또는 명령 타이머 스레드 — 복사해 큐에 넣고 바로 돌아온다)
void MoraiInterfaceNode::on_command_sent(
  const morai_grpc_vendor::VehicleCommand & cmd, bool stale, int64_t send_system_ns,
  const morai_grpc_vendor::Status & status)
{
  // 26.10.06 _ YSH _ 명세 25(/daq/ctrl_cmd)는 raw 설정과 무관하게 MORAI 가 받은(gRPC OK) 명령을 넣는다. 원문:
  // if (!daq_sampler_ || !daq_config_.raw) {
  //   return;
  // }
  if (!daq_sampler_) {
    return;
  }
  // 26.09.26 _ YSH _ 리뷰 반영: status.ok 는 MORAI Result 거부도 false 라 계약의 ok (전송 OK) 와 다르다 — 변환은
  //   make_daq_raw_command 가 (ok = grpc_code 0, Result.status 는 result_status 로 따로)
  auto rc = make_daq_raw_command(cmd, stale, send_system_ns, status, ++daq_tx_seq_);
  if (rc.ok) {
    daq_sampler_->submit_sent_command(cmd, send_system_ns);
  }
  if (daq_config_.raw) {
    daq_sampler_->submit_raw_command(std::move(rc));
  }
}

// 26.09.26 _ YSH _ DAQ 스레드에서 틱마다
void MoraiInterfaceNode::publish_daq(const DaqOutputs & outputs)
{
  // 26.10.05 _ YSH _ MORAI 메시지 (원문: outputs.state → pub_daq_state_, outputs.accel → pub_daq_accel_)
  if (outputs.ego && pub_daq_ego_) {
    pub_daq_ego_->publish(*outputs.ego);
  }
  // 26.11.12 _ YSH _ 같은 ego 표본을 world 로 돌린 계산 토픽 (곱셈 몇 번 — 주행 경로와 무관한 DAQ 스레드)
  if (outputs.ego && pub_daq_ego_vel_world_ && pub_daq_ego_acc_world_) {
    geometry_msgs::msg::Vector3Stamped vel;
    geometry_msgs::msg::Vector3Stamped acc;
    morai_to_autoware::to_daq_ego_world(*outputs.ego, vel, acc);
    pub_daq_ego_vel_world_->publish(vel);
    pub_daq_ego_acc_world_->publish(acc);
  }
  if (outputs.objects && pub_daq_objects_) {
    pub_daq_objects_->publish(*outputs.objects);
  }
  if (outputs.cmd && pub_daq_cmd_) {
    pub_daq_cmd_->publish(*outputs.cmd);
  }
  if (outputs.ctrl && pub_daq_ctrl_) {  // 26.10.06 _ YSH _
    pub_daq_ctrl_->publish(*outputs.ctrl);
  }
  if (outputs.collision && pub_daq_collision_) {  // 26.10.07 _ YSH _
    pub_daq_collision_->publish(*outputs.collision);
  }
}

// 26.09.26 _ YSH _ 런 요약 → /daq/run_summary (latched — cue_server 가 늦게 구독해도 받는다)
void MoraiInterfaceNode::on_daq_run_end(const DaqRunSummary & summary)
{
  // 26.11.10 _ YSH _ 외란 주입: 런이 끝나면 걸려 있던 외란을 전부 끈다 (cue_server 의 해제가 빠져도 다음 런·수동 주행에
  //   남지 않게). 시드 구간·카운터는 그대로 둔다. replaced 로 끝났고 다음 런이 이미 구간을 열었으면 건드리지 않는다.
  //   executor·DAQ 스레드·종료 경로 어디서든 불린다 — 제어기 mutex 만 잡는다. 아래 이른 return 보다 앞에 둔다
  if (disturbance_ctl_ && disturbance_ctl_->clear_on_run_end(summary.run, summary.end_reason)) {
    RCLCPP_INFO(
      get_logger(), "외란 주입: 런 '%s' 종료(%s) — 걸려 있던 외란을 전부 껐다 (시드 구간은 유지)",
      summary.run.c_str(), summary.end_reason.c_str());
  }
  if (!pub_daq_run_summary_) {
    return;
  }
  std_msgs::msg::String msg;
  msg.data = to_json(summary);
  pub_daq_run_summary_->publish(msg);
}

// 26.09.26 _ YSH _ 원시 gRPC 기록 → <prefix>/raw/* (DAQ 스레드, 넣은 순서대로)
void MoraiInterfaceNode::publish_daq_raw(const DaqRawOutputs & outputs)
{
  if (pub_daq_raw_actor_states_) {
    for (const auto & json : outputs.actor_states) {
      std_msgs::msg::String msg;
      msg.data = json;
      pub_daq_raw_actor_states_->publish(msg);
    }
  }
  if (pub_daq_raw_ctrl_cmd_) {
    for (const auto & json : outputs.vehicle_ctrl_cmd) {
      std_msgs::msg::String msg;
      msg.data = json;
      pub_daq_raw_ctrl_cmd_->publish(msg);
    }
  }
}

// 26.09.26 _ YSH _ cue_server 녹화 구간 게이트: "active <run>" / "idle <run>"
void MoraiInterfaceNode::on_daq_run_state(const std_msgs::msg::String::ConstSharedPtr msg)
{
  if (!daq_sampler_) {
    return;
  }
  const auto parsed = parse_daq_run_state(msg->data);
  if (!parsed) {
    RCLCPP_WARN(
      get_logger(), "%s/run_state 형식이 틀렸다: '%s' (\"active <run>\" 또는 \"idle <run>\")",
      daq_prefix_.c_str(), msg->data.c_str());
    return;
  }
  daq_sampler_->set_run_state(parsed->first, parsed->second);
}

void MoraiInterfaceNode::on_connected(bool reconnected)
{
  // MORAI 에 램프 read-back 이 없고 재기동했을 수도 있다 → 현재 캐시를 반드시 다시 보낸다
  post_lamp_sync(true);
  (void)reconnected;
  // 26.10.12 _ YSH _ VILS 고스트: GHOST 명령은 AUTO 모드에서만 먹는다 (MORAI 재기동하면 키보드 모드로 돌아온다 — 10-12 벤치)
  if (follow_mode_ && follow_config_.ghost) {
    if (!worker_->post([this]() {
          const auto st = link_->set_control_mode(morai_grpc_vendor::ControlMode::kAuto, std::chrono::milliseconds(500));
          if (!st.ok) {
            RCLCPP_WARN(get_logger(), "follow 고스트: 제어 모드 AUTO 실패 — %s", st.message.c_str());
          }
        })) {
      RCLCPP_WARN(get_logger(), "follow 고스트: 제어 모드 작업을 넣지 못했다 (작업 큐 가득)");
    }
  }
  if (!sensor_model_.empty()) {  // 이미 장착했으면 작업이 바로 돌아온다
    if (!worker_->post([this]() { mount_sensors_once(); })) {
      RCLCPP_WARN(get_logger(), "센서 장착 작업을 넣지 못했다 (작업 큐 가득)");
    }
  }
}

// =============================================================================
// executor 콜백
// =============================================================================

void MoraiInterfaceNode::on_control_cmd(const Control::ConstSharedPtr msg)
{
  // 26.09.26 _ YSH _ DAQ 명세 25: Autoware 가 낸 명령 원본 그대로 (forward 여부·NaN 과 무관)
  if (daq_sampler_) {
    daq_cmd_latch_.submit(*msg, steady_now_ns());
  }
  const auto result = submit_control(
    command_slot_, *msg, runtime_params_.get()->forward_control_to_sim, steady_now_ns());
  if (result == SubmitResult::kForwardingOff) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000,
      "forward_control_to_sim=false: 제어 명령 전달 차단 중 (MORAI 키보드로 수동 주행)");
    return;
  }
  if (result == SubmitResult::kRejectedNonFinite) {
    const uint64_t n = ++nan_commands_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "control_cmd 에 NaN/inf (steer %f, accel %f, velocity %f) — 무시하고 마지막 유효 명령 유지 (누적 %lu)",
      static_cast<double>(msg->lateral.steering_tire_angle),
      static_cast<double>(msg->longitudinal.acceleration),
      static_cast<double>(msg->longitudinal.velocity), ul(n));
  }
}

void MoraiInterfaceNode::on_gear_cmd(const GearCommand::ConstSharedPtr msg)
{
  if (msg->command == GearCommand::NONE) {
    return;
  }
  // 수동 주행 중 Autoware 가 기어를 P/N 으로 바꿔버리면 몰 수 없으므로 같이 막는다 (main.py 와 같음)
  if (!runtime_params_.get()->forward_control_to_sim) {
    gear_sender_->set_forwarding(false);
    return;
  }
  gear_sender_->submit(static_cast<int>(autoware_to_morai::to_gear_mode(msg->command)));
}

void MoraiInterfaceNode::on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg)
{
  // 26.10.13 _ YSH _ localize 도 (등화 명령은 실차 인터페이스가 받는다). 원문: if (follow_mode_) {
  if (real_car_vehicle_) {
    return;  // 26.10.12 _ YSH _ VILS: MORAI Ego 등화는 실차 등화 상태를 따른다 (명령이 아니라)
  }
  lamp_cache_.set_turn_indicators(msg->command);
  post_lamp_sync(false);
}

void MoraiInterfaceNode::on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg)
{
  // 26.10.13 _ YSH _ localize 도. 원문: if (follow_mode_) {
  if (real_car_vehicle_) {
    return;  // 26.10.12 _ YSH _ VILS: 실차 등화 상태를 따른다
  }
  lamp_cache_.set_hazard_lights(msg->command);
  post_lamp_sync(false);
}

void MoraiInterfaceNode::on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg)
{
  const auto params = runtime_params_.get();
  if (!params->teleport_on_initialpose) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 10000, "teleport_on_initialpose=false: /initialpose 무시");
    return;
  }
  if (msg->header.frame_id != "map" && !msg->header.frame_id.empty()) {
    RCLCPP_WARN(
      get_logger(), "/initialpose 의 frame_id 가 '%s' 라 무시 (map 이어야 함)",
      msg->header.frame_id.c_str());
    return;
  }
  std::optional<double> current_z;
  if (const auto state = state_cache_.latest()) {
    current_z = state->ego.position.z;
  }
  TeleportRequest req;
  req.target = autoware_to_morai::from_map_pose(
    msg->pose.pose, params->map_origin, current_z, &req.map_yaw_rad);
  req.map_x = msg->pose.pose.position.x;
  req.map_y = msg->pose.pose.position.y;
  {
    const std::lock_guard<std::mutex> lock(teleport_mutex_);
    teleport_pending_ = req;  // 가장 최근 요청만 남긴다
  }
  if (!worker_->post([this]() { run_teleport(); })) {
    RCLCPP_WARN(get_logger(), "텔레포트 작업을 넣지 못했다 (작업 큐 가득)");
  }
}

void MoraiInterfaceNode::on_control_mode_request(
  const std::shared_ptr<rclcpp::Service<ControlModeCommand>> service,
  const std::shared_ptr<rmw_request_id_t> header,
  const std::shared_ptr<ControlModeCommand::Request> request)
{
  auto reply = [service, header](bool success) {
    ControlModeCommand::Response response;
    response.success = success;
    rmw_request_id_t id = *header;
    service->send_response(id, response);
  };

  // 기어 명령과 같은 이유로 게이트한다 — 사람이 키보드로 모는 중에 AUTO 로 바꾸면 아무도 조종하지 않는다
  if (!runtime_params_.get()->forward_control_to_sim) {
    RCLCPP_WARN(
      get_logger(), "forward_control_to_sim=false: control mode 요청 거절 (수동 주행 중)");
    reply(false);
    return;
  }
  const auto mode = autoware_to_morai::to_control_mode(request->mode);
  if (mode == mgv::ControlMode::kUnspecified) {
    RCLCPP_WARN(
      get_logger(), "지원하지 않는 control mode 요청 %u — 거절", static_cast<unsigned>(request->mode));
    reply(false);
    return;
  }
  const uint8_t requested = request->mode;
  // 26.09.25 _ YSH _ 리뷰 반영: 전용 작업 스레드 (텔레포트 뒤에 ~2 s 줄 서지 않게), 요청 모드는 RPC **전에** 기록
  //   (되읽기 500 ms 사이에 재연결이 끼어도 엔진이 disengage 를 AUTO 로 되돌리지 않게). AUTO 가 실패하면 되돌린다
  const bool posted = mode_worker_->post([this, reply, mode, requested]() {
    const auto previous = engine_->requested_mode();
    engine_->note_requested_mode(mode);
    const auto st = link_->set_control_mode(mode, engine_config_.control_mode_verify);
    if (!st.ok && mode == mgv::ControlMode::kAuto) {
      engine_->restore_requested_mode(previous);
    }
    if (st.ok) {
      RCLCPP_INFO(
        get_logger(), "MORAI 제어 모드 → %s (Autoware 요청 %u, 되읽기 확인)",
        mgv::to_string(mode).c_str(), static_cast<unsigned>(requested));
    } else {
      RCLCPP_ERROR(
        get_logger(), "SetVehicleControlMode(%s) 실패: %s", mgv::to_string(mode).c_str(),
        st.message.c_str());
    }
    reply(st.ok);
  });
  if (!posted) {
    RCLCPP_ERROR(get_logger(), "control mode 요청을 작업 큐에 넣지 못했다 — 거절");
    reply(false);
  }
}

void MoraiInterfaceNode::on_lamp_timer()
{
  // 26.09.26 _ YSH _ 램프 리포트 발행은 publish() 로 옮겼다 (MORAI 업데이트마다 같은 stamp). 이 타이머는 forwarding 갱신만. 원문:
  // // 램프 리포트는 MORAI read-back 이 없어 마지막 명령의 로컬 에코다 (main.py 와 같음, 10 Hz)
  // const auto lamps = lamp_cache_.get();
  // const rclcpp::Time stamp = now();
  // autoware_vehicle_msgs::msg::TurnIndicatorsReport turn;
  // turn.stamp = stamp;
  // turn.report = lamps.turn_indicators;
  // pub_turn_indicators_->publish(turn);
  // autoware_vehicle_msgs::msg::HazardLightsReport hazard;
  // hazard.stamp = stamp;
  // hazard.report = lamps.hazard_lights;
  // pub_hazard_lights_->publish(hazard);

  // main.py _gear_refresh_forwarding / _cmd_resend_refresh_forwarding (10 Hz)
  const bool forwarding = runtime_params_.get()->forward_control_to_sim;
  gear_sender_->set_forwarding(forwarding);
  if (!forwarding) {
    command_slot_.clear();
  } else if (!was_forwarding_) {
    // 차단 중 사람이 키보드로 램프를 바꿨을 수 있다 → 다음 램프는 같아도 보낸다
    post_lamp_sync(true);
  }
  was_forwarding_ = forwarding;
}

void MoraiInterfaceNode::on_localization_init_timer()
{
  if (localization_initialized_) {
    return;
  }
  const int64_t now_ns = steady_now_ns();
  if (localization_init_requested_) {
    if (now_ns - localization_request_ns_ > kLocalizationResponseTimeoutNs) {
      init_localization_client_->remove_pending_request(localization_request_id_);
      localization_init_requested_ = false;
      RCLCPP_WARN(get_logger(), "측위 초기화 응답 없음 (10 s) — 재시도");
    }
    return;
  }
  if (!init_localization_client_->service_is_ready()) {
    return;
  }
  const auto state = state_cache_.latest();
  const auto age = engine_->sample_age_s();
  if (!state || !age || *age > kLocalizationMaxSampleAgeS) {
    return;
  }
  const auto params = runtime_params_.get();

  PoseWithCovarianceStamped pose;
  pose.header.stamp = now();
  pose.header.frame_id = "map";
  pose.pose.pose =
    morai_to_autoware::to_map_pose(state->ego.position, state->ego.rotation_deg, params->map_origin);

  auto request = std::make_shared<InitializeLocalization::Request>();
  request->method = InitializeLocalization::Request::DIRECT;
  request->pose_with_covariance.push_back(pose);

  localization_init_requested_ = true;
  localization_request_ns_ = now_ns;
  const auto sent = init_localization_client_->async_send_request(
    request, [this](rclcpp::Client<InitializeLocalization>::SharedFuture future) {
      try {
        const auto response = future.get();
        if (response->status.success) {
          localization_initialized_ = true;
          localization_init_timer_->cancel();
          RCLCPP_INFO(get_logger(), "측위 초기화 완료 (MORAI ego pose 로 DIRECT 초기화)");
        } else {
          RCLCPP_WARN(
            get_logger(), "측위 초기화 거부됨, 재시도: %s", response->status.message.c_str());
          localization_init_requested_ = false;
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "측위 초기화 호출 실패, 재시도: %s", e.what());
        localization_init_requested_ = false;
      }
    });
  localization_request_id_ = sent.request_id;
}

void MoraiInterfaceNode::on_guard_timer()
{
  // 26.09.25 _ YSH _ 자체 가드: main.py(morai_bridge)와 동시에 돌면 두 writer 가 번갈아 명령한다
  // 26.09.25 _ YSH _ 리뷰 반영: 자기 publisher 는 하나만 뺀다 (GID, 안 맞으면 같은 이름의 첫 endpoint — cyclonedds
  //   Humble 은 get_gid 가 graph GID 와 다르다). 이름으로 전부 빼면 같은 이름(node_name:=morai_bridge 와 main.py,
  //   또는 이 노드 두 개)의 다른 인스턴스가 안 보였다. 막히는 순간 엔진이 ~0.3 s 감속을 보내고 멈춘다
  //   (둘 다 막히면 MORAI 에는 감속이 남는다). duplicate_writer_guard=false 면 진단만
  // 26.10.10 _ YSH _ follow 모드면 자기 토픽이 /morai/ego/* 다. 원문: get_publishers_info_by_topic(kTopicVelocity) / (kTopicKinematicState)
  const std::string velocity_others = join_names(other_writers(
    get_publishers_info_by_topic(ego_topic(kTopicVelocity)), pub_velocity_->get_gid(), get_name(),
    get_namespace()));
  std::string kinematic_others;
  if (engine_config_.publish_ground_truth && pub_kinematic_state_) {
    kinematic_others = join_names(other_writers(
      get_publishers_info_by_topic(ego_topic(kTopicKinematicState)), pub_kinematic_state_->get_gid(), get_name(),
      get_namespace()));
  }
  if (follow_mode_) {
    // follow: 실차 측위를 내는 노드가 정확히 하나여야 한다 (없으면 MORAI Ego 가 멈추고, 둘이면 섞인다)
    const auto infos = get_publishers_info_by_topic(follow_input_topic_);
    std::vector<std::string> names;
    for (const auto & i : infos) {
      names.push_back(i.node_name());
    }
    const std::lock_guard<std::mutex> lock(guard_mutex_);
    localization_writer_names_ = join_names(names);
    localization_writer_count_ = infos.size();
  }
  const bool duplicate = !velocity_others.empty();
  if (duplicate != duplicate_writer_.load()) {
    if (duplicate) {
      RCLCPP_ERROR(
        get_logger(), "다른 publisher(%s)도 %s 를 발행한다 — main.py·이 노드와 동시 실행? %s",
        velocity_others.c_str(), kTopicVelocity,
        duplicate_guard_enabled_ ? "ControlVehicle 송신을 막는다"
                                 : "duplicate_writer_guard=false 라 막지 않는다");
    } else {
      RCLCPP_INFO(get_logger(), "중복 writer 가 사라졌다 — ControlVehicle 송신 재개");
    }
  }
  duplicate_writer_ = duplicate;
  // 26.10.10 _ YSH _ follow 모드는 늘 막는다. 원문: engine_->set_command_block(duplicate_guard_enabled_ && duplicate);
  // 26.10.13 _ YSH _ localize 도. 원문: engine_->set_command_block(follow_mode_ || (duplicate_guard_enabled_ && duplicate));
  engine_->set_command_block(real_car_vehicle_ || (duplicate_guard_enabled_ && duplicate));
  if (!guard_checked_) {
    guard_checked_ = true;
    // 26.10.13 _ YSH _ follow·localize 는 명령이 늘 잠겨 있어 "허용" 이 오해를 부른다. 원문: if (duplicate_guard_enabled_ && !duplicate) {
    if (real_car_vehicle_) {
      RCLCPP_INFO(
        get_logger(), "ego_mode=%s — ControlVehicle 은 늘 잠겨 있다 (명령은 실차 인터페이스로)", ego_mode_name_.c_str());
    } else if (duplicate_guard_enabled_ && !duplicate) {
      RCLCPP_INFO(get_logger(), "중복 writer 없음 — ControlVehicle 송신 허용");
    }
  }
  // 26.09.26 _ YSH _ DAQ: 다른 노드(daq_publisher.py 등)가 이미 /daq 를 내면 양보한다 (두 writer 가 섞이지 않게)
  // 26.10.05 _ YSH _ 새 ego_status 를 내는 다른 브리지와, 옛 vehicle_state 를 내는 daq_publisher.py 둘 다 본다
  //   (옛 writer 가 있으면 bag 에 두 형식이 섞이지 않게 지금처럼 양보한다 — 옛 bag 은 옛 형식으로 내보낸다)
  if (daq_sampler_ && pub_daq_ego_) {
    std::vector<std::string> others = other_writers(
      get_publishers_info_by_topic(daq_prefix_ + "/ego_status"), pub_daq_ego_->get_gid(), get_name(),
      get_namespace());
    for (auto & name : other_writers(
           get_publishers_info_by_topic(daq_prefix_ + "/vehicle_state"), pub_daq_ego_->get_gid(),
           get_name(), get_namespace())) {
      if (std::find(others.begin(), others.end(), name) == others.end()) {
        others.push_back(std::move(name));
      }
    }
    daq_writer_names_ = join_names(others);
    daq_sampler_->set_yield(!daq_writer_names_.empty());
  }
  // 원문: if (daq_sampler_ && pub_daq_state_) {
  // 원문:   daq_writer_names_ = join_names(other_writers(
  // 원문:     get_publishers_info_by_topic(daq_prefix_ + "/vehicle_state"), pub_daq_state_->get_gid(), get_name(),
  // 원문:     get_namespace()));
  // 원문:   daq_sampler_->set_yield(!daq_writer_names_.empty());
  // 원문: }

  const std::lock_guard<std::mutex> lock(guard_mutex_);
  if (kinematic_others != kinematic_writer_names_ && !kinematic_others.empty()) {
    RCLCPP_ERROR(
      get_logger(), "다른 노드(%s)도 %s 를 발행한다 — localization 스택이 켜져 있다? (use_ground_truth)",
      kinematic_others.c_str(), kTopicKinematicState);
  }
  duplicate_writer_names_ = velocity_others;
  kinematic_writer_names_ = kinematic_others;
}

// =============================================================================
// 작업 스레드
// =============================================================================

void MoraiInterfaceNode::post_lamp_sync(bool force)
{
  // 26.10.13 _ YSH _ localize: MORAI Ego 는 차량 PC 것이다 — SetVehicleTailLight 도 보내지 않는다 (연결 때 강제 재전송 포함)
  // 26.11.13 _ YSH _ 13_ 시각화 전용: lamp_sync=false 면 보내지 않는다. 원문: if (localize_mode_) {
  if (localize_mode_ || !lamp_sync_enable_) {
    return;
  }
  if (force) {
    lamp_force_ = true;
  }
  if (lamp_sync_queued_.exchange(true)) {
    return;  // 이미 하나 대기 중 — 그 작업이 최신 캐시를 본다
  }
  if (!worker_->post([this]() { run_lamp_sync(); })) {
    lamp_sync_queued_ = false;
  }
}

void MoraiInterfaceNode::run_lamp_sync()
{
  lamp_sync_queued_ = false;
  if (engine_->phase() != Engine::Phase::kRunning) {
    return;  // 연결되면 on_connected 가 강제 재전송을 건다
  }
  const bool force = lamp_force_.exchange(false);
  const auto signal = autoware_to_morai::to_turn_signal(lamp_cache_.get());
  const int value = static_cast<int>(signal);
  if (!force && last_sent_signal_ && *last_sent_signal_ == value) {
    return;  // 값이 바뀔 때만 보낸다
  }
  const auto st = link_->set_tail_light(signal);
  if (!st.ok) {
    last_sent_signal_.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "SetVehicleTailLight(%s) 실패: %s",
      mgv::to_string(signal).c_str(), st.message.c_str());
    return;
  }
  last_sent_signal_ = value;
}

void MoraiInterfaceNode::run_teleport()
{
  std::optional<TeleportRequest> req;
  {
    const std::lock_guard<std::mutex> lock(teleport_mutex_);
    req.swap(teleport_pending_);
  }
  if (!req) {
    return;
  }
  const auto & target = req->target;
  const int64_t t0 = steady_now_ns();
  const auto st = link_->set_transform(target);
  // 보내지 못한 경우만 여기서 끝낸다. Result 실패·deadline 은 적용됐을 수 있어 스냅샷으로 확인한다
  if (!st.ok && st.grpc_code != 0 && !st.deadline_exceeded()) {
    ++teleports_failed_;
    RCLCPP_ERROR(
      get_logger(), "SetTransform 실패 (deadline %ld ms): %s",
      static_cast<long>(link_options_.deadlines.transform.count()),  // NOLINT(runtime/int)
      st.message.c_str());
    return;
  }
  // 다음 스냅샷(엔진 StateCache) 위치로 확인한다 — MORAI Result 는 거짓일 수 있다
  bool matched = false;
  double err = -1.0;
  const int64_t end = steady_now_ns() + kTeleportVerifyNs;
  while (steady_now_ns() < end) {
    const auto state = state_cache_.latest();
    if (state && state->receive_steady_ns > t0) {
      err = std::hypot(state->ego.position.x - target.position.x, state->ego.position.y - target.position.y);
      if (err <= kTeleportToleranceM) {
        matched = true;
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
  if (matched) {
    ++teleports_ok_;
    RCLCPP_INFO(
      get_logger(), "ego 이동: map(%.2f, %.2f, %.1f도) → MORAI(%.2f, %.2f, %.1f도) — 스냅샷 위치 확인 (오차 %.2f m)",
      req->map_x, req->map_y, req->map_yaw_rad * kRadToDeg, target.position.x, target.position.y,
      target.rotation_deg.z, err);
  } else {
    ++teleports_failed_;
    RCLCPP_ERROR(
      get_logger(),
      "SetTransform 뒤 1 s 안에 MORAI 위치가 목표 %.1f m 안으로 오지 않았다 (마지막 오차 %.2f m, 호출: %s)",
      kTeleportToleranceM, err, st.ok ? "성공" : st.message.c_str());
  }
}

void MoraiInterfaceNode::mount_sensors_once()
{
  if (sensors_mounted_) {
    return;
  }
  // MORAI 의 AddSensor 는 멱등이 아니다. 이미 붙어 있으면 중복 장착이 되므로 건너뛴다
  mgv::ActorClientConfig config;
  config.address = link_options_.address;
  config.port = link_options_.port;
  config.ego_id = link_options_.ego_id;
  config.deadline = std::chrono::milliseconds(1000);
  mgv::ActorClient client(config);
  if (!client.wait_for_connected(std::chrono::milliseconds(1000))) {
    RCLCPP_WARN(get_logger(), "센서 장착: MORAI 연결 실패 — 다음 연결 때 다시 시도");
    return;
  }
  sensors_mounted_ = true;
  mgv::SensorClient sensor_client(client);
  int existing = 0;
  std::string error;
  if (!sensor_client.get_sensor_count(&existing, &error)) {
    RCLCPP_WARN(get_logger(), "Skipping sensor mount: %s", error.c_str());
    return;
  }
  if (existing > 0) {
    RCLCPP_INFO(
      get_logger(), "MORAI ego already carries %d sensor(s); not mounting '%s'", existing,
      sensor_model_.c_str());
    return;
  }
  std::vector<SensorKitEntry> entries;
  try {
    entries = load_sensor_kit(sensor_model_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(), "Could not read sensor kit '%s': %s", sensor_model_.c_str(), e.what());
    return;
  }
  const size_t mounted = mount_sensor_kit(*this, sensor_client, entries);
  RCLCPP_INFO(
    get_logger(), "Mounted %zu of %zu sensors from '%s'", mounted, entries.size(),
    sensor_model_.c_str());
}

// =============================================================================
// 진단
// =============================================================================

void MoraiInterfaceNode::refresh_diag_window()
{
  const std::lock_guard<std::mutex> lock(diag_mutex_);
  const int64_t now_ns = steady_now_ns();
  const EngineStats cur = engine_->stats();
  const uint64_t nan = nan_commands_.load();
  if (diag_prev_.t_ns == 0) {
    diag_dt_s_ = 0.0;
    diag_delta_ = EngineStats{};
    diag_nan_delta_ = 0;
  } else {
    diag_dt_s_ = static_cast<double>(now_ns - diag_prev_.t_ns) / 1e9;
    diag_delta_ = diff(cur, diag_prev_.stats);
    diag_nan_delta_ = nan - diag_prev_.nan_commands;
  }
  diag_prev_ = DiagWindow{now_ns, cur, nan};
}

void MoraiInterfaceNode::diagnose_link(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  refresh_diag_window();
  const bool running = engine_->phase() == Engine::Phase::kRunning;
  const bool fallback = engine_->fallback_active();
  const EngineStats total = engine_->stats();
  const auto c = link_->counters();
  stat.add("target", link_->target());
  stat.add("phase", running ? "running" : "connecting");
  stat.add("trigger", fallback ? "fallback_poll" : "wait_for_tick");
  stat.add("fallback_hz", engine_config_.fallback_hz);
  stat.add("reconnects", ul(total.reconnects));
  stat.add("link_losses", ul(total.link_losses));
  stat.add("link_recreates", ul(total.link_recreates));
  stat.add("snapshot_failures", ul(total.snapshot_failures));
  stat.add("wait_for_tick_failures", ul(total.wft_failures));
  stat.add("state_calls", ul(c.state_calls));
  stat.add("state_failures", ul(c.state_failures));
  stat.add("command_calls", ul(c.command_calls));
  stat.add("command_failures", ul(c.command_failures));
  stat.add("deadline_exceeded", ul(c.deadline_exceeded));
  // 26.09.25 _ YSH _ 리뷰 반영: 왕복·명령 deadline (main.py cmd_timeout_for_rtt), 트리거 헛돌기
  const auto rtt = engine_->snapshot_rtt_ms();
  stat.add("snapshot_rtt_p50_ms", rtt.p50);
  stat.add("command_deadline_ms", engine_->command_deadline_ms());
  stat.add("wait_for_tick_no_update_total", ul(total.wft_no_update));
  if (!running) {
    stat.summary(DiagnosticStatus::ERROR, "MORAI 에 연결되지 않음 (재연결·스펙 대기 중)");
  } else if (fallback) {
    stat.summaryf(
      DiagnosticStatus::WARN, "WaitForTick 폴백 — %.0f Hz 폴링+송신", engine_config_.fallback_hz);
  } else if (rtt.count > 0 && rtt.p50 > Engine::kRemoteRttMs) {
    stat.summaryf(
      DiagnosticStatus::WARN, "MORAI 왕복 p50 %.1f ms (> %.0f ms, 원격·느린 망) — 명령 deadline %ld ms",
      rtt.p50, Engine::kRemoteRttMs, static_cast<long>(engine_->command_deadline_ms()));  // NOLINT
  } else {
    stat.summary(DiagnosticStatus::OK, "MORAI 링크 정상 (WaitForTick 트리거)");
  }
}

void MoraiInterfaceNode::diagnose_update(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  EngineStats d;
  double dt = 0.0;
  {
    const std::lock_guard<std::mutex> lock(diag_mutex_);
    d = diag_delta_;
    dt = diag_dt_s_;
  }
  const EngineStats total = engine_->stats();
  const auto age = engine_->sample_age_s();
  const auto rtt = engine_->snapshot_rtt_ms();
  const auto interval = engine_->update_interval_ms();
  stat.add("update_rate_hz", dt > 0.0 ? static_cast<double>(d.updates) / dt : 0.0);
  stat.add("sample_age_ms", age ? *age * 1000.0 : -1.0);
  stat.add("duplicates_per_s", dt > 0.0 ? static_cast<double>(d.duplicates) / dt : 0.0);
  stat.add("gaps_over_45ms_window", ul(d.gaps_over_45ms));
  stat.add("updates_total", ul(total.updates));
  stat.add("duplicates_total", ul(total.duplicates));
  stat.add("backward_total", ul(total.backward));
  stat.add("rebased_total", ul(total.rebased));
  stat.add("no_ego_total", ul(total.no_ego));
  stat.add("invalid_ego_total", ul(total.invalid_ego));
  stat.add("snapshot_rtt_p50_ms", rtt.p50);
  stat.add("snapshot_rtt_p99_ms", rtt.p99);
  stat.add("update_interval_p50_ms", interval.p50);
  stat.add("update_interval_p99_ms", interval.p99);
  stat.add("update_interval_max_ms", interval.max);
  // 26.09.25 _ YSH _ 리뷰 반영: timestamp 를 못 쓰는 빌드면 내용 판정 (stamp = 수신 시각)
  const bool content_mode = engine_->gate_mode() == GateMode::kContent;
  stat.add("sample_gate", to_string(engine_->gate_mode()));
  stat.add("content_mode_entries", ul(total.content_mode_entries));
  stat.add("spec_refresh_pending", engine_->spec_refresh_pending());
  stat.add("spec_refreshes", ul(total.spec_refreshes));
  if (!age) {
    stat.summary(DiagnosticStatus::ERROR, "MORAI 표본을 아직 받지 못했다");
  } else if (*age > engine_config_.max_sample_age_s && d.duplicates > 0) {
    stat.summaryf(
      DiagnosticStatus::ERROR,
      "표본 나이 %.0f ms — 스냅샷은 오는데 timestamp·내용이 그대로다 (MORAI 일시정지·동기 모드 대기, 또는 "
      "timestamp 를 채우지 않는 빌드: morai_probe all 로 확인) — 상태·GT 발행·송신 중단",
      *age * 1000.0);
  } else if (*age > engine_config_.max_sample_age_s) {
    stat.summaryf(
      DiagnosticStatus::ERROR, "표본 나이 %.0f ms > %.0f ms — 상태·GT 발행 중단", *age * 1000.0,
      engine_config_.max_sample_age_s * 1000.0);
  } else if (content_mode) {
    stat.summary(
      DiagnosticStatus::WARN,
      "MORAI timestamp 를 쓸 수 없다 — Ego 내용·트리거로 업데이트를 가르고 stamp 는 수신 시각 (morai_probe all 로 확인)");
  } else {
    stat.summaryf(
      DiagnosticStatus::OK, "%.1f Hz, 표본 나이 %.0f ms",
      dt > 0.0 ? static_cast<double>(d.updates) / dt : 0.0, *age * 1000.0);
  }
}

void MoraiInterfaceNode::diagnose_command(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  EngineStats d;
  double dt = 0.0;
  uint64_t nan_delta = 0;
  {
    const std::lock_guard<std::mutex> lock(diag_mutex_);
    d = diag_delta_;
    dt = diag_dt_s_;
    nan_delta = diag_nan_delta_;
  }
  const EngineStats total = engine_->stats();
  const auto params = runtime_params_.get();
  const auto rtt = engine_->send_rtt_ms();
  const auto cmd_age = engine_->command_age_s();
  const auto gear = gear_sender_->counters();
  const uint64_t fails = d.send_timeouts + d.send_errors + d.send_result_failures;
  const auto per_s = [dt](uint64_t n) { return dt > 0.0 ? static_cast<double>(n) / dt : 0.0; };
  std::string dup_names;
  {
    const std::lock_guard<std::mutex> lock(guard_mutex_);
    dup_names = duplicate_writer_names_;
  }
  const auto mode = engine_->control_mode();
  const auto requested = engine_->requested_mode();
  stat.add("forward_control_to_sim", params->forward_control_to_sim);
  stat.add("duplicate_writer", dup_names.empty() ? std::string("none") : dup_names);
  stat.add("sends_per_s", per_s(d.sends));
  stat.add("send_ok_per_s", per_s(d.send_ok));
  stat.add("send_fail_per_s", per_s(fails));
  stat.add("stale_sends_window", ul(d.stale_sends));
  stat.add("stale_sends_total", ul(total.stale_sends));
  stat.add("nan_commands_window", ul(nan_delta));
  stat.add("nan_commands_total", ul(nan_commands_.load()));
  stat.add("send_timeouts_total", ul(total.send_timeouts));
  stat.add("send_result_failures_total", ul(total.send_result_failures));
  stat.add("send_errors_total", ul(total.send_errors));
  stat.add("send_rtt_p50_ms", rtt.p50);
  stat.add("send_rtt_p99_ms", rtt.p99);
  stat.add("command_age_ms", cmd_age ? *cmd_age * 1000.0 : -1.0);
  stat.add("gear_sends", ul(gear.sends));
  stat.add("gear_ok", ul(gear.ok));
  stat.add("gear_resyncs", ul(gear.resyncs));
  stat.add("morai_control_mode", mode ? mgv::to_string(*mode) : std::string("?"));
  stat.add("requested_control_mode", requested ? mgv::to_string(*requested) : std::string("none"));
  // 26.09.25 _ YSH _ 리뷰 반영: 모드 조회 나이 (낡으면 ControlModeReport 생략), 가드 설정·차단 순간 감속
  const auto mode_age = engine_->control_mode_age_s();
  stat.add("control_mode_age_ms", mode_age ? *mode_age * 1000.0 : -1.0);
  stat.add("duplicate_writer_guard", duplicate_guard_enabled_);
  stat.add("block_brake_sends_total", ul(total.block_brake_sends));
  stat.add("blocked_sends_total", ul(total.blocked_sends));
  stat.add("command_deadline_ms", engine_->command_deadline_ms());
  // 26.09.25 _ YSH _ A/B: 명령 트리거와 타이머 통계 (송신/s 는 위 sends_per_s). 업데이트당 송신 수·송신 간격(최근 500 개)·
  //   지연재설정(main.py late) — update 면 sends_per_update ≈ 1, timer 100 Hz + MORAI 50 Hz 면 ≈ 2
  const bool timer = params->command_trigger == CommandTrigger::kTimer;
  const auto gap = engine_->send_gap_ms();
  stat.add("command_trigger", to_string(params->command_trigger));
  stat.add("cmd_resend_hz", params->cmd_resend_hz);
  stat.add(
    "sends_per_update",
    d.updates > 0 ? static_cast<double>(d.sends) / static_cast<double>(d.updates) : 0.0);
  stat.add("send_gap_p50_ms", gap.p50);
  stat.add("send_gap_p99_ms", gap.p99);
  stat.add("send_gap_max_ms", gap.max);
  stat.add("cmd_timer_late_window", ul(d.cmd_timer_late));
  stat.add("cmd_timer_late_total", ul(total.cmd_timer_late));
  if (!dup_names.empty()) {
    stat.summaryf(
      DiagnosticStatus::ERROR, "다른 노드(%s)가 /vehicle/status 발행 중 — %s", dup_names.c_str(),
      duplicate_guard_enabled_ ? "ControlVehicle 차단" : "duplicate_writer_guard=false 라 송신은 계속");
  } else if (engine_->spec_refresh_pending()) {
    stat.summary(DiagnosticStatus::WARN, "Ego 복귀 뒤 스펙 재조회 대기 — 명령 보류");
  } else if (follow_mode_) {
    // 26.10.12 _ YSH _ VILS: 명령은 실차 인터페이스로 간다 (MORAI 로는 고스트 명령만)
    stat.summary(DiagnosticStatus::OK, "follow — 명령은 실차로 (MORAI 에는 고스트·위치만)");
  } else if (localize_mode_) {
    // 26.10.13 _ YSH _ VILS 차량 PC 고스트: MORAI 로는 아무것도 보내지 않는다
    stat.summary(DiagnosticStatus::OK, "localize — 명령은 실차로 (MORAI 에는 보내지 않는다)");
  } else if (!params->forward_control_to_sim) {
    stat.summary(DiagnosticStatus::OK, "forward_control_to_sim=false (MORAI 키보드 수동 주행)");
  } else if (d.stale_sends > 0) {
    stat.summaryf(
      DiagnosticStatus::WARN, "control_cmd 끊김 — 감속 대체 명령 %lu 회", ul(d.stale_sends));
  } else if (fails > 0) {
    stat.summaryf(DiagnosticStatus::WARN, "ControlVehicle 실패 %lu 회", ul(fails));
  } else if (nan_delta > 0) {
    stat.summaryf(DiagnosticStatus::WARN, "NaN/inf control_cmd %lu 개 무시", ul(nan_delta));
  } else if (timer && d.sends > 0 && d.cmd_timer_late * 10 > d.sends) {
    // 26.09.25 _ YSH _ timer 가 창의 10 % 넘게 늦으면(송신이 주기보다 오래 걸림 — 원격·느린 MORAI) WARN
    stat.summaryf(
      DiagnosticStatus::WARN, "명령 타이머 지연재설정 %lu / 송신 %lu (%g Hz 를 못 맞춘다)", ul(d.cmd_timer_late),
      ul(d.sends), params->cmd_resend_hz);
  } else {
    // 26.09.25 _ YSH _ 트리거를 같이 적는다. 원문: stat.summaryf(DiagnosticStatus::OK, "%.1f 송신/s", per_s(d.sends));
    stat.summaryf(
      DiagnosticStatus::OK, "%.1f 송신/s (command_trigger %s%s)", per_s(d.sends),
      to_string(params->command_trigger), timer ? ", 고정 주기" : ", 업데이트당 1 회");
  }
}

void MoraiInterfaceNode::diagnose_clock(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto off = engine_->clock_offset_ms();
  stat.add("time_source", to_string(engine_config_.time_source));
  stat.add("publish_clock", engine_config_.publish_clock);
  stat.add("samples", off.count);
  stat.add("offset_median_ms", off.p50);
  stat.add("offset_p01_ms", off.p01);
  stat.add("offset_p99_ms", off.p99);
  stat.add("offset_min_ms", off.min);
  stat.add("offset_max_ms", off.max);
  stat.add("warn_threshold_ms", engine_config_.clock_offset_warn_ms);
  // 26.09.25 _ YSH _ 리뷰 반영: 한 시간 넘게 어긋나면 시계 오차가 아니라 epoch 가 아닌 시각(sim time)이다
  const bool content_mode = engine_->gate_mode() == GateMode::kContent;
  stat.add("stamp_source", content_mode ? "receive (timestamp 못 씀)" : to_string(engine_config_.time_source));
  if (content_mode) {
    stat.summary(
      DiagnosticStatus::WARN, "MORAI timestamp 를 쓸 수 없어 stamp·/clock 은 이 PC 수신 시각이다");
  } else if (off.count == 0) {
    stat.summary(DiagnosticStatus::OK, "표본 없음");
  } else if (std::abs(off.p50) > kClockImplausibleMs) {
    stat.summaryf(
      DiagnosticStatus::ERROR,
      "MORAI timestamp − 수신 시각 %.0f s — 시계 오차가 아니다 (epoch 가 아닌 sim time?). time_source: "
      "morai 면 stamp 가 틀린다 → time_source: receive",
      off.p50 / 1000.0);
  } else if (std::abs(off.p50) > engine_config_.clock_offset_warn_ms) {
    stat.summaryf(
      DiagnosticStatus::WARN,
      "MORAI timestamp − 수신 시각 중앙값 %.1f ms (> %.0f ms) — 두 PC 시계 동기화(chrony) 확인", off.p50,
      engine_config_.clock_offset_warn_ms);
  } else {
    stat.summaryf(DiagnosticStatus::OK, "MORAI timestamp − 수신 시각 중앙값 %.1f ms", off.p50);
  }
}

// 26.10.10 _ YSH _ follow 모드 토픽 이름: /vehicle/status/velocity_status → /morai/ego/velocity_status
std::string MoraiInterfaceNode::ego_topic(const char * topic) const
{
  // 26.10.13 _ YSH _ localize 는 차량 상태만 /morai/ego/* 로 돌리고 측위(kinematic_state)는 그대로 낸다. 원문:
  // if (!follow_mode_) {
  //   return topic;
  // }
  if (!real_car_vehicle_ || (localize_mode_ && std::string(topic) == kTopicKinematicState)) {
    return topic;
  }
  const std::string t(topic);
  return "/morai/ego" + t.substr(t.rfind('/'));
}

// 26.10.10 _ YSH _ follow 모드: SetTransform 상태, 실차 측위 writer, MORAI Ego 가 따라오는가
// 26.10.12 _ YSH _ 신규 — 의사 VILS DAQ Ego: 실차 값으로 EgoVehicleStatus (DAQ 스레드에서 틱마다)
std::optional<morai_ros2_msgs::msg::EgoVehicleStatus> MoraiInterfaceNode::build_real_car_daq_ego()
{
  RealCarInputs in;
  {
    const std::lock_guard<std::mutex> lock(real_car_mutex_);
    in = real_car_;
  }
  const int64_t now = steady_now_ns();
  const auto fresh = [now](int64_t ns, int64_t max_age) { return ns > 0 && now - ns <= max_age; };
  if (!fresh(in.pose_ns, kRealCarPoseMaxAgeNs)) {
    ++follow_daq_missing_;
    return std::nullopt;
  }
  morai_to_autoware::RealCarEgoSample s;
  s.map_pose = in.pose.pose.pose;
  if (fresh(in.velocity_ns, kRealCarInputMaxAgeNs)) {
    s.speed_mps = in.velocity.longitudinal_velocity;
    s.lateral_mps = in.velocity.lateral_velocity;
    s.yaw_rate_rps = in.velocity.heading_rate;
  } else {  // CAN 이 끊기면 측위 속도
    s.speed_mps = in.pose.twist.twist.linear.x;
    s.lateral_mps = in.pose.twist.twist.linear.y;
    s.yaw_rate_rps = in.pose.twist.twist.angular.z;
  }
  if (fresh(in.kcan_imu_ns, kRealCarInputMaxAgeNs)) {
    s.accel_long_mps2 = in.kcan_imu.linear_acceleration.x;
    s.accel_lat_mps2 = in.kcan_imu.linear_acceleration.y;
  } else if (fresh(in.accel_ns, kRealCarInputMaxAgeNs)) {  // K-CAN 가속도가 없으면 속도 미분(twist2accel) + v·r
    s.accel_long_mps2 = in.accel.accel.accel.linear.x;
    s.accel_lat_mps2 = s.speed_mps * s.yaw_rate_rps;
  }
  if (fresh(in.steering_ns, kRealCarInputMaxAgeNs)) {
    s.tire_angle_rad = in.steering.steering_tire_angle;
  }
  if (fresh(in.actuation_ns, kRealCarInputMaxAgeNs)) {
    s.throttle = in.actuation.status.accel_status / 100.0;  // 가속페달 [%]
    s.brake = follow_daq_brake_full_bar_ > 0.0 ? in.actuation.status.brake_status / follow_daq_brake_full_bar_ : 0.0;
  }
  auto msg = morai_to_autoware::to_daq_ego_status_real(s, runtime_params_.get()->map_origin);
  msg.header.stamp = in.pose.header.stamp;
  return msg;
}

void MoraiInterfaceNode::diagnose_follow(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const auto s = follow_->stats();
  std::string writers;
  size_t count = 0;
  {
    const std::lock_guard<std::mutex> lock(guard_mutex_);
    writers = localization_writer_names_;
    count = localization_writer_count_;
  }
  const int64_t now = steady_now_ns();
  const double since_ok = s.last_ok_ns > 0 ? static_cast<double>(now - s.last_ok_ns) * 1e-9 : -1.0;
  // 따라가기 오차: 마지막으로 보낸 목표 vs 지금 MORAI Ego (MORAI 좌표, 지연 보상 없음 — 속도 × 지연만큼 커진다)
  double err = -1.0;
  const auto state = state_cache_.latest();
  if (state && s.last_target) {
    err = std::hypot(state->ego.position.x - s.last_target->position.x, state->ego.position.y - s.last_target->position.y);
  }
  stat.add("input_topic", follow_input_topic_);
  stat.add("input_writers", writers);
  stat.add("sent", s.sent);
  stat.add("ok", s.ok);
  stat.add("failed", s.failed);
  stat.add("stale_skipped", s.stale_skipped);
  stat.add("no_pose", s.no_pose);
  stat.add("rpc_p50_ms", s.rpc_p50_ms);
  stat.add("rpc_p99_ms", s.rpc_p99_ms);
  stat.add("rpc_max_ms", s.rpc_max_ms);
  stat.add("since_last_ok_s", since_ok);
  stat.add("follow_error_m", err);
  stat.add("last_error", s.last_error);
  // 26.10.12 _ YSH _ 고스트 명령 통계
  stat.add("ghost", follow_config_.ghost ? "on" : "off");
  stat.add("ghost_sent", s.ghost_sent);
  stat.add("ghost_failed", s.ghost_failed);
  stat.add("ghost_last_error", s.ghost_last_error);
  stat.add("daq_ego_missing_ticks", follow_daq_missing_.load());  // 26.10.12 _ YSH _ 실차 측위가 낡아 DAQ Ego 를 못 낸 틱
  // 26.10.13 _ YSH _ Result 거부 수 (전송은 됐지만 MORAI 가 받지 않았다)
  stat.add("result_rejected", s.result_rejected);
  stat.add("ghost_rejected", s.ghost_rejected);
  // 26.10.13 _ YSH _ 고스트 판정을 직전 진단 주기 창으로 (종전 누적 계수는 1 시간 뒤면 같은 수만큼 실패해야 경고였다)
  //   전송 실패 + Result 거부를 함께 센다. 원문:
  // const bool ghost_bad = follow_config_.ghost && s.ghost_sent > 20 && s.ghost_failed * 2 > s.ghost_sent;
  const uint64_t ghost_bad_total = s.ghost_failed + s.ghost_rejected;
  const uint64_t d_sent = s.ghost_sent - std::min(s.ghost_sent, follow_prev_ghost_sent_);
  const uint64_t d_bad = ghost_bad_total - std::min(ghost_bad_total, follow_prev_ghost_bad_);
  follow_prev_ghost_sent_ = s.ghost_sent;
  follow_prev_ghost_bad_ = ghost_bad_total;
  const bool ghost_bad = follow_config_.ghost && d_sent >= 5 && d_bad * 2 > d_sent;
  // 26.10.13 _ YSH _ 고스트가 거부되면 MORAI Ego 가 AUTO 가 아닐 가능성이 크다 — AUTO 를 5 s 마다 다시 요청한다
  //   (종전에는 연결 때 한 번만, 500 ms 확인이 늦으면 다시 하지 않았다)
  if (ghost_bad && now - follow_auto_retry_ns_ > 5'000'000'000LL) {
    follow_auto_retry_ns_ = now;
    (void)worker_->post([this]() {
      const auto st = link_->set_control_mode(morai_grpc_vendor::ControlMode::kAuto, std::chrono::milliseconds(500));
      RCLCPP_WARN(get_logger(), "follow 고스트 거부가 이어져 MORAI Ego 제어 모드 AUTO 를 다시 요청했다 — %s",
        st.ok ? "성공" : st.message.c_str());
    });
  }
  // 26.10.13 _ YSH _ 따라가기 오차 판정 — 보낸 목표와 MORAI Ego 가 오래 어긋나면 MORAI 가 따라오지 않는 것이다
  //   (Result 가 성공이어도 Ego 가 딴 곳에 있으면 NPC·충돌 판정이 엉뚱한 자리 기준이 된다). 허용 = max(3 m, 0.3 s × 실차 속도),
  //   절반이면 WARN. fail_timeout_s 와 1 s 중 긴 시간 동안 계속 넘어야 ERROR
  double v_real = 0.0;
  {
    const std::lock_guard<std::mutex> lock(real_car_mutex_);
    if (real_car_.pose_ns > 0) {
      v_real = std::fabs(real_car_.pose.twist.twist.linear.x);
    }
  }
  const double err_tol = std::max(3.0, 0.3 * v_real);
  if (err >= 0.0 && err > err_tol) {
    follow_err_since_ns_ = follow_err_since_ns_ ? follow_err_since_ns_ : now;
  } else {
    follow_err_since_ns_ = 0;
  }
  const double err_for_s = follow_err_since_ns_ ? static_cast<double>(now - follow_err_since_ns_) * 1e-9 : 0.0;
  stat.add("follow_error_tol_m", err_tol);
  stat.add("follow_error_over_s", err_for_s);
  // 26.10.13 _ YSH _ SetTransform 을 시도조차 못 한 것(실차 측위 없음·낡음)과 MORAI 실패를 가른다
  const double since_attempt =
    s.last_attempt_ns > 0 ? static_cast<double>(now - s.last_attempt_ns) * 1e-9 : -1.0;
  const bool no_attempt = s.last_attempt_ns == 0 || since_attempt > follow_config_.fail_timeout_s;
  const bool pose_skipping =
    s.last_skip_ns > 0 && static_cast<double>(now - s.last_skip_ns) * 1e-9 <= follow_config_.fail_timeout_s + 1.0;
  if (count == 0) {
    stat.summaryf(DiagnosticStatus::ERROR, "%s 를 내는 노드가 없다 (GNSS 위치 노드?)", follow_input_topic_.c_str());
  } else if (count > 1) {
    stat.summaryf(DiagnosticStatus::ERROR, "%s 를 내는 노드가 %zu 개: %s", follow_input_topic_.c_str(), count, writers.c_str());
  } else if ((s.last_ok_ns == 0 || since_ok > follow_config_.fail_timeout_s) && no_attempt && pose_skipping) {
    // 26.10.13 _ YSH _ 신규 — MORAI 가 아니라 실차 측위 쪽 문제다 (GNSS 위치 노드가 아직·더 이상 내지 않는다)
    stat.summaryf(
      DiagnosticStatus::ERROR, "실차 측위 없음·낡음 — SetTransform 을 보내지 못했다 (no_pose %lu, stale %lu) → GNSS 위치 노드 확인",
      static_cast<unsigned long>(s.no_pose), static_cast<unsigned long>(s.stale_skipped));  // NOLINT(runtime/int)
  } else if (s.last_ok_ns == 0 || since_ok > follow_config_.fail_timeout_s) {
    stat.summaryf(
      DiagnosticStatus::ERROR, "SetTransform 성공 없음 %.1f s (실패 %lu: %s)", since_ok,
      static_cast<unsigned long>(s.failed), s.last_error.c_str());  // NOLINT(runtime/int)
  } else if (err_for_s > std::max(1.0, follow_config_.fail_timeout_s)) {
    // 26.10.13 _ YSH _ 신규 — 보내지는데 MORAI Ego 가 따라오지 않는다 (Result 거부·MORAI 쪽 제어기가 되돌림 등)
    stat.summaryf(
      DiagnosticStatus::ERROR, "MORAI Ego 가 따라오지 않는다 — 오차 %.1f m > %.1f m 가 %.1f s (Result 거부 %lu)", err, err_tol,
      err_for_s, static_cast<unsigned long>(s.result_rejected));  // NOLINT(runtime/int)
  } else if (err >= 0.0 && err > 0.5 * err_tol) {
    // 26.10.13 _ YSH _ 신규 — 허용의 절반을 넘었다 (이 그래프에서 WARN 도 MRM 이 될 수 있다 — autoware-vils.yaml 참고)
    stat.summaryf(DiagnosticStatus::WARN, "MORAI Ego 따라가기 오차 %.1f m (허용 %.1f m)", err, err_tol);
  } else if (ghost_bad) {
    // 26.10.12 _ YSH _ 위치는 따라가지만 고스트가 아니다 (물리가 켜진 채 순간이동 — NPC 판정이 달라질 수 있다)
    stat.summaryf(
      DiagnosticStatus::WARN, "MORAI Ego 따라가는 중이나 고스트 명령 실패·거부 %lu/%lu (직전 주기): %s",
      static_cast<unsigned long>(d_bad), static_cast<unsigned long>(d_sent),  // NOLINT(runtime/int)
      s.ghost_last_error.c_str());
  } else {
    stat.summaryf(
      DiagnosticStatus::OK, "MORAI Ego 따라가는 중 — RPC p50 %.1f ms, 오차 %.2f m", s.rpc_p50_ms, err);
  }
}

void MoraiInterfaceNode::diagnose_ground_truth(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!engine_config_.publish_ground_truth) {
    stat.summary(DiagnosticStatus::OK, "publish_ground_truth=false (차량 인터페이스만)");
    return;
  }
  // 26.09.25 _ YSH _ 리뷰 반영: 액터 목록은 Ego 와 같은 호출에서 와 나이가 표본 나이와 같다 (actor_hold_s 는 예약)
  const auto actor_age = engine_->sample_age_s();
  const EngineStats total = engine_->stats();
  std::string kinematic_others;
  {
    const std::lock_guard<std::mutex> lock(guard_mutex_);
    kinematic_others = kinematic_writer_names_;
  }
  stat.add("actor_list_age_ms", actor_age ? *actor_age * 1000.0 : -1.0);
  stat.add("actor_count", engine_->last_actor_count());
  stat.add("objects_published", ul(total.objects_published));
  stat.add("localization_initialized", localization_initialized_);
  stat.add("teleports_ok", ul(teleports_ok_.load()));
  stat.add("teleports_failed", ul(teleports_failed_.load()));
  stat.add(
    "other_kinematic_state_writers", kinematic_others.empty() ? std::string("none") : kinematic_others);
  // 26.10.13 _ YSH _ localize: twist 출처 (실차 속도가 없거나 낡으면 위치 변화 속도로 대신 — WARN)
  std::optional<LocalizeTwistStats> lts;
  if (localize_twist_) {
    lts = localize_twist_->stats();
    stat.add("twist_speed_source", to_string(lts->last_source));
    stat.add("twist_vehicle_speed_age_ms", lts->vehicle_age_s < 0.0 ? -1.0 : lts->vehicle_age_s * 1000.0);
    stat.add("twist_speed_mps", lts->last_speed_mps);
    stat.add("twist_yaw_rate_rps", lts->last_yaw_rate_rps);
    stat.add("twist_pose_fallback_total", ul(lts->pose_fallback));
    stat.add("twist_no_speed_total", ul(lts->no_speed));
  }
  if (!kinematic_others.empty()) {
    stat.summaryf(
      DiagnosticStatus::ERROR, "다른 노드(%s)도 /localization/kinematic_state 발행", kinematic_others.c_str());
  } else if (!actor_age) {
    stat.summary(DiagnosticStatus::ERROR, "액터 목록을 아직 받지 못했다");
  } else if (*actor_age > engine_config_.max_sample_age_s) {
    stat.summaryf(
      DiagnosticStatus::ERROR, "표본 %.0f ms 낡음 (> %.0f ms) — kinematic_state·TF·TrackedObjects 중단",
      *actor_age * 1000.0, engine_config_.max_sample_age_s * 1000.0);
  } else if (follow_mode_) {
    // 26.10.12 _ YSH _ VILS: 측위·초기화는 실차 GNSS 노드가 한다 (브리지는 객체만)
    stat.summaryf(
      DiagnosticStatus::OK, "follow — 객체 발행 중 (액터 %zu), 측위는 외부 노드", engine_->last_actor_count());
  } else if (lts && lts->last_source != LocalizeSpeedSource::kVehicle) {
    // 26.10.13 _ YSH _ 신규 — 실차 속도(K-CAN)가 없거나 낡았다. 위치 변화 속도(또는 창이 차기 전 0)로 내는 중
    stat.summaryf(
      DiagnosticStatus::WARN, "localize — 실차 속도 없음·낡음 (%.0f ms) → twist 속도 출처 %s. 차량 인터페이스 확인",
      lts->vehicle_age_s < 0.0 ? -1.0 : lts->vehicle_age_s * 1000.0, to_string(lts->last_source));
  } else {
    stat.summaryf(
      DiagnosticStatus::OK, "GT 발행 중 (액터 %zu, 측위 초기화 %s)", engine_->last_actor_count(),
      localization_initialized_ ? "완료" : "대기");
  }
}

// 26.09.26 _ YSH _ DAQ 진단: 게이트·창 Hz·결손·지연·표본/명령 나이
void MoraiInterfaceNode::diagnose_daq(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  if (!daq_sampler_) {
    stat.summary(DiagnosticStatus::OK, "publish_daq=false");
    return;
  }
  const int64_t now = steady_now_ns();
  const DaqStats cur = daq_sampler_->stats();
  const DaqStats & p = daq_diag_prev_.stats;
  const double dt = daq_diag_prev_.t_ns > 0 ? static_cast<double>(now - daq_diag_prev_.t_ns) / 1e9 : 0.0;
  const double state_hz =
    dt > 0.0 ? static_cast<double>(cur.state_published - p.state_published) / dt : 0.0;
  const uint64_t gaps = cur.gap_ticks - p.gap_ticks;
  // 26.09.26 _ YSH _ 리뷰 반영: p 는 daq_diag_prev_ 를 가리킨다 — 아래에서 덮어쓰기 전에 창 차이를 구한다 (뒤에서 구하면
  //   늘 0 이라 큐 넘침 WARN 이 나지 않았다)
  const uint64_t raw_drops = (cur.raw_dropped - p.raw_dropped) + (cur.tx_dropped - p.tx_dropped);
  daq_diag_prev_ = {now, cur};

  stat.add("mode", std::string(to_string(cur.mode)));
  stat.add("publishing", cur.publishing);
  stat.add("run_active", cur.active);
  stat.add("run", cur.active ? cur.run : std::string("none"));
  stat.add("run_elapsed_s", cur.run_elapsed_s);
  stat.add("state_hz", state_hz);
  stat.add("gap_ticks_window", ul(gaps));
  stat.add("late_total", ul(cur.late));
  stat.add("runs_started", ul(cur.runs_started));
  stat.add("runs_ended", ul(cur.runs_ended));
  stat.add("run_timeouts", ul(cur.run_timeouts));
  stat.add("frame_age_ms", cur.frame_age_s ? *cur.frame_age_s * 1000.0 : -1.0);
  stat.add("cmd_age_ms", cur.cmd_age_s ? *cur.cmd_age_s * 1000.0 : -1.0);
  stat.add("other_daq_writers", daq_writer_names_.empty() ? std::string("none") : daq_writer_names_);
  // 26.09.26 _ YSH _ 원시 gRPC 기록 (누적). dropped = 큐 넘침 (자료 손실), discarded = 게이트가 닫혀 버림 (정상)
  stat.add("raw", daq_config_.raw);
  if (daq_config_.raw) {
    stat.add("raw_published", ul(cur.raw_published));
    stat.add("raw_dropped", ul(cur.raw_dropped));
    stat.add("raw_discarded", ul(cur.raw_discarded));
    stat.add("tx_published", ul(cur.tx_published));
    stat.add("tx_dropped", ul(cur.tx_dropped));
    stat.add("tx_discarded", ul(cur.tx_discarded));
  }
  const bool link_up = engine_ && engine_->phase() == Engine::Phase::kRunning;
  if (cur.yielding && daq_sampler_->yield_pending_first_check()) {
    stat.summary(DiagnosticStatus::OK, "첫 /daq writer 점검 대기 (기동 직후 1 s)");  // 26.09.26 _ YSH _ 리뷰 반영
  } else if (cur.yielding) {
    stat.summaryf(DiagnosticStatus::WARN, "다른 노드(%s)가 /daq 를 낸다 — 양보 중", daq_writer_names_.c_str());
  } else if (!cur.publishing) {
    stat.summary(DiagnosticStatus::OK, "대기 (gated — run_state idle)");
  } else if (link_up && (!cur.frame_age_s || *cur.frame_age_s >= daq_config_.max_sample_age_s)) {
    stat.summary(DiagnosticStatus::ERROR, "발행 중인데 MORAI 표본이 낡았다 — 상태 토픽 결손");
  } else if (!cur.cmd_age_s || *cur.cmd_age_s > daq_config_.cmd_stale_s) {
    stat.summaryf(DiagnosticStatus::WARN, "발행 중 %.1f Hz — control_cmd 없음 (명세 25 공란)", state_hz);
  } else if (raw_drops > 0) {
    // 26.09.26 _ YSH _ 원시 기록 큐가 넘쳤다 (DAQ 스레드가 밀렸다) — 그만큼 원시 기록이 빠졌다
    stat.summaryf(
      DiagnosticStatus::WARN, "발행 중 %.1f Hz — 원시 기록 큐 넘침 %lu 건 (daq.raw_queue_depth)", state_hz,
      ul(raw_drops));
  } else {
    stat.summaryf(DiagnosticStatus::OK, "발행 중 %.1f Hz%s", state_hz, cur.active ? " (런)" : "");
  }
}

}  // namespace autoware::morai_interface

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::morai_interface::MoraiInterfaceNode)
