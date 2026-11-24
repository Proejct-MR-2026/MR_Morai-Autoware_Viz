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

// MORAI 차량 인터페이스 노드 — 소유와 배선만 담당하고 실제 변환은 두 방향 클래스가 한다
// 26.09.25 _ YSH _ 전면 교체: StatusReporter/CommandForwarder(33 Hz 콜백 송신·50 Hz 폴링 타이머) 대신
//   프레임 고정 Engine(엔진 스레드)이 MORAI 업데이트마다 스냅샷·ControlVehicle·발행을 한 번씩 한다.
//   이 노드는 파라미터·ROS 입출력·보조 스레드(기어 송신기, 작업 스레드)·진단만 소유한다.
//   executor 콜백은 RPC 를 부르지 않는다. 원본: git show 94cdb2b:<이 경로>

#ifndef AUTOWARE__MORAI_INTERFACE__NODE_HPP_
#define AUTOWARE__MORAI_INTERFACE__NODE_HPP_

// 26.09.25 _ YSH _ StatusReporter/CommandForwarder 를 엔진으로 교체
// #include "autoware/morai_interface/command_forwarder.hpp"
// #include "autoware/morai_interface/ego_vehicle_spec.hpp"
// #include "autoware/morai_interface/lamp_command_cache.hpp"
// #include "autoware/morai_interface/sensor_kit_mounter.hpp"
// #include "autoware/morai_interface/status_reporter.hpp"
#include "autoware/morai_interface/command_slot.hpp"
// 26.09.26 _ YSH _ DAQ 샘플러 (ViLS 명세 25 항목 100 Hz, cue_server 녹화 구간 게이트)
#include "autoware/morai_interface/daq_sampler.hpp"
// 26.09.25 _ YSH _ 리뷰 반영: 중복 writer 가드는 자기 publisher 하나만 뺀다 (이름으로 전부 빼면 같은 이름의 두 번째 인스턴스가 안 보인다)
#include "autoware/morai_interface/duplicate_writer_guard.hpp"
#include "autoware/morai_interface/engine.hpp"
#include "autoware/morai_interface/follow_streamer.hpp"  // 26.10.10 _ YSH _ 의사 VILS follow 모드
#include "autoware/morai_interface/gear_sender.hpp"
#include "autoware/morai_interface/job_worker.hpp"
#include "autoware/morai_interface/lamp_command_cache.hpp"
#include "autoware/morai_interface/link_interface.hpp"
#include "autoware/morai_interface/localize_twist.hpp"  // 26.10.13 _ YSH _ VILS 차량 PC 고스트 twist
#include "autoware/morai_interface/object_disturbance.hpp"  // 26.11.10 _ YSH _ 외란 주입 (손잡이·제어기)

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <autoware_control_msgs/msg/control.hpp>
#include <autoware_localization_msgs/srv/initialize_localization.hpp>
#include <autoware_perception_msgs/msg/tracked_objects.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>  // 26.10.12 _ YSH _ VILS DAQ
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>  // 26.11.12 _ YSH _ /daq/calc/ego_*_world
// 26.10.05 _ YSH _ DAQ 명세 토픽
#include <morai_ros2_msgs/msg/ctrl_cmd.hpp>  // 26.10.06 _ YSH _
#include <morai_ros2_msgs/msg/ego_vehicle_status.hpp>
#include <morai_ros2_msgs/msg/object_status_list.hpp>
// 26.11.10 _ YSH _ 외란 주입: <prefix>/disturbance · 서비스 ~/set_disturbance
#include <morai_ros2_msgs/msg/disturbance_status.hpp>
#include <morai_ros2_msgs/srv/set_disturbance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>  // 26.10.12 _ YSH _ VILS DAQ
#include <std_msgs/msg/string.hpp>
#include <tier4_vehicle_msgs/msg/actuation_status_stamped.hpp>  // 26.10.12 _ YSH _ VILS DAQ

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::morai_interface
{

/// Vehicle interface + ground-truth bridge between Autoware and the MORAI simulator.
///
/// 26.09.25 _ YSH _ 구조 (main.py 대체):
///   - Engine 스레드 : WaitForTick → GetAllActorsState → 판정 → StateCache → ControlVehicle 1 회 → 발행
///   - GearSender    : 기어 변경 시 송신 / 0.2 s 재시도 / 1 s 재동기 (main.py _GearSender)
///   - JobWorker     : /initialpose 텔레포트, 램프, 센서 장착
///   - 모드 JobWorker : control_mode_request(지연 응답) — 텔레포트(최대 ~2 s) 뒤에 줄 서지 않게 따로 (리뷰 반영)
///   - executor      : 구독 콜백은 슬롯·캐시에 넣기만, 램프 리포트 10 Hz, 측위 초기화 1 Hz, 진단 1 Hz
/// Topic names and QoS (depth 10) are identical to main.py, so this node is a drop-in
/// replacement for it — and must NEVER run at the same time (a duplicate-writer guard blocks
/// commands if another publisher of /vehicle/status/velocity_status appears).
/// 26.09.25 _ YSH _ 리뷰 반영: GT launch 의 노드 이름 기본값은 main.py 와 같은 morai_bridge 다
///   (daq_publisher·mission_server·scenario_pose 가 /morai_bridge 파라미터를 읽고 쓴다).
/// 26.09.26 _ YSH _ DAQ(/daq/*) 도 낸다 (publish_daq) — DaqSink. 원문:
/// class MoraiInterfaceNode : public rclcpp::Node, private EngineSink
class MoraiInterfaceNode : public rclcpp::Node, private EngineSink, private DaqSink
{
public:
  explicit MoraiInterfaceNode(const rclcpp::NodeOptions & options);
  ~MoraiInterfaceNode() override;

  // 26.09.25 _ YSH _ 리뷰 반영: 엔진이 지금 쓰는 런타임 파라미터 사본 (읽기 전용). 파라미터 서버 값이 아니라 기동 대체·on-set
  //   검증을 거쳐 실제로 도는 값 — 노드 파라미터 시험(test_node_params)이 command_trigger·cmd_resend_hz 를 확인한다
  std::shared_ptr<const RuntimeParams> effective_runtime_params() const
  {
    return runtime_params_.get();
  }

private:
  using Control = autoware_control_msgs::msg::Control;
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using TurnIndicatorsCommand = autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
  using HazardLightsCommand = autoware_vehicle_msgs::msg::HazardLightsCommand;
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand;
  using InitializeLocalization = autoware_localization_msgs::srv::InitializeLocalization;
  using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;
  using SetDisturbance = morai_ros2_msgs::srv::SetDisturbance;  // 26.11.10 _ YSH _

  // --- EngineSink (엔진 스레드에서 불린다) ---
  void publish(const UpdateOutputs & outputs) override;
  void on_connected(bool reconnected) override;
  // 26.09.26 _ YSH _ DAQ: 엔진이 받아들인 표본을 DAQ 캐시에 (엔진 스레드)
  void on_sample(
    const morai_grpc_vendor::Snapshot & snapshot, const builtin_interfaces::msg::Time & stamp,
    uint64_t update_index, int64_t receive_steady_ns) override;
  // 26.09.26 _ YSH _ 원시 gRPC 기록: 보낸 ControlVehicle 을 DAQ 큐에 (엔진·명령 타이머 스레드, 복사만)
  void on_command_sent(
    const morai_grpc_vendor::VehicleCommand & cmd, bool stale, int64_t send_system_ns,
    const morai_grpc_vendor::Status & status) override;

  // --- 26.09.26 _ YSH _ DaqSink (DAQ 스레드 / 런을 끝낸 스레드에서 불린다) ---
  void publish_daq(const DaqOutputs & outputs) override;
  void on_daq_run_end(const DaqRunSummary & summary) override;
  void publish_daq_raw(const DaqRawOutputs & outputs) override;  // 26.09.26 _ YSH _ 원시 gRPC 기록
  void on_daq_run_state(const std_msgs::msg::String::ConstSharedPtr msg);

  // --- 파라미터 ---
  void declare_parameters();
  rcl_interfaces::msg::SetParametersResult on_set_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  // --- executor 콜백 (RPC 없음) ---
  void on_control_cmd(const Control::ConstSharedPtr msg);
  void on_gear_cmd(const GearCommand::ConstSharedPtr msg);
  void on_turn_indicators_cmd(const TurnIndicatorsCommand::ConstSharedPtr msg);
  void on_hazard_lights_cmd(const HazardLightsCommand::ConstSharedPtr msg);
  void on_initialpose(const PoseWithCovarianceStamped::ConstSharedPtr msg);
  void on_control_mode_request(
    const std::shared_ptr<rclcpp::Service<ControlModeCommand>> service,
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<ControlModeCommand::Request> request);
  // 26.11.10 _ YSH _ 외란 명령 (~/set_disturbance). 읽고 검사해 설정만 바꿔 끼우고 바로 답한다 — 적용은 다음 객체 프레임
  void on_set_disturbance(
    const std::shared_ptr<SetDisturbance::Request> request,
    const std::shared_ptr<SetDisturbance::Response> response);
  void on_lamp_timer();
  void on_localization_init_timer();
  void on_guard_timer();

  // --- 작업 스레드 ---
  void post_lamp_sync(bool force);
  void run_lamp_sync();
  void run_teleport();
  void mount_sensors_once();

  // --- 진단 ---
  void diagnose_link(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_update(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_command(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_clock(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_ground_truth(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_daq(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void diagnose_follow(diagnostic_updater::DiagnosticStatusWrapper & stat);  // 26.10.10 _ YSH _
  /// 26.10.10 _ YSH _ follow 모드면 Ego 출력 토픽을 /morai/ego/<이름> 으로 (실차 인터페이스·측위와 겹치지 않게)
  std::string ego_topic(const char * topic) const;

  int64_t steady_now_ns() const;

  // --- 기동 시 한 번 읽는 설정 ---
  EngineConfig engine_config_;
  morai_grpc_vendor::MoraiLinkOptions link_options_;
  GearSenderConfig gear_config_;
  std::string sensor_model_;

  // --- 런타임 파라미터 ---
  std::mutex params_mutex_;  ///< on-set 콜백의 읽기-수정-쓰기 직렬화
  RuntimeParams runtime_params_value_;
  RuntimeParamsHandle runtime_params_;

  // --- 26.11.10 _ YSH _ 외란 주입 (disturbance.enable). 손잡이·제어기는 engine_ 보다 먼저 선언한다 (더 오래 산다) ---
  bool disturbance_enable_{false};  ///< 파라미터 disturbance.enable (기동 시 1 회) — 서비스·제어기를 만든다
  /// enable && publish_ground_truth && 실차 모드 아님 — 엔진에 손잡이를 넘기고 외란 DAQ 토픽을 만든다
  bool disturbance_engine_{false};
  DisturbanceHandle disturbance_handle_;
  std::unique_ptr<DisturbanceController> disturbance_ctl_;  ///< 서비스(executor)·런 종료 훅(executor·DAQ 스레드·종료 경로)

  // --- 스레드 사이 공유 상태 ---
  CommandSlot command_slot_;
  StateCache state_cache_;
  LampCommandCache lamp_cache_;
  std::atomic<uint64_t> nan_commands_{0};
  std::atomic<bool> lamp_sync_queued_{false};
  std::atomic<bool> lamp_force_{false};
  std::optional<int> last_sent_signal_;  ///< 작업 스레드만 쓴다

  struct TeleportRequest
  {
    morai_grpc_vendor::TransformPose target;
    double map_x{0.0};
    double map_y{0.0};
    double map_yaw_rad{0.0};
  };
  std::mutex teleport_mutex_;
  std::optional<TeleportRequest> teleport_pending_;  ///< 가장 최근 요청만 (쌓이지 않게)
  std::atomic<uint64_t> teleports_ok_{0};
  std::atomic<uint64_t> teleports_failed_{0};

  // --- MORAI ---
  std::unique_ptr<MoraiLinkClient> link_;
  std::unique_ptr<Engine> engine_;
  std::unique_ptr<GearSender> gear_sender_;
  std::unique_ptr<JobWorker> worker_;
  // 26.09.25 _ YSH _ 리뷰 반영: control_mode_request 전용 (텔레포트·램프 뒤에 줄 서지 않게)
  std::unique_ptr<JobWorker> mode_worker_;
  bool sensors_mounted_{false};  ///< 작업 스레드만 쓴다

  // --- ROS 출력 ---
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr pub_velocity_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr pub_steering_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::GearReport>::SharedPtr pub_gear_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr pub_control_mode_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::TurnIndicatorsReport>::SharedPtr
    pub_turn_indicators_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::HazardLightsReport>::SharedPtr pub_hazard_lights_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_kinematic_state_;
  rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr pub_objects_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr pub_clock_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // --- ROS 입력 ---
  rclcpp::Subscription<Control>::SharedPtr sub_control_;
  rclcpp::Subscription<GearCommand>::SharedPtr sub_gear_;
  rclcpp::Subscription<TurnIndicatorsCommand>::SharedPtr sub_turn_indicators_;
  rclcpp::Subscription<HazardLightsCommand>::SharedPtr sub_hazard_lights_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;
  rclcpp::Service<ControlModeCommand>::SharedPtr srv_control_mode_;
  rclcpp::Client<InitializeLocalization>::SharedPtr init_localization_client_;

  // --- 26.10.10 _ YSH _ 의사 VILS follow 모드 (ego_mode follow) ---
  //   실차 측위(/localization/kinematic_state)를 MORAI Ego 에 SetTransform 으로 옮긴다. 브리지는 Ego 상태를 Autoware
  //   이름으로 내지 않고(/morai/ego/*), TF·control_mode_request·/initialpose 를 만들지 않으며, MORAI 로 명령을 보내지 않는다
  bool follow_mode_{false};
  // 26.10.13 _ YSH _ VILS(차량 PC 고스트) localize 모드 — 차량 PC 가 자기 GNSS 로 MORAI Ego 를 옮기고, 이 PC 는 그 Ego 를
  //   측위로 받는다. 측위(kinematic_state·TF·측위 초기화)는 gt 와 같고, 차량 쪽은 follow 와 같다 (Ego 상태 /morai/ego/*,
  //   명령·기어·모드·텔레포트·control_mode_request 없음). MORAI 로는 아무것도 쓰지 않는다 (등화 동기화도 안 한다)
  bool localize_mode_{false};
  // 26.11.13 _ YSH _ 13_ 시각화 전용: false 면 MORAI 로 등화(SetVehicleTailLight)를 보내지 않는다 (파라미터 lamp_sync)
  bool lamp_sync_enable_{true};
  // 26.10.13 _ YSH _ follow·localize 공통: Autoware 차량은 실차(K-CAN 인터페이스)다
  bool real_car_vehicle_{false};
  std::string ego_mode_name_{"gt"};
  // 26.10.13 _ YSH _ localize: kinematic_state twist 를 실차 속도·위치 헤딩 변화율로 (차량 PC 고스트는 MORAI 속도가 0).
  //   localize.twist_source=morai 면 만들지 않는다 (MORAI 보고값 그대로)
  bool localize_twist_from_vehicle_{true};
  LocalizeTwistConfig localize_twist_config_;
  std::unique_ptr<LocalizeTwist> localize_twist_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr sub_localize_velocity_;
  FollowConfig follow_config_;
  std::string follow_input_topic_{"/localization/kinematic_state"};
  std::unique_ptr<FollowStreamer> follow_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_follow_pose_;
  std::string localization_writer_names_;  ///< follow: /localization/kinematic_state 를 내는 다른 노드 (guard_mutex_)
  size_t localization_writer_count_{0};

  // --- 26.10.12 _ YSH _ 의사 VILS 2단계: 경로·장애물 말고는 실차에서 (DAQ Ego·등화) ---
  /// 실차 최신 값 (executor 콜백이 쓰고 DAQ 스레드가 읽는다). steady ns 0 = 아직 없음
  struct RealCarInputs
  {
    nav_msgs::msg::Odometry pose;
    int64_t pose_ns{0};
    autoware_vehicle_msgs::msg::VelocityReport velocity;
    int64_t velocity_ns{0};
    autoware_vehicle_msgs::msg::SteeringReport steering;
    int64_t steering_ns{0};
    sensor_msgs::msg::Imu kcan_imu;
    int64_t kcan_imu_ns{0};
    tier4_vehicle_msgs::msg::ActuationStatusStamped actuation;
    int64_t actuation_ns{0};
    geometry_msgs::msg::AccelWithCovarianceStamped accel;
    int64_t accel_ns{0};
  };
  mutable std::mutex real_car_mutex_;
  RealCarInputs real_car_;
  double follow_daq_brake_full_bar_{100.0};  ///< 브레이크 압력 [bar] → DAQ brake 0~1
  std::atomic<uint64_t> follow_daq_missing_{0};  ///< 실차 측위가 낡아 ego_status 를 못 낸 틱
  // 26.10.13 _ YSH _ follow 진단 상태 (진단 스레드만 쓴다): 오차 초과 시작 시각 · 고스트 창 계수 · AUTO 재요청 시각
  int64_t follow_err_since_ns_{0};
  uint64_t follow_prev_ghost_sent_{0};
  uint64_t follow_prev_ghost_bad_{0};
  int64_t follow_auto_retry_ns_{0};
  std::optional<morai_ros2_msgs::msg::EgoVehicleStatus> build_real_car_daq_ego();
  std::vector<rclcpp::SubscriptionBase::SharedPtr> real_car_subs_;

  // --- 26.09.26 _ YSH _ DAQ (publish_daq 일 때만 만든다 — cue_server preflight 가 publisher 수를 센다) ---
  bool publish_daq_{false};
  DaqConfig daq_config_;
  DaqMode daq_mode_initial_{DaqMode::kGated};
  std::string daq_prefix_{"/daq"};
  DaqFrameCache daq_cache_;
  DaqCommandLatch daq_cmd_latch_;
  std::unique_ptr<DaqSampler> daq_sampler_;
  // 26.09.26 _ YSH _ 리뷰 반영: Ctrl-C 때 컨텍스트가 닫히기 전에 DAQ 를 멈춰 'shutdown' 요약이 실제로 나가게
  rclcpp::PreShutdownCallbackHandle daq_pre_shutdown_;
  // 26.10.05 _ YSH _ 명세 토픽을 MORAI 공식 메시지로 — <prefix>/ego_status·<prefix>/object_status
  rclcpp::Publisher<morai_ros2_msgs::msg::EgoVehicleStatus>::SharedPtr pub_daq_ego_;
  // 26.11.12 _ YSH _ 계산 토픽 — ego_status 와 같은 틱·같은 stamp (world 로 돌린 자차 속도·가속도)
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_daq_ego_vel_world_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_daq_ego_acc_world_;
  rclcpp::Publisher<morai_ros2_msgs::msg::ObjectStatusList>::SharedPtr pub_daq_objects_;
  rclcpp::Publisher<morai_ros2_msgs::msg::CtrlCmd>::SharedPtr pub_daq_ctrl_;  // 26.10.06 _ YSH _ 명세 25 (gRPC)
  rclcpp::Publisher<morai_ros2_msgs::msg::CollisionData>::SharedPtr pub_daq_collision_;  // 26.10.07 _ YSH _ /daq/collision
  // 원문: rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_daq_state_;
  // 원문: rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>::SharedPtr pub_daq_accel_;
  // 원문: rclcpp::Publisher<autoware_perception_msgs::msg::TrackedObjects>::SharedPtr pub_daq_objects_;
  rclcpp::Publisher<Control>::SharedPtr pub_daq_cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_daq_run_summary_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_daq_run_state_;
  // 26.09.26 _ YSH _ 원시 gRPC 기록 (publish_daq && daq.raw 일 때만): <prefix>/raw/{actor_states,vehicle_ctrl_cmd}
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_daq_raw_actor_states_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_daq_raw_ctrl_cmd_;
  std::atomic<uint64_t> daq_tx_seq_{0};  ///< send_seq — ControlVehicle 을 부를 때마다 +1 (기록 여부와 무관)
  // 26.11.10 _ YSH _ 외란 DAQ (publish_daq && disturbance_engine_ 일 때만): 객체 프레임마다 한 건 — 엔진 스레드가
  //   TrackedObjects 직후, DAQ 게이트가 열린 동안만 낸다. stamp 는 그 TrackedObjects 의 stamp (daq.stamp 와 무관).
  //   <prefix>/object_status_disturbed 는 daq.publish_objects 도 켜져 있어야 만든다
  rclcpp::Publisher<morai_ros2_msgs::msg::DisturbanceStatus>::SharedPtr pub_daq_disturbance_;
  rclcpp::Publisher<morai_ros2_msgs::msg::ObjectStatusList>::SharedPtr pub_daq_objects_disturbed_;
  // 26.11.11 _ YSH _ <prefix>/object_disturbance_status — /daq/object_status 의 외란 짝 (객체마다 외란 전·후·변화·시드).
  //   object_status_disturbed 와 같은 조건(daq.publish_objects)·같은 프레임·같은 stamp
  rclcpp::Publisher<morai_ros2_msgs::msg::ObjectDisturbanceStatusList>::SharedPtr pub_daq_object_disturbance_;
  rclcpp::Service<SetDisturbance>::SharedPtr srv_disturbance_;
  std::mutex disturbance_status_mutex_;  ///< 아래 한 칸 (엔진 스레드가 쓰고 서비스 콜백이 읽는다)
  morai_ros2_msgs::msg::DisturbanceStatus last_disturbance_status_;  ///< 마지막 객체 프레임의 상태
  std::string daq_writer_names_;  ///< 진단용 (executor)
  struct DaqDiagWindow
  {
    int64_t t_ns{0};
    DaqStats stats;
  };
  DaqDiagWindow daq_diag_prev_;  ///< 진단 콜백만 (executor)

  rclcpp::TimerBase::SharedPtr lamp_timer_;
  rclcpp::TimerBase::SharedPtr localization_init_timer_;
  rclcpp::TimerBase::SharedPtr guard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  std::unique_ptr<diagnostic_updater::Updater> updater_;

  // --- executor 만 쓰는 상태 ---
  // 26.09.25 _ YSH _ 리뷰 반영: 아래 멤버는 잠금이 없다. 모든 콜백(구독·타이머·서비스·클라이언트 응답·파라미터·
  //   진단)이 노드 기본 콜백 그룹(MutuallyExclusive) 하나에 있어 MultiThreadedExecutor 에서도 한 번에 하나만
  //   돈다는 것에 기댄다. Reentrant 그룹을 더하거나 콜백을 다른 그룹으로 옮기면 경쟁이 생긴다.
  bool was_forwarding_{true};
  bool localization_initialized_{false};
  bool localization_init_requested_{false};
  int64_t localization_request_id_{-1};
  int64_t localization_request_ns_{0};
  bool duplicate_guard_enabled_{true};  ///< duplicate_writer_guard 파라미터 (기동 시 1 회)
  bool guard_checked_{false};           ///< 첫 가드 판정 전에는 ControlVehicle 을 막아 둔다
  std::atomic<bool> duplicate_writer_{false};
  std::mutex guard_mutex_;
  std::string duplicate_writer_names_;
  std::string kinematic_writer_names_;

  // --- 진단 창 ---
  struct DiagWindow
  {
    int64_t t_ns{0};
    EngineStats stats;
    uint64_t nan_commands{0};
  };
  std::mutex diag_mutex_;
  DiagWindow diag_prev_;
  double diag_dt_s_{0.0};
  EngineStats diag_delta_;
  uint64_t diag_nan_delta_{0};
  void refresh_diag_window();
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__NODE_HPP_
