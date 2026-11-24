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

// SIM -> ADS 방향 순수 변환 — MORAI 상태를 Autoware 리포트로. 조향 부호 반전이 여기 있다
// 26.09.25 _ YSH _ 입력을 proto(ActorState)에서 MoraiLink 원값 구조체(EgoSample/ActorSample)로 바꾸고,
//   main.py 가 이미 고친 두 가지(각속도 = world rad/s 를 body 로 회전, CRUISE 매핑)와 GT 출력
//   (kinematic_state·TF·TrackedObjects·uuid5)을 main.py 식 그대로 옮겼다. 원본: git show 94cdb2b:<이 경로>

#ifndef AUTOWARE__MORAI_INTERFACE__CONVERSION__MORAI_TO_AUTOWARE_HPP_
#define AUTOWARE__MORAI_INTERFACE__CONVERSION__MORAI_TO_AUTOWARE_HPP_

#include "autoware/morai_interface/ego_vehicle_spec.hpp"
// 26.09.25 _ YSH _ map 변환 파라미터 (main.py map_origin_*)
#include "autoware/morai_interface/map_origin.hpp"

// 26.09.25 _ YSH _ MoraiLink 원값 구조체 (grpc·protobuf 비의존)
#include <morai_grpc_vendor/morai_link_types.hpp>

#include <autoware_control_msgs/msg/control.hpp>  // 26.10.12 _ YSH _ VILS /daq/ctrl_cmd (실차에 보낸 명령)
#include <autoware_perception_msgs/msg/tracked_object.hpp>
#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_report.hpp>
#include <autoware_vehicle_msgs/msg/steering_report.hpp>
#include <autoware_vehicle_msgs/msg/velocity_report.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>  // 26.11.12 _ YSH _ /daq/calc/ego_*_world
// 26.10.05 _ YSH _ DAQ 명세 토픽 (MORAI 공식 메시지 26.R1)
#include <morai_ros2_msgs/msg/collision_data.hpp>  // 26.10.07 _ YSH _ /daq/collision
#include <morai_ros2_msgs/msg/ctrl_cmd.hpp>  // 26.10.06 _ YSH _
#include <morai_ros2_msgs/msg/ego_vehicle_status.hpp>
#include <morai_ros2_msgs/msg/object_status_list.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

// 26.09.25 _ YSH _ proto 헤더 대신 MoraiLink 구조체를 쓴다
// // Generated protobuf headers end in .h, so .clang-format's "C system headers" rule
// // (<.*\.h>) claims them and they sort into this group rather than next to the other
// // package includes. That grouping is correct; do not reorder.
// #include <morai/actor/actor_enum.pb.h>
// #include <morai/actor/actor_get.pb.h>

#include <array>
#include <cstdint>
#include <string>
#include <vector>  // 26.10.05 _ YSH _ to_daq_object_status_list

namespace autoware::morai_interface::morai_to_autoware
{

/// MORAI reports speeds in km/h. Verified live: the value matched the simulator's own
/// dashboard reading. autoware_vehicle_msgs/VelocityReport is specified in m/s.
inline constexpr double kKmphToMps = 1.0 / 3.6;

/// Sign flip applied to lateral quantities coming out of MORAI (lateral velocity and the
/// steering report).
///
/// MORAI is a Unity application and uses a left-handed frame, so lateral velocity and
/// steering angle come out with the opposite sign to the ROS convention.
/// 26.09.25 _ YSH _ 각속도는 더 이상 이 부호로 처리하지 않는다 — angular_velocity_in_body() 참고.
///
/// Live evidence for the steering case: during a left turn the IMU reported
/// angular_velocity.z = +0.94 rad/s (ROS convention: left is positive) while the raw
/// MORAI field implied -33.26 deg. Magnitude agreed with the bicycle model to within
/// 0.88-1.11, and the sign disagreed on 15 of 15 samples.
///
/// NOTE this is a constant, not a parameter, and it is deliberately NOT symmetric with
/// the command direction -- see conversion/autoware_to_morai.hpp, where the equivalent
/// sign is +1.0 because VehicleCtrlCmd.steer already follows the ROS convention. Do not
/// "unify" the two; that asymmetry is a property of the MORAI API.
inline constexpr double kMoraiToRosLateralSign = -1.0;

// 26.10.06 _ YSH _ MORAI 차체 velocity 의 축은 (x 앞, y 위, z 왼)이다 (DAQ 감사 D1, 82 bag 207,927 표본 — heading 으로 돌려
//   global_velocity 를 RMS 1e-4 m/s 로 재현). 위 상수를 velocity.y 에 쓰면 횡속도 자리에 −수직속도가 들어갔다.
//   위 상수는 조향(front_wheel_angle)에만 쓴다. 속도 축은 아래 두 함수로만 읽는다.
//   한계: velocity 는 transform.location(뒤차축)보다 앞 점(Ego 약 1.25 m)의 속도라 선회 중 횡속도에 ψ̇·d 가 섞인다 (최대 약 0.27 m/s)
/// 차체 횡속도 [m/s], 왼쪽 + (ROS)
inline double lateral_velocity_mps(const morai_grpc_vendor::Vec3 & velocity_kmh)
{
  return velocity_kmh.z * kKmphToMps;
}
/// 차체 수직속도 [m/s], 위 +
inline double vertical_velocity_mps(const morai_grpc_vendor::Vec3 & velocity_kmh)
{
  return velocity_kmh.y * kKmphToMps;
}

/// MORAI 는 각도를 degree 로 준다 (rotation, max_steering_angle).
/// 이 패키지에서 deg->rad 는 여기 한 곳에서만 정의한다 — 복제본이 생기면
/// README 검증 3단계가 잡으려는 deg/rad 오류의 씨앗이 된다.
inline constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

// 26.09.25 _ YSH _ main.py 와 비트 단위로 같게 하려고 main.py 의 식(deg / (180/pi))을 그대로 쓰는 변환.
/// GetVehicleSpec.max_steering_angle [deg] -> [rad]. main.py get_max_steering_angle:
/// `res.max_steering_angle / (180.0 / math.pi)`
double max_steering_angle_rad(double max_steering_angle_deg);

// 26.09.25 _ YSH _ 신규 — main.py 의 math.radians 와 같은 식 (x * pi / 180)
double radians(double deg);

// 26.09.25 _ YSH _ 신규 — morai_coordinates.py angular_velocity_in_body 이식.
/// MORAI angular_velocity -> ROS body-frame angular velocity [rad/s].
///
/// The raw field is NOT deg/s in the body frame (the old C++ assumed that). Recorded MORAI
/// data (2026-09-17/21) matches a WORLD angular vector in rad/s with ENU axes
/// (wx, wy, wz) = (-raw.z, raw.x, -raw.y); it is rotated into the body with
/// R = Rz(yaw) Ry(pitch) Rx(roll) from Transform.rotation [deg] (the ORIGINAL MORAI rotation,
/// before map-origin offsets — those cancel for a body vector).
std::array<double, 3> angular_velocity_in_body(
  const morai_grpc_vendor::Vec3 & angular_velocity_raw,
  const morai_grpc_vendor::Vec3 & rotation_deg);

// 26.09.25 _ YSH _ 신규 — main.py _quaternion_from_rpy 이식 (ZYX intrinsic). 반환 (x, y, z, w)
std::array<double, 4> quaternion_from_rpy(double roll, double pitch, double yaw);

// 26.09.25 _ YSH _ 신규 — main.py get_map_pose 이식 (ego·NPC 공통 리지드 변환)
/// MORAI transform (ENU location [m], rotation [deg], CCW east-0) -> map-frame Pose.
geometry_msgs::msg::Pose to_map_pose(
  const morai_grpc_vendor::Vec3 & location, const morai_grpc_vendor::Vec3 & rotation_deg,
  const MapOrigin & origin);

// 26.09.25 _ YSH _ 입력을 EgoSample 로, heading_rate 를 body 각속도 z 로 교체 (main.py publish_velocity_report)
/// MORAI ego sample -> VelocityReport (header.frame_id = base_link, stamp left to the caller).
///
/// velocity is local km/h -> m/s with the lateral sign flip; heading_rate is the body-frame
/// z of angular_velocity_in_body().
autoware_vehicle_msgs::msg::VelocityReport to_velocity_report(
  const morai_grpc_vendor::EgoSample & ego);
// autoware_vehicle_msgs::msg::VelocityReport to_velocity_report(
//   const morai_sim_api::actor::ActorState & state);

// 26.09.25 _ YSH _ 조향 결손 보정 역보정(× steer_comp_gain)을 더하고 입력을 원값 double 로 (main.py publish_steering_report)
/// MORAI front_wheel_angle -> SteeringReport (stamp left to the caller).
///
///   steering_tire_angle = -front_wheel_angle * max_steering_angle * steer_comp_gain
///
/// front_wheel_angle is a normalised [-1, 1] value (full lock = +-1.0), Unity sign.
/// steer_comp_gain is the gain the LAST SENT command was divided by (1.0 when compensation
/// is off): the read-back echoes what we sent, so this undoes the compensation.
/// Returns a zeroed report if the spec has not been fetched yet.
autoware_vehicle_msgs::msg::SteeringReport to_steering_report(
  double front_wheel_angle, const EgoVehicleSpec & spec, double steer_comp_gain);
// autoware_vehicle_msgs::msg::SteeringReport to_steering_report(
//   const morai_sim_api::actor::VehicleState & state, const EgoVehicleSpec & spec);

/// MORAI GearMode -> GearReport.report.
///
///   M=0 -> NONE(0)      P=1 -> PARK(22)     R=2 -> REVERSE(20)
///   N=3 -> NEUTRAL(1)   D=4 -> DRIVE(2)     L=5 -> LOW(23)
// 26.09.25 _ YSH _ 입력을 MoraiLink 열거형 사본으로
uint8_t to_gear_report(morai_grpc_vendor::GearMode gear);
// uint8_t to_gear_report(morai_sim_api::actor::GearMode gear);

// 26.09.25 _ YSH _ 매핑 변경: CRUISE(6) 는 main.py 대로 DISENGAGED(5) (MORAI 가 직접 운전하는 상태를
//   Autoware 제어로 가리지 않는다 — 호출자가 WARN), SYNCHRONOUS(7) 은 AUTONOMOUS(1) 유지.
/// MORAI VehicleControlMode -> ControlModeReport.mode.
///
///   UNSPECIFIED(0) -> NO_COMMAND(0)      KEYBOARD(1)/GAME_WHEEL(2) -> MANUAL(4)
///   AUTO(3) -> AUTONOMOUS(1)             AUTO_LATERAL(4) -> AUTONOMOUS_STEER_ONLY(2)
///   AUTO_LONGITUDINAL(5) -> AUTONOMOUS_VELOCITY_ONLY(3)
///   CRUISE(6) -> DISENGAGED(5)           SYNCHRONOUS(7) -> AUTONOMOUS(1)
///   other -> NOT_READY(6)
///
/// CRUISE is MORAI driving by itself (a second control source); reporting it as an
/// autonomous mode would hide that from the operation-mode transition manager.
/// SYNCHRONOUS still means Autoware drives (lockstep); leaving it unmapped made the
/// transition manager cancel engagement.
uint8_t to_control_mode_report(morai_grpc_vendor::ControlMode mode);
// uint8_t to_control_mode_report(morai_sim_api::actor::VehicleControlMode mode);

// 26.09.25 _ YSH _ 신규 — main.py publish_kinematic_state 이식 (header.stamp 는 호출자)
/// Ego -> /localization/kinematic_state. frame map, child base_link; twist in base_link
/// (linear km/h -> m/s with the lateral flip, angular = body angular velocity); diagonal
/// covariances 1e-4 (position/linear) and 1e-6 (orientation/angular) for pose and twist.
nav_msgs::msg::Odometry to_odometry(
  const morai_grpc_vendor::EgoSample & ego, const MapOrigin & origin);

// 26.09.25 _ YSH _ 신규 — map -> base_link TF (kinematic_state 와 같은 pose·stamp)
geometry_msgs::msg::TransformStamped to_map_to_base_link(
  const geometry_msgs::msg::Pose & pose, const builtin_interfaces::msg::Time & stamp);

// 26.09.25 _ YSH _ 신규 — main.py _object_id_from_name 이식: uuid5(NAMESPACE_OID, "morai:" + name).bytes
/// Stable per-actor object id. Byte-identical to Python uuid.uuid5 (and to daq_publisher.py),
/// so prediction keeps its tracks and logs from both bridges line up.
unique_identifier_msgs::msg::UUID object_id_from_name(const std::string & name);

// 26.09.25 _ YSH _ 신규 — main.py label_map (VEHICLE->CAR, PEDESTRIAN->PEDESTRIAN, 그 밖 UNKNOWN)
uint8_t to_object_label(morai_grpc_vendor::ObjectType type);

// 26.09.25 _ YSH _ 신규 — main.py publish_tracked_objects 의 객체 한 개 이식
/// MORAI actor -> TrackedObject. Pose uses the same rigid map transform as the ego;
/// twist is local (km/h -> m/s, lateral flip) with body angular velocity; bounding box
/// extent is a HALF size with x = half width, y = half length, so dimensions are
/// (2*y, 2*x, 2*z) floored at 0.1 m. Covariances are left zero (as main.py).
autoware_perception_msgs::msg::TrackedObject to_tracked_object(
  const morai_grpc_vendor::ActorSample & actor, const MapOrigin & origin);

// 26.09.25 _ YSH _ 신규 — epoch ns -> builtin_interfaces/Time (음수는 0 으로)
builtin_interfaces::msg::Time to_time_msg(int64_t epoch_ns);

// 26.09.26 _ YSH _ 신규 — DAQ(ViLS 명세 25 항목) 변환. daq_publisher.py _tick·_publish_objects 와 같은 식·같은
//   연산 순서라 값이 비트 단위로 같다. 기존 kinematic_state·tracking/objects 는 main.py 대로 두고 DAQ 만 더 채운다
/// /daq/vehicle_state (명세 1~6, 10~12): to_odometry 에 twist.linear.z = vz 를 더한 것 (kinematic_state 는 main.py
/// 대로 0). frame map, child base_link, 공분산은 to_odometry 와 같다. header.stamp 는 호출자
nav_msgs::msg::Odometry to_daq_vehicle_state(
  const morai_grpc_vendor::EgoSample & ego, const MapOrigin & origin);

/// /daq/acceleration (명세 7~9): MORAI acceleration(local) → (x, −y, z) × accel_scale, frame base_link,
/// 공분산 대각 1e-4 (선형만). header.stamp 는 호출자
geometry_msgs::msg::AccelWithCovarianceStamped to_daq_acceleration(
  const morai_grpc_vendor::EgoSample & ego, double accel_scale);

/// /daq/track_objects 한 개 (명세 13~24): to_tracked_object 에 가속도 (x, −y, z) × accel_scale 을 더한 것
/// (tracking/objects 는 main.py 대로 가속도를 비워 둔다)
autoware_perception_msgs::msg::TrackedObject to_daq_tracked_object(
  const morai_grpc_vendor::ActorSample & actor, const MapOrigin & origin, double accel_scale);

// 26.10.05 _ YSH _ 신규 — DAQ 명세 토픽을 MORAI 공식 메시지(morai_ros2_msgs 26.R1)로. 값은 gRPC 스냅샷을 MORAI 정의 단위로만
//   바꿔 담는다 (map_origin·축 반전 없음 — MORAI world·MORAI 축). 위 to_daq_* 셋은 시험·옛 bag 대조용으로 남긴다
/// /daq/ego_status (명세 1~9, 12): position = location [m], velocity = velocity(local) km/h ÷ 3.6 [m/s],
/// acceleration = acceleration(local) × accel_scale [m/s²], angular_velocity = 원값(world, 축 (−z, x, −y)) [deg/s],
/// heading = rotation.z [deg], accel·brake = vehicle_state throttle·brake, front_steer_angle = −front_wheel_angle ×
/// steer_input_norm_deg [deg, 좌 +]. gRPC 에 없는 칸(rear_steer·타이어·슬립·차선 거리)은 0. unique_id 0, frame map.
/// header.stamp 는 호출자. 명세 10~11(roll·pitch)은 칸이 없어 <prefix>/raw/actor_states 에서 읽는다
morai_ros2_msgs::msg::EgoVehicleStatus to_daq_ego_status(
  const morai_grpc_vendor::EgoSample & ego, double accel_scale, double steer_input_norm_deg);

/// uuid5("morai:"+이름) 앞 4바이트를 big-endian 으로 읽은 비트를 int32 로 (CSV objN_id_u32 와 같은 비트)
int32_t object_unique_id_from_name(const std::string & name);

/// ObjectStatus.type (MORAI 정의 0 보행자 · 1 NPC · 2 정적 장애물)
int32_t to_morai_object_status_type(morai_grpc_vendor::ObjectType type);

/// /daq/object_status 한 개 (명세 13~24): unique_id = object_unique_id_from_name, name = gRPC 이름, heading = rotation.z [deg],
/// velocity = global_velocity 원값(world) [km/h], acceleration = acceleration(local) × accel_scale [m/s²],
/// size = (2·extent.x 폭, 2·extent.y 길이, 2·extent.z 높이) [m] (하한 없음), position = location [m]
morai_ros2_msgs::msg::ObjectStatus to_daq_object_status(
  const morai_grpc_vendor::ActorSample & actor, double accel_scale);

// 26.11.12 _ YSH _ 신규 — /daq/calc/ego_velocity_world · ego_accel_world (KATECH 요청: 계산 값도 bag 토픽으로).
///   같은 틱의 /daq/ego_status 한 개에서: 차체 속도 (vx, vy) = (velocity.x, velocity.z) [MORAI 축 앞·위·왼],
///   차체 가속도 (ax, ay) = (acceleration.x, −acceleration.y) [MORAI 축 앞·오른·위] 를 자차 heading 으로 돌린다.
///   x = c·bx − s·by, y = s·bx + c·by (daq_all.csv vx_world·vy_world·ax_world·ay_world 와 같은 식).
///   z 는 돌리지 않은 위 성분 (velocity.y · acceleration.z). header 는 ego 것 그대로 (frame map)
void to_daq_ego_world(
  const morai_ros2_msgs::msg::EgoVehicleStatus & ego, geometry_msgs::msg::Vector3Stamped & velocity_world,
  geometry_msgs::msg::Vector3Stamped & accel_world);

/// actor 들을 type 별 목록(npc·pedestrian·obstacle)으로 나누고 num_of_* 를 채운다. frame map, header.stamp 는 호출자
// 26.10.06 _ YSH _ 신규 — /daq/ctrl_cmd (명세 25): MORAI 에 gRPC ControlVehicle 로 보낸 VehicleCtrlCmd 를 그대로.
/// longl_cmd_type = long_cmd_type (1 throttle · 2 speed · 3 acceleration), accel·brake = throttle·brake (0~1),
/// velocity = 보낸 velocity [km/h], acceleration = 보낸 acceleration [m/s²] — 전부 gRPC 값 그대로.
/// front_steer 만 MORAI 정의 단위(rad, 좌 +)라 보낸 steer(정규화 −1~1, 좌 +) × steer_input_norm_deg 를 rad 로. header.stamp 는 호출자
morai_ros2_msgs::msg::CtrlCmd to_daq_ctrl_cmd(
  const morai_grpc_vendor::VehicleCommand & cmd, double steer_input_norm_deg);

morai_ros2_msgs::msg::ObjectStatusList to_daq_object_status_list(
  const std::vector<morai_grpc_vendor::ActorSample> & actors, double accel_scale);

// 26.10.07 _ YSH _ 신규 — /daq/collision: gRPC Ego vehicle_state.collision_objects(지금 Ego 와 닿은 상대의 이름, 순서 그대로)
///   → CollisionData. 이름이 actor 목록에 있으면 그 actor 의 ObjectStatus (to_daq_object_status 와 같은 값), 없으면(actor 가
///   아닌 건물·가드레일 등) name·unique_id 만 채우고 type = -1, 나머지 0. global_offset 은 0 (MORAI world 그대로 —
///   map_origin 을 쓰지 않는다). 충돌이 없으면 collision_object 가 빈 목록. frame map, header.stamp 는 호출자
morai_ros2_msgs::msg::CollisionData to_daq_collision_data(
  const morai_grpc_vendor::EgoSample & ego, const std::vector<morai_grpc_vendor::ActorSample> & actors,
  double accel_scale);

// 26.10.12 _ YSH _ 신규 — 의사 VILS(follow): DAQ Ego 를 실차 값으로. MORAI Ego 는 순간이동만 해서 속도·페달·조향이
//   의미가 없다 (10-12 벤치: 보고 속도 약 0). 필드 규약은 to_daq_ego_status 와 같다 (MORAI world·MORAI 축) — CSV 변환기가
//   GT 런과 같은 식으로 읽게
struct RealCarEgoSample
{
  geometry_msgs::msg::Pose map_pose;  ///< Autoware map (뒤차축) — /localization/kinematic_state
  double speed_mps{0.0};              ///< 차체 앞 + (CAN 휠속도)
  double lateral_mps{0.0};            ///< 차체 왼 +
  double yaw_rate_rps{0.0};           ///< 반시계 + (K-CAN YAW_RATE)
  double accel_long_mps2{0.0};        ///< 앞 +
  double accel_lat_mps2{0.0};         ///< 왼 +
  double tire_angle_rad{0.0};         ///< 왼 + (steering_status)
  double throttle{0.0};               ///< 0~1 (K-CAN 가속페달)
  double brake{0.0};                  ///< 0~1 (K-CAN 브레이크 압력 정규화)
};
/// position·heading = from_map_pose(map_pose, origin) (MORAI world, z 는 map z), velocity = (speed, 0, lateral)
/// (MORAI local 축 (앞, 위, 왼)), acceleration = (long, −lat, 0) (MORAI 축 (앞, 오른, 위)), angular_velocity =
/// (0, −yaw_rate, 0) deg/s (MORAI world 축 — −y 가 요레이트), accel·brake = throttle·brake, front_steer_angle =
/// tire_angle [deg, 좌 +]. header.stamp 는 호출자
morai_ros2_msgs::msg::EgoVehicleStatus to_daq_ego_status_real(
  const RealCarEgoSample & sample, const MapOrigin & origin);

/// 26.10.12 _ YSH _ 신규 — 의사 VILS(follow): /daq/ctrl_cmd 를 Autoware 가 실차에 보낸 명령으로 (MORAI 에는 명령이 안 간다).
/// longl_cmd_type 3 (acceleration), acceleration = longitudinal.acceleration [m/s²], velocity = longitudinal.velocity × 3.6
/// [km/h], front_steer = steering_tire_angle [rad, 좌 +], accel·brake = 0 (페달 명령이 아니다). header.stamp = 명령 stamp
morai_ros2_msgs::msg::CtrlCmd to_daq_ctrl_cmd_from_control(const autoware_control_msgs::msg::Control & control);

}  // namespace autoware::morai_interface::morai_to_autoware

#endif  // AUTOWARE__MORAI_INTERFACE__CONVERSION__MORAI_TO_AUTOWARE_HPP_
