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

// SIM -> ADS 변환 구현 — 단위 환산, 좌표계 부호 반전, enum 매핑
// 26.09.25 _ YSH _ main.py(publish_* / get_map_pose / morai_coordinates.py) 식을 그대로 옮겼다.
//   골든 값은 test/test_morai_to_autoware.cpp 가 Python 실행 결과로 고정한다.

#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"  // 26.10.12 _ YSH _ from_map_pose (VILS DAQ)

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>

#include <boost/uuid/name_generator_sha1.hpp>
#include <boost/uuid/uuid.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>  // 26.10.05 _ YSH _ object_unique_id_from_name
#include <string>
#include <utility>

namespace autoware::morai_interface::morai_to_autoware
{

namespace
{
/// Python math.pi (Py_MATH_PI) 와 같은 값
constexpr double kPi = 3.14159265358979323846;

/// uuid.NAMESPACE_OID (6ba7b812-9dad-11d1-80b4-00c04fd430c8)
constexpr boost::uuids::uuid kNamespaceOid = {{0x6b, 0xa7, 0xb8, 0x12, 0x9d, 0xad, 0x11, 0xd1, 0x80,
                                               0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8}};

/// main.py 가 kinematic_state 공분산에 넣는 대각값 (GT 라 0 이지만 0 은 하위 모듈 나눗셈 문제)
constexpr double kPositionVariance = 1e-4;
constexpr double kOrientationVariance = 1e-6;
}  // namespace

double max_steering_angle_rad(double max_steering_angle_deg)
{
  return max_steering_angle_deg / (180.0 / kPi);
}

double radians(double deg)
{
  return deg * kDegToRad;
}

std::array<double, 3> angular_velocity_in_body(
  const morai_grpc_vendor::Vec3 & angular_velocity_raw,
  const morai_grpc_vendor::Vec3 & rotation_deg)
{
  // morai_coordinates.py 와 한 줄씩 같다
  const double wx = -angular_velocity_raw.z;
  const double wy = angular_velocity_raw.x;
  const double wz = -angular_velocity_raw.y;

  const double roll = radians(rotation_deg.x);
  const double pitch = radians(rotation_deg.y);
  const double yaw = radians(rotation_deg.z);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);

  // R = Rz(yaw) Ry(pitch) Rx(roll); body = R^T * world
  return {
    cy * cp * wx + sy * cp * wy - sp * wz,
    (cy * sp * sr - sy * cr) * wx + (sy * sp * sr + cy * cr) * wy + cp * sr * wz,
    (cy * sp * cr + sy * sr) * wx + (sy * sp * cr - cy * sr) * wy + cp * cr * wz,
  };
}

std::array<double, 4> quaternion_from_rpy(double roll, double pitch, double yaw)
{
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  return {
    sr * cp * cy - cr * sp * sy,
    cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy,
    cr * cp * cy + sr * sp * sy,
  };
}

geometry_msgs::msg::Pose to_map_pose(
  const morai_grpc_vendor::Vec3 & location, const morai_grpc_vendor::Vec3 & rotation_deg,
  const MapOrigin & origin)
{
  const double yaw_offset = radians(origin.yaw_deg);
  const double cos_o = std::cos(yaw_offset);
  const double sin_o = std::sin(yaw_offset);
  const double mx = location.x;
  const double my = location.y;

  geometry_msgs::msg::Pose pose;
  pose.position.x = cos_o * mx - sin_o * my + origin.x;
  pose.position.y = sin_o * mx + cos_o * my + origin.y;
  pose.position.z = location.z + origin.z;

  // rotation 은 도 단위, CCW 동쪽 0 → ROS yaw 와 같은 규약 (부호 반전 없음)
  const auto q = quaternion_from_rpy(
    radians(rotation_deg.x), radians(rotation_deg.y), radians(rotation_deg.z) + yaw_offset);
  pose.orientation.x = q[0];
  pose.orientation.y = q[1];
  pose.orientation.z = q[2];
  pose.orientation.w = q[3];
  return pose;
}

// 26.09.25 _ YSH _ EgoSample 입력, heading_rate 는 body 각속도 z (main.py publish_velocity_report)
autoware_vehicle_msgs::msg::VelocityReport to_velocity_report(
  const morai_grpc_vendor::EgoSample & ego)
{
  autoware_vehicle_msgs::msg::VelocityReport report;
  report.header.frame_id = "base_link";

  report.longitudinal_velocity = static_cast<float>(ego.velocity_kmh.x * kKmphToMps);
  // 26.10.06 _ YSH _ 횡 = velocity.z (D1). 원문:
  // report.lateral_velocity =
  //   static_cast<float>(kMoraiToRosLateralSign * ego.velocity_kmh.y * kKmphToMps);
  report.lateral_velocity = static_cast<float>(lateral_velocity_mps(ego.velocity_kmh));
  report.heading_rate =
    static_cast<float>(angular_velocity_in_body(ego.angular_velocity_raw, ego.rotation_deg)[2]);
  return report;
}
// autoware_vehicle_msgs::msg::VelocityReport to_velocity_report(
//   const morai_sim_api::actor::ActorState & state)
// {
//   autoware_vehicle_msgs::msg::VelocityReport report;
//   report.header.frame_id = "base_link";
//
//   // Longitudinal is along the vehicle axis, so only the unit changes.
//   report.longitudinal_velocity = static_cast<float>(state.velocity().x() * kKmphToMps);
//
//   // Lateral and yaw rate additionally flip sign (Unity left-handed -> ROS right-handed).
//   // Autoware spec: lateral_velocity positive = left, heading_rate positive = CCW.
//   report.lateral_velocity =
//     static_cast<float>(kMoraiToRosLateralSign * state.velocity().y() * kKmphToMps);
//   report.heading_rate =
//     static_cast<float>(kMoraiToRosLateralSign * state.angular_velocity().z() * kDegToRad);
//
//   return report;
// }

// 26.09.25 _ YSH _ × steer_comp_gain 역보정 추가, 입력을 원값 double 로 (main.py publish_steering_report)
autoware_vehicle_msgs::msg::SteeringReport to_steering_report(
  double front_wheel_angle, const EgoVehicleSpec & spec, double steer_comp_gain)
{
  autoware_vehicle_msgs::msg::SteeringReport report;
  if (spec.max_steering_angle <= 0.0) {
    report.steering_tire_angle = 0.0F;
    return report;
  }
  // main.py: -fwa * self.max_steering_angle * self._steer_comp_gain (같은 곱셈 순서)
  report.steering_tire_angle = static_cast<float>(
    kMoraiToRosLateralSign * front_wheel_angle * spec.max_steering_angle * steer_comp_gain);
  return report;
}
// autoware_vehicle_msgs::msg::SteeringReport to_steering_report(
//   const morai_sim_api::actor::VehicleState & state, const EgoVehicleSpec & spec)
// {
//   autoware_vehicle_msgs::msg::SteeringReport report;
//
//   // Defensive: the caller is not supposed to publish before the spec is fetched, but a
//   // zero here would silently scale the whole report to zero rather than fail loudly.
//   if (spec.max_steering_angle <= 0.0) {
//     report.steering_tire_angle = 0.0F;
//     return report;
//   }
//
//   report.steering_tire_angle = static_cast<float>(
//     kMoraiToRosLateralSign * static_cast<double>(state.front_wheel_angle()) *
//     spec.max_steering_angle);
//   return report;
// }

// 26.09.25 _ YSH _ MoraiLink 열거형 사본으로 (매핑은 그대로, main.py publish_gear_report 와 같음)
uint8_t to_gear_report(morai_grpc_vendor::GearMode gear)
{
  using GearReport = autoware_vehicle_msgs::msg::GearReport;
  using morai_grpc_vendor::GearMode;

  // A default case is mandatory rather than exhaustive enumeration: the MORAI enum is
  // open, and a MORAI update adding a value must not produce an out-of-range constant.
  switch (gear) {
    case GearMode::kP:
      return GearReport::PARK;
    case GearMode::kR:
      return GearReport::REVERSE;
    case GearMode::kN:
      return GearReport::NEUTRAL;
    case GearMode::kD:
      return GearReport::DRIVE;
    case GearMode::kL:
      return GearReport::LOW;
    case GearMode::kM:
    default:
      // GEAR_MODE_M ("manual") has no Autoware counterpart.
      return GearReport::NONE;
  }
}
// uint8_t to_gear_report(morai_sim_api::actor::GearMode gear)
// {
//   using GearReport = autoware_vehicle_msgs::msg::GearReport;
//   switch (gear) {
//     case morai_sim_api::actor::GEAR_MODE_P:
//       return GearReport::PARK;
//     case morai_sim_api::actor::GEAR_MODE_R:
//       return GearReport::REVERSE;
//     case morai_sim_api::actor::GEAR_MODE_N:
//       return GearReport::NEUTRAL;
//     case morai_sim_api::actor::GEAR_MODE_D:
//       return GearReport::DRIVE;
//     case morai_sim_api::actor::GEAR_MODE_L:
//       return GearReport::LOW;
//     case morai_sim_api::actor::GEAR_MODE_M:
//     default:
//       return GearReport::NONE;
//   }
// }

// 26.09.25 _ YSH _ CRUISE -> DISENGAGED(5) (main.py 대로, 호출자가 WARN), SYNCHRONOUS -> AUTONOMOUS 유지
uint8_t to_control_mode_report(morai_grpc_vendor::ControlMode mode)
{
  using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
  using morai_grpc_vendor::ControlMode;

  switch (mode) {
    case ControlMode::kUnspecified:
      return ControlModeReport::NO_COMMAND;
    case ControlMode::kKeyboard:
    case ControlMode::kGameWheel:
      return ControlModeReport::MANUAL;
    case ControlMode::kAuto:
      return ControlModeReport::AUTONOMOUS;
    case ControlMode::kAutoLateral:
      return ControlModeReport::AUTONOMOUS_STEER_ONLY;
    case ControlMode::kAutoLongitudinal:
      return ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
    case ControlMode::kCruise:
      // MORAI 가 스스로 운전하는 상태 — Autoware 제어로 보고하면 제2 제어원이 가려진다
      return ControlModeReport::DISENGAGED;
    case ControlMode::kSynchronous:
      // lockstep 에서도 운전은 Autoware 가 한다
      return ControlModeReport::AUTONOMOUS;
    default:
      return ControlModeReport::NOT_READY;
  }
}
// uint8_t to_control_mode_report(morai_sim_api::actor::VehicleControlMode mode)
// {
//   using ControlModeReport = autoware_vehicle_msgs::msg::ControlModeReport;
//
//   switch (mode) {
//     case morai_sim_api::actor::VEHICLE_CONTROL_KEYBOARD:
//     case morai_sim_api::actor::VEHICLE_CONTROL_GAME_WHEEL:
//       return ControlModeReport::MANUAL;
//     case morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE:
//       return ControlModeReport::AUTONOMOUS;
//     case morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LATERAL:
//       return ControlModeReport::AUTONOMOUS_STEER_ONLY;
//     case morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LONGITUDINAL:
//       return ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
//     case morai_sim_api::actor::VEHICLE_CONTROL_CRUISE_MODE:
//       // Cruise control is longitudinal-only autonomy. The Python bridge emitted 5 here,
//       // which is DISENGAGED, contradicting its own comment.
//       return ControlModeReport::AUTONOMOUS_VELOCITY_ONLY;
//     case morai_sim_api::actor::VEHICLE_CONTROL_SYNCHRONOUS_MODE:
//       // Sync mode still means Autoware drives; leaving it unmapped made the transition
//       // manager believe Autoware was not in control and cancel engagement.
//       return ControlModeReport::AUTONOMOUS;
//     case morai_sim_api::actor::VEHICLE_CONTROL_UNSPECIFIED:
//       return ControlModeReport::NO_COMMAND;
//     default:
//       return ControlModeReport::NOT_READY;
//   }
// }

nav_msgs::msg::Odometry to_odometry(
  const morai_grpc_vendor::EgoSample & ego, const MapOrigin & origin)
{
  nav_msgs::msg::Odometry msg;
  msg.header.frame_id = "map";
  msg.child_frame_id = "base_link";
  msg.pose.pose = to_map_pose(ego.position, ego.rotation_deg, origin);

  // twist 는 child_frame_id(base_link) 기준, VelocityReport 와 같은 변환 (linear.z 는 main.py 대로 0)
  msg.twist.twist.linear.x = ego.velocity_kmh.x * kKmphToMps;
  // 26.10.06 _ YSH _ 횡 = velocity.z (D1). 원문: msg.twist.twist.linear.y = kMoraiToRosLateralSign * ego.velocity_kmh.y * kKmphToMps;
  msg.twist.twist.linear.y = lateral_velocity_mps(ego.velocity_kmh);
  const auto w = angular_velocity_in_body(ego.angular_velocity_raw, ego.rotation_deg);
  msg.twist.twist.angular.x = w[0];
  msg.twist.twist.angular.y = w[1];
  msg.twist.twist.angular.z = w[2];

  for (const size_t i : {0U, 7U, 14U}) {
    msg.pose.covariance[i] = kPositionVariance;
    msg.twist.covariance[i] = kPositionVariance;
  }
  for (const size_t i : {21U, 28U, 35U}) {
    msg.pose.covariance[i] = kOrientationVariance;
    msg.twist.covariance[i] = kOrientationVariance;
  }
  return msg;
}

geometry_msgs::msg::TransformStamped to_map_to_base_link(
  const geometry_msgs::msg::Pose & pose, const builtin_interfaces::msg::Time & stamp)
{
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "map";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = pose.position.x;
  tf.transform.translation.y = pose.position.y;
  tf.transform.translation.z = pose.position.z;
  tf.transform.rotation = pose.orientation;
  return tf;
}

unique_identifier_msgs::msg::UUID object_id_from_name(const std::string & name)
{
  // Python str.encode('utf-8') 바이트 그대로 해시한다 (protobuf string 도 UTF-8)
  const std::string key = "morai:" + name;
  const boost::uuids::name_generator_sha1 gen(kNamespaceOid);
  const boost::uuids::uuid id = gen(key.data(), key.size());

  unique_identifier_msgs::msg::UUID msg;
  std::copy(id.begin(), id.end(), msg.uuid.begin());
  return msg;
}

uint8_t to_object_label(morai_grpc_vendor::ObjectType type)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  switch (type) {
    case morai_grpc_vendor::ObjectType::kVehicle:
      return ObjectClassification::CAR;
    case morai_grpc_vendor::ObjectType::kPedestrian:
      return ObjectClassification::PEDESTRIAN;
    case morai_grpc_vendor::ObjectType::kObstacle:
    default:
      return ObjectClassification::UNKNOWN;
  }
}

autoware_perception_msgs::msg::TrackedObject to_tracked_object(
  const morai_grpc_vendor::ActorSample & actor, const MapOrigin & origin)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_perception_msgs::msg::Shape;
  using autoware_perception_msgs::msg::TrackedObjectKinematics;

  autoware_perception_msgs::msg::TrackedObject obj;
  obj.object_id = object_id_from_name(actor.id);
  obj.existence_probability = 1.0F;  // GT 라 확실

  ObjectClassification cls;
  cls.label = to_object_label(actor.object_type);
  cls.probability = 1.0F;
  obj.classification.push_back(cls);

  // 자세: ego 와 완전히 같은 리지드 변환이어야 map 에서 정합된다
  // 26.10.06 _ YSH _ Autoware 객체 pose 는 상자 중심이어야 한다 (D2). MORAI transform.location 은 차량이면 뒤차축이라
  //   상자가 1.3~1.5 m 뒤로 그려졌다. bounding_box.location 은 world 좌표의 상자 중심 — 없으면(0,0,0) 예전대로 location.
  //   원문: obj.kinematics.pose_with_covariance.pose = to_map_pose(actor.position, actor.rotation_deg, origin);
  const bool has_bbox_center =
    actor.bbox_location.x != 0.0 || actor.bbox_location.y != 0.0 || actor.bbox_location.z != 0.0;
  obj.kinematics.pose_with_covariance.pose =
    to_map_pose(has_bbox_center ? actor.bbox_location : actor.position, actor.rotation_deg, origin);
  obj.kinematics.orientation_availability = TrackedObjectKinematics::AVAILABLE;

  auto & twist = obj.kinematics.twist_with_covariance.twist;
  twist.linear.x = actor.velocity_kmh.x * kKmphToMps;
  // 26.10.06 _ YSH _ 횡 = velocity.z (D1). 원문: twist.linear.y = kMoraiToRosLateralSign * actor.velocity_kmh.y * kKmphToMps;
  twist.linear.y = lateral_velocity_mps(actor.velocity_kmh);
  const auto w = angular_velocity_in_body(actor.angular_velocity_raw, actor.rotation_deg);
  twist.angular.x = w[0];
  twist.angular.y = w[1];
  twist.angular.z = w[2];
  obj.kinematics.is_stationary = false;

  // extent 는 반치수 x=반폭 y=반길이 → Autoware dimensions x=길이 y=폭
  obj.shape.type = Shape::BOUNDING_BOX;
  obj.shape.dimensions.x = std::max(actor.bbox_extent.y * 2.0, 0.1);
  obj.shape.dimensions.y = std::max(actor.bbox_extent.x * 2.0, 0.1);
  obj.shape.dimensions.z = std::max(actor.bbox_extent.z * 2.0, 0.1);
  return obj;
}

// 26.09.26 _ YSH _ 신규 — DAQ 변환 (daq_publisher.py:419-453, 521-566 과 같은 식·순서)
nav_msgs::msg::Odometry to_daq_vehicle_state(
  const morai_grpc_vendor::EgoSample & ego, const MapOrigin & origin)
{
  nav_msgs::msg::Odometry msg = to_odometry(ego, origin);
  // daq_publisher: vz = float(ego.velocity.z) * KMPH_TO_MPS (kinematic_state 는 main.py 대로 0)
  // 26.10.06 _ YSH _ 수직 = velocity.y (D1). 원문: msg.twist.twist.linear.z = ego.velocity_kmh.z * kKmphToMps;
  msg.twist.twist.linear.z = vertical_velocity_mps(ego.velocity_kmh);
  return msg;
}

geometry_msgs::msg::AccelWithCovarianceStamped to_daq_acceleration(
  const morai_grpc_vendor::EgoSample & ego, double accel_scale)
{
  geometry_msgs::msg::AccelWithCovarianceStamped msg;
  msg.header.frame_id = "base_link";  // MORAI acceleration 은 local(ego frame)
  // daq_publisher: ax = a.x * k, ay = -a.y * k, az = a.z * k (같은 연산 순서)
  msg.accel.accel.linear.x = ego.acceleration_raw.x * accel_scale;
  msg.accel.accel.linear.y = -ego.acceleration_raw.y * accel_scale;
  msg.accel.accel.linear.z = ego.acceleration_raw.z * accel_scale;
  for (const size_t i : {0U, 7U, 14U}) {
    msg.accel.covariance[i] = kPositionVariance;
  }
  return msg;
}

autoware_perception_msgs::msg::TrackedObject to_daq_tracked_object(
  const morai_grpc_vendor::ActorSample & actor, const MapOrigin & origin, double accel_scale)
{
  autoware_perception_msgs::msg::TrackedObject obj = to_tracked_object(actor, origin);
  auto & ac = obj.kinematics.acceleration_with_covariance.accel;
  ac.linear.x = actor.acceleration_raw.x * accel_scale;
  ac.linear.y = -actor.acceleration_raw.y * accel_scale;
  ac.linear.z = actor.acceleration_raw.z * accel_scale;
  return obj;
}

// 26.10.05 _ YSH _ 신규 — DAQ 명세 토픽 MORAI 메시지 변환 (헤더 설명 참고)
morai_ros2_msgs::msg::EgoVehicleStatus to_daq_ego_status(
  const morai_grpc_vendor::EgoSample & ego, double accel_scale, double steer_input_norm_deg)
{
  constexpr double kRadToDeg = 180.0 / kPi;
  morai_ros2_msgs::msg::EgoVehicleStatus msg;
  msg.header.frame_id = "map";
  msg.unique_id = 0;
  msg.position.x = ego.position.x;
  msg.position.y = ego.position.y;
  msg.position.z = ego.position.z;
  // daq_publisher 와 같은 곱 (km/h × KMPH_TO_MPS) — 옛 /daq/vehicle_state 와 비트가 같다 (y 는 MORAI 부호 그대로)
  msg.velocity.x = ego.velocity_kmh.x * kKmphToMps;
  msg.velocity.y = ego.velocity_kmh.y * kKmphToMps;
  msg.velocity.z = ego.velocity_kmh.z * kKmphToMps;
  msg.acceleration.x = ego.acceleration_raw.x * accel_scale;
  msg.acceleration.y = ego.acceleration_raw.y * accel_scale;
  msg.acceleration.z = ego.acceleration_raw.z * accel_scale;
  msg.angular_velocity.x = ego.angular_velocity_raw.x * kRadToDeg;
  msg.angular_velocity.y = ego.angular_velocity_raw.y * kRadToDeg;
  msg.angular_velocity.z = ego.angular_velocity_raw.z * kRadToDeg;
  msg.heading = ego.rotation_deg.z;
  msg.accel = ego.vehicle_state.throttle;
  msg.brake = ego.vehicle_state.brake;
  msg.front_steer_angle =
    kMoraiToRosLateralSign * ego.vehicle_state.front_wheel_angle * steer_input_norm_deg;
  return msg;
}

int32_t object_unique_id_from_name(const std::string & name)
{
  const auto id = object_id_from_name(name);
  const uint32_t u = (static_cast<uint32_t>(id.uuid[0]) << 24) |
                     (static_cast<uint32_t>(id.uuid[1]) << 16) |
                     (static_cast<uint32_t>(id.uuid[2]) << 8) | static_cast<uint32_t>(id.uuid[3]);
  int32_t out = 0;
  std::memcpy(&out, &u, sizeof(out));  // 비트 그대로 (2의 보수)
  return out;
}

int32_t to_morai_object_status_type(morai_grpc_vendor::ObjectType type)
{
  switch (type) {
    case morai_grpc_vendor::ObjectType::kVehicle:
      return 1;
    case morai_grpc_vendor::ObjectType::kPedestrian:
      return 0;
    case morai_grpc_vendor::ObjectType::kObstacle:
    default:
      return 2;
  }
}

morai_ros2_msgs::msg::ObjectStatus to_daq_object_status(
  const morai_grpc_vendor::ActorSample & actor, double accel_scale)
{
  morai_ros2_msgs::msg::ObjectStatus obj;
  obj.unique_id = object_unique_id_from_name(actor.id);
  obj.type = to_morai_object_status_type(actor.object_type);
  obj.name = actor.id;
  obj.heading = actor.rotation_deg.z;
  obj.velocity.x = actor.global_velocity_raw.x;
  obj.velocity.y = actor.global_velocity_raw.y;
  obj.velocity.z = actor.global_velocity_raw.z;
  obj.acceleration.x = actor.acceleration_raw.x * accel_scale;
  obj.acceleration.y = actor.acceleration_raw.y * accel_scale;
  obj.acceleration.z = actor.acceleration_raw.z * accel_scale;
  obj.size.x = actor.bbox_extent.x * 2.0;
  obj.size.y = actor.bbox_extent.y * 2.0;
  obj.size.z = actor.bbox_extent.z * 2.0;
  obj.position.x = actor.position.x;
  obj.position.y = actor.position.y;
  obj.position.z = actor.position.z;
  return obj;
}

// 26.11.12 _ YSH _ 신규 (헤더 설명 참고)
void to_daq_ego_world(
  const morai_ros2_msgs::msg::EgoVehicleStatus & ego, geometry_msgs::msg::Vector3Stamped & velocity_world,
  geometry_msgs::msg::Vector3Stamped & accel_world)
{
  const double yaw = radians(ego.heading);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double vx = ego.velocity.x;
  const double vy = ego.velocity.z;   // MORAI velocity 축 (앞, 위, 왼) — z 가 왼쪽
  const double ax = ego.acceleration.x;
  const double ay = -ego.acceleration.y;  // MORAI acceleration 축 (앞, 오른, 위) — 부호를 바꿔 왼쪽 +
  velocity_world.header = ego.header;
  velocity_world.header.frame_id = "map";
  velocity_world.vector.x = c * vx - s * vy;
  velocity_world.vector.y = s * vx + c * vy;
  velocity_world.vector.z = ego.velocity.y;
  accel_world.header = velocity_world.header;
  accel_world.vector.x = c * ax - s * ay;
  accel_world.vector.y = s * ax + c * ay;
  accel_world.vector.z = ego.acceleration.z;
}

morai_ros2_msgs::msg::ObjectStatusList to_daq_object_status_list(
  const std::vector<morai_grpc_vendor::ActorSample> & actors, double accel_scale)
{
  morai_ros2_msgs::msg::ObjectStatusList msg;
  msg.header.frame_id = "map";
  for (const auto & actor : actors) {
    auto obj = to_daq_object_status(actor, accel_scale);
    switch (obj.type) {
      case 1:
        msg.npc_list.push_back(std::move(obj));
        break;
      case 0:
        msg.pedestrian_list.push_back(std::move(obj));
        break;
      default:
        msg.obstacle_list.push_back(std::move(obj));
        break;
    }
  }
  msg.num_of_npcs = static_cast<int32_t>(msg.npc_list.size());
  msg.num_of_pedestrian = static_cast<int32_t>(msg.pedestrian_list.size());
  msg.num_of_obstacle = static_cast<int32_t>(msg.obstacle_list.size());
  return msg;
}

// 26.10.07 _ YSH _ 신규 — /daq/collision (헤더 설명 참고)
morai_ros2_msgs::msg::CollisionData to_daq_collision_data(
  const morai_grpc_vendor::EgoSample & ego, const std::vector<morai_grpc_vendor::ActorSample> & actors,
  double accel_scale)
{
  morai_ros2_msgs::msg::CollisionData msg;
  msg.header.frame_id = "map";
  msg.global_offset_x = 0.0;
  msg.global_offset_y = 0.0;
  msg.global_offset_z = 0.0;
  msg.collision_object.reserve(ego.vehicle_state.collision_objects.size());
  for (const auto & name : ego.vehicle_state.collision_objects) {
    const auto it = std::find_if(
      actors.begin(), actors.end(), [&name](const morai_grpc_vendor::ActorSample & a) { return a.id == name; });
    if (it != actors.end()) {
      msg.collision_object.push_back(to_daq_object_status(*it, accel_scale));
      continue;
    }
    morai_ros2_msgs::msg::ObjectStatus obj;  // actor 가 아닌 상대 — 이름만 안다
    obj.unique_id = object_unique_id_from_name(name);
    obj.type = -1;
    obj.name = name;
    msg.collision_object.push_back(std::move(obj));
  }
  return msg;
}

// 26.10.06 _ YSH _ 신규 — 명세 25 를 gRPC 송신값으로 (헤더 설명 참고)
morai_ros2_msgs::msg::CtrlCmd to_daq_ctrl_cmd(
  const morai_grpc_vendor::VehicleCommand & cmd, double steer_input_norm_deg)
{
  morai_ros2_msgs::msg::CtrlCmd m;
  m.longl_cmd_type = static_cast<int32_t>(cmd.long_cmd_type);
  m.accel = cmd.throttle;
  m.brake = cmd.brake;
  m.front_steer = cmd.steer * steer_input_norm_deg * kDegToRad;
  m.rear_steer = 0.0;
  m.velocity = cmd.velocity;
  m.acceleration = cmd.acceleration;
  return m;
}

// 26.10.12 _ YSH _ 신규 — 의사 VILS DAQ (헤더 설명 참고)
morai_ros2_msgs::msg::EgoVehicleStatus to_daq_ego_status_real(
  const RealCarEgoSample & sample, const MapOrigin & origin)
{
  constexpr double kRadToDeg = 180.0 / kPi;
  morai_ros2_msgs::msg::EgoVehicleStatus msg;
  msg.header.frame_id = "map";
  msg.unique_id = 0;
  const auto t = autoware_to_morai::from_map_pose(sample.map_pose, origin, std::nullopt);
  msg.position.x = t.position.x;
  msg.position.y = t.position.y;
  msg.position.z = sample.map_pose.position.z - origin.z;  // from_map_pose 와 같은 식 (map z − origin z)
  msg.velocity.x = sample.speed_mps;
  msg.velocity.y = 0.0;
  msg.velocity.z = sample.lateral_mps;
  msg.acceleration.x = sample.accel_long_mps2;
  msg.acceleration.y = -sample.accel_lat_mps2;
  msg.acceleration.z = 0.0;
  msg.angular_velocity.x = 0.0;
  msg.angular_velocity.y = -sample.yaw_rate_rps * kRadToDeg;
  msg.angular_velocity.z = 0.0;
  msg.heading = t.rotation_deg.z;
  msg.accel = std::clamp(sample.throttle, 0.0, 1.0);
  msg.brake = std::clamp(sample.brake, 0.0, 1.0);
  msg.front_steer_angle = sample.tire_angle_rad * kRadToDeg;
  return msg;
}

morai_ros2_msgs::msg::CtrlCmd to_daq_ctrl_cmd_from_control(const autoware_control_msgs::msg::Control & control)
{
  morai_ros2_msgs::msg::CtrlCmd m;
  m.header.stamp = control.stamp;
  m.longl_cmd_type = 3;  // LONG_CMD_TYPE_ACCELERATION
  m.accel = 0.0;
  m.brake = 0.0;
  m.front_steer = control.lateral.steering_tire_angle;
  m.rear_steer = 0.0;
  m.velocity = control.longitudinal.velocity * 3.6;
  m.acceleration = control.longitudinal.acceleration;
  return m;
}

builtin_interfaces::msg::Time to_time_msg(int64_t epoch_ns)
{
  builtin_interfaces::msg::Time t;
  if (epoch_ns <= 0) {
    return t;
  }
  t.sec = static_cast<int32_t>(epoch_ns / 1000000000LL);
  t.nanosec = static_cast<uint32_t>(epoch_ns % 1000000000LL);
  return t;
}

}  // namespace autoware::morai_interface::morai_to_autoware
