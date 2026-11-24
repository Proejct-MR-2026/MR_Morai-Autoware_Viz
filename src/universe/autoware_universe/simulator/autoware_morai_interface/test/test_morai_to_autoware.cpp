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

// SIM -> ADS 변환 단위 테스트 — 부호 반전과 enum 표를 빌드 단계에서 고정한다
// 26.09.25 _ YSH _ deg/s 부호 반전 heading_rate(틀림)를 고정하던 테스트를 body 각속도 골든으로 바꾸고,
//   CRUISE 매핑을 main.py(DISENGAGED)로 되돌렸다. map pose·uuid5·TrackedObject·Odometry 골든 추가.
//   골든 값: python3 — 각속도는 Morai_Autoware_Bridge/scripts/morai_coordinates.py 를 직접 import(math 만 씀),
//   map pose 는 main.py get_map_pose/_quaternion_from_rpy 식, uuid 는 uuid.uuid5(NAMESPACE_OID, ...).

#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

#include <autoware_perception_msgs/msg/object_classification.hpp>
#include <autoware_perception_msgs/msg/shape.hpp>
#include <autoware_perception_msgs/msg/tracked_object_kinematics.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <string>
#include <vector>  // 26.10.05 _ YSH _ DaqMorai

namespace autoware::morai_interface
{
namespace
{

using autoware_vehicle_msgs::msg::ControlModeReport;
using autoware_vehicle_msgs::msg::GearReport;
using morai_grpc_vendor::ControlMode;
using morai_grpc_vendor::GearMode;
using morai_grpc_vendor::Vec3;

EgoVehicleSpec make_spec(double max_steering_angle_rad)
{
  EgoVehicleSpec spec;
  spec.max_steering_angle = max_steering_angle_rad;
  spec.wheel_base = 2.7;
  return spec;
}

// ============================================================================
// Velocity: units and the Unity -> ROS frame sign flip
// ============================================================================

TEST(MoraiToAutoware, LongitudinalVelocityConvertsKmphToMps)
{
  morai_grpc_vendor::EgoSample ego;
  ego.velocity_kmh.x = 36.0;  // 36 km/h
  EXPECT_NEAR(morai_to_autoware::to_velocity_report(ego).longitudinal_velocity, 10.0, 1e-6);
}

TEST(MoraiToAutoware, LongitudinalVelocityKeepsItsSign)
{
  morai_grpc_vendor::EgoSample ego;
  ego.velocity_kmh.x = -10.0;
  EXPECT_LT(morai_to_autoware::to_velocity_report(ego).longitudinal_velocity, 0.0F);
}

// 26.10.06 _ YSH _ MORAI velocity 는 (앞, 위, 왼) — 횡속도는 +z (DAQ 감사 D1). 원래 시험(LateralVelocityFlipsSign)은
//   velocity.y = 3.6 → lateral −1.0 을 기대했다
TEST(MoraiToAutoware, LateralVelocityIsMoraiZLeftPositive)
{
  morai_grpc_vendor::EgoSample ego;
  ego.velocity_kmh.z = 3.6;  // MORAI +z = 왼
  EXPECT_NEAR(morai_to_autoware::to_velocity_report(ego).lateral_velocity, 1.0, 1e-6);
  morai_grpc_vendor::EgoSample up;
  up.velocity_kmh.y = 3.6;  // MORAI +y = 위 — 횡속도가 아니다
  EXPECT_NEAR(morai_to_autoware::to_velocity_report(up).lateral_velocity, 0.0, 1e-9);
  EXPECT_EQ(morai_to_autoware::to_velocity_report(ego).header.frame_id, "base_link");
}

// 26.09.25 _ YSH _ heading_rate 는 deg/s 부호 반전이 아니라 world rad/s 의 body 회전 (morai_coordinates.py)
// TEST(MoraiToAutoware, HeadingRateConvertsDegPerSecAndFlipsSign)
// {
//   morai_sim_api::actor::ActorState state;
//   state.mutable_angular_velocity()->set_z(-57.29577951308232);  // -1 rad/s expressed in deg/s
//
//   const auto report = morai_to_autoware::to_velocity_report(state);
//
//   // Autoware spec: heading_rate positive means counter-clockwise (left).
//   EXPECT_NEAR(report.heading_rate, 1.0, 1e-5);
// }
TEST(MoraiToAutoware, HeadingRateIsBodyZOfWorldAngularVelocity)
{
  // 평지(roll=pitch=0)면 body z = world z = -raw.y [rad/s]. raw.z 는 world x 다
  morai_grpc_vendor::EgoSample ego;
  ego.angular_velocity_raw = Vec3{0.0, -0.3, 57.0};
  ego.rotation_deg = Vec3{0.0, 0.0, 90.0};
  EXPECT_NEAR(morai_to_autoware::to_velocity_report(ego).heading_rate, 0.3, 1e-6);
}

// ============================================================================
// 26.09.25 _ YSH _ angular_velocity_in_body 골든 (morai_coordinates.py 실행 결과)
// ============================================================================

TEST(AngularVelocity, MatchesMoraiCoordinatesPyGolden)
{
  const struct
  {
    Vec3 raw;
    Vec3 rot;
    std::array<double, 3> body;
  } golden[] = {
    {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}},
    {{0.1, -0.5, 0.2}, {0.0, 0.0, 0.0}, {-0.2, 0.1, 0.5}},
    {{0.0, -0.3, 0.0}, {0.0, 0.0, 90.0}, {0.0, 0.0, 0.3}},
    {{0.02, -0.41, 0.03},
     {1.5, -2.0, 37.0},
     {0.002393291835844795, 0.04475240365351502, 0.40913505657841315}},
    {{-0.7, 0.9, 0.25},
     {-10.0, 5.0, -135.0},
     {0.7476353935555131, 0.4588859835340827, -0.7700421887267285}},
    {{0.3, 0.1, -0.2},
     {179.0, 45.0, 270.5},
     {-0.14017916041401707, -0.20749409339494573, 0.278021589434761}},
  };
  for (const auto & g : golden) {
    const auto b = morai_to_autoware::angular_velocity_in_body(g.raw, g.rot);
    for (size_t i = 0; i < 3; ++i) {
      EXPECT_NEAR(b[i], g.body[i], 1e-12) << "axis " << i;
    }
  }
}

// ============================================================================
// 26.09.25 _ YSH _ map pose 골든 (main.py get_map_pose)
// ============================================================================

TEST(MapPose, MatchesMainPyGolden)
{
  const struct
  {
    Vec3 loc;
    Vec3 rot;
    MapOrigin origin;
    double out[7];  // px py pz qx qy qz qw
  } golden[] = {
    {{10.0, 20.0, 0.5},
     {0.0, 0.0, 30.0},
     {0.0, 0.0, 0.0, 0.0},
     {10.0, 20.0, 0.5, 0.0, 0.0, 0.25881904510252074, 0.9659258262890683}},
    {{-1234.5, 987.25, 12.0},
     {1.0, -2.0, -170.0},
     {300000.0, 4100000.0, -3.0, 1.75},
     {298735.9266349457, 4100949.089742996, 9.0, -0.01646697883008008, -0.0104657121338405,
      -0.9945426089853399, 0.1024905185389111}},
    {{0.0, 0.0, 0.0},
     {0.0, 0.0, 0.0},
     {5.0, -7.0, 1.0, -90.0},
     {5.0, -7.0, 1.0, 0.0, 0.0, -0.7071067811865475, 0.7071067811865476}},
    {{55.5, -66.6, 2.2},
     {0.5, 0.25, 359.0},
     {-12.0, 34.0, 0.0, 12.345},
     {56.4556529219671, -19.194300287034288, 2.2, -0.004126294486312029, -0.0026022336802931545,
      -0.09883149716120598, -0.9950922254959889}},
  };
  for (const auto & g : golden) {
    const auto p = morai_to_autoware::to_map_pose(g.loc, g.rot, g.origin);
    EXPECT_NEAR(p.position.x, g.out[0], 1e-9);
    EXPECT_NEAR(p.position.y, g.out[1], 1e-9);
    EXPECT_NEAR(p.position.z, g.out[2], 1e-12);
    EXPECT_NEAR(p.orientation.x, g.out[3], 1e-12);
    EXPECT_NEAR(p.orientation.y, g.out[4], 1e-12);
    EXPECT_NEAR(p.orientation.z, g.out[5], 1e-12);
    EXPECT_NEAR(p.orientation.w, g.out[6], 1e-12);
  }
}

TEST(MapPose, OdometryAndTfShareThePose)
{
  morai_grpc_vendor::EgoSample ego;
  ego.position = Vec3{10.0, 20.0, 0.5};
  ego.rotation_deg = Vec3{0.0, 0.0, 30.0};
  ego.velocity_kmh = Vec3{36.0, 0.0, 3.6};  // 26.10.06 _ YSH _ 횡 = z (원문: Vec3{36.0, 3.6, 0.0} → y −1.0)
  ego.angular_velocity_raw = Vec3{0.0, -0.3, 0.0};
  const MapOrigin origin{};
  const auto odom = morai_to_autoware::to_odometry(ego, origin);
  EXPECT_EQ(odom.header.frame_id, "map");
  EXPECT_EQ(odom.child_frame_id, "base_link");
  EXPECT_NEAR(odom.twist.twist.linear.x, 10.0, 1e-12);
  EXPECT_NEAR(odom.twist.twist.linear.y, 1.0, 1e-12);  // 26.10.06 _ YSH _ 원문 −1.0
  EXPECT_DOUBLE_EQ(odom.twist.twist.linear.z, 0.0);  // main.py 는 z 를 채우지 않는다
  EXPECT_NEAR(odom.twist.twist.angular.z, 0.3, 1e-12);
  for (const size_t i : {0U, 7U, 14U}) {
    EXPECT_DOUBLE_EQ(odom.pose.covariance[i], 1e-4);
    EXPECT_DOUBLE_EQ(odom.twist.covariance[i], 1e-4);
  }
  for (const size_t i : {21U, 28U, 35U}) {
    EXPECT_DOUBLE_EQ(odom.pose.covariance[i], 1e-6);
    EXPECT_DOUBLE_EQ(odom.twist.covariance[i], 1e-6);
  }
  EXPECT_DOUBLE_EQ(odom.pose.covariance[1], 0.0);

  builtin_interfaces::msg::Time stamp;
  stamp.sec = 1758000000;
  stamp.nanosec = 123000;
  const auto tf = morai_to_autoware::to_map_to_base_link(odom.pose.pose, stamp);
  EXPECT_EQ(tf.header.frame_id, "map");
  EXPECT_EQ(tf.child_frame_id, "base_link");
  EXPECT_EQ(tf.header.stamp, stamp);
  EXPECT_DOUBLE_EQ(tf.transform.translation.x, odom.pose.pose.position.x);
  EXPECT_DOUBLE_EQ(tf.transform.translation.y, odom.pose.pose.position.y);
  EXPECT_DOUBLE_EQ(tf.transform.translation.z, odom.pose.pose.position.z);
  EXPECT_EQ(tf.transform.rotation, odom.pose.pose.orientation);
}

// ============================================================================
// Steering: normalised -> radians, hard-coded report-side sign flip, × comp gain
// ============================================================================

// 26.09.25 _ YSH _ 입력이 front_wheel_angle 원값 + 보정 배율로 바뀜
TEST(MoraiToAutoware, SteeringDenormalisesByMaxSteeringAngle)
{
  // front_wheel_angle is a normalised [-1, 1] value, NOT an angle. Full lock is +-1.0.
  EXPECT_NEAR(
    morai_to_autoware::to_steering_report(-1.0, make_spec(0.632), 1.0).steering_tire_angle, 0.632,
    1e-5);
}

TEST(MoraiToAutoware, SteeringFlipsSignRelativeToMorai)
{
  const auto spec = make_spec(0.632);
  EXPECT_GT(morai_to_autoware::to_steering_report(-0.5, spec, 1.0).steering_tire_angle, 0.0F)
    << "MORAI negative front_wheel_angle must report as a POSITIVE (left) tire angle";
  EXPECT_LT(morai_to_autoware::to_steering_report(0.5, spec, 1.0).steering_tire_angle, 0.0F)
    << "MORAI positive front_wheel_angle must report as a NEGATIVE (right) tire angle";
}

TEST(MoraiToAutoware, SteeringUndoesTheCompensationGain)
{
  // main.py: -fwa * max_steering_angle * steer_comp_gain
  const auto spec = make_spec(0.698);
  EXPECT_FLOAT_EQ(
    morai_to_autoware::to_steering_report(-0.25, spec, 0.855).steering_tire_angle,
    static_cast<float>(0.25 * 0.698 * 0.855));
}

TEST(MoraiToAutoware, SteeringIsZeroWhenSpecUnavailable)
{
  EXPECT_FLOAT_EQ(
    morai_to_autoware::to_steering_report(1.0, make_spec(0.0), 1.0).steering_tire_angle, 0.0F);
}

TEST(MoraiToAutoware, MaxSteeringAngleUsesMainPyFormula)
{
  // main.py: res.max_steering_angle / (180.0 / math.pi)
  EXPECT_DOUBLE_EQ(morai_to_autoware::max_steering_angle_rad(40.0), 40.0 / (180.0 / M_PI));
}

// ============================================================================
// Enum tables, pinned against NAMED Autoware constants
// ============================================================================

// 26.09.25 _ YSH _ 입력을 MoraiLink 열거형 사본으로 (매핑은 main.py publish_gear_report 와 같다)
TEST(MoraiToAutoware, GearReportMapsEveryMoraiMode)
{
  using morai_to_autoware::to_gear_report;
  EXPECT_EQ(to_gear_report(GearMode::kP), GearReport::PARK);
  EXPECT_EQ(to_gear_report(GearMode::kR), GearReport::REVERSE);
  EXPECT_EQ(to_gear_report(GearMode::kN), GearReport::NEUTRAL);
  EXPECT_EQ(to_gear_report(GearMode::kD), GearReport::DRIVE);
  EXPECT_EQ(to_gear_report(GearMode::kL), GearReport::LOW);
  EXPECT_EQ(to_gear_report(GearMode::kM), GearReport::NONE);
}

TEST(MoraiToAutoware, GearReportFallsBackToNoneOnUnknownValue)
{
  EXPECT_EQ(morai_to_autoware::to_gear_report(static_cast<GearMode>(99)), GearReport::NONE);
}

TEST(MoraiToAutoware, ControlModeMapsManualAndAutonomousModes)
{
  using morai_to_autoware::to_control_mode_report;
  EXPECT_EQ(to_control_mode_report(ControlMode::kUnspecified), ControlModeReport::NO_COMMAND);
  EXPECT_EQ(to_control_mode_report(ControlMode::kKeyboard), ControlModeReport::MANUAL);
  EXPECT_EQ(to_control_mode_report(ControlMode::kGameWheel), ControlModeReport::MANUAL);
  EXPECT_EQ(to_control_mode_report(ControlMode::kAuto), ControlModeReport::AUTONOMOUS);
  EXPECT_EQ(
    to_control_mode_report(ControlMode::kAutoLateral), ControlModeReport::AUTONOMOUS_STEER_ONLY);
  EXPECT_EQ(
    to_control_mode_report(ControlMode::kAutoLongitudinal),
    ControlModeReport::AUTONOMOUS_VELOCITY_ONLY);
}

// 26.09.25 _ YSH _ CRUISE 는 main.py 대로 DISENGAGED(5) — MORAI 가 스스로 운전하는 제2 제어원을 가리지 않는다
// TEST(MoraiToAutoware, CruiseModeIsVelocityOnlyNotDisengaged)
// {
//   EXPECT_EQ(
//     morai_to_autoware::to_control_mode_report(morai_sim_api::actor::VEHICLE_CONTROL_CRUISE_MODE),
//     ControlModeReport::AUTONOMOUS_VELOCITY_ONLY);
//   EXPECT_NE(
//     morai_to_autoware::to_control_mode_report(morai_sim_api::actor::VEHICLE_CONTROL_CRUISE_MODE),
//     ControlModeReport::DISENGAGED);
// }
TEST(MoraiToAutoware, CruiseModeIsDisengagedLikeMainPy)
{
  EXPECT_EQ(
    morai_to_autoware::to_control_mode_report(ControlMode::kCruise), ControlModeReport::DISENGAGED);
}

TEST(MoraiToAutoware, SynchronousModeIsAutonomous)
{
  const auto mode = morai_to_autoware::to_control_mode_report(ControlMode::kSynchronous);
  EXPECT_EQ(mode, ControlModeReport::AUTONOMOUS);
  EXPECT_NE(mode, ControlModeReport::NOT_READY);
}

TEST(MoraiToAutoware, UnknownControlModeIsNotReady)
{
  EXPECT_EQ(
    morai_to_autoware::to_control_mode_report(static_cast<ControlMode>(99)),
    ControlModeReport::NOT_READY);
}

// ============================================================================
// 26.09.25 _ YSH _ TrackedObjects (main.py publish_tracked_objects) + uuid5 바이트 골든
// ============================================================================

TEST(TrackedObjects, ObjectIdIsByteIdenticalToPythonUuid5)
{
  const struct
  {
    const char * name;
    std::array<uint8_t, 16> bytes;
  } golden[] = {
    {"Car_1", {159, 219, 20, 190, 5, 228, 81, 25, 152, 229, 202, 160, 78, 158, 141, 109}},
    {"Ego", {87, 96, 92, 135, 57, 71, 95, 8, 146, 76, 238, 245, 253, 147, 39, 196}},
    {"Pedestrian_12", {30, 114, 183, 107, 149, 163, 93, 121, 177, 175, 196, 71, 87, 218, 163, 197}},
    {"Obstacle", {148, 140, 89, 160, 195, 224, 90, 92, 148, 180, 178, 121, 192, 241, 135, 145}},
    {"차량_7", {115, 129, 12, 143, 35, 94, 90, 17, 166, 65, 6, 144, 228, 29, 67, 113}},
    {"", {198, 92, 202, 85, 213, 107, 94, 32, 178, 212, 196, 49, 191, 166, 136, 39}},
  };
  for (const auto & g : golden) {
    const auto id = morai_to_autoware::object_id_from_name(g.name);
    for (size_t i = 0; i < 16; ++i) {
      EXPECT_EQ(id.uuid[i], g.bytes[i]) << g.name << " byte " << i;
    }
  }
}

TEST(TrackedObjects, ActorConvertsLikeMainPy)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using autoware_perception_msgs::msg::Shape;
  using autoware_perception_msgs::msg::TrackedObjectKinematics;

  morai_grpc_vendor::ActorSample actor;
  actor.id = "Car_1";
  actor.object_type = morai_grpc_vendor::ObjectType::kVehicle;
  actor.position = Vec3{10.0, 20.0, 0.5};
  actor.rotation_deg = Vec3{0.0, 0.0, 30.0};
  actor.velocity_kmh = Vec3{18.0, 0.0, 3.6};  // 26.10.06 _ YSH _ 횡 = z (원문: Vec3{18.0, -3.6, 0.0})
  actor.angular_velocity_raw = Vec3{0.0, -0.2, 0.0};
  actor.bbox_extent = Vec3{1.06, 2.32, 0.75};  // Ioniq 실측 반치수 (x=반폭, y=반길이)

  const auto obj = morai_to_autoware::to_tracked_object(actor, MapOrigin{});
  EXPECT_EQ(obj.object_id, morai_to_autoware::object_id_from_name("Car_1"));
  EXPECT_FLOAT_EQ(obj.existence_probability, 1.0F);
  ASSERT_EQ(obj.classification.size(), 1U);
  EXPECT_EQ(obj.classification[0].label, ObjectClassification::CAR);
  EXPECT_FLOAT_EQ(obj.classification[0].probability, 1.0F);
  EXPECT_NEAR(obj.kinematics.pose_with_covariance.pose.orientation.z, 0.25881904510252074, 1e-12);
  EXPECT_EQ(obj.kinematics.orientation_availability, TrackedObjectKinematics::AVAILABLE);
  EXPECT_NEAR(obj.kinematics.twist_with_covariance.twist.linear.x, 5.0, 1e-12);
  EXPECT_NEAR(obj.kinematics.twist_with_covariance.twist.linear.y, 1.0, 1e-12);
  EXPECT_NEAR(obj.kinematics.twist_with_covariance.twist.angular.z, 0.2, 1e-12);
  EXPECT_FALSE(obj.kinematics.is_stationary);
  EXPECT_EQ(obj.shape.type, Shape::BOUNDING_BOX);
  EXPECT_NEAR(obj.shape.dimensions.x, 4.64, 1e-12);  // 길이 = 2 × extent.y
  EXPECT_NEAR(obj.shape.dimensions.y, 2.12, 1e-12);  // 폭 = 2 × extent.x
  EXPECT_NEAR(obj.shape.dimensions.z, 1.50, 1e-12);

  actor.bbox_extent = Vec3{0.0, 0.0, 0.0};
  actor.object_type = morai_grpc_vendor::ObjectType::kObstacle;
  const auto tiny = morai_to_autoware::to_tracked_object(actor, MapOrigin{});
  EXPECT_DOUBLE_EQ(tiny.shape.dimensions.x, 0.1);  // 0.1 m 하한
  EXPECT_EQ(tiny.classification[0].label, ObjectClassification::UNKNOWN);
}

TEST(TrackedObjects, LabelMapMatchesMainPy)
{
  using autoware_perception_msgs::msg::ObjectClassification;
  using morai_grpc_vendor::ObjectType;
  EXPECT_EQ(morai_to_autoware::to_object_label(ObjectType::kVehicle), ObjectClassification::CAR);
  EXPECT_EQ(
    morai_to_autoware::to_object_label(ObjectType::kPedestrian), ObjectClassification::PEDESTRIAN);
  EXPECT_EQ(morai_to_autoware::to_object_label(ObjectType::kObstacle), ObjectClassification::UNKNOWN);
  EXPECT_EQ(
    morai_to_autoware::to_object_label(ObjectType::kSpawnPoint), ObjectClassification::UNKNOWN);
}

TEST(MoraiToAutoware, TimeMessageFromEpochNs)
{
  const auto t = morai_to_autoware::to_time_msg(1758000000123456789LL);
  EXPECT_EQ(t.sec, 1758000000);
  EXPECT_EQ(t.nanosec, 123456789U);
  const auto zero = morai_to_autoware::to_time_msg(-5);
  EXPECT_EQ(zero.sec, 0);
  EXPECT_EQ(zero.nanosec, 0U);
}

// 26.09.26 _ YSH _ DAQ 변환 — daq_publisher.py 와 비트 단위로 같아야 한다. 기대값은 같은 식을 Python 으로 계산한 repr
//   (vz = 1.8 * (1/3.6), ax = 0.5 * k, ay = -0.25 * k, az = -9.81 * k)
TEST(Daq, VehicleStateIsOdometryPlusVerticalVelocity)
{
  morai_grpc_vendor::EgoSample ego;
  ego.position = Vec3{100.0, -50.0, 2.0};
  ego.rotation_deg = Vec3{1.5, -2.0, 37.0};
  ego.velocity_kmh = Vec3{45.0, -7.2, 1.8};
  ego.angular_velocity_raw = Vec3{0.02, -0.41, 0.03};
  MapOrigin origin;
  origin.x = 3.0;
  origin.yaw_deg = 1.5;

  const auto odom = morai_to_autoware::to_odometry(ego, origin);
  const auto daq = morai_to_autoware::to_daq_vehicle_state(ego, origin);
  EXPECT_EQ(daq.header.frame_id, "map");
  EXPECT_EQ(daq.child_frame_id, "base_link");
  EXPECT_EQ(daq.pose.pose, odom.pose.pose);
  EXPECT_EQ(daq.pose.covariance, odom.pose.covariance);
  EXPECT_EQ(daq.twist.twist.linear.x, odom.twist.twist.linear.x);
  EXPECT_EQ(daq.twist.twist.linear.y, odom.twist.twist.linear.y);
  EXPECT_EQ(daq.twist.twist.angular, odom.twist.twist.angular);
  EXPECT_EQ(daq.twist.covariance, odom.twist.covariance);
  EXPECT_EQ(odom.twist.twist.linear.z, 0.0) << "kinematic_state keeps main.py's vz = 0";
  // 26.10.06 _ YSH _ 수직 = velocity.y (D1). 원문: EXPECT_EQ(daq.twist.twist.linear.z, 0.5);  // 1.8 * (1.0 / 3.6)
  EXPECT_DOUBLE_EQ(daq.twist.twist.linear.z, -7.2 * (1.0 / 3.6));
  EXPECT_DOUBLE_EQ(daq.twist.twist.linear.y, 1.8 * (1.0 / 3.6));
}

TEST(Daq, AccelerationMatchesDaqPublisherBitForBit)
{
  morai_grpc_vendor::EgoSample ego;
  ego.acceleration_raw = Vec3{0.5, 0.25, -9.81};
  const auto a1 = morai_to_autoware::to_daq_acceleration(ego, 1.0);
  EXPECT_EQ(a1.header.frame_id, "base_link");
  EXPECT_EQ(a1.accel.accel.linear.x, 0.5);
  EXPECT_EQ(a1.accel.accel.linear.y, -0.25);
  EXPECT_EQ(a1.accel.accel.linear.z, -9.81);
  const auto a2 = morai_to_autoware::to_daq_acceleration(ego, 0.2778);
  EXPECT_EQ(a2.accel.accel.linear.x, 0.1389);
  EXPECT_EQ(a2.accel.accel.linear.y, -0.06945);
  EXPECT_EQ(a2.accel.accel.linear.z, -2.725218);
  for (size_t i = 0; i < 36; ++i) {
    const bool diag = i == 0 || i == 7 || i == 14;
    EXPECT_EQ(a1.accel.covariance[i], diag ? 1e-4 : 0.0) << i;
  }
}

TEST(Daq, TrackedObjectAddsAccelerationToTheBridgeObject)
{
  morai_grpc_vendor::ActorSample actor;
  actor.id = "NPC_Car_1";
  actor.object_type = morai_grpc_vendor::ObjectType::kVehicle;
  actor.position = Vec3{10.0, 20.0, 0.5};
  actor.rotation_deg = Vec3{0.0, 0.0, 30.0};
  actor.velocity_kmh = Vec3{18.0, -3.6, 0.0};
  actor.angular_velocity_raw = Vec3{0.0, -0.2, 0.0};
  actor.acceleration_raw = Vec3{1.2, -0.4, 0.05};
  actor.bbox_extent = Vec3{1.06, 2.32, 0.75};

  auto base = morai_to_autoware::to_tracked_object(actor, MapOrigin{});
  const auto daq = morai_to_autoware::to_daq_tracked_object(actor, MapOrigin{}, 0.2778);
  const auto & ac = daq.kinematics.acceleration_with_covariance.accel.linear;
  EXPECT_EQ(ac.x, 0.33336);  // Python 1.2 * 0.2778
  EXPECT_EQ(ac.y, 0.11112);  // Python -(-0.4) * 0.2778
  EXPECT_EQ(ac.z, 0.01389);  // Python 0.05 * 0.2778
  EXPECT_EQ(
    base.kinematics.acceleration_with_covariance.accel.linear.x, 0.0)
    << "tracking/objects keeps main.py's empty acceleration";
  base.kinematics.acceleration_with_covariance = daq.kinematics.acceleration_with_covariance;
  EXPECT_EQ(daq, base) << "everything but the acceleration is the bridge's object";
}

// 26.10.06 _ YSH _ Autoware 객체 pose 는 MORAI bounding_box.location(상자 중심, world) — 없으면 transform.location (D2)
TEST(MoraiToAutoware, TrackedObjectPoseIsBoundingBoxCenter)
{
  morai_grpc_vendor::ActorSample actor;
  actor.id = "Car_1";
  actor.object_type = morai_grpc_vendor::ObjectType::kVehicle;
  actor.position = Vec3{864.65, -428.12, 0.84};      // 뒤차축 (실측 scenario_2_19)
  actor.bbox_location = Vec3{865.53, -427.07, 1.22};  // 상자 중심 — 1.37 m 앞
  actor.rotation_deg = Vec3{0.0, 0.0, 49.83};
  const auto p = morai_to_autoware::to_tracked_object(actor, MapOrigin{}).kinematics.pose_with_covariance.pose;
  EXPECT_DOUBLE_EQ(p.position.x, 865.53);
  EXPECT_DOUBLE_EQ(p.position.y, -427.07);
  EXPECT_DOUBLE_EQ(p.position.z, 1.22);
  actor.bbox_location = Vec3{};
  const auto q = morai_to_autoware::to_tracked_object(actor, MapOrigin{}).kinematics.pose_with_covariance.pose;
  EXPECT_DOUBLE_EQ(q.position.x, 864.65) << "상자 중심이 없으면 transform.location";
}

// 26.10.06 _ YSH _ 명세 25 = MORAI 에 보낸 gRPC 명령을 CtrlCmd 로 그대로 (steer 만 rad)
TEST(DaqMorai, CtrlCmdIsTheSentGrpcCommand)
{
  morai_grpc_vendor::VehicleCommand c;
  c.long_cmd_type = morai_grpc_vendor::LongCmdType::kAcceleration;
  c.throttle = 0.25;
  c.brake = 0.1;
  c.steer = 0.5;
  c.velocity = 28.8;   // km/h (8 m/s × 3.6)
  c.acceleration = -1.25;
  const auto m = morai_to_autoware::to_daq_ctrl_cmd(c, 40.0);
  EXPECT_EQ(m.longl_cmd_type, 3);
  EXPECT_EQ(m.accel, 0.25);
  EXPECT_EQ(m.brake, 0.1);
  EXPECT_EQ(m.velocity, 28.8) << "gRPC velocity (km/h) as sent";
  EXPECT_EQ(m.acceleration, -1.25);
  EXPECT_DOUBLE_EQ(m.front_steer, 0.5 * 40.0 * M_PI / 180.0);
  EXPECT_EQ(m.rear_steer, 0.0);
}

// 26.10.12 _ YSH _ 의사 VILS: 실차 값으로 만든 ego_status 가 같은 물리 상태의 MORAI Ego 와 같은 칸·같은 축이 된다
//   (CSV 변환기가 GT 런과 VILS 런을 같은 식으로 읽는 근거)
TEST(DaqMorai, RealCarEgoStatusUsesTheMoraiAxesAndUnits)
{
  constexpr double kRad = M_PI / 180.0;
  morai_to_autoware::RealCarEgoSample s;
  s.map_pose.position.x = 10.0;
  s.map_pose.position.y = 20.0;
  s.map_pose.position.z = 1.5;
  s.map_pose.orientation.z = std::sin(M_PI / 4.0);  // yaw 90°
  s.map_pose.orientation.w = std::cos(M_PI / 4.0);
  s.speed_mps = 5.0;
  s.lateral_mps = 0.2;
  s.yaw_rate_rps = 0.1;
  s.accel_long_mps2 = 1.0;
  s.accel_lat_mps2 = 0.5;
  s.tire_angle_rad = 0.05;
  s.throttle = 0.3;
  s.brake = 1.7;  // 0~1 로 자른다
  const MapOrigin origin{};
  const auto real = morai_to_autoware::to_daq_ego_status_real(s, origin);

  // 같은 상태를 MORAI 원값 축으로: velocity (앞, 위, 왼) km/h, acceleration (앞, 오른, 위), 각속도 world (−y = 요레이트)
  morai_grpc_vendor::EgoSample ego;
  ego.position = Vec3{10.0, 20.0, 1.5};
  ego.rotation_deg = Vec3{0.0, 0.0, 90.0};
  ego.velocity_kmh = Vec3{5.0 * 3.6, 0.0, 0.2 * 3.6};
  ego.acceleration_raw = Vec3{1.0, -0.5, 0.0};
  ego.angular_velocity_raw = Vec3{0.0, -0.1, 0.0};
  ego.vehicle_state.throttle = 0.3;
  ego.vehicle_state.brake = 1.0;
  ego.vehicle_state.front_wheel_angle = -(0.05 / kRad) / 40.0;  // 정규화 (오른 +) × 40 deg
  const auto morai = morai_to_autoware::to_daq_ego_status(ego, 1.0, 40.0);

  EXPECT_NEAR(real.position.x, morai.position.x, 1e-9);
  EXPECT_NEAR(real.position.y, morai.position.y, 1e-9);
  EXPECT_NEAR(real.position.z, morai.position.z, 1e-9);
  EXPECT_NEAR(real.heading, morai.heading, 1e-9);
  EXPECT_NEAR(real.velocity.x, morai.velocity.x, 1e-9);
  EXPECT_NEAR(real.velocity.y, morai.velocity.y, 1e-9);
  EXPECT_NEAR(real.velocity.z, morai.velocity.z, 1e-9);
  EXPECT_NEAR(real.acceleration.x, morai.acceleration.x, 1e-9);
  EXPECT_NEAR(real.acceleration.y, morai.acceleration.y, 1e-9);
  EXPECT_NEAR(real.angular_velocity.y, morai.angular_velocity.y, 1e-9);
  EXPECT_NEAR(real.front_steer_angle, morai.front_steer_angle, 1e-9);
  EXPECT_DOUBLE_EQ(real.accel, 0.3);
  EXPECT_DOUBLE_EQ(real.brake, 1.0);
  EXPECT_EQ(real.header.frame_id, "map");
}

// 26.10.12 _ YSH _ 의사 VILS: 명세 25 = 실차에 보낸 Autoware 명령 (가속도 모드, km/h, rad)
TEST(DaqMorai, CtrlCmdFromTheAutowareCommandSentToTheRealCar)
{
  autoware_control_msgs::msg::Control c;
  c.stamp.sec = 42;
  c.longitudinal.acceleration = -1.5F;
  c.longitudinal.velocity = 5.0F;
  c.lateral.steering_tire_angle = 0.1F;
  const auto m = morai_to_autoware::to_daq_ctrl_cmd_from_control(c);
  EXPECT_EQ(m.header.stamp.sec, 42);
  EXPECT_EQ(m.longl_cmd_type, 3);
  EXPECT_DOUBLE_EQ(m.acceleration, -1.5);
  EXPECT_NEAR(m.velocity, 18.0, 1e-5);
  EXPECT_NEAR(m.front_steer, 0.1, 1e-7);
  EXPECT_EQ(m.accel, 0.0);
  EXPECT_EQ(m.brake, 0.0);
}

// 26.10.05 _ YSH _ DAQ 명세 토픽 MORAI 메시지 — gRPC 값을 MORAI 정의 단위로만 바꾼다. 옛 /daq/* 와의 관계(부호 반전·비트 일치)도
//   여기서 묶어 둔다 — daq_export 가 새 bag 에서 옛 CSV 와 같은 값을 만드는 근거
TEST(DaqMorai, EgoStatusKeepsMoraiUnitsAndAxes)
{
  morai_grpc_vendor::EgoSample ego;
  ego.position = Vec3{100.0, -50.0, 2.0};
  ego.rotation_deg = Vec3{1.5, -2.0, 37.0};
  ego.velocity_kmh = Vec3{45.0, -7.2, 1.8};
  ego.angular_velocity_raw = Vec3{0.02, -0.41, 0.03};
  ego.acceleration_raw = Vec3{0.5, 0.25, -9.81};
  ego.vehicle_state.present = true;
  ego.vehicle_state.throttle = 0.3;
  ego.vehicle_state.brake = 0.1;
  ego.vehicle_state.front_wheel_angle = 0.2;

  const auto m = morai_to_autoware::to_daq_ego_status(ego, 1.0, 40.0);
  EXPECT_EQ(m.header.frame_id, "map");
  EXPECT_EQ(m.unique_id, 0);
  EXPECT_EQ(m.position.x, 100.0);
  EXPECT_EQ(m.position.y, -50.0);
  EXPECT_EQ(m.position.z, 2.0);
  EXPECT_EQ(m.heading, 37.0);
  EXPECT_EQ(m.accel, 0.3);
  EXPECT_EQ(m.brake, 0.1);
  EXPECT_DOUBLE_EQ(m.front_steer_angle, -8.0) << "Unity sign flipped to left +";
  EXPECT_EQ(m.rear_steer_angle, 0.0);
  EXPECT_EQ(m.acceleration.x, 0.5);
  EXPECT_EQ(m.acceleration.y, 0.25) << "MORAI axes";
  EXPECT_EQ(m.acceleration.z, -9.81);
  EXPECT_DOUBLE_EQ(m.angular_velocity.y, -0.41 * 180.0 / M_PI);

  // 옛 토픽과 비트 관계 (map_origin 0): vx·vz 같고 vy 는 부호만, 가속도는 (x, −y, z)
  const auto old_state = morai_to_autoware::to_daq_vehicle_state(ego, MapOrigin{});
  const auto old_accel = morai_to_autoware::to_daq_acceleration(ego, 1.0);
  EXPECT_EQ(m.position.x, old_state.pose.pose.position.x);
  EXPECT_EQ(m.position.y, old_state.pose.pose.position.y);
  EXPECT_EQ(m.position.z, old_state.pose.pose.position.z);
  EXPECT_EQ(m.velocity.x, old_state.twist.twist.linear.x);
  // 26.10.06 _ YSH _ MORAI (앞, 위, 왼) → ROS (앞, 왼, 위). 원문: −m.velocity.y == linear.y, m.velocity.z == linear.z
  EXPECT_EQ(m.velocity.z, old_state.twist.twist.linear.y);
  EXPECT_EQ(m.velocity.y, old_state.twist.twist.linear.z);
  EXPECT_EQ(m.acceleration.x, old_accel.accel.accel.linear.x);
  EXPECT_EQ(-m.acceleration.y, old_accel.accel.accel.linear.y);
  EXPECT_EQ(m.acceleration.z, old_accel.accel.accel.linear.z);

  const auto scaled = morai_to_autoware::to_daq_ego_status(ego, 0.2778, 40.0);
  EXPECT_EQ(scaled.acceleration.x, 0.1389);  // daq.accel_scale 는 지금처럼 곱한다
}

TEST(DaqMorai, ObjectUniqueIdIsTheFirstFourUuid5BytesAsInt32Bits)
{
  // Python: struct.unpack('>i', uuid.uuid5(uuid.NAMESPACE_OID, 'morai:NPC_Car_1').bytes[:4]) — u32 0x935a5939
  EXPECT_EQ(morai_to_autoware::object_unique_id_from_name("NPC_Car_1"), -1822795463);
  EXPECT_EQ(static_cast<uint32_t>(morai_to_autoware::object_unique_id_from_name("NPC_Car_1")), 0x935a5939U);
  EXPECT_EQ(static_cast<uint32_t>(morai_to_autoware::object_unique_id_from_name("Ped_1")), 0x848ee341U);
}

TEST(DaqMorai, ObjectStatusListSplitsByTypeAndKeepsGrpcValues)
{
  std::vector<morai_grpc_vendor::ActorSample> actors(4);
  actors[0].id = "NPC_Car_1";
  actors[0].object_type = morai_grpc_vendor::ObjectType::kVehicle;
  actors[0].position = Vec3{10.0, 20.0, 0.5};
  actors[0].rotation_deg = Vec3{0.5, -1.0, 30.0};
  actors[0].velocity_kmh = Vec3{18.0, -3.6, 0.0};
  actors[0].global_velocity_raw = Vec3{15.6, 9.0, 0.1};
  actors[0].acceleration_raw = Vec3{1.2, -0.4, 0.05};
  actors[0].bbox_extent = Vec3{1.06, 2.32, 0.75};
  actors[1].id = "Ped_1";
  actors[1].object_type = morai_grpc_vendor::ObjectType::kPedestrian;
  actors[2].id = "Cone_1";
  actors[2].object_type = morai_grpc_vendor::ObjectType::kObstacle;
  actors[3].id = "Unknown_1";  // kUnspecified → 정적 장애물 목록 (TrackedObjects 의 UNKNOWN 과 같은 자리)

  const auto list = morai_to_autoware::to_daq_object_status_list(actors, 1.0);
  EXPECT_EQ(list.header.frame_id, "map");
  ASSERT_EQ(list.npc_list.size(), 1U);
  ASSERT_EQ(list.pedestrian_list.size(), 1U);
  ASSERT_EQ(list.obstacle_list.size(), 2U);
  EXPECT_EQ(list.num_of_npcs, 1);
  EXPECT_EQ(list.num_of_pedestrian, 1);
  EXPECT_EQ(list.num_of_obstacle, 2);
  EXPECT_EQ(list.npc_list[0].type, 1);
  EXPECT_EQ(list.pedestrian_list[0].type, 0);
  EXPECT_EQ(list.obstacle_list[0].type, 2);
  EXPECT_EQ(list.obstacle_list[1].name, "Unknown_1");

  const auto & o = list.npc_list[0];
  EXPECT_EQ(o.name, "NPC_Car_1");
  EXPECT_EQ(o.unique_id, morai_to_autoware::object_unique_id_from_name("NPC_Car_1"));
  EXPECT_EQ(o.heading, 30.0);
  EXPECT_EQ(o.position.x, 10.0);
  EXPECT_EQ(o.position.y, 20.0);
  EXPECT_EQ(o.position.z, 0.5);
  EXPECT_EQ(o.velocity.x, 15.6) << "global_velocity km/h as received";
  EXPECT_EQ(o.velocity.y, 9.0);
  EXPECT_EQ(o.velocity.z, 0.1);
  EXPECT_EQ(o.acceleration.x, 1.2);
  EXPECT_EQ(o.acceleration.y, -0.4) << "MORAI axes";
  EXPECT_EQ(o.size.x, 2.12) << "width";
  EXPECT_EQ(o.size.y, 4.64) << "length";
  EXPECT_EQ(o.size.z, 1.5);

  // 옛 TrackedObject 과 위치·크기 관계 (map_origin 0): 길이 = size.y, 폭 = size.x
  const auto old_obj = morai_to_autoware::to_tracked_object(actors[0], MapOrigin{});
  EXPECT_EQ(o.position.x, old_obj.kinematics.pose_with_covariance.pose.position.x);
  EXPECT_EQ(o.size.y, old_obj.shape.dimensions.x);
  EXPECT_EQ(o.size.x, old_obj.shape.dimensions.y);
  EXPECT_EQ(o.size.z, old_obj.shape.dimensions.z);
}

// 26.10.07 _ YSH _ /daq/collision: gRPC Ego collision_objects 의 이름 → 그 actor 의 ObjectStatus (actor 가 아니면 이름만, type -1)
TEST(DaqMorai, CollisionDataListsTheEgoCollisionObjectsAsObjectStatus)
{
  std::vector<morai_grpc_vendor::ActorSample> actors(2);
  actors[0].id = "NPC_Car_1";
  actors[0].object_type = morai_grpc_vendor::ObjectType::kVehicle;
  actors[0].position = Vec3{10.0, 20.0, 0.5};
  actors[0].global_velocity_raw = Vec3{15.6, 9.0, 0.1};
  actors[0].bbox_extent = Vec3{1.06, 2.32, 0.75};
  actors[1].id = "Ped_1";
  actors[1].object_type = morai_grpc_vendor::ObjectType::kPedestrian;

  morai_grpc_vendor::EgoSample ego;
  const auto none = morai_to_autoware::to_daq_collision_data(ego, actors, 1.0);
  EXPECT_EQ(none.header.frame_id, "map");
  EXPECT_TRUE(none.collision_object.empty()) << "no collision → empty list (still published)";
  EXPECT_EQ(none.global_offset_x, 0.0);

  ego.vehicle_state.present = true;
  ego.vehicle_state.collision_objects = {"Guardrail_7", "NPC_Car_1"};
  ego.vehicle_state.collision_count = 2;
  const auto msg = morai_to_autoware::to_daq_collision_data(ego, actors, 1.0);
  ASSERT_EQ(msg.collision_object.size(), 2U);
  const auto & wall = msg.collision_object[0];
  EXPECT_EQ(wall.name, "Guardrail_7") << "gRPC order kept";
  EXPECT_EQ(wall.type, -1) << "not an actor";
  EXPECT_EQ(wall.unique_id, morai_to_autoware::object_unique_id_from_name("Guardrail_7"));
  EXPECT_EQ(wall.position.x, 0.0);
  const auto & car = msg.collision_object[1];
  const auto want = morai_to_autoware::to_daq_object_status(actors[0], 1.0);
  EXPECT_EQ(car.name, "NPC_Car_1");
  EXPECT_EQ(car.type, 1);
  EXPECT_EQ(car.unique_id, want.unique_id);
  EXPECT_EQ(car.position.x, 10.0);
  EXPECT_EQ(car.velocity.x, 15.6) << "same values as /daq/object_status";
  EXPECT_EQ(car.size.y, want.size.y);
}

}  // namespace
}  // namespace autoware::morai_interface


// 26.11.12 _ YSH _ 신규 — /daq/calc/ego_*_world: 차체 축(MORAI 앞·위·왼 / 앞·오른·위)을 heading 으로 돌린다
TEST(MoraiToAutoware, DaqEgoWorldRotatesBodyAxesByHeading)
{
  morai_ros2_msgs::msg::EgoVehicleStatus ego;
  ego.header.stamp.sec = 12;
  ego.header.stamp.nanosec = 345u;
  ego.heading = 90.0;               // 북쪽을 본다
  ego.velocity.x = 10.0;            // 앞 10 m/s
  ego.velocity.z = 2.0;             // 왼 2 m/s (MORAI 축 z = 왼)
  ego.velocity.y = 0.5;             // 위
  ego.acceleration.x = 1.0;         // 앞 1
  ego.acceleration.y = 3.0;         // 오른 3 (MORAI 축 y = 오른) → 왼 −3
  ego.acceleration.z = -0.25;
  geometry_msgs::msg::Vector3Stamped v;
  geometry_msgs::msg::Vector3Stamped a;
  autoware::morai_interface::morai_to_autoware::to_daq_ego_world(ego, v, a);
  EXPECT_EQ(v.header.stamp, ego.header.stamp);
  EXPECT_EQ(a.header.stamp, ego.header.stamp);
  EXPECT_EQ(v.header.frame_id, "map");
  // heading 90°: 앞 → +y(북), 왼 → −x(서)
  EXPECT_NEAR(v.vector.x, -2.0, 1e-12);
  EXPECT_NEAR(v.vector.y, 10.0, 1e-12);
  EXPECT_EQ(v.vector.z, 0.5);
  EXPECT_NEAR(a.vector.x, 3.0, 1e-12);   // 왼 −3 → 서쪽 −(−3) = +3
  EXPECT_NEAR(a.vector.y, 1.0, 1e-12);
  EXPECT_EQ(a.vector.z, -0.25);
  // heading 0°: 그대로 (vx, vy_left)
  ego.heading = 0.0;
  autoware::morai_interface::morai_to_autoware::to_daq_ego_world(ego, v, a);
  EXPECT_EQ(v.vector.x, 10.0);
  EXPECT_EQ(v.vector.y, 2.0);
  EXPECT_EQ(a.vector.y, -3.0);
}
