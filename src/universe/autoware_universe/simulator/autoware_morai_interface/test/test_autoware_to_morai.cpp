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

// ADS -> SIM 변환 단위 테스트 — 명령 부호가 리포트와 반대라는 사실을 고정한다
// 26.09.25 _ YSH _ 정규화 조향(1.43 배 과조향)을 고정하던 테스트를 라디안 조향으로 다시 썼고, main.py 의
//   조향 결손 보정·종방향 모드·stale 대체 명령·/initialpose 역변환 골든을 더했다.
//   골든 값: python3 으로 main.py 식을 그대로 돌린 결과 (scratchpad golden.py, 결과는 아래에 박제).

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"
#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

#include <autoware_vehicle_msgs/msg/control_mode_report.hpp>
#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>

namespace autoware::morai_interface
{
namespace
{

using autoware_vehicle_msgs::msg::GearCommand;
using morai_grpc_vendor::ControlMode;
using morai_grpc_vendor::GearMode;
using morai_grpc_vendor::LongCmdType;

constexpr double kMaxSteer = 0.698;  // 실측 기계 한계 (40 deg) 근처

autoware_control_msgs::msg::Control make_control(
  double steering_tire_angle, double acceleration, double velocity = 0.0)
{
  autoware_control_msgs::msg::Control control;
  control.lateral.steering_tire_angle = static_cast<float>(steering_tire_angle);
  control.longitudinal.acceleration = static_cast<float>(acceleration);
  control.longitudinal.velocity = static_cast<float>(velocity);
  return control;
}

autoware_to_morai::CommandParams no_comp_params()
{
  autoware_to_morai::CommandParams p;
  p.steer_comp.enable = false;
  return p;
}

// ============================================================================
// Steering command: RADIANS (main.py eba6b04) and the deliberate sign ASYMMETRY
// ============================================================================

// 26.09.25 _ YSH _ 정규화(÷max)를 고정하던 테스트 → 라디안 그대로를 고정 (옛 식은 1.43 배 과조향)
// TEST(AutowareToMorai, SteerIsNormalisedByMaxSteeringAngle)
// {
//   const auto spec = make_spec(0.632);
//   const auto cmd = autoware_to_morai::to_vehicle_ctrl_cmd(
//     make_control(0.316, 0.0), spec, autoware_to_morai::kDefaultSteerCmdSign);
//
//   // MORAI's steer field is `tire angle / max steering angle`.
//   EXPECT_NEAR(cmd.steer(), 0.5, 1e-6);
// }
TEST(AutowareToMorai, SteerIsRadiansNotNormalised)
{
  const auto built =
    autoware_to_morai::build_command(make_control(0.316, 0.0), 0.0, kMaxSteer, no_comp_params());
  // MORAI applies VehicleCtrlCmd.steer as a tire angle in radians: 0.316 rad goes out as 0.316.
  EXPECT_NEAR(built.cmd.steer, 0.316, 1e-6);
  EXPECT_DOUBLE_EQ(built.steer_comp_gain, 1.0);
}

TEST(AutowareToMorai, SteerKeepsRosSignConvention)
{
  // Direct regression test for the incident: commanding left must move the simulator left.
  const auto left =
    autoware_to_morai::build_command(make_control(0.2, 0.0), 0.0, kMaxSteer, no_comp_params());
  EXPECT_GT(left.cmd.steer, 0.0) << "A positive (left) Autoware command must stay positive";

  const auto right =
    autoware_to_morai::build_command(make_control(-0.2, 0.0), 0.0, kMaxSteer, no_comp_params());
  EXPECT_LT(right.cmd.steer, 0.0) << "A negative (right) Autoware command must stay negative";
}

TEST(AutowareToMorai, CommandAndReportSignsAreIntentionallyOpposite)
{
  // The two directions must NOT be unified. Commanding left sends a POSITIVE steer,
  // while a left turn is reported from a NEGATIVE MORAI front_wheel_angle.
  EXPECT_GT(autoware_to_morai::kDefaultSteerCmdSign, 0.0);
  EXPECT_LT(morai_to_autoware::kMoraiToRosLateralSign, 0.0);
}

// 26.09.25 _ YSH _ [-1,1] 클램프 → ±max_steering_angle [rad] 클램프
// TEST(AutowareToMorai, SteerIsClampedToProtoRange) { ... EXPECT_DOUBLE_EQ(over.steer(), 1.0); ... }
TEST(AutowareToMorai, SteerIsClampedToMaxSteeringAngle)
{
  const auto over =
    autoware_to_morai::build_command(make_control(1.0, 0.0), 0.0, 0.5, no_comp_params());
  EXPECT_DOUBLE_EQ(over.cmd.steer, 0.5);
  const auto under =
    autoware_to_morai::build_command(make_control(-1.0, 0.0), 0.0, 0.5, no_comp_params());
  EXPECT_DOUBLE_EQ(under.cmd.steer, -0.5);
}

TEST(AutowareToMorai, SteerCmdSignParameterFlipsTheCommand)
{
  auto params = no_comp_params();
  params.steer_cmd_sign = -1.0;
  const auto flipped =
    autoware_to_morai::build_command(make_control(0.2, 0.0), 0.0, kMaxSteer, params);
  EXPECT_LT(flipped.cmd.steer, 0.0);
}

// ============================================================================
// 26.09.25 _ YSH _ 조향 결손 보정 (main.py _steer_comp_gain_now)
// ============================================================================

TEST(SteerComp, GainMatchesMainPyGolden)
{
  // python3: main.py _steer_comp_gain_now 식, 기본 표 (2,5,8,11 m/s → 0.870,0.855,0.807,0.796)
  const autoware_to_morai::SteerCompParams table;
  const struct
  {
    double v;
    double gain;
  } golden[] = {
    {0.0, 0.87},   {1.0, 0.87},  {2.0, 0.87},   {3.5, 0.8625},
    {5.0, 0.855},  {6.5, 0.831}, {8.0, 0.807},  {9.9, 0.8000333333333334},
    {11.0, 0.796}, {30.0, 0.796},
  };
  for (const auto & g : golden) {
    EXPECT_NEAR(autoware_to_morai::steer_comp_gain(g.v, table), g.gain, 1e-12) << "v=" << g.v;
  }
}

TEST(SteerComp, CommandIsDividedByGainThenClamped)
{
  autoware_to_morai::CommandParams params;  // 보정 켬 (main.py 기본)
  const auto built = autoware_to_morai::build_command(make_control(0.2, 0.0), 3.5, kMaxSteer, params);
  EXPECT_NEAR(built.steer_comp_gain, 0.8625, 1e-12);
  EXPECT_NEAR(built.cmd.steer, static_cast<double>(0.2F) / 0.8625, 1e-12);

  // 클램프는 보정 뒤에 건다: 0.65 / 0.796 = 0.8166 → 0.698
  const auto clamped =
    autoware_to_morai::build_command(make_control(0.65, 0.0), 20.0, kMaxSteer, params);
  EXPECT_DOUBLE_EQ(clamped.cmd.steer, kMaxSteer);
}

TEST(SteerComp, DisabledOrInvalidTableIsUnity)
{
  autoware_to_morai::SteerCompParams off;
  off.enable = false;
  EXPECT_DOUBLE_EQ(autoware_to_morai::steer_comp_gain(5.0, off), 1.0);

  bool invalid = false;
  autoware_to_morai::SteerCompParams bad;
  bad.gains = {0.9, 0.9, 0.9};  // 길이 불일치
  EXPECT_DOUBLE_EQ(autoware_to_morai::steer_comp_gain(5.0, bad, &invalid), 1.0);
  EXPECT_TRUE(invalid);

  bad = autoware_to_morai::SteerCompParams{};
  bad.speeds_mps = {2.0, 2.0, 8.0, 11.0};  // 엄격 오름차순 아님
  EXPECT_DOUBLE_EQ(autoware_to_morai::steer_comp_gain(5.0, bad, &invalid), 1.0);
  EXPECT_TRUE(invalid);

  bad = autoware_to_morai::SteerCompParams{};
  bad.gains = {0.870, 0.855, 0.3, 0.796};  // 0.5~1.5 밖
  EXPECT_DOUBLE_EQ(autoware_to_morai::steer_comp_gain(5.0, bad, &invalid), 1.0);
  EXPECT_TRUE(invalid);
}

// ============================================================================
// NaN / inf 정책 (26.09.25 _ YSH _ 메시지를 버리고 마지막 유효 명령 유지 — 판정만 여기서)
// ============================================================================

// 26.09.25 _ YSH _ 옛 C++ 는 NaN 이면 0 명령을 만들었다 → 이제는 메시지 자체를 거부한다 (엔진 시험이 유지 동작 확인)
// TEST(AutowareToMorai, NonFiniteCommandsAreRejected) { ... EXPECT_DOUBLE_EQ(cmd.steer(), 0.0); ... }
TEST(AutowareToMorai, NonFiniteControlIsDetected)
{
  const double nan = std::nan("");
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(autoware_to_morai::control_is_finite(make_control(0.2, 1.0, 5.0)));
  EXPECT_FALSE(autoware_to_morai::control_is_finite(make_control(nan, 0.0)));
  EXPECT_FALSE(autoware_to_morai::control_is_finite(make_control(inf, 0.0)));
  EXPECT_FALSE(autoware_to_morai::control_is_finite(make_control(0.2, nan)));
  EXPECT_FALSE(autoware_to_morai::control_is_finite(make_control(0.2, -inf)));
  EXPECT_FALSE(autoware_to_morai::control_is_finite(make_control(0.2, 0.0, nan)));
}

// ============================================================================
// Longitudinal (main.py _fill_longitudinal) — acceleration mode must be exact
// ============================================================================

TEST(AutowareToMorai, LongitudinalAccelerationModeIsExact)
{
  const auto built = autoware_to_morai::build_command(
    make_control(0.0, -1.25, 8.0), 0.0, kMaxSteer, no_comp_params());
  // main.py 기본: LONG_CMD_TYPE_ACCELERATION + acceleration 만, throttle/brake 0, velocity = m/s × 3.6
  EXPECT_EQ(built.cmd.long_cmd_type, LongCmdType::kAcceleration);
  EXPECT_DOUBLE_EQ(built.cmd.acceleration, -1.25);
  EXPECT_DOUBLE_EQ(built.cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(built.cmd.brake, 0.0);
  EXPECT_DOUBLE_EQ(built.cmd.velocity, 8.0 * 3.6);
  EXPECT_EQ(built.cmd.frame, 0U);
}

TEST(AutowareToMorai, AccelerationModeWithPedals)
{
  autoware_to_morai::LongitudinalParams lp;
  lp.send_pedal_with_accel = true;
  morai_grpc_vendor::VehicleCommand cmd;
  autoware_to_morai::fill_longitudinal(&cmd, 1.5, 0.0, lp);
  EXPECT_EQ(cmd.long_cmd_type, LongCmdType::kAcceleration);
  EXPECT_DOUBLE_EQ(cmd.acceleration, 1.5);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.5);  // 1.5 / 3.0
  EXPECT_DOUBLE_EQ(cmd.brake, 0.0);
  autoware_to_morai::fill_longitudinal(&cmd, -9.0, 0.0, lp);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 1.0);  // clip(9 / 4.5)
}

TEST(AutowareToMorai, ThrottleModeSendsPedalsOnly)
{
  autoware_to_morai::LongitudinalParams lp;
  lp.mode = autoware_to_morai::LongCmdMode::kThrottle;
  morai_grpc_vendor::VehicleCommand cmd;
  autoware_to_morai::fill_longitudinal(&cmd, -2.25, 3.0, lp);
  EXPECT_EQ(cmd.long_cmd_type, LongCmdType::kThrottle);
  EXPECT_DOUBLE_EQ(cmd.acceleration, 0.0);
  EXPECT_DOUBLE_EQ(cmd.throttle, 0.0);
  EXPECT_DOUBLE_EQ(cmd.brake, 0.5);  // 2.25 / 4.5
}

TEST(AutowareToMorai, VelocityModeUsesUnitScale)
{
  autoware_to_morai::LongitudinalParams lp;
  lp.mode = autoware_to_morai::LongCmdMode::kVelocity;
  morai_grpc_vendor::VehicleCommand cmd;
  bool forced = true;
  autoware_to_morai::fill_longitudinal(&cmd, 1.0, 5.0, lp, nullptr, &forced);
  EXPECT_EQ(cmd.long_cmd_type, LongCmdType::kSpeed);
  EXPECT_DOUBLE_EQ(cmd.velocity, 18.0);
  EXPECT_DOUBLE_EQ(cmd.acceleration, 0.0);
  EXPECT_FALSE(forced);

  lp.velocity_cmd_scale = 0.0;  // velocity_cmd_unit=off
  autoware_to_morai::fill_longitudinal(&cmd, 1.0, 5.0, lp, nullptr, &forced);
  EXPECT_DOUBLE_EQ(cmd.velocity, 0.0);
  EXPECT_TRUE(forced);
}

TEST(AutowareToMorai, StaleCommandBrakesAndKeepsSteer)
{
  const auto built = autoware_to_morai::build_command(
    make_control(0.3, 1.0, 10.0), 0.0, kMaxSteer, no_comp_params());
  const auto stale =
    autoware_to_morai::make_stale_command(built.cmd, -2.4, autoware_to_morai::LongitudinalParams{});
  EXPECT_DOUBLE_EQ(stale.steer, built.cmd.steer);  // 조향은 마지막 값
  EXPECT_EQ(stale.long_cmd_type, LongCmdType::kAcceleration);
  EXPECT_DOUBLE_EQ(stale.acceleration, -2.4);
  EXPECT_DOUBLE_EQ(stale.velocity, 0.0);  // 목표속도 0

  autoware_to_morai::LongitudinalParams speed;
  speed.mode = autoware_to_morai::LongCmdMode::kVelocity;
  const auto stale_speed = autoware_to_morai::make_stale_command(built.cmd, -2.4, speed);
  EXPECT_EQ(stale_speed.long_cmd_type, LongCmdType::kSpeed);
  EXPECT_DOUBLE_EQ(stale_speed.velocity, 0.0);  // SPEED 모드는 감속 필드를 무시하므로 목표속도 0
}

TEST(AutowareToMorai, StaleAccelMustBeNegative)
{
  bool replaced = false;
  EXPECT_DOUBLE_EQ(autoware_to_morai::sanitize_stale_accel(-1.0, &replaced), -1.0);
  EXPECT_FALSE(replaced);
  EXPECT_DOUBLE_EQ(autoware_to_morai::sanitize_stale_accel(0.0, &replaced), -2.4);
  EXPECT_TRUE(replaced);
  EXPECT_DOUBLE_EQ(autoware_to_morai::sanitize_stale_accel(1.0), -2.4);
  EXPECT_DOUBLE_EQ(autoware_to_morai::sanitize_stale_accel(std::nan("")), -2.4);
}

TEST(AutowareToMorai, ParsesModeAndUnitLikeMainPy)
{
  using autoware_to_morai::LongCmdMode;
  EXPECT_EQ(autoware_to_morai::parse_long_cmd_mode(" Acceleration "), LongCmdMode::kAcceleration);
  EXPECT_EQ(autoware_to_morai::parse_long_cmd_mode("THROTTLE"), LongCmdMode::kThrottle);
  EXPECT_EQ(autoware_to_morai::parse_long_cmd_mode("velocity"), LongCmdMode::kVelocity);
  EXPECT_FALSE(autoware_to_morai::parse_long_cmd_mode("speed").has_value());
  EXPECT_EQ(autoware_to_morai::parse_velocity_cmd_unit("KPH"), 3.6);
  EXPECT_EQ(autoware_to_morai::parse_velocity_cmd_unit("mps"), 1.0);
  EXPECT_EQ(autoware_to_morai::parse_velocity_cmd_unit("off"), 0.0);
  EXPECT_FALSE(autoware_to_morai::parse_velocity_cmd_unit("knots").has_value());
}

// ============================================================================
// Gear: a RANGE map, not the inverse of the report map
// ============================================================================

// 26.09.25 _ YSH _ 기대값을 MoraiLink 열거형 사본으로 (매핑 자체는 그대로)
TEST(AutowareToMorai, GearMapsEachAutowareRange)
{
  using autoware_to_morai::to_gear_mode;
  EXPECT_EQ(to_gear_mode(GearCommand::NEUTRAL), GearMode::kN);
  EXPECT_EQ(to_gear_mode(GearCommand::DRIVE), GearMode::kD);
  EXPECT_EQ(to_gear_mode(GearCommand::REVERSE), GearMode::kR);
  EXPECT_EQ(to_gear_mode(GearCommand::PARK), GearMode::kP);
  EXPECT_EQ(to_gear_mode(GearCommand::LOW), GearMode::kL);
}

TEST(AutowareToMorai, GearRangeBoundariesAreCorrect)
{
  using autoware_to_morai::to_gear_mode;
  EXPECT_EQ(to_gear_mode(GearCommand::DRIVE_18), GearMode::kD);
  EXPECT_EQ(to_gear_mode(GearCommand::REVERSE), GearMode::kR);
  EXPECT_EQ(to_gear_mode(GearCommand::REVERSE_2), GearMode::kR);
  EXPECT_EQ(to_gear_mode(GearCommand::PARK), GearMode::kP);
  EXPECT_EQ(to_gear_mode(GearCommand::LOW_2), GearMode::kL);
  for (uint8_t c = GearCommand::DRIVE; c <= GearCommand::DRIVE_18; ++c) {
    EXPECT_EQ(to_gear_mode(c), GearMode::kD) << "gear command " << +c;
  }
}

TEST(AutowareToMorai, GearIsNotTheInverseOfTheReportMap)
{
  EXPECT_EQ(autoware_to_morai::to_gear_mode(10), GearMode::kD);
  EXPECT_NE(autoware_to_morai::to_gear_mode(10), GearMode::kM);
}

TEST(AutowareToMorai, UnknownGearFallsBackToManual)
{
  EXPECT_EQ(autoware_to_morai::to_gear_mode(200), GearMode::kM);
}

// ============================================================================
// Control mode request: Autoware engage -> MORAI mode switch
// ============================================================================

TEST(AutowareToMorai, ControlModeMapsAutonomousVariants)
{
  using Request = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
  using autoware_to_morai::to_control_mode;
  EXPECT_EQ(to_control_mode(Request::AUTONOMOUS), ControlMode::kAuto);
  EXPECT_EQ(to_control_mode(Request::AUTONOMOUS_STEER_ONLY), ControlMode::kAutoLateral);
  EXPECT_EQ(to_control_mode(Request::AUTONOMOUS_VELOCITY_ONLY), ControlMode::kAutoLongitudinal);
}

TEST(AutowareToMorai, ControlModeManualHandsBackToSimulator)
{
  using Request = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
  EXPECT_EQ(autoware_to_morai::to_control_mode(Request::MANUAL), ControlMode::kKeyboard);
}

TEST(AutowareToMorai, ControlModeRejectsNoCommandAndUnknown)
{
  using Request = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
  EXPECT_EQ(autoware_to_morai::to_control_mode(Request::NO_COMMAND), ControlMode::kUnspecified);
  EXPECT_EQ(autoware_to_morai::to_control_mode(200), ControlMode::kUnspecified);
}

TEST(AutowareToMorai, ControlModeRoundTripsThroughTheReportMap)
{
  using Request = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
  using autoware_vehicle_msgs::msg::ControlModeReport;
  EXPECT_EQ(
    morai_to_autoware::to_control_mode_report(autoware_to_morai::to_control_mode(Request::AUTONOMOUS)),
    ControlModeReport::AUTONOMOUS);
  EXPECT_EQ(
    morai_to_autoware::to_control_mode_report(autoware_to_morai::to_control_mode(Request::MANUAL)),
    ControlModeReport::MANUAL);
}

// ============================================================================
// 26.09.25 _ YSH _ /initialpose 역변환 (main.py initialpose_callback) 골든
// ============================================================================

struct InverseCase
{
  double p[3];
  double q[4];
  double o[4];  // offset x, y, z, yaw_deg
  double ego_z;  // NaN = 모름
  double out[4];  // morai x, y, z, yaw_deg
};

TEST(InitialPose, InverseMatchesMainPyGolden)
{
  const double nan = std::nan("");
  const InverseCase cases[] = {
    {{10.0, 20.0, 0.0}, {0.0, 0.0, 0.24740395925452294, 0.9689124217106447}, {0.0, 0.0, 0.0, 0.0},
     1.25, {10.0, 20.0, 1.25, 28.64788975654116}},
    {{300100.0, 4100200.0, 0.0}, {0.0, 0.0, -0.8414709848078965, 0.5403023058681398},
     {300000.0, 4100000.0, -3.0, 1.75}, 4.5,
     {106.06106172563582, 196.8528668463603, 4.5, -116.34155902616465}},
    {{1.0, 2.0, 7.5}, {0.0, 0.0, 0.9974949866040544, 0.0707372016677029}, {5.0, -7.0, 1.0, -90.0},
     nan, {-9.0, -3.9999999999999996, 6.5, 261.88733853924697}},
    {{-5.0, 3.0, 0.005},
     {0.05581655424205321, 0.0020439397720577457, 0.4797731630378106, 0.8756129547987779},
     {-12.0, 34.0, 0.0, 12.345}, nan, {0.21041731698231203, -31.77980057446418, 0.0, 44.950779513082324}},
  };
  for (const auto & c : cases) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = c.p[0];
    pose.position.y = c.p[1];
    pose.position.z = c.p[2];
    pose.orientation.x = c.q[0];
    pose.orientation.y = c.q[1];
    pose.orientation.z = c.q[2];
    pose.orientation.w = c.q[3];
    const MapOrigin origin{c.o[0], c.o[1], c.o[2], c.o[3]};
    const std::optional<double> z = std::isnan(c.ego_z) ? std::nullopt : std::optional<double>(c.ego_z);
    const auto t = autoware_to_morai::from_map_pose(pose, origin, z);
    EXPECT_NEAR(t.position.x, c.out[0], 1e-9);
    EXPECT_NEAR(t.position.y, c.out[1], 1e-9);
    EXPECT_NEAR(t.position.z, c.out[2], 1e-12);
    EXPECT_NEAR(t.rotation_deg.z, c.out[3], 1e-9);
    EXPECT_DOUBLE_EQ(t.rotation_deg.x, 0.0);  // roll/pitch 는 지면에 맡긴다
    EXPECT_DOUBLE_EQ(t.rotation_deg.y, 0.0);
  }
}

TEST(InitialPose, InverseUndoesTheForwardMapTransform)
{
  const MapOrigin origin{298000.5, 4101000.25, -2.0, 7.5};
  morai_grpc_vendor::Vec3 loc{123.4, -56.7, 3.0};
  const morai_grpc_vendor::Vec3 rot{0.0, 0.0, -33.0};
  const auto map_pose = morai_to_autoware::to_map_pose(loc, rot, origin);
  const auto back = autoware_to_morai::from_map_pose(map_pose, origin, std::nullopt);
  EXPECT_NEAR(back.position.x, loc.x, 1e-6);
  EXPECT_NEAR(back.position.y, loc.y, 1e-6);
  EXPECT_NEAR(back.position.z, loc.z, 1e-9);  // |map z| > 0.01 이면 map z - offset z
  EXPECT_NEAR(back.rotation_deg.z, rot.z, 1e-9);
}

}  // namespace
}  // namespace autoware::morai_interface
