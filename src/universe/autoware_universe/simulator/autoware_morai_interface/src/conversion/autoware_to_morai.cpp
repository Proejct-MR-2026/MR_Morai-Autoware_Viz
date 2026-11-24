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

// 26.09.25 _ YSH _ 정규화 제거(라디안 그대로) · main.py 종방향/보정/stale/역변환 이식
// ADS -> SIM 변환 구현 — 조향 정규화와 기어 범위 매핑

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"

#include <autoware_vehicle_msgs/msg/gear_command.hpp>
#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>
#include <autoware_vehicle_msgs/srv/control_mode_command.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <string>

namespace autoware::morai_interface::autoware_to_morai
{

namespace
{
/// Python math.pi 와 같은 값 (degrees/radians 식을 main.py 와 비트 단위로 맞춘다)
constexpr double kPi = 3.14159265358979323846;

std::string strip_lower(const std::string & text)
{
  size_t b = 0;
  size_t e = text.size();
  while (b < e && std::isspace(static_cast<unsigned char>(text[b]))) {
    ++b;
  }
  while (e > b && std::isspace(static_cast<unsigned char>(text[e - 1]))) {
    --e;
  }
  std::string out = text.substr(b, e - b);
  std::transform(out.begin(), out.end(), out.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return out;
}

/// Python f"{x:g}"
std::string fmt_g(double x)
{
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%g", x);
  return buf;
}
}  // namespace

std::optional<LongCmdMode> parse_long_cmd_mode(const std::string & text)
{
  const std::string m = strip_lower(text);
  if (m == "acceleration") {
    return LongCmdMode::kAcceleration;
  }
  if (m == "throttle") {
    return LongCmdMode::kThrottle;
  }
  if (m == "velocity") {
    return LongCmdMode::kVelocity;
  }
  return std::nullopt;
}

std::string to_string(LongCmdMode mode)
{
  switch (mode) {
    case LongCmdMode::kThrottle:
      return "throttle";
    case LongCmdMode::kVelocity:
      return "velocity";
    case LongCmdMode::kAcceleration:
    default:
      return "acceleration";
  }
}

std::optional<double> parse_velocity_cmd_unit(const std::string & text)
{
  // main.py 는 lower() 만 한다
  std::string u = text;
  std::transform(u.begin(), u.end(), u.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  if (u == "kph") {
    return 3.6;
  }
  if (u == "mps") {
    return 1.0;
  }
  if (u == "off") {
    return 0.0;
  }
  return std::nullopt;
}

bool control_is_finite(const autoware_control_msgs::msg::Control & control)
{
  return std::isfinite(control.lateral.steering_tire_angle) &&
         std::isfinite(control.longitudinal.acceleration) &&
         std::isfinite(control.longitudinal.velocity);
}

bool steer_comp_table_is_valid(const SteerCompParams & params)
{
  const auto & s = params.speeds_mps;
  const auto & g = params.gains;
  if (s.empty() || s.size() != g.size()) {
    return false;
  }
  for (const double gain : g) {
    // main.py: any(g < 0.5 or g > 1.5) — NaN 은 두 비교 모두 거짓이라 main.py 에서는 통과했다. 여기선 거부
    if (!(gain >= 0.5 && gain <= 1.5)) {
      return false;
    }
  }
  for (size_t i = 1; i < s.size(); ++i) {
    if (!(s[i] > s[i - 1])) {
      return false;
    }
  }
  return true;
}

double steer_comp_gain(double speed_mps, const SteerCompParams & params, bool * table_invalid)
{
  if (table_invalid != nullptr) {
    *table_invalid = false;
  }
  if (!params.enable) {
    return 1.0;
  }
  if (!steer_comp_table_is_valid(params)) {
    if (table_invalid != nullptr) {
      *table_invalid = true;
    }
    return 1.0;
  }
  const auto & speeds = params.speeds_mps;
  const auto & gains = params.gains;
  const double v = speed_mps;
  if (v <= speeds.front()) {
    return gains.front();
  }
  for (size_t i = 1; i < speeds.size(); ++i) {
    if (v <= speeds[i]) {
      const double r = (v - speeds[i - 1]) / (speeds[i] - speeds[i - 1]);
      return gains[i - 1] + r * (gains[i] - gains[i - 1]);
    }
  }
  return gains.back();
}

void fill_longitudinal(
  morai_grpc_vendor::VehicleCommand * cmd, double accel, double velocity,
  const LongitudinalParams & params, std::string * active, bool * velocity_forced_zero)
{
  using morai_grpc_vendor::LongCmdType;

  // 0 이하로 잘못 넣어도 0 으로 나누지 않게 하한을 둔다 (main.py 와 같음)
  const double accel_max = std::max(params.pedal_accel_max, 1e-3);
  const double brake_max = std::max(params.pedal_brake_max, 1e-3);

  const double a = accel;
  const double throttle = a > 0.0 ? std::min(std::max(a / accel_max, 0.0), 1.0) : 0.0;
  const double brake = a < 0.0 ? std::min(std::max(-a / brake_max, 0.0), 1.0) : 0.0;

  // 사본(stale 명령 포함)에 이전 모드의 활성 필드가 남지 않게 한다
  cmd->acceleration = 0.0;
  cmd->throttle = 0.0;
  cmd->brake = 0.0;
  const double scale = params.velocity_cmd_scale;
  cmd->velocity = scale != 0.0 ? velocity * scale : 0.0;
  if (velocity_forced_zero != nullptr) {
    *velocity_forced_zero = false;
  }

  std::string desc;
  if (params.mode == LongCmdMode::kVelocity) {
    cmd->long_cmd_type = LongCmdType::kSpeed;
    if (scale == 0.0 || !std::isfinite(cmd->velocity)) {
      cmd->velocity = 0.0;
      if (velocity_forced_zero != nullptr) {
        *velocity_forced_zero = true;
      }
    }
    desc = "long_cmd_mode=velocity → LONG_CMD_TYPE_SPEED, velocity=Autoware m/s × " +
           fmt_g(scale) + ", throttle/brake/acceleration=0";
  } else if (params.mode == LongCmdMode::kThrottle) {
    cmd->long_cmd_type = LongCmdType::kThrottle;
    cmd->throttle = throttle;
    cmd->brake = brake;
    desc = "long_cmd_mode=throttle → LONG_CMD_TYPE_THROTTLE, throttle=a/" + fmt_g(accel_max) +
           ", brake=-a/" + fmt_g(brake_max) + ", acceleration 미송신";
  } else {
    cmd->long_cmd_type = LongCmdType::kAcceleration;
    cmd->acceleration = a;
    if (params.send_pedal_with_accel) {
      cmd->throttle = throttle;
      cmd->brake = brake;
      desc =
        "long_cmd_mode=acceleration + send_pedal_with_accel=true → LONG_CMD_TYPE_ACCELERATION + "
        "acceleration, 병행 throttle=a/" +
        fmt_g(accel_max) + ", brake=-a/" + fmt_g(brake_max);
    } else {
      desc =
        "long_cmd_mode=acceleration, send_pedal_with_accel=false → 종전 동작 "
        "(LONG_CMD_TYPE_ACCELERATION + acceleration 만, throttle/brake=0)";
    }
  }
  if (active != nullptr) {
    *active = desc;
  }
}

BuiltCommand build_command(
  const autoware_control_msgs::msg::Control & control, double speed_mps,
  double max_steering_angle_rad, const CommandParams & params)
{
  BuiltCommand out;
  fill_longitudinal(
    &out.cmd, static_cast<double>(control.longitudinal.acceleration),
    static_cast<double>(control.longitudinal.velocity), params.longitudinal,
    &out.longitudinal_desc, &out.velocity_forced_zero);

  // 정규화하지 않는다: MORAI 는 steer 를 타이어각 라디안으로 적용한다
  const double steer =
    params.steer_cmd_sign * static_cast<double>(control.lateral.steering_tire_angle);
  out.steer_comp_gain = steer_comp_gain(speed_mps, params.steer_comp, &out.comp_table_invalid);
  const double m = max_steering_angle_rad;
  // main.py: max(-m, min(m, steer / gain))
  out.cmd.steer = std::max(-m, std::min(m, steer / out.steer_comp_gain));
  return out;
}
// 26.09.25 _ YSH _ 정규화(÷max) + [-1,1] 클램프는 1.43 배 과조향이라 폐기 (build_command 로 대체)
// morai_sim_api::actor::VehicleCtrlCmd to_vehicle_ctrl_cmd(
//   const autoware_control_msgs::msg::Control & control, const EgoVehicleSpec & spec,
//   double steer_cmd_sign)
// {
//   morai_sim_api::actor::VehicleCtrlCmd cmd;
//   cmd.set_long_cmd_type(morai_sim_api::actor::LONG_CMD_TYPE_ACCELERATION);
//
//   if (spec.max_steering_angle <= 0.0) {
//     return cmd;
//   }
//
//   if (
//     !std::isfinite(control.longitudinal.acceleration) ||
//     !std::isfinite(control.lateral.steering_tire_angle)) {
//     return cmd;
//   }
//
//   cmd.set_acceleration(control.longitudinal.acceleration);
//
//   const double normalised = steer_cmd_sign *
//                             static_cast<double>(control.lateral.steering_tire_angle) /
//                             spec.max_steering_angle;
//
//   cmd.set_steer(std::clamp(normalised, -1.0, 1.0));
//
//   return cmd;
// }

morai_grpc_vendor::VehicleCommand make_stale_command(
  const morai_grpc_vendor::VehicleCommand & base, double stale_accel,
  const LongitudinalParams & params)
{
  morai_grpc_vendor::VehicleCommand stale = base;
  // SPEED 모드에서는 감속 필드가 무시되므로 목표속도 자체를 0 으로 (main.py 와 같음)
  fill_longitudinal(&stale, stale_accel, 0.0, params);
  return stale;
}

double sanitize_stale_accel(double stale_accel, bool * replaced)
{
  const bool bad = !(stale_accel < 0.0);
  if (replaced != nullptr) {
    *replaced = bad;
  }
  return bad ? -2.4 : stale_accel;
}

// 26.09.25 _ YSH _ 반환형만 MoraiLink 열거형 사본으로 (범위 매핑은 main.py gear_command_callback 과 같음)
morai_grpc_vendor::GearMode to_gear_mode(uint8_t gear_command)
{
  using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
  using morai_grpc_vendor::GearMode;

  if (gear_command == GearCommand::NEUTRAL) {
    return GearMode::kN;
  }
  if (gear_command >= GearCommand::DRIVE && gear_command <= GearCommand::DRIVE_18) {
    return GearMode::kD;
  }
  if (gear_command >= GearCommand::REVERSE && gear_command <= GearCommand::REVERSE_2) {
    return GearMode::kR;
  }
  if (gear_command == GearCommand::PARK) {
    return GearMode::kP;
  }
  if (gear_command >= GearCommand::LOW && gear_command <= GearCommand::LOW_2) {
    // MORAI accepts the value but does not model a low range.
    return GearMode::kL;
  }
  return GearMode::kM;
}
// morai_sim_api::actor::GearMode to_gear_mode(uint8_t gear_command)
// {
//   using GearCommand = autoware_vehicle_msgs::msg::GearCommand;
//
//   if (gear_command == GearCommand::NEUTRAL) {
//     return morai_sim_api::actor::GEAR_MODE_N;
//   }
//   if (gear_command >= GearCommand::DRIVE && gear_command <= GearCommand::DRIVE_18) {
//     return morai_sim_api::actor::GEAR_MODE_D;
//   }
//   if (gear_command >= GearCommand::REVERSE && gear_command <= GearCommand::REVERSE_2) {
//     return morai_sim_api::actor::GEAR_MODE_R;
//   }
//   if (gear_command == GearCommand::PARK) {
//     return morai_sim_api::actor::GEAR_MODE_P;
//   }
//   if (gear_command >= GearCommand::LOW && gear_command <= GearCommand::LOW_2) {
//     // MORAI accepts the value but does not model a low range.
//     return morai_sim_api::actor::GEAR_MODE_L;
//   }
//   return morai_sim_api::actor::GEAR_MODE_M;
// }

// 26.09.25 _ YSH _ 반환형만 MoraiLink 열거형 사본으로
morai_grpc_vendor::TurnSignal to_turn_signal(const LampCommandCache::Snapshot & lamps)
{
  using HazardLightsReport = autoware_vehicle_msgs::msg::HazardLightsReport;
  using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
  using morai_grpc_vendor::TurnSignal;

  // Hazard overrides the turn indicators; MORAI has no way to express both.
  if (lamps.hazard_lights == HazardLightsReport::ENABLE) {
    return TurnSignal::kAll;
  }
  if (lamps.turn_indicators == TurnIndicatorsReport::ENABLE_LEFT) {
    return TurnSignal::kLeft;
  }
  if (lamps.turn_indicators == TurnIndicatorsReport::ENABLE_RIGHT) {
    return TurnSignal::kRight;
  }
  return TurnSignal::kOff;
}
// morai_sim_api::actor::VehicleTurnSignal to_turn_signal(const LampCommandCache::Snapshot & lamps)
// {
//   using HazardLightsReport = autoware_vehicle_msgs::msg::HazardLightsReport;
//   using TurnIndicatorsReport = autoware_vehicle_msgs::msg::TurnIndicatorsReport;
//
//   // Hazard overrides the turn indicators; MORAI has no way to express both.
//   if (lamps.hazard_lights == HazardLightsReport::ENABLE) {
//     return morai_sim_api::actor::VEHICLE_TURN_SIGNAL_ALL;
//   }
//   if (lamps.turn_indicators == TurnIndicatorsReport::ENABLE_LEFT) {
//     return morai_sim_api::actor::VEHICLE_TURN_SIGNAL_LEFT;
//   }
//   if (lamps.turn_indicators == TurnIndicatorsReport::ENABLE_RIGHT) {
//     return morai_sim_api::actor::VEHICLE_TURN_SIGNAL_RIGHT;
//   }
//   return morai_sim_api::actor::VEHICLE_TURN_SIGNAL_UNSPECIFIED;
// }

// 26.09.25 _ YSH _ 반환형만 MoraiLink 열거형 사본으로
morai_grpc_vendor::ControlMode to_control_mode(uint8_t control_mode_command)
{
  using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
  using morai_grpc_vendor::ControlMode;

  switch (control_mode_command) {
    case ControlModeCommand::AUTONOMOUS:
      return ControlMode::kAuto;
    case ControlModeCommand::AUTONOMOUS_STEER_ONLY:
      return ControlMode::kAutoLateral;
    case ControlModeCommand::AUTONOMOUS_VELOCITY_ONLY:
      return ControlMode::kAutoLongitudinal;
    case ControlModeCommand::MANUAL:
      // Hand control back to the simulator's own keyboard input.
      return ControlMode::kKeyboard;
    default:
      return ControlMode::kUnspecified;
  }
}
// morai_sim_api::actor::VehicleControlMode to_control_mode(uint8_t control_mode_command)
// {
//   using ControlModeCommand = autoware_vehicle_msgs::srv::ControlModeCommand::Request;
//
//   switch (control_mode_command) {
//     case ControlModeCommand::AUTONOMOUS:
//       return morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE;
//     case ControlModeCommand::AUTONOMOUS_STEER_ONLY:
//       return morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LATERAL;
//     case ControlModeCommand::AUTONOMOUS_VELOCITY_ONLY:
//       return morai_sim_api::actor::VEHICLE_CONTROL_AUTO_MODE_LONGITUDINAL;
//     case ControlModeCommand::MANUAL:
//       // Hand control back to the simulator's own keyboard input.
//       return morai_sim_api::actor::VEHICLE_CONTROL_KEYBOARD;
//     default:
//       return morai_sim_api::actor::VEHICLE_CONTROL_UNSPECIFIED;
//   }
// }

morai_grpc_vendor::TransformPose from_map_pose(
  const geometry_msgs::msg::Pose & map_pose, const MapOrigin & origin,
  std::optional<double> current_morai_z, double * map_yaw_rad)
{
  const double yaw_offset = origin.yaw_deg * (kPi / 180.0);  // math.radians

  // 평행이동 제거 후 -yaw_offset 만큼 역회전
  const double dx = map_pose.position.x - origin.x;
  const double dy = map_pose.position.y - origin.y;
  const double cos_i = std::cos(-yaw_offset);
  const double sin_i = std::sin(-yaw_offset);

  morai_grpc_vendor::TransformPose out;
  out.position.x = cos_i * dx - sin_i * dy;
  out.position.y = sin_i * dx + cos_i * dy;

  const auto & q = map_pose.orientation;
  const double map_yaw =
    std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  if (map_yaw_rad != nullptr) {
    *map_yaw_rad = map_yaw;
  }

  // z: RViz 는 보통 0 을 준다. 지면 높이는 MORAI 가 정리하므로 현재 차량 z 로 시작한다
  double mz = current_morai_z.value_or(0.0);
  if (std::abs(map_pose.position.z) > 0.01) {
    mz = map_pose.position.z - origin.z;
  }
  out.position.z = mz;

  out.rotation_deg.x = 0.0;  // roll/pitch 는 지면에 맡긴다
  out.rotation_deg.y = 0.0;
  out.rotation_deg.z = (map_yaw - yaw_offset) * (180.0 / kPi);  // math.degrees
  return out;
}

}  // namespace autoware::morai_interface::autoware_to_morai
