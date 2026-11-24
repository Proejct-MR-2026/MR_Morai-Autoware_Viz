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

// ADS -> SIM 방향 순수 변환 — Autoware 명령을 MORAI 요청으로. 조향 부호가 여기선 +1.0 이다
// 26.09.25 _ YSH _ 출력을 proto(VehicleCtrlCmd)에서 MoraiLink 구조체(VehicleCommand)로 바꾸고, 조향을
//   정규화(÷max)에서 main.py(eba6b04)대로 **라디안 그대로 ±max 클램프**로 교정했다 (옛 식은 1.43 배 과조향).
//   main.py 의 조향 결손 보정(_steer_comp_gain_now)·종방향 모드(_fill_longitudinal)·stale 대체 명령·
//   /initialpose 역변환도 여기 옮겼다. 원본: git show 94cdb2b:<이 경로>

#ifndef AUTOWARE__MORAI_INTERFACE__CONVERSION__AUTOWARE_TO_MORAI_HPP_
#define AUTOWARE__MORAI_INTERFACE__CONVERSION__AUTOWARE_TO_MORAI_HPP_

#include "autoware/morai_interface/ego_vehicle_spec.hpp"
#include "autoware/morai_interface/lamp_command_cache.hpp"
// 26.09.25 _ YSH _ /initialpose 역변환용
#include "autoware/morai_interface/map_origin.hpp"

// 26.09.25 _ YSH _ MoraiLink 구조체 (grpc·protobuf 비의존)
#include <morai_grpc_vendor/morai_link_types.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <geometry_msgs/msg/pose.hpp>

// 26.09.25 _ YSH _ proto 헤더 대신 MoraiLink 구조체를 쓴다
// // Generated protobuf headers end in .h, so .clang-format's "C system headers" rule
// // (<.*\.h>) claims them. This grouping is correct; do not reorder.
// #include <morai/actor/actor_control.pb.h>
// #include <morai/actor/actor_enum.pb.h>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace autoware::morai_interface::autoware_to_morai
{

/// Sign applied to the steering COMMAND. Default +1.0, i.e. no flip.
///
/// This is the counterpart of morai_to_autoware::kMoraiToRosLateralSign (-1.0), and the
/// asymmetry is deliberate: MORAI's state field and command field use opposite sign
/// conventions. The command field follows the ROS convention (positive = left), so no
/// flip is needed here.
///
/// Live evidence: with -1.0 configured, the controller commanded right (-0.415 rad)
/// while the vehicle sat at full left lock (+0.632 rad) and saturated the steering rate
/// limiter. The value is exposed as a node parameter only so that a contradicting case
/// (e.g. a MORAI update) can be diagnosed without a rebuild.
inline constexpr double kDefaultSteerCmdSign = 1.0;

// =============================================================================
// 26.09.25 _ YSH _ 신규 — 명령 파라미터 (main.py 이름 그대로)
// =============================================================================

/// main.py steer_comp_enable / steer_comp_speeds_mps / steer_comp_gains (기본 켬).
/// 근거: MORAI 차가 명령보다 덜 돈다 (실측 곡률비 2 m/s 0.870 ... 11 m/s 0.796).
struct SteerCompParams
{
  bool enable{true};
  std::vector<double> speeds_mps{2.0, 5.0, 8.0, 11.0};
  std::vector<double> gains{0.870, 0.855, 0.807, 0.796};
};

/// main.py long_cmd_mode ('acceleration' | 'throttle' | 'velocity')
enum class LongCmdMode { kAcceleration, kThrottle, kVelocity };

/// main.py 와 같이 앞뒤 공백 제거 + 소문자로 비교. 모르는 값이면 nullopt
std::optional<LongCmdMode> parse_long_cmd_mode(const std::string & text);
std::string to_string(LongCmdMode mode);

/// main.py velocity_cmd_unit: kph -> 3.6, mps -> 1.0, off -> 0.0 (소문자 비교). 모르는 값이면 nullopt
std::optional<double> parse_velocity_cmd_unit(const std::string & text);

/// main.py _fill_longitudinal 이 읽는 값 전부
struct LongitudinalParams
{
  LongCmdMode mode{LongCmdMode::kAcceleration};
  bool send_pedal_with_accel{false};
  double pedal_accel_max{3.0};  ///< [m/s^2] throttle = clip(a / 이 값, 0, 1)
  double pedal_brake_max{4.5};  ///< [m/s^2] brake = clip(-a / 이 값, 0, 1)
  /// VehicleCtrlCmd.velocity = Autoware m/s × 이 값 (velocity_cmd_unit, 기동 시 1 회만 읽는다)
  double velocity_cmd_scale{3.6};
};

struct CommandParams
{
  double steer_cmd_sign{kDefaultSteerCmdSign};
  SteerCompParams steer_comp;
  LongitudinalParams longitudinal;
};

// =============================================================================
// 26.09.25 _ YSH _ 신규 — main.py control_command_callback / _fill_longitudinal / _steer_comp_gain_now
// =============================================================================

/// NaN·inf 가 하나라도 있으면 false (steering_tire_angle, acceleration, velocity).
/// 호출자는 그 메시지를 버리고 마지막 유효 명령을 유지한다. main.py 는 거르지 않아
/// max(-m, min(m, nan)) = +m, 즉 좌측 끝까지 꺾는 명령이 될 수 있었다.
bool control_is_finite(const autoware_control_msgs::msg::Control & control);

/// 보정 표가 쓸 만한가: 비어 있지 않고, 길이가 같고, 배율이 0.5~1.5, 속도가 엄격 오름차순
bool steer_comp_table_is_valid(const SteerCompParams & params);

/// main.py _steer_comp_gain_now. 꺼짐이면 1.0, 표가 잘못됐으면 1.0 (*table_invalid = true).
/// 표 구간 안은 선형 보간, 양끝 밖은 끝값. speed_mps 는 |ego velocity.x| / 3.6
double steer_comp_gain(
  double speed_mps, const SteerCompParams & params, bool * table_invalid = nullptr);

/// main.py _fill_longitudinal. accel [m/s^2], velocity [m/s, Autoware].
/// 모드별로 long_cmd_type·acceleration·throttle·brake·velocity 를 채운다 (조향은 건드리지 않음).
/// *active 에는 main.py 로그와 같은 뜻의 활성 모드 설명을, *velocity_forced_zero 에는 velocity 모드에서
/// 단위 off·비유한 때문에 목표속도를 0 으로 바꿨는지를 돌려준다.
void fill_longitudinal(
  morai_grpc_vendor::VehicleCommand * cmd, double accel, double velocity,
  const LongitudinalParams & params, std::string * active = nullptr,
  bool * velocity_forced_zero = nullptr);

/// 한 Control 메시지로 만든 명령과 그때 쓴 값들
struct BuiltCommand
{
  morai_grpc_vendor::VehicleCommand cmd;
  /// 이 명령의 조향을 나눈 배율 (조향 리포트가 × 로 되돌린다). 보정 꺼짐이면 1.0
  double steer_comp_gain{1.0};
  bool comp_table_invalid{false};
  bool velocity_forced_zero{false};
  std::string longitudinal_desc;
};

// 26.09.25 _ YSH _ to_vehicle_ctrl_cmd 대체 — 정규화하지 않는다 (main.py eba6b04)
/// Autoware Control -> MORAI VehicleCommand (main.py control_command_callback).
///
///   steer = clamp(steer_cmd_sign * steering_tire_angle / steer_comp_gain(speed), ±max_steering_angle)
///
/// in RADIANS. MORAI applies VehicleCtrlCmd.steer as a tire angle despite the proto comment's
/// "[-1,1] / max" wording: a 0.487 rad command already read back as the 0.698 rad mechanical
/// limit, and the round-trip gain regressed to 1.4331 = 1/0.698. Dividing by max (the old C++)
/// over-commanded the lateral loop by 1.43x. Longitudinal is fill_longitudinal().
/// The caller must have checked control_is_finite(); max_steering_angle_rad must be > 0.
BuiltCommand build_command(
  const autoware_control_msgs::msg::Control & control, double speed_mps,
  double max_steering_angle_rad, const CommandParams & params);
// morai_sim_api::actor::VehicleCtrlCmd to_vehicle_ctrl_cmd(
//   const autoware_control_msgs::msg::Control & control, const EgoVehicleSpec & spec,
//   double steer_cmd_sign);

/// main.py 의 stale 대체 명령: base 를 복사해 종방향만 fill_longitudinal(stale_accel, 0.0) 로 바꾼다
/// (조향은 마지막 명령 그대로, SPEED 모드면 목표속도 0).
morai_grpc_vendor::VehicleCommand make_stale_command(
  const morai_grpc_vendor::VehicleCommand & base, double stale_accel,
  const LongitudinalParams & params);

/// main.py: cmd_stale_accel 은 감속(음수)이어야 한다. 0 이상·NaN 이면 -2.4 로 대체
double sanitize_stale_accel(double stale_accel, bool * replaced = nullptr);

/// Autoware GearCommand.command -> MORAI GearMode.
///
/// This is a RANGE map, not a value map -- Autoware numbers DRIVE..DRIVE_18 as 2..19 and
/// LOW..LOW_2 as 23..24, so it is not the inverse of morai_to_autoware::to_gear_report.
///
///   1      NEUTRAL      -> N(3)
///   2-19   DRIVE*       -> D(4)
///   20-21  REVERSE*     -> R(2)
///   22     PARK         -> P(1)
///   23-24  LOW*         -> L(5)   (MORAI does not really implement L)
///   other               -> M(0)
///
/// NONE(0) is not handled here: the caller must not send anything at all for it.
// 26.09.25 _ YSH _ 반환형을 MoraiLink 열거형 사본으로
morai_grpc_vendor::GearMode to_gear_mode(uint8_t gear_command);
// morai_sim_api::actor::GearMode to_gear_mode(uint8_t gear_command);

/// Autoware's two independent lamp commands -> MORAI's single VehicleTurnSignal.
///
/// MORAI's VehicleTailLight has one field, so the two must be merged. Hazard wins,
/// matching road convention and Autoware's own behaviour:
///
///   hazard ENABLE            -> ALL(3)
///   else turn ENABLE_LEFT    -> LEFT(1)
///   else turn ENABLE_RIGHT   -> RIGHT(2)
///   else                     -> OFF(0)
///
/// Lives here rather than on LampCommandCache so that the shared-state header never
/// names a MORAI type. This is a conversion, and conversions live in the
/// direction-split conversion headers.
// 26.09.25 _ YSH _ 반환형을 MoraiLink 열거형 사본으로
morai_grpc_vendor::TurnSignal to_turn_signal(const LampCommandCache::Snapshot & lamps);
// morai_sim_api::actor::VehicleTurnSignal to_turn_signal(const LampCommandCache::Snapshot & lamps);

/// Autoware ControlModeCommand.mode -> MORAI VehicleControlMode.
///
///   AUTONOMOUS(1)               -> AUTO_MODE(3)
///   AUTONOMOUS_STEER_ONLY(2)    -> AUTO_MODE_LATERAL(4)
///   AUTONOMOUS_VELOCITY_ONLY(3) -> AUTO_MODE_LONGITUDINAL(5)
///   MANUAL(4)                   -> KEYBOARD(1), i.e. hand control back to the
///                                  simulator's own input
///
/// NO_COMMAND(0) and anything unknown map to UNSPECIFIED(0); the caller must reject the
/// request rather than send it.
// 26.09.25 _ YSH _ 반환형을 MoraiLink 열거형 사본으로
morai_grpc_vendor::ControlMode to_control_mode(uint8_t control_mode_command);
// morai_sim_api::actor::VehicleControlMode to_control_mode(uint8_t control_mode_command);

// 26.09.25 _ YSH _ 신규 — main.py initialpose_callback 의 역변환 (get_map_pose 의 정확한 역)
/// map-frame pose (RViz 2D Pose Estimate) -> MORAI SetTransform target.
///
///   morai_xy = R(-yaw_offset) * (map_xy - offset_xy)
///   morai_yaw [deg] = degrees(map_yaw - yaw_offset), roll = pitch = 0 (ground decides)
///   z: current MORAI ego z (0 if unknown); if |map z| > 0.01, map z - offset_z instead
///
/// *map_yaw_rad (optional) returns the map yaw for logging.
morai_grpc_vendor::TransformPose from_map_pose(
  const geometry_msgs::msg::Pose & map_pose, const MapOrigin & origin,
  std::optional<double> current_morai_z, double * map_yaw_rad = nullptr);

}  // namespace autoware::morai_interface::autoware_to_morai

#endif  // AUTOWARE__MORAI_INTERFACE__CONVERSION__AUTOWARE_TO_MORAI_HPP_
