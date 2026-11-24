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

// 시작 시 한 번 조회하는 ego 차량 제원 — 상태/명령 두 방향이 공유하는 불변 값

#ifndef AUTOWARE__MORAI_INTERFACE__EGO_VEHICLE_SPEC_HPP_
#define AUTOWARE__MORAI_INTERFACE__EGO_VEHICLE_SPEC_HPP_

namespace autoware::morai_interface
{

/// Ego vehicle geometry, fetched once from Actor.GetVehicleSpec at start-up.
///
/// This is one of only two pieces of state shared between the command direction and the
/// status direction, and it is deliberately a plain value type: it is copied into both,
/// so neither can mutate it and no synchronisation is needed on the hot paths.
///
/// The name is EgoVehicleSpec, not VehicleSpec, because morai_sim_api::actor::VehicleSpec
/// appears in the same signatures — two identically-named types one qualification apart
/// is a review hazard worth one extra word.
struct EgoVehicleSpec
{
  /// [rad]. MORAI reports this in DEGREES (see actor_get.proto:112 "[Unit : Deg]");
  /// it is converted exactly once, where it is fetched.
  ///
  // 26.09.25 _ YSH _ 명령 쪽은 더 이상 나누지 않는다 (main.py eba6b04: 라디안 그대로 ±max 클램프)
  /// Used by the two directions:
  ///   status : steering_tire_angle = -1.0 * front_wheel_angle * max_steering_angle * gain
  ///   command: steer = clamp(steer_cmd_sign * steering_tire_angle / gain, ±max_steering_angle)
  /// A zero here would clamp every command to 0 and zero the report, so the engine sends
  /// no command until this has been fetched (and re-fetches it after every reconnect).
  // /// Used mirrored by the two directions:
  // ///   status : steering_tire_angle = -1.0 * front_wheel_angle * max_steering_angle
  // ///   command: steer = steer_cmd_sign * steering_tire_angle / max_steering_angle
  // /// The command path divides, so a zero here is a real hazard — that is why the command
  // /// direction is not constructed until this has been fetched.
  double max_steering_angle{0.0};

  /// [m]
  double wheel_base{0.0};
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__EGO_VEHICLE_SPEC_HPP_
