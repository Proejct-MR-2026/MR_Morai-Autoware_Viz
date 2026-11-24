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

// 램프 명령 캐시 — 명령/상태 두 방향이 공유하는 유일한 가변 상태

#ifndef AUTOWARE__MORAI_INTERFACE__LAMP_COMMAND_CACHE_HPP_
#define AUTOWARE__MORAI_INTERFACE__LAMP_COMMAND_CACHE_HPP_

#include <autoware_vehicle_msgs/msg/hazard_lights_report.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_report.hpp>

#include <cstdint>
#include <mutex>

namespace autoware::morai_interface
{

/// The last lamp state Autoware asked for.
///
/// This class exists because of two facts about the MORAI API:
///
///   1. Actor.SetVehicleTailLight takes a SINGLE VehicleTurnSignal, while Autoware sends
///      TurnIndicatorsCommand and HazardLightsCommand independently. Something has to
///      hold both so the merged value can be computed.
///   2. MORAI offers no lamp read-back at all. The status direction therefore has
///      nothing to report except what the command direction last sent.
///
/// So this is the only mutable state shared between the two directions:
///   - CommandForwarder writes it, then sends the merged value.
///   - StatusReporter reads it and echoes it as TurnIndicatorsReport / HazardLightsReport.
///
/// It deals only in Autoware constants and never names a MORAI type. The merge lives in
/// conversion/autoware_to_morai.hpp, so that this shared header does not become a third
/// place where MORAI leaks into the Autoware-facing package.
class LampCommandCache
{
public:
  struct Snapshot
  {
    uint8_t turn_indicators{autoware_vehicle_msgs::msg::TurnIndicatorsReport::DISABLE};
    uint8_t hazard_lights{autoware_vehicle_msgs::msg::HazardLightsReport::DISABLE};
  };

  /// NO_COMMAND is ignored, matching the "no command means do not change anything"
  /// convention the command callbacks use.
  void set_turn_indicators(uint8_t command);
  void set_hazard_lights(uint8_t command);

  /// Both fields returned under one lock.
  ///
  /// Deliberately not two std::atomic<uint8_t>: two atomics permit a torn pair, i.e. a
  /// frame in which the merge sees the new turn indicator together with the stale hazard
  /// state. At 10-50 Hz a mutex around two bytes costs nothing measurable and removes an
  /// unbounded "prove this is safe" argument.
  Snapshot get() const;

private:
  mutable std::mutex mutex_;
  Snapshot snapshot_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__LAMP_COMMAND_CACHE_HPP_
