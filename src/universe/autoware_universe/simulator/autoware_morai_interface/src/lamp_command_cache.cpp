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

// LampCommandCache 구현 — mutex 로 두 바이트를 한 스냅샷으로 묶는다

#include "autoware/morai_interface/lamp_command_cache.hpp"

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

namespace autoware::morai_interface
{

void LampCommandCache::set_turn_indicators(uint8_t command)
{
  if (command == autoware_vehicle_msgs::msg::TurnIndicatorsCommand::NO_COMMAND) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.turn_indicators = command;
}

void LampCommandCache::set_hazard_lights(uint8_t command)
{
  if (command == autoware_vehicle_msgs::msg::HazardLightsCommand::NO_COMMAND) {
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  snapshot_.hazard_lights = command;
}

LampCommandCache::Snapshot LampCommandCache::get() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return snapshot_;
}

}  // namespace autoware::morai_interface
