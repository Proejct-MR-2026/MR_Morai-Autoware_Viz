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

// 램프 캐시 + 병합 테스트 — 두 방향이 만나는 유일한 지점의 동작을 고정한다

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"
#include "autoware/morai_interface/lamp_command_cache.hpp"

#include <autoware_vehicle_msgs/msg/hazard_lights_command.hpp>
#include <autoware_vehicle_msgs/msg/turn_indicators_command.hpp>

#include <gtest/gtest.h>

namespace autoware::morai_interface
{
namespace
{

using autoware_vehicle_msgs::msg::HazardLightsCommand;
using autoware_vehicle_msgs::msg::HazardLightsReport;
using autoware_vehicle_msgs::msg::TurnIndicatorsCommand;
using autoware_vehicle_msgs::msg::TurnIndicatorsReport;

// ============================================================================
// Cache behaviour
// ============================================================================

TEST(LampCommandCache, StartsDisabled)
{
  const LampCommandCache cache;
  const auto snapshot = cache.get();

  // MORAI has no lamp read-back, so the initial value IS the reported state until
  // Autoware says otherwise. It must be a defined "off", not zero (which is NO_COMMAND
  // and not a valid report value).
  EXPECT_EQ(snapshot.turn_indicators, TurnIndicatorsReport::DISABLE);
  EXPECT_EQ(snapshot.hazard_lights, HazardLightsReport::DISABLE);
}

TEST(LampCommandCache, StoresEachChannelIndependently)
{
  LampCommandCache cache;

  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_LEFT);
  EXPECT_EQ(cache.get().turn_indicators, TurnIndicatorsReport::ENABLE_LEFT);
  EXPECT_EQ(cache.get().hazard_lights, HazardLightsReport::DISABLE);

  cache.set_hazard_lights(HazardLightsCommand::ENABLE);
  EXPECT_EQ(cache.get().turn_indicators, TurnIndicatorsReport::ENABLE_LEFT);
  EXPECT_EQ(cache.get().hazard_lights, HazardLightsReport::ENABLE);
}

TEST(LampCommandCache, NoCommandIsIgnored)
{
  LampCommandCache cache;
  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_RIGHT);
  cache.set_hazard_lights(HazardLightsCommand::ENABLE);

  // NO_COMMAND means "do not change anything", not "turn off".
  cache.set_turn_indicators(TurnIndicatorsCommand::NO_COMMAND);
  cache.set_hazard_lights(HazardLightsCommand::NO_COMMAND);

  EXPECT_EQ(cache.get().turn_indicators, TurnIndicatorsReport::ENABLE_RIGHT);
  EXPECT_EQ(cache.get().hazard_lights, HazardLightsReport::ENABLE);
}

TEST(LampCommandCache, DisableIsDistinctFromNoCommand)
{
  LampCommandCache cache;
  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_LEFT);
  cache.set_turn_indicators(TurnIndicatorsCommand::DISABLE);

  EXPECT_EQ(cache.get().turn_indicators, TurnIndicatorsReport::DISABLE);
}

// ============================================================================
// Merge into MORAI's single VehicleTurnSignal
// ============================================================================

TEST(LampMerge, MapsEachTurnIndicator)
{
  LampCommandCache cache;

  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_LEFT);
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()), morai_sim_api::actor::VEHICLE_TURN_SIGNAL_LEFT);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()), morai_grpc_vendor::TurnSignal::kLeft);

  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_RIGHT);
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()),
  // morai_sim_api::actor::VEHICLE_TURN_SIGNAL_RIGHT);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()),
    morai_grpc_vendor::TurnSignal::kRight);

  cache.set_turn_indicators(TurnIndicatorsCommand::DISABLE);
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()),
  // morai_sim_api::actor::VEHICLE_TURN_SIGNAL_UNSPECIFIED);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()),
    morai_grpc_vendor::TurnSignal::kOff);
}

TEST(LampMerge, HazardWinsOverTurnIndicator)
{
  LampCommandCache cache;
  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_LEFT);
  cache.set_hazard_lights(HazardLightsCommand::ENABLE);

  // MORAI's VehicleTailLight has one field, so the two commands must be merged and
  // hazard has to take precedence -- that is the road convention and Autoware's.
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()), morai_sim_api::actor::VEHICLE_TURN_SIGNAL_ALL);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()), morai_grpc_vendor::TurnSignal::kAll);
}

TEST(LampMerge, TurnIndicatorResumesWhenHazardClears)
{
  LampCommandCache cache;
  cache.set_turn_indicators(TurnIndicatorsCommand::ENABLE_RIGHT);
  cache.set_hazard_lights(HazardLightsCommand::ENABLE);
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // ASSERT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()), morai_sim_api::actor::VEHICLE_TURN_SIGNAL_ALL);
  ASSERT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()), morai_grpc_vendor::TurnSignal::kAll);

  // The turn indicator was never cleared, only masked, so it must come back.
  cache.set_hazard_lights(HazardLightsCommand::DISABLE);
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()),
  // morai_sim_api::actor::VEHICLE_TURN_SIGNAL_RIGHT);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()),
    morai_grpc_vendor::TurnSignal::kRight);
}

TEST(LampMerge, DefaultCacheMeansLampsOff)
{
  const LampCommandCache cache;
  // 26.09.25 _ YSH _ to_turn_signal 반환형이 MoraiLink 열거형 사본으로 바뀜
  // EXPECT_EQ(
  // autoware_to_morai::to_turn_signal(cache.get()),
  // morai_sim_api::actor::VEHICLE_TURN_SIGNAL_UNSPECIFIED);
  EXPECT_EQ(
    autoware_to_morai::to_turn_signal(cache.get()),
    morai_grpc_vendor::TurnSignal::kOff);
}

}  // namespace
}  // namespace autoware::morai_interface
