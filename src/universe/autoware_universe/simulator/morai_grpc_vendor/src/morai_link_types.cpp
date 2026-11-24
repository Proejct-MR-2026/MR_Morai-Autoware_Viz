// 신규 — 26.09.25 _ YSH _ morai_link_types.hpp 순수 헬퍼 구현 (이름·시계 오프셋·유효성)
//
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

#include "morai_grpc_vendor/morai_link_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace morai_grpc_vendor
{

namespace
{

std::string unknown(int32_t v)
{
  return "UNKNOWN(" + std::to_string(v) + ")";
}

}  // namespace

std::string to_string(GearMode g)
{
  switch (g) {
    case GearMode::kM:
      return "M";
    case GearMode::kP:
      return "P";
    case GearMode::kR:
      return "R";
    case GearMode::kN:
      return "N";
    case GearMode::kD:
      return "D";
    case GearMode::kL:
      return "L";
  }
  return unknown(static_cast<int32_t>(g));
}

std::string to_string(ControlMode m)
{
  switch (m) {
    case ControlMode::kUnspecified:
      return "UNSPECIFIED";
    case ControlMode::kKeyboard:
      return "KEYBOARD";
    case ControlMode::kGameWheel:
      return "GAME_WHEEL";
    case ControlMode::kAuto:
      return "AUTO";
    case ControlMode::kAutoLateral:
      return "AUTO_LATERAL";
    case ControlMode::kAutoLongitudinal:
      return "AUTO_LONGITUDINAL";
    case ControlMode::kCruise:
      return "CRUISE";
    case ControlMode::kSynchronous:
      return "SYNCHRONOUS";
  }
  return unknown(static_cast<int32_t>(m));
}

std::string to_string(LongCmdType t)
{
  switch (t) {
    case LongCmdType::kUnspecified:
      return "UNSPECIFIED";
    case LongCmdType::kThrottle:
      return "THROTTLE";
    case LongCmdType::kSpeed:
      return "SPEED";
    case LongCmdType::kAcceleration:
      return "ACCELERATION";
    case LongCmdType::kGhost:  // 26.10.12 _ YSH _ 의사 VILS follow
      return "GHOST";
  }
  return unknown(static_cast<int32_t>(t));
}

std::string to_string(TurnSignal s)
{
  switch (s) {
    case TurnSignal::kOff:
      return "OFF";
    case TurnSignal::kLeft:
      return "LEFT";
    case TurnSignal::kRight:
      return "RIGHT";
    case TurnSignal::kAll:
      return "ALL";
  }
  return unknown(static_cast<int32_t>(s));
}

std::string to_string(ObjectType t)
{
  switch (t) {
    case ObjectType::kUnspecified:
      return "UNSPECIFIED";
    case ObjectType::kVehicle:
      return "VEHICLE";
    case ObjectType::kPedestrian:
      return "PEDESTRIAN";
    case ObjectType::kObstacle:
      return "OBSTACLE";
    case ObjectType::kSpawnPoint:
      return "SPAWN_POINT";
  }
  return unknown(static_cast<int32_t>(t));
}

double clock_offset_lower_bound_ms(const std::vector<ClockSample> & samples)
{
  constexpr int64_t kMaxUs = std::numeric_limits<int64_t>::max() / 1000;
  bool any = false;
  int64_t best = 0;
  for (const auto & s : samples) {
    // 0·쓰레기 값 방어 (server_us * 1000 이 int64 를 넘지 않게, 수신 시각이 없는 표본은 버린다)
    if (s.server_us <= 0 || s.server_us > kMaxUs || s.local_recv_ns <= 0) {
      continue;
    }
    const int64_t d = s.server_us * 1000 - s.local_recv_ns;
    if (!any || d > best) {
      best = d;
      any = true;
    }
  }
  return any ? static_cast<double>(best) / 1e6 : std::numeric_limits<double>::quiet_NaN();
}

double clock_offset_from_baseline_ms(double remote_lower_bound_ms, double local_baseline_ms)
{
  if (!std::isfinite(remote_lower_bound_ms) || !std::isfinite(local_baseline_ms)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return remote_lower_bound_ms - local_baseline_ms;
}

bool ego_is_valid(const Vec3 & position, const Vec3 & rotation_deg, const Vec3 & velocity_kmh)
{
  const double v[] = {position.x,     position.y,     position.z,
                      rotation_deg.x, rotation_deg.y, rotation_deg.z,
                      velocity_kmh.x, velocity_kmh.y, velocity_kmh.z};
  // NaN != 0.0 이 참이라 NaN 은 "0 이 아님" 으로 유효 쪽에 선다 — daq_publisher 와 같은 판정
  return std::any_of(std::begin(v), std::end(v), [](double a) { return a != 0.0; });
}

bool command_is_finite(const VehicleCommand & cmd)
{
  return std::isfinite(cmd.throttle) && std::isfinite(cmd.brake) && std::isfinite(cmd.steer) &&
         std::isfinite(cmd.velocity) && std::isfinite(cmd.acceleration);
}

}  // namespace morai_grpc_vendor
