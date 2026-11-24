// 신규 — 26.09.25 _ YSH _ MORAI ENU -> Autoware map 리지드 변환 파라미터 (두 방향이 공유하는 값 타입)
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

#ifndef AUTOWARE__MORAI_INTERFACE__MAP_ORIGIN_HPP_
#define AUTOWARE__MORAI_INTERFACE__MAP_ORIGIN_HPP_

namespace autoware::morai_interface
{

/// main.py 의 map_origin_offset_x/y/z, map_origin_yaw_offset_deg.
///
///   map_pos = R(yaw_offset) * morai_pos + (x, y, z)
///   map_yaw = morai_yaw + yaw_offset
///
/// 정방향(상태·GT)은 conversion/morai_to_autoware, 역방향(/initialpose 텔레포트)은
/// conversion/autoware_to_morai 가 쓴다. 전부 0 이면 항등변환이다.
struct MapOrigin
{
  double x{0.0};  ///< [m]
  double y{0.0};  ///< [m]
  double z{0.0};  ///< [m]
  double yaw_deg{0.0};  ///< [deg]
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__MAP_ORIGIN_HPP_
