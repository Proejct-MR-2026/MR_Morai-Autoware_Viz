// 신규 — 26.09.25 _ YSH _ MoraiLink 내부 변환 (proto <-> 공개 구조체). 설치하지 않는다
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

// 이 패키지 안(morai_link.cpp, tools/, test/)에서만 쓴다. protobuf 타입이 보이므로 include/ 로
// 옮기지 말 것 — 공개 헤더 격리(morai_link_types.hpp)가 깨진다. 네트워크 없이 gtest 로 검증한다.

#ifndef UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__SRC__MORAI_LINK_CONVERT_HPP_
#define UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__SRC__MORAI_LINK_CONVERT_HPP_

#include "morai_grpc_vendor/morai_link_types.hpp"

#include <morai/actor/actor_control.pb.h>
#include <morai/actor/actor_get.pb.h>
#include <morai/actor/actor_set.pb.h>
#include <morai/common/object_info.pb.h>
#include <morai/common/result.pb.h>
#include <morai/common/type.pb.h>

#include <string>

namespace morai_grpc_vendor::link_convert
{

Vec3 to_vec3(const morai_sim_api::Vector3 & v);

void to_vehicle_state(const morai_sim_api::actor::VehicleState & in, VehicleStateSample * out);

/// found·exact_match·timestamp·수신 시각은 건드리지 않는다 (호출자가 채운다)
void to_ego_sample(const morai_sim_api::actor::ActorState & in, EgoSample * out);

void to_actor_sample(const morai_sim_api::actor::ActorState & in, ActorSample * out);

/// Ego 는 id 가 ego_id 와 정확히 같은 첫 항목, 없으면 대소문자 무시로 같은 첫 항목이다.
/// actors 에서는 main.py 처럼 id 가 ego_id 와 **정확히** 같은 항목을 전부 뺀다. 대소문자만 같은
/// 대체 항목을 ego 로 골랐으면(exact_match false) 그 항목도 뺀다. 송수신 시각·rpc_ms 는 건드리지 않는다.
/// ego.timestamp_us / has_timestamp 는 스냅샷 timestamp 로 채운다 (Ego 가 있을 때).
void to_snapshot(
  const morai_sim_api::actor::ActorStates & in, const std::string & ego_id, Snapshot * out);

// 26.09.26 _ YSH _ MoraiLink 가 요청마다 싣는 Ego 식별자 — 링크 생성자와 원시 기록(vehicle_ctrl_cmd_json)이 같은 것을 쓴다
/// main.py 와 같다: id.value = ego_id, object_type = VEHICLE, client_key
morai_sim_api::ObjectInfo ego_object_info(const std::string & ego_id, const std::string & client_key);

/// actor_info 는 ego 로 덮어쓴다
void to_ctrl_cmd(
  const VehicleCommand & in, const morai_sim_api::ObjectInfo & ego,
  morai_sim_api::actor::VehicleCtrlCmd * out);

void to_set_transform(
  const TransformPose & in, const morai_sim_api::ObjectInfo & ego,
  morai_sim_api::actor::SetTransformParam * out);

VehicleSpecInfo to_spec(const morai_sim_api::actor::VehicleSpec & in);

/// MORAI 응용 계층 결과 반영. status->ok 는 전송 성공(이미 true)일 때만 Result 로 다시 정한다.
/// StatusCode 는 UNKNOWN_FAILURE = 0, SUCCESS = 1 이다
void apply_result(const morai_sim_api::Result & result, const char * rpc_name, Status * status);

/// 되읽은 Ego 가 목표 자세와 수평 위치·yaw 로 일치하는가 (yaw 는 ±180 래핑)
bool transform_matches(
  const EgoSample & ego, const TransformPose & target, double tolerance_m, double tolerance_deg);

}  // namespace morai_grpc_vendor::link_convert

#endif  // UNIVERSE__AUTOWARE_UNIVERSE__SIMULATOR__MORAI_GRPC_VENDOR__SRC__MORAI_LINK_CONVERT_HPP_
