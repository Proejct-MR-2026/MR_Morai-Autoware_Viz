// 신규 — 26.09.26 _ YSH _ 원시 gRPC 기록 — 받은 ActorStates·보낸 VehicleCtrlCmd 를 proto3 JSON 으로 (grpc·protobuf 비노출)
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

// "gRPC 가 준 값을 그대로 기록" (DAQ raw). 값은 하나도 바꾸지 않는다 — 단위·축·부호 모두 MORAI 원값이다.
// JSON 은 google::protobuf::util::MessageToJsonString 에 옵션 셋을 고정해 찍는다
//   preserve_proto_field_names = true, always_print_primitive_fields = true, always_print_enums_as_ints = true
// 그래서 int64/uint64 는 문자열("timestamp": "1790..."), double 은 되읽으면 같은 값이 되는 자릿수, NaN/Infinity 는
// 문자열 "NaN"/"Infinity" 로 나온다. 값이 기본값인 원시 필드도 나오고, 없는 하위 메시지(vehicle_state 등)는 나오지
// 않는다 — "0 이 왔다" 와 "안 왔다" 가 갈린다. 이 우리 proto 에 없는 필드(MORAI 가 더 새 proto 를 쓸 때)는 JSON 에 없다.
// 변환은 무거울 수 있으니 엔진·명령 스레드가 아니라 기록 스레드(DAQ)에서 부른다. 스레드 안전하다.
// 이 헤더도 grpc·protobuf 를 include 하지 않는다 (test_morai_link_isolation 이 고정).

#ifndef MORAI_GRPC_VENDOR__MORAI_LINK_RAW_HPP_
#define MORAI_GRPC_VENDOR__MORAI_LINK_RAW_HPP_

#include "morai_grpc_vendor/morai_link.hpp"
#include "morai_grpc_vendor/morai_link_types.hpp"

#include <string>

namespace morai_grpc_vendor
{

/// Snapshot::raw_actor_states (GetAllActorsState 응답 ActorStates 의 직렬화 바이트) → proto3 JSON (응답 전체).
/// 바이트가 ActorStates 로 읽히지 않으면 빈 문자열
std::string actor_states_json(const std::string & bytes);

/// MoraiLink::control_vehicle(cmd) 가 이 옵션으로 보내는 VehicleCtrlCmd 요청 그대로 (actor_info 포함) → proto3 JSON.
/// 요청은 링크와 같은 변환(to_ctrl_cmd)·같은 ObjectInfo 로 만든다. 비유한 값도 그대로 찍는다 ("NaN") — 링크는 그런
/// 명령을 보내지 않으므로(grpc_code -1) 기록하는 쪽이 보낸 결과와 함께 적는다. 실패하면 빈 문자열
std::string vehicle_ctrl_cmd_json(const VehicleCommand & cmd, const MoraiLinkOptions & options);

}  // namespace morai_grpc_vendor

#endif  // MORAI_GRPC_VENDOR__MORAI_LINK_RAW_HPP_
