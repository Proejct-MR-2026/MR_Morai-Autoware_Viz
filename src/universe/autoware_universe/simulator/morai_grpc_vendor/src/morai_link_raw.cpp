// 신규 — 26.09.26 _ YSH _ morai_link_raw.hpp 구현 (직렬화 바이트·명령 → proto3 JSON, 값 변환 없음)
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

#include "morai_grpc_vendor/morai_link_raw.hpp"

#include "morai_link_convert.hpp"

#include <google/protobuf/util/json_util.h>
#include <morai/actor/actor_control.pb.h>
#include <morai/actor/actor_get.pb.h>

#include <exception>
#include <string>

namespace morai_grpc_vendor
{

namespace
{

/// 계약(DAQ raw)이 정한 JSON 옵션. 바꾸면 오프라인 도구(daq_export)가 읽는 모양이 바뀐다
google::protobuf::util::JsonPrintOptions raw_json_options()
{
  google::protobuf::util::JsonPrintOptions o;
  o.add_whitespace = false;
  o.always_print_primitive_fields = true;
  o.always_print_enums_as_ints = true;
  o.preserve_proto_field_names = true;
  return o;
}

std::string to_json(const google::protobuf::Message & m)
{
  std::string out;
  if (!google::protobuf::util::MessageToJsonString(m, &out, raw_json_options()).ok()) {
    return std::string();
  }
  return out;
}

}  // namespace

std::string actor_states_json(const std::string & bytes)
{
  try {
    morai_sim_api::actor::ActorStates msg;
    if (!msg.ParseFromString(bytes)) {
      return std::string();
    }
    return to_json(msg);
  } catch (const std::exception &) {
    return std::string();
  }
}

std::string vehicle_ctrl_cmd_json(const VehicleCommand & cmd, const MoraiLinkOptions & options)
{
  try {
    morai_sim_api::actor::VehicleCtrlCmd req;
    link_convert::to_ctrl_cmd(
      cmd, link_convert::ego_object_info(options.ego_id, options.client_key), &req);
    return to_json(req);
  } catch (const std::exception &) {
    return std::string();
  }
}

}  // namespace morai_grpc_vendor
