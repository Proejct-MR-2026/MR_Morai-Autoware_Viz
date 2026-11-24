// 신규 — 26.09.25 _ YSH _ 중복 writer 가드의 순수 판정 — 자기 publisher 는 하나만 뺀다 (이름으로 전부 빼면 같은 이름의 두 번째 인스턴스가 안 보인다)
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

#ifndef AUTOWARE__MORAI_INTERFACE__DUPLICATE_WRITER_GUARD_HPP_
#define AUTOWARE__MORAI_INTERFACE__DUPLICATE_WRITER_GUARD_HPP_

#include <rclcpp/node_interfaces/node_graph_interface.hpp>
#include <rmw/types.h>

#include <string>
#include <vector>

namespace autoware::morai_interface
{

/// endpoint 가 이 publisher(self) 인가 — GID 바이트 비교
bool is_same_endpoint(const rclcpp::TopicEndpointInfo & info, const rmw_gid_t & self);

/// 자기 publisher 하나만 뺀 나머지 publisher 들의 "/ns/name" 목록 (순서 유지).
///
/// 자기는 **딱 하나**만 뺀다: GID 가 같은 endpoint, 없으면 노드 이름·namespace 가 같은 첫 endpoint.
/// GID 만으로는 안 된다 — rmw_cyclonedds_cpp(Humble) 에서 Publisher::get_gid() 는 graph 의 endpoint GID 와
/// 다른 값이다 (실측: 8 바이트 instance handle ↔ 16 바이트 GUID). 이름으로 **전부** 빼면 같은 이름의 두 번째
/// 인스턴스(main.py /morai_bridge 와 node_name:=morai_bridge 로 띄운 이 노드, 또는 이 노드 두 개)가 안 보인다.
std::vector<std::string> other_writers(
  const std::vector<rclcpp::TopicEndpointInfo> & infos, const rmw_gid_t & self_gid,
  const std::string & self_name, const std::string & self_namespace);

/// ", " 로 잇는다 (비면 "")
std::string join_names(const std::vector<std::string> & names);

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__DUPLICATE_WRITER_GUARD_HPP_
