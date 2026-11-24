// 신규 — 26.09.25 _ YSH _ 중복 writer 가드 판정 구현
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

#include "autoware/morai_interface/duplicate_writer_guard.hpp"

#include <algorithm>
#include <string>
#include <vector>

namespace autoware::morai_interface
{

bool is_same_endpoint(const rclcpp::TopicEndpointInfo & info, const rmw_gid_t & self)
{
  const auto & gid = info.endpoint_gid();
  return std::equal(gid.begin(), gid.end(), self.data);
}

std::vector<std::string> other_writers(
  const std::vector<rclcpp::TopicEndpointInfo> & infos, const rmw_gid_t & self_gid,
  const std::string & self_name, const std::string & self_namespace)
{
  size_t self_index = infos.size();
  for (size_t i = 0; i < infos.size(); ++i) {
    if (is_same_endpoint(infos[i], self_gid)) {
      self_index = i;
      break;
    }
  }
  if (self_index == infos.size()) {
    for (size_t i = 0; i < infos.size(); ++i) {
      if (infos[i].node_name() == self_name && infos[i].node_namespace() == self_namespace) {
        self_index = i;
        break;
      }
    }
  }
  std::vector<std::string> out;
  for (size_t i = 0; i < infos.size(); ++i) {
    if (i == self_index) {
      continue;
    }
    std::string ns = infos[i].node_namespace();
    if (ns.empty() || ns.back() != '/') {
      ns += "/";
    }
    out.push_back(ns + infos[i].node_name());
  }
  return out;
}

std::string join_names(const std::vector<std::string> & names)
{
  std::string out;
  for (const auto & n : names) {
    out += (out.empty() ? "" : ", ") + n;
  }
  return out;
}

}  // namespace autoware::morai_interface
