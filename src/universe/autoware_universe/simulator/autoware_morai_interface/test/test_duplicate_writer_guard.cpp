// 신규 — 26.09.25 _ YSH _ 중복 writer 가드 판정 시험 (자기 publisher 는 하나만 뺀다)
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

#include <rmw/topic_endpoint_info.h>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace autoware::morai_interface
{
namespace
{

rmw_gid_t gid(uint8_t seed)
{
  rmw_gid_t g{};
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    g.data[i] = static_cast<uint8_t>(seed + i);
  }
  return g;
}

rclcpp::TopicEndpointInfo endpoint(const char * name, const char * ns, const rmw_gid_t & g)
{
  rmw_topic_endpoint_info_t info = rmw_get_zero_initialized_topic_endpoint_info();
  info.node_name = name;
  info.node_namespace = ns;
  info.topic_type = "autoware_vehicle_msgs/msg/VelocityReport";
  info.endpoint_type = RMW_ENDPOINT_PUBLISHER;
  for (size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
    info.endpoint_gid[i] = g.data[i];
  }
  return rclcpp::TopicEndpointInfo(info);
}

TEST(DuplicateWriterGuard, SameNamedSecondInstanceIsSeenWhenGidsDoNotMatch)
{
  // rmw_cyclonedds_cpp(Humble): Publisher::get_gid() 는 graph 의 endpoint GID 와 다르다 (실측)
  const rmw_gid_t self = gid(1);
  const std::vector<rclcpp::TopicEndpointInfo> infos{
    endpoint("morai_bridge", "/", gid(40)),  // 자기 (graph GID 는 다른 값)
    endpoint("morai_bridge", "/", gid(50)),  // 같은 이름의 main.py (node_name:=morai_bridge)
  };
  const auto others = other_writers(infos, self, "morai_bridge", "/");
  ASSERT_EQ(others.size(), 1U) << "a same-named second instance must not be invisible";
  EXPECT_EQ(others[0], "/morai_bridge");
}

TEST(DuplicateWriterGuard, GidMatchIsPreferredWhenTheRmwProvidesIt)
{
  const rmw_gid_t self = gid(1);
  const std::vector<rclcpp::TopicEndpointInfo> infos{
    endpoint("morai_bridge", "/", gid(50)),  // 같은 이름의 다른 인스턴스가 먼저 나와도
    endpoint("morai_bridge", "/", self),
  };
  const auto others = other_writers(infos, self, "morai_bridge", "/");
  ASSERT_EQ(others.size(), 1U);
}

TEST(DuplicateWriterGuard, AloneMeansNoOthers)
{
  const rmw_gid_t self = gid(7);
  EXPECT_TRUE(other_writers({endpoint("morai_bridge", "/", self)}, self, "morai_bridge", "/").empty());
  EXPECT_TRUE(other_writers({endpoint("morai_bridge", "/", gid(9))}, self, "morai_bridge", "/").empty())
    << "own endpoint found by name when the GIDs differ";
  EXPECT_TRUE(other_writers({}, self, "morai_bridge", "/").empty());
}

TEST(DuplicateWriterGuard, NamesAreFullyQualified)
{
  const rmw_gid_t self = gid(1);
  const auto others = other_writers(
    {endpoint("morai_bridge", "/", gid(9)), endpoint("morai_interface", "/", gid(2)),
     endpoint("relay", "/sim", gid(3))},
    self, "morai_bridge", "/");
  EXPECT_EQ(join_names(others), "/morai_interface, /sim/relay");
}

}  // namespace
}  // namespace autoware::morai_interface
