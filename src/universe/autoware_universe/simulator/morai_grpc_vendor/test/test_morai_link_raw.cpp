// 신규 — 26.09.26 _ YSH _ 원시 gRPC 기록 gtest — 직렬화 바이트 → JSON → JsonStringToMessage 가 원래 메시지와 같은가
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

// 고정하는 것: 값이 바뀌지 않는다 (0.1, 1e-300, 실측 크기의 µs timestamp, NaN, float32 필드) · 없는 vehicle_state 는
// JSON 에도 없다 · 기본값 원시 필드도 찍힌다 · 필드 이름은 proto 이름 · enum 은 정수 (proto 에 없는 값도) ·
// int64/uint64 는 문자열 · 보낸 VehicleCtrlCmd 는 링크가 만드는 요청과 같다 · 깨진 바이트는 빈 문자열.
// -0.0 은 값만 남고 부호는 남지 않는다: protobuf 3.12 직렬화는 proto3 double 을 '!= 0' 으로 걸러 -0.0 을 필드째 뺀다 —
// 받은 응답을 다시 직렬화하는 순간 0 이 된다 (0.0 == -0.0 이라 값은 같다). 아래 시험이 이 동작을 고정한다

#include "morai_grpc_vendor/morai_link.hpp"
#include "morai_grpc_vendor/morai_link_raw.hpp"
#include "morai_grpc_vendor/morai_link_types.hpp"
#include "morai_link_convert.hpp"

#include <google/protobuf/util/field_comparator.h>
#include <google/protobuf/util/json_util.h>
#include <google/protobuf/util/message_differencer.h>
#include <gtest/gtest.h>
#include <morai/actor/actor_control.pb.h>
#include <morai/actor/actor_enum.pb.h>
#include <morai/actor/actor_get.pb.h>
#include <morai/common/enum.pb.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

namespace
{

namespace mv = morai_grpc_vendor;
namespace lc = morai_grpc_vendor::link_convert;
using google::protobuf::util::DefaultFieldComparator;
using google::protobuf::util::JsonStringToMessage;
using google::protobuf::util::MessageDifferencer;
using morai_sim_api::actor::ActorState;
using morai_sim_api::actor::ActorStates;
using morai_sim_api::actor::VehicleCtrlCmd;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
/// 실측 ActorStates.timestamp 와 같은 크기의 Unix epoch µs (2^53 보다 작지만 double 로 옮기면 안 되는 값)
constexpr int64_t kEpochUs = 1790000000123457;

void set_vec(morai_sim_api::Vector3 * v, double x, double y, double z)
{
  v->set_x(x);
  v->set_y(y);
  v->set_z(z);
}

/// Ego (vehicle_state 있음) + 장애물 (vehicle_state 없음). 까다로운 double 을 여러 필드에 둔다
ActorStates tricky_states()
{
  ActorStates s;
  s.set_timestamp(kEpochUs);
  s.set_sync_mode_tick_count(18446744073709551615ULL);  // uint64 최댓값 — 문자열로만 온전히 간다

  ActorState * ego = s.add_states();
  ego->mutable_actor_info()->mutable_id()->set_value("Ego");
  ego->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
  set_vec(ego->mutable_transform()->mutable_location(), 0.1, -0.0, 1e-300);
  set_vec(ego->mutable_transform()->mutable_rotation(), 0.0, 359.99999999999994, -179.123456789012345);
  set_vec(ego->mutable_velocity(), 72.00000000000001, kNaN, 4.9e-324);  // 최소 준정규수까지
  set_vec(ego->mutable_angular_velocity(), 1.0 / 3.0, -2.0 / 3.0, 1e300);
  set_vec(ego->mutable_acceleration(), 0.1 + 0.2, -0.3, 0.0);
  set_vec(ego->mutable_global_velocity(), -20.0, 0.5, 1e-7);
  set_vec(ego->mutable_bounding_box()->mutable_extent(), 1.06, 2.32, 0.75);
  auto * vs = ego->mutable_vehicle_state();
  vs->set_throttle(0.1F);  // float32 — JSON 은 float 의 최단 표기, 되읽으면 같은 float
  vs->set_brake(0.003F);
  vs->set_steer(-0.4F);
  vs->set_front_wheel_angle(std::nanf(""));
  vs->set_gear_mode(static_cast<morai_sim_api::actor::GearMode>(9));  // proto 에 없는 값도 정수 그대로
  vs->mutable_current_link_info()->mutable_id()->set_value("A219BS010345");
  vs->mutable_current_link_info()->set_waypoint_idx(17);
  vs->set_remaining_distance(123.5F);
  vs->set_remaining_link_count(3);
  vs->set_is_pass_des_pos(false);
  vs->add_collision_objects("Npc_9");

  ActorState * cone = s.add_states();
  cone->mutable_actor_info()->mutable_id()->set_value("Obstacle_3");
  cone->mutable_actor_info()->set_object_type(morai_sim_api::OBJECT_TYPE_OBSTACLE);
  cone->mutable_actor_info()->set_client_key("");
  set_vec(cone->mutable_transform()->mutable_location(), 120.25, -0.1, 0.0);
  set_vec(cone->mutable_bounding_box()->mutable_extent(), 0.2, 0.2, 0.5);
  // vehicle_state 없음 — JSON 에 키가 없어야 한다 ("0 이 왔다" 와 가른다)
  return s;
}

/// NaN 을 같다고 보는 것만 빼고 기본(EXACT) 비교
bool same(const google::protobuf::Message & a, const google::protobuf::Message & b, std::string * diff)
{
  DefaultFieldComparator cmp;
  cmp.set_float_comparison(DefaultFieldComparator::EXACT);
  cmp.set_treat_nan_as_equal(true);
  MessageDifferencer d;
  d.set_field_comparator(&cmp);
  d.ReportDifferencesToString(diff);
  return d.Compare(a, b);
}

TEST(MoraiLinkRaw, ActorStatesSurviveBytesToJsonToMessage)
{
  const ActorStates original = tricky_states();
  std::string bytes;
  ASSERT_TRUE(original.SerializeToString(&bytes));

  const std::string json = mv::actor_states_json(bytes);
  ASSERT_FALSE(json.empty());

  ActorStates back;
  ASSERT_TRUE(JsonStringToMessage(json, &back).ok()) << json;
  std::string diff;
  EXPECT_TRUE(same(original, back, &diff)) << diff << "\n" << json;
  // MessageDifferencer 가 놓치지 않게 까다로운 값은 비트 단위로도 본다
  EXPECT_EQ(back.timestamp(), kEpochUs);
  EXPECT_EQ(back.sync_mode_tick_count(), 18446744073709551615ULL);
  const auto & ego = back.states(0);
  EXPECT_EQ(ego.transform().location().x(), 0.1);
  EXPECT_EQ(ego.transform().location().z(), 1e-300);
  EXPECT_EQ(ego.transform().rotation().y(), 359.99999999999994);
  EXPECT_EQ(ego.velocity().x(), 72.00000000000001);
  EXPECT_TRUE(std::isnan(ego.velocity().y()));
  EXPECT_EQ(ego.velocity().z(), 4.9e-324);
  EXPECT_EQ(ego.angular_velocity().x(), 1.0 / 3.0);
  EXPECT_EQ(ego.acceleration().x(), 0.1 + 0.2);
  EXPECT_EQ(ego.vehicle_state().throttle(), 0.1F);
  EXPECT_TRUE(std::isnan(ego.vehicle_state().front_wheel_angle()));
  EXPECT_EQ(static_cast<int>(ego.vehicle_state().gear_mode()), 9);
  EXPECT_EQ(ego.vehicle_state().current_link_info().id().value(), "A219BS010345");
  // -0.0: 값은 같다 (0.0 == -0.0). 부호는 다시 직렬화할 때 빠진다 (위 머리 주석)
  EXPECT_EQ(ego.transform().location().y(), 0.0);
  EXPECT_FALSE(std::signbit(ego.transform().location().y()));
  EXPECT_FALSE(back.states(1).has_vehicle_state());
}

TEST(MoraiLinkRaw, ActorStatesJsonUsesTheContractOptions)
{
  std::string bytes;
  ASSERT_TRUE(tricky_states().SerializeToString(&bytes));
  const std::string json = mv::actor_states_json(bytes);

  // int64/uint64 는 문자열 (JSON 숫자로는 2^53 넘는 값이 깨진다)
  EXPECT_NE(json.find("\"timestamp\":\"1790000000123457\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"sync_mode_tick_count\":\"18446744073709551615\""), std::string::npos) << json;
  // proto 필드 이름 그대로 (lowerCamelCase 아님)
  EXPECT_NE(json.find("\"actor_info\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"global_velocity\""), std::string::npos) << json;
  EXPECT_EQ(json.find("\"actorInfo\""), std::string::npos) << json;
  // enum 은 정수 (proto 에 없는 값도)
  EXPECT_NE(json.find("\"object_type\":1"), std::string::npos) << json;
  EXPECT_NE(json.find("\"object_type\":3"), std::string::npos) << json;
  EXPECT_NE(json.find("\"gear_mode\":9"), std::string::npos) << json;
  // 기본값 원시 필드도 찍힌다 (0 이 온 것과 없는 것을 가른다)
  EXPECT_NE(json.find("\"is_pass_des_pos\":false"), std::string::npos) << json;
  EXPECT_NE(json.find("\"client_key\":\"\""), std::string::npos) << json;
  // 비유한 double 은 문자열, 까다로운 유한 값은 되읽으면 같은 표기
  EXPECT_NE(json.find("\"y\":\"NaN\""), std::string::npos) << json;
  EXPECT_NE(json.find("\"x\":0.1,"), std::string::npos) << json;
  EXPECT_NE(json.find("\"location\":{\"x\":0.1,\"y\":0,\"z\":1e-300}"), std::string::npos) << json;
  EXPECT_NE(json.find("\"x\":0.30000000000000004"), std::string::npos) << json;
  // 되읽으면 같은 값이 되는 17 자리 (가장 짧은 표기는 아니다 — 72.00000000000001 이 72.000000000000014 로)
  EXPECT_NE(json.find("\"x\":72.000000000000014"), std::string::npos) << json;
  // vehicle_state 는 Ego 에만 한 번
  const auto first = json.find("\"vehicle_state\"");
  ASSERT_NE(first, std::string::npos);
  EXPECT_EQ(json.find("\"vehicle_state\"", first + 1), std::string::npos) << json;
  // 한 줄 (공백·줄바꿈 없음)
  EXPECT_EQ(json.find('\n'), std::string::npos);
}

TEST(MoraiLinkRaw, EmptyResponseAndBrokenBytes)
{
  // 빈 응답(actor 없음, timestamp 0)도 온전한 ActorStates 다 — 기본값이 전부 찍힌다
  const std::string empty = mv::actor_states_json(std::string());
  EXPECT_EQ(empty, "{\"states\":[],\"sync_mode_tick_count\":\"0\",\"timestamp\":\"0\"}");
  // 깨진 바이트 (길이 접두가 남는 바이트보다 길다)
  EXPECT_TRUE(mv::actor_states_json(std::string("\x0a\x7f\x01", 3)).empty());
}

TEST(MoraiLinkRaw, VehicleCtrlCmdJsonIsTheRequestTheLinkSends)
{
  mv::MoraiLinkOptions options;
  options.ego_id = "Ego";
  options.client_key = "bridge";
  mv::VehicleCommand cmd;
  cmd.long_cmd_type = mv::LongCmdType::kAcceleration;
  cmd.throttle = 0.0;
  cmd.brake = -0.0;
  cmd.steer = 0.1;
  cmd.velocity = 36.00000000000001;
  cmd.acceleration = -2.4;
  cmd.frame = 0;

  const std::string json = mv::vehicle_ctrl_cmd_json(cmd, options);
  ASSERT_FALSE(json.empty());
  VehicleCtrlCmd back;
  ASSERT_TRUE(JsonStringToMessage(json, &back).ok()) << json;

  // 링크(MoraiLink::control_vehicle)가 만드는 요청과 같은 변환·같은 ObjectInfo
  VehicleCtrlCmd sent;
  lc::to_ctrl_cmd(cmd, lc::ego_object_info(options.ego_id, options.client_key), &sent);
  std::string diff;
  EXPECT_TRUE(same(sent, back, &diff)) << diff << "\n" << json;
  EXPECT_EQ(back.actor_info().id().value(), "Ego");
  EXPECT_EQ(back.actor_info().client_key(), "bridge");
  EXPECT_EQ(back.steer(), 0.1);
  EXPECT_EQ(back.velocity(), 36.00000000000001);
  EXPECT_NE(json.find("\"long_cmd_type\":3"), std::string::npos) << json;
  EXPECT_NE(json.find("\"object_type\":1"), std::string::npos) << json;
  EXPECT_NE(json.find("\"frame\":0"), std::string::npos) << json;
  EXPECT_NE(json.find("\"acceleration\":-2.4"), std::string::npos) << json;

  // 비유한 명령 — 링크는 보내지 않지만 기록은 그 값 그대로
  cmd.acceleration = kNaN;
  cmd.steer = std::numeric_limits<double>::infinity();
  const std::string nan_json = mv::vehicle_ctrl_cmd_json(cmd, options);
  EXPECT_NE(nan_json.find("\"acceleration\":\"NaN\""), std::string::npos) << nan_json;
  EXPECT_NE(nan_json.find("\"steer\":\"Infinity\""), std::string::npos) << nan_json;
}

}  // namespace
