// 신규 — 26.09.25 _ YSH _ morai_link_convert.hpp 구현 (proto <-> 공개 구조체, 값 변환 없음)
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

#include "morai_link_convert.hpp"

#include <morai/actor/actor_enum.pb.h>
#include <morai/common/enum.pb.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <string>

namespace morai_grpc_vendor::link_convert
{

namespace
{

bool iequals(const std::string & a, const std::string & b)
{
  return a.size() == b.size() && std::equal(a.begin(), a.end(), b.begin(), [](char x, char y) {
           return std::tolower(static_cast<unsigned char>(x)) ==
                  std::tolower(static_cast<unsigned char>(y));
         });
}

/// 각도 차 [deg] 를 [-180, 180] 으로
double wrap_deg(double d)
{
  d = std::fmod(d, 360.0);
  if (d > 180.0) {
    d -= 360.0;
  } else if (d < -180.0) {
    d += 360.0;
  }
  return d;
}

}  // namespace

Vec3 to_vec3(const morai_sim_api::Vector3 & v)
{
  return Vec3{v.x(), v.y(), v.z()};
}

void to_vehicle_state(const morai_sim_api::actor::VehicleState & in, VehicleStateSample * out)
{
  out->present = true;
  out->throttle = static_cast<double>(in.throttle());
  out->brake = static_cast<double>(in.brake());
  out->steer = static_cast<double>(in.steer());
  out->front_wheel_angle = static_cast<double>(in.front_wheel_angle());
  out->gear_mode = static_cast<GearMode>(in.gear_mode());
  out->collision_count = static_cast<size_t>(in.collision_objects_size());
  out->collision_objects.assign(in.collision_objects().begin(), in.collision_objects().end());
}

void to_ego_sample(const morai_sim_api::actor::ActorState & in, EgoSample * out)
{
  out->id = in.actor_info().id().value();
  out->object_type = static_cast<ObjectType>(in.actor_info().object_type());
  out->position = to_vec3(in.transform().location());
  out->rotation_deg = to_vec3(in.transform().rotation());
  out->velocity_kmh = to_vec3(in.velocity());
  out->angular_velocity_raw = to_vec3(in.angular_velocity());
  out->acceleration_raw = to_vec3(in.acceleration());
  out->global_velocity_raw = to_vec3(in.global_velocity());
  out->valid = ego_is_valid(out->position, out->rotation_deg, out->velocity_kmh);
  if (in.has_vehicle_state()) {
    to_vehicle_state(in.vehicle_state(), &out->vehicle_state);
  } else {
    out->vehicle_state = VehicleStateSample{};
  }
}

void to_actor_sample(const morai_sim_api::actor::ActorState & in, ActorSample * out)
{
  out->id = in.actor_info().id().value();
  out->object_type = static_cast<ObjectType>(in.actor_info().object_type());
  out->client_key = in.actor_info().client_key();
  out->position = to_vec3(in.transform().location());
  out->rotation_deg = to_vec3(in.transform().rotation());
  out->velocity_kmh = to_vec3(in.velocity());
  out->angular_velocity_raw = to_vec3(in.angular_velocity());
  out->acceleration_raw = to_vec3(in.acceleration());
  out->global_velocity_raw = to_vec3(in.global_velocity());
  out->bbox_extent = to_vec3(in.bounding_box().extent());
  out->bbox_location = to_vec3(in.bounding_box().location());
  out->bbox_rotation = to_vec3(in.bounding_box().rotation());
  out->has_vehicle_state = in.has_vehicle_state();
  if (out->has_vehicle_state) {
    to_vehicle_state(in.vehicle_state(), &out->vehicle_state);
  } else {
    out->vehicle_state = VehicleStateSample{};
  }
}

void to_snapshot(
  const morai_sim_api::actor::ActorStates & in, const std::string & ego_id, Snapshot * out)
{
  out->timestamp_us = in.timestamp();
  out->sync_mode_tick_count = in.sync_mode_tick_count();

  // Ego 고르기: 정확 일치 첫 항목, 없으면 대소문자 무시 첫 항목 (morai_probe frames 와 같은 규칙).
  // actors 에서는 main.py publish_tracked_objects 처럼 id 가 ego_id 와 정확히 같은 항목을 **전부**
  // 뺀다. 대소문자만 같은 대체 항목을 ego 로 골랐으면 그 항목도 뺀다 — 남기면 자차가 base_link 에
  // 장애물로 발행된다 (리뷰 반영)
  int ego_index = -1;
  bool exact = false;
  size_t n_exact = 0;
  for (int i = 0; i < in.states_size(); ++i) {
    if (in.states(i).actor_info().id().value() == ego_id) {
      if (ego_index < 0) {
        ego_index = i;
        exact = true;
      }
      ++n_exact;
    }
  }
  if (ego_index < 0) {
    for (int i = 0; i < in.states_size(); ++i) {
      if (iequals(in.states(i).actor_info().id().value(), ego_id)) {
        ego_index = i;
        break;
      }
    }
  }

  out->ego = EgoSample{};
  if (ego_index >= 0) {
    to_ego_sample(in.states(ego_index), &out->ego);
    out->ego.found = true;
    out->ego.exact_match = exact;
    out->ego.timestamp_us = in.timestamp();
    out->ego.has_timestamp = true;
  }

  // resize 후 덮어쓰기 — 업데이트마다 문자열·벡터 버퍼를 재사용한다
  const size_t n_removed = n_exact + ((ego_index >= 0 && !exact) ? 1U : 0U);
  out->actors.resize(static_cast<size_t>(in.states_size()) - n_removed);
  size_t k = 0;
  for (int i = 0; i < in.states_size(); ++i) {
    if (in.states(i).actor_info().id().value() == ego_id || (i == ego_index && !exact)) {
      continue;
    }
    to_actor_sample(in.states(i), &out->actors[k++]);
  }
}

// 26.09.26 _ YSH _ 링크 생성자에 있던 Ego 식별자 만들기를 옮겼다 (원시 기록도 같은 것을 쓴다)
morai_sim_api::ObjectInfo ego_object_info(const std::string & ego_id, const std::string & client_key)
{
  morai_sim_api::ObjectInfo ego;
  ego.mutable_id()->set_value(ego_id);
  ego.set_object_type(morai_sim_api::OBJECT_TYPE_VEHICLE);
  ego.set_client_key(client_key);
  return ego;
}

void to_ctrl_cmd(
  const VehicleCommand & in, const morai_sim_api::ObjectInfo & ego,
  morai_sim_api::actor::VehicleCtrlCmd * out)
{
  *out->mutable_actor_info() = ego;
  out->set_long_cmd_type(static_cast<morai_sim_api::actor::LongCmdType>(in.long_cmd_type));
  out->set_throttle(in.throttle);
  out->set_brake(in.brake);
  out->set_steer(in.steer);
  out->set_velocity(in.velocity);
  out->set_acceleration(in.acceleration);
  out->set_frame(in.frame);
}

void to_set_transform(
  const TransformPose & in, const morai_sim_api::ObjectInfo & ego,
  morai_sim_api::actor::SetTransformParam * out)
{
  *out->mutable_actor_info() = ego;
  auto * loc = out->mutable_transform()->mutable_location();
  loc->set_x(in.position.x);
  loc->set_y(in.position.y);
  loc->set_z(in.position.z);
  auto * rot = out->mutable_transform()->mutable_rotation();
  rot->set_x(in.rotation_deg.x);
  rot->set_y(in.rotation_deg.y);
  rot->set_z(in.rotation_deg.z);
}

VehicleSpecInfo to_spec(const morai_sim_api::actor::VehicleSpec & in)
{
  VehicleSpecInfo s;
  s.sub_category = static_cast<int32_t>(in.sub_category());
  s.length_m = static_cast<double>(in.length());
  s.width_m = static_cast<double>(in.width());
  s.height_m = static_cast<double>(in.height());
  s.wheel_base_m = static_cast<double>(in.wheel_base());
  s.front_overhang_m = static_cast<double>(in.front_overhang());
  s.rear_overhang_m = static_cast<double>(in.rear_overhang());
  s.max_steering_angle_deg = static_cast<double>(in.max_steering_angle());
  s.wheel_radius_m = static_cast<double>(in.wheel_radius());
  return s;
}

void apply_result(const morai_sim_api::Result & result, const char * rpc_name, Status * status)
{
  status->result_status = static_cast<int32_t>(result.status());
  if (result.status() == morai_sim_api::STATUS_CODE_SUCCESS) {
    return;
  }
  status->ok = false;
  std::string msg = std::string(rpc_name) + " rejected by MORAI (Result.status=" +
                    std::to_string(static_cast<int>(result.status())) + ")";
  if (!result.description().empty()) {
    msg += " description='" + result.description().substr(0, 300) + "'";
  }
  if (!result.custom_message().empty()) {
    msg += " custom='" + result.custom_message().substr(0, 300) + "'";
  }
  status->message = msg;
}

bool transform_matches(
  const EgoSample & ego, const TransformPose & target, double tolerance_m, double tolerance_deg)
{
  if (!ego.found || !ego.valid) {
    return false;
  }
  return std::fabs(ego.position.x - target.position.x) <= tolerance_m &&
         std::fabs(ego.position.y - target.position.y) <= tolerance_m &&
         std::fabs(wrap_deg(ego.rotation_deg.z - target.rotation_deg.z)) <= tolerance_deg;
}

}  // namespace morai_grpc_vendor::link_convert
