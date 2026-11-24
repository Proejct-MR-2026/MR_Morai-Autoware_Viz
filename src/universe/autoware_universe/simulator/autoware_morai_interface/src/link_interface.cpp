// 신규 — 26.09.25 _ YSH _ MoraiLinkClient — MoraiLink 로 위임하고 링크를 원자적으로 바꿔 끼운다
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

#include "autoware/morai_interface/link_interface.hpp"

#include <memory>
#include <string>

namespace autoware::morai_interface
{

namespace mgv = morai_grpc_vendor;

MoraiLinkClient::MoraiLinkClient(const mgv::MoraiLinkOptions & options)
: options_(options), link_(std::make_shared<mgv::MoraiLink>(options))
{
}

std::shared_ptr<mgv::MoraiLink> MoraiLinkClient::current() const
{
  return std::atomic_load(&link_);
}

mgv::Status MoraiLinkClient::wait_connected(std::chrono::milliseconds timeout)
{
  return current()->wait_connected(timeout);
}

mgv::Reply<int64_t> MoraiLinkClient::wait_for_tick(std::chrono::milliseconds deadline)
{
  return current()->wait_for_tick(deadline);
}

mgv::Status MoraiLinkClient::get_snapshot(mgv::Snapshot * out)
{
  return current()->get_snapshot(out);
}

mgv::Reply<mgv::ControlMode> MoraiLinkClient::get_control_mode()
{
  return current()->get_control_mode();
}

mgv::Reply<mgv::VehicleSpecInfo> MoraiLinkClient::get_vehicle_spec()
{
  return current()->get_vehicle_spec();
}

mgv::Status MoraiLinkClient::control_vehicle(
  const mgv::VehicleCommand & cmd, std::chrono::milliseconds deadline)
{
  return current()->control_vehicle(cmd, deadline);
}

mgv::Status MoraiLinkClient::set_gear(mgv::GearMode gear, std::chrono::milliseconds verify)
{
  return current()->set_gear(gear, verify);
}

mgv::Status MoraiLinkClient::set_control_mode(
  mgv::ControlMode mode, std::chrono::milliseconds verify)
{
  return current()->set_control_mode(mode, verify);
}

mgv::Status MoraiLinkClient::set_transform(const mgv::TransformPose & pose)
{
  return current()->set_transform(pose);
}

mgv::Status MoraiLinkClient::set_tail_light(mgv::TurnSignal signal)
{
  return current()->set_tail_light(signal);
}

void MoraiLinkClient::recreate()
{
  std::atomic_store(&link_, std::make_shared<mgv::MoraiLink>(options_));
}

mgv::LinkCounters MoraiLinkClient::counters() const
{
  return current()->counters();
}

std::string MoraiLinkClient::target() const
{
  return options_.address + ":" + std::to_string(options_.port);
}

}  // namespace autoware::morai_interface
