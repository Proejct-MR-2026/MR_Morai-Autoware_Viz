// 신규 — 26.09.25 _ YSH _ CommandSlot 구현 (mutex 로 메시지·시각·seq 를 한 번에)
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

#include "autoware/morai_interface/command_slot.hpp"

#include "autoware/morai_interface/conversion/autoware_to_morai.hpp"

namespace autoware::morai_interface
{

void CommandSlot::submit(const autoware_control_msgs::msg::Control & msg, int64_t receive_steady_ns)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  entry_ = Entry{msg, receive_steady_ns, ++seq_};
}

std::optional<CommandSlot::Entry> CommandSlot::latest() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return entry_;
}

void CommandSlot::clear()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  entry_.reset();
}

bool CommandSlot::empty() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return !entry_.has_value();
}

SubmitResult submit_control(
  CommandSlot & slot, const autoware_control_msgs::msg::Control & msg, bool forwarding,
  int64_t receive_steady_ns)
{
  if (!forwarding) {
    slot.clear();
    return SubmitResult::kForwardingOff;
  }
  if (!autoware_to_morai::control_is_finite(msg)) {
    return SubmitResult::kRejectedNonFinite;
  }
  slot.submit(msg, receive_steady_ns);
  return SubmitResult::kAccepted;
}

}  // namespace autoware::morai_interface
