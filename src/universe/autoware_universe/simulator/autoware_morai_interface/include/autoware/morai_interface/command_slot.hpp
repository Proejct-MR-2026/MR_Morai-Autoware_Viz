// 신규 — 26.09.25 _ YSH _ 최신 Autoware 제어 명령 1 칸 (콜백은 넣기만, 엔진이 업데이트마다 꺼내 보낸다)
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

#ifndef AUTOWARE__MORAI_INTERFACE__COMMAND_SLOT_HPP_
#define AUTOWARE__MORAI_INTERFACE__COMMAND_SLOT_HPP_

#include <autoware_control_msgs/msg/control.hpp>

#include <cstdint>
#include <mutex>
#include <optional>

namespace autoware::morai_interface
{

/// main.py _CmdResender 의 슬롯과 같은 역할. executor 콜백은 submit 만 하고 RPC 를 부르지 않는다.
/// 엔진 스레드가 MORAI 업데이트마다 latest() 를 꺼내 명령을 한 번 보낸다. seq 는 새 메시지마다
/// 1 씩 늘어 엔진이 "같은 메시지를 다시 보내는 중" 인지 가린다.
class CommandSlot
{
public:
  struct Entry
  {
    autoware_control_msgs::msg::Control msg;
    int64_t receive_steady_ns{0};  ///< 콜백이 받은 시각 (steady) — stale 판정 기준
    uint64_t seq{0};
  };

  void submit(const autoware_control_msgs::msg::Control & msg, int64_t receive_steady_ns);
  std::optional<Entry> latest() const;
  /// forward_control_to_sim=false 면 비운다 — 다시 켜도 다음 메시지가 올 때까지 보내지 않는다
  void clear();
  bool empty() const;

private:
  mutable std::mutex mutex_;
  std::optional<Entry> entry_;
  uint64_t seq_{0};
};

/// control_cmd 콜백의 판단 (노드가 쓰고 시험이 직접 부른다)
enum class SubmitResult {
  kAccepted,           ///< 슬롯에 넣었다
  kForwardingOff,      ///< forward_control_to_sim=false — 슬롯을 비웠다
  kRejectedNonFinite,  ///< NaN·inf — 버렸다. 슬롯의 마지막 유효 명령은 그대로 (나이가 차면 stale 감속)
};
SubmitResult submit_control(
  CommandSlot & slot, const autoware_control_msgs::msg::Control & msg, bool forwarding,
  int64_t receive_steady_ns);

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__COMMAND_SLOT_HPP_
