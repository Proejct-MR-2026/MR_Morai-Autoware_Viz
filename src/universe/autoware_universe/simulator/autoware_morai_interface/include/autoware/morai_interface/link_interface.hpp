// 신규 — 26.09.25 _ YSH _ 엔진이 쓰는 MORAI 링크 추상 (시험용 가짜 링크 주입 + 링크 재생성)
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

#ifndef AUTOWARE__MORAI_INTERFACE__LINK_INTERFACE_HPP_
#define AUTOWARE__MORAI_INTERFACE__LINK_INTERFACE_HPP_

#include <morai_grpc_vendor/morai_link.hpp>
#include <morai_grpc_vendor/morai_link_types.hpp>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace autoware::morai_interface
{

/// 엔진·기어 송신기·작업 스레드가 쓰는 MoraiLink 의 부분 집합.
///
/// 추상화한 이유는 둘이다. (1) 엔진 루프를 네트워크 없이 gtest 로 돌린다 (가짜 링크).
/// (2) 링크를 통째로 새로 만들 수 있어야 한다 — keepalive 가 꺼진 채 TCP 가 RST 없이 죽으면
/// gRPC 채널은 수 분간 READY 로 남고 호출은 DEADLINE_EXCEEDED 만 낸다. 그때는 채널을 새로 만드는
/// 것이 유일한 복구다 (morai_link.hpp keepalive 설명).
///
/// 모든 메서드는 여러 스레드에서 동시에 불러도 된다. 예외는 던지지 않는다.
class ILink
{
public:
  virtual ~ILink() = default;

  virtual morai_grpc_vendor::Status wait_connected(std::chrono::milliseconds timeout) = 0;

  // --- 상태 채널 ---
  virtual morai_grpc_vendor::Reply<int64_t> wait_for_tick(std::chrono::milliseconds deadline) = 0;
  virtual morai_grpc_vendor::Status get_snapshot(morai_grpc_vendor::Snapshot * out) = 0;
  virtual morai_grpc_vendor::Reply<morai_grpc_vendor::ControlMode> get_control_mode() = 0;
  virtual morai_grpc_vendor::Reply<morai_grpc_vendor::VehicleSpecInfo> get_vehicle_spec() = 0;

  // --- 명령 채널 ---
  /// deadline 은 엔진이 왕복에 맞춰 정한다 (main.py cmd_timeout_for_rtt)
  virtual morai_grpc_vendor::Status control_vehicle(
    const morai_grpc_vendor::VehicleCommand & cmd, std::chrono::milliseconds deadline) = 0;
  /// verify > 0 이면 상태 채널로 되읽어 확인한다 (ok = 되읽기 일치)
  virtual morai_grpc_vendor::Status set_gear(
    morai_grpc_vendor::GearMode gear, std::chrono::milliseconds verify) = 0;
  virtual morai_grpc_vendor::Status set_control_mode(
    morai_grpc_vendor::ControlMode mode, std::chrono::milliseconds verify) = 0;
  /// 되읽기는 호출자가 다음 스냅샷으로 한다 (엔진 StateCache)
  virtual morai_grpc_vendor::Status set_transform(const morai_grpc_vendor::TransformPose & pose) = 0;
  virtual morai_grpc_vendor::Status set_tail_light(morai_grpc_vendor::TurnSignal signal) = 0;

  /// 채널을 새로 만든다 (진행 중인 호출은 옛 채널로 끝난다)
  virtual void recreate() = 0;
  virtual morai_grpc_vendor::LinkCounters counters() const = 0;
  /// "주소:포트" (로그·진단용)
  virtual std::string target() const = 0;
};

/// 실제 MoraiLink 로 가는 ILink. 링크는 shared_ptr 을 원자적으로 바꿔 끼운다 — 호출마다 지역
/// 사본을 잡으므로 다른 스레드가 recreate() 해도 진행 중인 호출은 안전하다.
class MoraiLinkClient final : public ILink
{
public:
  explicit MoraiLinkClient(const morai_grpc_vendor::MoraiLinkOptions & options);

  morai_grpc_vendor::Status wait_connected(std::chrono::milliseconds timeout) override;
  morai_grpc_vendor::Reply<int64_t> wait_for_tick(std::chrono::milliseconds deadline) override;
  morai_grpc_vendor::Status get_snapshot(morai_grpc_vendor::Snapshot * out) override;
  morai_grpc_vendor::Reply<morai_grpc_vendor::ControlMode> get_control_mode() override;
  morai_grpc_vendor::Reply<morai_grpc_vendor::VehicleSpecInfo> get_vehicle_spec() override;
  morai_grpc_vendor::Status control_vehicle(
    const morai_grpc_vendor::VehicleCommand & cmd, std::chrono::milliseconds deadline) override;
  morai_grpc_vendor::Status set_gear(
    morai_grpc_vendor::GearMode gear, std::chrono::milliseconds verify) override;
  morai_grpc_vendor::Status set_control_mode(
    morai_grpc_vendor::ControlMode mode, std::chrono::milliseconds verify) override;
  morai_grpc_vendor::Status set_transform(const morai_grpc_vendor::TransformPose & pose) override;
  morai_grpc_vendor::Status set_tail_light(morai_grpc_vendor::TurnSignal signal) override;
  void recreate() override;
  morai_grpc_vendor::LinkCounters counters() const override;
  std::string target() const override;

private:
  std::shared_ptr<morai_grpc_vendor::MoraiLink> current() const;

  const morai_grpc_vendor::MoraiLinkOptions options_;
  std::shared_ptr<morai_grpc_vendor::MoraiLink> link_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__LINK_INTERFACE_HPP_
