// 신규 — 26.09.25 _ YSH _ MORAI gRPC 링크 계층 (상태·명령 두 채널, PIMPL, grpc·protobuf 비노출)
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

// 엔진(autoware_morai_interface)이 쓸 전송 계층. 한 업데이트에 GetAllActorsState 한 번으로
// Ego+NPC+timestamp 를 받고, ControlVehicle 은 업데이트당 정확히 한 번 보내는 구조를 전제로 한다
// (실측: 비동기 200 Hz 몰아 보내기에서 MORAI 가속 제어기가 진동). 주기·트리거 정책은 여기 없다 —
// 루프 본체 on_update(snapshot) 와 트리거(WaitForTick / 이후 lockstep Tick)를 엔진이 분리한다.
//
// 기존 ActorClient / SensorClient 는 그대로 둔다 (AMI 이전은 별도 단계).

#ifndef MORAI_GRPC_VENDOR__MORAI_LINK_HPP_
#define MORAI_GRPC_VENDOR__MORAI_LINK_HPP_

#include "morai_grpc_vendor/morai_link_types.hpp"

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

namespace morai_grpc_vendor
{

/// 호출 종류별 deadline. 기본값은 main.py 의 실전 값을 따른다.
struct MoraiLinkDeadlines
{
  std::chrono::milliseconds state{100};       ///< GetAllActorsState / GetActorState
  std::chrono::milliseconds control_mode{100};  ///< GetVehicleControlMode
  std::chrono::milliseconds command{50};      ///< ControlVehicle (main.py cmd_rpc_timeout_s 0.05)
  std::chrono::milliseconds gear{200};        ///< SetVehicleGear (gear_rpc_timeout_s 0.2)
  std::chrono::milliseconds set_mode{200};    ///< SetVehicleControlMode
  std::chrono::milliseconds transform{1000};  ///< SetTransform (SET_TRANSFORM_TIMEOUT_S 1.0)
  std::chrono::milliseconds tail_light{200};  ///< SetVehicleTailLight
  std::chrono::milliseconds spec{3000};       ///< GetVehicleSpec (main.py timeout=3.0)
  std::chrono::milliseconds time_probe{100};  ///< Simulator.GetTimestamp (프레임 시각)
  std::chrono::milliseconds sync_mode{200};   ///< Simulation.GetSynchronousMode
};

struct MoraiLinkOptions
{
  std::string address{"127.0.0.1"};
  uint16_t port{7789};

  /// ObjectInfo.id.value. main.py 와 같이 "Ego" (숫자가 아니라 문자열)
  std::string ego_id{"Ego"};
  /// ObjectInfo.client_key. main.py 는 비워 둔다 — 그대로 비워 둔다
  std::string client_key;
  /// GetAllActorsFilter.client_key. proto: 비어 있으면 MORAI 가 관리하는 id 로 준다.
  /// 채우면 actor id 체계가 바뀔 수 있으므로 main.py 처럼 비워 둔다
  std::string filter_client_key;
  /// GetAllActorsFilter — main.py 와 같이 전부 true
  bool include_vehicles{true};
  bool include_pedestrians{true};
  bool include_obstacles{true};

  MoraiLinkDeadlines deadlines;

  /// gRPC 재연결 백오프 상한. 기본(최대 120 s)이면 MORAI 재기동 뒤 최대 2 분 동안 붙지 않는다
  int32_t max_reconnect_backoff_ms{1000};
  int32_t min_reconnect_backoff_ms{100};

  /// gRPC keepalive (두 채널 공통). 0 = 끔 (기본). 원격 MORAI 에서 소리 없이 죽은 TCP(케이블·스위치
  /// 단절, RST 없음)는 keepalive 가 없으면 커널 재전송 한도(수 분~15 분)까지 READY 로 남고 호출은
  /// DEADLINE_EXCEEDED 만 낸다. 켜면 keepalive_timeout_ms 안에 PING 응답이 없을 때 연결을 끊고
  /// 재연결(백오프 상한 위)한다. MORAI 서버의 PING 정책은 아직 모른다 — 너무 잦으면 서버가
  /// GOAWAY(too_many_pings)로 끊으므로 켜기 전에 실측으로 확인한다 (예: 5000 / 1000).
  /// 끈 채로도 엔진은 연속 DEADLINE_EXCEEDED 가 이어지면 MoraiLink 를 새로 만들어 복구할 수 있다
  int32_t keepalive_time_ms{0};
  int32_t keepalive_timeout_ms{1000};

  /// 되읽기 확인(set_* 의 verify_timeout > 0) 중 상태 채널을 읽는 간격
  std::chrono::milliseconds readback_poll{10};

  // 26.09.26 _ YSH _ 원시 gRPC 기록 (DAQ raw). true 면 get_snapshot 이 받은 ActorStates 를 직렬화한 바이트를
  //   Snapshot::raw_actor_states 에 호출마다 새 버퍼로 싣는다 (JSON 변환은 쓰는 쪽 스레드가 morai_link_raw.hpp 로)
  bool capture_raw{false};
};

enum class LinkChannel : int32_t {
  kState = 0,    ///< WaitForTick, GetAllActorsState, 모든 Get*
  kCommand = 1,  ///< ControlVehicle, SetVehicleGear, SetTransform, SetVehicleControlMode, SetVehicleTailLight
};

/// grpc_connectivity_state 사본
enum class ChannelState : int32_t {
  kIdle = 0,
  kConnecting = 1,
  kReady = 2,
  kTransientFailure = 3,
  kShutdown = 4,
};
std::string to_string(ChannelState s);

/// 누적 호출 수 (진단용, 스레드 안전 스냅샷)
struct LinkCounters
{
  uint64_t state_calls{0};
  uint64_t state_failures{0};
  uint64_t command_calls{0};
  uint64_t command_failures{0};  ///< 전송 실패 + Result 거부
  uint64_t deadline_exceeded{0};
  uint64_t rejected_inputs{0};  ///< 보내지 않은 입력 (NaN 명령 등)
};

/// Simulator.GetTimestamp 한 번. 값은 호출 시점의 서버 시각이 **아니라** 현재 프레임의 timestamp
/// 다 (ActorStates.timestamp 와 같은 값, 실측). 그래서 한 표본은 오프셋의 하한만 준다 —
/// morai_link_types.hpp "시계 오프셋" 참고
struct ServerTimeSample
{
  ClockSample clock;
  double rtt_ms{0.0};
  /// server - recv [ms]: MORAI - 로컬 >= 이 값. 참 오프셋보다 이 프레임의 나이만큼 낮다
  double offset_lower_bound_ms{0.0};
};

/// MORAI gRPC 링크.
///
/// 채널 두 개를 따로 연다 (서로 다른 subchannel = 서로 다른 TCP 연결). 큰 상태 응답과
/// 50 ms deadline 명령이 한 HTTP/2 연결의 흐름 제어를 나눠 쓰지 않게 하고, 한쪽이 끊겨도
/// 다른 쪽 진단이 가능하게 한다. main.py 도 송신 스레드마다 자기 SimAdapter(채널)를 썼다.
///
/// 모든 메서드는 blocking + deadline 이며 스레드 안전하다. 두 채널은 서로 다른 스레드에서
/// 동시에 써도 되고, 한 채널을 여러 스레드가 써도 된다 (gRPC 채널·스텁은 스레드 안전).
/// Result 를 돌려주는 RPC 는 Result.status == SUCCESS 까지 확인한다. 예외는 밖으로 나가지 않는다.
class MoraiLink
{
public:
  explicit MoraiLink(const MoraiLinkOptions & options);
  ~MoraiLink();

  MoraiLink(const MoraiLink &) = delete;
  MoraiLink & operator=(const MoraiLink &) = delete;
  MoraiLink(MoraiLink &&) = delete;
  MoraiLink & operator=(MoraiLink &&) = delete;

  const MoraiLinkOptions & options() const;

  /// 두 채널이 모두 READY 가 될 때까지 (또는 timeout). 채널 생성은 lazy 라 이것만이 도달 증명이다
  Status wait_connected(std::chrono::milliseconds timeout);

  /// 채널 상태 조회. try_to_connect 면 IDLE 채널에 연결을 시작시킨다
  ChannelState channel_state(LinkChannel channel, bool try_to_connect = false) const;

  // ===========================================================================
  // 상태 채널
  // ===========================================================================

  /// Simulation.WaitForTick. 실시간 모드에서도 업데이트마다 한 번 돌아온다 (실측).
  /// value 는 ActorStates.timestamp 와 **다른 시계**다 (일치 0%) — 트리거로만 쓰고 시각은
  /// 스냅샷에서 가져온다. deadline 은 한 업데이트 대기보다 길게 준다 (예: 200 ms)
  Reply<int64_t> wait_for_tick(std::chrono::milliseconds deadline);

  /// GetAllActorsState 한 번. 성공하면 *out 을 통째로 채운다 (벡터 용량은 재사용). 실패하면
  /// *out 은 건드리지 않는다
  Status get_snapshot(Snapshot * out);
  Reply<Snapshot> get_snapshot();

  /// GetActorState(ego). timestamp 가 없어 has_timestamp = false
  Reply<EgoSample> get_ego_state();

  Reply<ControlMode> get_control_mode();
  Reply<VehicleSpecInfo> get_vehicle_spec();
  Reply<SyncModeInfo> get_sync_mode();

  /// Simulator.GetTimestamp 한 번 + 로컬 송수신 시각. 값은 프레임 시각이라 get_snapshot() 의
  /// timestamp 와 같은 정보다 (응답이 작을 뿐). 오프셋은 여러 표본의 clock_offset_lower_bound_ms 와
  /// 같은 PC 기준선으로 본다
  Reply<ServerTimeSample> probe_server_time();

  // ===========================================================================
  // 명령 채널
  // ===========================================================================

  /// ControlVehicle 한 번. 비유한 값이 있으면 보내지 않고 ok=false (grpc_code -1)
  Status control_vehicle(const VehicleCommand & cmd);
  /// 같은 호출을 이번만 다른 deadline 으로 (원격 MORAI: main.py cmd_timeout_for_rtt 처럼 왕복에 맞춘다)
  Status control_vehicle(const VehicleCommand & cmd, std::chrono::milliseconds deadline);

  /// verify_timeout > 0 이면 보낸 뒤 상태 채널로 되읽어 확인한다 (MORAI Result 는 거짓일 수 있다
  /// — SetSynchronousMode 실측). 그때 ok 는 되읽기 일치 여부이고 result_status 는 참고값이다.
  /// DEADLINE_EXCEEDED 는 적용 여부를 모르므로 되읽기를 한다. 그 밖의 전송 실패는 되읽지 않는다
  Status set_gear(GearMode gear, std::chrono::milliseconds verify_timeout = std::chrono::milliseconds(0));
  Status set_control_mode(
    ControlMode mode, std::chrono::milliseconds verify_timeout = std::chrono::milliseconds(0));
  /// 되읽기는 수평 위치(|dx|,|dy| <= tolerance_m)와 yaw(<= tolerance_deg)만 본다 — z 는 MORAI 가
  /// 지면에 맞춘다
  Status set_transform(
    const TransformPose & pose,
    std::chrono::milliseconds verify_timeout = std::chrono::milliseconds(0),
    double tolerance_m = 0.5, double tolerance_deg = 2.0);
  Status set_tail_light(TurnSignal signal);

  LinkCounters counters() const;

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace morai_grpc_vendor

#endif  // MORAI_GRPC_VENDOR__MORAI_LINK_HPP_
