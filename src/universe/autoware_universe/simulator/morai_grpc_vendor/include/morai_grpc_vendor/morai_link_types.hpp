// 신규 — 26.09.25 _ YSH _ MoraiLink 공개 자료형 (grpc·protobuf 비의존 순수 구조체·열거형·헬퍼)
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

// 값은 전부 MORAI 가 준 그대로다 (단위·부호·축 변환 없음). Autoware 규약으로의 변환은
// autoware_morai_interface 의 conversion/ 이 맡는다 — 여기서 변환하면 조향 부호 비대칭
// (context-notes.md "절대 통일하지 말 것") 같은 규칙이 두 곳에 흩어진다.
// 이 헤더는 <grpcpp/...>, <google/protobuf/...>, *.pb.h 를 절대 include 하지 않는다
// (test_morai_link 의 격리 검사가 고정).

#ifndef MORAI_GRPC_VENDOR__MORAI_LINK_TYPES_HPP_
#define MORAI_GRPC_VENDOR__MORAI_LINK_TYPES_HPP_

#include <cstddef>
#include <cstdint>
// 26.09.26 _ YSH _ Snapshot::raw_actor_states (shared_ptr)
#include <memory>
#include <string>
#include <vector>

namespace morai_grpc_vendor
{

// =============================================================================
// MORAI 열거형 사본 (값은 proto 와 같다 — morai_link.cpp 의 static_assert 가 고정)
// =============================================================================

/// actor_enum.proto LongCmdType
enum class LongCmdType : int32_t {
  kUnspecified = 0,
  kThrottle = 1,
  kSpeed = 2,
  kAcceleration = 3,
  // 26.10.12 _ YSH _ 의사 VILS follow: MORAI Ego 를 고스트로 (물리 없이 SetTransform 을 따라간다 — 10-12 실 MORAI 벤치)
  kGhost = 4,
};

/// actor_enum.proto GearMode. main.py 매핑: P=1 R=2 N=3 D=4 (M=0, L=5)
enum class GearMode : int32_t {
  kM = 0,
  kP = 1,
  kR = 2,
  kN = 3,
  kD = 4,
  kL = 5,
};

/// actor_enum.proto VehicleControlMode. ControlVehicle 은 kAuto(3)·kSynchronous(7) 에서만 먹는다 (실측)
enum class ControlMode : int32_t {
  kUnspecified = 0,
  kKeyboard = 1,
  kGameWheel = 2,
  kAuto = 3,
  kAutoLateral = 4,
  kAutoLongitudinal = 5,
  kCruise = 6,
  kSynchronous = 7,
};

/// actor_enum.proto VehicleTurnSignal (0 = 소등)
enum class TurnSignal : int32_t {
  kOff = 0,
  kLeft = 1,
  kRight = 2,
  kAll = 3,
};

/// common/enum.proto ObjectType
enum class ObjectType : int32_t {
  kUnspecified = 0,
  kVehicle = 1,
  kPedestrian = 2,
  kObstacle = 3,
  kSpawnPoint = 4,
};

/// simulation_enum.proto SyncModeType 의 값 그대로 (0 = 실시간 / UNSPECIFIED)
struct SyncModeInfo
{
  int32_t type{0};
  std::string type_name;
  int32_t tick_period_ms{0};  ///< proto: msec, 1~100 (실시간 모드면 0)
};

/// 로그용 이름. 모르는 값은 "UNKNOWN(n)".
std::string to_string(GearMode g);
std::string to_string(ControlMode m);
std::string to_string(LongCmdType t);
std::string to_string(TurnSignal s);
std::string to_string(ObjectType t);

// =============================================================================
// 호출 결과
// =============================================================================

/// 모든 호출의 결과. 예외는 API 밖으로 나가지 않는다.
///
/// 실패는 두 층이다. 전송(grpc_code, 0 = OK) 과 MORAI 응용(result_status, Result 를 돌려주는
/// RPC 만 — StatusCode SUCCESS = 1, UNKNOWN_FAILURE = 0). 0 을 성공으로 읽으면 전부 뒤집힌다.
struct Status
{
  bool ok{false};
  /// grpc::StatusCode 값. -1 = 보내지 않음 (입력 거부·연결 객체 없음)
  int32_t grpc_code{-1};
  /// morai Result.status. -1 = Result 가 없는 RPC 이거나 응답을 못 받음
  int32_t result_status{-1};
  std::string message;
  /// 호출 벽시계 시간 (steady, ms). 보내지 않았으면 0
  double rpc_ms{0.0};
  /// 되읽기 확인을 요청했을 때만 true — 그때 readback_matched 가 최종 판정이다
  bool readback_checked{false};
  bool readback_matched{false};

  bool deadline_exceeded() const { return grpc_code == 4; }  ///< DEADLINE_EXCEEDED
  bool unavailable() const { return grpc_code == 14; }       ///< UNAVAILABLE
};

/// 값을 돌려주는 호출의 결과. status.ok 가 false 면 value 는 기본값이다.
template <typename T>
struct Reply
{
  Status status;
  T value{};
  bool ok() const { return status.ok; }
};

// =============================================================================
// 상태 표본 (MORAI 원값)
// =============================================================================

struct Vec3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

/// ActorState.vehicle_state — 차량일 때만 MORAI 가 채운다. 우리가 보낸 명령의 에코에 가깝다
/// (main.py: fwa == steer 300/300). 명령이 없는 프레임은 유휴 레코드(throttle 0, brake ~0.003)
struct VehicleStateSample
{
  bool present{false};  ///< has_vehicle_state
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};              ///< proto 주석 "-1~1"
  double front_wheel_angle{0.0};  ///< 정규화값 [-1,1], Unity 부호 (ROS 와 반대 — 변환은 AMI)
  GearMode gear_mode{GearMode::kM};
  size_t collision_count{0};                 ///< collision_objects 개수
  std::vector<std::string> collision_objects;
};

/// Ego 한 표본.
///
/// velocity 는 local(차체) km/h, angular_velocity 는 실측상 WORLD rad/s 를 축 (-z, x, -y)
/// 로 담은 원값이다 (morai_coordinates.py). rotation 은 도(deg), CCW, 동쪽 0.
struct EgoSample
{
  bool found{false};        ///< 스냅샷에 Ego 항목이 있었다
  bool exact_match{false};  ///< ego_id 와 대소문자까지 같은 항목이었다 (false 면 대소문자 무시 대체 매치)
  /// daq_publisher.ego_is_valid — 위치·자세·속도 9개가 전부 정확히 0 이면 무효 (미스폰·리셋 기본값)
  bool valid{false};
  std::string id;
  ObjectType object_type{ObjectType::kUnspecified};

  /// ActorStates.timestamp (µs, 실시간 모드에선 MORAI 호스트 Unix epoch). GetActorState 로 받은
  /// 표본은 timestamp 가 없어 0 이고 has_timestamp = false
  int64_t timestamp_us{0};
  bool has_timestamp{false};
  int64_t receive_system_ns{0};  ///< 응답 도착 system_clock (Unix epoch ns)
  int64_t receive_steady_ns{0};  ///< 응답 도착 steady_clock

  Vec3 position;             ///< transform.location, ENU [m]
  Vec3 rotation_deg;         ///< transform.rotation (x=roll, y=pitch, z=yaw) [deg]
  Vec3 velocity_kmh;         ///< velocity (local) [km/h]
  Vec3 angular_velocity_raw;  ///< angular_velocity 원값 (world rad/s, 축 (-z, x, -y))
  Vec3 acceleration_raw;      ///< acceleration 원값 (local)
  Vec3 global_velocity_raw;   ///< global_velocity 원값

  VehicleStateSample vehicle_state;
};

/// Ego 가 아닌 actor 한 표본 — main.py publish_tracked_objects 가 쓰는 필드 전부
struct ActorSample
{
  std::string id;  ///< actor_info.id.value (uuid5("morai:"+id) 가 main.py 의 object_id 규칙)
  ObjectType object_type{ObjectType::kUnspecified};
  std::string client_key;

  Vec3 position;              ///< transform.location, ENU [m]
  Vec3 rotation_deg;          ///< transform.rotation [deg]
  Vec3 velocity_kmh;          ///< local [km/h]
  Vec3 angular_velocity_raw;  ///< Ego 와 같은 규약
  Vec3 acceleration_raw;
  Vec3 global_velocity_raw;

  /// bounding_box.extent — **반치수**이고 x=반폭, y=반길이, z=반높이 (main.py 실측 Ioniq
  /// (1.06, 2.32, 0.75)). Autoware Shape.dimensions(x=길이) 로는 x/y 를 맞바꿔 2 배 한다
  Vec3 bbox_extent;
  Vec3 bbox_location;
  Vec3 bbox_rotation;

  bool has_vehicle_state{false};
  VehicleStateSample vehicle_state;
};

/// GetAllActorsState 한 번 = 한 업데이트의 Ego + NPC + timestamp
struct Snapshot
{
  int64_t timestamp_us{0};           ///< ActorStates.timestamp
  uint64_t sync_mode_tick_count{0};  ///< 실시간 모드에선 항상 0
  EgoSample ego;                     ///< ego.found == false 면 Ego 가 없었다
  /// id 가 ego_id 와 정확히 같은 항목을 전부 뺀 나머지 (main.py TrackedObjects 와 같다). ego 가
  /// 대소문자 무시 대체 매치(exact_match false)면 그 항목도 뺀다 (자차를 장애물로 내지 않게)
  std::vector<ActorSample> actors;
  int64_t send_system_ns{0};         ///< 요청 직전 system_clock
  int64_t receive_system_ns{0};      ///< 응답 직후 system_clock
  int64_t send_steady_ns{0};
  int64_t receive_steady_ns{0};
  double rpc_ms{0.0};
  // 26.09.26 _ YSH _ 원시 gRPC 기록: 받은 ActorStates 응답을 그대로 다시 직렬화한 바이트 (MoraiLinkOptions::capture_raw
  //   일 때만, 아니면 nullptr). 호출마다 새 버퍼라 엔진이 Snapshot 을 재사용해도 앞서 넘긴 바이트는 바뀌지 않는다
  std::shared_ptr<const std::string> raw_actor_states;
};

/// 제어 명령. 값은 MORAI 로 그대로 나간다.
///
/// steer 는 main.py 대로 타이어각 [rad] (proto 주석의 [-1,1] 정규화가 아니다 — 실측 0.698 rad
/// 에서 포화). 부호·클램프·보정은 AMI conversion 이 정한다. velocity 는 LONG_CMD_TYPE_SPEED 일
/// 때만, acceleration 은 ACCELERATION 일 때만 쓰인다. frame 은 동기 모드 전용 (실시간 0).
struct VehicleCommand
{
  LongCmdType long_cmd_type{LongCmdType::kAcceleration};
  double throttle{0.0};
  double brake{0.0};
  double steer{0.0};
  double velocity{0.0};
  double acceleration{0.0};
  uint32_t frame{0};
};

/// SetTransform 대상 (MORAI 규약: ENU [m], roll/pitch/yaw [deg], CCW 동쪽 0)
struct TransformPose
{
  Vec3 position;
  Vec3 rotation_deg;
};

/// GetVehicleSpec (단위는 proto 그대로: m, deg)
struct VehicleSpecInfo
{
  int32_t sub_category{0};
  double length_m{0.0};
  double width_m{0.0};
  double height_m{0.0};
  double wheel_base_m{0.0};
  double front_overhang_m{0.0};
  double rear_overhang_m{0.0};
  double max_steering_angle_deg{0.0};
  double wheel_radius_m{0.0};
};

// =============================================================================
// 시계 오프셋 (원격 MORAI + chrony 대비)
// =============================================================================
//
// MORAI 는 "호출 시점의 서버 시각" 을 주지 않는다. Simulator.GetTimestamp 도 ActorStates.timestamp
// 와 같은 **프레임 시각**이다 (실측: 폴링의 99.7% 가 직전 값 반복, 초당 ~50 번만 바뀜, 두 값 차 p50 0).
// 그래서 NTP 식(server - 왕복 중간점)은 쓸 수 없다 — 프레임 나이(p50 ~10 ms, p95 ~28 ms)만큼 틀린다.
//
// 쓸 수 있는 것은 한쪽 경계다. 프레임은 자기 timestamp 이후에야 읽히므로
//   offset (= MORAI 시계 - 로컬 시계) >= server - recv
// 이고, 가장 빠듯한 표본은 프레임이 바뀐 직후의 응답이다 (자주 폴링할수록 빠듯하다).
// 이 하한은 참 오프셋보다 "최소 프레임 나이"(MORAI 내부 발행 지연 + 응답 편도 + 폴링 간격)만큼
// 낮다. 그 값은 같은 PC(참 오프셋 0)에서 잰 하한이 곧 기준선이고, 원격 MORAI 의 오프셋은
//   offset ~= 원격 하한 - 같은 PC 기준선   (clock_offset_from_baseline_ms)
// 으로 본다. 두 값은 같은 방식(스냅샷 폴링, 같은 주기)으로 잰 것이어야 한다. 교차 확인은 두 PC
// 모두에서 `chronyc tracking` / `chronyc sources -v`.

/// 프레임 시계 한 표본. server_us 는 MORAI 프레임 timestamp (epoch µs), local_* 는 요청 직전·응답
/// 직후 로컬 system_clock (epoch ns)
struct ClockSample
{
  int64_t server_us{0};
  int64_t local_send_ns{0};
  int64_t local_recv_ns{0};
};

/// offset >= max(server - recv) [ms]. 쓸 표본이 없으면 NaN. 위 설명대로 참 오프셋보다 최소 프레임
/// 나이만큼 낮다 — 같은 PC 에서도 0 이 아니라 음수(-나이)가 나온다
double clock_offset_lower_bound_ms(const std::vector<ClockSample> & samples);

/// 원격 하한 - 같은 PC 기준선 = MORAI - 로컬 오프셋 추정 [ms]. 둘 중 하나라도 비유한이면 NaN
double clock_offset_from_baseline_ms(double remote_lower_bound_ms, double local_baseline_ms);

// =============================================================================
// 순수 판정
// =============================================================================

/// daq_publisher.ego_is_valid 와 같다: 위치·자세·속도 9 값이 전부 정확히 0 이면 무효
bool ego_is_valid(const Vec3 & position, const Vec3 & rotation_deg, const Vec3 & velocity_kmh);

/// 모든 필드가 유한할 때만 true (NaN·inf 명령은 보내지 않는다)
bool command_is_finite(const VehicleCommand & cmd);

}  // namespace morai_grpc_vendor

#endif  // MORAI_GRPC_VENDOR__MORAI_LINK_TYPES_HPP_
