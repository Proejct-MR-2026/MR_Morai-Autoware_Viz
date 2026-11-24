// 신규 — 26.09.26 _ YSH _ DAQ(ViLS 명세 25 항목, 100 Hz) 샘플러 — daq_publisher.py 의 수집부를 브리지 노드 안으로.
//   MORAI 를 따로 조회하지 않고, 엔진이 업데이트마다 받은 스냅샷(DaqFrameCache)을 전용 스레드가 10 ms 마다 다시
//   발행한다(ZOH). cue_server 의 녹화 구간(/daq/run_state)에 맞춰 켜고 끈다 (daq.mode gated) — 또는 늘 켠다(always).
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

#ifndef AUTOWARE__MORAI_INTERFACE__DAQ_SAMPLER_HPP_
#define AUTOWARE__MORAI_INTERFACE__DAQ_SAMPLER_HPP_

#include "autoware/morai_interface/engine.hpp"

#include <morai_grpc_vendor/morai_link_types.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logger.hpp>

#include <autoware_control_msgs/msg/control.hpp>
#include <builtin_interfaces/msg/time.hpp>
// 26.10.05 _ YSH _ DAQ 명세 토픽을 MORAI 공식 메시지로 (Odometry·Accel·TrackedObjects → EgoVehicleStatus·ObjectStatusList)
#include <morai_ros2_msgs/msg/collision_data.hpp>  // 26.10.07 _ YSH _ /daq/collision
#include <morai_ros2_msgs/msg/ctrl_cmd.hpp>  // 26.10.06 _ YSH _ 명세 25 = MORAI 에 보낸 gRPC 명령
#include <morai_ros2_msgs/msg/ego_vehicle_status.hpp>
#include <morai_ros2_msgs/msg/object_status_list.hpp>
// 원문: #include <autoware_perception_msgs/msg/tracked_objects.hpp>
// 원문: #include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
// 원문: #include <nav_msgs/msg/odometry.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
// 26.09.26 _ YSH _ 원시 gRPC 기록 큐
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace autoware::morai_interface
{

/// 엔진이 받아들인 MORAI 표본 하나 (엔진 스레드가 만들고 DAQ 스레드가 읽는다)
struct DaqFrame
{
  morai_grpc_vendor::Snapshot snapshot;
  builtin_interfaces::msg::Time stamp;  ///< 그 표본의 발행 stamp (kinematic_state 와 같다)
  uint64_t update_index{0};
  int64_t receive_steady_ns{0};  ///< 엔진 시계
};

/// 최신 DaqFrame 하나 (StateCache 와 같은 atomic shared_ptr)
class DaqFrameCache
{
public:
  void store(std::shared_ptr<const DaqFrame> frame) { std::atomic_store(&frame_, std::move(frame)); }
  std::shared_ptr<const DaqFrame> latest() const { return std::atomic_load(&frame_); }

private:
  std::shared_ptr<const DaqFrame> frame_;
};

/// 명세 25 원천: Autoware control_cmd 원본 그대로 (forward_control_to_sim·NaN 과 무관 — Autoware 가 낸 것)
class DaqCommandLatch
{
public:
  struct Entry
  {
    autoware_control_msgs::msg::Control msg;
    int64_t receive_steady_ns{0};
    uint64_t seq{0};
  };
  void submit(const autoware_control_msgs::msg::Control & msg, int64_t receive_steady_ns);
  std::optional<Entry> latest() const;
  uint64_t count() const;

private:
  mutable std::mutex mutex_;
  std::optional<Entry> entry_;
  uint64_t seq_{0};
};

// =============================================================================
// 26.09.26 _ YSH _ 원시 gRPC 기록 (daq.raw) — "gRPC 가 준 값을 그대로". 기존 /daq/* 25 항목은 그대로 두고 아래 두 토픽을 더한다
//   <prefix>/raw/actor_states    받아들인 MORAI 업데이트마다 GetAllActorsState 응답 전체 (ZOH 아님, ~50/s)
//   <prefix>/raw/vehicle_ctrl_cmd ControlVehicle 을 부를 때마다 보낸 요청 그대로 + 결과 (timer 100/s)
// 둘 다 std_msgs/String 에 JSON 한 개 (스키마 morai_raw.*/1). 엔진·명령 스레드는 복사해 큐에 넣기만 하고, proto3 JSON
// 변환은 DAQ 스레드가 틱마다 한다. /daq/* 와 같은 게이트 — 넣을 때와 낼 때 모두 발행 중이어야 나간다
// =============================================================================

/// 받아들인 MORAI 업데이트 하나의 응답 (엔진 스레드가 넣는다)
struct DaqRawActorStates
{
  std::shared_ptr<const std::string> bytes;  ///< ActorStates 직렬화 바이트 (Snapshot::raw_actor_states, 호출마다 새 버퍼)
  uint64_t update_index{0};                  ///< 엔진이 받아들인 업데이트 번호 (on_sample 과 같다)
  int64_t rx_system_ns{0};                   ///< 응답 도착 system_clock (Snapshot::receive_system_ns)
  int64_t morai_timestamp_us{0};             ///< ActorStates.timestamp
};

/// ControlVehicle 한 번 (엔진 또는 명령 타이머 스레드가 넣는다)
struct DaqRawCommand
{
  morai_grpc_vendor::VehicleCommand cmd;  ///< 링크에 넘긴 명령 그대로 (stale 이면 감속 대체 명령)
  uint64_t send_seq{0};                   ///< 노드 수명 동안 송신마다 +1 (기록하지 않은 송신도 센다 — 빈 번호 = 게이트 밖)
  int64_t send_system_ns{0};              ///< RPC 직전 system_clock
  bool stale{false};
  bool ok{false};                         ///< 계약 "RPC status OK" = grpc_code 0 (MORAI Result 는 result_status 로 따로)
  int32_t grpc_code{-1};                  ///< grpc::StatusCode, -1 = 보내지 않음 (비유한 명령)
  std::optional<double> rpc_ms;           ///< RPC 벽시계 [ms], 보내지 않았으면 없음 (JSON null)
  // 26.09.26 _ YSH _ 리뷰 반영: 응답 Result.status (1 = SUCCESS, 0 = UNKNOWN_FAILURE …). -1 = 응답을 못 받음 (JSON null)
  int32_t result_status{-1};
};

/// 26.09.26 _ YSH _ 리뷰 반영: ControlVehicle 한 번 → 원시 기록 한 칸. 링크 Status.ok 는 MORAI Result 거부도 false 로 접으므로
///   그대로 쓰지 않는다 — ok = 전송 OK (grpc_code 0), 거부 여부는 result_status 로 ("안 닿음"과 "MORAI 가 거부"를 가른다)
DaqRawCommand make_daq_raw_command(
  const morai_grpc_vendor::VehicleCommand & cmd, bool stale, int64_t send_system_ns,
  const morai_grpc_vendor::Status & status, uint64_t send_seq);

/// 한 틱에 낼 원시 기록 (JSON 문자열, 넣은 순서)
struct DaqRawOutputs
{
  std::vector<std::string> actor_states;
  std::vector<std::string> vehicle_ctrl_cmd;
};

/// 계약 morai_raw.actor_states/1 한 건 (run = 넣을 때의 런, always 모드에서 런이 없으면 ""). 바이트가 없거나 읽히지 않으면
/// actor_states 는 null
std::string daq_raw_actor_states_json(const DaqRawActorStates & item, const std::string & run);
/// 계약 morai_raw.vehicle_ctrl_cmd/1 한 건. link = 명령을 보낸 링크의 옵션 (요청의 actor_info 를 같게 만든다)
std::string daq_raw_vehicle_ctrl_cmd_json(
  const DaqRawCommand & item, const std::string & run,
  const morai_grpc_vendor::MoraiLinkOptions & link);

enum class DaqMode { kGated, kAlways };
enum class DaqStamp { kSample, kTick };

std::optional<DaqMode> parse_daq_mode(const std::string & text);
const char * to_string(DaqMode mode);
std::optional<DaqStamp> parse_daq_stamp(const std::string & text);
const char * to_string(DaqStamp stamp);

/// /daq/run_state 의 data: "active <run>" 또는 "idle [<run>]" → (active, run). 형식이 틀리면 nullopt
std::optional<std::pair<bool, std::string>> parse_daq_run_state(const std::string & data);

inline constexpr double kDaqRateHzMin = 10.0;
inline constexpr double kDaqRateHzMax = 400.0;
bool daq_rate_is_valid(double hz);

struct DaqConfig
{
  double rate_hz{100.0};           ///< 명세 100 Hz (daq/spec.py EXPECT_HZ)
  double max_sample_age_s{0.1};    ///< 표본이 이만큼 낡으면 상태 토픽을 멈추고 결손으로 센다 (daq_publisher 와 같음)
  double cmd_stale_s{0.2};         ///< 새 control_cmd 가 이만큼 없으면 vehicle_cmd 를 멈춘다 (0 을 지어내지 않음)
  double accel_scale{1.0};         ///< daq_publisher --accel-scale
  bool publish_objects{true};
  bool publish_collision{true};    ///< 26.10.07 _ YSH _ <prefix>/collision (gRPC Ego collision_objects → MORAI CollisionData)
  DaqStamp stamp{DaqStamp::kSample};  ///< sample = MORAI 표본 stamp·control_cmd 원래 stamp, tick = 발행 시각
  double run_timeout_s{1200.0};    ///< active 가 이보다 길면 자동 종료 (cue_server 사망 대비). 0 = 끔
  // 26.09.26 _ YSH _ 원시 gRPC 기록
  bool raw{false};                 ///< <prefix>/raw/{actor_states,vehicle_ctrl_cmd} (false 면 submit_raw_* 는 아무것도 안 한다)
  size_t raw_queue_depth{256};     ///< 엔진·명령 스레드 → DAQ 스레드 큐 (종류마다). 넘치면 가장 오래된 것을 버리고 센다
  morai_grpc_vendor::MoraiLinkOptions raw_link;  ///< 보낸 VehicleCtrlCmd 를 다시 만들 때 (ego_id·client_key — 링크와 같게)
  // 26.10.12 _ YSH _ 의사 VILS(follow): <prefix>/ctrl_cmd 를 MORAI 송신값 대신 Autoware control_cmd(실차에 보낸 명령)로
  bool ctrl_from_command{false};
};

/// 한 틱에 낼 것 (없는 것은 내지 않는다)
struct DaqOutputs
{
  // 26.10.05 _ YSH _ 명세 1~9·12 = <prefix>/ego_status, 13~24 = <prefix>/object_status (10~11 roll·pitch 는 raw/actor_states)
  std::optional<morai_ros2_msgs::msg::EgoVehicleStatus> ego;
  std::optional<morai_ros2_msgs::msg::ObjectStatusList> objects;
  // 원문: std::optional<nav_msgs::msg::Odometry> state;
  // 원문: std::optional<geometry_msgs::msg::AccelWithCovarianceStamped> accel;
  // 원문: std::optional<autoware_perception_msgs::msg::TrackedObjects> objects;
  std::optional<autoware_control_msgs::msg::Control> cmd;
  // 26.10.06 _ YSH _ 명세 25 를 gRPC 값으로: MORAI 에 ControlVehicle 로 실제 보낸 VehicleCtrlCmd → <prefix>/ctrl_cmd
  std::optional<morai_ros2_msgs::msg::CtrlCmd> ctrl;
  // 26.10.07 _ YSH _ Ego 충돌 상대 (명세 밖, ego_status 와 같은 틱·같은 stamp·같은 ZOH. 충돌이 없으면 빈 목록)
  std::optional<morai_ros2_msgs::msg::CollisionData> collision;
};

/// 런 하나의 요약 (끝날 때 /daq/run_summary 로 나간다)
struct DaqRunSummary
{
  std::string run;
  std::string end_reason;  ///< idle | timeout | replaced | shutdown
  double duration_s{0.0};
  uint64_t ticks{0};             ///< 발행 틱 (켜진 동안 10 ms 마다)
  uint64_t state_published{0};   ///< ego_status 를 낸 틱 (26.10.05 _ YSH _ 원문: vehicle_state 를 낸 틱)
  uint64_t cmd_published{0};
  uint64_t gap_ticks{0};         ///< 표본이 없거나 낡아 상태를 못 낸 틱
  uint64_t link_down_ticks{0};   ///< 그중 MORAI 링크가 끊겨 있던 틱
  uint64_t longest_gap_ticks{0};
  uint64_t late{0};              ///< 늦게 깨어 기준을 다시 잡은 횟수
  uint64_t updates{0};           ///< 낸 서로 다른 MORAI 업데이트 수
  uint64_t yield_ticks{0};       ///< 켜진 동안 다른 /daq writer 에 양보해(또는 첫 점검 전이라) 내지 않은 틱
  // 26.09.26 _ YSH _ 원시 gRPC 기록 (daq.raw). published = 이 런 이름으로 낸 수, dropped = 큐가 넘쳐 버린 수
  uint64_t raw_published{0};     ///< <prefix>/raw/actor_states
  uint64_t raw_dropped{0};
  uint64_t tx_published{0};      ///< <prefix>/raw/vehicle_ctrl_cmd
  uint64_t tx_dropped{0};
  double state_hz() const { return duration_s > 0.0 ? static_cast<double>(state_published) / duration_s : 0.0; }
};
std::string to_json(const DaqRunSummary & summary);

class DaqSink
{
public:
  virtual ~DaqSink() = default;
  /// DAQ 스레드에서 틱마다
  virtual void publish_daq(const DaqOutputs & outputs) = 0;
  /// 런이 끝날 때 (끝낸 스레드에서 — executor 또는 DAQ 스레드)
  virtual void on_daq_run_end(const DaqRunSummary & /*summary*/) {}
  // 26.09.26 _ YSH _ 원시 gRPC 기록 — DAQ 스레드에서 틱마다 (그 틱까지 쌓인 것 전부, 없으면 부르지 않는다). stop() 이
  //   스레드를 거둔 뒤 남은 것을 낼 때는 stop 을 부른 스레드에서
  virtual void publish_daq_raw(const DaqRawOutputs & /*outputs*/) {}
};

/// 누적 통계 (진단용)
struct DaqStats
{
  uint64_t ticks{0};
  uint64_t state_published{0};
  uint64_t cmd_published{0};
  uint64_t gap_ticks{0};
  uint64_t link_down_ticks{0};
  uint64_t late{0};
  uint64_t runs_started{0};
  uint64_t runs_ended{0};
  uint64_t run_timeouts{0};
  // 26.09.26 _ YSH _ 원시 gRPC 기록 누적. dropped = 큐 넘침 (자료 손실), discarded = 게이트가 닫혀 버림 (정상)
  uint64_t raw_published{0};
  uint64_t raw_dropped{0};
  uint64_t raw_discarded{0};
  uint64_t tx_published{0};
  uint64_t tx_dropped{0};
  uint64_t tx_discarded{0};
  bool publishing{false};
  bool active{false};
  bool yielding{false};
  DaqMode mode{DaqMode::kGated};
  std::string run;                       ///< 지금 런 (없으면 빈 문자열)
  double run_elapsed_s{0.0};
  std::optional<double> frame_age_s;     ///< 최신 표본 나이
  std::optional<double> cmd_age_s;       ///< 최신 control_cmd 나이
};

class DaqSampler
{
public:
  DaqSampler(
    const DaqFrameCache & cache, const DaqCommandLatch & cmd_latch, const RuntimeParamsHandle & params,
    DaqSink & sink, const DaqConfig & config, DaqMode mode, const rclcpp::Logger & logger,
    EngineClock clock = EngineClock::real(), std::function<bool()> link_up = {});
  ~DaqSampler();

  DaqSampler(const DaqSampler &) = delete;
  DaqSampler & operator=(const DaqSampler &) = delete;

  void start();
  /// 26.10.12 _ YSH _ 의사 VILS(follow): ego_status 를 MORAI Ego 대신 이 함수 값으로 (실차). start() 전에 준다.
  ///   nullopt 면 그 틱 ego_status 를 내지 않는다. MORAI 표본이 낡아도 낸다 (객체·충돌만 MORAI 신선도를 따른다).
  ///   stamp: sample 모드면 함수가 채운 stamp, tick 모드면 발행 시각
  using EgoOverrideFn = std::function<std::optional<morai_ros2_msgs::msg::EgoVehicleStatus>()>;
  void set_ego_override(EgoOverrideFn fn) { ego_override_ = std::move(fn); }
  /// 스레드를 거둔다. 켜진 런이 있으면 'shutdown' 으로 끝내고 요약을 낸다
  void stop();

  /// 루프 한 바퀴 (스레드 없이 시험할 때): deadline 전이면 deadline 까지 자고, deadline 이면 tick 후 다음 deadline
  void step();
  /// 한 번 발행 (시험용 공개)
  void tick(int64_t now_steady_ns);

  // --- 게이트 (스레드 안전, executor 에서 부른다) ---
  void set_mode(DaqMode mode);
  DaqMode mode() const;
  /// run_state 한 건. active: 그 런을 시작 (같은 런이면 무시 — latched 재수신, 다른 런이 켜져 있으면 끝내고 새로).
  /// 26.09.26 _ YSH _ 리뷰 반영: idle <run> 은 이름이 같은 켜진 런만 끝낸다 (늦게 온 앞 런의 idle 이 다음 런을 끄지
  ///   않게). 이름 없는 idle 은 켜진 런을 무엇이든 끝낸다. 원문: idle: 켜진 런을 끝낸다
  void set_run_state(bool active, const std::string & run);
  /// 다른 노드가 이미 /daq 를 낸다 — 양보해 내지 않는다 (가드 타이머가 판정마다 부른다)
  void set_yield(bool yield);
  /// 26.09.26 _ YSH _ 리뷰 반영: 기동 직후 첫 writer 판정 전까지 조용히 양보해 둔다 (거짓 '다른 writer' 경고를 내지 않게)
  void yield_until_first_check();
  bool yield_pending_first_check() const;
  bool publishing() const;

  DaqStats stats() const;
  const DaqConfig & config() const { return config_; }

  // --- 26.09.26 _ YSH _ 원시 gRPC 기록 (스레드 안전 — 엔진·명령 타이머 스레드가 부른다. 복사해 넣기만 한다) ---
  /// config.raw 가 아니면 아무것도 안 한다. 발행 중이 아니면 버린다 (discarded). 발행 중이면 지금 런 이름을 붙여 큐에
  /// 넣고, 큐가 raw_queue_depth 만큼 차 있으면 가장 오래된 것을 버린다 (dropped). DAQ 틱이 전부 꺼내 JSON 으로 낸다.
  /// 26.09.26 _ YSH _ 리뷰 반영: 첫 stop() 뒤에는 always 모드여도 받지 않는다 (discarded — 종료 감속 등)
  void submit_raw_actor_states(DaqRawActorStates item);
  void submit_raw_command(DaqRawCommand item);
  // 26.10.06 _ YSH _ MORAI 가 받은(grpc OK) ControlVehicle 명령 — 틱마다 최신 것을 <prefix>/ctrl_cmd 로 (cmd_stale_s 넘으면 멈춤)
  void submit_sent_command(const morai_grpc_vendor::VehicleCommand & cmd, int64_t send_system_ns);

  static constexpr int64_t kLogPeriodNs = 10000000000LL;  ///< 10 s 통계 줄

private:
  EgoOverrideFn ego_override_;  ///< 26.10.12 _ YSH _ VILS 실차 Ego (없으면 MORAI Ego)
  struct RunState
  {
    bool active{false};
    std::string run;
    int64_t start_ns{0};
    DaqRunSummary summary;  ///< 켜진 동안 쌓는다
    uint64_t current_gap{0};
    uint64_t last_update_index{0};
    bool has_update{false};
  };

  // 26.09.26 _ YSH _ 원시 기록 큐 한 칸 — 넣을 때의 런 (이름·번호)을 같이 둔다 (런이 바뀐 뒤 나가도 이름이 맞게)
  template <typename T>
  struct RawEntry
  {
    T item;
    std::string run;           ///< 넣을 때 켜진 런 (없으면 "")
    uint64_t run_serial{0};    ///< 그 런의 번호 (없으면 0) — 요약은 번호가 지금 런과 같을 때만 센다
  };
  struct RawBatch
  {
    std::deque<RawEntry<DaqRawActorStates>> raw;
    std::deque<RawEntry<DaqRawCommand>> tx;
    bool empty() const { return raw.empty() && tx.empty(); }
  };
  /// run_mutex_ 를 잡은 채: 발행 중이면 큐를 통째로 꺼내 돌려주고 낸 것으로 센다 (꺼낸 것은 반드시 낸다),
  /// 아니면 버린다 (discarded 로 센다). 26.09.26 _ YSH _ 리뷰 반영: 앞에 raw_carry_ (이미 센 것)를 게이트와 무관하게 붙인다
  RawBatch take_raw_locked(bool publishing);
  /// 26.09.26 _ YSH _ 리뷰 반영: run_mutex_ 를 잡은 채, 런이 replaced 로 끝나기 직전. 큐를 지금 꺼내 끝나는 런 요약에 세고
  ///   raw_carry_ 로 옮긴다 — 다음 틱(또는 stop)이 게이트와 무관하게 낸다 (그사이 idle·양보가 와도 요약과 낸 수가 맞게)
  void carry_raw_locked();
  /// 락 밖에서 JSON 으로 바꿔 sink 에 낸다 (DAQ 스레드, 또는 스레드를 거둔 뒤 stop)
  void publish_raw(const RawBatch & batch);

  void run_loop();
  bool stop_requested() const;
  void sleep_until_ns(int64_t deadline_ns);
  void end_run_locked(const std::string & reason, int64_t now, std::optional<DaqRunSummary> * ended);
  void finish_run(const DaqRunSummary & summary);
  void build_state(const DaqFrame & frame, const RuntimeParams & params);
  void maybe_log(int64_t now);

  const DaqFrameCache & cache_;
  const DaqCommandLatch & cmd_latch_;
  const RuntimeParamsHandle & params_;
  DaqSink & sink_;
  const DaqConfig config_;
  rclcpp::Logger logger_;
  EngineClock clock_;
  std::function<bool()> link_up_;
  mutable rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};
  int64_t period_ns_{10000000};
  int64_t max_age_ns_{100000000};
  int64_t cmd_stale_ns_{200000000};
  // 26.10.06 _ YSH _ 마지막으로 MORAI 에 보낸 명령 (명령 타이머 스레드가 넣고 DAQ 스레드가 읽는다)
  struct SentCommand
  {
    morai_grpc_vendor::VehicleCommand cmd;
    int64_t steady_ns{0};
    int64_t system_ns{0};
  };
  mutable std::mutex sent_mutex_;
  std::optional<SentCommand> sent_;
  int64_t run_timeout_ns_{0};

  // --- 스레드 ---
  std::thread thread_;
  mutable std::mutex stop_mutex_;
  std::condition_variable stop_cv_;
  bool stop_requested_{false};

  // --- 게이트·통계 (run_mutex_) ---
  mutable std::mutex run_mutex_;
  DaqMode mode_;
  bool yield_{false};
  bool yield_checked_{false};  ///< 26.09.26 _ YSH _ 가드 타이머가 한 번이라도 판정했다
  RunState run_;
  DaqStats totals_;
  // 26.09.26 _ YSH _ 원시 기록 큐 (run_mutex_ — 게이트 판정과 같은 락이라 넣기·꺼내기가 게이트 변화와 엇갈리지 않는다)
  size_t raw_capacity_{256};
  uint64_t run_serial_{0};  ///< 런이 시작할 때마다 +1
  std::deque<RawEntry<DaqRawActorStates>> raw_queue_;
  std::deque<RawEntry<DaqRawCommand>> tx_queue_;
  // 26.09.26 _ YSH _ 리뷰 반영: replaced 로 끝난 런의 꼬리 (끝날 때 그 런 요약에 이미 셌다 — 반드시 낸다)
  RawBatch raw_carry_;
  // 26.09.26 _ YSH _ 리뷰 반영: 첫 stop() 이 남은 것을 낸 뒤 true — 그 뒤 submit 은 버리고 stop 은 sink 를 부르지 않는다.
  //   always 모드는 stop 뒤에도 게이트가 열려 있어, ~DaqSampler 의 두 번째 stop 이 이미 지워진 노드 publisher 로 내려 했다
  bool raw_closed_{false};

  // --- DAQ 스레드만 ---
  int64_t next_ns_{0};
  bool frame_seen_{false};
  uint64_t built_update_index_{0};
  std::shared_ptr<const RuntimeParams> built_params_;  ///< 잡고 있어 주소 재사용(ABA)으로 오판하지 않는다
  bool has_built_{false};
  // 26.10.05 _ YSH _ MORAI 메시지로
  morai_ros2_msgs::msg::EgoVehicleStatus built_ego_;
  morai_ros2_msgs::msg::ObjectStatusList built_objects_;
  morai_ros2_msgs::msg::CollisionData built_collision_;  // 26.10.07 _ YSH _
  // 원문: nav_msgs::msg::Odometry built_state_;
  // 원문: geometry_msgs::msg::AccelWithCovarianceStamped built_accel_;
  // 원문: autoware_perception_msgs::msg::TrackedObjects built_objects_;
  int64_t log_window_start_ns_{0};
  DaqStats log_window_prev_;
};

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__DAQ_SAMPLER_HPP_
