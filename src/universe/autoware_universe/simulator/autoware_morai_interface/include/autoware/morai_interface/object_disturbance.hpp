// 신규 — 26.11.10 _ YSH _ 외란 주입 (MRDIST-1): Autoware 로 가는 GT 객체에 위치 편차·잡음, 확률 드롭아웃, 완전 끊김을 건다.
//   - 난수는 SplitMix64 카운터 스트림이라 (기본 시드, 객체 이름, 프레임 번호)만 알면 Python 에서 같은 값을 다시 낸다.
//   - 명령은 서비스 ~/set_disturbance → DisturbanceController 가 읽고 검사해 DisturbanceHandle 을 통째로 바꿔 끼운다.
//   - 적용은 엔진 스레드가 객체 프레임마다 한 번 ObjectDisturber::apply. 스냅샷은 고치지 않는다 (GT DAQ 는 그대로).
//   ROS 노드·실행기와 무관한 순수 로직이다 (gtest 로 돈다). 규격은 design_261110/10 §4~6 + 05 §1
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
// 26.11.11 _ YSH _ 외란 메시지를 morai_ros2_msgs(외란 추가판)로 옮겼다 — 이 파일의 mr_disturbance_msgs 를 morai_ros2_msgs 로 바꿨다 (이름만, 주석 속 옛 이름·원문은 그대로)

#ifndef AUTOWARE__MORAI_INTERFACE__OBJECT_DISTURBANCE_HPP_
#define AUTOWARE__MORAI_INTERFACE__OBJECT_DISTURBANCE_HPP_

#include <morai_grpc_vendor/morai_link_types.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <morai_ros2_msgs/msg/disturbance_command.hpp>
#include <morai_ros2_msgs/msg/disturbance_status.hpp>
#include <morai_ros2_msgs/msg/object_disturbance_status_list.hpp>  // 26.11.11 _ YSH _ /daq/object_disturbance_status

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace autoware::morai_interface
{

// =============================================================================
// MRDIST-1 — 정수는 전부 부호 없는 64 비트, 넘치면 감긴다. C++ 과 Python 이 같은 식을 쓴다
// =============================================================================
namespace mrdist
{
inline constexpr char kAlgorithm[] = "MRDIST-1";
inline constexpr uint64_t kGamma = 0x9E3779B97F4A7C15ULL;
inline constexpr uint64_t kTagObject = 0x4F424A5345454431ULL;   ///< ASCII "OBJSEED1" (big-endian)
inline constexpr uint64_t kTagDropout = 0x44524F504F555431ULL;  ///< ASCII "DROPOUT1" (big-endian)
inline constexpr int64_t kNominalPeriodNs = 100000000LL;        ///< 객체 프레임 명목 주기 (Engine::kObjectsPeriodNs)
inline constexpr int64_t kMaxPeriodNs = 1000000000LL;           ///< 이보다 긴 간격은 명목 주기로 센다
inline constexpr uint64_t kMaxBaseSeed = 999999999999999ULL;    ///< Excel 이 그대로 보여 주는 15 자리

/// SplitMix64 마무리 섞기
uint64_t mix64(uint64_t z);
/// 그 시드의 SplitMix64 출력 counter 번째 = mix64(seed + counter × kGamma)
uint64_t stream(uint64_t seed, uint64_t counter);
/// (0, 1) 균등 — ((stream >> 12) + 0.5) × 2^-52. 정확한 double 이고 0·1 은 나오지 않는다
double u01(uint64_t seed, uint64_t counter);
/// 객체 시드. uuid5(NAMESPACE_OID, "morai:" + name) 의 앞 8 바이트 hi, 뒤 8 바이트 lo (big-endian) 로
/// mix64(mix64(base ^ kTagObject ^ hi) ^ lo)
uint64_t object_seed(uint64_t base_seed, const std::string & name);
/// 드롭아웃 사슬 시드 = mix64(base ^ kTagDropout)
uint64_t dropout_seed(uint64_t base_seed);

struct Gauss2
{
  double lon{0.0};
  double lat{0.0};
};
/// Box-Muller 한 쌍. u1 = u01(seed, 2k), u2 = u01(seed, 2k+1), r = sqrt(-2 ln u1), a = 2π u2 → (r cos a, r sin a)
Gauss2 gauss2(uint64_t seed, uint64_t k);

struct DropoutProb
{
  double p_hide{0.0};  ///< 보이는 프레임에서 숨김으로 넘어갈 확률
  double p_show{0.0};  ///< 숨긴 프레임에서 보임으로 돌아올 확률
};
/// 0.1 s 격자 위 두 상태 사슬의 전이 확률. 보이는 비율 = keep_prob, 평균 끊김 = max(mean_gap_s, 0.1) s.
/// 0 < keep_prob < 1 에서만 뜻이 있다 (keep ≤ 0 → (1, 0), keep ≥ 1 → (0, 1) 로 막아 둔다 — 사슬은 그 값에서 뽑지 않는다)
DropoutProb dropout_prob(double keep_prob, double mean_gap_s);
}  // namespace mrdist

// =============================================================================
// 트리거 문장
// =============================================================================

enum class DisturbanceKind { kNone, kClearAll, kPosition, kDropout, kBlackout };
/// position | dropout | blackout | clear, kNone 은 빈 문자열 (DisturbanceCommand.type)
const char * to_string(DisturbanceKind kind);

/// 문장 한 줄을 읽은 값. kNone = 상태 조회, kClearAll = 전체 해제, clear = 그 종류만 해제
struct ParsedTrigger
{
  DisturbanceKind kind{DisturbanceKind::kNone};
  bool clear{false};
  double lon_offset_m{0.0};
  double lon_std_m{0.0};
  double lat_offset_m{0.0};
  double lat_std_m{0.0};
  double keep_prob{0.0};
  double mean_gap_s{0.0};
  double duration_s{0.0};
  bool recover{false};
};

struct TriggerParse
{
  bool ok{false};
  std::string code;    ///< 거절 코드 (ok 면 빈 문자열)
  std::string detail;  ///< 사람이 읽을 이유
  ParsedTrigger cmd;
};

/// 문법 (소문자, 쉼표 구분, 토큰 앞뒤 공백·탭은 버린다)
///   disturbance, clear                                    disturbance, <kind>, clear
///   disturbance, position, lon_offset, lon_std, lat_offset, lat_std
///   disturbance, dropout, keep_prob, mean_gap_s           disturbance, blackout, duration_s, recover(1|0|true|false)
///   빈 문자열 = 상태 조회
/// 범위: |offset| ≤ 100, 0 ≤ std ≤ 100, 0 ≤ keep ≤ 1, 0 < keep < 1 이면 0 < gap ≤ 600 (아니면 gap ≥ 0),
///   0 ≤ duration ≤ 3600, 길이 ≤ 256. 거절 코드: TOO_LONG, BAD_PREFIX, UNKNOWN_KIND, UNSUPPORTED_KIND,
///   WRONG_ARITY, BAD_NUMBER, OUT_OF_RANGE
TriggerParse parse_disturbance_trigger(const std::string & text);

/// 그 종류를 끄는 것과 같은 값인가 — position 0,0,0,0 / dropout keep 1 / blackout duration 0 + recover 1
bool is_off_equivalent(const ParsedTrigger & cmd);

// 26.11.11 _ YSH _ 신규 — 앞에서 max_bytes 바이트 안쪽까지 자른다. UTF-8 글자 중간에서 자르지 않는다 (잘린 글은 그대로 유효한 UTF-8)
std::string utf8_prefix(const std::string & text, size_t max_bytes);

// =============================================================================
// 설정 (명령이 만든 상태) — 손잡이에 넣은 뒤로는 바꾸지 않는다
// =============================================================================

struct DisturbanceSettings
{
  uint64_t epoch_id{0};     ///< 시드 구간 번호 (0 = 아직 없음). 바뀌면 frame_seq 가 0 부터 다시 센다
  uint64_t base_seed{0};    ///< 1 ~ kMaxBaseSeed, 0 = 시드 구간 없음
  std::string run;          ///< 시드 구간을 연 run 이름
  uint64_t command_seq{0};  ///< 받아들인 명령 수. 노드가 사는 동안 늘기만 한다 (시드 구간이 바뀌어도 이어 센다)

  struct Position
  {
    bool on{false};
    double lon_offset_m{0.0};
    double lon_std_m{0.0};
    double lat_offset_m{0.0};
    double lat_std_m{0.0};
    std::string trigger;
    uint64_t cmd_seq{0};  ///< 이 종류를 건 명령의 command_seq
  } position;

  struct Dropout
  {
    bool on{false};
    double keep_prob{1.0};
    double mean_gap_s{0.0};
    std::string trigger;
    uint64_t cmd_seq{0};
  } dropout;

  struct Blackout
  {
    bool on{false};
    double duration_s{0.0};
    bool recover{false};
    std::string trigger;
    uint64_t cmd_seq{0};
  } blackout;
};

/// DisturbanceSettings 를 원자적으로 바꿔 끼우는 손잡이 (RuntimeParamsHandle 과 같은 방식 — 따로 둔 까닭: DaqSampler 가
/// RuntimeParams 포인터가 바뀔 때마다 GT 메시지를 다시 만들고, 런 종료 해제는 DAQ 스레드에서 온다)
class DisturbanceHandle
{
public:
  DisturbanceHandle() : settings_(std::make_shared<const DisturbanceSettings>()) {}
  std::shared_ptr<const DisturbanceSettings> get() const { return std::atomic_load(&settings_); }
  void set(const DisturbanceSettings & settings)
  {
    std::atomic_store(&settings_, std::make_shared<const DisturbanceSettings>(settings));
  }

  /// 엔진 → 제어기 되먹임: 자동 복구(recover 1)로 끝난 blackout 명령의 cmd_seq (0 = 없음). 제어기는 이것으로
  /// "같은 blackout 을 다시 보냄" 이 진행 중(그대로 둔다)인지 끝난 뒤(다시 건다)인지 가른다. 설정은 건드리지 않는다
  void note_blackout_recovered(uint64_t cmd_seq) const { blackout_recovered_seq_.store(cmd_seq); }
  uint64_t blackout_recovered_seq() const { return blackout_recovered_seq_.load(); }

private:
  std::shared_ptr<const DisturbanceSettings> settings_;
  mutable std::atomic<uint64_t> blackout_recovered_seq_{0};
};

// =============================================================================
// 명령 받기
// =============================================================================

struct DisturbanceRequest
{
  std::string trigger;
  std::string run;
  std::string source;
  uint64_t base_seed{0};
  uint64_t seq{0};
};

struct DisturbanceReply
{
  bool ok{false};
  std::string message;    ///< "OK [토큰...]" 또는 "<CODE>: 이유"
  ParsedTrigger command;  ///< 읽은 값 (문장을 읽지 못했으면 기본값)
  std::string run;        ///< 지금 시드 구간의 run
  uint64_t base_seed{0};  ///< 지금 시드 구간의 기본 시드
};

/// 명령을 읽고 검사해 설정을 바꾼다. 전부 아니면 전무 — 거절한 호출은 아무것도 바꾸지 않는다.
/// 스레드 안전 (자체 mutex): 서비스 콜백(executor), 런 종료 훅(executor·DAQ 스레드·종료 경로), 시험이 부른다.
///
/// 순서 (05 §1)
///   1. 길이·문법·범위                           → TOO_LONG, BAD_PREFIX, ... OUT_OF_RANGE
///   2. base_seed > kMaxBaseSeed                 → BAD_SEED
///   3. 켜는 명령인데 실차 모드 / GT 미발행      → REAL_CAR / NO_GROUND_TRUTH (해제·끔 값·조회는 늘 통과)
///   4. 켜는 명령 + base_seed 0 + run 이 지금 구간과 다름 → RUN_MISMATCH (해제·끔 값·조회는 run 을 보지 않는다)
///   5. 시드 구간: base_seed ≠ 0 이고 (run, base_seed) 가 지금 구간과 같으면 유지(EPOCH_KEPT), 다르면 새 구간(EPOCH —
///      전부 끄고 frame_seq 0). base_seed 0 + 켜는 명령 + 구간 없음이면 브리지가 시드를 뽑는다(AUTO_SEED)
///   6. 명령 적용. command_seq 는 줄지 않는다. 진행 중인 종류에 같은 값을 다시 보내면 그대로 둔다(UNCHANGED)
class DisturbanceController
{
public:
  struct Options
  {
    bool real_car{false};      ///< follow·localize — 켜는 명령을 거절한다
    bool ground_truth{true};   ///< publish_ground_truth — false 면 객체를 내지 않으므로 켜는 명령을 거절한다
    std::function<uint64_t()> seed_source;  ///< 자동 시드 (비어 있으면 std::random_device). 시험이 고정한다
  };
  struct Counters
  {
    uint64_t accepted{0};
    uint64_t rejected{0};
    uint64_t auto_clears{0};
    std::string last_reject;
  };

  DisturbanceController(DisturbanceHandle & handle, Options options);

  DisturbanceReply handle(const DisturbanceRequest & request);

  /// DAQ 런 종료 때 전부 끈다 (시드 구간·시드·카운터는 그대로). end_reason 이 "replaced" 이고 구간의 run 이 비어 있지
  /// 않으며 끝난 run 과 다르면 건드리지 않는다 (다음 런이 이미 구간을 열었다). 걸려 있던 것이 있었으면 true
  bool clear_on_run_end(const std::string & run, const std::string & end_reason);

  Counters counters() const;

private:
  uint64_t draw_seed();

  DisturbanceHandle & handle_;
  Options options_;
  mutable std::mutex mutex_;
  DisturbanceSettings current_;  ///< 손잡이에 넣은 것과 같은 값 (mutex_ 아래)
  Counters counters_;
};

// =============================================================================
// 프레임 적용
// =============================================================================

struct ObjectDisturbanceRecord
{
  int32_t unique_id{0};     ///< /daq/object_status 의 unique_id 와 같은 값
  std::string name;
  uint64_t seed{0};         ///< object_seed(base_seed, name), base_seed 0 이면 0
  bool visible{true};       ///< 이 프레임에 Autoware 로 보냈다
  double noise_lon_m{0.0};  ///< lon_std × z_lon (자차 앞 +)
  double noise_lat_m{0.0};  ///< lat_std × z_lat (자차 왼 +)
  double dx_m{0.0};         ///< 보낸 position.x − GT position.x (더한 뒤에 잰 값, MORAI world)
  double dy_m{0.0};
  // 26.11.11 _ YSH _ 외란 전·후를 한 칸에 (/daq/object_disturbance_status). 위치 외란 꺼짐·숨김이면 z·d 는 0
  double z_lon{0.0};        ///< 표준정규 난수 gauss2(seed, frame_seq).lon
  double z_lat{0.0};
  double d_lon_m{0.0};      ///< 자차 기준 이동 = lon_offset + noise_lon
  double d_lat_m{0.0};
  morai_grpc_vendor::ActorSample before;  ///< 외란 전 = 같은 스냅샷의 GT
  morai_grpc_vendor::ActorSample after;   ///< 외란 후 = Autoware 에 보낸 것 (visible 일 때만 뜻이 있다)
};

/// 객체 프레임 한 장 = 상태(/daq/disturbance) + Autoware 에 실제로 준 actor 목록
struct DisturbanceFrame
{
  uint64_t frame_seq{0};       ///< 난수 카운터 k
  double frame_period_s{0.1};
  bool fault_flag{false};
  bool position_on{false};
  bool dropout_on{false};
  bool blackout_on{false};     ///< 완전 끊김 진행 중 (자동 복구 뒤에는 false)
  bool objects_visible{true};  ///< false = 이 프레임은 빈 목록
  std::vector<std::string> active_triggers;  ///< position, dropout, blackout 순
  double lon_offset_m{0.0};
  double lon_std_m{0.0};
  double lat_offset_m{0.0};
  double lat_std_m{0.0};
  double keep_prob{1.0};
  double mean_gap_s{0.0};
  double blackout_duration_s{0.0};
  bool blackout_recover{false};
  uint64_t base_seed{0};
  uint64_t dropout_seed{0};
  // 26.11.11 _ YSH _ 위치 외란을 돌릴 때 쓴 자차 (같은 스냅샷) — 분석하는 쪽이 dx·dy 를 다시 계산할 수 있게
  double ego_x{0.0};
  double ego_y{0.0};
  double ego_yaw_deg{0.0};
  std::vector<ObjectDisturbanceRecord> objects;               ///< GT 객체마다 한 칸 (숨김 프레임에도 전부)
  std::vector<morai_grpc_vendor::ActorSample> sent_actors;    ///< Autoware 에 준 것. 숨김 프레임이면 비어 있다
  std::string event;  ///< 상태가 바뀐 프레임에만 (엔진이 한 줄 로그로 남긴다)
};

/// 객체 프레임마다 한 번 부른다 (엔진 스레드만). 프레임 안 순서: 시드 구간·카운터 → 주기 → blackout → dropout → 객체.
///   - frame_seq 는 프레임마다 +1 (보이든 숨기든, 외란이 있든 없든). epoch_id 가 바뀐 첫 프레임이 0
///   - 주기 = 직전 프레임과의 stamp 차. 0 이하·1 s 초과·첫 프레임이면 0.1 s 로 센다
///   - "새 명령" 은 (epoch_id, 종류별 cmd_seq) 로 가린다. epoch_id 가 바뀌거나 그 종류가 꺼지면 본 값을 잊는다
class ObjectDisturber
{
public:
  DisturbanceFrame apply(
    const DisturbanceSettings & settings, const morai_grpc_vendor::Snapshot & snapshot,
    int64_t stamp_ns);

  /// 위와 같되 손잡이에서 설정을 한 번 읽고, 이 프레임에 blackout 이 자동 복구됐으면 손잡이에 알린다 (엔진이 쓴다)
  DisturbanceFrame apply(
    const DisturbanceHandle & handle, const morai_grpc_vendor::Snapshot & snapshot, int64_t stamp_ns);

  /// 마지막으로 자동 복구된 blackout 명령의 cmd_seq (0 = 없음)
  uint64_t blackout_recovered_seq() const { return blackout_recovered_seq_; }

private:
  bool started_{false};
  uint64_t epoch_id_{0};
  uint64_t next_frame_seq_{0};
  bool has_prev_stamp_{false};
  int64_t prev_stamp_ns_{0};

  bool blackout_seen_{false};
  uint64_t blackout_seen_seq_{0};
  bool blackout_active_{false};
  int64_t blackout_elapsed_ns_{0};
  uint64_t blackout_recovered_seq_{0};
  bool blackout_recovered_now_{false};  ///< 이번 프레임에 자동 복구됐다

  bool dropout_seen_{false};
  uint64_t dropout_seen_seq_{0};
  bool dropout_hidden_{false};

  // 로그용 직전 상태
  bool prev_position_on_{false};
  bool prev_dropout_on_{false};
  bool prev_blackout_on_{false};
  bool prev_yaw_bad_{false};  ///< 26.11.11 _ YSH _ 직전 프레임에 자차 yaw 가 유한하지 않아 위치 외란을 건너뛰었다
};

// =============================================================================
// 메시지 변환
// =============================================================================

/// 프레임 → /daq/disturbance. header.stamp = stamp (같은 프레임 TrackedObjects 의 stamp), frame_id map
morai_ros2_msgs::msg::DisturbanceStatus to_disturbance_status_msg(
  const DisturbanceFrame & frame, const builtin_interfaces::msg::Time & stamp);

// 26.11.11 _ YSH _ 신규 — 프레임 → /daq/object_disturbance_status (/daq/object_status 의 외란 짝).
///   객체마다 외란 후(앞 8칸, ObjectStatus 와 같은 계산) + before(같은 프레임 GT) + 시드·난수·잡음·이동량.
///   숨김 객체는 외란 후 heading·velocity·acceleration·position 이 NaN. 목록은 type 으로 나눈다 (to_daq_object_status_list 와 같다)
morai_ros2_msgs::msg::ObjectDisturbanceStatusList to_object_disturbance_status_list(
  const DisturbanceFrame & frame, const builtin_interfaces::msg::Time & stamp, double accel_scale);

/// 읽은 명령 → SetDisturbance 응답의 command. trigger 는 받은 문장 그대로
morai_ros2_msgs::msg::DisturbanceCommand to_disturbance_command_msg(
  const ParsedTrigger & cmd, const std::string & trigger);

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__OBJECT_DISTURBANCE_HPP_
