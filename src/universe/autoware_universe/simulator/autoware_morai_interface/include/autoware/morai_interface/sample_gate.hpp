// 신규 — 26.09.25 _ YSH _ 스냅샷 유효성·신선도 판정과 구간 통계 (엔진이 쓰는 순수 로직)
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

#ifndef AUTOWARE__MORAI_INTERFACE__SAMPLE_GATE_HPP_
#define AUTOWARE__MORAI_INTERFACE__SAMPLE_GATE_HPP_

#include <morai_grpc_vendor/morai_link_types.hpp>

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace autoware::morai_interface
{

/// 스냅샷 한 개의 판정
enum class SampleVerdict {
  kNew,         ///< 새 업데이트 — 보내고 발행한다
  kRebased,     ///< 역행이 이어지거나(3 표본) 크게(>0.5 s) 역행해 새 기준으로 받아들였다 — kNew 처럼 처리
  kDuplicate,   ///< 직전과 같은 업데이트 — 보내지도 발행하지도 않는다
  kBackward,    ///< timestamp 가 직전보다 작다 — 건너뛴다 (곧 새 기준이 된다)
  kNoEgo,       ///< 스냅샷에 Ego 가 없다 (시나리오 교체 등)
  kInvalidEgo,  ///< daq_publisher.ego_is_valid 가 거짓 (위치·자세·속도 9 값이 전부 0 — 미스폰·리셋)
};

const char * to_string(SampleVerdict v);

/// 무엇으로 "새 업데이트" 를 가르나
enum class GateMode {
  kTimestamp,  ///< ActorStates.timestamp 가 늘면 새 업데이트 (실측 빌드의 정상 경로)
  /// timestamp 를 쓸 수 없다 (0 이거나, 그대로인데 Ego 내용이 바뀐다) → Ego 내용이 바뀌거나 트리거가
  /// 새 틱을 확인하면 새 업데이트. stamp 는 수신 시각을 써야 한다 (엔진이 처리)
  kContent,
};

const char * to_string(GateMode m);

struct SampleGateConfig
{
  int rebase_after_backward{3};          ///< 서로 다른 역행 표본이 이만큼 이어지면 새 기준 (~60 ms)
  int64_t rebase_jump_us{500000};        ///< 한 번에 이보다 크게 역행하면 곧바로 새 기준 (MORAI 재시작·시계 계단)
  int frozen_content_changes{3};         ///< timestamp 가 그대로인데 Ego 내용이 이만큼 바뀌면 멈춘 timestamp
  int64_t content_min_period_ns{15000000};  ///< 내용 모드: 같은 내용을 트리거 확인으로 받는 최소 간격
  int timestamp_recover_after{5};        ///< 내용 모드: timestamp 가 연속 이만큼 늘면 timestamp 모드로 복귀
};

/// timestamp 는 늘어나야 하고, 같으면 건너뛴다. 역행은 짧게만 건너뛰고 새 기준으로 받아들인다 — 오래
/// 기다리면 MORAI 호스트 시계가 뒤로 계단을 탈 때(chrony makestep, Windows 시간 서비스) 그동안 Autoware
/// 명령이 MORAI 로 가지 않는다 (MORAI 는 마지막 명령을 유지한다 — 실측).
///
/// timestamp 가 0 이거나, 그대로인데 Ego 내용이 계속 바뀌면(timestamp 를 채우지 않는 빌드) 내용 모드로
/// 바꾼다 — 그러지 않으면 첫 표본 뒤 영원히 송신·발행이 멈춘다. 내용 모드에서는 Ego 내용이 바뀌면 새
/// 업데이트이고, 내용이 같아도 트리거가 새 틱을 확인했으면(WaitForTick·lockstep Tick) 최소 간격을 두고
/// 새 업데이트로 받는다 (정지한 차도 명령을 받게). 폴링 폴백에서 같은 내용은 중복이다.
class SampleGate
{
public:
  explicit SampleGate(const SampleGateConfig & config = SampleGateConfig{}) : config_(config) {}

  /// tick_confirmed: 트리거가 이 스냅샷 직전에 새 MORAI 업데이트를 확인했다 (WaitForTick 성공 등)
  SampleVerdict check(
    const morai_grpc_vendor::Snapshot & snapshot, int64_t now_steady_ns, bool tick_confirmed = false);

  bool has_last() const { return has_last_; }
  GateMode mode() const { return mode_; }
  int64_t last_timestamp_us() const { return last_ts_us_; }
  /// 직전에 받아들인 표본과의 간격 [us] (timestamp 모드는 MORAI 시간, 내용 모드는 수신 steady 시간.
  /// 첫 표본·새 기준 뒤면 nullopt)
  std::optional<int64_t> last_step_us() const { return last_step_us_; }
  /// 마지막 check 에서 모드가 바뀌었으면 그 이유 (한 번만 돌려준다 — 로그용)
  std::optional<std::string> take_mode_change();

private:
  SampleVerdict check_timestamp(
    const morai_grpc_vendor::EgoSample & ego, int64_t ts, int64_t now, bool tick_confirmed);
  SampleVerdict check_content(
    const morai_grpc_vendor::EgoSample & ego, int64_t ts, int64_t now, bool tick_confirmed);
  void enter_content(const std::string & why);
  void accept_timestamp(int64_t ts);

  SampleGateConfig config_;
  GateMode mode_{GateMode::kTimestamp};
  bool has_last_{false};
  int64_t last_ts_us_{0};
  std::optional<int64_t> last_step_us_;
  // timestamp 모드
  int backward_streak_{0};
  int64_t last_backward_ts_us_{0};
  int frozen_changes_{0};
  bool has_seen_{false};
  morai_grpc_vendor::EgoSample last_seen_;  ///< 직전 유효 스냅샷의 Ego (내용 비교)
  // 내용 모드
  bool content_has_last_{false};
  morai_grpc_vendor::EgoSample last_accepted_;
  int64_t last_accept_ns_{0};
  int64_t content_last_ts_us_{0};
  int timestamp_advancing_{0};
  std::optional<std::string> mode_change_;
};

/// Ego 표본의 "내용" 이 같은가 (위치·자세·속도·각속도·가속도·차량 상태). timestamp·수신 시각은 보지 않는다
bool same_ego_content(const morai_grpc_vendor::EgoSample & a, const morai_grpc_vendor::EgoSample & b);

/// 고정 길이 원형 버퍼 + 분위수 (스레드 안전). 진단·10 초 통계용
class RollingWindow
{
public:
  struct Summary
  {
    size_t count{0};
    double min{0.0};
    double p01{0.0};
    double p50{0.0};
    double p99{0.0};
    double max{0.0};
  };

  explicit RollingWindow(size_t capacity);
  void add(double value);
  /// 비어 있으면 count 0 (나머지 0)
  Summary summary() const;
  void clear();

private:
  mutable std::mutex mutex_;
  std::vector<double> buffer_;
  size_t capacity_;
  size_t next_{0};
};

/// 정렬된 값의 분위수 (가장 가까운 순위, p 는 0~1)
double percentile_sorted(const std::vector<double> & sorted, double p);

}  // namespace autoware::morai_interface

#endif  // AUTOWARE__MORAI_INTERFACE__SAMPLE_GATE_HPP_
