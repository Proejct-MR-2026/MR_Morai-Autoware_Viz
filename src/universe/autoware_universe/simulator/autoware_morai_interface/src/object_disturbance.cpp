// 신규 — 26.11.10 _ YSH _ 외란 주입 (MRDIST-1) 구현 — 난수·트리거 읽기·명령 제어기·프레임 적용·메시지 변환
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

#include "autoware/morai_interface/object_disturbance.hpp"

// uuid5(object_id_from_name)·unique_id·radians 는 여기 것을 그대로 쓴다 (복제하지 않는다)
#include "autoware/morai_interface/conversion/morai_to_autoware.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <limits>  // 26.11.11 _ YSH _ quiet_NaN (숨김 객체의 외란 후 값)
#include <random>
#include <system_error>
#include <utility>

namespace autoware::morai_interface
{

// =============================================================================
// MRDIST-1
// =============================================================================
namespace mrdist
{

uint64_t mix64(uint64_t z)
{
  z ^= z >> 30;
  z *= 0xBF58476D1CE4E5B9ULL;
  z ^= z >> 27;
  z *= 0x94D049BB133111EBULL;
  z ^= z >> 31;
  return z;
}

uint64_t stream(uint64_t seed, uint64_t counter)
{
  return mix64(seed + counter * kGamma);
}

double u01(uint64_t seed, uint64_t counter)
{
  // 52 비트 정수 + 0.5 는 53 비트라 double 에 그대로 들어가고, 2^-52 곱은 지수만 바꾼다 — 전부 정확하다
  return (static_cast<double>(stream(seed, counter) >> 12) + 0.5) * 0x1.0p-52;
}

uint64_t object_seed(uint64_t base_seed, const std::string & name)
{
  const auto id = morai_to_autoware::object_id_from_name(name);
  uint64_t hi = 0;
  uint64_t lo = 0;
  for (size_t i = 0; i < 8; ++i) {
    hi = (hi << 8) | static_cast<uint64_t>(id.uuid[i]);
    lo = (lo << 8) | static_cast<uint64_t>(id.uuid[8 + i]);
  }
  return mix64(mix64(base_seed ^ kTagObject ^ hi) ^ lo);
}

uint64_t dropout_seed(uint64_t base_seed)
{
  return mix64(base_seed ^ kTagDropout);
}

Gauss2 gauss2(uint64_t seed, uint64_t k)
{
  const double u1 = u01(seed, 2 * k);
  const double u2 = u01(seed, 2 * k + 1);
  const double r = std::sqrt(-2.0 * std::log(u1));
  const double a = 6.283185307179586 * u2;
  Gauss2 z;
  z.lon = r * std::cos(a);
  z.lat = r * std::sin(a);
  return z;
}

DropoutProb dropout_prob(double keep_prob, double mean_gap_s)
{
  DropoutProb p;
  if (!(keep_prob > 0.0)) {  // 늘 숨김 (사슬은 이 값에서 뽑지 않는다)
    p.p_hide = 1.0;
    p.p_show = 0.0;
    return p;
  }
  if (!(keep_prob < 1.0)) {  // 늘 보임
    p.p_hide = 0.0;
    p.p_show = 1.0;
    return p;
  }
  const double t = 0.1;
  const double g = std::max(mean_gap_s, t);
  p.p_show = std::min(1.0, t / g);
  const double mean_on = g * keep_prob / (1.0 - keep_prob);
  p.p_hide = std::min(1.0, t / mean_on);
  return p;
}

}  // namespace mrdist

// =============================================================================
// 트리거 문장
// =============================================================================
namespace
{
constexpr size_t kMaxTriggerLength = 256;
constexpr double kMaxOffsetM = 100.0;
constexpr double kMaxStdM = 100.0;
constexpr double kMaxMeanGapS = 600.0;
constexpr double kMaxDurationS = 3600.0;

bool is_blank(char c)
{
  return c == ' ' || c == '\t';
}

std::string trim_blank(const std::string & text)
{
  size_t begin = 0;
  size_t end = text.size();
  while (begin < end && is_blank(text[begin])) {
    ++begin;
  }
  while (end > begin && is_blank(text[end - 1])) {
    --end;
  }
  return text.substr(begin, end - begin);
}

std::vector<std::string> split_tokens(const std::string & text)
{
  std::vector<std::string> tokens;
  size_t begin = 0;
  while (true) {
    const size_t comma = text.find(',', begin);
    if (comma == std::string::npos) {
      tokens.push_back(trim_blank(text.substr(begin)));
      break;
    }
    tokens.push_back(trim_blank(text.substr(begin, comma - begin)));
    begin = comma + 1;
  }
  return tokens;
}

/// 토큰 전체가 유한한 10 진 실수일 때만 true ('+' 시작, nan, inf, 16 진, 뒤에 붙은 글자는 거절)
bool parse_number(const std::string & token, double & out)
{
  if (token.empty() || token.front() == '+') {
    return false;
  }
  const char * const first = token.data();
  const char * const last = first + token.size();
  double value = 0.0;
  const auto result = std::from_chars(first, last, value, std::chars_format::general);
  if (result.ec != std::errc() || result.ptr != last || !std::isfinite(value)) {
    return false;
  }
  out = (value == 0.0) ? 0.0 : value;  // -0 은 0 으로 (상태에 음의 0 이 남지 않게)
  return true;
}

TriggerParse parse_fail(const char * code, const std::string & detail)
{
  TriggerParse out;
  out.ok = false;
  out.code = code;
  out.detail = detail;
  return out;
}

std::string number_text(double value)
{
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%.9g", value);
  return buf;
}

std::string join(const std::vector<std::string> & parts, const char * sep)
{
  std::string out;
  for (const auto & part : parts) {
    if (!out.empty()) {
      out += sep;
    }
    out += part;
  }
  return out;
}
}  // namespace

const char * to_string(DisturbanceKind kind)
{
  switch (kind) {
    case DisturbanceKind::kClearAll:
      return "clear";
    case DisturbanceKind::kPosition:
      return "position";
    case DisturbanceKind::kDropout:
      return "dropout";
    case DisturbanceKind::kBlackout:
      return "blackout";
    case DisturbanceKind::kNone:
    default:
      return "";
  }
}

TriggerParse parse_disturbance_trigger(const std::string & text)
{
  if (text.size() > kMaxTriggerLength) {
    return parse_fail("TOO_LONG", "trigger is longer than 256 characters");
  }
  TriggerParse out;
  const std::string body = trim_blank(text);
  if (body.empty()) {  // 상태 조회
    out.ok = true;
    return out;
  }

  const std::vector<std::string> tokens = split_tokens(body);
  if (tokens[0] != "disturbance") {
    return parse_fail("BAD_PREFIX", "trigger must start with 'disturbance'");
  }
  if (tokens.size() < 2) {
    return parse_fail("WRONG_ARITY", "kind is missing");
  }

  const std::string & kind = tokens[1];
  if (kind == "clear") {
    if (tokens.size() != 2) {
      return parse_fail("WRONG_ARITY", "'disturbance, clear' takes no value");
    }
    out.cmd.kind = DisturbanceKind::kClearAll;
    out.cmd.clear = true;
    out.ok = true;
    return out;
  }

  size_t arity = 0;
  if (kind == "position") {
    out.cmd.kind = DisturbanceKind::kPosition;
    arity = 4;
  } else if (kind == "dropout") {
    out.cmd.kind = DisturbanceKind::kDropout;
    arity = 2;
  } else if (kind == "blackout") {
    out.cmd.kind = DisturbanceKind::kBlackout;
    arity = 2;
  } else if (kind == "heading" || kind == "mount") {
    return parse_fail("UNSUPPORTED_KIND", "kind '" + kind + "' is not supported yet");
  } else {
    return parse_fail("UNKNOWN_KIND", "kind must be position | dropout | blackout | clear");
  }

  if (tokens.size() == 3 && tokens[2] == "clear") {  // 그 종류만 해제
    out.cmd.clear = true;
    out.ok = true;
    return out;
  }
  if (tokens.size() != 2 + arity) {
    return parse_fail(
      "WRONG_ARITY", kind + " takes " + std::to_string(arity) + " values, got " +
                       std::to_string(tokens.size() - 2));
  }

  // 숫자를 전부 읽은 뒤 범위를 본다
  double values[4] = {0.0, 0.0, 0.0, 0.0};
  for (size_t i = 0; i < arity; ++i) {
    const std::string & token = tokens[2 + i];
    if (out.cmd.kind == DisturbanceKind::kBlackout && i == 1 && (token == "true" || token == "false")) {
      values[i] = (token == "true") ? 1.0 : 0.0;
      continue;
    }
    if (!parse_number(token, values[i])) {
      return parse_fail("BAD_NUMBER", "value " + std::to_string(i + 1) + " is not a finite decimal number");
    }
  }

  ParsedTrigger & cmd = out.cmd;
  switch (cmd.kind) {
    case DisturbanceKind::kPosition:
      cmd.lon_offset_m = values[0];
      cmd.lon_std_m = values[1];
      cmd.lat_offset_m = values[2];
      cmd.lat_std_m = values[3];
      if (std::fabs(cmd.lon_offset_m) > kMaxOffsetM || std::fabs(cmd.lat_offset_m) > kMaxOffsetM) {
        return parse_fail("OUT_OF_RANGE", "|offset| must be <= 100 m");
      }
      if (
        cmd.lon_std_m < 0.0 || cmd.lon_std_m > kMaxStdM || cmd.lat_std_m < 0.0 ||
        cmd.lat_std_m > kMaxStdM) {
        return parse_fail("OUT_OF_RANGE", "std must be within [0, 100] m");
      }
      break;
    case DisturbanceKind::kDropout:
      cmd.keep_prob = values[0];
      cmd.mean_gap_s = values[1];
      if (cmd.keep_prob < 0.0 || cmd.keep_prob > 1.0) {
        return parse_fail("OUT_OF_RANGE", "keep_prob must be within [0, 1]");
      }
      if (cmd.keep_prob > 0.0 && cmd.keep_prob < 1.0) {
        if (!(cmd.mean_gap_s > 0.0) || cmd.mean_gap_s > kMaxMeanGapS) {
          return parse_fail("OUT_OF_RANGE", "mean_gap_s must be within (0, 600] s");
        }
      } else if (cmd.mean_gap_s < 0.0) {
        return parse_fail("OUT_OF_RANGE", "mean_gap_s must be >= 0");
      }
      break;
    case DisturbanceKind::kBlackout:
      cmd.duration_s = values[0];
      if (cmd.duration_s < 0.0 || cmd.duration_s > kMaxDurationS) {
        return parse_fail("OUT_OF_RANGE", "duration_s must be within [0, 3600] s");
      }
      if (values[1] != 0.0 && values[1] != 1.0) {
        return parse_fail("OUT_OF_RANGE", "recover must be 1 or 0");
      }
      cmd.recover = values[1] == 1.0;
      break;
    default:
      break;
  }
  out.ok = true;
  return out;
}

bool is_off_equivalent(const ParsedTrigger & cmd)
{
  if (cmd.clear) {
    return false;
  }
  switch (cmd.kind) {
    case DisturbanceKind::kPosition:
      return cmd.lon_offset_m == 0.0 && cmd.lon_std_m == 0.0 && cmd.lat_offset_m == 0.0 &&
             cmd.lat_std_m == 0.0;
    case DisturbanceKind::kDropout:
      return cmd.keep_prob == 1.0;
    case DisturbanceKind::kBlackout:
      return cmd.recover && cmd.duration_s == 0.0;
    default:
      return false;
  }
}

// 26.11.11 _ YSH _ 신규 — UTF-8 경계에서 자르기 (이어지는 바이트 10xxxxxx 앞에서 끊지 않는다)
std::string utf8_prefix(const std::string & text, size_t max_bytes)
{
  if (text.size() <= max_bytes) {
    return text;
  }
  size_t end = max_bytes;
  while (end > 0 && (static_cast<unsigned char>(text[end]) & 0xC0U) == 0x80U) {
    --end;
  }
  return text.substr(0, end);
}

// =============================================================================
// 명령 받기
// =============================================================================

DisturbanceController::DisturbanceController(DisturbanceHandle & handle, Options options)
: handle_(handle), options_(std::move(options)), current_(*handle.get())
{
}

uint64_t DisturbanceController::draw_seed()
{
  uint64_t value = 0;
  if (options_.seed_source) {
    value = options_.seed_source();
  } else {
    std::random_device device;
    const uint64_t high = device();
    const uint64_t low = device();
    value = (high << 32) | (low & 0xFFFFFFFFULL);
  }
  if (value == 0 || value > mrdist::kMaxBaseSeed) {
    value = 1 + value % mrdist::kMaxBaseSeed;  // 1 ~ kMaxBaseSeed
  }
  return value;
}

DisturbanceReply DisturbanceController::handle(const DisturbanceRequest & request)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  DisturbanceReply reply;
  reply.run = current_.run;
  reply.base_seed = current_.base_seed;
  const auto reject = [this, &reply](const std::string & code, const std::string & detail) {
    reply.ok = false;
    reply.message = code + ": " + detail;
    ++counters_.rejected;
    counters_.last_reject = reply.message;
    return reply;
  };

  // 1. 길이·문법·범위
  const TriggerParse parsed = parse_disturbance_trigger(request.trigger);
  if (!parsed.ok) {
    return reject(parsed.code, parsed.detail);
  }
  const ParsedTrigger & cmd = parsed.cmd;
  reply.command = cmd;

  // 2. 시드 범위
  if (request.base_seed > mrdist::kMaxBaseSeed) {
    return reject("BAD_SEED", "base_seed must be within 1 ~ 999999999999999 (0 = keep)");
  }

  // 3. 켜는 명령인가 (해제·끔 값·상태 조회는 아니다)
  const bool is_kind = cmd.kind == DisturbanceKind::kPosition ||
                       cmd.kind == DisturbanceKind::kDropout ||
                       cmd.kind == DisturbanceKind::kBlackout;
  const bool off_value = is_kind && is_off_equivalent(cmd);
  const bool effective_on = is_kind && !cmd.clear && !off_value;
  if (effective_on && options_.real_car) {
    return reject("REAL_CAR", "disturbance cannot be switched on in a real-car mode (follow, localize)");
  }
  if (effective_on && !options_.ground_truth) {
    return reject("NO_GROUND_TRUTH", "ground-truth objects are not published (publish_ground_truth false)");
  }

  // 4. run 표시 — 켜는 명령만 본다
  const bool has_epoch = current_.base_seed != 0;
  if (
    effective_on && request.base_seed == 0 && !request.run.empty() && has_epoch &&
    !current_.run.empty() && current_.run != request.run) {
    return reject(
      "RUN_MISMATCH", "command run '" + request.run + "' differs from the seed epoch run '" +
                        current_.run + "'");
  }

  // 5. 시드 구간
  DisturbanceSettings next = current_;
  std::vector<std::string> tokens;
  bool changed = false;
  const auto start_epoch = [this, &next, &changed, &request](uint64_t seed) {
    DisturbanceSettings fresh;  // 전부 꺼진 새 구간
    fresh.epoch_id = current_.epoch_id + 1;
    fresh.base_seed = seed;
    fresh.run = request.run;
    fresh.command_seq = current_.command_seq;  // 명령 번호는 이어 센다
    next = fresh;
    changed = true;
  };
  if (request.base_seed != 0) {
    if (has_epoch && current_.run == request.run && current_.base_seed == request.base_seed) {
      tokens.emplace_back("EPOCH_KEPT");
    } else {
      start_epoch(request.base_seed);
      tokens.emplace_back("EPOCH");
    }
  } else if (effective_on && !has_epoch) {
    start_epoch(draw_seed());
    tokens.emplace_back("AUTO_SEED");
  }

  // 6. 명령 적용
  const std::string trigger = trim_blank(request.trigger);
  switch (cmd.kind) {
    case DisturbanceKind::kNone:
      tokens.emplace_back("STATUS");
      break;
    case DisturbanceKind::kClearAll:
      next.position = DisturbanceSettings::Position{};
      next.dropout = DisturbanceSettings::Dropout{};
      next.blackout = DisturbanceSettings::Blackout{};
      next.command_seq += 1;
      changed = true;
      break;
    case DisturbanceKind::kPosition:
      if (cmd.clear || off_value) {
        next.position = DisturbanceSettings::Position{};
        next.command_seq += 1;
        changed = true;
        if (off_value) {
          tokens.emplace_back("POSITION_OFF");
        }
      } else if (
        next.position.on && next.position.lon_offset_m == cmd.lon_offset_m &&
        next.position.lon_std_m == cmd.lon_std_m && next.position.lat_offset_m == cmd.lat_offset_m &&
        next.position.lat_std_m == cmd.lat_std_m) {
        tokens.emplace_back("UNCHANGED");
      } else {
        next.command_seq += 1;
        next.position.on = true;
        next.position.lon_offset_m = cmd.lon_offset_m;
        next.position.lon_std_m = cmd.lon_std_m;
        next.position.lat_offset_m = cmd.lat_offset_m;
        next.position.lat_std_m = cmd.lat_std_m;
        next.position.trigger = trigger;
        next.position.cmd_seq = next.command_seq;
        changed = true;
      }
      break;
    case DisturbanceKind::kDropout:
      if (cmd.clear || off_value) {
        next.dropout = DisturbanceSettings::Dropout{};
        next.command_seq += 1;
        changed = true;
        if (off_value) {
          tokens.emplace_back("DROPOUT_OFF");
        }
      } else {
        if (
          next.dropout.on && next.dropout.keep_prob == cmd.keep_prob &&
          next.dropout.mean_gap_s == cmd.mean_gap_s) {
          tokens.emplace_back("UNCHANGED");
        } else {
          next.command_seq += 1;
          next.dropout.on = true;
          next.dropout.keep_prob = cmd.keep_prob;
          next.dropout.mean_gap_s = cmd.mean_gap_s;
          next.dropout.trigger = trigger;
          next.dropout.cmd_seq = next.command_seq;
          changed = true;
        }
        // 10 Hz 격자로 못 내는 조합 (끊김 평균 < 0.1 s, 또는 보이는 구간 평균 < 0.1 s). 0 < keep < 1 에서만 따진다
        if (
          cmd.keep_prob > 0.0 && cmd.keep_prob < 1.0 &&
          (cmd.mean_gap_s < 0.1 || cmd.mean_gap_s * cmd.keep_prob / (1.0 - cmd.keep_prob) < 0.1)) {
          tokens.emplace_back("SATURATED");
        }
      }
      break;
    case DisturbanceKind::kBlackout:
      if (cmd.clear || off_value) {
        next.blackout = DisturbanceSettings::Blackout{};
        next.command_seq += 1;
        changed = true;
        if (off_value) {
          tokens.emplace_back("BLACKOUT_OFF");
        }
      } else if (
        // 진행 중 = 걸려 있고 아직 자동 복구되지 않았다 (엔진이 복구를 손잡이로 알린다)
        next.blackout.on && handle_.blackout_recovered_seq() != next.blackout.cmd_seq &&
        next.blackout.duration_s == cmd.duration_s && next.blackout.recover == cmd.recover) {
        tokens.emplace_back("UNCHANGED");
      } else {
        next.command_seq += 1;
        next.blackout.on = true;
        next.blackout.duration_s = cmd.duration_s;
        next.blackout.recover = cmd.recover;
        next.blackout.trigger = trigger;
        next.blackout.cmd_seq = next.command_seq;
        changed = true;
      }
      break;
    default:
      break;
  }

  // 7. 저장
  if (changed) {
    current_ = next;
    handle_.set(next);
  }
  reply.ok = true;
  reply.message = tokens.empty() ? std::string("OK") : "OK " + join(tokens, " ");
  reply.run = current_.run;
  reply.base_seed = current_.base_seed;
  ++counters_.accepted;
  return reply;
}

bool DisturbanceController::clear_on_run_end(const std::string & run, const std::string & end_reason)
{
  const std::lock_guard<std::mutex> lock(mutex_);
  if (end_reason == "replaced" && !current_.run.empty() && current_.run != run) {
    return false;  // 다음 런이 이미 시드 구간을 열었다 — 그 런의 외란을 지우지 않는다
  }
  if (!current_.position.on && !current_.dropout.on && !current_.blackout.on) {
    return false;
  }
  // 자동 복구로 이미 끝난 blackout 만 남아 있었으면 "걸려 있던 것" 으로 치지 않는다 (설정은 같이 정리한다)
  const bool blackout_live =
    current_.blackout.on && handle_.blackout_recovered_seq() != current_.blackout.cmd_seq;
  const bool was_on = current_.position.on || current_.dropout.on || blackout_live;

  DisturbanceSettings next = current_;
  next.position = DisturbanceSettings::Position{};
  next.dropout = DisturbanceSettings::Dropout{};
  next.blackout = DisturbanceSettings::Blackout{};
  next.command_seq += 1;
  current_ = next;
  handle_.set(next);
  if (was_on) {
    ++counters_.auto_clears;
  }
  return was_on;
}

DisturbanceController::Counters DisturbanceController::counters() const
{
  const std::lock_guard<std::mutex> lock(mutex_);
  return counters_;
}

// =============================================================================
// 프레임 적용
// =============================================================================

DisturbanceFrame ObjectDisturber::apply(
  const DisturbanceSettings & settings, const morai_grpc_vendor::Snapshot & snapshot, int64_t stamp_ns)
{
  DisturbanceFrame frame;
  std::vector<std::string> events;
  blackout_recovered_now_ = false;

  // --- 1. 시드 구간·카운터 ---
  if (settings.epoch_id != epoch_id_) {
    epoch_id_ = settings.epoch_id;
    next_frame_seq_ = 0;
    blackout_seen_ = false;
    blackout_active_ = false;
    blackout_elapsed_ns_ = 0;
    dropout_seen_ = false;
    dropout_hidden_ = false;
    prev_position_on_ = false;
    prev_dropout_on_ = false;
    prev_blackout_on_ = false;
    events.push_back(
      "시드 구간 " + std::to_string(settings.epoch_id) + " 시작 (base_seed " +
      std::to_string(settings.base_seed) + ", run '" + settings.run + "')");
  }
  const uint64_t k = next_frame_seq_++;
  frame.frame_seq = k;
  frame.base_seed = settings.base_seed;
  frame.dropout_seed = settings.base_seed != 0 ? mrdist::dropout_seed(settings.base_seed) : 0;

  // --- 2. 주기 (0 이하·1 s 초과·첫 프레임은 명목 0.1 s) ---
  int64_t period_ns = mrdist::kNominalPeriodNs;
  if (has_prev_stamp_) {
    const int64_t delta = stamp_ns - prev_stamp_ns_;
    if (delta > 0 && delta <= mrdist::kMaxPeriodNs) {
      period_ns = delta;
    }
  }
  prev_stamp_ns_ = stamp_ns;
  has_prev_stamp_ = true;
  frame.frame_period_s = static_cast<double>(period_ns) * 1e-9;

  // --- 3. blackout: 새 명령을 본 프레임부터 숨긴다 (경과 0), 뒤 프레임마다 주기를 더한다 ---
  if (!settings.blackout.on) {
    blackout_seen_ = false;
    blackout_active_ = false;
    blackout_elapsed_ns_ = 0;
  } else if (!blackout_seen_ || blackout_seen_seq_ != settings.blackout.cmd_seq) {
    blackout_seen_ = true;
    blackout_seen_seq_ = settings.blackout.cmd_seq;
    blackout_active_ = true;
    blackout_elapsed_ns_ = 0;
    events.push_back("blackout 시작 [" + settings.blackout.trigger + "]");
  } else if (blackout_active_) {
    blackout_elapsed_ns_ += period_ns;
    const auto duration_ns = static_cast<int64_t>(std::floor(settings.blackout.duration_s * 1e9 + 0.5));
    if (settings.blackout.recover && blackout_elapsed_ns_ >= duration_ns) {
      blackout_active_ = false;  // 이 프레임부터 보인다
      blackout_recovered_seq_ = settings.blackout.cmd_seq;
      blackout_recovered_now_ = true;
      events.push_back(
        "blackout 자동 복구 (" + number_text(static_cast<double>(blackout_elapsed_ns_) * 1e-9) + " s)");
    }
  }

  // --- 4. dropout: 모든 객체가 같이 쓰는 두 상태 사슬. blackout 중에도 계속 돈다 ---
  if (!settings.dropout.on) {
    dropout_seen_ = false;
    dropout_hidden_ = false;
  } else if (!dropout_seen_ || dropout_seen_seq_ != settings.dropout.cmd_seq) {
    dropout_seen_ = true;
    dropout_seen_seq_ = settings.dropout.cmd_seq;
    dropout_hidden_ = !(settings.dropout.keep_prob > 0.0);  // 건 프레임은 뽑지 않는다 (keep 0 이면 바로 숨김)
    events.push_back("dropout 시작 [" + settings.dropout.trigger + "]");
  } else if (settings.dropout.keep_prob > 0.0 && settings.dropout.keep_prob < 1.0) {
    const auto prob = mrdist::dropout_prob(settings.dropout.keep_prob, settings.dropout.mean_gap_s);
    const double u = mrdist::u01(frame.dropout_seed, k);
    if (dropout_hidden_) {
      if (u < prob.p_show) {
        dropout_hidden_ = false;
      }
    } else if (u < prob.p_hide) {
      dropout_hidden_ = true;
    }
  }

  const bool hidden = blackout_active_ || dropout_hidden_;

  // --- 상태 값 (꺼진 종류는 기본값) ---
  frame.position_on = settings.position.on;
  frame.dropout_on = settings.dropout.on;
  frame.blackout_on = blackout_active_;
  frame.fault_flag = frame.position_on || frame.dropout_on || frame.blackout_on;
  frame.objects_visible = !hidden;
  if (frame.position_on) {
    frame.lon_offset_m = settings.position.lon_offset_m;
    frame.lon_std_m = settings.position.lon_std_m;
    frame.lat_offset_m = settings.position.lat_offset_m;
    frame.lat_std_m = settings.position.lat_std_m;
    frame.active_triggers.push_back(settings.position.trigger);
  }
  if (frame.dropout_on) {
    frame.keep_prob = settings.dropout.keep_prob;
    frame.mean_gap_s = settings.dropout.mean_gap_s;
    frame.active_triggers.push_back(settings.dropout.trigger);
  }
  if (frame.blackout_on) {
    frame.blackout_duration_s = settings.blackout.duration_s;
    frame.blackout_recover = settings.blackout.recover;
    frame.active_triggers.push_back(settings.blackout.trigger);
  }

  // --- 5. 객체 ---
  const double psi = morai_to_autoware::radians(snapshot.ego.rotation_deg.z);  // 같은 스냅샷의 자차 yaw
  const double cos_psi = std::cos(psi);
  const double sin_psi = std::sin(psi);
  // 26.11.11 _ YSH _ 자차 yaw 가 NaN·inf 면 이 프레임은 위치를 옮기지 않는다 — NaN 좌표를 Autoware 에 주지 않는다.
  //   객체는 GT 그대로, 잡음·이동량 칸은 0 (D2 의 보낸 값 − GT = 이동량 은 그대로 맞는다). 로그는 바뀔 때 한 번
  const bool yaw_ok = std::isfinite(psi);
  frame.ego_x = snapshot.ego.position.x;          // 26.11.11 _ YSH _ 다시 계산용 (같은 스냅샷)
  frame.ego_y = snapshot.ego.position.y;
  frame.ego_yaw_deg = snapshot.ego.rotation_deg.z;
  if (frame.position_on && !yaw_ok && !prev_yaw_bad_) {
    events.emplace_back("자차 yaw 가 유한하지 않다 — 위치 외란을 건너뛴다");
  } else if (prev_yaw_bad_ && (yaw_ok || !frame.position_on)) {
    events.emplace_back("자차 yaw 회복 — 위치 외란 다시 적용");
  }
  prev_yaw_bad_ = frame.position_on && !yaw_ok;
  frame.objects.reserve(snapshot.actors.size());
  if (!hidden) {
    frame.sent_actors.reserve(snapshot.actors.size());
  }
  for (const auto & actor : snapshot.actors) {
    ObjectDisturbanceRecord record;
    record.unique_id = morai_to_autoware::object_unique_id_from_name(actor.id);
    record.name = actor.id;
    record.seed = settings.base_seed != 0 ? mrdist::object_seed(settings.base_seed, actor.id) : 0;
    record.visible = !hidden;
    record.before = actor;  // 26.11.11 _ YSH _ 외란 전 (같은 스냅샷 GT)
    if (!hidden) {
      morai_grpc_vendor::ActorSample sent = actor;  // 스냅샷은 고치지 않는다
      // 26.11.11 _ YSH _ 자차 yaw 가 유한할 때만 옮긴다. 원문: if (settings.position.on) {
      if (settings.position.on && yaw_ok) {
        const auto z = mrdist::gauss2(record.seed, k);
        const double noise_lon =
          settings.position.lon_std_m > 0.0 ? settings.position.lon_std_m * z.lon : 0.0;
        const double noise_lat =
          settings.position.lat_std_m > 0.0 ? settings.position.lat_std_m * z.lat : 0.0;
        const double d_lon = settings.position.lon_offset_m + noise_lon;
        const double d_lat = settings.position.lat_offset_m + noise_lat;
        const double wx = d_lon * cos_psi - d_lat * sin_psi;
        const double wy = d_lon * sin_psi + d_lat * cos_psi;
        sent.position.x += wx;
        sent.position.y += wy;
        // TrackedObject pose 는 상자 중심(bbox_location)이 있으면 그것을 쓴다 (to_tracked_object 와 같은 판정) — 같이 옮긴다
        if (sent.bbox_location.x != 0.0 || sent.bbox_location.y != 0.0 || sent.bbox_location.z != 0.0) {
          sent.bbox_location.x += wx;
          sent.bbox_location.y += wy;
        }
        record.noise_lon_m = noise_lon;
        record.noise_lat_m = noise_lat;
        record.z_lon = z.lon;  // 26.11.11 _ YSH _
        record.z_lat = z.lat;
        record.d_lon_m = d_lon;
        record.d_lat_m = d_lat;
        // 더한 뒤에 잰다 — 그래야 (보낸 값 − GT) 와 비트까지 같다
        record.dx_m = sent.position.x - actor.position.x;
        record.dy_m = sent.position.y - actor.position.y;
      }
      record.after = sent;  // 26.11.11 _ YSH _ 외란 후 (Autoware 에 보낸 것)
      frame.sent_actors.push_back(std::move(sent));
    }
    frame.objects.push_back(std::move(record));
  }

  // --- 로그용 사건 ---
  if (frame.position_on != prev_position_on_) {
    events.push_back(
      frame.position_on ? "position 켬 [" + settings.position.trigger + "]" : std::string("position 끔"));
  }
  if (!frame.dropout_on && prev_dropout_on_) {
    events.emplace_back("dropout 끔");
  }
  if (!frame.blackout_on && prev_blackout_on_ && !blackout_recovered_now_) {
    events.emplace_back("blackout 끔");
  }
  prev_position_on_ = frame.position_on;
  prev_dropout_on_ = frame.dropout_on;
  prev_blackout_on_ = frame.blackout_on;
  if (!events.empty()) {
    frame.event = "frame_seq " + std::to_string(k) + " — " + join(events, ", ");
  }
  return frame;
}

DisturbanceFrame ObjectDisturber::apply(
  const DisturbanceHandle & handle, const morai_grpc_vendor::Snapshot & snapshot, int64_t stamp_ns)
{
  const auto settings = handle.get();  // 객체 프레임마다 한 번
  DisturbanceFrame frame = apply(*settings, snapshot, stamp_ns);
  if (blackout_recovered_now_) {
    handle.note_blackout_recovered(blackout_recovered_seq_);
  }
  return frame;
}

// =============================================================================
// 메시지 변환
// =============================================================================

morai_ros2_msgs::msg::DisturbanceStatus to_disturbance_status_msg(
  const DisturbanceFrame & frame, const builtin_interfaces::msg::Time & stamp)
{
  morai_ros2_msgs::msg::DisturbanceStatus msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  msg.frame_seq = frame.frame_seq;
  msg.frame_period_s = frame.frame_period_s;
  msg.fault_flag = frame.fault_flag;
  msg.position_on = frame.position_on;
  msg.dropout_on = frame.dropout_on;
  msg.blackout_on = frame.blackout_on;
  msg.objects_visible = frame.objects_visible;
  msg.active_triggers = frame.active_triggers;
  msg.lon_offset_m = frame.lon_offset_m;
  msg.lon_std_m = frame.lon_std_m;
  msg.lat_offset_m = frame.lat_offset_m;
  msg.lat_std_m = frame.lat_std_m;
  msg.keep_prob = frame.keep_prob;
  msg.mean_gap_s = frame.mean_gap_s;
  msg.blackout_duration_s = frame.blackout_duration_s;
  msg.blackout_recover = frame.blackout_recover;
  msg.base_seed = frame.base_seed;
  msg.dropout_seed = frame.dropout_seed;
  msg.seed_algorithm = mrdist::kAlgorithm;
  msg.objects.reserve(frame.objects.size());
  for (const auto & record : frame.objects) {
    morai_ros2_msgs::msg::ObjectDisturbance obj;
    obj.unique_id = record.unique_id;
    obj.name = record.name;
    obj.seed = record.seed;
    obj.visible = record.visible;
    obj.noise_lon_m = record.noise_lon_m;
    obj.noise_lat_m = record.noise_lat_m;
    obj.dx_m = record.dx_m;
    obj.dy_m = record.dy_m;
    msg.objects.push_back(std::move(obj));
  }
  return msg;
}

// 26.11.11 _ YSH _ 신규 (헤더 설명 참고)
morai_ros2_msgs::msg::ObjectDisturbanceStatusList to_object_disturbance_status_list(
  const DisturbanceFrame & frame, const builtin_interfaces::msg::Time & stamp, double accel_scale)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  morai_ros2_msgs::msg::ObjectDisturbanceStatusList msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "map";
  for (const auto & record : frame.objects) {
    morai_ros2_msgs::msg::ObjectDisturbanceStatus o;
    o.before = morai_to_autoware::to_daq_object_status(record.before, accel_scale);
    if (record.visible) {
      const auto after = morai_to_autoware::to_daq_object_status(record.after, accel_scale);
      o.unique_id = after.unique_id;
      o.type = after.type;
      o.name = after.name;
      o.heading = after.heading;
      o.velocity = after.velocity;
      o.acceleration = after.acceleration;
      o.size = after.size;
      o.position = after.position;
    } else {  // 보낸 값이 없다 — 이름·종류·크기만 남기고 나머지는 NaN
      o.unique_id = o.before.unique_id;
      o.type = o.before.type;
      o.name = o.before.name;
      o.size = o.before.size;
      o.heading = nan;
      o.velocity.x = o.velocity.y = o.velocity.z = nan;
      o.acceleration.x = o.acceleration.y = o.acceleration.z = nan;
      o.position.x = o.position.y = o.position.z = nan;
    }
    o.visible = record.visible;
    o.seed = record.seed;
    o.z_lon = record.z_lon;
    o.z_lat = record.z_lat;
    o.noise_lon_m = record.noise_lon_m;
    o.noise_lat_m = record.noise_lat_m;
    o.d_lon_m = record.d_lon_m;
    o.d_lat_m = record.d_lat_m;
    o.dx_m = record.dx_m;
    o.dy_m = record.dy_m;
    switch (o.before.type) {
      case 1:
        msg.npc_list.push_back(std::move(o));
        break;
      case 0:
        msg.pedestrian_list.push_back(std::move(o));
        break;
      default:
        msg.obstacle_list.push_back(std::move(o));
        break;
    }
  }
  msg.num_of_npcs = static_cast<int32_t>(msg.npc_list.size());
  msg.num_of_pedestrian = static_cast<int32_t>(msg.pedestrian_list.size());
  msg.num_of_obstacle = static_cast<int32_t>(msg.obstacle_list.size());
  msg.frame_seq = frame.frame_seq;
  msg.base_seed = frame.base_seed;
  msg.ego_x = frame.ego_x;
  msg.ego_y = frame.ego_y;
  msg.ego_yaw_deg = frame.ego_yaw_deg;
  return msg;
}

morai_ros2_msgs::msg::DisturbanceCommand to_disturbance_command_msg(
  const ParsedTrigger & cmd, const std::string & trigger)
{
  morai_ros2_msgs::msg::DisturbanceCommand msg;
  msg.trigger = trigger;
  msg.type = to_string(cmd.kind);
  msg.clear = cmd.clear;
  msg.lon_offset_m = cmd.lon_offset_m;
  msg.lon_std_m = cmd.lon_std_m;
  msg.lat_offset_m = cmd.lat_offset_m;
  msg.lat_std_m = cmd.lat_std_m;
  msg.keep_prob = cmd.keep_prob;
  msg.mean_gap_s = cmd.mean_gap_s;
  msg.duration_s = cmd.duration_s;
  msg.recover = cmd.recover;
  msg.angle_offset_deg = 0.0;  // 예약 (heading, mount)
  msg.angle_std_deg = 0.0;
  return msg;
}

}  // namespace autoware::morai_interface
