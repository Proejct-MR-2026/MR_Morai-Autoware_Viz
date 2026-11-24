// 신규 — 26.09.25 _ YSH _ morai_actuate 가 부를 수 있는 RPC 허용 목록 + 조향 0 강제 (소스 스캔)
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

// morai_actuate is state-changing, so it is NOT covered by test_probe_readonly (which scans
// only tools/morai_probe.cpp). This test bounds what it may change instead: control mode,
// gear, and ControlVehicle with steer 0 — nothing else.

#include <gtest/gtest.h>

// 26.09.25 _ YSH _ 디렉토리 스캔(동기 모드 RPC 가 허용된 두 파일에만 있는지)용
#include <filesystem>
#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#ifndef MORAI_ACTUATE_SOURCE
#error "MORAI_ACTUATE_SOURCE must point at tools/morai_actuate.cpp"
#endif
// 26.09.25 _ YSH _ lockstep 경로·비상 복구 도구·tools 디렉토리 경로
#ifndef MORAI_ACTUATE_LOCKSTEP_SOURCE
#error "MORAI_ACTUATE_LOCKSTEP_SOURCE must point at tools/actuate_lockstep.cpp"
#endif
#ifndef MORAI_SYNC_RESTORE_SOURCE
#error "MORAI_SYNC_RESTORE_SOURCE must point at tools/morai_sync_restore.cpp"
#endif
#ifndef MORAI_TOOLS_DIR
#error "MORAI_TOOLS_DIR must point at tools/"
#endif

namespace
{

/// Every RPC morai_actuate may issue. The three state-changing ones are the minimum the
/// measurement needs; each is sent with a deadline and a Result.status check.
const std::set<std::string> kAllowedRpcs = {
  // state-changing
  "ControlVehicle", "AsyncControlVehicle", "SetVehicleControlMode", "SetVehicleGear",
  // read-only
  "GetAllActorsState", "GetVehicleControlMode", "GetVehicleSpec", "GetSynchronousMode",
  "WaitForTick", "GetSimulatorVersion"};

std::string code_only()
{
  std::ifstream f(MORAI_ACTUATE_SOURCE);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  s = std::regex_replace(s, std::regex(R"("(?:[^"\\\n]|\\.)*")"), "\"\"");
  s = std::regex_replace(s, std::regex(R"(//[^\n]*)"), "");
  s = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), "");
  return s;
}

TEST(ActuateScope, SourceIsReadable)
{
  EXPECT_GT(code_only().size(), 1000u) << MORAI_ACTUATE_SOURCE;
}

TEST(ActuateScope, EveryStubCallIsAllowListed)
{
  const std::string src = code_only();
  const std::regex call(R"((\w+_stub_)\s*->\s*(\w+)\s*\()");
  std::set<std::string> seen;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), call); it != std::sregex_iterator();
       ++it) {
    const std::string method = (*it)[2].str();
    seen.insert(method);
    EXPECT_TRUE(kAllowedRpcs.count(method)) << "non-allow-listed RPC in morai_actuate: " << method;
  }
  // Proves the regex matches this file's call style.
  EXPECT_EQ(seen, kAllowedRpcs);
}

TEST(ActuateScope, NoOtherStateChangingRpcNameAppearsAsACall)
{
  const std::string src = code_only();
  const std::regex forbidden(
    R"(\b(Set(?!MaxReceiveMessageSize|VehicleControlMode|VehicleGear|Int\b)[A-Z]\w*|)"
    R"(Control(Pedestrian|Obstacle)|Tick|Start|Stop|Pause|Resume|Spawn\w*|Destroy\w*|)"
    R"(SendMessage|Resim\w*|AddResim\w*|Load\w*|Create\w*SpawnPoint|Enable\w*|Raycast|)"
    R"(Add\w*Sensor|Remove\w*)\s*\()");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, forbidden)) << "found: " << m.str();
}

TEST(ActuateScope, SteeringIsAlwaysZero)
{
  const std::string src = code_only();
  const std::regex steer(R"(set_steer\s*\(\s*([^)]*)\))");
  size_t n = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), steer); it != std::sregex_iterator();
       ++it) {
    ++n;
    EXPECT_EQ((*it)[1].str(), "0.0") << "steer must be the literal 0.0: " << (*it)[0].str();
  }
  EXPECT_EQ(n, 1u) << "exactly one VehicleCtrlCmd builder (make_cmd)";
  // No other path to a steering value: VehicleSteer / SetVehicleSteer are not used at all.
  EXPECT_EQ(src.find("VehicleSteer"), std::string::npos);
}

TEST(ActuateScope, OnlyTheAccelerationCommandTypeIsSent)
{
  const std::string src = code_only();
  const std::regex type(R"(set_long_cmd_type\s*\(\s*([^)]*)\))");
  size_t n = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), type); it != std::sregex_iterator();
       ++it) {
    ++n;
    EXPECT_NE((*it)[1].str().find("LONG_CMD_TYPE_ACCELERATION"), std::string::npos)
      << (*it)[0].str();
  }
  EXPECT_EQ(n, 1u);
}

TEST(ActuateScope, EveryStateChangingCallHasADeadline)
{
  // Each Set*/ControlVehicle call site is preceded (within its function) by set_deadline.
  const std::string src = code_only();
  const std::regex site(R"(_stub_\s*->\s*(ControlVehicle|AsyncControlVehicle|SetVehicleControlMode|SetVehicleGear)\s*\()");
  size_t n = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), site); it != std::sregex_iterator();
       ++it) {
    ++n;
    const auto pos = static_cast<size_t>(it->position(0));
    const std::string before = src.substr(pos > 800 ? pos - 800 : 0, pos > 800 ? 800 : pos);
    EXPECT_NE(before.find("set_deadline"), std::string::npos) << (*it)[0].str();
  }
  EXPECT_EQ(n, 4u);
}

// =============================================================================
// 26.09.25 _ YSH _ 동기 모드(lockstep) 범위 — SetSynchronousMode·Tick 은 actuate_lockstep.cpp(구동)와
// morai_sync_restore.cpp(복구)에서만. 실시간 경로(morai_actuate.cpp)는 위 테스트대로 여전히 금지,
// lockstep 경로에는 WaitForTick(동기 모드에서 즉시 0)이 없다. morai_probe 읽기 전용 가드는 따로 그대로.
// =============================================================================

std::string code_of(const std::string & path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  s = std::regex_replace(s, std::regex(R"("(?:[^"\\\n]|\\.)*")"), "\"\"");
  s = std::regex_replace(s, std::regex(R"(//[^\n]*)"), "");
  s = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), "");
  return s;
}

std::set<std::string> stub_calls(const std::string & src)
{
  const std::regex call(R"((\w+_stub_)\s*->\s*(\w+)\s*\()");
  std::set<std::string> seen;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), call); it != std::sregex_iterator();
       ++it) {
    seen.insert((*it)[2].str());
  }
  return seen;
}

size_t count_matches(const std::string & src, const std::regex & re)
{
  return static_cast<size_t>(
    std::distance(std::sregex_iterator(src.begin(), src.end(), re), std::sregex_iterator()));
}

/// morai_actuate --lockstep: the minimum for synchronous-mode driving and its restore.
const std::set<std::string> kLockstepRpcs = {
  // state-changing
  "SetSynchronousMode", "Tick", "ControlVehicle", "SetVehicleGear", "SetVehicleControlMode",
  // read-only
  "GetSynchronousMode", "GetAllActorsState", "GetVehicleControlMode", "GetSimulatorVersion"};

/// morai_sync_restore: back to real time, optionally KEYBOARD — never drives the Ego.
const std::set<std::string> kRestoreRpcs = {
  "SetSynchronousMode", "SetVehicleControlMode", "GetSynchronousMode", "GetAllActorsState",
  "GetVehicleControlMode"};

TEST(LockstepScope, SourceIsReadable)
{
  EXPECT_GT(code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE).size(), 1000u) << MORAI_ACTUATE_LOCKSTEP_SOURCE;
  EXPECT_GT(code_of(MORAI_SYNC_RESTORE_SOURCE).size(), 1000u) << MORAI_SYNC_RESTORE_SOURCE;
}

TEST(LockstepScope, EveryStubCallIsAllowListed)
{
  const std::set<std::string> seen = stub_calls(code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE));
  for (const auto & m : seen) {
    EXPECT_TRUE(kLockstepRpcs.count(m)) << "non-allow-listed RPC in actuate_lockstep: " << m;
  }
  EXPECT_EQ(seen, kLockstepRpcs);
}

TEST(LockstepScope, NoWaitForTickNoAsyncSends)
{
  // 동기 모드의 WaitForTick 은 즉시 0 을 돌려준다 — lockstep 경로에서는 쓰지 않는다
  const std::string src = code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE);
  EXPECT_EQ(src.find("WaitForTick"), std::string::npos);
  EXPECT_EQ(src.find("AsyncControlVehicle"), std::string::npos);
}

TEST(LockstepScope, NoOtherStateChangingRpcNameAppearsAsACall)
{
  const std::string src = code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE);
  const std::regex forbidden(
    R"(\b(Set(?!MaxReceiveMessageSize|VehicleControlMode|VehicleGear|SynchronousMode|Int\b)[A-Z]\w*|)"
    R"(Control(Pedestrian|Obstacle)|Start|Stop|Pause|Resume|Spawn\w*|Destroy\w*|)"
    R"(SendMessage|Resim\w*|AddResim\w*|Load\w*|Create\w*SpawnPoint|Enable\w*|Raycast|)"
    R"(Add\w*Sensor|Remove\w*|SetSimulationMode|SetResimRecordMode)\s*\()");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, forbidden)) << "found: " << m.str();
}

TEST(LockstepScope, SteeringIsAlwaysZeroAndOnlyAcceleration)
{
  const std::string src = code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE);
  const std::regex steer(R"(set_steer\s*\(\s*([^)]*)\))");
  size_t n = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), steer); it != std::sregex_iterator();
       ++it) {
    ++n;
    EXPECT_EQ((*it)[1].str(), "0.0") << (*it)[0].str();
  }
  EXPECT_EQ(n, 1u) << "exactly one VehicleCtrlCmd builder (make_cmd)";
  EXPECT_EQ(src.find("VehicleSteer"), std::string::npos);
  const std::regex type(R"(set_long_cmd_type\s*\(\s*([^)]*)\))");
  size_t t = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), type); it != std::sregex_iterator();
       ++it) {
    ++t;
    EXPECT_NE((*it)[1].str().find("LONG_CMD_TYPE_ACCELERATION"), std::string::npos);
  }
  EXPECT_EQ(t, 1u);
}

TEST(LockstepScope, EveryStateChangingCallHasADeadline)
{
  for (const char * path : {MORAI_ACTUATE_LOCKSTEP_SOURCE, MORAI_SYNC_RESTORE_SOURCE}) {
    const std::string src = code_of(path);
    const std::regex site(
      R"(_stub_\s*->\s*(ControlVehicle|SetVehicleControlMode|SetVehicleGear|SetSynchronousMode|Tick)\s*\()");
    size_t n = 0;
    for (auto it = std::sregex_iterator(src.begin(), src.end(), site);
         it != std::sregex_iterator(); ++it) {
      ++n;
      const auto pos = static_cast<size_t>(it->position(0));
      const std::string before = src.substr(pos > 800 ? pos - 800 : 0, pos > 800 ? 800 : pos);
      EXPECT_NE(before.find("set_deadline"), std::string::npos) << path << ": " << (*it)[0].str();
    }
    EXPECT_EQ(n, std::string(path) == MORAI_SYNC_RESTORE_SOURCE ? 2u : 5u) << path;
  }
}

TEST(LockstepScope, ControlModeIsOnlySetByTheRestoreGuard)
{
  // 동기 모드에서는 서버가 제어 모드를 7 로 강제한다 — SetVehicleControlMode 는 복구(실시간 복귀 뒤)에서만
  const std::string src = code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE);
  const size_t guard = src.find("class RestoreGuard");
  ASSERT_NE(guard, std::string::npos);
  const std::regex call(R"(client_\s*\.\s*set_mode\s*\()");
  size_t n = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), call); it != std::sregex_iterator();
       ++it) {
    ++n;
    EXPECT_GT(static_cast<size_t>(it->position(0)), guard);
  }
  EXPECT_EQ(n, 2u);  // 원래 모드 복구 + 실시간 안전 정지용 AUTO (둘 다 동기 모드 해제 뒤)
  // 실시간 브레이크(AUTO 포함)는 exit_sync 뒤에서만 불리고, 원래 모드 복구는 쓰기 허용(mode_writes_ok) 검사 뒤에 있다
  const size_t exit_pos = src.find("exit_sync(ls::Limits::kExitRetryS)", guard);
  ASSERT_NE(exit_pos, std::string::npos);
  const std::regex brake_call(R"(=\s*realtime_brake\s*\(\s*\)\s*;)");
  size_t brakes = 0;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), brake_call);
       it != std::sregex_iterator(); ++it) {
    ++brakes;
    EXPECT_GT(static_cast<size_t>(it->position(0)), exit_pos);
  }
  EXPECT_EQ(brakes, 2u);  // 3b(해제 직후) + 4b(실시간 확인 뒤)
  const size_t gate = src.find("mode_writes_ok", exit_pos);
  ASSERT_NE(gate, std::string::npos);
  const size_t restore_mode = src.find("client_.set_mode(mode_)", guard);
  ASSERT_NE(restore_mode, std::string::npos);
  EXPECT_GT(restore_mode, gate);
}

TEST(SyncRestoreScope, EveryStubCallIsAllowListed)
{
  const std::set<std::string> seen = stub_calls(code_of(MORAI_SYNC_RESTORE_SOURCE));
  for (const auto & m : seen) {
    EXPECT_TRUE(kRestoreRpcs.count(m)) << "non-allow-listed RPC in morai_sync_restore: " << m;
  }
  EXPECT_EQ(seen, kRestoreRpcs);
}

TEST(SyncRestoreScope, NeverDrivesTheEgo)
{
  const std::string src = code_of(MORAI_SYNC_RESTORE_SOURCE);
  const std::regex forbidden(
    R"(\b(Set(?!MaxReceiveMessageSize|VehicleControlMode|SynchronousMode|Int\b)[A-Z]\w*|)"
    R"(Control(Vehicle|Pedestrian|Obstacle)|Tick|WaitForTick|Start|Stop|Pause|Resume|)"
    R"(Spawn\w*|Destroy\w*|SendMessage|Resim\w*|Load\w*|Enable\w*|Remove\w*)\s*\()");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, forbidden)) << "found: " << m.str();
  EXPECT_EQ(src.find("VehicleCtrlCmd"), std::string::npos);
  EXPECT_EQ(src.find("VehicleGear"), std::string::npos);
}

TEST(SyncRestoreScope, OnlyUnspecifiedAndKeyboardAreSet)
{
  const std::string src = code_of(MORAI_SYNC_RESTORE_SOURCE);
  const std::regex type(R"(set_type\s*\(\s*([^)]*)\))");
  EXPECT_EQ(count_matches(src, type), 1u);
  for (auto it = std::sregex_iterator(src.begin(), src.end(), type); it != std::sregex_iterator();
       ++it) {
    EXPECT_NE((*it)[1].str().find("SYNC_MODE_TYPE_UNSPECIFIED"), std::string::npos);
  }
  const std::regex mode(R"(set_mode\s*\(\s*([^)]*)\))");
  EXPECT_EQ(count_matches(src, mode), 1u);
  for (auto it = std::sregex_iterator(src.begin(), src.end(), mode); it != std::sregex_iterator();
       ++it) {
    EXPECT_NE((*it)[1].str().find("VEHICLE_CONTROL_KEYBOARD"), std::string::npos);
  }
}

TEST(SyncModeScope, OnlyLockstepAndRestoreCallSetSynchronousModeOrTick)
{
  const std::regex sync_call(R"(_stub_\s*->\s*(SetSynchronousMode|Tick)\s*\()");
  const std::set<std::string> allowed = {"actuate_lockstep.cpp", "morai_sync_restore.cpp"};
  std::set<std::string> callers;
  size_t scanned = 0;
  for (const auto & e : std::filesystem::directory_iterator(MORAI_TOOLS_DIR)) {
    if (e.path().extension() != ".cpp" && e.path().extension() != ".hpp") {
      continue;
    }
    ++scanned;
    const std::string src = code_of(e.path().string());
    if (std::regex_search(src, sync_call)) {
      callers.insert(e.path().filename().string());
    }
  }
  EXPECT_GE(scanned, 10u);
  for (const auto & c : callers) {
    EXPECT_TRUE(allowed.count(c)) << "SetSynchronousMode / Tick called from " << c;
  }
  EXPECT_EQ(callers, allowed);  // 두 파일 모두 실제로 부른다 (스캔이 헛돌지 않음)
}

TEST(SyncModeScope, NoOtherToolsFileReachesSetSynchronousModeOrTickAnyWay)
{
  // 위 스캔은 '_stub_ ->' 호출만 본다. 다른 파일이 AsyncTick / PrepareAsyncSetSynchronousMode,
  // stub->async()->Tick(...), 이름이 _stub_ 로 끝나지 않는 스텁 변수로 부르는 것도 막는다
  const std::regex any_call(R"(\b(Async|PrepareAsync)?(SetSynchronousMode|Tick)\s*\()");
  const std::regex async_call(R"(async\s*\(\s*\)\s*->\s*\w*(SetSynchronousMode|Tick)\b)");
  const std::set<std::string> allowed = {"actuate_lockstep.cpp", "morai_sync_restore.cpp"};
  size_t scanned = 0;
  for (const auto & e : std::filesystem::directory_iterator(MORAI_TOOLS_DIR)) {
    const std::string name = e.path().filename().string();
    if ((e.path().extension() != ".cpp" && e.path().extension() != ".hpp") || allowed.count(name)) {
      continue;
    }
    ++scanned;
    const std::string src = code_of(e.path().string());
    std::smatch m;
    EXPECT_FALSE(std::regex_search(src, m, any_call)) << name << ": " << m.str();
    EXPECT_FALSE(std::regex_search(src, m, async_call)) << name << ": " << m.str();
  }
  EXPECT_GE(scanned, 10u);
  // 패턴이 실제 호출 모양을 잡는지 (헛돌지 않음): 허용된 두 파일에서는 걸려야 한다
  EXPECT_TRUE(std::regex_search(code_of(MORAI_ACTUATE_LOCKSTEP_SOURCE), any_call));
  EXPECT_TRUE(std::regex_search(code_of(MORAI_SYNC_RESTORE_SOURCE), any_call));
  const std::string probe_like = "x.async()->Tick(&ctx, &req, &res, cb); y->AsyncTick(&c, r, q);";
  EXPECT_TRUE(std::regex_search(probe_like, async_call));
  EXPECT_TRUE(std::regex_search(std::string("y->AsyncTick(&c, r, q);"), any_call));
  EXPECT_FALSE(std::regex_search(std::string("s->WaitForTick(&c, r, &q);"), any_call));
}

TEST(SyncModeScope, RealTimePathOnlyDispatchesToLockstep)
{
  // morai_actuate.cpp 는 --lockstep 일 때 run_lockstep 으로 넘길 뿐 동기 모드 RPC 를 직접 부르지 않는다
  const std::string src = code_only();
  const std::regex dispatch(
    R"(if\s*\(\s*opt\.lockstep\s*\)\s*\{\s*return\s+act::run_lockstep\s*\(\s*opt\s*\)\s*;\s*\})");
  EXPECT_EQ(count_matches(src, dispatch), 1u);
  EXPECT_EQ(count_matches(src, std::regex(R"(run_lockstep\s*\()")), 1u);
  EXPECT_EQ(src.find("SetSynchronousMode"), std::string::npos);
  EXPECT_EQ(count_matches(src, std::regex(R"(\bTick\s*\()")), 0u);
}

}  // namespace
