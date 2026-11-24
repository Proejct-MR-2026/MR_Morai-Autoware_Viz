// 신규 — 26.09.24 _ YSH _ morai_probe 읽기 전용 강제 가드 (소스 스캔)
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

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <set>
#include <sstream>
#include <string>

#ifndef MORAI_PROBE_SOURCE
#error "MORAI_PROBE_SOURCE must point at tools/morai_probe.cpp"
#endif

// 26.09.25 _ YSH _ 도구 분리 설명 — 이 가드는 읽기 전용 morai_probe.cpp 하나만 스캔한다.
//   Ego 를 움직이는 morai_actuate 는 일부러 별도 소스(tools/morai_actuate.cpp)·별도 실행 파일로 두었고,
//   그쪽 RPC 범위(모드·기어·ControlVehicle 조향 0 만)는 test/test_actuate_scope.cpp 가 따로 검증한다.
//   probe 에 상태 변경 호출을 섞지 말고, 새 구동 기능은 actuate 쪽에 넣는다.

namespace
{

/// The complete set of RPCs morai_probe may issue. Adding anything here needs a reason
/// why it cannot change simulator state.
const std::set<std::string> kAllowedRpcs = {
  "GetActorState",      "GetAllActorsState", "GetVehicleSpec", "GetVehicleControlMode",
  "GetSynchronousMode", "WaitForTick",       "GetTimestamp",   "GetSimulatorVersion"};

/// Source with string literals and comments removed, so prose cannot trip the scan.
std::string code_only()
{
  std::ifstream f(MORAI_PROBE_SOURCE);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  s = std::regex_replace(s, std::regex(R"("(?:[^"\\\n]|\\.)*")"), "\"\"");
  s = std::regex_replace(s, std::regex(R"(//[^\n]*)"), "");
  s = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), "");
  return s;
}

TEST(ProbeReadOnly, SourceIsReadable)
{
  EXPECT_GT(code_only().size(), 1000u) << MORAI_PROBE_SOURCE;
}

// 26.09.25 _ YSH _ 스캔 대상이 읽기 전용 probe 그 자체인지 고정 (구동 도구 morai_actuate.cpp 로 바뀌면 실패)
TEST(ProbeReadOnly, ScansOnlyTheReadOnlyProbe)
{
  const std::string path = MORAI_PROBE_SOURCE;
  const std::string tail = "/tools/morai_probe.cpp";
  ASSERT_GE(path.size(), tail.size());
  EXPECT_EQ(path.substr(path.size() - tail.size()), tail);
}

TEST(ProbeReadOnly, EveryStubCallIsAllowListed)
{
  const std::string src = code_only();
  const std::regex call(R"((\w+_stub_)\s*->\s*(\w+)\s*\()");
  std::set<std::string> seen;
  for (auto it = std::sregex_iterator(src.begin(), src.end(), call); it != std::sregex_iterator();
       ++it) {
    const std::string method = (*it)[2].str();
    seen.insert(method);
    EXPECT_TRUE(kAllowedRpcs.count(method)) << "non-allow-listed RPC in morai_probe: " << method;
  }
  // Proves the regex actually matches the probe's call style.
  EXPECT_EQ(seen, kAllowedRpcs);
}

TEST(ProbeReadOnly, NoStateChangingRpcNameAppearsAsACall)
{
  // Belt and braces for calls that do not go through `*_stub_->`.
  const std::string src = code_only();
  const std::regex forbidden(
    R"(\b(Set(?!MaxReceiveMessageSize)[A-Z]\w*|Control(Vehicle|Pedestrian|Obstacle)|Tick|)"
    R"(Start|Stop|Pause|Resume|Spawn\w*|Destroy\w*|SendMessage|Resim\w*|AddResim\w*|)"
    R"(Load\w*|Create\w*SpawnPoint|Enable\w*|Raycast|Add\w*Sensor|Remove\w*)\s*\()");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, forbidden)) << "found: " << m.str();
}

// 26.09.25 _ YSH _ monitor 가 MoraiLink 의 proto->구조체 변환(morai_link_convert)을 가져다 쓰게 되면서 추가.
//   MoraiLink 에는 쓰기 메서드(control_vehicle, set_gear, ...)가 있고 스텁 호출이 `*_stub_->` 형태로
//   이 파일에 드러나지 않으므로 위 두 검사로는 못 막는다 — 객체도, 쓰기 메서드 이름의 호출도 금지한다.
TEST(ProbeReadOnly, NeverUsesTheWritableLink)
{
  const std::string src = code_only();
  EXPECT_EQ(src.find("MoraiLink"), std::string::npos) << "morai_probe must not construct MoraiLink";
  EXPECT_EQ(src.find("ActorClient"), std::string::npos)
    << "morai_probe must not construct ActorClient";
  const std::regex writer(
    R"(\b(control_vehicle|set_gear|set_control_mode|set_transform|set_tail_light|)"
    R"(set_vehicle_\w+)\s*\()");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, writer)) << "found: " << m.str();
}

}  // namespace
