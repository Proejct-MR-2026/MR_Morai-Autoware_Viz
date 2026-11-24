// 신규 — 26.09.25 _ YSH _ MoraiLink 공개 헤더 격리 검사 (grpc·protobuf 가 새어 나오면 컴파일/테스트 실패)
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

// 공개 헤더를 이 번역 단위에서 **가장 먼저** include 한다. 그 뒤에 protobuf / grpc 의 가드 매크로가
// 정의돼 있으면 공개 헤더가 그것들을 끌고 들어온 것이다. protobuf 는 /usr/include 에 있어 include
// 경로를 빼는 방식으로는 못 잡으므로 매크로로 본다.
#include "morai_grpc_vendor/morai_link.hpp"
// 26.09.26 _ YSH _ 원시 기록 헤더도 같은 격리 대상
#include "morai_grpc_vendor/morai_link_raw.hpp"
#include "morai_grpc_vendor/morai_link_types.hpp"

#if defined(GOOGLE_PROTOBUF_VERSION) || defined(GRPCPP_GRPCPP_H) || defined(GRPC_GRPC_H) || \
  defined(GRPCPP_IMPL_CODEGEN_CONFIG_H)
#error "MoraiLink public headers must not include protobuf or gRPC"
#endif

#include <gtest/gtest.h>

#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#ifndef MORAI_LINK_HEADER
#error "MORAI_LINK_HEADER must point at include/morai_grpc_vendor/morai_link.hpp"
#endif
#ifndef MORAI_LINK_TYPES_HEADER
#error "MORAI_LINK_TYPES_HEADER must point at include/morai_grpc_vendor/morai_link_types.hpp"
#endif
// 26.09.26 _ YSH _
#ifndef MORAI_LINK_RAW_HEADER
#error "MORAI_LINK_RAW_HEADER must point at include/morai_grpc_vendor/morai_link_raw.hpp"
#endif

namespace
{

/// 주석을 뺀 소스 (주석 속 설명 문구가 검사에 걸리지 않게)
std::string read(const char * path)
{
  std::ifstream f(path);
  std::stringstream ss;
  ss << f.rdbuf();
  std::string s = ss.str();
  s = std::regex_replace(s, std::regex(R"(//[^\n]*)"), "");
  s = std::regex_replace(s, std::regex(R"(/\*[\s\S]*?\*/)"), "");
  return s;
}

// 26.09.26 _ YSH _ 잘못된 파일(빈 파일)을 읽지 않았는지 보는 하한을 파일마다 준다 — morai_link_raw.hpp 는 선언 둘뿐이라 짧다. 원문:
// void expect_isolated(const char * path)
// {
//   const std::string src = read(path);
//   ASSERT_GT(src.size(), 500u) << path;
void expect_isolated(const char * path, size_t min_size = 500u)
{
  const std::string src = read(path);
  ASSERT_GT(src.size(), min_size) << path;
  const std::regex bad(R"(#\s*include\s*[<"](grpc|grpcpp|google/protobuf|morai/)[^>"]*[>"])");
  std::smatch m;
  EXPECT_FALSE(std::regex_search(src, m, bad)) << path << " includes " << m.str();
  EXPECT_EQ(src.find(".pb.h"), std::string::npos) << path;
  // 공개 헤더는 자기 패키지 공개 헤더와 표준 라이브러리만 include 한다
  const std::regex any_include(R"(#\s*include\s*([<"][^>"]+[>"]))");
  for (auto it = std::sregex_iterator(src.begin(), src.end(), any_include);
       it != std::sregex_iterator(); ++it) {
    const std::string inc = (*it)[1].str();
    const bool own = inc.rfind("\"morai_grpc_vendor/", 0) == 0;
    const bool std_header = inc.front() == '<' && inc.find('/') == std::string::npos &&
                            inc.find('.') == std::string::npos;
    EXPECT_TRUE(own || std_header) << path << " includes " << inc;
  }
}

TEST(MoraiLinkIsolation, PublicHeadersIncludeNoGrpcOrProtobuf)
{
  expect_isolated(MORAI_LINK_HEADER);
  expect_isolated(MORAI_LINK_TYPES_HEADER);
  expect_isolated(MORAI_LINK_RAW_HEADER, 300u);  // 26.09.26 _ YSH _
}

TEST(MoraiLinkIsolation, PublicTypesAreUsableWithoutGrpc)
{
  // 이 번역 단위는 grpc·protobuf 없이 공개 타입을 전부 쓸 수 있어야 한다
  morai_grpc_vendor::MoraiLinkOptions o;
  EXPECT_EQ(o.ego_id, "Ego");
  EXPECT_TRUE(o.client_key.empty());         // main.py 와 같이 비움
  EXPECT_TRUE(o.filter_client_key.empty());  // 채우면 actor id 체계가 바뀔 수 있다
  EXPECT_TRUE(o.include_vehicles && o.include_pedestrians && o.include_obstacles);
  EXPECT_EQ(o.deadlines.command.count(), 50);
  EXPECT_EQ(o.deadlines.transform.count(), 1000);
  morai_grpc_vendor::Snapshot s;
  EXPECT_FALSE(s.ego.found);
  // 26.09.26 _ YSH _ 원시 기록은 기본 꺼짐 (켜지 않으면 바이트를 만들지 않는다)
  EXPECT_FALSE(o.capture_raw);
  EXPECT_EQ(s.raw_actor_states, nullptr);
  morai_grpc_vendor::Reply<int64_t> r;
  EXPECT_FALSE(r.ok());
  EXPECT_EQ(r.status.grpc_code, -1);
}

}  // namespace
