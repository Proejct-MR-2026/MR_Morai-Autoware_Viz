# Copyright 2026 The Autoware Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.14)
project(morai_grpc_vendor)

# NOTE: autoware_package() is deliberately NOT called here.
# It injects -Wall -Wextra -Wpedantic -Werror via add_compile_options(), which is
# directory-scoped and would therefore apply to generated third-party protobuf code.
# Not calling it is the primary mechanism that keeps generated code out of -Werror;
# see README.md "Why not autoware_package()".
find_package(ament_cmake REQUIRED)
find_package(Protobuf 3.12 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# gRPC has no rosdep key, so it cannot be declared in package.xml without breaking
# `rosdep install` for the whole workspace. Fail here instead, with an actionable
# message at the exact moment it is needed. See README.md "Prerequisites".
pkg_check_modules(GRPCPP QUIET IMPORTED_TARGET grpc++)
find_program(GRPC_CPP_PLUGIN NAMES grpc_cpp_plugin)
if(NOT GRPCPP_FOUND OR NOT GRPC_CPP_PLUGIN)
  message(FATAL_ERROR
    "morai_grpc_vendor requires the gRPC C++ runtime and codegen plugin, which have no "
    "rosdep key and are therefore NOT installed by 'rosdep install'.\n"
    "Install them with:\n"
    "    sudo apt-get install -y libgrpc++-dev protobuf-compiler-grpc\n"
    "See src/universe/autoware_universe/simulator/morai_grpc_vendor/README.md.")
endif()

# ==============================================================================
# Code generation
# ==============================================================================
set(MORAI_PROTO_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/proto)
set(MORAI_GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)

# 26.09.24 _ YSH _ closure 대상에 sensor(기존)·simulation·simulator 서비스 반영 (22 -> 31개)
# # The transitive import closure of morai/actor/actor.proto. Verified with
# #   cd proto && protoc -I. --descriptor_set_out=/dev/null --include_imports \
# #     morai/actor/actor.proto
# The transitive import closure of the four service files below. Verified with
#   cd proto && protoc -I. --descriptor_set_out=/dev/null --include_imports \
#     morai/actor/actor.proto morai/sensor/sensor.proto \
#     morai/simulation/simulation.proto morai/simulator/simulator.proto
# If upstream adds an import, this list and PROVENANCE.md must both be extended.
set(MORAI_PROTOS
  morai/actor/actor.proto
  morai/actor/actor_control.proto
  morai/actor/actor_enum.proto
  morai/actor/actor_get.proto
  morai/actor/actor_network.proto
  morai/actor/actor_set.proto
  morai/actor/actor_spawn.proto
  morai/common/enum.proto
  morai/common/object_identifier.proto
  morai/common/object_info.proto
  morai/common/resource_identifier.proto
  morai/common/result.proto
  morai/common/type.proto
  morai/infrastructure/infrastructure_enum.proto
  morai/map/link_info.proto
  morai/sensor/add_sensor_param.proto
  morai/sensor/sensor.proto
  morai/sensor/sensor_data.proto
  morai/sensor/sensor_data_save_config.proto
  morai/sensor/sensor_enum.proto
  morai/sensor/sensor_info.proto
  morai/sensor/sensor_settings.proto
  # 26.09.24 _ YSH _ Simulation / Simulator 서비스 closure 9개 추가 (morai_probe 의 GetSynchronousMode·WaitForTick·GetTimestamp 용)
  morai/simulation/resim.proto
  morai/simulation/simulation.proto
  morai/simulation/simulation_enum.proto
  morai/simulation/start_param.proto
  morai/simulation/sync_mode.proto
  morai/simulator/available_object.proto
  morai/simulator/category_obstacles.proto
  morai/simulator/rendering.proto
  morai/simulator/simulator.proto
)

set(MORAI_PROTO_ABS "")
set(MORAI_PROTO_SRCS "")
set(MORAI_PROTO_HDRS "")
foreach(_proto IN LISTS MORAI_PROTOS)
  string(REGEX REPLACE "\\.proto$" "" _stem ${_proto})
  list(APPEND MORAI_PROTO_ABS ${MORAI_PROTO_ROOT}/${_proto})
  list(APPEND MORAI_PROTO_SRCS ${MORAI_GENERATED_DIR}/${_stem}.pb.cc)
  list(APPEND MORAI_PROTO_HDRS ${MORAI_GENERATED_DIR}/${_stem}.pb.h)
endforeach()

file(MAKE_DIRECTORY ${MORAI_GENERATED_DIR})

# Files declaring a service; only these are --grpc_out inputs.
set(MORAI_SERVICE_PROTOS
  morai/actor/actor.proto
  morai/sensor/sensor.proto
  # 26.09.24 _ YSH _ Simulation / Simulator 서비스 스텁 생성 (actor/sensor 와 같은 방식, 설치는 하지 않음)
  morai/simulation/simulation.proto
  morai/simulator/simulator.proto
)
set(MORAI_SERVICE_PROTO_ABS "")
set(MORAI_GRPC_SRCS "")
set(MORAI_GRPC_HDRS "")
foreach(_proto IN LISTS MORAI_SERVICE_PROTOS)
  string(REGEX REPLACE "\\.proto$" "" _stem ${_proto})
  list(APPEND MORAI_SERVICE_PROTO_ABS ${MORAI_PROTO_ROOT}/${_proto})
  list(APPEND MORAI_GRPC_SRCS ${MORAI_GENERATED_DIR}/${_stem}.grpc.pb.cc)
  list(APPEND MORAI_GRPC_HDRS ${MORAI_GENERATED_DIR}/${_stem}.grpc.pb.h)
endforeach()

add_custom_command(
  OUTPUT ${MORAI_PROTO_SRCS} ${MORAI_PROTO_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS -I ${MORAI_PROTO_ROOT} --cpp_out=${MORAI_GENERATED_DIR} ${MORAI_PROTO_ABS}
  DEPENDS ${MORAI_PROTO_ABS}
  COMMENT "Generating MORAI protobuf message sources"
  VERBATIM
)

add_custom_command(
  OUTPUT ${MORAI_GRPC_SRCS} ${MORAI_GRPC_HDRS}
  COMMAND ${Protobuf_PROTOC_EXECUTABLE}
  ARGS -I ${MORAI_PROTO_ROOT}
       --grpc_out=${MORAI_GENERATED_DIR}
       --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
       ${MORAI_SERVICE_PROTO_ABS}
  DEPENDS ${MORAI_PROTO_ABS} ${GRPC_CPP_PLUGIN}
  # 26.09.24 _ YSH _ 스텁이 Actor 하나가 아니므로 문구 정정
  # COMMENT "Generating MORAI gRPC Actor service stub"
  COMMENT "Generating MORAI gRPC service stubs (Actor, Sensor, Simulation, Simulator)"
  VERBATIM
)

# ==============================================================================
# Library
# ==============================================================================
add_library(${PROJECT_NAME} SHARED
  ${MORAI_PROTO_SRCS}
  ${MORAI_GRPC_SRCS}
  src/actor_client.cpp
  src/sensor_client.cpp
  # 26.09.25 _ YSH _ 엔진용 링크 계층 MoraiLink (상태·명령 두 채널, 공개 헤더는 grpc·protobuf 비의존)
  src/morai_link.cpp
  src/morai_link_convert.cpp
  src/morai_link_types.cpp
  # 26.09.26 _ YSH _ 원시 gRPC 기록 (ActorStates 바이트·VehicleCtrlCmd → proto3 JSON, 공개 헤더는 grpc·protobuf 비의존)
  src/morai_link_raw.cpp
)

# 26.09.25 _ YSH _ 새로 손으로 쓴 라이브러리 소스는 도구와 같은 경고 수준(-Werror)을 소스 단위로 건다.
# 생성 헤더는 이 타깃 안에서도 -isystem 이라 걸리지 않는다. 기존 actor_client·sensor_client 는 그대로 둔다.
set(MORAI_LINK_WARNINGS -Wall -Wextra -Wpedantic -Werror)
# 26.09.26 _ YSH _ morai_link_raw.cpp 추가. 원문:
# set_source_files_properties(
#   src/morai_link.cpp src/morai_link_convert.cpp src/morai_link_types.cpp
#   PROPERTIES COMPILE_OPTIONS "${MORAI_LINK_WARNINGS}"
# )
set_source_files_properties(
  src/morai_link.cpp src/morai_link_convert.cpp src/morai_link_types.cpp src/morai_link_raw.cpp
  PROPERTIES COMPILE_OPTIONS "${MORAI_LINK_WARNINGS}"
)

# Generated third-party code: silence warnings on those translation units only.
# Source-level COMPILE_OPTIONS are appended after directory- and target-level options,
# so -w wins even if someone builds with --cmake-args -DCMAKE_CXX_FLAGS="-Wall -Werror".
set_source_files_properties(
  ${MORAI_PROTO_SRCS} ${MORAI_GRPC_SRCS}
  PROPERTIES COMPILE_OPTIONS "-w"
)

# SYSTEM so that -Werror consumers (i.e. autoware_morai_interface) never warn on the
# generated headers they include.
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC
  "$<BUILD_INTERFACE:${MORAI_GENERATED_DIR}>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

# Hand-written headers. The INSTALL_INTERFACE path is already declared by the SYSTEM
# block above (both resolve to include/${PROJECT_NAME}); declaring it twice would make
# the SYSTEM / non-SYSTEM distinction above unverifiable at a glance, and dropping the
# SYSTEM form is what would put generated protobuf headers back under -Werror in every
# consumer.
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

# protobuf is PUBLIC because the installed *.pb.h are part of this package's API.
#
# gRPC is PRIVATE: *.grpc.pb.h is never installed and actor_client.hpp is a PIMPL, so no
# consumer needs grpc++ headers or a link-line entry. That is what stops the missing
# rosdep key from propagating into autoware_morai_interface.
target_link_libraries(${PROJECT_NAME}
  PUBLIC protobuf::libprotobuf
  PRIVATE PkgConfig::GRPCPP Threads::Threads
)

# 26.09.24 _ YSH _ 읽기 전용 측정 도구 morai_probe (Stage 1: 프레임 주기·timestamp 단위·RTT·WaitForTick 실측).
# 순수 계산부(morai_probe_stats)는 gRPC·protobuf 비의존 정적 라이브러리로 분리해 gtest 가 네트워크 없이 검증한다.
# 생성 스텁은 이 패키지 안에서만 PRIVATE 로 링크하고, 도구는 lib/${PROJECT_NAME} 에 설치해 'ros2 run' 으로 실행한다.
# 손으로 쓴 코드라 autoware 와 같은 경고 수준(-Werror)을 타깃 단위로 건다 — 디렉토리 스코프가 아니므로 생성 코드엔 안 걸린다.
# ==============================================================================
# Tools
# ==============================================================================
set(MORAI_PROBE_WARNINGS -Wall -Wextra -Wpedantic -Werror)

add_library(morai_probe_stats STATIC
  tools/probe_json.cpp
  # 26.09.24 _ YSH _ 옵션 파서를 gRPC 비의존 정적 라이브러리로 옮겨 입력 검증을 gtest 로 확인
  tools/probe_options.cpp
  tools/probe_stats.cpp
  # 26.09.25 _ YSH _ monitor 서브커맨드 순수 집계부 (진동·0 떨어짐·모드 변화)
  tools/monitor_stats.cpp
)
target_include_directories(morai_probe_stats PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_compile_features(morai_probe_stats PUBLIC cxx_std_17)
target_compile_options(morai_probe_stats PRIVATE ${MORAI_PROBE_WARNINGS})

add_executable(morai_probe tools/morai_probe.cpp)
target_compile_options(morai_probe PRIVATE ${MORAI_PROBE_WARNINGS})
# 26.09.25 _ YSH _ monitor 가 MoraiLink 와 같은 proto->구조체 변환(src/morai_link_convert.hpp, 미설치)을 쓴다
target_include_directories(morai_probe PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(morai_probe PRIVATE
  ${PROJECT_NAME}
  morai_probe_stats
  PkgConfig::GRPCPP
  Threads::Threads
)
# 26.09.24 _ YSH _ 다른 오버레이의 LD_LIBRARY_PATH 가 남아 있으면 옛 libmorai_grpc_vendor.so(Simulation/Simulator 스텁 없음)가
# 먼저 로드돼 'undefined symbol' 로 죽는다. DT_RUNPATH 는 LD_LIBRARY_PATH 뒤에 검색되므로 DT_RPATH 로 바꿔 앞서게 한다.
# --symlink-install 이면 설치본이 build 바이너리 링크라 빌드 RPATH 가, 일반 설치면 $ORIGIN/.. (= lib/) 가 쓰인다.
target_link_options(morai_probe PRIVATE -Wl,--disable-new-dtags)
set_target_properties(morai_probe PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# 26.09.25 _ YSH _ Ego 를 움직이는 측정 도구 morai_actuate (ControlVehicle 이 업데이트당/물리 스텝당 소비되는지 실측).
# 읽기 전용 morai_probe 와 일부러 소스·실행 파일을 분리한다 — test_probe_readonly 는 morai_probe.cpp 만 스캔하고,
# 이 도구의 RPC 범위는 test_actuate_scope 가 따로 검증한다. 순수 계산부는 gRPC 비의존 정적 라이브러리로 gtest.
# 스텁 PRIVATE 링크·-Werror 타깃 한정·DT_RPATH 처리는 morai_probe 와 같다.
# 26.09.25 _ YSH _ 동기 모드(lockstep) 진입·복구 판정 순수 헬퍼 — morai_actuate --lockstep 과 morai_sync_restore 가 공유 (gRPC 비의존)
add_library(morai_sync_mode STATIC
  tools/sync_mode.cpp
)
target_include_directories(morai_sync_mode PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_compile_features(morai_sync_mode PUBLIC cxx_std_17)
target_compile_options(morai_sync_mode PRIVATE ${MORAI_PROBE_WARNINGS})

# 26.09.25 _ YSH _ lockstep 분석(lockstep_analysis.cpp) 추가, 동기 모드 헬퍼 링크
# add_library(morai_actuate_analysis STATIC
#   tools/actuate_analysis.cpp
#   tools/actuate_options.cpp
# )
add_library(morai_actuate_analysis STATIC
  tools/actuate_analysis.cpp
  tools/actuate_options.cpp
  tools/lockstep_analysis.cpp
)
target_include_directories(morai_actuate_analysis PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/tools)
target_compile_features(morai_actuate_analysis PUBLIC cxx_std_17)
target_compile_options(morai_actuate_analysis PRIVATE ${MORAI_PROBE_WARNINGS})
# 26.09.25 _ YSH _ morai_sync_mode 추가
# target_link_libraries(morai_actuate_analysis PUBLIC morai_probe_stats)
target_link_libraries(morai_actuate_analysis PUBLIC morai_probe_stats morai_sync_mode)

# 26.09.25 _ YSH _ --lockstep 경로(actuate_lockstep.cpp)를 같은 실행 파일에 넣는다. SetSynchronousMode·Tick 은 그 파일에만 있다
# add_executable(morai_actuate tools/morai_actuate.cpp)
add_executable(morai_actuate tools/morai_actuate.cpp tools/actuate_lockstep.cpp)
target_compile_options(morai_actuate PRIVATE ${MORAI_PROBE_WARNINGS})
target_link_libraries(morai_actuate PRIVATE
  ${PROJECT_NAME}
  morai_actuate_analysis
  PkgConfig::GRPCPP
  Threads::Threads
)
target_link_options(morai_actuate PRIVATE -Wl,--disable-new-dtags)
set_target_properties(morai_actuate PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# 26.09.25 _ YSH _ 동기 모드 비상 복구 도구 morai_sync_restore (lockstep 클라이언트가 죽어 MORAI 가 얼어 있을 때).
# Ego 를 움직이는 RPC 는 없다 — 허용 RPC 는 test_actuate_scope 가 검증. 스텁 PRIVATE·-Werror·DT_RPATH 는 probe 와 같다
add_executable(morai_sync_restore tools/morai_sync_restore.cpp)
target_compile_options(morai_sync_restore PRIVATE ${MORAI_PROBE_WARNINGS})
target_link_libraries(morai_sync_restore PRIVATE
  ${PROJECT_NAME}
  morai_sync_mode
  morai_probe_stats
  PkgConfig::GRPCPP
  Threads::Threads
)
target_link_options(morai_sync_restore PRIVATE -Wl,--disable-new-dtags)
set_target_properties(morai_sync_restore PROPERTIES INSTALL_RPATH "$ORIGIN/..")

# ==============================================================================
# Install
# ==============================================================================
# Hand-written headers: include/morai_grpc_vendor/x.hpp is installed as
# include/morai_grpc_vendor/morai_grpc_vendor/x.hpp, matching the exported include dir.
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Generated MESSAGE headers only, preserving the morai/... subtree that the generated
# #include lines mirror. An explicit loop rather than install(DIRECTORY ...) so that
# ${MORAI_GRPC_HDRS} can never be installed by accident — keeping the gRPC stub out of
# the install tree is what makes the PIMPL boundary real.
foreach(_hdr IN LISTS MORAI_PROTO_HDRS)
  file(RELATIVE_PATH _rel ${MORAI_GENERATED_DIR} ${_hdr})
  get_filename_component(_dir ${_rel} DIRECTORY)
  install(FILES ${_hdr} DESTINATION include/${PROJECT_NAME}/${_dir})
endforeach()

# Ship the .proto sources and their provenance for offline regeneration and audit.
install(DIRECTORY proto DESTINATION share/${PROJECT_NAME})
install(FILES PROVENANCE.md DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 26.09.24 _ YSH _ 'ros2 run morai_grpc_vendor morai_probe' 가 찾는 위치. export 대상이 아니다.
install(TARGETS morai_probe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# 26.09.25 _ YSH _ 'ros2 run morai_grpc_vendor morai_actuate' 위치 (probe 와 같음, export 대상 아님)
install(TARGETS morai_actuate
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# 26.09.25 _ YSH _ 'ros2 run morai_grpc_vendor morai_sync_restore' 위치 (export 대상 아님)
install(TARGETS morai_sync_restore
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 26.09.24 _ YSH _ morai_probe 순수 헬퍼 gtest + 읽기 전용 가드(소스 스캔). 네트워크를 쓰지 않는다
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_morai_probe
    test/test_probe_stats.cpp
    test/test_probe_readonly.cpp
    # 26.09.24 _ YSH _ 옵션 파서 검증 (기본 측정 창 길이, 음수·nan·inf·범위 밖 값 거부)
    test/test_probe_options.cpp
    # 26.09.25 _ YSH _ monitor 집계 (throttle 전환·브레이크 개입·유휴 레코드·조향 0 떨어짐·간격·모드 변화)
    test/test_monitor_stats.cpp
  )
  target_link_libraries(test_morai_probe morai_probe_stats)
  target_compile_options(test_morai_probe PRIVATE ${MORAI_PROBE_WARNINGS})
  # 26.09.25 _ YSH _ 읽기 전용 가드는 morai_probe.cpp 하나만 본다 — Ego 를 움직이는 morai_actuate.cpp 를 여기 넣지 말 것
  target_compile_definitions(test_morai_probe PRIVATE
    MORAI_PROBE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/tools/morai_probe.cpp"
  )

  # 26.09.25 _ YSH _ morai_actuate: 순수 계산부(합성 데이터)·옵션 파서(확인 플래그·잘못된 값 거부)·RPC 범위 스캔. 네트워크 없음
  # 26.09.25 _ YSH _ lockstep 분석·동기 모드 헬퍼·복구 도구 옵션 gtest 추가, 범위 스캔 대상에 lockstep·restore 소스와 tools/ 디렉토리 추가
  # ament_add_gtest(test_morai_actuate
  #   test/test_actuate_analysis.cpp
  #   test/test_actuate_options.cpp
  #   test/test_actuate_scope.cpp
  # )
  ament_add_gtest(test_morai_actuate
    test/test_actuate_analysis.cpp
    test/test_actuate_options.cpp
    test/test_actuate_scope.cpp
    test/test_lockstep_analysis.cpp
    test/test_sync_mode.cpp
  )
  target_link_libraries(test_morai_actuate morai_actuate_analysis)
  target_compile_options(test_morai_actuate PRIVATE ${MORAI_PROBE_WARNINGS})
  # target_compile_definitions(test_morai_actuate PRIVATE
  #   MORAI_ACTUATE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/tools/morai_actuate.cpp"
  # )
  target_compile_definitions(test_morai_actuate PRIVATE
    MORAI_ACTUATE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/tools/morai_actuate.cpp"
    MORAI_ACTUATE_LOCKSTEP_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/tools/actuate_lockstep.cpp"
    MORAI_SYNC_RESTORE_SOURCE="${CMAKE_CURRENT_SOURCE_DIR}/tools/morai_sync_restore.cpp"
    MORAI_TOOLS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tools"
  )

  # 26.09.25 _ YSH _ MoraiLink: proto->구조체 변환·명령 채우기·Result 판정·되읽기 일치·시계 오프셋 등
  #   네트워크 없는 순수부 + 공개 헤더 격리(컴파일 시 매크로 검사 + 소스 스캔). 연결 없는 MoraiLink 는
  #   닫힌 포트로 만들어 실패 경로(예외 없음, 입력 거부)만 본다 — 실제 MORAI(7789)에는 붙지 않는다
  # 26.09.26 _ YSH _ 원시 기록 JSON 왕복(바이트 → JSON → 메시지 비교) 시험 추가, 격리 검사에 morai_link_raw.hpp 추가. 원문:
  # ament_add_gtest(test_morai_link
  #   test/test_morai_link_isolation.cpp
  #   test/test_morai_link.cpp
  # )
  ament_add_gtest(test_morai_link
    test/test_morai_link_isolation.cpp
    test/test_morai_link.cpp
    test/test_morai_link_raw.cpp
  )
  target_link_libraries(test_morai_link ${PROJECT_NAME})
  target_include_directories(test_morai_link PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
  target_compile_options(test_morai_link PRIVATE ${MORAI_PROBE_WARNINGS})
  # target_compile_definitions(test_morai_link PRIVATE
  #   MORAI_LINK_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/morai_grpc_vendor/morai_link.hpp"
  #   MORAI_LINK_TYPES_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/morai_grpc_vendor/morai_link_types.hpp"
  # )
  target_compile_definitions(test_morai_link PRIVATE
    MORAI_LINK_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/morai_grpc_vendor/morai_link.hpp"
    MORAI_LINK_TYPES_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/morai_grpc_vendor/morai_link_types.hpp"
    MORAI_LINK_RAW_HEADER="${CMAKE_CURRENT_SOURCE_DIR}/include/morai_grpc_vendor/morai_link_raw.hpp"
  )

  # 26.09.25 _ YSH _ 가짜 MORAI gRPC 서버 (노드 종단 시험용, autoware_morai_interface/test/e2e 가 띄운다).
  #   ctest 로 돌지 않고 설치하지 않는다 — build/morai_grpc_vendor/fake_morai_server 로만 쓴다. 7789 는 거부한다.
  #   서버 스텁(생성 코드)은 이 실행 파일에만 PRIVATE 로 들어간다
  add_executable(fake_morai_server test/fake/fake_morai_server.cpp)
  target_compile_options(fake_morai_server PRIVATE ${MORAI_PROBE_WARNINGS})
  target_link_libraries(fake_morai_server PRIVATE
    ${PROJECT_NAME}
    PkgConfig::GRPCPP
    Threads::Threads
  )
  target_link_options(fake_morai_server PRIVATE -Wl,--disable-new-dtags)
endif()

# ORDER IS LOAD-BEARING: ament_register_extension() is a plain list(APPEND), so the
# generated -extras files are included in call order. find_package(Protobuf) must run
# before export_morai_grpc_vendorExport.cmake, which references protobuf::libprotobuf.
ament_export_dependencies(Protobuf)
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
