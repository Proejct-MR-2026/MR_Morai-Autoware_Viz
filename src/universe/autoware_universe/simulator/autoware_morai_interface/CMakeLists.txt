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
project(autoware_morai_interface)

find_package(autoware_cmake REQUIRED)
# yaml-cpp is a plain CMake package, so ament_auto does not link it automatically.
find_package(yaml-cpp REQUIRED)
autoware_package()

# No explicit find_package(morai_grpc_vendor) is needed: autoware_package() calls
# ament_auto_find_build_dependencies(), which find_package()es every <depend> in
# package.xml. ament_auto_add_library() then calls
#   ament_target_dependencies(${target} SYSTEM ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
# which links morai_grpc_vendor::morai_grpc_vendor and adds its include directories as
# SYSTEM -- that is what keeps the generated protobuf headers out of -Werror.
# 26.09.25 _ YSH _ StatusReporter/CommandForwarder 를 프레임 고정 엔진으로 교체 (파일 삭제, 원본은 94cdb2b)
# ament_auto_add_library(${PROJECT_NAME} SHARED
#   src/conversion/autoware_to_morai.cpp
#   src/conversion/morai_to_autoware.cpp
#   src/command_forwarder.cpp
#   src/lamp_command_cache.cpp
#   src/sensor_kit_mounter.cpp
#   src/status_reporter.cpp
#   src/node.cpp
# )
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/conversion/autoware_to_morai.cpp
  src/conversion/morai_to_autoware.cpp
  src/command_slot.cpp
  # 26.09.26 _ YSH _ DAQ 샘플러 (ViLS 명세 100 Hz, cue_server 녹화 구간 게이트)
  src/daq_sampler.cpp
  src/duplicate_writer_guard.cpp
  src/engine.cpp
  src/follow_streamer.cpp
  src/gear_sender.cpp
  src/job_worker.cpp
  src/lamp_command_cache.cpp
  src/link_interface.cpp
  # 26.10.13 _ YSH _ VILS 차량 PC 고스트(ego_mode localize) twist
  src/localize_twist.cpp
  # 26.11.10 _ YSH _ 외란 주입 (MRDIST-1: 난수·트리거 읽기·명령 제어기·프레임 적용)
  src/object_disturbance.cpp
  src/sample_gate.cpp
  src/sensor_kit_mounter.cpp
  src/steering_report_hold.cpp
  src/node.cpp
)

target_link_libraries(${PROJECT_NAME} yaml-cpp)

# 26.09.25 _ YSH _ 단독 실행 파일은 MultiThreadedExecutor (과제 명세). 동시성은 없다: 모든 콜백이 노드 기본
#   MutuallyExclusive 그룹 하나라 한 번에 하나씩 돈다 — 막히는 일(RPC)은 엔진·기어·작업 스레드에 있어 콜백은
#   막히지 않는다. node.hpp 의 executor 전용 멤버는 이 직렬화에 기댄다 (Reentrant 그룹을 더하면 경쟁)
# rclcpp_components_register_node(${PROJECT_NAME}
#   PLUGIN "autoware::morai_interface::MoraiInterfaceNode"
#   EXECUTABLE ${PROJECT_NAME}_node
# )
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::morai_interface::MoraiInterfaceNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  # Pure functions with no node, executor or DDS involvement, so a plain gtest rather
  # than ament_add_ros_isolated_gtest.
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_autoware_to_morai.cpp
    # 26.09.25 _ YSH _ 엔진 루프(가짜 링크)·기어 송신기·표본 판정·조향 유지 필터·중복 writer 가드
    # 26.09.26 _ YSH _ DAQ 샘플러 (가짜 시계)
    test/test_daq_sampler.cpp
    test/test_duplicate_writer_guard.cpp
    test/test_engine.cpp
    # 26.10.10 _ YSH _ 의사 VILS follow 스트리머
    test/test_follow_streamer.cpp
    test/test_gear_sender.cpp
    test/test_lamp_command_cache.cpp
    # 26.10.13 _ YSH _ VILS 차량 PC 고스트 twist
    test/test_localize_twist.cpp
    test/test_morai_to_autoware.cpp
    # 26.11.10 _ YSH _ 외란 주입 (골든 값·문법·제어기·프레임 적용 — 엔진 수준은 test_engine.cpp)
    test/test_object_disturbance.cpp
    test/test_sample_gate.cpp
    test/test_sensor_kit_mounter.cpp
  )
endif()

# 26.09.25 _ YSH _ GT 모드 단독 실행용 launch/morai_interface_gt.launch.xml 추가. 토픽 이름을 main.py 와 같은
#   절대 이름으로 코드에 두었으므로 remap 없이 떠도 Autoware 와 이어진다 (위 우려가 사라졌다).
# # launch/ 는 두지 않는다. 진입점은 morai_vehicle_launch/launch/vehicle_interface.launch.xml
# # 하나뿐이다 — 여기에 컨테이너 정의를 복제해두면 remap 11개가 빠진 채로 기동해서,
# # 노드는 정상으로 보이는데 Autoware 쪽에서는 아무 토픽도 안 보이는 상태가 된다.
# ament_auto_package(
#   INSTALL_TO_SHARE
#   config
#   USE_SCOPED_HEADER_INSTALL_DIR
# )
ament_auto_package(
  INSTALL_TO_SHARE
  config
  launch
  USE_SCOPED_HEADER_INSTALL_DIR
)
