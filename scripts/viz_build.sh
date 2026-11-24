#!/bin/bash
# 신규 — 26.11.13 _ YSH _ 13_ 시각화 전용 Autoware(MR_Morai-Autoware_Viz) 빌드. 11_ dist_build.sh 와 같은 깨끗한 환경.
#
#   평소 셸은 .bashrc 가 10_ install 을 source 한다. 그 셸에서 colcon 을 부르면 13_ 이 10_ 위에 얹혀 빌드된다.
#   그래서 env -i 로 자신을 다시 띄운다 (11_ 클린 빌드와 같은 변수).
#
# 쓰는 법 (어느 셸에서 불러도 된다)
#   ./viz_build.sh viz             시각화에 쓰는 것만 (RViz 플러그인·브리지·차량 모델·지도 로더, 83개) ← 보통 이것
#   ./viz_build.sh pkg [이름...]   고른 패키지만 (기본: morai_ros2_msgs autoware_morai_interface)
#   ./viz_build.sh all             전체 (약 1시간 45분, 16코어) — 로그 <작업공간>/log/clean_build_<날짜>.log
#   THROTTLE=1 ./viz_build.sh viz  다른 리그가 떠 있을 때 (코어 8~15, 병렬 4 — 훨씬 느리다)
#
# 지키는 것
#   - Autoware/브리지가 떠 있으면 멈춘다 (빌드가 CPU 를 다 쓴다). 그래도 하려면 BUILD_WHILE_RIG=1.
#   - 같은 작업공간에서 colcon 을 둘 띄우지 않는다 (잠금 파일).
#   - 빌드한 뒤에는 13_ 폴더 이름을 바꾸거나 옮기지 않는다 — --symlink-install 의 심링크가 절대 경로다.

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPTS="$(dirname "$SELF")"
ROOT="$(dirname "$SCRIPTS")"
# 26.11.24 _ YSH _ 공유본(MR_Morai-Autoware_Viz 저장소)은 src/ 가 저장소 바로 아래다. 이 PC 는 13_ 안의 MR_Morai-Autoware_Viz. 원문:
# AW="$ROOT/MR_Morai-Autoware_Viz"
if [ -d "$ROOT/src" ]; then AW="$ROOT"; else AW="$ROOT/MR_Morai-Autoware_Viz"; fi

if [ -z "${VIZ_BUILD_INNER:-}" ]; then
  # 26.11.24 _ YSH _ 다른 컴퓨터에서도 돌게 HOME·USER 를 지금 값으로. 원문:
  # exec env -i HOME=/home/autoware-morai USER=autoware LANG=C.UTF-8 TERM=dumb \
  exec env -i HOME="$HOME" USER="${USER:-$(id -un)}" LANG=C.UTF-8 TERM=dumb \
    VIZ_BUILD_INNER=1 THROTTLE="${THROTTLE:-}" BUILD_WHILE_RIG="${BUILD_WHILE_RIG:-}" \
    bash --noprofile --norc "$SELF" "$@"
fi

# 26.11.24 _ YSH _ HOME 은 바깥에서 받은 값 그대로, ccache 는 있을 때만 (다른 컴퓨터). 원문:
# export HOME=/home/autoware-morai
# export CCACHE_DIR="$HOME/.cache/ccache/" CC=/usr/lib/ccache/gcc CXX=/usr/lib/ccache/g++
if [ -x /usr/lib/ccache/gcc ]; then
  export CCACHE_DIR="$HOME/.cache/ccache/" CC=/usr/lib/ccache/gcc CXX=/usr/lib/ccache/g++
fi
export ACADOS_SOURCE_DIR=/opt/acados CMAKE_PREFIX_PATH=/opt/acados
export LD_LIBRARY_PATH=/opt/acados/lib:/usr/local/cuda/lib64
export PATH=/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set +u
source /opt/ros/humble/setup.bash

MODE="${1:-}"
[ $# -gt 0 ] && shift
# 26.11.13 _ YSH _ viz 모드 (사용자: 전체 빌드가 너무 오래 걸린다 → RViz 용만). 원문:
#   pkg|all) ;;
#   *) sed -n '2,15p' "$SELF" | sed 's/^# \{0,1\}//'; exit 2 ;;
case "$MODE" in
  viz|pkg|all) ;;
  *) sed -n '2,16p' "$SELF" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac

# 26.11.13 _ YSH _ 시각화에 쓰는 패키지 (의존성째로 빌드). autoware_launch 는 빌드하지 않는다 (launch 를 경로로 띄운다)
#   RViz: autoware.rviz 의 플러그인 전부 (이름에 rviz 가 든 22개 + autoware_polar_grid — 없으면 그 표시가 오류로 뜬다)
VIZ_RVIZ=(autoware_bag_time_manager_rviz_plugin autoware_localization_rviz_plugin autoware_mission_details_overlay_rviz_plugin
  autoware_overlay_rviz_plugin autoware_perception_rviz_plugin autoware_planning_rviz_plugin autoware_string_stamped_rviz_plugin
  autoware_topic_text_overlay_rviz_plugin autoware_traffic_light_rviz_plugin rviz_2d_overlay_msgs rviz_2d_overlay_plugins
  tier4_adapi_rviz_plugin tier4_camera_view_rviz_plugin tier4_control_mode_rviz_plugin tier4_datetime_rviz_plugin
  tier4_dummy_object_rviz_plugin tier4_manual_lane_change_rviz_plugin tier4_perception_rviz_plugin tier4_planning_factor_rviz_plugin
  tier4_state_rviz_plugin tier4_traffic_light_rviz_plugin tier4_vehicle_rviz_plugin
  autoware_polar_grid)
# 26.11.13 _ YSH _ autoware_polar_grid 더함 — PolarGridDisplay 는 이름에 rviz 가 없는 이 패키지에 있다 (첫 리허설에서 원이 안 그려짐).
#   RViz 설정의 클래스를 플러그인 설명 파일과 맞춰 본 결과 빠진 것은 이것뿐. 원문 끝줄: tier4_state_rviz_plugin tier4_traffic_light_rviz_plugin tier4_vehicle_rviz_plugin)
#   브리지·메시지·가짜 MORAI · 전역 파라미터·차량 모델 · 지도(다시 보기 때 얹기)
VIZ_BASE=(autoware_morai_interface morai_ros2_msgs morai_grpc_vendor autoware_internal_debug_msgs autoware_perception_msgs
  autoware_global_parameter_loader tier4_vehicle_launch morai_vehicle_description morai_sensor_kit_description morai_vehicle_launch
  tier4_map_launch autoware_map_loader autoware_lanelet2_map_visualizer autoware_map_tf_generator autoware_map_projection_loader)

[ -d "$AW/src" ] || { echo "viz_build: 작업공간이 없다: $AW"; exit 1; }
cd "$AW" || exit 1
mkdir -p log

rig_procs() {
  local d exe name
  for d in /proc/[0-9]*; do
    exe=$(readlink "$d/exe" 2>/dev/null) || continue
    name=${exe##*/}
    case "$name" in
      component_container|component_container_mt|component_container_isolated|autoware_morai_interface_node)
        echo "${d#/proc/} $exe" ;;
    esac
  done
}
if [ -z "$BUILD_WHILE_RIG" ]; then
  RIG_PROCS=$(rig_procs)
  if [ -n "$RIG_PROCS" ]; then
    echo "viz_build: Autoware/브리지 프로세스가 떠 있다 — 내리고 다시 부를 것 (그래도 하려면 BUILD_WHILE_RIG=1)"
    echo "$RIG_PROCS" | head -5 | cut -c1-160 | sed 's/^/    /'
    exit 1
  fi
fi

exec 9>"$AW/log/.viz_build.lock"
if ! flock -n 9; then
  echo "viz_build: 이 작업공간에서 다른 빌드가 돌고 있다 ($AW/log/.viz_build.lock)"
  exit 1
fi

COMMON=(--symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release)
PRE=()
EXTRA=()
if [ -n "$THROTTLE" ]; then
  PRE=(nice -n 19 ionice -c 3 taskset -c 8-15)
  EXTRA=(--parallel-workers 4)
  export MAKEFLAGS="-j8 -l8"
fi

run_colcon() {
  "${PRE[@]}" colcon build "$@" "${EXTRA[@]}" "${COMMON[@]}"
}

case "$MODE" in
  viz)
    LOG="$AW/log/viz_build_$(date +%y%m%d_%H%M).log"
    {
      echo "=== 13_ 시각화용 빌드 시작: $(date '+%F %T') ==="
      echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
      run_colcon --continue-on-error --packages-up-to "${VIZ_RVIZ[@]}" "${VIZ_BASE[@]}"
      rc1=$?
      # 26.11.13 _ YSH _ autoware_launch 는 빼었다 — --packages-select 도 실행 의존 240개가 설치돼 있기를 요구한다.
      #   viz_disturbance.launch.xml 을 소스 경로로 띄우고 같은 폴더의 rviz·config 를 바로 읽는다. 원문:
      #   run_colcon --packages-select autoware_launch
      #   rc2=$?
      #   echo "=== 끝: $(date '+%F %T') up-to rc=$rc1 · autoware_launch rc=$rc2 ==="
      #   [ $rc1 -eq 0 ] && [ $rc2 -eq 0 ]
      echo "=== 끝: $(date '+%F %T') rc=$rc1 ==="
      [ $rc1 -eq 0 ]
    } >>"$LOG" 2>&1
    rc=$?
    echo "viz_build viz: rc=$rc — 로그 $LOG"
    ;;
  pkg)
    if [ $# -eq 0 ]; then set -- morai_ros2_msgs autoware_morai_interface; fi
    run_colcon --packages-select "$@"
    rc=$?
    ;;
  all)
    LOG="$AW/log/clean_build_$(date +%y%m%d).log"
    {
      echo "=== 13_ 전체 빌드 시작: $(date '+%F %T') ==="
      echo "작업공간=$AW"
      echo "CC=$CC / CXX=$CXX / ACADOS_SOURCE_DIR=$ACADOS_SOURCE_DIR / CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH"
      echo "AMENT_PREFIX_PATH=$AMENT_PREFIX_PATH"
      [ -n "$THROTTLE" ] && echo "THROTTLE=1 (코어 8-15, 병렬 4)"
      ccache -s 2>/dev/null | grep -i -m1 "cache size"
      run_colcon --continue-on-error
      rc=$?
      echo "=== 끝: $(date '+%F %T') rc=$rc ==="
      exit $rc
    } >>"$LOG" 2>&1
    rc=$?
    echo "viz_build all: rc=$rc — 로그 $LOG"
    ;;
esac
exit $rc
