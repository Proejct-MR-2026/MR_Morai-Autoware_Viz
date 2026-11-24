#!/bin/bash
# 신규 — 26.11.13 _ YSH _ 외란 시각화 리그 켜기·끄기 (연차보고서 1차). Autoware 주행 스택·지도 없이 브리지 + 수신기 + RViz.
# 26.11.13 _ YSH _ 화면을 e2e_simulator 의 Autoware RViz 그대로로 바꿨다: 13_ 자체 Autoware(MR_Morai-Autoware_Viz)의
#   autoware_launch viz_disturbance.launch.xml (차량 모델·[지도]·rviz2 autoware_disturbance.rviz) + viz_autoware.py
#   (Prediction·GT·상태 한 줄). 옛 자체 디자인(viz_marker.py·rviz/disturbance_viz.rviz)은 legacy/ 로 옮겼다.
#
# 쓰는 법 (도커 안, 아무 셸에서나)
#   ./run_viz.sh start          브리지(MORAI 읽기만) → 표시 노드 → cue 수신기(50052) → Autoware RViz
#   ./run_viz.sh status         떠 있는 것·포트·MORAI 도달
#   ./run_viz.sh stop           수신기 먼저(외란 clear·녹화 닫기) → 나머지
#   ./run_viz.sh replay <bag>   녹화한 bag 을 RViz 로 다시 보기 (MORAI·브리지 없이, 보고서 캡처용)
#   MAP_PATH=<지도 폴더> ./run_viz.sh replay <bag>   지도(lanelet2)를 얹어 다시 보기 (start 에도 쓸 수 있다)
#
# 환경변수 (선택)
#   MORAI_SIM_ADDRESS (10.10.10.1)  MORAI_SIM_PORT (7789)  MORAI_CUE_ADDRESS (<MORAI>:50052, READY 받는 곳)
#   VIZ_CUE_PORT (50052)  VIZ_READY (auto|key|off)  VIZ_RECORD (scenario|session|off)  VIZ_DOMAIN (13)
#   VIZ_NO_RVIZ=1 (RViz 없이)  VIZ_DATA (~/01_DAQ_Data/Viz_Disturbance)  MAP_PATH (지도 폴더, 기본 없음)
#
# 주의: 10_/11_ 리그와 동시에 띄우지 않는다 (같은 MORAI·50052). 우리 프로세스는 환경변수 VIZ13_RIG=1 로 구분한다.

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPTS="$(dirname "$SELF")"
ROOT="$(dirname "$SCRIPTS")"
VIZ_SHELL="$SCRIPTS/viz_shell.sh"
# 26.11.24 _ YSH _ 다른 컴퓨터(공유본)에서는 저장소 안 data/ 에 쌓는다. 이 PC 는 그대로. 원문:
# DATA="${VIZ_DATA:-$HOME/01_DAQ_Data/Viz_Disturbance}"
if [ -d "$HOME/01_DAQ_Data" ]; then DATA="${VIZ_DATA:-$HOME/01_DAQ_Data/Viz_Disturbance}"; else DATA="${VIZ_DATA:-$ROOT/data}"; fi
LOGDIR="$DATA/logs"
TS="$(date +%Y-%m-%d-%H-%M-%S)"
MORAI_ADDR="${MORAI_SIM_ADDRESS:-10.10.10.1}"
MORAI_PORT="${MORAI_SIM_PORT:-7789}"
MORAI_CUE="${MORAI_CUE_ADDRESS:-$MORAI_ADDR:50052}"
CUE_PORT="${VIZ_CUE_PORT:-50052}"
READY="${VIZ_READY:-auto}"
RECORD="${VIZ_RECORD:-scenario}"
BRIDGE_NODE=morai_viz_bridge
MARK=VIZ13_RIG
# 26.11.24 _ YSH _ 공유본은 src/ 가 저장소 바로 아래다. 원문:
# AW="$ROOT/MR_Morai-Autoware_Viz"             # 26.11.13 _ YSH _ 13_ 자체 Autoware
if [ -d "$ROOT/src" ]; then AW="$ROOT"; else AW="$ROOT/MR_Morai-Autoware_Viz"; fi
# 26.11.13 _ YSH _ autoware_launch 는 빌드하지 않으므로(viz 모드) launch 를 소스 경로로 띄운다
VIZ_LAUNCH="$AW/src/launcher/autoware_launch/autoware_launch/launch/viz_disturbance.launch.xml"
MAP_PATH="${MAP_PATH:-}"
export VIZ_DOMAIN="${VIZ_DOMAIN:-13}"

ours() {      # 우리 프로세스 pid (environ 에 VIZ13_RIG=1)
  local p
  for p in /proc/[0-9]*; do
    [ "${p#/proc/}" = "$$" ] && continue
    { tr '\0' '\n' < "$p/environ"; } 2>/dev/null | grep -qx "$MARK=1" && echo "${p#/proc/}"
  done
}

cmd_of() { { tr '\0' ' ' < "/proc/$1/cmdline"; } 2>/dev/null; }   # 자르지 않는다 (브리지 실행 파일 경로가 길다)

foreign() {   # 같이 띄우면 안 되는 남의 리그 프로세스
  local p c mine
  mine=" $(ours | tr '\n' ' ') "
  for p in /proc/[0-9]*; do
    case "$mine" in *" ${p#/proc/} "*) continue ;; esac
    c="$({ tr '\0' ' ' < "$p/cmdline"; } 2>/dev/null)"
    case "$c" in
      *autoware_morai_interface_node*|*cue_server.py*|*e2e_simulator.launch.xml*|*bag_cue_server.py*)
        echo "${p#/proc/} ${c:0:140}" ;;
    esac
  done
}

port_busy() { ss -ltnH 2>/dev/null | awk '{print $4}' | grep -qE "[:.]$1\$"; }

morai_ok() { timeout 3 bash -c "cat </dev/null >/dev/tcp/$MORAI_ADDR/$MORAI_PORT" 2>/dev/null; }

# 26.11.13 _ YSH _ viz_disturbance.launch.xml 인자 (use_sim_time, MAP_PATH, VIZ_NO_RVIZ)
launch_args() {
  local a="use_sim_time:=$1"
  [ -n "$MAP_PATH" ] && a="$a map_path:=$MAP_PATH"
  [ "${VIZ_NO_RVIZ:-0}" = "1" ] && a="$a rviz:=false"
  echo "$a"
}

# 26.11.13 _ YSH _ RViz 설정(autoware.rviz + 디버그 그룹)이 원본과 맞는지. 낡았으면 다시 만든다
rviz_config() {
  if ! python3 -B "$SCRIPTS/make_rviz.py" --check >/dev/null 2>&1; then
    python3 -B "$SCRIPTS/make_rviz.py" | sed 's/^/      /'
  fi
}

spawn() {     # 이름, 명령 (viz_shell 안에서, 표식 달고, 로그 파일로)
  local name="$1"; shift
  setsid -f nohup "$VIZ_SHELL" -c "export $MARK=1; exec $*" > "$LOGDIR/${name}_$TS.log" 2>&1 < /dev/null
  echo "    $name → $LOGDIR/${name}_$TS.log"
}

status() {
  local pids p
  pids="$(ours)"
  echo "== 외란 시각화 (도메인 $VIZ_DOMAIN)"
  if [ -z "$pids" ]; then
    echo "  떠 있는 것 없음"
  else
    for p in $pids; do
      # 26.11.13 _ YSH _ Autoware RViz launch·표시 노드 이름을 바꿨다. 원문:
      #   *ros2\ launch*) n="브리지 launch" ;; *autoware_morai_interface_node*) n="브리지 노드" ;;
      #   *viz_cue_server.py*) n="cue 수신기" ;; *viz_marker.py*) n="표시 노드" ;; *rviz2*) n="RViz" ;;
      case "$(cmd_of "$p")" in
        *viz_disturbance.launch.xml*) n="RViz launch" ;; *ros2\ launch*) n="브리지 launch" ;;
        *autoware_morai_interface_node*) n="브리지 노드" ;;
        *viz_cue_server.py*) n="cue 수신기" ;; *viz_autoware.py*) n="표시 노드" ;; *rviz2*) n="RViz" ;;
        *robot_state_publisher*) n="차량 모델" ;; *component_container*) n="지도" ;;
        *bag\ record*) n="bag 녹화" ;; *bag\ play*) n="bag 재생" ;; *) n="" ;;
      esac
      [ -n "$n" ] && printf "  %-14s pid=%s\n" "$n" "$p"
    done
  fi
  if port_busy "$CUE_PORT"; then echo "  :$CUE_PORT 열림"; else echo "  :$CUE_PORT 닫힘"; fi
  if morai_ok; then echo "  MORAI $MORAI_ADDR:$MORAI_PORT 도달 OK"; else echo "  MORAI $MORAI_ADDR:$MORAI_PORT 닫힘"; fi
  local f
  f="$(ls -t "$LOGDIR"/cue_server_*.log 2>/dev/null | head -1)"
  [ -n "$f" ] && { echo "  -- 수신기 로그 마지막 6줄 ($f)"; tail -n 6 "$f" | sed 's/^/    /'; }
}

guard_start() {
  local f
  # 26.11.13 _ YSH _ 13_ 자체 Autoware 가 빌드돼 있어야 한다
  # 26.11.13 _ YSH _ 시각화용만 빌드(viz 모드)하므로 그 결과물(launch·브리지)로 본다. 원문:
  #   if [ ! -f "$AW/install/setup.bash" ]; then
  #     echo "  ★ 13_ Autoware 가 아직 빌드되지 않았다 — $SCRIPTS/viz_build.sh all"; exit 1
  if [ ! -f "$AW/install/setup.bash" ] || [ ! -f "$VIZ_LAUNCH" ] \
     || [ ! -d "$AW/install/autoware_morai_interface" ] || [ ! -d "$AW/install/autoware_overlay_rviz_plugin" ]; then
    echo "  ★ 13_ Autoware 시각화용 빌드가 안 돼 있다 — $SCRIPTS/viz_build.sh viz"; exit 1
  fi
  if [ -n "$MAP_PATH" ] && [ ! -f "$MAP_PATH/lanelet2_map.osm" ]; then
    echo "  ★ MAP_PATH=$MAP_PATH 에 lanelet2_map.osm 이 없다"; exit 1
  fi
  if [ -n "$(ours)" ]; then echo "  ★ 이미 떠 있다 — ./run_viz.sh stop 먼저"; status; exit 1; fi
  f="$(foreign)"
  if [ -n "$f" ]; then
    echo "  ★ 다른 리그(10_/11_ 브리지·cue_server·Autoware 또는 KATECH bag_cue_server)가 떠 있다 — 같이 띄우지 않는다:"
    echo "$f" | sed 's/^/      /'
    exit 1
  fi
}

start() {
  mkdir -p "$LOGDIR"
  echo "== 외란 시각화 켜기 ($TS)"
  guard_start
  if port_busy "$CUE_PORT"; then
    echo "  ★ :$CUE_PORT 를 다른 프로그램이 쓰고 있다 (pid 가 안 보이면 호스트의 KATECH bag_cue_server.py) — 끄고 다시"
    ss -ltnp 2>/dev/null | grep -E "[:.]$CUE_PORT " | sed 's/^/      /'
    exit 1
  fi
  rviz_config
  if morai_ok; then echo "  [0] MORAI $MORAI_ADDR:$MORAI_PORT 도달 OK"
  else echo "  [0] ! MORAI $MORAI_ADDR:$MORAI_PORT 에 아직 못 붙는다 — 브리지가 붙을 때까지 다시 시도한다"; fi
  echo "  [1] 브리지 (MORAI 읽기만 · 외란 엔진 · 노드 /$BRIDGE_NODE)"
  spawn bridge "ros2 launch autoware_morai_interface morai_interface_gt.launch.xml node_name:=$BRIDGE_NODE" \
    "morai_address:=$MORAI_ADDR morai_port:=$MORAI_PORT overlay_param_file:=$ROOT/config/viz_bridge.param.yaml"
  # 26.11.13 _ YSH _ 표시 노드를 Autoware 메시지만 내는 viz_autoware.py 로. 원문:
  #   echo "  [2] 표시 노드"
  #   spawn marker "python3 -B $SCRIPTS/viz_marker.py"
  echo "  [2] 표시 노드 (Prediction · GT · 상태 한 줄$([ -z "$MAP_PATH" ] && echo ' · viewer=자차'))"
  spawn display "python3 -B $SCRIPTS/viz_autoware.py $([ -n "$MAP_PATH" ] && echo --no-viewer)"
  echo "  [3] cue 수신기 (:$CUE_PORT · READY $READY → $MORAI_CUE · 녹화 $RECORD)"
  spawn cue_server "python3 -B $SCRIPTS/viz_cue_server.py --port $CUE_PORT --morai-cue $MORAI_CUE --ready $READY" \
    "--record $RECORD --data $DATA --bridge-node $BRIDGE_NODE"
  # 26.11.13 _ YSH _ RViz 를 e2e 와 같은 launch(차량 모델·[지도]·autoware RViz)로. 원문:
  #   if [ "${VIZ_NO_RVIZ:-0}" != "1" ]; then
  #     echo "  [4] RViz"
  #     spawn rviz "rviz2 -d $ROOT/rviz/disturbance_viz.rviz"
  #   fi
  echo "  [4] Autoware RViz (차량 모델$([ -n "$MAP_PATH" ] && echo " · 지도 $MAP_PATH")$([ "${VIZ_NO_RVIZ:-0}" = "1" ] && echo ' · RViz 없이'))"
  # 26.11.13 _ YSH _ 경로로. 원문: spawn rviz "ros2 launch autoware_launch viz_disturbance.launch.xml $(launch_args false)"
  spawn rviz "ros2 launch $VIZ_LAUNCH $(launch_args false)"
  local i
  for i in $(seq 1 20); do
    grep -aq -e "대기 중" -e "열 수 없다" "$LOGDIR/cue_server_$TS.log" 2>/dev/null && break
    sleep 0.5
  done
  if ! grep -aq "대기 중" "$LOGDIR/cue_server_$TS.log" 2>/dev/null; then
    echo "  ★ cue 수신기가 뜨지 않았다 — $LOGDIR/cue_server_$TS.log"
  fi
  sleep 2
  status
  echo "  실시간 보기(다른 터미널): $(dirname "$(readlink -f "$VIZ_SHELL")")/viz_shell.sh -c" \
       "'python3 -B diag/dist_watch.py --log-dir $DATA/cue_logs --flips'"
}

stop() {
  local pids p i
  pids="$(ours)"
  [ -z "$pids" ] && { echo "== 떠 있는 것 없음"; return 0; }
  echo "== 외란 시각화 끄기"
  for p in $pids; do                       # 1) 수신기 먼저 — clear 를 보내고 녹화를 닫는다
    case "$(cmd_of "$p")" in *viz_cue_server.py*|*bag\ play*) kill -INT "$p" 2>/dev/null ;; esac
  done
  for i in $(seq 1 16); do
    [ -z "$(for p in $(ours); do cmd_of "$p"; done | grep -e viz_cue_server.py -e 'bag record')" ] && break
    sleep 0.5
  done
  # 26.11.13 _ YSH _ RViz 는 respawn 이라 launch 를 먼저 내린다 (RViz 가 먼저 죽으면 launch 가 다시 띄운다)
  for p in $(ours); do case "$(cmd_of "$p")" in *ros2\ launch*) kill -INT "$p" 2>/dev/null ;; esac; done
  for i in $(seq 1 16); do
    [ -z "$(for p in $(ours); do cmd_of "$p"; done | grep -e 'ros2 launch')" ] && break
    sleep 0.5
  done
  for p in $(ours); do kill -INT "$p" 2>/dev/null; done   # 2) 나머지
  for i in $(seq 1 16); do [ -z "$(ours)" ] && break; sleep 0.5; done
  for p in $(ours); do kill -TERM "$p" 2>/dev/null; done
  sleep 2
  for p in $(ours); do kill -KILL "$p" 2>/dev/null; done
  sleep 0.5
  if [ -z "$(ours)" ]; then echo "  다 내렸다"; else echo "  ! 남은 것:"; for p in $(ours); do echo "    $p $(cmd_of "$p")"; done; fi
}

replay() {
  local bag="$1"
  [ -n "$bag" ] && [ -f "$bag/metadata.yaml" ] || { echo "  쓰는 법: ./run_viz.sh replay <bag 폴더 (metadata.yaml 있는 곳)>"; exit 2; }
  bag="$(readlink -f "$bag")"
  mkdir -p "$LOGDIR"
  guard_start
  echo "== 다시 보기 $bag$([ -n "$MAP_PATH" ] && echo " · 지도 $MAP_PATH")"
  rviz_config
  # 26.11.13 _ YSH _ Autoware RViz launch + viz_autoware.py 로. bag 의 /tf 는 틀지 않는다 (live 때 낸 viewer 가 섞여 있어
  #   지도의 viewer 와 싸운다) — map→base_link 는 viz_autoware 가 kinematic_state 로 다시 낸다 (브리지 TF 와 같은 자세).
  #   표시 노드가 다시 만드는 토픽(Prediction·GT·상태 글자)도 bag 에 있으면 빼고 튼다. 원문:
  #   spawn marker "python3 -B $SCRIPTS/viz_marker.py --ros-args -p use_sim_time:=true"
  #   [ "${VIZ_NO_RVIZ:-0}" != "1" ] && spawn rviz "rviz2 -d $ROOT/rviz/disturbance_viz.rviz --ros-args -p use_sim_time:=true"
  #   "$VIZ_SHELL" -c "export $MARK=1; ros2 bag play --clock 50 $bag ${VIZ_PLAY_ARGS:-}"
  local topics
  topics="$(python3 - "$bag/metadata.yaml" <<'PYEOF'
import sys, yaml
skip = {'/tf', '/tf_static', '/perception/object_recognition/objects', '/clock'}
m = yaml.safe_load(open(sys.argv[1]))['rosbag2_bagfile_information']
print(' '.join(t['topic_metadata']['name'] for t in m['topics_with_message_count']
               if t['topic_metadata']['name'] not in skip and not t['topic_metadata']['name'].startswith('/viz/debug/')))
PYEOF
)"
  [ -n "$topics" ] || { echo "  ★ bag 에 틀 토픽이 없다"; exit 1; }
  # 26.11.13 _ YSH _ 다시 보기는 벽시계로 (use_sim_time 끔). sim time 이면 재생 시작 때 시간이 과거로 뛰어 RViz 가 모든 표시를
  #   reset 하고, 한 번만 오는 지도(차선)·속도판이 지워진다 (리허설에서 확인). 메시지·TF 는 모두 bag 의 같은 stamp 라 서로 맞는다.
  #   sim time 이 꼭 필요하면 VIZ_REPLAY_SIMTIME=1. 원문:
  #   spawn display "python3 -B $SCRIPTS/viz_autoware.py --tf-from-odom $([ -n "$MAP_PATH" ] && echo --no-viewer) --ros-args -p use_sim_time:=true"
  #   spawn rviz "ros2 launch $VIZ_LAUNCH $(launch_args true)"
  local st=false clk=""
  [ "${VIZ_REPLAY_SIMTIME:-0}" = "1" ] && { st=true; clk="--clock 50"; }
  spawn display "python3 -B $SCRIPTS/viz_autoware.py --tf-from-odom $([ -n "$MAP_PATH" ] && echo --no-viewer) --ros-args -p use_sim_time:=$st"
  spawn rviz "ros2 launch $VIZ_LAUNCH $(launch_args $st)"
  sleep "${VIZ_REPLAY_WAIT:-6}"
  echo "  재생 중 (Ctrl+C 로 멈춤) — 끝나면 ./run_viz.sh stop"
  # 원문: "$VIZ_SHELL" -c "export $MARK=1; ros2 bag play $bag --clock 50 ${VIZ_PLAY_ARGS:-} --topics $topics"
  "$VIZ_SHELL" -c "export $MARK=1; ros2 bag play $bag $clk ${VIZ_PLAY_ARGS:-} --topics $topics"
  echo "  재생 끝 — 다시: ./run_viz.sh replay $bag · 닫기: ./run_viz.sh stop"
}

case "${1:-}" in
  start) start ;;
  stop) stop ;;
  status) status ;;
  replay) replay "${2:-}" ;;
  *) sed -n '2,15p' "$SELF" | sed 's/^# \{0,1\}//'; exit 2 ;;
esac
