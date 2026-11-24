#!/bin/bash
# 신규 — 26.11.13 _ YSH _ 13_ 외란 시각화용 깨끗한 셸.
# 26.11.13 _ YSH _ 11_ dist_shell.sh 를 부르던 것을 13_ 자체 Autoware(MR_Morai-Autoware_Viz) install 을 source 하도록 바꿨다
#   (11_ dist_shell.sh 와 같은 방식). 원문:
#   DIST_SHELL="${VIZ_DIST_SHELL:-/home/autoware-morai/11_Disturbance-Autoware/MR_Morai-Autoware-Bridge_Disturbance/scripts/dist_shell.sh}"
#   DOMAIN="${VIZ_DOMAIN:-13}"
#   if [ ! -x "$DIST_SHELL" ]; then
#     echo "viz_shell: $DIST_SHELL 가 없다 — 11_ 폴더 이름이 바뀌었나? (13_ 은 11_ 설치본을 쓴다)"
#     exit 2
#   fi
#   case "$DOMAIN" in
#     ''|*[!0-9]*|10) echo "viz_shell: 도메인 '$DOMAIN' 은 쓸 수 없다 (10 은 운용 리그)"; exit 2 ;;
#   esac
#   exec "$DIST_SHELL" "$DOMAIN" "$@"
#
#   평소 셸은 .bashrc 가 10_ install 을 source 한다. 이 스크립트는 env -i 로 비운 셸에서 13_ install 만 source 한다.
#   도메인 10(운용 리그)은 거부. ROS_LOCALHOST_ONLY=1 — 이 PC 안에서만 보인다 (RViz 도 같은 PC).
#
# 쓰는 법
#   ./viz_shell.sh              시각화용 셸 (도메인 13)
#   ./viz_shell.sh -c '명령'    명령 하나만 돌리고 나온다
#   VIZ_DOMAIN=91 ./viz_shell.sh   다른 도메인 (오프라인 리허설)

SELF="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPTS="$(dirname "$SELF")"
ROOT="$(dirname "$SCRIPTS")"
# 26.11.24 _ YSH _ 공유본은 src/ 가 저장소 바로 아래다. 원문:
# AW="$ROOT/MR_Morai-Autoware_Viz"
if [ -d "$ROOT/src" ]; then AW="$ROOT"; else AW="$ROOT/MR_Morai-Autoware_Viz"; fi

if [ -n "${VIZ_SHELL_INNER:-}" ]; then
  # ── 안쪽: 비운 환경에서 이 파일을 source 해 환경을 차린다 (--rcfile 또는 -c 의 첫 줄) ──
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="$VIZ_SHELL_DOMAIN"
  export ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
  export ACADOS_SOURCE_DIR=/opt/acados
  export LD_LIBRARY_PATH="/opt/acados/lib:/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  set +u
  source /opt/ros/humble/setup.bash
  if [ -f "$AW/install/setup.bash" ]; then
    source "$AW/install/setup.bash"
  else
    echo "viz_shell: $AW 가 아직 빌드되지 않았다 — /opt/ros/humble 만 source 했다 (scripts/viz_build.sh viz)"
  fi
  export VIZ_ROOT="$ROOT"
  cd "$SCRIPTS" || true
  PS1='[13_시각화 d'"$ROS_DOMAIN_ID"'] \w\$ '
  unset VIZ_SHELL_INNER
  return 0 2>/dev/null || exit 0
fi

DOMAIN="${VIZ_DOMAIN:-13}"
case "$DOMAIN" in
  ''|*[!0-9]*|10) echo "viz_shell: 도메인 '$DOMAIN' 은 쓸 수 없다 (10 은 운용 리그)"; exit 2 ;;
esac

BASE_ENV=(HOME="$HOME" USER="${USER:-autoware}" TERM="${TERM:-xterm}" LANG=C.UTF-8 LC_ALL=C.UTF-8
  DISPLAY="${DISPLAY:-:0}" QT_X11_NO_MITSHM=1 NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
  PATH="$HOME/.local/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
  VIZ_SHELL_INNER=1 VIZ_SHELL_DOMAIN="$DOMAIN")

if [ "${1:-}" = "-c" ]; then
  [ $# -ge 2 ] || { echo "viz_shell: -c 뒤에 명령이 없다"; exit 2; }
  exec env -i "${BASE_ENV[@]}" bash --noprofile --norc -c 'source "$0" && eval "$1"' "$SELF" "$2"
fi
exec env -i "${BASE_ENV[@]}" bash --noprofile --rcfile "$SELF" -i
