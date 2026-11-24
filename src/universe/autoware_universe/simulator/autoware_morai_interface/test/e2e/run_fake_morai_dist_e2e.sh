#!/bin/bash
# 신규 — 26.11.10 _ YSH _ 외란 주입 종단 시험 실행기 — 깨끗한 셸(ROS_DOMAIN_ID 91, localhost)로 fake_morai_dist_e2e.py 를 돌린다
#
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
#
# 사용: run_fake_morai_dist_e2e.sh OUT_DIR [--port 17881]
#   먼저 워크스페이스를 빌드한다 (fake_morai_server 는 BUILD_TESTING 빌드의 build/morai_grpc_vendor/ 에만 있다).
#   가짜 MORAI 와 노드는 --port 로 준 고포트(기본 17881, 제어 17882)만 쓴다. 실제 MORAI(7789)·리그 포트에는 붙지 않는다.
#   리그(Autoware·브리지)가 떠 있으면 돌리지 않는다 — 같은 PC 에서 DDS·CPU 를 같이 쓴다 (그래도 하려면 E2E_WHILE_RIG=1).
#   환경변수: WS(워크스페이스, 기본은 이 파일에서 7 단계 위), E2E_DOMAIN(기본 91, 10 은 거절)
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${WS:-$(cd "$HERE/../../../../../../.." && pwd)}"
OUT="${1:?usage: run_fake_morai_dist_e2e.sh OUT_DIR [--port 17881]}"
shift
DOMAIN="${E2E_DOMAIN:-91}"
if [ "$DOMAIN" = "10" ]; then
  echo "run_fake_morai_dist_e2e: ROS_DOMAIN_ID 10 은 리그 도메인이다 — 쓰지 않는다" >&2
  exit 2
fi
# 리그가 떠 있으면 멈춘다 (/proc 의 실행 파일 이름을 직접 읽는다 — dist_build.sh 와 같은 판정)
if [ -z "${E2E_WHILE_RIG:-}" ]; then
  for d in /proc/[0-9]*; do
    exe=$(readlink "$d/exe" 2>/dev/null) || continue
    case "${exe##*/}" in
      component_container|component_container_mt|component_container_isolated|autoware_morai_interface_node)
        echo "run_fake_morai_dist_e2e: 리그 프로세스가 떠 있다 (${d#/proc/} $exe) — 리그를 내리고 다시 부를 것" >&2
        exit 1 ;;
    esac
  done
fi
mkdir -p "$OUT"
exec env -i HOME="$HOME" TERM=xterm LANG=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="$DOMAIN" ROS_LOCALHOST_ONLY=1 \
  bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash && source "$0/install/setup.bash" && exec python3 -B "$1" --ws "$0" --out "$2" "${@:3}"' \
  "$WS" "$HERE/fake_morai_dist_e2e.py" "$OUT" "$@"
