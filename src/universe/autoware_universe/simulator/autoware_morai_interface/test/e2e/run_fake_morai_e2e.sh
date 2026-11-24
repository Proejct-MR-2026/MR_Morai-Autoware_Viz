#!/bin/bash
# 신규 — 26.09.25 _ YSH _ 가짜 MORAI 종단 시험 실행기 — 깨끗한 셸(ROS_DOMAIN_ID 91, localhost)로 fake_morai_e2e.py 를 돌린다
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
# 26.09.25 _ YSH _ 명령 트리거 A/B 인자 추가 (--command-trigger update|timer, --cmd-resend-hz). 원문:
#   사용: run_fake_morai_e2e.sh OUT_DIR [--port 17871] [--drive-s 10]
# 사용: run_fake_morai_e2e.sh OUT_DIR [--port 17871] [--drive-s 10] [--command-trigger update|timer] [--cmd-resend-hz 100]
#   먼저 워크스페이스를 빌드한다 (fake_morai_server 는 BUILD_TESTING 빌드의 build/morai_grpc_vendor/ 에만 있다).
#   실제 MORAI(7789)에는 붙지 않는다 — 가짜와 노드 모두 --port 로 준 고포트만 쓴다.
#   환경변수: WS(워크스페이스, 기본은 이 파일에서 7 단계 위), E2E_DOMAIN(기본 91)
set -eu
HERE="$(cd "$(dirname "$0")" && pwd)"
WS="${WS:-$(cd "$HERE/../../../../../../.." && pwd)}"
OUT="${1:?usage: run_fake_morai_e2e.sh OUT_DIR [checker args]}"
shift
mkdir -p "$OUT"
exec env -i HOME="$HOME" TERM=xterm LANG=C.UTF-8 PATH=/usr/local/bin:/usr/bin:/bin \
  RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_DOMAIN_ID="${E2E_DOMAIN:-91}" ROS_LOCALHOST_ONLY=1 \
  bash --noprofile --norc -c 'source /opt/ros/humble/setup.bash && source "$0/install/setup.bash" && exec python3 "$1" --ws "$0" --out "$2" "${@:3}"' \
  "$WS" "$HERE/fake_morai_e2e.py" "$OUT" "$@"
