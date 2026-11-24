#!/usr/bin/env python3
# 신규 — 26.11.10 _ YSH _ 외란 주입(MRDIST-1) 종단 시험 — 가짜 MORAI + C++ 노드, 수동 실행 전용 (colcon test 아님)
# 26.11.11 _ YSH _ 외란 메시지를 morai_ros2_msgs(외란 추가판)로 옮겼다 — 이 파일의 mr_disturbance_msgs 를 morai_ros2_msgs 로 바꿨다 (이름만, 주석 속 옛 이름·원문은 그대로)
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
"""가짜 MORAI + autoware_morai_interface 노드로 외란 주입 경로를 끝에서 끝까지 본다.

run_fake_morai_dist_e2e.sh 가 깨끗한 셸(ROS_DOMAIN_ID 91, localhost)로 부른다. 하네스(Fake·NodeProc·Checker·
Evidence)는 fake_morai_e2e.py 것을 그대로 쓴다. 노드 접착부(서비스 ~/set_disturbance, DAQ 게이트, 런 종료 자동 해제)는
gtest 로 못 보므로 여기서만 확인된다. 실제 MORAI(7789)와 리그 포트에는 붙지 않는다.

단계
  X0 서비스·상태 조회·게이트 전 무발행      X1 시드 구간 + 그대로 통과 (10 Hz, frame_seq, stamp)
  X2 위치 편차·잡음 = Python 기준 구현       X3 드롭아웃 사슬 = 기준 구현, 빈 목록도 같은 주기
  X4 blackout 1 s 자동 복구                  X5 끝없는 blackout + 런 종료 자동 해제
  X6 거절 3 건 뒤 상태 그대로                X7 GT 토픽은 그대로 (object_status == raw)
  X8 실차 모드(follow) 배선: 켜는 명령 REAL_CAR, 해제 통과, 외란 DAQ 발행자 없음
결과는 --out 디렉토리의 evidence.json·node.log·fake.log.
"""

import argparse
import json
import math
import os
import sys
import threading
import time
import uuid

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_morai_e2e import Checker  # noqa: E402
from fake_morai_e2e import Evidence  # noqa: E402
from fake_morai_e2e import Fake  # noqa: E402
from fake_morai_e2e import NODE_NAME  # noqa: E402
from fake_morai_e2e import NodeProc  # noqa: E402
from fake_morai_e2e import now_ns  # noqa: E402
from fake_morai_e2e import stamp_ns  # noqa: E402
from fake_morai_e2e import wait_until  # noqa: E402
from morai_ros2_msgs.msg import ObjectStatusList  # noqa: E402
from morai_ros2_msgs.msg import DisturbanceStatus  # noqa: E402
from morai_ros2_msgs.srv import SetDisturbance  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.qos import QoSProfile  # noqa: E402

FORBIDDEN_PORTS = (7789, 50052, 50053, 50152)   # 실제 MORAI · 리그 cue 포트
BASE_SEED = 123456789012345
SEED_2 = 987654321
RUN_1 = 'e2e_dist_1'
RUN_2 = 'e2e_dist_2'
SEC = 1_000_000_000

# =============================================================================
# MRDIST-1 기준 구현 (설계 문서 §4 글만 보고 다시 쓴 것 — C++ 을 옮긴 것이 아니다)
# =============================================================================

M64 = (1 << 64) - 1


def mix64(z):
    z &= M64
    z ^= z >> 30
    z = (z * 0xBF58476D1CE4E5B9) & M64
    z ^= z >> 27
    z = (z * 0x94D049BB133111EB) & M64
    z ^= z >> 31
    return z


def stream(seed, c):
    return mix64((seed + c * 0x9E3779B97F4A7C15) & M64)


def u01(seed, c):
    return ((stream(seed, c) >> 12) + 0.5) * 2.0 ** -52


def name_uuid(name):
    return uuid.uuid5(uuid.NAMESPACE_OID, 'morai:' + name).bytes


def object_seed(base, name):
    b = name_uuid(name)
    hi = int.from_bytes(b[:8], 'big')
    lo = int.from_bytes(b[8:], 'big')
    return mix64(mix64(base ^ 0x4F424A5345454431 ^ hi) ^ lo)


def dropout_seed(base):
    return mix64(base ^ 0x44524F504F555431)


def gauss2(seed, k):
    u1 = u01(seed, 2 * k)
    u2 = u01(seed, 2 * k + 1)
    r = math.sqrt(-2.0 * math.log(u1))
    a = 6.283185307179586 * u2
    return r * math.cos(a), r * math.sin(a)


def dropout_prob(keep, gap):
    t = 0.1
    g = max(gap, t)
    p_show = min(1.0, t / g)
    mean_on = g * keep / (1.0 - keep)
    p_hide = min(1.0, t / mean_on)
    return p_hide, p_show


def dropout_chain(base, keep, gap, k_on, k_last):
    """건 프레임 k_on 부터 k_last 까지 {k: hidden}."""
    seed = dropout_seed(base)
    p_hide, p_show = dropout_prob(keep, gap) if 0.0 < keep < 1.0 else (0.0, 0.0)
    hidden = keep <= 0.0
    out = {k_on: hidden}
    for k in range(k_on + 1, k_last + 1):
        if 0.0 < keep < 1.0:
            u = u01(seed, k)
            if hidden:
                if u < p_show:
                    hidden = False
            elif u < p_hide:
                hidden = True
        out[k] = hidden
    return out


def self_check():
    """기준 구현이 설계 문서의 골든 값을 내는지 (틀리면 아래 판정이 전부 무의미하다)."""
    car = object_seed(BASE_SEED, 'NPC_Car_1')
    z0 = gauss2(car, 0)
    chain = dropout_chain(BASE_SEED, 0.5, 0.2, 0, 59)
    pattern = ''.join('H' if chain[k] else '.' for k in range(60))
    return (mix64(1) == 0x5692161D100B05E5 and stream(0, 2) == 0x6E789E6AA1B965F4 and
            car == 0x8DF6BF7B5DBCDFCB and object_seed(BASE_SEED, 'NPC_Ped_1') == 0x11C916F768E8BDE8 and
            dropout_seed(BASE_SEED) == 0xFCC081FFA48A068B and u01(car, 0) == 0.47180387289432024 and
            abs(z0[0] + 0.7003756082673686) < 1e-12 and abs(z0[1] - 1.0059114344229763) < 1e-12 and
            dropout_prob(0.75, 0.5) == (0.06666666666666667, 0.2) and
            pattern == '...H.H....H..H.H....H.HH..H.HHHH.HH..HH...H..H..H...H.HH.HH.')


# =============================================================================
# ROS 쪽 — 기존 Checker 에 외란 토픽·서비스를 더한다
# =============================================================================


class DistChecker(Checker):

    def __init__(self):
        super().__init__()
        qos = QoSProfile(depth=100)
        with self.lock:
            for key in ('dist', 'dist_objects', 'gt_objects'):
                self.rec[key] = []
        self.create_subscription(DisturbanceStatus, '/daq/disturbance', self.on_dist, qos)
        self.create_subscription(
            ObjectStatusList, '/daq/object_status_disturbed',
            lambda m: self.add('dist_objects', (now_ns(), stamp_ns(m.header.stamp), self.objects_of(m),
                                                m.num_of_npcs + m.num_of_pedestrian + m.num_of_obstacle)), qos)
        # 객체별 GT (기존 Checker 는 개수만 적는다)
        self.create_subscription(
            ObjectStatusList, '/daq/object_status',
            lambda m: self.add('gt_objects', (now_ns(), stamp_ns(m.header.stamp), self.objects_of(m))), qos)
        self.dist_client = self.create_client(SetDisturbance, f'/{NODE_NAME}/set_disturbance')
        self.dist_seq = 0

    def add(self, key, item):
        with self.lock:
            self.rec[key].append(item)

    @staticmethod
    def objects_of(msg):
        return {o.name: (o.position.x, o.position.y, o.position.z, o.unique_id)
                for lst in (msg.npc_list, msg.pedestrian_list, msg.obstacle_list) for o in lst}

    def on_dist(self, msg):
        self.add('dist', (now_ns(), stamp_ns(msg.header.stamp), msg))

    def send(self, trigger, run='', base_seed=0, timeout=3.0):
        req = SetDisturbance.Request()
        req.trigger = trigger
        req.run = run
        req.base_seed = base_seed
        req.source = 'fake_morai_dist_e2e'
        self.dist_seq += 1
        req.seq = self.dist_seq
        return self.call(self.dist_client, req, timeout=timeout)

    def frames(self, t0=None, t1=None):
        """/daq/disturbance (도착 ns, stamp ns, msg), frame_seq 가 아니라 도착 순서."""
        return self.records('dist', t0, t1)

    def tracked_by_stamp(self):
        return {r[1]: r[3] for r in self.records('objects')}

    def dist_objects_by_stamp(self):
        return {r[1]: (r[2], r[3]) for r in self.records('dist_objects')}

    def raw_by_stamp(self):
        """stamp(time_source morai → morai_timestamp_us × 1000) → (자차 yaw [deg], {이름: (x, y, z)})."""
        out = {}
        for _, m in self.records('raw_states'):
            ego_yaw = None
            actors = {}
            for st in m['actor_states']['states']:
                name = st['actor_info']['id']['value']
                loc = st.get('transform', {}).get('location', {})
                rot = st.get('transform', {}).get('rotation', {})
                if name == 'Ego':
                    ego_yaw = rot.get('z', 0.0)
                else:
                    actors[name] = (loc.get('x', 0.0), loc.get('y', 0.0), loc.get('z', 0.0))
            out[int(m['morai_timestamp_us']) * 1000] = (ego_yaw, actors)
        return out

    def gt_by_stamp(self):
        return {r[1]: {n: v[:3] for n, v in r[2].items()} for r in self.records('gt_objects')}


def state_signature(msg):
    """명령으로만 바뀌는 칸 (프레임마다 달라지는 frame_seq·주기·객체 값은 뺀다)."""
    return (msg.fault_flag, msg.position_on, msg.dropout_on, msg.blackout_on, msg.objects_visible,
            tuple(msg.active_triggers), msg.lon_offset_m, msg.lon_std_m, msg.lat_offset_m, msg.lat_std_m,
            msg.keep_prob, msg.mean_gap_s, msg.blackout_duration_s, msg.blackout_recover, msg.base_seed,
            msg.dropout_seed, msg.seed_algorithm)


def reject_code(res):
    if res is None:
        return 'no response'
    return res.message.split(':', 1)[0] if not res.ok else 'OK'


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--ws', required=True, help='colcon workspace root (install/, build/)')
    ap.add_argument('--port', type=int, default=17881, help='가짜 MORAI 포트 (제어 포트 = +1)')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if args.port in FORBIDDEN_PORTS or args.port + 1 in FORBIDDEN_PORTS:
        raise SystemExit(f'refusing port {args.port} (real MORAI / rig ports {FORBIDDEN_PORTS})')
    if os.environ.get('ROS_DOMAIN_ID') == '10':
        raise SystemExit('refusing ROS_DOMAIN_ID 10 (rig domain) — use run_fake_morai_dist_e2e.sh')
    os.makedirs(args.out, exist_ok=True)
    fake_bin = os.path.join(args.ws, 'build/morai_grpc_vendor/fake_morai_server')
    if not os.path.exists(fake_bin):
        raise SystemExit(f'{fake_bin} not found — build morai_grpc_vendor with BUILD_TESTING')

    ev = Evidence()
    marks = {'port': args.port, 'domain': os.environ.get('ROS_DOMAIN_ID')}
    ev.add('R', 'Python MRDIST-1 reference reproduces the design golden values', 'True', str(self_check()),
           self_check())

    rclpy.init()
    checker = DistChecker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(checker)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    fake = Fake(fake_bin, args.port, args.out)
    node = NodeProc(args.ws, args.port, args.out, extra_params=('command_trigger:=update',))
    node2 = None
    try:
        fake.start()
        node.start()
        marks['node_start'] = now_ns()
        first = wait_until(lambda: checker.records('objects', marks['node_start']), 15.0)
        if not first:
            ev.add('X0', 'node connects to the fake and publishes TrackedObjects', 'within 15 s', 'never', False)
            raise RuntimeError('node did not publish')
        time.sleep(1.5)  # 첫 가드 판정(1 s) 뒤 — DAQ 가 양보를 풀어야 게이트가 뜻이 있다

        # ---------------------------------------------------------------- X0 서비스·상태 조회·게이트
        ready = checker.dist_client.wait_for_service(timeout_sec=5.0)
        res = checker.send('') if ready else None
        quiet = len(checker.frames()) + len(checker.records('dist_objects'))
        ok = (ready and res is not None and res.ok and res.message == 'OK STATUS' and res.base_seed == 0 and
              res.run == '' and res.command.type == '' and not res.command.clear and
              res.status.seed_algorithm == 'MRDIST-1' and not res.status.fault_flag and quiet == 0)
        ev.add('X0', '~/set_disturbance present; status query; nothing on the two DAQ topics before active',
               "service ready; ok, 'OK STATUS', base_seed 0, last status = MRDIST-1 / no fault; 0 messages",
               f"ready={ready} ok={res.ok if res else None} message='{res.message if res else ''}' "
               f"base_seed={res.base_seed if res else None} "
               f"status_alg='{res.status.seed_algorithm if res else ''}' "
               f"status_frame_seq={res.status.frame_seq if res else None} messages_before_active={quiet}", ok)

        # ---------------------------------------------------------------- X1 시드 구간 + 그대로 통과
        res = checker.send('disturbance, clear', RUN_1, BASE_SEED)
        epoch_ok = (res is not None and res.ok and 'EPOCH' in res.message.split() and
                    res.base_seed == BASE_SEED and res.run == RUN_1 and res.command.type == 'clear' and
                    res.command.clear and (res.applied_time.sec > 0))
        time.sleep(0.4)
        t_on = checker.set_daq(f'active {RUN_1}')
        marks['active_1'] = t_on
        time.sleep(2.6)
        w0, w1 = t_on + 300_000_000, t_on + 2_300_000_000
        fr = checker.frames(w0, w1)
        hz = len(fr) / ((w1 - w0) / 1e9)
        all_fr = checker.frames(None, now_ns() - 200_000_000)   # 짝(raw·GT)이 도착할 여유를 둔다
        seqs = [m.frame_seq for _, _, m in all_fr]
        seq_ok = bool(seqs) and all(b == a + 1 for a, b in zip(seqs, seqs[1:]))
        tracked = checker.tracked_by_stamp()
        stamps_ok = bool(all_fr) and all(st in tracked for _, st, _ in all_fr)
        gt = checker.gt_by_stamp()
        raw = checker.raw_by_stamp()
        dobj = checker.dist_objects_by_stamp()
        car_seed = object_seed(BASE_SEED, 'NPC_Car_1')
        ped_seed = object_seed(BASE_SEED, 'NPC_Ped_1')
        bad = []
        joined = 0
        for _, st, m in all_fr:
            names = [o.name for o in m.objects]
            truth = gt.get(st) or (raw[st][1] if st in raw else None)
            sent = dobj.get(st)
            if truth is None or sent is None:
                bad.append((m.frame_seq, 'no GT/disturbed message with this stamp'))
                continue
            joined += 1
            same = all(n in sent[0] and sent[0][n][:3] == tuple(truth[n]) for n in names)
            flags = (not m.fault_flag and not m.position_on and not m.dropout_on and not m.blackout_on and
                     m.objects_visible and not m.active_triggers and m.base_seed == BASE_SEED and
                     m.dropout_seed == dropout_seed(BASE_SEED) and m.seed_algorithm == 'MRDIST-1' and
                     m.keep_prob == 1.0 and m.header.frame_id == 'map')
            objs = (sorted(names) == ['NPC_Car_1', 'NPC_Ped_1'] and len(sent[0]) == 2 and sent[1] == 2 and
                    all(o.visible and o.dx_m == 0.0 and o.dy_m == 0.0 and o.noise_lon_m == 0.0 and
                        o.seed == object_seed(BASE_SEED, o.name) and
                        o.unique_id == int.from_bytes(name_uuid(o.name)[:4], 'big', signed=True) and
                        o.unique_id == sent[0][o.name][3] for o in m.objects))
            if not (same and flags and objs):
                bad.append((m.frame_seq, f'same={same} flags={flags} objs={objs}'))
        ev.add('X1', 'seed epoch call, then active: pass-through frames on /daq/disturbance',
               "epoch call ok with EPOCH; 9-11 Hz; frame_seq +1 per message; every stamp is a TrackedObjects "
               'stamp; fault_flag false, base_seed echoed; disturbed list == GT of the same stamp',
               f"epoch '{res.message if res else None}' ok={epoch_ok}; {hz:.2f} Hz ({len(fr)} in 2 s); "
               f'frame_seq {seqs[0] if seqs else None}..{seqs[-1] if seqs else None} contiguous {seq_ok}; '
               f'stamps ok {stamps_ok}; joined {joined}/{len(all_fr)}; bad {len(bad)} {bad[:2]}',
               epoch_ok and 9.0 <= hz <= 11.0 and seq_ok and stamps_ok and joined == len(all_fr) and
               not bad and seqs[0] > 0)

        # ---------------------------------------------------------------- X2 위치 편차·잡음
        trigger = 'disturbance, position, 1.4, 0.4, -0.5, 0.2'
        res = checker.send(trigger, RUN_1, BASE_SEED)   # 리그처럼 run·시드를 늘 싣는다
        t_x2 = now_ns()
        cmd_ok = (res is not None and res.ok and res.message == 'OK EPOCH_KEPT' and
                  res.command.type == 'position' and not res.command.clear and
                  (res.command.lon_offset_m, res.command.lon_std_m, res.command.lat_offset_m,
                   res.command.lat_std_m) == (1.4, 0.4, -0.5, 0.2) and res.base_seed == BASE_SEED)
        time.sleep(3.8)
        res_clear = checker.send('disturbance, position, clear', RUN_1, BASE_SEED)
        t_x2_end = now_ns()
        time.sleep(0.5)
        fr = [f for f in checker.frames(t_x2 - SEC, t_x2_end + SEC) if f[2].position_on]
        gt = checker.gt_by_stamp()
        raw = checker.raw_by_stamp()
        dobj = checker.dist_objects_by_stamp()
        tracked = checker.tracked_by_stamp()
        n_obj = 0
        exact = 0
        max_rot = 0.0
        max_noise = 0.0
        max_track = 0.0
        problems = []
        for _, st, m in fr:
            if st not in raw or st not in dobj or st not in tracked:
                problems.append((m.frame_seq, 'missing raw/disturbed/tracked for this stamp'))
                continue
            yaw_deg, raw_actors = raw[st]
            truth = gt.get(st) or raw_actors
            psi = math.radians(yaw_deg)
            tr = {o[0]: o for o in tracked[st]}
            for o in m.objects:
                n_obj += 1
                sent = dobj[st][0].get(o.name)
                if sent is None or o.name not in truth:
                    problems.append((m.frame_seq, o.name, 'not in disturbed list / GT'))
                    continue
                gx, gy, gz = truth[o.name]
                # (1) 보낸 값 − GT == dx/dy (정확히). z 는 GT
                if sent[0] - gx == o.dx_m and sent[1] - gy == o.dy_m and sent[2] == gz and o.visible:
                    exact += 1
                else:
                    problems.append((m.frame_seq, o.name, 'dx/dy != sent - GT'))
                # (2) 잡음 = std × gauss2(object_seed, frame_seq)
                z_lon, z_lat = gauss2(object_seed(BASE_SEED, o.name), m.frame_seq)
                max_noise = max(max_noise, abs(o.noise_lon_m - 0.4 * z_lon), abs(o.noise_lat_m - 0.2 * z_lat))
                # (3) dx/dy = R(자차 yaw) × (편차 + 잡음)
                d_lon = 1.4 + 0.4 * z_lon
                d_lat = -0.5 + 0.2 * z_lat
                wx = d_lon * math.cos(psi) - d_lat * math.sin(psi)
                wy = d_lon * math.sin(psi) + d_lat * math.cos(psi)
                max_rot = max(max_rot, abs(o.dx_m - wx), abs(o.dy_m - wy))
                # (4) TrackedObjects pose = 보낸 위치 (map 원점 0, 가짜는 상자 중심을 주지 않는다)
                t = tr.get(name_uuid(o.name).hex())
                if t is None:
                    problems.append((m.frame_seq, o.name, 'not in TrackedObjects'))
                else:
                    max_track = max(max_track, abs(t[2] - sent[0]), abs(t[3] - sent[1]))
                if o.seed != object_seed(BASE_SEED, o.name):
                    problems.append((m.frame_seq, o.name, 'seed'))
        latency = (fr[0][0] - t_x2) / 1e9 if fr else None
        status_ok = bool(fr) and all(
            m.fault_flag and m.objects_visible and list(m.active_triggers) == [trigger] and
            (m.lon_offset_m, m.lon_std_m, m.lat_offset_m, m.lat_std_m) == (1.4, 0.4, -0.5, 0.2)
            for _, _, m in fr)
        ev.add('X2', 'position 1.4, 0.4, -0.5, 0.2 against the Python reference (both NPCs, same-stamp join)',
               "command 'OK EPOCH_KEPT'; >= 30 frames; disturbed - GT == (dx_m, dy_m) exactly; (dx, dy) == "
               'R(ego yaw from raw) x (offset + noise) within 1e-9; noise == std x gauss2 within 1e-12; '
               'TrackedObjects pose == disturbed position within 1e-9; two different seeds',
               f"command '{res.message if res else None}' ok={cmd_ok}; frames {len(fr)} objects {n_obj}; "
               f'exact {exact}/{n_obj}; max |rot err| {max_rot:.3g}; max |noise err| {max_noise:.3g}; '
               f'max |tracked - sent| {max_track:.3g}; seeds {car_seed:#x} / {ped_seed:#x}; status ok '
               f'{status_ok}; first affected frame {latency if latency is None else round(latency, 3)} s after '
               f'the call; problems {len(problems)} {problems[:3]}',
               cmd_ok and len(fr) >= 30 and n_obj == 2 * len(fr) and exact == n_obj and max_rot <= 1e-9 and
               max_noise <= 1e-12 and max_track <= 1e-9 and car_seed != ped_seed and status_ok and
               not problems and res_clear is not None and res_clear.ok)

        # ---------------------------------------------------------------- X3 드롭아웃
        trigger = 'disturbance, dropout, 0.5, 0.2'
        res = checker.send(trigger, RUN_1, BASE_SEED)
        t_x3 = now_ns()
        time.sleep(6.2)
        res_clear = checker.send('disturbance, dropout, clear', RUN_1, BASE_SEED)
        t_x3_end = now_ns()
        time.sleep(0.8)
        fr = sorted((f for f in checker.frames(t_x3 - SEC, t_x3_end + SEC) if f[2].dropout_on),
                    key=lambda f: f[1])
        tracked = checker.tracked_by_stamp()
        dobj = checker.dist_objects_by_stamp()
        mism = []
        hidden_n = 0
        list_bad = []
        if fr:
            k_on = fr[0][2].frame_seq
            ref = dropout_chain(BASE_SEED, 0.5, 0.2, k_on, fr[-1][2].frame_seq)
            for _, st, m in fr:
                want_hidden = ref[m.frame_seq]
                hidden_n += 0 if m.objects_visible else 1
                if m.objects_visible == want_hidden:
                    mism.append(m.frame_seq)
                t = tracked.get(st)
                d = dobj.get(st)
                want_n = 0 if not m.objects_visible else len(m.objects)
                if t is None or d is None or len(t) != want_n or len(d[0]) != want_n or d[1] != want_n or \
                        any(o.visible != m.objects_visible for o in m.objects) or len(m.objects) != 2:
                    list_bad.append(m.frame_seq)
        seqs = [m.frame_seq for _, _, m in fr]
        seq_ok = bool(seqs) and all(b == a + 1 for a, b in zip(seqs, seqs[1:]))
        tr_arr = [r[0] for r in checker.records('objects', t_x3, t_x3_end)]
        tr_gap = max((b - a) / 1e9 for a, b in zip(tr_arr, tr_arr[1:])) if len(tr_arr) > 1 else float('inf')
        gt_rows = checker.records('gt_objects', t_x3 + 200_000_000, t_x3_end - 200_000_000)
        gt_hz = len(gt_rows) / ((t_x3_end - t_x3 - 400_000_000) / 1e9)
        gt_full = bool(gt_rows) and all(len(r[2]) == 2 for r in gt_rows)
        after = [m for _, _, m in checker.frames(t_x3_end + 300_000_000)]
        after_ok = bool(after) and all(
            not m.dropout_on and m.objects_visible and not m.fault_flag and m.keep_prob == 1.0 for m in after)
        ev.add('X3', 'dropout 0.5, 0.2 for ~6 s: hidden pattern == reference chain from the ON frame',
               'objects_visible per frame == reference; ON frame visible; TrackedObjects on every frame (no '
               'arrival gap > 0.2 s) and empty exactly on hidden frames; /daq/object_status keeps 2 objects at '
               '100 Hz; after the kind clear all visible, fault_flag false',
               f"command '{res.message if res else None}'; frames {len(fr)} (contiguous {seq_ok}) hidden "
               f'{hidden_n}; pattern mismatches {len(mism)} {mism[:5]}; list mismatches {len(list_bad)} '
               f'{list_bad[:5]}; TrackedObjects max gap {tr_gap:.3f} s; GT object_status {gt_hz:.1f} Hz full '
               f'{gt_full}; after clear ok {after_ok} ({len(after)} frames)',
               res is not None and res.ok and len(fr) >= 50 and seq_ok and not mism and not list_bad and
               0 < hidden_n < len(fr) and fr[0][2].objects_visible and tr_gap <= 0.2 and
               abs(gt_hz - 100.0) <= 5.0 and gt_full and after_ok and res_clear is not None and res_clear.ok)

        # ---------------------------------------------------------------- X4 blackout 1 s 자동 복구
        trigger = 'disturbance, blackout, 1.0, 1'
        t_x4 = now_ns()
        res = checker.send(trigger, RUN_1, BASE_SEED)
        time.sleep(2.2)
        t_x4_end = now_ns()
        time.sleep(0.3)   # 같은 stamp 의 TrackedObjects 가 도착할 여유
        fr = sorted(checker.frames(t_x4 - 200_000_000, t_x4_end), key=lambda f: f[1])
        hid = [f for f in fr if f[2].blackout_on]
        tracked = checker.tracked_by_stamp()
        span = first_vis = period_last = period_vis = None
        vis_ok = False
        lists_ok = False
        if hid:
            s0 = hid[0][1]
            span = (hid[-1][1] - s0) / 1e9
            later = [f for f in fr if f[2].frame_seq > hid[-1][2].frame_seq]
            # 26.11.10 _ YSH _ MRDIST-1 은 지속을 정수 ns 로 더한다 — 0.1 s 를 실수로 열 번 더하면 0.9999999999999999 라
            #   경계(정확히 1.0 s 에 복구)에서 시험만 떨어졌다. frame_period_s 를 ns 로 되돌려 정수로 더한다. 원문:
            # period_last = sum(f[2].frame_period_s for f in hid[1:])
            period_last = sum(round(f[2].frame_period_s * 1e9) for f in hid[1:]) / 1e9
            if later:
                first_vis = (later[0][1] - s0) / 1e9
                # 원문: period_vis = period_last + later[0][2].frame_period_s
                period_vis = (round(period_last * 1e9) + round(later[0][2].frame_period_s * 1e9)) / 1e9
                vis_ok = all(not m.blackout_on and not m.fault_flag and not m.active_triggers and
                             m.objects_visible and m.blackout_duration_s == 0.0 and not m.blackout_recover
                             for _, _, m in later)
            lists_ok = (all(st in tracked and len(tracked[st]) == 0 for _, st, _ in hid) and
                        all(st in tracked and len(tracked[st]) == 2 for _, st, _ in later) and
                        all(not m.objects_visible and list(m.active_triggers) == [trigger] and m.fault_flag and
                            m.blackout_duration_s == 1.0 and m.blackout_recover for _, _, m in hid))
        ev.add('X4', 'blackout 1.0 s with recovery',
               'empty frames span < 1.0 s from the first empty frame; first visible frame at >= 1.0 s and '
               '< 1.2 s; summed frame_period_s < 1.0 at the last hidden frame and >= 1.0 at the first visible; '
               'afterwards blackout_on false, fault_flag false, active_triggers empty',
               f"command '{res.message if res else None}'; hidden frames {len(hid)}; span {span}; first visible "
               f'{first_vis}; period sums {period_last} / {period_vis}; after ok {vis_ok}; lists ok {lists_ok}',
               res is not None and res.ok and bool(hid) and span is not None and span < 1.0 and
               first_vis is not None and 1.0 <= first_vis < 1.2 and period_last < 1.0 <= period_vis and
               vis_ok and lists_ok)

        # ---------------------------------------------------------------- X5 끝없는 blackout + 런 종료 자동 해제
        res = checker.send('disturbance, blackout, 0, 0', RUN_1, BASE_SEED)
        t_x5 = now_ns()
        time.sleep(1.5)
        hid = [f for f in checker.frames(t_x5) if f[2].blackout_on]
        tr_hidden = [r for r in checker.records('objects', hid[0][0] + 50_000_000)] if hid else []
        all_empty = bool(tr_hidden) and all(len(r[3]) == 0 for r in tr_hidden)
        t_idle = checker.set_daq(f'idle {RUN_1}')
        marks['idle_1'] = t_idle
        back = wait_until(lambda: [r for r in checker.records('objects', t_idle) if len(r[3]) == 2], 2.0)
        back_s = (back[0][0] - t_idle) / 1e9 if back else None
        time.sleep(1.0)
        late = checker.frames(t_idle + 150_000_000) + checker.records('dist_objects', t_idle + 150_000_000)
        q = checker.send('', RUN_1)
        q_ok = (q is not None and q.ok and q.message == 'OK STATUS' and q.base_seed == BASE_SEED and
                q.run == RUN_1 and not q.status.fault_flag and not q.status.blackout_on and
                not q.status.active_triggers and q.status.objects_visible and q.status.base_seed == BASE_SEED)
        ev.add('X5', 'permanent blackout (0, 0), then idle: auto-clear at run end',
               'only empty TrackedObjects during the blackout; after idle TrackedObjects carry 2 objects again '
               'within 0.3 s; the two DAQ topics stop; status query: nothing active, epoch kept',
               f"command '{res.message if res else None}'; blackout frames {len(hid)}, TrackedObjects "
               f'{len(tr_hidden)} all empty {all_empty}; objects back {back_s} s after idle; DAQ messages '
               f"after idle+0.15 s {len(late)}; query '{q.message if q else None}' ok={q_ok} (status frame_seq "
               f'{q.status.frame_seq if q else None})',
               res is not None and res.ok and len(hid) >= 10 and all_empty and back_s is not None and
               back_s <= 0.3 and not late and q_ok)

        # ---------------------------------------------------------------- X6 거절은 아무것도 바꾸지 않는다
        res = checker.send('disturbance, clear', RUN_2, SEED_2)
        begin_ok = res is not None and res.ok and 'EPOCH' in res.message.split() and res.base_seed == SEED_2
        t_on2 = checker.set_daq(f'active {RUN_2}')
        marks['active_2'] = t_on2
        res_on = checker.send('disturbance, position, 2, 0, 0, 0', RUN_2, SEED_2)
        time.sleep(1.2)
        t_bad0 = now_ns()
        bad_calls = [
            ('disturbance, dropout, 1.5, 1', RUN_2, SEED_2, 'OUT_OF_RANGE'),
            ('disturbance, mount, 1', RUN_2, SEED_2, 'UNSUPPORTED_KIND'),
            ('disturbance, blackout, 0, 0', RUN_1, 0, 'RUN_MISMATCH'),   # 앞 런의 표시를 단 켜는 명령
        ]
        got = []
        rej_ok = True
        for trig, run, seed, code in bad_calls:
            r = checker.send(trig, run, seed)
            got.append(reject_code(r))
            rej_ok = rej_ok and (r is not None and not r.ok and r.message.startswith(code + ': ') and
                                 r.applied_time.sec == 0 and r.applied_time.nanosec == 0 and
                                 r.run == RUN_2 and r.base_seed == SEED_2)
        t_bad1 = now_ns()
        time.sleep(1.0)
        before = [f for f in checker.frames(t_on2 + 300_000_000, t_bad0)][-3:]
        after = [f for f in checker.frames(t_bad1 + 150_000_000)][:3]
        span_fr = checker.frames(before[0][0] if before else t_bad0)
        seqs = [m.frame_seq for _, _, m in span_fr]
        seq_ok = len(seqs) >= 6 and all(b == a + 1 for a, b in zip(seqs, seqs[1:]))
        sig = {state_signature(m) for _, _, m in before + after}
        tracked = checker.tracked_by_stamp()
        tr_ok = bool(after) and all(st in tracked and len(tracked[st]) == 2 for _, st, _ in after)
        same = len(before) == 3 and len(after) == 3 and len(sig) == 1 and before[0][2].position_on and \
            before[0][2].lon_offset_m == 2.0
        # 결정 (b): 다른 run 표시를 단 해제는 받는다
        foreign_clear = checker.send('disturbance, clear', RUN_1, 0)
        fc_ok = foreign_clear is not None and foreign_clear.ok
        time.sleep(0.4)
        cleared = [m for _, _, m in checker.frames(now_ns() - 200_000_000)]
        fc_ok = fc_ok and bool(cleared) and not cleared[-1].fault_flag
        t_idle2 = checker.set_daq(f'idle {RUN_2}')
        marks['idle_2'] = t_idle2
        ev.add('X6', 'three rejected calls leave the next frames unchanged (stale run tag on an ON command)',
               'OUT_OF_RANGE, UNSUPPORTED_KIND, RUN_MISMATCH with ok false, applied_time 0, current run/seed '
               'echoed; 3 frames before == 3 frames after (state fields), frame_seq keeps +1; a clear with a '
               'foreign run tag is accepted',
               f"begin ok {begin_ok}; ON '{res_on.message if res_on else None}'; codes {got} ok={rej_ok}; "
               f'before/after {len(before)}/{len(after)} distinct states {len(sig)}; frame_seq contiguous '
               f'{seq_ok} ({seqs[:1]}..{seqs[-1:]}); TrackedObjects 2 objects {tr_ok}; foreign-tag clear ok '
               f'{fc_ok}',
               begin_ok and res_on is not None and res_on.ok and rej_ok and same and seq_ok and tr_ok and fc_ok)

        # ---------------------------------------------------------------- X7 GT 토픽은 그대로
        raw = checker.raw_by_stamp()
        rows = checker.records('gt_objects', t_x2, t_x4_end)
        compared = 0
        differ = 0
        for _, st, objs in rows:
            if st not in raw:
                continue
            for name, pos in objs.items():
                compared += 1
                if tuple(pos[:3]) != tuple(raw[st][1].get(name, ())):
                    differ += 1
        ev.add('X7', '/daq/object_status stays ground truth while position / dropout / blackout are applied',
               'every object position == raw actor_states location of the same stamp (X2..X4 windows)',
               f'{len(rows)} messages, {compared} positions compared, {differ} differ',
               compared > 500 and differ == 0)

        # ---------------------------------------------------------------- 종료
        time.sleep(0.5)
        code = node.sigint(timeout=10.0)
        ev.add('XS', 'SIGINT the node', 'exit code 0', f'exit code {code}', code == 0)

        # ---------------------------------------------------------------- X8 실차 모드 배선 (follow)
        node2 = NodeProc(args.ws, args.port, args.out, extra_params=('ego_mode:=follow',))
        node2.start()
        ready = wait_until(lambda: checker.dist_client.service_is_ready(), 10.0)
        time.sleep(1.5)
        on_codes = [reject_code(checker.send(t, 'vils_run', 0)) for t in (
            'disturbance, position, 1, 0, 0, 0', 'disturbance, dropout, 0.75, 0.5',
            'disturbance, blackout, 0, 0')]
        passes = [checker.send(t, 'vils_run', s) for t, s in (
            ('disturbance, clear', 777), ('disturbance, dropout, clear', 0),
            ('disturbance, position, 0, 0, 0, 0', 0), ('', 0))]
        pubs = (checker.count_publishers('/daq/disturbance') +
                checker.count_publishers('/daq/object_status_disturbed'))
        gt_pubs = checker.count_publishers('/daq/object_status')
        ev.add('X8', 'ego_mode follow (real car): ON rejected, clears pass, no disturbance DAQ publishers',
               'REAL_CAR x3; clear (with seed) / kind clear / off value / status query ok; 0 publishers on '
               '/daq/disturbance and /daq/object_status_disturbed (GT /daq/object_status still has 1)',
               f"service ready {bool(ready)}; ON codes {on_codes}; passes "
               f'{[p.ok if p else None for p in passes]}; publishers {pubs} (GT object_status {gt_pubs}); '
               f'node alive {node2.alive()}',
               bool(ready) and on_codes == ['REAL_CAR'] * 3 and all(p is not None and p.ok for p in passes) and
               pubs == 0 and gt_pubs == 1 and node2.alive())
    finally:
        checker.stop_threads = True
        for n in (node, node2):
            if n is not None and n.alive():
                n.sigint(timeout=10.0)
        fake.stop()
        executor.shutdown()
        checker.destroy_node()
        rclpy.try_shutdown()

    # 노드가 남긴 외란 로그 줄을 증거에 같이 둔다
    try:
        with open(node.log_path, encoding='utf-8', errors='replace') as f:
            marks['node_log'] = [ln.strip() for ln in f if '외란' in ln][:80]
    except OSError:
        pass
    with open(os.path.join(args.out, 'evidence.json'), 'w') as f:
        json.dump({'rows': ev.rows, 'marks': marks}, f, indent=1, default=str, ensure_ascii=False)
    passed = sum(1 for r in ev.rows if r['result'] == 'PASS')
    print(f'==== {passed}/{len(ev.rows)} PASS — {args.out}/evidence.json', flush=True)
    return 0 if ev.ok() else 1


if __name__ == '__main__':
    sys.exit(main())
