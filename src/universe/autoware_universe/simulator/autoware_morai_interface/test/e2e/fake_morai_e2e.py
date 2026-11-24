#!/usr/bin/env python3
# 신규 — 26.09.25 _ YSH _ 가짜 MORAI(fake_morai_server)로 C++ 노드 종단 시험 — 수동 실행 전용 (colcon test 아님)
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
"""가짜 MORAI + autoware_morai_interface 노드 종단 시험.

run_fake_morai_e2e.sh 가 깨끗한 셸(ROS_DOMAIN_ID 91, localhost)로 이 스크립트를 부른다. 스크립트가
노드를 먼저(가짜 없이), 그다음 가짜 MORAI 를 띄우고 Autoware 쪽 역할(control_cmd 33 Hz, gear_cmd 10 Hz,
control_mode_request, /initialpose, /localization/initialize 서버)을 하면서 단계별로 판정한다.
실제 MORAI(7789)에는 절대 붙지 않는다. 결과는 --out 디렉토리에 evidence.json·노드/가짜 로그·이벤트 JSONL.
"""

import argparse
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid

from autoware_control_msgs.msg import Control
from autoware_localization_msgs.srv import InitializeLocalization
from autoware_perception_msgs.msg import TrackedObjects
from autoware_vehicle_msgs.msg import ControlModeReport
from autoware_vehicle_msgs.msg import GearCommand
from autoware_vehicle_msgs.msg import GearReport
from autoware_vehicle_msgs.msg import HazardLightsReport
from autoware_vehicle_msgs.msg import SteeringReport
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand
from autoware_vehicle_msgs.msg import TurnIndicatorsReport
from autoware_vehicle_msgs.msg import VelocityReport
from autoware_vehicle_msgs.srv import ControlModeCommand
from diagnostic_msgs.msg import DiagnosticArray
# 26.09.26 _ YSH _ DAQ 단계 (/daq/*, /daq/run_state 게이트)
# 26.10.05 _ YSH _ DAQ 명세 토픽이 MORAI 메시지로 바뀌어 안 쓴다. 원문: from geometry_msgs.msg import AccelWithCovarianceStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
# 26.10.05 _ YSH _ DAQ 명세 토픽 (/daq/ego_status·/daq/object_status)
from morai_ros2_msgs.msg import CollisionData  # 26.10.07 _ YSH _ /daq/collision
from morai_ros2_msgs.msg import CtrlCmd  # 26.10.06 _ YSH _ 명세 25 (gRPC 송신)
from morai_ros2_msgs.msg import EgoVehicleStatus
from morai_ros2_msgs.msg import ObjectStatusList
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

REAL_MORAI_PORT = 7789
NODE_NAME = 'morai_bridge'
TICK_US = 20000
WHEEL_BASE = 3.0
MAX_STEER_RAD = math.radians(40.0)
STEER_COMP_SPEEDS = [2.0, 5.0, 8.0, 11.0]
STEER_COMP_GAINS = [0.870, 0.855, 0.807, 0.796]
STALE_TIMEOUT_S = 0.2
STALE_ACCEL = -2.4


def now_ns():
    return time.time_ns()


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def steer_comp_gain(v):
    if v <= STEER_COMP_SPEEDS[0]:
        return STEER_COMP_GAINS[0]
    for i in range(1, len(STEER_COMP_SPEEDS)):
        if v <= STEER_COMP_SPEEDS[i]:
            r = (v - STEER_COMP_SPEEDS[i - 1]) / (STEER_COMP_SPEEDS[i] - STEER_COMP_SPEEDS[i - 1])
            return STEER_COMP_GAINS[i - 1] + r * (STEER_COMP_GAINS[i] - STEER_COMP_GAINS[i - 1])
    return STEER_COMP_GAINS[-1]


def slope(points):
    """최소제곱 기울기 (x, y) 목록."""
    n = len(points)
    if n < 2:
        return float('nan')
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    if sxx == 0.0:
        return float('nan')
    return sum((p[0] - mx) * (p[1] - my) for p in points) / sxx


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


# =============================================================================
# 프로세스
# =============================================================================


class Fake:
    """가짜 MORAI 프로세스 + 제어 포트."""

    def __init__(self, binary, port, out_dir, extra=()):
        if port == REAL_MORAI_PORT:
            raise SystemExit('refusing port 7789 (real MORAI)')
        self.binary = binary
        self.port = port
        self.ctl_port = port + 1
        self.events = os.path.join(out_dir, 'fake_events.jsonl')
        self.log_path = os.path.join(out_dir, 'fake.log')
        self.extra = list(extra)
        self.proc = None
        self.sock = None
        self.file = None
        self.lock = threading.Lock()

    def start(self):
        log = open(self.log_path, 'a')
        self.proc = subprocess.Popen(
            [self.binary, '--port', str(self.port), '--events', self.events] + self.extra,
            stdout=log, stderr=subprocess.STDOUT)
        deadline = time.time() + 10.0
        while time.time() < deadline:
            try:
                self.sock = socket.create_connection(('127.0.0.1', self.ctl_port), timeout=2.0)
                self.file = self.sock.makefile('rw')
                return
            except OSError:
                time.sleep(0.05)
        raise RuntimeError('fake control port did not open')

    def ctl(self, line):
        with self.lock:
            self.file.write(line + '\n')
            self.file.flush()
            return json.loads(self.file.readline())

    def stats(self):
        return self.ctl('stats')

    def stop(self):
        if self.proc is None:
            return
        try:
            self.ctl('quit')
        except Exception:
            pass
        try:
            self.proc.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.proc.wait()

    def read_events(self):
        self.stats()  # 로그를 flush 시킨다
        out = []
        with open(self.events) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass  # 쓰는 중인 마지막 줄
        return out


class NodeProc:
    """노드 실행 파일 (ros2 launch 없이 — PID 를 곧바로 재기 위해)."""

    def __init__(self, ws, port, out_dir, extra_params=()):
        self.exe = os.path.join(
            ws, 'install/autoware_morai_interface/lib/autoware_morai_interface/'
            'autoware_morai_interface_node')
        self.params = os.path.join(
            ws, 'install/autoware_morai_interface/share/autoware_morai_interface/config/'
            'morai_interface.gt.param.yaml')
        self.port = port
        self.log_path = os.path.join(out_dir, 'node.log')
        self.extra = list(extra_params)
        self.proc = None
        self.exit_code = None

    def start(self):
        # 26.09.27 _ YSH _ GT yaml 이 원래 컴퓨터용 time_source receive 가 됐다. 이 시험의 stamp 검사(stamp == 가짜 MORAI
        #   timestamp)는 morai 시각 경로를 검증하므로 코드 기본값인 morai 를 명시한다. 원문:
        # args = [self.exe, '--ros-args', '-r', f'__node:={NODE_NAME}', '--params-file', self.params,
        #         '-p', 'morai.address:=127.0.0.1', '-p', f'morai.port:={self.port}']
        args = [self.exe, '--ros-args', '-r', f'__node:={NODE_NAME}', '--params-file', self.params,
                '-p', 'morai.address:=127.0.0.1', '-p', f'morai.port:={self.port}', '-p', 'time_source:=morai']
        for p in self.extra:
            args += ['-p', p]
        log = open(self.log_path, 'a')
        self.proc = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT)

    def alive(self):
        return self.proc is not None and self.proc.poll() is None

    def cpu_ticks(self):
        with open(f'/proc/{self.proc.pid}/stat') as f:
            fields = f.read().rsplit(')', 1)[1].split()
        return int(fields[11]) + int(fields[12])  # utime + stime (clock ticks)

    def threads(self):
        with open(f'/proc/{self.proc.pid}/status') as f:
            for line in f:
                if line.startswith('Threads:'):
                    return int(line.split()[1])
        return -1

    def sigint(self, timeout=10.0):
        if not self.alive():
            return self.proc.returncode if self.proc else None
        self.proc.send_signal(signal.SIGINT)
        try:
            self.exit_code = self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            self.exit_code = 'killed'
        return self.exit_code


# =============================================================================
# ROS 쪽 (Autoware 역할 + 기록)
# =============================================================================


class Checker(Node):

    def __init__(self):
        super().__init__('fake_morai_e2e_checker')
        qos = QoSProfile(depth=100)
        self.lock = threading.Lock()
        # 26.09.26 _ YSH _ DAQ 토픽·런 요약도 기록한다. 원문:
        # self.rec = {k: [] for k in (
        #     'velocity', 'steering', 'gear', 'mode', 'turn', 'hazard', 'kinematic', 'tf', 'objects')}
        # 26.09.26 _ YSH _ 원시 gRPC 기록 (/daq/raw/*) 도 기록한다. 원문:
        # self.rec = {k: [] for k in (
        #     'velocity', 'steering', 'gear', 'mode', 'turn', 'hazard', 'kinematic', 'tf', 'objects',
        #     'daq_state', 'daq_accel', 'daq_objects', 'daq_cmd', 'daq_summary')}
        self.rec = {k: [] for k in (
            'velocity', 'steering', 'gear', 'mode', 'turn', 'hazard', 'kinematic', 'tf', 'objects',
            'daq_state', 'daq_accel', 'daq_objects', 'daq_cmd', 'daq_summary', 'raw_states', 'raw_cmds',
            'daq_ctrl', 'daq_collision')}   # 26.10.06 _ YSH _ daq_ctrl 추가 · 26.10.07 _ YSH _ daq_collision 추가
        self.diag = {}
        self.init_requests = []

        def add(key, item):
            with self.lock:
                self.rec[key].append(item)

        self.create_subscription(
            VelocityReport, '/vehicle/status/velocity_status',
            lambda m: add('velocity', (now_ns(), stamp_ns(m.header.stamp), m.longitudinal_velocity,
                                       m.heading_rate, m.header.frame_id)), qos)
        self.create_subscription(
            SteeringReport, '/vehicle/status/steering_status',
            lambda m: add('steering', (now_ns(), stamp_ns(m.stamp), m.steering_tire_angle)), qos)
        self.create_subscription(
            GearReport, '/vehicle/status/gear_status',
            lambda m: add('gear', (now_ns(), stamp_ns(m.stamp), m.report)), qos)
        self.create_subscription(
            ControlModeReport, '/vehicle/status/control_mode',
            lambda m: add('mode', (now_ns(), stamp_ns(m.stamp), m.mode)), qos)
        self.create_subscription(
            TurnIndicatorsReport, '/vehicle/status/turn_indicators_status',
            lambda m: add('turn', (now_ns(), stamp_ns(m.stamp), m.report)), qos)
        self.create_subscription(
            HazardLightsReport, '/vehicle/status/hazard_lights_status',
            lambda m: add('hazard', (now_ns(), stamp_ns(m.stamp), m.report)), qos)
        self.create_subscription(
            Odometry, '/localization/kinematic_state',
            lambda m: add('kinematic', (now_ns(), stamp_ns(m.header.stamp), m.pose.pose.position.x,
                                        m.pose.pose.position.y, m.pose.pose.position.z,
                                        yaw_of(m.pose.pose.orientation), m.twist.twist.linear.x,
                                        m.header.frame_id, m.child_frame_id)), qos)
        self.create_subscription(TFMessage, '/tf', self.on_tf, qos)
        self.create_subscription(
            TrackedObjects, '/perception/object_recognition/tracking/objects', self.on_objects, qos)
        self.create_subscription(DiagnosticArray, '/diagnostics', self.on_diag, qos)
        # 26.09.26 _ YSH _ DAQ (노드 publish_daq, daq.mode gated) — cue_server 역할로 /daq/run_state 를 낸다
        # 26.10.05 _ YSH _ 명세 토픽을 MORAI 메시지로 — daq_state 는 /daq/ego_status (key 이름은 그대로 둔다), daq_accel 은 없어졌다.
        #   원문: Odometry '/daq/vehicle_state' → daq_state (now, stamp, pose x, twist x, twist z),
        #         AccelWithCovarianceStamped '/daq/acceleration' → daq_accel (now, stamp, accel x, frame),
        #         TrackedObjects '/daq/track_objects' → daq_objects (now, stamp, len(objects))
        self.create_subscription(
            EgoVehicleStatus, '/daq/ego_status',
            lambda m: add('daq_state', (now_ns(), stamp_ns(m.header.stamp), m.position.x,
                                        m.velocity.x, m.velocity.z, m.header.frame_id)), qos)
        self.create_subscription(
            ObjectStatusList, '/daq/object_status',
            lambda m: add('daq_objects', (now_ns(), stamp_ns(m.header.stamp),
                                          len(m.npc_list) + len(m.pedestrian_list) + len(m.obstacle_list),
                                          m.num_of_npcs + m.num_of_pedestrian + m.num_of_obstacle)), qos)
        # 26.10.07 _ YSH _ /daq/collision (gRPC Ego collision_objects → CollisionData)
        self.create_subscription(
            CollisionData, '/daq/collision',
            lambda m: add('daq_collision', (now_ns(), stamp_ns(m.header.stamp),
                                            [(o.name, o.type, o.position.x) for o in m.collision_object])), qos)
        # 26.10.06 _ YSH _ 명세 25 = MORAI 에 보낸 gRPC 명령 (/daq/ctrl_cmd)
        self.create_subscription(
            CtrlCmd, '/daq/ctrl_cmd',
            lambda m: add('daq_ctrl', (now_ns(), stamp_ns(m.header.stamp), m.longl_cmd_type, m.velocity,
                                       m.acceleration)), qos)
        self.create_subscription(
            Control, '/daq/vehicle_cmd',
            lambda m: add('daq_cmd', (now_ns(), stamp_ns(m.stamp), m.longitudinal.acceleration)), qos)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, '/daq/run_summary',
            lambda m: add('daq_summary', (now_ns(), json.loads(m.data))), latched)
        self.pub_daq_run_state = self.create_publisher(String, '/daq/run_state', latched)
        # 26.09.26 _ YSH _ 원시 gRPC 기록 (노드 daq.raw — GT yaml 기본 true). data 는 JSON 한 개
        raw_qos = QoSProfile(depth=512, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            String, '/daq/raw/actor_states',
            lambda m: add('raw_states', (now_ns(), json.loads(m.data))), raw_qos)
        self.create_subscription(
            String, '/daq/raw/vehicle_ctrl_cmd',
            lambda m: add('raw_cmds', (now_ns(), json.loads(m.data))), raw_qos)

        self.pub_control = self.create_publisher(Control, '/control/command/control_cmd', 10)
        self.pub_gear = self.create_publisher(GearCommand, '/control/command/gear_cmd', 10)
        self.pub_turn = self.create_publisher(
            TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', 10)
        self.pub_initialpose = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_service(
            InitializeLocalization, '/localization/initialize', self.on_initialize)
        self.mode_client = self.create_client(ControlModeCommand, '/control/control_mode_request')
        self.param_client = self.create_client(SetParameters, f'/{NODE_NAME}/set_parameters')

        # control_cmd 33 Hz · gear_cmd 10 Hz 발행기 (Autoware 역할)
        self.cmd_lock = threading.Lock()
        self.cmd_on = False
        self.cmd_accel = 0.0
        self.cmd_steer = 0.0
        self.cmd_nan = False
        self.cmd_log = []  # (wall_ns, accel, steer, nan)
        self.gear_on = False
        self.gear_value = GearCommand.DRIVE
        self.turn_value = TurnIndicatorsCommand.DISABLE
        self.stop_threads = False
        self.threads = [threading.Thread(target=self.cmd_loop, daemon=True),
                        threading.Thread(target=self.slow_loop, daemon=True)]
        for t in self.threads:
            t.start()

    def on_tf(self, msg):
        for t in msg.transforms:
            if t.header.frame_id == 'map' and t.child_frame_id == 'base_link':
                with self.lock:
                    self.rec['tf'].append((now_ns(), stamp_ns(t.header.stamp),
                                           t.transform.translation.x, t.transform.translation.y))

    def on_objects(self, msg):
        objs = []
        for o in msg.objects:
            objs.append((bytes(o.object_id.uuid).hex(), o.classification[0].label,
                         o.kinematics.pose_with_covariance.pose.position.x,
                         o.kinematics.pose_with_covariance.pose.position.y,
                         o.shape.dimensions.x, o.shape.dimensions.y))
        with self.lock:
            self.rec['objects'].append((now_ns(), stamp_ns(msg.header.stamp), msg.header.frame_id,
                                        objs))

    def on_diag(self, msg):
        with self.lock:
            for st in msg.status:
                level = st.level[0] if isinstance(st.level, (bytes, bytearray)) else int(st.level)
                self.diag[st.name] = (now_ns(), level, st.message,
                                      {kv.key: kv.value for kv in st.values})

    def on_initialize(self, request, response):
        with self.lock:
            self.init_requests.append((now_ns(), request))
        response.status.success = True
        return response

    def cmd_loop(self):
        period = 1.0 / 33.0
        nxt = time.monotonic()
        while not self.stop_threads:
            nxt += period
            with self.cmd_lock:
                on, a, s, nan = self.cmd_on, self.cmd_accel, self.cmd_steer, self.cmd_nan
            if on:
                m = Control()
                m.stamp = self.get_clock().now().to_msg()
                m.longitudinal.acceleration = float('nan') if nan == 'accel' else a
                m.longitudinal.velocity = 5.0
                m.lateral.steering_tire_angle = float('nan') if nan == 'steer' else s
                self.pub_control.publish(m)
                self.cmd_log.append((now_ns(), a, s, nan))
            time.sleep(max(0.0, nxt - time.monotonic()))

    def slow_loop(self):
        while not self.stop_threads:
            if self.gear_on:
                g = GearCommand()
                g.stamp = self.get_clock().now().to_msg()
                g.command = self.gear_value
                self.pub_gear.publish(g)
                t = TurnIndicatorsCommand()
                t.stamp = g.stamp
                t.command = self.turn_value
                self.pub_turn.publish(t)
            time.sleep(0.1)

    def set_cmd(self, on=None, accel=None, steer=None, nan=None):
        with self.cmd_lock:
            if on is not None:
                self.cmd_on = on
            if accel is not None:
                self.cmd_accel = accel
            if steer is not None:
                self.cmd_steer = steer
            if nan is not None:
                self.cmd_nan = nan

    def set_daq(self, data):
        """26.09.26 _ YSH _ cue_server 역할: /daq/run_state (latched) 'active <run>' / 'idle <run>'"""
        msg = String()
        msg.data = data
        self.pub_daq_run_state.publish(msg)
        return now_ns()

    def last_cmd_publish_ns(self):
        return self.cmd_log[-1][0] if self.cmd_log else None

    def records(self, key, t0=None, t1=None):
        with self.lock:
            data = list(self.rec[key])
        return [r for r in data if (t0 is None or r[0] >= t0) and (t1 is None or r[0] <= t1)]

    def diag_of(self, task):
        with self.lock:
            return self.diag.get(f'{NODE_NAME}: {task}')

    def call(self, client, request, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        end = time.time() + timeout
        while not fut.done() and time.time() < end:
            time.sleep(0.01)
        return fut.result() if fut.done() else None

    def set_params(self, values):
        req = SetParameters.Request()
        for name, v in values.items():
            p = Parameter()
            p.name = name
            if isinstance(v, bool):
                p.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=v)
            # 26.09.26 _ YSH _ 리뷰 반영: 문자열 파라미터 (daq.mode)
            elif isinstance(v, str):
                p.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=v)
            else:
                p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(v))
            req.parameters.append(p)
        res = self.call(self.param_client, req)
        return res is not None and all(r.successful for r in res.results)


# =============================================================================
# 판정
# =============================================================================


class Evidence:

    def __init__(self):
        self.rows = []

    def add(self, check_id, what, expected, measured, ok):
        self.rows.append({'id': check_id, 'check': what, 'expected': expected,
                          'measured': measured, 'result': 'PASS' if ok else 'FAIL'})
        print(f"[{'PASS' if ok else 'FAIL'}] {check_id} {what}: {measured} (expected {expected})",
              flush=True)

    def ok(self):
        return all(r['result'] == 'PASS' for r in self.rows)


def rate(records, t0, t1):
    n = sum(1 for r in records if t0 <= r[0] <= t1)
    return n / ((t1 - t0) / 1e9)


def wait_until(pred, timeout, step=0.02):
    end = time.time() + timeout
    while time.time() < end:
        v = pred()
        if v:
            return v
        time.sleep(step)
    return None


def cv_events(events, t0=None, t1=None):
    return [e for e in events if e.get('rpc') == 'ControlVehicle' and
            (t0 is None or e['t_ns'] >= t0) and (t1 is None or e['t_ns'] <= t1)]


def update_events(events, t0=None, t1=None):
    return [e for e in events if e.get('ev') == 'update' and
            (t0 is None or e['t_ns'] >= t0) and (t1 is None or e['t_ns'] <= t1)]


# 26.09.25 _ YSH _ command_trigger=timer 판정용: 송신 간격 분포와 짧은 창 안 최대 송신 수 (몰아 보내기 검출)
def send_gaps_ms(events, t0, t1):
    ts = [e['t_ns'] for e in cv_events(events, t0, t1)]
    return sorted((b - a) / 1e6 for a, b in zip(ts, ts[1:]))


def max_in_window(events, t0, t1, window_ns):
    ts = [e['t_ns'] for e in cv_events(events, t0, t1)]
    best = 0
    j = 0
    for i, t in enumerate(ts):
        while ts[j] < t - window_ns:
            j += 1
        best = max(best, i - j + 1)
    return best


def pct(sorted_vals, p):
    if not sorted_vals:
        return float('nan')
    return sorted_vals[min(len(sorted_vals) - 1, int(len(sorted_vals) * p))]


def per_update_counts(events, t0, t1):
    """창 안의 업데이트마다 받은 ControlVehicle 수 (업데이트 u 와 u+1 사이)."""
    ups = [e['u'] for e in update_events(events, t0, t1)]
    if len(ups) < 3:
        return {}
    lo, hi = ups[0], ups[-1] - 1  # 마지막 업데이트 뒤 구간은 아직 안 끝났을 수 있다
    counts = {u: 0 for u in ups if lo <= u <= hi}
    for e in cv_events(events):
        if lo <= e['u'] <= hi and e['u'] in counts:
            counts[e['u']] += 1
    hist = {}
    for c in counts.values():
        k = str(min(c, 3)) if c < 3 else '3+'
        hist[k] = hist.get(k, 0) + 1
    return hist


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--ws', required=True, help='colcon workspace root (install/, build/)')
    ap.add_argument('--port', type=int, default=17871)
    ap.add_argument('--out', required=True)
    ap.add_argument('--drive-s', type=float, default=10.0)
    # 26.09.25 _ YSH _ A/B: 노드의 명령 트리거 (update = 업데이트당 1 회, timer = cmd_resend_hz 고정 주기, main.py 조건)
    ap.add_argument('--command-trigger', choices=('update', 'timer'), default='update')
    ap.add_argument('--cmd-resend-hz', type=float, default=100.0)
    args = ap.parse_args()
    timer = args.command_trigger == 'timer'
    hz = args.cmd_resend_hz
    period_ms = 1000.0 / hz
    if args.port == REAL_MORAI_PORT or args.port + 1 == REAL_MORAI_PORT:
        raise SystemExit('refusing port 7789 (real MORAI)')
    os.makedirs(args.out, exist_ok=True)
    fake_bin = os.path.join(args.ws, 'build/morai_grpc_vendor/fake_morai_server')
    if not os.path.exists(fake_bin):
        raise SystemExit(f'{fake_bin} not found — build morai_grpc_vendor with BUILD_TESTING')

    ev = Evidence()
    rclpy.init()
    checker = Checker()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(checker)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()

    fake = Fake(fake_bin, args.port, args.out, extra=['--lie-mode-result'])
    # 26.09.25 _ YSH _ 명령 트리거를 늘 명시해서 넘긴다 (param yaml 값보다 뒤라 -p 가 이긴다)
    # node = NodeProc(args.ws, args.port, args.out)
    node = NodeProc(args.ws, args.port, args.out, extra_params=(
        f'command_trigger:={args.command_trigger}', f'cmd_resend_hz:={hz}'))
    marks = {'command_trigger': args.command_trigger, 'cmd_resend_hz': hz}
    print(f'==== command_trigger={args.command_trigger} cmd_resend_hz={hz:g}', flush=True)
    try:
        # ---------------------------------------------------------------- (9) MORAI 없이 기동
        node.start()
        marks['node_start'] = now_ns()
        time.sleep(5.0)
        link = checker.diag_of('link')
        ev.add('9a', 'node alive 5 s without MORAI (no fake yet)', 'alive, link diag ERROR, no status',
               f"alive={node.alive()} link={link[1] if link else None} '{link[2] if link else ''}' "
               f"velocity_msgs={len(checker.records('velocity'))}",
               node.alive() and link is not None and link[1] == 2 and
               not checker.records('velocity'))

        fake.start()
        marks['fake_start'] = now_ns()
        first = wait_until(lambda: checker.records('velocity', marks['fake_start']), 15.0)
        dt = (first[0][0] - marks['fake_start']) / 1e9 if first else None
        ev.add('9b', 'fake started later -> node connects and publishes',
               'first velocity_status < 3 s after fake start',
               f'{dt:.3f} s' if dt is not None else 'never', dt is not None and dt < 3.0)

        # ---------------------------------------------------------------- (8) 측위 초기화
        got = wait_until(lambda: checker.init_requests, 5.0)
        time.sleep(2.5)  # 한 번만 부르는지 (성공 뒤 타이머 취소)
        st = fake.stats()
        if got:
            req = checker.init_requests[0][1]
            p = req.pose_with_covariance[0] if req.pose_with_covariance else None
            yaw = math.degrees(yaw_of(p.pose.pose.orientation)) if p else float('nan')
            ok = (len(checker.init_requests) == 1 and req.method == InitializeLocalization.Request.DIRECT
                  and p is not None and p.header.frame_id == 'map' and
                  abs(p.pose.pose.position.x - st['ego']['x']) < 1e-3 and
                  abs(p.pose.pose.position.y - st['ego']['y']) < 1e-3 and
                  abs(p.pose.pose.position.z - st['ego']['z']) < 1e-3 and
                  abs(yaw - st['ego']['yaw_deg']) < 1e-3)
            ev.add('8', '/localization/initialize called by node (checker serves it)',
                   'once, method DIRECT(1), frame map, pose == MORAI ego pose (origin 0)',
                   f'calls={len(checker.init_requests)} method={req.method} '
                   f'frame={p.header.frame_id if p else None} pose=({p.pose.pose.position.x:.3f}, '
                   f'{p.pose.pose.position.y:.3f}, {p.pose.pose.position.z:.3f}, yaw {yaw:.2f}) '
                   f"fake=({st['ego']['x']:.3f}, {st['ego']['y']:.3f}, {st['ego']['z']:.3f}, "
                   f"yaw {st['ego']['yaw_deg']:.2f})", ok)
        else:
            ev.add('8', '/localization/initialize called by node', 'called', 'never', False)

        # ---------------------------------------------------------------- engage (AUTO + D + 램프)
        req = ControlModeCommand.Request()
        req.mode = ControlModeCommand.Request.AUTONOMOUS
        t_req = now_ns()
        res = checker.call(checker.mode_client, req, timeout=5.0)
        st = fake.stats()
        rep = wait_until(lambda: [r for r in checker.records('mode', t_req) if r[2] == 1], 2.0)
        ev.add('E1', 'control_mode_request AUTONOMOUS (fake Result lies = failure)',
               'response success (readback), fake mode 3, ControlModeReport AUTONOMOUS(1)',
               f"success={res.success if res else None} fake_mode={st['mode']} "
               f"report_autonomous={'yes' if rep else 'no'}",
               res is not None and res.success and st['mode'] == 3 and bool(rep))
        checker.gear_on = True
        checker.turn_value = TurnIndicatorsCommand.ENABLE_LEFT
        g = wait_until(lambda: fake.stats()['gear'] == 4, 3.0)
        rep = wait_until(lambda: [r for r in checker.records('gear') if r[2] == GearReport.DRIVE], 2.0)
        lamp = wait_until(lambda: fake.stats()['tail'] == 1, 2.0)
        lamp_rep = wait_until(
            lambda: [r for r in checker.records('turn') if r[2] == TurnIndicatorsReport.ENABLE_LEFT],
            2.0)
        ev.add('E2', 'gear_cmd DRIVE 10 Hz + turn_indicators ENABLE_LEFT',
               'fake gear 4 (D) once, GearReport DRIVE, fake tail light LEFT(1), TurnIndicatorsReport LEFT',
               f"gear={fake.stats()['gear']} gear_sets={fake.stats()['rpc'].get('SetVehicleGear', {}).get('n')} "
               f"report_drive={'yes' if rep else 'no'} tail={fake.stats()['tail']} "
               f"turn_report_left={'yes' if lamp_rep else 'no'}",
               bool(g) and bool(rep) and bool(lamp) and bool(lamp_rep))

        # ---------------------------------------------------------------- (1)(2)(3)(10) 주행 창
        checker.set_cmd(on=True, accel=1.0, steer=0.05)
        time.sleep(1.0)  # 첫 명령이 들어가고 속도가 붙게
        cpu0 = node.cpu_ticks()
        t0 = now_ns()
        s0 = fake.stats()
        time.sleep(args.drive_s)
        t1 = now_ns()
        s1 = fake.stats()
        cpu1 = node.cpu_ticks()
        events = fake.read_events()
        marks['drive'] = (t0, t1)

        n_upd = s1['updates'] - s0['updates']
        n_cv = s1['rpc']['ControlVehicle']['n'] - s0['rpc']['ControlVehicle']['n']
        hist = {k: s1['hist'][k] - s0['hist'][k] for k in s1['hist']}
        hist_ev = per_update_counts(events, t0, t1)
        up_t = {e['u']: e['t_ns'] for e in update_events(events)}
        lags = sorted((e['t_ns'] - up_t[e['u']]) / 1e6 for e in cv_events(events, t0, t1) if e['u'] in up_t)
        lag = (lags[len(lags) // 2], lags[int(len(lags) * 0.99)], lags[-1]) if lags else (0, 0, 0)
        # 26.09.25 _ YSH _ 트리거별 판정. update 는 원래대로(업데이트당 정확히 1), timer 는 업데이트와 무관한 고정 주기
        #   (±3 %), 간격 중앙값 = 주기(±1 ms), 5 주기 창 안 최대 6 개(늦은 틱 뒤 1 개만 — 몰아 보내기 없음)
        drive_s = (t1 - t0) / 1e9
        gaps = send_gaps_ms(events, t0, t1)
        burst = max_in_window(events, t0, t1, int(5 * period_ms * 1e6))
        marks['ab'] = {'trigger': args.command_trigger, 'target_hz': hz if timer else None,
                       'updates_per_s': n_upd / drive_s, 'cv_per_s': n_cv / drive_s,
                       'cv_per_update': n_cv / max(n_upd, 1), 'hist_stats': hist, 'hist_events': hist_ev,
                       'gap_ms_p50': pct(gaps, 0.5), 'gap_ms_p99': pct(gaps, 0.99),
                       'gap_ms_max': gaps[-1] if gaps else None, 'gap_ms_min': gaps[0] if gaps else None,
                       'max_in_5_periods': burst}
        if not timer:
            ev.add('1', 'ControlVehicle per MORAI update (fake counts between updates)',
                   '1 per update, never 2+, each lands < 20 ms after its update (before the next)',
                   f'updates={n_upd} ControlVehicle={n_cv} ratio={n_cv / max(n_upd, 1):.3f} '
                   f'hist(stats)={hist} hist(events)={hist_ev}; update->ControlVehicle p50 {lag[0]:.2f} '
                   f'p99 {lag[1]:.2f} max {lag[2]:.2f} ms',
                   abs(n_cv / max(n_upd, 1) - 1.0) < 0.02 and hist.get('2', 0) == 0 and
                   hist.get('3+', 0) == 0 and hist_ev.get('2', 0) == 0 and hist_ev.get('3+', 0) == 0 and
                   bool(lags) and lag[2] < 20.0)
        else:
            ev.add('1', f'ControlVehicle at a fixed {hz:g} Hz, independent of MORAI updates (timer)',
                   f'{hz:g}/s +-3%, {hz / max(n_upd / drive_s, 1e-9):.2f} per update, gap p50 '
                   f'{period_ms:.1f} ms +-1, <= 6 sends in any {5 * period_ms:.0f} ms (no catch-up burst)',
                   f'updates={n_upd} ({n_upd / drive_s:.2f}/s) ControlVehicle={n_cv} ({n_cv / drive_s:.2f}/s) '
                   f'per update {n_cv / max(n_upd, 1):.3f} hist(stats)={hist} hist(events)={hist_ev}; gap ms '
                   f'min {gaps[0] if gaps else float("nan"):.2f} p50 {pct(gaps, 0.5):.2f} '
                   f'p99 {pct(gaps, 0.99):.2f} max {gaps[-1] if gaps else float("nan"):.2f}; '
                   f'max in {5 * period_ms:.0f} ms {burst}',
                   abs(n_cv / drive_s - hz) <= 0.03 * hz and abs(pct(gaps, 0.5) - period_ms) <= 1.0 and
                   burst <= 6)

        ups = update_events(events, t0, t1)
        a_fake = slope([(e['t_ns'] / 1e9, e['v']) for e in ups])
        vel = checker.records('velocity', t0, t1)
        a_node = slope([(r[1] / 1e9, r[2]) for r in vel])
        cvs = cv_events(events, t0, t1)
        accel_ok = all(abs(e['accel'] - 1.0) < 1e-9 and e['type'] == 3 for e in cvs)
        # 조향: steer/gain(v) (명령을 처음 만들 때 속도), 리포트 = 적용 조향 × gain = 0.05
        steers = [e['steer'] for e in cvs]
        steer_lo, steer_hi = 0.05 / STEER_COMP_GAINS[0], 0.05 / STEER_COMP_GAINS[-1]
        steer_ok = all(steer_lo - 1e-9 <= s <= steer_hi + 1e-9 for s in steers)
        ev.add('2a', 'control_cmd 33 Hz accel 1.0 -> fake Ego accelerates per command',
               'fake dv/dt 1.00, velocity_status slope 1.00, every ControlVehicle ACCELERATION(3) a=1.0',
               f'fake {a_fake:.3f} m/s^2, velocity_status {a_node:.3f} m/s^2 (n={len(vel)}), '
               f'cmds a=1.0/type3 all={accel_ok}, v {ups[0]["v"]:.2f}->{ups[-1]["v"]:.2f} m/s',
               abs(a_fake - 1.0) < 0.02 and abs(a_node - 1.0) < 0.02 and accel_ok)
        steer_rep = checker.records('steering', t0, t1)
        rep_err = max(abs(r[2] - 0.05) for r in steer_rep) if steer_rep else float('nan')
        hr = [(r[1], r[3]) for r in vel]
        ups_by_ts = {e['ts']: e for e in update_events(events)}
        hr_err = [abs(h - (ups_by_ts[s // 1000]['v'] * math.tan(ups_by_ts[s // 1000]['steer']) /
                           WHEEL_BASE)) for s, h in hr if s // 1000 in ups_by_ts]
        ev.add('2b', 'steering: command steer/gain(v) rad, report = steer (sign kept), heading_rate',
               f'cmd steer in [{steer_lo:.4f}, {steer_hi:.4f}], |report-0.05|<1e-3, '
               'heading_rate == v*tan(steer)/L (>0 left)',
               f'cmd steer {min(steers):.5f}..{max(steers):.5f}, report max err {rep_err:.2e}, '
               f'heading_rate max err {max(hr_err) if hr_err else float("nan"):.2e} '
               f'(n={len(hr_err)}), last heading_rate {hr[-1][1] if hr else float("nan"):.4f}',
               steer_ok and rep_err < 1e-3 and hr_err and max(hr_err) < 1e-4 and hr[-1][1] > 0)

        rates = {k: rate(checker.records(k), t0, t1) for k in
                 ('velocity', 'steering', 'kinematic', 'tf', 'gear', 'mode', 'objects', 'turn',
                  'hazard')}
        upd_hz = n_upd / ((t1 - t0) / 1e9)
        # 26.09.26 _ YSH _ CARLA·simple_planning_simulator 방식: 차량 리포트 전부(gear·mode·turn·hazard 포함) 매 업데이트,
        #   objects 는 MORAI 시각 100 ms 스케줄(10 Hz). 원문:
        # expect = {'velocity': upd_hz, 'steering': upd_hz, 'kinematic': upd_hz, 'tf': upd_hz,
        #           'gear': upd_hz / 2, 'mode': upd_hz / 2, 'objects': upd_hz / 5, 'turn': 10.0,
        #           'hazard': 10.0}
        expect = {'velocity': upd_hz, 'steering': upd_hz, 'kinematic': upd_hz, 'tf': upd_hz,
                  'gear': upd_hz, 'mode': upd_hz, 'objects': 10.0, 'turn': upd_hz,
                  'hazard': upd_hz}
        rate_ok = all(abs(rates[k] - expect[k]) <= max(0.6, 0.03 * expect[k]) for k in rates)
        # 26.09.26 _ YSH _ 한 업데이트의 리포트는 모두 같은 stamp (fake timestamp) — 램프도 이제 now() 가 아니다
        lamp_st = {r[1] for r in checker.records('turn', t0, t1)} | \
            {r[1] for r in checker.records('hazard', t0, t1)} | \
            {r[1] for r in checker.records('gear', t0, t1)} | {r[1] for r in checker.records('mode', t0, t1)}
        lamp_ok = bool(lamp_st) and all(s % 1000 == 0 and s // 1000 in ups_by_ts for s in lamp_st)
        ev.add('3a', 'publish rates (fake 50 upd/s minus 1 skipped tick per 97)',
               'velocity/steering/kinematic/tf/gear/mode/turn/hazard = update rate, one stamp per update; '
               'objects 10 Hz (MORAI time)',
               'update ' + f'{upd_hz:.2f} Hz; ' + ', '.join(f'{k} {v:.2f}' for k, v in rates.items()) +
               f'; gear/mode/turn/hazard stamps == fake ts: {lamp_ok}',
               rate_ok and lamp_ok)

        ts_set = set(ups_by_ts)
        vel_st = {r[1] for r in vel}
        kin = checker.records('kinematic', t0, t1)
        tf = checker.records('tf', t0, t1)
        kin_st = {r[1] for r in kin}
        tf_st = {r[1] for r in tf}
        st_st = {r[1] for r in steer_rep}
        in_fake = all((s % 1000 == 0 and s // 1000 in ts_set) for s in vel_st | kin_st | tf_st)
        pose_err = max((math.hypot(r[2] - ups_by_ts[r[1] // 1000]['x'],
                                   r[3] - ups_by_ts[r[1] // 1000]['y'])
                        for r in kin if r[1] // 1000 in ups_by_ts), default=float('nan'))
        tf_pose_err = max((math.hypot(t[2] - k[2], t[3] - k[3]) for t in tf for k in kin
                           if t[1] == k[1]), default=float('nan'))
        frames_ok = all(r[7] == 'map' and r[8] == 'base_link' for r in kin) and \
            all(r[4] == 'base_link' for r in vel)
        # 창 가장자리 한두 개는 짝이 창 밖에 있을 수 있다
        common = vel_st & kin_st & tf_st & st_st
        age = sorted((r[0] - r[1]) / 1e6 for r in vel)
        marks['recv_minus_stamp_ms'] = (age[len(age) // 2], age[-1]) if age else None
        ev.add('3b', 'stamps: velocity/steering/kinematic_state/TF identical and == fake ActorStates.timestamp',
               'same stamp set, every stamp is a fake update timestamp (us), kinematic pose == fake pose '
               'at that timestamp, TF == kinematic_state',
               f'vel {len(vel_st)} steer {len(st_st)} kin {len(kin_st)} tf {len(tf_st)} common {len(common)}; '
               f'all in fake ts set={in_fake}; kinematic pose err {pose_err:.2e} m; TF-kin err '
               f'{tf_pose_err:.2e}; frames ok={frames_ok}; duplicates vel={len(vel) - len(vel_st)}; '
               f"checker recv - stamp p50 {marks['recv_minus_stamp_ms'][0]:.2f} max "
               f"{marks['recv_minus_stamp_ms'][1]:.2f} ms",
               in_fake and len(common) >= min(len(vel_st), len(kin_st), len(tf_st)) - 2 and
               pose_err < 1e-6 and tf_pose_err < 1e-9 and frames_ok and len(vel) == len(vel_st))

        objs = checker.records('objects', t0, t1)
        want = {uuid.uuid5(uuid.NAMESPACE_OID, 'morai:NPC_Car_1').bytes.hex(): 1,
                uuid.uuid5(uuid.NAMESPACE_OID, 'morai:NPC_Ped_1').bytes.hex(): 7}
        obj_ok = bool(objs) and all(
            len(o[3]) == 2 and {x[0]: x[1] for x in o[3]} == want and o[2] == 'map' and
            o[1] // 1000 in ts_set for o in objs)
        ev.add('3c', 'TrackedObjects: 2 NPCs, stable uuid5("morai:<id>"), labels, stamp == fake ts',
               'every msg: {uuid5(NPC_Car_1): CAR(1), uuid5(NPC_Ped_1): PEDESTRIAN(7)}, frame map, '
               'Ego excluded',
               f'msgs {len(objs)}, counts {sorted({len(o[3]) for o in objs})}, uuids stable={obj_ok}, '
               f"car dims {objs[-1][3][0][4:] if objs else None}", obj_ok)

        cpu_pct = (cpu1 - cpu0) / os.sysconf('SC_CLK_TCK') / ((t1 - t0) / 1e9) * 100.0
        ev.add('10', 'node CPU at ~50 updates/s while driving (/proc utime+stime)',
               'measured (informational; < 10% of one core)',
               f'{cpu_pct:.1f}% of one core over {(t1 - t0) / 1e9:.1f} s, threads {node.threads()}',
               cpu_pct < 10.0)
        marks['cpu_pct'] = cpu_pct

        # 26.09.25 _ YSH _ A/B: command 진단이 트리거·업데이트당 송신·간격·지연재설정을 싣는지 (1 s 창이라 여유 0.15)
        diag_cmd = checker.diag_of('command')
        dv = diag_cmd[3] if diag_cmd else {}
        keys = ('command_trigger', 'cmd_resend_hz', 'sends_per_s', 'sends_per_update', 'send_gap_p50_ms',
                'send_gap_p99_ms', 'send_gap_max_ms', 'cmd_timer_late_window', 'cmd_timer_late_total')
        marks['ab']['cpu_pct'] = cpu_pct
        marks['ab']['diag_command'] = {k: dv.get(k) for k in keys}
        want_ratio = n_cv / max(n_upd, 1)
        try:
            spu = float(dv.get('sends_per_update', 'nan'))
        except ValueError:
            spu = float('nan')
        ev.add('AB', 'command diagnostic carries the trigger and the timer stats',
               f'command_trigger={args.command_trigger}, sends_per_update ~{want_ratio:.2f} (+-0.15), '
               'gap/late fields present',
               f"summary '{diag_cmd[2] if diag_cmd else ''}'; " +
               ', '.join(f'{k}={dv.get(k)}' for k in keys),
               dv.get('command_trigger') == args.command_trigger and abs(spu - want_ratio) <= 0.15 and
               all(k in dv for k in keys))

        # ---------------------------------------------------------------- (2c) NaN 조향
        t_nan0 = now_ns()
        checker.set_cmd(nan='steer')
        time.sleep(0.15)
        checker.set_cmd(nan=False)
        t_nan1 = now_ns()
        time.sleep(0.3)
        events = fake.read_events()
        # 창: 첫 NaN 발행 ~ NaN 뒤 첫 유효 발행. 직전 유효 메시지(30 ms 전 발행)는 첫 NaN 전에 이미 만들어졌다
        nan_pubs = [c[0] for c in checker.cmd_log if c[0] >= t_nan0 and c[3] == 'steer']
        valid_after = [c[0] for c in checker.cmd_log if nan_pubs and c[0] > nan_pubs[-1] and not c[3]]
        w0 = nan_pubs[0] if nan_pubs else t_nan0
        w1 = valid_after[0] if valid_after else t_nan1
        before = cv_events(events, w0 - 200_000_000, w0)
        during = cv_events(events, w0, w1)
        n_nan_pub = len(nan_pubs)
        last_valid = before[-1]['steer'] if before else None
        nan_ok = (fake.stats()['nonfinite_cmds'] == 0 and during and
                  all(e['finite'] == 1 and e['steer'] == last_valid and e['accel'] == 1.0
                      for e in during))
        diag_cmd = checker.diag_of('command')
        ev.add('2c', 'NaN steering_tire_angle for 150 ms (< stale timeout)',
               'fake receives no non-finite value; ControlVehicle keeps last valid steer & accel',
               f'NaN msgs published {n_nan_pub}; ControlVehicle during {len(during)}, steer values '
               f'{sorted({e["steer"] for e in during})} (last valid {last_valid}); fake nonfinite '
               f"{fake.stats()['nonfinite_cmds']}; diag nan_commands_total "
               f"{diag_cmd[3].get('nan_commands_total') if diag_cmd else '?'}",
               bool(nan_ok))

        # ---------------------------------------------------------------- (2d) stale 감속
        checker.set_cmd(on=False)
        time.sleep(0.05)
        t_last = checker.last_cmd_publish_ns()
        time.sleep(1.5)
        events = fake.read_events()
        after = cv_events(events, t_last)
        first_stale = next((e for e in after if e['accel'] == STALE_ACCEL), None)
        last_normal = [e for e in cv_events(events, t_last - 500_000_000)
                       if first_stale and e['t_ns'] < first_stale['t_ns'] and e['accel'] != STALE_ACCEL]
        non_stale_after = [e for e in after if first_stale and e['t_ns'] > first_stale['t_ns'] and
                           e['accel'] != STALE_ACCEL]
        lat = (first_stale['t_ns'] - t_last) / 1e9 if first_stale else None
        # "0.2 s + 1 업데이트": 마지막 control_cmd + 0.2 s 뒤 첫 업데이트(수신 지연 몫으로 그다음 하나까지)에 감속
        u_due = next((e['u'] for e in update_events(events, t_last + int(STALE_TIMEOUT_S * 1e9))), None)
        ups_after = update_events(events, t_last + 400_000_000, t_last + 1_400_000_000)
        decel = slope([(e['t_ns'] / 1e9, e['v']) for e in ups_after if e['v'] > 0.05])
        steer_kept = bool(last_normal) and first_stale is not None and \
            first_stale['steer'] == last_normal[-1]['steer']
        # 26.09.25 _ YSH _ timer 면 "0.2 s 뒤 첫 업데이트" 가 아니라 "0.2 s 뒤 첫 타이머 틱" (수신 지연 몫 20 ms 여유)
        if timer:
            due_ok = lat is not None and lat <= STALE_TIMEOUT_S + period_ms / 1000.0 + 0.02
            due_text = f'first timer tick >= 0.2 s (<= 0.2 + {period_ms:.0f} ms + 20 ms)'
        else:
            due_ok = u_due is not None and first_stale is not None and first_stale['u'] in (u_due, u_due + 1)
            due_text = 'first MORAI update >= 0.2 s after the last control_cmd (+1 update for receive delay)'
        marks['stale_latency_s'] = lat
        ev.add('2d', 'stop control_cmd -> stale brake (accel -2.4, last steer kept)',
               f'first -2.4 command on the {due_text}; no normal command after it; fake decel -2.4',
               f"latency {lat:.4f} s (update u={first_stale['u']}, due u={u_due}); steer kept="
               f'{steer_kept} ({first_stale["steer"]:.5f}); normal-after={len(non_stale_after)}; '
               f'fake dv/dt {decel:.2f} m/s^2' if first_stale else 'no stale command',
               first_stale is not None and lat >= STALE_TIMEOUT_S and due_ok and not non_stale_after and
               steer_kept and (math.isnan(decel) or abs(decel - STALE_ACCEL) < 0.1))

        # ---------------------------------------------------------------- (4) 1 s 멈춤
        checker.set_cmd(on=True, accel=0.5, steer=0.02)
        time.sleep(2.0)
        r = fake.ctl('freeze 1.0')
        t_freeze = r['t_ns']
        time.sleep(2.5)
        events = fake.read_events()
        resume = next((e for e in events if e.get('ev') == 'resume' and e['t_ns'] > t_freeze), None)
        vel = checker.records('velocity', t_freeze - 500_000_000, t_freeze + 2_500_000_000)
        before = [v for v in vel if v[0] <= t_freeze + 60_000_000]
        after = [v for v in vel if resume and v[0] >= resume['t_ns']]
        gap_msgs = [v for v in vel if t_freeze + 100_000_000 < v[0] < (resume['t_ns'] if resume else 0)]
        cv_gap = cv_events(events, t_freeze + 30_000_000, resume['t_ns'] if resume else None)
        stop_lat = (before[-1][0] - t_freeze) / 1e9 if before else None
        resume_lat = (after[0][0] - resume['t_ns']) / 1e9 if (after and resume) else None
        link = checker.diag_of('link')
        ups_after_resume = update_events(events, resume['t_ns'] if resume else 0,
                                         (resume['t_ns'] if resume else 0) + 1_000_000_000)
        post = cv_events(events, resume['t_ns'] if resume else 0,
                         (resume['t_ns'] if resume else 0) + 1_000_000_000)
        # 26.09.25 _ YSH _ timer 는 main.py 처럼 MORAI 가 멈춰도(링크는 살아 있음) 최신 명령을 주기대로 계속 보낸다
        if timer and resume:
            want_cv = hz * (resume['t_ns'] - t_freeze - 30_000_000) / 1e9
            cv_gap_ok = abs(len(cv_gap) - want_cv) <= max(3.0, 0.05 * want_cv)
            cv_gap_text = f'ControlVehicle keeps flowing at {hz:g} Hz while frozen (~{want_cv:.0f}, main.py)'
        else:
            cv_gap_ok = not cv_gap
            cv_gap_text = 'no ControlVehicle while frozen, 1/update after'
        ev.add('4', 'freeze fake 1 s -> publishing stops, resumes after',
               'last msg <= freeze + max_sample_age_s (0.1 s), none during, first msg after resume '
               f'<= 0.1 s, {cv_gap_text}',
               f'last msg at freeze+{stop_lat:.3f} s, msgs during {len(gap_msgs)}, first after resume '
               f'+{resume_lat:.3f} s, ControlVehicle during {len(cv_gap)}, after: {len(post)} per '
               f'{len(ups_after_resume)} updates; trigger now '
               f"{link[3].get('trigger') if link else '?'} (wft failures "
               f"{link[3].get('wait_for_tick_failures') if link else '?'})"
               if (stop_lat is not None and resume_lat is not None) else f'resume={resume}',
               stop_lat is not None and resume_lat is not None and stop_lat <= 0.1 and
               not gap_msgs and resume_lat <= 0.1 and cv_gap_ok)
        marks['freeze_trigger'] = link[3].get('trigger') if link else None

        # ---------------------------------------------------------------- (5) 연결 끊김 + 모드 리셋
        time.sleep(1.0)
        link0 = checker.diag_of('link')
        r = fake.ctl('drop 2.0')
        t_drop = r['t_ns']
        fake.ctl('mode 1')  # MORAI 재기동처럼 KEYBOARD 로 — 재연결 뒤 AUTO 재지정(auto_reassert_auto) 확인
        time.sleep(2.2)
        events = fake.read_events()
        serving = next((e for e in events if e.get('ev') == 'serving' and e['t_ns'] > t_drop), None)
        resumed = wait_until(lambda: checker.records(
            'velocity', serving['t_ns'] if serving else now_ns()), 10.0)
        time.sleep(2.0)
        events = fake.read_events()
        st = fake.stats()
        vel = checker.records('velocity', t_drop - 500_000_000)
        before = [v for v in vel if v[0] <= t_drop + 100_000_000]
        res_lat = (resumed[0][0] - serving['t_ns']) / 1e9 if (resumed and serving) else None
        reassert = [e for e in events if e.get('rpc') == 'SetVehicleControlMode' and e['t_ns'] > t_drop]
        time.sleep(1.2)
        link1 = checker.diag_of('link')
        ev.add('5', 'drop connection 2 s (server closed, MORAI mode reset to KEYBOARD) -> reconnect',
               'publishing stops, resumes < 2 s after server reopens, reconnects +1, AUTO re-asserted',
               f"last msg at drop+{(before[-1][0] - t_drop) / 1e9:.3f} s; resume +{res_lat:.3f} s after "
               f"reopen; reconnects {link0[3].get('reconnects') if link0 else '?'}->"
               f"{link1[3].get('reconnects') if link1 else '?'}; SetVehicleControlMode after drop "
               f"{[e['mode'] for e in reassert]}; fake mode {st['mode']}"
               if (before and res_lat is not None) else f'serving={serving} resumed={bool(resumed)}',
               res_lat is not None and res_lat < 2.0 and st['mode'] == 3 and
               any(e['mode'] == 3 for e in reassert) and link1 is not None and link0 is not None and
               int(link1[3].get('reconnects', 0)) == int(link0[3].get('reconnects', 0)) + 1)

        # ---------------------------------------------------------------- (6) 전 필드 0 Ego
        time.sleep(1.0)
        spec0 = fake.stats()['rpc']['GetVehicleSpec']['n']
        r = fake.ctl('zero 1.5')
        t_zero = r['t_ns']
        time.sleep(2.2)
        events = fake.read_events()
        zero_ups = [e for e in update_events(events, t_zero) if e['zero'] == 1]
        z0, z1 = zero_ups[0]['t_ns'], zero_ups[-1]['t_ns']
        zero_ts = {e['ts'] for e in zero_ups}
        pub_zero = [r2 for k in ('velocity', 'kinematic', 'tf', 'steering')
                    for r2 in checker.records(k) if r2[1] // 1000 in zero_ts]
        obj_zero = [o for o in checker.records('objects') if o[1] // 1000 in zero_ts]
        all_zero_kin = [k for k in checker.records('kinematic') if k[2] == 0.0 and k[3] == 0.0]
        cv_zero = cv_events(events, z0 + 30_000_000, z1)
        back = [v for v in checker.records('velocity', z1)]
        back_lat = (back[0][0] - z1) / 1e9 if back else None
        spec1 = fake.stats()['rpc']['GetVehicleSpec']['n']
        # 26.09.25 _ YSH _ timer 는 Ego 무효 첫 1 s 동안 최신 명령을 계속 보내고(main.py 와 같음), 1 s 넘으면 스펙 재조회까지 보류
        cv_hold = cv_events(events, z0 + 1_100_000_000, z1)
        if timer:
            cv_zero_ok = not cv_hold
            cv_zero_text = 'ControlVehicle only in the first ~1 s (timer keeps the latest), none after (spec hold)'
        else:
            cv_zero_ok = not cv_zero
            cv_zero_text = 'no ControlVehicle meanwhile'
        ev.add('6', 'all-zero Ego for 1.5 s -> nothing published for those samples',
               f'no status/GT msg stamped with a zero-ego timestamp, no (0,0) pose ever, {cv_zero_text}, '
               'resumes after, spec re-read once (Ego gone >= 1 s)',
               f'zero updates {len(zero_ups)}; msgs with those stamps {len(pub_zero)} (+objects '
               f'{len(obj_zero)}); (0,0) kinematic ever {len(all_zero_kin)}; ControlVehicle during '
               f'{len(cv_zero)} (after 1.1 s {len(cv_hold)}); first velocity after {back_lat:.3f} s; '
               f'GetVehicleSpec +{spec1 - spec0}'
               if back_lat is not None else 'no resume',
               len(zero_ups) > 50 and not pub_zero and not obj_zero and not all_zero_kin and
               cv_zero_ok and back_lat is not None and back_lat < 0.1 and spec1 - spec0 == 1)

        # ---------------------------------------------------------------- (7) /initialpose 텔레포트
        checker.set_cmd(on=False)
        time.sleep(0.5)
        origin = {'map_origin_offset_x': 100.0, 'map_origin_offset_y': -50.0,
                  'map_origin_offset_z': 0.0, 'map_origin_yaw_offset_deg': 30.0}
        params_ok = checker.set_params(origin)
        z_now = fake.stats()['ego']['z']
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = checker.get_clock().now().to_msg()
        pose.pose.pose.position.x = 250.0
        pose.pose.pose.position.y = 80.0
        pose.pose.pose.position.z = 0.0
        q = quat_from_yaw(math.radians(45.0))
        (pose.pose.pose.orientation.x, pose.pose.pose.orientation.y,
         pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) = q
        t_pose = now_ns()
        checker.pub_initialpose.publish(pose)
        wait_until(lambda: fake.stats()['transforms'] >= 1, 3.0)
        time.sleep(0.5)
        events = fake.read_events()
        tfe = next((e for e in events if e.get('rpc') == 'SetTransform' and e['t_ns'] > t_pose), None)
        yo = math.radians(origin['map_origin_yaw_offset_deg'])
        dx, dy = 250.0 - origin['map_origin_offset_x'], 80.0 - origin['map_origin_offset_y']
        want_x = math.cos(-yo) * dx - math.sin(-yo) * dy
        want_y = math.sin(-yo) * dx + math.cos(-yo) * dy
        want_yaw = 45.0 - origin['map_origin_yaw_offset_deg']
        kin = checker.records('kinematic', t_pose + 100_000_000)
        k_last = kin[-1] if kin else None
        tf_ok = (tfe is not None and abs(tfe['x'] - want_x) < 1e-6 and abs(tfe['y'] - want_y) < 1e-6 and
                 abs(tfe['yaw'] - want_yaw) < 1e-6 and abs(tfe['z'] - z_now) < 1e-6 and
                 tfe['roll'] == 0.0 and tfe['pitch'] == 0.0 and tfe['id'] == 'Ego')
        back_ok = (k_last is not None and abs(k_last[2] - 250.0) < 0.05 and
                   abs(k_last[3] - 80.0) < 0.05 and abs(math.degrees(k_last[5]) - 45.0) < 0.05)
        ev.add('7', '/initialpose (map 250,80,45deg) with map origin (+100,-50, 30deg)',
               f'SetTransform MORAI ({want_x:.4f}, {want_y:.4f}, yaw {want_yaw:.1f}, z=current '
               f'{z_now:.2f}); kinematic_state back at map (250, 80, 45deg)',
               f"params set={params_ok}; SetTransform ({tfe['x']:.4f}, {tfe['y']:.4f}, z {tfe['z']:.2f}, "
               f"yaw {tfe['yaw']:.4f}, roll/pitch {tfe['roll']}/{tfe['pitch']}, id {tfe['id']}); "
               f'kinematic after ({k_last[2]:.3f}, {k_last[3]:.3f}, {math.degrees(k_last[5]):.3f} deg)'
               if (tfe and k_last) else f'SetTransform={tfe} kinematic={k_last}',
               params_ok and tf_ok and back_ok)
        checker.set_params({k: 0.0 for k in origin})

        # ---------------------------------------------------------------- (F) forward_control_to_sim
        checker.set_cmd(on=True, accel=0.3, steer=0.0)
        time.sleep(1.0)
        off_ok = checker.set_params({'forward_control_to_sim': False})
        t_off = now_ns()
        time.sleep(1.2)
        t_on = now_ns()
        on_ok = checker.set_params({'forward_control_to_sim': True})
        time.sleep(1.0)
        events = fake.read_events()
        cv_off = cv_events(events, t_off + 50_000_000, t_on)
        cv_on = cv_events(events, t_on + 100_000_000, t_on + 900_000_000)
        ups_on = update_events(events, t_on + 100_000_000, t_on + 900_000_000)
        vel_off = checker.records('velocity', t_off + 50_000_000, t_on)
        # 26.09.25 _ YSH _ timer 면 다시 켠 뒤 0.8 s 창에 ~hz × 0.8 개
        if timer:
            want_on = hz * 0.8
            on_ok_count = abs(len(cv_on) - want_on) <= max(3.0, 0.05 * want_on)
            on_text = f'~{want_on:.0f} ({hz:g} Hz) again after true'
        else:
            on_ok_count = abs(len(cv_on) - len(ups_on)) <= 1
            on_text = '1 per update again after true'
        ev.add('F', 'forward_control_to_sim false -> true at runtime (control_cmd keeps flowing)',
               f'no ControlVehicle while false (status still published); {on_text}',
               f'set {off_ok}/{on_ok}; ControlVehicle while off {len(cv_off)} (velocity msgs '
               f'{len(vel_off)}); after on {len(cv_on)} per {len(ups_on)} updates',
               off_ok and on_ok and not cv_off and len(vel_off) > 40 and on_ok_count)

        # ---------------------------------------------------------------- (D) 26.09.26 _ YSH _ DAQ
        #   노드가 /daq/* 를 cue_server 녹화 구간(/daq/run_state active)에만 100 Hz 로 낸다. 켜진 채 노드가
        #   재시작돼도 latched run_state 를 다시 받아 곧바로 이어 낸다
        checker.set_cmd(on=True, accel=0.2, steer=0.0)
        t_q = now_ns()
        time.sleep(0.6)
        # 26.09.26 _ YSH _ 원시 기록 토픽도 게이트를 따른다. 원문:
        # quiet = sum(len(checker.records(k, t_q)) for k in ('daq_state', 'daq_accel', 'daq_objects',
        #                                                     'daq_cmd'))
        quiet = sum(len(checker.records(k, t_q)) for k in ('daq_state', 'daq_accel', 'daq_objects',
                                                            'daq_cmd', 'raw_states', 'raw_cmds'))
        ev.add('D0', 'DAQ gated: nothing on /daq/* (incl. /daq/raw/*) before run_state active',
               '0 messages in 0.6 s', f'{quiet} messages', quiet == 0)

        t_on = checker.set_daq('active e2e_run_1')
        time.sleep(3.0)
        w0, w1 = t_on + 300_000_000, t_on + 2_800_000_000
        # 26.10.05 _ YSH _ 3 토픽 (원문: for k in ('daq_state', 'daq_accel', 'daq_objects', 'daq_cmd')})
        daq_rates = {k: rate(checker.records(k, w0, w1), w0, w1)
                     for k in ('daq_state', 'daq_objects', 'daq_cmd')}
        ds = checker.records('daq_state', w0, w1)
        kin_stamps = {r[1] for r in checker.records('kinematic', t_on - 500_000_000, w1 + 500_000_000)}
        per_stamp = {}
        for r in ds:
            per_stamp[r[1]] = per_stamp.get(r[1], 0) + 1
        stamps_ok = bool(ds) and all(st in kin_stamps for st in per_stamp)
        max_rep = max(per_stamp.values()) if per_stamp else 0
        reps = sorted(per_stamp.values())
        # 가짜 MORAI 는 97 틱마다 한 틱을 건너뛴다(40 ms 구멍) → 그 표본은 ZOH 로 4 행. 실제 MORAI 도 같다
        stamp_gaps = sorted((b - a) / 1e6 for a, b in zip(sorted(per_stamp), sorted(per_stamp)[1:]))
        max_rep_allowed = int(round((stamp_gaps[-1] if stamp_gaps else 20.0) / 10.0)) + 1
        gaps = sorted((b[0] - a[0]) / 1e6 for a, b in zip(ds, ds[1:]))
        # 26.10.05 _ YSH _ MORAI 메시지: ego frame map, ego velocity.x (m/s, local) = 같은 stamp 의 kinematic twist.x,
        #   num_of_* 합 = 세 목록 길이 합. 원문: acc = records('daq_accel'); acc_ok = 전부 frame base_link;
        #   ev.add('D1', 'DAQ active: 4 topics at 100 Hz …', 'each 100 +- 3 Hz; accel frame base_link', …, rate_ok and acc_ok)
        kin_vx = {r[1]: r[6] for r in checker.records('kinematic', t_on - 500_000_000, w1 + 500_000_000)}
        frame_ok = bool(ds) and all(r[5] == 'map' for r in ds)
        matched = [r for r in ds if r[1] in kin_vx]
        vx_ok = bool(matched) and all(r[3] == kin_vx[r[1]] for r in matched)
        objs = checker.records('daq_objects', w0, w1)
        count_ok = bool(objs) and all(r[2] == r[3] for r in objs)
        rate_ok = all(abs(v - 100.0) <= 3.0 for v in daq_rates.values())
        ev.add('D1', 'DAQ active: 3 topics at 100 Hz (ZOH of the ~50 Hz MORAI samples), MORAI messages',
               'each 100 +- 3 Hz; ego_status frame map; ego velocity.x = kinematic twist.x at the same stamp; '
               'object_status num_of_* = list sizes',
               ', '.join(f'{k} {v:.1f}' for k, v in daq_rates.items()) +
               f'; frame ok {frame_ok}; vx ok {vx_ok} ({len(matched)} rows); counts ok {count_ok}',
               rate_ok and frame_ok and vx_ok and count_ok)
        ev.add('D2', 'DAQ stamps = MORAI sample time (same as kinematic_state), ZOH rows per sample',
               'every /daq/ego_status stamp is a kinematic_state stamp; median 2 rows per stamp, max = '
               'longest MORAI gap / 10 ms + 1; arrival gap p99 <= 15 ms',
               f'stamps ok {stamps_ok}, distinct {len(per_stamp)} for {len(ds)} rows, rows/stamp median '
               f'{pct(reps, 0.5) if reps else 0} max {max_rep} (allowed {max_rep_allowed}, longest MORAI gap '
               f'{stamp_gaps[-1] if stamp_gaps else float("nan"):.0f} ms), gap p50 {pct(gaps, 0.5):.1f} p99 '
               f'{pct(gaps, 0.99):.1f} max {gaps[-1] if gaps else float("nan"):.1f} ms',
               stamps_ok and bool(reps) and pct(reps, 0.5) == 2 and max_rep <= max_rep_allowed and
               bool(gaps) and pct(gaps, 0.99) <= 15.0)

        # 26.09.26 _ YSH _ D1r: 원시 gRPC 기록 — 받아들인 업데이트마다 응답 전체 한 건(ZOH 아님), 보낸 ControlVehicle
        #   마다 한 건이고, 보낸 값이 가짜 MORAI 가 받은 값과 같다
        rs = [r for r in checker.records('raw_states', w0, w1)]
        rs_all = [r[1] for r in checker.records('raw_states', t_on - 500_000_000, w1 + 500_000_000)]
        raw_stamps = {m['morai_timestamp_us'] * 1000 for m in rs_all}
        kin_mid = {r[1] for r in checker.records('kinematic', w0 + 100_000_000, w1 - 100_000_000)}
        idx = [r[1]['update_index'] for r in rs]
        env_ok = bool(rs) and all(
            m['schema'] == 'morai_raw.actor_states/1' and m['run'] == 'e2e_run_1' and
            m['actor_states']['timestamp'] == str(m['morai_timestamp_us']) and
            any(st['actor_info']['id']['value'] == 'Ego' and 'vehicle_state' in st
                for st in m['actor_states']['states'])
            for _, m in rs)
        idx_ok = bool(idx) and all(b == a + 1 for a, b in zip(idx, idx[1:]))
        one_to_one = {r[1]['morai_timestamp_us'] * 1000 for r in rs} <= kin_stamps and kin_mid <= raw_stamps
        rc = [r[1] for r in checker.records('raw_cmds', t_on - 500_000_000, w1 + 500_000_000)]
        rc_win = [m for m in rc if w0 <= m['send_system_ns'] <= w1]
        seq = [m['send_seq'] for m in rc_win]
        seq_ok = bool(seq) and all(b == a + 1 for a, b in zip(seq, seq[1:]))
        cmd_hz = len(rc_win) / ((w1 - w0) / 1e9)
        want_hz = hz if timer else rate(checker.records('kinematic', w0, w1), w0, w1)
        # 송신은 cmd_mutex_ 아래 하나씩이라 k 번째 요청은 send_k 와 send_k+1 사이에 가짜에 도착한다
        fake_cv = cv_events(fake.read_events(), w0 - 100_000_000, w1 + 100_000_000)
        matched = 0
        for m, nxt in zip(rc_win, rc_win[1:]):
            got = [e for e in fake_cv if m['send_system_ns'] <= e['t_ns'] < nxt['send_system_ns']]
            q = m['vehicle_ctrl_cmd']
            if len(got) == 1 and got[0]['type'] == q['long_cmd_type'] and all(
                    math.isclose(got[0][k], q[f], rel_tol=1e-8, abs_tol=1e-12)
                    for k, f in (('accel', 'acceleration'), ('steer', 'steer'), ('vel', 'velocity'))):
                matched += 1
        # 26.09.26 _ YSH _ 리뷰 반영: MORAI 응답 Result.status 도 그대로 남는다 (가짜는 STATUS_CODE_SUCCESS = 1)
        cmd_ok = bool(rc_win) and all(
            m['schema'] == 'morai_raw.vehicle_ctrl_cmd/1' and m['run'] == 'e2e_run_1' and m['ok'] and
            m['grpc_code'] == 0 and m['rpc_ms'] is not None and m.get('result_status') == 1 and not m['stale'] and
            m['vehicle_ctrl_cmd']['actor_info']['id']['value'] == 'Ego' for m in rc_win)
        ev.add('D1r', 'raw gRPC recording: one /daq/raw/actor_states per accepted update, one '
               '/daq/raw/vehicle_ctrl_cmd per ControlVehicle, values = what the fake received',
               'update_index and send_seq contiguous; raw MORAI stamps = kinematic stamps (both ways); '
               f'vehicle_ctrl_cmd {want_hz:.0f} +- 5 %/s, each matches the fake ControlVehicle between it and '
               'the next send',
               f'actor_states {len(rs)} (envelope ok {env_ok}, index contiguous {idx_ok}, 1:1 with kinematic '
               f'{one_to_one}); vehicle_ctrl_cmd {len(rc_win)} = {cmd_hz:.1f}/s (fields ok {cmd_ok}, seq '
               f'contiguous {seq_ok}, matched fake {matched}/{max(len(rc_win) - 1, 0)})',
               env_ok and idx_ok and one_to_one and cmd_ok and seq_ok and
               abs(cmd_hz - want_hz) <= 0.05 * want_hz and matched == len(rc_win) - 1)

        # 26.10.06 _ YSH _ D1c: 명세 25 /daq/ctrl_cmd — 100 Hz, 값 = 같은 send_system_ns 의 raw 송신(= 가짜가 받은 값, D1r)
        ctl = checker.records('daq_ctrl', w0, w1)
        ctl_hz = rate(ctl, w0, w1)
        sent = {m['send_system_ns']: m['vehicle_ctrl_cmd'] for m in rc}
        paired = [(c, sent.get(c[1])) for c in ctl]
        ctl_ok = bool(ctl) and all(q is not None and c[2] == q['long_cmd_type'] and c[3] == q['velocity'] and
                                   c[4] == q['acceleration'] for c, q in paired)
        ev.add('D1c', 'spec 25 /daq/ctrl_cmd (MORAI CtrlCmd) = the gRPC VehicleCtrlCmd actually sent, 100 Hz ZOH',
               'rate 100 +- 3 Hz; every message equals the raw send with the same send_system_ns',
               f'{ctl_hz:.1f} Hz, {len(ctl)} msgs, paired {sum(1 for _, q in paired if q is not None)}, values ok {ctl_ok}',
               abs(ctl_hz - 100.0) <= 3.0 and ctl_ok)

        # 26.10.07 _ YSH _ D1k: /daq/collision — 100 Hz, stamp = ego_status stamp, 내용 = 같은 표본 raw 의 Ego collision_objects
        #   (가짜는 홀수 초에 [NPC_Car_1, Wall_1]) — NPC 는 type 1 + object_status 와 같은 위치, 벽은 type −1
        col = checker.records('daq_collision', w0, w1)
        col_hz = rate(col, w0, w1)
        ego_stamps = {r[1] for r in checker.records('daq_state', w0, w1)}
        raw_col = {}
        for _, m in checker.records('raw_states', w0 - 200_000_000, w1):
            for st in m['actor_states']['states']:
                if st['actor_info']['id']['value'] == 'Ego':
                    names = st.get('vehicle_state', {}).get('collision_objects', [])
                    # stamp 는 time_source 에 따라 rx_system_ns 또는 morai_timestamp_us × 1000 (daq/run.py _raw_for 와 같다)
                    raw_col[m['rx_system_ns']] = names
                    if m.get('morai_timestamp_us'):
                        raw_col[int(m['morai_timestamp_us']) * 1000] = names
        hit = [c for c in col if c[2]]
        bad_stamp = [c for c in col if c[1] not in ego_stamps]
        bad_names = [(c[1], [n for n, _, _ in c[2]], raw_col.get(c[1])) for c in col
                     if [n for n, _, _ in c[2]] != raw_col.get(c[1])]   # 짝 raw 가 없으면 빈 목록이어도 실패로 센다
        bad_type = [c for c in col if not all((t == 1) if n == 'NPC_Car_1' else (t == -1 and x == 0.0) for n, t, x in c[2])]
        col_ok = bool(col) and bool(hit) and len(hit) < len(col) and not bad_stamp and not bad_names and not bad_type
        ev.add('D1k', '/daq/collision (MORAI CollisionData) = Ego collision_objects of the same sample, 100 Hz ZOH',
               'rate 100 +- 3 Hz; stamp = an ego_status stamp; names = raw Ego vehicle_state.collision_objects '
               '(same rx_system_ns); NPC_Car_1 type 1, Wall_1 type -1; both empty and non-empty rows seen',
               f'{col_hz:.1f} Hz, {len(col)} msgs, with collision {len(hit)}, values ok {col_ok} '
               f'(stamp miss {len(bad_stamp)}, names miss {len(bad_names)} e.g. {bad_names[:2]}, type miss {len(bad_type)})',
               abs(col_hz - 100.0) <= 3.0 and col_ok)

        # D3: Autoware 가 조용해지면 vehicle_cmd 는 0.2 s 뒤 멈추고, 상태는 계속
        checker.set_cmd(on=False)
        t_cmd_off = checker.last_cmd_publish_ns()
        time.sleep(0.8)
        late_cmd = checker.records('daq_cmd', t_cmd_off + int((STALE_TIMEOUT_S + 0.03) * 1e9))
        still_state = checker.records('daq_state', t_cmd_off + 300_000_000)
        # 26.10.05 _ YSH _ D3·D4·D5 설명의 vehicle_state → ego_status (기록 key daq_state 는 이제 /daq/ego_status)
        ev.add('D3', 'control_cmd stops -> /daq/vehicle_cmd stops after cmd_stale_s, state continues',
               f'no vehicle_cmd later than {STALE_TIMEOUT_S * 1000:.0f}+30 ms; ego_status still ~100 Hz',
               f'vehicle_cmd after: {len(late_cmd)}; ego_status in the last 0.5 s: {len(still_state)}',
               not late_cmd and len(still_state) >= 40)
        # 26.09.26 _ YSH _ D3r: 그동안 엔진이 보낸 감속 대체 명령은 원시 기록에 stale=true, 보낸 값(-2.4) 그대로 남는다
        stale_raw = [r[1] for r in checker.records('raw_cmds', t_cmd_off)
                     if r[1]['send_system_ns'] > t_cmd_off + int((STALE_TIMEOUT_S + 0.03) * 1e9)]
        ev.add('D3r', 'raw vehicle_ctrl_cmd marks the stale substitute',
               f'every send later than {STALE_TIMEOUT_S * 1000:.0f}+30 ms has stale true and acceleration '
               f'{STALE_ACCEL}',
               f'{len(stale_raw)} sends, stale {sum(1 for m in stale_raw if m["stale"])}, accel '
               f'{sorted({m["vehicle_ctrl_cmd"]["acceleration"] for m in stale_raw})}',
               bool(stale_raw) and all(m['stale'] and m['vehicle_ctrl_cmd']['acceleration'] == STALE_ACCEL
                                       for m in stale_raw))
        checker.set_cmd(on=True, accel=0.2, steer=0.0)

        # D4: 켜진 채 노드 재시작 → latched run_state 를 다시 받아 이어 낸다 (감시기 재시작 대비)
        t_kill = now_ns()
        node.sigint(timeout=10.0)
        node.start()
        t_back = None
        deadline = time.time() + 15.0
        while time.time() < deadline:
            recs = checker.records('daq_state', t_kill + 500_000_000)
            if recs:
                t_back = recs[0][0]
                break
            time.sleep(0.05)
        time.sleep(1.5)
        # 종료 요약은 pre-shutdown 콜백이 컨텍스트가 닫히기 전에 낸다 (수신 콜백이 늦을 수 있어 재개 뒤에 읽는다)
        summ_kill = [r for r in checker.records('daq_summary', t_kill)]
        after = checker.records('daq_state', (t_back or now_ns()) + 300_000_000, now_ns())
        resumed_hz = rate(after, (t_back or now_ns()) + 300_000_000, now_ns()) if after else 0.0
        ev.add('D4', 'node restarted mid-run -> DAQ resumes from the latched run_state',
               'shutdown summary for e2e_run_1; /daq/ego_status again at ~100 Hz without a new active',
               f'shutdown summary {[s[1].get("end_reason") for s in summ_kill]}; back after '
               f'{(t_back - t_kill) / 1e9 if t_back else float("nan"):.1f} s at {resumed_hz:.1f} Hz',
               t_back is not None and abs(resumed_hz - 100.0) <= 5.0 and
               any(s[1].get('run') == 'e2e_run_1' and s[1].get('end_reason') == 'shutdown' for s in summ_kill))

        # D5: idle → 멈춤 + 런 요약
        t_idle = checker.set_daq('idle e2e_run_1')
        time.sleep(1.0)
        after_idle = checker.records('daq_state', t_idle + 150_000_000)
        summ = [r[1] for r in checker.records('daq_summary', t_idle)]
        # 재시작한 노드의 요약은 재시작 뒤부터다. 첫 writer 점검(1 s) 전에는 양보해 내지 않으므로(yield_ticks) 상태
        #   수는 '다시 나온 뒤 ~ idle' 구간의 100 Hz 로 본다
        s_ok = False
        if summ and t_back:
            sm = summ[-1]
            span_s = (t_idle - t_back) / 1e9
            s_ok = sm.get('run') == 'e2e_run_1' and sm.get('end_reason') == 'idle' and \
                abs(int(sm.get('state_published', 0)) / span_s - 100.0) <= 6.0 and \
                int(sm.get('yield_ticks', 0)) > 0
        ev.add('D5', 'run_state idle -> /daq/* stops, run summary on /daq/run_summary',
               'no ego_status 150 ms after idle; summary run e2e_run_1, end_reason idle, ~100 Hz since '
               'the restart resumed, yield_ticks > 0 (restart waited for the writer check)',
               f'ego_status after idle {len(after_idle)}; summary {summ[-1] if summ else None}',
               not after_idle and s_ok)
        # 26.09.26 _ YSH _ D5r: 원시 기록도 idle 뒤 멈추고, 요약의 수가 받은 수와 맞는다 (넘침 0)
        raw_after = checker.records('raw_states', t_idle + 150_000_000) + \
            checker.records('raw_cmds', t_idle + 150_000_000)
        r_ok = False
        r_text = 'no summary'
        if summ and t_back:
            sm = summ[-1]
            # 재시작한 노드 것만 (앞 노드는 종료 요약에 따로 셌다). 새 노드는 첫 writer 점검(1 s) 뒤에야 내므로
            #   t_back - 0.5 s 는 앞 노드가 끝난 뒤이고 새 노드의 첫 원시 기록보다 앞이다
            t_new = t_back - 500_000_000
            got_rs = [r[1] for r in checker.records('raw_states', t_new) if r[1]['run'] == 'e2e_run_1']
            got_rc = [r[1] for r in checker.records('raw_cmds', t_new) if r[1]['run'] == 'e2e_run_1']
            r_text = (f"summary raw {sm.get('raw_published')}/{sm.get('raw_dropped')} tx "
                      f"{sm.get('tx_published')}/{sm.get('tx_dropped')}; received after restart "
                      f"{len(got_rs)}/{len(got_rc)}")
            r_ok = int(sm.get('raw_published', -1)) == len(got_rs) > 0 and \
                int(sm.get('tx_published', -1)) == len(got_rc) > 0 and \
                sm.get('raw_dropped') == 0 and sm.get('tx_dropped') == 0
        ev.add('D5r', 'run_state idle -> /daq/raw/* stops; summary raw counts = messages received',
               'no raw message 150 ms after idle; raw_published / tx_published equal what the checker got '
               'since the restart; dropped 0',
               f'raw after idle {len(raw_after)}; {r_text}', not raw_after and r_ok)
        marks['daq'] = {'rates': daq_rates, 'summary': summ[-1] if summ else None}

        # ---------------------------------------------------------------- 종료 (SIGINT → 감속)
        checker.set_cmd(on=True, accel=0.3, steer=0.0)
        # 26.09.26 _ YSH _ 리뷰 반영: always 모드로 끈다. stop 뒤에도 게이트가 열린 채라 종료 감속이 원시 큐에 들어가
        #   ~DaqSampler 가 이미 지워진 publisher 로 내던 경로 (고친 뒤: 첫 stop 뒤 submit 은 버린다)
        always_ok = checker.set_params({'daq.mode': 'always'})
        time.sleep(1.5)
        t_sig = now_ns()
        code = node.sigint(timeout=10.0)
        t_exit = now_ns()
        events = fake.read_events()
        brakes = [e for e in cv_events(events, t_sig) if e['accel'] == STALE_ACCEL]
        ev.add('S', 'SIGINT node while driving',
               'exits cleanly (code 0) within 5 s after ~0.3 s of brake commands (-2.4)',
               f'exit code {code} after {(t_exit - t_sig) / 1e9:.2f} s; brake commands {len(brakes)}',
               code == 0 and (t_exit - t_sig) / 1e9 < 5.0 and len(brakes) >= 5)
        # 26.09.26 _ YSH _ 리뷰 반영 Sr: always 모드에서도 깨끗이 끝나고, 종료 감속(stale)은 원시 기록에 없다
        raw_always = [r[1] for r in checker.records('raw_cmds', t_sig - 1_000_000_000, t_sig)]
        brake_raw = [r[1] for r in checker.records('raw_cmds', t_sig) if r[1]['stale']]
        ev.add('Sr', 'daq.mode always + SIGINT: the raw path closes at the DAQ stop',
               'raw vehicle_ctrl_cmd with run "" in the last 1 s before SIGINT; exit code 0; no stale '
               '(shutdown brake) raw command after SIGINT',
               f'set daq.mode always {always_ok}; raw cmds in the last 1 s {len(raw_always)} (run "" '
               f'{bool(raw_always) and all(m["run"] == "" for m in raw_always)}); exit code {code}; '
               f'stale raw after SIGINT {len(brake_raw)}',
               always_ok and bool(raw_always) and all(m['run'] == '' for m in raw_always) and code == 0 and
               not brake_raw)
    finally:
        checker.stop_threads = True
        if node.alive():
            node.sigint(timeout=10.0)
        fake.stop()
        executor.shutdown()
        checker.destroy_node()
        rclpy.try_shutdown()

    # 26.09.25 _ YSH _ 노드의 10 초 통계 줄(제어 재송신 / MORAI 갱신)을 증거에 남긴다 (목표·지연재설정·명령 트리거)
    try:
        with open(node.log_path, encoding='utf-8', errors='replace') as f:
            marks['stats_lines'] = [ln.strip() for ln in f if '제어 재송신' in ln or 'MORAI 갱신' in ln]
        for ln in marks['stats_lines'][:4]:
            print(ln, flush=True)
    except OSError:
        pass
    with open(os.path.join(args.out, 'evidence.json'), 'w') as f:
        json.dump({'rows': ev.rows, 'marks': {k: v for k, v in marks.items()}}, f, indent=1,
                  default=str)
    passed = sum(1 for r in ev.rows if r['result'] == 'PASS')
    print(f'==== {passed}/{len(ev.rows)} PASS — {args.out}/evidence.json', flush=True)
    return 0 if ev.ok() else 1


if __name__ == '__main__':
    sys.exit(main())
