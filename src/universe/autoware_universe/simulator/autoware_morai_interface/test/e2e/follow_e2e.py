#!/usr/bin/env python3
# 26.11.09 _ YSH _ 홈 정리로 설명문·주석 속 경로도 새 위치로 바꿨다 (예 ~/DAQ_Bag → ~/01_DAQ_Data/DAQ_Bag, 옛→새 목록은 ~/99_Archive/_move_261109.tsv)
# 신규 — 26.10.10 _ YSH _ 의사 VILS follow 모드 종단 시험 (가짜 MORAI · 수동 실행 전용, colcon test 아님)
"""
구성: fake_gnss_ins.py(합성 INS) → katech_vils_gnss_pose(실차 위치 → map) → /localization/kinematic_state
      → 브리지 ego_mode=follow → SetTransform → fake_morai_server
확인
  F1 SetTransform 이 follow.rate_hz(20) ± 15 % 로 가고, 보낸 위치 = 같은 시각의 /localization/kinematic_state (+ 외삽 ≤ 0.6 m)
  F2 가짜 MORAI 에 ControlVehicle · SetVehicleControlMode · SetVehicleGear 가 0 회
  F3 브리지가 /vehicle/status/velocity_status · /localization/kinematic_state · TF 를 내지 않고 /morai/ego/* 를 낸다
     · /control/control_mode_request 서비스를 만들지 않는다 · 객체(/perception/.../tracking/objects)는 낸다
  F4 forward_control_to_sim=true · teleport_on_initialpose=true 요청을 거부한다
  F5 진단 follow 가 OK (writer 1개, RPC 성공)
  F6 실차 측위가 끊기면(합성 INS 종료) SetTransform 도 0.5 s 안에 멈춘다
  26.10.12 _ YSH _ 2단계 (경로·장애물 말고는 실차):
  F2 를 바꾼다 — ControlVehicle 은 전부 GHOST(4) 이고 SetTransform 수와 맞는다, SetVehicleControlMode 는 AUTO(3) 만, 기어 0 회
     (원문: F2 가짜 MORAI 에 ControlVehicle · SetVehicleControlMode · SetVehicleGear 가 0 회)
  F7 /daq/ego_status 속도 = 실차 /vehicle/status/velocity_status (MORAI Ego 값이 아니다), 위치 = 실차 측위 (MORAI 좌표)
  F8 /daq/ctrl_cmd = Autoware control_cmd (가속도 모드, 실차에 보낸 명령)
  F9 MORAI Ego 방향지시등은 실차 등화 상태(turn_indicators_status)를 따르고 Autoware 명령은 무시한다
사용 (도메인은 비어 있어야 한다, gRPC 는 127.0.0.1 만)
  source /opt/ros/humble/setup.bash && source ~/10_Masstesting-Autoware/Morai_Autoware/install/setup.bash && source ~/12_VILS-Autoware/katech-vils-ws/install/setup.bash
  ROS_DOMAIN_ID=93 ROS_LOCALHOST_ONLY=1 python3 follow_e2e.py --ws ~/10_Masstesting-Autoware/Morai_Autoware --out /tmp/follow_e2e
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from autoware_control_msgs.msg import Control  # 26.10.12 _ YSH _ F8
from autoware_vehicle_msgs.msg import TurnIndicatorsCommand, TurnIndicatorsReport, VelocityReport  # 26.10.12 _ YSH _ F7·F9
from morai_ros2_msgs.msg import CtrlCmd, EgoVehicleStatus  # 26.10.12 _ YSH _ F7·F8

NODE = 'morai_bridge'


def popen(args, log):
    return subprocess.Popen(args, stdout=open(log, 'w'), stderr=subprocess.STDOUT, preexec_fn=os.setsid)


def stop(p):
    if p and p.poll() is None:
        try:
            os.killpg(p.pid, signal.SIGINT)
            p.wait(timeout=8)
        except Exception:  # noqa: BLE001
            os.killpg(p.pid, signal.SIGKILL)


def main():
    ap = argparse.ArgumentParser()
    # 26.11.09 _ YSH _ 홈 정리 — 새 위치. 원문:
    # ap.add_argument('--ws', default=os.path.expanduser('~/Morai_Autoware'))
    ap.add_argument('--ws', default=os.path.expanduser('~/10_Masstesting-Autoware/Morai_Autoware'))
    ap.add_argument('--out', required=True)
    ap.add_argument('--port', type=int, default=17891)
    ap.add_argument('--rate', type=float, default=20.0)
    a = ap.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') in (None, '', '0', '10'):
        sys.exit('격리 도메인(예: 93)에서 돌린다')
    os.makedirs(a.out, exist_ok=True)
    events = os.path.join(a.out, 'fake_events.jsonl')
    if os.path.exists(events):
        os.remove(events)
    offset = os.path.join(a.out, 'offset.yaml')
    # 실차 기준점 → 가짜 MORAI 월드의 (120, 210) · yaw 90° (가짜 Ego 는 (100, 200) 에서 시작)
    open(offset, 'w').write("""/**:
  ros__parameters:
    offset: {ref_lat: 36.7845, ref_lon: 127.1867, ref_alt: 0.0, ref_heading: 30.0, ref_heading_convention: ned_cw_deg,
             target_x: 120.0, target_y: 210.0, target_yaw_deg: 90.0}
    lever_arm: {x: 1.2, y: 0.0, z: 1.5}
    z_source: constant
    init_localization: false
""")
    procs = []
    fake = popen([os.path.join(a.ws, 'build/morai_grpc_vendor/fake_morai_server'), '--port', str(a.port),
                  '--events', events], os.path.join(a.out, 'fake.log'))
    procs.append(fake)
    time.sleep(1.0)
    params = os.path.join(a.ws, 'install/autoware_morai_interface/share/autoware_morai_interface/config/'
                                'morai_interface.gt.param.yaml')
    bridge = popen([os.path.join(a.ws, 'install/autoware_morai_interface/lib/autoware_morai_interface/'
                                       'autoware_morai_interface_node'),
                    '--ros-args', '-r', f'__node:={NODE}', '--params-file', params,
                    '-p', 'morai.address:=127.0.0.1', '-p', f'morai.port:={a.port}', '-p', 'time_source:=morai',
                    # 26.10.12 _ YSH _ DAQ 를 켠다 (F7·F8). 원문: '-p', 'publish_daq:=false'],
                    '-p', 'ego_mode:=follow', '-p', f'follow.rate_hz:={a.rate}', '-p', 'publish_daq:=true',
                    '-p', 'daq.mode:=always'],
                   os.path.join(a.out, 'bridge.log'))
    procs.append(bridge)
    gnss = popen(['ros2', 'run', 'katech_vils_gnss_pose', 'katech_vils_gnss_pose_node', '--ros-args',
                  '--params-file', offset, '-r', '~/input/navsatfix:=/sensing/gnss/nav_sat_fix',
                  '-r', '~/input/imu:=/sensing/gnss/imu', '-r', '~/input/velocity_status:=/vehicle/status/velocity_status',
                  '-r', '~/output/kinematic_state:=/localization/kinematic_state'], os.path.join(a.out, 'gnss.log'))
    procs.append(gnss)
    time.sleep(3.0)
    ins = popen(['ros2', 'run', 'katech_vils_gnss_pose', 'fake_gnss_ins.py', '--duration', '12'],
                os.path.join(a.out, 'ins.log'))
    procs.append(ins)

    rclpy.init()
    n = Node('follow_e2e_checker')
    loc = []
    diag = []
    n.create_subscription(Odometry, '/localization/kinematic_state', lambda m: loc.append((time.time_ns(), m)), 50)
    n.create_subscription(DiagnosticArray, '/diagnostics',
                          lambda m: diag.extend(s for s in m.status if s.name.endswith(': follow')), 50)
    # 26.10.12 _ YSH _ F7·F8·F9: 실차 속도·DAQ·Autoware 명령·실차 등화
    vel, daq_ego, daq_ctrl = [], [], []
    n.create_subscription(VelocityReport, '/vehicle/status/velocity_status', lambda m: vel.append(m), 50)
    n.create_subscription(EgoVehicleStatus, '/daq/ego_status', lambda m: daq_ego.append((time.time_ns(), m)), 200)
    n.create_subscription(CtrlCmd, '/daq/ctrl_cmd', lambda m: daq_ctrl.append(m), 200)
    pub_cmd = n.create_publisher(Control, '/control/command/control_cmd', 10)
    pub_turn_report = n.create_publisher(TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 10)
    pub_turn_cmd = n.create_publisher(TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', 10)

    def send_inputs(phase):
        c = Control()
        c.stamp = n.get_clock().now().to_msg()
        c.longitudinal.acceleration = 0.7
        c.longitudinal.velocity = 4.0
        c.lateral.steering_tire_angle = 0.05
        pub_cmd.publish(c)
        r = TurnIndicatorsReport()
        r.stamp = c.stamp
        r.report = TurnIndicatorsReport.ENABLE_LEFT if phase else TurnIndicatorsReport.DISABLE
        pub_turn_report.publish(r)
        k = TurnIndicatorsCommand()
        k.stamp = c.stamp
        k.command = TurnIndicatorsCommand.ENABLE_RIGHT  # 무시돼야 한다
        pub_turn_cmd.publish(k)
    res = []

    def check(name, desc, ok, detail):
        res.append(dict(name=name, ok=bool(ok), desc=desc, detail=detail))
        print(f"[{'PASS' if ok else 'FAIL'}] {name} {desc}: {detail}", flush=True)

    t_end = time.time() + 10.0
    t_left = time.time() + 5.0  # 5 s 뒤 실차 좌측 지시등
    t_next = 0.0
    while time.time() < t_end:
        if time.time() >= t_next:
            send_inputs(time.time() >= t_left)
            t_next = time.time() + 0.05
        rclpy.spin_once(n, timeout_sec=0.02)

    # F3 그래프
    topics = dict(n.get_topic_names_and_types())
    pubs = lambda t: [i.node_name for i in n.get_publishers_info_by_topic(t)]  # noqa: E731
    services = [s for s, _ in n.get_service_names_and_types()]
    f3 = (NODE not in pubs('/vehicle/status/velocity_status') and NODE not in pubs('/localization/kinematic_state')
          and NODE not in pubs('/tf') and NODE in pubs('/morai/ego/velocity_status')
          and NODE in pubs('/morai/ego/kinematic_state') and NODE in pubs('/perception/object_recognition/tracking/objects')
          and '/control/control_mode_request' not in services)
    check('F3', 'bridge publishes /morai/ego/* only (no vehicle status, kinematic_state, TF, control_mode_request); objects yes',
          f3, f"velocity {pubs('/vehicle/status/velocity_status')} · kinematic {pubs('/localization/kinematic_state')} · "
              f"tf {pubs('/tf')} · /morai/ego/velocity {pubs('/morai/ego/velocity_status')} · objects "
              f"{pubs('/perception/object_recognition/tracking/objects')} · control_mode_request "
              f"{'/control/control_mode_request' in services}")

    # F4 잠금
    cli = n.create_client(SetParameters, f'/{NODE}/set_parameters')
    cli.wait_for_service(timeout_sec=5.0)
    req = SetParameters.Request()
    req.parameters = [Parameter(name='forward_control_to_sim', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)),
                      Parameter(name='teleport_on_initialpose', value=ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True))]
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=5.0)
    rs = fut.result().results if fut.result() else []
    check('F4', 'enabling forward_control_to_sim / teleport_on_initialpose is rejected',
          len(rs) == 2 and not any(r.successful for r in rs), [r.reason for r in rs])

    # INS 가 끝날 때까지 (duration 12 s) 기다린 뒤 끊김 확인
    t_ins_end = time.time() + 6.0
    while time.time() < t_ins_end:
        rclpy.spin_once(n, timeout_sec=0.05)
    t_loc_last = loc[-1][0] if loc else 0
    t_wait = time.time() + 2.0
    while time.time() < t_wait:
        rclpy.spin_once(n, timeout_sec=0.05)
    for p in reversed(procs):
        stop(p)

    ev = [json.loads(l) for l in open(events)] if os.path.exists(events) else []
    st = [e for e in ev if e.get('rpc') == 'SetTransform']
    # 26.10.12 _ YSH _ 고스트: ControlVehicle 은 GHOST(4) 만, 제어 모드는 AUTO(3) 만, 기어 0. 원문:
    # banned = {k: sum(1 for e in ev if e.get('rpc') == k) for k in ('ControlVehicle', 'SetVehicleControlMode', 'SetVehicleGear')}
    # check('F2', 'no ControlVehicle / SetVehicleControlMode / SetVehicleGear', not any(banned.values()), banned)
    cv = [e for e in ev if e.get('rpc') == 'ControlVehicle']
    non_ghost = [e for e in cv if e.get('type') != 4]
    modes = sorted({e.get('mode') for e in ev if e.get('rpc') == 'SetVehicleControlMode'})
    gears = sum(1 for e in ev if e.get('rpc') == 'SetVehicleGear')
    check('F2', 'MORAI gets only GHOST ControlVehicle (one per SetTransform), control mode AUTO only, no gear',
          not non_ghost and cv and abs(len(cv) - len(st)) <= 2 and modes == [3] and gears == 0,
          f'ControlVehicle {len(cv)} (non-GHOST {len(non_ghost)}) · SetTransform {len(st)} · modes {modes} · gear {gears}')
    # 26.10.12 _ YSH _ F7 DAQ Ego = 실차 (fake_gnss_ins 의 CAN 속도 5 m/s), 위치 = 실차 측위 (원점 0 이라 map = MORAI)
    if daq_ego and vel and loc:
        sp = [m.velocity.x for _, m in daq_ego]
        v_can = vel[-1].longitudinal_velocity
        t_e, e_last = daq_ego[len(daq_ego) // 2]
        o = min(loc, key=lambda q: abs(q[0] - t_e))[1]
        d = math.hypot(e_last.position.x - o.pose.pose.position.x, e_last.position.y - o.pose.pose.position.y)
        check('F7', '/daq/ego_status speed = real-car CAN speed, position = real-car localization',
              abs(sp[len(sp) // 2] - v_can) < 0.05 and d < 0.6,
              f'daq speed {sp[len(sp) // 2]:.2f} m/s vs CAN {v_can:.2f} · position diff {d:.2f} m · {len(daq_ego)} rows')
    else:
        check('F7', '/daq/ego_status from the real car', False, f'daq {len(daq_ego)} vel {len(vel)} loc {len(loc)}')
    check('F8', '/daq/ctrl_cmd = Autoware control_cmd (acceleration mode)',
          bool(daq_ctrl) and daq_ctrl[-1].longl_cmd_type == 3 and abs(daq_ctrl[-1].acceleration - 0.7) < 1e-3
          and abs(daq_ctrl[-1].velocity - 14.4) < 1e-3,
          f'{len(daq_ctrl)} rows, last type {daq_ctrl[-1].longl_cmd_type} acc {daq_ctrl[-1].acceleration:.2f} '
          f'vel {daq_ctrl[-1].velocity:.2f} km/h' if daq_ctrl else 'none')
    lights = [e.get('signal') for e in ev if e.get('rpc') == 'SetVehicleTailLight']
    check('F9', 'MORAI Ego turn signal follows the real-car report (left), not the Autoware command (right)',
          bool(lights) and lights[-1] == 1 and 2 not in lights,  # TurnSignal kLeft 1 · kRight 2
          f'SetVehicleTailLight signals {lights[:3]} … {lights[-3:]}')

    # F1 주기·값: 측위가 흐르는 구간
    if loc and st:
        t0, t1 = loc[0][0] + 1_000_000_000, t_loc_last
        win = [e for e in st if t0 <= e['t_ns'] <= t1]
        rate = len(win) / max((t1 - t0) * 1e-9, 1e-6)
        errs = []
        for e in win:
            o = min(loc, key=lambda q: abs(q[0] - e['t_ns']))[1]
            errs.append(math.hypot(e['x'] - o.pose.pose.position.x, e['y'] - o.pose.pose.position.y))
        errs.sort()
        ok = abs(rate - a.rate) <= 0.15 * a.rate and errs and errs[-1] < 0.6
        check('F1', f'SetTransform at {a.rate:g} Hz with the localization pose (+extrapolation)', ok,
              f'{rate:.1f} Hz, {len(win)} calls, pose diff median {errs[len(errs) // 2]:.3f} m max {errs[-1]:.3f} m')
        late = [e for e in st if e['t_ns'] > t_loc_last + 500_000_000]
        check('F6', 'SetTransform stops within 0.5 s after localization stops', not late,
              f'{len(late)} calls more than 0.5 s after the last kinematic_state')
    else:
        check('F1', 'SetTransform stream', False, f'loc {len(loc)} SetTransform {len(st)}')
    ok_diag = [d for d in diag if d.level == b'\x00' or d.level == 0]
    check('F5', 'follow diagnostic reaches OK', bool(ok_diag),
          (ok_diag[-1].message if ok_diag else (diag[-1].message if diag else 'no follow diagnostic')))
    json.dump(res, open(os.path.join(a.out, 'evidence.json'), 'w'), ensure_ascii=False, indent=1)
    passed = sum(r['ok'] for r in res)
    print(f'==== {passed}/{len(res)} PASS — {a.out}')
    sys.exit(0 if passed == len(res) else 1)


if __name__ == '__main__':
    main()
