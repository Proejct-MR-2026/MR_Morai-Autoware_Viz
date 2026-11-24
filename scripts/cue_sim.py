#!/usr/bin/env python3
"""MORAI PC 흉내내기 — cue_server.py 검증용 하네스.

26.08.21 _ YSH _ MORAI PC 없이 cue_server 를 끝까지 돌려 보기 위해 신규.

gRPC 상으로는 이게 MORAI PC 그 자체다. 실제 MORAI 자동화 스크립트가 하는 순서를 그대로
흉내내고, 거기에 "상대가 오작동할 때" 를 주입할 수 있는 플래그를 붙였다.

  1) SendCue(scenario, start, goal, v0)  → ADS
  2) ADS 의 SendState(READY) 를 기다린다   ← 우리가 서버로 받는다
  3) --sim-time 만큼 시뮬레이션한 셈 치고
     (26.10.01 _ YSH _ 그 사이 ADS 가 SendState(END)(도착, cue_server --send-end-on-arrival)를 보내면 일찍 끝낸다 —
      MORAI 쪽에 바라는 동작의 흉내. --ignore-ads-end 면 거부(success=false)하고 끝까지 돈다 = 구현 전 MORAI)
  4) SendState(END) → ADS
  5) --repeat 만큼 반복

ROS 도 MORAI 라이브러리도 쓰지 않는다. grpc + cue_pb2 만 있으면 돈다.

    ./cue_sim.py --ads 127.0.0.1:50052 --listen 0.0.0.0:50152 \
                 --scenario scenario_1_11 --start 809.3,-909.3,-52.9 \
                 --goal 886.1,-1005.8,-51.2 --v0 13.89 --sim-time 20 --repeat 3
"""
import argparse
import csv
import glob
import json
import os
import re          # 26.09.09 _ YSH _ loaded_map_site() 용
import subprocess  # 26.09.09 _ YSH _ loaded_map_site() 용
import sys
import threading
import time
from concurrent import futures
from datetime import datetime

CURRENT_PATH = os.path.dirname(os.path.realpath(__file__))
sys.path.append(CURRENT_PATH)
sys.path.append(os.path.join(CURRENT_PATH, 'cue'))

import grpc

import cue_pb2
import cue_pb2_grpc
# 26.11.10 _ YSH _ 외란 주입 — KATECH 시나리오 API 대역으로 SendCustomCommand 를 보낸다 (--custom)
import json
import threading
from google.protobuf import json_format


def log(msg):
    print(f"[{datetime.now().strftime('%H:%M:%S')}] SIM  {msg}", flush=True)


class FakeMoraiServicer(cue_pb2_grpc.CueServiceServicer):
    """ADS 가 우리에게 보내는 것을 받는다. 실제로는 SendState(READY) 뿐이다.
    (26.10.01 _ YSH _ 도착 END 송신을 켠 ADS 는 SendState(END) 도 보낸다)"""

    # def __init__(self):
    def __init__(self, ignore_ads_end=False):
        self.ready = threading.Event()
        self.received = []
        # 26.10.01 _ YSH _ ADS 의 END(도착). 시뮬레이션 대기를 일찍 끊는다. ignore 면 거부 응답만 하고 끊지 않는다
        self.ads_end = threading.Event()
        self.ignore_ads_end = ignore_ads_end

    def SendState(self, request, context):
        name = cue_pb2.TestState.Name(request.state)
        log(f'← SendState({name}) sender={request.sender!r}')
        self.received.append(name)
        if request.state == cue_pb2.READY:
            self.ready.set()
        # 26.10.01 _ YSH _ ADS 의 END — 시나리오를 끝낸다고 답한다(구현 전 MORAI 흉내면 거부)
        if request.state == cue_pb2.END:
            if self.ignore_ads_end:
                return cue_pb2.StateResponse(success=False, message='END from ADS not supported (ignored)')
            self.ads_end.set()
            return cue_pb2.StateResponse(success=True, message='scenario stop requested')
        return cue_pb2.StateResponse(success=True, message='ok')

    def SendCue(self, request, context):
        log(f'← SendCue scenario={request.scenario_name!r} sender={request.sender!r}')
        return cue_pb2.CueResponse(success=True, message='ok')

    def Ping(self, request, context):
        return cue_pb2.PingResponse(alive=True)


def parse_triple(s):
    parts = [p.strip() for p in s.split(',')]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError('x,y,h 형식이어야 한다')
    return tuple(float(p) for p in parts)


def scenario_poses(scenario_id, site):
    """실제 CSV 에서 start/goal 을 읽는다. (start, goal, length_m) 반환.

    KATECH 가 보낼 숫자와 같은 출처(trajectory 파일)라 rotation.z 편향까지 똑같이 겪는다.
    describe() 는 중첩 dict 를 돌려주므로 키로 꺼낸다 (속성 접근이 아니다).
    yaw 는 라디안으로 오는데 cue.proto 의 rotation_z 는 도 단위라 변환한다.
    """
    import math
    sys.path.append(CURRENT_PATH)
    import scenario_pose
    d = scenario_pose.describe(scenario_id, site)
    s, g = d['start'], d['goal']
    return (
        (s['x'], s['y'], math.degrees(s['yaw'])),
        (g['x'], g['y'], math.degrees(g['yaw'])),
        d['length_m'],
    )


# 26.09.09 _ YSH _ 로드된 맵을 실제로 보고 판단한다.
#   원래는 ON_MAP_SITE='KIAPI' 하드코딩이라 --site KATRI 면 무조건 "맵 밖" 경고를 찍었다.
#   09-08 에 MORAI/Autoware 를 KATRI 로 바꾼 뒤로는 이게 낡은 오경보다 — 성공한 36런 sweep
#   두 번(09-09 00:30, 02:08) 에서도 계속 떴고, 하필 진짜 실패 코드인 ROUTE_NOT_SET 을 예고해서
#   분류를 방해했다. 이제는 Autoware 가 띄운 map_path 로 판단하고, 못 읽으면 경고하지 않는다.
ON_MAP_SITE = 'KIAPI'          # 판단 불가일 때의 예전 기본값(참고용)
SITE_MAP_DIR = {'KIAPI': 'kiapi', 'KATRI': 'katri', 'Sangam': 'sangam'}


def loaded_map_site():
    """실행 중인 e2e_simulator 의 map_path 로 현재 로드된 site 를 추정한다. 모르면 None."""
    try:
        out = subprocess.run(['ps', '-eo', 'args'], capture_output=True, text=True, timeout=5).stdout
    except Exception:
        return None
    for line in out.splitlines():
        if 'e2e_simulator' not in line:
            continue
        m = re.search(r'map_path:=(\S+)', line)
        if not m:
            continue
        low = m.group(1).rstrip('/').lower()
        for site, token in SITE_MAP_DIR.items():
            if low.endswith(token) or f'/{token}' in low:
                return site
    return None

# 전체 기능 테스트용 5개. 경로 길이 범위를 고르게 덮도록 골랐다(전부 KATECH_7 안).
#   1_13 119.8m(최단) / 1_11 123.4m / 2_13 243.3m / 1_14 523.9m / 1_16 796.3m(최장)
# 최장은 route_timeout_s=20 이 길이와 무관하게 고정이라 제일 먼저 티가 나는 자리다.
SMOKE_5 = ['1_13', '1_11', '2_13', '1_14', '1_16']


def load_scenario_list(path):
    """한 줄에 시나리오 id 하나. '#' 주석과 빈 줄은 무시."""
    ids = []
    with open(os.path.expanduser(path), encoding='utf-8') as f:
        for line in f:
            line = line.split('#', 1)[0].strip()
            if line:
                ids.append(line)
    return ids


def build_queue(args):
    """(label, start, goal) 목록을 만든다.

    항목마다 자기 좌표를 갖는 게 핵심이다. 예전에는 --from-scenario-pose 하나를 모든
    반복에 같은 값으로 써서 여러 시나리오를 연속으로 돌릴 방법이 아예 없었다.
    """
    ids = []
    if args.preset == 'smoke5':
        ids = list(SMOKE_5)
    elif args.preset == 'katech7':
        sys.path.append(CURRENT_PATH)
        import scenario_pose
        ids = list(scenario_pose.KATECH_7)
    elif args.preset == 'all':
        # 해당 site 의 scenario_* 폴더 전부. KIAPI 는 38개.
        sys.path.append(CURRENT_PATH)
        import scenario_pose
        root = os.path.join(scenario_pose.SCENARIO_ROOT, args.site)
        try:
            ids = sorted(n[len('scenario_'):] for n in os.listdir(root)
                         if n.startswith('scenario_'))
        except OSError as e:
            log(f'! {root} 를 읽을 수 없다: {e}')
    if args.list:
        ids += load_scenario_list(args.list)
    if args.scenario_id:
        ids += list(args.scenario_id)
    if args.from_scenario_pose:
        ids.append(args.from_scenario_pose)

    items = []
    if ids:
        # 26.09.09 _ YSH _ 하드코딩 대신 실제 로드 맵과 비교한다 (원래 줄은 아래 주석)
        loaded = loaded_map_site()
        if loaded and args.site != loaded:
            log(f'!! --site {args.site} 인데 Autoware 가 띄운 맵은 {loaded} 다.')
            log(f'!! 전부 ROUTE_NOT_SET 이 되고, cue_server 는 --max-route-failures(기본 2)')
            log(f'!! 만에 DEGRADED_HOLD 로 배치를 멈춘다. --site {loaded} 를 쓰거나 맵을 바꿀 것.')
        elif not loaded:
            log(f'   (로드된 맵을 확인하지 못했다 — e2e_simulator 의 map_path 를 못 읽음)')
        # if args.site != ON_MAP_SITE:
        #     log(f'!! --site {args.site} 는 현재 로드 맵(Kiapi_r1s60) 밖이다.')
        #     log(f'!! 전부 ROUTE_NOT_SET 이 되고, cue_server 는 --max-route-failures(기본 2)')
        #     log(f'!! 만에 DEGRADED_HOLD 로 배치를 멈춘다. --site {ON_MAP_SITE} 를 쓸 것.')
        for sid in ids:
            try:
                start, goal, length = scenario_poses(sid, args.site)
            except Exception as e:
                log(f'! 시나리오 {sid} 해석 실패 — 건너뛴다: {e}')
                continue
            items.append((f'scenario_{sid}', start, goal, length))
    else:
        # 자유 라벨 + 명시적 --start/--goal
        for name in (args.scenario or ['cue_sim_run']):
            items.append((name, args.start, args.goal, None))
    return items


# 26.11.10 _ YSH _ 신규 — --custom 인자 해석
CUSTOM_UNSET = object()


def parse_custom_specs(specs, path=None):
    """'WHEN:TYPE:VALUE' 들 → [{'base': 'r'|'c'|'e', 'at': 초, 'type': str, 'value': 값}].

    WHEN   S 또는 r+S (READY 를 받은 뒤 S 초) · c+S (SendCue 응답 뒤) · e+S (END 를 보낸 뒤)
    TYPE   그대로 Type 에 넣는다 (예 dropout)
    VALUE  JSON 글 (예 [0.75,0.5]) · clear(글자 "clear") · unset(Value 없음) · null · nan · 그 밖의 글자는 글자 그대로
    """
    lines = list(specs or [])
    if path:
        with open(os.path.expanduser(path), encoding='utf-8') as f:
            lines += [ln.strip() for ln in f if ln.strip() and not ln.strip().startswith('#')]
    out = []
    for spec in lines:
        parts = spec.split(':', 2)
        if len(parts) != 3:
            raise ValueError(f'--custom 은 WHEN:TYPE:VALUE 꼴이다: {spec!r}')
        when, type_str, val = (p.strip() for p in parts)
        base = 'r'
        if when[:2] in ('r+', 'c+', 'e+'):
            base, when = when[0], when[2:]
        at = float(when)
        if val == 'unset':
            value = CUSTOM_UNSET
        elif val == 'nan':
            value = float('nan')
        elif val == 'clear':
            value = 'clear'
        else:
            try:
                value = json.loads(val)
            except ValueError:
                value = val
        out.append({'base': base, 'at': at, 'type': type_str, 'value': value})
    return out


def print_queue(items, v0):
    log(f'{"시나리오":<22} {"start (x, y, h°)":<28} {"goal (x, y, h°)":<28} {"길이":>8}')
    log('─' * 92)
    for name, s, g, length in items:
        sl = f'({s[0]:.1f}, {s[1]:.1f}, {s[2]:.1f})'
        gl = f'({g[0]:.1f}, {g[1]:.1f}, {g[2]:.1f})'
        ln = f'{length:.0f} m' if length else '-'
        log(f'{name:<22} {sl:<28} {gl:<28} {ln:>8}')
    log('─' * 92)
    log(f'총 {len(items)}개, v0={v0:.2f} m/s')


class Sim:
    def __init__(self, args):
        self.args = args
        # 26.10.01 _ YSH _ --ignore-ads-end 전달. 원문:
        # self.servicer = FakeMoraiServicer()
        self.servicer = FakeMoraiServicer(ignore_ads_end=getattr(self.args, "ignore_ads_end", False))
        self.server = None
        self.sent_log = []      # [(label, {start, goal, v0, ts})] — --verify 가 쓴다
        # 26.11.10 _ YSH _ 신규 — 외란 명령(--custom) 일정과 집계
        self.custom_items = parse_custom_specs(getattr(self.args, 'custom', None) or [],
                                               getattr(self.args, 'custom_file', None))
        self.custom_stats = {'sent': 0, 'success': 0, 'rejected': 0, 'error': 0, 'dropped': 0}
        self._custom_stop = None
        self._custom_thread = None
        self._custom_lock = threading.Lock()

    def serve(self):
        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=4),
                                  options=[('grpc.so_reuseport', 0)])
        cue_pb2_grpc.add_CueServiceServicer_to_server(self.servicer, self.server)
        bound = 0
        try:
            bound = self.server.add_insecure_port(self.args.listen)
        except RuntimeError as e:
            log(f'! 바인드 실패: {e}')
        if bound == 0:
            log(f'! {self.args.listen} 를 열 수 없다')
            return False
        self.server.start()
        log(f'가짜 MORAI 서버 대기: {self.args.listen}')
        return True

    def send_cue(self, scenario, start, goal, v0):
        req = cue_pb2.CueRequest(
            cue_type='', scenario_name=scenario, sender='morai_pc',
            start_x=start[0], start_y=start[1], start_rotation_z=start[2],
            goal_x=goal[0], goal_y=goal[1], goal_rotation_z=goal[2], v0_sv_mps=v0)
        with grpc.insecure_channel(self.args.ads) as ch:
            stub = cue_pb2_grpc.CueServiceStub(ch)
            res = stub.SendCue(req, timeout=10.0)
        log(f'→ SendCue({scenario!r}) success={res.success} msg={res.message!r}')
        return res

    def send_state(self, state):
        name = cue_pb2.TestState.Name(state)
        with grpc.insecure_channel(self.args.ads) as ch:
            stub = cue_pb2_grpc.CueServiceStub(ch)
            res = stub.SendState(cue_pb2.StateRequest(state=state, sender='morai_pc'),
                                 timeout=10.0)
        log(f'→ SendState({name}) success={res.success} msg={res.message!r}')
        return res

    def ping(self):
        with grpc.insecure_channel(self.args.ads) as ch:
            stub = cue_pb2_grpc.CueServiceStub(ch)
            res = stub.Ping(cue_pb2.PingRequest(), timeout=5.0)
        log(f'→ Ping alive={res.alive}')
        return res.alive

    # 26.11.10 _ YSH _ 신규 — 외란 명령 송신 (KATECH 시나리오 API 가 보내는 것과 같은 RPC).
    #   26.11.10 캡처: Type="dropout" Value=[0.75, 0.5] 로 켜고 Type="dropout" Value="clear" 로 끈다
    def send_custom(self, type_str, value):
        """value: 파이썬 값(JSON 으로 읽은 것) · CUSTOM_UNSET(Value 없음) · float('nan'). 반환 success(bool) 또는 None(RPC 오류)."""
        req = cue_pb2.CustomCommandRequest(Type=type_str)
        if value is CUSTOM_UNSET:
            shown = '(없음)'
        elif isinstance(value, float) and value != value:
            req.Value.number_value = value
            shown = 'NaN'
        else:
            json_format.ParseDict(value, req.Value)
            shown = json.dumps(value, ensure_ascii=False)
        t0 = time.time()
        try:
            with grpc.insecure_channel(self.args.ads) as ch:
                res = cue_pb2_grpc.CueServiceStub(ch).SendCustomCommand(req, timeout=5.0)
        except grpc.RpcError as e:
            with self._custom_lock:
                self.custom_stats['sent'] += 1
                self.custom_stats['error'] += 1
            log(f'! SendCustomCommand(Type={type_str!r}, Value={shown}) 실패: {e.code().name} {e.details()}')
            return None
        with self._custom_lock:
            self.custom_stats['sent'] += 1
            self.custom_stats['success' if res.success else 'rejected'] += 1
        log(f'→ SendCustomCommand(Type={type_str!r}, Value={shown}) success={res.success} '
            f'rtt_ms={(time.time() - t0) * 1e3:.1f}')
        return res.success

    def _custom_start(self):
        """SendCue 응답 직후 부른다. c+S(cue 뒤)·r+S(READY 뒤) 항목을 스레드 하나가 순서대로 보낸다."""
        items = [it for it in self.custom_items if it['base'] in ('c', 'r')]
        if not items:
            return
        stop = threading.Event()
        t_cue = time.time()
        ready = self.servicer.ready

        def loop():
            pend = sorted(items, key=lambda it: (0 if it['base'] == 'c' else 1, it['at']))
            t_ready = None
            for it in pend:
                if it['base'] == 'r' and t_ready is None:
                    while not stop.is_set() and not ready.wait(timeout=0.05):
                        pass
                    if stop.is_set():
                        break
                    t_ready = time.time()
                due = (t_cue if it['base'] == 'c' else t_ready) + it['at']
                if stop.wait(timeout=max(0.0, due - time.time())):
                    break
                it['sent'] = True
                self.send_custom(it['type'], it['value'])

        for it in items:
            it['sent'] = False
        self._custom_stop = stop
        self._custom_thread = threading.Thread(target=loop, name='custom-sched', daemon=True)
        self._custom_thread.start()

    def _custom_finish(self, after_end):
        """시나리오 끝. 아직 못 보낸 c+·r+ 항목은 버리고, END 를 보냈으면 e+S 항목을 보낸다."""
        if self._custom_thread is not None:
            self._custom_stop.set()
            self._custom_thread.join(timeout=6.0)
            self._custom_thread = None
            dropped = [it for it in self.custom_items if it['base'] in ('c', 'r') and not it.get('sent')]
            if dropped:
                with self._custom_lock:
                    self.custom_stats['dropped'] += len(dropped)
                log(f'  외란 명령 {len(dropped)}건은 보낼 때가 오기 전에 시나리오가 끝나 버렸다')
        if after_end:
            t_end = time.time()
            for it in sorted((it for it in self.custom_items if it['base'] == 'e'), key=lambda it: it['at']):
                time.sleep(max(0.0, t_end + it['at'] - time.time()))
                self.send_custom(it['type'], it['value'])
        if self.custom_items:
            c = self.custom_stats
            log(f"  CUSTOM 보냄 {c['sent']} · success {c['success']} · 거부 {c['rejected']} · 오류 {c['error']}"
                f" · 못 보냄 {c['dropped']}")

    # 26.11.10 _ YSH _ 신규 — one() 은 이제 얇은 겉이다. 본체(_one_core)가 어디서 끝나든 외란 일정을 정리한다
    def one(self, idx, scenario, start, goal, v0):
        result = None
        try:
            result = self._one_core(idx, scenario, start, goal, v0)
            return result
        finally:
            self._custom_finish(after_end=result in ('OK', 'NO_READY'))

    # ------------------------------------------------------------------ 시나리오 1회
    # 26.11.10 _ YSH _ 이름을 _one_core 로 (겉 one() 이 외란 일정을 정리한다). 원문: def one(self, idx, scenario, start, goal, v0):
    def _one_core(self, idx, scenario, start, goal, v0):
        a = self.args
        log('─' * 60)
        log(f'[{idx}] {scenario}')
        self.servicer.ready.clear()
        self.servicer.ads_end.clear()      # 26.10.01 _ YSH _ 앞 시나리오의 ADS END 를 들고 오지 않는다

        if a.bad_coords:
            bad = float('nan') if a.bad_coords == 'nan' else float('inf')
            start = (bad, start[1], start[2])
            log(f'  (fault) start_x={bad}')

        res = self.send_cue(scenario, start, goal, v0)
        if not res.success:
            log(f'  cue 거부됨 — 이 시나리오는 건너뛴다')
            return 'REJECTED'
        self._custom_start()      # 26.11.10 _ YSH _ 외란 명령 일정 시작 (--custom 이 없으면 아무것도 안 한다)

        if a.duplicate_cue == 'same':
            time.sleep(0.3)
            log('  (fault) 같은 cue 재전송')
            self.send_cue(scenario, start, goal, v0)
        elif a.duplicate_cue == 'diff':
            time.sleep(0.3)
            log('  (fault) 다른 cue 로 선점 시도')
            self.send_cue(scenario + '_preempt', (start[0] + 5, start[1], start[2]), goal, v0)

        if a.kill_peer:
            log('  (fault) READY 를 받을 서버를 내려 버린다')
            self.server.stop(grace=0).wait()

        if a.end_after is not None:
            log(f'  (fault) READY 를 기다리지 않고 {a.end_after:g}s 뒤 END 를 보낸다')
            time.sleep(a.end_after)
            self.send_state(cue_pb2.END)
            time.sleep(2.0)
            return 'END_EARLY'

        got = self.servicer.ready.wait(timeout=a.ready_timeout)
        if got:
            log(f'  READY 수신 (t={a.ready_timeout - 0:.0f}s 안)')
        else:
            log(f'  ! READY 를 {a.ready_timeout:g}s 안에 못 받았다')

        if a.send_running:
            self.send_state(cue_pb2.RUNNING)

        log(f'  시뮬레이션 {a.sim_time:g}s ...')
        # 26.10.01 _ YSH _ ADS 가 도착 END 를 보내면 일찍 끝낸다 (MORAI 에 바라는 동작). 끝난 뒤 END 는 종전대로 보낸다 —
        #   ADS 는 이미 정리한 런이라 END_WITHOUT_RUN 으로 무시한다. 원문:
        # time.sleep(a.sim_time)
        t_sim = time.time()
        if self.servicer.ads_end.wait(timeout=a.sim_time):
            log(f'  ADS END 수신 → 시나리오 조기 종료 ({time.time() - t_sim:.1f}s / {a.sim_time:g}s)')

        if a.no_end:
            log('  (fault) END 를 보내지 않는다 — 서버의 run-timeout 을 기다린다')
            return 'NO_END'
        self.send_state(cue_pb2.END)
        return 'OK' if got else 'NO_READY'

    def run(self):
        a = self.args
        results = []
        if a.end_only:
            log('(fault) run 없이 END 만 보낸다')
            self.send_state(cue_pb2.END)
            return results
        if a.rapid:
            log(f'(fault) cue {a.rapid}개를 연속으로 쏜다 (좌표는 --start/--goal 고정)')
            for i in range(a.rapid):
                try:
                    self.send_cue(f'rapid_{i:03d}', a.start, a.goal, a.v0)
                except grpc.RpcError as e:
                    log(f'  ! {e.code().name}')
                time.sleep(0.05)
            time.sleep(2.0)
            return results

        items = build_queue(a)
        if not items:
            log('! 돌릴 시나리오가 없다')
            return results
        queue = [it for _ in range(a.repeat) for it in items]
        print_queue(items, a.v0)
        if len(queue) > 5:
            per = 6 + a.sim_time + 13 + a.gap      # 설정 + 주행 + 정지수렴 + 간격
            log(f'  총 {len(queue)}회 × 약 {per:.0f}초 ≈ {len(queue) * per / 60:.0f}분 예상')
        for i, (label, start, goal, _len) in enumerate(queue, 1):
            sent = {'start': start, 'goal': goal, 'v0': a.v0, 'ts': time.time()}
            r = self.one(i, label, start, goal, a.v0)
            results.append((label, r))
            self.sent_log.append((label, sent))
            if i < len(queue):
                time.sleep(a.gap)
        return results


# ─────────────────────────────────────────────────────────────────────────────
# 검증 — 각 run 의 cue_run JSON 을 읽어 4항목을 판정한다
# ─────────────────────────────────────────────────────────────────────────────
TOL_M, TOL_DEG, STOP_V = 1.5, 8.0, 0.5


def _find_run_json(out_dir, label, after_ts):
    """보낸 시각 이후에 만들어진 같은 이름의 run JSON 을 고른다."""
    pat = os.path.join(os.path.expanduser(out_dir), 'cue_logs', f'{label}_*.json')
    best = None
    for p in glob.glob(pat):
        try:
            if os.path.getmtime(p) + 2.0 < after_ts:   # 여유 2초
                continue
            d = json.load(open(p, encoding='utf-8'))
        except Exception:
            continue
        if d.get('cue', {}).get('scenario_name') != label:
            continue
        if best is None or os.path.getmtime(p) > os.path.getmtime(best[0]):
            best = (p, d)
    return best


def _ts(events, code, phase=None):
    for e in events:
        if e.get('code') == code and (phase is None or e.get('phase') == phase):
            return e.get('t')
    return None


def verify_run(d, sent):
    """(항목명, PASS, 근거) 4개를 돌려준다. sent 는 우리가 보낸 값."""
    ev = d.get('events', [])
    cue = d.get('cue', {})
    tp = d.get('teleport') or {}
    bag = d.get('bag') or {}
    out = []

    # ① 타겟·위치 정보가 제대로 적용됐는가
    ok_xy = (abs(cue.get('start_x', 1e9) - sent['start'][0]) < 0.1
             and abs(cue.get('goal_x', 1e9) - sent['goal'][0]) < 0.1)
    err_m, err_deg = tp.get('err_m'), tp.get('err_deg')
    ok_tp = err_m is not None and err_m <= TOL_M and err_deg <= TOL_DEG
    ok_route = _ts(ev, 'SET', 'ROUTE') is not None
    ok_v = (d.get('velocity', {}).get('code') == 'VELOCITY_LIMIT_OK'
            and abs((d.get('velocity', {}).get('effective') or -1) - sent['v0']) < 0.1)
    out.append(('1.타겟·위치', ok_xy and ok_tp and ok_route and ok_v,
                f"수신좌표={'일치' if ok_xy else '불일치'} "
                f"텔레포트오차={err_m if err_m is not None else '없음'}m/"
                f"{err_deg if err_deg is not None else '-'}° "
                f"route={'SET' if ok_route else '실패'} "
                f"속도={d.get('velocity', {}).get('effective')}"))

    # ② 시작 시 bag 이 따지고, 끝낼 때 저장되는가
    t_start, t_eng = _ts(ev, 'BAG_START'), _ts(ev, 'ENGAGED')
    t_stop, t_bstop = _ts(ev, 'STOP_OK'), _ts(ev, 'BAG_STOP')
    ok_order_s = bool(t_start) and (not t_eng or t_start < t_eng)
    ok_order_e = bool(t_bstop) and (not t_stop or t_bstop > t_stop)
    ok_file = bag.get('exists') is True
    ok_msgs = (bag.get('total_msgs') or 0) > 0 and not (bag.get('empty_topics') or '')
    out.append(('2.bag 시작·저장', ok_order_s and ok_order_e and ok_file and ok_msgs,
                f"시작<engage={'O' if ok_order_s else 'X'} "
                f"종료>STOP={'O' if ok_order_e else 'X'} "
                f"metadata={'있음' if ok_file else '없음'} "
                f"msgs={bag.get('total_msgs')} 빈토픽='{bag.get('empty_topics')}'"))

    # ③ 시작 신호(READY)를 제대로 주는가 — 준비가 끝난 뒤에 나가야 한다
    rd = next((e for e in ev if e.get('code') == 'READY' and e.get('phase') == 'STATE_TX'), None)
    t_v = _ts(ev, 'VELOCITY_LIMIT_OK', 'VLIMIT')
    ok_after = bool(rd) and all(t is None or rd['t'] > t for t in (t_start, _ts(ev, 'SET', 'ROUTE'), t_v))
    out.append(('3.READY 신호', bool(rd) and rd.get('accepted') is True and ok_after,
                f"전송={'O' if rd else 'X'} accepted={rd.get('accepted') if rd else '-'} "
                f"rtt={rd.get('rtt_ms') if rd else '-'}ms "
                f"순서={'준비후' if ok_after else '뒤바뀜'}"))

    # ④ STOP 신호에 맞게 서는가
    so = next((e for e in ev if e.get('code') == 'STOP_OK'), None)
    sv = (d.get('stop') or {}).get('speed_mps')
    ok_stop = bool(so) and str(so.get('op_mode')) == '1' and sv is not None and abs(sv) < STOP_V
    ss = (d.get('stop') or {}).get('settle_s')
    out.append(('4.STOP 정지', ok_stop,
                f"op_mode={so.get('op_mode') if so else '-'} 정지속도={sv} "
                f"정지까지={ss}s end_source={d.get('timing', {}).get('end_source')}"))
    return out


def print_verify(results):
    """results: [(label, sent, run_json or None)]"""
    log('=' * 96)
    log('검증 결과')
    log('=' * 96)
    allok = True
    for label, sent, d in results:
        if d is None:
            log(f'  {label:<18} run 기록을 못 찾음 — FAIL'); allok = False; continue
        rows = verify_run(d, sent)
        st = d.get('result', {}).get('status')
        log(f'  {label:<18} status={st} end={d.get("timing", {}).get("end_source")}')
        for name, ok, why in rows:
            allok = allok and ok
            log(f'      {"PASS" if ok else "FAIL"}  {name:<14} {why}')
    log('=' * 96)
    log('전체: ' + ('PASS' if allok else 'FAIL'))
    return allok


def check_manifest(out_dir, expect):
    """cue_manifest.csv 에 expect 행 이상 있는지. CI 용."""
    path = os.path.join(os.path.expanduser(out_dir), 'cue_manifest.csv')
    if not os.path.isfile(path):
        log(f'! {path} 가 없다')
        return False
    with open(path, newline='', encoding='utf-8') as f:
        rows = list(csv.DictReader(f))
    log(f'cue_manifest.csv 행 {len(rows)}개 (기대 {expect}개 이상)')
    for r in rows[-expect:] if expect else rows:
        log(f'  {r["run_id"]} {r["scenario"]:<24} {r["status"]:<12} '
            f'fc={r["failure_code"] or "-":<18} end={r["end_source"] or "-"}')
    return len(rows) >= expect


def main():
    ap = argparse.ArgumentParser(description='cue_server.py 검증용 가짜 MORAI PC')
    ap.add_argument('--ads', default='127.0.0.1:50052', help='ADS PC 의 CueService')
    ap.add_argument('--listen', default='0.0.0.0:50152', help='READY 를 받을 우리 주소')
    ap.add_argument('--scenario', action='append', help='시나리오 이름 (반복 가능)')
    ap.add_argument('--start', type=parse_triple, default=(809.3, -909.3, -52.9))
    ap.add_argument('--goal', type=parse_triple, default=(886.1, -1005.8, -51.2))
    ap.add_argument('--v0', type=float, default=8.0)
    ap.add_argument('--sim-time', type=float, default=10.0)
    ap.add_argument('--repeat', type=int, default=1)
    ap.add_argument('--gap', type=float, default=2.0)
    ap.add_argument('--ready-timeout', type=float, default=120.0)
    ap.add_argument('--from-scenario-pose', help='실제 CSV 에서 start/goal 을 읽는다 (예: 1_11)')
    ap.add_argument('--scenario-id', action='append',
                    help='시나리오 id. 반복 가능하며 각자 자기 start/goal 을 CSV 에서 읽는다')
    ap.add_argument('--list', help='시나리오 id 목록 파일 (한 줄에 하나, # 주석 허용)')
    ap.add_argument('--preset', choices=['smoke5', 'katech7', 'all'],
                    help='smoke5=길이 범위를 덮는 5개, katech7=인수 대상 7개, all=--site 의 전부')
    ap.add_argument('--dry-list', action='store_true',
                    help='해석된 좌표·길이만 출력하고 끝 (ROS·Autoware 불필요)')
    ap.add_argument('--site', default='KIAPI')
    ap.add_argument('--send-running', action='store_true',
                    help='RUNNING 을 보낸다 (KATECH 순서에는 없다)')

    ap.add_argument('--ping', action='store_true', help='Ping 만 해 보고 끝')
    ap.add_argument('--check', type=int, metavar='N', help='cue_manifest.csv 에 N행 이상 있는지 확인')
    ap.add_argument('--verify', action='store_true',
                    help='끝나고 각 run 의 기록을 읽어 4항목 PASS/FAIL 판정')
    ap.add_argument('--verify-only', type=int, metavar='N',
                    help='cue 를 보내지 않고 최근 N개 run 기록만 읽어 판정 (진짜 MORAI 로 돌렸을 때)')
    ap.add_argument('--verify-wait', type=float, default=40.0,
                    help='run 기록 파일이 나올 때까지 기다릴 최대 시간 [s]')
    # 26.11.09 _ YSH _ 홈 정리 — 새 위치. 원문:
    # ap.add_argument('--out', default='~/DAQ_Bag')
    # 26.11.10 _ YSH _ 11_(외란) DAQ 폴더. 원문:
    # ap.add_argument('--out', default='~/01_DAQ_Data/DAQ_Bag')
    ap.add_argument('--out', default='~/01_DAQ_Data/DAQ_Bag_Disturbance')
    # 26.11.10 _ YSH _ 외란 명령 (KATECH 시나리오 API 대역). 26.11.10 캡처를 흉내 내려면
    #   --send-running --custom '1:dropout:[0.75,0.5]' --custom '4:dropout:clear'
    ap.add_argument('--custom', action='append', default=[], metavar='WHEN:TYPE:VALUE',
                    help="외란 명령 SendCustomCommand 를 보낸다 (여러 번 줄 수 있다). WHEN = S 또는 r+S(READY 뒤 S초) · "
                         "c+S(cue 뒤) · e+S(END 뒤). VALUE = JSON · clear · unset · null · nan. 예: 1:dropout:[0.75,0.5]")
    ap.add_argument('--custom-file', default=None, help='--custom 을 한 줄에 하나씩 적은 파일 (# 은 주석)')

    # fault injection
    ap.add_argument('--no-end', action='store_true')
    # 26.10.01 _ YSH _ ADS 의 도착 END 를 거부하고 --sim-time 끝까지 돈다 (END 처리를 구현하기 전의 MORAI 흉내)
    ap.add_argument('--ignore-ads-end', action='store_true',
                    help='ADS 가 보낸 END 를 success=false 로 거부하고 시뮬레이션을 끝까지 돈다')
    ap.add_argument('--end-after', type=float, metavar='S',
                    help='cue 후 S초 뒤에 END 를 보낸다 (READY 를 안 기다림 — 세팅 중 END 재현)')
    ap.add_argument('--end-only', action='store_true')
    ap.add_argument('--duplicate-cue', choices=['same', 'diff'])
    ap.add_argument('--bad-coords', choices=['nan', 'inf'])
    ap.add_argument('--kill-peer', action='store_true')
    ap.add_argument('--rapid', type=int)
    args = ap.parse_args()

    if args.verify_only:
        pat = os.path.join(os.path.expanduser(args.out), 'cue_logs', '*.json')
        files = sorted(glob.glob(pat), key=os.path.getmtime)[-args.verify_only:]
        if not files:
            log(f'! {pat} 에 run 기록이 없다'); return 1
        rows = []
        for f in files:
            try:
                d = json.load(open(f, encoding='utf-8'))
            except Exception as e:
                log(f'! {os.path.basename(f)} 읽기 실패: {e}'); continue
            c = d.get('cue', {})
            sent = {'start': (c.get('start_x'), c.get('start_y'), c.get('start_rotation_z')),
                    'goal': (c.get('goal_x'), c.get('goal_y'), c.get('goal_rotation_z')),
                    'v0': c.get('v0_sv_mps')}
            rows.append((c.get('scenario_name', os.path.basename(f)), sent, d))
        return 0 if print_verify(rows) else 1

    if args.dry_list:
        items = build_queue(args)
        if not items:
            log('! 해석된 시나리오가 없다')
            return 1
        print_queue(items, args.v0)
        return 0

    sim = Sim(args)
    if args.ping:
        return 0 if sim.ping() else 1
    # --check 만 주면 "돌리지 말고 manifest 만 확인" 이다. 시나리오 소스가 하나라도
    # 있으면 실행이 우선 — 새 인자(--preset 등)를 여기 빠뜨리면 조용히 실행을 건너뛴다.
    has_work = any([args.scenario, args.scenario_id, args.list, args.preset,
                    args.from_scenario_pose, args.rapid, args.end_only])
    if args.check is not None and not has_work:
        return 0 if check_manifest(args.out, args.check) else 1

    if not sim.serve():
        return 1
    try:
        results = sim.run()
        log('─' * 60)
        for name, r in results:
            log(f'  {name:<28} {r}')
        if args.verify:
            # 서버가 run JSON 을 쓰는 시점은 END 수신보다 한참 뒤다 — STOP 수렴 대기
            # (최대 15초) + bag 마감(tail + SIGINT) 이 끝나야 쓴다. 고정 3초로 기다리면
            # 마지막 run 만 못 찾는다(실측: 파일이 12초 늦게 나왔다). 나올 때까지 기다린다.
            rows = []
            for label, sent in sim.sent_log:
                hit = None
                deadline = time.time() + args.verify_wait
                while time.time() < deadline:
                    hit = _find_run_json(args.out, label, sent['ts'])
                    if hit is not None:
                        break
                    time.sleep(1.0)
                if hit is None:
                    log(f'  ! {label}: run 기록을 {args.verify_wait:g}s 기다렸지만 못 찾음')
                rows.append((label, sent, hit[1] if hit else None))
            if not print_verify(rows):
                return 1
        if args.check is not None:
            time.sleep(2.0)
            return 0 if check_manifest(args.out, args.check) else 1
    finally:
        if sim.server is not None:
            try:
                sim.server.stop(grace=0.5).wait()
            except Exception:
                pass
    return 0


if __name__ == '__main__':
    sys.exit(main() or 0)
