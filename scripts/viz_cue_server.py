#!/usr/bin/env python3
# 신규 — 26.11.13 _ YSH _ 외란 시각화 전용 gRPC 수신기 (KATECH CueService) — 연차보고서 1차.
#   KATECH 이 MORAI 에서 Ego 를 직접 조종한다. 우리는 Autoware 없이 외란만 걸어 RViz 로 보여 준다.
#   11_ cue_server.py 는 Autoware(AD API)가 없으면 런 준비에서 멈춰 외란 켜기를 기록만 한다 → 쓸 수 없다.
#   이 수신기는 받은 외란을 곧바로 브리지(/morai_viz_bridge/set_disturbance)에 건다.
#
#   SendCue(시나리오 시작)   → 즉시 success · 새 시드로 clear · bag 녹화 시작 · READY 자동 송신(--ready auto)
#   SendState(END)          → clear · 2 s 뒤 녹화 정지
#   SendCustomCommand       → 11_ disturbance/command.py 로 해석 → 브리지에 건다 (success = 읽을 수 있고 받는 종류)
#   기록: <data>/cue_logs/viz_<시각>.dist.jsonl (11_ diag/dist_watch.py --log-dir 로 그대로 보인다) · 화면 로그
#   RViz 글자판용 상태: /viz/cue_state (std_msgs/String, JSON, latched)
#
#   26.11.13 _ YSH _ 스텁(cue/)·해석기(disturbance/)는 11_ 에서 13_ scripts 로 복사한 것을 쓴다 (COPIED_FROM_11.txt). 원문:
#   11_ 의 스텁(cue/)·해석기(disturbance/)를 읽어 쓴다 — 11_ 파일은 고치지 않는다.
import argparse
import hashlib
import json
import os
import queue
import secrets
import signal
import subprocess
import sys
import threading
import time
from concurrent import futures
from datetime import datetime

# 26.11.13 _ YSH _ 13_ 이 11_ 없이 돌게 cue/·disturbance/ 를 13_ scripts 로 복사했다 (COPIED_FROM_11.txt). 원문:
# RIG = os.environ.get('VIZ_RIG_SCRIPTS',
#                      '/home/autoware-morai/11_Disturbance-Autoware/MR_Morai-Autoware-Bridge_Disturbance/scripts')
RIG = os.environ.get('VIZ_RIG_SCRIPTS', os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, RIG)
sys.path.insert(0, os.path.join(RIG, 'cue'))

import grpc                                    # noqa: E402
import cue_pb2                                 # noqa: E402
import cue_pb2_grpc                            # noqa: E402
from disturbance import command as dcmd        # noqa: E402

import rclpy                                   # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node                    # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from std_msgs.msg import String                # noqa: E402
from morai_ros2_msgs.srv import SetDisturbance  # noqa: E402

SEED_MAX = 999999999999999          # 브리지 MRDIST-1 기본 시드 상한
CALL_TIMEOUT_S = 1.0                # 브리지 서비스 한 번 (cue_server 와 같다)
QUEUE_MAX = 64
REAL_MORAI_PORT = 7789
KATECH_PROTO = os.path.expanduser('~/02_Mission/2_Disturbance/katech_ws/cue.proto')
# 26.11.13 _ YSH _ Autoware RViz 기본 화면(SignalDisplay)을 다시 보기에서도 그리게 /vehicle/status/* 를 더했다.
#   Prediction·GT·상태 글자는 녹화하지 않는다 (다시 보기 때 viz_autoware.py 가 다시 만든다). 원문:
# REC_TOPICS = [
#     '/daq/object_disturbance_status', '/daq/disturbance', '/daq/object_status', '/daq/object_status_disturbed',
#     '/daq/ego_status', '/perception/object_recognition/tracking/objects', '/localization/kinematic_state',
#     '/tf', '/viz/cue_state',
# ]
REC_TOPICS = [
    '/daq/object_disturbance_status', '/daq/disturbance', '/daq/object_status', '/daq/object_status_disturbed',
    '/daq/ego_status', '/perception/object_recognition/tracking/objects', '/localization/kinematic_state',
    '/tf', '/viz/cue_state',
    '/vehicle/status/velocity_status', '/vehicle/status/steering_status', '/vehicle/status/gear_status',
    '/vehicle/status/turn_indicators_status', '/vehicle/status/hazard_lights_status', '/vehicle/status/control_mode',
]
ADHOC_IDLE_S = 30.0                 # 시나리오 없이 명령만 왔을 때 마지막 명령 뒤 이만큼 지나면 녹화를 닫는다


def now_iso():
    return datetime.now().isoformat(timespec='milliseconds')


def log(msg):
    print(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] VIZ  {msg}', flush=True)


# ───────────────────────────────────────────────────────────── ROS 쪽 (브리지 서비스·상태 토픽)
class VizNode(Node):
    def __init__(self, bridge_node):
        super().__init__('viz_cue_server')
        self.srv_name = f'/{bridge_node}/set_disturbance'
        self.cli = self.create_client(SetDisturbance, self.srv_name)
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_state = self.create_publisher(String, '/viz/cue_state', latched)

    def ready(self):
        try:
            return bool(self.cli.service_is_ready())
        except Exception:
            return False

    @staticmethod
    def _request(trigger, run, base_seed, source, seq):
        req = SetDisturbance.Request()
        req.trigger = str(trigger)
        req.run = str(run or '')
        req.base_seed = int(base_seed or 0)
        req.source = str(source or '')[:64]
        req.seq = int(seq or 0)
        return req

    def call(self, trigger, run, base_seed, source, seq, timeout=CALL_TIMEOUT_S):
        """(응답 dict | None, 오류 코드). executor 가 다른 스레드에서 돌므로 future 를 폴링한다 (cue_server 와 같은 방식)."""
        try:
            if not self.ready():
                return None, 'BRIDGE_DOWN'
            fut = self.cli.call_async(self._request(trigger, run, base_seed, source, seq))
            t_end = time.monotonic() + timeout
            while time.monotonic() < t_end:
                if fut.done():
                    r = fut.result()
                    return {'ok': bool(r.ok), 'message': str(r.message),
                            'applied_time_ns': int(r.applied_time.sec) * 1000000000 + int(r.applied_time.nanosec),
                            'run': str(r.run), 'base_seed': int(r.base_seed)}, ''
                time.sleep(0.002)
            fut.cancel()
            return None, 'TIMEOUT'
        except Exception as e:
            return None, f'EXCEPTION:{type(e).__name__}'

    def call_nowait(self, trigger, run, base_seed, source, seq):
        try:
            if self.ready():
                self.cli.call_async(self._request(trigger, run, base_seed, source, seq))
                return True
        except Exception:
            pass
        return False

    def publish_state(self, obj):
        try:
            self.pub_state.publish(String(data=json.dumps(obj, ensure_ascii=False)))
        except Exception:
            pass


# ───────────────────────────────────────────────────────────── bag 녹화
class Recorder:
    """ros2 bag record 한 개를 띄우고 닫는다. 예외를 내지 않는다."""

    def __init__(self, bag_root, log_dir):
        self.bag_root = bag_root
        self.log_dir = log_dir
        self.lock = threading.Lock()
        self.cur = None            # {'proc', 'path', 'name', 'kind', 't0', 'meta'}

    def active(self):
        with self.lock:
            return self.cur is not None

    def kind(self):
        with self.lock:
            return self.cur['kind'] if self.cur else ''

    def start(self, name, kind, meta):
        self.stop()
        safe = ''.join(ch if ch.isalnum() or ch in '-_.' else '_' for ch in name)[:80] or 'viz'
        path = os.path.join(self.bag_root, f'{safe}_{datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}')
        try:
            os.makedirs(self.bag_root, exist_ok=True)
            logf = open(os.path.join(self.log_dir, os.path.basename(path) + '_record.log'), 'w')
            proc = subprocess.Popen(['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', path] + REC_TOPICS,
                                    stdin=subprocess.DEVNULL, stdout=logf, stderr=subprocess.STDOUT,
                                    start_new_session=True)
            logf.close()
        except Exception as e:
            log(f'! 녹화를 시작하지 못했다: {type(e).__name__}: {e}')
            return ''
        with self.lock:
            self.cur = {'proc': proc, 'path': path, 'name': os.path.basename(path), 'kind': kind,
                        't0': time.time(), 'meta': meta}
        log(f'● 녹화 시작 {os.path.basename(path)} ({kind})')
        return os.path.basename(path)

    def name(self):
        with self.lock:
            return self.cur['name'] if self.cur else ''

    def stop(self, meta_update=None, only=None):
        """녹화를 닫는다. only 를 주면 그 이름의 녹화일 때만 (늦게 온 타이머가 새 녹화를 닫지 않게)."""
        with self.lock:
            if self.cur is None or (only is not None and self.cur['name'] != only):
                return
            cur, self.cur = self.cur, None
        proc = cur['proc']
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except Exception:
            pass
        try:
            proc.wait(timeout=10.0)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5.0)
            except Exception:
                pass
        meta = dict(cur['meta'] or {})
        if meta_update:
            meta.update(meta_update)
        meta.update({'bag': cur['name'], 'kind': cur['kind'], 'record_start': datetime.fromtimestamp(cur['t0']).isoformat(),
                     'record_end': now_iso(), 'topics': REC_TOPICS})
        try:
            if os.path.isdir(cur['path']):
                with open(os.path.join(cur['path'], 'viz_run.json'), 'w', encoding='utf-8') as f:
                    json.dump(meta, f, ensure_ascii=False, indent=1)
        except Exception as e:
            log(f'! viz_run.json 을 쓰지 못했다: {e}')
        log(f'■ 녹화 정지 {cur["name"]} ({time.time() - cur["t0"]:.0f} s)')


# ───────────────────────────────────────────────────────────── 수신기 본체
class VizServer:
    def __init__(self, args, node):
        self.args = args
        self.node = node
        self.lock = threading.Lock()
        self.io_lock = threading.Lock()
        self.seq = 0
        self.scn = None                 # 지금 시나리오
        self.last_cmd = None
        self.last_rx_t = 0.0
        self.counts = {'received': 0, 'forwarded': 0, 'applied': 0, 'failed': 0, 'rejected': 0}
        self.q = queue.Queue(maxsize=QUEUE_MAX)
        self.stop_ev = threading.Event()
        data = os.path.expanduser(args.data)
        self.cue_logs = os.path.join(data, 'cue_logs')
        self.logs = os.path.join(data, 'logs')
        os.makedirs(self.cue_logs, exist_ok=True)
        os.makedirs(self.logs, exist_ok=True)
        self.sidecar = os.path.join(self.cue_logs, f'viz_{datetime.now().strftime("%Y-%m-%d-%H-%M-%S")}.dist.jsonl')
        self.rec = Recorder(os.path.join(data, 'bags'), self.logs) if args.record != 'off' else None
        self.worker = threading.Thread(target=self._worker, name='viz-fwd', daemon=True)
        self.worker.start()
        self.ticker = threading.Thread(target=self._tick, name='viz-tick', daemon=True)
        self.ticker.start()

    # ── 기록
    def _jsonl(self, obj):
        try:
            line = json.dumps(obj, ensure_ascii=False, allow_nan=False)
        except ValueError:
            line = json.dumps({k: (str(v) if isinstance(v, float) else v) for k, v in obj.items()}, ensure_ascii=False)
        with self.io_lock:
            try:
                with open(self.sidecar, 'a', encoding='utf-8') as f:
                    f.write(line + '\n')
            except Exception:
                pass

    def publish_state(self):
        with self.lock:
            s = self.scn
            obj = {'scenario': s['name'] if s else '', 'state': s['state'] if s else 'NO_SCENARIO',
                   'tag': s['tag'] if s else '', 'base_seed': s['seed'] if s else 0,
                   'ready': s.get('ready', '') if s else '', 'bag': s.get('bag', '') if s else '',
                   'last_cmd': self.last_cmd, 'counts': dict(self.counts), 't': now_iso()}
        self.node.publish_state(obj)

    # ── 브리지로 넘기기 (도착 순서)
    def enqueue(self, trigger, source, seq, run, seed, t_rx, rec=None, done=None):
        item = {'trigger': trigger, 'source': source, 'seq': seq, 'run': run, 'seed': seed, 't_rx': t_rx,
                't_q': time.monotonic(), 'rec': rec, 'done': done}
        try:
            self.q.put_nowait(item)
            return True
        except queue.Full:
            log(f'✘ 전달 줄이 가득 찼다 — #{seq} {trigger} 를 버린다')
            return False

    def _worker(self):
        while True:
            item = self.q.get()
            if item is None:
                return
            try:
                self._forward(item)
            except Exception as e:
                log(f'! 전달 스레드 예외: {type(e).__name__}: {e}')
            finally:
                if item.get('done') is not None:
                    item['done'].set()

    def _forward(self, item):
        t0 = time.monotonic()
        resp, err = self.node.call(item['trigger'], item['run'], item['seed'], item['source'], item['seq'])
        call_ms = round((time.monotonic() - t0) * 1e3, 1)
        ok = bool(resp and resp.get('ok'))
        why = '' if ok else (err or 'REJECTED')
        apply = {'ok': ok, 'why': why, 'message': (resp.get('message', '') if resp else '')[:200],
                 'applied_time_ns': resp.get('applied_time_ns') if ok else None,
                 'rx_to_applied_ms': (round(resp['applied_time_ns'] / 1e6 - item['t_rx'] * 1e3, 1) if ok else None),
                 'queue_ms': round((t0 - item['t_q']) * 1e3, 1), 'call_ms': call_ms,
                 'run': resp.get('run') if resp else None, 'base_seed': resp.get('base_seed') if resp else None}
        self._jsonl({'ev': 'fwd', 't': now_iso(), 'seq': item['seq'], 'source': item['source'],
                     'trigger': item['trigger'], 'run': item['run'], 'base_seed_sent': item['seed'], 'apply': apply})
        src = item['source']
        if src.startswith('grpc'):
            with self.lock:
                self.counts['applied' if ok else 'failed'] += 1
                if self.last_cmd and self.last_cmd.get('seq') == item['seq']:
                    self.last_cmd.update({'applied': ok, 'why': why, 'bridge': apply['message'],
                                          'rx_to_applied_ms': apply['rx_to_applied_ms']})
                if self.scn is not None and item['rec'] is not None:
                    item['rec']['apply'] = apply
        if ok:
            what = '자동 clear (' + src[5:] + ')' if src.startswith('auto:') else f'#{item["seq"]}'
            log(f'✔ 걸림 {what}  {item["trigger"]}  · 받은 뒤 {apply["rx_to_applied_ms"]} ms · 브리지 "{apply["message"]}"')
        else:
            log(f'✘ 못 걸림 #{item["seq"]}  {item["trigger"]}  · {why} · 브리지 "{apply["message"]}"')
        self.publish_state()

    # ── READY
    def send_ready(self, why):
        target = self.args.morai_cue
        for attempt in range(1, 4):
            try:
                t0 = time.time()
                with grpc.insecure_channel(target) as ch:
                    res = cue_pb2_grpc.CueServiceStub(ch).SendState(
                        cue_pb2.StateRequest(state=cue_pb2.READY, sender=self.args.sender_name), timeout=5.0)
                ok = bool(getattr(res, 'success', True))
                log(f'→ READY → {target} ({why}) accepted={ok} · {int((time.time() - t0) * 1000)} ms')
                self._set_ready('sent' if ok else 'refused')
                return True
            except grpc.RpcError as e:
                log(f'! READY 송신 실패 {attempt}/3 → {target}: {e.code().name} {e.details()}')
            except Exception as e:
                log(f'! READY 송신 실패 {attempt}/3 → {target}: {e}')
            time.sleep(0.5)
        self._set_ready('failed')
        return False

    def _set_ready(self, v):
        with self.lock:
            if self.scn is not None:
                self.scn['ready'] = v
        self.publish_state()

    # ── 시나리오 경계
    def begin_scenario(self, req, peer):
        name = (req.scenario_name or '').strip() or 'scenario'
        tag = f'viz_{name}_{datetime.now().strftime("%H%M%S")}'
        seed = self.args.seed or (secrets.randbelow(SEED_MAX) + 1)
        self.end_scenario('new_cue', immediate=True)
        with self.lock:
            self.seq += 1
            seq = self.seq
            self.scn = {'name': name, 'tag': tag, 'seed': seed, 'state': 'CUE', 't_cue': now_iso(), 'peer': peer,
                        'start': [req.start_x, req.start_y, req.start_rotation_z],
                        'goal': [req.goal_x, req.goal_y, req.goal_rotation_z], 'v0_mps': req.v0_sv_mps,
                        'ready': '', 'bag': '', 'commands': [], 'closed': False}
        log(f'▶ 시나리오 {name} · start ({req.start_x:.1f}, {req.start_y:.1f}, {req.start_rotation_z:.1f}°) '
            f'goal ({req.goal_x:.1f}, {req.goal_y:.1f}) · v0 {req.v0_sv_mps:.2f} m/s · 시드 {seed} · peer {peer}')
        done = threading.Event()
        self.enqueue('disturbance, clear', 'auto:begin', seq, tag, seed, time.time(), done=done)
        if self.rec is not None and self.args.record == 'scenario':
            bag = self.rec.start(name, 'scenario', self._meta())
            with self.lock:
                self.scn['bag'] = bag
        self.publish_state()
        if self.args.ready == 'auto':
            threading.Thread(target=self._ready_after, args=(done, name), daemon=True).start()
        elif self.args.ready == 'key':
            log('  READY 를 보내려면 이 터미널에서 Enter')
        return True

    def _ready_after(self, done, name):
        done.wait(2.0)
        time.sleep(max(0.0, self.args.ready_delay))
        with self.lock:
            same = self.scn is not None and self.scn['name'] == name and not self.scn['closed']
        if same:
            self.send_ready(f'시나리오 {name}')

    def end_scenario(self, why, immediate=False):
        with self.lock:
            s = self.scn
            if s is None or s['closed']:
                return
            s['closed'] = True
            s['state'] = 'END' if why == 'end' else s['state']
            self.seq += 1
            seq = self.seq
            tag, seed = s['tag'], s['seed']
        self.enqueue('disturbance, clear', 'auto:reset', seq, tag, seed, time.time())
        log(f'■ 시나리오 {s["name"]} 끝 ({why})')
        if self.rec is not None and self.rec.kind() == 'scenario':
            meta = self._meta()
            if immediate:
                self.rec.stop(meta)
            else:
                threading.Timer(2.0, self.rec.stop, args=(meta, self.rec.name())).start()
        self.publish_state()

    def _meta(self):
        with self.lock:
            s = dict(self.scn) if self.scn else {}
            return {'scenario': s.get('name', ''), 'tag': s.get('tag', ''), 'base_seed': s.get('seed', 0),
                    'start': s.get('start'), 'goal': s.get('goal'), 'v0_mps': s.get('v0_mps'), 't_cue': s.get('t_cue'),
                    'ready': s.get('ready', ''), 'commands': list(s.get('commands', [])),
                    'counts': dict(self.counts), 'seed_algorithm': 'MRDIST-1'}

    # ── 외란 명령
    def custom(self, req, peer):
        t_rx = time.time()
        try:
            has_value = req.HasField('Value')
        except Exception:
            has_value = False
        dec = dcmd.decode(getattr(req, 'Type', ''), req.Value if has_value else None)
        if not dec.ok:
            decision, why = 'reject', dec.reason
        elif not dec.supported:
            decision, why = 'reject', 'UNSUPPORTED_KIND'
        else:
            decision, why = 'forward', ''
        success = decision == 'forward'
        with self.lock:
            self.seq += 1
            seq = self.seq
            self.last_rx_t = t_rx
            self.counts['received'] += 1
            self.counts['forwarded' if success else 'rejected'] += 1
            s = self.scn
            tag, seed = (s['tag'], s['seed']) if s is not None else ('', 0)
            rec = {'seq': seq, 't': now_iso(), 'ref': dec.ref[:400], 'trigger': dec.trigger, 'ok': dec.ok,
                   'reason': dec.reason, 'detail': dec.detail[:120], 'decision': decision, 'why': why,
                   'success': success, 'peer': peer, 'apply': None}
            if s is not None:
                s['commands'].append(rec)
            self.last_cmd = {'seq': seq, 'ref': dec.ref[:120], 'trigger': dec.trigger, 'decision': decision,
                             'why': why, 'applied': None, 'rx_to_applied_ms': None, 't': now_iso()}
        self._jsonl({'ev': 'rx', 't': now_iso(), 't_unix': round(t_rx, 6), 'seq': seq, 'window': 'VIZ',
                     'raw': dec.raw, 'ref': dec.ref[:400], 'ok': dec.ok, 'reason': dec.reason, 'detail': dec.detail[:120],
                     'kind': dec.kind, 'action': dec.action, 'trigger': dec.trigger, 'values': list(dec.values),
                     'decision': decision, 'why': why, 'success': success, 'peer': peer})
        if success:
            log(f'◀ 외란 명령 #{seq}  {dec.ref}  → {dec.trigger}')
            if (self.rec is not None and self.args.record == 'scenario' and not self.rec.active()):
                self.rec.start('viz_nocue', 'adhoc', {'note': '시나리오(SendCue) 없이 들어온 명령'})
            self.enqueue(dec.trigger, 'grpc:' + peer, seq, tag, seed, t_rx, rec=rec)
        else:
            log(f'✘ 거절 #{seq}  {dec.ref}  → {why} {dec.detail}')
        self.publish_state()
        return success

    def state(self, req, peer):
        try:
            name = cue_pb2.TestState.Name(req.state)
        except Exception:
            name = str(req.state)
        log(f'◀ 상태 {name} (sender={req.sender or "-"}, peer {peer})')
        if req.state == cue_pb2.END:
            self.end_scenario('end')
        else:
            with self.lock:
                if self.scn is not None and not self.scn['closed']:
                    self.scn['state'] = name
            self.publish_state()

    def _tick(self):
        """1 s 마다: 시나리오 없는 녹화를 닫을지, 브리지 서비스가 보이는지."""
        seen = None
        last_warn = 0.0
        while not self.stop_ev.wait(1.0):
            ready = self.node.ready()
            if ready != seen:
                log(('● 브리지 서비스 보임 ' if ready else '! 브리지 서비스가 안 보인다 ') + self.node.srv_name)
                seen = ready
                last_warn = time.time()
            elif not ready and time.time() - last_warn > 30.0:
                log('! 아직 브리지 서비스가 없다 — MORAI 연결·브리지 로그를 볼 것')
                last_warn = time.time()
            if self.rec is not None and self.rec.kind() == 'adhoc':
                with self.lock:
                    idle = time.time() - self.last_rx_t
                if idle > ADHOC_IDLE_S:
                    self.rec.stop({'counts': dict(self.counts)})

    def shutdown(self):
        self.stop_ev.set()
        with self.lock:
            s = self.scn
            tag, seed = (s['tag'], s['seed']) if s is not None else ('', 0)
        sent = self.node.call_nowait('disturbance, clear', tag, seed, 'auto:shutdown', 0)
        log(f'  종료 — clear {"보냄" if sent else "못 보냄 (브리지 없음)"}')
        if self.rec is not None:
            self.rec.stop(self._meta())
        try:
            self.q.put_nowait(None)
        except Exception:
            pass


# ───────────────────────────────────────────────────────────── gRPC
class VizServicer(cue_pb2_grpc.CueServiceServicer):
    def __init__(self, server):
        self.s = server

    @staticmethod
    def _peer(context):
        p = context.peer() or ''
        s = p.split(':', 1)[1] if p.startswith(('ipv4:', 'ipv6:')) else p
        return (s.rsplit(':', 1)[0] if ':' in s else s).strip()

    def SendCue(self, request, context):
        peer = self._peer(context)
        try:
            if request.sender == 'cue_input':          # KATECH cue_input.py 의 SPACE — READY 를 보낸다
                log(f'◀ cue_input 키 {request.cue_type!r} (peer {peer})')
                if request.cue_type == ' ' and self.s.args.ready != 'off':
                    threading.Thread(target=self.s.send_ready, args=('cue_input SPACE',), daemon=True).start()
                return cue_pb2.CueResponse(success=True, message='viz: key handled')
            if not (request.scenario_name or '').strip():
                return cue_pb2.CueResponse(success=False, message='viz: empty scenario_name')
            self.s.begin_scenario(request, peer)
            return cue_pb2.CueResponse(success=True, message='viz cue received')
        except Exception as e:
            log(f'! SendCue 처리 예외: {type(e).__name__}: {e}')
            return cue_pb2.CueResponse(success=False, message=f'viz error: {type(e).__name__}')

    def Ping(self, request, context):
        return cue_pb2.PingResponse(alive=True)

    def SendState(self, request, context):
        try:
            self.s.state(request, self._peer(context))
            return cue_pb2.StateResponse(success=True, message='viz state received')
        except Exception as e:
            log(f'! SendState 처리 예외: {type(e).__name__}: {e}')
            return cue_pb2.StateResponse(success=False, message=f'viz error: {type(e).__name__}')

    def SendCustomCommand(self, request, context):
        try:
            return cue_pb2.CustomCommandResponse(success=self.s.custom(request, self._peer(context)))
        except Exception as e:
            log(f'! SendCustomCommand 처리 예외: {type(e).__name__}: {e}')
            return cue_pb2.CustomCommandResponse(success=False)


def check_proto():
    here = os.path.join(RIG, 'cue', 'cue.proto')
    try:
        h = hashlib.sha256(open(here, 'rb').read()).hexdigest()
        log(f'  cue.proto sha256 = {h[:16]}…')
        if os.path.exists(KATECH_PROTO) and hashlib.sha256(open(KATECH_PROTO, 'rb').read()).hexdigest() != h:
            log(f'! KATECH 원본({KATECH_PROTO})과 해시가 다르다')
    except OSError:
        log('! cue/cue.proto 를 읽을 수 없다')
    if not hasattr(cue_pb2, 'CustomCommandRequest'):
        log('! 스텁에 SendCustomCommand 가 없다 — 외란 명령을 못 받는다')


def main():
    default_morai = os.environ.get('MORAI_SIM_ADDRESS', '10.10.10.1')
    ap = argparse.ArgumentParser(description='외란 시각화 전용 cue 수신기 (Autoware 없음)')
    ap.add_argument('--host', default='0.0.0.0')
    ap.add_argument('--port', type=int, default=50052)
    ap.add_argument('--morai-cue', default=os.environ.get('MORAI_CUE_ADDRESS', f'{default_morai}:50052'),
                    help='READY 를 보낼 KATECH CueService (기본 MORAI_CUE_ADDRESS, 없으면 <MORAI_SIM_ADDRESS>:50052)')
    ap.add_argument('--ready', choices=['auto', 'key', 'off'], default='auto')
    ap.add_argument('--ready-delay', type=float, default=1.0, help='SendCue 뒤 READY 까지 [s] (clear 응답 뒤)')
    ap.add_argument('--sender-name', default='bag_pc')
    ap.add_argument('--bridge-node', default='morai_viz_bridge')
    ap.add_argument('--data', default='~/01_DAQ_Data/Viz_Disturbance')
    ap.add_argument('--record', choices=['scenario', 'session', 'off'], default='scenario')
    ap.add_argument('--seed', type=int, default=0, help='고정 기본 시드 (0 = 시나리오마다 새로)')
    a = ap.parse_args()
    if a.port == REAL_MORAI_PORT:
        sys.exit('7789 는 MORAI 포트다')
    if a.seed < 0 or a.seed > SEED_MAX:
        sys.exit(f'--seed 는 0 ~ {SEED_MAX}')

    log(f'외란 시각화 수신기 — bind {a.host}:{a.port} · READY {a.ready} → {a.morai_cue} · 브리지 /{a.bridge_node} · '
        f'녹화 {a.record} · 기록 {os.path.expanduser(a.data)}')
    check_proto()

    rclpy.init(args=None)
    node = VizNode(a.bridge_node)
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, name='ros-spin', daemon=True).start()
    srv = VizServer(a, node)
    srv.publish_state()
    if a.record == 'session' and srv.rec is not None:
        srv.rec.start('viz_session', 'session', {'note': '세션 전체'})

    grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=8), options=[('grpc.so_reuseport', 0)])
    cue_pb2_grpc.add_CueServiceServicer_to_server(VizServicer(srv), grpc_server)
    if grpc_server.add_insecure_port(f'{a.host}:{a.port}') == 0:
        log(f'! {a.host}:{a.port} 를 열 수 없다 — 다른 cue 서버(11_ cue_server·KATECH bag_cue_server.py)가 쓰고 있다')
        srv.shutdown()
        rclpy.shutdown()
        sys.exit(1)
    grpc_server.start()
    log(f'대기 중 — {a.host}:{a.port} (Ctrl+C 로 끝)')

    if a.ready == 'key':
        if sys.stdin is not None and sys.stdin.isatty():
            def keys():
                for _ in sys.stdin:
                    srv.send_ready('Enter')
            threading.Thread(target=keys, daemon=True).start()
        else:
            log('! --ready key 는 터미널에서 띄울 때만 된다 — READY 를 보내지 않는다')

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    stop.wait()
    log('끝내는 중 …')
    grpc_server.stop(grace=1.0)
    srv.shutdown()
    time.sleep(0.3)
    try:
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass
    log('끝')


if __name__ == '__main__':
    main()
