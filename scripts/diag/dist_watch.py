#!/usr/bin/env python3
# 신규 — 26.11.11 _ YSH _ 외란 실시간 보기 (읽기 전용). 리그가 도는 동안 다른 터미널에서 띄워
# 26.11.11 _ YSH _ 외란 메시지를 morai_ros2_msgs(외란 추가판)로 옮겼다 — 이 파일의 mr_disturbance_msgs 를 morai_ros2_msgs 로 바꿨다 (이름만, 주석 속 옛 이름·원문은 그대로)
#   "지금 외란 명령이 들어왔나 · 전달됐나 · 걸렸나 · 객체가 숨었나 · Autoware 에 몇 개 가나"를 본다
"""외란 실시간 보기 (읽기 전용).

보는 것
  1. cue_server 기록 (cue_logs 폴더)
     - KATECH 이 보낸 명령 원문 (Type=…, Value=…) · 판정 (전달 · 기록만 · 거절) · 브리지에 걸리기까지 ms
     - 런 흐름 (cue 받음 · 녹화 · READY · KATECH 상태 · engage · 주행 사건 · 결과) 과 ERROR 줄
  2. 브리지 토픽 (ROS, 구독만)
     - <prefix>/disturbance      프레임마다 켜진 외란 · 이번 프레임 숨김 여부 · GT 객체 수 · 시드
     - <prefix>/run_state        DAQ 게이트 (active / idle)
     - /perception/object_recognition/tracking/objects   Autoware 가 실제로 받는 객체 수 · 주기
  맨 아래 상태줄은 0.2 s 마다 다시 쓴다 (터미널일 때만).

쓰는 법 (./dist_shell.sh 로 연 셸 — 리그와 같은 도메인 10)
  ./run_rig.sh dist                               # 이 스크립트
  python3 -B diag/dist_watch.py --since 300       # 지난 5분 기록부터 보여 주고 이어서
  python3 -B diag/dist_watch.py --flips           # dropout 숨김·보임을 바뀔 때마다 한 줄씩
  python3 -B diag/dist_watch.py --no-ros          # 기록 파일만 (ROS 없이)

아무것도 쓰지 않는다 — 서비스 호출 · 파라미터 변경 · 발행이 없다. 파일은 읽기만, 토픽은 구독만 한다.
"""
import argparse
import json
import os
import re
import shutil
import sys
import time
import unicodedata
from collections import deque
from datetime import datetime

HOME = os.path.expanduser('~')
DEFAULT_LOG_DIR = os.path.join(HOME, '01_DAQ_Data', 'DAQ_Bag_Disturbance', 'cue_logs')
TRACKED_TOPIC = '/perception/object_recognition/tracking/objects'
# cue_logs 안의 다른 도구 기록 — 크고 빨리 자라서 따라 읽지 않는다
SKIP_PREFIX = ('autoware_launch_', 'watchdog_', 'topicwatch_', 'run_debug_', 'bridge_', 'daq_publisher_', 'speed_cap_')
RESCAN_S = 1.0
TICK_S = 0.05
STATUS_EVERY_S = 0.2
SILENT_S = 1.5          # /daq/disturbance 가 이만큼 안 오면 멈춘 것으로 본다
INITIAL_TAIL_BYTES = 2_000_000
HIDE_WINDOW_S = 10.0    # 상태줄의 "최근 숨김 비율" 창

# ----------------------------------------------------------------------------- 색
_COLOR = True
_C = {'dim': '2', 'b': '1', 'red': '31', 'grn': '32', 'yel': '33', 'blu': '34', 'mag': '35', 'cyn': '36', 'org': '38;5;208'}


def c(text, *styles):
    if not _COLOR or not styles:
        return text
    return '\033[' + ';'.join(_C[s] for s in styles) + 'm' + text + '\033[0m'


# ----------------------------------------------------------------------------- 시각
def hms(epoch):
    return datetime.fromtimestamp(epoch).strftime('%H:%M:%S.%f')[:-3]


def iso_epoch(text):
    """'2026-11-11T13:29:49.371+09:00' → epoch. 못 읽으면 None."""
    try:
        return datetime.fromisoformat(text).timestamp()
    except (TypeError, ValueError):
        return None


# ----------------------------------------------------------------------------- cue_server 기록 한 줄
LOG_RE = re.compile(r'^(\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d(?:\.\d+)?(?:[+-]\d\d:\d\d)?)\s+([A-Z]+)\s+(\S+)\s+([A-Z_]+)\s+(\S+)\s?(.*)$')
KV_RE = re.compile(r'(\w+)=(\S+)')


def parse_log_line(line):
    """cue_server 런 로그 한 줄 → dict. 형식이 아니면 None."""
    m = LOG_RE.match(line.rstrip('\n'))
    if not m:
        return None
    t = iso_epoch(m.group(1))
    if t is None:
        return None
    rest = m.group(6)
    head, _, msg = rest.partition(' | ')
    if rest.startswith('| '):
        head, msg = '', rest[2:]
    return {'t': t, 'level': m.group(2), 'run': m.group(3), 'cat': m.group(4), 'code': m.group(5),
            'kv': dict(KV_RE.findall(head)), 'msg': msg.strip()}


# jsonl(사이드카)에서 더 자세히 보여 주므로 텍스트 로그에서는 빼는 것
DIST_DUP = {'DIST_RX', 'DIST_BAD', 'DIST_REJECT', 'DIST_APPLIED', 'DIST_APPLY_FAILED'}

WHY_KO = {
    'RUN_NOT_STARTED': '런 시작 전', 'NO_ACTIVE_RUN': '런 없음', 'RUN_FINISHING': '런 끝나는 중',
    'SERVER_HOLD': '서버 보류(DEGRADED)', 'RECORD_ONLY': '기록 전용 모드', 'DRY_RUN': '건식 모드',
    'FORWARD_OFF': '전달 꺼짐(브리지 서비스 없음)', 'VILS': '실차 모드', 'UNSUPPORTED_KIND': '지원 안 하는 종류',
    'QUEUE_FULL': '전달 줄 가득', 'NO_CLIENT': '브리지 클라이언트 없음', 'TIMEOUT': '브리지 응답 없음',
    'REJECTED': '브리지가 거절', 'RUN_FINISHED_BEFORE_FORWARD': '전달 전에 런이 끝남', 'BRIDGE_DOWN': '브리지 서비스 없음',
}
DECISION_KO = {'forward': '전달', 'record': '기록만', 'reject': '거절'}


def _why(code):
    if not code or code == '-':
        return ''
    head = code.split(':', 1)[0]
    return f'{WHY_KO.get(head, head)}'


def event_from_log(r):
    """런 로그 레코드 → (epoch, 글) 또는 None (보여 줄 것만)."""
    cat, code, kv, msg, lv = r['cat'], r['code'], r['kv'], r['msg'], r['level']
    run = r['run']
    if cat == 'DIST':
        if code in DIST_DUP:
            return None
        if code == 'DIST_SEED':
            return r['t'], c(f'  시드 확인 base_seed={kv.get("base_seed", "?")} ({kv.get("algorithm", "")})', 'cyn')
        if code == 'DIST_SEED_CHANGED':
            return r['t'], c(f'! 기본 시드가 바뀌었다 {kv.get("prev", "?")} → {kv.get("base_seed", "?")} (브리지 재기동?)', 'yel')
        if code == 'DIST_NO_SEED':
            return r['t'], c(f'! 런 시작 시드 없음 — {_why(kv.get("why"))} · {msg}', 'yel')
        if code == 'DIST_SUMMARY':
            return r['t'], c(f'■ 런 외란 요약  받음 {kv.get("received", "?")} · 전달 {kv.get("forwarded", "?")} · '
                             f'적용 {kv.get("applied", "?")} · 실패 {kv.get("failed", "?")} · 거절 {kv.get("rejected", "?")} · '
                             f'기록만 {kv.get("recorded_only", "?")}', 'b')
        style = 'red' if lv == 'ERROR' else ('yel' if lv == 'WARN' else 'dim')
        return r['t'], c(f'  {code} {" ".join(f"{k}={v}" for k, v in kv.items())} {msg}'.rstrip(), style)
    if cat == 'CUE_RECV' and code == 'CUE':
        return r['t'], c(f'▶ 새 런 [{run}] cue 받음 · {kv.get("scenario", "?")} ({kv.get("cue_type", "")})', 'b', 'blu')
    if cat == 'REC' and code in ('BAG_START', 'BAG_STOP'):
        return r['t'], c(f'  녹화 {"시작" if code == "BAG_START" else "끝"} [{run}]', 'dim')
    if cat == 'DAQ' and code in ('DAQ_ON', 'DAQ_OFF'):
        return r['t'], c(f'  DAQ 게이트 {"열림" if code == "DAQ_ON" else "닫힘"} (cue_server)', 'dim')
    if cat == 'STATE_TX':
        ok = kv.get('accepted', '')
        return r['t'], c(f'  {code} → KATECH (accepted={ok})', 'blu' if ok != 'False' else 'red')
    if cat == 'STATE_RX':
        return r['t'], c(f'  KATECH 상태 {code}', 'blu')
    if cat == 'ENGAGE' and code == 'ENGAGED':
        return r['t'], c('  engage — 자율주행 시작', 'blu')
    if cat == 'DRIVE':
        return r['t'], c(f'  주행 사건 {code} {msg}'.rstrip(), 'yel')
    if cat == 'RESULT':
        fc = kv.get('failure_code', '')
        style = 'grn' if code.startswith('OK') or code == 'ARRIVED' else 'red'
        return r['t'], c(f'■ 결과 {code}{" failure_code=" + fc if fc else ""} [{run}]', 'b', style)
    if lv == 'ERROR':
        return r['t'], c(f'  ERROR {cat} {code} {msg}'.rstrip(), 'red')
    return None


def event_from_jsonl(o):
    """사이드카(.dist.jsonl) 한 줄 → (epoch, 글) 또는 None."""
    ev = o.get('ev')
    t = o.get('t_unix') or iso_epoch(o.get('t')) or time.time()
    seq = o.get('seq', '?')
    if ev == 'rx':
        dec = o.get('decision', '')
        ref = o.get('ref') or o.get('trigger') or '?'
        why = _why(o.get('why'))
        if not o.get('ok'):
            why = f'{o.get("reason", "")} {o.get("detail", "")}'.strip()
        tail = f'{DECISION_KO.get(dec, dec)}{" · " + why if why else ""}'
        style = {'forward': ('b', 'org'), 'record': ('yel',), 'reject': ('b', 'red')}.get(dec, ())
        succ = 'success=true' if o.get('success') else 'success=false'
        return t, c(f'◀ 외란 명령 #{seq}  {ref}  → {tail}  ({succ})', *style)
    if ev == 'fwd':
        a = o.get('apply') or {}
        src = str(o.get('source', ''))
        trig = o.get('trigger', '')
        if src.startswith('auto:'):
            when = {'begin': '런 시작', 'reset': '런 끝', 'fatal': '비상 정리'}.get(src[5:], src[5:])
            if a.get('ok'):
                return t, c(f'  자동 clear ({when}) 적용 · {a.get("rx_to_applied_ms", "?")} ms', 'dim')
            return t, c(f'! 자동 clear ({when}) 실패 — {_why(a.get("why"))} · {a.get("message", "")}', 'yel')
        if a.get('ok'):
            return t, c(f'✔ 걸림 #{seq}  {trig}  · 받은 뒤 {a.get("rx_to_applied_ms", "?")} ms (브리지 "{a.get("message", "")}")',
                        'b', 'grn')
        return t, c(f'✘ 못 걸림 #{seq}  {trig}  · {_why(a.get("why"))} · 브리지 "{a.get("message", "")}"', 'b', 'red')
    if ev == 'queue_full':
        return t, c(f'✘ 전달 줄 가득 — #{seq} 거절로 고침', 'red')
    return None


# ----------------------------------------------------------------------------- 파일 따라 읽기
class LogTail:
    """cue_logs 폴더의 런 로그(.log)·사이드카(.dist.jsonl)를 따라 읽는다. 읽기만 한다."""

    def __init__(self, log_dir, since_s):
        self.dir = log_dir
        self.t0 = time.time()
        self.since_s = max(0.0, float(since_s))
        self.files = {}          # path → [offset, 남은 조각]
        self.next_scan = 0.0
        self.first_scan = True
        self.missing_noted = False

    def _candidates(self):
        try:
            it = os.scandir(self.dir)
        except OSError:
            return []
        out = []
        cutoff = self.t0 - self.since_s - 5.0
        with it:
            for e in it:
                n = e.name
                if not (n.endswith('.log') or n.endswith('.dist.jsonl')) or n.startswith(SKIP_PREFIX):
                    continue
                try:
                    st = e.stat()
                except OSError:
                    continue
                if st.st_mtime >= cutoff:
                    out.append((e.path, st.st_size))
        return out

    def poll(self, now):
        """새 줄 [(path, line)]."""
        if now >= self.next_scan:
            self.next_scan = now + RESCAN_S
            for path, size in self._candidates():
                if path in self.files:
                    continue
                if self.first_scan:
                    # 시작할 때 있던 파일: 기본은 끝부터, --since 면 마지막 일부부터 (시각으로 거른다)
                    start = size if self.since_s == 0 else max(0, size - INITIAL_TAIL_BYTES)
                else:
                    start = 0
                self.files[path] = [start, b'']
            self.first_scan = False
        lines = []
        for path, st in list(self.files.items()):
            try:
                size = os.path.getsize(path)
            except OSError:
                del self.files[path]
                continue
            if size < st[0]:
                st[0], st[1] = 0, b''          # 잘렸다 — 처음부터
            if size == st[0]:
                continue
            try:
                with open(path, 'rb') as f:
                    f.seek(st[0])
                    data = f.read(size - st[0])
            except OSError:
                continue
            st[0] = size
            data = st[1] + data
            parts = data.split(b'\n')
            st[1] = parts.pop()                # 줄 끝이 없는 조각은 다음에
            for p in parts:
                if p:
                    lines.append((path, p.decode('utf-8', 'replace')))
        return lines

    def events(self, now):
        out = []
        floor = self.t0 - self.since_s
        for path, line in self.poll(now):
            ev = None
            if path.endswith('.dist.jsonl'):
                try:
                    ev = event_from_jsonl(json.loads(line))
                except (ValueError, AttributeError):
                    ev = None
            else:
                r = parse_log_line(line)
                ev = event_from_log(r) if r else None
            if ev is not None and ev[0] >= floor:
                out.append(ev)
        out.sort(key=lambda e: e[0])
        return out


# ----------------------------------------------------------------------------- 외란 상태 (토픽)
def _fmt(v):
    return f'{v:g}'


def describe_on(kind, m):
    if kind == 'position':
        return (f'position 켬 · 종 {_fmt(m.lon_offset_m)}±{_fmt(m.lon_std_m)} m · '
                f'횡 {_fmt(m.lat_offset_m)}±{_fmt(m.lat_std_m)} m (Ego 기준 앞·왼쪽)')
    if kind == 'dropout':
        return f'dropout 켬 · 유지 {_fmt(m.keep_prob)} · 평균 끊김 {_fmt(m.mean_gap_s)} s'
    if kind == 'blackout':
        rec = '지속 뒤 저절로 복구' if m.blackout_recover else 'clear 까지'
        return f'blackout 시작 · {_fmt(m.blackout_duration_s)} s · {rec}'
    return kind


class StatusTracker:
    """/daq/disturbance·/daq/run_state·TrackedObjects 를 받아 사건 줄과 상태줄을 만든다. ROS 를 모른다(시험용)."""

    def __init__(self, flips=False):
        self.flips = flips
        self.receiving = False
        self.last_rx = 0.0
        self.last = None              # 직전 상태 (dict)
        self.seq = None
        self.hide_start = None        # (stamp_s, frame_seq, 원인)
        self.drop_hidden_n = 0
        self.drop_hidden_s = 0.0
        self.drop_on_since = None
        self.vis_hist = deque()       # (stamp_s, visible)
        self.gate = '?'
        self.tracked = deque()        # (도착 시각, 객체 수)
        self.n_gt = None
        self.triggers = []
        self.params = None

    @staticmethod
    def _stamp(m):
        try:
            return m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        except AttributeError:
            return time.time()

    def on_status(self, m, now):
        out = []
        st = self._stamp(m)
        if not self.receiving:
            out.append((now, c(f'● 외란 상태 토픽 들어옴 · frame_seq {m.frame_seq} · base_seed {m.base_seed}', 'cyn')))
            self.receiving = True
        self.last_rx = now
        cur = {'position': bool(m.position_on), 'dropout': bool(m.dropout_on), 'blackout': bool(m.blackout_on),
               'visible': bool(m.objects_visible), 'seed': int(m.base_seed)}
        prev = self.last or {'position': False, 'dropout': False, 'blackout': False, 'visible': True, 'seed': cur['seed']}
        if cur['seed'] != prev['seed'] and cur['seed']:
            out.append((now, c(f'● 새 시드 구간 base_seed {cur["seed"]} (frame_seq {m.frame_seq} 부터)', 'cyn')))
        # 켜기 줄을 먼저 (숨김 시작보다 앞에 보이게)
        for kind in ('position', 'dropout', 'blackout'):
            if cur[kind] and not prev[kind]:
                out.append((now, c('● ' + describe_on(kind, m), 'b', 'org')))
                if kind == 'dropout':
                    self.drop_hidden_n, self.drop_hidden_s, self.drop_on_since = 0, 0.0, st
        # 숨김 구간
        if not cur['visible'] and prev['visible']:
            why = 'blackout' if cur['blackout'] else 'dropout'
            self.hide_start = (st, m.frame_seq, why)
            if self.flips and why == 'dropout':
                out.append((now, c(f'  숨김 시작 (frame_seq {m.frame_seq}) — Autoware 에 빈 목록', 'org')))
        elif cur['visible'] and not prev['visible'] and self.hide_start is not None:
            dur = max(0.0, st - self.hide_start[0])
            if self.hide_start[2] == 'dropout':
                self.drop_hidden_n += 1
                self.drop_hidden_s += dur
                if self.flips:
                    out.append((now, c(f'  다시 보임 (frame_seq {m.frame_seq}) · 숨김 {dur:.1f} s', 'grn')))
            else:
                out.append((now, c(f'  숨김 {dur:.1f} s ({m.frame_seq - self.hide_start[1]} 프레임)', 'dim')))
            self.hide_start = None
        # 끄기 줄 (마지막 숨김까지 센 뒤)
        for kind in ('position', 'dropout', 'blackout'):
            if prev[kind] and not cur[kind]:
                if kind == 'dropout':
                    span = (st - self.drop_on_since) if self.drop_on_since else 0.0
                    pct = (100.0 * self.drop_hidden_s / span) if span > 0 else 0.0
                    out.append((now, c(f'● dropout 끔 · 숨김 {self.drop_hidden_n}회 · 합 {self.drop_hidden_s:.1f} s / '
                                       f'{span:.1f} s ({pct:.0f}%)', 'b', 'grn')))
                    self.drop_on_since = None
                elif kind == 'blackout':
                    out.append((now, c('● blackout 끝 — 객체 돌아옴', 'b', 'grn')))
                else:
                    out.append((now, c('● position 끔', 'b', 'grn')))
        self.vis_hist.append((st, cur['visible']))
        while self.vis_hist and self.vis_hist[0][0] < st - HIDE_WINDOW_S:
            self.vis_hist.popleft()
        self.last = cur
        self.seq = m.frame_seq
        self.n_gt = len(m.objects)
        self.triggers = list(m.active_triggers)
        self.params = m
        return out

    def on_run_state(self, text, now):
        word = (text or '').split(' ', 1)
        self.gate = word[0] if word and word[0] else '?'
        rest = word[1] if len(word) > 1 else ''
        style = ('b', 'cyn') if self.gate == 'active' else ('dim',)
        return [(now, c(f'● DAQ 게이트 {self.gate} {rest}'.rstrip(), *style))]

    def on_tracked(self, n_objects, now):
        self.tracked.append((now, n_objects))
        while self.tracked and self.tracked[0][0] < now - 2.0:
            self.tracked.popleft()

    def tick(self, now):
        if self.receiving and now - self.last_rx > SILENT_S:
            self.receiving = False
            return [(now, c(f'● 외란 상태 토픽 멈춤 ({SILENT_S:g} s 넘게 없음 — 런 끝·게이트 닫힘·브리지 재기동)', 'dim'))]
        return []

    def tracked_rate(self, now):
        pts = [t for t, _ in self.tracked if t >= now - 2.0]
        if len(pts) < 2:
            return 0.0
        return (len(pts) - 1) / max(1e-6, pts[-1] - pts[0])

    def status_line(self, now):
        parts = [hms(now)[:8], f'게이트 {self.gate}']
        if self.receiving and self.last is not None:
            on = [k for k in ('position', 'dropout', 'blackout') if self.last[k]]
            parts.append('외란 ' + (','.join(on) if on else '없음'))
            if not self.last['visible']:
                dur = (self._stamp(self.params) - self.hide_start[0]) if self.hide_start else 0.0
                parts.append(c(f'지금 숨김 {dur:.1f} s', 'b', 'org'))
            else:
                parts.append('지금 보임')
            if self.vis_hist:
                hid = sum(1 for _, v in self.vis_hist if not v)
                parts.append(f'최근 {HIDE_WINDOW_S:g} s 숨김 {100.0 * hid / len(self.vis_hist):.0f}%')
            parts.append(f'k {self.seq}')
        else:
            parts.append(c('외란 토픽 없음', 'dim'))
        rate = self.tracked_rate(now)
        n_aw = self.tracked[-1][1] if self.tracked else None
        gt = '?' if self.n_gt is None or not self.receiving else str(self.n_gt)
        aw = '?' if n_aw is None else str(n_aw)
        txt = f'객체 GT {gt} → Autoware {aw} ({rate:.1f} Hz)'
        parts.append(c(txt, 'red') if (self.gate == 'active' and rate < 8.0) else txt)
        return ' │ '.join(parts)


# ----------------------------------------------------------------------------- 화면
_ANSI = re.compile(r'\033\[[0-9;]*m')


def fit_width(text, cols):
    """색 코드는 그대로 두고 화면 칸 수(한글 2칸)로 cols 안에 자른다 — 넘쳐 줄이 바뀌면 상태줄이 겹쳐 찍힌다."""
    out, w, i = [], 0, 0
    while i < len(text):
        m = _ANSI.match(text, i)
        if m:
            out.append(m.group(0))
            i = m.end()
            continue
        ch = text[i]
        cw = 2 if unicodedata.east_asian_width(ch) in ('W', 'F') else 1
        if w + cw > cols:
            out.append('\033[0m' if _COLOR else '')
            break
        out.append(ch)
        w += cw
        i += 1
    return ''.join(out)


class Screen:
    def __init__(self, tty):
        self.tty = tty
        self.has_status = False

    def event(self, t, text):
        if self.tty and self.has_status:
            sys.stdout.write('\r\033[K')
            self.has_status = False
        sys.stdout.write(f'{c(hms(t), "dim")}  {text}\n')
        sys.stdout.flush()

    def status(self, text):
        if not self.tty:
            return
        sys.stdout.write('\r\033[K' + fit_width(text, shutil.get_terminal_size((120, 24)).columns - 1))
        sys.stdout.flush()
        self.has_status = True

    def close(self):
        if self.tty and self.has_status:
            sys.stdout.write('\n')
            sys.stdout.flush()


# ----------------------------------------------------------------------------- ROS (구독만)
def make_ros(prefix, tracker, pending):
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
    from std_msgs.msg import String
    from morai_ros2_msgs.msg import DisturbanceStatus
    try:
        from autoware_perception_msgs.msg import TrackedObjects
    except ImportError:
        TrackedObjects = None
    rclpy.init(args=None)
    node = rclpy.create_node(f'dist_watch_{os.getpid()}')
    rel = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         history=HistoryPolicy.KEEP_LAST)
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(DisturbanceStatus, prefix + '/disturbance',
                             lambda m: pending.extend(tracker.on_status(m, time.time())), rel)
    # /daq/run_state 구독자 수를 cue_server 가 "브리지 DAQ 있음" 근거로 센다. 리그에는 토픽 감시 [6] 이 이미 구독하고 있어
    #   하나 더 붙어도 판단이 바뀌지 않는다 (브리지가 없을 때 런 끝 요약을 0.5 s 더 기다릴 뿐)
    node.create_subscription(String, prefix + '/run_state',
                             lambda m: pending.extend(tracker.on_run_state(m.data, time.time())), latched)
    if TrackedObjects is not None:
        node.create_subscription(TrackedObjects, TRACKED_TOPIC,
                                 lambda m: tracker.on_tracked(len(m.objects), time.time()), best)
    return rclpy, node


def main(argv=None):
    global _COLOR
    ap = argparse.ArgumentParser(description='외란 실시간 보기 (읽기 전용)')
    ap.add_argument('--log-dir', default=DEFAULT_LOG_DIR, help=f'cue_server 기록 폴더 (기본 {DEFAULT_LOG_DIR})')
    ap.add_argument('--since', type=float, default=0.0, help='시작할 때 지난 N 초 기록부터 보여 준다 (기본 0 = 새 것만)')
    ap.add_argument('--prefix', default='/daq', help='DAQ 토픽 접두 (기본 /daq)')
    ap.add_argument('--flips', action='store_true', help='dropout 숨김·보임을 바뀔 때마다 한 줄씩')
    ap.add_argument('--no-ros', action='store_true', help='토픽을 보지 않고 기록 파일만')
    ap.add_argument('--no-color', action='store_true', help='색 끄기')
    ap.add_argument('--duration', type=float, default=0.0, help='N 초 뒤 끝낸다 (0 = Ctrl+C 까지, 시험용)')
    a = ap.parse_args(argv)
    tty = sys.stdout.isatty()
    _COLOR = tty and not a.no_color and not os.environ.get('NO_COLOR')

    screen = Screen(tty)
    tracker = StatusTracker(flips=a.flips)
    tail = LogTail(a.log_dir, a.since)
    pending = []
    rclpy = node = None
    dom = os.environ.get('ROS_DOMAIN_ID', '(없음)')
    print(c('=== 외란 실시간 보기 (읽기 전용 · Ctrl+C 로 끝) ===', 'b'))
    print(f'    기록  {a.log_dir}' + ('' if os.path.isdir(a.log_dir) else c('  (아직 없음 — 생기면 읽는다)', 'yel')))
    if not a.no_ros:
        try:
            rclpy, node = make_ros(a.prefix, tracker, pending)
            print(f'    토픽  {a.prefix}/disturbance · {a.prefix}/run_state · {TRACKED_TOPIC}   (ROS_DOMAIN_ID {dom})')
            if dom != '10':
                print(c(f'    ! 리그는 도메인 10 이다 — 지금 {dom}', 'yel'))
        except Exception as e:      # ROS 가 없어도 기록 파일은 본다
            print(c(f'    ! 토픽을 못 본다 ({type(e).__name__}: {e}) — 기록 파일만 본다. ./dist_shell.sh 로 연 셸인가?', 'yel'))
            rclpy = node = None
    print(c('    ◀ 명령 받음 · ✔ 걸림 · ✘ 못 걸림 · ● 브리지가 실제로 낸 상태 · ■ 런 결과', 'dim'))
    t_end = time.time() + a.duration if a.duration > 0 else None
    next_status = 0.0
    try:
        while True:
            now = time.time()
            if t_end is not None and now >= t_end:
                break
            if node is not None:
                rclpy.spin_once(node, timeout_sec=TICK_S)
            else:
                time.sleep(TICK_S)
            now = time.time()
            evs = tail.events(now) + pending + tracker.tick(now)
            pending.clear()
            for t, text in sorted(evs, key=lambda e: e[0]):
                screen.event(t, text)
            if now >= next_status:
                next_status = now + STATUS_EVERY_S
                screen.status(tracker.status_line(now))
    except KeyboardInterrupt:
        pass
    except Exception as e:                 # Ctrl+C 에 rclpy 가 먼저 내려가면 ExternalShutdownException
        if type(e).__name__ != 'ExternalShutdownException':
            raise
    finally:
        screen.close()
        if node is not None:
            try:
                node.destroy_node()
                rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
