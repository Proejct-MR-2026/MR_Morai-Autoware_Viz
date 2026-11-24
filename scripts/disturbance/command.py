# 신규 — 26.11.10 _ YSH _ KATECH SendCustomCommand(Type, Value) 를 표준 문장으로 바꾼다. ROS·gRPC 를 임포트하지 않는다.
"""외란 명령 해석 (리그 쪽).

KATECH 시나리오 API 는 cue 통로(cue.CueService)의 SendCustomCommand 로 외란을 알린다. 26.11.10 실제 캡처

    Type="dropout", Value=[0.75, 0.5]     켜기 (숫자 수열)
    Type="dropout", Value="clear"         끄기 (같은 종류 이름에 글자 clear)

이 모듈은 그 (Type, Value) 를 브리지(/morai_bridge/set_disturbance)가 받는 표준 문장으로 바꾼다

    disturbance, position, <종편향>, <종표준편차>, <횡편향>, <횡표준편차>
    disturbance, dropout, <유지확률>, <평균끊김 s>
    disturbance, blackout, <지속 s>, <복구 1|0>
    disturbance, <종류>, clear            그 종류만 끈다
    disturbance, clear                    전부 끈다

받는 모양은 넉넉하게, 값은 엄격하게 본다.
  넉넉하게  숫자 수열 · 숫자 글자("0.4") · 참거짓(복구) · Type="clear" · "disturbance" 접두 · 종류가 수열 첫 칸 ·
            원문 한 줄 · 엑셀 8장 이름의 묶음(struct)
  엄격하게  개수(position 4 · dropout 2 · blackout 2 · heading 2 · mount 1) · 유한한 수 · 범위

범위는 브리지(C++ object_disturbance.cpp)와 같아야 한다 — 여기서 받아 놓고 브리지가 거부하면 KATECH 는 걸린 줄 안다.
  |편향| <= 100 m · 0 <= 표준편차 <= 100 m · 0 <= 유지확률 <= 1 ·
  0 < 유지확률 < 1 이면 0 < 평균끊김 <= 600 s (아니면 0 이상 유한) · 0 <= 지속 <= 3600 s · 복구는 0 또는 1
범위는 6자리로 반올림한 값(브리지에 가는 값)으로 본다.

heading · mount 는 읽기는 하되 supported=False 다 (브리지가 아직 안 받는다).
decode() 는 예외를 내지 않는다.
"""
import math
import re

KINDS = ('position', 'dropout', 'blackout', 'heading', 'mount')
SUPPORTED_KINDS = ('position', 'dropout', 'blackout')
COUNTS = {'position': 4, 'dropout': 2, 'blackout': 2, 'heading': 2, 'mount': 1}
# 엑셀(악의조건_트리거예시 v0.2) 8장의 이름 — Value 가 묶음(struct)으로 올 때의 칸 순서
STRUCT_KEYS = {
    'position': ('lon_offset_m', 'lon_std_m', 'lat_offset_m', 'lat_std_m'),
    'dropout': ('keep_prob', 'mean_gap_s'),
    'blackout': ('duration_s', 'recover'),
    'heading': ('angle_offset_deg', 'angle_std_deg'),
    'mount': ('angle_offset_deg',),
}

MAX_ITEMS = 32
MAX_STR = 256
MAX_DEPTH = 3

OFFSET_MAX_M = 100.0
STD_MAX_M = 100.0
GAP_MAX_S = 600.0
DURATION_MAX_S = 3600.0
ANGLE_MAX_DEG = 360.0

_NUM_RE = re.compile(r'^[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?$')
_TRUE = ('true',)
_FALSE = ('false',)


class Decoded:
    """decode() 의 결과. ok=False 면 reason 이 거부 코드다."""

    __slots__ = ('ok', 'reason', 'detail', 'kind', 'action', 'values', 'trigger', 'supported', 'shape', 'raw', 'ref')

    def __init__(self):
        self.ok = False
        self.reason = ''        # 거부 코드 (ok 면 빈 문자열)
        self.detail = ''        # 사람이 읽는 한 줄
        self.kind = ''          # position | dropout | blackout | heading | mount | '' (전체 해제)
        self.action = ''        # set | off(끄기와 같은 값) | clear(그 종류만) | clear_all
        self.values = []        # 브리지에 가는 값 (반올림 뒤 float, 복구는 1.0/0.0)
        self.trigger = ''       # 표준 문장
        self.supported = False  # 브리지가 받는 종류인가
        self.shape = ''         # Value 가 어떤 모양으로 왔나 (list · string · number · bool · struct · null · unset · empty)
        self.raw = None         # 받은 Value 의 JSON 으로 쓸 수 있는 사본 [종류, 내용]
        self.ref = ''           # KATECH 참조 서버가 찍는 줄과 같은 꼴 ("Type=…, Value=…")

    def record(self):
        """로그·JSON 용 사본."""
        return {k: getattr(self, k) for k in self.__slots__}

    def __repr__(self):
        return f'Decoded(ok={self.ok}, reason={self.reason!r}, trigger={self.trigger!r}, action={self.action!r})'


def fmt_number(v):
    """표준 문장의 숫자 한 칸. 6자리로 반올림하고 뒤의 0 을 뗀다 (0.75 · 0.5 · 0 · -1.4 · 1)."""
    s = format(round(float(v), 6), '.6f').rstrip('0').rstrip('.')
    return '0' if s in ('-0', '') else s


def canonical(kind, action, values):
    """표준 문장."""
    if action == 'clear_all':
        return 'disturbance, clear'
    if action == 'clear':
        return f'disturbance, {kind}, clear'
    return 'disturbance, ' + kind + ', ' + ', '.join(fmt_number(v) for v in values)


# ───────────────────────────────────────────────────────────── Value 읽기
class _TooLarge(Exception):
    pass


def _tree(v, depth=0):
    """protobuf Value → [종류, 내용] (JSON 으로 쓸 수 있다). 크기를 넘으면 _TooLarge."""
    if v is None:
        return ['unset', None]
    kind = v.WhichOneof('kind')
    if kind is None:
        return ['empty', None]
    if kind == 'null_value':
        return ['null', None]
    if kind == 'number_value':
        x = v.number_value
        if math.isnan(x):
            return ['number', 'NaN']
        if math.isinf(x):
            return ['number', 'Infinity' if x > 0 else '-Infinity']
        return ['number', x]
    if kind == 'string_value':
        if len(v.string_value) > MAX_STR:
            raise _TooLarge('string')
        return ['string', v.string_value]
    if kind == 'bool_value':
        return ['bool', bool(v.bool_value)]
    if depth >= MAX_DEPTH:
        raise _TooLarge('depth')
    if kind == 'list_value':
        items = v.list_value.values
        if len(items) > MAX_ITEMS:
            raise _TooLarge('list')
        return ['list', [_tree(x, depth + 1) for x in items]]
    if kind == 'struct_value':
        fields = v.struct_value.fields
        if len(fields) > MAX_ITEMS:
            raise _TooLarge('struct')
        out = {}
        for k in sorted(fields):
            if len(k) > MAX_STR:
                raise _TooLarge('key')
            out[k] = _tree(fields[k], depth + 1)
        return ['struct', out]
    return ['empty', None]


def _plain(tree):
    """[종류, 내용] → 파이썬 값 (참조 서버의 MessageToDict 결과와 같은 꼴)."""
    kind, body = tree
    if kind == 'list':
        return [_plain(x) for x in body]
    if kind == 'struct':
        return {k: _plain(x) for k, x in body.items()}
    return body


def _str_tokens(text):
    """글자 → 토큰. 쉼표가 있으면 쉼표로, 없으면 공백으로 자른다. 빈 토큰은 버린다."""
    parts = text.split(',') if ',' in text else text.split()
    return [p.strip() for p in parts if p.strip()]


def _classify(tok):
    """글자 토큰 → ('num', float) | ('bool', bool) | ('word', 소문자)."""
    if _NUM_RE.match(tok):
        return ('num', float(tok))
    low = tok.lower()
    if low in _TRUE:
        return ('bool', True)
    if low in _FALSE:
        return ('bool', False)
    return ('word', low)


class _Bad(Exception):
    def __init__(self, code, detail=''):
        super().__init__(code)
        self.code = code
        self.detail = detail


def _items_from_tree(tree, kind_hint, depth=0):
    """Value 사본 → 토큰 목록 [('num', x) | ('bool', b) | ('word', w)]."""
    kind, body = tree
    if kind in ('unset', 'empty', 'null'):
        return []
    if kind == 'number':
        if isinstance(body, str):
            raise _Bad('NOT_FINITE', body)
        return [('num', float(body))]
    if kind == 'bool':
        return [('bool', body)]
    if kind == 'string':
        return [_classify(t) for t in _str_tokens(body)]
    if kind == 'list':
        out = []
        for sub in body:
            if sub[0] == 'list':
                if depth >= 1:
                    raise _Bad('BAD_VALUE_KIND', '수열 안의 수열이 두 겹을 넘는다')
                out.extend(_items_from_tree(sub, kind_hint, depth + 1))
            elif sub[0] == 'struct':
                raise _Bad('BAD_VALUE_KIND', '수열 안에 묶음(struct)이 있다')
            elif sub[0] in ('unset', 'empty', 'null'):
                raise _Bad('BAD_VALUE_KIND', '수열 안에 빈 칸(null)이 있다')
            else:
                out.extend(_items_from_tree(sub, kind_hint, depth + 1))
        return out
    if kind == 'struct':
        if kind_hint not in STRUCT_KEYS:
            raise _Bad('BAD_STRUCT', '묶음(struct)은 Type 에 종류가 있어야 읽는다')
        want = STRUCT_KEYS[kind_hint]
        if set(body) != set(want):
            raise _Bad('BAD_STRUCT', f'{kind_hint} 묶음의 칸은 {", ".join(want)} 이어야 한다')
        out = []
        for k in want:
            sub = body[k]
            if sub[0] in ('list', 'struct', 'unset', 'empty', 'null'):
                raise _Bad('BAD_STRUCT', f'{k} 가 값 하나가 아니다')
            out.extend(_items_from_tree(sub, kind_hint, depth + 1))
        return out
    raise _Bad('BAD_VALUE_KIND', str(kind))


# ───────────────────────────────────────────────────────────── 값 검사
def _number(item, name):
    k, x = item
    if k != 'num':
        raise _Bad('NOT_NUMBER', f'{name} 가 숫자가 아니다')
    if not math.isfinite(x):
        raise _Bad('NOT_FINITE', name)
    return round(x, 6) + 0.0      # -0.0 → 0.0


def _in(x, lo, hi, name):
    if not (lo <= x <= hi):
        raise _Bad('OUT_OF_RANGE', f'{name}={fmt_number(x)} (허용 {fmt_number(lo)}~{fmt_number(hi)})')
    return x


def _recover(item):
    k, x = item
    if k == 'bool':
        return 1.0 if x else 0.0
    if k == 'num' and math.isfinite(x) and x in (0.0, 1.0):
        return float(x)
    if k == 'num' and not math.isfinite(x):
        raise _Bad('NOT_FINITE', 'recover')
    if k == 'word':
        raise _Bad('NOT_NUMBER', 'recover 가 1/0 이 아니다')
    raise _Bad('OUT_OF_RANGE', 'recover 는 1 또는 0')


def _check(kind, items):
    """종류별 값 검사 → (values, action)."""
    need = COUNTS[kind]
    if len(items) != need:
        raise _Bad('BAD_COUNT', f'{kind} 는 값 {need}개인데 {len(items)}개가 왔다')
    if kind == 'position':
        lon_off = _in(_number(items[0], 'lon_offset'), -OFFSET_MAX_M, OFFSET_MAX_M, 'lon_offset')
        lon_std = _in(_number(items[1], 'lon_std'), 0.0, STD_MAX_M, 'lon_std')
        lat_off = _in(_number(items[2], 'lat_offset'), -OFFSET_MAX_M, OFFSET_MAX_M, 'lat_offset')
        lat_std = _in(_number(items[3], 'lat_std'), 0.0, STD_MAX_M, 'lat_std')
        vals = [lon_off, lon_std, lat_off, lat_std]
        return vals, ('off' if all(v == 0.0 for v in vals) else 'set')
    if kind == 'dropout':
        keep = _in(_number(items[0], 'keep_prob'), 0.0, 1.0, 'keep_prob')
        gap = _number(items[1], 'mean_gap_s')
        if 0.0 < keep < 1.0:
            if not (0.0 < gap <= GAP_MAX_S):
                raise _Bad('OUT_OF_RANGE', f'mean_gap_s={fmt_number(gap)} (0 초과 {fmt_number(GAP_MAX_S)} 이하)')
        elif gap < 0.0:
            raise _Bad('OUT_OF_RANGE', f'mean_gap_s={fmt_number(gap)} (0 이상)')
        return [keep, gap], ('off' if keep == 1.0 else 'set')
    if kind == 'blackout':
        dur = _in(_number(items[0], 'duration_s'), 0.0, DURATION_MAX_S, 'duration_s')
        rec = _recover(items[1])
        return [dur, rec], ('off' if (rec == 1.0 and dur == 0.0) else 'set')
    if kind == 'heading':
        off = _in(_number(items[0], 'angle_offset_deg'), -ANGLE_MAX_DEG, ANGLE_MAX_DEG, 'angle_offset_deg')
        std = _in(_number(items[1], 'angle_std_deg'), 0.0, ANGLE_MAX_DEG, 'angle_std_deg')
        return [off, std], 'set'
    off = _in(_number(items[0], 'angle_offset_deg'), -ANGLE_MAX_DEG, ANGLE_MAX_DEG, 'angle_offset_deg')
    return [off], 'set'


# ───────────────────────────────────────────────────────────── 해석
def decode(type_str, value):
    """(Type 글자, protobuf Value 또는 None) → Decoded. 예외를 내지 않는다.

    value 는 google.protobuf.Value 다 (요청에 Value 가 없으면 None 을 넘긴다 — request.HasField('Value')).
    """
    d = Decoded()
    try:
        return _decode(d, type_str, value)
    except _Bad as e:
        d.ok = False
        d.reason = e.code
        d.detail = e.detail
        return d
    except _TooLarge as e:
        d.ok = False
        d.reason = 'TOO_LARGE'
        d.detail = str(e)
        return d
    except Exception as e:      # 해석기 버그가 gRPC 핸들러를 죽이면 안 된다
        d.ok = False
        d.reason = 'DECODER_ERROR'
        d.detail = type(e).__name__
        return d


def _decode(d, type_str, value):
    type_str = '' if type_str is None else str(type_str)
    if len(type_str) > MAX_STR:
        d.ref = 'Type=<%d자>, Value=?' % len(type_str)
        raise _TooLarge('type')
    tree = _tree(value)
    d.raw = tree
    d.shape = tree[0]
    d.ref = 'Type={}, Value={}'.format(type_str, _plain(tree))

    type_items = [_classify(t) for t in _str_tokens(type_str)]
    type_items = [it for it in type_items if it != ('word', 'disturbance')]
    kind_hint = next((w for k, w in type_items if k == 'word' and w in KINDS), None)
    items = type_items + [it for it in _items_from_tree(tree, kind_hint) if it != ('word', 'disturbance')]

    if not items:
        raise _Bad('NO_KIND', 'Type 과 Value 가 비어 있다')
    if items[0][0] != 'word':
        raise _Bad('NO_KIND', '종류(position·dropout·blackout·clear)가 맨 앞에 없다')

    kind = None
    clear = False
    vals = []
    for it in items:
        k, x = it
        if k == 'word':
            if x in KINDS:
                if kind is None:
                    kind = x
                elif kind != x:
                    raise _Bad('KIND_CONFLICT', f'{kind} 와 {x}')
            elif x == 'clear':
                clear = True
            elif kind is None and not clear and not vals:
                raise _Bad('UNKNOWN_KIND', x[:32])
            else:
                raise _Bad('NOT_NUMBER', x[:32])
        else:
            vals.append(it)

    if clear:
        if vals:
            raise _Bad('BAD_CLEAR', 'clear 에 값이 붙어 왔다')
        d.ok = True
        d.kind = kind or ''
        d.action = 'clear' if kind else 'clear_all'
        d.values = []
        d.supported = (kind is None) or (kind in SUPPORTED_KINDS)
        d.trigger = canonical(d.kind, d.action, [])
        return d
    if kind is None:
        raise _Bad('NO_KIND', '종류가 없다')

    values, action = _check(kind, vals)
    d.ok = True
    d.kind = kind
    d.action = action
    d.values = values
    d.supported = kind in SUPPORTED_KINDS
    d.trigger = canonical(kind, action, values)
    return d


def parse_canonical(text):
    """표준 문장 → (kind, action, values). 시험·대조용. 표준 문장이 아니면 ValueError."""
    toks = [t.strip() for t in str(text).split(',')]
    if len(toks) < 2 or toks[0] != 'disturbance' or any(t == '' for t in toks):
        raise ValueError(text)
    if toks[1] == 'clear':
        if len(toks) != 2:
            raise ValueError(text)
        return ('', 'clear_all', [])
    kind = toks[1]
    if kind not in KINDS:
        raise ValueError(text)
    if len(toks) == 3 and toks[2] == 'clear':
        return (kind, 'clear', [])
    try:
        values, action = _check(kind, [_classify(t) for t in toks[2:]])
    except _Bad as e:
        raise ValueError(f'{text} ({e.code})') from None
    return (kind, action, values)
