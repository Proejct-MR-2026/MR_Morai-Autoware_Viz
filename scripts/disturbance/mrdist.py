# -*- coding: utf-8 -*-
# 신규 — 26.11.10 _ YSH _ MRDIST-1 기준 구현 (순수 파이썬). C++ 브리지 object_disturbance.cpp 와 같은 값이 나와야 한다
"""외란 난수·시드 규칙 MRDIST-1 의 파이썬 기준 구현.

무엇인가
    브리지(/morai_bridge)가 GT 객체에 외란을 걸 때 쓰는 규칙을 글자 그대로 옮긴 것이다. 규칙 원문은
    ~/02_Mission/2_Disturbance/design_261110/10_cpp_bridge_msgs.md 4절이고, 고정 값(golden)은 diag/test_mrdist.py 가 본다.
    ROS·numpy 를 쓰지 않는다 — 시험, 합성 bag, 손 계산 확인용이다.

누가 쓰나
    diag/test_mrdist.py · diag/test_daq_disturb.py (합성 bag 을 이 규칙으로 만든다) · cue_server (draw_base_seed).
    daq_verify.py 는 일부러 이 파일을 임포트하지 않는다 (독립 검증 — 같은 식을 그쪽에 손으로 다시 적었다).

정수는 전부 부호 없는 64비트이고 넘치면 감긴다 (& MASK64).
"""
import math
import secrets
import uuid

ALGORITHM = 'MRDIST-1'
MASK64 = 0xFFFFFFFFFFFFFFFF
GAMMA = 0x9E3779B97F4A7C15
TAG_OBJECT = 0x4F424A5345454431    # ASCII "OBJSEED1" (big-endian)
TAG_DROPOUT = 0x44524F504F555431   # ASCII "DROPOUT1" (big-endian)
NOMINAL_PERIOD_NS = 100_000_000    # 객체 프레임 기준 주기 0.1 s (브리지 Engine::kObjectsPeriodNs)
NOMINAL_PERIOD_S = 0.1
MAX_PERIOD_NS = 1_000_000_000      # 이보다 긴 간격은 기준 주기로 센다
MAX_BASE_SEED = 999_999_999_999_999
TWO_PI = 6.283185307179586
UUID_PREFIX = 'morai:'             # 브리지 object_id_from_name 과 같은 이름 규칙


# ───────────────────────────────────────────────────────────── 난수
def mix64(z):
    """SplitMix64 마무리 함수."""
    z &= MASK64
    z ^= z >> 30
    z = (z * 0xBF58476D1CE4E5B9) & MASK64
    z ^= z >> 27
    z = (z * 0x94D049BB133111EB) & MASK64
    z ^= z >> 31
    return z


def stream(seed, counter):
    """seed 의 SplitMix64 출력 counter 번째."""
    return mix64((seed + counter * GAMMA) & MASK64)


def u01(seed, counter):
    """(0, 1) 균등 난수. 0 과 1 은 나오지 않는다 (52비트, 정확한 double)."""
    return ((stream(seed, counter) >> 12) + 0.5) * 2.0 ** -52


def name_hi_lo(name):
    """객체 이름 → uuid5(NAMESPACE_OID, 'morai:'+이름) 의 앞 8바이트·뒤 8바이트 (big-endian u64)."""
    b = uuid.uuid5(uuid.NAMESPACE_OID, UUID_PREFIX + name).bytes
    return int.from_bytes(b[:8], 'big'), int.from_bytes(b[8:], 'big')


def object_seed(base_seed, name):
    """객체 시드. base_seed 0(시드 구간 없음)이면 0."""
    if not base_seed:
        return 0
    hi, lo = name_hi_lo(name)
    return mix64(mix64(base_seed ^ TAG_OBJECT ^ hi) ^ lo)


def dropout_seed(base_seed):
    """간헐 끊김 시드. base_seed 0 이면 0."""
    if not base_seed:
        return 0
    return mix64(base_seed ^ TAG_DROPOUT)


def gauss2(seed, k):
    """프레임 k 의 표준정규 난수 한 쌍 (z_lon, z_lat). Box-Muller."""
    u1 = u01(seed, 2 * k)
    u2 = u01(seed, 2 * k + 1)
    r = math.sqrt(-2.0 * math.log(u1))
    a = TWO_PI * u2
    return r * math.cos(a), r * math.sin(a)


def dropout_prob(keep_prob, mean_gap_s):
    """(p_hide, p_show) — 프레임마다 숨김으로 갈 확률·보임으로 돌아올 확률.

    0 < keep_prob < 1 에서만 부른다 (결정 05 1절 9항). 가장자리에서는 식이 0 으로 나누므로 ValueError 를 낸다 —
    keep_prob <= 0 은 켠 프레임부터 전부 숨김(뽑지 않는다), keep_prob >= 1 은 꺼짐과 같다 (DropoutChain 이 그렇게 한다).
    """
    if not 0.0 < keep_prob < 1.0:
        raise ValueError(f'dropout_prob 는 0 < keep_prob < 1 에서만 정의된다 (받은 값 {keep_prob!r})')
    g = max(mean_gap_s, NOMINAL_PERIOD_S)
    p_show = min(1.0, NOMINAL_PERIOD_S / g)
    mean_on = g * keep_prob / (1.0 - keep_prob)
    p_hide = min(1.0, NOMINAL_PERIOD_S / mean_on)
    return p_hide, p_show


def dropout_saturated(keep_prob, mean_gap_s):
    """10 Hz 격자로는 요청한 (keep_prob, mean_gap_s)를 그대로 못 낸다 (브리지 응답 SATURATED). 가장자리에서는 False."""
    if not 0.0 < keep_prob < 1.0:
        return False
    return mean_gap_s < NOMINAL_PERIOD_S or mean_gap_s * keep_prob / (1.0 - keep_prob) < NOMINAL_PERIOD_S


def draw_base_seed():
    """런 기본 시드 1 ~ 999999999999999 (15자리 — 2^53 아래라 JSON·엑셀에서 안 깨진다)."""
    return secrets.randbelow(MAX_BASE_SEED) + 1


# ───────────────────────────────────────────────────────────── 시간
def period_ns(prev_stamp_ns, stamp_ns):
    """직전 객체 프레임과의 간격 [ns]. 첫 프레임·0 이하·1 s 초과는 기준 주기 0.1 s."""
    if prev_stamp_ns is None:
        return NOMINAL_PERIOD_NS
    d = stamp_ns - prev_stamp_ns
    return d if 0 < d <= MAX_PERIOD_NS else NOMINAL_PERIOD_NS


def duration_ns(duration_s):
    """완전 끊김 지속 [ns] = floor(duration_s × 1e9 + 0.5)."""
    return int(math.floor(duration_s * 1e9 + 0.5))


# ───────────────────────────────────────────────────────────── 종류별 상태
class DropoutChain:
    """간헐 끊김 사슬 (모든 객체가 같이 쓴다). 켠 프레임은 뽑지 않고, 그 뒤 프레임마다 u01(dropout_seed, k) 한 번."""

    def __init__(self, base_seed, keep_prob, mean_gap_s):
        self.seed = dropout_seed(base_seed)
        self.keep = keep_prob
        self.first = True
        self.hidden = False
        self.p_hide, self.p_show = (dropout_prob(keep_prob, mean_gap_s) if 0.0 < keep_prob < 1.0 else (None, None))

    def step(self, k):
        """프레임 k 를 지난 뒤의 숨김 여부."""
        if self.first:
            self.first = False
            self.hidden = self.keep <= 0.0
            return self.hidden
        if 0.0 < self.keep < 1.0:
            u = u01(self.seed, k)
            if self.hidden:
                if u < self.p_show:
                    self.hidden = False
            elif u < self.p_hide:
                self.hidden = True
        return self.hidden


def dropout_pattern(base_seed, keep_prob, mean_gap_s, k_on, n):
    """켠 프레임(k_on)부터 n 프레임의 숨김 여부 목록."""
    chain = DropoutChain(base_seed, keep_prob, mean_gap_s)
    return [chain.step(k_on + i) for i in range(n)]


def position_shift(seed, k, lon_offset_m, lon_std_m, lat_offset_m, lat_std_m, ego_yaw_deg):
    """객체 하나의 프레임 k 이동량. (noise_lon, noise_lat, wx, wy) — wx·wy 는 MORAI world 로 돌린 것.

    보낸 위치 = GT + (wx, wy). 기록하는 dx_m·dy_m 는 더한 뒤에 다시 뺀 값(보낸 − GT)이라 wx·wy 와 마지막 비트가 다를 수 있다.
    """
    z_lon, z_lat = gauss2(seed, k)
    noise_lon = lon_std_m * z_lon if lon_std_m > 0.0 else 0.0
    noise_lat = lat_std_m * z_lat if lat_std_m > 0.0 else 0.0
    d_lon = lon_offset_m + noise_lon
    d_lat = lat_offset_m + noise_lat
    psi = math.radians(ego_yaw_deg)
    wx = d_lon * math.cos(psi) - d_lat * math.sin(psi)
    wy = d_lon * math.sin(psi) + d_lat * math.cos(psi)
    return noise_lon, noise_lat, wx, wy


# ───────────────────────────────────────────────────────────── 프레임 단위 기준기
class Settings:
    """걸려 있는 외란 (브리지 DisturbanceSettings 의 필요한 부분). 종류마다 None = 꺼짐.

    position = (lon_offset_m, lon_std_m, lat_offset_m, lat_std_m) · dropout = (keep_prob, mean_gap_s) ·
    blackout = (duration_s, recover). *_cmd 는 그 종류를 건 명령 번호 — 값이 같아도 번호가 바뀌면 새 명령이다.
    """

    def __init__(self, base_seed=0, epoch_id=0):
        self.base_seed = base_seed
        self.epoch_id = epoch_id
        self.position = self.dropout = self.blackout = None
        self.position_trigger = self.dropout_trigger = self.blackout_trigger = ''
        self.position_cmd = self.dropout_cmd = self.blackout_cmd = 0
        self._seq = 0

    def set(self, kind, values, trigger=''):
        """kind 를 values 로 건다 (values None = 그 종류 해제)."""
        self._seq += 1
        setattr(self, kind, None if values is None else tuple(values))
        setattr(self, kind + '_trigger', trigger if values is not None else '')
        setattr(self, kind + '_cmd', self._seq)

    def clear(self):
        for kind in ('position', 'dropout', 'blackout'):
            self.set(kind, None)


class Disturber:
    """객체 프레임 하나씩 MRDIST-1 을 적용한다 (브리지 ObjectDisturber 와 같은 순서 — 시드 구간·카운터, 주기, 완전 끊김,
    간헐 끊김, 객체). apply() 가 돌려주는 dict 의 키는 DisturbanceStatus·ObjectDisturbance 필드 이름과 같다."""

    def __init__(self):
        self.epoch_id = None
        self.k = -1
        self.prev_stamp = None
        self._drop_key = self._black_key = None
        self._chain = None
        self._black_active = False
        self._black_elapsed = 0

    def apply(self, settings, actors, ego_yaw_deg, stamp_ns):
        """actors = [(이름, x, y)] (GT, MORAI 응답 순서). 돌려주는 값에 'sent' = [(이름, x, y)] (Autoware 로 보낸 목록)."""
        s = settings
        # 1) 시드 구간·카운터
        if s.epoch_id != self.epoch_id:
            self.epoch_id = s.epoch_id
            self.k = 0
            self._drop_key = self._black_key = None
            self._chain = None
            self._black_active = False
        else:
            self.k += 1
        k = self.k
        # 2) 주기
        per = period_ns(self.prev_stamp, stamp_ns)
        self.prev_stamp = stamp_ns
        # 3) 완전 끊김
        if s.blackout is None:
            self._black_active = False
            self._black_key = None
        elif (s.epoch_id, s.blackout_cmd) != self._black_key:
            self._black_key = (s.epoch_id, s.blackout_cmd)
            self._black_active = True
            self._black_elapsed = 0
        elif self._black_active:
            self._black_elapsed += per
            dur, recover = s.blackout
            if recover and self._black_elapsed >= duration_ns(dur):
                self._black_active = False
        # 4) 간헐 끊김 (완전 끊김 중에도 사슬은 계속 돈다)
        drop_hidden = False
        if s.dropout is None:
            self._chain = None
            self._drop_key = None
        else:
            if (s.epoch_id, s.dropout_cmd) != self._drop_key:
                self._drop_key = (s.epoch_id, s.dropout_cmd)
                self._chain = DropoutChain(s.base_seed, s.dropout[0], s.dropout[1])
            drop_hidden = self._chain.step(k)
        hidden = self._black_active or drop_hidden
        # 5) 객체
        pos_on = s.position is not None
        objects, sent = [], []
        for name, x, y in actors:
            seed = object_seed(s.base_seed, name)
            rec = dict(name=name, seed=seed, visible=not hidden, noise_lon_m=0.0, noise_lat_m=0.0, dx_m=0.0, dy_m=0.0)
            if not hidden:
                sx, sy = x, y
                if pos_on:
                    n_lon, n_lat, wx, wy = position_shift(seed, k, *s.position, ego_yaw_deg)
                    sx, sy = x + wx, y + wy
                    rec.update(noise_lon_m=n_lon, noise_lat_m=n_lat, dx_m=sx - x, dy_m=sy - y)   # 더한 뒤에 뺀다
                sent.append((name, sx, sy))
            objects.append(rec)
        black_on = self._black_active
        return dict(
            frame_seq=k, frame_period_s=per * 1e-9,
            fault_flag=pos_on or s.dropout is not None or black_on,
            position_on=pos_on, dropout_on=s.dropout is not None, blackout_on=black_on,
            objects_visible=not hidden,
            active_triggers=[t for on, t in ((pos_on, s.position_trigger), (s.dropout is not None, s.dropout_trigger),
                                             (black_on, s.blackout_trigger)) if on],
            lon_offset_m=s.position[0] if pos_on else 0.0, lon_std_m=s.position[1] if pos_on else 0.0,
            lat_offset_m=s.position[2] if pos_on else 0.0, lat_std_m=s.position[3] if pos_on else 0.0,
            keep_prob=s.dropout[0] if s.dropout is not None else 1.0,
            mean_gap_s=s.dropout[1] if s.dropout is not None else 0.0,
            blackout_duration_s=s.blackout[0] if black_on else 0.0,
            blackout_recover=bool(s.blackout[1]) if black_on else False,
            base_seed=s.base_seed, dropout_seed=dropout_seed(s.base_seed), seed_algorithm=ALGORITHM,
            objects=objects, sent=sent)
