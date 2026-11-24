#!/usr/bin/env python3
# 신규 — 26.11.13 _ YSH _ autoware.rviz 를 그대로 두고 맨 끝에 "Disturbance (외란 · GT)" 그룹 하나만 더해
#   autoware_disturbance.rviz 를 만든다 (13_ Autoware 의 autoware_launch/rviz 안). e2e_simulator 의 RViz 와 같은 화면 + 디버그.
#
#   글자로 끼워 넣는다 (yaml 로 다시 쓰면 숫자·순서 모양이 바뀐다). 끼운 뒤 yaml 로 읽어
#   "원본 Displays + 그룹 하나" 말고는 같은지 확인한다 — 다르면 쓰지 않는다.
#
# 쓰는 법
#   python3 make_rviz.py            만들기 (이미 같으면 그대로)
#   python3 make_rviz.py --check    만들지 않고 대조만
import argparse
import os
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
# 26.11.24 _ YSH _ 공유본은 src/ 가 저장소 바로 아래다. 원문:
# RVIZ_DIR = os.path.join(os.path.dirname(HERE), 'MR_Morai-Autoware_Viz', 'src', 'launcher', 'autoware_launch',
#                         'autoware_launch', 'rviz')
_ROOT = os.path.dirname(HERE)
_AW = _ROOT if os.path.isdir(os.path.join(_ROOT, 'src')) else os.path.join(_ROOT, 'MR_Morai-Autoware_Viz')
RVIZ_DIR = os.path.join(_AW, 'src', 'launcher', 'autoware_launch', 'autoware_launch', 'rviz')
SRC = os.path.join(RVIZ_DIR, 'autoware.rviz')
DST = os.path.join(RVIZ_DIR, 'autoware_disturbance.rviz')

HEADER = ('# 신규 — 26.11.13 _ YSH _ 13_ scripts/make_rviz.py 가 만든다 (손으로 고치지 말 것).\n'
          '#   autoware.rviz 그대로 + 맨 끝 "Disturbance (외란 · GT)" 그룹 하나 (GT 회색 상자 · 외란 상태 한 줄)\n')


# 26.11.13 _ YSH _ 보고서 크기에서 GT 테두리가 흐려 굵고 진하게. 원문: Alpha 0.5 · Color 200; 200; 200 · Line Width 0.03
def class_block(name):
    return (f'          {name}:\n'
            f'            Alpha: 0.8\n'
            f'            Color: 210; 210; 210\n')


# 들여쓰기: 맨 위 Displays 항목은 "    - " (4칸)
GROUP = (
    '    - Class: rviz_common/Group\n'
    '      Displays:\n'
    '        - BUS:\n'
    '            Alpha: 0.8\n'
    '            Color: 210; 210; 210\n'
    + class_block('CAR') + class_block('CYCLIST') +
    '          Class: autoware_perception_rviz_plugin/TrackedObjects\n'
    '          Display 3d polygon: true\n'
    '          Display Label: false\n'
    '          Display PoseWithCovariance: false\n'
    '          Display Predicted Path Confidence: false\n'
    '          Display Predicted Paths: false\n'
    '          Display Twist: false\n'
    '          Display UUID: false\n'
    '          Display Velocity: false\n'
    '          Line Width: 0.08\n'
    '          Enabled: true\n'
    + class_block('MOTORCYCLE') +
    '          Name: GroundTruth (MORAI 정답)\n'
    '          Namespaces: {}\n'
    + class_block('PEDESTRIAN') + class_block('TRAILER') + class_block('TRUCK') +
    '          Topic:\n'
    '            Depth: 5\n'
    '            Durability Policy: Volatile\n'
    '            History Policy: Keep Last\n'
    '            Reliability Policy: Best Effort\n'
    '            Value: /viz/debug/ground_truth/objects\n'
    + class_block('UNKNOWN') +
    '          Value: true\n'
    '        - Class: autoware_string_stamped_rviz_plugin/StringStampedOverlayDisplay\n'
    '          Enabled: true\n'
    '          Font Size: 14\n'
    '          Left: 10\n'
    '          Max Letter Num: 90\n'
    '          Name: DisturbanceStatus (외란 상태)\n'
    '          Text Color: 51; 201; 220\n'
    '          Time To Erase Last Diag: 2\n'
    '          Time To Keep Last Diag: 1\n'
    '          Top: 120\n'
    '          Topic:\n'
    '            Depth: 1\n'
    '            Durability Policy: Volatile\n'
    '            History Policy: Keep Last\n'
    '            Reliability Policy: Reliable\n'
    '            Value: /viz/debug/disturbance_text\n'
    '          Value: true\n'
    '          Value height offset: 0\n'
    '      Enabled: true\n'
    '      Name: Disturbance (외란 · GT)\n'
)


def build(src_text):
    lines = src_text.splitlines(keepends=True)
    try:
        vm = lines.index('Visualization Manager:\n')
        dp = lines.index('  Displays:\n', vm)
    except ValueError:
        raise SystemExit('make_rviz: autoware.rviz 모양이 예상과 다르다 (Visualization Manager / Displays 줄 없음)')
    end = None
    for i in range(dp + 1, len(lines)):
        ln = lines[i]
        if ln.startswith('  ') and not ln.startswith('   ') and ln.strip():
            end = i                                   # Displays 다음 2칸 들여쓰기 키 = 목록 끝
            break
    if end is None:
        raise SystemExit('make_rviz: Displays 목록 끝을 못 찾았다')
    return HEADER + ''.join(lines[:end]) + GROUP + ''.join(lines[end:])


def verify(src_text, out_text):
    a = yaml.safe_load(src_text)
    b = yaml.safe_load(out_text)
    extra = b['Visualization Manager']['Displays'][-1]
    b['Visualization Manager']['Displays'] = b['Visualization Manager']['Displays'][:-1]
    if a != b:
        return 'autoware.rviz 와 그룹 말고도 다른 데가 있다'
    if extra.get('Name') != 'Disturbance (외란 · GT)' or len(extra.get('Displays') or []) != 2:
        return '더한 그룹 모양이 틀렸다'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    args = ap.parse_args()
    with open(SRC, encoding='utf-8') as f:
        src_text = f.read()
    out_text = build(src_text)
    err = verify(src_text, out_text)
    if err:
        print('make_rviz: ' + err)
        return 1
    old = None
    if os.path.exists(DST):
        with open(DST, encoding='utf-8') as f:
            old = f.read()
    if args.check:
        print('make_rviz: 대조 통과 (원본 + 그룹 하나)' + ('' if old == out_text else ' — 단, 파일이 낡았다 (다시 만들 것)'))
        return 0 if old == out_text else 1
    if old == out_text:
        print(f'make_rviz: 그대로 — {DST}')
        return 0
    with open(DST, 'w', encoding='utf-8') as f:
        f.write(out_text)
    print(f'make_rviz: 썼다 — {DST} (원본 + "Disturbance (외란 · GT)" 그룹)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
