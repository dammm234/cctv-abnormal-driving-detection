"""
Role C 메트릭 모듈 — 최종 정리
주력: weaving (lateral_offset 기반) — B/A 양쪽 작동
보조: abrupt — B에선 작동, A 실제CCTV에선 YOLO 노이즈로 신뢰도 낮음 (한계 명시)
"""
import json
import numpy as np
from collections import defaultdict

def weaving(lo): return float(np.std(lo)) if len(lo) >= 2 else 0.0
def abrupt(sp, fps):
    if len(sp) < 2: return 0.0
    return float(np.std(np.diff(sp) * fps))

SCN = '../../B/week3/scenarios_v1.1'
ORDER = ['normal_50kmh','normal_60kmh','wobble_mild','wobble_medium',
         'wobble_strong','abrupt_mild','abrupt_medium','abrupt_strong']

print("="*55)
print("[1] B 시나리오 검증 (ground truth) — 단조 증가")
print("="*55)
bres={}
for name in ORDER:
    d=json.load(open(f'{SCN}/{name}_cam2.json',encoding='utf-8'))
    fps=d['fps']
    lo=[fr['vehicles'][0]['lateral_offset_m'] for fr in d['frames'] if fr.get('vehicles')]
    sp=[fr['vehicles'][0]['speed_est_mps'] for fr in d['frames'] if fr.get('vehicles')]
    bres[name]=(weaving(lo),abrupt(sp,fps))
print(f"{'시나리오':<15}{'weaving':>9}{'abrupt':>9}")
for name in ORDER:
    print(f"{name:<15}{bres[name][0]:>9.3f}{bres[name][1]:>9.3f}")
w=[bres[k][0] for k in ['wobble_mild','wobble_medium','wobble_strong']]
print(f"\nweaving 단조증가: {w[0]:.3f}<{w[1]:.3f}<{w[2]:.3f}  {'통과' if w[0]<w[1]<w[2] else '실패'}")

print("\n"+"="*55)
print("[2] A 실제 CCTV 적용 — weaving 주력")
print("="*55)
d=json.load(open('../../A/week2/test_tracks_v1.1.json',encoding='utf-8'))
fps=d['fps']
tr=defaultdict(lambda:{'lo':[],'sp':[]})
for fr in d['frames']:
    for v in fr.get('vehicles',[]):
        tr[v['track_id']]['lo'].append(v['lateral_offset_m'])
        tr[v['track_id']]['sp'].append(v['speed_est_mps'])

# weaving 임계값: B의 normal(0.16) < TH < wobble_mild(0.605)
TH=0.30
rows=[(t,len(s['lo']),weaving(s['lo'])) for t,s in tr.items() if len(s['lo'])>=10]
rows.sort(key=lambda r:-r[2])
sus=[r for r in rows if r[2]>TH]
print(f"분석: {len(rows)}대 (10프레임 이상) / 전체 41대")
print(f"weaving 임계값 {TH} 초과 = 의심: {len(sus)}대\n")
print(f"{'순위':>3} {'ID':>4} {'프레임':>5} {'weaving':>8}")
for i,(t,n,wv) in enumerate(sus,1):
    print(f"{i:>3} {t:>4} {n:>5} {wv:>8.3f}")
