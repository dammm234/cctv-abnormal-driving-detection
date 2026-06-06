"""
Role C 메트릭 검증 — 첫 실행
B의 24개 CARLA 시나리오에 메트릭을 적용해서
강도별로 점수가 단조 증가하는지 확인.
"""
import json, glob, os
import numpy as np
from collections import defaultdict

SCN = 'scenarios_v1.1'

def load(path):
    with open(path, encoding='utf-8') as f:
        return json.load(f)

def extract(data):
    """단일 차량 시계열 추출"""
    t, lo, sp = [], [], []
    for fr in data['frames']:
        if not fr.get('vehicles'): continue
        v = fr['vehicles'][0]
        t.append(fr['timestamp_sec'])
        lo.append(v['lateral_offset_m'])
        sp.append(v['speed_est_mps'])
    return np.array(t), np.array(lo), np.array(sp)

# --- 메트릭 1: 차선 흔들림 (lateral offset 표준편차) ---
def weaving_score(lo):
    return float(np.std(lo))

# --- 메트릭 2: 급가속/급정거 (가속도 표준편차) ---
def abrupt_score(sp, fps):
    if len(sp) < 2: return 0.0
    accel = np.diff(sp) * fps
    return float(np.std(accel))

# --- 메트릭 3: jerk (가속도의 변화율 = 속도 2차 미분) — B가 원한 그것 ---
def jerk_score(sp, fps):
    if len(sp) < 3: return 0.0
    accel = np.diff(sp) * fps
    jerk = np.diff(accel) * fps
    return float(np.std(jerk))

ORDER = ['normal_50kmh','normal_60kmh',
         'wobble_mild','wobble_medium','wobble_strong',
         'abrupt_mild','abrupt_medium','abrupt_strong']

print(f"{'시나리오':<16} {'weaving':>9} {'abrupt':>9} {'jerk':>9}")
print("-"*48)
results = {}
for name in ORDER:
    path = os.path.join(SCN, f'{name}_cam2.json')
    data = load(path)
    fps = data['fps']
    t, lo, sp = extract(data)
    w = weaving_score(lo)
    a = abrupt_score(sp, fps)
    j = jerk_score(sp, fps)
    results[name] = (w, a, j)
    print(f"{name:<16} {w:>9.3f} {a:>9.3f} {j:>9.3f}")

print("\n=== 단조 증가 검증 ===")
def check(label, keys, idx, metric_name):
    vals = [results[k][idx] for k in keys]
    ok = all(vals[i] < vals[i+1] for i in range(len(vals)-1))
    arrow = " < ".join(f"{v:.3f}" for v in vals)
    mark = "✅ 통과" if ok else "❌ 깨짐"
    print(f"[{metric_name}] {label}: {arrow}  {mark}")

check("wobble mild<med<strong", ['wobble_mild','wobble_medium','wobble_strong'], 0, "weaving")
check("abrupt mild<med<strong", ['abrupt_mild','abrupt_medium','abrupt_strong'], 1, "abrupt ")
check("abrupt mild<med<strong", ['abrupt_mild','abrupt_medium','abrupt_strong'], 2, "jerk   ")

# normal이 이상행동보다 낮은지
print("\n=== normal이 가장 낮은지 ===")
nw = max(results['normal_50kmh'][0], results['normal_60kmh'][0])
print(f"weaving: normal 최대={nw:.3f}, wobble_mild={results['wobble_mild'][0]:.3f}  {'✅' if nw < results['wobble_mild'][0] else '❌'}")
na = max(results['normal_50kmh'][1], results['normal_60kmh'][1])
print(f"abrupt : normal 최대={na:.3f}, abrupt_mild={results['abrupt_mild'][1]:.3f}  {'✅' if na < results['abrupt_mild'][1] else '❌'}")
