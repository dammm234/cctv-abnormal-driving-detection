"""
이상운전 탐지 통합 파이프라인.

흩어진 부품을 하나로 연결:
  ReID(카메라 간 차량 연결) → 횡방향 궤적 수집 → 이상 메트릭(PTE/변위) → 판정

파이프라인:
1. ReID 매칭 결과(reid_multi_results.json)로 글로벌 차량 ID 구성.
   - "cam0:track_a ↔ cam1:track_b" 매칭들을 union-find로 묶어
     같은 실제 차량을 하나의 global_id로.
2. 각 global 차량의 횡방향(y) 위치 시계열을 3개 카메라에서 수집.
   (v1.1 JSON의 position_road_m[1] = CARLA y = 직선도로의 횡방향)
3. 이상 메트릭:
   - PTE: smoothed trajectory 대비 고주파 흔들림 → wobble
   - lateral_span: 횡방향 총 변위 → 차선 변경/이탈
4. 판정: 임계 기준 normal / lane_change / wobble.
"""
import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np

try:
    from scipy.signal import savgol_filter
    HAVE_SCIPY = True
except ImportError:
    HAVE_SCIPY = False


# ── 판정 임계값 (CARLA 직선도로, 차선폭 ~3.5m 기준) ──
PTE_WOBBLE_THRESHOLD = 0.15      # smoothed 대비 흔들림 std (m) — 참고용
LANE_CHANGE_SPAN = 1.75          # 횡방향 총 변위 (m). 차선폭 절반 초과 시 변경으로 간주

# wobble 강화: 진폭 게이트로 차선 안 미세 진동(미숙 운전)은 흔들림으로 안 셈.
# 0.3m = 차선폭의 ~8.5%. 미숙 운전의 미세 떨림(보통 ≤0.1m)과 확실히 구분되는
# 엄격한 기준. 졸음·음주급의 의미 있는 흔들림만 wobble로 카운트.
WOBBLE_MIN_AMP = 0.30            # 방향전환 카운트 최소 진폭 (m)
WOBBLE_ZIGZAG_FLOOR = 0.30       # zigzag 임계 자동산정의 하한 (회/초)

# (swerve 판정은 제거됨 — 이상은 wobble만. set_transform wobble 주입 시
#  횡위치 점프가 swerve 지표를 오염시켜, wobble 단일 판정으로 단순화함)


# ============ Union-Find: ReID 매칭 → 글로벌 ID ============

class UnionFind:
    def __init__(self):
        self.parent = {}

    def find(self, x):
        self.parent.setdefault(x, x)
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def build_global_ids(reid_results, scenario):
    """ReID 예측 매칭으로 (camera, track_id) → global_id 매핑 구성.

    ReID가 외형 feature로 예측한 매칭(matched_pairs) 전체를 union-find로
    묶는다. CARLA 정답 ID는 일절 보지 않는다 (correct 필드로 거르지 않음).
    즉 이 연결은 100% 외형 기반 ReID 결과다.

    union-find: a↔b, b↔c 매칭이 있으면 {a,b,c}가 한 글로벌 차량으로 전이적 병합.
    """
    uf = UnionFind()

    # 시나리오 결과 찾기
    target = None
    for res in reid_results['results']:
        if res['scenario'] == scenario:
            target = res
            break
    if target is None:
        raise ValueError(f'시나리오 {scenario} ReID 결과 없음')

    # 모든 예측 매칭을 union (correct 여부 무관 = 외형 기반)
    for pair_key, sc in target['pair_results'].items():
        cam_a, cam_b = pair_key.split('-')
        for m in sc['matched_pairs']:
            node_a = (cam_a, m['track_a'])
            node_b = (cam_b, m['track_b'])
            uf.union(node_a, node_b)

    # global_id 부여
    groups = defaultdict(list)
    for node in list(uf.parent.keys()):
        groups[uf.find(node)].append(node)

    node_to_global = {}
    for gid, (root, nodes) in enumerate(groups.items()):
        for node in nodes:
            node_to_global[node] = gid

    return node_to_global, target


# ============ 횡방향 궤적 수집 ============

def collect_lateral_trajectory(json_dir, scenario, node_to_global):
    """각 global 차량의 (frame, y, speed) 시계열을 카메라들에서 수집.

    return: {global_id: {'frames':[...], 'y':[...], 'cams':[...], 'speed':[...]}}
    """
    traj = defaultdict(lambda: {'frames': [], 'y': [], 'cams': [], 'speed': []})

    for cam in ['cam0', 'cam1', 'cam2']:
        path = os.path.join(json_dir, f'{scenario}_{cam}.json')
        if not os.path.exists(path):
            continue
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        for fr in data['frames']:
            fid = fr['frame_id']
            for v in fr['vehicles']:
                node = (cam, v['track_id'])
                if node not in node_to_global:
                    continue
                gid = node_to_global[node]
                pos = v.get('position_road_m')
                if pos is None or len(pos) < 2:
                    continue
                traj[gid]['frames'].append(fid)
                traj[gid]['y'].append(float(pos[1]))  # y = 횡방향
                traj[gid]['cams'].append(cam)
                traj[gid]['speed'].append(float(v.get('speed_est_mps', 0.0)))

    # frame 순 정렬
    for gid in traj:
        order = np.argsort(traj[gid]['frames'])
        traj[gid]['frames'] = list(np.array(traj[gid]['frames'])[order])
        traj[gid]['y'] = list(np.array(traj[gid]['y'])[order])
        traj[gid]['cams'] = list(np.array(traj[gid]['cams'])[order])
        traj[gid]['speed'] = list(np.array(traj[gid]['speed'])[order])

    return dict(traj)


# ============ 이상 메트릭 ============

def compute_pte(y_series, window=21, polyorder=3):
    """Path Tracking Error: smoothed 대비 흔들림 std."""
    arr = np.asarray(y_series, dtype=float)
    if len(arr) < window:
        return float(np.std(arr - np.mean(arr), ddof=1)) if len(arr) > 1 else 0.0
    if HAVE_SCIPY:
        w = window if window % 2 == 1 else window + 1
        w = min(w, len(arr) if len(arr) % 2 == 1 else len(arr) - 1)
        try:
            smoothed = savgol_filter(arr, w, polyorder)
        except Exception:
            smoothed = np.convolve(arr, np.ones(window) / window, mode='same')
    else:
        smoothed = np.convolve(arr, np.ones(window) / window, mode='same')
    return float(np.std(arr - smoothed, ddof=1))


def compute_zigzag_rate(y_series, fps=20.0, smooth_win=21, min_amp=WOBBLE_MIN_AMP):
    """좌우 방향 전환 빈도 (회/초), 진폭 게이트 적용. wobble의 핵심 지표.

    wobble = 좌우로 '의미 있는 폭'으로 반복 흔들림 → 방향 전환 많음.
    차선 변경 = 한 방향 이동 → 전환 적음.
    정상/미숙 = 미세 노이즈만 → min_amp 게이트로 걸러져 0.

    핵심: 방향 전환을 셀 때 직전 극값 대비 진폭이 min_amp(m) 이상일 때만
    카운트. min_amp를 0.3m로 키워 차선 안 미세 진동(운전 미숙으로 인한
    살짝 흔들림)은 흔들림으로 세지 않고, 졸음·음주처럼 의미 있는 폭으로
    반복 흔드는 경우만 잡는다.
    """
    arr = np.asarray(y_series, dtype=float)
    if len(arr) < smooth_win + 2:
        return 0.0
    if HAVE_SCIPY and len(arr) >= smooth_win:
        w = smooth_win if smooth_win % 2 == 1 else smooth_win + 1
        w = min(w, len(arr) if len(arr) % 2 == 1 else len(arr) - 1)
        try:
            sm = savgol_filter(arr, w, 2)
        except Exception:
            sm = np.convolve(arr, np.ones(smooth_win) / smooth_win, mode='same')
    else:
        sm = np.convolve(arr, np.ones(smooth_win) / smooth_win, mode='same')

    dy = np.diff(sm)
    band = 1e-4
    signs = np.where(dy > band, 1, np.where(dy < -band, -1, 0))
    nz_idx = np.where(signs != 0)[0]
    if len(nz_idx) < 2:
        return 0.0
    nz = signs[nz_idx]

    reversals = 0
    last_extreme = sm[0]
    for i in range(1, len(nz)):
        if nz[i] != nz[i - 1]:
            cur = sm[nz_idx[i]]
            if abs(cur - last_extreme) >= min_amp:
                reversals += 1
                last_extreme = cur
    duration_s = len(arr) / fps
    return reversals / duration_s if duration_s > 0 else 0.0


def classify_vehicle(y_series, zigzag_threshold, fps=20.0):
    """차량 1대의 궤적 → 이상 판정.

    이상(anomaly)은 wobble만. 차선 내 좌우 반복 흔들림 (졸음·음주 전형).
    - wobble: zigzag_rate(방향전환/초)가 임계 초과. 진폭 게이트(0.3m)로
      미숙 운전의 미세 흔들림은 제외.

    lane_change(횡변위 큼)는 정상 차선 변경일 수 있어 '이상'으로 보지 않고
    거동 이벤트로만 표시 (is_lane_change). 빈도 기반 위험도는 future work.
    PTE는 참고용으로만 계산.
    """
    arr = np.asarray(y_series, dtype=float)
    if len(arr) < 5:
        return {'verdict': 'insufficient_data', 'is_anomaly': False,
                'is_lane_change': False, 'lateral_span': 0.0,
                'pte': 0.0, 'zigzag': 0.0, 'n': len(arr)}

    lateral_span = float(arr.max() - arr.min())
    pte = compute_pte(arr)
    zigzag = compute_zigzag_rate(arr, fps=fps)

    # 이상(anomaly) 플래그: wobble만. 차선 내 좌우 반복 흔들림.
    flags = []
    if zigzag >= zigzag_threshold:
        flags.append('wobble')

    # lane_change는 '거동 이벤트'로 별도 기록 (이상 아님).
    # 정상적인 차선 변경일 수 있으므로 anomaly로 카운트하지 않는다.
    is_lane_change = lateral_span >= LANE_CHANGE_SPAN

    if flags:
        verdict = '+'.join(flags)        # 이상: wobble
    elif is_lane_change:
        verdict = 'lane_change'          # 거동 이벤트 (이상 아님, 표시만)
    else:
        verdict = 'normal'

    return {
        'verdict': verdict,
        'is_anomaly': bool(flags),       # wobble만 이상으로 카운트
        'is_lane_change': is_lane_change,
        'lateral_span': round(lateral_span, 3),
        'pte': round(pte, 4),
        'zigzag': round(zigzag, 3),
        'n': len(arr),
    }


# ============ 파이프라인 ============

def run_pipeline(reid_results_path, json_dir, scenario):
    with open(reid_results_path, encoding='utf-8') as f:
        reid_results = json.load(f)

    print(f'=== 이상운전 탐지 파이프라인: {scenario} ===\n')

    # 1. ReID → 글로벌 ID
    node_to_global, reid_target = build_global_ids(reid_results, scenario)
    n_global = len(set(node_to_global.values()))
    print(f'[1] ReID 차량 연결: {len(node_to_global)}개 관측(track) '
          f'→ {n_global}대 글로벌 차량')

    # 글로벌 ID별 구성 카메라 표시
    global_nodes = defaultdict(list)
    for node, gid in node_to_global.items():
        global_nodes[gid].append(node)
    for gid in sorted(global_nodes):
        nodes = sorted(global_nodes[gid])
        cams = ', '.join(f'{c}:t{t}' for c, t in nodes)
        print(f'    차량 G{gid}: {cams}')

    # 2. 횡방향 궤적 수집
    traj = collect_lateral_trajectory(json_dir, scenario, node_to_global)
    print(f'\n[2] 횡방향 궤적 수집 완료 ({len(traj)}대)')

    # 3-pre. zigzag(방향전환율) wobble 임계.
    # 고정값 사용. 자동산정은 wobble 차량이 차선폭 안에서 흔들리면 정상군에
    # 섞여 임계를 부풀리는 문제가 있어(오염), 안정적인 고정 임계로 판정.
    # wobble 흔들림은 zigzag 0.5+ , 정상 주행은 ~0.03 으로 명확히 갈리므로
    # 0.4를 기준으로 둔다 (정상 노이즈는 안 걸리고 흔들림은 확실히 잡힘).
    WOBBLE_ZIGZAG_THRESHOLD = 0.40
    zig_threshold = WOBBLE_ZIGZAG_THRESHOLD

    # 참고용: 정상 추정군(횡변위 작고 지그재그 낮은 차량)의 분포 출력
    base_zig = []
    for gid in traj:
        arr = np.asarray(traj[gid]['y'], dtype=float)
        if len(arr) < 5:
            continue
        z = compute_zigzag_rate(arr)
        if float(arr.max() - arr.min()) < LANE_CHANGE_SPAN and z < zig_threshold:
            base_zig.append(z)
    base_med = float(np.median(base_zig)) if base_zig else 0.0
    print(f'\n[3] wobble 임계(방향전환율): {zig_threshold:.2f} 회/초 (고정)')
    print(f'    참고: 정상 추정군 {len(base_zig)}대 지그재그 중앙값 {base_med:.3f}')
    print(f'    차선변경≥{LANE_CHANGE_SPAN}m (이상 아님, 거동 표시)\n')

    print(f"    {'차량':<6} {'카메라':<14} {'횡변위':>8} "
          f"{'지그재그':>9} {'프레임':>7}  {'판정'}")
    print('    ' + '-' * 70)

    results = []
    for gid in sorted(traj):
        t = traj[gid]
        cls = classify_vehicle(t['y'], zig_threshold)
        cams_seen = sorted(set(t['cams']))
        if cls['verdict'] == 'normal':
            verdict_mark = '✓정상'
        elif cls['verdict'] == 'lane_change':
            verdict_mark = '↪ 차선변경(이상아님)'
        else:
            verdict_mark = f"⚠ {cls['verdict']}"
        print(f"    G{gid:<5} {','.join(cams_seen):<14} "
              f"{cls['lateral_span']:>8.2f} {cls['zigzag']:>9.3f} "
              f"{cls['n']:>7}  {verdict_mark}")
        results.append({
            'global_id': gid,
            'cameras': cams_seen,
            'lateral_span_m': cls['lateral_span'],
            'zigzag_per_s': cls['zigzag'],
            'pte_m': cls['pte'],
            'n_frames': cls['n'],
            'verdict': cls['verdict'],
            'is_anomaly': cls['is_anomaly'],
            'is_lane_change': cls['is_lane_change'],
        })

    # 요약: 이상(wobble)과 차선변경(이상 아님)을 분리 집계
    n_anomaly = sum(1 for r in results if r['is_anomaly'])
    n_lane = sum(1 for r in results if r['is_lane_change'] and not r['is_anomaly'])
    print(f'\n[4] 요약: 전체 {len(results)}대 중 '
          f'이상운전(흔들림) {n_anomaly}대'
          f' | 차선변경 {n_lane}대 (이상 아님, 참고)')

    return {
        'scenario': scenario,
        'n_global_vehicles': n_global,
        'lane_change_threshold_m': LANE_CHANGE_SPAN,
        'wobble_zigzag_threshold_per_s': round(zig_threshold, 3),
        'vehicles': results,
    }


def main():
    parser = argparse.ArgumentParser(description='이상운전 탐지 통합 파이프라인')
    parser.add_argument('--reid-results', default='reid_multi_results.json')
    parser.add_argument('--json-dir', default='scenarios_v1.1_multi')
    parser.add_argument('--scenario', default='multi_reid')
    parser.add_argument('--output', default='anomaly_pipeline_result.json')
    args = parser.parse_args()

    if not os.path.exists(args.reid_results):
        print(f'[오류] ReID 결과 없음: {args.reid_results}')
        print('먼저 reid_match_validate_multi.py 실행')
        sys.exit(1)

    result = run_pipeline(args.reid_results, args.json_dir, args.scenario)

    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump(result, f, indent=2, ensure_ascii=False)
    print(f'\n→ {args.output}')


if __name__ == '__main__':
    main()
