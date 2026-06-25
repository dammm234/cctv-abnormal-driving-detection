"""
Step G: 차량 trajectory 클러스터링 모듈

알고리즘:
1. 차량 ID별로 검출/관측을 묶어 trajectory 생성
2. 너무 짧은 trajectory 필터링
3. 모든 trajectory를 arc length 기준 동일 점 개수로 리샘플링
4. Modified Hausdorff distance로 쌍별 거리 행렬 계산
5. DBSCAN으로 클러스터링 (precomputed metric 모드)
6. 각 클러스터의 평균 trajectory를 차선 centerline으로 추출

입력 형태 (두 가지 모두 지원):
- A의 YOLO+ByteTrack CSV: frame_idx, timestamp, vehicle_id, x1, y1, x2, y2, confidence
  (image space, 픽셀 좌표)
- CARLA ground_truth.jsonl: world coordinates (미터)
- 같은 알고리즘이 양쪽 다 동작 (거리 단위만 다름)

출력:
    data/lane_hypotheses.json      (클러스터링 결과)
    data/trajectory_clustering_viz.png  (시각화)
"""

import csv
import json
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from sklearn.cluster import DBSCAN


SCENARIOS_DIR = 'data/scenarios'
OUTPUT_JSON = 'data/lane_hypotheses.json'
OUTPUT_VIZ = 'data/trajectory_clustering_viz.png'

# ============ 알고리즘 파라미터 ============
RESAMPLE_N = 50            # trajectory를 이 개수의 점으로 통일
DBSCAN_EPS = 2.0           # 같은 클러스터로 묶일 trajectory 최대 거리 (미터)
DBSCAN_MIN_SAMPLES = 2     # 클러스터 최소 trajectory 개수
MIN_TRAJECTORY_POINTS = 10 # 이보다 짧은 trajectory는 버림
# ==========================================


# ============ Data Loading ============

def load_trajectories_from_carla(scenarios_dir):
    """CARLA ground_truth.jsonl에서 trajectory 로드.
    각 시나리오 = 1개 trajectory (한 차량의 통과 기록).
    좌표계: world space (미터)."""
    trajectories = []
    if not os.path.isdir(scenarios_dir):
        return trajectories

    for name in sorted(os.listdir(scenarios_dir)):
        path = os.path.join(scenarios_dir, name)
        if not os.path.isdir(path):
            continue
        gt_path = os.path.join(path, 'ground_truth.jsonl')
        if not os.path.exists(gt_path):
            continue

        points = []
        with open(gt_path, encoding='utf-8') as f:
            for line in f:
                if not line.strip():
                    continue
                d = json.loads(line)
                if 'vehicles' not in d or not d['vehicles']:
                    continue
                loc = d['vehicles'][0]['location']
                points.append([loc[0], loc[1]])  # x, y (z 무시)

        if len(points) < MIN_TRAJECTORY_POINTS:
            print(f'  [SKIP] {name}: {len(points)} points (너무 짧음)')
            continue

        trajectories.append({
            'name': name,
            'points': np.array(points),
            'n_points': len(points),
            'source': 'carla',
        })

    return trajectories


def load_trajectories_from_csv(csv_path):
    """A의 YOLO+ByteTrack CSV에서 trajectory 로드.
    포맷: frame_idx, timestamp, vehicle_id, x1, y1, x2, y2, confidence
    trajectory 점 = bbox 하단 중심 ((x1+x2)/2, y2) — 지면 접점 근사.
    좌표계: image space (픽셀)."""
    detections_by_id = defaultdict(list)

    with open(csv_path, encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                vid = int(row['vehicle_id'])
                frame = int(row['frame_idx'])
                x1 = float(row['x1'])
                y1 = float(row['y1'])
                x2 = float(row['x2'])
                y2 = float(row['y2'])
                ground_x = (x1 + x2) / 2
                ground_y = y2
                detections_by_id[vid].append((frame, ground_x, ground_y))
            except (KeyError, ValueError):
                continue

    trajectories = []
    for vid, dets in detections_by_id.items():
        dets.sort(key=lambda d: d[0])
        points = [(x, y) for _, x, y in dets]
        if len(points) < MIN_TRAJECTORY_POINTS:
            continue
        trajectories.append({
            'name': f'vehicle_{vid}',
            'points': np.array(points),
            'n_points': len(points),
            'source': 'csv',
        })

    return trajectories


# ============ Trajectory Preprocessing ============

def resample_trajectory(points, n_samples=50):
    """trajectory를 arc length 기준 균일 리샘플링.
    원본 점들이 시간 간격으로 분포돼 있으면 속도에 따라 밀도가 다름.
    arc length로 리샘플하면 공간상 균등 분포 → 비교가 공평."""
    if len(points) < 2:
        return points

    diffs = np.diff(points, axis=0)
    seg_lens = np.sqrt((diffs ** 2).sum(axis=1))
    cum_lens = np.concatenate([[0], np.cumsum(seg_lens)])
    total = cum_lens[-1]

    if total < 1e-6:  # 거의 정지한 trajectory
        return np.tile(points[0], (n_samples, 1))

    sample_pos = np.linspace(0, total, n_samples)
    new_x = np.interp(sample_pos, cum_lens, points[:, 0])
    new_y = np.interp(sample_pos, cum_lens, points[:, 1])
    return np.column_stack([new_x, new_y])


# ============ Distance Metric ============

def modified_hausdorff(traj_a, traj_b):
    """대칭 modified Hausdorff distance.
    
    원본 Hausdorff: max{각 점의 최단 거리}
    Modified: mean{각 점의 최단 거리} — outlier에 robust
    대칭: max(h(A,B), h(B,A))
    
    Reference: Dubuisson & Jain (1994)"""
    # Pairwise distance matrix (na, nb)
    diffs = traj_a[:, np.newaxis, :] - traj_b[np.newaxis, :, :]
    dist_matrix = np.sqrt((diffs ** 2).sum(axis=2))

    d_ab = dist_matrix.min(axis=1).mean()  # A의 각 점 → 가장 가까운 B 점
    d_ba = dist_matrix.min(axis=0).mean()  # B의 각 점 → 가장 가까운 A 점
    return max(d_ab, d_ba)


def compute_distance_matrix(trajectories):
    """모든 trajectory 쌍의 distance matrix."""
    n = len(trajectories)
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            d = modified_hausdorff(trajectories[i], trajectories[j])
            D[i, j] = d
            D[j, i] = d
    return D


# ============ Clustering ============

def cluster_trajectories(distance_matrix, eps=2.0, min_samples=2):
    """DBSCAN으로 trajectory 클러스터링.
    
    eps: 같은 클러스터로 묶일 최대 거리
    min_samples: 클러스터 형성 최소 점 개수
    Returns: cluster label array. -1은 noise (outlier)."""
    db = DBSCAN(eps=eps, min_samples=min_samples, metric='precomputed')
    return db.fit_predict(distance_matrix)


def extract_lane_centerlines(resampled_trajectories, labels):
    """각 클러스터의 평균 trajectory를 차선 centerline으로 추출.
    
    리샘플링으로 모든 trajectory가 같은 점 개수라서 점별 평균 가능."""
    unique_labels = sorted(set(labels) - {-1})
    centerlines = []

    for lane_id in unique_labels:
        members = [resampled_trajectories[i]
                   for i, l in enumerate(labels) if l == lane_id]
        centerline = np.mean(np.stack(members), axis=0)
        centerlines.append({
            'lane_id': int(lane_id),
            'n_trajectories': len(members),
            'centerline': centerline.tolist(),
        })

    return centerlines


# ============ Visualization ============

def visualize(traj_data, resampled, labels, centerlines, output_path):
    """trajectory와 클러스터링 결과를 한 장에 시각화."""
    fig, ax = plt.subplots(figsize=(14, 6))
    cmap = plt.cm.tab10

    # 각 trajectory
    for i, (td, traj) in enumerate(zip(traj_data, resampled)):
        cl = labels[i]
        if cl == -1:
            color = 'gray'
            alpha = 0.4
            ls = ':'
            tag = 'NOISE'
        else:
            color = cmap(cl % 10)
            alpha = 0.65
            ls = '-'
            tag = f'C{cl}'
        ax.plot(traj[:, 0], traj[:, 1], color=color, alpha=alpha,
                linestyle=ls, linewidth=1.5,
                label=f'{td["name"]} ({tag})')

    # 차선 centerline (강조)
    for cl_info in centerlines:
        arr = np.array(cl_info['centerline'])
        ax.plot(arr[:, 0], arr[:, 1],
                color=cmap(cl_info['lane_id'] % 10),
                linewidth=4, alpha=0.95,
                label=f'==> Lane {cl_info["lane_id"]} centerline '
                      f'({cl_info["n_trajectories"]} trajs)')

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Vehicle Trajectory Clustering Result\n'
                 f'(modified Hausdorff + DBSCAN eps={DBSCAN_EPS}m)')
    ax.legend(loc='center left', bbox_to_anchor=(1.0, 0.5), fontsize=8)
    ax.grid(alpha=0.3)
    ax.set_aspect('equal', adjustable='box')

    plt.tight_layout()
    plt.savefig(output_path, dpi=100, bbox_inches='tight')
    plt.close()


# ============ Main ============

def main():
    print('=' * 60)
    print('Step G: 차량 trajectory 클러스터링')
    print('=' * 60)

    # 1. Load
    print(f'\n1. CARLA 시나리오에서 trajectory 로드')
    raw_trajectories = load_trajectories_from_carla(SCENARIOS_DIR)
    if not raw_trajectories:
        print(f'  [ERROR] {SCENARIOS_DIR}에 시나리오 없음')
        return

    for t in raw_trajectories:
        print(f'  {t["name"]:<22s} : {t["n_points"]} points')
    print(f'  총 {len(raw_trajectories)}개 trajectory')

    # 2. Resample
    print(f'\n2. {RESAMPLE_N}개 점으로 리샘플 (arc length 기준)')
    resampled = [resample_trajectory(t['points'], RESAMPLE_N)
                 for t in raw_trajectories]

    # 3. Distance matrix
    print(f'\n3. Modified Hausdorff distance matrix')
    D = compute_distance_matrix(resampled)
    upper = D[np.triu_indices(len(D), k=1)]
    print(f'  거리 범위: [{upper.min():.2f}m, {upper.max():.2f}m], '
          f'평균 {upper.mean():.2f}m')

    # Pretty print matrix
    names = [t['name'] for t in raw_trajectories]
    print(f'\n  Distance matrix (미터):')
    header = ' ' * 24 + ''.join(f'{n[:10]:>11s}' for n in names)
    print(f'  {header}')
    for i, n in enumerate(names):
        row = ''.join(f'{D[i,j]:>11.2f}' for j in range(len(names)))
        print(f'  {n:<22s} {row}')

    # 4. Cluster
    print(f'\n4. DBSCAN (eps={DBSCAN_EPS}, min_samples={DBSCAN_MIN_SAMPLES})')
    labels = cluster_trajectories(D, eps=DBSCAN_EPS,
                                  min_samples=DBSCAN_MIN_SAMPLES)

    print('  결과:')
    for i, t in enumerate(raw_trajectories):
        cl = labels[i]
        cl_str = 'NOISE' if cl == -1 else f'cluster {cl}'
        print(f'    {t["name"]:<22s} -> {cl_str}')

    n_clusters = len(set(labels) - {-1})
    n_noise = sum(1 for l in labels if l == -1)
    print(f'\n  클러스터 {n_clusters}개, noise {n_noise}개')

    # 5. Centerlines
    print(f'\n5. 차선 centerline 추출')
    centerlines = extract_lane_centerlines(resampled, labels)
    if not centerlines:
        print('  [WARN] 클러스터 없음. eps를 키워서 재시도 권장.')
    for cl in centerlines:
        arr = np.array(cl['centerline'])
        x_range = (arr[:, 0].min(), arr[:, 0].max())
        y_range = (arr[:, 1].min(), arr[:, 1].max())
        print(f'  Lane {cl["lane_id"]}: '
              f'{cl["n_trajectories"]}개 trajectory, '
              f'x range [{x_range[0]:.1f}, {x_range[1]:.1f}], '
              f'y range [{y_range[0]:.2f}, {y_range[1]:.2f}]')

    # 6. Save
    print(f'\n6. 결과 저장')
    os.makedirs(os.path.dirname(OUTPUT_JSON), exist_ok=True)
    output = {
        'algorithm': 'modified_hausdorff_dbscan',
        'parameters': {
            'resample_n': RESAMPLE_N,
            'dbscan_eps': DBSCAN_EPS,
            'dbscan_min_samples': DBSCAN_MIN_SAMPLES,
            'min_trajectory_points': MIN_TRAJECTORY_POINTS,
        },
        'n_input_trajectories': len(raw_trajectories),
        'n_clusters': n_clusters,
        'n_noise': n_noise,
        'trajectories': [
            {'name': t['name'], 'cluster_id': int(labels[i]),
             'n_points': t['n_points']}
            for i, t in enumerate(raw_trajectories)
        ],
        'lanes': centerlines,
    }
    with open(OUTPUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(output, f, ensure_ascii=False, indent=2)
    print(f'  {OUTPUT_JSON}')

    # 7. Visualize
    print(f'\n7. 시각화')
    visualize(raw_trajectories, resampled, labels, centerlines, OUTPUT_VIZ)
    print(f'  {OUTPUT_VIZ}')

    print()
    print('=' * 60)
    print('완료')
    print('=' * 60)


if __name__ == '__main__':
    main()
