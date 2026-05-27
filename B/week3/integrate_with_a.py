"""
A의 v1.1 데이터로 B의 trajectory clustering 검증.

목적:
- A의 position_road_m (미터 좌표)로 B의 알고리즘 동작 확인
- A의 lane_id와 B의 클러스터링 결과 일치성 검증
- 발표용 비교 시각화

입력: A의 test_tracks_v1.1.json
출력:
- lane_hypotheses_a_video.json (A 영상 좌표계 차선 가설)
- trajectory_vs_a_lanes.png (B의 clustering vs A의 lane_id 비교)
"""
import json
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.distance import cdist
from sklearn.cluster import DBSCAN


# trajectory_clustering.py와 동일한 파라미터
MIN_TRAJECTORY_POINTS = 10
RESAMPLE_POINTS = 50
DBSCAN_EPS = 1.5      # A의 영상이 5차로 3.5m 간격이므로 약간 줄임
DBSCAN_MIN_SAMPLES = 2


def modified_hausdorff(a, b):
    """Modified Hausdorff distance (Dubuisson & Jain, 1994)."""
    d_ab = cdist(a, b).min(axis=1).mean()
    d_ba = cdist(b, a).min(axis=1).mean()
    return max(d_ab, d_ba)


def resample_arc_length(points, n):
    """Arc length 기준 균일 리샘플링."""
    if len(points) < 2:
        return points
    dists = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum = np.concatenate([[0], np.cumsum(dists)])
    total = cum[-1]
    if total < 1e-6:
        return np.tile(points[0], (n, 1))
    target = np.linspace(0, total, n)
    new_pts = []
    for t in target:
        idx = np.searchsorted(cum, t)
        if idx == 0:
            new_pts.append(points[0])
        elif idx >= len(points):
            new_pts.append(points[-1])
        else:
            ratio = (t - cum[idx - 1]) / (cum[idx] - cum[idx - 1])
            new_pts.append(points[idx - 1] + ratio
                           * (points[idx] - points[idx - 1]))
    return np.array(new_pts)


def load_a_data(json_path):
    """A의 v1.1 JSON 로드. position_road_m (미터)을 trajectory 점으로 사용."""
    with open(json_path, encoding='utf-8') as f:
        data = json.load(f)

    track_data = defaultdict(lambda: {'pts': [], 'lanes': []})

    for frame in data['frames']:
        for v in frame.get('vehicles', []):
            tid = v['track_id']
            track_data[tid]['pts'].append((
                frame['frame_id'],
                v['position_road_m'][0],
                v['position_road_m'][1],
            ))
            track_data[tid]['lanes'].append(v['lane_id'])

    trajectories = []
    for tid, d in track_data.items():
        d['pts'].sort(key=lambda p: p[0])
        coords = np.array([(p[1], p[2]) for p in d['pts']])
        if len(coords) < MIN_TRAJECTORY_POINTS:
            continue
        # 가장 많이 등장한 lane_id를 대표값으로
        lane_counts = defaultdict(int)
        for l in d['lanes']:
            lane_counts[l] += 1
        dominant_lane = max(lane_counts.items(), key=lambda x: x[1])[0]
        trajectories.append({
            'track_id': tid,
            'points': coords,
            'n_points': len(coords),
            'a_lane_id': dominant_lane,
            'a_lane_distribution': dict(lane_counts),
        })

    return trajectories, data


def cluster_trajectories(trajectories):
    """B의 trajectory clustering 적용."""
    # 리샘플링
    resampled = [resample_arc_length(t['points'], RESAMPLE_POINTS)
                 for t in trajectories]
    n = len(resampled)

    # 거리 행렬
    D = np.zeros((n, n))
    for i in range(n):
        for j in range(i + 1, n):
            d = modified_hausdorff(resampled[i], resampled[j])
            D[i, j] = D[j, i] = d

    # DBSCAN
    db = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES,
                metric='precomputed')
    labels = db.fit_predict(D)

    # 클러스터별 centerline 추출
    clusters = {}
    for label in set(labels):
        if label == -1:
            continue
        cluster_idx = [i for i, l in enumerate(labels) if l == label]
        members = [resampled[i] for i in cluster_idx]
        centerline = np.mean(members, axis=0)
        clusters[int(label)] = {
            'centerline': centerline,
            'member_count': len(cluster_idx),
            'members': [trajectories[i]['track_id']
                        for i in cluster_idx],
        }

    return labels, clusters, D


def visualize_comparison(trajectories, labels, clusters, output_path):
    """A의 lane_id vs B의 cluster 비교 시각화."""
    fig, axes = plt.subplots(1, 2, figsize=(16, 7))

    # 왼쪽: A의 lane_id 색칠
    ax = axes[0]
    lane_colors = {1: 'tab:red', 2: 'tab:orange', 3: 'tab:green',
                   4: 'tab:blue', 5: 'tab:purple'}
    seen_lanes = set()
    for t in trajectories:
        lane = t['a_lane_id']
        color = lane_colors.get(lane, 'gray')
        label = f'Lane {lane}' if lane not in seen_lanes else None
        seen_lanes.add(lane)
        ax.plot(t['points'][:, 0], t['points'][:, 1],
                '-', color=color, alpha=0.6, lw=1.5, label=label)

    # 5차로 경계선
    for x in [0, 3.5, 7, 10.5, 14, 17.5]:
        ax.axvline(x, color='k', linestyle=':', alpha=0.3, lw=0.8)

    ax.set_xlabel('x (m, road width)')
    ax.set_ylabel('y (m, road length)')
    ax.set_title("A's lane_id labeling\n(BEV 17.5m width divided into 3.5m intervals)")
    ax.legend(loc='upper right', fontsize=8)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 18)
    ax.invert_yaxis()

    # 오른쪽: B의 클러스터링 결과
    ax = axes[1]
    cmap = plt.get_cmap('tab10')
    n_clusters = len(clusters)
    seen_clusters = set()
    for i, t in enumerate(trajectories):
        cl = labels[i]
        if cl == -1:
            color = 'gray'
            label = 'Noise' if 'noise' not in seen_clusters else None
            seen_clusters.add('noise')
        else:
            color = cmap(cl % 10)
            label = f'Cluster {cl}' if cl not in seen_clusters else None
            seen_clusters.add(cl)
        ax.plot(t['points'][:, 0], t['points'][:, 1],
                '-', color=color, alpha=0.6, lw=1.5, label=label)

    # B의 centerline
    for cl, info in clusters.items():
        cl_color = cmap(cl % 10)
        cline = info['centerline']
        ax.plot(cline[:, 0], cline[:, 1],
                '-', color=cl_color, lw=4, alpha=0.9,
                label=f'Centerline {cl}')

    ax.set_xlabel('x (m, road width)')
    ax.set_ylabel('y (m, road length)')
    ax.set_title(f"B's trajectory clustering\n"
                 f"({n_clusters} clusters auto-detected)")
    ax.legend(loc='upper right', fontsize=7, ncol=2)
    ax.grid(True, alpha=0.3)
    ax.set_xlim(0, 18)
    ax.invert_yaxis()

    plt.suptitle("A's lane detection vs B's trajectory clustering — "
                 "Gyeongbu Dongtan Tunnel 5-lane CCTV (A's video)",
                 fontsize=13, y=1.00)
    plt.tight_layout()
    plt.savefig(output_path, dpi=100, bbox_inches='tight')
    plt.close()
    print(f'  저장: {output_path}')


def save_lane_hypotheses(clusters, trajectories, labels, output_path,
                          metadata):
    """B의 클러스터링 결과를 lane_hypotheses.json으로 저장."""
    lane_hypotheses = []
    for cl, info in clusters.items():
        cline = info['centerline']
        # 이 cluster에 속한 trajectory들의 dominant A lane_id 통계
        member_a_lanes = []
        for i, lab in enumerate(labels):
            if lab == cl:
                member_a_lanes.append(trajectories[i]['a_lane_id'])
        from collections import Counter
        a_lane_counter = Counter(member_a_lanes)
        dominant_a_lane = a_lane_counter.most_common(1)[0]

        lane_hypotheses.append({
            'cluster_id': cl,
            'centerline_points': cline.tolist(),
            'member_count': info['member_count'],
            'member_track_ids': info['members'],
            'avg_x': float(np.mean(cline[:, 0])),
            'avg_y': float(np.mean(cline[:, 1])),
            'matches_a_lane_id': dominant_a_lane[0],
            'a_lane_match_ratio':
                dominant_a_lane[1] / info['member_count'],
        })

    output = {
        'version': '1.0',
        'method': 'modified_hausdorff_dbscan',
        'source_video': metadata.get('video', 'unknown'),
        'source': 'role_a_v1.1',
        'fps': metadata.get('fps', 24.0),
        'total_frames': metadata.get('total_frames', 520),
        'coord_system': 'meter (BEV)',
        'algorithm_params': {
            'min_trajectory_points': MIN_TRAJECTORY_POINTS,
            'resample_points': RESAMPLE_POINTS,
            'dbscan_eps': DBSCAN_EPS,
            'dbscan_min_samples': DBSCAN_MIN_SAMPLES,
        },
        'n_clusters_found': len(clusters),
        'lane_hypotheses': lane_hypotheses,
    }

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    print(f'  저장: {output_path}')


def main():
    json_path = 'test_tracks_v1.1.json'
    print(f'1. A의 v1.1 데이터 로드: {json_path}')
    trajectories, data = load_a_data(json_path)
    print(f'   유효 trajectory: {len(trajectories)}개')

    print('\n2. B의 trajectory clustering 적용')
    labels, clusters, D = cluster_trajectories(trajectories)
    n_clusters = len(clusters)
    n_noise = list(labels).count(-1)
    print(f'   cluster 수: {n_clusters}, noise: {n_noise}')

    print('\n3. 결과 분석')
    print('   B의 cluster vs A의 lane_id 매칭:')
    for cl, info in sorted(clusters.items()):
        avg_x = np.mean(info['centerline'][:, 0])
        # 이 cluster의 member들이 A에서 어느 lane이었는지
        member_a_lanes = defaultdict(int)
        for i, lab in enumerate(labels):
            if lab == cl:
                member_a_lanes[trajectories[i]['a_lane_id']] += 1
        a_lane_str = ', '.join(f'L{l}:{c}' for l, c
                                in sorted(member_a_lanes.items()))
        print(f'     Cluster {cl}: avg_x={avg_x:5.2f}m, '
              f'members={info["member_count"]}, '
              f'A lanes: {a_lane_str}')

    print('\n4. 시각화 생성')
    visualize_comparison(trajectories, labels, clusters,
                          'trajectory_vs_a_lanes.png')

    print('\n5. lane_hypotheses.json 저장')
    metadata = {
        'video': '경부동탄터널 5차로 CCTV (A 영상)',
        'fps': data.get('fps', 24.0),
        'total_frames': data.get('total_frames', 520),
    }
    save_lane_hypotheses(clusters, trajectories, labels,
                          'lane_hypotheses_a_video.json', metadata)

    print('\n완료.')


if __name__ == '__main__':
    main()
