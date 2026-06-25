"""
ReID 모듈 (다중 차량) - Cross-Camera 매칭 + 헝가리안 할당 + GT 검증

단일 차량용 reid_match_validate.py를 다중 차량으로 재작성.

입력:
- reid_features_multi/{시나리오}.npz
    키: '{cam}_track_{track_id}_feature'  (reid_extract_features.py 출력)

출력:
- reid_multi_results.json
- reid_multi_report.md

매칭 채점 정의 (카메라 쌍 단위):
- GT 매칭: 두 카메라에 공통으로 등장하는 track_id 쌍 (id_a == id_b)
- 예측 매칭: 헝가리안이 1:1로 묶고 유사도 >= threshold 인 쌍
- TP: 예측 매칭의 두 track_id가 실제로 같은 id
- FP: 예측 매칭인데 id가 다름
- FN: GT 매칭인데 예측 못 함 (할당 안 됐거나 threshold 미달)

"""
import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np

try:
    from scipy.optimize import linear_sum_assignment
except ImportError:
    print('[오류] scipy 필요: pip install scipy')
    sys.exit(1)

import reid_eval_protocol as evalproto


DEFAULT_THRESHOLD = 0.3


class UnionFind:
    """전이적 연결용 (cam0=cam1, cam1=cam2 → cam0=cam2)."""
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


def cosine_similarity_matrix(feats_a, feats_b):
    """정규화된 feature 행렬 간 코사인 유사도 행렬 (N x M).

    feats_a: (N, D), feats_b: (M, D)
    """
    a = feats_a / (np.linalg.norm(feats_a, axis=1, keepdims=True) + 1e-8)
    b = feats_b / (np.linalg.norm(feats_b, axis=1, keepdims=True) + 1e-8)
    return a @ b.T


def load_features(npz_path):
    """npz에서 {camera: {track_id: feature}} 로드.

    reid_extract_features.py 저장 형식:
        '{cam}_track_{track_id}_feature'
    """
    data = np.load(npz_path, allow_pickle=True)
    features = defaultdict(dict)
    for key in data.files:
        if key.endswith('_feature'):
            parts = key.rsplit('_feature', 1)[0]   # 'cam0_track_137'
            tokens = parts.split('_')
            camera = tokens[0]                      # 'cam0'
            track_id = int(tokens[2])               # 137
            features[camera][track_id] = data[key]
    scenario = (data['scenario'].item()
                if 'scenario' in data.files else 'unknown')
    return dict(features), scenario


def match_pair_hungarian(tracks_a, feats_a, tracks_b, feats_b, threshold):
    """카메라 쌍 헝가리안 1:1 매칭.

    Args:
        tracks_a: [track_id...] (len N)
        feats_a: (N, D)
        tracks_b: [track_id...] (len M)
        feats_b: (M, D)

    Returns:
        matches: [(track_a, track_b, similarity), ...]  (threshold 통과분만)
        sim_matrix: (N, M)
    """
    if len(tracks_a) == 0 or len(tracks_b) == 0:
        return [], np.zeros((len(tracks_a), len(tracks_b)))

    sim = cosine_similarity_matrix(feats_a, feats_b)   # (N, M)
    # 헝가리안은 비용 최소화 → 유사도를 비용으로: cost = -sim
    cost = -sim
    row_ind, col_ind = linear_sum_assignment(cost)

    matches = []
    for r, c in zip(row_ind, col_ind):
        s = float(sim[r, c])
        if s >= threshold:   # gating
            matches.append((tracks_a[r], tracks_b[c], s))
    return matches, sim


def score_pair(matches, tracks_a, tracks_b):
    """카메라 쌍 매칭 채점 (다중 차량).

    GT 매칭 = 양쪽에 공통으로 존재하는 track_id (CARLA id 동일).
    """
    set_a = set(tracks_a)
    set_b = set(tracks_b)
    gt_common = set_a & set_b   # 양쪽에 다 나온 차량 = 매칭돼야 할 차량들

    tp = 0
    fp = 0
    matched_gt_ids = set()
    for ta, tb, s in matches:
        if ta == tb:
            tp += 1
            matched_gt_ids.add(ta)
        else:
            fp += 1
    # FN: GT 매칭인데 못 잡은 것
    fn = len(gt_common - matched_gt_ids)

    return {
        'n_gt': len(gt_common),
        'TP': tp, 'FP': fp, 'FN': fn,
        'matched_pairs': [
            {'track_a': int(ta), 'track_b': int(tb),
             'similarity': round(s, 4), 'correct': bool(ta == tb)}
            for ta, tb, s in matches
        ],
    }


def process_scenario(npz_path, threshold, adjacent_only=False):
    features, scenario = load_features(npz_path)
    cameras = sorted(features.keys())

    pair_results = {}
    total_tp = total_fp = total_fn = 0
    uf = UnionFind()   # 전이적 글로벌 연결용

    # 매칭할 카메라 쌍 결정
    # adjacent_only=True: 인접 쌍만 (cam0-cam1, cam1-cam2). cam0-cam2 직접매칭 제외.
    #   → 차가 cam0→cam1→cam2 순차 통과하므로 전이적으로 cam0-cam2 자동 연결.
    #   가장 어려운 원거리 직접매칭(cam0-cam2)을 회피 (camera link model, Hsu 2021).
    pairs_to_match = []
    for i, cam_a in enumerate(cameras):
        for cam_b in cameras[i + 1:]:
            if adjacent_only:
                # 인접 = 카메라 인덱스 차이 1
                ia = cameras.index(cam_a)
                ib = cameras.index(cam_b)
                if abs(ia - ib) != 1:
                    continue
            pairs_to_match.append((cam_a, cam_b))

    for cam_a, cam_b in pairs_to_match:
        tracks_a = sorted(features[cam_a].keys())
        tracks_b = sorted(features[cam_b].keys())
        feats_a = np.array([features[cam_a][t] for t in tracks_a]) \
            if tracks_a else np.zeros((0, 1))
        feats_b = np.array([features[cam_b][t] for t in tracks_b]) \
            if tracks_b else np.zeros((0, 1))

        matches, _ = match_pair_hungarian(
            tracks_a, feats_a, tracks_b, feats_b, threshold)
        sc = score_pair(matches, tracks_a, tracks_b)
        sc['n_tracks_a'] = len(tracks_a)
        sc['n_tracks_b'] = len(tracks_b)

        pair_key = f'{cam_a}-{cam_b}'
        pair_results[pair_key] = sc

        # 전이 연결: 매칭된 (cam,track) 노드를 union
        for ta, tb, s in matches:
            uf.union((cam_a, ta), (cam_b, tb))

        if not adjacent_only:
            total_tp += sc['TP']
            total_fp += sc['FP']
            total_fn += sc['FN']

    # adjacent_only일 때는 전이 클러스터 단위로 채점
    # (개별 쌍 합산이 아니라, 최종 글로벌 클러스터가 올바른지)
    if adjacent_only:
        total_tp, total_fp, total_fn = score_clusters(uf, features)

    p = total_tp / (total_tp + total_fp) if (total_tp + total_fp) else 0.0
    r = total_tp / (total_tp + total_fn) if (total_tp + total_fn) else 0.0
    f1 = 2 * p * r / (p + r) if (p + r) else 0.0

    # 표준 retrieval 평가 (mAP + CMC Rank-k) — VeRi/BoT 프로토콜
    retrieval = evalproto.retrieval_eval(features, max_rank=5)

    return {
        'scenario': scenario,
        'threshold': threshold,
        'cameras': cameras,
        'adjacent_only': adjacent_only,
        'pair_results': pair_results,
        'overall': {
            'TP': total_tp, 'FP': total_fp, 'FN': total_fn,
            'precision': round(p, 4), 'recall': round(r, 4), 'f1': round(f1, 4),
        },
        'retrieval': retrieval,
    }


def score_clusters(uf, features):
    """전이적 글로벌 클러스터를 카메라 쌍 단위 TP/FP/FN으로 채점.

    각 클러스터(=글로벌 차량으로 추정된 노드 묶음)에서, 같은 CARLA track_id
    끼리 묶였으면 올바른 연결. 카메라 쌍 관점에서:
    - 두 카메라에 공통 등장하는 track_id(GT 매칭) 대비
    - 클러스터가 그 둘을 같은 그룹에 넣었으면 TP, 다른 id를 같은 그룹에 넣으면 FP.

    채점 단위: 직접 비교 가능한 모든 카메라 쌍 (cam0-cam1, cam0-cam2, cam1-cam2).
    전이 연결 덕에 cam0-cam2도 (직접 매칭 안 했지만) 평가됨.
    """
    cameras = sorted(features.keys())
    # 클러스터 id 부여
    cluster = {}
    for node in list(uf.parent.keys()):
        cluster[node] = uf.find(node)

    tp = fp = fn = 0
    for i, cam_a in enumerate(cameras):
        for cam_b in cameras[i + 1:]:
            ta_set = set(features[cam_a].keys())
            tb_set = set(features[cam_b].keys())
            gt_common = ta_set & tb_set   # 두 카메라 공통 = 매칭돼야 할 차량

            # 이 카메라 쌍에서 같은 클러스터에 묶인 (track_a, track_b) 쌍 추출
            linked = []
            for ta in ta_set:
                na = (cam_a, ta)
                if na not in cluster:
                    continue
                for tb in tb_set:
                    nb = (cam_b, tb)
                    if nb not in cluster:
                        continue
                    if cluster[na] == cluster[nb]:
                        linked.append((ta, tb))

            matched_gt = set()
            for ta, tb in linked:
                if ta == tb:
                    tp += 1
                    matched_gt.add(ta)
                else:
                    fp += 1
            fn += len(gt_common - matched_gt)

    return tp, fp, fn


def make_report(all_results, output_path):
    lines = []
    lines.append('# ReID 다중 차량 Cross-Camera 매칭 검증\n')
    lines.append('헝가리안 1:1 최적 할당 + threshold gating. '
                 'GT는 CARLA track_id 동일 여부.\n')

    for res in all_results:
        lines.append(f"## 시나리오: {res['scenario']} "
                     f"(threshold={res['threshold']})\n")
        ov = res['overall']
        lines.append(f"- 매칭(헝가리안): TP={ov['TP']} FP={ov['FP']} FN={ov['FN']} "
                     f"| P={ov['precision']:.3f} R={ov['recall']:.3f} "
                     f"**F1={ov['f1']:.3f}**\n")
        rt = res['retrieval']
        lines.append(f"- Retrieval(VeRi/BoT 프로토콜): "
                     f"**mAP={rt['mAP']:.3f}**, Rank-1={rt['rank1']:.3f}, "
                     f"Rank-5={rt['rank5']:.3f} "
                     f"(valid query={rt['num_query']}, samples={rt['num_samples']})\n")
        cmc_str = ', '.join(f'R{k+1}={v:.3f}'
                            for k, v in enumerate(rt['cmc']))
        lines.append(f"- CMC: {cmc_str}\n")
        lines.append('| 카메라 쌍 | tracks A | tracks B | GT | TP | FP | FN |')
        lines.append('|---|---|---|---|---|---|---|')
        for pair, sc in res['pair_results'].items():
            lines.append(
                f"| {pair} | {sc['n_tracks_a']} | {sc['n_tracks_b']} | "
                f"{sc['n_gt']} | {sc['TP']} | {sc['FP']} | {sc['FN']} |")
        lines.append('')
        # 오매칭 상세 (FP) 표시 — 발표에서 "어떤 차끼리 헷갈렸나" 분석용
        for pair, sc in res['pair_results'].items():
            wrong = [m for m in sc['matched_pairs'] if not m['correct']]
            if wrong:
                lines.append(f"- {pair} 오매칭: " + ', '.join(
                    f"{m['track_a']}↔{m['track_b']}(sim={m['similarity']:.2f})"
                    for m in wrong))
        lines.append('')

    lines.append('## 해석\n')
    lines.append('- 단일 차량(F1=1.000)과 달리 다중 차량에서는 후보가 여러 대라 '
                 '진짜 변별력이 요구됨.')
    lines.append('- 원거리 카메라(cam1/cam2)는 박스가 작아 feature 품질이 낮고, '
                 '이것이 현실 multi-camera ReID의 핵심 난점.')
    lines.append('- 헝가리안 할당으로 한 차량에 매칭이 몰리는 문제를 방지.')

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(
        description='ReID 다중 차량 cross-camera 매칭 (헝가리안)')
    parser.add_argument('--features-dir', default='reid_features_multi')
    parser.add_argument('--threshold', type=float, default=DEFAULT_THRESHOLD)
    parser.add_argument('--output-prefix', default='reid_multi')
    parser.add_argument('--sweep', action='store_true',
                        help='여러 threshold 스윕 (0.1~0.7)')
    parser.add_argument('--adjacent-only', action='store_true',
                        help='인접 카메라만 매칭 + 전이 연결 (camera link model)')
    args = parser.parse_args()

    if not os.path.isdir(args.features_dir):
        print(f'[오류] features 폴더 없음: {args.features_dir}')
        print('먼저 reid_extract_features.py 실행 (--output-dir reid_features_multi)')
        sys.exit(1)

    npz_files = sorted(f for f in os.listdir(args.features_dir)
                       if f.endswith('.npz'))
    if not npz_files:
        print(f'[오류] npz 없음: {args.features_dir}')
        sys.exit(1)

    print('=== ReID 다중 차량 매칭 (헝가리안 1:1 할당) ===')

    if args.sweep:
        thresholds = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        print('\n[Threshold Sweep]')
        print(f"{'thr':>6} {'TP':>5} {'FP':>5} {'FN':>5} "
              f"{'P':>7} {'R':>7} {'F1':>7}")
        for thr in thresholds:
            tp = fp = fn = 0
            for fname in npz_files:
                res = process_scenario(
                    os.path.join(args.features_dir, fname), thr,
                    adjacent_only=args.adjacent_only)
                tp += res['overall']['TP']
                fp += res['overall']['FP']
                fn += res['overall']['FN']
            p = tp / (tp + fp) if (tp + fp) else 0.0
            r = tp / (tp + fn) if (tp + fn) else 0.0
            f1 = 2 * p * r / (p + r) if (p + r) else 0.0
            print(f"{thr:>6.1f} {tp:>5} {fp:>5} {fn:>5} "
                  f"{p:>7.3f} {r:>7.3f} {f1:>7.3f}")
        print()

    all_results = []
    for fname in npz_files:
        res = process_scenario(
            os.path.join(args.features_dir, fname), args.threshold,
            adjacent_only=args.adjacent_only)
        all_results.append(res)
        ov = res['overall']
        print(f"\n[{res['scenario']}] (threshold={args.threshold})")
        for pair, sc in res['pair_results'].items():
            print(f"  {pair}: A={sc['n_tracks_a']} B={sc['n_tracks_b']} "
                  f"GT={sc['n_gt']} TP={sc['TP']} FP={sc['FP']} FN={sc['FN']}")
        print(f"  → [매칭] P={ov['precision']:.3f} R={ov['recall']:.3f} "
              f"F1={ov['f1']:.3f}")
        rt = res['retrieval']
        print(f"  → [retrieval] mAP={rt['mAP']:.3f} "
              f"Rank-1={rt['rank1']:.3f} Rank-5={rt['rank5']:.3f} "
              f"(query={rt['num_query']}, samples={rt['num_samples']})")

    json_path = f'{args.output_prefix}_results.json'
    with open(json_path, 'w', encoding='utf-8') as f:
        json.dump({'threshold': args.threshold, 'results': all_results},
                  f, indent=2, ensure_ascii=False)
    print(f'\n→ {json_path}')

    md_path = f'{args.output_prefix}_report.md'
    make_report(all_results, md_path)
    print(f'→ {md_path}')


if __name__ == '__main__':
    main()
