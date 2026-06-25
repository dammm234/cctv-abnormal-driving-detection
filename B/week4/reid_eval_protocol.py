"""
ReID 표준 평가 프로토콜: mAP + CMC Rank-k (query-gallery retrieval)

평가 단위:
- 각 (camera, track_id) = 하나의 sample, feature 1개.
- query를 각 sample로 순회.
- gallery = query와 '다른 카메라'의 모든 sample.
  (같은 카메라 sample은 cross-camera 평가에서 제외 = VeRi/Market 규칙)
- 정답(positive) = gallery 중 track_id가 query와 같은 sample.

지표:
- CMC Rank-k: 정렬된 gallery 상위 k 안에 정답이 하나라도 있으면 hit.
- mAP: query별 Average Precision의 평균.
"""
import numpy as np


def cosine_distance_matrix(query_feats, gallery_feats):
    """코사인 거리 (1 - cosine similarity). 작을수록 유사.

    query_feats: (Q, D), gallery_feats: (G, D)
    return: (Q, G)
    """
    q = query_feats / (np.linalg.norm(query_feats, axis=1, keepdims=True) + 1e-8)
    g = gallery_feats / (np.linalg.norm(gallery_feats, axis=1, keepdims=True) + 1e-8)
    sim = q @ g.T
    return 1.0 - sim


def build_samples(features):
    """{camera: {track_id: feat}} → 평탄화된 sample 리스트.

    return:
        feats: (N, D)
        pids:  (N,) track_id
        camids:(N,) 카메라 인덱스
        cam_names: 인덱스→이름
    """
    cam_names = sorted(features.keys())
    cam_to_idx = {c: i for i, c in enumerate(cam_names)}
    feats, pids, camids = [], [], []
    for cam in cam_names:
        for tid, f in features[cam].items():
            feats.append(np.asarray(f, dtype=float))
            pids.append(int(tid))
            camids.append(cam_to_idx[cam])
    if not feats:
        return (np.zeros((0, 1)), np.array([]), np.array([]), cam_names)
    return (np.vstack(feats), np.array(pids), np.array(camids), cam_names)


def evaluate_rank(dist_mat, q_pids, g_pids, q_camids, g_camids,
                  max_rank=5):
    """CMC + mAP 계산 (Market-1501 / VeRi 표준 규칙).

    각 query에 대해:
    - gallery를 거리 오름차순 정렬.
    - query와 같은 (pid, camid) gallery 샘플은 제외 (junk).
      → 여기선 sample이 track당 1개라 같은 camid의 같은 pid 자기 자신만 해당.
        cross-camera 평가이므로 같은 camid 전부 제외하는 변형을 쓴다.
    - 같은 camid gallery는 전부 제외(cross-camera 강제).
    """
    num_q, num_g = dist_mat.shape
    if num_q == 0 or num_g == 0:
        return np.zeros(max_rank), 0.0, 0

    indices = np.argsort(dist_mat, axis=1)   # 가까운 순
    # matches[i, j] = True if gallery (정렬 후 j번째) 가 query i와 같은 pid
    matches = (g_pids[indices] == q_pids[:, np.newaxis])

    all_cmc = []
    all_AP = []
    num_valid_q = 0

    for i in range(num_q):
        q_pid = q_pids[i]
        q_cam = q_camids[i]

        order = indices[i]
        # cross-camera: 같은 카메라의 gallery 전부 제거
        remove = (g_camids[order] == q_cam)
        keep = ~remove

        orig_match = matches[i][keep]
        if not np.any(orig_match):
            # 다른 카메라에 정답이 없는 query → 평가 제외
            continue

        # CMC
        cmc = orig_match.cumsum()
        cmc[cmc > 1] = 1
        all_cmc.append(cmc[:max_rank])

        # AP
        num_rel = orig_match.sum()
        tmp_cmc = orig_match.cumsum()
        precision_at_hits = [tmp_cmc[k] / (k + 1.0)
                             for k in range(len(orig_match)) if orig_match[k]]
        AP = np.sum(precision_at_hits) / num_rel
        all_AP.append(AP)
        num_valid_q += 1

    if num_valid_q == 0:
        return np.zeros(max_rank), 0.0, 0

    all_cmc = np.asarray(all_cmc).astype(np.float32)
    cmc = all_cmc.sum(axis=0) / num_valid_q
    mAP = float(np.mean(all_AP))
    return cmc, mAP, num_valid_q


def retrieval_eval(features, max_rank=5):
    """한 시나리오의 모든 sample로 cross-camera retrieval 평가.

    각 sample을 query로, 다른 카메라 sample들을 gallery로.
    """
    feats, pids, camids, cam_names = build_samples(features)
    n = len(pids)
    if n < 2:
        return {'mAP': 0.0, 'cmc': [0.0] * max_rank,
                'rank1': 0.0, 'rank5': 0.0,
                'num_query': 0, 'num_samples': n, 'cameras': cam_names}

    dist = cosine_distance_matrix(feats, feats)
    # query=gallery=전체. 자기 자신은 거리 0이지만 같은 camid라 어차피 제외됨.
    cmc, mAP, num_valid = evaluate_rank(
        dist, pids, pids, camids, camids, max_rank=max_rank)

    return {
        'mAP': round(float(mAP), 4),
        'cmc': [round(float(c), 4) for c in cmc],
        'rank1': round(float(cmc[0]), 4) if len(cmc) > 0 else 0.0,
        'rank5': round(float(cmc[min(4, len(cmc) - 1)]), 4) if len(cmc) else 0.0,
        'num_query': int(num_valid),
        'num_samples': int(n),
        'cameras': cam_names,
    }
