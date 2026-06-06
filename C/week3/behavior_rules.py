"""
C 모듈 — 행동 탐지 룰 (통합 인터페이스)
=========================================
A의 인식 파이프라인 출력(JSON v1.1)을 입력받아
차량별 위험 행동을 판정하고 의심 차량 리스트를 반환.

사용법 (A 파이프라인에서):
    from behavior_rules import detect_suspects

    result = detect_suspects(tracks_json)        # dict(JSON) 또는 파일경로
    result["suspects"]      # 의심 차량 상세 리스트
    result["suspect_ids"]   # 의심 차량 ID만 (간단 통합용)
    result["summary"]       # 사유별 집계

    # 파일로 저장하려면:
    detect_suspects(tracks_json, save_path="suspects.json")

행동 룰 (3종):
    - lane_weaving   : 차선 표류 (차선 변경 없이 측방 흔들림)
    - lane_change    : 급차선 변경
    - tailgating     : 짧은 차간거리
  (속도 위반은 CARLA 속도 분포 한계로 현재 제외)
"""
import json
import numpy as np
from collections import defaultdict

# --- 임계값 (CARLA 통제 시나리오 기준으로 검증된 값) ---
DEFAULT_THRESHOLDS = {
    "weaving_std": 0.30,     # lateral_offset 표준편차 (m)
    "lane_change": 1,        # 차선 변경 횟수
    "tail_gap": 5.0,         # 동일 차선 앞뒤 최소 간격 (m)
    "min_track_frames": 10,  # 이보다 짧은 트랙은 신뢰 불가 → 제외
}


def _load(data):
    """dict 또는 파일경로를 받아 dict로 반환."""
    if isinstance(data, str):
        with open(data, encoding="utf-8") as f:
            return json.load(f)
    return data


def _collect_tracks(d):
    """프레임 단위 JSON → track_id별 시계열."""
    tr = defaultdict(lambda: {"lo": [], "lane": [], "pos": []})
    for fr in d.get("frames", []):
        for v in fr.get("vehicles", []):
            t = v["track_id"]
            tr[t]["lo"].append(v["lateral_offset_m"])
            tr[t]["lane"].append(v["lane_id"])
            tr[t]["pos"].append(v["position_road_m"])
    return tr


def detect_suspects(data, thresholds=None, save_path=None):
    """
    메인 인터페이스.
    Parameters
    ----------
    data : dict | str   - JSON v1.1 데이터(dict) 또는 파일 경로
    thresholds : dict   - (선택) 임계값 덮어쓰기
    save_path : str     - (선택) 결과를 JSON 파일로 저장
    Returns
    -------
    dict {suspects, suspect_ids, summary, total_analyzed, thresholds}
    """
    TH = dict(DEFAULT_THRESHOLDS)
    if thresholds:
        TH.update(thresholds)

    d = _load(data)
    tr = _collect_tracks(d)
    valid = {t: s for t, s in tr.items()
             if len(s["lo"]) >= TH["min_track_frames"]}

    # tailgating: 같은 프레임·같은 차선에서 바짝 붙은 뒤차
    tail_ids = set()
    for fr in d.get("frames", []):
        by_lane = defaultdict(list)
        for v in fr.get("vehicles", []):
            by_lane[v["lane_id"]].append((v["position_road_m"][1], v["track_id"]))
        for ps in by_lane.values():
            ps.sort()
            for (y1, _), (y2, i2) in zip(ps, ps[1:]):
                if abs(y2 - y1) < TH["tail_gap"]:
                    tail_ids.add(i2)

    suspects = []
    summary = defaultdict(int)
    for tid, s in valid.items():
        reasons = []
        lane = np.array(s["lane"])
        changes = int(np.sum(np.abs(np.diff(lane)) >= 1))
        weaving_std = float(np.std(s["lo"]))

        # 룰1: 차선 표류 (차선 변경이 없을 때만)
        if changes == 0 and weaving_std > TH["weaving_std"]:
            reasons.append("lane_weaving")
            summary["lane_weaving"] += 1
        # 룰2: 급차선 변경
        if changes >= TH["lane_change"]:
            reasons.append("lane_change")
            summary["lane_change"] += 1
        # 룰3: 짧은 차간거리
        if tid in tail_ids:
            reasons.append("tailgating")
            summary["tailgating"] += 1

        if reasons:
            suspects.append({
                "track_id": int(tid),
                "reasons": reasons,
                "weaving_std": round(weaving_std, 3),
                "lane_changes": changes,
                "track_frames": len(s["lo"]),
            })

    suspects.sort(key=lambda x: -len(x["reasons"]))
    result = {
        "suspects": suspects,
        "suspect_ids": [s["track_id"] for s in suspects],
        "summary": dict(summary),
        "total_analyzed": len(valid),
        "thresholds": TH,
    }
    if save_path:
        with open(save_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)
    return result


if __name__ == "__main__":
    # 데모: 인자로 JSON 경로를 주면 분석 결과 출력
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else \
        "../../A/week3/v01/outputs/test_tracks_v1.1.json"
    r = detect_suspects(path)
    print(f"분석 차량: {r['total_analyzed']}대")
    print(f"의심 차량: {len(r['suspect_ids'])}대  ID={r['suspect_ids'][:20]}")
    print(f"사유별 집계: {r['summary']}")
