"""
이상운전 탐지 데모 영상 생성 (카메라별 개별 영상).

anomaly_pipeline의 판정 결과를 각 카메라 영상에 오버레이:
- 정상      → 초록 박스
- 차선 변경  → 주황 박스
- wobble    → 빨강 박스
- 관찰 중    → 회색 박스 (판정 보류)
박스 위 라벨: "G{global_id} {판정}"
상단 바: 카메라명 + 실시간 정상/관찰/이상 카운트.

3카메라를 가로로 붙이지 않고 cam0/cam1/cam2 각각 별도 mp4로 생성한다.
→ 각 화면이 풀 해상도라 라벨이 크고 잘 보인다.

"""
import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np

try:
    import cv2
except ImportError:
    print('[오류] opencv-python 필요: pip install opencv-python')
    sys.exit(1)

# 파이프라인 로직 재사용
import anomaly_pipeline as ap


CAMERA_IDS = ['cam0', 'cam1', 'cam2']

# 판정별 색 (BGR)
VERDICT_COLORS = {
    'normal': (80, 200, 80),         # 초록
    'lane_change': (200, 150, 0),    # 파랑/청록 (거동 이벤트, 이상 아님)
    'wobble': (60, 60, 230),         # 빨강 (이상)
    'observing': (180, 180, 180),    # 회색 (관찰 중, 판정 보류)
}
DEFAULT_COLOR = (200, 200, 200)


def verdict_color(verdict):
    """이상(wobble) 우선 표시. lane_change는 중립색."""
    if verdict == 'observing':
        return VERDICT_COLORS['observing']
    if 'wobble' in verdict:
        return VERDICT_COLORS['wobble']
    if 'lane_change' in verdict:
        return VERDICT_COLORS['lane_change']
    if verdict == 'normal':
        return VERDICT_COLORS['normal']
    return DEFAULT_COLOR


def verdict_label_ko(verdict):
    m = {'normal': 'NORMAL', 'lane_change': 'LANE CHANGE',
         'wobble': 'WOBBLE', 'observing': 'OBSERVING...'}
    if verdict in m:
        return m[verdict]
    # 복합
    return '+'.join(m.get(v, v.upper()) for v in verdict.split('+'))


def find_frames(scenario_dir, camera):
    """카메라 PNG 시퀀스 경로 목록."""
    cam_dir = os.path.join(scenario_dir, camera)
    if not os.path.isdir(cam_dir):
        return []
    files = sorted(
        f for f in os.listdir(cam_dir) if f.endswith('.png'))
    return [os.path.join(cam_dir, f) for f in files]


def build_track_verdict_map(reid_results_path, json_dir, scenario):
    """ (camera, track_id) → (global_id, verdict) 매핑 구성.

    anomaly_pipeline을 호출해 글로벌 ID + 판정을 얻고,
    그것을 카메라별 track_id로 역매핑.
    """
    with open(reid_results_path, encoding='utf-8') as f:
        reid_results = json.load(f)

    node_to_global, _ = ap.build_global_ids(reid_results, scenario)
    traj = ap.collect_lateral_trajectory(json_dir, scenario, node_to_global)

    # anomaly_pipeline과 동일한 고정 wobble(zigzag) 임계 (0.4 회/초).
    # 자동산정은 wobble 차량이 정상군에 섞여 임계를 부풀리는 오염 문제가 있어
    # 고정값 사용 (wobble은 0.5+, 정상은 ~0 으로 명확히 갈림).
    import numpy as np
    zig_threshold = 0.40

    global_verdict = {}
    for gid in traj:
        cls = ap.classify_vehicle(traj[gid]['y'], zig_threshold)
        global_verdict[gid] = cls['verdict']

    # (cam, track) → (gid, verdict)
    node_info = {}
    for node, gid in node_to_global.items():
        node_info[node] = (gid, global_verdict.get(gid, 'normal'))

    return node_info, global_verdict, node_to_global, traj, zig_threshold


def build_progressive_verdicts(traj, zig_threshold, json_dir, scenario,
                               node_to_global, update_every=5,
                               min_frames=20):
    """진짜 실시간 누적 판정: 각 프레임에서 '그 시점까지 실제 관측된 궤적'으로만 판정.

    전체 궤적의 최종 답을 미리 칠하지 않는다. 차가 아직 차선을 안 바꿨으면
    normal, 바꾸는 순간 lane_change로 전환된다 (미래를 모르는 실시간 동작).

    구현:
    - 각 글로벌 차량의 (frame_id → y)를 정리 (같은 fid 중복은 평균).
    - 정렬된 frame_id를 따라가며, 현재 fid까지의 y 시퀀스로 classify.
    - min_frames(기본 20=1.0s) 미만이면 observing (데이터 부족).
    - 결과를 frame_id별로 저장 → 영상에서 해당 fid로 조회.

    return: {global_id: {frame_id: verdict}}
    """
    import numpy as np

    progressive = {}
    for gid in traj:
        frames = traj[gid]['frames']
        ys = traj[gid]['y']
        if len(frames) < 1:
            progressive[gid] = {}
            continue

        # frame_id별 y 정리 (같은 fid 여러 카메라 → 평균)
        fid_to_ys = defaultdict(list)
        for fr, y in zip(frames, ys):
            fid_to_ys[fr].append(y)
        sorted_fids = sorted(fid_to_ys.keys())
        fid_mean_y = {fr: float(np.mean(fid_to_ys[fr])) for fr in sorted_fids}

        # 각 fid까지 누적된 y로 그 시점 판정
        verdict_by_frame = {}
        cumulative_y = []
        last_verdict = 'observing'
        for idx, fr in enumerate(sorted_fids):
            cumulative_y.append(fid_mean_y[fr])
            if idx % update_every == 0 or idx == len(sorted_fids) - 1:
                if len(cumulative_y) < min_frames:
                    last_verdict = 'observing'
                else:
                    cls = ap.classify_vehicle(cumulative_y, zig_threshold)
                    last_verdict = cls['verdict']
            verdict_by_frame[fr] = last_verdict
        progressive[gid] = verdict_by_frame

    return progressive


def load_bboxes(json_dir, scenario):
    """{camera: {frame_id: [(track_id, bbox_pixel), ...]}}"""
    per_cam = {}
    for cam in CAMERA_IDS:
        path = os.path.join(json_dir, f'{scenario}_{cam}.json')
        if not os.path.exists(path):
            per_cam[cam] = {}
            continue
        with open(path, encoding='utf-8') as f:
            data = json.load(f)
        frames = defaultdict(list)
        for fr in data['frames']:
            for v in fr['vehicles']:
                frames[fr['frame_id']].append((v['track_id'], v['bbox_pixel']))
        per_cam[cam] = dict(frames)
    return per_cam


def draw_box(img, bbox, color, label, scale=1.0):
    """박스 + 라벨. scale로 두께/폰트 크기 조절 (고해상도일수록 크게)."""
    x1, y1, x2, y2 = [int(c) for c in bbox]
    thick = max(2, int(3 * scale))
    font_scale = 0.6 * scale
    font_thick = max(1, int(2 * scale))
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX,
                                  font_scale, font_thick)
    cv2.rectangle(img, (x1, max(0, y1 - th - 10)),
                  (x1 + tw + 8, y1), color, -1)
    cv2.putText(img, label, (x1 + 4, y1 - 6),
                cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 255),
                font_thick)


def main():
    parser = argparse.ArgumentParser(description='이상운전 탐지 데모 영상')
    parser.add_argument('--reid-results', default='reid_multi_results.json')
    parser.add_argument('--json-dir', default='scenarios_v1.1_multi')
    parser.add_argument('--scenarios-dir', default='data/scenarios')
    parser.add_argument('--scenario', default='multi_reid')
    parser.add_argument('--output-dir', default='demo_videos')
    parser.add_argument('--target-width', type=int, default=1280,
                        help='카메라별 출력 영상 너비 (화질). 0이면 원본 해상도')
    parser.add_argument('--fps', type=float, default=24.0,
                        help='출력 영상 fps (높을수록 부드러움. 24=영상 표준)')
    parser.add_argument('--repeat', type=int, default=2,
                        help='각 프레임 반복 횟수 (느린 재생용). '
                             '1=원속도, 2=2배 느림')
    parser.add_argument('--update-every', type=int, default=10,
                        help='판정 갱신 간격 (프레임)')
    args = parser.parse_args()

    # 1. 글로벌 ID + 궤적 + 임계 (최종 판정은 요약용)
    (node_info, global_verdict, node_to_global,
     traj, zig_threshold) = build_track_verdict_map(
        args.reid_results, args.json_dir, args.scenario)
    n_total = len(global_verdict)
    print(f'[최종 판정] 전체 {n_total}대 (요약용)')
    for gid, vd in sorted(global_verdict.items()):
        print(f'  G{gid}: {vd}')

    # 1b. 프레임별 누적 판정 (정답 없이 관찰하며 판정이 바뀌는 과정)
    progressive = build_progressive_verdicts(
        traj, zig_threshold, args.json_dir, args.scenario,
        node_to_global, update_every=args.update_every)
    print(f'[프로그레시브] 프레임별 누적 판정 계산 완료 '
          f'(임계 zigzag={zig_threshold:.3f}/s)')

    # (cam, track) → global_id  (프레임별 판정은 progressive에서 조회)
    node_to_gid = {node: gid for node, gid in node_to_global.items()}

    # 2. bbox 로드
    bboxes = load_bboxes(args.json_dir, args.scenario)

    # 3. 프레임 경로
    scenario_dir = os.path.join(args.scenarios_dir, args.scenario)
    cam_frames = {cam: find_frames(scenario_dir, cam) for cam in CAMERA_IDS}
    n_frames = min((len(v) for v in cam_frames.values() if v), default=0)
    if n_frames == 0:
        print(f'[오류] 프레임 없음. scenarios-dir 확인: {scenario_dir}')
        sys.exit(1)
    print(f'[영상] 카메라별 개별 생성 (각 ~{n_frames}프레임 ×{args.repeat} '
          f'@ {args.fps}fps)')

    # 4. 크기 계산 (카메라별 동일 기준)
    sample = cv2.imread(cam_frames[CAMERA_IDS[0]][0])
    h0, w0 = sample.shape[:2]
    if args.target_width and args.target_width > 0:
        out_w = args.target_width
        scale = out_w / w0
        out_h = int(h0 * scale)
    else:
        out_w, out_h = w0, h0
        scale = 1.0
    top_bar_h = max(64, int(64 * scale))
    frame_h = out_h + top_bar_h

    os.makedirs(args.output_dir, exist_ok=True)

    # 5. 카메라별로 개별 영상 생성
    for cam in CAMERA_IDS:
        if not cam_frames[cam]:
            print(f'[{cam}] 프레임 없음, skip')
            continue

        out_path = os.path.join(
            args.output_dir, f'{args.scenario}_{cam}_anomaly.mp4')

        # 코덱: mp4v 우선 (외부 DLL 불필요, 항상 동작). 실패 시 avc1 시도.
        writer = None
        for codec in ['mp4v', 'avc1']:
            fourcc = cv2.VideoWriter_fourcc(*codec)
            writer = cv2.VideoWriter(out_path, fourcc, args.fps,
                                     (out_w, frame_h))
            if writer.isOpened():
                used_codec = codec
                break
        if not writer or not writer.isOpened():
            print(f'[오류] {cam} VideoWriter 열기 실패')
            continue

        n_cam_frames = len(cam_frames[cam])
        for fid in range(n_cam_frames):
            img = cv2.imread(cam_frames[cam][fid])
            if img is None:
                img = np.zeros((h0, w0, 3), dtype=np.uint8)

            # 이 프레임의 실시간 카운트 (해당 카메라에 보이는 차량 기준)
            live = {'normal': 0, 'observing': 0, 'anomaly': 0, 'lane': 0}
            seen = set()

            for track_id, bbox in bboxes.get(cam, {}).get(fid, []):
                node = (cam, track_id)
                gid = node_to_gid.get(node, None)
                if gid is None:
                    verdict = 'observing'
                    gid_disp = '?'
                else:
                    gid_disp = gid
                    verdict = progressive.get(gid, {}).get(fid, 'observing')
                    if gid not in seen:
                        seen.add(gid)
                        if verdict == 'observing':
                            live['observing'] += 1
                        elif verdict == 'normal':
                            live['normal'] += 1
                        elif verdict == 'lane_change':
                            live['lane'] += 1     # 이상 아님, 별도 집계
                        else:
                            live['anomaly'] += 1  # wobble만 이상
                color = verdict_color(verdict)
                label = f'G{gid_disp} {verdict_label_ko(verdict)}'
                draw_box(img, bbox, color, label, scale=scale)

            img = cv2.resize(img, (out_w, out_h))

            # 상단 정보 바
            top = np.zeros((top_bar_h, out_w, 3), dtype=np.uint8)
            t_sec = fid / 20.0   # 원본 20fps 기준 경과시간
            fs1 = 0.7 * scale
            fs2 = 0.55 * scale
            ft = max(1, int(2 * scale))
            title = (f'{cam.upper()}  t={t_sec:4.1f}s   '
                     f'NORMAL {live["normal"]}  '
                     f'ANOMALY {live["anomaly"]}  '
                     f'LANE-CHG {live["lane"]}')
            cv2.putText(top, title, (16, int(28 * scale)),
                        cv2.FONT_HERSHEY_SIMPLEX, fs1, (255, 255, 255), ft)
            cv2.putText(
                top,
                'green=normal  red=wobble  '
                'blue=lane change(not anomaly)  gray=observing',
                (16, int(52 * scale)),
                cv2.FONT_HERSHEY_SIMPLEX, fs2, (180, 180, 180), 1)
            full = np.vstack([top, img])

            for _ in range(args.repeat):
                writer.write(full)

        writer.release()
        out_len = n_cam_frames * args.repeat
        print(f'[{cam}] {used_codec} | {n_cam_frames}프레임 ×{args.repeat} '
              f'@ {args.fps}fps ≈ {out_len / args.fps:.1f}초 → {out_path}')

    print('\n=== 카메라별 영상 생성 완료 ===')


if __name__ == '__main__':
    main()
