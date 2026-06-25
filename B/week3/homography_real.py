"""
Step (B-2): 실제 CCTV 영상의 4점 대응 호모그래피

영상에서 프레임 추출 → 사용자가 도로 위 4점 클릭 → 한국 고속도로 표준 치수로
OpenCV findHomography로 픽셀↔월드 변환 매트릭스 계산.


사용법:
    python homography_real.py clip01_gumi_view1
    python homography_real.py clip02_gumi_view2 --frame 90
    python homography_real.py clip01_gumi_view1 --width 3.5 --length 26

  --frame: 추출할 프레임 인덱스 (default 30)
  --width: 4점이 이루는 사각형의 폭 (m, default 3.5)
  --length: 사각형의 길이 (m, default 13)


출력:
    data/real/{clip}_frame.png            (추출된 프레임)
    config/homography_real_{clip}.json    (호모그래피 매트릭스)
    data/real/{clip}_homography_viz.png   (검증 시각화)
"""
import argparse
import json
import os

import cv2
import matplotlib.pyplot as plt
import numpy as np


REAL_DIR = 'data/real'
OUTPUT_DIR = 'config'


def extract_frame(video_path, output_path, frame_idx):
    """영상에서 frame_idx번째 프레임 추출."""
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f'  [ERROR] 영상을 열 수 없음: {video_path}')
        return False

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if frame_idx >= total:
        print(f'  [ERROR] frame {frame_idx} 너무 큼 (총 {total} 프레임)')
        cap.release()
        return False

    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_idx)
    ret, frame = cap.read()
    cap.release()

    if not ret:
        print('  [ERROR] 프레임 읽기 실패')
        return False

    cv2.imwrite(output_path, frame)
    return True


def click_4_points(image_path, width, length):
    """이미지 표시 후 사용자에게 4점 클릭 받음."""
    img_bgr = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    fig, ax = plt.subplots(figsize=(15, 9))
    ax.imshow(img_rgb)
    ax.set_title(
        f'Click 4 corners of a road rectangle (width {width}m x length {length}m).\n'
        'Order: P1 (near-left) -> P2 (near-right) -> P3 (far-right) -> P4 (far-left)\n'
        '"near" = closer to bottom of image, "far" = closer to top'
    )
    ax.axis('on')

    print('\n  이미지 창이 열렸어요. 4점을 차례로 클릭하세요:')
    print('    P1 (near-left)  : 가까운 쪽 왼쪽 모서리')
    print(f'    P2 (near-right) : 가까운 쪽 오른쪽 (P1에서 {width}m 옆)')
    print(f'    P3 (far-right)  : 먼 쪽 오른쪽 (P2에서 {length}m 앞)')
    print('    P4 (far-left)   : 먼 쪽 왼쪽')

    points = plt.ginput(n=4, timeout=0, show_clicks=True)
    plt.close()

    if len(points) != 4:
        print(f'  [ERROR] 4점이 아닌 {len(points)}점 받음')
        return None

    return [(float(p[0]), float(p[1])) for p in points]


def visualize_homography(image_path, H_world_to_pixel, pixel_pts,
                         world_pts, output_path, grid_extent):
    """월드 1m 격자를 픽셀에 투영하여 검증."""
    img_bgr = cv2.imread(image_path)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    H, W = img_rgb.shape[:2]

    fig, axes = plt.subplots(1, 2, figsize=(22, 9))

    # Left panel: 4점 + 라벨
    axes[0].imshow(img_rgb)
    for i, ((u, v), (wx, wy)) in enumerate(zip(pixel_pts, world_pts)):
        axes[0].plot(u, v, 'o', color='red', markersize=14,
                     markerfacecolor='none', markeredgewidth=3)
        axes[0].plot(u, v, '+', color='red', markersize=18,
                     markeredgewidth=2)
        axes[0].annotate(
            f'P{i+1}\n({wx:.1f}, {wy:.1f})m',
            (u, v), textcoords="offset points", xytext=(12, -8),
            color='red', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                      alpha=0.85)
        )
    # 사각형 윤곽선
    pix_arr = pixel_pts + [pixel_pts[0]]
    axes[0].plot([p[0] for p in pix_arr], [p[1] for p in pix_arr],
                 '-', color='red', linewidth=2, alpha=0.6)
    axes[0].set_title('Clicked 4 points and world coords (m)')
    axes[0].axis('off')

    # Right panel: 1m 격자 투영
    axes[1].imshow(img_rgb)

    x_min, x_max = grid_extent[0]
    y_min, y_max = grid_extent[1]

    def project_world(points_world):
        """월드 (X, Y) 배열을 픽셀로 투영. shape: (N, 2)."""
        n = len(points_world)
        ws = np.column_stack([
            np.array(points_world)[:, 0],
            np.array(points_world)[:, 1],
            np.ones(n),
        ]).T  # (3, N)
        ps = H_world_to_pixel @ ws  # (3, N)
        return (ps[:2] / ps[2]).T  # (N, 2)

    # 세로선 (X 일정)
    for x in np.arange(x_min, x_max + 0.01, 1.0):
        ys = np.linspace(y_min, y_max, 100)
        pts = project_world(np.column_stack([np.full_like(ys, x), ys]))
        mask = (pts[:, 0] >= 0) & (pts[:, 0] < W) & \
               (pts[:, 1] >= 0) & (pts[:, 1] < H)
        if mask.any():
            color = 'lime' if abs(x % 3.5) < 0.01 else 'lightgreen'
            lw = 2 if abs(x % 3.5) < 0.01 else 0.8
            axes[1].plot(pts[mask, 0], pts[mask, 1], '-',
                         color=color, linewidth=lw, alpha=0.8)

    # 가로선 (Y 일정)
    for y in np.arange(y_min, y_max + 0.01, 1.0):
        xs = np.linspace(x_min, x_max, 100)
        pts = project_world(np.column_stack([xs, np.full_like(xs, y)]))
        mask = (pts[:, 0] >= 0) & (pts[:, 0] < W) & \
               (pts[:, 1] >= 0) & (pts[:, 1] < H)
        if mask.any():
            color = 'cyan' if y % 5 < 0.01 else 'lightblue'
            lw = 2 if y % 5 < 0.01 else 0.6
            axes[1].plot(pts[mask, 0], pts[mask, 1], '-',
                         color=color, linewidth=lw, alpha=0.7)

    # 원본 4점 강조
    for u, v in pixel_pts:
        axes[1].plot(u, v, 'o', color='red', markersize=12,
                     markerfacecolor='none', markeredgewidth=2.5)

    axes[1].set_title(
        f'World 1m grid projected to image\n'
        f'Green = constant X (every 1m, thick=3.5m lane width), '
        f'Cyan = constant Y (every 1m, thick=5m)\n'
        f'World extent: X={grid_extent[0]}m, Y={grid_extent[1]}m'
    )
    axes[1].axis('off')

    plt.tight_layout()
    plt.savefig(output_path, dpi=80, bbox_inches='tight')
    plt.close()
    print(f'  저장: {output_path}')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('clip_name', help='Clip 이름 (확장자 제외)')
    parser.add_argument('--frame', type=int, default=30,
                        help='추출할 프레임 인덱스 (default 30)')
    parser.add_argument('--width', type=float, default=3.5,
                        help='사각형 폭 m (default 3.5 = 차선 폭)')
    parser.add_argument('--length', type=float, default=13.0,
                        help='사각형 길이 m (default 13 = 점선 주기)')
    args = parser.parse_args()

    clip = args.clip_name
    video_path = os.path.join(REAL_DIR, f'{clip}.mp4')
    frame_path = os.path.join(REAL_DIR, f'{clip}_frame.png')
    output_json = os.path.join(OUTPUT_DIR, f'homography_real_{clip}.json')
    viz_path = os.path.join(REAL_DIR, f'{clip}_homography_viz.png')

    print('=' * 60)
    print(f'실제 CCTV 호모그래피: {clip}')
    print('=' * 60)

    # 1. 프레임 추출
    print(f'\n1. 영상에서 frame {args.frame} 추출')
    if not os.path.exists(video_path):
        print(f'  [ERROR] 영상 없음: {video_path}')
        return
    if not extract_frame(video_path, frame_path, args.frame):
        return
    print(f'  {frame_path}')

    # 2. 4점 클릭
    print(f'\n2. 4점 대응 입력 (사각형 {args.width}m × {args.length}m)')
    pixel_points = click_4_points(frame_path, args.width, args.length)
    if pixel_points is None:
        return

    print('\n  클릭된 픽셀 좌표:')
    for i, (u, v) in enumerate(pixel_points):
        print(f'    P{i+1}: ({u:.1f}, {v:.1f})')

    # 3. 월드 좌표 (직사각형)
    w, l = args.width, args.length
    world_points = np.array([
        [0.0, 0.0],    # P1 near-left
        [w,   0.0],    # P2 near-right
        [w,   l  ],    # P3 far-right
        [0.0, l  ],    # P4 far-left
    ], dtype=np.float32)

    print(f'\n  월드 좌표 (단위 미터):')
    for i, (x, y) in enumerate(world_points):
        labels = ['near-left', 'near-right', 'far-right', 'far-left']
        print(f'    P{i+1} ({labels[i]:11s}): ({x:.2f}, {y:.2f})')

    # 4. Homography 계산
    print('\n3. cv2.findHomography 계산')
    pix_arr = np.array(pixel_points, dtype=np.float32)
    H_pixel_to_world, _ = cv2.findHomography(pix_arr, world_points)
    if H_pixel_to_world is None:
        print('  [ERROR] findHomography 실패')
        return

    H_world_to_pixel = np.linalg.inv(H_pixel_to_world)

    print('  H_pixel_to_world:')
    print(H_pixel_to_world)

    # 5. Reprojection 오차
    print('\n4. Reprojection 오차 (4점 자기 검증)')
    pts_h = np.column_stack([pix_arr, np.ones(4)]).T
    projected = H_pixel_to_world @ pts_h
    projected = (projected[:2] / projected[2]).T

    total_err = 0
    for i in range(4):
        err_m = np.linalg.norm(projected[i] - world_points[i])
        total_err += err_m
        print(f'  P{i+1}: {err_m * 100:.2f} cm')
    avg_err_cm = (total_err / 4) * 100
    print(f'  평균: {avg_err_cm:.2f} cm '
          + ('(매우 양호)' if avg_err_cm < 5 else
             '(허용)' if avg_err_cm < 20 else '(점검 권장)'))

    # 6. JSON 저장
    print(f'\n5. JSON 저장: {output_json}')
    output = {
        'clip_name': clip,
        'frame_used': args.frame,
        'algorithm': 'opencv_findHomography_4point',
        'world_units': 'meters',
        'rectangle_dimensions': {'width': w, 'length': l},
        'pixel_points': pixel_points,
        'world_points': world_points.tolist(),
        'H_pixel_to_world': H_pixel_to_world.tolist(),
        'H_world_to_pixel': H_world_to_pixel.tolist(),
        'reprojection_error_cm': {
            f'P{i+1}': float(
                np.linalg.norm(projected[i] - world_points[i]) * 100
            )
            for i in range(4)
        },
    }
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    with open(output_json, 'w', encoding='utf-8') as f:
        json.dump(output, f, ensure_ascii=False, indent=2)

    # 7. 검증 시각화
    print('\n6. 검증 시각화 (월드 격자 → 픽셀 투영)')
    # 격자 범위: 사각형의 약 2배
    grid_extent = (
        (-w * 0.5, w * 1.5),     # X 좌우 약간 여유
        (-l * 0.2, l * 1.5),     # Y 앞뒤 여유
    )
    visualize_homography(frame_path, H_world_to_pixel, pixel_points,
                         world_points.tolist(), viz_path, grid_extent)

    print()
    print('=' * 60)
    print('완료')
    print('=' * 60)
    print('\n검증 시각화 확인:')
    print(f'  {viz_path}')
    print('\n  검증 통과 기준:')
    print('  - 좌측: 4점이 사각형 모양 (도로 위 합리적 위치)')
    print('  - 우측: 1m 격자가 도로 표면을 따라 정렬됨')
    print('  - 진한 초록선(3.5m 폭)이 실제 차선과 일치')
    print('  - 진한 청록선(5m 길이)이 점선 시작점과 일치')


if __name__ == '__main__':
    main()
