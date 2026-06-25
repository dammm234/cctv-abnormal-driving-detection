"""
CARLA 카메라 호모그래피 계산 및 검증

CARLA cameras.yaml의 FOV, 이미지 크기, 위치, 회전 정보로부터
다음 매트릭스 자동 계산:
  - Intrinsic K (3×3)
  - World → Camera 변환 (4×4)
  - 지면 평면(z=0) 호모그래피 H (3×3)

CARLA 좌표계:
  +X = forward, +Y = right, +Z = up  (left-handed)

CARLA 카메라 local 좌표 (회전 적용 후):
  +X = forward (보는 방향)
  +Y = right
  +Z = up

OpenCV 카메라 좌표 (투영 시 사용):
  +X = right, +Y = down, +Z = forward

검증:
  wobble_strong 시나리오의 ground truth 차량 위치를 각 카메라로 투영하여
  실제 PNG 위에 trajectory 시각화. 투영된 점들이 실제 차량 위치와 일치하면 통과.

출력:
    config/homography_carla.json
    data/homography_validation/cam{0,1,2}_validation.png
"""
import json
import math
import os

import matplotlib.pyplot as plt
import numpy as np
import yaml
from PIL import Image


CAMERAS_CONFIG = 'config/cameras.yaml'
HOMOGRAPHY_OUTPUT = 'config/homography_carla.json'
VALIDATION_DIR = 'data/homography_validation'
VALIDATION_SCENARIO = 'wobble_strong'  # 가장 변화 큰 시나리오로 검증


# ============ Core Math Functions ============

def build_intrinsic_matrix(fov_deg, image_w, image_h):
    """카메라 intrinsic K (3×3).

    CARLA는 horizontal FOV 기준이므로 focal length는 image_w로 계산.
    pixel은 square 가정 (fx = fy).
    """
    fov_rad = math.radians(fov_deg)
    focal = image_w / (2.0 * math.tan(fov_rad / 2.0))
    K = np.array([
        [focal, 0.0,   image_w / 2.0],
        [0.0,   focal, image_h / 2.0],
        [0.0,   0.0,   1.0          ],
    ])
    return K


def build_carla_rotation_matrix(pitch_deg, yaw_deg, roll_deg):
    """CARLA pitch/yaw/roll → 회전 매트릭스 R (3×3).

    R은 camera_local → world 변환:
      world_col = R @ camera_local_col + T

    CARLA Unreal FRotator 컨벤션 (left-handed, row-vector convention)을
    column-vector convention으로 변환한 형태.
    """
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    roll = math.radians(roll_deg)

    cy, sy = math.cos(yaw), math.sin(yaw)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cr, sr = math.cos(roll), math.sin(roll)

    R = np.array([
        [cp * cy, cy * sp * sr - sy * cr, -cy * sp * cr - sy * sr],
        [cp * sy, sy * sp * sr + cy * cr, -sy * sp * cr + cy * sr],
        [sp,      -cp * sr,                cp * cr               ],
    ])
    return R


def build_world_to_camera_matrix(location, rotation):
    """World → CARLA camera local 변환 (4×4)."""
    R_cam_to_world = build_carla_rotation_matrix(
        rotation['pitch'], rotation['yaw'], rotation['roll']
    )
    T = np.array([location['x'], location['y'], location['z']])

    # World→Camera = inverse of Camera→World
    R_world_to_cam = R_cam_to_world.T
    t_world_to_cam = -R_world_to_cam @ T

    M = np.eye(4)
    M[:3, :3] = R_world_to_cam
    M[:3, 3] = t_world_to_cam
    return M


# CARLA camera local → OpenCV camera frame
# CARLA: X=forward, Y=right, Z=up
# OpenCV: X=right, Y=down, Z=forward
CARLA_TO_OPENCV = np.array([
    [0.0,  1.0,  0.0],   # CV_X = CARLA_Y
    [0.0,  0.0, -1.0],   # CV_Y = -CARLA_Z
    [1.0,  0.0,  0.0],   # CV_Z = CARLA_X
])


def world_to_pixel(world_point, K, world_to_camera):
    """월드 좌표 (x, y, z) → 픽셀 좌표 (u, v).

    Returns: (u, v) or None if behind camera.
    """
    p4 = np.array([world_point[0], world_point[1], world_point[2], 1.0])
    p_carla_cam = (world_to_camera @ p4)[:3]
    p_cv = CARLA_TO_OPENCV @ p_carla_cam

    if p_cv[2] <= 0:  # 카메라 뒤
        return None

    pixel_h = K @ p_cv
    u = pixel_h[0] / pixel_h[2]
    v = pixel_h[1] / pixel_h[2]
    return (u, v)


def compute_ground_plane_homography(K, world_to_camera):
    """z=0 지면 평면을 가정한 3×3 호모그래피.

    [u·w, v·w, w]^T = H · [X_world, Y_world, 1]^T

    역변환 H^-1로 픽셀 → 월드 (X, Y) 가능 (z=0 가정).
    """
    # 4×4 axis swap
    axis_swap_4 = np.eye(4)
    axis_swap_4[:3, :3] = CARLA_TO_OPENCV

    # world → CV camera (4×4) → 3×4 projection (이미지 평면 투영)
    world_to_cv_cam = axis_swap_4 @ world_to_camera
    K_ext = K @ world_to_cv_cam[:3, :]  # 3×4

    # z=0이면 X·1 + Y·2 + 0·3 + 1·4 → X, Y, w(=1) 컬럼만 추출
    H = K_ext[:, [0, 1, 3]]
    return H


def pixel_to_world_ground(pixel, H):
    """픽셀 (u, v) → 월드 (X, Y) 변환 (z=0 가정).

    H^-1 @ [u, v, 1]^T = [X·w, Y·w, w]^T → normalize.
    """
    H_inv = np.linalg.inv(H)
    pix_h = np.array([pixel[0], pixel[1], 1.0])
    world_h = H_inv @ pix_h
    if abs(world_h[2]) < 1e-9:
        return None
    return (world_h[0] / world_h[2], world_h[1] / world_h[2])


# ============ Camera Setup from Config ============

def setup_camera(cam_cfg, image_size, fov):
    """카메라 한 대의 모든 호모그래피 관련 매트릭스 계산."""
    K = build_intrinsic_matrix(fov, image_size[0], image_size[1])
    world_to_cam = build_world_to_camera_matrix(
        cam_cfg['location'], cam_cfg['rotation']
    )
    H_ground = compute_ground_plane_homography(K, world_to_cam)

    return {
        'id': cam_cfg['id'],
        'K': K,
        'world_to_camera': world_to_cam,
        'H_ground': H_ground,
        'image_size': image_size,
        'fov': fov,
        'location': cam_cfg['location'],
        'rotation': cam_cfg['rotation'],
    }


# ============ Validation ============

def load_vehicle_trajectory(scenario_dir):
    """ground_truth.jsonl에서 차량 위치 trajectory 로드."""
    gt_path = os.path.join(scenario_dir, 'ground_truth.jsonl')
    points = []
    with open(gt_path, encoding='utf-8') as f:
        for line in f:
            if not line.strip():
                continue
            d = json.loads(line)
            if not d.get('vehicles'):
                continue
            loc = d['vehicles'][0]['location']
            points.append(loc)
    return np.array(points)


def validate_camera(camera, scenario_dir, output_path):
    """차량 trajectory를 카메라로 투영하여 실제 이미지 위에 오버레이.

    검증 통과 기준:
    - 투영된 trajectory가 실제 차량이 보이는 위치를 따라감
    - 차량이 시야 안에 있을 때 점이 차량 위에 찍힘
    """
    cam_id = camera['id']
    print(f'  {cam_id}: 검증 시작')

    # GT trajectory 로드
    traj_world = load_vehicle_trajectory(scenario_dir)
    print(f'    trajectory: {len(traj_world)} points')

    # 각 점을 픽셀로 투영
    proj_pixels = []
    for w_pt in traj_world:
        pix = world_to_pixel(w_pt, camera['K'], camera['world_to_camera'])
        proj_pixels.append(pix)

    # 시야 안 (None 아님 + 이미지 영역 안)
    W, H = camera['image_size']
    in_view = [
        (i, p) for i, p in enumerate(proj_pixels)
        if p is not None and 0 <= p[0] < W and 0 <= p[1] < H
    ]
    print(f'    시야 안 점: {len(in_view)}/{len(traj_world)}')

    # 기준 프레임: 첫 in-view 프레임
    if not in_view:
        print(f'    [WARN] {cam_id} 시야에 차량이 한 번도 안 들어옴')
        return None

    first_frame_idx = in_view[0][0]
    last_frame_idx = in_view[-1][0]

    # 시각화: 4 프레임 (in-view 범위 균등 분포) 위에 trajectory 오버레이
    sample_frames = np.linspace(first_frame_idx, last_frame_idx, 4,
                                dtype=int).tolist()

    fig, axes = plt.subplots(2, 2, figsize=(16, 9))
    axes = axes.flatten()

    for ax, frame_idx in zip(axes, sample_frames):
        png_path = os.path.join(scenario_dir, cam_id, f'{frame_idx:06d}.png')
        if not os.path.exists(png_path):
            ax.text(0.5, 0.5, f'No image\n{frame_idx}',
                    ha='center', va='center')
            continue

        img = np.array(Image.open(png_path))
        ax.imshow(img)

        # 이 프레임까지의 trajectory 점들 (시야 안만)
        traj_so_far_u = []
        traj_so_far_v = []
        for i in range(frame_idx + 1):
            if proj_pixels[i] is not None:
                u, v = proj_pixels[i]
                if 0 <= u < W and 0 <= v < H:
                    traj_so_far_u.append(u)
                    traj_so_far_v.append(v)

        # trajectory를 옅은 노란 선으로
        if len(traj_so_far_u) > 1:
            ax.plot(traj_so_far_u, traj_so_far_v, '-',
                    color='yellow', alpha=0.6, linewidth=2)

        # 현재 프레임 점을 빨간 원으로
        if proj_pixels[frame_idx] is not None:
            u, v = proj_pixels[frame_idx]
            if 0 <= u < W and 0 <= v < H:
                ax.plot(u, v, 'o', color='red', markersize=15,
                        markerfacecolor='none', markeredgewidth=3)
                ax.plot(u, v, '+', color='red', markersize=15,
                        markeredgewidth=2)

        ax.set_title(f'Frame {frame_idx} (t={frame_idx*0.05:.2f}s)')
        ax.axis('off')

    fig.suptitle(f'{cam_id} 호모그래피 검증 — wobble_strong\n'
                 '노란 선: 차량 GT trajectory 투영, 빨간 ○+: 현재 프레임 위치')
    plt.tight_layout()
    plt.savefig(output_path, dpi=80, bbox_inches='tight')
    plt.close()

    print(f'    저장: {output_path}')
    return {
        'frames_in_view': len(in_view),
        'first_in_view_frame': first_frame_idx,
        'last_in_view_frame': last_frame_idx,
    }


# ============ Main ============

def main():
    print('=' * 60)
    print('CARLA 호모그래피 계산 및 검증')
    print('=' * 60)

    # Config
    with open(CAMERAS_CONFIG, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    image_size = cfg.get('image_size', [1920, 1080])
    fov = cfg.get('fov', 90)
    print(f'image_size: {image_size}, fov: {fov}')

    # 각 카메라 호모그래피 계산
    print('\n호모그래피 계산:')
    cameras = []
    for cam_cfg in cfg['cameras']:
        camera = setup_camera(cam_cfg, image_size, fov)
        cameras.append(camera)
        print(f'  {camera["id"]}: K, world_to_camera, H_ground 계산 완료')

    # 카메라 정보 출력
    print('\nIntrinsic K (모든 카메라 동일):')
    print(cameras[0]['K'])

    for cam in cameras:
        print(f'\n{cam["id"]} world_to_camera (top-left 3×3):')
        print(cam['world_to_camera'][:3, :3])
        print(f'{cam["id"]} world_to_camera (translation):')
        print(cam['world_to_camera'][:3, 3])

    # JSON 저장
    print(f'\nJSON 저장: {HOMOGRAPHY_OUTPUT}')
    os.makedirs(os.path.dirname(HOMOGRAPHY_OUTPUT), exist_ok=True)
    output_data = {
        'image_size': image_size,
        'fov': fov,
        'coordinate_conventions': {
            'world': 'CARLA: +X forward, +Y right, +Z up (left-handed)',
            'camera_local': 'CARLA: +X forward, +Y right, +Z up',
            'camera_for_projection':
                'OpenCV: +X right, +Y down, +Z forward',
        },
        'carla_to_opencv_axis_swap': CARLA_TO_OPENCV.tolist(),
        'cameras': [
            {
                'id': c['id'],
                'location': c['location'],
                'rotation': c['rotation'],
                'K': c['K'].tolist(),
                'world_to_camera': c['world_to_camera'].tolist(),
                'H_ground': c['H_ground'].tolist(),
            }
            for c in cameras
        ],
    }
    with open(HOMOGRAPHY_OUTPUT, 'w', encoding='utf-8') as f:
        json.dump(output_data, f, ensure_ascii=False, indent=2)

    # 검증
    print('\n검증 (wobble_strong 시나리오 trajectory 투영):')
    scenario_dir = f'data/scenarios/{VALIDATION_SCENARIO}'
    if not os.path.isdir(scenario_dir):
        print(f'  [WARN] {scenario_dir} 없음. 검증 스킵.')
        return

    os.makedirs(VALIDATION_DIR, exist_ok=True)
    validation_results = {}
    for cam in cameras:
        out_path = os.path.join(VALIDATION_DIR, f'{cam["id"]}_validation.png')
        result = validate_camera(cam, scenario_dir, out_path)
        if result:
            validation_results[cam['id']] = result

    print()
    print('=' * 60)
    print('완료')
    print('=' * 60)
    print('\n검증 시각화 확인:')
    for cam_id, r in validation_results.items():
        print(f'  {cam_id}: 시야 내 {r["frames_in_view"]} frame '
              f'(frame {r["first_in_view_frame"]} ~ '
              f'{r["last_in_view_frame"]})')
    print(f'\n시각화 파일: {VALIDATION_DIR}/cam{{0,1,2}}_validation.png')
    print('검증 통과 기준: 노란 trajectory가 실제 차량 위치를 따라감')


if __name__ == '__main__':
    main()
