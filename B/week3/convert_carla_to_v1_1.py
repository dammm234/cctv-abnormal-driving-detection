"""
CARLA 8 시나리오 → A의 v1.1 JSON 포맷 변환

각 시나리오의 ground_truth.jsonl + scenario_config.yaml + homography_carla.json을
A의 schema_v1.1.md 규격에 맞춰 변환.

출력: scenarios_v1.1/{시나리오명}_cam{0,1,2}.json (총 24개)

CARLA 좌표계 → 도로 좌표계 변환:
- CARLA 차량은 -X 방향으로 진행
- target_y가 차로 중심 (예: -17.54)
- road coord x (가로) = lane_center + (carla_y - target_y)
- road coord y (세로) = spawn_x - carla_x (진행 거리)
"""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    import yaml
except ImportError:
    print("[오류] pyyaml 필요: pip install pyyaml")
    sys.exit(1)


# 변환 상수
LANE_WIDTH_M = 3.5            # 한국 고속도로 표준
DEFAULT_LANE_ID = 2           # CARLA 시나리오 차량은 항상 lane 2에 있다고 가정
DEFAULT_FPS = 20.0            # CARLA tick 기본
IMAGE_W = 1920                # CARLA 카메라 해상도
IMAGE_H = 1080
VEHICLE_LENGTH = 4.5          # 차량 크기 (bbox 근사)
VEHICLE_WIDTH = 1.8
VEHICLE_HEIGHT = 1.5


def get_position_from_vehicle(vehicle):
    """vehicle dict에서 (x, y, z) 추출.
    실제 포맷: vehicle['location'] = [x, y, z]"""
    loc = vehicle.get('location')
    if isinstance(loc, (list, tuple)) and len(loc) >= 2:
        z = loc[2] if len(loc) >= 3 else 0.5
        return float(loc[0]), float(loc[1]), float(z)
    return None, None, 0.5


def get_speed_from_vehicle(vehicle):
    """vehicle dict에서 속도 (m/s) 추출.
    우선순위: speed_kmh > velocity 벡터."""
    speed_kmh = vehicle.get('speed_kmh')
    if speed_kmh is not None:
        return float(speed_kmh) / 3.6

    vel = vehicle.get('velocity')
    if isinstance(vel, (list, tuple)) and len(vel) >= 2:
        return math.sqrt(sum(c * c for c in vel[:3]))

    return 0.0


def get_lateral_offset_from_vehicle(vehicle, target_y):
    """이미 계산된 lateral_offset 우선 사용, 없으면 직접 계산."""
    if 'lateral_offset' in vehicle:
        return float(vehicle['lateral_offset'])
    loc = vehicle.get('location')
    if isinstance(loc, (list, tuple)) and len(loc) >= 2:
        return float(loc[1]) - target_y
    return 0.0


def get_intensity_from_name(scenario_name):
    """시나리오 이름에서 intensity 파싱.
    예: abrupt_mild → mild, normal_50kmh → 50kmh"""
    if '_' in scenario_name:
        return scenario_name.split('_', 1)[1]
    return 'unknown'


def get_field(d, *candidates, default=None):
    """딕셔너리에서 후보 키 중 첫 번째로 발견되는 값."""
    for k in candidates:
        if k in d:
            return d[k]
    return default


def load_scenarios(scenarios_dir):
    """시나리오 폴더 목록."""
    if not os.path.isdir(scenarios_dir):
        return []
    return sorted([d for d in os.listdir(scenarios_dir)
                   if os.path.isdir(os.path.join(scenarios_dir, d))])


def load_homography(path):
    """homography_carla.json 로드. list/dict 구조 모두 지원."""
    with open(path, encoding='utf-8') as f:
        data = json.load(f)

    # cameras 추출
    cameras_raw = None
    if isinstance(data, dict):
        if 'cameras' in data:
            cameras_raw = data['cameras']
        elif 'cam0' in data:
            return data  # 이미 dict 형태
        else:
            # 루트 키가 cam 이름이 아니면 전체를 cameras로 가정
            cameras_raw = data
    elif isinstance(data, list):
        cameras_raw = data

    # list → dict 변환
    if isinstance(cameras_raw, list):
        result = {}
        for i, cam in enumerate(cameras_raw):
            if isinstance(cam, dict):
                # 이름 결정: name 필드 우선, 없으면 인덱스
                name = (cam.get('name') or cam.get('id')
                        or cam.get('camera_id') or f'cam{i}')
                result[name] = cam
        return result
    elif isinstance(cameras_raw, dict):
        return cameras_raw

    print(f"[경고] homography 구조 분석 실패. type={type(data).__name__}")
    return {}


def get_camera_matrices(cam_info):
    """카메라 정보에서 K, world_to_camera, H_ground 추출."""
    K = cam_info.get('K') or cam_info.get('intrinsic')
    w2c = (cam_info.get('world_to_camera')
           or cam_info.get('extrinsic')
           or cam_info.get('w2c'))
    H_ground = (cam_info.get('H_ground')
                or cam_info.get('h_ground'))

    K = np.array(K) if K is not None else None
    w2c = np.array(w2c) if w2c is not None else None
    H_ground = np.array(H_ground) if H_ground is not None else None

    return K, w2c, H_ground


def project_ground_to_pixel(world_xy, H_ground):
    """차량 지면 좌표 (x, y) → 픽셀 (u, v).
    H_ground는 검증된 3x3 호모그래피."""
    p = np.array([world_xy[0], world_xy[1], 1.0])
    p_pixel = H_ground @ p
    if abs(p_pixel[2]) < 1e-6:
        return None, None
    u = p_pixel[0] / p_pixel[2]
    v = p_pixel[1] / p_pixel[2]
    return float(u), float(v)


def make_simple_bbox(u, v):
    """지면 픽셀 위치를 기준으로 간단한 bbox 생성.
    Role C는 주로 position_road_m을 쓰므로 bbox는 시각화용 근사."""
    # 화면 위쪽일수록 멀어서 작아지는 휴리스틱
    distance_factor = max(0.3, 1.0 - v / IMAGE_H)
    half_w = 40 * distance_factor
    height = 60 * distance_factor

    u1 = max(0, u - half_w)
    u2 = min(IMAGE_W - 1, u + half_w)
    v1 = max(0, v - height)
    v2 = min(IMAGE_H - 1, v)

    if u2 - u1 < 2 or v2 - v1 < 2:
        return None
    return [float(u1), float(v1), float(u2), float(v2)]


def world_to_road_coords(world_x, world_y, spawn_x, target_y, lane_id):
    """CARLA world → A의 road coordinate."""
    lane_center_x_road = (lane_id - 0.5) * LANE_WIDTH_M
    lateral_offset = world_y - target_y
    x_road = lane_center_x_road + lateral_offset
    y_road = spawn_x - world_x  # 진행 거리 (양수)
    return x_road, y_road, lateral_offset


def get_tick_id(tick, idx):
    """tick 데이터에서 frame 번호 추출."""
    return get_field(tick, 'frame', 'tick', 'frame_id', 'tick_id', default=idx)


def load_ground_truth(gt_path):
    """ground_truth.jsonl 로드."""
    ticks = []
    with open(gt_path, encoding='utf-8') as f:
        for line_no, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                ticks.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"    [경고] {gt_path} 줄 {line_no} 파싱 실패: {e}")
    return ticks


def convert_scenario(scenario_dir, cameras, output_dir):
    """1개 시나리오의 3개 카메라용 JSON 생성."""
    scenario_name = os.path.basename(scenario_dir)
    print(f"\n[{scenario_name}]")

    # config 로드
    cfg_path = os.path.join(scenario_dir, 'scenario_config.yaml')
    if not os.path.exists(cfg_path):
        print(f"  [skip] scenario_config.yaml 없음")
        return 0

    with open(cfg_path, encoding='utf-8') as f:
        config = yaml.safe_load(f)

    # _applied 중첩 처리 (실제 구조에 맞춤)
    applied = config.get('_applied', {})

    # target_y: _applied 안에 있음
    target_y = (applied.get('target_y')
                or get_field(config, 'target_y', 'lane_y', 'y_target',
                             default=-17.5))

    # fps: _applied.fixed_delta_seconds의 역수
    delta = applied.get('fixed_delta_seconds') or applied.get('delta_seconds')
    if delta:
        fps = 1.0 / float(delta)
    else:
        fps = get_field(config, 'fps', 'tick_rate', default=DEFAULT_FPS)

    # spawn_x: config에 없을 가능성 큼 → ground_truth 첫 프레임에서 추출 예정
    spawn_x = get_field(config, 'spawn_x', 'start_x', 'x_spawn', default=None)

    behavior = get_field(config, 'behavior', 'type', default='unknown')
    # intensity: 시나리오 이름에서 파싱
    intensity = get_intensity_from_name(os.path.basename(scenario_dir))

    # ground_truth 로드
    gt_path = os.path.join(scenario_dir, 'ground_truth.jsonl')
    if not os.path.exists(gt_path):
        print(f"  [skip] ground_truth.jsonl 없음")
        return 0

    ticks = load_ground_truth(gt_path)
    if not ticks:
        print(f"  [skip] tick 데이터 없음")
        return 0

    # spawn_x를 ground_truth 첫 프레임에서 추출 (config에 없을 가능성 큼)
    if spawn_x is None:
        first_vehicles = ticks[0].get('vehicles', [])
        if first_vehicles:
            loc = first_vehicles[0].get('location')
            if isinstance(loc, (list, tuple)) and len(loc) >= 1:
                spawn_x = float(loc[0])
        if spawn_x is None:
            spawn_x = 437.64  # 기본값

    total_frames = len(ticks)
    print(f"  config: spawn_x={spawn_x:.2f}, target_y={target_y:.2f}, "
          f"behavior={behavior}, intensity={intensity}")
    print(f"  ticks: {total_frames}, fps: {fps}")

    # 각 카메라별로 JSON 생성
    n_outputs = 0
    for cam_key, cam_info in cameras.items():
        K, w2c, H_ground = get_camera_matrices(cam_info)
        if H_ground is None:
            print(f"    [{cam_key}] H_ground 없음, skip")
            continue

        frames = []
        n_visible = 0
        for idx, tick in enumerate(ticks):
            frame_id = get_tick_id(tick, idx)
            timestamp_sec = tick.get('timestamp', frame_id / fps)

            vehicles_out = []
            # tick['vehicles']를 순회 (보통 1개지만 다중 차량 대비)
            for vehicle in tick.get('vehicles', []):
                x, y, z = get_position_from_vehicle(vehicle)
                if x is None or y is None:
                    continue

                # 지면 좌표 → 픽셀
                u, v = project_ground_to_pixel((x, y), H_ground)
                if u is None or not (0 <= u < IMAGE_W and 0 <= v < IMAGE_H):
                    continue

                bbox = make_simple_bbox(u, v)
                if bbox is None:
                    continue

                x_road, y_road, _ = world_to_road_coords(
                    x, y, spawn_x, target_y, DEFAULT_LANE_ID,
                )
                speed = get_speed_from_vehicle(vehicle)
                lateral_offset = get_lateral_offset_from_vehicle(
                    vehicle, target_y,
                )

                vehicles_out.append({
                    'track_id': int(vehicle.get('id', 1)),
                    'bbox_pixel': bbox,
                    'position_road_m': [float(x_road), float(y_road)],
                    'lane_id': DEFAULT_LANE_ID,
                    'lateral_offset_m': float(lateral_offset),
                    'speed_est_mps': float(speed),
                })

            if vehicles_out:
                n_visible += 1

            frames.append({
                'frame_id': int(frame_id),
                'timestamp_sec': float(timestamp_sec),
                'vehicles': vehicles_out,
            })

        # 출력 JSON
        output = {
            'version': 'v1.1',
            'note': (f'CARLA {scenario_name} {cam_key} - '
                     f'ground truth (no detection error)'),
            'source': 'carla_ground_truth',
            'scenario': scenario_name,
            'behavior': behavior,
            'intensity': intensity,
            'camera': cam_key,
            'fps': float(fps),
            'total_frames': total_frames,
            'road_info': {
                'lane_width_m': LANE_WIDTH_M,
                'num_lanes': 6,  # Town06 highway 가정
                'road_width_m': LANE_WIDTH_M * 6,
            },
            'frames': frames,
        }

        out_path = os.path.join(output_dir, f'{scenario_name}_{cam_key}.json')
        with open(out_path, 'w', encoding='utf-8') as f:
            json.dump(output, f, indent=2, ensure_ascii=False)

        print(f"    [{cam_key}] {n_visible}/{total_frames} frames visible "
              f"→ {os.path.basename(out_path)}")
        n_outputs += 1

    return n_outputs


def main():
    parser = argparse.ArgumentParser(
        description='CARLA 8 시나리오 → A의 v1.1 JSON 변환',
    )
    parser.add_argument(
        '--scenarios-dir',
        default='../../../data/scenarios',
        help='CARLA 시나리오 루트 폴더 (기본: ../../../data/scenarios)',
    )
    parser.add_argument(
        '--homography',
        default='homography_carla.json',
        help='CARLA 호모그래피 JSON (기본: ./homography_carla.json)',
    )
    parser.add_argument(
        '--output-dir',
        default='scenarios_v1.1',
        help='출력 폴더 (기본: ./scenarios_v1.1)',
    )
    args = parser.parse_args()

    print(f'시나리오 폴더: {args.scenarios_dir}')
    print(f'호모그래피: {args.homography}')
    print(f'출력 폴더: {args.output_dir}')

    # 호모그래피 로드
    if not os.path.exists(args.homography):
        print(f"\n[오류] 호모그래피 파일 없음: {args.homography}")
        sys.exit(1)
    cameras = load_homography(args.homography)
    print(f'\n카메라 {len(cameras)}개: {list(cameras.keys())}')

    if not cameras:
        print("[오류] 카메라 정보 없음. JSON 구조 확인 필요.")
        sys.exit(1)

    # 시나리오 폴더
    if not os.path.isdir(args.scenarios_dir):
        print(f"\n[오류] 시나리오 폴더 없음: {args.scenarios_dir}")
        print("--scenarios-dir 옵션으로 정확한 경로 지정")
        sys.exit(1)
    scenarios = load_scenarios(args.scenarios_dir)
    print(f'시나리오 {len(scenarios)}개: {scenarios}')

    if not scenarios:
        print("[오류] 시나리오 폴더 없음")
        sys.exit(1)

    # 출력 폴더 생성
    os.makedirs(args.output_dir, exist_ok=True)

    # 변환
    total_outputs = 0
    for sd in scenarios:
        sdir = os.path.join(args.scenarios_dir, sd)
        n = convert_scenario(sdir, cameras, args.output_dir)
        total_outputs += n

    print(f"\n=== 변환 완료 ===")
    print(f"  생성된 JSON: {total_outputs}개")
    print(f"  위치: {args.output_dir}")


if __name__ == '__main__':
    main()
