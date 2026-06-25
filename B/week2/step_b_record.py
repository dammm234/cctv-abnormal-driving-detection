"""
Step B: 3 카메라 동시 동기 녹화 검증

Step A에서 검증된 sync mode 위에 다음을 쌓아서 검증:
- 3개 카메라(cam0, cam1, cam2) 동시 spawn
- 각 카메라마다 별도 큐로 프레임 수집
- 동일 tick에서 세 카메라의 carla_frame이 일치하는지 검증
- 카메라별 폴더에 같은 frame_idx로 PNG 저장
- 차량 1대 autopilot으로 진행 (cam0 → cam1 → cam2 통과)
- 10초간(200 tick @ 20fps) 녹화
"""
import json
import math
import os
import queue
import time
import traceback

import carla
import yaml


CONFIG_PATH = 'config/cameras.yaml'
OUT_BASE = 'data/scenarios/step_b_test'
N_TICKS = 200  # 10초 @ 20fps
FIXED_DELTA = 0.05
TIMEOUT_PER_TICK = 5.0
CAMERA_IDS = ['cam0', 'cam1', 'cam2']


def load_config():
    with open(CONFIG_PATH, encoding='utf-8') as f:
        return yaml.safe_load(f)


def setup_world(client, map_name):
    current = client.get_world().get_map().name.split('/')[-1]
    if current != map_name:
        print(f'맵 로드: {map_name} (현재 {current})')
        world = client.load_world(map_name)
        time.sleep(2.0)
    else:
        world = client.get_world()
        print(f'맵: {current}')
    return world


def spawn_camera(world, cam_cfg, image_size, fov):
    transform = carla.Transform(
        carla.Location(
            x=cam_cfg['location']['x'],
            y=cam_cfg['location']['y'],
            z=cam_cfg['location']['z'],
        ),
        carla.Rotation(
            pitch=cam_cfg['rotation']['pitch'],
            yaw=cam_cfg['rotation']['yaw'],
            roll=cam_cfg['rotation']['roll'],
        ),
    )
    bp = world.get_blueprint_library().find('sensor.camera.rgb')
    bp.set_attribute('image_size_x', str(image_size[0]))
    bp.set_attribute('image_size_y', str(image_size[1]))
    bp.set_attribute('fov', str(fov))
    return world.spawn_actor(bp, transform)


def spawn_vehicle(world, cam0_cfg):
    """cam0 시야 방향 40m 앞 도로 위에 차량 spawn."""
    yaw_rad = math.radians(cam0_cfg['rotation']['yaw'])
    target = carla.Location(
        x=cam0_cfg['location']['x'] + 40 * math.cos(yaw_rad),
        y=cam0_cfg['location']['y'] + 40 * math.sin(yaw_rad),
        z=cam0_cfg['location']['z'] - 5,
    )
    wp = world.get_map().get_waypoint(target, project_to_road=True)
    spawn_t = wp.transform
    spawn_t.location.z += 0.5

    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    vehicle = world.try_spawn_actor(bp, spawn_t)
    if vehicle is not None:
        return vehicle

    # 폴백
    cam_loc = carla.Location(**cam0_cfg['location'])
    spawn_points = sorted(
        world.get_map().get_spawn_points(),
        key=lambda sp: sp.location.distance(cam_loc),
    )
    for sp in spawn_points[:20]:
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is not None:
            return vehicle
    raise RuntimeError('차량 spawn 실패')


def main():
    print('=' * 60)
    print('Step B: 3 카메라 동시 녹화 + 동기 검증')
    print('=' * 60)

    cfg = load_config()
    map_name = cfg.get('map', 'Town06')
    image_size = cfg.get('image_size', [1920, 1080])
    fov = cfg.get('fov', 90)

    # cameras.yaml의 각 카메라 cfg를 dict로 변환
    cam_configs = {c['id']: c for c in cfg['cameras']}
    for cam_id in CAMERA_IDS:
        if cam_id not in cam_configs:
            raise RuntimeError(f'cameras.yaml에 {cam_id} 정의 없음')

    # 출력 디렉토리
    for cam_id in CAMERA_IDS:
        os.makedirs(os.path.join(OUT_BASE, cam_id), exist_ok=True)
    gt_path = os.path.join(OUT_BASE, 'ground_truth.jsonl')

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    print(f'서버: {client.get_server_version()}')

    world = setup_world(client, map_name)
    original_settings = world.get_settings()
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()

    # 카메라마다 별도 큐
    queues = {cam_id: queue.Queue() for cam_id in CAMERA_IDS}
    cameras = {}
    vehicle = None
    gt_lines = []

    try:
        # Sync mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        print('Sync mode + TM sync ON')

        # 3개 카메라 spawn
        for cam_id in CAMERA_IDS:
            cc = cam_configs[cam_id]
            cam = spawn_camera(world, cc, image_size, fov)
            cam.listen(queues[cam_id].put)
            cameras[cam_id] = cam
            print(f'  {cam_id} spawn at '
                  f'({cc["location"]["x"]:.1f}, '
                  f'{cc["location"]["y"]:.1f}, '
                  f'{cc["location"]["z"]:.1f})')

        # 차량 spawn
        vehicle = spawn_vehicle(world, cam_configs['cam0'])
        print(f'\n차량 id={vehicle.id} spawn 완료')

        # 첫 tick으로 액터 등록, 첫 프레임 폐기
        world.tick()
        for cam_id in CAMERA_IDS:
            try:
                queues[cam_id].get(timeout=2.0)
            except queue.Empty:
                pass

        vehicle.set_autopilot(True, tm_port)
        print('Autopilot ON\n')

        # 본 녹화 + 동기 검증
        print(f'{N_TICKS}번 tick 녹화:')
        sync_check = []  # 각 tick의 carla_frame 기록
        loss_count = {cam_id: 0 for cam_id in CAMERA_IDS}

        for i in range(N_TICKS):
            world.tick()

            tick_frames = {}
            for cam_id in CAMERA_IDS:
                try:
                    image = queues[cam_id].get(timeout=TIMEOUT_PER_TICK)
                    out_path = os.path.join(
                        OUT_BASE, cam_id, f'{i:06d}.png'
                    )
                    image.save_to_disk(out_path)
                    tick_frames[cam_id] = image.frame
                except queue.Empty:
                    print(f'  [WARN] Tick {i}: {cam_id} timeout')
                    loss_count[cam_id] += 1

            sync_check.append((i, tick_frames))

            # Ground truth (차량 정보 + 카메라까지 거리)
            loc = vehicle.get_location()
            vel = vehicle.get_velocity()
            rot = vehicle.get_transform().rotation
            speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

            distances = {}
            for cam_id in CAMERA_IDS:
                cc = cam_configs[cam_id]
                cam_loc = carla.Location(
                    x=cc['location']['x'],
                    y=cc['location']['y'],
                    z=cc['location']['z'],
                )
                distances[cam_id] = round(loc.distance(cam_loc), 2)

            gt = {
                'frame': i,
                'timestamp': round(i * FIXED_DELTA, 3),
                'vehicles': [{
                    'id': vehicle.id,
                    'location': [round(loc.x, 3), round(loc.y, 3),
                                 round(loc.z, 3)],
                    'velocity': [round(vel.x, 3), round(vel.y, 3),
                                 round(vel.z, 3)],
                    'rotation': [round(rot.pitch, 2), round(rot.yaw, 2),
                                 round(rot.roll, 2)],
                    'speed_kmh': round(speed_ms * 3.6, 2),
                    'distance_to': distances,
                }],
            }
            gt_lines.append(json.dumps(gt))

            if i % 40 == 0:
                d = distances
                print(f'  Tick {i:3d}: x={loc.x:6.1f}, '
                      f'{speed_ms * 3.6:5.1f} km/h, '
                      f'dist=[cam0:{d["cam0"]:.0f} '
                      f'cam1:{d["cam1"]:.0f} '
                      f'cam2:{d["cam2"]:.0f}]')

        with open(gt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(gt_lines) + '\n')

        # 결과 분석
        print()
        print('=' * 60)
        print('결과')
        print('=' * 60)

        all_pass = True

        # 1. PNG 개수 검증
        print('PNG 저장:')
        for cam_id in CAMERA_IDS:
            saved = len(os.listdir(os.path.join(OUT_BASE, cam_id)))
            status = '[OK]' if saved == N_TICKS else '[FAIL]'
            print(f'  {status} {cam_id}: {saved}/{N_TICKS} '
                  f'(loss={loss_count[cam_id]})')
            if saved != N_TICKS:
                all_pass = False

        print(f'\nGround truth: {len(gt_lines)}/{N_TICKS} 행')
        if len(gt_lines) != N_TICKS:
            all_pass = False

        # 2. 카메라 간 carla_frame 일치 검증 
        print('\n카메라 간 동기 검증 (carla_frame 일치):')
        sync_violations = 0
        complete_ticks = 0
        for tick_i, frames in sync_check:
            if len(frames) == len(CAMERA_IDS):
                complete_ticks += 1
                if len(set(frames.values())) != 1:
                    sync_violations += 1
        print(f'  3 카메라 모두 응답한 tick: {complete_ticks}/{N_TICKS}')
        print(f'  그 중 carla_frame 불일치 tick: {sync_violations}')

        # 샘플 5개 출력
        print('\n  샘플 (앞 5 tick):')
        for tick_i, frames in sync_check[:5]:
            if len(frames) == len(CAMERA_IDS):
                vals = list(frames.values())
                same = '[동기]' if len(set(vals)) == 1 else '[불일치]'
                print(f'    Tick {tick_i}: {frames} {same}')
            else:
                print(f'    Tick {tick_i}: 불완전 응답 {frames}')

        if sync_violations > 0:
            all_pass = False

        # 3. 차량이 각 카메라를 지나갔는지 
        print('\n차량 통과 검증 (최단 거리 기준):')
        min_distances = {cam_id: float('inf') for cam_id in CAMERA_IDS}
        for line in gt_lines:
            d = json.loads(line)
            for cam_id in CAMERA_IDS:
                dist = d['vehicles'][0]['distance_to'][cam_id]
                min_distances[cam_id] = min(min_distances[cam_id], dist)
        for cam_id in CAMERA_IDS:
            md = min_distances[cam_id]
            close = '✓ 가까이 통과' if md < 30 else '○ 멀리 있음'
            print(f'  {cam_id} 최단 거리: {md:.1f}m  {close}')

        print()
        if all_pass:
            print('>>> Step B 통과. 다중 카메라 동기 녹화 검증 완료. <<<')
            print('>>> 다음 — Step C: 차량 행동 제어 (비틀거림 시나리오) <<<')
        else:
            print('>>> 일부 검증 실패. 위 로그 확인. <<<')

    except Exception as e:
        print(f'\n[ERROR] {type(e).__name__}: {e}')
        traceback.print_exc()

    finally:
        for cam_id, cam in cameras.items():
            try:
                cam.stop()
                cam.destroy()
            except Exception:
                pass
        if cameras:
            print('\n카메라 정리')
        if vehicle is not None:
            try:
                vehicle.destroy()
                print('차량 정리')
            except Exception:
                pass
        try:
            tm.set_synchronous_mode(False)
            world.apply_settings(original_settings)
            print('Sync mode + TM sync 해제')
        except Exception as e:
            print(f'[ERROR] 설정 복원 실패: {e}')
            print('CARLA 서버 재시작 권장.')


if __name__ == '__main__':
    main()
