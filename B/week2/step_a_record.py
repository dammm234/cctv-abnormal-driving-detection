"""
Step A: 단일 카메라 + 차량 1대 + 짧은 시퀀스 녹화 검증

Sync mode 위에 다음 요소들을 쌓아서 검증:
- Town06 로드
- cameras.yaml의 cam0 위치에 카메라 spawn
- 차량 1대 autopilot spawn
- 5초간(100 tick @ 20fps) 동기 녹화
- ground_truth.jsonl에 매 tick 차량 정보 기록
- Sync mode + Traffic Manager 정상 해제

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
OUT_BASE = 'data/scenarios/step_a_test'
N_TICKS = 100
FIXED_DELTA = 0.05  # 20 fps
TIMEOUT_PER_TICK = 5.0


def load_config():
    with open(CONFIG_PATH, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    cam0 = next(c for c in cfg['cameras'] if c['id'] == 'cam0')
    return {
        'map': cfg.get('map', 'Town06'),
        'cam0': cam0,
        'image_size': cfg.get('image_size', [1920, 1080]),
        'fov': cfg.get('fov', 90),
    }


def setup_world(client, map_name):
    """필요시 맵을 로드. 이미 로드돼 있으면 그대로 사용."""
    current = client.get_world().get_map().name.split('/')[-1]
    if current != map_name:
        print(f'맵 로드: {map_name} (현재 {current})')
        world = client.load_world(map_name)
        time.sleep(2.0)
    else:
        world = client.get_world()
        print(f'맵: {current} (재로드 불필요)')
    return world


def spawn_camera(world, cam_cfg, image_size, fov, image_queue):
    """yaml에 정의된 cam0 위치에 RGB 카메라 spawn."""
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
    camera = world.spawn_actor(bp, transform)
    camera.listen(image_queue.put)
    print(f'카메라 spawn at ({transform.location.x:.1f}, '
          f'{transform.location.y:.1f}, {transform.location.z:.1f})')
    return camera


def spawn_vehicle_near_camera(world, cam_cfg):
    """카메라 시야 방향(+yaw)으로 약 40m 떨어진 도로 위에 차량 spawn.
    실패하면 가장 가까운 예비 spawn point로 폴백."""
    yaw_rad = math.radians(cam_cfg['rotation']['yaw'])
    target = carla.Location(
        x=cam_cfg['location']['x'] + 40 * math.cos(yaw_rad),
        y=cam_cfg['location']['y'] + 40 * math.sin(yaw_rad),
        z=cam_cfg['location']['z'] - 5,  # 지면 근처
    )
    wp = world.get_map().get_waypoint(target, project_to_road=True)
    spawn_t = wp.transform
    spawn_t.location.z += 0.5  # collision 방지

    bp = world.get_blueprint_library().filter('vehicle.tesla.model3')[0]
    vehicle = world.try_spawn_actor(bp, spawn_t)
    if vehicle is not None:
        return vehicle

    # 폴백: 카메라에 가장 가까운 예비 spawn point들 시도
    print('첫 spawn 실패, 예비 spawn point 시도...')
    cam_loc = carla.Location(**cam_cfg['location'])
    spawn_points = sorted(
        world.get_map().get_spawn_points(),
        key=lambda sp: sp.location.distance(cam_loc),
    )
    for sp in spawn_points[:20]:
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is not None:
            print(f'예비 spawn 성공 at ({sp.location.x:.1f}, '
                  f'{sp.location.y:.1f})')
            return vehicle

    raise RuntimeError('차량 spawn 모두 실패')


def main():
    print('=' * 60)
    print('Step A: 단일 카메라 + 차량 1대 녹화')
    print('=' * 60)

    cfg = load_config()
    os.makedirs(os.path.join(OUT_BASE, 'cam0'), exist_ok=True)
    gt_path = os.path.join(OUT_BASE, 'ground_truth.jsonl')

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    print(f'서버: {client.get_server_version()}')

    world = setup_world(client, cfg['map'])
    original_settings = world.get_settings()
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()

    image_queue = queue.Queue()
    camera = None
    vehicle = None
    gt_lines = []

    try:
        # Sync mode 활성화
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        print('Sync mode + TM sync ON')

        # 카메라 spawn
        camera = spawn_camera(
            world, cfg['cam0'], cfg['image_size'], cfg['fov'], image_queue
        )

        # 차량 spawn
        vehicle = spawn_vehicle_near_camera(world, cfg['cam0'])
        print(f'차량 id={vehicle.id} spawn 완료')

        # 첫 tick으로 액터 등록, 첫 프레임은 폐기
        world.tick()
        try:
            image_queue.get(timeout=2.0)
        except queue.Empty:
            pass

        # Autopilot 활성화
        vehicle.set_autopilot(True, tm_port)
        print('Autopilot ON')
        print()

        # 본 녹화
        print(f'{N_TICKS}번 tick 녹화:')
        loss = 0
        for i in range(N_TICKS):
            world.tick()

            try:
                image = image_queue.get(timeout=TIMEOUT_PER_TICK)
                out_path = os.path.join(OUT_BASE, 'cam0', f'{i:06d}.png')
                image.save_to_disk(out_path)
            except queue.Empty:
                print(f'  Tick {i}: 이미지 timeout')
                loss += 1
                continue

            # Ground truth
            loc = vehicle.get_location()
            vel = vehicle.get_velocity()
            rot = vehicle.get_transform().rotation
            speed_ms = math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
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
                }],
            }
            gt_lines.append(json.dumps(gt))

            if i % 20 == 0:
                print(f'  Tick {i:3d}: '
                      f'pos=({loc.x:7.1f}, {loc.y:7.1f}), '
                      f'{speed_ms * 3.6:5.1f} km/h')

        # Ground truth 저장
        with open(gt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(gt_lines) + '\n')

        # 결과
        print()
        print('=' * 60)
        print('결과')
        print('=' * 60)
        saved_pngs = len(os.listdir(os.path.join(OUT_BASE, 'cam0')))
        print(f'저장된 PNG:       {saved_pngs} / {N_TICKS}')
        print(f'Ground truth 행:  {len(gt_lines)} / {N_TICKS}')
        print(f'이미지 손실:      {loss}')
        print(f'출력 폴더:        {OUT_BASE}')

        if saved_pngs == N_TICKS and len(gt_lines) == N_TICKS:
            print('\n>>> Step A 통과. PNG와 ground_truth.jsonl을 직접 확인하세요. <<<')
            print('>>> 차량이 카메라 시야에 보이면 Step B(카메라 3개 확장) 진행 가능. <<<')
        else:
            print('\n>>> 일부 손실. 위 로그 확인. <<<')

    except Exception as e:
        print(f'\n[ERROR] {type(e).__name__}: {e}')
        traceback.print_exc()

    finally:
        if camera is not None:
            try:
                camera.stop()
                camera.destroy()
                print('\n카메라 정리')
            except Exception:
                pass
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
            print('CARLA 서버 재시작을 권장합니다.')


if __name__ == '__main__':
    main()
