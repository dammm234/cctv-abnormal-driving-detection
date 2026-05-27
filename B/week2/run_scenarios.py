"""
Step D: 시나리오 배치 실행기

config/scenarios.yaml에 정의된 모든 시나리오를 일괄 실행하여
data/scenarios/{name}/ 폴더에 PNG + ground_truth.jsonl + scenario_config.yaml 저장.

성능 최적화:
- Town06 한 번만 로드
- 카메라 한 번만 spawn (시나리오 간 재사용)
- Sync mode 한 번만 켜고 끄기
- 시나리오 사이엔 차량만 spawn/destroy

사용:
    conda activate carla37
    cd D:\\CARLA_0.9.14\\WindowsNoEditor\\PythonAPI\\strange_drive
    python run_scenarios.py

특정 시나리오만 실행: scenarios.yaml에서 enabled: false 로 설정
"""
import json
import math
import os
import queue
import statistics
import time
import traceback

import carla
import yaml


CAMERAS_CONFIG_PATH = 'config/cameras.yaml'
SCENARIOS_CONFIG_PATH = 'config/scenarios.yaml'
OUTPUT_BASE = 'data/scenarios'
CAMERA_IDS = ['cam0', 'cam1', 'cam2']
TIMEOUT_PER_TICK = 5.0


# ============ Behavior 클래스들 ============

class BehaviorController:
    def __init__(self, target_speed_kmh=60.0):
        self.target_speed_kmh = target_speed_kmh

    def get_control(self, t, vehicle_state):
        raise NotImplementedError

    def maintain_speed_throttle(self, current_kmh):
        error = self.target_speed_kmh - current_kmh
        return max(0.0, min(1.0, 0.35 + error * 0.02))


class WobbleBehavior(BehaviorController):
    def __init__(self, amplitude=0.04, period=2.0, target_speed_kmh=60.0,
                 target_y=None, lane_correction_gain=0.025):
        super().__init__(target_speed_kmh)
        self.amplitude = amplitude
        self.period = period
        self.target_y = target_y
        self.correction_gain = lane_correction_gain

    def get_control(self, t, vehicle_state):
        throttle = self.maintain_speed_throttle(vehicle_state['speed_kmh'])
        wobble = self.amplitude * math.sin(2 * math.pi * t / self.period)

        if self.target_y is not None and 'y' in vehicle_state:
            lane_correction = self.correction_gain * (
                vehicle_state['y'] - self.target_y
            )
        else:
            lane_correction = 0.0

        steer = wobble + lane_correction
        steer = max(-0.15, min(0.15, steer))

        return carla.VehicleControl(throttle=throttle, steer=steer, brake=0.0)


class AbruptBehavior(BehaviorController):
    def __init__(self, period=4.0, target_speed_kmh=60.0):
        super().__init__(target_speed_kmh)
        self.period = period

    def get_control(self, t, vehicle_state):
        phase = (t % self.period) / self.period
        if phase < 0.5:
            return carla.VehicleControl(throttle=1.0, steer=0.0, brake=0.0)
        else:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=0.7)


class NormalBehavior(BehaviorController):
    def get_control(self, t, vehicle_state):
        return None  # autopilot 사용 신호


def make_behavior(scenario, defaults, target_y=None):
    """scenario 정의에서 behavior 인스턴스 생성."""
    behavior_type = scenario['behavior']
    target_speed = scenario.get(
        'target_speed_kmh', defaults['target_speed_kmh']
    )
    params = scenario.get('params', {})

    if behavior_type == 'wobble':
        return WobbleBehavior(
            amplitude=params.get('amplitude', 0.04),
            period=params.get('period', 2.0),
            target_speed_kmh=target_speed,
            target_y=target_y,
            lane_correction_gain=params.get('lane_correction_gain', 0.025),
        )
    elif behavior_type == 'abrupt':
        return AbruptBehavior(
            period=params.get('period', 4.0),
            target_speed_kmh=target_speed,
        )
    elif behavior_type == 'normal':
        return NormalBehavior(target_speed_kmh=target_speed)
    else:
        raise ValueError(f'Unknown behavior: {behavior_type}')


# ============ Setup 유틸 ============

def load_yaml(path):
    with open(path, encoding='utf-8') as f:
        return yaml.safe_load(f)


def setup_world(client, map_name):
    current = client.get_world().get_map().name.split('/')[-1]
    if current != map_name:
        print(f'맵 로드: {map_name} (현재 {current})')
        world = client.load_world(map_name)
        time.sleep(2.0)
    else:
        world = client.get_world()
        print(f'맵: {current} (재로드 불필요)')
    return world


def spawn_cameras(world, cameras_cfg, queues):
    """3 카메라 spawn 후 큐에 연결. 시나리오 전체에서 재사용."""
    cameras = {}
    cam_configs = {c['id']: c for c in cameras_cfg['cameras']}
    image_size = cameras_cfg.get('image_size', [1920, 1080])
    fov = cameras_cfg.get('fov', 90)

    for cam_id in CAMERA_IDS:
        cc = cam_configs[cam_id]
        transform = carla.Transform(
            carla.Location(
                x=cc['location']['x'],
                y=cc['location']['y'],
                z=cc['location']['z'],
            ),
            carla.Rotation(
                pitch=cc['rotation']['pitch'],
                yaw=cc['rotation']['yaw'],
                roll=cc['rotation']['roll'],
            ),
        )
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(image_size[0]))
        bp.set_attribute('image_size_y', str(image_size[1]))
        bp.set_attribute('fov', str(fov))
        cam = world.spawn_actor(bp, transform)
        cam.listen(queues[cam_id].put)
        cameras[cam_id] = cam
        print(f'  {cam_id} spawn')
    return cameras


def spawn_vehicle(world, cam0_cfg):
    """cam0 시야 방향 40m 앞에 차량 spawn."""
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


def set_initial_velocity(vehicle, speed_kmh):
    yaw_rad = math.radians(vehicle.get_transform().rotation.yaw)
    speed_ms = speed_kmh / 3.6
    vx = speed_ms * math.cos(yaw_rad)
    vy = speed_ms * math.sin(yaw_rad)
    vehicle.set_target_velocity(carla.Vector3D(x=vx, y=vy, z=0.0))


def drain_queues(queues):
    """시나리오 시작 전 큐에 남은 이전 프레임 제거."""
    for q in queues.values():
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break


# ============ 시나리오 단일 실행 ============

def run_one_scenario(world, cameras, queues, scenario, defaults,
                     cameras_cfg, tm_port):
    """단일 시나리오 녹화. 결과 dict 반환."""
    name = scenario['name']
    behavior_type = scenario['behavior']

    # 파라미터 머지
    duration_ticks = scenario.get(
        'duration_ticks', defaults['duration_ticks']
    )
    fixed_delta = scenario.get(
        'fixed_delta_seconds', defaults['fixed_delta_seconds']
    )
    initial_speed = scenario.get(
        'initial_speed_kmh', defaults['initial_speed_kmh']
    )

    cam_configs = {c['id']: c for c in cameras_cfg['cameras']}

    # 출력 폴더
    out_dir = os.path.join(OUTPUT_BASE, name)
    for cam_id in CAMERA_IDS:
        os.makedirs(os.path.join(out_dir, cam_id), exist_ok=True)

    # 큐 비우기
    drain_queues(queues)

    # 차량 spawn
    vehicle = spawn_vehicle(world, cam_configs['cam0'])

    # 첫 tick (액터 등록), 첫 프레임 폐기
    world.tick()
    for cam_id in CAMERA_IDS:
        try:
            queues[cam_id].get(timeout=2.0)
        except queue.Empty:
            pass

    # 초기 속도 + target_y 설정
    set_initial_velocity(vehicle, initial_speed)
    target_y = vehicle.get_location().y

    # Behavior 생성
    behavior = make_behavior(scenario, defaults, target_y=target_y)
    use_autopilot = isinstance(behavior, NormalBehavior)
    if use_autopilot:
        vehicle.set_autopilot(True, tm_port)

    # 녹화 루프
    gt_lines = []
    loss_count = {cam_id: 0 for cam_id in CAMERA_IDS}

    try:
        for i in range(duration_ticks):
            if not use_autopilot:
                vel = vehicle.get_velocity()
                speed_kmh = math.sqrt(vel.x ** 2 + vel.y ** 2) * 3.6
                cur_loc = vehicle.get_location()
                t = i * fixed_delta
                control = behavior.get_control(
                    t, {'speed_kmh': speed_kmh, 'y': cur_loc.y}
                )
                if control is not None:
                    vehicle.apply_control(control)

            world.tick()

            for cam_id in CAMERA_IDS:
                try:
                    image = queues[cam_id].get(timeout=TIMEOUT_PER_TICK)
                    out_path = os.path.join(
                        out_dir, cam_id, f'{i:06d}.png'
                    )
                    image.save_to_disk(out_path)
                except queue.Empty:
                    loss_count[cam_id] += 1

            # Ground truth
            loc = vehicle.get_location()
            vel = vehicle.get_velocity()
            rot = vehicle.get_transform().rotation
            ctrl = vehicle.get_control()
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
                'timestamp': round(i * fixed_delta, 3),
                'scenario': name,
                'behavior': behavior_type,
                'vehicles': [{
                    'id': vehicle.id,
                    'location': [round(loc.x, 3), round(loc.y, 3),
                                 round(loc.z, 3)],
                    'velocity': [round(vel.x, 3), round(vel.y, 3),
                                 round(vel.z, 3)],
                    'rotation': [round(rot.pitch, 2), round(rot.yaw, 2),
                                 round(rot.roll, 2)],
                    'speed_kmh': round(speed_ms * 3.6, 2),
                    'control': {
                        'throttle': round(ctrl.throttle, 3),
                        'steer': round(ctrl.steer, 3),
                        'brake': round(ctrl.brake, 3),
                    },
                    'distance_to': distances,
                    'lateral_offset': round(loc.y - target_y, 3),
                }],
            }
            gt_lines.append(json.dumps(gt))

        # 저장
        gt_path = os.path.join(out_dir, 'ground_truth.jsonl')
        with open(gt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(gt_lines) + '\n')

        # 시나리오 config 저장 (재현용)
        scenario_with_meta = dict(scenario)
        scenario_with_meta['_applied'] = {
            'duration_ticks': duration_ticks,
            'fixed_delta_seconds': fixed_delta,
            'initial_speed_kmh': initial_speed,
            'target_y': round(target_y, 3),
        }
        with open(os.path.join(out_dir, 'scenario_config.yaml'), 'w',
                  encoding='utf-8') as f:
            yaml.safe_dump(scenario_with_meta, f, allow_unicode=True,
                           sort_keys=False)

        # 요약 계산
        speeds = [json.loads(l)['vehicles'][0]['speed_kmh']
                  for l in gt_lines]
        offsets = [json.loads(l)['vehicles'][0]['lateral_offset']
                   for l in gt_lines]
        min_dists = {cam_id: min(json.loads(l)['vehicles'][0]
                                 ['distance_to'][cam_id]
                                 for l in gt_lines)
                     for cam_id in CAMERA_IDS}

        result = {
            'name': name,
            'behavior': behavior_type,
            'ticks_completed': duration_ticks,
            'png_loss': loss_count,
            'speed_avg': round(sum(speeds) / len(speeds), 2),
            'speed_std': round(statistics.stdev(speeds), 2),
            'offset_max': round(max(abs(o) for o in offsets), 3),
            'offset_std': round(statistics.stdev(offsets), 3),
            'min_distances': min_dists,
            'success': True,
        }

    finally:
        if use_autopilot:
            try:
                vehicle.set_autopilot(False, tm_port)
            except Exception:
                pass
        try:
            vehicle.destroy()
        except Exception:
            pass

    return result


# ============ Main ============

def main():
    print('=' * 70)
    print('Step D: 시나리오 배치 실행')
    print('=' * 70)

    # Config 로드
    cameras_cfg = load_yaml(CAMERAS_CONFIG_PATH)
    scenarios_cfg = load_yaml(SCENARIOS_CONFIG_PATH)
    defaults = scenarios_cfg.get('defaults', {})

    # enabled 시나리오만 선별
    all_scenarios = scenarios_cfg.get('scenarios', [])
    scenarios = [s for s in all_scenarios if s.get('enabled', True)]
    skipped = [s for s in all_scenarios if not s.get('enabled', True)]

    print(f'시나리오 총 {len(all_scenarios)}개 (실행 {len(scenarios)}, '
          f'스킵 {len(skipped)})')
    if skipped:
        print('  스킵: ' + ', '.join(s['name'] for s in skipped))
    print()

    # CARLA 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    print(f'서버: {client.get_server_version()}')

    world = setup_world(client, cameras_cfg.get('map', 'Town06'))
    original_settings = world.get_settings()
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()

    queues = {cam_id: queue.Queue() for cam_id in CAMERA_IDS}
    cameras = {}
    results = []

    try:
        # Sync mode 한 번만 ON
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = defaults.get(
            'fixed_delta_seconds', 0.05
        )
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        print('Sync mode + TM sync ON')

        # 카메라 한 번만 spawn (시나리오 전체에서 재사용)
        print('\n카메라 spawn:')
        cameras = spawn_cameras(world, cameras_cfg, queues)
        print()

        # 시나리오 일괄 실행
        for idx, scenario in enumerate(scenarios, 1):
            name = scenario['name']
            print(f'[{idx}/{len(scenarios)}] {name} ({scenario["behavior"]}) '
                  '실행 중...')
            start_time = time.time()

            try:
                result = run_one_scenario(
                    world, cameras, queues, scenario, defaults,
                    cameras_cfg, tm_port
                )
                elapsed = time.time() - start_time
                result['elapsed_sec'] = round(elapsed, 1)
                results.append(result)

                total_loss = sum(result['png_loss'].values())
                print(f'  완료 ({elapsed:.1f}s)  '
                      f'speed=[{result["speed_avg"]}±{result["speed_std"]}] '
                      f'offset=[max {result["offset_max"]}m, '
                      f'std {result["offset_std"]}m] '
                      f'loss={total_loss}')

            except Exception as e:
                elapsed = time.time() - start_time
                print(f'  실패: {type(e).__name__}: {e}')
                results.append({
                    'name': name,
                    'success': False,
                    'error': str(e),
                    'elapsed_sec': round(elapsed, 1),
                })

        # ============ 최종 요약 ============
        print()
        print('=' * 70)
        print('전체 결과')
        print('=' * 70)

        successful = [r for r in results if r.get('success')]
        failed = [r for r in results if not r.get('success')]

        print(f'성공: {len(successful)}/{len(scenarios)}')
        print(f'실패: {len(failed)}')
        print()

        # 시나리오별 요약 테이블
        if successful:
            print(f'{"시나리오":<20s} {"행동":<10s} '
                  f'{"평균속도":>10s} {"속도std":>9s} '
                  f'{"offset_max":>11s} {"offset_std":>11s}')
            print('-' * 75)
            for r in successful:
                print(f'{r["name"]:<20s} {r["behavior"]:<10s} '
                      f'{r["speed_avg"]:>8.1f}   '
                      f'{r["speed_std"]:>7.1f}   '
                      f'{r["offset_max"]:>9.2f}m '
                      f'{r["offset_std"]:>9.2f}m')

        if failed:
            print()
            print('실패한 시나리오:')
            for r in failed:
                print(f'  {r["name"]}: {r.get("error", "?")}')

        # 디스크 사용량 추정
        print()
        total_size_mb = 0
        for r in successful:
            for cam_id in CAMERA_IDS:
                folder = os.path.join(OUTPUT_BASE, r['name'], cam_id)
                if os.path.exists(folder):
                    for f in os.listdir(folder):
                        total_size_mb += os.path.getsize(
                            os.path.join(folder, f)
                        ) / 1024 / 1024
        print(f'총 디스크 사용량: {total_size_mb:.0f} MB '
              f'({total_size_mb / 1024:.1f} GB)')

        print()
        if not failed:
            print('>>> Step D 통과. 모든 시나리오 데이터 확보. <<<')
            print('>>> 다음: Step G — 차량 trajectory 클러스터링 모듈 <<<')
        else:
            print('>>> 일부 시나리오 실패. 위 로그 확인. <<<')

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
        try:
            tm.set_synchronous_mode(False)
            world.apply_settings(original_settings)
            print('Sync mode + TM sync 해제')
        except Exception as e:
            print(f'[ERROR] 설정 복원 실패: {e}')


if __name__ == '__main__':
    main()
