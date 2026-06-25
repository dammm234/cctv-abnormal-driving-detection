"""
Step C: 차량 행동 제어 — 차선 유지 보정 추가

이전 시도에서 amplitude=0.08이 너무 강해 차량이 차선을 이탈하여 충돌함.
수정:
- amplitude 0.08 → 0.04 (절반)
- WobbleBehavior에 차선 유지 보정(lane keeping) 추가
- 차선 중심 y는 vehicle spawn 시점의 y로 자동 설정
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


# ============ CONFIG ============
BEHAVIOR_TYPE = 'normal'         # 'wobble' | 'abrupt' | 'normal'
INITIAL_SPEED_KMH = 60.0
N_TICKS = 250                     # 12.5초
TARGET_SPEED_KMH = 60.0

# Wobble 파라미터 (수정)
WOBBLE_AMPLITUDE = 0.04           
WOBBLE_PERIOD = 2.0
LANE_CORRECTION_GAIN = 0.025      # 차선 유지 보정 강도

# Abrupt 파라미터
ABRUPT_PERIOD = 4.0
# ================================

CONFIG_PATH = 'config/cameras.yaml'
OUT_BASE = f'data/scenarios/step_c_{BEHAVIOR_TYPE}'
FIXED_DELTA = 0.05
TIMEOUT_PER_TICK = 5.0
CAMERA_IDS = ['cam0', 'cam1', 'cam2']


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
    """sin 곡선 좌우 진동 + 차선 유지 보정."""

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

        # 차선 유지 보정 (vehicle facing -X 가정)
        # current_y가 target_y보다 작으면(=-Y쪽 드리프트) negative steer로 +Y 방향 유도
        if self.target_y is not None and 'y' in vehicle_state:
            lane_correction = self.correction_gain * (
                vehicle_state['y'] - self.target_y
            )
        else:
            lane_correction = 0.0

        steer = wobble + lane_correction
        steer = max(-0.15, min(0.15, steer))  # 안전 clamp

        return carla.VehicleControl(throttle=throttle, steer=steer, brake=0.0)


class AbruptBehavior(BehaviorController):
    """주기적 throttle/brake 토글."""

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
        return None


def make_behavior(behavior_type, target_y=None):
    if behavior_type == 'wobble':
        return WobbleBehavior(
            amplitude=WOBBLE_AMPLITUDE,
            period=WOBBLE_PERIOD,
            target_speed_kmh=TARGET_SPEED_KMH,
            target_y=target_y,
            lane_correction_gain=LANE_CORRECTION_GAIN,
        )
    elif behavior_type == 'abrupt':
        return AbruptBehavior(
            period=ABRUPT_PERIOD,
            target_speed_kmh=TARGET_SPEED_KMH,
        )
    elif behavior_type == 'normal':
        return NormalBehavior(target_speed_kmh=TARGET_SPEED_KMH)
    else:
        raise ValueError(f'Unknown behavior: {behavior_type}')



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


def main():
    print('=' * 60)
    print(f'Step C: 차량 행동 제어 ({BEHAVIOR_TYPE})')
    print('=' * 60)
    print(f'시나리오: {BEHAVIOR_TYPE}')
    if BEHAVIOR_TYPE == 'wobble':
        print(f'  amplitude={WOBBLE_AMPLITUDE}, period={WOBBLE_PERIOD}s, '
              f'lane_correction_gain={LANE_CORRECTION_GAIN}')
    elif BEHAVIOR_TYPE == 'abrupt':
        print(f'  period={ABRUPT_PERIOD}s')
    print(f'초기 속도: {INITIAL_SPEED_KMH} km/h')
    print(f'녹화 길이: {N_TICKS * FIXED_DELTA}s ({N_TICKS} tick)')
    print('=' * 60)

    cfg = load_config()
    map_name = cfg.get('map', 'Town06')
    image_size = cfg.get('image_size', [1920, 1080])
    fov = cfg.get('fov', 90)
    cam_configs = {c['id']: c for c in cfg['cameras']}

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

    queues = {cam_id: queue.Queue() for cam_id in CAMERA_IDS}
    cameras = {}
    vehicle = None
    gt_lines = []
    behavior = None

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        print('Sync mode ON\n')

        for cam_id in CAMERA_IDS:
            cam = spawn_camera(world, cam_configs[cam_id], image_size, fov)
            cam.listen(queues[cam_id].put)
            cameras[cam_id] = cam
            print(f'  {cam_id} spawn')

        vehicle = spawn_vehicle(world, cam_configs['cam0'])
        print(f'\n차량 id={vehicle.id} spawn')

        world.tick()
        for cam_id in CAMERA_IDS:
            try:
                queues[cam_id].get(timeout=2.0)
            except queue.Empty:
                pass

        set_initial_velocity(vehicle, INITIAL_SPEED_KMH)
        print(f'초기 속도 {INITIAL_SPEED_KMH} km/h 설정')

        # 차선 중심 y를 vehicle spawn 위치 y로 설정
        target_y = vehicle.get_location().y
        print(f'차선 중심 y={target_y:.2f}로 설정')

        # target_y 알아낸 다음 behavior 생성
        behavior = make_behavior(BEHAVIOR_TYPE, target_y=target_y)
        use_autopilot = isinstance(behavior, NormalBehavior)

        if use_autopilot:
            vehicle.set_autopilot(True, tm_port)
            print('Autopilot ON (Normal behavior)')
        else:
            print(f'수동 제어 ON ({BEHAVIOR_TYPE} behavior)')

        print(f'\n{N_TICKS}번 tick 녹화:')
        loss_count = {cam_id: 0 for cam_id in CAMERA_IDS}

        for i in range(N_TICKS):
            if not use_autopilot:
                vel = vehicle.get_velocity()
                speed_kmh = math.sqrt(vel.x ** 2 + vel.y ** 2) * 3.6
                cur_loc = vehicle.get_location()
                t = i * FIXED_DELTA
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
                        OUT_BASE, cam_id, f'{i:06d}.png'
                    )
                    image.save_to_disk(out_path)
                except queue.Empty:
                    print(f'  [WARN] Tick {i}: {cam_id} timeout')
                    loss_count[cam_id] += 1

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
                'timestamp': round(i * FIXED_DELTA, 3),
                'behavior': BEHAVIOR_TYPE,
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

            if i % 50 == 0:
                offset = loc.y - target_y
                print(f'  Tick {i:3d}: x={loc.x:6.1f}, '
                      f'y_offset={offset:+5.2f}m, '
                      f'{speed_ms * 3.6:5.1f} km/h, '
                      f'steer={ctrl.steer:+.3f}')

        with open(gt_path, 'w', encoding='utf-8') as f:
            f.write('\n'.join(gt_lines) + '\n')

        # 결과 분석
        print()
        print('=' * 60)
        print('결과')
        print('=' * 60)

        all_pass = True

        for cam_id in CAMERA_IDS:
            saved = len(os.listdir(os.path.join(OUT_BASE, cam_id)))
            status = '[OK]' if saved == N_TICKS else '[FAIL]'
            print(f'{status} {cam_id}: {saved}/{N_TICKS} '
                  f'(loss={loss_count[cam_id]})')
            if saved != N_TICKS:
                all_pass = False
        print(f'Ground truth: {len(gt_lines)}/{N_TICKS} 행')

        print('\n차량 통과 검증:')
        min_distances = {cam_id: float('inf') for cam_id in CAMERA_IDS}
        for line in gt_lines:
            d = json.loads(line)
            for cam_id in CAMERA_IDS:
                dist = d['vehicles'][0]['distance_to'][cam_id]
                min_distances[cam_id] = min(min_distances[cam_id], dist)
        for cam_id in CAMERA_IDS:
            md = min_distances[cam_id]
            close = '✓ 가까이 통과' if md < 30 else '○ 멀리 있음'
            print(f'  {cam_id} 최단 거리: {md:5.1f}m  {close}')

        print(f'\n{BEHAVIOR_TYPE} 행동 적용 검증:')
        skip = 20
        late = [json.loads(line) for line in gt_lines[skip:]]
        steers = [d['vehicles'][0]['control']['steer'] for d in late]
        throttles = [d['vehicles'][0]['control']['throttle'] for d in late]
        brakes = [d['vehicles'][0]['control']['brake'] for d in late]
        speeds = [d['vehicles'][0]['speed_kmh'] for d in late]
        offsets = [d['vehicles'][0]['lateral_offset'] for d in late]

        if BEHAVIOR_TYPE == 'wobble':
            steer_max = max(abs(s) for s in steers)
            offset_std = statistics.stdev(offsets) if len(offsets) > 1 else 0
            offset_max = max(abs(o) for o in offsets)
            speed_avg = sum(speeds) / len(speeds)
            print(f'  최대 |steer|:           {steer_max:.3f}')
            print(f'  lateral offset 표준편차: {offset_std:.3f}m '
                  f'(정상 ~0.1, wobble 0.2~0.6, 이탈 >1.0)')
            print(f'  lateral offset 최대값:   {offset_max:.3f}m '
                  f'(이탈 신호 >2m)')
            print(f'  평균 속도:              {speed_avg:.1f} km/h '
                  f'(목표 {TARGET_SPEED_KMH})')

            if offset_max > 2.0:
                print('  ✗ 차량이 차선을 이탈함')
                all_pass = False
            elif offset_std > 0.15 and offset_std < 1.0:
                print('  ✓ wobble 패턴 명확 + 차선 내 유지')
            elif offset_std < 0.1:
                print('  ? wobble이 너무 약함')
            else:
                print('  ? 패턴이 모호함')

        elif BEHAVIOR_TYPE == 'abrupt':
            throttle_max = max(throttles)
            brake_max = max(brakes)
            speed_std = statistics.stdev(speeds) if len(speeds) > 1 else 0
            print(f'  최대 throttle:    {throttle_max:.3f}')
            print(f'  최대 brake:       {brake_max:.3f}')
            print(f'  속도 표준편차:    {speed_std:.1f} km/h')
            if throttle_max > 0.8 and brake_max > 0.5 and speed_std > 8:
                print('  ✓ abrupt 패턴 명확히 관찰됨')

        elif BEHAVIOR_TYPE == 'normal':
            offset_std = statistics.stdev(offsets) if len(offsets) > 1 else 0
            speed_std = statistics.stdev(speeds) if len(speeds) > 1 else 0
            print(f'  lateral offset 표준편차: {offset_std:.3f}m')
            print(f'  속도 표준편차:          {speed_std:.1f} km/h')
            if offset_std < 0.3 and speed_std < 8:
                print('  ✓ 정상 운전 baseline 확보')

        print()
        if all_pass:
            print(f'>>> Step C ({BEHAVIOR_TYPE}) 통과 <<<')
        else:
            print('>>> 일부 검증 실패 <<<')

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
            print('Sync mode 해제')
        except Exception as e:
            print(f'[ERROR] 설정 복원 실패: {e}')


if __name__ == '__main__':
    main()
