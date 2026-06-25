"""
다중 차량 ReID 시나리오 생성기

"""
import argparse
import json
import math
import os
import queue
import time
import traceback

import numpy as np

import carla
import yaml


CAMERAS_CONFIG_PATH = 'config/cameras.yaml'
OUTPUT_BASE = 'data/scenarios'
V11_OUTPUT_BASE = 'scenarios_v1.1_multi'
CAMERA_IDS = ['cam0', 'cam1', 'cam2']
TIMEOUT_PER_TICK = 5.0


class WobbleBehavior:
    """차량 횡위치를 매 프레임 직접 사인파로 지정 (set_transform 방식).

    P제어/조향 방식은 CARLA 동역학·관성 때문에 진동이 깨지거나 발산했다.
    이 방식은 물리에 맡기지 않고 횡위치(y)를 매 프레임 강제로 설정 →
    진폭·주기대로 정확히 좌우 진동. 동역학 변수 없음, 발산 없음.

    종방향(x)·속도는 그대로 두고 횡방향(y)만 center + amp_m·sin으로 덮어씀.
    영상에서 차량이 amp_m(미터) 진폭으로 눈에 띄게 좌우로 흔들린다.

    apply 방식이 달라 control() 대신 lateral_offset()을 제공:
    호출측에서 set_transform으로 위치를 직접 갱신한다.
    """
    def __init__(self, amp_m=0.6, period=1.5, center_y=None):
        self.amp_m = amp_m
        self.period = period
        self.center_y = center_y

    def lateral_offset(self, t):
        """시각 t에서 중심 대비 횡방향 오프셋(m)."""
        return self.amp_m * math.sin(2 * math.pi * t / self.period)


# wobble 강도 프리셋 — amp_m(횡진동 진폭, m), period(주기, s).
# 위치를 직접 지정하므로 값이 그대로 횡변위가 됨.
# period 1.5s면 방향전환이 초당 ~1.3회 → 지그재그 임계(0.3) 확실히 초과.
WOBBLE_PRESETS = {
    'wobble_strong': dict(amp_m=0.70, period=1.4),
    'wobble_medium': dict(amp_m=0.55, period=1.6),
    'wobble_mild':   dict(amp_m=0.40, period=1.8),
}


class SwerveBehavior:
    """한 번 강하게 휘청(급조향) 후 복귀. 졸음 중 각성/급회피 재현.

    평소엔 거의 직진하다가, 지정 시점(trigger_t)에 짧고 강하게 한쪽으로
    꺾었다가 돌아온다. wobble(반복)과 달리 단발성 큰 횡 이동.

    복귀 강화: swerve 직후 target_y로 적극 복귀시켜 횡변위를 1~1.5m로 통제
    (이전엔 복귀가 약해 8.5m까지 표류 → lane_change로 변질됨).
    """
    def __init__(self, target_speed_kmh=60.0, trigger_t=6.0,
                 swerve_dur=1.0, swerve_strength=0.13, target_y=None,
                 recover_gain=0.06, recover_deadzone=0.15):
        self.target_speed_kmh = target_speed_kmh
        self.trigger_t = trigger_t
        self.swerve_dur = swerve_dur
        self.swerve_strength = swerve_strength
        self.target_y = target_y
        self.recover_gain = recover_gain
        self.recover_deadzone = recover_deadzone

    def control(self, t, speed_kmh, cur_y):
        error = self.target_speed_kmh - speed_kmh
        throttle = max(0.0, min(1.0, 0.35 + error * 0.02))

        dt = t - self.trigger_t
        if 0.0 <= dt < self.swerve_dur:
            # 반주기 sin: 한쪽으로 확 꺾었다가 복귀 (단발성)
            steer = self.swerve_strength * math.sin(math.pi * dt / self.swerve_dur)
        else:
            # swerve 전후 모두: target_y로 적극 복귀 (표류 방지)
            steer = 0.0
            if self.target_y is not None:
                dev = cur_y - self.target_y
                if abs(dev) > self.recover_deadzone:
                    steer = -self.recover_gain * dev

        steer = max(-0.30, min(0.30, steer))
        return carla.VehicleControl(throttle=throttle, steer=steer, brake=0.0)


# swerve 프리셋. strength를 낮춰 횡변위를 차선 1~1.5칸 수준으로 통제.
SWERVE_PRESETS = {
    'swerve_strong': dict(swerve_dur=1.0, swerve_strength=0.15),
    'swerve_medium': dict(swerve_dur=1.2, swerve_strength=0.11),
}

# 외형이 뚜렷이 다른 차량 blueprint + 색 (BGR 아님, CARLA는 RGB 문자열)
# ReID 변별력을 위해 모델과 색을 모두 다르게.
VEHICLE_SPECS = [
    ('vehicle.audi.tt', '255,0,0'),          # 빨강
    ('vehicle.bmw.grandtourer', '0,0,255'),  # 파랑
    ('vehicle.mercedes.coupe', '255,255,255'), # 흰색
    ('vehicle.tesla.model3', '0,0,0'),       # 검정
    ('vehicle.nissan.patrol', '255,255,0'),  # 노랑
    ('vehicle.audi.etron', '0,255,0'),       # 초록
    ('vehicle.chevrolet.impala', '128,0,128'), # 보라
    ('vehicle.dodge.charger_2020', '255,128,0'), # 주황
    ('vehicle.lincoln.mkz_2017', '0,255,255'),   # 청록(cyan)
    ('vehicle.toyota.prius', '128,128,128'),     # 회색
    ('vehicle.jeep.wrangler_rubicon', '139,69,19'), # 갈색
    ('vehicle.mini.cooper_s', '255,0,255'),      # 핑크/마젠타
]


# ============ 카메라 intrinsic / 투영 ============

def build_intrinsic(image_w, image_h, fov_deg):
    """카메라 intrinsic matrix K."""
    f = image_w / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    cx = image_w / 2.0
    cy = image_h / 2.0
    K = np.array([
        [f, 0, cx],
        [0, f, cy],
        [0, 0, 1],
    ], dtype=float)
    return K


def camera_world_to_camera_matrix(cam_transform):
    """world → camera 좌표 변환 행렬 (4x4).

    CARLA carla.Transform.get_inverse_matrix() 사용.
    """
    return np.array(cam_transform.get_inverse_matrix())


def project_point_world_to_image(world_point, w2c, K):
    """월드 좌표 1점 → 이미지 픽셀 (또는 None if 카메라 뒤).

    CARLA(UE4)는 좌수 좌표계. 카메라 좌표 (x_c forward, y_c right, z_c up)에서
    표준 핀홀 카메라 좌표 (X right, Y down, Z forward)로 재배열:
        X =  y_c
        Y = -z_c
        Z =  x_c
    """
    p = np.array([world_point.x, world_point.y, world_point.z, 1.0])
    p_cam = w2c @ p  # (x_c, y_c, z_c, 1)

    x_c, y_c, z_c = p_cam[0], p_cam[1], p_cam[2]

    # 카메라 뒤 (forward = x_c <= 0)면 투영 불가
    if x_c <= 0.01:
        return None

    # 표준 핀홀 좌표로 재배열
    point_std = np.array([y_c, -z_c, x_c])
    pixel = K @ point_std
    u = pixel[0] / pixel[2]
    v = pixel[1] / pixel[2]
    return (u, v)


def vehicle_bbox_to_2d(vehicle, w2c, K, image_w, image_h):
    """차량의 3D bounding box 8개 꼭짓점을 투영 → 2D bbox_pixel [x1,y1,x2,y2].

    꼭짓점 중 하나라도 카메라 앞에 있고 화면과 겹치면 박스 반환.
    전부 카메라 뒤거나 완전히 화면 밖이면 None.
    """
    bb = vehicle.bounding_box
    # bounding_box.get_world_vertices(transform)로 8 꼭짓점 (world 좌표)
    verts = bb.get_world_vertices(vehicle.get_transform())

    us, vs = [], []
    any_in_front = False
    for vtx in verts:
        proj = project_point_world_to_image(vtx, w2c, K)
        if proj is None:
            continue
        any_in_front = True
        us.append(proj[0])
        vs.append(proj[1])

    if not any_in_front or len(us) < 2:
        return None

    x1 = min(us)
    y1 = min(vs)
    x2 = max(us)
    y2 = max(vs)

    # 화면과 교집합 없음 → 제외
    if x2 < 0 or x1 > image_w or y2 < 0 or y1 > image_h:
        return None

    # 화면 경계로 clip
    x1 = max(0.0, min(x1, image_w))
    y1 = max(0.0, min(y1, image_h))
    x2 = max(0.0, min(x2, image_w))
    y2 = max(0.0, min(y2, image_h))

    # 너무 작으면 (멀거나 가장자리 살짝) 제외
    if (x2 - x1) < 8 or (y2 - y1) < 8:
        return None

    return [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)]


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


def make_camera_transform(cc):
    return carla.Transform(
        carla.Location(x=cc['location']['x'], y=cc['location']['y'],
                       z=cc['location']['z']),
        carla.Rotation(pitch=cc['rotation']['pitch'],
                       yaw=cc['rotation']['yaw'],
                       roll=cc['rotation']['roll']),
    )


def spawn_cameras(world, cameras_cfg, queues):
    """3 카메라 spawn + 큐 연결. transform도 함께 반환 (투영용)."""
    cameras = {}
    cam_transforms = {}
    cam_configs = {c['id']: c for c in cameras_cfg['cameras']}
    image_size = cameras_cfg.get('image_size', [1920, 1080])
    fov = cameras_cfg.get('fov', 90)

    for cam_id in CAMERA_IDS:
        cc = cam_configs[cam_id]
        transform = make_camera_transform(cc)
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', str(image_size[0]))
        bp.set_attribute('image_size_y', str(image_size[1]))
        bp.set_attribute('fov', str(fov))
        cam = world.spawn_actor(bp, transform)
        cam.listen(queues[cam_id].put)
        cameras[cam_id] = cam
        cam_transforms[cam_id] = transform
        print(f'  {cam_id} spawn')
    return cameras, cam_transforms


def get_lane_waypoints(base_wp):
    """base_wp 기준 주행 가능한 같은 방향 차선들의 waypoint 목록.

    좌/우로 탐색해 driving lane만 수집. (반대 방향 lane 제외)
    """
    lanes = [base_wp]
    # 오른쪽으로
    wp = base_wp
    for _ in range(4):
        r = wp.get_right_lane()
        if r is None or r.lane_type != carla.LaneType.Driving:
            break
        # 같은 방향만 (lane_id 부호 동일)
        if (r.lane_id > 0) != (base_wp.lane_id > 0):
            break
        lanes.append(r)
        wp = r
    # 왼쪽으로
    wp = base_wp
    for _ in range(4):
        l = wp.get_left_lane()
        if l is None or l.lane_type != carla.LaneType.Driving:
            break
        if (l.lane_id > 0) != (base_wp.lane_id > 0):
            break
        lanes.insert(0, l)
        wp = l
    return lanes


def spawn_vehicles(world, cam0_cfg, n_vehicles, tm_port,
                   behavior_plan=None):
    """차선 분산 spawn + behavior 할당.

    behavior_plan: [behavior_str, ...] (차량 순서대로).
        'normal' / 'wobble_strong' / 'wobble_mild' 등.
        None이면 전부 normal(autopilot).

    여러 차선 × 종방향 위치로 분산해 화면에서 박스가 겹치지 않게.
    return: [(vehicle, behavior_str), ...]
    """
    bp_lib = world.get_blueprint_library()
    base = carla.Location(x=cam0_cfg['location']['x'] + 30,
                          y=cam0_cfg['location']['y'],
                          z=cam0_cfg['location']['z'] - 5)
    base_wp = world.get_map().get_waypoint(base, project_to_road=True)

    lane_wps = get_lane_waypoints(base_wp)
    n_lanes = len(lane_wps)
    print(f'  주행 차선 {n_lanes}개 탐지')

    n = min(n_vehicles, len(VEHICLE_SPECS))
    if behavior_plan is None:
        behavior_plan = ['normal'] * n

    vehicles = []
    # 차선별로 종방향 위치를 다르게 (대각선 배치 → 화면에서 분리)
    lane_counters = {i: 0 for i in range(n_lanes)}

    for i in range(n):
        model, color = VEHICLE_SPECS[i]
        behavior = behavior_plan[i] if i < len(behavior_plan) else 'normal'

        bp = bp_lib.filter(model)
        if not bp:
            bp = bp_lib.filter('vehicle.tesla.model3')
        bp = bp[0]
        if bp.has_attribute('color'):
            try:
                bp.set_attribute('color', color)
            except Exception:
                pass

        # 차선 라운드로빈 배치
        lane_idx = i % n_lanes
        slot = lane_counters[lane_idx]
        lane_counters[lane_idx] += 1

        # 해당 차선 waypoint에서 종방향으로 slot*간격 뒤로
        wp = lane_wps[lane_idx]
        back_steps = slot * 9  # 같은 차선 내 차간 약 18m
        for _ in range(back_steps):
            nxt = wp.previous(2.0)
            if nxt:
                wp = nxt[0]
        spawn_t = wp.transform
        spawn_t.location.z += 0.5

        v = world.try_spawn_actor(bp, spawn_t)
        if v is None:
            for dz in (0.5, 1.0, 1.5):
                spawn_t.location.z += dz
                v = world.try_spawn_actor(bp, spawn_t)
                if v is not None:
                    break
        if v is None:
            print(f'  [WARN] 차량 {i} ({model}) spawn 실패, skip')
            continue
        vehicles.append((v, behavior))
        print(f'  veh{i} id={v.id} {model} color={color} '
              f'lane={lane_idx} behavior={behavior}')

    return vehicles


def drain_queues(queues):
    for q in queues.values():
        while not q.empty():
            try:
                q.get_nowait()
            except queue.Empty:
                break


# ============ 시나리오 실행 ============

def run_multi(world, cameras, cam_transforms, queues, cameras_cfg,
              name, n_vehicles, duration_ticks, fixed_delta,
              initial_speed, tm, tm_port, behavior_plan=None):
    cam_configs = {c['id']: c for c in cameras_cfg['cameras']}
    image_size = cameras_cfg.get('image_size', [1920, 1080])
    fov = cameras_cfg.get('fov', 90)
    image_w, image_h = image_size[0], image_size[1]

    K = build_intrinsic(image_w, image_h, fov)
    w2c = {cam_id: camera_world_to_camera_matrix(cam_transforms[cam_id])
           for cam_id in CAMERA_IDS}

    out_dir = os.path.join(OUTPUT_BASE, name)
    for cam_id in CAMERA_IDS:
        os.makedirs(os.path.join(out_dir, cam_id), exist_ok=True)

    drain_queues(queues)

    veh_beh = spawn_vehicles(world, cam_configs['cam0'], n_vehicles, tm_port,
                             behavior_plan=behavior_plan)
    if not veh_beh:
        raise RuntimeError('차량 spawn 0대')
    vehicles = [v for v, _ in veh_beh]

    # 첫 tick (액터 등록), 첫 프레임 폐기
    world.tick()
    for cam_id in CAMERA_IDS:
        try:
            queues[cam_id].get(timeout=2.0)
        except queue.Empty:
            pass

    # behavior별 제어 설정: normal=autopilot, wobble/swerve=수동
    manual_ctrl = {}   # vehicle.id → WobbleBehavior or SwerveBehavior
    for v, beh in veh_beh:
        if beh.startswith('wobble'):
            preset = WOBBLE_PRESETS.get(beh, WOBBLE_PRESETS['wobble_strong'])
            wb = WobbleBehavior(center_y=v.get_location().y, **preset)
            manual_ctrl[v.id] = wb
            # 종방향 주행은 autopilot에 맡기고, 횡위치만 매 프레임 덮어씀
            v.set_autopilot(True, tm_port)
        elif beh.startswith('swerve'):
            preset = SWERVE_PRESETS.get(beh, SWERVE_PRESETS['swerve_strong'])
            manual_ctrl[v.id] = SwerveBehavior(
                target_speed_kmh=initial_speed,
                target_y=v.get_location().y, **preset)
            v.set_autopilot(False, tm_port)
        else:
            v.set_autopilot(True, tm_port)

    gt_lines = []
    # v1.1 누적: cam별 frames
    v11 = {cam_id: {'version': '1.1', 'scenario': name, 'camera': cam_id,
                    'fps': round(1.0 / fixed_delta, 2), 'frames': []}
           for cam_id in CAMERA_IDS}
    loss_count = {cam_id: 0 for cam_id in CAMERA_IDS}

    try:
        for i in range(duration_ticks):
            # 이상거동 차량 제어 (tick 전에 적용)
            t_sec = i * fixed_delta
            for v, beh in veh_beh:
                if v.id not in manual_ctrl or not v.is_alive:
                    continue
                ctrl_obj = manual_ctrl[v.id]
                if isinstance(ctrl_obj, WobbleBehavior):
                    # 횡위치(y)를 사인파로 직접 덮어씀. 종방향은 autopilot 유지.
                    tf = v.get_transform()
                    loc = tf.location
                    target_y = ctrl_obj.center_y + ctrl_obj.lateral_offset(t_sec)
                    loc.y = target_y
                    tf.location = loc
                    v.set_transform(tf)
                else:
                    # swerve 등: 조향 제어
                    vel = v.get_velocity()
                    spd_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)
                    cur_y = v.get_location().y
                    ctrl = ctrl_obj.control(t_sec, spd_kmh, cur_y)
                    v.apply_control(ctrl)

            world.tick()

            # 카메라 프레임 저장
            for cam_id in CAMERA_IDS:
                try:
                    image = queues[cam_id].get(timeout=TIMEOUT_PER_TICK)
                    image.save_to_disk(
                        os.path.join(out_dir, cam_id, f'{i:06d}.png'))
                except queue.Empty:
                    loss_count[cam_id] += 1

            # 차량별 GT + 카메라별 bbox 투영
            frame_vehicles_3d = []
            v11_frame = {cam_id: {'frame_id': i, 'vehicles': []}
                         for cam_id in CAMERA_IDS}

            for v in vehicles:
                if not v.is_alive:
                    continue
                loc = v.get_location()
                vel = v.get_velocity()
                rot = v.get_transform().rotation
                speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

                bbox_per_cam = {}
                for cam_id in CAMERA_IDS:
                    box = vehicle_bbox_to_2d(v, w2c[cam_id], K,
                                             image_w, image_h)
                    if box is not None:
                        bbox_per_cam[cam_id] = box
                        v11_frame[cam_id]['vehicles'].append({
                            'track_id': v.id,
                            'bbox_pixel': box,
                            'position_road_m': [round(loc.x, 3),
                                                round(loc.y, 3)],
                            'speed_est_mps': round(speed_ms, 3),
                        })

                frame_vehicles_3d.append({
                    'id': v.id,
                    'location': [round(loc.x, 3), round(loc.y, 3),
                                 round(loc.z, 3)],
                    'velocity': [round(vel.x, 3), round(vel.y, 3),
                                 round(vel.z, 3)],
                    'rotation': [round(rot.pitch, 2), round(rot.yaw, 2),
                                 round(rot.roll, 2)],
                    'speed_kmh': round(speed_ms * 3.6, 2),
                    'bbox_pixel': bbox_per_cam,  # {cam_id: [x1,y1,x2,y2]}
                })

            gt_lines.append(json.dumps({
                'frame': i,
                'timestamp': round(i * fixed_delta, 3),
                'scenario': name,
                'vehicles': frame_vehicles_3d,
            }))
            for cam_id in CAMERA_IDS:
                v11[cam_id]['frames'].append(v11_frame[cam_id])

        # 저장: ground_truth.jsonl
        with open(os.path.join(out_dir, 'ground_truth.jsonl'),
                  'w', encoding='utf-8') as f:
            f.write('\n'.join(gt_lines) + '\n')

        # 저장: v1.1 JSON (cam별)
        os.makedirs(V11_OUTPUT_BASE, exist_ok=True)
        for cam_id in CAMERA_IDS:
            v11_path = os.path.join(V11_OUTPUT_BASE, f'{name}_{cam_id}.json')
            with open(v11_path, 'w', encoding='utf-8') as f:
                json.dump(v11[cam_id], f, ensure_ascii=False)

        # 요약: 카메라별 차량별 박스 등장 프레임 수
        box_counts = {cam_id: {} for cam_id in CAMERA_IDS}
        for cam_id in CAMERA_IDS:
            for fr in v11[cam_id]['frames']:
                for veh in fr['vehicles']:
                    tid = veh['track_id']
                    box_counts[cam_id][tid] = box_counts[cam_id].get(tid, 0) + 1

        result = {
            'name': name,
            'n_vehicles': len(vehicles),
            'vehicle_ids': [v.id for v in vehicles],
            'ticks': duration_ticks,
            'png_loss': loss_count,
            'box_counts_per_cam': box_counts,
            'success': True,
        }

    finally:
        for v in vehicles:
            try:
                v.set_autopilot(False, tm_port)
            except Exception:
                pass
            try:
                v.destroy()
            except Exception:
                pass

    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--n-vehicles', type=int, default=6)
    parser.add_argument('--duration', type=int, default=250)
    parser.add_argument('--name', default='multi_reid')
    parser.add_argument('--initial-speed', type=float, default=60.0)
    parser.add_argument('--abnormal-vehicles', '--wobble-vehicles',
                        dest='wobble_vehicles', default='',
                        help='이상거동 차량 인덱스:종류. '
                             'wobble_strong/medium/mild, swerve_strong/medium. '
                             '예: "1:wobble_strong,3:swerve_strong,4:wobble_mild"')
    args = parser.parse_args()

    cameras_cfg = load_yaml(CAMERAS_CONFIG_PATH)
    fixed_delta = 0.05

    # behavior_plan 구성
    n = min(args.n_vehicles, len(VEHICLE_SPECS))
    behavior_plan = ['normal'] * n
    if args.wobble_vehicles:
        for item in args.wobble_vehicles.split(','):
            idx_str, beh = item.split(':')
            idx = int(idx_str)
            if 0 <= idx < n:
                behavior_plan[idx] = beh
    print(f'behavior_plan: {behavior_plan}')

    print('=' * 60)
    print(f'다중 차량 ReID 시나리오: {args.name} '
          f'({args.n_vehicles}대, {args.duration} ticks)')
    print('=' * 60)

    client = carla.Client('localhost', 2000)
    client.set_timeout(20.0)
    print(f'서버: {client.get_server_version()}')

    world = setup_world(client, cameras_cfg.get('map', 'Town06'))
    original_settings = world.get_settings()
    tm = client.get_trafficmanager()
    tm_port = tm.get_port()

    queues = {cam_id: queue.Queue() for cam_id in CAMERA_IDS}
    cameras = {}

    try:
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = fixed_delta
        world.apply_settings(settings)
        tm.set_synchronous_mode(True)
        print('Sync mode + TM sync ON')

        print('\n카메라 spawn:')
        cameras, cam_transforms = spawn_cameras(world, cameras_cfg, queues)

        print(f'\n[{args.name}] 실행 중...')
        start = time.time()
        result = run_multi(
            world, cameras, cam_transforms, queues, cameras_cfg,
            args.name, args.n_vehicles, args.duration, fixed_delta,
            args.initial_speed, tm, tm_port, behavior_plan=behavior_plan)
        elapsed = time.time() - start
        print(f'  완료 ({elapsed:.1f}s)')
        print(f'  차량 {result["n_vehicles"]}대, ids={result["vehicle_ids"]}')
        print(f'  png_loss={result["png_loss"]}')
        print('\n  카메라별 차량 박스 등장 프레임 수:')
        for cam_id in CAMERA_IDS:
            counts = result['box_counts_per_cam'][cam_id]
            print(f'    {cam_id}: ' +
                  ', '.join(f'id{tid}={c}' for tid, c in sorted(counts.items())))

        # 검증 힌트
        print('\n  [검증] 각 차량이 2개 이상 카메라에 잡혀야 cross-camera 매칭 가능.')
        all_ids = set(result['vehicle_ids'])
        for tid in sorted(all_ids):
            cams_seen = [c for c in CAMERA_IDS
                         if tid in result['box_counts_per_cam'][c]]
            mark = 'OK' if len(cams_seen) >= 2 else '부족'
            print(f'    id{tid}: {len(cams_seen)}개 카메라 {cams_seen} [{mark}]')

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
        try:
            tm.set_synchronous_mode(False)
            world.apply_settings(original_settings)
            print('\nSync mode 해제')
        except Exception as e:
            print(f'[ERROR] 복원 실패: {e}')


if __name__ == '__main__':
    main()
