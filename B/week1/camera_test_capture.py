"""
cameras.yaml에 정의된 카메라 위치에서 각각 1장씩 PNG로 캡처.

cameras.yaml 편집 → 이 스크립트 실행 → camera_check/cam*.png를 열어 확인.
도로 위치, 차선 가시성, 인접 카메라 시야 겹침을 점검할 수 있음.

"""
import argparse
import os
import time

import carla
import yaml


def spawn_camera(world, bp_lib, fov, image_size, transform):
    cam_bp = bp_lib.find('sensor.camera.rgb')
    cam_bp.set_attribute('image_size_x', str(image_size[0]))
    cam_bp.set_attribute('image_size_y', str(image_size[1]))
    cam_bp.set_attribute('fov', str(fov))
    return world.spawn_actor(cam_bp, transform)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', default='config/cameras.yaml')
    parser.add_argument('--out', default='camera_check')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--timeout', type=float, default=5.0,
                        help='카메라당 캡처 대기 timeout(초)')
    args = parser.parse_args()

    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    os.makedirs(args.out, exist_ok=True)

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()

    # 맵이 다르면 로드
    current_map = world.get_map().name.split('/')[-1]
    if cfg.get('map') and cfg['map'] != current_map:
        print(f'{cfg["map"]} 로드 중...')
        world = client.load_world(cfg['map'])
        time.sleep(2.0)

    bp_lib = world.get_blueprint_library()
    fov = cfg.get('fov', 90)
    image_size = cfg.get('image_size', [1920, 1080])

    for cam_cfg in cfg['cameras']:
        cam_id = cam_cfg['id']
        loc = cam_cfg['location']
        rot = cam_cfg['rotation']

        transform = carla.Transform(
            carla.Location(x=loc['x'], y=loc['y'], z=loc['z']),
            carla.Rotation(
                pitch=rot['pitch'],
                yaw=rot['yaw'],
                roll=rot.get('roll', 0.0),
            ),
        )

        cam = spawn_camera(world, bp_lib, fov, image_size, transform)
        saved = {'done': False}
        out_path = os.path.join(args.out, f'{cam_id}.png')

        def make_cb(saved_ref, path):
            def cb(image):
                if not saved_ref['done']:
                    image.save_to_disk(path)
                    saved_ref['done'] = True
            return cb

        cam.listen(make_cb(saved, out_path))

        deadline = time.time() + args.timeout
        while not saved['done'] and time.time() < deadline:
            time.sleep(0.1)

        cam.stop()
        cam.destroy()

        if saved['done']:
            print(f'OK  {cam_id} -> {out_path}')
        else:
            print(f'FAIL {cam_id} (timeout, 서버가 tick 중인지 확인하세요)')


if __name__ == '__main__':
    main()
