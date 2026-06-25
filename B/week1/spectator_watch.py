"""
현재 spectator(자유 카메라) 위치를 주기적으로 출력.

CARLA 창에서 WASD로 비행하면서 좋은 카메라 위치를 찾을 때 사용.
마음에 드는 위치에서 멈추면 터미널에 좌표가 찍히고, 그 값을
cameras.yaml에 입력하면 됨.

"""
import argparse
import time

import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--interval', type=float, default=1.0,
                        help='출력 주기(초)')
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()

    print('현재 spectator 위치를 출력합니다. Ctrl+C로 종료.')
    print('-' * 90)

    try:
        while True:
            t = spectator.get_transform()
            loc = t.location
            rot = t.rotation
            print(
                f'Loc(x={loc.x:8.2f}, y={loc.y:8.2f}, z={loc.z:6.2f})  '
                f'Rot(pitch={rot.pitch:7.2f}, yaw={rot.yaw:7.2f}, '
                f'roll={rot.roll:6.2f})'
            )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print('\n종료.')


if __name__ == '__main__':
    main()
