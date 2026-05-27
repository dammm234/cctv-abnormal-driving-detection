"""
Town06 (또는 지정한 맵) 로드 스크립트.

사용:
    python load_town06.py
    python load_town06.py --town Town04
"""
import argparse
import time

import carla


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--town', default='Town06', help='로드할 맵 이름')
    parser.add_argument('--host', default='localhost')
    parser.add_argument('--port', type=int, default=2000)
    parser.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)

    print(f'{args.town} 로드 중... (수 초 걸릴 수 있음)')
    world = client.load_world(args.town)
    time.sleep(2.0)

    map_name = world.get_map().name
    spawn_points = world.get_map().get_spawn_points()

    print(f'로드 완료: {map_name}')
    print(f'스폰 포인트: {len(spawn_points)}개')
    print()
    print('이제 CARLA 창에서 WASD + 마우스로 자유롭게 둘러보세요.')
    print('별도 터미널에서 spectator_watch.py를 띄우면 현재 좌표가 출력됩니다.')


if __name__ == '__main__':
    main()
