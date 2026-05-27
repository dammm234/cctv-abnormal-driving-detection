"""
CARLA Synchronous mode 검증 스크립트.

검증 항목:
  1. sync mode 활성화 후 안정적으로 동작하는가
  2. world.tick() 1회당 카메라 프레임이 정확히 1개 오는가
  3. 종료 시 sync mode가 정상적으로 해제되는가

사용:
  1. CARLA 서버를 띄움 (CarlaUE4.exe)
  2. 별도 터미널에서 이 스크립트 실행:
       conda activate carla37
       python test_sync_mode.py

  결과 PNG는 sync_test_frames/ 폴더에 저장됨.

주의:
  스크립트가 중간에 실패해도 finally 블록이 sync mode를 해제해줌.
  그래도 만약 서버가 hang됐다면 CARLA 서버 재시작이 안전함.
"""
import os
import queue
import sys

import carla


N_TICKS = 10
FIXED_DELTA = 0.05  # 20 fps (1초당 20 tick)
TIMEOUT_PER_TICK = 2.0
OUT_DIR = 'sync_test_frames'


def main():
    print('=' * 60)
    print('CARLA Synchronous Mode 검증')
    print('=' * 60)

    # 서버 연결
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    print(f'서버 버전: {client.get_server_version()}')
    print(f'클라이언트 버전: {client.get_client_version()}')

    world = client.get_world()
    current_map = world.get_map().name.split('/')[-1]
    print(f'현재 맵: {current_map}')

    # 원본 설정 백업 (나중에 복원)
    original_settings = world.get_settings()
    was_sync = original_settings.synchronous_mode
    print(f'원본 sync mode: {was_sync}')

    os.makedirs(OUT_DIR, exist_ok=True)

    image_queue = queue.Queue()
    camera = None

    try:
        # Sync mode 활성화
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = FIXED_DELTA
        world.apply_settings(settings)
        print(f'\nSync mode ON (fixed_delta_seconds={FIXED_DELTA})')

        # Spectator 위치에 카메라 spawn (현재 화면에서 보고 있는 곳)
        spectator = world.get_spectator()
        cam_transform = spectator.get_transform()

        bp_lib = world.get_blueprint_library()
        cam_bp = bp_lib.find('sensor.camera.rgb')
        cam_bp.set_attribute('image_size_x', '1280')
        cam_bp.set_attribute('image_size_y', '720')
        cam_bp.set_attribute('fov', '90')

        camera = world.spawn_actor(cam_bp, cam_transform)
        camera.listen(image_queue.put)
        print(f'카메라 spawn at {cam_transform.location}')
        print()

        # N번 tick, 매번 큐에서 프레임 1개씩 받음
        print(f'{N_TICKS}번 tick:')
        success_count = 0
        carla_frames = []

        for i in range(N_TICKS):
            world.tick()
            try:
                image = image_queue.get(timeout=TIMEOUT_PER_TICK)
                out_path = os.path.join(OUT_DIR, f'frame_{i:02d}.png')
                image.save_to_disk(out_path)
                carla_frames.append(image.frame)
                print(f'  Tick {i:2d}: carla_frame={image.frame}, '
                      f'timestamp={image.timestamp:.3f}s')
                success_count += 1
            except queue.Empty:
                print(f'  Tick {i:2d}: TIMEOUT (no frame in {TIMEOUT_PER_TICK}s)')

        # 결과 요약
        print()
        print('=' * 60)
        print('결과')
        print('=' * 60)
        print(f'Ticks 호출:    {N_TICKS}')
        print(f'Frames 수신:   {success_count}')

        all_pass = True

        if success_count == N_TICKS:
            print('[OK] 모든 tick에 대해 프레임이 수신됨')
        else:
            print('[FAIL] tick과 frame 수가 불일치')
            all_pass = False

        # CARLA frame 번호가 sequential인지 (= 같은 카메라가 매 tick 정확히 1프레임)
        if len(carla_frames) >= 2:
            diffs = [carla_frames[i+1] - carla_frames[i]
                     for i in range(len(carla_frames)-1)]
            if all(d == 1 for d in diffs):
                print('[OK] carla_frame이 1씩 증가 (tick:frame = 1:1)')
            else:
                print(f'[WARN] carla_frame 증분이 일정하지 않음: {diffs}')
                print('       sync mode가 완벽히 동작하지 않거나 다른 클라이언트가 tick 중일 수 있음')
                all_pass = False

        # 파일 검증
        saved_files = sorted(os.listdir(OUT_DIR))
        print(f'저장된 파일 수: {len(saved_files)}')

        print()
        if all_pass:
            print('>>> Sync mode 검증 통과. 2주차 진행 가능. <<<')
        else:
            print('>>> 점검 필요. 위 경고 메시지 확인. <<<')

    except Exception as e:
        print(f'\n[ERROR] {type(e).__name__}: {e}')

    finally:
        # 카메라 정리
        if camera is not None:
            try:
                camera.stop()
                camera.destroy()
                print('\n카메라 정리 완료')
            except Exception as e:
                print(f'카메라 정리 실패: {e}')

        # CRITICAL: sync mode 복원 (안 하면 서버 hang)
        try:
            world.apply_settings(original_settings)
            print('Sync mode 원본 설정 복원')
        except Exception as e:
            print(f'[ERROR] 설정 복원 실패: {e}')
            print('CARLA 서버 재시작을 권장합니다.')


if __name__ == '__main__':
    main()
