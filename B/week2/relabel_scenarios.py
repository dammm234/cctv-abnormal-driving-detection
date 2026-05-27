"""
relabel_scenarios.py

시나리오 라벨링 재정렬.

run_scenarios.py 실행 결과 wobble의 강도 ordering이 의도와 거꾸로 나옴을 발견.
원인: lateral displacement가 amplitude × period²에 비례. 우리는 amplitude만 늘리고
period는 줄였기에 결과적으로 "mild" 파라미터가 가장 강한 wobble 생성.

해결:
- wobble_mild ↔ wobble_strong 폴더 swap
- abrupt_slow → abrupt_strong, abrupt_fast → abrupt_mild (이름 일관성)
- 각 폴더 내부 scenario_config.yaml의 name 필드 업데이트
- 각 폴더 내부 ground_truth.jsonl의 scenario 필드 업데이트
- config/scenarios.yaml의 시나리오 정의도 함께 업데이트

사용:
    cd D:\\CARLA_0.9.14\\WindowsNoEditor\\PythonAPI\\strange_drive
    python relabel_scenarios.py
"""
import json
import os
import statistics

import yaml


SCENARIOS_DIR = 'data/scenarios'
CONFIG_PATH = 'config/scenarios.yaml'


def update_scenario_config(folder, new_name):
    """폴더 안 scenario_config.yaml의 name 필드 업데이트."""
    p = os.path.join(folder, 'scenario_config.yaml')
    if not os.path.exists(p):
        return
    with open(p, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    cfg['name'] = new_name
    with open(p, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def update_ground_truth(folder, new_name):
    """ground_truth.jsonl의 모든 행에서 scenario 필드 업데이트."""
    p = os.path.join(folder, 'ground_truth.jsonl')
    if not os.path.exists(p):
        return
    with open(p, encoding='utf-8') as f:
        lines = [line for line in f if line.strip()]
    new_lines = []
    for line in lines:
        data = json.loads(line)
        data['scenario'] = new_name
        new_lines.append(json.dumps(data))
    with open(p, 'w', encoding='utf-8') as f:
        f.write('\n'.join(new_lines) + '\n')


def safe_rename(old, new):
    """폴더 안전 rename + 내부 파일 업데이트."""
    old_path = os.path.join(SCENARIOS_DIR, old)
    new_path = os.path.join(SCENARIOS_DIR, new)
    if not os.path.exists(old_path):
        return False, f'{old} 없음 (이미 처리됨?)'
    if os.path.exists(new_path):
        return False, f'{new} 이미 존재'
    os.rename(old_path, new_path)
    update_scenario_config(new_path, new)
    update_ground_truth(new_path, new)
    return True, 'OK'


def update_config_yaml():
    """config/scenarios.yaml에서 wobble params 스왑 + abrupt 이름 변경."""
    if not os.path.exists(CONFIG_PATH):
        print(f'  [WARN] {CONFIG_PATH} 없음, 수동 업데이트 필요')
        return
    with open(CONFIG_PATH, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    scenarios = cfg.get('scenarios', [])

    # wobble: mild와 strong의 params 스왑
    wm = next((s for s in scenarios if s['name'] == 'wobble_mild'), None)
    ws = next((s for s in scenarios if s['name'] == 'wobble_strong'), None)
    if wm and ws:
        wm['params'], ws['params'] = ws['params'], wm['params']
        # description도 스왑
        if 'description' in wm and 'description' in ws:
            wm['description'], ws['description'] = (
                ws['description'], wm['description']
            )
        print('  wobble_mild ↔ wobble_strong: params + description 스왑')

    # abrupt: slow → strong, fast → mild
    for s in scenarios:
        if s['name'] == 'abrupt_slow':
            s['name'] = 'abrupt_strong'
        elif s['name'] == 'abrupt_fast':
            s['name'] = 'abrupt_mild'
    print('  abrupt_slow → abrupt_strong, abrupt_fast → abrupt_mild')

    with open(CONFIG_PATH, 'w', encoding='utf-8') as f:
        yaml.safe_dump(cfg, f, allow_unicode=True, sort_keys=False)


def print_summary():
    """최종 시나리오 폴더의 측정 지표 출력."""
    print(f'  {"시나리오":<22s} {"offset_max":>11s} '
          f'{"offset_std":>11s} {"speed_std":>11s}')
    print('  ' + '-' * 60)
    for name in sorted(os.listdir(SCENARIOS_DIR)):
        path = os.path.join(SCENARIOS_DIR, name)
        if not os.path.isdir(path):
            continue
        gt_path = os.path.join(path, 'ground_truth.jsonl')
        if not os.path.exists(gt_path):
            print(f'  {name:<22s} (no ground_truth)')
            continue
        offsets, speeds = [], []
        with open(gt_path, encoding='utf-8') as f:
            for line in f:
                if not line.strip():
                    continue
                d = json.loads(line)
                offsets.append(abs(d['vehicles'][0]['lateral_offset']))
                speeds.append(d['vehicles'][0]['speed_kmh'])
        max_off = max(offsets) if offsets else 0
        off_std = statistics.stdev(offsets) if len(offsets) > 1 else 0
        spd_std = statistics.stdev(speeds) if len(speeds) > 1 else 0
        print(f'  {name:<22s} {max_off:>9.2f}m {off_std:>9.2f}m '
              f'{spd_std:>9.2f} km/h')


def main():
    print('=' * 70)
    print('시나리오 재라벨링')
    print('=' * 70)
    print()

    if not os.path.exists(SCENARIOS_DIR):
        print(f'[ERROR] {SCENARIOS_DIR} 없음')
        return

    # 1. Wobble swap (mild ↔ strong via _swap_temp)
    print('Wobble 스왑:')
    ok, msg = safe_rename('wobble_mild', '_swap_temp')
    print(f'  wobble_mild → _swap_temp: {msg}')
    ok, msg = safe_rename('wobble_strong', 'wobble_mild')
    print(f'  wobble_strong → wobble_mild: {msg}')
    ok, msg = safe_rename('_swap_temp', 'wobble_strong')
    print(f'  _swap_temp → wobble_strong: {msg}')

    # 2. Abrupt rename
    print()
    print('Abrupt 라벨 변경:')
    ok, msg = safe_rename('abrupt_slow', 'abrupt_strong')
    print(f'  abrupt_slow → abrupt_strong: {msg}')
    ok, msg = safe_rename('abrupt_fast', 'abrupt_mild')
    print(f'  abrupt_fast → abrupt_mild: {msg}')

    # 3. config/scenarios.yaml 업데이트
    print()
    print(f'{CONFIG_PATH} 업데이트:')
    update_config_yaml()

    # 4. 최종 검증
    print()
    print('=' * 70)
    print('최종 시나리오 측정값')
    print('=' * 70)
    print_summary()

    print()
    print('검증 포인트:')
    print('  - wobble: mild < medium < strong 순서로 offset_max 단조 증가')
    print('  - abrupt: mild < medium < strong 순서로 speed_std 단조 증가')


if __name__ == '__main__':
    main()
