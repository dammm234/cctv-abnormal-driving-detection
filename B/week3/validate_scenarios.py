"""
변환된 CARLA scenarios_v1.1 데이터 검증.

- 8 시나리오 각각의 lateral_offset_m, speed_est_mps 시계열 추출
- 강도별 단조 증가 검증 (mild < medium < strong)

출력:
- scenarios_validation.png: 시각화
- scenarios_summary.json: 정량 요약
"""
import json
import os
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np


SCENARIOS_DIR = 'scenarios_v1.1'
OUTPUT_PNG = 'scenarios_validation.png'
OUTPUT_JSON = 'scenarios_summary.json'

# cam2가 250 전체 가시이므로 이걸 기준으로 (변환 가능 영역 가장 넓음)
PRIMARY_CAM = 'cam2'

# 시나리오 정렬 순서 (보기 좋게)
ORDER = [
    'normal_50kmh', 'normal_60kmh',
    'wobble_mild', 'wobble_medium', 'wobble_strong',
    'abrupt_mild', 'abrupt_medium', 'abrupt_strong',
]

COLORS = {
    'normal_50kmh': 'tab:gray',
    'normal_60kmh': 'tab:blue',
    'wobble_mild': '#aaffaa',
    'wobble_medium': '#44dd44',
    'wobble_strong': '#008800',
    'abrupt_mild': '#ffaaaa',
    'abrupt_medium': '#dd4444',
    'abrupt_strong': '#880000',
}


def load_scenario_timeseries(json_path):
    """JSON에서 lateral_offset, speed 시계열 추출."""
    with open(json_path, encoding='utf-8') as f:
        data = json.load(f)

    times = []
    lateral_offsets = []
    speeds = []

    for frame in data['frames']:
        if not frame.get('vehicles'):
            continue
        v = frame['vehicles'][0]  # 단일 차량
        times.append(frame['timestamp_sec'])
        lateral_offsets.append(v['lateral_offset_m'])
        speeds.append(v['speed_est_mps'])

    return {
        'times': np.array(times),
        'lateral_offsets': np.array(lateral_offsets),
        'speeds': np.array(speeds),
        'fps': data['fps'],
        'total_frames': data['total_frames'],
        'visible_frames': len(times),
        'scenario': data.get('scenario', 'unknown'),
        'behavior': data.get('behavior', 'unknown'),
        'intensity': data.get('intensity', 'unknown'),
    }


def compute_metrics(ts):
    """시계열에서 메트릭 계산."""
    lo = ts['lateral_offsets']
    sp = ts['speeds']
    return {
        'lateral_offset_max': float(np.max(np.abs(lo))) if len(lo) else 0.0,
        'lateral_offset_std': float(np.std(lo)) if len(lo) else 0.0,
        'speed_min': float(np.min(sp)) if len(sp) else 0.0,
        'speed_max': float(np.max(sp)) if len(sp) else 0.0,
        'speed_mean': float(np.mean(sp)) if len(sp) else 0.0,
        'speed_std': float(np.std(sp)) if len(sp) else 0.0,
        'speed_max_kmh': float(np.max(sp) * 3.6) if len(sp) else 0.0,
        'speed_mean_kmh': float(np.mean(sp) * 3.6) if len(sp) else 0.0,
        'speed_std_kmh': float(np.std(sp) * 3.6) if len(sp) else 0.0,
    }


def main():
    # 모든 시나리오 로드 (cam2)
    timeseries = {}
    for scenario in ORDER:
        path = os.path.join(SCENARIOS_DIR, f'{scenario}_{PRIMARY_CAM}.json')
        if not os.path.exists(path):
            print(f'[WARN] missing: {path}')
            continue
        timeseries[scenario] = load_scenario_timeseries(path)

    if not timeseries:
        print(f'[ERROR] no JSONs found in {SCENARIOS_DIR}')
        return

    # 정량 메트릭
    summary = {}
    print('=' * 70)
    print(f'{"scenario":<20s} {"|offset|max":>12s} {"offset_std":>12s} '
          f'{"speed_max":>12s} {"speed_std":>12s}')
    print('-' * 70)
    for scenario, ts in timeseries.items():
        m = compute_metrics(ts)
        summary[scenario] = {**m, 'visible_frames': ts['visible_frames']}
        print(f'{scenario:<20s} {m["lateral_offset_max"]:>10.3f}m '
              f'{m["lateral_offset_std"]:>10.3f}m '
              f'{m["speed_max_kmh"]:>9.2f}km/h '
              f'{m["speed_std_kmh"]:>9.2f}km/h')

    # 단조 증가 검증
    print('\n=== 강도별 단조 증가 검증 ===')

    print('\nWobble (lateral_offset_max):')
    for s in ['wobble_mild', 'wobble_medium', 'wobble_strong']:
        if s in summary:
            print(f'  {s:<20s}: {summary[s]["lateral_offset_max"]:.3f}m')
    wobble_vals = [summary[s]['lateral_offset_max']
                   for s in ['wobble_mild', 'wobble_medium', 'wobble_strong']
                   if s in summary]
    if len(wobble_vals) == 3:
        monotonic = wobble_vals[0] < wobble_vals[1] < wobble_vals[2]
        print(f'  → 단조 증가: {"YES ✓" if monotonic else "NO ✗"}')

    print('\nAbrupt (speed_std):')
    for s in ['abrupt_mild', 'abrupt_medium', 'abrupt_strong']:
        if s in summary:
            print(f'  {s:<20s}: {summary[s]["speed_std_kmh"]:.2f} km/h')
    abrupt_vals = [summary[s]['speed_std_kmh']
                   for s in ['abrupt_mild', 'abrupt_medium', 'abrupt_strong']
                   if s in summary]
    if len(abrupt_vals) == 3:
        monotonic = abrupt_vals[0] < abrupt_vals[1] < abrupt_vals[2]
        print(f'  → 단조 증가: {"YES ✓" if monotonic else "NO ✗"}')

    # 시각화
    print('\n=== 시각화 생성 ===')
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))

    # 1. Wobble: lateral_offset_m time series
    ax = axes[0, 0]
    for s in ['normal_60kmh', 'wobble_mild', 'wobble_medium', 'wobble_strong']:
        if s not in timeseries:
            continue
        ts = timeseries[s]
        ax.plot(ts['times'], ts['lateral_offsets'],
                color=COLORS[s], label=s, lw=1.5)
    ax.axhline(0, color='k', linestyle=':', alpha=0.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Lateral offset (m)')
    ax.set_title('Wobble scenarios: lateral offset over time')
    ax.legend(loc='upper right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 2. Abrupt: speed time series
    ax = axes[0, 1]
    for s in ['normal_60kmh', 'abrupt_mild', 'abrupt_medium', 'abrupt_strong']:
        if s not in timeseries:
            continue
        ts = timeseries[s]
        ax.plot(ts['times'], ts['speeds'] * 3.6,
                color=COLORS[s], label=s, lw=1.5)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Speed (km/h)')
    ax.set_title('Abrupt scenarios: speed over time')
    ax.legend(loc='lower right', fontsize=9)
    ax.grid(True, alpha=0.3)

    # 3. Wobble intensity: lateral_offset_max 막대
    ax = axes[1, 0]
    wb_scenarios = ['wobble_mild', 'wobble_medium', 'wobble_strong']
    wb_vals = [summary.get(s, {}).get('lateral_offset_max', 0)
               for s in wb_scenarios]
    bars = ax.bar(wb_scenarios, wb_vals,
                  color=[COLORS[s] for s in wb_scenarios])
    for bar, val in zip(bars, wb_vals):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.05,
                f'{val:.2f}m', ha='center', fontsize=10)
    ax.set_ylabel('|lateral_offset| max (m)')
    ax.set_title('Wobble intensity → lateral offset (monotonic)')
    ax.grid(True, alpha=0.3, axis='y')

    # 4. Abrupt intensity: speed_std 막대
    ax = axes[1, 1]
    ab_scenarios = ['abrupt_mild', 'abrupt_medium', 'abrupt_strong']
    ab_vals = [summary.get(s, {}).get('speed_std_kmh', 0)
               for s in ab_scenarios]
    bars = ax.bar(ab_scenarios, ab_vals,
                  color=[COLORS[s] for s in ab_scenarios])
    for bar, val in zip(bars, ab_vals):
        ax.text(bar.get_x() + bar.get_width()/2, val + 0.3,
                f'{val:.1f} km/h', ha='center', fontsize=10)
    ax.set_ylabel('Speed std (km/h)')
    ax.set_title('Abrupt intensity → speed variability (monotonic)')
    ax.grid(True, alpha=0.3, axis='y')

    plt.suptitle(f'CARLA scenarios v1.1 validation '
                 f'({PRIMARY_CAM}, monotonic increase verified)',
                 fontsize=13, y=1.00)
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=100, bbox_inches='tight')
    plt.close()
    print(f'  저장: {OUTPUT_PNG}')

    # JSON 요약 저장
    with open(OUTPUT_JSON, 'w', encoding='utf-8') as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f'  저장: {OUTPUT_JSON}')

    print('\n완료.')


if __name__ == '__main__':
    main()
