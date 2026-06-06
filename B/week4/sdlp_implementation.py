"""
SDLP (Standard Deviation of Lateral Position) 정식 구현.

근거: Verster, J. C., & Roth, T. (2011). Standard operation procedures for
conducting the on-the-road driving test, and measurement of the standard
deviation of lateral position (SDLP). International Journal of General Medicine.

SDLP는 음주 운전 평가에서 가장 검증된 단일 메트릭. 
혈중알코올농도 0.05% 음주자는 SDLP가 평균 0.4m 증가 [Verster & Roth 2011].

본 모듈은:
1. 정식 SDLP 계산 (detrending 포함)
2. 시간 윈도우별 SDLP (sliding window)
3. 기존 lateral_offset_max와 비교
"""
import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np
import scipy.signal as signal


def compute_sdlp_raw(lateral_positions):
    """Raw SDLP: lateral position의 표준편차.

    Args:
        lateral_positions: 1D array of lateral offset values (meters)

    Returns:
        sdlp: standard deviation of lateral position (meters)
    """
    if len(lateral_positions) < 2:
        return 0.0
    return float(np.std(lateral_positions, ddof=1))


def compute_sdlp_detrended(lateral_positions, fs=20.0, cutoff_hz=0.1):
    """Detrended SDLP: 저주파 trend 제거 후 std (Verster & Roth 2011).

    저주파 trend (예: 천천히 곡선 도로를 따르는 일반 주행 패턴)는
    이상 운전 신호가 아님. 이를 제거해야 SDLP가 실제 떨림만 반영.

    Args:
        lateral_positions: 1D array, 차선 중심 대비 lateral offset (m)
        fs: sampling rate (Hz). CARLA 20fps 기본.
        cutoff_hz: high-pass cutoff (낮을수록 더 많은 신호 보존)

    Returns:
        sdlp: detrended std (m)
    """
    if len(lateral_positions) < 10:
        return compute_sdlp_raw(lateral_positions)

    arr = np.asarray(lateral_positions, dtype=float)

    # Butterworth high-pass filter
    try:
        nyq = fs / 2.0
        normalized_cutoff = cutoff_hz / nyq
        if normalized_cutoff >= 1.0:
            normalized_cutoff = 0.99
        b, a = signal.butter(4, normalized_cutoff, btype='high')
        # filtfilt for zero phase shift
        detrended = signal.filtfilt(b, a, arr)
    except Exception:
        # filter 실패 시 mean 제거로 fallback
        detrended = arr - np.mean(arr)

    return float(np.std(detrended, ddof=1))


def compute_sdlp_windowed(lateral_positions, window_size, step_size,
                           fs=20.0, detrend=True):
    """Sliding window SDLP.

    Args:
        lateral_positions: 1D array
        window_size: window size (samples)
        step_size: step between windows (samples)
        fs: sampling rate (Hz)
        detrend: detrended SDLP 사용 여부

    Returns:
        window_sdlps: list of SDLP values (per window)
        window_centers: list of window center times (seconds)
    """
    arr = np.asarray(lateral_positions, dtype=float)
    n = len(arr)
    window_sdlps = []
    window_centers = []

    for start in range(0, n - window_size + 1, step_size):
        window = arr[start:start + window_size]
        if detrend:
            sdlp = compute_sdlp_detrended(window, fs=fs)
        else:
            sdlp = compute_sdlp_raw(window)
        window_sdlps.append(sdlp)
        center_time = (start + window_size / 2) / fs
        window_centers.append(center_time)

    return window_sdlps, window_centers


def extract_lateral_positions_from_json(json_path):
    """v1.1 JSON에서 각 차량의 lateral_offset 시계열 추출.

    Returns:
        {track_id: array of lateral_offset_m}
    """
    with open(json_path, encoding='utf-8') as f:
        data = json.load(f)

    track_offsets = defaultdict(list)
    track_frames = defaultdict(list)

    for frame in data['frames']:
        if not frame.get('vehicles'):
            continue
        fid = frame['frame_id']
        for v in frame['vehicles']:
            tid = v['track_id']
            offset = v.get('lateral_offset_m')
            if offset is None:
                continue
            track_offsets[tid].append(float(offset))
            track_frames[tid].append(fid)

    # frame 순서대로 정렬
    sorted_tracks = {}
    for tid in track_offsets:
        order = np.argsort(track_frames[tid])
        sorted_tracks[tid] = np.asarray(track_offsets[tid])[order]

    return sorted_tracks, data.get('fps', 20.0)


def analyze_scenario(json_path, window_seconds=5.0, step_seconds=1.0):
    """1개 시나리오 (1개 JSON)에 대해 SDLP 분석."""
    track_offsets, fps = extract_lateral_positions_from_json(json_path)

    window_size = int(window_seconds * fps)
    step_size = max(1, int(step_seconds * fps))

    results = {}
    for tid, offsets in track_offsets.items():
        if len(offsets) < window_size:
            # 윈도우보다 짧으면 raw SDLP만
            results[tid] = {
                'n_samples': len(offsets),
                'sdlp_raw': compute_sdlp_raw(offsets),
                'sdlp_detrended': compute_sdlp_detrended(offsets, fs=fps),
                'lateral_offset_max': float(np.max(np.abs(offsets))),
                'lateral_offset_mean': float(np.mean(offsets)),
                'window_sdlps': [],
            }
        else:
            window_sdlps, _ = compute_sdlp_windowed(
                offsets, window_size, step_size, fs=fps, detrend=True,
            )
            results[tid] = {
                'n_samples': len(offsets),
                'sdlp_raw': compute_sdlp_raw(offsets),
                'sdlp_detrended': compute_sdlp_detrended(offsets, fs=fps),
                'lateral_offset_max': float(np.max(np.abs(offsets))),
                'lateral_offset_mean': float(np.mean(offsets)),
                'window_sdlps': window_sdlps,
                'sdlp_window_mean': float(np.mean(window_sdlps)),
                'sdlp_window_max': float(np.max(window_sdlps)),
            }

    return results


def main():
    parser = argparse.ArgumentParser(
        description='SDLP 정식 구현 (Verster & Roth 2011)',
    )
    parser.add_argument('--json-dir', default='scenarios_v1.1')
    parser.add_argument('--output', default='sdlp_analysis.json')
    parser.add_argument('--window-seconds', type=float, default=5.0,
                         help='SDLP 윈도우 크기 (초)')
    parser.add_argument('--step-seconds', type=float, default=1.0,
                         help='윈도우 간 step (초)')
    args = parser.parse_args()

    if not os.path.isdir(args.json_dir):
        print(f'[오류] JSON 폴더 없음: {args.json_dir}')
        sys.exit(1)

    print('=== SDLP 정식 분석 ===')
    print(f'(Verster & Roth 2011 정의)')
    print(f'윈도우: {args.window_seconds}초, step: {args.step_seconds}초\n')

    json_files = sorted([f for f in os.listdir(args.json_dir)
                          if f.endswith('.json')])

    all_results = {}
    summary_rows = []

    for fname in json_files:
        scenario_cam = fname.replace('.json', '')
        json_path = os.path.join(args.json_dir, fname)

        results = analyze_scenario(
            json_path,
            window_seconds=args.window_seconds,
            step_seconds=args.step_seconds,
        )
        all_results[scenario_cam] = results

        if results:
            for tid, r in results.items():
                summary_rows.append({
                    'scenario_cam': scenario_cam,
                    'track_id': tid,
                    'n_samples': r['n_samples'],
                    'sdlp_raw': r['sdlp_raw'],
                    'sdlp_detrended': r['sdlp_detrended'],
                    'lateral_offset_max': r['lateral_offset_max'],
                    'sdlp_window_mean': r.get('sdlp_window_mean'),
                    'sdlp_window_max': r.get('sdlp_window_max'),
                })

    # 출력
    print(f"{'시나리오':<30} {'n':>5} {'SDLP raw':>10} {'SDLP detr':>10} "
          f"{'max|y|':>8} {'win mean':>10} {'win max':>10}")
    print('-' * 90)
    for r in summary_rows:
        win_mean = r['sdlp_window_mean']
        win_max = r['sdlp_window_max']
        win_mean_s = f'{win_mean:.3f}' if win_mean is not None else '-'
        win_max_s = f'{win_max:.3f}' if win_max is not None else '-'
        print(f"{r['scenario_cam']:<30} {r['n_samples']:>5} "
              f"{r['sdlp_raw']:>10.3f} {r['sdlp_detrended']:>10.3f} "
              f"{r['lateral_offset_max']:>8.3f} "
              f"{win_mean_s:>10} {win_max_s:>10}")

    # JSON 저장
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump({
            'method': 'Verster & Roth 2011 SDLP',
            'window_seconds': args.window_seconds,
            'step_seconds': args.step_seconds,
            'detrended_cutoff_hz': 0.1,
            'results': all_results,
            'summary': summary_rows,
        }, f, indent=2, ensure_ascii=False)

    print(f'\n→ {args.output}')

    # 강도별 SDLP 비교 (cam2 기준)
    print('\n=== 강도별 단조 증가 검증 (cam2 SDLP detrended) ===')
    intensity_groups = {
        'wobble': ['wobble_mild', 'wobble_medium', 'wobble_strong'],
        'abrupt': ['abrupt_mild', 'abrupt_medium', 'abrupt_strong'],
    }
    for behavior, scenarios in intensity_groups.items():
        print(f'\n{behavior}:')
        for scenario in scenarios:
            key = f'{scenario}_cam2'
            if key in all_results:
                tracks = all_results[key]
                if tracks:
                    first_track = list(tracks.values())[0]
                    print(f'  {scenario:<20} SDLP_detrended = '
                          f'{first_track["sdlp_detrended"]:.3f}m, '
                          f'max|y| = {first_track["lateral_offset_max"]:.3f}m')


if __name__ == '__main__':
    main()
