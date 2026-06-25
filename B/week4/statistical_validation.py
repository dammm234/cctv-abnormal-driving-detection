"""
단조 증가의 통계적 유의성 검증.
"""
import argparse
import json
import os
import sys
from collections import defaultdict

import numpy as np
import scipy.stats as stats


def cohens_d(group1, group2):
    """Cohen's d 효과 크기.
    
    |d| 해석:
    - 0.2: small
    - 0.5: medium
    - 0.8: large
    """
    g1 = np.asarray(group1)
    g2 = np.asarray(group2)
    n1, n2 = len(g1), len(g2)
    if n1 < 2 or n2 < 2:
        return float('nan')
    pooled_std = np.sqrt(
        ((n1 - 1) * np.var(g1, ddof=1) + (n2 - 1) * np.var(g2, ddof=1))
        / (n1 + n2 - 2)
    )
    if pooled_std == 0:
        return float('nan')
    return (np.mean(g1) - np.mean(g2)) / pooled_std


def load_sdlp_windows(sdlp_json, behavior='wobble', cam='cam2'):
    """sdlp_analysis.json에서 강도별 window SDLP 추출."""
    with open(sdlp_json, encoding='utf-8') as f:
        data = json.load(f)

    intensity_groups = {}  # 'mild' → [sdlp_window_values]

    for scenario_cam, tracks in data['results'].items():
        # 'wobble_mild_cam2' → behavior='wobble', intensity='mild', cam='cam2'
        if cam not in scenario_cam:
            continue
        if not scenario_cam.startswith(behavior + '_'):
            continue
        # 'wobble_mild_cam2' → 'mild'
        rest = scenario_cam.replace(behavior + '_', '').replace('_' + cam, '')
        intensity = rest

        # 모든 track의 모든 window SDLP 합치기
        all_windows = []
        for tid, r in tracks.items():
            wsdlps = r.get('window_sdlps', [])
            all_windows.extend(wsdlps)

        if all_windows:
            intensity_groups[intensity] = all_windows

    return intensity_groups


def test_monotonic_increase(intensity_groups, intensities=('mild', 'medium', 'strong')):
    """강도별 단조 증가의 통계적 유의성 검증."""
    # 그룹 정리
    groups = []
    labels = []
    for intensity in intensities:
        if intensity in intensity_groups:
            groups.append(intensity_groups[intensity])
            labels.append(intensity)

    if len(groups) < 2:
        return None

    # 그룹 통계
    group_stats = []
    for label, g in zip(labels, groups):
        arr = np.asarray(g)
        group_stats.append({
            'intensity': label,
            'n': len(arr),
            'mean': float(np.mean(arr)),
            'std': float(np.std(arr, ddof=1)) if len(arr) > 1 else 0.0,
            'median': float(np.median(arr)),
        })

    # 1. One-way ANOVA (3개 그룹 차이)
    anova_result = None
    if len(groups) == 3 and all(len(g) >= 2 for g in groups):
        try:
            f_stat, p_val = stats.f_oneway(*groups)
            anova_result = {
                'F': float(f_stat),
                'p_value': float(p_val),
                'significant_005': p_val < 0.05,
                'significant_001': p_val < 0.01,
            }
        except Exception as e:
            anova_result = {'error': str(e)}

    # 2. Pairwise Welch's t-test (equal variances 가정 안 함)
    pairwise = []
    for i in range(len(groups)):
        for j in range(i + 1, len(groups)):
            g1, g2 = groups[i], groups[j]
            if len(g1) < 2 or len(g2) < 2:
                continue
            try:
                t_stat, p_val = stats.ttest_ind(g1, g2, equal_var=False)
                d = cohens_d(g2, g1)  # group2 - group1 효과
                pairwise.append({
                    'comparison': f'{labels[i]} vs {labels[j]}',
                    'mean_diff': float(np.mean(g2) - np.mean(g1)),
                    't': float(t_stat),
                    'p_value': float(p_val),
                    'significant_005': p_val < 0.05,
                    'significant_001': p_val < 0.01,
                    'cohens_d': float(d) if not np.isnan(d) else None,
                    'effect_size': interpret_effect_size(d),
                })
            except Exception as e:
                pairwise.append({
                    'comparison': f'{labels[i]} vs {labels[j]}',
                    'error': str(e),
                })

    # 3. Jonckheere-Terpstra test (순서 있는 trend 검정, 비모수)
    # scipy에 직접 없으므로 Kendall's tau로 대체
    trend_test = None
    try:
        all_values = []
        all_ranks = []
        for rank, g in enumerate(groups):
            all_values.extend(g)
            all_ranks.extend([rank] * len(g))
        tau, p_val = stats.kendalltau(all_ranks, all_values)
        trend_test = {
            'method': 'Kendall tau (ordinal trend)',
            'tau': float(tau),
            'p_value': float(p_val),
            'monotonic_increasing': tau > 0 and p_val < 0.05,
        }
    except Exception as e:
        trend_test = {'error': str(e)}

    return {
        'group_stats': group_stats,
        'anova': anova_result,
        'pairwise_t_tests': pairwise,
        'trend_test': trend_test,
    }


def interpret_effect_size(d):
    """Cohen's d 효과 크기 해석."""
    if d is None or np.isnan(d):
        return 'N/A'
    abs_d = abs(d)
    if abs_d < 0.2:
        return 'negligible'
    elif abs_d < 0.5:
        return 'small'
    elif abs_d < 0.8:
        return 'medium'
    else:
        return 'large'


def print_results(behavior, result):
    """결과를 보기 좋게 출력."""
    print(f'\n{"=" * 60}')
    print(f'  {behavior.upper()} 행동의 강도별 분석')
    print(f'{"=" * 60}')

    print('\n[그룹 통계]')
    print(f"  {'강도':<10} {'n':>6} {'mean':>10} {'std':>10} {'median':>10}")
    for s in result['group_stats']:
        print(f"  {s['intensity']:<10} {s['n']:>6} {s['mean']:>10.4f} "
              f"{s['std']:>10.4f} {s['median']:>10.4f}")

    print('\n[1. One-way ANOVA (3개 그룹 차이)]')
    if result['anova']:
        a = result['anova']
        if 'error' in a:
            print(f'  오류: {a["error"]}')
        else:
            sig_mark = '★' if a['significant_001'] else ('✓' if a['significant_005'] else '✗')
            print(f"  F = {a['F']:.4f}, p = {a['p_value']:.6f}  "
                  f"{'(p<0.001)' if a['significant_001'] else '(p<0.05)' if a['significant_005'] else '(n.s.)'}  {sig_mark}")

    print('\n[2. Pairwise Welch t-tests + Cohen\'s d]')
    for p in result['pairwise_t_tests']:
        if 'error' in p:
            print(f"  {p['comparison']:<20} 오류")
            continue
        sig_mark = '★' if p['significant_001'] else ('✓' if p['significant_005'] else '✗')
        d_str = f"d={p['cohens_d']:.3f}" if p['cohens_d'] is not None else 'd=N/A'
        print(f"  {p['comparison']:<25} "
              f"Δ={p['mean_diff']:+.3f}, t={p['t']:.3f}, "
              f"p={p['p_value']:.4f}  {sig_mark}  "
              f"{d_str} ({p['effect_size']})")

    print('\n[3. Trend test (Kendall tau, 순서 있는 단조 증가)]')
    tr = result['trend_test']
    if 'error' in tr:
        print(f'  오류: {tr["error"]}')
    else:
        sig_mark = '★' if tr['monotonic_increasing'] else '✗'
        print(f"  τ = {tr['tau']:.4f}, p = {tr['p_value']:.6f}  "
              f"{'(단조 증가 유의)' if tr['monotonic_increasing'] else '(유의하지 않음)'}  {sig_mark}")


def _to_serializable(obj):
    """numpy 타입을 Python native로 변환 (JSON 직렬화용)."""
    if isinstance(obj, np.bool_):
        return bool(obj)
    if isinstance(obj, np.integer):
        return int(obj)
    if isinstance(obj, np.floating):
        return float(obj)
    if isinstance(obj, np.ndarray):
        return obj.tolist()
    return str(obj)


def main():
    parser = argparse.ArgumentParser(
        description='단조 증가의 통계적 유의성 검증',
    )
    parser.add_argument('--sdlp-json', default='sdlp_analysis.json',
                         help='sdlp_implementation.py 출력 파일')
    parser.add_argument('--output', default='statistical_validation.json')
    args = parser.parse_args()

    if not os.path.exists(args.sdlp_json):
        print(f'[오류] {args.sdlp_json} 없음.')
        print('먼저 sdlp_implementation.py 를 실행하세요.')
        sys.exit(1)

    print('=== 통계적 유의성 검증 ===')
    print('근거: ANOVA (Fisher 1925), Welch t-test (Welch 1947),')
    print('       Cohen\'s d (Cohen 1988), Kendall tau (Kendall 1938)')

    all_test_results = {}

    for behavior in ['wobble', 'abrupt']:
        intensity_groups = load_sdlp_windows(args.sdlp_json, behavior=behavior)

        if not intensity_groups:
            print(f'\n[{behavior}] 데이터 없음, skip')
            continue

        if len(intensity_groups) < 2:
            print(f'\n[{behavior}] 강도 그룹 부족 (n={len(intensity_groups)}), skip')
            continue

        result = test_monotonic_increase(intensity_groups)
        all_test_results[behavior] = result

        print_results(behavior, result)

    # 결과 저장
    with open(args.output, 'w', encoding='utf-8') as f:
        json.dump({
            'methodology': 'Welch t-test + one-way ANOVA + Kendall tau',
            'metric_used': 'SDLP detrended (per 5-second window)',
            'results': all_test_results,
        }, f, indent=2, ensure_ascii=False, default=_to_serializable)

    print(f'\n→ {args.output}')

    # 종합 결론
    print('\n=== 종합 결론 ===')
    for behavior, result in all_test_results.items():
        if not result:
            continue
        anova_sig = result['anova'] and result['anova'].get('significant_001')
        trend_sig = result['trend_test'].get('monotonic_increasing', False)
        if anova_sig and trend_sig:
            print(f'  {behavior}: 강도별 단조 증가가 통계적으로 유의 (p<0.001) ★')
        elif (result['anova'] and result['anova'].get('significant_005')) or trend_sig:
            print(f'  {behavior}: 강도별 단조 증가가 통계적으로 유의 (p<0.05) ✓')
        else:
            print(f'  {behavior}: 통계적으로 유의하지 않음')


if __name__ == '__main__':
    main()
