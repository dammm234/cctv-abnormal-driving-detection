# Week 2 — 시나리오 데이터셋 + Trajectory 클러스터링

## 작업 요약

1. **Sync mode 검증** — 다중 카메라 동기 녹화 인프라
2. **Step A~D** — 점진적으로 데이터 생성 파이프라인 구축 (단일→3카메라→행동제어→배치)
3. **시나리오 8개 양산** — 정상 2 + wobble 3강도 + abrupt 3강도
4. **라벨링 정합성** — wobble 강도 역전 발견 후 자동 재정렬 도구로 해결
5. **Trajectory 클러스터링 모듈** — Modified Hausdorff + DBSCAN (UFLDv2 대안)

## 파일

### 코드

| 파일 | 설명 |
|---|---|
| `test_sync_mode.py` | CARLA synchronous mode 안정성 검증 |
| `step_a_record.py` | Step A — 단일 카메라 + 차량 1대 검증 |
| `step_b_record.py` | Step B — 3카메라 동기 녹화 검증 |
| `step_c_record.py` | Step C — 차량 행동 제어 (wobble/abrupt/normal) |
| `run_scenarios.py` | yaml 기반 배치 실행기 (8 시나리오 자동 생성) |
| `relabel_scenarios.py` | 라벨 정합성 도구 (wobble 강도 재정렬) |
| `trajectory_clustering.py` | Modified Hausdorff + DBSCAN 클러스터링 |
| `scenarios.yaml` | 8개 시나리오 정의 (행동×강도) |

### 산출물

| 파일 | 설명 |
|---|---|
| `lane_hypotheses.json` | 클러스터링 결과 (Role C로 전달) |
| `trajectory_clustering_viz.png` | 클러스터링 결과 시각화 |
| `week2_report.md` | 2주차 단독 보고서 |
| `midterm_report.md` | 1~2주차 통합 보고서 |

### .gitignore에 포함된 산출물 (로컬에만)

- `data/scenarios/{시나리오}/` — 8 시나리오 × 250 tick × 3 카메라 (총 6,000 PNG, 19GB)
- 각 시나리오 폴더의 `ground_truth.jsonl`, `scenario_config.yaml`

## 8 시나리오 핵심 측정값

| 시나리오 | offset_max | offset_std | speed_std |
|---|---|---|---|
| wobble_mild | 1.10m | 0.32m | 1.97 km/h |
| wobble_medium | 2.32m | 0.51m | 1.98 km/h |
| wobble_strong | 3.02m | 0.76m | 2.07 km/h |
| abrupt_mild | 0.36m | 0.10m | 10.54 km/h |
| abrupt_medium | 0.38m | 0.11m | 16.85 km/h |
| abrupt_strong | 0.40m | 0.12m | 20.52 km/h |
| normal_60kmh | 0.46m | 0.14m | 11.57 km/h |
| normal_50kmh | 0.44m | 0.14m | 12.70 km/h |

- **wobble**: offset_max 단조 증가 (1.10 → 2.32 → 3.02m) ✓
- **abrupt**: speed_std 단조 증가 (10.54 → 16.85 → 20.52 km/h) ✓

## Trajectory 클러스터링 알고리즘

1. 차량 ID별 검출 묶기 → trajectory 생성
2. arc length 기준 균일 리샘플링 (50점)
3. Modified Hausdorff distance 계산 (Dubuisson & Jain, 1994)
4. DBSCAN (eps=2.0m, min_samples=2, precomputed metric)
5. 각 클러스터의 평균 trajectory → 차선 centerline

검증: 8개 CARLA 시나리오 trajectory 입력 → 모두 lane 0으로 정상 클러스터링, noise 0개.

## 핵심 발견 사항

### Wobble 강도는 amplitude × period² 에 의존

첫 batch 결과에서 "mild"가 가장 큰 차선 이탈을 보이는 역전 현상 발견. 이론적 분석 결과 lateral 변위 ∝ amplitude × period². `relabel_scenarios.py`로 자동 재정렬.

### Modified Hausdorff의 trajectory 길이 민감성

abrupt_mild ↔ abrupt_strong이 3.18m로 가장 멈 — 평균 속도 차이로 trajectory 총 길이가 다름 (162m vs 191m). 현 데이터에선 DBSCAN density chain으로 흡수, 향후 실제 CCTV에선 DTW 검토 가능.

## 사용 방법

```bash
conda activate carla37
cd path/to/strange_drive

# 1. 시나리오 양산 (CARLA 서버 실행 필요, ~70분 소요)
python run_scenarios.py

# 2. 라벨 정합성 보정 (선택)
python relabel_scenarios.py

# 3. trajectory 클러스터링 (CARLA 불필요)
python trajectory_clustering.py
```

## 다음 주

`week3/`에서 호모그래피 캘리브레이션 + Role A 통합.
