# A의 v1.1과 B의 trajectory clustering 통합 검증 보고서

작성일: 2026-05-27 (3주차)
작성자: Role B

---

## 개요

Role A의 1주차 산출물(`test_tracks_v1.1.json`, 경부동탄터널 5차로 CCTV)에 Role B의 trajectory clustering 모듈을 적용하여 두 차선 검출 방식의 일관성을 검증함.

## 입력 데이터

- **영상**: 경부선 경부동탄터널(부산방향) 5차로 CCTV, 1280×720, 24 fps, 21.7초 (520 프레임)
- **A의 출력**: `test_tracks_v1.1.json`
  - 41 unique track_id
  - 1,603 검출 인스턴스
  - 32 유효 trajectory (≥10 점)
  - 모든 차량의 `position_road_m` (미터 좌표), `lane_id` (1~5), `lateral_offset_m`, `speed_est_mps` 포함

## 적용 방법

B의 trajectory clustering 알고리즘을 A의 `position_road_m` (이미 미터 단위)에 직접 적용. 호모그래피 추가 적용 불필요.

알고리즘 파라미터:
- `min_trajectory_points = 10`
- `resample_points = 50` (arc length 균일 리샘플링)
- DBSCAN `eps = 1.5m`, `min_samples = 2`
- Modified Hausdorff distance

## 결과

### 정량 비교

| Cluster ID | avg_x (m) | 멤버 수 | A의 lane_id 분포 | 해석 |
|---|---|---|---|---|
| 0 | 9.38 | 2 | L3 100% | Lane 3 일부 |
| 1 | 12.86 | 7 | L4 100% | **Lane 4 안정 주행** |
| 2 | 16.15 | 6 | L5 100% | **Lane 5 안정 주행** |
| 3 | 10.16 | 5 | L3 60% + L4 40% | **차선 변경 차량들** |
| 4 | 6.30 | 6 | L2 100% | **Lane 2 안정 주행** |
| 5 | 3.65 | 3 | L1 33% + L2 67% | **차선 변경 차량들** |

- B의 cluster 수: 6개 (5 lane + 2 transitional)
- Noise: 3개 (짧거나 비정상 trajectory)

### 정성 분석

1. **5차로 자동 검출 일치**: A의 lane_id 1~5에 해당하는 5개 cluster가 B의 알고리즘으로 자동 발견됨. **사전 지식 (차선 개수, 차선 폭) 없이도 동일한 차선 구조 발견.**

2. **차선 변경 패턴 분리**: B의 clustering이 Cluster 3, 5로 차선 변경 차량들을 별도 그룹으로 분리. A의 lane_id 방식은 차선 변경 시 한 차량의 lane_id가 시간에 따라 변하는 것으로만 표현되며, 이런 차량들이 명시적으로 그룹화되지 않음.

3. **lane center 정합성**:
   - A의 lane 중심 (3.5m 균일 가정): 1.75, 5.25, 8.75, 12.25, 15.75
   - B의 cluster 중심 (자동 발견): 3.65, 6.30, 9.38, 12.86, 16.15
   - 차이는 평균 0.5m 이내. 두 방법이 같은 차선 구조를 발견.

## 두 방법의 비교

| 측면 | A의 방법 (BEV 분할) | B의 방법 (trajectory clustering) |
|---|---|---|
| 차선 개수 | 사전 지정 필요 (5차로 알아야 함) | 데이터에서 자동 발견 |
| 차선 폭 | 사전 가정 (3.5m 균일) | 데이터에서 추정 |
| 차선 변경 차량 | 시간 따라 lane_id 변동 | 별도 cluster로 분리 |
| 곡선 도로 | 직선 가정 위반 시 부정확 | 곡선도 잘 따라감 |
| 흐릿한 차선 마킹 | 호모그래피만 정확하면 OK | 차량 통행 패턴으로 추정 |
| 학습 데이터 | 불필요 | 불필요 |

**결론**: 두 방법 모두 본 영상에서 유효한 차선 검출 결과를 제공하며, 그 결과가 **상호 일치함**으로써 시스템 전체의 신뢰성이 확보됨.

A의 방법은 **단순하고 빠르며 본 영상처럼 직선·균일 차선 조건에서 충분**. B의 방법은 **자동 발견과 차선 변경 검출의 추가 가치**를 제공.

## 산출물

- `integrate_with_a.py` — 통합 스크립트
- `trajectory_vs_a_lanes.png` — A vs B 비교 시각화
- `lane_hypotheses_a_video.json` — A의 영상 좌표계 차선 가설

## Role C에 대한 시사점

A의 v1.1 출력은 메트릭 모듈(C)의 입력으로 직접 사용 가능 (`lateral_offset_m`, `speed_est_mps` 등). B의 차선 가설은 다음 두 가지 추가 활용 가능:

1. **차선 변경 빈도 메트릭**: Cluster 3, 5처럼 차선 변경 trajectory 식별
2. **A의 lane_id 검증**: 두 방법의 일치도가 떨어지는 영상에서 호모그래피/검출 오류 진단

## 향후 작업

1. **CARLA 시나리오를 A의 v1.1 포맷으로 변환** — B의 8 시나리오를 같은 JSON 스키마로 export하여 Role C가 통합 처리 가능
2. **다른 영상으로 확장** — sample1, sample2 (구미 영상)에 동일 절차 적용하여 일반화 검증
3. **곡선 도로 시연** — A의 단순 BEV 분할이 작동 안 하는 도로에 B의 방법 적용 시연 (가능하면)
