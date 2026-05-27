# 2주차 진행 보고서 — Role B

다중 CCTV 기반 이상 운전 탐지 시스템 / Role B: 좌표계 & 시뮬레이션
작성일: 2026-05-26

---

## 한 줄 요약

CARLA Town06에 3 카메라 + 8개 시뮬레이션 시나리오를 구축하고, 차량 trajectory 클러스터링 모듈을 구현하여 차선 가설 도출 파이프라인 검증을 완료함.

---

## 주요 산출물

| 항목 | 내용 | 검증 결과 |
|---|---|---|
| CARLA 시나리오 데이터셋 | 8개 시나리오 × 250 tick × 3 카메라 | 6,000 PNG, loss 0, 19GB |
| 시나리오 정의 | scenarios.yaml (3종 행동 × 3강도) | wobble, abrupt 강도별 단조 증가 검증 |
| 시나리오 양산 도구 | run_scenarios.py 배치 실행기 | 8/8 성공, 자동 ground truth dump |
| trajectory 클러스터링 모듈 | modified Hausdorff + DBSCAN | 동일 차선 8개 trajectory → 1 클러스터 |
| 실제 CCTV 보조 영상 | 경부선 174km 부근 2개 클립 | ITS.go.kr, 1640×1236, 합계 3분 14초 |

---

## 진행 상세

### 시나리오 데이터셋 생성

목표: 정상/이상 운전 시나리오를 CARLA에서 양산하여 메트릭 학습/평가 데이터 확보.

설계 단계를 5개 Step으로 나누어 점진적으로 검증함. 각 Step은 직전 Step에서 검증된 인프라 위에 새 요소 하나씩만 추가하는 방식. 디버깅 시 변수 격리에 효과적이었음.

| Step | 추가된 요소 | 검증 통과 기준 |
|---|---|---|
| A | 단일 카메라 + 차량 1대 + autopilot | 100/100 PNG + 차량 시야 진입 |
| B | 3 카메라 동시 spawn + 동기 | tick:frame = 1:1, carla_frame 일치 |
| C | 차량 수동 제어 (wobble/abrupt/normal) | 각 행동 패턴이 ground truth에 명확히 관찰됨 |
| D | yaml 기반 시나리오 양산 + 배치 실행 | 8 시나리오 자동 생성 |

시나리오 정의는 행동 3종(정상/비틀거림/급조작) × 강도 3단계(mild/medium/strong) + 정상의 속도 변형(50km/h)으로 총 8개. 각 시나리오는 12.5초간 녹화.

핵심 측정값 (강도별 단조 증가 검증):

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

wobble은 lateral offset이, abrupt는 speed 변동이 강도별 단조 증가. 이 단조성은 추후 메트릭 학습 시 강도별 점수가 일관되게 나오는지 확인하는 ground truth 역할.

### 트라젝토리 클러스터링 모듈

1주차에 UFLDv2 차선 검출의 한국 CCTV 적용 한계를 발견하고, 학술 문헌 조사 후 trajectory 클러스터링으로 방향 전환을 결정했음 (1주차 결정 보고서 참조).

구현 알고리즘:

1. 차량 ID별 검출/관측 묶기 → trajectory 생성
2. arc length 기준 균일 리샘플링 (속도 차이 보정)
3. Modified Hausdorff distance (Dubuisson & Jain, 1994)로 쌍별 거리 계산
4. DBSCAN with precomputed metric으로 클러스터링
5. 각 클러스터의 평균 trajectory를 차선 centerline으로 추출

코드 약 300줄, scikit-learn DBSCAN + numpy 기반 직접 구현 Hausdorff.

CARLA 8개 시나리오 입력 검증 결과:

- 8개 trajectory 모두 동일 클러스터 (lane 0)로 묶임 — 의도대로 (모두 같은 차선 출발)
- noise 0개
- 추출된 centerline: x ∈ [299.8, 476.8], y ∈ [-17.86, -17.19] (spawn y = -17.5 와 일치)
- 시각화: `data/trajectory_clustering_viz.png`

본 모듈은 입력 인터페이스가 두 가지로 분리되어 있음.

- `load_trajectories_from_csv(csv_path)`: A의 YOLO+ByteTrack 출력 (image space, 픽셀)
- `load_trajectories_from_carla(scenarios_dir)`: CARLA ground truth (world space, 미터)

3주차에 A의 출력이 준비되면 입력 함수만 교체하여 동일 알고리즘 적용 가능.

### 실제 CCTV 영상 확보

경부선 174km 부근(구미지사 관할)의 ITS.go.kr 공개 CCTV에서 2개 클립 녹화. 다리 위 elevated 시점과 측면 시점으로 perspective 다양화. 4주차 단일 카메라 데모용으로 보관.

---

## 알아낸 점

### Wobble 강도가 amplitude × period²에 의존

시나리오 정의 초안에서 amplitude는 키우고 period는 줄이는 방식으로 강도 차등화를 의도했으나, 실제 lateral 변위는 amplitude × period²에 비례함을 결과로부터 발견. 첫 양산 결과에서 "mild"가 가장 큰 차선 이탈(3m)을 보이고 "strong"이 가장 작은 변위(1m)를 보이는 역전 현상이 발생.

폴더 이름과 시나리오 정의 파일을 측정된 실제 강도에 맞게 재정렬하여 해결 (`relabel_scenarios.py`). 보고서엔 이 발견을 명시하고, 향후 시나리오 추가 시 period를 고정하고 amplitude로만 강도 조정하는 가이드라인을 따르기로 함.

### Modified Hausdorff의 trajectory 길이 민감성

서로 다른 시나리오는 평균 속도가 달라 trajectory 총 길이가 다름 (예: abrupt_mild 162m, abrupt_strong 191m). Hausdorff 거리는 한 trajectory의 끝부분이 다른 trajectory의 범위를 벗어날 때 큰 거리를 산출함. 우리 데이터에서 abrupt_mild ↔ abrupt_strong이 3.18m로 가장 큰 거리를 보임.

다행히 DBSCAN의 density chain 특성으로 인해 중간 trajectory가 다리 역할을 하여 모두 한 클러스터로 정상 묶임. 그러나 실제 CCTV 데이터에선 차량별 카메라 체류 시간이 다양해 이 효과가 더 두드러질 가능성 있음. 대안으로 DTW(Dynamic Time Warping)나 trim-to-common-range 전처리를 3주차에 검토 예정.

### Sync mode 안정성 검증

다중 카메라 동기 녹화는 본 프로젝트의 평가 신뢰성을 좌우하는 인프라. 200 tick 동안 3 카메라에서 carla_frame이 모두 일치(불일치 0), 프레임 손실 0을 확인. 시나리오 당 ~9분의 wall clock 시간 소요(예상 17초 대비 30배)는 PNG 저장 + 렌더링 I/O 부하로 추정. 1회성 데이터 생성이라 수용 가능하나, 향후 데이터 확장 시 JPEG 인코딩이나 비동기 저장 검토 가능.

---

## 3주차 계획

다음 작업:

1. Role A의 YOLO+ByteTrack 출력 수신 후 trajectory_clustering.py에 연결하여 실제 적용 검증
2. 호모그래피 캘리브레이션 — 카메라 픽셀 좌표 ↔ CARLA world 좌표 변환 매트릭스 구성. 메트릭 계산 시 픽셀 거리를 실거리로 환산할 때 필요
3. Role C의 메트릭 모듈과 통합 — lane_hypotheses.json을 메트릭 입력으로 전달, 차량 trajectory 대비 차선 reference 편차 계산
4. 실제 CCTV 영상에 파이프라인 적용 — Role A가 클립에 YOLO+ByteTrack 적용한 결과로 trajectory 클러스터링 검증

---

## 산출 파일 목록

코드:
- `step_a_record.py` ~ `step_c_record.py` — Step별 검증 스크립트 (보관)
- `run_scenarios.py` — 시나리오 배치 실행기
- `relabel_scenarios.py` — 라벨 정합성 도구
- `trajectory_clustering.py` — 차량 trajectory 클러스터링 모듈
- `test_sync_mode.py` — sync mode 검증 도구

설정:
- `config/cameras.yaml` — 카메라 3개 좌표 정의
- `config/scenarios.yaml` — 시나리오 8개 정의

데이터:
- `data/scenarios/{시나리오}/` — 카메라별 PNG + ground_truth.jsonl + scenario_config.yaml (8개, 19GB)
- `data/lane_hypotheses.json` — Step G 클러스터링 결과
- `data/trajectory_clustering_viz.png` — 클러스터링 시각화
- `data/real/clip01_gumi_view1.mp4`, `clip02_gumi_view2.mp4` — 실제 CCTV 보조 영상

문서:
- `docs/lane_reference_decision.md` — 1주차 결정 보고서 (UFLDv2 → trajectory 클러스터링)
- `docs/week2_report.md` — 본 문서

---

## 참고 문헌

- Melo, J., Naftel, A., Bernardino, A., & Santos-Victor, J. (2006). Detection and classification of highway lanes using vehicle motion trajectories. *IEEE Transactions on Intelligent Transportation Systems*.
- Tang, Z., Naphade, M., Liu, M.-Y., Yang, X., Birchfield, S., Wang, S., et al. (2017). CityFlow: A city-scale benchmark for multi-target multi-camera vehicle tracking and re-identification.
- Dubuisson, M.-P., & Jain, A. K. (1994). A modified Hausdorff distance for object matching. *International Conference on Pattern Recognition*.
- Ester, M., Kriegel, H.-P., Sander, J., & Xu, X. (1996). A density-based algorithm for discovering clusters (DBSCAN). *KDD*.
