# 1~2주차 통합 보고서 — Role B

다중 CCTV 기반 이상 운전 탐지 시스템
Role B: 좌표계 & 시뮬레이션
보고 기간: 1주차 ~ 2주차 종료 (2026-05-12 ~ 2026-05-26)

---

## 한 줄 요약

CARLA Town06에 다중 카메라 시뮬레이션 환경을 구축하고 8개 이상/정상 운전 시나리오 데이터셋을 양산함. 1주차에 차선 검출 모듈을 UFLDv2에서 차량 trajectory 클러스터링으로 전환하기로 결정한 후, 2주차에 해당 모듈을 구현하고 검증 완료.

---

## 1. 프로젝트 개요

### 1.1 배경

본 프로젝트는 다중 CCTV에서 차량을 검출·추적하여 운전 행동의 이상성을 정량화하는 시스템 구축을 목표로 함. 음주, 졸음, 난폭 운전 등을 직접 판정하지 않고, 횡방향 비틀거림, jerk, 곡률 변화, 속도 안정성 등의 메트릭을 통해 0~1 점수의 "이상성 점수"를 산출하여 운영자가 최종 판단하는 보조 도구 형태.

3명의 역할 분담:
- Role A: YOLOv11 + ByteTrack 기반 차량 검출 및 추적
- **Role B (본 보고서 담당): 좌표계 설계, 카메라 캘리브레이션, CARLA 시뮬레이션 시나리오 제작**
- Role C: 룰 기반 메트릭 산출, 라벨링 및 평가

### 1.2 Role B 책임 범위

당초 계획상 Role B의 작업은 다음 네 가지:

1. CARLA 시뮬레이션 환경 구축 및 시나리오 데이터셋 생성
2. 차선 검출 모듈 (계획서 초안에선 UFLDv2 활용)
3. BEV (Bird's Eye View) 호모그래피 캘리브레이션
4. Role A의 출력과 Role C의 메트릭을 연결하는 데이터 인터페이스 정의

1주차에 (2)의 방향을 전환하여 차량 trajectory 클러스터링으로 변경했으며, 2주차 끝까지 (1)과 (2)를 완료함. (3)과 (4)는 3주차 통합 단계에서 진행 예정.

---

## 2. 1주차 진행

### 2.1 협업 인터페이스 합의

Role A의 출력 형식을 시나리오별 `detections/cam{0,1,2}.csv`로 합의함:

```
frame_idx, timestamp, vehicle_id, x1, y1, x2, y2, confidence
```

세부 사항: vehicle_id는 카메라별 독립(ReID는 Role B 담당), 검출 실패 프레임은 행 제외, timestamp는 시나리오 시작 기준 상대시간(초). YOLO 검출 클래스는 car / truck / bus / motorcycle 4종으로 한정.

이 인터페이스 합의로 2주차 데이터 생성 단계에서 Role A의 출력과 호환되는 ground truth 포맷을 미리 설계할 수 있었음.

### 2.2 좌표계와 카메라 위치 설계

CARLA의 Town06 맵을 선택. 직선 본선 구간이 가장 길어 다중 카메라 배치에 유리함. CARLA에 Town06이 기본 설치돼 있지 않아 AdditionalMaps_0.9.14.zip을 다운로드 후 7-Zip으로 직접 압축 해제하여 설치함.

세 카메라를 본선 진행 방향에 따라 cam0 → cam1 → cam2 순으로 배치:

| 카메라 | 위치 (x, y, z) | 회전 (pitch, yaw, roll) | 비고 |
|---|---|---|---|
| cam0 | (437.64, -17.54, 13.25) | (-17.72°, -0.28°, 0°) | 가장 앞 (차량 진입 첫 카메라) |
| cam1 | (312.21, -18.09, 11.21) | (-14.44°, 3.49°, 0°) | 중간 |
| cam2 | (219.30, -17.80, 11.14) | (-17.14°, 1.36°, 0°) | 가장 뒤 (마지막 카메라) |

카메라 간격은 cam0→cam1이 125m, cam1→cam2가 93m. y 좌표가 거의 일정(-17.5±0.5)하여 같은 도로 위에 일렬 배치됨이 확인됨. 카메라는 모두 +X 방향을 향하며, 차량은 -X 방향으로 진행하여 카메라 시야에 정면으로 접근하는 구성.

`config/cameras.yaml`에 좌표 정보를 분리 저장하여 코드와 설정의 분리를 확보함.

### 2.3 차선 검출 모듈 시도 (UFLDv2)

계획서 초안의 UFLDv2 (Ultra-Fast-Lane-Detection v2) 차선 검출을 위해 다음 작업 수행:

1. CULane 데이터셋으로 사전 학습된 ResNet18 가중치 다운로드
2. UFLDv2의 다수 의존성 충돌 해결 (tensorboard, six, pathspec, p_tqdm, matplotlib, einops 등)
3. NVIDIA DALI 의존성 우회 패치: `utils/common.py`의 DALI 임포트를 try/except로 감쌈
4. 커스텀 단일 이미지 추론 스크립트 `infer_one.py` 작성

CARLA Town06에서 캡처한 cam2 이미지로 첫 테스트 결과, 차선 위치가 부정확하게 추정됨. 도메인 갭 (CARLA 합성 환경 vs. CULane 실제 영상) 때문으로 추정.

다음 테스트로 한국 고속도로 CCTV 이미지를 적용한 결과, 3개 차선 중 1개(파란색)만 정확하게 검출되고 나머지는 위치가 어긋남. 원인은 perspective gap: UFLDv2의 학습 데이터인 CULane은 차량 dashcam 시점이며, 한국 CCTV는 elevated 시점이라 차선의 vanishing point 위치가 본질적으로 다름.

### 2.4 방향 전환 결정: trajectory 클러스터링

UFLDv2의 학습 데이터를 한국 elevated CCTV에 맞게 fine-tuning하려면 자체 라벨링 데이터셋 구축이 필요하며, 4주 일정 안에는 비현실적임. 대안 검토를 위해 학술 문헌을 조사함:

- Melo et al. (2006): 차량 motion trajectory로 차선 검출 및 분류
- Ren et al. (2014): trajectory 기반 lane geometry 추론
- Tang et al. (2017): CityFlow 다중 카메라 차량 추적 데이터셋
- Qiu et al. (2024): 차량 trajectory 클러스터링 기반 차선 인식

위 문헌들의 공통점: **고정 카메라에서 다수 차량의 trajectory를 누적하여 클러스터링하면 차선 자체를 학습 데이터 없이 추정 가능**.

이 접근은 본 프로젝트의 계획서 4.1 정신 ("차량 자체의 평활화된 궤적을 reference로 사용")과 정합. 또한 Role A의 YOLO+ByteTrack 출력을 그대로 입력으로 사용 가능하여 추가 학습 모델이 필요 없음. CARLA 시뮬레이션과 실제 CCTV 양쪽에 동일 코드가 적용 가능한 것도 장점.

방향 전환을 정식 결정으로 문서화하여 `docs/lane_reference_decision.md`로 보관함. 4주차 발표 시 "문제 발견 → 학술 조사 → 대안 도출 → 검증" 스토리로 풀 수 있는 자료.

### 2.5 실제 CCTV 영상 확보

본 프로젝트는 CARLA 시뮬레이션을 주된 평가 축으로 두고, 실제 영상은 단일 카메라 보조 데모로 사용 (계획서 10장 위험요소 참조: 동일 차량이 다수 한국 CCTV에 연속 통과하는 영상 페어는 공개 데이터에서 거의 없음).

여러 시도 끝에 적합한 영상 2개 확보:

- `clip01_gumi_view1.mp4`: 경부선 174km 부근, 구미지사 관할, 다리 위 elevated 시점, 양방향 4차로, 1:14, 1640×1236
- `clip02_gumi_view2.mp4`: 같은 174km 부근의 다른 카메라, 측면 elevated 시점, 1:59

이전 시도들(브라우저 UI 포함 영상, 야간 IR 영상)은 brouser overlay, 글레어, 야간 도메인 갭 등으로 부적합하여 폐기.

---

## 3. 2주차 진행

### 3.1 Sync mode 검증

다중 카메라 동기 녹화의 전제 조건은 CARLA의 synchronous mode가 안정적으로 동작하는 것. 데이터 생성에 앞서 검증 스크립트 `test_sync_mode.py` 작성.

검증 항목 3가지:

1. `synchronous_mode = True` 설정의 안정적 적용
2. `world.tick()` 1회당 카메라 프레임 정확히 1개 발생 (다중 카메라 동기의 전제)
3. 종료 시 sync mode 정상 해제 (안 하면 서버 hang)

결과: 10 tick에서 10 frame 정확히 수신, carla_frame이 1씩 증가 (329 → 338), timestamp가 0.05초씩 정확히 증가. **모든 검증 통과**.

### 3.2 시나리오 녹화 파이프라인 (Step A~D)

데이터 생성 단계를 4개 Step으로 나누어 점진적 검증. 각 Step은 직전 Step의 인프라 위에 새 요소 하나만 추가하는 방식.

| Step | 추가된 요소 | 검증 결과 |
|---|---|---|
| A | 단일 카메라 + 차량 1대 + autopilot | 100/100 PNG, 차량 시야 진입 |
| B | 3 카메라 동시 spawn + 동기 | 200 tick × 3 카메라 = 600 PNG, carla_frame 모두 일치 |
| C | 차량 수동 제어 (wobble/abrupt/normal) | 행동 패턴이 ground truth에 명확히 관찰 |
| D | yaml 기반 시나리오 양산 + 배치 실행 | 8 시나리오 자동 생성, 19GB |

Step B의 핵심 검증: 같은 tick에서 3 카메라의 `carla_frame` 값이 모두 일치 — 다중 카메라 동시 관찰의 신뢰성 근거. 200 tick 전체에서 불일치 0건.

Step D는 yaml에 정의된 8개 시나리오를 일괄 실행하여 `data/scenarios/{name}/` 폴더에 카메라별 PNG, ground_truth.jsonl, scenario_config.yaml을 저장. 시나리오 사이엔 카메라를 재사용하고 차량만 spawn/destroy하여 효율화.

### 3.3 차량 행동 제어 설계

본 프로젝트의 핵심은 정상/이상 운전 시나리오를 명확한 강도 차이로 생성하는 것. 세 가지 행동 클래스를 추상화로 설계:

**WobbleBehavior (음주/졸음 시뮬레이션)**: sin 곡선 형태의 좌우 스티어링 진동 + 차선 유지 보정.
```
steer(t) = amplitude × sin(2π · t / period) + gain × (current_y - target_y)
```
차선 유지 보정 항은 vehicle의 spawn 시점 y를 target으로 삼아 누적 드리프트를 방지. 초기 시도에서 amplitude=0.08로 너무 강하게 설정하여 차량이 8m 드리프트 후 차선을 벗어나 충돌하는 문제가 발생. amplitude를 0.04로 축소하고 차선 유지 항을 추가하여 해결.

**AbruptBehavior (난폭 운전 시뮬레이션)**: 주기적 throttle/brake 토글.
```
phase = (t mod period) / period
phase < 0.5: throttle=1.0, brake=0
phase >= 0.5: throttle=0, brake=0.7
```

**NormalBehavior**: CARLA autopilot 활용. False positive 측정용 baseline.

### 3.4 시나리오 정의와 라벨링 정합성

8개 시나리오 정의 (`config/scenarios.yaml`): 정상 2개 (속도 다양화) + wobble 3강도 + abrupt 3강도.

첫 번째 batch 실행 결과 측정에서 **wobble 강도가 의도와 역전**되어 나옴: "mild"가 가장 큰 차선 이탈(3m), "strong"이 가장 작은 변위(1m).

분석 결과 lateral 변위가 **amplitude × period²** 에 비례함을 발견. 초기 설계에서 amplitude는 키우고 period는 줄였기에 결과적으로 mild의 wobble 에너지가 strong보다 50% 큰 상태였음:

| | amplitude | period² | amplitude × period² |
|---|---|---|---|
| mild | 0.025 | 6.25 | 0.156 (가장 큼) |
| medium | 0.035 | 4.00 | 0.140 |
| strong | 0.045 | 2.25 | 0.101 (가장 작음) |

자동 라벨링 수정 도구 `relabel_scenarios.py`를 제작하여 폴더명, 내부 ground_truth.jsonl의 scenario 필드, scenario_config.yaml, 그리고 `config/scenarios.yaml`의 정의까지 일관되게 수정. 향후 시나리오 추가 시 가이드라인: period는 고정하고 amplitude로만 강도 조정.

최종 측정값 (강도별 단조 증가 검증):

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

wobble은 lateral offset이, abrupt는 speed 변동이 강도별 단조 증가. 이는 추후 메트릭 모듈이 강도별로 일관된 점수를 산출하는지 검증하는 기준점 역할.

### 3.5 Trajectory 클러스터링 모듈 구현

1주차에서 결정한 방향대로 `trajectory_clustering.py` 작성 (~300줄).

알고리즘:

1. 차량 ID별 검출/관측 묶기 → trajectory 생성
2. arc length 기준 균일 리샘플링 (속도 차이 보정)
3. Modified Hausdorff distance (Dubuisson & Jain, 1994)로 쌍별 거리 계산
4. DBSCAN with precomputed metric으로 클러스터링
5. 각 클러스터의 평균 trajectory를 차선 centerline으로 추출

입력 인터페이스를 두 가지로 분리:
- `load_trajectories_from_csv(csv_path)`: Role A의 YOLO+ByteTrack 출력 (image space, 픽셀)
- `load_trajectories_from_carla(scenarios_dir)`: CARLA ground truth (world space, 미터)

3주차에 Role A의 출력 수신 시 입력 함수만 교체하면 동일 알고리즘 적용 가능.

CARLA 8개 시나리오 입력으로 검증한 결과:

- 8개 trajectory 모두 동일 클러스터(lane 0)로 묶임 — 의도대로 (모두 같은 차선 출발)
- noise 0개
- 추출된 centerline의 x ∈ [299.8, 476.8], y ∈ [-17.86, -17.19] (spawn y = -17.5 와 일치)
- 시각화 결과는 `data/trajectory_clustering_viz.png`

거리 행렬에서 한 가지 quirk: abrupt_mild ↔ abrupt_strong 거리가 3.18m로 가장 큼. 두 시나리오의 평균 속도 차이로 trajectory 총 길이가 달라(162m vs 191m), Hausdorff 거리가 endpoint mismatch 영향을 받음. 다행히 DBSCAN의 density chain 특성으로 중간 trajectory가 다리 역할을 하여 정상 클러스터링됨.

---

## 4. 주요 발견과 결정

본 보고 기간 중 핵심 발견 3가지를 정리:

**(1) UFLDv2의 한국 CCTV 부적합 / 학술 대안 도출 (1주차)**: CULane으로 사전 학습된 모델이 한국 elevated CCTV에 적용되지 않는다는 점을 시각적으로 확인. 학술 문헌 조사를 통해 차량 trajectory 클러스터링이라는 학습 데이터 불필요한 대안 발견. 일정 단축과 동시에 본 프로젝트 계획서의 정신과 더 부합하는 결정.

**(2) 시뮬레이션 차량 행동의 비선형성 (2주차)**: 비틀거림의 lateral 변위가 단순히 amplitude에 비례하지 않고 amplitude × period²에 의존. 첫 batch 결과에서 발견하여 라벨링을 수정하고 향후 가이드라인을 정립함. 자체 검증 시스템(강도별 단조 증가 확인)이 이 오류를 자동 탐지한 점이 설계적 의의.

**(3) Modified Hausdorff의 trajectory 길이 민감성 (2주차)**: trajectory 총 길이 차이가 거리 행렬에 영향을 미침을 확인. 본 데이터셋에서는 DBSCAN의 density chain으로 흡수되었으나, 실제 CCTV 적용 시 차량별 카메라 체류 시간이 다양해 더 두드러질 가능성. 3주차에 DTW 또는 공통 공간 trim 전처리 검토 예정.

---

## 5. 산출물 정리

코드:
- `step_a_record.py` ~ `step_c_record.py` — Step별 검증 스크립트 (보관)
- `run_scenarios.py` — 시나리오 배치 실행기
- `relabel_scenarios.py` — 라벨 정합성 도구
- `trajectory_clustering.py` — 차량 trajectory 클러스터링 모듈
- `test_sync_mode.py` — sync mode 검증
- `infer_one.py` — UFLDv2 단일 이미지 추론 (1주차, 보고서용 보관)
- `load_town06.py`, `camera_test_capture.py`, `spectator_watch.py` — 1주차 카메라 설정 도구

설정:
- `config/cameras.yaml` — 카메라 3개 좌표 정의
- `config/scenarios.yaml` — 시나리오 8개 정의

데이터:
- `data/scenarios/{시나리오}/` — 8 시나리오 × 250 tick × 3 카메라 = 6,000 PNG + ground_truth.jsonl + scenario_config.yaml. 총 19GB
- `data/lane_hypotheses.json` — Step G 클러스터링 결과
- `data/trajectory_clustering_viz.png` — 클러스터링 시각화
- `data/real/clip01_gumi_view1.mp4`, `clip02_gumi_view2.mp4` — 실제 CCTV 보조 영상 (경부선 174km, 구미지사)

문서:
- `docs/lane_reference_decision.md` — 1주차 결정 보고서 (UFLDv2 → trajectory 클러스터링)
- `docs/week2_report.md` — 2주차 단독 보고서
- `docs/midterm_report.md` — 본 통합 보고서

전체 코드 약 1,500줄, 데이터 19GB.

---

## 6. 3~4주차 계획

3주차 주요 작업:

1. Role A의 YOLO+ByteTrack 출력 수신 후 trajectory_clustering.py에 연결 — 입력 인터페이스만 교체
2. 호모그래피 캘리브레이션 — 카메라 픽셀 좌표 ↔ CARLA world 좌표 변환 매트릭스 구성. 메트릭 계산 시 픽셀 거리를 실거리로 환산 시 필요
3. Modified Hausdorff의 trajectory 길이 민감성 대응 — DTW 또는 공통 공간 trim 검토
4. Role C의 메트릭 모듈과 통합 — lane_hypotheses.json을 메트릭 입력으로 전달

4주차 주요 작업:

1. 8 시나리오 + 실제 CCTV 2 클립에 전체 파이프라인 적용
2. 강도별 메트릭 점수가 단조 증가하는지 정량 검증
3. 최종 보고서 및 발표 자료 작성

---

## 7. 참고 문헌

- Melo, J., Naftel, A., Bernardino, A., & Santos-Victor, J. (2006). Detection and classification of highway lanes using vehicle motion trajectories. *IEEE Transactions on Intelligent Transportation Systems*, 7(2), 188-200.
- Ren, X., Wang, B., Xu, J., et al. (2014). Vehicle trajectory clustering based on density-based clustering algorithm. *International Conference on Image Processing*.
- Tang, Z., Naphade, M., Liu, M.-Y., et al. (2017). CityFlow: A city-scale benchmark for multi-target multi-camera vehicle tracking and re-identification. *CVPR*.
- Qiu, J., et al. (2024). Vehicle trajectory clustering for lane recognition in highway CCTV. (학회 발표 자료)
- Dubuisson, M.-P., & Jain, A. K. (1994). A modified Hausdorff distance for object matching. *International Conference on Pattern Recognition*, 566-568.
- Ester, M., Kriegel, H.-P., Sander, J., & Xu, X. (1996). A density-based algorithm for discovering clusters in large spatial databases with noise. *KDD'96 Proceedings*.

---

## 부록 A. 시각 자료

다음 산출물 이미지가 본 보고서를 시각적으로 보완:

- `camera_check/cam0.png`, `cam1.png`, `cam2.png` — 1주차 카메라 위치 검증 캡처
- `data/scenarios/wobble_strong/cam0/000050.png` 등 — 시나리오 PNG 예시
- `data/trajectory_clustering_viz.png` — Step G 클러스터링 결과 시각화
- UFLDv2 시도 결과 이미지: `test_output.png` (CARLA), `real_result.png` (한국 CCTV)

## 부록 B. CARLA 시뮬레이션 환경

- 운영체제: Windows
- CARLA 버전: 0.9.14
- Python 환경: conda env `carla37` (Python 3.7) for CARLA, `ufld` (Python 3.8) for UFLDv2 시도
- 맵: Town06 (AdditionalMaps_0.9.14.zip에서 별도 설치)
- 카메라 sensor: sensor.camera.rgb, FOV 90°, 1920×1080
- Sync mode: fixed_delta_seconds = 0.05 (20 fps)
- 차량 모델: vehicle.tesla.model3
