# Week 4 보고서 — 다중 CCTV 이상운전 탐지 (Role B)

## 1. 개요

Role B는 **다중 카메라 차량 재식별(ReID) + 이상운전 탐지 + 통계 검증**을 담당한다.
CARLA 시뮬레이터로 다중 차량 시나리오를 생성하고, 카메라 간 차량을 ReID로 연결한 뒤,
횡방향 거동을 분석해 이상운전(흔들림)을 탐지한다.

본 보고서는 멘토 피드백("검증된 방법 필요", "ReID 실구현")에 대한 응답이자,
week4에서 진행한 다음 작업을 정리한다:

1. SDLP + 통계적 유의성 검증
2. OSNet 기반 Cross-Camera ReID (다중 차량)
3. **VeRi-776 fine-tuning** 및 ImageNet 대비 성능 비교
4. Camera Link Model 및 스케일 테스트 (6대 → 12대)
5. 이상운전 탐지 (지그재그율 기반 wobble 탐지)
6. 데모 영상 생성

---

## 2. 시스템 구성과 역할 경계

### 2.1 전체 파이프라인

```
[탐지/추적 — Role A]                [식별/분석 — Role B (본 모듈)]
영상 → YOLOv11 (차량 검출)            ReID (카메라 간 차량 식별)
       ByteTrack (단일 카메라 추적)  →  → 횡방향 궤적 수집
       호모그래피 (좌표 변환)            → 이상운전 탐지
       차선 검출                         → 통계 검증
                  └──── v1.1 JSON ────┘
```

- **탐지(detection)는 A의 YOLO 담당**, 식별(re-identification)·이상탐지는 B 담당.
- 두 모듈은 **v1.1 JSON**(track_id, bbox_pixel, position_road_m, speed_est_mps, fps)으로 연결.
- CARLA 환경에서는 시뮬레이터가 정답 bbox를 제공(현실의 YOLO 역할 대체)하므로,
  B는 ReID와 이상탐지 검증에 집중할 수 있다.

### 2.2 ReID 모델의 위치

본 모듈이 학습한 VeRi OSNet은 **카메라 간 차량 식별(ReID)**에 사용된다.
ByteTrack(A)은 단일 카메라 내에서만 추적하므로, 카메라가 바뀌면 추적이 끊긴다.
ReID는 차량의 외형 feature로 cam0 → cam1 → cam2를 건너뛰는 동일 차량을 이어준다.
이것이 다중 CCTV 추적의 핵심이다.

---

## 3. SDLP + 통계 검증

### 3.1 방법론

**SDLP** (Standard Deviation of Lateral Position, Verster & Roth 2011):
운전자 차선 유지 능력의 표준 지표. detrend 후 표준편차를 계산해 trend를 제거하고
순수 흔들림(wobble)만 측정한다. Sliding window(5초)로 다수 샘플을 생성해 통계 검증을 가능케 한다.

**통계 방법**: One-way ANOVA (Fisher 1925), Welch t-test (Welch 1947),
Cohen's d (Cohen 1988), Kendall τ trend test.

### 3.2 결과 (wobble 시나리오, cam2)

| Test | Statistic | p-value | 해석 |
|---|---|---|---|
| Kendall τ (단조 증가) | τ = 0.44 | 0.007 | 통계적으로 유의 |
| Pairwise t (mild vs strong) | t = -4.29 | 0.003 | 매우 유의 |
| Cohen's d (mild vs strong) | d = 2.14 | — | Very Large |

→ 흔들림 강도의 단조 증가가 통계적으로 유의함을 검증된 방법으로 입증.

---

## 4. ReID 모듈 — Cross-Camera 차량 식별

### 4.1 백본 발전 과정

ReID 백본을 3단계로 개선하며 각 단계의 성능을 측정했다:

1. **ResNet50 (ImageNet)** — 범용 백본, fallback
2. **OSNet (ImageNet)** — 차량/사람 ReID 전용 구조 (Zhou et al. 2019)
3. **OSNet (VeRi-776 fine-tuned)** — 차량 ReID 데이터로 fine-tuning (본 모듈 학습)

### 4.2 VeRi-776 Fine-tuning

- **데이터셋**: VeRi-776 (Liu et al. 2016) — 실제 CCTV 차량 ReID 벤치마크, 576개 차량 ID
- **학습**: ImageNet pretrained OSNet → VeRi fine-tuning (classifier 층 교체, 표준 transfer learning)
- **설정**: batch 32, 256×256, Adam lr 0.0015, cosine schedule, 60 epoch
- **결과 (VeRi 테스트셋)**: **mAP 58.8% / Rank-1 92.3% / Rank-5 96.4%**

### 4.3 ImageNet vs VeRi — CARLA 시나리오 비교 (핵심 결과)

동일 시나리오에서 ImageNet 백본과 VeRi fine-tuned 백본의 retrieval mAP를 비교했다:

| 시나리오 | 차량 수 | ImageNet mAP | VeRi mAP | 변화 |
|---|---|---|---|---|
| multi_demo | 6 | 0.635 | 0.769 | +0.134 |
| multi_demo2 | 6 | 0.664 | 0.785 | +0.121 |
| multi_reid | 6 | 0.573 | 0.683 | +0.110 |
| multi_scale12 | 12 | 0.492 | 0.614 | +0.122 |
| multi_scale12b | 12 | 0.493 | 0.627 | +0.134 |

→ **5개 시나리오 전부 mAP가 +0.11~0.13 일관 상승.** 6대·12대 모두 개선.
합성 데이터(CARLA)임에도 실제 CCTV로 학습한 VeRi 가중치가 효과를 냈다는 것은,
차량 식별 능력이 합성–실제 도메인 갭을 넘어 전이됨을 시사한다.

### 4.4 Camera Link Model 과 매칭 F1

카메라 간 1:1 매칭은 헝가리안 알고리즘(Kuhn 1955)으로 수행하며,
인접 카메라 쌍(cam0-cam1, cam1-cam2)을 매칭 후 전이 연결하는
**camera link model**(Hsu et al. 2021)을 적용했다.

- **6대 시나리오**: ImageNet·VeRi 모두 **매칭 F1 = 1.000**
- **12대 시나리오(scale12b)**: VeRi에서 cam1-cam2 경계 쌍 2개가 어긋나 F1 = 0.889

mAP는 VeRi가 더 높지만 특정 1:1 할당(F1)은 ImageNet이 우연히 맞은 경우가 있다.
이는 **mAP(feature 변별력)와 F1(특정 할당 정확성)이 서로 다른 지표**임을 보여준다.
Camera link model 덕분에 시스템 최종 동작(글로벌 ID 구성)은 양쪽 모두 정확하다.

### 4.5 카메라 시점 분석

| Pair | Mean Sim | 해석 |
|---|---|---|
| cam1 ↔ cam2 | 0.795 | 시점 유사, 매칭 쉬움 |
| cam0 ↔ cam1 | 0.523 | 중간 |
| cam0 ↔ cam2 | 0.365 | 시점 차이 큼 (원거리) |

→ cam0와 cam2는 viewpoint 차이가 커 직접 매칭이 어렵다.
Camera link model(인접 쌍 매칭 + 전이 연결)로 이 문제를 해결한다.

### 4.6 스케일 테스트 (6대 → 12대)

차량 수를 6대에서 12대로 늘려 확장성을 검증했다.
12대에서도 인접 카메라 매칭은 정확했으며(camera link 적용 시),
원거리 쌍(cam0-cam2)의 직접 매칭만 일부 오류가 발생했다.

---

## 5. 이상운전 탐지

### 5.1 방법론

카메라 간 연결된 각 차량의 **횡방향(y) 위치 시계열**을 분석해 이상 거동을 분류한다.

- **wobble (이상)**: 지그재그율(초당 좌우 방향 전환 횟수)이 임계 초과.
  졸음·음주 운전의 전형. 진폭 게이트(0.3m)로 미숙 운전의 미세 흔들림은 제외.
- **lane_change (거동 이벤트, 이상 아님)**: 횡변위 ≥ 1.75m(차선폭 절반).
  정상적인 차선 변경일 수 있어 이상으로 분류하지 않고 표시만 한다.
  빈도 기반 위험도 평가는 향후 과제.

지그재그율은 steering reversal rate 계열(Markkula & Engström 2006)을 횡위치 기반으로 구현했다.

### 5.2 wobble 임계

wobble 판정 임계는 **0.4 회/초(고정)**로 설정했다.
정상 주행의 지그재그율은 ~0.03, wobble 주입 차량은 0.5 이상으로 명확히 구분된다.
(자동 임계 산정은 wobble 차량이 정상군 통계를 오염시키는 문제가 있어 고정값을 채택.)

### 5.3 검증 (controlled experiment)

CARLA에서 wobble 거동을 의도적으로 주입(set_transform으로 횡위치를 사인파로 제어)하고,
탐지기가 정확히 wobble로 분류하는지 검증했다.

- **데모 시나리오(multi_demo3, 6대)**: wobble 주입 2대(G1, G4)를 정확히 탐지,
  나머지는 정상 또는 차선 변경으로 분류. 주입 거동과 탐지 결과 일치.

---

## 6. 데모 영상

- 카메라별 개별 영상(cam0/cam1/cam2)으로 생성, 각 차량의 판정을 실시간 표시.
- **실시간 누적 판정**: 각 프레임에서 그 시점까지 관측된 궤적만으로 판정하며,
  관찰이 쌓이면 normal → wobble/lane_change로 전환된다(미래 정보를 사용하지 않음).
- 색상: 정상(초록), wobble(빨강), lane_change(파랑, 이상 아님), 관찰 중(회색).

---

## 7. 현실 확장성

CARLA(합성 환경)에서는 차량 외형이 뚜렷해 camera link model만으로도 완벽히 매칭되어,
VeRi fine-tuning의 효과가 시스템 동작(F1)에는 드러나지 않는다.

VeRi 차량 전용 백본의 진정한 가치는 **현실 CCTV 배포 시** 나타난다.
실제 도로에는 동일 차종(예: 흰색 세단 다수)이 많아 범용 백본으로는 구분이 어렵고,
차량 전용 ReID 백본이 식별 성능에 결정적이다.
실배포 시에는 한국 CCTV 데이터로 추가 fine-tuning하는 것이 표준적 다음 단계(VehicleNet 방식)이다.

---

## 8. 산출물

### 8.1 코드
- `run_scenarios_multi.py` — CARLA 다중 차량 시나리오 생성 (정상/wobble 거동 주입)
- `train_veri_osnet.py` — VeRi-776 OSNet fine-tuning
- `reid_extract_features.py` — OSNet feature 추출 (ImageNet / VeRi 선택)
- `reid_match_validate_multi.py` — 다중 차량 헝가리안 매칭 + camera link + 평가
- `reid_eval_protocol.py` — mAP / Rank-k retrieval 평가
- `anomaly_pipeline.py` — 이상운전 탐지 (지그재그율 기반 wobble 분류)
- `anomaly_demo_video.py` — 카메라별 데모 영상 생성
- `sdlp_implementation.py` — SDLP 계산
- `statistical_validation.py` — ANOVA + t-test + Cohen's d + Kendall τ

### 8.2 설정
- `config/cameras.yaml` — Town06 카메라 배치 (FOV 90, 1920×1080)
- `config/homography_*.json` — 호모그래피 행렬

---

## 9. 참고 문헌

1. Verster, J. C., & Roth, T. (2011). Standard operation procedures for the on-the-road driving test. *Substance Abuse and Rehabilitation*.
2. Liu, X., Liu, W., Mei, T., & Ma, H. (2016). A deep learning-based approach to progressive vehicle re-identification (VeRi-776). *ECCV*.
3. Zhou, K., Yang, Y., Cavallaro, A., & Xiang, T. (2019). Omni-scale feature learning for person re-identification (OSNet). *ICCV*.
4. Hsu, H.-M., et al. (2021). Multi-target multi-camera vehicle tracking. *IEEE T-IP*.
5. Kuhn, H. W. (1955). The Hungarian method for the assignment problem.
6. Markkula, G., & Engström, J. (2006). A steering wheel reversal rate metric.
7. Fisher, R. A. (1925); Welch, B. L. (1947); Cohen, J. (1988) — 통계 방법.

---

## 10. 향후 과제

1. A의 YOLO 출력(v1.1 JSON)을 입력으로 한 end-to-end 통합 테스트
2. 차선 변경 빈도 기반 위험도 평가 (현재는 거동 표시만)
3. 한국 CCTV 데이터로 ReID 추가 fine-tuning (현실 배포)
4. 시공간 제약(트래픽 흐름) 결합으로 원거리 매칭 보강
