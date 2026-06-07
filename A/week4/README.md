# Week 4 — 시연 영상 및 통합 시스템 데모

**작성일:** 2026-06-07
**대상:** A+C 통합 시스템 시연 영상 + 분석 보고서

## 개요

3주차에서 완성한 A+C 통합 시스템(영상 처리 + 행동 룰)의 시연 영상을 제작하고 분석한 결과.

- **CARLA 시뮬레이션 5개**: 통제 환경, GT 검증 가능
- **CCTV 실제 환경 5개**: 실제 도로, 극엄격 임계값 적용

## 시스템 흐름

```
영상 (mp4)
    ↓
[A 모듈] YOLOv11 + ByteTrack + 호모그래피
    ↓
JSON v1.1 (차량 위치, 속도, 차로)
    ↓
[C 모듈] 행동 룰 (차선 표류, 차선 변경, 차간거리)
    ↓
의심 차량 시각화 영상 (빨간/초록 박스 + 사유 자막)
```

## 시연 영상

**※ 영상 파일은 용량 문제로 GitHub에 업로드되지 않음. 카톡으로 별도 공유.**

### CARLA 시뮬레이션 영상 (5개)

기본 임계값 적용. GT 검증으로 룰 정확도 100% 입증.

| 파일명 | 시나리오 | 의심 차량 | 검출 사유 | 결과 |
|---|---|---|---|---|
| `carla_normal_demo.mp4` | 정상 주행 (대조군) | 0대 | - | ✅ 정확 |
| `carla_lane_weaving_demo.mp4` | 차선 표류 | 1대 | lane_weaving | ✅ 정확 |
| `carla_tailgating_demo.mp4` | 짧은 차간거리 | 1대 | tailgating | ✅ 정확 |
| `carla_sudden_lane_change_demo.mp4` | 급차선 변경 | 1대 | lane_change | ✅ 정확 |
| `carla_speeding_demo.mp4` | 속도 위반 | 0대 | (룰 비활성) | ⚠️ 검출 안됨 |

### CCTV 실제 영상 (5개)

극엄격 임계값 적용. GT 없어 절대 정확도 측정 불가.

**임계값:** tail_gap=1.5m, lane_change=5회, weaving_std=0.50m, min_track_frames=60

| 파일명 | 위치 | 환경 | 의심 비율 |
|---|---|---|---|
| `v01_demo_tight.mp4` | 강서구청 시내 교차로 | 흐름, 3차로 | 56.2% |
| `v02_demo_tight.mp4` | 김포공항 직선 | 정체, 6차로 | 9.1% |
| `v04_demo_tight.mp4` | 성수대교 | 흐름, 4차로 | 86.1% |
| `v05_demo_tight.mp4` | 올림픽대로 | 흐름, 5차로 | 50.0% |
| `v06_demo_tight.mp4` | 양재IC→반포IC | 정체, 4차로 | 50.0% |

## 시연 영상 시청 가이드

### 시각화 요소

- 🟢 **초록 박스**: 정상 차량
- 🔴 **빨간 박스**: 의심 차량 (위험 행동)
- **자막 (영문)**: 의심 사유
  - `lane_weaving`: 차선 표류
  - `lane_change`: 급차선 변경
  - `tailgating`: 짧은 차간거리
- **상단 정보 바**: 영상 ID, 프레임 번호, 의심 차량 총 수

### 추천 시청 순서

1. `carla_normal_demo.mp4` — 정상 주행 (의심 없음, 대조군)
2. `carla_lane_weaving_demo.mp4` — 차선 표류 검출
3. `carla_tailgating_demo.mp4` — 차간거리 검출
4. `carla_sudden_lane_change_demo.mp4` — 차선 변경 검출
5. `v01_demo_tight.mp4` — 실제 CCTV 적용 사례
6. (선택) 다른 CCTV 영상들

## 주요 결과

### ✅ CARLA 통제 환경 — 알고리즘 정확성 입증

- 활성 룰 3개 정확도: **Precision = Recall = F1 = 1.00**
- False positive 0% (normal 영상에서 의심 0대)
- 사유 매칭 100% (lane_weaving, tailgating, lane_change 모두 정확)

### ⚠️ CCTV 실제 환경 — 도메인 격차 발견

- 극엄격 임계값 적용에도 의심 비율 평균 50%
- 한국 도시 도로 특성 (짧은 차간거리, 잦은 차로 변경)
- A 시스템 잡음 (호모그래피, lane_id 변동) 영향
- GT 부재로 false positive vs true positive 구분 불가

### ⚠️ Speeding 룰 비활성

- C 모듈에서 속도 위반 룰 의도적 비활성
- 이유: CARLA 속도 분포 한계로 검증 어려움
- 향후 A 속도 추정 정밀화 + 검증 데이터 확보 시 활성화 예정

## 보고서

- **`week4_demo_report.md`**: 시연 영상 상세 분석
  - CARLA 시연 결과
  - CCTV 시연 결과
  - CARLA vs CCTV 비교
  - 성능 저하 원인 분석
  - 향후 개선 방향

## 이전 단계와의 관계

- **week1** (`../week1/`): 기본 YOLO+ByteTrack 파이프라인 구축
- **week2** (`../week2/`): 호모그래피 캘리브레이션 도입, schema v1.1
- **week3** (`../week3/`): 다중 영상 처리 + CARLA GT + C 모듈 통합
- **week4** (본 폴더): 시연 영상 제작 + 종합 분석

**week3의 통합 보고서** (`../week3/ac_integration_report.md`)에 시스템 검증 결과 상세 기록.
**week4의 데모 보고서** (`week4_demo_report.md`)는 시연 영상 분석에 집중.

## 폴더 구성

```
A/week4/
├── README.md                # 본 문서
└── week4_demo_report.md     # 시연 영상 분석 보고서
```

**시연 영상 파일 (별도 공유):**
- CARLA 5개: `carla_*_demo.mp4`
- CCTV 5개: `v*_demo_tight.mp4`

## 시스템 강점과 한계 요약

### 강점

- ✅ CARLA 통제 환경에서 100% 정확도 달성
- ✅ A → C 데이터 인터페이스 안정성 (JSON v1.1)
- ✅ 임계값 동적 조정으로 환경 적응 가능

### 한계

- ⚠️ CCTV 실제 환경 정량 평가 어려움 (GT 부재)
- ⚠️ A 시스템 잡음으로 false positive 발생 가능
- ⚠️ 속도 룰 비활성 (활성화 필요)
- ⚠️ 한국 도로 특성과 CARLA 기준 임계값 간 격차

### 향후 개선

1. 한국 도로 GT 데이터 확보 → CCTV 정량 평가
2. A 시계열 평활화 (Kalman Filter) → 잡음 감소
3. 속도 룰 활성화 → 4개 룰 완전 시스템
4. 자동 임계값 학습 → 환경별 동적 조정

## 팀 구성

- **A 모듈**: 영상 처리 파이프라인 (본 작업)
- **B 모듈**: CARLA 시뮬레이션 + GT 생성
- **C 모듈**: 행동 룰 + 위험 차량 판단
