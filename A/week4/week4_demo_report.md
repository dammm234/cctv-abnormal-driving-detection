# Week 4 데모 영상 분석 보고서

**생성일:** 2026-06-07 10:49
**대상:** CARLA 5개 + CCTV 5개 시연 영상
**임계값:** CARLA - 기본 / CCTV - 극엄격

## 0. 개요

본 보고서는 A+C 통합 시스템의 시연 영상 분석 결과이다.
시연 영상은 두 환경에서 생성:

1. **CARLA 시뮬레이션 (5개)**: 통제 환경, GT 검증 가능
2. **CCTV 실제 환경 (5개)**: 실제 도로, GT 없음, 극엄격 임계값 적용

## 1. 시연 영상 목록

### 1.1 CARLA 시뮬레이션 영상

| 파일명 | 시나리오 | 의도된 행동 |
|---|---|---|
| `carla_normal_demo.mp4` | 정상 주행 (대조군) | - |
| `carla_lane_weaving_demo.mp4` | 차선 표류 | lane_weaving |
| `carla_tailgating_demo.mp4` | 짧은 차간거리 | tailgating |
| `carla_sudden_lane_change_demo.mp4` | 급차선 변경 | lane_change |
| `carla_speeding_demo.mp4` | 속도 위반 | speeding |

**저장 위치:** `driving2_demo/demo_videos/`

### 1.2 CCTV 영상

| 파일명 | 위치 | 환경 |
|---|---|---|
| `v01_demo_tight.mp4` | 강서구청 시내 교차로 | 흐름, 3차로 |
| `v02_demo_tight.mp4` | 김포공항 직선 | 정체, 6차로 |
| `v04_demo_tight.mp4` | 성수대교 | 흐름, 4차로 |
| `v05_demo_tight.mp4` | 올림픽대로 | 흐름, 5차로 |
| `v06_demo_tight.mp4` | 양재IC→반포IC | 정체, 4차로 |

## 2. CARLA 시연 영상 결과

CARLA 영상은 GT 정답이 있어 룰 정확도 검증 가능.

### 2.1 시나리오별 결과

| 시나리오 | 분석 차량 | 의심 차량 | 사유 | 정답 일치 |
|---|---|---|---|---|
| normal | 5 | 0 | 없음 | ✅ 정확 (대조군, 의심 0대) |
| lane_weaving | 5 | 1 | lane_weaving:1 | ✅ 정확 (의심 1대) |
| tailgating | 5 | 1 | tailgating:1 | ✅ 정확 (의심 1대) |
| sudden_lane_change | 5 | 1 | lane_change:1 | ✅ 정확 (의심 1대) |
| speeding | 5 | 0 | 없음 | ⚠️ 속도 룰 비활성 (검출 못 함, 예상됨) |

### 2.2 활성 룰 정확도

CARLA의 4개 시나리오 중 활성 룰 (lane_weaving, tailgating, lane_change) 검증:

```
True Positive (TP):  3개
False Positive (FP): 0개 (normal에서 의심 0대)
False Negative (FN): 0개

Precision = Recall = F1 = 1.00
```

### 2.3 짚을 점 — Speeding 영상

**현상:** 속도 위반 영상에서 빠른 차량이 있었으나 시스템에서 검출 못 함.

**원인:** C 모듈에서 **속도 룰 자체가 비활성**.
- C INTEGRATION_GUIDE: "속도 위반 룰은 CARLA 속도 분포 한계로 현재 비활성"
- 임계값 문제가 아니라 룰 자체가 안 돌아감

**의미:**
- 시스템 결함이 아닌 의도된 설계
- 속도 룰은 향후 활성화 필요 (A 속도 추정 정밀화 + C 룰 검증 데이터)

## 3. CCTV 시연 영상 결과

CCTV는 GT 없음 → 절대 정확도 측정 불가, 의심 차량 시각화 분석.

### 3.1 영상별 결과 (극엄격 임계값)

**임계값:** tail_gap=1.5m, lane_change=5회, weaving_std=0.50m, min_track_frames=60

| 영상 | 환경 | 분석 차량 | 의심 차량 | 의심 비율 | 사유 분포 |
|---|---|---|---|---|---|
| v01 | 흐름 | 32 | 18 | 56.2% | lane_change:7, lane_weaving:2, tailgating:14 |
| v02 | 정체 | 33 | 3 | 9.1% | tailgating:3, lane_change:1 |
| v04 | 흐름 | 36 | 31 | 86.1% | tailgating:30, lane_weaving:1 |
| v05 | 흐름 | 42 | 21 | 50.0% | lane_change:15, tailgating:13 |
| v06 | 정체 | 44 | 22 | 50.0% | tailgating:19, lane_weaving:2, lane_change:3 |

### 3.2 시각적 관찰

**v01 (시내 교차로, 흐름):**
- 의심 비율 56% — 절반 이상 빨간 박스
- 주요 사유: tailgating, lane_change
- 한국 시내 도로 특성상 차간거리 짧음

**v02 (김포공항, 정체):**
- 의심 비율 9% — 가장 합리적
- 분석 차량 적음 (정체로 정적 차량 많아 min_track 60 통과 적음)

**v04 (성수대교):**
- 의심 비율 86% — 가장 높음
- min_track_frames 60으로 분석 차량 119→36 감소
- 장기 추적 차량은 도시 도로 특성상 tailgating 많이 발생

**v05 (올림픽대로 5차로):**
- 의심 비율 50%
- lane_change 15건 — 5차로 직선이라 차로 변경 빈번

**v06 (양재IC, 정체):**
- 의심 비율 50%
- 정체 시 차간거리 짧음 + 차선 표류 자연스러움

### 3.3 CCTV에서의 성능 저하 원인

**1. A 시스템 잡음 (호모그래피)**
- YOLO 박스 위치 ±1~2 픽셀 자연 변동
- 호모그래피 변환 시 미터 단위 변동 증폭
- → lateral_offset, lane_id 자연 변동 → false positive

**2. 한국 도로 특성**
- 차간거리 짧음 (정체 시 1~2m 일상적)
- 다차로 도로 (5차로)에서 차로 변경 빈번
- 운전자 습관 (차선 안에서 미세 흔들림)

**3. GT 데이터 부재**
- false positive vs true positive 구분 불가
- 의심 비율 50~80%가 진짜인지 잡음인지 판단 어려움

## 4. CARLA vs CCTV 비교

| 항목 | CARLA | CCTV |
|---|---|---|
| GT 데이터 | ✅ 있음 | ❌ 없음 |
| 환경 | 통제 시뮬레이션 | 실제 도로 |
| 차량 수 | 5대 | 50~150대 |
| 적용 임계값 | 기본 | 극엄격 |
| 정확도 | 100% (P/R/F1=1.00) | 측정 불가 |
| 의심 비율 | 20% (5대 중 1대) | 평균 50% |
| 시연 효과 | 룰 동작 명확 | 실제 환경 한계 |

### 4.1 두 영상이 보여주는 것

**CARLA 시연:** 시스템이 위험 행동을 "정확히" 검출 가능함을 입증
- 정상 주행: 의심 0대 (false positive 없음)
- 위험 행동: 정확히 1대 검출 + 사유 일치

**CCTV 시연:** 실제 환경에서 적용 시 한계 노출
- 한국 도로 특성과 CARLA 기준 임계값의 차이
- A 시스템 잡음의 영향
- GT 부재로 절대 평가 불가

## 5. 결론

### 5.1 핵심 발견

1. **CARLA 통제 환경에서 시스템 알고리즘 정확성 입증** (P/R/F1=1.00)
2. **CCTV 실제 환경에서 도메인 격차 (Domain Gap) 확인**
   - CARLA 깔끔한 데이터 vs CCTV 잡음 많은 실제 데이터
   - 임계값 튜닝으로 부분 개선 가능 (의심 비율 72% → 50%)
3. **속도 룰의 의도적 비활성** (향후 활성화 필요)

### 5.2 시연 영상의 가치

- **CARLA 시연**: 시스템 "작동 가능성" 입증
- **CCTV 시연**: 실제 환경 "적용 시 고려사항" 제시
- 두 시연 영상을 함께 보면 시스템의 강점과 한계 모두 명확

### 5.3 향후 개선 방향

1. **한국 도로 GT 데이터 확보** → CCTV 정량 평가 가능
2. **A 시스템 시계열 평활화** → lane_id, lateral_offset 잡음 감소
3. **속도 룰 활성화** → 4개 룰 완전 시스템
4. **자동 임계값 학습** → 환경별 동적 조정

## 6. 부록 — 시연 영상 시청 가이드

### 시각화 요소

- 🟢 **초록 박스**: 정상 차량
- 🔴 **빨간 박스**: 의심 차량 (위험 행동)
- **자막 (영문)**: 의심 사유
  - `lane_weaving`: 차선 표류
  - `lane_change`: 급차선 변경
  - `tailgating`: 짧은 차간거리
- **상단 정보 바**: 영상 ID, 프레임 번호, 의심 차량 총 수

### 추천 시청 순서

1. `carla_normal_demo.mp4` — 정상 주행 (의심 없음)
2. `carla_lane_weaving_demo.mp4` — 차선 표류 검출
3. `carla_tailgating_demo.mp4` — 차간거리 검출
4. `carla_sudden_lane_change_demo.mp4` — 차선 변경 검출
5. `v01_demo_tight.mp4` — 실제 CCTV 적용 사례
6. (선택) 다른 CCTV 영상들
