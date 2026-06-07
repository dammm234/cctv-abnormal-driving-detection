# A+C 통합 시스템 평가 보고서

**생성일:** 2026-06-07 09:02
**대상:** A (영상 처리) + C (행동 룰) 통합 시스템

## 0. 요약 (Executive Summary)

본 보고서는 A 모듈(영상 처리)과 C 모듈(행동 룰)을 결합한 위험 운전 검출 시스템의 통합 평가 결과이다.

**핵심 결과:**
- ✅ **CARLA 통제 환경 정확도 100%** (활성 룰 3개)
- ⚠️ **CCTV 실제 환경**: GT 없어 절대 정확도 측정 불가, 임계값 튜닝 효과 분석
- ✅ **시스템 데이터 흐름 완벽** (A JSON → C 룰 적용)

## 1. 시스템 개요

```
영상 (mp4)
    ↓
[A 모듈]
  YOLOv11 + ByteTrack + 호모그래피
    ↓
JSON v1.1 (차량 위치, 속도, 차로)
    ↓
[C 모듈]
  행동 룰 (차선 표류, 차선 변경, 차간거리)
    ↓
의심 차량 리스트 + 사유
```

### 1.1 A → C 인터페이스

- **데이터 형식:** JSON v1.1 (schema_v1.1.md 참조)
- **C 입력:** `detect_suspects(json_data)`
- **C 출력:** `{suspect_ids, suspects, summary, total_analyzed, thresholds}`

## 2. C 모듈 룰 정의

| 룰 | 의미 | 판정 기준 (기본) |
|---|---|---|
| `lane_weaving` | 차선 표류 | 차로 변경 없이 lateral_offset 표준편차 > 0.30m |
| `lane_change` | 급차선 변경 | lane_id 변경 1회 이상 |
| `tailgating` | 짧은 차간거리 | 동일 차로 앞뒤 간격 < 5.0m |

**비활성 룰:** 속도 위반 (CARLA 속도 분포 한계로 제외)

**기본 임계값:**
```python
  weaving_std: 0.3
  lane_change: 1
  tail_gap: 5.0
  min_track_frames: 10
```

## 3. CARLA GT 정량 평가 (핵심)

### 3.1 시나리오별 결과

| 시나리오 | GT 의심 차량 | C 검출 차량 | 일치 여부 |
|---|---|---|---|
| normal | 0대 | 0대 | ✅ 완벽 일치 |
| lane_weaving | 1대 | 1대 | ✅ 완벽 일치 |
| tailgating | 1대 | 1대 | ✅ 완벽 일치 |
| sudden_lane_change | 1대 | 1대 | ✅ 완벽 일치 |
| speeding | 1대 | 0대 | 비활성 룰 (예상됨) |

### 3.2 정확도 평가 (활성 룰 3개)

CARLA의 4개 시나리오 중 활성 룰 (lane_weaving, tailgating, lane_change) 검증:

```
True Positive (TP):  3개
False Positive (FP): 0개 (normal에서 의심 0대)
False Negative (FN): 0개 (놓친 위험 차량 없음)

Precision = TP / (TP + FP) = 1.00
Recall    = TP / (TP + FN) = 1.00
F1 Score  = 1.00
```

**해석:** 통제된 시뮬레이션 환경에서 A+C 결합 시스템이 완벽한 정확도 달성.

### 3.3 사유 매칭

| 시나리오 | GT 정답 행동 | C 검출 사유 | 일치 |
|---|---|---|---|
| normal | 없음 | 없음 | ✅ 둘 다 의심 없음 |
| lane_weaving | ['lane_weaving'] | {'lane_weaving': 1} | ✅ 사유 일치 |
| tailgating | ['tailgating'] | {'tailgating': 1} | ✅ 사유 일치 |
| sudden_lane_change | ['sudden_lane_change'] | {'lane_change': 1} | ✅ (sudden_lane_change ↔ lane_change) |
| speeding | ['speeding'] | 없음 | ⚠️ 속도 룰 비활성 |

## 4. CCTV 실제 환경 분석

**중요:** CCTV는 GT 정답 데이터가 없어 절대 정확도 측정 불가. 임계값 영향 분석.

### 4.1 기본 임계값 적용 결과

| 영상 | 환경 | 분석 차량 | 의심 차량 | 의심 비율 | 사유 분포 |
|---|---|---|---|---|---|
| v01 | 흐름 | 57 | 52 | 91.2% | lane_change:27, tailgating:41, lane_weaving:6 |
| v02 | 정체 | 67 | 22 | 32.8% | tailgating:20, lane_weaving:4, lane_change:4 |
| v04 | 흐름 | 119 | 84 | 70.6% | lane_change:13, tailgating:65, lane_weaving:10 |
| v05 | 흐름 | 71 | 58 | 81.7% | lane_change:38, tailgating:45, lane_weaving:5 |
| v06 | 정체 | 108 | 91 | 84.3% | tailgating:84, lane_weaving:19, lane_change:4 |

### 4.2 임계값 조정 효과

3가지 임계값 비교:

**기본 임계값:** tail_gap=5.0, lane_change=1, weaving_std=0.30

**조정 임계값 (영상별):**
- 흐름 영상 (v01, v04, v05): tail_gap=3.0, lane_change=2
- 정체 영상 (v02, v06): tail_gap=2.0, lane_change=2

**엄격한 임계값 (전체):** tail_gap=2.0, lane_change=3, weaving_std=0.40, min_track_frames=30

| 영상 | 기본 | 조정 | 엄격 |
|---|---|---|---|
| v01 | 52대 (91.2%) | 46대 (80.7%) | 30대 (63.8%) |
| v02 | 22대 (32.8%) | 9대 (13.4%) | 6대 (14.0%) |
| v04 | 84대 (70.6%) | 71대 (59.7%) | 43대 (59.7%) |
| v05 | 58대 (81.7%) | 56대 (78.9%) | 42대 (77.8%) |
| v06 | 91대 (84.3%) | 64대 (59.3%) | 49대 (59.8%) |

### 4.3 결과 분석

**주요 발견:**

- **v02 (정체)**: 임계값 조정 효과 큼 (33% → 13%)
- **v06 (정체)**: 조정 효과 큼 (84% → 59%)
- **v05 (흐름)**: 조정 효과 미미 (82% → 78%)
  - 원인: lane_change가 실제로 많이 발생 (5차로 도로 특성)

**한국 도시 도로의 특성:**
- 차간거리 짧음 (5m 이하 일상적)
- 차로 변경 자주 (다차로 도로)
- 차선 표류 자연스러움

**임계값 영향:**
- 정체 영상은 차간거리 임계값에 매우 민감
- 흐름 영상은 lane_change 임계값에 영향 받음
- 영상별 맞춤 임계값 필요

## 5. 시스템 강점과 한계

### 5.1 강점

**✅ 알고리즘 정확성 (CARLA 정량 검증)**
- 활성 룰 3개 정확도: P/R/F1 = 1.00
- False positive 0%
- A의 호모그래피 정확도 (lane 100%, lateral 0.15m) + C의 룰 정확성

**✅ 데이터 인터페이스 안정성**
- JSON v1.1 스키마 통한 명확한 A→C 데이터 전달
- 10개 영상 모두 처리 성공

**✅ 임계값 유연성**
- 영상 환경 따라 동적 조정 가능
- C 모듈의 thresholds 파라미터로 손쉬운 튜닝

### 5.2 한계

**⚠️ CCTV 실제 환경 검증 어려움**
- GT 라벨링 데이터 부재
- 의심 비율 60~80%이 false positive인지 true positive인지 판단 불가
- 한국 도로 특성 반영한 데이터셋 필요

**⚠️ A 시스템 한계로 인한 룰 영향**
- 호모그래피 영역 제한 (50m) → 영역 밖 차량 미검출
- lane_id 변동성 → lane_change 룰 false positive 가능
- 속도 추정 잡음 → 속도 룰 비활성 원인

**⚠️ 임계값 보편성 부족**
- CARLA 기준 임계값이 실제 도로에 그대로 적용 어려움
- 영상별 환경 (정체/흐름) 따라 다른 임계값 필요
- 자동 임계값 학습 시스템 필요

## 6. 결론

### 6.1 검증된 사실

- **A+C 시스템 알고리즘 정확성** (CARLA 100%)
- **데이터 흐름 안정성** (A JSON → C 룰 100% 성공)
- **임계값 조정 가능성** (CCTV 영상별 동적 튜닝)

### 6.2 향후 개선 방향

1. **GT 라벨링된 실제 데이터 확보**
   - 한국 도시 도로 영상에 위험 운전 행동 라벨링
   - CCTV 환경에서 정량 평가 가능해짐

2. **속도 룰 활성화**
   - A의 속도 추정 정밀화 (Kalman Filter 등)
   - C의 속도 룰 검증 데이터 확보

3. **자동 임계값 시스템**
   - 교통 밀도 기반 자동 임계값 조정
   - 영상 환경 자동 분류 (정체/흐름)

4. **A 시스템 정밀도 개선**
   - 호모그래피 영역 확장
   - lane_id 안정화 (히스테리시스 적용)
