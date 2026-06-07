# 운전 행동 검출 시스템 — A 모듈 (영상 처리 파이프라인)

**프로젝트:** 4주 대학 팀 프로젝트 — 운전 행동 검출 및 위험 운전자 식별
**작성일:** 2026-06-02
**역할:** A 모듈 — 영상으로부터 차량 위치/속도/차로 데이터 추출

## 개요

CCTV 또는 시뮬레이션 영상으로부터 차량을 검출/추적하고, 호모그래피 변환으로 미터 좌표를 계산하여 차로 번호, 차로 중심 이탈, 추정 속도를 산출하는 시스템.

출력 JSON은 C 모듈(룰 적용)에 입력되어 위험 운전 행동(차선 표류, 짧은 차간거리 등) 판단에 사용된다.

## 시스템 구성

```
영상 입력 (mp4)
    ↓
YOLOv11n (Ultralytics) — 차량 검출
    ↓
ByteTrack — 차량 추적 (ID 부여)
    ↓
호모그래피 변환 (수동 4점 캘리브레이션)
    ↓
미터 좌표 기반 계산
  - lane_id (차로 번호)
  - lateral_offset_m (차로 중심 이탈)
  - speed_est_mps (추정 속도, 5프레임 이동 평균)
    ↓
JSON v1.1 출력 → C 모듈 입력
```

## 처리 결과 요약

### CCTV 영상 (실제 환경)

| 영상 | 위치 | 차로 수 | Unique 차량 | 비고 |
|---|---|---|---|---|
| v01 | 강서구청 시내 교차로 | 3차로 | 70대 | 시내 교차로 |
| v02 | 김포공항 직선 | 6차로 | 75대 | 정체 영상 |
| v04 | 성수대교 도시 4차로 | 4차로 | 166대 | 도시 도로 |
| v05 | 올림픽대로 둔촌 방향 | 5차로 | 109대 | 5차로 직선 |
| v06 | 양재IC→반포IC | 4차로 | 134대 | 정체 영상 |

**v03**: 부적합 (카메라 수직 시점) — 처리 중단

### CARLA 시뮬레이션 (GT 정량 평가)

| 시나리오 | lane 정확도 | lateral 오차 | 속도 오차 |
|---|---|---|---|
| normal | 100.0% | 0.15m | 6.48 km/h |
| lane_weaving | 100.0% | 0.17m | 7.87 km/h |
| tailgating | 100.0% | 0.13m | 5.87 km/h |
| sudden_lane_change | 99.9% | 0.24m | 10.91 km/h |
| speeding | 99.4% | 0.15m | 12.91 km/h |

**매칭률 100%**: A가 검출한 모든 차량이 GT와 일치 (오검출 0개)

## 폴더 구조

```
driving2/
│
├── data/
│   ├── videos/                 # CCTV 원본 영상
│   │   └── v01.mp4 ~ v06.mp4
│   └── carla/                  # CARLA 영상 + GT
│       ├── normal_video.mp4 ~ speeding_video.mp4
│       ├── homography_info.json     # B 제공
│       └── gt/
│           └── *.json               # B 제공 GT
│
├── v01/, v02/, v04/, v05/, v06/   # CCTV 처리 결과
│   ├── configs/calibration/
│   │   ├── H_matrix.npy             # 호모그래피 행렬
│   │   ├── H_meta.json              # 캘리브레이션 메타
│   │   ├── road_only.jpg            # 차량 제거 배경
│   │   ├── bev_check.jpg            # BEV 검증 이미지
│   │   ├── opencv_lanes.jpg         # 차선 검출 시각화
│   │   └── lanes_detected.json      # 차선 검출 결과
│   └── outputs/
│       ├── test_tracks_v1.1.json    # 핵심 데이터
│       ├── videos/track/             # 추적 시각화 영상
│       └── reports/
│           └── validation_v{ID}.md  # 영상별 검증 보고서
│
├── carla_calibration/              # CARLA 공통 호모그래피
│   └── configs/
│       ├── H_matrix.npy
│       ├── H_meta.json
│       └── ...
│
├── carla_normal/, carla_lane_weaving/, ...   # CARLA 처리 결과
│   └── outputs/
│       ├── test_tracks_v1.1.json
│       └── videos/track/
│
├── carla_gt_evaluation.json       # GT 비교 정량 결과
│
├── cctv_summary_report.md          # CCTV 종합 보고서
├── carla_summary_report.md         # CARLA 종합 보고서
├── final_report.md                 # 최종 종합 보고서
├── schema_v1.1.md                  # JSON v1.1 스키마
└── README.md                       # 본 문서
```

## JSON v1.1 데이터 형식

각 영상마다 `outputs/test_tracks_v1.1.json` 생성:

```json
{
  "version": "v1.1",
  "video_id": "v01",
  "fps": 24.0,
  "total_frames": 538,
  "road_info": {
    "lane_width_m": 3.5,
    "num_lanes": 3,
    "road_width_m": 10.5
  },
  "frames": [
    {
      "frame_id": 0,
      "timestamp_sec": 0.0,
      "vehicles": [
        {
          "track_id": 5,
          "bbox_pixel": [820, 540, 920, 640],
          "position_road_m": [3.5, 10.0],
          "lane_id": 1,
          "lateral_offset_m": 0.2,
          "speed_est_mps": 13.5
        }
      ]
    }
  ]
}
```

상세 스키마: [`schema_v1.1.md`](schema_v1.1.md)

## 실행 환경

- **플랫폼**: Google Colab
- **GPU**: T4 (CUDA 가속)
- **저장소**: Google Drive (`/content/drive/MyDrive/driving2/`)

**주요 라이브러리:**
- ultralytics (YOLOv11)
- opencv-python
- numpy, scikit-learn
- matplotlib

## 실행 방법

### CCTV 영상 처리

**노트북:** `A_pipeline_week3_cctv.ipynb`

```
1. 환경 세팅 (Drive 마운트, ultralytics 설치)
2. 영상 정보 + 미리보기
3. road_only 생성 (차량 제거 배경)
4. 격자 이미지 (4점 선정용)
5. 호모그래피 4점 캘리브레이션 + BEV 검증
6. YOLO + ByteTrack 영상 처리
7. 호모그래피 적용 → JSON v1.1 생성
8. 검증 통계
9. OpenCV 차선 검출 (보조)
10. 검증 보고서 자동 생성
```

### CARLA 영상 처리

**노트북:** `A_pipeline_week3_carla.ipynb`

```
1. 환경 세팅 + CARLA 영상/GT 로드
2. 호모그래피 1번 캘리브레이션 (5개 영상 공통)
3. 5개 영상 자동 처리 (process_carla_video 함수)
4. GT 비교 정량 평가
5. 종합 보고서 생성
```

## 주요 산출물

| 파일 | 설명 |
|---|---|
| `final_report.md` | 최종 종합 보고서 (CCTV + CARLA) |
| `cctv_summary_report.md` | CCTV 5개 영상 종합 |
| `carla_summary_report.md` | CARLA GT 정량 평가 |
| `schema_v1.1.md` | JSON v1.1 스키마 문서 |
| `v{ID}/outputs/test_tracks_v1.1.json` | 영상별 핵심 데이터 |
| `v{ID}/outputs/reports/validation_v{ID}.md` | 영상별 검증 보고서 |
| `carla_gt_evaluation.json` | CARLA GT 비교 결과 |
| `ac_integration_report.md` | A+C 통합 시스템 평가 (CARLA GT 검증 + CCTV 임계값 분석) |

## 시스템 강점

- ✅ **알고리즘 정확성** (CARLA GT 정량 검증)
  - lane_id 정확도: 99.4~100%
  - lateral 오차: 평균 0.15m
  - 매칭률: 100% (오검출 0)

- ✅ **다양한 환경 대응**
  - 5개 다양한 CCTV 영상 (시내, 정체, 고속도로)
  - 3~6차로 다양한 도로 폭

- ✅ **표준화된 인터페이스**
  - JSON v1.1 스키마
  - C 모듈에 일관된 데이터 전달

## 한계 및 개선 방향

- ⚠️ **호모그래피 영역 제한** (현재 20~50m)
  - 영역 밖 차량 검출 불가
  - 개선: 더 큰 영역 + 도로 곡률 보정

- ⚠️ **OpenCV 차선 검출 한계** (검출률 20~50%)
  - 픽셀 기반 알고리즘의 본질적 한계
  - 개선: UFLDv2 등 딥러닝 차선 검출
  - 영향: 호모그래피 lane_id로 보완되어 분석에 영향 미미

- ⚠️ **속도 추정 잡음** (오차 6~13 km/h)
  - YOLO 박스 변동, 호모그래피 외삽 오차
  - 개선: Kalman Filter, 더 긴 평활화

- ⚠️ **카메라 시점 의존성**
  - 비스듬한 각도만 가능 (수직 시점 부적합)
  - 개선: 다양한 시점 학습 데이터

## 시행착오 기록

**효과 있었던 시도:**
- 깨끗한 구간으로 road_only 재생성 (v02)
- 수동 4점 호모그래피 (다양한 환경 안정적)
- CARLA 시뮬레이션 사용 (정량 평가 가능)

**효과 없었던 시도:**
- OpenCV 차선 검출 임계값 조정 (영상별 미미)
- 호모그래피 영역 확장 (CARLA 40m → 50m)
- YOLO 모델 업그레이드 (n → s, ID switching 부작용)

**부적합 영상 판단:**
- v03 (카메라 수직 시점): YOLO 검출 실패 → 처리 중단

## 데이터셋 참고

**CCTV 영상**: 한국 공공 CCTV (UTIC 등)에서 수집
**CARLA 시뮬레이션**: B 모듈에서 생성 (CARLA Town06, 5차로)

## 팀 구성

- **A 모듈**: 영상 처리 파이프라인 (본 작업)
- **B 모듈**: CARLA 시뮬레이션 + GT 생성
- **C 모듈**: 룰 적용 + 위험 운전자 판단

## 라이선스 및 참고

본 프로젝트는 대학 4주 팀 프로젝트의 결과물.

**주요 참고:**
- Ultralytics YOLOv11
- ByteTrack (Yifu Zhang et al., 2022)
- CARLA Simulator (Dosovitskiy et al., 2017)
