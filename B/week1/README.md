# Week 1 — 환경 설정 및 방향 결정

## 작업 요약

1. CARLA Town06 맵 추가 설치—
2. UFLDv2 차선 검출 시도 — CULane 모델로 한국 CCTV에 적용 시도, perspective gap 발견


## 파일

| 파일 | 설명 |
|---|---|
| `cameras.yaml` | 카메라 3대 위치/회전 정의 (Town06 본선 구간) |
| `load_town06.py` | Town06 맵 로드 도구 |
| `camera_test_capture.py` | 카메라 위치 검증 (3개 카메라에서 한 프레임씩 캡처) |
| `spectator_watch.py` | CARLA 시점 도구 (수동으로 맵 둘러볼 때) |
| `infer_one.py` | UFLDv2 단일 이미지 추론 (시도 결과 보존용) |
| `lane_reference_decision.md` | UFLDv2 → trajectory 클러스터링 전환 결정 보고서 |

