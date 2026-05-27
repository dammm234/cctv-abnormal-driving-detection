# Role B — 좌표계 & 시뮬레이션

다중 CCTV 기반 이상 운전 탐지 시스템의 Role B 작업 산출물.

## 역할

- CARLA 시뮬레이션 환경 구축 및 시나리오 데이터셋 생성
- 차량 trajectory 클러스터링 모듈 (UFLDv2 대안)
- 호모그래피 캘리브레이션 (CARLA + 실제 CCTV)
- Role A의 검출/추적 출력과 Role C의 메트릭을 연결하는 좌표계 인터페이스

## 폴더 구조

```
B/
├── week1/   - 환경 설정, 카메라 좌표 결정, UFLDv2 → trajectory 클러스터링 결정
├── week2/   - CARLA 8 시나리오 데이터셋 + trajectory 클러스터링 모듈
└── week3/   - 호모그래피 캘리브레이션 (CARLA + 실제 CCTV)
```

각 주차별 README 참조.

## 주요 산출물

| 산출물 | 위치 | 설명 |
|---|---|---|
| 시나리오 데이터셋 | (로컬, .gitignore) | 8 시나리오 × 250 tick × 3 카메라, 19GB |
| trajectory 클러스터링 모듈 | `week2/trajectory_clustering.py` | Modified Hausdorff + DBSCAN |
| 차선 가설 | `week2/lane_hypotheses.json` | 클러스터링 결과, Role C로 전달 |
| CARLA 호모그래피 | `week3/homography_carla.json` | 3개 카메라 픽셀↔월드 변환 |
| 실제 CCTV 호모그래피 | `week3/homography_real_*.json` | 4점 대응 기반 |
| 통합 보고서 | `week2/midterm_report.md` | 1~2주차 전체 정리 |

## 전제 환경

- OS: Windows
- CARLA: 0.9.14 (Town06 맵 추가 설치)
- Python: conda env `carla37` (Python 3.7)
- 필수 패키지: `pyyaml`, `scikit-learn`, `matplotlib`, `Pillow`, `opencv-python`

## Role A와의 인터페이스

A의 출력 (YOLO + ByteTrack)을 `trajectory_clustering.py`의 입력으로 사용:
- 1주차 합의 포맷: `frame_idx, timestamp, vehicle_id, x1, y1, x2, y2, confidence` (CSV)
- 2주차 A 업데이트 (`schema_v1.1.md`) 반영 예정 (3주차 통합 작업)

## Role C와의 인터페이스

C의 메트릭 모듈에 다음 산출 전달:
- 차선 가설: `week2/lane_hypotheses.json`
- 픽셀↔월드 변환: `week3/homography_carla.json` (CARLA), `week3/homography_real_*.json` (실제 CCTV)
- 시나리오 ground truth: `data/scenarios/{name}/ground_truth.jsonl` (로컬, .gitignore)

C는 차량 trajectory를 lane reference 대비 편차로 평가하여 jerk, 곡률 변화, 속도 안정성 등의 메트릭을 산출.
