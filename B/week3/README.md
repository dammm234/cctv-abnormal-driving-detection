# Week 3 — 호모그래피 캘리브레이션


1. **(A) CARLA 호모그래피** ✓ 완료
2. **(B) 실제 CCTV 호모그래피** ✓ sample2 완료 (sample1 보류)
3. **(C) Role A 통합** — 진행 예정 (A가 `schema_v1.1.md` 공개)
4. **(D) Trajectory 클러스터링 개선 (DTW)** — 진행 예정 (선택)

## 파일

### 코드

| 파일 | 설명 |
|---|---|
| `homography.py` | CARLA 카메라 호모그래피 (intrinsic + extrinsic 자동 계산) |
| `homography_real.py` | 실제 CCTV 4점 대응 호모그래피 (OpenCV findHomography) |

### 산출물

| 파일 | 설명 |
|---|---|
| `homography_carla.json` | 3개 CARLA 카메라 매트릭스 (K, world_to_camera, H_ground) |
| `homography_real_sample1.json` | 실제 CCTV sample1 4점 호모그래피 |
| `homography_real_sample2.json` | 실제 CCTV sample2 4점 호모그래피 |
| `homography_validation_cam0.png` | CARLA cam0 검증 (GT trajectory 투영) |
| `homography_validation_cam1.png` | CARLA cam1 검증 |
| `homography_validation_cam2.png` | CARLA cam2 검증 |
| `sample1_homography_viz.png` | 실제 CCTV sample1 검증 시각화 |
| `sample2_homography_viz.png` | 실제 CCTV sample2 검증 시각화 |

## CARLA 호모그래피 (homography.py)

CARLA의 카메라 intrinsic(FOV=90°, 1920×1080)과 extrinsic(cameras.yaml)을 알고 있어 자동 계산.

**검증**: `wobble_strong` 시나리오의 ground truth 차량 위치 250개를 각 카메라로 투영하여 실제 PNG 위에 오버레이.

| 카메라 | 시야 내 프레임 | 검증 결과 |
|---|---|---|
| cam0 | 35/250 | ✓ Frame 22에서 마커가 차량 위에 정확 |
| cam1 | 212/250 | ✓ wobble trajectory 곡선까지 정확히 표시 |
| cam2 | 250/250 | ✓ 차량과 마커 일치 |

## 실제 CCTV 호모그래피 (homography_real.py)

OpenCV `cv2.findHomography`로 4점 대응 기반.

워크플로:
1. 영상에서 프레임 추출 (default frame 30)
2. matplotlib에서 사용자가 도로 위 4점 클릭
3. 한국 고속도로 표준 (차선 폭 3.5m, 점선 주기 13m)으로 H 계산
4. 1m 격자를 픽셀로 투영하여 정렬 검증

sample2 결과: P1-P2 픽셀 차이 314px, P3-P4는 116px → perspective 비율 2.7. 정상.

sample1 결과 : P1-P2 vs P3-P4 비율이 1:1에 가까움 → 4점이 실제 사각형이 아닐 가능성. 4주차 데모는 sample2 우선.


