# Week 1 — 환경 설정 및 방향 결정

## 작업 요약

1. **CARLA Town06 맵 추가 설치** — AdditionalMaps_0.9.14.zip을 7-Zip으로 직접 압축 해제
2. **카메라 3대 좌표 확정** — 본선 직선 구간에 일렬 배치 (cam0 → cam1 → cam2)
3. **Role A 출력 포맷 합의** — `frame_idx, timestamp, vehicle_id, x1, y1, x2, y2, confidence`
4. **UFLDv2 차선 검출 시도** — CULane 모델로 한국 CCTV에 적용 시도, perspective gap 발견
5. **방향 전환 결정** — UFLDv2 → 차량 trajectory 클러스터링 (학술 문헌 6편 근거)

## 파일

| 파일 | 설명 |
|---|---|
| `cameras.yaml` | 카메라 3대 위치/회전 정의 (Town06 본선 구간) |
| `load_town06.py` | Town06 맵 로드 도구 |
| `camera_test_capture.py` | 카메라 위치 검증 (3개 카메라에서 한 프레임씩 캡처) |
| `spectator_watch.py` | CARLA 시점 도구 (수동으로 맵 둘러볼 때) |
| `infer_one.py` | UFLDv2 단일 이미지 추론 (시도 결과 보존용) |
| `lane_reference_decision.md` | UFLDv2 → trajectory 클러스터링 전환 결정 보고서 |

## 카메라 배치

Town06 직선 본선 구간, 차량이 -X 방향으로 진행:

| 카메라 | 위치 (x, y, z) | 회전 (pitch, yaw, roll) | 간격 |
|---|---|---|---|
| cam0 | (437.64, -17.54, 13.25) | (-17.72°, -0.28°, 0°) | — (시작) |
| cam1 | (312.21, -18.09, 11.21) | (-14.44°, 3.49°, 0°) | 125m |
| cam2 | (219.30, -17.80, 11.14) | (-17.14°, 1.36°, 0°) | 93m |

## UFLDv2 시도 결과

- CARLA Town06 합성 이미지: 차선 위치 부정확 (도메인 갭)
- 한국 고속도로 elevated CCTV: 3개 차선 중 1개만 정확 (perspective gap)

원인: UFLDv2의 학습 데이터(CULane)는 차량 dashcam 시점이며, 한국 CCTV는 elevated 시점이라 차선의 vanishing point 위치가 본질적으로 다름.

## 방향 전환 근거

학술 문헌 조사 후 차량 trajectory 클러스터링 채택. 자세한 내용은 `lane_reference_decision.md` 참조.

핵심 장점:
- 학습 데이터 불필요 (Korean CCTV fine-tuning 회피)
- Role A의 출력을 그대로 입력으로 사용
- CARLA와 실제 CCTV 양쪽에 동일 코드 적용

## 다음 주

`week2/`에서 trajectory 클러스터링 모듈 구현 및 CARLA 시나리오 데이터셋 생성.
