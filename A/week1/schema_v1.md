# 출력 스키마 v1.0

A 파이프라인 → C 룰 모듈로 전달되는 JSON 데이터 구조.

## 파일 구조

루트 레벨:
- `version` (str): 스키마 버전 ("v1.0-dummy" = 더미값 포함, "v1.0" = 실제값)
- `note` (str): 비고
- `frames` (list): 프레임 배열

각 프레임:
- `frame_id` (int): 프레임 번호 (0부터)
- `timestamp_sec` (float): 시각 (초)
- `vehicles` (list): 그 프레임에서 검출된 차량 배열

각 차량:
- `track_id` (int): 추적 ID. 같은 차량이면 영상 내내 동일
- `bbox_pixel` (list[float]): 픽셀 박스 [x1, y1, x2, y2]
- `position_road_m` (list[float]): 도로 평면 좌표 [x, y] (미터)
- `lane_id` (int): 차선 번호 (1=가장 왼쪽 차선)
- `lateral_offset_m` (float): 차선 중심선 대비 횡방향 편차 (미터)
- `speed_est_mps` (float): 추정 속도 (m/s)

## 현재 버전(v1.0-dummy) 한계

- `position_road_m`: 픽셀 좌표를 미터로 가정한 임시값
- `lane_id`: 화면 가로 3분할 추정 (실제 차선 검출 아님)
- `lateral_offset_m`: 100픽셀=1m 임시 가정
- `speed_est_mps`: 호모그래피 미적용 픽셀 기반

v1.0 정식 버전(2주차 말)에서 실제값으로 교체 예정. **스키마(필드 이름/타입)는 동결, 값만 정확해짐.**

## 룰 적용 가이드

### 차선 표류 (Lane Weaving)
- 입력: 동일 track_id의 `lateral_offset_m` 시계열
- 계산: 10초 윈도우 표준편차 σ
- 임계: σ > 0.4m가 5초 이상 지속

### 짧은 차간거리 (Tailgating)
- 입력: 같은 `lane_id`의 두 차량의 `position_road_m`, `speed_est_mps`
- 계산: 앞차와의 거리 ÷ 자차 속도 = time headway
- 임계: < 1.0초가 5초 이상 지속

## 변경 이력

- v1.0-dummy (1주차 종료): 초기 버전, 일부 필드 더미값
- v1.0 (2주차 종료 예정): 호모그래피 + 차선 검출 적용 실제값
