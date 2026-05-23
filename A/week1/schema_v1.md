# 출력 스키마 v1.0

A 파이프라인 → C 룰 모듈로 전달되는 JSON 데이터 구조.

## 파일 구조

루트 레벨:
- `version` (str): 스키마 버전 ("v1.0" = 1주차 더미값 포함, "v1.1" = 2주차 실제값)
- `note` (str): 비고
- `fps` (float): 원본 영상 프레임 레이트
- `total_frames` (int): 총 프레임 수
- `frames` (list): 프레임 배열

각 프레임:
- `frame_id` (int): 프레임 번호 (0부터)
- `timestamp_sec` (float): 시각 (초) = frame_id / fps
- `vehicles` (list): 그 프레임에서 검출된 차량 배열 (비어있을 수 있음)

각 차량:
- `track_id` (int): 추적 ID. 같은 차량이면 영상 내내 동일 유지(목표)
- `bbox_pixel` (list[float]): 픽셀 박스 [x1, y1, x2, y2] (좌상, 우하)
- `position_road_m` (list[float]): 도로 평면 좌표 [x, y] (미터)
- `lane_id` (int): 차선 번호 (1=가장 왼쪽 차선)
- `lateral_offset_m` (float): 차선 중심선 대비 횡방향 편차 (미터, 양수=오른쪽, 음수=왼쪽)
- `speed_est_mps` (float): 추정 속도 (m/s)

## 좌표계 정의

- **픽셀 좌표 (`bbox_pixel`)**: 영상 좌상단 원점. x는 오른쪽, y는 아래쪽 양의 방향. OpenCV 관례.
- **도로 좌표 (`position_road_m`)**: 호모그래피 변환 후의 평면 좌표 (미터). 차량 위치 기준점은 bbox 하단 중앙 = (`(x1+x2)/2`, `y2`).

## 시간 기준

- `timestamp_sec`는 **영상 시작 기준 상대시간** (0.000부터 시작).
- Unix timestamp나 절대 시각이 아님.
- 디버깅 시 영상 재생 위치와 직접 대응.

## track_id 유효 범위

- `track_id`는 **단일 영상 내에서만 유효**한 ID. ByteTrack이 세션별로 독립 부여.
- 서로 다른 영상 간에는 같은 `track_id`라도 다른 차량으로 간주.
- 멀티 카메라 간 동일 차량 매칭(ReID)은 본 프로젝트 범위 외.

## 검출 대상 클래스

YOLOv11 (COCO pretrained) 출력 중 다음 클래스만 검출 대상으로 사용:
- `car` (class 2)
- `motorcycle` (class 3)
- `bus` (class 5)
- `truck` (class 7)

사람, 자전거 등 비차량 객체는 검출 대상에서 제외. 필요 시 추후 확장 가능.

## 빈 프레임 처리

차량 미검출 프레임은 `vehicles: []`로 표기. **프레임 자체를 생략하지 않음.**

이유: `frame_id`가 시간 축의 단위라 프레임이 빠지면 시계열 분석(차선 표류 σ 계산 등)이 깨짐.

## 현재 버전(v1.0) 한계 (1주차 종료 시점)

다음 필드들은 **더미값**으로 채워져 있음:
- `position_road_m`: 호모그래피 미적용. bbox 하단 중앙의 픽셀 좌표를 그대로 넣은 임시값
- `lane_id`: 차선 검출 미적용. 0으로 통일
- `lateral_offset_m`: 0.0으로 통일
- `speed_est_mps`: 0.0으로 통일

2주차 종료 시 v1.1로 교체 예정. **스키마(필드 이름/타입)는 동결, 값만 실제값으로 채워짐.**

## 룰 적용 가이드 (C 모듈 참고용)

### 차선 표류 (Lane Weaving)
- 입력: 동일 `track_id`의 `lateral_offset_m` 시계열
- 계산: 슬라이딩 윈도우(예: 10초) 표준편차 σ
- 임계 예시: σ > 0.4m가 5초 이상 지속 → 의심

### 짧은 차간거리 (Tailgating)
- 입력: 같은 `lane_id`의 두 차량의 `position_road_m`, `speed_est_mps`
- 계산: (앞차와의 도로 좌표상 거리) ÷ (자차 속도) = time headway
- 임계 예시: < 1.0초가 5초 이상 지속 → 의심

## 변경 이력

- **v1.0** (1주차 종료): 초기 확정. position_road_m, lane_id, lateral_offset_m, speed_est_mps는 더미값
- **v1.1** (2주차 종료 예정): 호모그래피 + 차선 검출 적용, 실제값으로 교체

## 스키마 변경 정책

v1.0 확정 이후 필드명/타입 변경은 A에게 사전 협의 필수.
