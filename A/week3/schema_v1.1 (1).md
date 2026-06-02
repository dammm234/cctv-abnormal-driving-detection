# JSON v1.1 스키마 문서

**버전:** v1.1
**생성일:** 2026-05-31
**대상:** A → C 데이터 인터페이스

## 1. 개요

본 문서는 A 파이프라인(영상 처리)이 생성하고 C 파이프라인(룰 적용)이 사용하는 JSON 데이터의 형식을 정의한다.

## 2. 파일 위치

각 영상마다 별도 파일 생성:

```
driving2/v{ID}/outputs/test_tracks_v1.1.json
```

## 3. 전체 구조

```json
{
  "version": "v1.1",
  "video_id": "v01",
  "video_file": "v01.mp4",
  "note": "...",
  "fps": 24.0,
  "total_frames": 538,
  "road_info": {
    "lane_width_m": 3.5,
    "num_lanes": 3,
    "road_width_m": 10.5
  },
  "frames": [...]
}
```

## 4. 최상위 필드

| 필드 | 타입 | 설명 |
|---|---|---|
| `version` | string | 스키마 버전 (현재 "v1.1") |
| `video_id` | string | 영상 ID (예: "v01") |
| `video_file` | string | 원본 영상 파일명 (예: "v01.mp4") |
| `note` | string | 영상 환경 설명 |
| `fps` | float | 영상 FPS |
| `total_frames` | int | 총 프레임 수 |
| `road_info` | object | 도로 정보 |
| `frames` | array | 프레임별 차량 데이터 |

## 5. road_info 필드

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `lane_width_m` | float | m | 차로 폭 (기본 3.5m) |
| `num_lanes` | int | - | 차로 수 |
| `road_width_m` | float | m | 도로 폭 = lane_width_m × num_lanes |

**참고:** 영상마다 차로 수가 다름 (3차로, 4차로, 5차로, 6차로 등)

## 6. frames 필드 (프레임별 데이터)

각 프레임 객체:

```json
{
  "frame_id": 0,
  "timestamp_sec": 0.0,
  "vehicles": [...]
}
```

| 필드 | 타입 | 설명 |
|---|---|---|
| `frame_id` | int | 프레임 번호 (0부터 시작) |
| `timestamp_sec` | float | 영상 시작 후 경과 시간 (초) |
| `vehicles` | array | 해당 프레임의 차량 데이터 |

## 7. vehicles 필드 (차량별 데이터)

각 차량 객체:

```json
{
  "track_id": 5,
  "bbox_pixel": [x1, y1, x2, y2],
  "position_road_m": [3.5, 10.0],
  "lane_id": 1,
  "lateral_offset_m": 0.2,
  "speed_est_mps": 13.5
}
```

| 필드 | 타입 | 단위 | 설명 |
|---|---|---|---|
| `track_id` | int | - | 차량 고유 ID (ByteTrack) |
| `bbox_pixel` | array[4] | px | 픽셀 좌표 바운딩박스 [x1, y1, x2, y2] |
| `position_road_m` | array[2] | m | 도로 좌표 [x, y] - 호모그래피 변환 결과 |
| `lane_id` | int | - | 차로 번호 (1~num_lanes), 0 또는 num_lanes+1은 도로 밖 |
| `lateral_offset_m` | float | m | 차로 중심에서의 가로 이탈 (양수: 오른쪽) |
| `speed_est_mps` | float | m/s | 추정 속도 (5프레임 이동 평균) |

## 8. lane_id 해석

| lane_id | 의미 |
|---|---|
| 0 | 도로 왼쪽 밖 (분석 제외) |
| 1 ~ num_lanes | 정상 차로 |
| num_lanes + 1 | 도로 오른쪽 밖 (분석 제외) |

C 모듈은 `1 <= lane_id <= num_lanes`인 차량만 룰 적용.

## 9. position_road_m 좌표계

호모그래피 영역 좌표:
- **x축 (가로):** 0 ~ road_width_m
  - 0: 도로 왼쪽 끝
  - road_width_m: 도로 오른쪽 끝
- **y축 (세로):** 0 ~ segment_length_m
  - 0: 카메라에 가까운 쪽
  - segment_length_m: 멀리 쪽

영역 밖 차량도 좌표 변환됨 (외삽 영역, 정확도 낮음).

## 10. speed_est_mps 계산

```
raw_speed = sqrt((dx)^2 + (dy)^2) / dt
smoothed = 최근 5프레임 평균
```

**주의:** ID switching 발생 시 극단값(50km/h+) 가능. C 모듈은 평균/중앙값 사용 권장.

## 11. 호모그래피 메타 (별도 파일)

```
driving2/v{ID}/configs/calibration/H_meta.json
```

```json
{
  "video_id": "v01",
  "src_pts": [[x, y], ...],
  "dst_pts": [[x, y], ...],
  "lane_width_m": 3.5,
  "num_lanes": 3,
  "road_width_m": 10.5,
  "segment_length_m": 20.0,
  "note": "..."
}
```

## 12. C 모듈 사용 예시

```python
import json

with open("driving2/v01/outputs/test_tracks_v1.1.json") as f:
    data = json.load(f)

num_lanes = data['road_info']['num_lanes']

for frame in data['frames']:
    for vehicle in frame['vehicles']:
        # 분석 영역 내 차량만 처리
        if not (1 <= vehicle['lane_id'] <= num_lanes):
            continue
        
        # 룰 적용
        track_id = vehicle['track_id']
        lane_id = vehicle['lane_id']
        speed = vehicle['speed_est_mps']
        position = vehicle['position_road_m']
        
        # ... 룰 코드 ...
```

## 13. 영상별 매개변수 (참고용)

| 영상 | num_lanes | road_width_m | segment_length_m |
|---|---|---|---|
| v01 | 3 | 10.5 | 20.0 |
| v02 | 6 | 21.0 | 30.0 |
| v04 | 4 | 14.0 | 20.0 |
| v05 | 5 | 17.5 | 30.0 |
| v06 | 4 | 14.0 | 30.0 |

## 14. 차후 변경 사항 (선택)

향후 추가 가능한 필드 (현재 미포함):
- `is_suspected` (boolean): C가 룰 적용 후 의심 여부 표시
- `confidence` (float): 검출 신뢰도

스키마 변경 시 version 업데이트 필요 (v1.2, v1.3 등).
