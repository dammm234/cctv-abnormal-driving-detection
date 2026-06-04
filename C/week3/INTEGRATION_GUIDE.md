# C 모듈 통합 가이드 (A 파이프라인 연동용)

A의 인식 파이프라인 출력(JSON v1.1)을 C의 행동 룰에 통과시켜
의심 차량을 판정하는 방법.

## 빠른 시작

```python
from behavior_rules import detect_suspects

# 방법 1) A가 메모리에 가진 JSON dict를 바로 전달
result = detect_suspects(tracks_dict)

# 방법 2) 파일 경로 전달
result = detect_suspects("outputs/test_tracks_v1.1.json")

# 방법 3) 결과를 파일로 저장
detect_suspects(tracks_dict, save_path="suspects.json")
```

## 입력 형식
A의 schema_v1.1 JSON. 각 frame의 vehicles에 다음 필드 필요:
`track_id`, `lane_id`, `lateral_offset_m`, `position_road_m`

## 출력 형식

```json
{
  "suspect_ids": [12, 6, 17],
  "suspects": [
    {
      "track_id": 12,
      "reasons": ["tailgating"],
      "weaving_std": 0.289,
      "lane_changes": 0,
      "track_frames": 78
    }
  ],
  "summary": {"tailgating": 1, "lane_weaving": 1},
  "total_analyzed": 5,
  "thresholds": { ... }
}
```

- `suspect_ids` — 의심 차량 ID 리스트 (결과 통합용, 가장 간단)
- `suspects` — 차량별 상세(사유, 점수)
- `summary` — 사유별 집계

## 행동 룰 (3종)
| reason 값 | 의미 | 판정 기준 |
|---|---|---|
| `lane_weaving` | 차선 표류 | 차선 변경 없이 lateral_offset 표준편차 > 0.30m |
| `lane_change` | 급차선 변경 | lane_id 변경 1회 이상 |
| `tailgating` | 짧은 차간거리 | 동일 차선 앞뒤 간격 < 5.0m |

(속도 위반 룰은 CARLA 속도 분포 한계로 현재 비활성)

## 임계값 재보정 (중요)
CARLA 기준 임계값을 실제 CCTV에 그대로 쓰면 차간거리 과탐 발생
(정체 구간은 원래 차들이 붙어 다님). 교통 밀도에 맞게 조정:

```python
# 정체 구간: 차간거리 기준을 더 빡빡하게
result = detect_suspects(data, thresholds={"tail_gap": 2.5})
```

## 검증 상태
- CARLA 통제 시나리오 4종에서 P/R/F1 = 1.00 (개념 증명)
- 실제 CCTV(v01~v06) 동작 확인 완료
