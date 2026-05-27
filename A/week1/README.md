# A - 1주차 산출물

## 산출물 목록
- `schema_v1.md` — 출력 JSON 스키마 v1.0 문서
- `test_tracks_v1.json` — 테스트 영상에 대한 1주차 출력 (더미값 포함)
- `tracking_report_v1.md` — 추적 안정성 검증 보고서
- `custom_bytetrack.yaml` — ByteTrack 튜닝 설정
- `A_pipeline_week1.ipynb` — Colab 노트북-코드

## 진행 상황
- YOLOv11 + ByteTrack 동작
- 출력 스키마 v1.0 확정
- ByteTrack 파라미터 튜닝 (track_buffer=90, conf=0.15)
- 검증 보고서 작성

## 알려진 한계 (2주차 해결 예정)
- `position_road_m`, `lane_id`, `lateral_offset_m`, `speed_est_mps`는 더미값
- ID switching 일부 남음 (호모그래피 적용 후 재평가 예정)

## 영상 파일
영상은 용량 문제로 리포에 포함하지 않음
