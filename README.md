# C (행동 탐지 메트릭 모듈) — week3

A의 인식 결과와 B의 CARLA 시나리오를 입력받아, 차량의 이상 운전 행동을
점수화하는 메트릭 모듈.

## 메트릭
- **weaving**: lateral_offset 표준편차 (차선 내 흔들림). 주력 지표.
- **abrupt**: 가속도 표준편차 (급가감속). CARLA에선 동작, 실제 CCTV는 YOLO 노이즈로 신뢰도 낮음(한계 명시).

## 파일
- `metrics.py` — 메트릭 함수 + B 시나리오 단조 증가 검증
- `run_metrics.py` — B 검증 + A 실제 CCTV 적용 (최종 실행 스크립트)
- `plot_results.py` — 발표용 그래프 생성
- `metric_results.png` — 결과 그래프 (B 단조증가 / A 차량별 weaving)
- `metric_report.md` — 메트릭 모듈 상세 보고서

## 실행
```bash
# 저장소 루트에서
python3 C/week3/run_metrics.py
```
(B/week3/scenarios_v1.1/ 와 A/week2/test_tracks_v1.1.json 필요)

## 결과 요약
- B 시나리오: weaving 단조 증가 통과 (0.605 < 0.926 < 1.287)
- A 실제 CCTV: 32대 중 weaving 0.30 초과 14대를 의심 차량으로 탐지

## 알려진 한계
- abrupt 메트릭은 실제 CCTV의 검출 노이즈로 신뢰도 낮음 → weaving 주력
- 임계값은 데이터 출처(CARLA/실제)별로 별도 설정 필요
