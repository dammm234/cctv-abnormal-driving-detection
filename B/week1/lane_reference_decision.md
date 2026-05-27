# 차선 Reference 방식 결정 보고서

**작성자:** B (좌표계 & 시뮬레이션 담당)
**일자:** 2026-05-XX
**관련 작업:** 1주차 Step 3 — UFLDv2 동작 확인 및 후속 결정

---

## 1. 초기 계획

다중 CCTV 이상 운전 탐지 시스템의 차선 표류 metric 계산을 위해, **UFLDv2 (Ultra Fast Lane Detection v2)** 의 CULane pre-trained 가중치를 적용하기로 계획. 카메라마다 차선 polyline을 자동 추출하여 C의 metric 계산 모듈에 reference로 제공할 계획이었음.

## 2. 실험 결과

**(1) CARLA 합성 영상 (Town06, cam2.png)**
모델이 차선 2개를 검출하였으나 검출 위치가 실제 차선 마킹과 일치하지 않음 (Fig. `test_output.png`). 차선의 대략적 방향은 맞으나 픽셀 단위 정확도 부족. → **합성-실제 도메인 갭**.

**(2) 실제 한국 고속도로 CCTV (real_test.png)**
3개 차선이 검출되었으나 그중 1개(Lane 1)만 실제 마킹을 정확히 따라감 (Fig. `real_result.png`). 나머지 2개는 도로 가장자리 또는 차선 사이 영역으로 어긋남. → **Perspective 갭**.

## 3. 근본 원인 분석

UFLDv2 학습 데이터(CULane, TuSimple)는 **차량 내부 dashcam 시점**(낮은 높이, 정면)에서 촬영됨. 본 프로젝트의 배포 환경은 **고속도로 갠트리 CCTV 시점**(높은 위치, 비스듬한 내려보기)으로 학습 데이터와 시점이 근본적으로 다름. 모델 자체가 아닌 **학습 데이터 분포 문제(domain shift)** 로 진단. 다른 SOTA 모델(CLRNet, LaneATT 등)도 동일 학습 데이터셋을 사용하므로 같은 한계 예상.

## 4. 대안 검토

| 접근법 | 채택 여부 | 사유 |
|---|---|---|
| 다른 DL 모델 (CLRNet 등) | ✗ | 동일 데이터셋 학습으로 같은 도메인 갭 |
| Domain Adaptation (MLDA) [1] | ✗ | 학습 필요, 4주 프로젝트 범위 밖 |
| Generative Augmentation [2] | ✗ | 학습 필요 |
| BEV 변환 후 검출 | ✗ | BEV 변환 후에도 모델 도메인 미스매치 |
| 수동 annotation | △ | 정확하나 새 카메라 적용 시 매번 작업 필요 |
| **차량 trajectory 클러스터링** | **✓** | **차선 마킹 비의존, perspective 무관, 시스템 통합성** |

## 5. 채택안 — 차량 Trajectory 클러스터링

YOLOv11+ByteTrack으로 수집된 차량 trajectory를 일정 시간 누적한 후 클러스터링하면 각 클러스터의 중심선이 곧 차선 reference가 됨. 차선 마킹의 가시성, 카메라 perspective와 무관하게 동작.

**관련 문헌:**
- [3] Melo et al. (2006), "Detection and Classification of Highway Lanes Using Vehicle Motion Trajectories," *IEEE Transactions on Intelligent Transportation Systems* — Trajectory 기반 lane geometry 학습 최초 제안.
- [4] Ren et al. (2014), "Lane Detection in Video-Based Intelligent Transportation Monitoring via Fast Extracting and Clustering of Vehicle Motion Trajectories," *Mathematical Problems in Engineering* — Hausdorff distance + k-means, 카메라 변화 자동 적응.
- [5] Tang et al. (2017), "Lane Detection by Combining Trajectory Clustering and Curve Complexity Computing in Urban Environments," IEEE — 회전 차선까지 검출.
- [6] Qiu et al. (2024), "Real-time Lane-wise Traffic Monitoring in Optimal ROIs," arXiv:2404.15212 — PTZ 카메라 자동 적응형, F1 0.79+.

## 6. 본 프로젝트와의 정합성

계획서 4.1: *"차선 검출에 의존하지 않고 차량 자체의 평활화된 궤적을 reference로 사용한다."*

본 결정은 계획서의 원래 정신으로 복귀하는 결정. 또한 다음 부가 효과:

- A의 출력(차량 trajectory)을 lane reference 학습에도 재사용하여 모듈 절감.
- CARLA와 실제 CCTV에 동일 코드 적용 가능.
- 새 카메라가 추가되어도 일정 시간 데이터 수집만으로 자동 적응.

## 7. 구현 계획 (2주차)

1. 정상 traffic 시나리오에서 trajectory 100개 이상 수집 (시나리오 녹화 시 자동 확보).
2. DBSCAN 또는 Hausdorff distance 기반 클러스터링.
3. RANSAC으로 차선 변경 trajectory outlier 제거.
4. 클러스터 중심선 polynomial fitting → polyline 추출.
5. 카메라별 `lanes_<cam_id>.json` 저장 (C의 차선 표류 metric 입력).

**산출물:** UFLDv2 시도 코드(`infer_one.py`)와 실험 결과 이미지는 보고서 자료로 보관.

---

## 참고 문헌

[1] J. Li et al., "Multi-level Domain Adaptation for Lane Detection," arXiv:2206.10692, 2022.
[2] "Data Augmentation Strategies for Robust Lane Marking Detection," arXiv:2511.18668, 2025.
[3] J. C. Melo et al., "Detection and Classification of Highway Lanes Using Vehicle Motion Trajectories," IEEE TITS, 2006.
[4] J. Ren et al., "Lane Detection in Video-Based Intelligent Transportation Monitoring," *Math. Problems Eng.*, 2014.
[5] Tang et al., "Lane Detection by Combining Trajectory Clustering and Curve Complexity Computing," IEEE, 2017.
[6] M. Qiu et al., "Real-time Lane-wise Traffic Monitoring in Optimal ROIs," arXiv:2404.15212, 2024.
