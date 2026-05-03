# sonar_3d_reconstruction Regression Infrastructure

> Phase B-1.0 에 도입. 후속 phase (B-2 / B-3 / C / D) 모두 동일 스크립트 재사용.

이 디렉토리의 스크립트는 refactor PR 머지 전 알고리즘 회귀 0 (또는 spec 에 명시된 임계 이내) 를 정량 검증하기 위한 회귀 인프라입니다. baseline (보통 main HEAD 또는 archive baseline tag) 과 candidate (현재 refactor branch) 두 환경에서 동일 bag 을 replay 하여 출력 octomap 을 비교합니다.

스펙 정의 위치: `docs/source/design/2026-05-03-quality-perf-uplift-design.md` §4 (회귀 검증 정책 / 임계 / 데이터셋).

## 1. 구성

| 파일 | 책임 |
|------|------|
| `regression_metric.py` | 두 octomap (`*.bt`) 을 입력받아 `jaccard_set`, `mean_log_odds_diff`, `voxel_count_*` 등 메트릭 산출 (라이브러리 + CLI). |
| `regression_test.sh` | bag replay → octomap 저장 오케스트레이터. `baseline` / `candidate` / `compare` 서브커맨드. |
| `regression_compare.py` | baseline + candidate 결과 디렉토리에서 메트릭을 계산해 PASS/FAIL 판정 (임계 비교). |
| `regression_plot.py` | 메트릭/타이밍 시각 비교 plot (PNG) 생성. PR 본문 attach 용. |

## 2. 사용법 (Phase B-1 예시)

### a) baseline 빌드 + 측정

baseline 은 main HEAD (또는 phase 시작 시 찍은 archive baseline tag) 를 의미합니다.

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git checkout main
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh baseline
```

### b) candidate 빌드 + 측정

candidate 는 현재 refactor branch HEAD 입니다.

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git checkout refactor/phase-b1-perf-surgical
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh candidate
```

### c) 비교 + plot

```bash
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh compare
python3 src/sonar_3d_reconstruction/scripts/regression/regression_plot.py
```

`compare` 가 PASS 면 머지 가능. FAIL 이면 알고리즘 회귀가 발생한 것이므로 candidate 코드를 재검토.

## 3. 환경 변수

| 변수 | 기본값 | 의미 |
|------|--------|------|
| `BAG_PATH` | (P-1 경로, §6 참조) | replay 할 bag 파일 절대 경로 |
| `PLAY_DURATION` | `60` | bag 재생 시간 (초). 짧게 잡으면 노이즈 증가, 길수록 안정 |
| `OUT_DIR` | `/tmp/sonar3d_regression` | 결과 저장 루트 (`baseline/`, `candidate/`, `compare/` 하위 생성) |
| `JACCARD_THRESHOLD` | `1.0` (B-1) | jaccard_set 최소값 (phase 별 §5 표 참조) |
| `MEAN_LOG_ODDS_THRESHOLD` | `0.0` (B-1) | mean_log_odds_diff 최대값 |
| `LAUNCH_FILE` | `octree_sonar_mapper.launch.py` | 실행할 launch 파일 |
| `SLAM_LAUNCH_PKG` | `sonar_3d_reconstruction` | launch 가 속한 패키지 |
| `PC_TOPIC` | `/sonar/pointcloud` | replay 측 토픽 (필요 시 remap) |

## 4. 임계값 (Spec §4.4)

| Phase | jaccard_set | mean_log_odds_diff | avg_proc_time |
|-------|-------------|--------------------|----------------|
| B-1   | = 1.0       | = 0.0              | ≤ baseline |
| B-2   | ≥ 0.95      | 의도된 변화        | ≤ baseline |
| B-3   | ≥ 0.99      | ≤ 0.1              | ≤ baseline |
| C     | ≥ 0.95~0.99 | ≤ 0.5              | ≤ baseline |
| D     | ≥ 0.99      | ≤ 0.1              | ≤ baseline / 1.5~5 |

각 phase 에서 위 임계를 모두 통과해야 머지 가능.

## 5. 데이터셋 (P-1 + P-2)

각 phase 는 두 데이터셋 모두에서 임계 통과해야 머지.

- **P-1 (m750d)**: `/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1`
  - duration 272.8s, 1284 frames
- **P-2 (m3000d)**: `/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m3000d_blueboat/m3000d-range15-tilt90`
  - duration 352.0s, 1757 frames

## 6. 운영 주의사항

- 결과 디렉토리(`/tmp/sonar3d_regression/`) 는 `.gitignore` 대상 — repo commit 금지 (재생성 가능).
- plot 이미지(PNG) 는 PR 본문에 attach 만 하고 repo 에는 commit 하지 않습니다.
- bag/db3/metadata.yaml 절대 삭제·수정 금지 (CLAUDE.md 데이터 안전 정책). `mv` 만 허용.
- 격리 worktree(`/workspace/ros2_ws_phase_a/`) 에서 작업합니다 — 메인 worktree(`/workspace/ros2_ws/`) 와 환경 충돌 방지.

## 7. 환경 셋업 주의 (Task 0 발견)

메인 worktree 에 동일 패키지가 설치돼 있으면 의존성 자동 source 결과 generic_type 이중 register 충돌이 발생합니다. 격리 worktree 에서 측정 시 다음 절차로 환경 변수를 정리한 뒤 진행합니다.

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws_phase_a/install/setup.bash
# 메인 ws sonar_3d_reconstruction 항목 제거 (의존성 자동 source 결과)
for var in PYTHONPATH AMENT_PREFIX_PATH LD_LIBRARY_PATH; do
    val=$(eval "echo \$$var")
    new=$(echo "$val" | tr ':' '\n' | grep -v '/workspace/ros2_ws/install/sonar_3d_reconstruction' | paste -sd:)
    eval "export $var=\"$new\""
done
```
