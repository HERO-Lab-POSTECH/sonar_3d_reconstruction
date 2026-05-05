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
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git checkout main
cd /workspace/ros2_ws
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_DOMAIN_ID=42 bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh baseline
```

### b) candidate 빌드 + 측정

candidate 는 현재 refactor branch HEAD 입니다.

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git checkout refactor/phase-b1-perf-surgical
cd /workspace/ros2_ws
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_DOMAIN_ID=42 bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh candidate
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
| `BAG_PATH` | (P-1 경로, §5 참조) | replay 할 bag 파일 절대 경로 |
| `PLAY_DURATION` | `90` | bag 재생 시간 (초). 짧게 잡으면 노이즈 증가, 길수록 안정 |
| `OUT_DIR` | `/tmp/sonar3d_regression` | 결과 저장 루트 (`baseline/`, `candidate/`, `compare/` 하위 생성) |
| `JACCARD_THRESHOLD` | `1.0` (B-1) | jaccard_set 최소값 (phase 별 §4 표 참조) |
| `MEAN_LOG_ODDS_THRESHOLD` | `0.0` (B-1) | mean_log_odds_diff 최대값 |
| `LAUNCH_PKG` / `LAUNCH_FILE` | `sonar_3d_reconstruction` / `3d_mapping.launch.py` | sonar 처리 launch |
| `SLAM_LAUNCH_PKG` / `SLAM_LAUNCH_FILE` | `fast_lio` / `mapping.launch.py` | SLAM (odom 공급) launch |
| `PC_TOPIC` | `/pkrc/sonar/cpp_pointcloud` | record 대상 출력 토픽 |
| `ROS_DOMAIN_ID` | `42` | DDS 도메인 격리 (§8 참조). 0 이 아니면 default group 과 분리 |

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

## 7. 환경 셋업 (단일 worktree, 2026-05-04 통합 후)

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
cd /workspace/ros2_ws
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## 8. 측정 환경 함정 (2026-05-04 발견, 후속 phase 모두 적용)

### a) DDS 도메인 격리 (`ROS_DOMAIN_ID`)

같은 컨테이너의 다른 Claude 세션이 default `ROS_DOMAIN_ID=0` 으로 sonar 노드를 띄우고 있으면 토픽이 cross-talk 되어 `cloud_0.db3` messages=0 같은 이상 결과가 나옵니다. `regression_test.sh` 헤더에서 `ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"` 로 격리 — override 가능.

다른 세션 활동 검사 (kill 금지, 격리만으로 충분):

```bash
ps aux | grep '/workspace/ros2_ws/install/sonar_3d_reconstruction' | grep -v grep
ps aux | grep -E '3d_mapper_node|fastlio_mapping' | grep -v grep
```

### b) `ros2 launch` SIGINT 자식 미정리 → process group 강제 정리

`fastlio_mapping` 등 C++ 노드가 SIGINT 만으로는 종료되지 않아 `wait` 가 hang 합니다. `regression_test.sh` 는 `setsid` 로 launch 를 새 process group 에 띄우고 종료 시 `kill -INT/-TERM/-KILL -<pgid>` 3 단계 escalation 으로 정리합니다.

### c) cpp module 이중 dlopen 회피

테스트/스모크 코드는 반드시 top-level 만 import:

```python
from sonar_3d_reconstruction import ProbabilityUpdater, OutofcoreTileMapper, MemoryStats
```

`from sonar_3d_reconstruction.cpp_module import ...` 또는 `from sonar_3d_reconstruction import sonar_3d_reconstruction_cpp` 는 **금지** (pybind11 `generic_type already registered` 충돌).
