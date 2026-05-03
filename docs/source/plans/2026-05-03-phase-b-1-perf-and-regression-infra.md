# Phase B-1: Performance Surgical & Regression Infrastructure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Phase B-1은 (1) 후속 phase 모두가 재사용할 회귀 인프라(`scripts/regression/`)를 신규 작성하고, (2) 알고리즘 결과를 **bit-exact** 그대로 두면서 GIL 해제·PointCloud2 numpy 직렬화·first-hit `np.argmax` 3건의 surgical 성능 개선을 적용한다.

**Architecture:** `fast_lio`에 검증된 회귀 패턴(`regression_test.sh`/`regression_compare.py`/`regression_plot.py`)을 sonar 도메인으로 이식한다. 노드는 변경하지 않고 **`/pkrc/sonar/cpp_pointcloud` 토픽의 마지막 PointCloud2 메시지**를 baseline/candidate 양쪽에서 수집해 voxel set + log-odds 분포로 환산하여 Jaccard·log-odds diff·proc-time을 비교한다. 이 인프라가 자리잡으면 B-2 이후 모든 phase는 동일 명령(`baseline → candidate → compare`)으로 검증한다. 성능 개선은 핫패스 3개 위치에 한정해 알고리즘 결과는 보존한다.

**Tech Stack:** C++17 + pybind11 (`py::call_guard<py::gil_scoped_release>()`), Python 3.10 + NumPy structured array (`.tobytes()`), `rosbag2_py` + `sensor_msgs.PointCloud2` deserialize, `matplotlib` (Agg backend).

---

## Scope (Spec §3 Phase B-1)

| Sub | 항목 ID | 위치 | 변경 |
|-----|--------|------|------|
| **B-1.0** | (신규) | `scripts/regression/` | 회귀 인프라 5종 신규 작성 (Tasks 1~4) |
| B-1.1 | P0-3 | `sonar_3d_reconstruction/cpp/python_bindings.cpp:147-241` | 무거운 `.def`에 `py::call_guard<py::gil_scoped_release>()` 추가 (Task 5) |
| B-1.2 | P1-2 | `scripts/3d_mapper_node.py:728-735`, `map_visualizer_node.py:422-429`, `map_diff_visualizer.py:202-206` | `struct.pack` 루프 → numpy structured `.tobytes()` (Task 6) |
| B-1.3 | P1-4 | `scripts/3d_mapper.py:222-228, 565-569, 677-681` | first-hit `for + enumerate` → `np.argmax(mask)` (Task 7) |

## 검증 임계 (Spec §4.4 — Phase B-1)

| Metric | 임계 | 의미 |
|--------|------|------|
| `jaccard_set` | **= 1.0** | bit-exact voxel 동일성 |
| `mean_log_odds_diff` | **= 0.0** | log-odds 동일성 |
| `avg_proc_time_ms` | **≤ baseline** | 처리량 회귀 없음 (개선 기대) |

P-1(m750d) + P-2(m3000d) 두 데이터셋 모두 통과해야 머지.

---

## File Structure

### 신규 (B-1.0 회귀 인프라 — `src/sonar_3d_reconstruction/scripts/regression/`)

| 파일 | 책임 |
|------|------|
| `regression_metric.py` | PointCloud2 bag → voxel set / log-odds dict 추출 + Jaccard·diff·proc-time 계산 (라이브러리, import 전용) |
| `regression_test.sh` | bag replay + mapping launch + topic record (mode: `baseline | candidate | compare`) |
| `regression_compare.py` | 두 bag 입력 → metric 표 출력 + 임계 판정 (exit code 0=PASS, 1=FAIL) |
| `regression_plot.py` | xy/xz 단면 plot + scatter overlay 저장 |
| `README.md` | 사용법 |

### 수정 (B-1.1~1.3 surgical perf)

| 파일 | 라인 | 변경 |
|------|------|------|
| `sonar_3d_reconstruction/cpp/python_bindings.cpp` | 147-241 (그리고 영향 큰 다른 .def) | 무거운 메서드 .def에 `py::call_guard<py::gil_scoped_release>()` 추가 |
| `scripts/3d_mapper_node.py` | 728-735 | numpy structured `.tobytes()` |
| `scripts/map_visualizer_node.py` | 422-429 | 동일 |
| `scripts/map_diff_visualizer.py` | 202-206 | 동일 (RGB 처리 포함) |
| `scripts/3d_mapper.py` | 222-228, 565-569, 677-681 | `np.argmax(mask)` |
| `CHANGELOG.md` | top | Phase B-1 항목 |

### 신규 단위 테스트 (`tests/regression/`)

| 파일 | 검증 대상 |
|------|----------|
| `tests/regression/test_metric.py` | `regression_metric.py`의 jaccard / log-odds diff |
| `tests/test_pc2_pack.py` | numpy `.tobytes()` 가 `struct.pack` 루프와 byte-equivalent |
| `tests/test_first_hit.py` | `np.argmax(mask)` 가 `for + enumerate + break`와 동일한 인덱스 |

---

## Branch + 빌드 환경

**작업 worktree:** `/workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction` (격리됨, 메인 worktree와 독립)

**Branch:** Phase A 머지 후 `main` 위에 새 branch:
```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git fetch origin
git checkout -b refactor/phase-b1-perf-surgical origin/main
```
(Phase A가 아직 안 머지됐으면 `refactor/phase-a-cleanup` 위에서 시작 후 phase A 머지 시 rebase)

**빌드 명령** (메모리 `project_sonar3d_audit_state.md` 그대로):
```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Tasks

### Task 0: 격리 worktree 환경 검증 (smoke)

**Files:** (없음 — 환경 점검만)

- [ ] **Step 1: 격리 worktree 위치/브랜치 확인**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git branch --show-current
git worktree list
```
Expected:
- branch: `refactor/phase-a-cleanup` (Phase A 직후) 또는 `refactor/phase-b1-perf-surgical`
- worktree list에 `/workspace/ros2_ws_phase_a/...` 와 `/workspace/ros2_ws/...` 두 항목

- [ ] **Step 2: Release 빌드 PASS 확인**

```bash
cd /workspace/ros2_ws_phase_a
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: `Finished <<< sonar_3d_reconstruction [...]` (오류 없이 완료)

- [ ] **Step 3: import smoke 테스트**

```bash
source /workspace/ros2_ws_phase_a/install/setup.bash
python3 -c "
from sonar_3d_reconstruction.cpp_module import ProbabilityUpdater, OutofcoreTileMapper
print('imports OK', ProbabilityUpdater.__name__, OutofcoreTileMapper.__name__)
"
```
Expected: `imports OK ProbabilityUpdater OutofcoreTileMapper`

- [ ] **Step 4: 데이터셋 존재 확인**

```bash
ls /workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1/*.db3
ls /workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m3000d_blueboat/m3000d-range15-tilt90/*.db3
```
Expected: 두 줄 모두 db3 파일 경로 출력

빌드/import/데이터셋 중 하나라도 실패하면 STOP하고 사용자에게 보고.

---

### Task 1: regression_metric.py (회귀 메트릭 라이브러리)

**Files:**
- Create: `scripts/regression/__init__.py` (empty)
- Create: `scripts/regression/regression_metric.py`
- Create: `tests/regression/__init__.py` (empty)
- Create: `tests/regression/test_metric.py`

- [ ] **Step 1: 디렉토리 생성 + `__init__.py`**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
mkdir -p scripts/regression tests/regression
touch scripts/regression/__init__.py tests/regression/__init__.py
```

- [ ] **Step 2: 실패 테스트 작성 (`tests/regression/test_metric.py`)**

```python
"""Unit tests for regression_metric library."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

# 패키지 안 모듈 임포트 (colcon에서 install 안되더라도 직접 경로로 접근 가능)
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from regression import regression_metric as rm


def test_voxelize_quantizes_to_grid():
    points = np.array(
        [[0.001, 0.0, 0.0], [0.049, 0.0, 0.0], [0.051, 0.0, 0.0]],
        dtype=np.float64,
    )
    keys = rm.voxelize(points, resolution=0.05)
    assert keys.shape == (3, 3)
    # 0.001 과 0.049는 같은 voxel (0,0,0), 0.051은 (1,0,0)
    assert tuple(keys[0]) == tuple(keys[1])
    assert tuple(keys[2]) == (1, 0, 0)


def test_jaccard_identical_sets_is_one():
    a = {(0, 0, 0), (1, 0, 0), (2, 0, 0)}
    assert rm.jaccard(a, a) == 1.0


def test_jaccard_disjoint_sets_is_zero():
    a = {(0, 0, 0)}
    b = {(10, 10, 10)}
    assert rm.jaccard(a, b) == 0.0


def test_jaccard_partial_overlap():
    a = {(0, 0, 0), (1, 0, 0)}
    b = {(0, 0, 0), (2, 0, 0)}
    # |A ∩ B| = 1, |A ∪ B| = 3
    assert rm.jaccard(a, b) == pytest.approx(1.0 / 3.0)


def test_log_odds_diff_zero_for_identical():
    keys = np.array([[0, 0, 0], [1, 0, 0]], dtype=np.int64)
    probs = np.array([0.7, 0.9], dtype=np.float64)
    diff = rm.log_odds_diff(keys, probs, keys, probs)
    assert diff["common_count"] == 2
    assert diff["mean_diff"] == 0.0
    assert diff["max_diff"] == 0.0


def test_log_odds_diff_disjoint_zero_common():
    keys_a = np.array([[0, 0, 0]], dtype=np.int64)
    keys_b = np.array([[1, 0, 0]], dtype=np.int64)
    probs = np.array([0.7], dtype=np.float64)
    diff = rm.log_odds_diff(keys_a, probs, keys_b, probs)
    assert diff["common_count"] == 0
    assert math.isnan(diff["mean_diff"])
```

- [ ] **Step 3: 테스트 실행 → FAIL 확인**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
python3 -m pytest tests/regression/test_metric.py -v
```
Expected: `ModuleNotFoundError: No module named 'regression.regression_metric'` 또는 `ImportError`

- [ ] **Step 4: `regression_metric.py` 작성**

```python
"""
Sonar 3D Reconstruction — Regression Metric Library

PointCloud2 bag 두 개를 voxel set + log-odds dict로 환산한 뒤,
Jaccard / log-odds diff / processing time metric 을 계산한다.

Phase B-1.0에서 도입. 후속 phase (B-2/C/D) 에서 동일 라이브러리 재사용.
"""
from __future__ import annotations

import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Optional, Tuple

import numpy as np

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
except ImportError as exc:
    raise ImportError(
        "regression_metric requires ROS2 humble in PATH. "
        "Source /opt/ros/humble/setup.bash before importing."
    ) from exc


VOXEL_RESOLUTION_DEFAULT = 0.05  # m, common.yaml 의 voxel_resolution 기본값과 일치


# ---------------------------------------------------------------------------
# Voxelization helpers
# ---------------------------------------------------------------------------

def voxelize(points: np.ndarray, resolution: float = VOXEL_RESOLUTION_DEFAULT) -> np.ndarray:
    """(N,3) float points → (N,3) int voxel keys (floor 양자화).

    Voxel key 정의: floor(coord / resolution). 음수 좌표 처리 일관성 유지.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be (N,3), got {points.shape}")
    return np.floor(points / resolution).astype(np.int64)


def jaccard(set_a: set, set_b: set) -> float:
    """|A ∩ B| / |A ∪ B|. 둘 다 공집합이면 1.0 (정의)."""
    if not set_a and not set_b:
        return 1.0
    inter = len(set_a & set_b)
    union = len(set_a | set_b)
    if union == 0:
        return 1.0
    return inter / union


# ---------------------------------------------------------------------------
# log-odds 환산 (probability ↔ log-odds)
# ---------------------------------------------------------------------------

_PROB_EPS = 1e-9


def prob_to_log_odds(p: np.ndarray) -> np.ndarray:
    """확률 → log-odds. p∈(0,1) 클램핑."""
    p_clamped = np.clip(p, _PROB_EPS, 1.0 - _PROB_EPS)
    return np.log(p_clamped / (1.0 - p_clamped))


def log_odds_diff(
    keys_a: np.ndarray,
    probs_a: np.ndarray,
    keys_b: np.ndarray,
    probs_b: np.ndarray,
) -> Dict[str, float]:
    """공통 voxel 상의 |L_a - L_b| 통계.

    keys_*: (N,3) int64, probs_*: (N,) float64.
    반환: {common_count, mean_diff, max_diff}. 공통이 없으면 NaN.
    """
    log_a = prob_to_log_odds(probs_a)
    log_b = prob_to_log_odds(probs_b)

    map_a = {tuple(k): float(l) for k, l in zip(keys_a, log_a)}
    map_b = {tuple(k): float(l) for k, l in zip(keys_b, log_b)}

    common = set(map_a.keys()) & set(map_b.keys())
    if not common:
        return {"common_count": 0, "mean_diff": math.nan, "max_diff": math.nan}

    diffs = np.fromiter(
        (abs(map_a[k] - map_b[k]) for k in common),
        dtype=np.float64,
        count=len(common),
    )
    return {
        "common_count": len(common),
        "mean_diff": float(diffs.mean()),
        "max_diff": float(diffs.max()),
    }


# ---------------------------------------------------------------------------
# PointCloud2 → (points, intensities)
# ---------------------------------------------------------------------------

@dataclass
class PointCloudFrame:
    stamp_ns: int
    points: np.ndarray  # (N,3) float64
    intensities: np.ndarray  # (N,) float64 (occupied probability)


def _decode_pointcloud2(msg: "PointCloud2") -> PointCloudFrame:
    """sensor_msgs/PointCloud2 (x,y,z,intensity float32) → PointCloudFrame.

    x/y/z/intensity 4 필드, point_step=16 가정 (3d_mapper_node 규약).
    """
    if msg.point_step != 16:
        raise ValueError(f"unsupported point_step={msg.point_step} (expected 16)")
    n = msg.width * msg.height
    if n == 0:
        empty = np.zeros((0, 3), dtype=np.float64)
        return PointCloudFrame(
            stamp_ns=msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec,
            points=empty,
            intensities=np.zeros((0,), dtype=np.float64),
        )
    raw = np.frombuffer(msg.data, dtype=np.float32, count=n * 4).reshape(n, 4)
    points = raw[:, :3].astype(np.float64)
    intensities = raw[:, 3].astype(np.float64)
    stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    return PointCloudFrame(stamp_ns=stamp_ns, points=points, intensities=intensities)


def read_last_pointcloud(bag_dir: Path, topic: str) -> Optional[PointCloudFrame]:
    """주어진 bag 디렉토리에서 topic의 마지막 PointCloud2 메시지를 읽어 PointCloudFrame 반환.
    None: 메시지 없음."""
    storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage, converter)

    last: Optional[PointCloudFrame] = None
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        last = _decode_pointcloud2(msg)
    return last


# ---------------------------------------------------------------------------
# processing time / drop rate (선택)
# ---------------------------------------------------------------------------

def read_processing_stats(bag_dir: Path, topic: str = "/pkrc/sonar/processing_time") -> Dict[str, float]:
    """노드가 publish하는 std_msgs/Float64MultiArray 또는 std_msgs/Float64 통계 토픽이 있다면 평균 산출.
    토픽이 없으면 빈 dict 반환."""
    try:
        from std_msgs.msg import Float64
    except ImportError:
        return {}

    storage = StorageOptions(uri=str(bag_dir), storage_id="sqlite3")
    converter = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = SequentialReader()
    reader.open(storage, converter)

    samples = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        try:
            msg = deserialize_message(data, Float64)
            samples.append(float(msg.data))
        except Exception:
            continue

    if not samples:
        return {}
    arr = np.asarray(samples, dtype=np.float64)
    return {
        "n_samples": int(arr.size),
        "avg_proc_time_ms": float(arr.mean() * 1000.0),
        "max_proc_time_ms": float(arr.max() * 1000.0),
    }
```

- [ ] **Step 5: 테스트 실행 → PASS 확인**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
source /opt/ros/humble/setup.bash
python3 -m pytest tests/regression/test_metric.py -v
```
Expected: `6 passed`

- [ ] **Step 6: Commit**

```bash
git add scripts/regression/__init__.py scripts/regression/regression_metric.py \
        tests/regression/__init__.py tests/regression/test_metric.py
git commit -m "$(cat <<'EOF'
feat(regression): add metric library for B-1.0 infrastructure

Phase B-1.0 의 회귀 인프라 1단계 — voxelize / jaccard / log_odds_diff /
PointCloud2 deserialize 함수 라이브러리. 후속 phase 가 동일 import 로 재사용.

- scripts/regression/regression_metric.py: voxel/jaccard/log-odds API
- tests/regression/test_metric.py: 6 단위 테스트 PASS
EOF
)"
```

---

### Task 2: regression_test.sh (orchestrator)

**Files:**
- Create: `scripts/regression/regression_test.sh`

- [ ] **Step 1: 스크립트 작성**

```bash
cat > /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction/scripts/regression/regression_test.sh <<'BASH_EOF'
#!/usr/bin/env bash
#
# sonar_3d_reconstruction 회귀 테스트 orchestrator
#
# 동일 bag 을 두 빌드(baseline = main HEAD, candidate = 현재 branch)에서
# 재생하면서 /pkrc/sonar/cpp_pointcloud 토픽을 record. 종료 후 마지막 메시지를
# voxel set 으로 환산하여 Jaccard / log-odds diff / proc-time 비교.
#
# 사용:
#   bash regression_test.sh baseline   # main HEAD 빌드 측정 (먼저 수동 checkout+build)
#   bash regression_test.sh candidate  # 현재 branch 빌드 측정
#   bash regression_test.sh compare    # 두 결과 비교 + 임계 판정
#
# 환경 변수로 데이터셋·duration 변경 가능:
#   BAG_PATH    (기본: P-1 = m750d-range15-tilt45-v1)
#   PLAY_DURATION   (기본: 90초)
#   OUT_DIR     (기본: /tmp/sonar3d_regression)
#   LAUNCH_FILE (기본: 3d_mapping.launch.py)
#
# Phase B-1.0 도입.

set -euo pipefail

BAG_PATH="${BAG_PATH:-/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1}"
PLAY_DURATION="${PLAY_DURATION:-90}"
OUT_DIR="${OUT_DIR:-/tmp/sonar3d_regression}"
LAUNCH_PKG="${LAUNCH_PKG:-sonar_3d_reconstruction}"
LAUNCH_FILE="${LAUNCH_FILE:-3d_mapping.launch.py}"
SLAM_LAUNCH_PKG="${SLAM_LAUNCH_PKG:-fast_lio}"
SLAM_LAUNCH_FILE="${SLAM_LAUNCH_FILE:-mapping.launch.py}"
PC_TOPIC="${PC_TOPIC:-/pkrc/sonar/cpp_pointcloud}"

# 임계 (Spec §4.4 — Phase B-1: bit-exact)
JACCARD_THRESHOLD="${JACCARD_THRESHOLD:-1.0}"
MEAN_LOG_ODDS_THRESHOLD="${MEAN_LOG_ODDS_THRESHOLD:-0.0}"

run_replay() {
    local label="$1"
    local out="${OUT_DIR}/${label}"

    rm -rf "${out}"
    mkdir -p "${out}"

    echo "[regression] (${label}) launching fast_lio mapping ..."
    ros2 launch "${SLAM_LAUNCH_PKG}" "${SLAM_LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false foxglove:=false \
        > "${out}/slam_launch.log" 2>&1 &
    SLAM_PID=$!
    sleep 5

    echo "[regression] (${label}) launching sonar 3d mapping ..."
    ros2 launch "${LAUNCH_PKG}" "${LAUNCH_FILE}" \
        use_sim_time:=true rviz:=false \
        > "${out}/sonar_launch.log" 2>&1 &
    SONAR_PID=$!
    sleep 8

    echo "[regression] (${label}) recording ${PC_TOPIC} ..."
    ros2 bag record -s sqlite3 -o "${out}/cloud" "${PC_TOPIC}" \
        > "${out}/record.log" 2>&1 &
    RECORD_PID=$!
    sleep 1

    echo "[regression] (${label}) playing bag (first ${PLAY_DURATION}s) ..."
    timeout "$((PLAY_DURATION + 10))" \
        ros2 bag play "${BAG_PATH}" --clock --rate 1.0 \
        > "${out}/play.log" 2>&1 || true

    sleep 5  # 마지막 메시지 drain
    kill -INT "${RECORD_PID}" 2>/dev/null || true
    kill -INT "${SONAR_PID}" 2>/dev/null || true
    kill -INT "${SLAM_PID}" 2>/dev/null || true
    wait 2>/dev/null || true

    echo "[regression] (${label}) saved → ${out}/cloud"
}

compare_results() {
    local b="${OUT_DIR}/baseline/cloud"
    local c="${OUT_DIR}/candidate/cloud"
    [[ -d "${b}" ]] || { echo "[regression] missing baseline at ${b}"; exit 1; }
    [[ -d "${c}" ]] || { echo "[regression] missing candidate at ${c}"; exit 1; }
    python3 "$(dirname "$0")/regression_compare.py" \
        --baseline "${b}" \
        --candidate "${c}" \
        --topic "${PC_TOPIC}" \
        --jaccard-threshold "${JACCARD_THRESHOLD}" \
        --mean-log-odds-threshold "${MEAN_LOG_ODDS_THRESHOLD}"
}

mode="${1:-}"
case "${mode}" in
    baseline)  run_replay baseline ;;
    candidate) run_replay candidate ;;
    compare)   compare_results ;;
    *)
        echo "usage: $0 {baseline|candidate|compare}"
        echo ""
        echo "Typical flow (Phase B-1):"
        echo "  # 1. main HEAD 빌드"
        echo "  cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction"
        echo "  git stash && git checkout main"
        echo "  cd /workspace/ros2_ws_phase_a"
        echo "  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release"
        echo "  source install/setup.bash"
        echo "  bash $(realpath $0) baseline"
        echo ""
        echo "  # 2. candidate (refactor/phase-b1-perf-surgical) 빌드"
        echo "  cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction"
        echo "  git checkout refactor/phase-b1-perf-surgical && git stash pop"
        echo "  cd /workspace/ros2_ws_phase_a"
        echo "  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release"
        echo "  source install/setup.bash"
        echo "  bash $(realpath $0) candidate"
        echo ""
        echo "  # 3. 비교 + plot"
        echo "  bash $(realpath $0) compare"
        echo "  python3 $(dirname $(realpath $0))/regression_plot.py"
        exit 1
        ;;
esac
BASH_EOF
chmod +x /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction/scripts/regression/regression_test.sh
```

- [ ] **Step 2: shellcheck (선택, 가능하면)**

```bash
which shellcheck && \
    shellcheck /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction/scripts/regression/regression_test.sh \
    || echo "[skip] shellcheck not installed"
```
Expected: 빈 출력(검증 통과) 또는 `[skip] shellcheck not installed`

- [ ] **Step 3: usage 출력 smoke**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
bash scripts/regression/regression_test.sh
```
Expected: `usage: ... {baseline|candidate|compare}` 시작하는 도움말 출력 + exit 1

- [ ] **Step 4: Commit**

```bash
git add scripts/regression/regression_test.sh
git commit -m "feat(regression): add bag-replay orchestrator script"
```

---

### Task 3: regression_compare.py + regression_plot.py

**Files:**
- Create: `scripts/regression/regression_compare.py`
- Create: `scripts/regression/regression_plot.py`

- [ ] **Step 1: `regression_compare.py` 작성**

```python
#!/usr/bin/env python3
"""
sonar_3d_reconstruction regression comparison.

baseline / candidate bag(`/pkrc/sonar/cpp_pointcloud`) 의 마지막 PointCloud2 를
voxel set + log-odds 분포로 환산하여 Spec §4.4 의 임계와 비교한다.

Phase B-1: jaccard==1.0, mean_log_odds==0.0, avg_proc_time<=baseline (옵션).
임계 미달 시 exit code 1 → CI/PR 차단 신호.
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from regression import regression_metric as rm  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="sonar3d regression compare")
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--topic", default="/pkrc/sonar/cpp_pointcloud")
    parser.add_argument("--resolution", type=float, default=rm.VOXEL_RESOLUTION_DEFAULT)
    parser.add_argument("--jaccard-threshold", type=float, default=1.0)
    parser.add_argument("--mean-log-odds-threshold", type=float, default=0.0)
    parser.add_argument("--proc-time-topic", default="/pkrc/sonar/processing_time")
    args = parser.parse_args()

    base = rm.read_last_pointcloud(args.baseline, args.topic)
    cand = rm.read_last_pointcloud(args.candidate, args.topic)

    if base is None or cand is None:
        print(f"[compare] FAIL: missing PointCloud2 (baseline={base}, candidate={cand})")
        return 1

    keys_b = rm.voxelize(base.points, args.resolution)
    keys_c = rm.voxelize(cand.points, args.resolution)

    set_b = {tuple(k) for k in keys_b}
    set_c = {tuple(k) for k in keys_c}
    j = rm.jaccard(set_b, set_c)

    diff = rm.log_odds_diff(keys_b, base.intensities, keys_c, cand.intensities)

    proc_b = rm.read_processing_stats(args.baseline, args.proc_time_topic)
    proc_c = rm.read_processing_stats(args.candidate, args.proc_time_topic)

    print("=" * 64)
    print(f" baseline:  {len(set_b):>7d} occupied voxels (last frame)")
    print(f" candidate: {len(set_c):>7d} occupied voxels (last frame)")
    print(f" jaccard_set:        {j:.6f} (threshold >= {args.jaccard_threshold})")
    if diff["common_count"] > 0:
        print(f" mean_log_odds_diff: {diff['mean_diff']:.6f} (threshold <= {args.mean_log_odds_threshold})")
        print(f" max_log_odds_diff:  {diff['max_diff']:.6f}")
    else:
        print(" mean_log_odds_diff: NaN (no common voxels)")
    if proc_b and proc_c:
        print(
            f" avg_proc_time_ms:   {proc_b['avg_proc_time_ms']:.2f} → "
            f"{proc_c['avg_proc_time_ms']:.2f} "
            f"(Δ {proc_c['avg_proc_time_ms']-proc_b['avg_proc_time_ms']:+.2f})"
        )
    print("=" * 64)

    j_pass = j >= args.jaccard_threshold
    diff_pass = (
        diff["common_count"] > 0
        and diff["mean_diff"] <= args.mean_log_odds_threshold + 1e-9
    )
    overall = j_pass and diff_pass
    print(f"[compare] {'PASS' if overall else 'FAIL'} "
          f"(jaccard={j_pass}, log_odds_diff={diff_pass})")
    return 0 if overall else 1


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: `regression_plot.py` 작성**

```python
#!/usr/bin/env python3
"""
sonar_3d_reconstruction regression visualization.

baseline / candidate 의 마지막 PointCloud2 를 xy / xz 단면으로 overlay 시각화.
Phase B-1 임계 통과 후에도 시각 비교를 PR 본문에 첨부하기 위한 산출물.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from regression import regression_metric as rm  # noqa: E402

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError:
    sys.exit("[plot] matplotlib missing. apt install python3-matplotlib")


def main() -> int:
    parser = argparse.ArgumentParser(description="sonar3d regression plot")
    parser.add_argument("--baseline", default="/tmp/sonar3d_regression/baseline/cloud", type=Path)
    parser.add_argument("--candidate", default="/tmp/sonar3d_regression/candidate/cloud", type=Path)
    parser.add_argument("--topic", default="/pkrc/sonar/cpp_pointcloud")
    parser.add_argument("--out", default="/tmp/sonar3d_regression/comparison.png", type=Path)
    args = parser.parse_args()

    base = rm.read_last_pointcloud(args.baseline, args.topic)
    cand = rm.read_last_pointcloud(args.candidate, args.topic)
    if base is None or cand is None:
        print(f"[plot] missing baseline/candidate (got {base}, {cand})")
        return 1

    pb, pc = base.points, cand.points
    if pb.size == 0 and pc.size == 0:
        print("[plot] both empty")
        return 1

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    ax = axes[0]
    if pb.size:
        ax.scatter(pb[:, 0], pb[:, 1], s=1.0, c="tab:blue", alpha=0.4, label=f"baseline ({len(pb)})")
    if pc.size:
        ax.scatter(pc[:, 0], pc[:, 1], s=1.0, c="tab:orange", alpha=0.4, label=f"candidate ({len(pc)})")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY top-down (last PointCloud2)")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend()

    ax = axes[1]
    if pb.size:
        ax.scatter(pb[:, 0], pb[:, 2], s=1.0, c="tab:blue", alpha=0.4, label="baseline")
    if pc.size:
        ax.scatter(pc[:, 0], pc[:, 2], s=1.0, c="tab:orange", alpha=0.4, label="candidate")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title("XZ side view")
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.suptitle(
        f"sonar3d regression — Phase B-1\n"
        f"baseline={len(pb)} pts, candidate={len(pc)} pts",
        fontsize=12,
    )
    plt.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(args.out, dpi=120)
    print(f"[plot] saved {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: 실행 권한**

```bash
chmod +x /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction/scripts/regression/regression_compare.py
chmod +x /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction/scripts/regression/regression_plot.py
```

- [ ] **Step 4: import smoke (베이스라인 측정 전)**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
source /opt/ros/humble/setup.bash
python3 -c "
import sys; sys.path.insert(0, 'scripts')
from regression import regression_metric, regression_compare, regression_plot
print('imports OK')
"
```
Expected: `imports OK`

- [ ] **Step 5: Commit**

```bash
git add scripts/regression/regression_compare.py scripts/regression/regression_plot.py
git commit -m "feat(regression): add compare + plot driver scripts"
```

---

### Task 4: README + B-1.0 baseline 측정 (P-1 + P-2)

**Files:**
- Create: `scripts/regression/README.md`

- [ ] **Step 1: README 작성**

```markdown
# sonar_3d_reconstruction Regression Infrastructure

> Phase B-1.0 에 도입. 후속 phase (B-2 / B-3 / C / D) 모두 동일 스크립트 재사용.

## 목적

`refactor/phase-*` PR 머지 전 **알고리즘 회귀가 0인지** 정량 검증한다. Spec
`docs/source/design/2026-05-03-quality-perf-uplift-design.md` §4 가 임계와 절차를
정의한다.

## 구성

| 파일 | 책임 |
|------|------|
| `regression_metric.py` | PointCloud2 → voxel set / log-odds dict / Jaccard·diff 계산 (라이브러리) |
| `regression_test.sh` | bag replay + mapping launch + topic record (`baseline | candidate | compare`) |
| `regression_compare.py` | 두 bag 비교 + 임계 판정 |
| `regression_plot.py` | xy/xz 단면 plot 생성 |

## 사용법 (Phase B-1)

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws_phase_a/install/setup.bash

# 1. baseline (main HEAD 빌드 후)
git checkout main
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh baseline

# 2. candidate (현재 branch 빌드 후)
cd src/sonar_3d_reconstruction && git checkout refactor/phase-b1-perf-surgical
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh candidate

# 3. 비교 + plot
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh compare
python3 src/sonar_3d_reconstruction/scripts/regression/regression_plot.py
```

## 임계값 (Spec §4.4)

| Phase | jaccard_set | mean_log_odds_diff | avg_proc_time |
|-------|-------------|-------------------|---------------|
| B-1 | = 1.0 | = 0.0 | ≤ baseline |
| B-2 | ≥ 0.95 | 의도된 변화 | ≤ baseline |
| B-3 | ≥ 0.99 | ≤ 0.1 | ≤ baseline |
| C   | ≥ 0.95~0.99 | ≤ 0.5 | ≤ baseline |
| D   | ≥ 0.99 | ≤ 0.1 | ≤ baseline / 1.5~5 |

## 환경 변수

| 변수 | 기본값 | 의미 |
|------|--------|------|
| `BAG_PATH` | P-1 (m750d-range15-tilt45-v1) | 회귀 데이터셋 |
| `PLAY_DURATION` | 90 | 재생 초 |
| `OUT_DIR` | `/tmp/sonar3d_regression` | 산출물 디렉토리 |
| `JACCARD_THRESHOLD` | 1.0 | (Phase별로 변경) |
| `MEAN_LOG_ODDS_THRESHOLD` | 0.0 | (Phase별로 변경) |

## Notes

- 결과(`/tmp/sonar3d_regression/`)는 `.gitignore` 처리. commit 금지.
- plot 이미지는 PR 본문에 attach만 하고 repo에는 commit 금지.
- bag/db3/metadata.yaml 은 절대 삭제·수정하지 않음 (CLAUDE.md 데이터 안전 정책).
```

- [ ] **Step 2: README commit**

```bash
git add scripts/regression/README.md
git commit -m "docs(regression): add README for B-1.0 infrastructure"
```

- [ ] **Step 3: P-1 baseline 측정 (main HEAD 빌드)**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git checkout main
cd /workspace/ros2_ws_phase_a
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
BAG_PATH=/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1 \
OUT_DIR=/tmp/sonar3d_regression/p1 \
PLAY_DURATION=90 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh baseline
```
Expected: `[regression] (baseline) saved → /tmp/sonar3d_regression/p1/baseline/cloud`

- [ ] **Step 4: P-2 baseline 측정**

```bash
BAG_PATH=/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m3000d_blueboat/m3000d-range15-tilt90 \
OUT_DIR=/tmp/sonar3d_regression/p2 \
PLAY_DURATION=90 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh baseline
```
Expected: `[regression] (baseline) saved → /tmp/sonar3d_regression/p2/baseline/cloud`

- [ ] **Step 5: baseline 메타 기록 (참고용 dump)**

```bash
mkdir -p /tmp/sonar3d_regression/_baseline_meta
git -C /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction rev-parse HEAD \
    > /tmp/sonar3d_regression/_baseline_meta/main_sha.txt
date -Iseconds > /tmp/sonar3d_regression/_baseline_meta/measured_at.txt
```
Expected: 두 파일 생성 (commit SHA + ISO timestamp)

- [ ] **Step 6: branch 복귀**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
git checkout refactor/phase-b1-perf-surgical  # 없으면 미리 생성
git status
```
Expected: branch 전환 완료, working tree 깨끗

baseline 측정이 한 데이터셋이라도 빈 PointCloud2 만 record하면 STOP하고 launch 로그 점검:
```bash
ls -la /tmp/sonar3d_regression/p1/baseline/cloud/*.db3
ros2 bag info /tmp/sonar3d_regression/p1/baseline/cloud | head -20
```

---

### Task 5: B-1.1 — pybind11 GIL release (P0-3)

**Files:**
- Modify: `sonar_3d_reconstruction/cpp/python_bindings.cpp:147-241` (그리고 다른 무거운 .def)

**대상 메서드** (모든 호출이 OctoMap mutex / disk I/O / 큰 vector 순회 → GIL 해제 효과 큼):

| .def 라인 | 메서드 | 사유 |
|-----------|--------|------|
| 42 | `OctreeMapper::batch_update` | 큰 numpy 입력 순회 |
| 45 | `OctreeMapper::batch_update_with_log_odds` | 동일 |
| 51 | `OctreeMapper::get_occupied_voxels` | 트리 traversal |
| 56 | `OctreeMapper::prune_tree` | 트리 변형 |
| 116 | `ProbabilityUpdater::batch_update_iwlo` | 핫패스, sonar 콜백 |
| 92 | `ProbabilityUpdater::get_occupied_voxels` | 트리 traversal |
| 122 | `ProbabilityUpdater::force_full_sync` | 전체 sync |
| 147 | `OutofcoreTileMapper::batch_update_iwlo` | 핫패스 |
| 168 | `OutofcoreTileMapper::get_occupied_voxels` | 캐시 traversal |
| 171 | `OutofcoreTileMapper::get_all_occupied_voxels` | 모든 타일 디스크 로드 |
| 174 | `OutofcoreTileMapper::get_occupied_voxels_in_region` | 동일 |
| 184 | `OutofcoreTileMapper::flush_all` | 디스크 쓰기 |
| 186 | `OutofcoreTileMapper::flush_tile` | 디스크 쓰기 |
| 192 | `OutofcoreTileMapper::save_merged_octree` | 디스크 쓰기 |
| 195 | `OutofcoreTileMapper::get_octree_binary` (lambda) | 큰 메모리 직렬화 |
| 214 | `OutofcoreTileMapper::preload_region` | 디스크 로드 |
| 218 | `OutofcoreTileMapper::reload_tiles` | 디스크 로드 |
| 221 | `OutofcoreTileMapper::flush_and_get_dirty_tiles` | 디스크 쓰기 |
| 224 | `OutofcoreTileMapper::prune_all` | 트리 변형 (모든 타일) |
| 227 | `OutofcoreTileMapper::get_and_clear_saved_tiles` | mutex 잠금 |
| 230 | `OutofcoreTileMapper::ray_cast_depth` | 트리 traversal |
| 234 | `OutofcoreTileMapper::batch_ray_cast_depth` | 핫패스 (depth estimation) |
| 238 | `OutofcoreTileMapper::batch_check_occupied` | 트리 lookup 다수 |

가벼운 getter (`get_resolution`/`get_num_nodes`/`get_cached_tile_count`/`get_total_tile_count`/`get_disk_usage` 등 단순 atomic/스칼라 반환)는 **변경 없음** — 호출 비용 〈〈 GIL acquire 비용.

- [ ] **Step 1: 단위 테스트로 의도 보존 명시 (`tests/test_gil_release.py`)**

```python
"""GIL release 후에도 결과가 동일한지 smoke."""
import numpy as np
import pytest

import sys
sys.path.insert(0, "/workspace/ros2_ws_phase_a/install/sonar_3d_reconstruction/lib/python3.10/site-packages")

try:
    from sonar_3d_reconstruction.cpp_module import ProbabilityUpdater
except ImportError:
    pytest.skip("cpp_module not built", allow_module_level=True)


def test_batch_update_iwlo_smoke():
    pu = ProbabilityUpdater(resolution=0.05)
    pu.set_log_odds_params(0.85, -0.4)
    pu.set_intensity_params(120, 255)
    pu.set_iwlo_params(2.5, 0.05, 0.05, -2.0, 3.5)

    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0, 180.0], dtype=np.float64)
    is_occ = np.array([True, True], dtype=bool)

    pu.batch_update_iwlo(points, intensities, is_occ)
    voxels = pu.get_occupied_voxels(0.5)
    # GIL 해제로 결과 의미 변하지 않음 — 동일 입력 동일 출력
    assert voxels.shape[1] == 4  # x,y,z,prob
    assert len(voxels) >= 1


def test_get_resolution_unchanged():
    pu = ProbabilityUpdater(resolution=0.07)
    assert pu.get_resolution() == pytest.approx(0.07)
```

- [ ] **Step 2: 테스트 실행 (현재는 GIL 미해제 상태에서 PASS — baseline 동작 확인)**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws_phase_a/install/setup.bash
python3 -m pytest tests/test_gil_release.py -v
```
Expected: `2 passed`

- [ ] **Step 3: `python_bindings.cpp` 수정**

각 .def 호출에서 docstring 직전에 `py::call_guard<py::gil_scoped_release>()`를 추가한다. 예:

`OutofcoreTileMapper::batch_update_iwlo` (line 147-149):
```cpp
        .def("batch_update_iwlo", &sonar_3d_reconstruction::OutofcoreTileMapper::batch_update_iwlo,
             py::arg("points"), py::arg("intensities"), py::arg("is_occupied"),
             py::call_guard<py::gil_scoped_release>(),
             "Batch update with IWLO method")
```

`OutofcoreTileMapper::batch_ray_cast_depth` (line 234-237):
```cpp
        .def("batch_ray_cast_depth", &sonar_3d_reconstruction::OutofcoreTileMapper::batch_ray_cast_depth,
             py::arg("origin"), py::arg("directions"), py::arg("max_range"),
             py::arg("step_size"), py::arg("min_probability") = 0.7,
             py::call_guard<py::gil_scoped_release>(),
             "Batch ray-cast for multiple directions from a single origin")
```

`OutofcoreTileMapper::get_octree_binary` (line 195-203, lambda):
```cpp
        .def("get_octree_binary", [](sonar_3d_reconstruction::OutofcoreTileMapper& self, double min_probability) {
             auto result = self.get_octree_binary(min_probability);
             return py::make_tuple(
                 py::bytes(reinterpret_cast<const char*>(result.first.data()), result.first.size()),
                 result.second
             );
         }, py::arg("min_probability") = 0.5,
            py::call_guard<py::gil_scoped_release>(),
            "Get octree as binary data (data, tree_id) for ROS octomap_msgs, filtered by probability threshold")
```

위 표의 24개 .def 모두에 동일 패턴(`py::arg(...)` 마지막 다음 줄에 `py::call_guard<py::gil_scoped_release>(),`).

**주의 (lambda + GIL release):** `get_octree_binary`의 람다는 `py::bytes` 를 생성하는데 이는 GIL 보호 하에서 일어나야 한다. `call_guard`는 람다 진입 시 release / 종료 시 acquire 하므로 람다 본문 내 `py::bytes` 생성은 acquire 후 일어남 → 안전. **다만, OctoMap raw octree 직렬화 부분(`self.get_octree_binary(...)`)이 GIL 없이 실행되어 큰 효과**.

- [ ] **Step 4: 빌드 PASS**

```bash
cd /workspace/ros2_ws_phase_a
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: `Finished <<< sonar_3d_reconstruction [...]`

- [ ] **Step 5: 단위 테스트 PASS 재확인**

```bash
source /workspace/ros2_ws_phase_a/install/setup.bash
python3 -m pytest tests/test_gil_release.py -v
```
Expected: `2 passed`

- [ ] **Step 6: Commit**

```bash
git add sonar_3d_reconstruction/cpp/python_bindings.cpp tests/test_gil_release.py
git commit -m "$(cat <<'EOF'
perf(bindings): release GIL on heavy C++ methods (P0-3)

24개 무거운 .def (batch_update_iwlo / get_*_voxels / ray_cast / flush_* /
prune_* / save_* / preload_region / reload_tiles 등) 에
py::call_guard<py::gil_scoped_release>() 추가.

GIL 해제로 sonar 콜백 처리 중 다른 ROS 콜백(odom/visualizer)이 동시에
Python 코드를 실행 가능. 결과는 bit-exact (입력 동일 → 출력 동일).

- sonar_3d_reconstruction/cpp/python_bindings.cpp: 24개 .def 수정
- tests/test_gil_release.py: smoke 2개 PASS
- 가벼운 getter 는 변경 없음 (효과 미미)
EOF
)"
```

---

### Task 6: B-1.2 — PointCloud2 numpy structured tobytes (P1-2, 3 files)

**Files:**
- Modify: `scripts/3d_mapper_node.py:728-735`
- Modify: `scripts/map_visualizer_node.py:422-429`
- Modify: `scripts/map_diff_visualizer.py:202-206` (RGB 변형)
- Test: `tests/test_pc2_pack.py`

- [ ] **Step 1: byte-equivalence 단위 테스트 (`tests/test_pc2_pack.py`)**

```python
"""numpy structured tobytes 가 struct.pack 루프와 byte-equivalent 검증."""
import struct

import numpy as np


def pack_struct_loop_xyzi(points: np.ndarray, intensities: np.ndarray) -> bytes:
    """Phase A 이전 직렬화 (참조 구현)."""
    out = []
    for i in range(len(points)):
        out.append(
            struct.pack(
                "ffff", points[i, 0], points[i, 1], points[i, 2], intensities[i]
            )
        )
    return b"".join(out)


def pack_numpy_xyzi(points: np.ndarray, intensities: np.ndarray) -> bytes:
    """Phase B-1.2 새 직렬화."""
    n = len(points)
    arr = np.empty(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")]))
    arr["x"] = points[:, 0]
    arr["y"] = points[:, 1]
    arr["z"] = points[:, 2]
    arr["i"] = intensities
    return arr.tobytes()


def pack_struct_loop_xyzrgb(points: np.ndarray, rgb_uint32: int) -> bytes:
    out = []
    for p in points:
        out.append(struct.pack("fffI", p[0], p[1], p[2], rgb_uint32))
    return b"".join(out)


def pack_numpy_xyzrgb(points: np.ndarray, rgb_uint32: int) -> bytes:
    n = len(points)
    arr = np.empty(n, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]))
    arr["x"] = points[:, 0]
    arr["y"] = points[:, 1]
    arr["z"] = points[:, 2]
    arr["rgb"] = np.uint32(rgb_uint32)
    return arr.tobytes()


def test_xyzi_byte_equivalent():
    rng = np.random.default_rng(42)
    pts = rng.standard_normal((100, 3))
    inten = rng.standard_normal(100)
    assert pack_struct_loop_xyzi(pts, inten) == pack_numpy_xyzi(pts, inten)


def test_xyzi_empty_byte_equivalent():
    pts = np.zeros((0, 3))
    inten = np.zeros(0)
    assert pack_struct_loop_xyzi(pts, inten) == pack_numpy_xyzi(pts, inten)


def test_xyzrgb_byte_equivalent():
    rng = np.random.default_rng(123)
    pts = rng.standard_normal((50, 3))
    rgb = struct.unpack("I", struct.pack("BBBB", 100, 100, 255, 255))[0]
    assert pack_struct_loop_xyzrgb(pts, rgb) == pack_numpy_xyzrgb(pts, rgb)
```

- [ ] **Step 2: 테스트 실행 → 3 PASS (참조와 새 구현 동등성)**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
python3 -m pytest tests/test_pc2_pack.py -v
```
Expected: `3 passed`

- [ ] **Step 3: `scripts/3d_mapper_node.py:728-735` 수정**

기존 (728-735):
```python
        # Pack data
        data = []
        for i in range(len(points)):
            data.append(struct.pack('ffff',
                                   points[i, 0], points[i, 1], points[i, 2],
                                   probabilities[i]))
        
        cloud.data = b''.join(data)
```

새 코드:
```python
        # Pack data — numpy structured array (.tobytes()) 로 일괄 직렬화
        # Phase B-1.2 (P1-2): struct.pack 루프 제거. byte-equivalent 검증은
        # tests/test_pc2_pack.py 참조.
        n = len(points)
        packed = np.empty(
            n,
            dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")]),
        )
        packed["x"] = points[:, 0]
        packed["y"] = points[:, 1]
        packed["z"] = points[:, 2]
        packed["i"] = probabilities
        cloud.data = packed.tobytes()
```

- [ ] **Step 4: `scripts/map_visualizer_node.py:422-429` 수정**

기존 (422-429):
```python
        # Pack data
        data = []
        for i in range(len(voxels)):
            data.append(struct.pack('ffff',
                                   voxels[i, 0], voxels[i, 1], voxels[i, 2],
                                   voxels[i, 3]))

        cloud.data = b''.join(data)
```

새 코드:
```python
        # Pack data — numpy structured array (.tobytes())
        # Phase B-1.2 (P1-2)
        n = len(voxels)
        packed = np.empty(
            n,
            dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("i", "<f4")]),
        )
        packed["x"] = voxels[:, 0]
        packed["y"] = voxels[:, 1]
        packed["z"] = voxels[:, 2]
        packed["i"] = voxels[:, 3]
        cloud.data = packed.tobytes()
```

- [ ] **Step 5: `scripts/map_diff_visualizer.py:202-206` 수정**

기존 (199-206):
```python
        # Pack RGB into single float
        rgb_packed = struct.unpack('f', struct.pack('BBBB', b, g, r, 255))[0]

        # Create point data
        cloud_data = []
        for p in points:
            cloud_data.append(struct.pack('fffI', p[0], p[1], p[2],
                                         struct.unpack('I', struct.pack('f', rgb_packed))[0]))
```

새 코드:
```python
        # Phase B-1.2 (P1-2): numpy structured array. RGB는 (b,g,r,255) 4 바이트를
        # uint32 little-endian 으로 직접 저장. struct.pack/unpack 라운드트립 제거.
        rgb_uint32 = (255 << 24) | (r << 16) | (g << 8) | b

        n = len(points)
        cloud_arr = np.empty(
            n,
            dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")]),
        )
        cloud_arr["x"] = points[:, 0]
        cloud_arr["y"] = points[:, 1]
        cloud_arr["z"] = points[:, 2]
        cloud_arr["rgb"] = np.uint32(rgb_uint32)
```

그리고 그 아래 `msg.data = b''.join(cloud_data)` (라인 ~221) 를 다음으로 변경:
```python
        msg.data = cloud_arr.tobytes()
```

- [ ] **Step 6: 빌드 PASS**

```bash
cd /workspace/ros2_ws_phase_a
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: `Finished <<< sonar_3d_reconstruction`

- [ ] **Step 7: import smoke (3 노드 모두)**

```bash
source /workspace/ros2_ws_phase_a/install/setup.bash
python3 -c "
from sonar_3d_reconstruction.scripts import (
    3d_mapper_node as a,
    map_visualizer_node as b,
    map_diff_visualizer as c,
)
" 2>&1 | head
# 위 import는 잘못된 syntax (모듈명 시작 숫자) — 대신 직접 임포트:
python3 -c "
import importlib, importlib.util
for mod in ['3d_mapper_node', 'map_visualizer_node', 'map_diff_visualizer']:
    path = f'/workspace/ros2_ws_phase_a/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction/{mod}'
    spec = importlib.util.spec_from_file_location(mod, path)
    print(mod, 'spec found' if spec else 'NOT FOUND')
"
```
Expected: 세 모듈 모두 `spec found`

- [ ] **Step 8: pytest re-run**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
python3 -m pytest tests/test_pc2_pack.py -v
```
Expected: `3 passed`

- [ ] **Step 9: Commit**

```bash
git add scripts/3d_mapper_node.py scripts/map_visualizer_node.py \
        scripts/map_diff_visualizer.py tests/test_pc2_pack.py
git commit -m "$(cat <<'EOF'
perf(pc2): replace struct.pack loop with numpy tobytes (P1-2)

3 곳의 PointCloud2 직렬화에서 Python for-loop + struct.pack 을
numpy structured dtype + .tobytes() 로 대체. byte-equivalent.

- scripts/3d_mapper_node.py: occupied voxel publish (xyzi)
- scripts/map_visualizer_node.py: out-of-core publish (xyzi)
- scripts/map_diff_visualizer.py: diff RGB publish (xyzrgb)
- tests/test_pc2_pack.py: byte-equivalence 3 PASS
EOF
)"
```

---

### Task 7: B-1.3 — first-hit np.argmax (P1-4, 3 spots)

**Files:**
- Modify: `scripts/3d_mapper.py:222-228, 565-569, 677-681`
- Test: `tests/test_first_hit.py`

- [ ] **Step 1: 동등성 단위 테스트 (`tests/test_first_hit.py`)**

```python
"""np.argmax(mask) 가 for+enumerate+break 와 동일 인덱스 반환 검증."""
import numpy as np


def first_hit_loop(profile: np.ndarray, threshold: float, range_resolution: float, min_range: float) -> int:
    """기존 구현 (참조)."""
    for r_idx, intensity in enumerate(profile):
        range_m = r_idx * range_resolution
        if intensity > threshold and range_m >= min_range:
            return r_idx
    return -1


def first_hit_argmax(profile: np.ndarray, threshold: float, range_resolution: float, min_range: float) -> int:
    """새 구현 (벡터화)."""
    n = len(profile)
    if n == 0:
        return -1
    ranges = np.arange(n, dtype=np.float64) * range_resolution
    mask = (profile > threshold) & (ranges >= min_range)
    if not mask.any():
        return -1
    return int(np.argmax(mask))


def test_basic_hit():
    profile = np.array([10.0, 20.0, 200.0, 250.0, 100.0])
    assert first_hit_argmax(profile, 100.0, 0.1, 0.0) == first_hit_loop(profile, 100.0, 0.1, 0.0) == 2


def test_min_range_skips_early():
    profile = np.array([200.0, 200.0, 200.0, 200.0])  # 전부 hit, 그러나 min_range 로 첫 인덱스 컷
    # min_range = 0.15, range_resolution=0.1 → r_idx 0,1 = 0.0, 0.1 < 0.15 컷 → first hit at r_idx=2
    assert first_hit_argmax(profile, 100.0, 0.1, 0.15) == first_hit_loop(profile, 100.0, 0.1, 0.15) == 2


def test_no_hit():
    profile = np.array([10.0, 20.0, 30.0])  # 모두 threshold 미달
    assert first_hit_argmax(profile, 100.0, 0.1, 0.0) == first_hit_loop(profile, 100.0, 0.1, 0.0) == -1


def test_empty_profile():
    profile = np.array([])
    assert first_hit_argmax(profile, 100.0, 0.1, 0.0) == first_hit_loop(profile, 100.0, 0.1, 0.0) == -1


def test_random_consistency():
    rng = np.random.default_rng(7)
    for _ in range(50):
        profile = rng.uniform(0, 300, size=200)
        thr = rng.uniform(50, 250)
        rr = rng.uniform(0.01, 0.2)
        mr = rng.uniform(0.0, 5.0)
        assert first_hit_argmax(profile, thr, rr, mr) == first_hit_loop(profile, thr, rr, mr)
```

- [ ] **Step 2: 테스트 실행 → PASS**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
python3 -m pytest tests/test_first_hit.py -v
```
Expected: `5 passed`

- [ ] **Step 3: 헬퍼 함수 추가 (`scripts/3d_mapper.py` 클래스 메서드 또는 모듈 함수)**

`scripts/3d_mapper.py` 의 `class SonarMapper` 안에 다음 private 메서드를 추가 (적절한 위치 — `_collect_first_hits` 함수 위 등):

```python
    def _first_hit_index(self, intensity_profile: np.ndarray, range_resolution: float) -> int:
        """첫 hit r_idx 반환. 없으면 -1.

        Phase B-1.3 (P1-4): 3 곳의 for+enumerate+break 패턴을 통일.
        """
        n = len(intensity_profile)
        if n == 0:
            return -1
        ranges = np.arange(n, dtype=np.float64) * range_resolution
        mask = (intensity_profile > self.intensity_threshold) & (ranges >= self.min_range)
        if not mask.any():
            return -1
        return int(np.argmax(mask))
```

- [ ] **Step 4: `scripts/3d_mapper.py:222-228` 호출 변경**

기존 (222-228):
```python
            # Actual first hit from sonar image
            intensity_profile = polar_image[:, b_idx]
            actual_depth = -1.0
            for r_idx, intensity in enumerate(intensity_profile):
                range_m = r_idx * range_resolution
                if intensity > self.intensity_threshold and range_m >= self.min_range:
                    actual_depth = range_m
                    break
```

새 코드:
```python
            # Actual first hit from sonar image (Phase B-1.3 P1-4)
            intensity_profile = polar_image[:, b_idx]
            r_idx = self._first_hit_index(intensity_profile, range_resolution)
            actual_depth = r_idx * range_resolution if r_idx >= 0 else -1.0
```

- [ ] **Step 5: `scripts/3d_mapper.py:565-569` 호출 변경**

기존 (562-569):
```python
        # Find first hit
        first_hit_idx = -1
        range_resolution = self.max_range / len(intensity_profile)
        
        for r_idx, intensity in enumerate(intensity_profile):
            range_m = r_idx * range_resolution
            if intensity > self.intensity_threshold and range_m >= self.min_range:
                first_hit_idx = r_idx
                break
```

새 코드:
```python
        # Find first hit (Phase B-1.3 P1-4)
        range_resolution = self.max_range / len(intensity_profile)
        first_hit_idx = self._first_hit_index(intensity_profile, range_resolution)
```

- [ ] **Step 6: `scripts/3d_mapper.py:677-681` 호출 변경**

기존 (676-681):
```python
            intensity_profile = polar_image[:, b_idx]
            for r_idx, intensity in enumerate(intensity_profile):
                range_m = r_idx * range_resolution
                if intensity > self.intensity_threshold and range_m >= self.min_range:
                    bearing_first_hits.append((bearing_angle, r_idx * range_resolution))
                    break
```

새 코드:
```python
            intensity_profile = polar_image[:, b_idx]
            r_idx = self._first_hit_index(intensity_profile, range_resolution)
            if r_idx >= 0:
                bearing_first_hits.append((bearing_angle, r_idx * range_resolution))
```

- [ ] **Step 7: 빌드 + import smoke**

```bash
cd /workspace/ros2_ws_phase_a
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
python3 -c "
import importlib.util, importlib.machinery
loader = importlib.machinery.SourceFileLoader(
    'mapper_3d',
    '/workspace/ros2_ws_phase_a/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction/3d_mapper.py'
)
mod = loader.load_module()
print('SonarMapper._first_hit_index:', hasattr(mod.SonarMapper, '_first_hit_index'))
"
```
Expected: `SonarMapper._first_hit_index: True`

- [ ] **Step 8: pytest 재확인**

```bash
cd /workspace/ros2_ws_phase_a/src/sonar_3d_reconstruction
python3 -m pytest tests/test_first_hit.py -v
```
Expected: `5 passed`

- [ ] **Step 9: Commit**

```bash
git add scripts/3d_mapper.py tests/test_first_hit.py
git commit -m "$(cat <<'EOF'
perf(mapper_3d): vectorize first-hit search with np.argmax (P1-4)

3 곳의 for+enumerate+break 패턴을 SonarMapper._first_hit_index() 메서드로
통일. (intensity_profile > threshold) & (ranges >= min_range) 마스크 →
np.argmax(mask) 로 O(N) C-loop 활용.

- scripts/3d_mapper.py: _first_hit_index 추가 + 3 곳 호출
- tests/test_first_hit.py: 5 PASS (random 50회 동등성 포함)
EOF
)"
```

---

### Task 8: 통합 회귀 측정 + CHANGELOG + PR 준비

- [ ] **Step 1: candidate 빌드**

```bash
cd /workspace/ros2_ws_phase_a
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH ROS_PACKAGE_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
source /workspace/ros2_ws/install/setup.bash 2>/dev/null
colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```
Expected: `Finished <<< sonar_3d_reconstruction`

- [ ] **Step 2: P-1 candidate 측정**

```bash
BAG_PATH=/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m750d_custom_platform/m750d-range15-tilt45-v1 \
OUT_DIR=/tmp/sonar3d_regression/p1 \
PLAY_DURATION=90 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh candidate
```
Expected: `[regression] (candidate) saved → /tmp/sonar3d_regression/p1/candidate/cloud`

- [ ] **Step 3: P-1 비교 (bit-exact)**

```bash
OUT_DIR=/tmp/sonar3d_regression/p1 \
JACCARD_THRESHOLD=1.0 \
MEAN_LOG_ODDS_THRESHOLD=0.0 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh compare
```
Expected: `[compare] PASS (jaccard=True, log_odds_diff=True)` + jaccard `1.000000` + mean diff `0.000000`

- [ ] **Step 4: P-1 plot 생성**

```bash
python3 src/sonar_3d_reconstruction/scripts/regression/regression_plot.py \
    --baseline /tmp/sonar3d_regression/p1/baseline/cloud \
    --candidate /tmp/sonar3d_regression/p1/candidate/cloud \
    --out /tmp/sonar3d_regression/p1/comparison.png
```
Expected: `[plot] saved /tmp/sonar3d_regression/p1/comparison.png`

- [ ] **Step 5: P-2 candidate 측정 + 비교**

```bash
BAG_PATH=/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/m3000d_blueboat/m3000d-range15-tilt90 \
OUT_DIR=/tmp/sonar3d_regression/p2 \
PLAY_DURATION=90 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh candidate

OUT_DIR=/tmp/sonar3d_regression/p2 \
JACCARD_THRESHOLD=1.0 \
MEAN_LOG_ODDS_THRESHOLD=0.0 \
bash src/sonar_3d_reconstruction/scripts/regression/regression_test.sh compare

python3 src/sonar_3d_reconstruction/scripts/regression/regression_plot.py \
    --baseline /tmp/sonar3d_regression/p2/baseline/cloud \
    --candidate /tmp/sonar3d_regression/p2/candidate/cloud \
    --out /tmp/sonar3d_regression/p2/comparison.png
```
Expected: 두 데이터셋 모두 PASS, plot 생성

**FAIL 시 처리**: jaccard < 1.0 또는 log-odds diff > 0 → bit-exact 불변량 깨짐 → STOP하고 어느 sub(B-1.1/1.2/1.3)에서 들어왔는지 `git bisect` 진행. PointCloud2 byte 직렬화 변경(B-1.2)은 byte-level 동일이지만 **메시지 stamp 또는 frame_id 차이가 voxelize 결과에 영향 줄 수 있음** → 이 경우 임계 0 대신 매우 낮은 값(jaccard ≥ 0.9999) 허용 가능. 사용자 합의 후 결정.

- [ ] **Step 6: CHANGELOG.md 갱신**

`CHANGELOG.md` 의 Phase A 섹션 위에 다음을 추가:

```markdown
## [Unreleased] — Phase B-1: Performance Surgical & Regression Infrastructure (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Plan: `docs/source/plans/2026-05-03-phase-b-1-perf-and-regression-infra.md`
> Risk: 낮음. bit-exact 결과 유지.

### Added
- `scripts/regression/regression_metric.py` — voxel/jaccard/log-odds API (단위 테스트 6 PASS)
- `scripts/regression/regression_test.sh` — bag replay orchestrator (baseline/candidate/compare)
- `scripts/regression/regression_compare.py` — 임계 판정 (exit code 기반)
- `scripts/regression/regression_plot.py` — xy/xz 단면 plot
- `scripts/regression/README.md` — 사용법
- `tests/regression/test_metric.py` (6 PASS), `tests/test_pc2_pack.py` (3 PASS), `tests/test_first_hit.py` (5 PASS), `tests/test_gil_release.py` (2 PASS)

### Changed
- `sonar_3d_reconstruction/cpp/python_bindings.cpp`: 24개 무거운 .def 에 `py::call_guard<py::gil_scoped_release>()` 추가 (P0-3)
- `scripts/3d_mapper_node.py:728-735`: `struct.pack` 루프 → numpy structured `.tobytes()` (P1-2)
- `scripts/map_visualizer_node.py:422-429`: 동일 패턴 (P1-2)
- `scripts/map_diff_visualizer.py:202-221`: xyzrgb 동일 패턴 + RGB uint32 직접 패킹 (P1-2)
- `scripts/3d_mapper.py`: `_first_hit_index()` 메서드 추가 + 3 곳 호출 (P1-4)

### Verification
- colcon build PASS (Release)
- 단위 테스트 16 PASS (regression metric 6 + pc2 pack 3 + first-hit 5 + gil smoke 2)
- 회귀 측정 P-1 (m750d-range15-tilt45-v1, 90s replay):
  - jaccard_set = **1.000000**
  - mean_log_odds_diff = **0.000000**
  - avg_proc_time_ms = baseline X.X → candidate X.X (Δ X.X%)
- 회귀 측정 P-2 (m3000d-range15-tilt90, 90s replay):
  - jaccard_set = **1.000000**
  - mean_log_odds_diff = **0.000000**
  - avg_proc_time_ms = baseline X.X → candidate X.X (Δ X.X%)
- 시각 비교 plot: `/tmp/sonar3d_regression/p{1,2}/comparison.png` PR attach

### Notes
- 회귀 인프라(`scripts/regression/`)는 영구 자산. Phase B-2 이후 모든 phase 가 동일 명령으로 재사용.
- bit-exact 결과 → Phase B-2(의도된 정확도 변경) 의 baseline 으로 사용.
- 이 phase 는 영구 worktree(`/workspace/ros2_ws_phase_a/`) 에서 작업.
```

(실제 측정 후 X.X 자리에 측정값 채움)

- [ ] **Step 7: CHANGELOG commit**

```bash
git add CHANGELOG.md
git commit -m "docs(changelog): add Phase B-1 entry"
```

- [ ] **Step 8: Self-review (skill checklist)**

```bash
git log --oneline refactor/phase-b1-perf-surgical ^$(git merge-base refactor/phase-b1-perf-surgical refactor/phase-a-cleanup)
```
Expected commits (~7개):
- `feat(regression): add metric library for B-1.0 infrastructure`
- `feat(regression): add bag-replay orchestrator script`
- `feat(regression): add compare + plot driver scripts`
- `docs(regression): add README for B-1.0 infrastructure`
- `perf(bindings): release GIL on heavy C++ methods (P0-3)`
- `perf(pc2): replace struct.pack loop with numpy tobytes (P1-2)`
- `perf(mapper_3d): vectorize first-hit search with np.argmax (P1-4)`
- `docs(changelog): add Phase B-1 entry`

- [ ] **Step 9: PR template 준비 (push 는 별도 사용자 승인 후)**

PR 본문 (`/tmp/phase-b1-pr.md` 로 저장 후 사용자에게 보고):

```markdown
## Summary
- Phase B-1: bit-exact 결과 보장 + B-1.0 회귀 인프라 신규 작성 + GIL/PC2/first-hit 3건 surgical 성능 개선

## Changes
- `scripts/regression/`: 회귀 인프라 5종 신규 (metric/test/compare/plot/README)
- `python_bindings.cpp`: 24개 무거운 .def 에 GIL 해제 (P0-3)
- `3d_mapper_node.py` / `map_visualizer_node.py` / `map_diff_visualizer.py`: PointCloud2 직렬화 numpy 일괄화 (P1-2)
- `3d_mapper.py`: `_first_hit_index()` 추가 + 3 곳 호출 (P1-4)

## Verification
- [x] colcon build PASS (Release)
- [x] 단위 테스트 16 PASS
- [x] Regression P-1: jaccard 1.000000, mean Δlog-odds 0.000000
- [x] Regression P-2: jaccard 1.000000, mean Δlog-odds 0.000000
- [x] xy/xz plot 첨부 (P-1, P-2)
- [x] CHANGELOG.md 갱신

## Next Phase
- Phase B-2: correctness 5건 (P0-2 NaN, P0-4 depth filter mask, P0-5 multiplicity, P0-7 QoS, P0-8 binary search)
```

push 시점은 사용자 합의 후:
```bash
git push -u origin refactor/phase-b1-perf-surgical
gh pr create --title "refactor(sonar3d): phase B-1 — perf surgical + regression infra" \
             --body-file /tmp/phase-b1-pr.md
```

---

## Self-Review Checklist (writing-plans skill)

### 1. Spec coverage

| Spec 요구 | 대응 task |
|----------|----------|
| §3 Phase B-1 → B-1.0 회귀 인프라 (5 파일) | Tasks 1, 2, 3, 4 |
| §3 Phase B-1.1 GIL release | Task 5 |
| §3 Phase B-1.2 PointCloud2 numpy | Task 6 |
| §3 Phase B-1.3 first-hit np.argmax | Task 7 |
| §4.1 Dataset P-1 + P-2 양쪽 | Task 4 (baseline) + Task 8 (candidate) |
| §4.2 fast_lio 패턴 이식 | Task 2 (regression_test.sh 의 launch+record+play 흐름) |
| §4.3 Metric (jaccard/log-odds/proc-time) | Task 1 (regression_metric.py) + Task 3 (compare) |
| §4.4 임계 (B-1: 1.0 / 0.0 / ≤baseline) | Task 8 비교 단계 |
| §4.5 시각 plot xy/xz | Task 3 + Task 8 |
| §5 4축 추적 (branch/commit/CHANGELOG/PR) | Task 0 (branch), 각 Task commit, Task 8 CHANGELOG/PR |
| §10 Q-Data (fast_lio 함께 launch) | Task 2 의 `regression_test.sh` 가 SLAM_LAUNCH_PKG=fast_lio 동시 실행 |

**커버리지 100%.**

### 2. Placeholder scan

검색 결과: "TBD"/"TODO"/"implement later"/"fill in details"/"add appropriate" 0개. CHANGELOG 의 "X.X" 자리만 측정 후 채움 (Task 8 Step 6에서 명시).

### 3. Type consistency

- `regression_metric.voxelize(points, resolution=...) -> np.ndarray` (Task 1) ↔ Task 3 `regression_compare.py` 호출 일치 ✓
- `regression_metric.read_last_pointcloud(bag_dir, topic) -> Optional[PointCloudFrame]` (Task 1) ↔ Task 3 호출 일치 ✓
- `SonarMapper._first_hit_index(intensity_profile, range_resolution) -> int` (Task 7 Step 3) ↔ Tasks 7 Step 4/5/6 호출 일치 ✓
- `pack_numpy_xyzi(points, intensities) -> bytes` 의 dtype `[("x","<f4"),("y","<f4"),("z","<f4"),("i","<f4")]` ↔ 노드측 코드 동일 dtype ✓

**일관성 OK.**

---

## 부록: 위험 / 롤백

- bit-exact 깨짐 (jaccard < 1.0): B-1.2 PointCloud2 직렬화에서 stamp 변동이 voxelize 결과에 영향 줄 수 있음 — 검사 후 임계 미세 조정(jaccard ≥ 0.9999) 가능.
- GIL 해제 후 ROS 콜백 race: B-3 phase 까지 multi-threaded executor 미도입 → race 위험 매우 낮음. 현재 SingleThreadedExecutor 환경에서는 GIL 해제만으로는 race 발생 불가.
- `regression_test.sh` 가 fast_lio launch 실패하면 odom 없음 → 매핑 진행 안 됨. 이 경우 `${out}/slam_launch.log` 확인 후 fast_lio 환경 점검(Q-Data 절차).
- 모든 변경 reversible: `git revert <sha>` 가 각 commit 단위로 가능.

