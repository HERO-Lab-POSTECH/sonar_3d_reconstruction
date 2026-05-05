"""Phase B-1.0 회귀 인프라용 메트릭 라이브러리.

PointCloud2 bag 두 개를 voxel set + log-odds dict 로 환산하고
Jaccard / log-odds diff 통계를 계산한다. 후속 phase 의 회귀 스크립트가
이 모듈을 import 해 재사용한다.

공개 API
- ``VOXEL_RESOLUTION_DEFAULT``: 기본 voxel 해상도(m)
- ``voxelize``: (N,3) float → (N,3) int64 voxel key
- ``jaccard``: 두 set 의 Jaccard index
- ``prob_to_log_odds``: 점유 확률 → log-odds (eps clamp)
- ``log_odds_diff``: 공통 voxel 의 |L_a - L_b| 통계
- ``PointCloudFrame``: 한 프레임 데이터 클래스
- ``read_last_pointcloud``: bag 의 마지막 PointCloud2 deserialize
- ``read_processing_stats``: std_msgs/Float64 통계
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Set, Tuple

import math
import numpy as np

VOXEL_RESOLUTION_DEFAULT: float = 0.05
LOG_ODDS_EPS: float = 1.0e-6
POINT_STEP_EXPECTED: int = 16  # PointXYZI: 4 floats × 4 bytes


@dataclass
class PointCloudFrame:
    """단일 PointCloud2 메시지의 파싱 결과."""

    stamp_ns: int
    points: np.ndarray  # (N, 3) float64
    intensities: np.ndarray  # (N,) float64


def voxelize(
    points: np.ndarray,
    resolution: float = VOXEL_RESOLUTION_DEFAULT,
) -> np.ndarray:
    """(N,3) float 좌표를 voxel key (N,3) int64 로 양자화.

    ``np.floor(p / resolution)`` 으로 격자 인덱스를 만든다. 빈 입력은
    shape (0, 3) 의 int64 array 로 반환.
    """
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"points must be (N,3); got {points.shape}")
    if resolution <= 0.0:
        raise ValueError(f"resolution must be > 0; got {resolution}")
    if points.shape[0] == 0:
        return np.empty((0, 3), dtype=np.int64)
    keys = np.floor(points.astype(np.float64) / resolution).astype(np.int64)
    return keys


def jaccard(set_a: Set[Tuple[int, int, int]], set_b: Set[Tuple[int, int, int]]) -> float:
    """두 voxel-key 집합의 Jaccard index. 둘 다 비어있으면 1.0."""
    if not set_a and not set_b:
        return 1.0
    union = set_a | set_b
    if not union:
        return 1.0
    inter = set_a & set_b
    return len(inter) / len(union)


def prob_to_log_odds(p: np.ndarray) -> np.ndarray:
    """점유 확률 → log-odds. 0/1 근방은 eps 로 clamp."""
    p_arr = np.asarray(p, dtype=np.float64)
    p_clamped = np.clip(p_arr, LOG_ODDS_EPS, 1.0 - LOG_ODDS_EPS)
    return np.log(p_clamped / (1.0 - p_clamped))


def _keys_to_dict(
    keys: np.ndarray,
    probs: np.ndarray,
) -> Dict[Tuple[int, int, int], float]:
    """voxel keys + probs → {(i,j,k): log_odds} dict.

    동일 key 가 여러 번 나오면 마지막 값으로 덮어쓴다.
    """
    if keys.shape[0] != probs.shape[0]:
        raise ValueError(
            f"keys/probs length mismatch: {keys.shape[0]} vs {probs.shape[0]}"
        )
    log_odds = prob_to_log_odds(probs)
    out: Dict[Tuple[int, int, int], float] = {}
    for k, lo in zip(keys, log_odds):
        out[(int(k[0]), int(k[1]), int(k[2]))] = float(lo)
    return out


def log_odds_diff(
    keys_a: np.ndarray,
    probs_a: np.ndarray,
    keys_b: np.ndarray,
    probs_b: np.ndarray,
) -> Dict[str, float]:
    """공통 voxel 의 |L_a - L_b| 통계.

    Returns
    -------
    dict
        - common_count: 공통 voxel 수
        - mean_diff: 평균 절대 log-odds 차 (공통 0 → NaN)
        - max_diff: 최대 절대 log-odds 차 (공통 0 → NaN)
    """
    dict_a = _keys_to_dict(keys_a, probs_a)
    dict_b = _keys_to_dict(keys_b, probs_b)
    common = set(dict_a.keys()) & set(dict_b.keys())
    if not common:
        return {"common_count": 0, "mean_diff": math.nan, "max_diff": math.nan}
    diffs = np.array([abs(dict_a[k] - dict_b[k]) for k in common], dtype=np.float64)
    return {
        "common_count": int(len(common)),
        "mean_diff": float(diffs.mean()),
        "max_diff": float(diffs.max()),
    }


# ---------------------------------------------------------------------------
# rosbag2 reading helpers — ROS2 humble 환경 의존
# ---------------------------------------------------------------------------

def _import_rosbag2():
    """rosbag2_py + rclpy.serialization + msg 타입을 lazy import.

    ROS2 환경 미설정 시 명확한 메시지로 raise.
    """
    try:
        from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions  # noqa: F401
        from rclpy.serialization import deserialize_message  # noqa: F401
        from sensor_msgs.msg import PointCloud2  # noqa: F401
        from std_msgs.msg import Float64  # noqa: F401
    except ImportError as exc:
        raise ImportError(
            "regression_metric requires ROS2 humble (rosbag2_py, rclpy, "
            "sensor_msgs, std_msgs). Source /opt/ros/humble/setup.bash first."
        ) from exc
    return SequentialReader, StorageOptions, ConverterOptions, deserialize_message, PointCloud2, Float64


def _open_reader(bag_dir: Path):
    """SequentialReader 를 열어 sqlite3/mcap 자동 판별."""
    SequentialReader, StorageOptions, ConverterOptions, _, _, _ = _import_rosbag2()
    bag_path = Path(bag_dir)
    if not bag_path.exists():
        raise FileNotFoundError(f"bag directory not found: {bag_path}")
    storage_id = "mcap" if any(bag_path.glob("*.mcap")) else "sqlite3"
    storage = StorageOptions(uri=str(bag_path), storage_id=storage_id)
    converter = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage, converter)
    return reader


def _deserialize_pointcloud2(msg) -> Tuple[np.ndarray, np.ndarray]:
    """PointCloud2 → (points (N,3) float64, intensities (N,) float64).

    point_step != 16 이면 ValueError. 빈 cloud 는 (0,3)/(0,) 반환.
    """
    if msg.point_step != POINT_STEP_EXPECTED:
        raise ValueError(
            f"PointCloud2 point_step expected {POINT_STEP_EXPECTED} (XYZI), "
            f"got {msg.point_step}"
        )
    n = int(msg.width) * int(msg.height)
    if n == 0:
        return np.empty((0, 3), dtype=np.float64), np.empty((0,), dtype=np.float64)
    raw = np.frombuffer(bytes(msg.data), dtype=np.float32).reshape(n, 4)
    points = raw[:, :3].astype(np.float64)
    intensities = raw[:, 3].astype(np.float64)
    return points, intensities


def read_last_pointcloud(bag_dir: Path, topic: str) -> Optional[PointCloudFrame]:
    """bag 에서 ``topic`` 의 마지막 PointCloud2 메시지를 deserialize.

    토픽이 비어있거나 없으면 None.
    """
    _, _, _, deserialize_message, PointCloud2, _ = _import_rosbag2()
    reader = _open_reader(bag_dir)
    last_stamp_ns: Optional[int] = None
    last_points: Optional[np.ndarray] = None
    last_intens: Optional[np.ndarray] = None
    while reader.has_next():
        topic_name, data, t_ns = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, PointCloud2)
        last_points, last_intens = _deserialize_pointcloud2(msg)
        last_stamp_ns = int(t_ns)
    if last_points is None or last_stamp_ns is None:
        return None
    return PointCloudFrame(
        stamp_ns=last_stamp_ns,
        points=last_points,
        intensities=last_intens if last_intens is not None else np.empty((0,), np.float64),
    )


def read_processing_stats(
    bag_dir: Path,
    topic: str = "/pkrc/sonar/processing_time",
) -> Dict[str, float]:
    """std_msgs/Float64 토픽의 통계 {count, mean, p50, p95, max}.

    토픽이 없으면 빈 dict.
    """
    _, _, _, deserialize_message, _, Float64 = _import_rosbag2()
    reader = _open_reader(bag_dir)
    values = []
    while reader.has_next():
        topic_name, data, _ = reader.read_next()
        if topic_name != topic:
            continue
        msg = deserialize_message(data, Float64)
        values.append(float(msg.data))
    if not values:
        return {}
    arr = np.asarray(values, dtype=np.float64)
    return {
        "count": int(arr.size),
        "mean": float(arr.mean()),
        "p50": float(np.percentile(arr, 50)),
        "p95": float(np.percentile(arr, 95)),
        "max": float(arr.max()),
    }
