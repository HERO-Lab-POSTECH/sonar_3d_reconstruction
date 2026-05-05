"""Unit tests for regression_metric library."""
import math
from pathlib import Path
import sys

import numpy as np
import pytest

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
