"""Phase B-2d (P0-5): voxel multiplicity passed as IWLO weight.

`ProbabilityUpdater::batch_update_iwlo` accepts an optional per-point
`weights` vector that scales the per-update log-odds delta. When the
caller folds N sub-voxel observations of the same voxel into one
batch entry, passing N as the weight reproduces the cumulative effect
of N separate batches.

These tests verify (1) that weight=1 reproduces the legacy result and
(2) that weight=N produces a stronger log-odds shift than weight=1.
"""
import numpy as np
import pytest

try:
    from sonar_3d_reconstruction import (
        ProbabilityUpdater,
        CPP_MODULE_AVAILABLE,
    )
except ImportError:
    pytest.skip("sonar_3d_reconstruction package not built", allow_module_level=True)

if not CPP_MODULE_AVAILABLE:
    pytest.skip("cpp module not available", allow_module_level=True)


def _fresh_updater():
    pu = ProbabilityUpdater(resolution=0.05)
    pu.set_log_odds_params(0.85, -0.4)
    pu.set_intensity_params(120, 255)
    pu.set_iwlo_params(2.5, 0.05, 0.05, -2.0, 3.5)
    return pu


def _occupied_log_odds_at(updater, x, y, z) -> float:
    voxels = updater.get_occupied_voxels(0.0)
    if voxels.shape[0] == 0:
        return float("nan")
    # find the row closest to (x, y, z)
    diffs = np.linalg.norm(voxels[:, :3] - np.array([x, y, z]), axis=1)
    idx = int(np.argmin(diffs))
    return float(voxels[idx, 3])


def test_omitted_weights_match_legacy_path():
    pu_a = _fresh_updater()
    pu_b = _fresh_updater()

    points = np.array([[0.5, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0], dtype=np.float64)
    is_occ = np.array([True], dtype=bool)

    pu_a.batch_update_iwlo(points, intensities, is_occ)
    pu_b.batch_update_iwlo(points, intensities, is_occ, np.ones(1, dtype=np.float64))

    a = _occupied_log_odds_at(pu_a, 0.5, 0.0, 0.0)
    b = _occupied_log_odds_at(pu_b, 0.5, 0.0, 0.0)
    assert abs(a - b) < 1e-9, f"weight=1 path diverged from omitted weights: {a} vs {b}"


def test_weight_three_yields_stronger_log_odds_than_weight_one():
    pu_one = _fresh_updater()
    pu_three = _fresh_updater()

    points = np.array([[0.7, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0], dtype=np.float64)
    is_occ = np.array([True], dtype=bool)

    pu_one.batch_update_iwlo(points, intensities, is_occ, np.array([1.0]))
    pu_three.batch_update_iwlo(points, intensities, is_occ, np.array([3.0]))

    one = _occupied_log_odds_at(pu_one, 0.7, 0.0, 0.0)
    three = _occupied_log_odds_at(pu_three, 0.7, 0.0, 0.0)
    assert three > one, f"weight=3 should yield stronger log-odds: {three} <= {one}"


def test_weights_size_mismatch_raises():
    pu = _fresh_updater()
    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0, 180.0], dtype=np.float64)
    is_occ = np.array([True, True], dtype=bool)
    with pytest.raises((ValueError, RuntimeError)):
        pu.batch_update_iwlo(points, intensities, is_occ, np.array([1.0]))
