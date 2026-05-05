"""Phase B-2a (P0-2): IWLO intensity_to_weight 0-division guard.

`intensity_to_weight` 의 분모 (intensity_max - intensity_threshold) 가 0
이 되면 normalized 가 NaN 이 되고 sigmoid 입력으로 흘러가 결과가 NaN.
NaN 이 occupancy log-odds 누적에 들어가면 voxel 이 영구히 NaN 으로 오염.

이 테스트는 `set_intensity_params(threshold == max)` 로 호출 후
`batch_update_iwlo` 결과 voxel 의 intensity 값이 finite 임을 검증한다.
"""
import math
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


def _voxel_intensities_finite(updater) -> bool:
    voxels = updater.get_occupied_voxels(0.5)
    if voxels.shape[0] == 0:
        return True
    intensities = voxels[:, 3]
    return bool(np.all(np.isfinite(intensities)))


def test_threshold_equals_max_does_not_produce_nan():
    pu = ProbabilityUpdater(resolution=0.05)
    pu.set_log_odds_params(0.85, -0.4)
    pu.set_intensity_params(150, 150)  # threshold == max → range = 0
    pu.set_iwlo_params(2.5, 0.05, 0.05, -2.0, 3.5)

    points = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0, 180.0], dtype=np.float64)
    is_occ = np.array([True, True], dtype=bool)

    pu.batch_update_iwlo(points, intensities, is_occ)
    assert _voxel_intensities_finite(pu)


def test_threshold_just_below_max_produces_finite():
    pu = ProbabilityUpdater(resolution=0.05)
    pu.set_log_odds_params(0.85, -0.4)
    pu.set_intensity_params(150, 150 + 1e-12)  # range ≈ 0 but not exactly
    pu.set_iwlo_params(2.5, 0.05, 0.05, -2.0, 3.5)

    points = np.array([[1.0, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0], dtype=np.float64)
    is_occ = np.array([True], dtype=bool)

    pu.batch_update_iwlo(points, intensities, is_occ)
    assert _voxel_intensities_finite(pu)


def test_normal_range_unchanged():
    pu = ProbabilityUpdater(resolution=0.05)
    pu.set_log_odds_params(0.85, -0.4)
    pu.set_intensity_params(120, 255)  # normal range
    pu.set_iwlo_params(2.5, 0.05, 0.05, -2.0, 3.5)

    points = np.array([[0.0, 0.0, 0.0]], dtype=np.float64)
    intensities = np.array([200.0], dtype=np.float64)
    is_occ = np.array([True], dtype=bool)

    pu.batch_update_iwlo(points, intensities, is_occ)
    voxels = pu.get_occupied_voxels(0.5)
    assert voxels.shape[0] >= 1
    assert math.isfinite(float(voxels[0, 3]))
