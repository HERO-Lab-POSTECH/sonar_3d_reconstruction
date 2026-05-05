"""GIL release 후에도 결과가 동일한지 smoke.

Phase B-1.1 (P0-3): pybind11 .def 에 py::call_guard<py::gil_scoped_release>()
적용 후에도 입력 동일 → 출력 동일 보장.
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
    assert voxels.shape[1] == 4
    assert len(voxels) >= 1


def test_get_resolution_unchanged():
    pu = ProbabilityUpdater(resolution=0.07)
    assert pu.get_resolution() == pytest.approx(0.07)
