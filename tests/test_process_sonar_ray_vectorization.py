"""Phase D — process_sonar_ray vectorization regression test.

Verifies that the vectorized inner loop produces voxel updates
identical to the scalar baseline for a fixed random fixture.
"""
from __future__ import annotations

import os
import sys
import pickle
from pathlib import Path

import numpy as np
import pytest

# Make `scripts/` importable as a module.
HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.normpath(os.path.join(HERE, "..", "scripts"))
if SCRIPTS not in sys.path:
    sys.path.insert(0, SCRIPTS)

# `3d_mapper.py` starts with a digit; load via importlib.
import importlib.util
spec = importlib.util.spec_from_file_location(
    "_mapper3d", os.path.join(SCRIPTS, "3d_mapper.py"))
_mapper3d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(_mapper3d)
SonarTo3DMapper = _mapper3d.SonarTo3DMapper


GOLDEN_PATH = Path(HERE) / "fixtures" / "process_sonar_ray_scalar_golden.pkl"


@pytest.fixture
def mapper():
    """Construct a SonarOctoMapper with deterministic params (no ROS init)."""
    m = SonarTo3DMapper.__new__(SonarTo3DMapper)
    m.max_range = 15.0
    m.min_range = 0.3
    m.voxel_resolution = 0.05
    m.vertical_aperture = np.deg2rad(20.0)
    m.intensity_threshold = 50
    m.log_odds_free = -0.4
    m.log_odds_occupied = 0.85
    return m


@pytest.fixture
def fixed_intensity():
    rng = np.random.default_rng(2026_05_05)
    profile = rng.integers(0, 256, size=512, dtype=np.int32)
    profile[100:120] = 200  # ensure first hit + occupied region
    return profile.astype(np.float64)


@pytest.fixture
def fixed_transform():
    # Deterministic non-trivial transform: yaw 30°, pitch 10°, t=(2, -1, 0.5).
    yaw, pitch = np.deg2rad(30), np.deg2rad(10)
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    R = np.array([
        [cy * cp, -sy, cy * sp],
        [sy * cp,  cy, sy * sp],
        [    -sp,   0,      cp],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [2.0, -1.0, 0.5]
    return T


@pytest.fixture(scope="module")
def golden():
    """Load the scalar-baseline golden snapshot. Generate on first run."""
    if not GOLDEN_PATH.exists():
        # Build a local mapper / intensity / transform identical to the
        # function-scope fixtures so the snapshot is deterministic.
        m = SonarTo3DMapper.__new__(SonarTo3DMapper)
        m.max_range = 15.0
        m.min_range = 0.3
        m.voxel_resolution = 0.05
        m.vertical_aperture = np.deg2rad(20.0)
        m.intensity_threshold = 50
        m.log_odds_free = -0.4
        m.log_odds_occupied = 0.85

        rng = np.random.default_rng(2026_05_05)
        profile = rng.integers(0, 256, size=512, dtype=np.int32)
        profile[100:120] = 200
        intensity = profile.astype(np.float64)

        yaw, pitch = np.deg2rad(30), np.deg2rad(10)
        cy, sy = np.cos(yaw), np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        R = np.array([
            [cy * cp, -sy, cy * sp],
            [sy * cp,  cy, sy * sp],
            [    -sp,   0,      cp],
        ])
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [2.0, -1.0, 0.5]

        GOLDEN_PATH.parent.mkdir(parents=True, exist_ok=True)
        bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
        snap = [m.process_sonar_ray(b, intensity, T) for b in bearings]
        with GOLDEN_PATH.open("wb") as f:
            pickle.dump(snap, f, protocol=pickle.HIGHEST_PROTOCOL)
    with GOLDEN_PATH.open("rb") as f:
        return pickle.load(f)


def _voxelize(updates, res):
    """Reduce update list to a deterministic set of (kind, key)."""
    out = set()
    for pt, log_odds, kind, intensity in updates:
        ix = int(np.floor(pt[0] / res))
        iy = int(np.floor(pt[1] / res))
        iz = int(np.floor(pt[2] / res))
        out.add((kind, ix, iy, iz))
    return out


def test_vectorized_matches_scalar_for_100_bearings(mapper, fixed_intensity, fixed_transform):
    bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
    for b in bearings:
        result = mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
        # Result is a List[Tuple[ndarray(3,), float, str, Optional[float]]].
        assert isinstance(result, list)
        for entry in result:
            pt, log_odds, kind, intensity = entry
            assert pt.shape == (3,)
            assert kind in ("free", "occupied")
            if kind == "free":
                assert intensity is None
            else:
                assert intensity is not None


def test_vectorized_voxel_keys_bit_exact(mapper, fixed_intensity, fixed_transform, golden):
    """After Task 2/3 lands, vectorized output must produce identical voxel
    keys at the configured voxel_resolution. atol=0 — bit-exact requirement
    from design §4.3."""
    bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
    for i, b in enumerate(bearings):
        result = mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
        assert _voxelize(result, mapper.voxel_resolution) == \
               _voxelize(golden[i], mapper.voxel_resolution), \
               f"bearing #{i}={np.degrees(b):.2f}° differs"
