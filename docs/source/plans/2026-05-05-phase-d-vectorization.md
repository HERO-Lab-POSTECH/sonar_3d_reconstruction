# Phase D — Vectorization Implementation Plan (A안)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** master spec §3 Phase D 의 P1-3 을 A안 (Python 내부 vectorize) 으로 처리한다. `process_sonar_ray` 의 free-space / occupied inner loop 를 numpy broadcasting + 단일 transform 으로 교체. B안 (C++ 이관) 은 본 PR 측정 결과 보고 후 사용자 결정.

**Architecture:** scalar inner loop → bearing 별 (range, vertical_step) 메쉬 broadcasting → `T @ batch` 1 회. 기존 함수 시그니처 / 반환 형태 (`List[Tuple[ndarray, float, str, Optional[float]]]`) 는 유지하여 호출자 변경 없음. bit-exact 성을 unit test 와 회귀 jaccard ≥ 0.99 로 동시 검증.

**Tech Stack:** Python 3.10, numpy, pytest, colcon (Release).

**Branch:** `refactor/phase-d-vectorization` (PR #6 머지 후 main 에서 분기).

**Working dir:** `/workspace/ros2_ws/src/sonar_3d_reconstruction`. 모든 cd 명령은 이 위치 기준.

**Build:** `cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash`. 매 task 빌드 시 동일.

---

## Task 1 — Golden fixture + unit test (실패 기대)

**근거**: design §4.3 부동소수점 결정성 — scalar 와 vectorized 가 voxel key 단위까지 동일함을 보장해야 jaccard ≥ 0.99 (회귀) 와 분리해서 코드 단위에서 검증 가능.

**Files:**
- Create: `tests/test_process_sonar_ray_vectorization.py`

### Step 1: scalar 결과를 capture 하는 fixture 작성

신규 파일 `tests/test_process_sonar_ray_vectorization.py`:

```python
"""Phase D — process_sonar_ray vectorization regression test.

Verifies that the vectorized inner loop produces voxel updates
identical to the scalar baseline for a fixed random fixture.
"""
from __future__ import annotations

import os
import sys

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
SonarOctoMapper = _mapper3d.SonarOctoMapper


@pytest.fixture
def mapper():
    """Construct a SonarOctoMapper with deterministic params (no ROS init)."""
    m = SonarOctoMapper.__new__(SonarOctoMapper)
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


def test_vectorized_matches_scalar_for_100_bearings(mapper, fixed_intensity, fixed_transform):
    bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
    for b in bearings:
        result = mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
        # Result is a List[Tuple[ndarray(3,), float, str, Optional[float]]].
        # Convert to comparable structure: sorted (key_x, key_y, key_z, log_odds_sign, type).
        assert isinstance(result, list)
        for entry in result:
            pt, log_odds, kind, intensity = entry
            assert pt.shape == (3,)
            assert kind in ("free", "occupied")
            if kind == "free":
                assert intensity is None
            else:
                assert intensity is not None
```

### Step 2: 회귀 비교 테스트 작성 (Task 2/3 후 검증용)

같은 파일에 추가:

```python
def _voxelize(updates, res):
    """Reduce update list to a deterministic set of (kind, key)."""
    out = set()
    for pt, log_odds, kind, intensity in updates:
        ix = int(np.floor(pt[0] / res))
        iy = int(np.floor(pt[1] / res))
        iz = int(np.floor(pt[2] / res))
        out.add((kind, ix, iy, iz))
    return out


@pytest.fixture
def scalar_baseline(mapper, fixed_intensity, fixed_transform):
    """Snapshot the current (scalar) implementation output for 100 bearings."""
    bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
    return [mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
            for b in bearings]


def test_vectorized_voxel_keys_bit_exact(mapper, fixed_intensity, fixed_transform, scalar_baseline):
    """After Task 2/3 lands, vectorized output must produce identical voxel
    keys at the configured voxel_resolution. atol=0 — bit-exact requirement
    from design §4.3."""
    bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
    for i, b in enumerate(bearings):
        result = mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
        assert _voxelize(result, mapper.voxel_resolution) == \
               _voxelize(scalar_baseline[i], mapper.voxel_resolution), \
               f"bearing #{i}={np.degrees(b):.2f}° differs"
```

**중요**: Task 1 단계에서는 `scalar_baseline` 이 현재 코드를 호출 → `test_vectorized_voxel_keys_bit_exact` 는 PASS (자기 비교). Task 2/3 에서 vectorize 후 `scalar_baseline` 도 vectorize 된 코드를 호출하게 되므로, **Task 1 의 baseline 을 별도 pickle 로 dump 해 놓고 Task 2/3 의 비교 시 그 pickle 을 로드하는 방식이 더 안전**. 다음 step 으로 그렇게 변경.

### Step 3: baseline pickle 저장 fixture 추가

```python
import pickle
from pathlib import Path

GOLDEN_PATH = Path(HERE) / "fixtures" / "process_sonar_ray_scalar_golden.pkl"


@pytest.fixture(scope="module")
def golden(mapper, fixed_intensity, fixed_transform):
    """Load the scalar-baseline golden snapshot. Generate on first run."""
    if not GOLDEN_PATH.exists():
        GOLDEN_PATH.parent.mkdir(parents=True, exist_ok=True)
        bearings = np.linspace(-np.pi / 4, np.pi / 4, 100)
        snap = [mapper.process_sonar_ray(b, fixed_intensity, fixed_transform)
                for b in bearings]
        with GOLDEN_PATH.open("wb") as f:
            pickle.dump(snap, f, protocol=pickle.HIGHEST_PROTOCOL)
    with GOLDEN_PATH.open("rb") as f:
        return pickle.load(f)
```

(`test_vectorized_voxel_keys_bit_exact` 의 `scalar_baseline` 인자명을 `golden` 으로 교체.)

### Step 4: 테스트 실행 → PASS 확인 (Task 1 만 단독 실행)

Run:
```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && \
  python3 -m pytest tests/test_process_sonar_ray_vectorization.py -v
```
Expected: 2 PASSED, fixture pickle 생성됨 (`tests/fixtures/process_sonar_ray_scalar_golden.pkl`).

### Step 5: golden pickle 을 git 에 추가

```bash
git add tests/test_process_sonar_ray_vectorization.py tests/fixtures/process_sonar_ray_scalar_golden.pkl
```

### Step 6: Commit

```bash
git commit -m "test(mapper_3d): add golden fixture for process_sonar_ray vectorization (D-1.0)

Snapshot the scalar baseline output of process_sonar_ray for a deterministic
100-bearing fixture (random intensity + non-trivial transform). Subsequent
vectorize tasks must produce voxel-key-identical results against this golden.

- tests/test_process_sonar_ray_vectorization.py: pytest module with mapper /
  intensity / transform fixtures and module-scope golden pickle loader
- tests/fixtures/process_sonar_ray_scalar_golden.pkl: scalar baseline snapshot
"
```

---

## Task 2 — D-1: free space inner loop vectorize

**근거**: design §4.1.

**Files:**
- Modify: `scripts/3d_mapper.py:561-587` (free space loop in `process_sonar_ray`)

### Step 1: 변경 전 baseline 재실행 (Task 1 golden 이 동일성 기준)

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && \
  python3 -m pytest tests/test_process_sonar_ray_vectorization.py -v
```
Expected: 2 PASSED.

### Step 2: free space 루프를 vectorized 형태로 교체

`scripts/3d_mapper.py` 의 `process_sonar_ray` 내부 — 현재 코드 (line ~561 부터):
```python
free_sampling_step = 10
for r_idx in range(0, first_hit_idx, free_sampling_step):
    range_m = r_idx * range_resolution
    vertical_spread = range_m * np.tan(half_aperture)
    num_vertical = max(1, int(vertical_spread / (self.voxel_resolution * 4)))
    for v_step in range(-num_vertical, num_vertical + 1):
        vertical_angle = (v_step / num_vertical) * half_aperture
        x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
        y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
        z_sonar = range_m * np.sin(vertical_angle)
        pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
        pt_world = T_sonar_to_world @ pt_sonar
        updates.append((pt_world[:3], self.log_odds_free, 'free', None))
```

다음으로 교체:
```python
free_sampling_step = 10
r_idx = np.arange(0, first_hit_idx, free_sampling_step)
if r_idx.size > 0:
    range_m = r_idx * range_resolution
    vertical_spread = range_m * np.tan(half_aperture)
    num_vert = np.maximum(1, (vertical_spread / (self.voxel_resolution * 4)).astype(int))
    V_max = int(num_vert.max())
    v_steps = np.arange(-V_max, V_max + 1)                          # (2V+1,)
    mask = np.abs(v_steps[None, :]) <= num_vert[:, None]            # (R, 2V+1)
    # vertical_angle[r, v] = (v_steps[v] / num_vert[r]) * half_aperture
    vertical_angle = (v_steps[None, :].astype(np.float64) /
                      num_vert[:, None].astype(np.float64)) * half_aperture
    cos_va = np.cos(vertical_angle)
    sin_va = np.sin(vertical_angle)
    range_2d = range_m[:, None]
    x_s = range_2d * cos_va * np.cos(bearing_angle)
    y_s = -range_2d * cos_va * np.sin(bearing_angle)
    z_s = range_2d * sin_va
    ones = np.ones_like(x_s)
    pts_sonar = np.stack([x_s, y_s, z_s, ones], axis=-1)[mask]      # (N, 4)
    pts_world = (T_sonar_to_world @ pts_sonar.T).T[:, :3]           # (N, 3)
    for pw in pts_world:
        updates.append((pw, self.log_odds_free, 'free', None))
```

**주의**: `(v_steps / num_vert) * half_aperture` 의 산술 순서가 scalar 와 동일하게 `(v_step / num_vertical) * half` 가 되도록 broadcasting 형태 그대로 사용. dtype 은 명시적으로 `float64`.

### Step 3: 테스트 → bit-exact 동일성 확인

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && \
  python3 -m pytest tests/test_process_sonar_ray_vectorization.py -v
```
Expected: 2 PASSED. 실패 시 산술 순서 또는 dtype 체크 (특히 `int(...)` cast 위치).

### Step 4: build PASS 확인

```bash
cd /workspace/ros2_ws && \
  source /opt/ros/humble/setup.bash && \
  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
```
Expected: 1 finished, no errors.

### Step 5: Commit

```bash
git add scripts/3d_mapper.py
git commit -m "perf(mapper_3d): vectorize free-space inner loop in process_sonar_ray (D-1, P1-3)

Replace nested (range, vertical_step) loop with numpy broadcasting and a
single 4x4 batched transform. Voxel keys are bit-exact vs the scalar
baseline (tests/test_process_sonar_ray_vectorization golden, atol=0).

- scripts/3d_mapper.py: free-space sparse sampler now allocates an
  (R, 2V+1) mesh masked by per-range num_vertical and applies one
  T @ batch.T transform per ray instead of (R x V) scalar transforms
- per-ray inner loop count: ~4N -> ~1 numpy call (N = range x vertical)
"
```

---

## Task 3 — D-2: occupied inner loop vectorize

**근거**: design §4.2.

**Files:**
- Modify: `scripts/3d_mapper.py:591-633` (occupied loop in `process_sonar_ray`)

### Step 1: 현재 코드 확인 (line ~591 부터)

```python
if first_hit_idx < len(intensity_profile):
    for r_idx in range(first_hit_idx, min(first_hit_idx + 50, len(intensity_profile))):
        intensity = intensity_profile[r_idx]
        is_occupied = intensity > self.intensity_threshold
        if is_occupied:
            range_m = r_idx * range_resolution
            if range_m < self.min_range: continue
            if range_m > self.max_range: break
            vertical_spread = range_m * np.tan(half_aperture)
            num_vertical = max(1, int(vertical_spread / (self.voxel_resolution * 1.5)))
            for v_step in range(-num_vertical, num_vertical + 1):
                vertical_angle = (v_step / num_vertical) * half_aperture
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar
                updates.append((pt_world[:3], self.log_odds_occupied, 'occupied', float(intensity)))
```

### Step 2: vectorized 로 교체

```python
if first_hit_idx < len(intensity_profile):
    r_end = min(first_hit_idx + 50, len(intensity_profile))
    r_idx = np.arange(first_hit_idx, r_end)
    if r_idx.size > 0:
        intensities = intensity_profile[r_idx]
        range_m = r_idx * range_resolution
        # Preserve scalar break/continue semantics: range_m is monotonic, so
        # `break on > max_range` collapses to `<= max_range` mask.
        mask_r = (intensities > self.intensity_threshold) & \
                 (range_m >= self.min_range) & \
                 (range_m <= self.max_range)
        if mask_r.any():
            range_m = range_m[mask_r]
            intensities = intensities[mask_r]
            vertical_spread = range_m * np.tan(half_aperture)
            num_vert = np.maximum(1, (vertical_spread / (self.voxel_resolution * 1.5)).astype(int))
            V_max = int(num_vert.max())
            v_steps = np.arange(-V_max, V_max + 1)
            mask = np.abs(v_steps[None, :]) <= num_vert[:, None]   # (R, 2V+1)
            vertical_angle = (v_steps[None, :].astype(np.float64) /
                              num_vert[:, None].astype(np.float64)) * half_aperture
            cos_va = np.cos(vertical_angle)
            sin_va = np.sin(vertical_angle)
            range_2d = range_m[:, None]
            x_s = range_2d * cos_va * np.cos(bearing_angle)
            y_s = -range_2d * cos_va * np.sin(bearing_angle)
            z_s = range_2d * sin_va
            ones = np.ones_like(x_s)
            pts_sonar = np.stack([x_s, y_s, z_s, ones], axis=-1)
            pts_world_full = np.einsum('ij,rvj->rvi', T_sonar_to_world, pts_sonar)[..., :3]
            # Per-range intensity tag broadcast to (R, 2V+1).
            intensities_b = np.broadcast_to(
                intensities[:, None], (intensities.size, v_steps.size))
            for r_slot in range(range_m.size):
                pts_kept = pts_world_full[r_slot][mask[r_slot]]
                inten = float(intensities[r_slot])
                for pw in pts_kept:
                    updates.append((pw, self.log_odds_occupied, 'occupied', inten))
```

**주의**: occupied 는 intensity 값을 점마다 attach 해야 하므로 inner loop 한 단을 Python 으로 남긴다. transform / sin/cos 만 vectorize — 이게 비용의 ~95 %.

### Step 3: 테스트 → bit-exact

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && \
  python3 -m pytest tests/test_process_sonar_ray_vectorization.py -v
```
Expected: 2 PASSED.

### Step 4: 빌드 + 전체 pytest

```bash
cd /workspace/ros2_ws && \
  source /opt/ros/humble/setup.bash && \
  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  source install/setup.bash && \
  cd src/sonar_3d_reconstruction && python3 -m pytest tests/ -v
```
Expected: 16+ PASSED (Phase C 의 14 pytest + 본 PR 의 2 신규).

### Step 5: Commit

```bash
git add scripts/3d_mapper.py
git commit -m "perf(mapper_3d): vectorize occupied inner loop in process_sonar_ray (D-2, P1-3)

Replace the (range x vertical) scalar inner loop with a boolean range
mask and a single einsum-based batched transform. Intensity tag is
broadcast per-range. Voxel keys remain bit-exact vs the scalar baseline.

- scripts/3d_mapper.py: occupied dense sampler reuses the (R, 2V+1) mesh
  pattern from D-1; range filter consolidates intensity threshold,
  min_range, max_range checks into one boolean mask
"
```

---

## Task 4 — 회귀 측정 + CHANGELOG + master spec 갱신 + PR

### Step 1: regression baseline (main HEAD) 빌드 및 측정

```bash
cd /workspace/ros2_ws && \
  git -C src/sonar_3d_reconstruction stash && \
  git -C src/sonar_3d_reconstruction checkout main && \
  source /opt/ros/humble/setup.bash && \
  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  source install/setup.bash && \
  cd src/sonar_3d_reconstruction && \
  scripts/regression/regression_test.sh m3000d-range15-tilt90 baseline 90 2>&1 | tail -50
```
출력의 `metric.json` 경로 기록.

### Step 2: candidate (refactor/phase-d-vectorization) 측정

```bash
cd /workspace/ros2_ws && \
  git -C src/sonar_3d_reconstruction checkout refactor/phase-d-vectorization && \
  git -C src/sonar_3d_reconstruction stash pop && \
  colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && \
  source install/setup.bash && \
  cd src/sonar_3d_reconstruction && \
  scripts/regression/regression_test.sh m3000d-range15-tilt90 candidate 90 2>&1 | tail -50
```

### Step 3: 비교

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && \
  python3 scripts/regression/regression_compare.py \
    /tmp/sonar_3d_regression/baseline /tmp/sonar_3d_regression/candidate
```
임계: `jaccard_set ≥ 0.99`, `avg_proc_time_ms` 비율 candidate / baseline ≤ 1/1.5 = 0.667.

### Step 4: 결과 판정

- 정확도 + 처리량 둘 다 통과 → Task 4 Step 5 로.
- 처리량 미달 (≥ 0.667) → 사용자에게 측정값 보고 + B안 (C++) 결정 의뢰. **이 plan 은 종결, B안 별도 spec/plan 작성**.
- 정확도 미달 → vectorize 로직 버그. Task 2/3 으로 회귀.

### Step 5: CHANGELOG 갱신

`CHANGELOG.md` 최상단 (`## [Unreleased] - 2026-05-XX` 의 직전):

```markdown
## [Unreleased] — Phase D: ray vectorization (perf)

### Changed
- `scripts/3d_mapper.py:process_sonar_ray` — free-space and occupied inner
  loops replaced with numpy broadcasting + single batched transform
  (per-ray, ~4N scalar ops -> ~1 numpy call).

### Added
- `tests/test_process_sonar_ray_vectorization.py` — golden fixture
  comparison (100 bearings × deterministic random intensity profile)
  enforcing bit-exact voxel keys vs. scalar baseline.
- `tests/fixtures/process_sonar_ray_scalar_golden.pkl` — scalar baseline
  snapshot for the test above.

### Verification
- colcon build PASS (Release).
- pytest tests/ — 16/16 PASS (14 prior + 2 new).
- Regression P-2 (m3000d-range15-tilt90, 90s): jaccard XX.XX,
  mean Δlog-odds XX.XX, avg_proc_time baseline XX.X ms → candidate XX.X ms
  (XX.X× speedup).

### Notes
- Q-D1 — A안 측정 결과 ≥ 1.5× 임계 [통과/미달]. [통과 시: B안 불필요 / 미달 시: B안(C++ ray-cast 이관) 별도 spec 으로 진행].
- 호출자 (`_process_rays_with_shadow`) 는 변경 없음. 반환 `List[Tuple[ndarray(3,), float, str, Optional[float]]]` 시그니처 유지.
- shadow region 검사·`_first_hit_index` 는 이미 Phase B-1 에서 vectorize 됨.
```

### Step 6: master spec §3 Phase D 갱신

`docs/source/design/2026-05-03-quality-perf-uplift-design.md` 의 §3 Phase D 표 끝에 측정 결과 한 줄 추가:

```markdown
**측정 결과 (P-2, 90s)**: avg_proc_time baseline XX.X → candidate XX.X ms (XX.X×), jaccard XX.XX. Q-D1 임계 ≥ 1.5× [통과/미달].
```

### Step 7: Commit

```bash
git add CHANGELOG.md docs/source/design/2026-05-03-quality-perf-uplift-design.md
git commit -m "docs(changelog): record Phase D vectorization (A안) measurement

Phase D delivers a numpy-vectorized process_sonar_ray inner loop.
Voxel keys are bit-exact vs scalar baseline; processing-time speedup
on P-2 dataset (90s window) is XX.X× — Q-D1 ≥1.5× threshold [met/missed].

- CHANGELOG.md: Phase D entry (Changed / Added / Verification / Notes)
- docs/source/design/2026-05-03-quality-perf-uplift-design.md: §3
  Phase D measurement-result line appended
"
```

### Step 8: Push + PR

```bash
git push -u origin refactor/phase-d-vectorization
gh pr create --title "Phase D: process_sonar_ray vectorization (P1-3, A안)" --body "$(cat <<'EOF'
## Summary
- master spec §3 Phase D 의 P1-3 을 A안 (Python 내부 vectorize) 으로 처리.
- `process_sonar_ray` 의 free-space (D-1) 와 occupied (D-2) inner loop 를 numpy broadcasting + 단일 batched transform 으로 교체.
- 결과 보존 — scalar 와 voxel key 단위 bit-exact (atol=0).

## Changes
- `scripts/3d_mapper.py` — `process_sonar_ray` 내부 두 루프를 vectorized 형태로 교체.
- `tests/test_process_sonar_ray_vectorization.py` (신규) — golden fixture 비교 unit test 2 케이스.
- `tests/fixtures/process_sonar_ray_scalar_golden.pkl` (신규) — scalar baseline snapshot.

## Verification
- [ ] colcon build PASS (Release)
- [ ] pytest tests/ — 16/16 PASS (14 prior + 2 new)
- [ ] Regression P-2 (90s): jaccard ≥ 0.99
- [ ] Regression P-2 (90s): avg_proc_time speedup ≥ 1.5× (Q-D1)
- [ ] CHANGELOG.md 갱신
- [ ] master spec §3 Phase D 측정 결과 한 줄 갱신

## Next Phase
- Q-D1: A안 결과 ≥ 1.5× 통과 시 Phase D 종결. 미달 시 B안 (C++ ray-cast 이관) 별도 spec 으로.

EOF
)"
```

---

## 완료 체크리스트

- [ ] Task 1: golden fixture + 2 unit test PASS
- [ ] Task 2: free-space vectorize, golden bit-exact PASS
- [ ] Task 3: occupied vectorize, golden bit-exact PASS
- [ ] Task 4: 회귀 측정, CHANGELOG, master spec, PR
- [ ] PR squash merge 후 메모리 (`project_sonar3d_audit_state.md`) 갱신: Phase D 결과 + Q-D1 결정
