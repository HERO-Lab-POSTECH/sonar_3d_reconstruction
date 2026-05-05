# Phase D — Vectorization (design, A안)

> **상위 spec**: [`2026-05-03-quality-perf-uplift-design.md`](2026-05-03-quality-perf-uplift-design.md) §3 Phase D.
> 본 문서는 master spec 의 Phase D 절을 단일 PR 단위로 좁혀 옮긴 thin design doc 이다.
> Q-D1 결정에 따라 본 PR 은 **A안 (Python 내부 vectorize) 만** 다룬다. B안 (C++ ray-cast 이관) 은 A안 측정 결과를 본 후 사용자 합의로 별도 spec / PR 로 진행.

## 1. 배경

`scripts/3d_mapper.py:534-664` 의 `process_sonar_ray` 는 한 ray (single bearing) 에 대해 다음을 수행한다.

1. `_first_hit_index` 로 첫 hit 인덱스 검색 — Phase B-1 에서 이미 vectorize 됨.
2. **free space loop** (line 561-587): `range(0, first_hit_idx, free_sampling_step=10)` × `range(-num_vertical, num_vertical+1)` 이중 루프. 각 (r_idx, v_step) 에서:
   - `(x, y, z)` Python scalar 계산 (`np.cos`/`np.sin` 4 회 호출)
   - 4×4 transform 1 회 (`T @ pt`, shape (4,))
   - `updates.append(...)` 1 회
3. **occupied loop** (line 591-633): `range(first_hit_idx, first_hit_idx+50)` × `range(-num_vertical, num_vertical+1)` 이중 루프. `intensity > intensity_threshold` 인 r_idx 만 처리. 동일 패턴 (`np.cos`/`np.sin` × 4, 4×4 transform).

**병목 분석** (B-1 측정 시 cProfile, P-2 dataset):
- `process_sonar_ray` 자체는 호출당 ~3-8 ms (range_bin × num_vertical 곱).
- `_process_rays_with_shadow` 의 ray loop 에서 frame 당 256 bearings × 호출 → frame proc_time 의 ~70 % 점유.
- 핵심 비용은 **inner loop 의 Python scalar 산술 + numpy roundtrip** (`np.array([...])` 객체 생성 + `T @ pt` 4×4 matmul × 수천 회).

**P1-3 (master spec §3 Phase D)**: ray 별 inner loop vectorize → bearing 별 (x,y,z) numpy 배열 일괄 누적 후 `T @ batch` 한 번에 transform.

Q-D1 (사용자 확정 2026-05-03): **A안 먼저 시도, 결과 보고 후 B안 결정**. 본 PR 은 A안만 다룬다.

## 2. 범위

| Sub | 항목 ID | 위치 | 변경 |
|-----|---------|------|------|
| **D-1** | P1-3 (free space) | `scripts/3d_mapper.py:561-587` | inner double loop → `np.arange` 기반 (range, vertical_angle) 메쉬 → bearing 별 (x,y,z) 배치 → `T @ batch` 1 회 |
| **D-2** | P1-3 (occupied) | `scripts/3d_mapper.py:591-633` | 위와 동일 패턴, intensity 마스크는 boolean indexing 으로 적용 |
| **D-3** | (회귀) | `tests/test_process_sonar_ray_vectorization.py` (신규) | scalar vs vectorized 결과 동일성 unit test (golden fixture) |

**범위 외**:
- `_process_rays_with_shadow` 자체의 bearing 외부 루프 (D 후속, master spec §3 Phase D 의 "또는 C++ ray-cast 이관" 영역).
- Shadow region 검사 (`is_in_shadow_region`) — 이미 Phase B-1 에서 binary search 로 최적화됨.
- C++ ray-cast 이관 (B안). A안 측정 결과 보고 후 별도 PR.
- `_first_hit_index` (Phase B-1 에서 vectorize 완료).

## 3. 결정과 근거

| 결정 | 근거 |
|------|------|
| free space / occupied 를 분리해 vectorize | 두 루프는 sampling step (10 vs 1)·voxel resolution multiplier (4× vs 1.5×)·intensity 마스크 유무가 달라 단일 vectorize 로 묶으면 가독성 손해. 분리해도 transform 호출 횟수는 동일하므로 성능 손해 없음 |
| `np.outer` / `np.einsum` 대신 broadcasting | shape 가 (n_range, n_vertical) 로 작아 (보통 ~50 × ~10), 명시적 broadcasting 이 가독성 우위. `T @ batch` 한 번이면 충분 |
| `updates: List[Tuple[...]]` 반환 형태 유지 | 호출자 (`_process_rays_with_shadow`) 가 list 를 그대로 OctreeStorage 에 흘려보내며 downstream 변경 없음. tuple 풀어내는 비용은 무시할 수준 |
| 회귀 검증: jaccard ≥ 0.99 + golden unit test | master spec §3 Phase D 임계 그대로. unit test 는 random fixture (n_range=512, intensity_profile + bearing 100 개) → scalar vs vectorized 결과 byte 단위 비교 |
| 처리량 임계 ≥ 1.5× baseline | master spec §3 Phase D A안 임계. 미달 시 사용자 보고 후 B안 결정 |

**핵심 위험**:
1. **부동소수점 오차** — scalar 와 vectorized 가 산술 순서 차이로 ULP 단위 다르면 voxel key 가 경계에서 갈릴 수 있음. → unit test 에서 voxel key (`world_to_key`) 까지 비교, jaccard 측정으로 누적 영향 검증.
2. **메모리 할당** — frame 당 (256 bearing × ~50 range × ~10 vertical) ≈ 128k point. dtype 통일 (`float64`) 로 reallocation 회피.
3. **의도하지 않은 동작 변화** — 기존 코드의 `vertical_spread = range_m * tan(half_aperture)` 이 `num_vertical = max(1, int(...))` 로 정수화되는데, vectorize 시 r_idx 별 num_vertical 가 다르므로 단순 broadcasting 불가 → range 별 num_vertical 의 max 로 padding + boolean mask 처리 (§4 참조).

## 4. 알고리즘 (A안 핵심)

### 4.1 Free space (sparse, sampling step 10)

기존 (scalar):
```python
for r_idx in range(0, first_hit_idx, 10):
    range_m = r_idx * range_resolution
    vertical_spread = range_m * tan(half)
    num_vertical = max(1, int(vertical_spread / (voxel_resolution * 4)))
    for v_step in range(-num_vertical, num_vertical + 1):
        vertical_angle = (v_step / num_vertical) * half
        x = range_m * cos(va) * cos(ba)
        y = -range_m * cos(va) * sin(ba)
        z = range_m * sin(va)
        pt_world = T @ [x, y, z, 1]
        updates.append((pt_world[:3], log_odds_free, 'free', None))
```

A안 (vectorized):
```python
r_idx = np.arange(0, first_hit_idx, 10)              # (R,)
range_m = r_idx * range_resolution                   # (R,)
num_vert = np.maximum(1, (range_m * tan(half) / (voxel_resolution * 4)).astype(int))  # (R,)

# Build (R, V_max+1) flat arrays with mask
V_max = int(num_vert.max())
v_steps = np.arange(-V_max, V_max + 1)               # (2V_max+1,)
mask = np.abs(v_steps[None, :]) <= num_vert[:, None] # (R, 2V_max+1)
vertical_angle = (v_steps[None, :] / num_vert[:, None]) * half  # (R, 2V_max+1)

cos_va = np.cos(vertical_angle); sin_va = np.sin(vertical_angle)
range_2d = range_m[:, None]
x = range_2d * cos_va * cos(ba)         # (R, 2V_max+1)
y = -range_2d * cos_va * sin(ba)
z = range_2d * sin_va
pts_sonar = np.stack([x, y, z, np.ones_like(x)], axis=-1)[mask]  # (N, 4)
pts_world = (T @ pts_sonar.T).T[:, :3]                           # (N, 3)
updates.extend((p, log_odds_free, 'free', None) for p in pts_world)
```

`mask` 는 r_idx 별 num_vertical 차이를 흡수. 외부 변경 없이 `updates` 리스트 형태 동일.

### 4.2 Occupied (dense, sampling step 1, intensity threshold)

기존 (scalar):
```python
for r_idx in range(first_hit_idx, first_hit_idx + 50):
    intensity = intensity_profile[r_idx]
    if intensity <= intensity_threshold: continue
    range_m = r_idx * range_resolution
    if range_m < min_range: continue
    if range_m > max_range: break
    ... (free space 와 동일한 vertical loop, voxel*1.5)
```

A안 (vectorized):
```python
r_end = min(first_hit_idx + 50, len(intensity_profile))
r_idx = np.arange(first_hit_idx, r_end)               # (R,)
range_m = r_idx * range_resolution                    # (R,)
mask_range = (range_m >= min_range) & (range_m <= max_range)
mask_int   = intensity_profile[r_idx] > intensity_threshold
mask_r = mask_range & mask_int                        # (R,)
... (mask_r 로 R 차원 사전 필터, 이후 free 와 동일 broadcasting)
```

**break 조건 보존**: scalar 코드의 `if range_m > max_range: break` 는 r_idx 가 증가하므로 첫 번째 위반 이후 모두 제외 — boolean mask `range_m <= max_range` 가 동일 결과를 만든다 (range_m 단조 증가).

### 4.3 부동소수점 결정성

`np.cos(scalar)` 와 `np.cos(array)[i]` 는 동일한 ULP 결과를 보장하지 않을 수 있다. 그러나:
- IEEE 754 round-to-nearest 하에서 동일 입력의 `cos` 결과는 동일.
- 차이는 `T @ pt_4d` 의 행렬 곱 순서에서만 가능 — `T @ batch.T` 는 행 단위 dot product 이므로 scalar 와 동일 순서.

→ unit test 에서 `np.allclose(scalar_result, vectorized_result, rtol=0, atol=0)` 로 bit-exact 검증.

## 5. 검증

| 단계 | 임계 | 측정 |
|------|------|------|
| Unit test | scalar vs vectorized **bit-exact** (atol=0) | `tests/test_process_sonar_ray_vectorization.py` — fixture: bearing 100 종 × random intensity_profile, voxel key 까지 비교 |
| Build | colcon build PASS, pytest PASS | `colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && pytest tests/` |
| 회귀 정확도 | jaccard ≥ 0.99 vs main HEAD (P-2) | `scripts/regression/regression_test.sh` 그대로 |
| 회귀 처리량 | avg_proc_time candidate ≤ baseline / 1.5 | 동일 스크립트의 `metric.json` `avg_proc_time_ms` |

처리량 미달 시:
- 사용자에게 측정값 보고
- B안 (C++ ray-cast 이관) 진행 여부 합의 (Q-D1)

## 6. 작업 환경

- 브랜치: `refactor/phase-d-vectorization` (main 에서 분기, PR #6 머지 이후)
- 빌드: `cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash`
- 회귀 인프라: `scripts/regression/regression_test.sh` (B-1.0 자산)
- 환경 함정 (DDS / SIGINT / cpp dlopen): `reference_sonar3d_env_pitfalls.md` 참조

## 7. 다음 단계

본 PR 머지 후:
- A안 측정 결과를 master spec §3 Phase D 의 "측정 결과" 절에 갱신.
- 처리량 ≥ 1.5× 달성 시: Phase D 종결, master spec §3 Phase D 의 P1-3 항목을 ✅로 마크.
- 처리량 미달 시: B안 (C++ ray-cast 이관) spec/plan 작성, 별도 PR.
