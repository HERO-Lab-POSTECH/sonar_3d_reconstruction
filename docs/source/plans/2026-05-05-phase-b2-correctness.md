# Phase B-2: Correctness Fixes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 5건의 P0 정확도 결함을 sub-phase 단위(B-2a~B-2e)로 분리해 수정한다. 각 sub-phase는 단독 commit 으로 분리해 회귀 원인 추적이 가능하도록 하고, squash 머지 시 한 phase 로 묶는다. 머지 후 baseline 대비 결과는 **개선 또는 동률**(jaccard ≥ 0.95) 이어야 하며, 변화 방향이 회귀가 아닌 개선임을 plot 으로 증명한다.

**Architecture:** Phase B-1 에서 자산화한 회귀 인프라(`scripts/regression/`)를 그대로 재사용한다. **데이터셋은 P-2 단일** 로 시작하고, P-1 (`m3000d-range20-tilt30`)은 sonar-livox stamp_diff ≈ 0.21s 가 현재 TimeSync 임계 (`time_sync.max_diff = 0.1s`)를 초과해 baseline 단계에서 0/0 frame 으로 처리되는 문제가 있다. **B-2a 사전 작업** 으로 dataset 매트릭스를 점검하고, 필요 시 임계 완화 또는 P-1 교체로 정식화한다 (spec §4.1 Notes 참조).

각 sub-phase 는 다음 4축을 동기 갱신한다 (`.claude/rules/refactor-workflow.md`):
1. **branch** — Phase 단위 단일 branch (`refactor/phase-b2-correctness`), sub-phase 는 commit 분리
2. **commit message** — Conventional Commits + 본문에 sub-phase ID
3. **CHANGELOG.md** — 5 sub-phase 모두 한 항목으로 묶음
4. **PR description** — sub-phase 별 요약 + 회귀 결과 첨부

**Tech Stack:** C++17 (NaN 가드), Python 3.10 + NumPy (mask broadcast, IWLO weight), `rclpy` QoS API (RELIABLE + TRANSIENT_LOCAL), `bisect` (Python stdlib).

---

## 게이트 (시작 전 체크)

- [ ] Phase B-1 PR #3 머지 완료 (`gh pr view 3 --json state` → MERGED)
- [ ] `git fetch origin && git checkout main && git pull` 후 main HEAD 가 B-1 squash commit
- [ ] `git checkout -b refactor/phase-b2-correctness`
- [ ] Phase B-1 회귀 인프라 동작 확인 (`bash scripts/regression/regression_test.sh baseline` 한 번 — 약 90s)

## Scope (Spec §3 Phase B-2 / §4.4)

| Sub | 항목 ID | 위치 | 변경 | 검증 임계 |
|-----|--------|------|------|-----------|
| **B-2a** | P0-2 | `cpp/iwlo_updater.cpp` | `compute_intensity_weight` 분모 0-division 가드 | 단위 테스트 + 측정 통과 |
| **B-2b** | P0-7 | `scripts/3d_mapper_node.py:181-247` | sub/pub QoS 분리. `tile_indices` 는 `RELIABLE + TRANSIENT_LOCAL` | RViz 누락 카운트 비교 |
| **B-2c** | P0-4 | `scripts/3d_mapper.py:_collect_first_hits` | `depth_filter_mask` 인자 추가 | jaccard ≥ 0.95 + 시각 비교 |
| **B-2d** | P0-5 | `scripts/3d_mapper.py:_apply_updates_to_octree`/`process_sonar_image` 영역 | voxel multiplicity → IWLO weight 전달 (Q-B1 확정) | jaccard ≥ 0.95 + 시각 비교 (수렴 속도 ↑) |
| **B-2e** | P0-8 | `scripts/3d_mapper.py:is_in_shadow_region` | 수동 binary search → `bisect_left/right` 범위 순회 | jaccard ≥ 0.99 (결과 동일성 강제) |

## 검증 임계 (Spec §4.4 — Phase B-2)

| Metric | 임계 | 의미 |
|--------|------|------|
| `jaccard_set` (B-2a/2b/2e) | **≥ 0.99** | 거의 동일 (NaN/누락/리팩토링 fix) |
| `jaccard_set` (B-2c/2d) | **≥ 0.95** + 시각 plot | 의도된 변화 허용. 개선 증명 |
| `mean_log_odds_diff` | 의도된 변화 허용 | B-2d 수렴 속도 ↑ 가 핵심 효과 |
| `avg_proc_time_ms` | **≤ baseline** | 처리량 회귀 없음 |

데이터셋 매트릭스: 기본 **P-2** + (B-2a 후) **P-1** 정상화 시 추가.

---

## File Structure

### 수정 (sub-phase 별)

| Sub | 파일 | 라인 (예상) | 변경 요약 |
|-----|------|-------------|-----------|
| B-2a | `sonar_3d_reconstruction/cpp/iwlo_updater.cpp` | `compute_intensity_weight` 함수 본체 | `intensity_max == intensity_threshold` 케이스 ε 가드 |
| B-2a | `sonar_3d_reconstruction/cpp/iwlo_updater.h` | (필요 시) | static `kEpsilon` 상수 |
| B-2a | `tests/test_iwlo_intensity_guard.py` (신규) | — | 1-2 PASS smoke |
| B-2b | `scripts/3d_mapper_node.py` | 181-247 | sub/pub QoS profile 분리, `tile_indices` 별도 RELIABLE+TRANSIENT_LOCAL |
| B-2b | `scripts/3d_mapper_node.py` | (해당 .yaml 호출자) | tile_indices publisher 인자 변경 |
| B-2c | `scripts/3d_mapper.py:_collect_first_hits` | 655-684 | signature 에 `depth_filter_mask: Optional[np.ndarray] = None` 인자 추가 + 마스크 적용 |
| B-2c | `scripts/3d_mapper.py:_process_rays_with_shadow` | 686-737 | 호출 측 `depth_filter_mask` 전파 |
| B-2d | `scripts/3d_mapper.py:_apply_updates_to_octree` | 738-790 | `(point, type)` 빈도 카운트 → IWLO weight 로 전달 |
| B-2d | `sonar_3d_reconstruction/cpp/python_bindings.cpp` | (필요 시) | `batch_update_iwlo` 에 `weight` 인자 신설 (호환 default = 1.0) |
| B-2d | `sonar_3d_reconstruction/cpp/iwlo_updater.{h,cpp}` | `compute_delta_log_odds` | `weight` 매개변수 신규 받기 (default 1.0) |
| B-2e | `scripts/3d_mapper.py:is_in_shadow_region` | 460-525 | 수동 bisect → `bisect_left/right` + slice 순회 |
| 모두 | `CHANGELOG.md` | top | Phase B-2 단일 항목 (5 sub-bullet) |

### 회귀 측정 (재사용)

`scripts/regression/regression_test.sh` 와 `regression_compare.py` 그대로 사용. sub-phase 마다 baseline 과 candidate 디렉토리만 분리:

```
/tmp/sonar3d_regression/p2/
├── baseline/        # main HEAD 빌드 (B-1 머지 직후 1회만)
├── b2a/             # B-2a 적용 후
├── b2b/             # ...
├── b2c/
├── b2d/
└── b2e/
```

---

## Tasks

### Task 0: 게이트 + dataset 매트릭스 점검

- [ ] PR #3 머지 + main HEAD 갱신 + branch 분기 확인 (위 게이트)
- [ ] `BAG_PATH=…/m3000d-range20-tilt30 bash scripts/regression/regression_test.sh baseline` 1회 측정 → frame 처리 확인
  - 0/0 이면: B-2c 시점에 `time_sync.max_diff` 임계 완화 검토 (별도 mini-task) 또는 P-1 매트릭스 잠시 빠짐 (B-2c 까지 P-2 단일)
  - 처리되면: P-1 매트릭스 합류
- [ ] (선택) 결과를 `CHANGELOG` Notes 에 기록

### Task 1 — B-2a: IWLO intensity 0-division guard (P0-2)

**위치**: `sonar_3d_reconstruction/cpp/iwlo_updater.cpp` `compute_intensity_weight`

- [ ] `iwlo_updater.cpp` `compute_intensity_weight` 분모 (`intensity_max - intensity_threshold`) 가 `< kEpsilon` (예: `1e-9`) 이면 `0.0` (또는 sigmoid 입력 0) 반환
- [ ] 헤더 또는 cpp 파일 상단에 `kEpsilon = 1e-9` 도입
- [ ] 단위 테스트 `tests/test_iwlo_intensity_guard.py` 신규: `intensity_threshold == intensity_max` 케이스에서 NaN/Inf 미발생, finite 결과만 반환
- [ ] colcon build PASS
- [ ] 단위 테스트 PASS
- [ ] P-2 측정 → `b2a` 디렉토리, baseline 비교 jaccard ≥ 0.99 (정상 데이터에서는 가드 미발동)
- [ ] `git commit -m "fix(iwlo): guard against zero-range intensity normalization (P0-2, B-2a)"` (Task 7 후 squash 시 묶음)

### Task 2 — B-2b: QoS separation for tile_indices (P0-7)

**위치**: `scripts/3d_mapper_node.py:181-247`

- [ ] sub QoS profile (기존 그대로) 와 별도로 pub_qos_default (`KEEP_LAST + depth=10 + reliability=…`) 정의
- [ ] `tile_indices_pub` 만 `pub_qos_latched = QoSProfile(reliability=RELIABLE, durability=TRANSIENT_LOCAL, history=KEEP_LAST, depth=1)` 사용 (RViz late-joiner 가 마지막 인덱스 받기 가능)
- [ ] PointCloud2 / MarkerArray / filtered_image / range / confidence 등은 기존 best_effort 토픽 호환 유지
- [ ] colcon build PASS
- [ ] RViz 또는 `ros2 topic echo --qos-reliability reliable --qos-durability transient_local /sonar_3d_mapper/tile_indices` 로 last value latch 확인 (1회)
- [ ] P-2 측정 → `b2b` 디렉토리, baseline 비교 jaccard ≥ 0.99
- [ ] `git commit -m "fix(3d_mapper_node): separate tile_indices QoS (RELIABLE+TRANSIENT_LOCAL) (P0-7, B-2b)"`

### Task 3 — B-2c: depth_filter_mask in _collect_first_hits (P0-4)

**위치**: `scripts/3d_mapper.py:655-684` + `_process_rays_with_shadow:686-737`

- [ ] `_collect_first_hits(self, polar_image, bearing_step, range_resolution, depth_filter_mask=None)` signature 확장
- [ ] mask 가 None 이 아니면 bearing 루프 내 `if not depth_filter_mask[b_idx]: continue` 분기 추가 (기존 fov 체크와 동일 위치)
- [ ] `_process_rays_with_shadow` 호출자가 자기 mask 를 전파
- [ ] `process_sonar_image` 내 `_collect_first_hits` 호출 위치 (현재 인자 3개) 가 mask 를 넘기도록 확인 — 마스크는 `_process_rays_with_shadow` 가 받는 것과 동일한 것
- [ ] colcon build PASS
- [ ] P-2 측정 → `b2c`, baseline 비교 jaccard ≥ 0.95 + xy/xz plot 비교 (false-negative voxel 감소 시각 확인)
- [ ] `git commit -m "fix(3d_mapper): pass depth_filter_mask into _collect_first_hits (P0-4, B-2c)"`

### Task 4 — B-2d: voxel multiplicity → IWLO weight (P0-5)

**위치**: `scripts/3d_mapper.py:_apply_updates_to_octree` (738-790) + `cpp/iwlo_updater.{h,cpp}` + `cpp/python_bindings.cpp`

- [ ] `IWLOUpdater::compute_delta_log_odds(intensity, …, double weight = 1.0)` 시그니처 확장 — `delta *= weight`
- [ ] `ProbabilityUpdater::batch_update_iwlo(points, intensities, is_occupied, weights = nullptr)` 매개변수 추가 (binding 호환 위해 default nullptr → 내부 1.0)
- [ ] `pybind11` `.def` 에 `py::arg("weights") = py::none()` 추가, NumPy → `std::vector<double>` 변환 헬퍼 보강 (None 시 length 만 N 으로 일관 보장)
- [ ] `_apply_updates_to_octree` 에서 같은 voxel key 로 들어오는 update 를 `Counter`/`defaultdict(int)` 로 집계 후 가장 많이 등장한 type/intensity 와 함께 weight 도 함께 numpy array 로 전달 (점 자체는 unique 화)
- [ ] 단위 테스트 (Python): 동일 voxel 중복 → weight 누적 동작 확인 + 단일 voxel → weight=1.0 확인
- [ ] colcon build PASS (cpp 변경 포함)
- [ ] P-2 측정 → `b2d`, jaccard ≥ 0.95 + plot 으로 수렴 속도 향상 (frame N 시점 voxel intensity 분포 차이) 증명
- [ ] `git commit -m "fix(iwlo): pass voxel multiplicity as weight (P0-5, B-2d)"`

### Task 5 — B-2e: shadow bisect refactor (P0-8)

**위치**: `scripts/3d_mapper.py:is_in_shadow_region` (460-525)

- [ ] `import bisect` 추가
- [ ] `bearing_first_hits` 가 `(bearing, range)` tuple 정렬 list 임을 활용해 `keys = [b for b, _ in bearing_first_hits]` 별도 추출 또는 `bisect.bisect_left(bearing_first_hits, (bearing_angle - tolerance, …))` 패턴 사용
- [ ] left index = `bisect_left(keys, bearing_angle - tolerance)`, right index = `bisect_right(keys, bearing_angle + tolerance)` 로 후보 slice 결정
- [ ] slice 내에서 기존 same-bearing 제외 + `voxel_range > first_hit` 조건 그대로 평가
- [ ] full overlap 모드 (전체 bearing 검사) 정확도 동일성 micro test: 100 random bearing 으로 ref 수동 bisect vs new 결과 비교 → 동일
- [ ] colcon build PASS (Python 만 변경, build 영향 없으나 install symlink 갱신 위해 한 번 수행)
- [ ] P-2 측정 → `b2e`, jaccard ≥ 0.99
- [ ] `git commit -m "refactor(3d_mapper): use bisect for shadow region search (P0-8, B-2e)"`

### Task 6: CHANGELOG + PR

- [ ] `CHANGELOG.md` 상단에 Phase B-2 단일 항목 추가, 5 sub-phase 를 sub-bullet 로 분리 기재
- [ ] sub-phase 별 측정 결과 요약 (P-2 jaccard, baseline 대비 변화 방향)
- [ ] `git add CHANGELOG.md && git commit -m "docs(changelog): record Phase B-2 correctness fixes"`
- [ ] `git push origin refactor/phase-b2-correctness`
- [ ] `gh pr create --base main --head refactor/phase-b2-correctness --title "Phase B-2: correctness fixes (P0-2/-4/-5/-7/-8)" --body …` (5 sub-phase 요약 + verification 체크박스 + plot 첨부 위치)
- [ ] PR description 에서 P-1 매트릭스 처리 결정 명시 (Task 0 결과 기반)

---

## 위험 / 알려진 이슈

- **B-2d (P0-5) cpp 시그니처 확장**: `IWLOUpdater::compute_delta_log_odds` 의 weight 매개변수가 default = 1.0 이라 호출자 호환은 보장되나, pybind11 binding 의 인자 검증 (`py::arg`) 정합성 단위 테스트 필수.
- **B-2c (P0-4) mask 전파**: `_process_rays_with_shadow` 와 `_collect_first_hits` 가 같은 mask 인스턴스를 공유하는지 코드 흐름 검증 — 호출 컨텍스트 마다 mask 가 다르면 first_hits 와 ray loop 가 일관되지 않음.
- **P-1 dataset 0/0**: Task 0 에서 처리되지 않으면 전체 phase 가 P-2 단일로 진행 — spec §4.1 Notes 그대로 반영.
- **same-code measurement variance**: Phase B-1 에서 발견된 jaccard ≈ 0.82 same-code 변동 — B-2 임계 ≥ 0.95 는 이 floor 보다 위라 의미 있는 비교 가능. 단, 같은 측정을 2회 수행해 average jaccard 보고 권장 (sub-phase 5건 × 2회 = 10회 추가 측정).

## 안티패턴 (피하기)

- ❌ 5 sub-phase 를 한 commit 으로 묶기 — 회귀 원인 추적 불가
- ❌ baseline 측정 생략하고 `b2a`만 — 본 phase 와 main 사이 의미 단절
- ❌ B-2d 의 weight 도입을 cpp side 에서 default = 0 으로 두기 — 기존 호출자가 silent zero update 됨

## 다음 phase

- **B-3**: P0-1 (cpp lock 분리) + P1-1 (MultiThreadedExecutor). SLAM gating PR #2 머지 완료 (게이트 통과).
