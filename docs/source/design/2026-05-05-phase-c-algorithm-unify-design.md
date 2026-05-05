# Phase C — Algorithm Unification (design)

> **상위 spec**: [`2026-05-03-quality-perf-uplift-design.md`](2026-05-03-quality-perf-uplift-design.md) §3 Phase C.
> 본 문서는 master spec 의 Phase C 절을 단일 PR 단위로 좁혀 옮긴 thin design doc 이다. 결정사항 (Q-C1) 은 master 그대로 따른다.

## 1. 배경

`sonar_3d_reconstruction` 의 IWLO 갱신 경로는 세 곳에 흩어져 있다.

- `cpp/iwlo_updater.cpp::compute_delta_log_odds` — **연속형** IWLO. README §IWLO 알고리즘 절 (L.284-295) 과 일치.
- `cpp/probability_updater.cpp::batch_update_iwlo` (line 355-389) — **이진 분기** (`intensity > intensity_threshold` 시 occupied path, 아니면 free path). 연속형 `compute_delta_log_odds` 를 호출하지 않고 자체 로직 사용.
- `cpp/tile.cpp::update_voxel` (line 93-101) — `probability_updater` 와 동일한 이진 분기.

Q-C1 (사용자 확정 2026-05-03): **연속형으로 통일**. 사유는 master spec §10 참조 — 부드러운 전이로 임계 부근 노이즈 강건성 ↑, README 알고리즘 절과 정합.

또한 `cpp/octree_storage.cpp` 에 두 건의 별개 결함이 있다.

- **P2-6** `load_iwlo_meta` (line 249-260): `for` 루프 내 `ifs.read()` 결과를 검증하지 않아 EOF 후 garbage data 가 `iwlo_meta_` 에 들어갈 수 있음. **반환 시점의 `ifs.good()` 만 보면 부분 손상 감지 불가**.
- **P2-5** `sync_to_octree` (line 276-286): `iwlo_meta_` 전체를 매 호출마다 octree 에 재반영. flush 빈도가 높은 환경에서 비효율.

## 2. 범위 (Q-C1, master §3 Phase C 그대로)

| Sub | 항목 ID | 위치 | 변경 |
|-----|---------|------|------|
| **C-a** | P2-6 | `cpp/octree_storage.cpp:249-260` | `load_iwlo_meta` 루프 내부 `ifs.good()` 검사 → 손상 감지 시 `iwlo_meta_.clear()` + `false` 반환 |
| **C-b** | P2-5 | `cpp/octree_storage.cpp:276-286` + 헤더 `dirty_keys_` 멤버 | `dirty_keys_` 추적 → `sync_to_octree` 는 변경된 voxel 만 재반영. clear/load 시 `dirty_keys_` 도 초기화 |
| **C-c** | P2-2 | `cpp/probability_updater.cpp:355-389`, `cpp/tile.cpp:93-101` | 두 메서드 내부 이진 분기 제거 → `IWLOUpdater::compute_delta_log_odds` 단일 호출로 일원화. 기존 `adaptive_*` / `weights` 관통은 그대로 유지 |

**범위 외**:
- `IWLOUpdater::compute_delta_log_odds` 자체는 변경하지 않는다 (이미 연속형).
- `OutofcoreTileMapper` 자체 — 위 세 파일을 통해 간접 수혜 (Tile 이 통일된 알고리즘 사용).
- `process_sonar_ray` 벡터화 (Phase D 영역).

## 3. 결정과 근거

| 결정 | 근거 |
|------|------|
| C 를 단일 PR 로 진행 (C-a/b/c sub-commit) | refactor-workflow.md §"한 phase 의 11단계 절차" — sub-phase 는 commit 단위, squash 시 묶음 |
| C-a → C-b → C-c 순서 | 비용/위험 오름차순. C-a 는 한 줄 방어, C-b 는 부수효과 없는 최적화 (결과 동일), C-c 는 결과 의도 변화 (가장 큰 위험) |
| `dirty_keys_` 는 `std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash>` 사용 | `iwlo_meta_` 와 동일 키 타입. `update_voxel` / `batch_update_iwlo` 가 메타 변경 시 `dirty_keys_.insert(key)` 호출 |
| C-c 변경 시 `adaptive_scale` / `weights(i)` 곱은 `compute_delta_log_odds` 호출 *후* 적용 | `IWLOUpdater::compute_delta_log_odds` 는 이미 내부에 `compute_adaptive_scale` 를 호출하므로 호출자 측 `adapt_scale` 중복 적용은 제거. 단 `weights(i)` (P0-5) 는 호출자 책임 곱셈으로 남김 |

**핵심 위험**: C-c 의 결과 변화. 임계 부근 (`intensity ≈ intensity_threshold`) voxel 들이 이진 → 연속 전환되면서 log-odds 가 다르게 누적된다. 이는 의도된 변화이며 plot 비교에서 노이즈 voxel 의 부드러운 전이로 증명한다 (master spec §3 P2-2 임계 jaccard ≥ 0.95).

## 4. 검증 (master spec §4.4 그대로)

| Sub | 임계 | 측정 |
|-----|------|------|
| C-a | (단위 테스트) corrupt file 입력 시 `false` 반환 + `iwlo_meta_` 비어있음 | 신규 unit test `tests/test_octree_storage_load.py` 또는 cpp gtest. 기존 `iwlo_meta` round-trip 테스트는 그대로 PASS |
| C-b | jaccard ≥ 0.99 vs main HEAD baseline (P-2) | bit-exact 가 아니라도 동등성 — `dirty_keys_` 가 sync 결과를 변경시키지 않음을 검증 |
| C-c | jaccard ≥ 0.95 (P-2) + plot 비교에서 임계 부근 voxel 부드러운 전이 확인 | 결과 의도 변화 |

전체 PR 임계: **C-a/b 머지 단계는 결과 동일성 (jaccard ≥ 0.99), C-c 머지 단계는 결과 변화 허용 (≥ 0.95)**. PR 본문은 commit 별 metric 표 + plot 첨부.

처리량 임계: avg ≤ baseline. C-b 는 flush 빈도 높을 때 ↑.

회귀 dataset: P-2 (`m3000d-range15-tilt90`) 단일. P-1 은 TimeSync 0.21s 함정으로 사용자 결정에 따라 본 phase 에서도 deferred (project memory `project_sonar3d_audit_state.md` 명시).

## 5. 작업 환경

- 브랜치: `refactor/phase-c-algorithm-unify` (이미 main 에서 분기됨, 0 commit)
- 빌드: `colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 회귀 인프라: `scripts/regression/regression_test.sh` 그대로 재사용 (B-1.0 자산)

## 6. 다음 단계

본 design 승인 → writing-plans skill 로 implementation plan 작성 → C-a → C-b → C-c → CHANGELOG + push + PR.

Phase D (vectorization) 는 본 PR 머지 후 별도 spec/plan/PR 로 진행 (master spec §8 게이트, refactor-workflow.md "한 PR 에 여러 phase 묶기" 안티패턴 회피).
