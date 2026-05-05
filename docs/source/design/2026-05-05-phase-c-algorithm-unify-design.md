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

## 2. 범위 (Q-C1, master §3 Phase C — C-c 분리)

| Sub | 항목 ID | 위치 | 변경 |
|-----|---------|------|------|
| **C-a** | P2-6 | `cpp/octree_storage.cpp:249-260` | `load_iwlo_meta` 루프 내부 `ifs.good()` 검사 → 손상 감지 시 `iwlo_meta_.clear()` + `false` 반환 |
| **C-b** | P2-5 | `cpp/octree_storage.cpp:276-286` + 헤더 `dirty_keys_` 멤버 | `dirty_keys_` 추적 → `sync_to_octree` 는 변경된 voxel 만 재반영. clear/load 시 `dirty_keys_` 도 초기화 |
| ~~C-c~~ | ~~P2-2~~ | — | **본 PR 범위에서 제외** (2026-05-05 회귀 측정 결과). §6 참조 |

**범위 외**:
- `IWLOUpdater::compute_delta_log_odds` 자체는 변경하지 않는다 (이미 연속형).
- `OutofcoreTileMapper` 자체 — 위 세 파일을 통해 간접 수혜 (Tile 이 통일된 알고리즘 사용).
- `process_sonar_ray` 벡터화 (Phase D 영역).
- **P2-2 (이중 알고리즘 통일)** — 본 PR 분리, 별도 spec 으로 재설계 (§6).

## 3. 결정과 근거

| 결정 | 근거 |
|------|------|
| C 를 단일 PR 로 진행 (C-a/b/c sub-commit) | refactor-workflow.md §"한 phase 의 11단계 절차" — sub-phase 는 commit 단위, squash 시 묶음 |
| C-a → C-b → C-c 순서 | 비용/위험 오름차순. C-a 는 한 줄 방어, C-b 는 부수효과 없는 최적화 (결과 동일), C-c 는 결과 의도 변화 (가장 큰 위험) |
| `dirty_keys_` 는 `std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash>` 사용 | `iwlo_meta_` 와 동일 키 타입. `update_voxel` / `batch_update_iwlo` 가 메타 변경 시 `dirty_keys_.insert(key)` 호출 |
| C-c 변경 시 `adaptive_scale` / `weights(i)` 곱은 `compute_delta_log_odds` 호출 *후* 적용 | `IWLOUpdater::compute_delta_log_odds` 는 이미 내부에 `compute_adaptive_scale` 를 호출하므로 호출자 측 `adapt_scale` 중복 적용은 제거. 단 `weights(i)` (P0-5) 는 호출자 책임 곱셈으로 남김 |

**핵심 위험**: C-c 의 결과 변화. 임계 부근 (`intensity ≈ intensity_threshold`) voxel 들이 이진 → 연속 전환되면서 log-odds 가 다르게 누적된다. 이는 의도된 변화이며 plot 비교에서 노이즈 voxel 의 부드러운 전이로 증명한다 (master spec §3 P2-2 임계 jaccard ≥ 0.95).

## 4. 검증 (master spec §4.4 부분 적용)

| Sub | 임계 | 측정 |
|-----|------|------|
| C-a | (단위 테스트) corrupt file 입력 시 `false` 반환 + `iwlo_meta_` 비어있음 | cpp gtest 3 케이스. 기존 `iwlo_meta` round-trip 테스트도 PASS |
| C-b | jaccard ≥ 0.99 vs main HEAD baseline (P-2) | `dirty_keys_` 가 sync 결과를 변경시키지 않음을 검증 |

전체 PR 임계: **결과 동일성 (jaccard ≥ 0.99)**. C-a/b 모두 결과 보존 변경.

회귀 dataset: P-2 (`m3000d-range15-tilt90`) 단일. P-1 은 TimeSync 0.21s 함정으로 사용자 결정에 따라 deferred.

## 5. 작업 환경

- 브랜치: `refactor/phase-c-algorithm-unify` (main 에서 분기, 본 PR 시점 4 commit: spec + plan + C-a + C-b)
- 빌드: `colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release`
- 회귀 인프라: `scripts/regression/regression_test.sh` 그대로 재사용 (B-1.0 자산)

## 6. C-c 분리 결정 (2026-05-05)

**측정 결과** (P-2 dataset, 90s, fast_lio odom):

| | baseline (main 2facddf) | candidate (C-c 적용) |
|---|---|---|
| 처리 frame | 99 | 98 |
| 누적 voxel | 76,858 | 76,070 |
| **occupied voxel (≥0.5 prob)** | **21,776 (28.3%)** | **0 (0%)** |

baseline 대비 candidate 의 occupied voxel 이 0 — 즉 90s window 내 한 voxel 도 occupancy threshold 를 못 넘었다. spec §1.2 사용자 핵심 제약 "맵 결과는 더 좋아야 한다" 정면 위반 → 머지 차단 (spec §6.1).

**원인**: `tilt_90.yaml` preset 의 `L_occ=7.0` / `L_free=-10.0` 비대칭 + `intensity_threshold=85` + `sharpness=0.1`. Continuous form `delta = alpha * scale * (17w − 10)` 의 break-even 이 `w ≈ 0.59` → 매우 강한 intensity 가 일관되게 들어와야만 occupancy 형성. Legacy binary form 은 임계 초과 voxel 에 항상 양수 delta 를 적용해 occupancy 가 정상 형성됐다.

**결정**: C-c 는 본 PR 에서 분리. P2-2 (이중 알고리즘 통일) 는 IWLOParams (특히 `L_occ` / `L_free` 비대칭) 재튜닝과 함께 별도 spec 으로 재설계. preset 5 종 (`tilt_30/60/90`, `robot_detect_*`) 영향 분석 + 회귀 측정 포함.

**본 PR 의 변경 (C-a + C-b)**: 결과 보존 (jaccard ≥ 0.99) — Q-C1 결정 외, 마이너 결함 정리.

## 7. 다음 단계

본 PR 머지 후:
- Phase C-c 재설계 (P2-2 + IWLOParams 재튜닝): 별도 spec / plan / PR. master spec §3 Phase C 의 P2-2 항목 갱신.
- Phase D (vectorization, P1-3): Q-D1 정책에 따라 A안 → B안 결정. 별도 spec / plan / PR.
