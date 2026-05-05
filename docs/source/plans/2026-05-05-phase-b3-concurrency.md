# Phase B-3: Concurrency Fixes Implementation Plan

**Goal:** out-of-core tile cache 의 재귀 lock 위험을 제거하고 (P0-1), `3d_mapper_node` 의 콜백을 `MultiThreadedExecutor` + 콜백 그룹으로 직렬화 해소 (P1-1) 한다. 두 sub-phase 는 단일 branch (`refactor/phase-b3-concurrency`) 위 별 commit 로 분리한다.

**Architecture:** B-3a 는 cpp side, B-3b 는 Python side. 두 fix 가 함께 적용됐을 때만 의미: B-3b 가 콜백을 동시 실행 가능하게 만들고, 그때 B-3a 가 outofcore mapper 의 lock 충돌을 막는다. 한 쪽만 머지하면 표면 동작은 변하지 않으나 다른 쪽의 race 노출이 잠재.

**게이트:** Phase B-2 PR #4 머지 완료 (`a43af15`). SLAM quality gating PR #2 머지 완료 (`f45ac5c`). 통과.

---

## Scope (Spec §3 Phase B-3)

| Sub | 항목 ID | 위치 | 변경 |
|-----|---------|------|------|
| **B-3a** | P0-1 | `cpp/outofcore_tile_mapper.{h,cpp}` | `get_or_load_tile()` 의 lock 제거 → `_unlocked` 헬퍼 분리. 모든 외부 진입점만 `cache_mutex_` lock |
| **B-3b** | P1-1 | `scripts/3d_mapper_node.py` | `rclpy.spin(node)` → `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup` (odom) + `MutuallyExclusiveCallbackGroup` (sonar) |

## 검증 임계 (Spec §4.4 — Phase B-3)

| Metric | 임계 | 의미 |
|--------|------|------|
| 동시성 | helgrind/TSan race 0 | 가능한 경우 |
| 시간 동기화 | `[TimeSync] stamp_diff` 분포 분산 ↓ | odom 콜백이 더 자주 latest 갱신 |
| 드롭률 | `_sync_drop_count` ≤ baseline | 회귀 없음 |
| 처리량 | 변화 없음 또는 ↑ | MultiThreadedExecutor 효과 |
| 정확도 | jaccard ≥ 0.99 (P-2) | 결과 동일성 |

같은 phase B-1/B-2 와 동일하게 same-code variance ≈ 0.18 가 floor 라 정확도 임계는 단위 테스트 + smoke test 로 보강.

---

## Tasks

### Task 1 — B-3a: out-of-core lock split (P0-1)

**위치**: `cpp/outofcore_tile_mapper.cpp`

- [ ] `get_or_load_tile_unlocked(const TileIndex& idx)` 추가 — 기존 본문에서 `lock_guard` 만 제거
- [ ] `get_or_load_tile(const TileIndex& idx)` 는 `lock_guard` 잡고 `_unlocked` 호출
- [ ] 외부 호출 (preload_region, batch_update_iwlo 등 line 71/169/204/367/488/659/736) 은 자기 책임으로 `cache_mutex_` lock 잡고 `_unlocked` 호출. 또는 기존 그대로 `get_or_load_tile` (자체 lock) 사용 — 호출 컨텍스트 별 분류:
  - **자체 lock 없는 컨텍스트** (외부 entry 함수, public API): `get_or_load_tile` 그대로
  - **이미 cache_mutex_ 잡은 컨텍스트** (lock 잡힌 후 cache 접근): `_unlocked` 호출
- [ ] `outofcore_tile_mapper.h` 에 `_unlocked` 선언 (private)
- [ ] colcon build PASS
- [ ] 단위 테스트 11~14 PASS (기존)
- [ ] commit `fix(outofcore): split get_or_load_tile into locked/unlocked variants (P0-1, B-3a)`

### Task 2 — B-3b: MultiThreadedExecutor + callback groups (P1-1)

**위치**: `scripts/3d_mapper_node.py:786-791` (`main` 함수 영역, `rclpy.spin`)

- [ ] `from rclpy.executors import MultiThreadedExecutor` import 추가
- [ ] `from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup` import
- [ ] 노드 `__init__` 에 `self.odom_cbg = ReentrantCallbackGroup()` (200Hz odom 은 동시 처리 OK), `self.sonar_cbg = MutuallyExclusiveCallbackGroup()` (sonar callback 은 직렬화 — 같은 mapper 객체에 동시 진입 금지) 정의
- [ ] `create_subscription` 호출에 `callback_group=self.odom_cbg` (odom), `callback_group=self.sonar_cbg` (sonar/range/confidence) 전달
- [ ] `create_timer` 도 sonar_cbg 에 두어 publish 가 sonar 처리와 직렬화
- [ ] `main()` 에서 `rclpy.spin(node)` → `executor = MultiThreadedExecutor(num_threads=4)` + `executor.add_node(node)` + `executor.spin()`
- [ ] colcon build PASS
- [ ] smoke test: `bash scripts/regression/regression_test.sh candidate` 한 번 — 정상 종료 + cloud bag messages > 0
- [ ] commit `fix(3d_mapper_node): use MultiThreadedExecutor + callback groups (P1-1, B-3b)`

### Task 3: CHANGELOG + push + PR

- [ ] `CHANGELOG.md` 상단에 Phase B-3 단일 항목 추가 (B-3a/3b sub-bullet)
- [ ] 측정 결과 + same-code variance 메모 (B-1/B-2 동일 패턴)
- [ ] `git push -u origin refactor/phase-b3-concurrency`
- [ ] `gh pr create` — base main, head refactor/phase-b3-concurrency

---

## 위험

- **B-3b 콜백 그룹 mismatch**: SLAM quality gating PR #2 가 추가한 confidence 콜백이 어느 그룹에 들어가야 하는지 — 현재 코드 확인 시 default group 일 가능성 높음. 명시적으로 `sonar_cbg` 에 묶어 sonar 와 직렬화 (intent: confidence 변경이 진행 중인 sonar callback 을 가로채면 안 됨).
- **B-3a `_unlocked` 분류 실수**: 호출 컨텍스트 별 lock 보유 여부를 잘못 판단하면 race 또는 데드락. 변경 전후로 모든 `get_or_load_tile` 호출 위치를 검토 후 명시적 분류표를 commit body 에 기재.

## 다음 phase

- **C** (algorithm unify, 연속형 IWLO). Q-C1 확정 사양 그대로.
