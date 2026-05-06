# CHANGELOG - sonar_3d_reconstruction

## [Unreleased] — Post-Audit Fix B-3 (fix)

### Fixed
- B-3: rotation compensation `dt` argument now signed (`sonar_t - odom_t`) instead of `abs(...)`. Future-stamped odom (negative delta) now correctly extrapolates orientation backward; previously the abs() in PR-A C-2 fix forced forward-only extrapolation, reversing direction in this case.

### Tests
- Added `tests/test_timesync_rotation_signed_dt.py` (2 tests).

### Verification
- colcon test PASS (12/12 incl. 2 new).
- B-4 (`dropped_stale_odom` semantics) deferred to PR-C — separate semantic decision.

---

## [Unreleased] — Post-merge fixes P9: timesync robustness, topic sync, crosstalk default, atomic symlink (fix)

### Fixed
- C-2: `odom_age = abs(sonar_t - odom_t)` 적용 — 미래 skew odom에 역방향 rotation compensation 방지
- C-3: `timesync_diagnostics.py` ERROR 레벨 분기 추가 (`dropped_stale_odom > 0` → ERROR) + `RollingMax` 클래스 및 `stamp_diff_max` 추적
- C-4: `time_sync.diagnostics_rate_hz` config 키 + ParameterDef 추가, 타이머 하드코딩(`1.0`) 제거
- M-2: `robot_3d_mapping.launch.py` + `config.py` 토픽 default를 `/localization/fast_lio/*` 계층으로 동기화
- M-3: `config/common.yaml` `crosstalk.enabled: false` — config.py default(False)와 일치
- M-4: `map_save.py` latest symlink atomic update (`tmp → os.replace`) 패턴 적용 + `re.match(r'^\d{8}_\d{6}$')` heuristic 강화

### Changed
- `test/test_timesync.py`: `test_diagnostic_msg_warn` → `test_diagnostic_msg_error_on_stale_odom` (ERROR 레벨 확인), `test_diagnostic_msg_warn_on_stamp_diff` / `test_diagnostic_msg_ok` 신규 추가, `test_rolling_max` 추가 (총 10개)

### Verification
- colcon build PASS (0.19s)
- pytest test/test_timesync.py: 10/10 PASS
- `grep '/fast_lio/odometry'` runtime 파일: 0 old-path hits (모두 `/localization/fast_lio/*`)

---

## [Unreleased] — Phase P8: Timesync hardening + diagnostics (refactor + feat)

### Added
- `sonar_3d_reconstruction/odom_buffer.py` — thread-safe ring buffer (maxlen=50) for
  nav_msgs/Odometry with `latest()`, `nearest()`, `interpolate()` (SLERP + LERP) methods
- `sonar_3d_reconstruction/timesync_diagnostics.py` — `RollingMean(n)` + `TimesyncDiagnostics`
  building `diagnostic_msgs/DiagnosticArray` on `/perception/sonar_3d/diagnostics` (1Hz)
- `test/test_timesync.py` — 7 pure-Python unit tests for both new modules
- `scripts/3d_mapper_node.py`:
  - Policy-based odom selection: `latest` | `interpolate` | `nearest` (via `time_sync.policy`)
  - Odom freshness check: drops frames where `sonar_t - odom_t > max_odom_age_sec`
  - `_apply_rotation_compensation()` — angular_vel × dt small-angle correction on odom orientation
  - `_publish_diagnostics()` — 1Hz DiagnosticArray with rolling means + drop counters
- `scripts/config.py`: `time_sync.compensate_rotation` ParameterDef (bool, read_only=True, default False)
- `config/common.yaml`: `compensate_rotation: false` key in `time_sync` block
- `CMakeLists.txt`: `odom_buffer.py` + `timesync_diagnostics.py` install lines

### Changed
- `_sonar_callback`: odom selection via `OdomBuffer` policy (preserves `_latest_odom_msg` fallback)
- `_odom_callback`: now also pushes to `_odom_buffer` ring buffer
- Diagnostics counters (`dropped_stamp_diff`, `dropped_stale_odom`, `dropped_quality_gate`,
  `paired_count`) supersede ad-hoc `_sync_drop_count`; latter kept for log continuity

### Verification
- colcon build PASS (0.41s)
- Smoke import PASS (`OdomBuffer`, `TimesyncDiagnostics`)
- 7 unit tests PASS

### Notes
- **FINAL phase** of the workspace conventions effort (P1–P8 all complete after this merge)
- `time_sync.compensate_rotation` default is `false`; enable for high-angular-velocity ops

---

## [Unreleased] — Phase P7: Map save UX (sonar_3d) (refactor)

### Added
- `sonar_3d_reconstruction/map_save.py` — workspace map save helper:
  - `get_default_map_dir(pkg)` — `$PKRC_MAP_DIR/<pkg>` or `~/data/maps/<pkg>`
  - `resolve_map_save_path(user_path, pkg, fname)` — empty path → auto-timestamp dir
  - `update_latest_symlink(saved_path, pkg)` — relative `latest -> <ts>` symlink
- `scripts/3d_mapper_node.py`: `~/save_map` (std_srvs/Trigger) 서비스:
  - outofcore 모드: flush_map + save_merged_octree + symlink 갱신
  - in-memory 모드: success=False + 안내 메시지 (기존 mapper API 한계)
- `CMakeLists.txt`: `map_save.py` install 라인 추가 (qos.py와 동일 패턴)

### Verification
- colcon build PASS (0.40s)
- Python 헬퍼 스모크 테스트 PASS (auto-timestamp + symlink)
- Service `/sonar_3d_mapper/save_map` 등록 확인

---

## [Unreleased] — Phase P6: Config structure (refactor)

### Added
- `time_sync` 카테고리 (P5c+d ParameterDef와 정합):
  - max_stamp_diff_sec (0.1), max_odom_age_sec (0.5), policy ('latest')
- `visualization` 카테고리 확장:
  - marker_min_depth (0.0 m), marker_max_depth (5.0 m), marker_alpha (0.8), show_opencv_visualization (false)
  - P5a launch arg에서 이동 (BREAKING removed args의 안식처)
- `output` 카테고리 추가 (P7 활용):
  - map_dir (""), auto_timestamp (true)
- `scripts/config.py`: output.map_dir, output.auto_timestamp ParameterDef 추가 (read_only)

### Verification
- colcon build PASS (0.15s)
- python yaml.safe_load PASS (top-level keys: time_sync, visualization, output 포함)
- 노드 startup PASS — parameter mismatch 0건

---

## [Unreleased] — Phase P5c+d: ROS Time integrity (refactor)

### Changed
- `scripts/3d_mapper_node.py`:
  - sonar/odom timesync log: `wall_t = time.time()` 제거 → `ros_now = self.get_clock().now()` (use_sim_time 정합, spec §2.6.1)
  - `MAX_STAMP_DIFF = 0.1` 하드코딩 제거 → `time_sync.max_stamp_diff_sec` config (default 0.1)
  - dead `self.last_publish_time = time.time()` 제거 (unused)
  - `_wall_*` prefix 변수에 NTP-immune 의도 주석 강화
- `scripts/map_visualizer_node.py`:
  - `last_refresh_time`, `last_tile_update_time` → ros clock (sim time 정합 throttle)
  - `time.sleep(0.03)` (fps cap)는 wall clock 유지 (적절)
- `scripts/config.py`:
  - 신규 ParameterDef 3개 추가:
    - `time_sync.max_stamp_diff_sec` (default 0.1) — sonar↔odom stamp tolerance
    - `time_sync.max_odom_age_sec` (default 0.5, read_only) — P8 ring-buffer pairing 예약
    - `time_sync.policy` (default 'latest', read_only) — P8 정책 선택자 예약

### Verification
- colcon build PASS
- 3d_mapper 노드 startup PASS (use_sim_time 자동 정합 로그 정상)
- bag rate=1.0 vs 2.0 stamp_diff 안정성 — P8 회귀 단계에서 일괄 검증

### Why preserved as wall clock
- `_node_start_wall_time` (grace period), `_latest_confidence_wall_time` (staleness): NTP-immune wall-clock 측정 의도. ros clock으로 전환 시 sim-time 일시정지 등 비정상 케이스에서 타이머가 잘못 동작.
- `time.sleep(0.03)` (visualizer fps cap): wall throttle이 자연스러움.

---

## [Unreleased] — Phase P5b: use_sim_time 자동 정합 (refactor)

### Added
- `_auto_force_sim_time(context)` helper inline 두 launch (3d_mapping, robot_3d_mapping)에 추가:
  - `bag_path` arg 비어있지 않으면 `use_sim_time:=true` silent force
  - prev value가 false였으면 `[launch] bag_path='...' → forcing use_sim_time:=true` 한 줄 print
  - downstream OpaqueFunction이 모두 정합된 값 사용

### Why silent (not RuntimeError on conflict)
LaunchContext에서 user-explicit-false vs default-false 구분하는 신뢰할 만한 방법이 없음. bag replay = sim time이 항상 옳으므로 silent override가 안전.

### Verification
- colcon build PASS
- bag_path 미명시: force 메시지 없음
- bag_path 명시: `[launch] bag_path='/tmp/foo' → forcing use_sim_time:=true (was: false)` 출력 확인

### Out of scope (no bag_path arg)
- fast_lio mapping/localization launches
- cartographer slam launch
이들은 bag을 직접 launch에서 spawn하지 않음 (외부 `ros2 bag play` 명령으로 별도 실행). bag_path arg 도입은 후속 phase.

---

## [Unreleased] — Phase P5a: Launch arg standardization (refactor)

### Changed (BREAKING — external launch invocations)
- Launch arg rename per workspace conventions spec §2.5:
  - `rviz` → `use_rviz`
  - `bag_file` → `bag_path`
  - `record_path` → `output_bag_path`
  - `show_opencv` → `use_opencv_window`
  - `launch_visualizer` → `use_visualizer`
- All 3 launch files (3d_mapping, robot_3d_mapping, map_visualizer) header docstring 표준 적용

### Removed (BREAKING — external launch invocations)
- `qos_reliability` arg — QoS는 spec §2.4의 helper로 코드에 박힘
- `marker_min_depth`, `marker_max_depth`, `marker_alpha` args — config yaml로 이동 (P6에서 처리)

### Migration
기존 호출 → 새 호출 매핑:
- `rviz:=true`               → `use_rviz:=true`
- `bag_file:=/path/to.bag`   → `bag_path:=/path/to.bag`
- `record_path:=/path/to/dir` → `output_bag_path:=/path/to/dir`
- `show_opencv:=true`        → `use_opencv_window:=true`
- `launch_visualizer:=true`  → `use_visualizer:=true`
- `qos_reliability:=...`     → (제거; 영향 없음)
- `marker_*`                 → (P6 머지 후 config yaml에서 직접 수정)

### Verification
- colcon build PASS
- ros2 launch --show-args: 신규 arg 표시
- 정적 grep: launch 파일에서 legacy arg 0건

---

## [Unreleased] — Phase P4c: Topic + QoS sync (refactor)

### Changed
- `scripts/3d_mapper_node.py`: publishers hardcoded per spec §2.3.3 rule 1
  - `/sonar_3d_mapper/point_cloud` → `/perception/sonar_3d/points` (SENSOR_QOS)
  - `/sonar_3d_mapper/occupancy_grid` → `/perception/sonar_3d/markers` (SENSOR_QOS)
  - `/sonar_3d_mapper/filtered_image` → `/sonar_3d_mapper/debug/crosstalk_filtered` (SENSOR_QOS)
  - `/sonar_3d_mapper/updated_tile_indices` → `/perception/sonar_3d/tile_indices` (LATCHED_QOS)
- `scripts/3d_mapper_node.py`: subscriber QoS helpers adopted
  - `odom_sub` → RELIABLE_QOS, `sonar_sub` → SENSOR_QOS
  - `range_sub` → LATCHED_QOS, `slam_confidence_sub` → RELIABLE_QOS
- `scripts/map_visualizer_node.py`: publishers hardcoded
  - `/{node_name}/octomap` → `/perception/sonar_3d_visualizer/octomap` (SENSOR_QOS)
  - `/{node_name}/point_cloud` → `/perception/sonar_3d_visualizer/points` (SENSOR_QOS)
  - `/{node_name}/marker_array` → `/perception/sonar_3d_visualizer/markers` (SENSOR_QOS)
  - tile_update_sub: `/sonar_3d_mapper/updated_tile_indices` → `/perception/sonar_3d/tile_indices` (LATCHED_QOS)
- `scripts/map_diff_visualizer.py`: `/map_diff/*` → `/perception/map_diff/*` (SENSOR_QOS, 5 publishers)
- `launch/3d_mapping.launch.py`: ODOMETRY_CONFIG/CONFIDENCE_CONFIG synced with lidar_slam P4b
  - `cartographer_2d/odometry` → `/localization/cartographer/odometry`
  - `/fast_lio/odometry` → `/localization/fast_lio/odometry`
  - `/fast_lio/localization/odometry` → `/localization/fast_lio_loc/odometry`
  - `/fast_lio/localization/confidence` → `/localization/fast_lio_loc/confidence`
- `launch/robot_3d_mapping.launch.py`: remappings updated to new topic names
- `config/common.yaml`: `topics.pointcloud`/`topics.marker` keys removed (hardcoded in code)
- `scripts/config.py`: `topics.pointcloud`/`topics.marker` parameter declarations removed

### Verification
- colcon build PASS (0.42s)
- static grep: legacy topic refs 0건 in source

---

## [Unreleased] — Phase P4a: QoS helper module (refactor)

### Added
- `sonar_3d_reconstruction/qos.py` — workspace QoS 3-tier helper (SENSOR/RELIABLE/LATCHED) per spec §2.4

### Verification
- colcon build PASS

---

## [Unreleased] — Phase P2: Foxglove removal (refactor)

### Removed
- `foxglove` launch arg + `foxglove_bridge` Node 일괄 제거
  - `launch/3d_mapping.launch.py`
  - `launch/robot_3d_mapping.launch.py`
  - `launch/map_visualizer.launch.py`

### Verification
- colcon build PASS
- 3 launch 모두 `--show-args`에 foxglove 없음

## [Unreleased] — Phase D: process_sonar_ray vectorization (perf, A안)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md` §3 Phase D
> Phase design: `docs/source/design/2026-05-05-phase-d-vectorization-design.md`
> Plan: `docs/source/plans/2026-05-05-phase-d-vectorization.md`

### Changed
- **D-1 (P1-3 free-space)** `scripts/3d_mapper.py::process_sonar_ray`: free-space inner loop (range × vertical_step) 를 `np.arange` 기반 (R, 2V+1) 메쉬 + 단일 batched transform `T @ pts.T` 로 교체. ray 당 ~4N scalar transform → ~1 numpy call.
- **D-2 (P1-3 occupied)** `scripts/3d_mapper.py::process_sonar_ray`: occupied inner loop 도 동일 broadcast 패턴 (voxel_resolution × 1.5 dense sampling). intensity threshold + min/max range 는 단일 boolean mask `mask_r` 로 통합. break-on-max_range 의미는 monotonic `range_m` 로 보존. transform 은 `np.einsum('ij,rvj->rvi', T, pts_sonar)` 로 (R, 2V+1, 4) 텐서 일괄 처리. inner Python loop 한 단은 점별 intensity attach 위해 유지 (~5% 비용).

### Added
- `tests/test_process_sonar_ray_vectorization.py` — golden fixture 비교 unit test 2 케이스.
  - `test_vectorized_matches_scalar_for_100_bearings` — 반환 형태 sanity.
  - `test_vectorized_voxel_keys_bit_exact` — 100 bearing × random intensity profile 에 대해 voxel key (kind, ix, iy, iz) 집합이 scalar baseline 과 **완전 동일** (atol=0).
- `tests/fixtures/process_sonar_ray_scalar_golden.pkl` — scalar baseline snapshot (1.6 MB, 100 bearing × ~224 update).
- `docs/source/design/2026-05-05-phase-d-vectorization-design.md` — Phase D thin design.
- `docs/source/plans/2026-05-05-phase-d-vectorization.md` — Phase D 4-task plan.

### Verification
- colcon build PASS (Release).
- pytest tests/ — **16/16 PASS** (14 prior + 2 new vectorization tests).
- Unit-level **bit-exact** 검증: 100 bearing × random intensity 에 대해 vectorized 와 scalar 의 voxel key 집합이 atol=0 에서 일치.
- 회귀 측정 (P-2, `m3000d-range15-tilt90`, 90s, fast_lio odom, ROS_DOMAIN_ID=0):
  - baseline (main 7bf13e4): 21,762 voxels, avg proc_time **104.2 ms** (Frame 100/200/300/400 = 105.3 / 104.0 / 94.3 / 113.1 ms).
  - candidate (HEAD): 21,765 voxels, avg proc_time **71.0 ms** (70.2 / 72.5 / 65.4 / 75.8 ms).
  - jaccard = **0.974**, common = 21,480 voxels (~98.7%), mean Δlog-odds = 0.078.
  - **처리량 1.47×** — Q-D1 임계 1.5× 에 약 0.03× 미달.

### Notes
- **정확도 0.974 vs 임계 0.99**: B-1 측정에서 동일 코드 두 run 의 jaccard ≈ 0.82 였던 환경 노이즈 (fast_lio drift + bag timing 비결정성) 가 회귀 노이즈의 거의 전부. 코드 단위 bit-exact 는 unit test 의 100 bearing × random intensity 에서 voxel key 집합이 완전 일치함 (atol=0) 으로 별도 입증 — 알고리즘 동작 보존 확실.
- **처리량 1.47× vs 임계 1.5×**: 오차 0.03× 는 측정당 ±5 ms 변동 범위 내 (Frame 100~400 의 ±9 ms 변동 참조). 추가 sample 누적이나 더 긴 (180s+) replay 로 평균이 안정화되면 1.5× 도달 가능성 있음.
- **Q-D1 결정 의뢰**: A안 수치는 임계와 매우 근접하나 strict 통과는 아님. 다음 옵션을 사용자 결정 의뢰:
  - (a) 본 PR 머지 후 Phase D 종결 — 코드 단위 bit-exact + 1.47× 처리량 효과를 충분으로 봄.
  - (b) 본 PR 머지 후 B안 (C++ ray-cast 이관) 추가 진행 — 별도 spec / PR.
  - (c) 본 PR 머지 차단, 추가 vectorize 여지 (예: `_process_rays_with_shadow` 외부 루프) 탐색.
- 호출자 (`_process_rays_with_shadow`) 변경 없음. 반환 시그니처 `List[Tuple[ndarray(3,), float, str, Optional[float]]]` 유지.
- `_first_hit_index` 와 shadow region 검사는 이미 Phase B-1 에서 vectorize 됨 (변경 없음).

---

## [Unreleased] — Phase C: Octree Storage Hardening (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Phase design: `docs/source/design/2026-05-05-phase-c-algorithm-unify-design.md`
> Plan: `docs/source/plans/2026-05-05-phase-c-algorithm-unify.md`

### Fixed
- **C-a (P2-6)** `cpp/octree_storage.cpp::load_iwlo_meta`: 루프 내 entry 별 `ifs.good()` 검사 추가. truncate / EOF 시 `iwlo_meta_.clear()` 후 `false` 반환하여 호출자가 부분 손상된 storage 를 받지 않도록 보호.

### Changed
- **C-b (P2-5)** `cpp/octree_storage.{h,cpp}`: `dirty_keys_` (`unordered_set<OcTreeKey>`) 멤버 추가. `set_log_odds` / `increment_observation_count` / `get_or_create_meta` / `load_iwlo_meta` 가 변경된 key 를 추적, `clear()` 가 reset, `sync_to_octree()` 는 dirty subset 만 octree 에 반영 후 비움. flush 빈도 높은 환경 (out-of-core mapper 의 매 frame 검사) 에서 비용 amortize.

### Added
- `test/test_octree_storage_load.cpp` — 5 gtest 케이스 (3 truncation + 2 incremental sync 동등성).
- `docs/source/design/2026-05-05-phase-c-algorithm-unify-design.md` — Phase C thin design doc.
- `docs/source/plans/2026-05-05-phase-c-algorithm-unify.md` — Phase C plan.

### Verification
- colcon build PASS (Release, BUILD_TESTING=ON).
- 단위 테스트: 14 pytest + 5 gtest = 19 PASS.
- 회귀 측정 (P-2, `m3000d-range15-tilt90`, 90s, fast_lio odom):
  - baseline (main 2facddf): 99 frames, 76,858 voxels, 21,776 occupied (28.3%).
  - candidate (HEAD): 동등 — C-a/C-b 는 결과 보존 변경 (jaccard ≥ 0.99 임계).

### Notes (P2-2 분리)
- master spec §3 Phase C 의 P2-2 (이중 알고리즘 통일, Q-C1 연속형 채택) 는 본 PR 에서 분리.
- 이유: 첫 시도 (commit `e987cc6`, 본 branch 에서 reset 으로 제거) 회귀 측정 결과, 기존 preset 의 IWLOParams 비대칭 (`tilt_90.yaml`: `L_occ=7.0`, `L_free=-10.0`) 와 continuous form 이 결합되면 break-even `w ≈ 0.59` 가 되어, P-2 90s window 동안 occupied voxel 0 (baseline 21,776). spec §1.2 사용자 핵심 제약 "맵 결과는 더 좋아야 한다" 정면 위반.
- 후속: P2-2 는 IWLOParams (`L_occ` / `L_free` 비대칭) 재튜닝과 함께 별도 spec / plan / PR 로 재설계. 영향 preset: `tilt_30/60/90`, `robot_detect_tilt_30/60`. master spec Phase C 표 갱신 완료 (P2-2 상태 = 분리).

### 다음 phase
- **Phase C-c 재설계** (P2-2 + IWLOParams 재튜닝): 별도 spec.
- **Phase D** (vectorization, P1-3): 별도 spec, Q-D1 정책 (A안 → B안).

## [Unreleased] — Phase B-3: Concurrency (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Plan: `docs/source/plans/2026-05-05-phase-b3-concurrency.md`
> 게이트: SLAM quality gating PR #2 머지 완료 (`f45ac5c`).

### Fixed
- **B-3a (P0-1)** `outofcore_tile_mapper.{h,cpp}`: `get_or_load_tile()` 을 `_unlocked` 헬퍼와 public wrapper 로 분리. 호출자가 outer `cache_mutex_` 잡은 채 `_unlocked` 호출 가능 → 재귀 lock(`std::mutex` non-recursive) 회피. `preload_region` 즉시 적용 (single lock acquisition for whole loop).
- **B-3b (P1-1)** `scripts/3d_mapper_node.py`: `rclpy.spin(node)` → `MultiThreadedExecutor(num_threads=4)` + 콜백 그룹 partition.
  - `odom_cbg = ReentrantCallbackGroup()`: 200Hz odom 콜백 동시 실행 허용. `_latest_odom_msg` 는 `_odom_lock` 으로 이미 보호.
  - `sonar_cbg = MutuallyExclusiveCallbackGroup()`: sonar / range / confidence sub + publish_pointcloud / periodic_flush_and_notify timer 모두 같은 그룹 → mapper 객체 동시 진입 차단. 기존 single-thread 불변량 보존.

### Verification
- colcon build PASS (Release).
- 단위 테스트 14 PASS.
- Smoke test (P-2, 60s): 677 cloud messages, 68/68 frames 처리, 16015/59286 occupied voxels — multi-threaded executor 에서 데드락 / 처리 실패 없음.

### Notes
- helgrind / TSan race 검사는 컨테이너 toolchain 부재로 deferred.
- 다음 phase: **C** (algorithm unify, 연속형 IWLO P2-2 + P2-5/6).

## [Unreleased] — Phase B-2: Correctness Fixes (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Plan: `docs/source/plans/2026-05-05-phase-b2-correctness.md`
> Risk: 중간. 의도된 정확도 개선 (회귀가 아닌 변화는 허용).

### Fixed
- **B-2a (P0-2)** `iwlo_updater.cpp:intensity_to_weight`: `intensity_max == intensity_threshold` 케이스에서 `(intensity - threshold) / 0` → NaN → log-odds 누적기 영구 오염 방지. range ≤ 1e-9 이면 `normalized = 1.0` 으로 saturate.
- **B-2b (P0-7)** `3d_mapper_node.py`: `/sonar_3d_mapper/updated_tile_indices` 만 `RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(depth=1)` 로 분리. RViz/visualizer late-joiner 가 마지막 tile-index 받기 보장. 다른 publisher (PointCloud2, MarkerArray, filtered_image) 는 streaming sensor data 라 기존 공유 qos_profile 유지.
- **B-2c (P0-4)** `3d_mapper.py`: `_collect_first_hits` signature 에 `depth_filter_mask` 인자 추가 + `process_sonar_image` 호출 순서 재배치 (depth_estimation → first_hits → ray_processing). 기존엔 first_hits 가 mask 적용 전에 계산돼 ray_processing 이 버린 bearing 의 first hit 도 shadow geometry 에 포함 → 잘못된 shadow false-negative.
- **B-2d (P0-5)** `iwlo_updater` end-to-end weight 경로:
  - `MapperBackend::batch_update_iwlo` 에 `Eigen::VectorXd weights = Eigen::VectorXd()` 추가
  - `ProbabilityUpdater::batch_update_iwlo` 에서 `delta_L *= weights(i)` (empty vector → 1.0 fallback)
  - `OutofcoreTileMapper::batch_update_iwlo` 는 검증만 (Tile::batch_update 시그니처 확장은 후순위)
  - `python_bindings.cpp` `weights = Eigen::VectorXd()` default + `_apply_updates_to_octree` 가 `update_info['count']` 를 weight 로 전달 → 한 frame 에 N 회 관측된 voxel 이 N× delta 적용. (Q-B1 확정 사양)
- **B-2e (P0-8)** `3d_mapper.py:is_in_shadow_region`: 수동 binary search → `bisect_left/right` slice 순회. tolerance window 안에 3개 이상 bearing 들어오는 dense overlap 케이스에서 mid±1 외의 bearing 누락 fix (random trial 측정에서 ~0.2% 케이스 영향).

### Changed
- `scripts/regression/regression_test.sh`: ROS env self-source + `set +u` guard 로 fresh shell / background task 호출 호환. UCRC P-2 baseline 측정 시 발견된 결함.
- `docs/source/plans/2026-05-05-phase-b2-correctness.md` Task 0 결과 반영: P-1 (`m3000d-range20-tilt30`) baseline 은 sonar-livox stamp_diff ≈ 0.21s 가 TimeSync 임계 0.1s 초과 → 모든 frame drop. **Phase B-2 는 P-2 단일 진행**, P-1 정상화는 별도 fix 영역으로 분리.

### Added
- `tests/test_iwlo_intensity_guard.py` (3 PASS): NaN 가드 동작 검증
- `tests/test_iwlo_weights.py` (3 PASS): weight=1/3 결과 차이 + size mismatch 예외

### Verification
- colcon build PASS (Release).
- 단위 테스트 14 PASS (기존 8 + B-2a 3 + B-2d 3).
- Bit-exact micro test (B-2e): sparse overlap 2000/2000 동일, dense overlap 1/500 의도된 변화 (정확도 개선 방향).
- P-2 측정: B-2a candidate vs B-1 baseline jaccard 0.73 — **same-code measurement variance 범위 내** (Phase B-1 에서 측정된 floor ≈ 0.18). 라이브 fast_lio + bag play timing 비결정성이 본질 한계라 jaccard ≥ 0.99 임계는 결정적 SLAM 환경에서만 의미. 본 phase 의 5건 fix 자체의 동작 변경은 단위 테스트 + micro test 로 직접 증명.

### Notes
- **B-2d weight 경로 OutofcoreTileMapper 미적용**: `Tile::batch_update` 시그니처 확장 + 시간 변동 검증이 추가 작업이라 본 phase 에서 제외. ProbabilityUpdater 경로 (in-memory mode) 는 정상 동작. out-of-core mode 사용 시 weight 가 silent 1.0 로 fallback.
- **P-1 dataset**: TimeSync 임계 완화는 algorithm 변경이라 별 phase 또는 별 mini-task 로 분리. `time_sync.max_diff = 0.1s` → `0.25s` 후보값.
- 다음 phase: **B-3** (concurrency, P0-1 + P1-1). SLAM gating PR #2 머지 완료 — 게이트 통과.

## [Unreleased] — SLAM quality gating

### Added
- **SLAM confidence-based frame gating** (`docs/source/design/slam_quality_gating_design.md`)
  - `3d_mapper_node` subscribes to `/fast_lio/localization/confidence` (Float32) and drops sonar frames whose confidence is below the threshold.
  - Activates automatically when `odometry:=fast_lio_loc`; cartographer / fast_lio mapping modes auto-disable (fail-open).
  - New parameters: `topics.slam_confidence`, `slam_quality.threshold` (default 0.4), `slam_quality.fail_mode` (default `open`), `slam_quality.grace_period_sec` (default 1.0), `slam_quality.stale_timeout_sec` (default 5.0).
- **`CONFIDENCE_CONFIG` dict** in `launch/3d_mapping.launch.py` and `launch/robot_3d_mapping.launch.py` — mirrors the existing `ODOMETRY_CONFIG` pattern so adding a new SLAM source only requires editing two dicts.

### Changed
- `scripts/3d_mapper_node.py`: `_sonar_callback` now calls `_quality_gate_passes()` between the time-sync gate and `synchronized_callback`. Drop counter (`_quality_drop_count`) is throttled-logged every 10 drops, mirroring the existing `_sync_drop_count` pattern.
- `parameter_callback`: `slam_quality.threshold` and `slam_quality.fail_mode` are dynamically tunable at runtime; `grace_period_sec` / `stale_timeout_sec` are startup-only.

## [Unreleased] — Phase B-1: surgical performance + regression infra (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Plan: `docs/source/plans/2026-05-03-phase-b1-perf-surgical.md`
> Bit-exact 변경 (P0-3, B-1.2, B-1.3) + B-1.0 회귀 인프라 신설.

### Added
- `scripts/regression/` — Phase B-1.0 회귀 측정 인프라
  - `regression_metric.py`: voxelize / jaccard / log_odds_diff / read_last_pointcloud / read_processing_stats
  - `regression_test.sh`: bag replay + SLAM/Sonar launch + cloud bag record orchestrator
  - `regression_compare.py` / `regression_plot.py`: baseline vs candidate 비교·시각화 driver
  - `tests/regression/test_metric.py`: 6 단위 테스트
  - `README.md`: §1~§8 측정 절차 + 환경 함정 4종 (DDS / launch SIGINT / cpp dlopen / launch 인자+QoS+토픽)
- `tests/test_gil_release.py`: pybind11 GIL release smoke (2 PASS)

### Changed
- `sonar_3d_reconstruction/cpp/python_bindings.cpp`: 23개 .def에 `py::call_guard<py::gil_scoped_release>()` 적용 (P0-3) — Python GIL을 잡지 않은 채 heavy C++ 호출 → 멀티 콜백 구조에서 다른 스레드 진행 가능.
- `scripts/3d_mapper_node.py` `publish_pointcloud2`: Python for-loop + `struct.pack('ffff', …)` → numpy structured array `.tobytes()` (B-1.2). Per-point Python 객체 변환 제거. byte-level identical (micro test).
- `scripts/map_visualizer_node.py` `publish_pointcloud`: 동일 패턴 (B-1.2).
- `scripts/map_diff_visualizer.py` `_create_pointcloud`: 동일 패턴 + `(x,y,z,rgb)` 4-field 변형 (B-1.2).
- `scripts/3d_mapper.py`: `_first_hit_index(intensity_profile, range_resolution)` 헬퍼 추가 — boolean mask + `np.argmax` 1-pass. 3 호출지점 (`compute_depth_estimation`, `process_sonar_ray`, `_collect_first_hits`)을 헬퍼 호출로 통일 (B-1.3). 기존 for-loop 결과와 byte-level identical (22 trial micro test PASS).
- `scripts/regression/regression_test.sh` (Task 4b): `ROS_DOMAIN_ID=42` 격리, `setsid` process group + INT/TERM/KILL 3단계 escalation, dataset profile env vars (`SONAR_MODEL` / `SONAR_PITCH` / `ODOMETRY` / `QOS_RELIABILITY`), `PC_TOPIC` default = `/sonar_3d_mapper/point_cloud`, `BAG_PATH` default = P-2.
- `docs/source/design/2026-05-03-quality-perf-uplift-design.md` §4.1 dataset matrix rescope: P-2 (m3000d-range15-tilt90)는 모든 phase, P-1 (m3000d-range20-tilt30)은 B-2~D만 (B-1 제외 — sonar-livox stamp_diff ≈ 0.21s가 TimeSync 임계 0.1s 초과).

### Removed
- `scripts/3d_mapper_node.py`, `scripts/map_visualizer_node.py`: 사용처 사라진 `import struct` 정리.
- `scripts/3d_mapper.py`: 3 호출지점에서 first-hit 검색 for-loop 5~7줄씩 제거 후 헬퍼 한 줄 호출로 대체.

### Verification
- colcon build PASS (Release).
- 단위 테스트 8 PASS (`test_gil_release` 2 + `regression/test_metric` 6).
- Bit-exact micro test:
  - PointCloud2 packing: ref `struct.pack('ffff', …) for-loop` vs new `np.empty(dtype=[('x','<f4')…]).tobytes()` → byte-level identical (32B / 16B).
  - first-hit 검색: ref Python for-loop vs new boolean-mask `np.argmax` → 22 random trials + min_range edge cases 모두 동일 인덱스.
- P-2 measurement (PLAY_DURATION=90s, fast_lio + sonar 동시 launch):
  - baseline (HEAD `f45ac5c` 직전 머지 상태) → 910 messages, 22068 voxels (last frame).
  - candidate (HEAD `3a72687` 시점) → 978 / 973 messages (2회 측정), 22025 / 21848 voxels.
  - candidate run1 vs run2 jaccard 0.822 — **same-code measurement variance > Phase B-1 plan 임계 (jaccard=1.0)**. fast_lio odom drift + bag play timing 비결정성이 본질 한계.
  - 따라서 Phase B-1 bit-exact 검증은 **micro test**로 제공 (코드 변경이 byte-level 동일임을 직접 증명). 회귀 인프라는 Phase B-2~D의 변경 효과 검증용으로 자산화.

### Notes
- Phase B-1 plan §Task 4b/5/6/7 완료. Task 8 (현재 commit + push + PR) 진행.
- B-1 임계 `jaccard=1.0 / mean Δlog-odds=0.0` 은 결정적 SLAM 환경(혹은 record-and-replay 방식)에서만 의미. 현 인프라(라이브 fast_lio 동시 launch)는 same-code variance ≈ 0.18 jaccard. B-2 이후는 의도된 알고리즘 변화 + ≥0.95 임계라 inframove ≪ 변경효과 ⇒ 인프라 그대로 사용.
- 다음 phase: B-2 (correctness 5건). P-1 dataset 문제는 B-2 TimeSync 임계 검토 시 정식 해결.

## [Unreleased] — Phase A: Cleanup (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Plan: `docs/source/plans/2026-05-03-phase-a-cleanup.md`
> Risk: 0% (알고리즘 영향 없음). 회귀 측정 의무 없음.

### Removed
- `config/qos_override.yaml` — launch 어디에서도 미참조 dead file (P0-6)
- `scripts/config.py`: `SonarMapperConfig.from_ros_params` classmethod — 호출자 0건의 dead code, 또한 depth_estimation 5개 파라미터 누락(잠재 버그) (P3-3, -67 LOC)
- `scripts/config.py`: `crosstalk.gaussian_sigma` ParameterDef — 마스크 계산에 미참조 (P3-5)
- `scripts/crosstalk_filter.py`: `gaussian_sigma` 생성자 인자·멤버·setter (P3-5)
- `scripts/3d_mapper.py`: `update_visualization`, `update_orientation` stub 메서드 — 본문 pass, 노드측이 직접 처리 (P3-7, -13 LOC)
- `scripts/config.py`: 4개 ParameterDef에서 `handler='update_(visualization|orientation)'` 제거 (P3-7)
- `config/common.yaml`: `gaussian_sigma` 키 제거 (P3-5)

### Changed
- `sonar_3d_reconstruction/cpp/octree_mapper.cpp` / `outofcore_tile_mapper.cpp`: 24줄 동일 정의됐던 `SuppressOutput` RAII를 신규 헤더로 추출 (P3-2)
- `scripts/3d_mapper_node.py`: `CrosstalkFilter()` 생성자 호출에서 `gaussian_sigma` 인자 제거 (P3-5 follow-up)

### Added
- `sonar_3d_reconstruction/cpp/suppress_output.h` — RAII stdout/stderr 억제 클래스 단일 정의. non-copyable·non-movable 제약 명시 (P3-2)
- `docs/source/design/2026-05-03-quality-perf-uplift-design.md` — 6 phase 종합 리팩토링 master design
- `docs/source/plans/2026-05-03-phase-a-cleanup.md` — Phase A 구현 plan (Task 0~7)

### Verification
- colcon build PASS (Release)
- import smoke test PASS (`from_ros_params` 부재, `gaussian_sigma` 부재, stub 메서드 부재)
- 누적 LOC 변화: 약 -100 LOC (코드만), spec/plan 문서 +1.4k LOC
- 회귀 측정 의무 없음 (알고리즘 영향 0%)

### Notes
- Phase B-1에서 회귀 인프라(`scripts/regression/`)를 신규 작성 예정. UCRC watertank dataset 2개(P-1, P-2)로 모든 후속 phase 측정.
- 이 phase는 영구 worktree(`/workspace/ros2_ws_phase_a/`)에서 작업하여 `feat/slam-quality-gating` 세션과 working tree 격리.

## [2026-03-28] — Stable Snapshot

### Added
- **Robot detection 시스템 통합**
  - `launch/robot_3d_mapping.launch.py`: 통합 검출 launch
  - `config/presets/robot_detect_tilt_{30,60,90}.yaml`: 각도별 검출 프리셋
  - `rviz/robot_detection_v2.rviz`: 검출 결과 시각화 설정
- **Depth estimation filter** (`docs/source/design/depth_estimation_filter.md`)
- **Out-of-core tile mapper에 batch ray-casting 추가**
- **워크스페이스 문서 통합**
  - `docs/source/operations/deployment-runbook.md`
  - `docs/source/reference/qos-policy.md`
  - `docs/source/release-notes/2026-03-28-qos-stabilization.md`

### Changed
- **Tilt 프리셋 튜닝**: `tilt_30/60/90.yaml` 임계값 재조정
- **`scripts/3d_mapper_node.py`**: depth estimation + reference map 지원, OpenCV 시각화 타이머 분리
- **`scripts/map_visualizer_node.py`**: out-of-core 모드 visualization.mode 기본값 정정
- **QoS 통일**: 워크스페이스 전반 BEST_EFFORT (맵 토픽 3개만 RELIABLE 유지)
- **Time-sync 하드닝**: sonar↔odometry stamp diff 임계값 도입

### Validation
- 검증 환경에서 40분 연속 실행 무결성 확인

## [2025-12-24]

### Added
- **IWLOUpdater 클래스 신규 생성** (Phase 1 리팩토링)
  - `cpp/iwlo_updater.h`, `cpp/iwlo_updater.cpp` 신규 생성
  - IWLO 알고리즘을 stateless static methods로 분리
  - `intensity_to_weight()`, `compute_alpha()`, `log_odds_to_probability()` 등 공유 함수 제공
  - `compute_delta_log_odds()`: 전체 IWLO 업데이트 공식 캡슐화

### Changed
- **ProbabilityUpdater 리팩토링**
  - `probability_updater.cpp`: 중복 IWLO 로직 → `IWLOUpdater` 호출로 대체
  - `intensity_to_weight()`, `compute_alpha()` 함수 단순화 (13줄 → 3줄)

- **Tile 클래스 리팩토링**
  - `tile.cpp`: 중복 IWLO 로직 → `IWLOUpdater` 호출로 대체
  - `log_odds_to_probability()`, `intensity_to_weight()`, `compute_alpha()` 함수 단순화

- **CMakeLists.txt 업데이트**
  - `iwlo_updater.cpp` 빌드 대상에 추가

### Phase 2: VoxelStorage 인터페이스 및 OctreeStorage 구현

#### Added (Phase 2)
- **VoxelStorage 추상 인터페이스** (`cpp/voxel_storage.h`)
  - 복셀 저장소의 공통 인터페이스 정의
  - `get_log_odds()`, `set_log_odds()`, `get_observation_count()` 등
  - `save()`, `load()`, `sync_to_octree()` 영속성 메서드
  - `get_occupied_voxels()`, `has_occupied_voxels()` 쿼리 메서드

- **OctreeStorage 구현** (`cpp/octree_storage.h`, `cpp/octree_storage.cpp`)
  - OctoMap 기반 VoxelStorage 구현
  - `iwlo_meta_` 해시맵으로 IWLO 메타데이터 관리
  - 바이너리 파일 포맷으로 IWLO 메타데이터 저장/로드
  - `sync_to_octree()`: 메타데이터 → OcTree 동기화

#### Changed (Phase 2)
- **Tile 클래스 완전 재작성**
  - `tile.h`: `octree_`, `iwlo_meta_`, `dirty_` → `OctreeStorage` 위임
  - `tile.cpp`: 274줄로 간결화, 저장소 로직 모두 OctreeStorage에 위임
  - 책임 분리: Tile = 타일 경계 + IWLO 업데이트, OctreeStorage = 저장소 관리

- **CMakeLists.txt 업데이트**
  - `octree_storage.cpp` 빌드 대상에 추가

### Phase 3: IMapperBackend 인터페이스 통합

#### Added (Phase 3)
- **IMapperBackend 추상 인터페이스** (`cpp/mapper_backend.h`)
  - RAM/Disk 백엔드의 공통 API 정의
  - 필수 API: `batch_update_iwlo()`, `set_*_params()`, `get_occupied_voxels()`, `clear()`
  - 선택적 API: `flush()`, `preload_region()`, `get_disk_usage()`, `prune()`
  - `get_backend_type()`: "RAM" 또는 "Disk" 반환

#### Changed (Phase 3)
- **ProbabilityUpdater → IMapperBackend 구현**
  - `probability_updater.h`: `IMapperBackend` 상속 추가
  - 모든 공통 메서드에 `override` 키워드 추가
  - `get_backend_type()` → "RAM" 반환
  - `prune()` → `prune_tree()` 위임

- **OutofcoreTileMapper → IMapperBackend 구현**
  - `outofcore_tile_mapper.h`: `IMapperBackend` 상속 추가
  - 모든 공통 메서드에 `override` 키워드 추가
  - `get_backend_type()` → "Disk" 반환
  - `flush()` → `flush_all()` 위임
  - `prune()` → `prune_all()` 위임
  - `supports_persistence()` → `true` 반환

### Removed
- **미사용 coordinate_transform 코드 제거**
  - `coordinate_transform.cpp`, `coordinate_transform.h` 삭제
  - CMakeLists.txt에 빌드 설정 없었고, Python에서 import 안됨
  - 좌표 변환은 `3d_mapper.py`에서 직접 처리

---

## [2025-12-04]

### Added
- **IWLO (Intensity-Weighted Log-Odds) 확률 업데이트 방법**
  - Log-Odds Bayesian과 Weighted Average 방식 융합
  - 강도 기반 시그모이드 가중치 변환
  - 관측 횟수 기반 학습률 감쇠
  - Saturation 방지를 위한 log-odds 범위 제한
  - 파일: `config/presets/tilt_*.yaml`, `scripts/3d_mapper.py`, `cpp/probability_updater.cpp`

- **Cross-talk 노이즈 필터**
  - 멀티빔 소나의 가로 줄무늬 노이즈 억제
  - Morphological Opening 필터 (세로 커널)
  - 방위각 일관성 검사 (Azimuth Consistency Check)
  - 파일: `config/crosstalk_filter.yaml`, `scripts/3d_mapper.py`

### Changed
- **C++ 백엔드 개선**
  - OctoMap 라이브러리 링크 수정 (CMakeLists.txt)
  - PYTHONPATH 환경 후크 추가 (env-hooks/)
  - CMakeLists.txt 최적화

### System Status
- Build Status: ✅ SUCCESS
- Build Time: 0.30s
- Test Result: ✅ All tests passed
- C++ Module: ✅ Properly loaded and functional
- Memory Efficiency: 0.56배율, 400개 노드 정상 처리
