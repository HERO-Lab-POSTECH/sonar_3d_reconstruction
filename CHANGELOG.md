# CHANGELOG - sonar_3d_reconstruction

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
