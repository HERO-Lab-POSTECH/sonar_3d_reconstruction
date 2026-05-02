# CHANGELOG - sonar_3d_reconstruction

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
