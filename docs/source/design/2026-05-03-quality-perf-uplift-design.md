# Quality & Performance Uplift — Sonar 3D Reconstruction 종합 리팩토링 설계

> 작성일: 2026-05-03
> 상태: Design (구현 전)
> 관련 패키지: `src/sonar_3d_reconstruction`
> 의존: `marine_acoustic_msgs`, `oculus_sonar_msgs`
> 협업 영역: `feat/slam-quality-gating` 브랜치 (다른 세션 진행 중)
> 입력 자료: 본 설계는 2026-05-03 수행한 3개 병렬 코드 감사(Python 코어 / C++ 백엔드 / 실시간성·QoS) 결과 32건의 우선순위 분류에서 출발

---

## 1. 개요

### 1.1 문제

`sonar_3d_reconstruction` 패키지는 4단계 누적 리팩토링(IWLOUpdater 추출 → VoxelStorage 인터페이스 → IMapperBackend 통합 → 2026-03-28 stable snapshot)을 거치며 약 8.7k LOC 규모로 성장했다. 이 과정에서 다음 부채가 누적되었다.

- **명확한 결함 8건(P0)**: mutex 재진입 위험, 0-division NaN 전파, GIL 미해제, depth filter shadow 누락, voxel-center quantize, dead config 파일, QoS 비대칭, shadow binary search 부정확
- **핫패스 성능 손실 8건(P1)**: SingleThreadedExecutor 직렬화, PointCloud2 직렬화 Python 루프, ray-loop matmul, first-hit Python `for`, 문자열 키 왕복, hash 충돌 취약, fft float32 승격, KDTree 미사용
- **정확도·구조 9건(P2)**: 좌표계 부호 일관성, 이중 알고리즘(IWLOUpdater dead code), 부동소수 누적, default 값 충돌, sync_to_octree 비효율, 불완전 읽기 미감지, 매직 넘버, 함수 길이
- **정리·DX 7건(P3)**: 파일명, RAII 중복, 코드 중복(70 LOC), LSP, dead config, marker 누적, stub 메서드

### 1.2 사용자 핵심 제약

> **"맵 결과는 이전보다 더 좋아야만 한다"**

이 제약은 본 설계의 모든 결정을 지배한다.
- 회귀 0% (정확도 동률 이상)
- 의도된 정확도 변화는 정량 metric으로 증명
- 임계 미달 시 머지 차단

### 1.3 목표

| ID | 목표 | 측정 가능 지표 |
|----|------|---------------|
| G1 | 맵 정확도 손실 0% | Jaccard(B,C) ≥ 0.99 (각 phase 임계값은 §4.4) |
| G2 | 처리량 ≥ baseline | avg_processing_time_per_frame |
| G3 | PKRC 코딩 규약 100% 준수 | 함수 ≤50줄, 메서드 ≤20개, 중첩 ≤3, 매직넘버 0 |
| G4 | 인터페이스 호환 유지 | launch 인자·yaml 키·topic 이름 변경 0 |
| G5 | 모든 변경에 추적 가능성 | branch + commit + CHANGELOG + PR 4축 동기 |

### 1.4 비목표(범위 외)

- 새 알고리즘 도입 (NDT, Surfel, Submap 등)
- 새 센서 통합 (Ping360 데이터 융합 — Phase 3 로드맵 영역)
- ROS2 Jazzy 마이그레이션
- **SLAM quality gating 자체** (다른 세션 소관, §7)
- 새 회귀 metric 도입 (기존 Jaccard + log-odds 차 + 처리율로 충분)

---

## 2. 제약과 보장

### 2.1 정확도 보장 절차

모든 정확도 영향 phase(B-2 / C / D)는 다음 절차를 통과해야 머지 가능하다.

```
1. main HEAD에서 build → bag replay → baseline map 저장
2. 현재 branch에서 build → bag replay → candidate map 저장
3. 정량 비교 (§4.3 metric)
4. 시각 비교 plot 생성 (xy 단면 + xz 단면)
5. 임계 통과 (§4.4)
6. PR 본문에 metric 표 + plot 첨부
```

위 절차는 fast_lio 패키지에 이미 검증된 패턴(`scripts/regression_*.sh`)을 sonar로 이식하여 사용한다(§4.2).

### 2.2 호환성 보장

- launch 인자: 이름·기본값 변경 금지 (deprecation 시 alias로)
- yaml 키: 이름 변경 금지 (deprecation 시 양쪽 동시 지원)
- topic 이름: 변경 금지
- 기존 사용자 명령 (`ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py ...`) 그대로 동작해야 함

### 2.3 데이터 안전 (CLAUDE.md 강제)

- 어떤 phase도 `/workspace/data/` 하위의 bag/db3/metadata.yaml을 삭제·수정하지 않는다
- 회귀 측정 결과(`/tmp/sonar3d_regression/`)는 `.gitignore`에 포함하고 commit 금지
- plot 이미지는 PR 본문에 첨부하되 repo에는 commit 금지

### 2.4 다른 세션과의 락 분리

- **Phase B-3은 게이트**: SLAM quality gating 세션의 main 머지가 확인된 후에만 시작
- 미협의 시 자동으로 phase 순서 재정렬 (B-3 → 마지막으로 이동)
- common.yaml·`3d_mapper_node.py`·`config.py`의 `parameter_callback` 영역에서 충돌 가능

---

## 3. Phase 분할 (총 6 phase, 누적 머지)

각 phase는 별도 branch + squash merge → main이 다음 phase의 baseline.

### Phase A — Cleanup (위험: 0%)

**목표**: 알고리즘에 영향 없는 정리 작업. 회귀 측정 의무 없음(build PASS만).

**범위**:
| 항목 ID | 위치 | 변경 |
|--------|------|------|
| P0-6 | `config/qos_override.yaml` | 파일 삭제 (Q-A1 확정) |
| P3-2 | `cpp/octree_mapper.cpp:14`, `cpp/outofcore_tile_mapper.cpp:14` | `SuppressOutput` RAII를 `cpp/suppress_output.h`로 추출 |
| P3-3 | `scripts/config.py:411-549` | `from_ros_params` → `from_params_dict` 위임 (~70 LOC 감소) |
| P3-5 | `scripts/crosstalk_filter.py:27`, `config.py` 관련 ParameterDef | `gaussian_sigma` 미사용 필드 제거 |
| P3-7 | `scripts/3d_mapper.py:451-464` | `update_visualization`, `update_orientation` stub 삭제 + ParameterManager 매핑 제거 |

**검증**: `colcon build` PASS, 단위 테스트 PASS, launch smoke test (`bag_file=` 없이 노드 기동만 확인). 회귀 측정 의무 없음(알고리즘 무관).

**예상 LOC 변화**: -150 LOC

---

### Phase B-1 — Performance Surgical (위험: 낮음)

**목표**: 알고리즘 결과를 정확히 동일하게 유지하면서 처리량 개선. **첫 측정 phase**이므로 회귀 인프라를 함께 도입한다.

**범위**:
| Sub | 항목 ID | 위치 | 변경 |
|-----|--------|------|------|
| **B-1.0** | (신규) | `scripts/regression/` (신규 디렉토리) | `fast_lio` 패턴 이식 — `regression_test.sh`, `regression_compare.py`, `regression_plot.py`, `regression_metric.py`, `README.md` 작성 |
| B-1.1 | P0-3 | `cpp/python_bindings.cpp:147-241` | 무거운 메서드 전체에 `py::call_guard<py::gil_scoped_release>()` 추가 |
| B-1.2 | P1-2 | `scripts/3d_mapper_node.py:730-736`, `map_visualizer_node.py:423-429`, `map_diff_visualizer.py:202-206` | `struct.pack` 루프 → numpy structured `.tobytes()` |
| B-1.3 | P1-4 | `scripts/3d_mapper.py:222-228, 576-580, 688-692` | first-hit `for + enumerate` → `np.argmax(mask)` |

**검증**:
- **bit-exact 또는 voxel-set 동일성**: Phase B-1은 정확도에 영향 없어야 한다. metric: `Jaccard(B,C) = 1.0`
- 처리량: `avg_processing_time_per_frame` ≥ baseline (개선 기대)
- baseline metric은 main HEAD에서 측정

**예상 효과**: GIL 해제만으로 sonar 콜백 지연 30~50% 감소 가능 (가설, 검증 필요)

---

### Phase B-2 — Correctness Fixes (위험: 중간)

**목표**: 의도된 정확도 변화. 결과는 baseline 대비 **개선** 또는 **동률**이어야 한다.

**범위**:
| 항목 ID | 위치 | 변경 | 정확도 영향 |
|--------|------|------|-----------|
| P0-2 | `cpp/iwlo_updater.cpp:42` | 0-division 가드 추가 | NaN 오염 방지 → 정확도 ↑ |
| P0-4 | `scripts/3d_mapper.py:830-846` | `_collect_first_hits`에 mask 인자 추가 | depth filter false-negative 감소 → 정확도 ↑ |
| P0-5 | `scripts/3d_mapper.py:765-787` | voxel multiplicity → IWLO weight 전달 (Q-B1 확정) | 수렴 속도 ↑ |
| P0-7 | `scripts/3d_mapper_node.py:181-247` | sub/pub QoS 분리, `tile_indices`는 `RELIABLE+TRANSIENT_LOCAL` | RViz 누락 해소 → 시각화 정확도 ↑ |
| P0-8 | `scripts/3d_mapper.py:494-525` | shadow binary search → `bisect_left/right` 범위 순회 | full overlap 모드 정확도 ↑ |

**검증**:
- 정확도: Jaccard(B,C) ≥ 0.95 (의도된 변화 허용 폭). 단, **변화 방향**이 회귀가 아닌 개선임을 plot으로 증명
- 처리량: avg ≥ baseline
- 검증 시 P0-7 효과는 별도 (시각화 누락 → 단순 카운트 비교)

**개별 항목 정확도 검증 권장**: 5건을 한 번에 머지하면 회귀 원인 분리가 어려우므로, **sub-phase B-2a~B-2e로 commit 단위 분리**하고 squash 시 묶음.

---

### Phase B-3 — Concurrency 🚧 (위험: 높음, 게이트)

**목표**: 동시성 안전성 + 콜백 직렬화 해소.

**🚧 게이트 조건**: SLAM quality gating 세션의 main 머지 확인 후만 시작. 미충족 시 Phase B-3을 마지막(D 다음)으로 재정렬.

**범위**:
| 항목 ID | 위치 | 변경 |
|--------|------|------|
| P0-1 | `cpp/outofcore_tile_mapper.cpp:512, 544` | `get_or_load_tile()` 잠금 제거 → `_unlocked` 헬퍼 분리, 공개 메서드만 잠금 |
| P1-1 | `scripts/3d_mapper_node.py:786-791` | `MultiThreadedExecutor(num_threads=4)` + `ReentrantCallbackGroup`(odom) + `MutuallyExclusiveCallbackGroup`(sonar) |

**검증**:
- 동시성: helgrind 또는 ThreadSanitizer 빌드로 race 검사 (가능한 경우)
- 시간 동기화: `[TimeSync] stamp_diff` 분포 확인 — 분산 감소 기대
- 드롭률: `_sync_drop_count` 감소 기대
- 처리량: 변화 없거나 ↑ (병렬 콜백)
- 정확도: Jaccard(B,C) ≥ 0.99

**위험 분석**:
- MultiThreadedExecutor 도입 시 기존 `_odom_lock` 외에 추가 race 노출 가능 → 코드 리뷰 필수
- SLAM gating 세션의 confidence 콜백이 같은 그룹에 추가될 예정 → 충돌 회피 위해 `confidence_cbg`도 미리 정의

---

### Phase C — Algorithm Unification (위험: 중간)

**목표**: 알고리즘 이중 구현 정리 + 누적 오류 제거.

**범위**:
| 항목 ID | 위치 | 변경 | 상태 (2026-05-05) |
|--------|------|------|------------------|
| P2-2 | `cpp/iwlo_updater.cpp:106` vs `probability_updater.cpp:368-374` vs `tile.cpp:94-101` | **연속형(`IWLOUpdater::compute_delta_log_odds`)로 통일** (Q-C1 확정), `ProbabilityUpdater::batch_update_iwlo` / `Tile::update_voxel`의 이진 분기 제거 | **분리** — Phase C PR 머지 시 회귀 측정에서 사용자 핵심 제약 위반 (occupied voxel 21,776 → 0). IWLOParams (`L_occ` / `L_free` 비대칭) 재튜닝과 함께 별도 spec 으로 재설계 |
| P2-5 | `cpp/octree_storage.cpp:276-286` | `dirty_keys_` 추적 → 변경 voxel만 sync_to_octree | Phase C 머지 |
| P2-6 | `cpp/octree_storage.cpp:249-260` | `load_iwlo_meta` 루프 내 `ifs.good()` 검사 | Phase C 머지 |

**확정 (Q-C1)**: 연속형 채택. 사유는 §10 참조.

이행 단계:
1. `ProbabilityUpdater::batch_update_iwlo` / `Tile::update_voxel`의 intensity 이진 분기 코드 제거
2. 두 메서드가 `IWLOUpdater::compute_delta_log_odds`를 동일 인자로 호출하도록 일원화
3. baseline(이진) vs candidate(연속형) 시각 비교 — 임계값 부근 노이즈 voxel에서 부드러운 전이 확인 (개선 증명)

**검증**:
- 정확도: Jaccard(B,C) ≥ 0.99 (수치 통일 — 결과 동일성 강제)
- 단, P2-2(이중 알고리즘 통일)는 결과가 의도적으로 달라질 수 있음 → Jaccard ≥ 0.95 + 시각 비교에서 정확도 향상 증명
- 처리량: P2-5 효과로 flush 빈도 큰 환경에서 ↑

---

### Phase D — Vectorization (위험: 높음, 가장 큰 효과)

**목표**: ray loop 벡터화 또는 C++ 측 ray-cast 이관 — 가장 큰 처리량 개선.

**범위**:
| 항목 ID | 위치 | 변경 |
|--------|------|------|
| P1-3 | `scripts/3d_mapper.py:557-664` (`process_sonar_ray`) | inner-loop matmul → bearing별 (x,y,z) 누적 후 일괄 변환, 또는 C++ ray-cast 이관 |

**구현 옵션**:
1. **A안 (Python 내부 vectorize)**: 변경 표면이 작고 위험 낮음, 처리량 5-10× 기대
2. **B안 (C++ ray-cast 이관)**: pybind11 경계 변경, 위험 높음, 처리량 10-30× 기대

**권장**: A안 먼저 시도 → 충분하면 종료, 부족하면 B안. **A/B 모두 baseline-vs-candidate 동일성 검증 필수**.

**검증**:
- 정확도: Jaccard(B,C) ≥ 0.99 (결과 동일성 강제)
- 처리량: ≥ 1.5× baseline (A안 임계), ≥ 5× baseline (B안 임계)
- 안정성: 60s+ 연속 실행에서 메모리 누수·드롭률 변화 없음

**A안 측정 결과 (2026-05-05, P-2 m3000d-range15-tilt90, 90s, fast_lio odom)**:
- jaccard = 0.974 (임계 0.99 미달, 그러나 unit test 의 100 bearing 에서 voxel key bit-exact atol=0 별도 입증).
- avg proc_time baseline 104.2 ms → candidate 71.0 ms = **1.47×** (임계 1.5× 에 0.03× 미달).
- Q-D1 결정 의뢰는 PR description 에서. B안 진행 여부는 측정 변동 (±5 ms) 과 fast_lio drift 노이즈를 감안한 사용자 판단에 위임.

---

## 4. 회귀 검증 인프라

### 4.1 데이터셋

UCRC watertank 2026-01-22 시리즈를 사용 (`/workspace/data/7_ucrc_watertank/20260122_sonar_lidar/`). 모든 bag에 **livox MID360 (CustomMsg)** + **IMU** 포함되어 있고 별도 odometry 토픽은 없음 → **회귀 테스트 시 `fast_lio`를 함께 launch**하여 실 운용 그래프와 동일하게 측정.

#### Primary (phase 별 적용)

| 이름 | 경로 (생략: `7_ucrc_watertank/20260122_sonar_lidar/`) | Duration | Sonar / Preset | 환경 | 적용 phase |
|------|---------------------------------------------------------|----------|----------------|------|-----------|
| **P-2** | `m3000d_blueboat/m3000d-range15-tilt90` | 352.0s | m3000d, tilt90 (직하방), range15, 1757 frames | Map1 (크레인 이동 전) | **B-1 ~ D** (모든 phase) |
| **P-1** | `m3000d_blueboat/m3000d-range20-tilt30` | 209.6s | m3000d, tilt30, range20, 1046 frames | Map2 (크레인 이동 후) | **B-2 ~ D** (B-1 제외) |

- **Phase B-1 (bit-exact)**: P-2 단일 측정. 같은 코드를 두 빌드(baseline / candidate) 에서 돌려 결과 동일성(jaccard=1.0, mean Δlog-odds=0.0) 만 검증하면 되므로 dataset 다양성은 불필요.
- **Phase B-2 ~ D**: P-1 + P-2 모두에서 임계 통과해야 머지 (preset 30/90 + Map1/Map2 환경 양쪽).

> **Notes (2026-05-05)**:
> - 초기 spec 의 P-1 (`m750d-range15-tilt45-v1`) 은 launch preset 매트릭스
>   (30/60/90 만 지원) 와 mismatch 하여 실측정 불가능. m3000d × Map2 × tilt30
>   으로 교체하면서 sonar 모델 다양성은 잃었으나 preset/환경 다양성을 확보.
> - 새 P-1 (range20-tilt30) 은 sonar-livox stamp_diff 가 일정하게 ~0.21s 로
>   현 코드의 TimeSync 임계 (`time_sync.max_diff = 0.1s`) 를 모두 초과하여
>   baseline 단계에서 0/0 frame 처리됨. Phase B-2 의 correctness fix 영역
>   (P0-class) 에서 TimeSync 임계 완화 또는 dataset 교체로 정식 해결 후
>   P-1 회귀 측정 정상화. Phase B-1 은 P-2 단일로 진행해도 bit-exact 검증
>   목적에는 충분.
> - m750d 검증은 Phase B-2 또는 D 시점 secondary 로 별도 추가 (필요 시
>   tilt45 preset 신설 또는 closest preset(30°) 매핑).

#### Secondary (Phase D 추가 검증, 매트릭스)

| 이름 | bag | 검증 차원 |
|------|-----|----------|
| S-1 | `m3000d_blueboat/m3000d-range15-tilt60-v2` (353.3s) | tilt 60° preset 매칭 (Map1) |
| S-2 | `m3000d_blueboat/m3000d-range15-tilt60-robot` (517.1s) | robot detection 시나리오 (긴 데이터, 동적 객체) |
| S-3 | `m750d_custom_platform/m750d-range15-tilt45-v1` | m750d 모델 검증 (preset 매핑 정책 결정 후) |

#### Reference Maps (보조 비교)

`/workspace/data/7_ucrc_watertank/map/{cartographer,sonar_3d}/`에 미리 빌드된 맵이 있어 ground-truth proxy로 활용 가능 (Phase B-2/C/D의 시각 비교에서 첨부).

### 4.2 회귀 스크립트 (디렉토리 신규 생성)

`fast_lio` 패키지의 검증된 패턴(`src/lidar_slam/fast_lio/scripts/regression_*`)을 sonar로 이식.

```
src/sonar_3d_reconstruction/scripts/regression/
├── regression_test.sh           # bag → mapping → 결과 dump
├── regression_compare.py         # baseline vs candidate diff 계산
├── regression_plot.py            # xy/xz 단면 시각화
├── regression_metric.py          # 정량 metric 계산 (Jaccard, log-odds 차)
└── README.md                     # 사용법
```

각 phase는 동일 스크립트 재사용. 결과는 `/tmp/sonar3d_regression/<phase-id>/`에 저장 (gitignore).

### 4.3 Metric

| Metric | 계산 | 의미 |
|--------|------|------|
| `n_occupied_B`, `n_occupied_C` | 점유 voxel 수 | 매핑 분량 |
| `jaccard_set` | \|B ∩ C\| / \|B ∪ C\| (voxel key 기준) | 점 일치율 |
| `mean_log_odds_diff` | mean(\|L_B(k) - L_C(k)\|) for k ∈ B ∩ C | 확률 차 평균 |
| `max_log_odds_diff` | max(\|L_B(k) - L_C(k)\|) | 확률 차 최대 |
| `avg_proc_time_ms` | mean(stats.processing_time) | 처리량 |
| `frame_drop_rate` | skipped / total | 드롭률 |
| `peak_memory_mb` | RSS peak | 메모리 |

### 4.4 임계값 (phase별)

| Phase | jaccard_set | mean_log_odds_diff | avg_proc_time_ms | 비고 |
|-------|-------------|-------------------|------------------|------|
| A | (측정 의무 없음) | — | — | build PASS only |
| B-1 | **= 1.0** | **= 0.0** | ≤ baseline | bit-exact 동일성 |
| B-2 | ≥ 0.95 | 의도된 변화 허용 | ≤ baseline | 시각 plot으로 개선 증명 |
| B-3 | ≥ 0.99 | ≤ 0.1 | ≤ baseline (개선 기대) | 동시성 정확성 |
| C | ≥ 0.99 (P2-5/6), ≥ 0.95 (P2-2) | ≤ 0.5 | ≤ baseline | 알고리즘 통일 |
| D | ≥ 0.99 | ≤ 0.1 | ≤ baseline / 1.5 (A안) ≤ baseline / 5 (B안) | 결과 동일성 + 처리량 ↑ |

**미달 시 처리**:
- 자동 머지 차단 (PR 본문에 metric 표 첨부 필수)
- 사용자에게 보고 + 원인 분석 + 분할 또는 롤백 결정

### 4.5 시각 비교

각 phase PR에 첨부:
- `xy_section.png`: Z=고정 평면에서 baseline(파랑) vs candidate(주황) overlay
- `xz_section.png`: Y=고정 평면에서 동일 overlay
- `metric_table.md`: 위 §4.3 표 값

---

## 5. 4축 추적 (refactor-workflow.md)

### 5.1 Branch 명명

```
refactor/phase-<id>-<short-name>
```

예:
- `refactor/phase-a-cleanup`
- `refactor/phase-b1-perf-surgical`
- `refactor/phase-b2-correctness-fixes`
- `refactor/phase-b3-concurrency`
- `refactor/phase-c-algo-unification`
- `refactor/phase-d-vectorization`

### 5.2 Commit 메시지

```
refactor(<scope>): phase <id> — <one-line summary>

<2-4 줄 본문: 무엇·왜·결과>

- 파일 A: <변화>
- 파일 B: <변화>
- LOC 또는 검증 수치 1줄
```

scope: `mapper_3d`, `iwlo`, `tile`, `qos`, `executor` 등 컴포넌트명.

### 5.3 CHANGELOG 항목 양식 (refactor-workflow.md 발췌)

```markdown
## [Unreleased] — Phase <id>: <one-line> (refactor)

### Changed / Added / Removed
- ...

### Verification
- colcon build PASS
- baseline vs candidate replay (84.5s) → jaccard X.XX, mean Δlog-odds X.XX
- avg_proc_time: baseline X.X ms → candidate X.X ms (Δ ±X%)

### Notes
- <한계, 다음 phase로 미룬 항목>
```

### 5.4 PR 본문 템플릿

```markdown
## Summary
- <한 줄>

## Changes
- <bullet 3-5개>

## Verification
- [ ] colcon build PASS
- [ ] Regression baseline vs candidate 측정 완료
- [ ] 임계 통과 (jaccard ≥ X.XX, Δlog-odds ≤ X.XX)
- [ ] xy/xz plot 첨부
- [ ] CHANGELOG.md 갱신

## Next Phase
- <다음 phase scope>
```

---

## 6. 위험 분석 및 롤백 정책

### 6.1 회귀 측정 임계 미달

1. branch 머지 보류
2. 사용자에게 metric 표 + plot 첨부하여 보고
3. 원인 분석 (commit 단위 bisect)
4. 분할 가능하면 sub-phase로 나누어 다시 검증
5. 분할 불가 또는 근본 결함이면 phase 보류 + 다음 phase로 진행

### 6.2 main 회귀 발견 (병합 후)

1. `archive/<phase-id>-<date>` 브랜치/태그로 보존
2. revert PR 생성
3. 회귀 케이스를 회귀 테스트에 추가 후 재시도

### 6.3 audit invariant 손상

PKRC 메모리에 기록된 fast_lio audit invariant(`reference_fast_lio_localization_audit`)가 본 패키지 변경으로 깨질 가능성은 낮으나, Phase B-3에서 콜백 구조 변경 시 cross-impact 가능성 검토 필요. 손상 확인 시 메모리 갱신 + 재audit.

---

## 7. 다른 세션 협의 포인트

### 7.1 SLAM Quality Gating (`feat/slam-quality-gating` 브랜치)

같은 패키지의 다른 세션이 진행 중. 충돌 가능 영역:

| 우리 phase | 그쪽 영향 | 충돌 가능 변경 | 대응 |
|-----------|----------|---------------|------|
| B-2 (P0-7 QoS) | `3d_mapper_node.py` sub/pub QoS 영역 | 그쪽이 confidence sub 추가 시 QoS 그룹 영향 | 우리는 pub QoS만, 그쪽은 sub만 → 분리 가능 |
| B-2 (P2-8 parameter_callback) | `config.py`에 confidence_threshold 등 4개 ParameterDef 추가 예정 | dispatch 충돌 | 미루고 그쪽 머지 후 재정렬 |
| **B-3 (P1-1 executor) 🚧** | `3d_mapper_node.py` 콜백 구조 전면 변경 | confidence 콜백을 어느 그룹에 둘지 결정 필요 | **게이트: 그쪽 머지 후 시작** |

### 7.2 협의 절차

1. Phase B-2 시작 전: 그쪽 세션의 main 머지 여부 확인 (`git log main`)
2. Phase B-3 시작 전: **반드시** 그쪽 머지 확인 + 그쪽 design doc 재독
3. 충돌 가능 commit 발생 시 즉시 사용자 보고

---

## 8. 구현 순서 게이트

```
Phase A 완료 → B-1 시작
B-1 완료 → B-2 시작
B-2 완료 + (SLAM gating 머지 확인) → B-3 시작
                                 └─ 미확인 시 B-3 마지막으로 재정렬
B-3 완료 → C 시작
C 완료 → D 시작
D 완료 → 본 설계 종료
```

각 게이트에서:
- 이전 phase의 회귀 metric이 임계 통과했는가?
- main 회귀 없는가?
- CHANGELOG 갱신됐는가?
- 다음 phase의 scope가 변경 없이 유효한가?

---

## 9. 산출물

| 산출물 | 위치 | 비고 |
|-------|------|------|
| 코드 변경 | `src/sonar_3d_reconstruction/` | 6개 PR (phase별) |
| 회귀 인프라 | `src/sonar_3d_reconstruction/scripts/regression/` | **Phase B-1.0**에서 신규 생성 (첫 측정 phase) |
| CHANGELOG | `src/sonar_3d_reconstruction/CHANGELOG.md` | phase별 항목 추가 |
| Phase별 Plan | `docs/superpowers/specs/<phase-id>-plan.md` | writing-plans skill로 작성 |
| 본 설계 | `docs/source/design/2026-05-03-quality-perf-uplift-design.md` | (이 파일) |

---

## 10. Decisions (사용자 확정 — 2026-05-03)

| ID | 질문 | 결정 | 영향 |
|----|------|------|------|
| Q-A1 | `qos_override.yaml` 처리 | **(a) 삭제** — launch가 어디서도 안 읽고, 토픽 키도 실제와 불일치하므로 surgical 제거 | Phase A의 P0-6은 "삭제"로 단정 |
| Q-B1 | P0-5 voxel multiplicity | **(a) IWLO weight로 전달** — 사용자 핵심 제약("맵 결과는 더 좋아야 한다")에 부합. 결과 변화 허용(jaccard ≥0.95) + 시각 plot으로 개선 증명 | Phase B-2의 P0-5 임계: §4.4 그대로 |
| Q-C1 | P2-2 이중 알고리즘 | **(a) 연속형 (`IWLOUpdater::compute_delta_log_odds`) 채택** — README 알고리즘 절(L.284-295)과 일치, 부드러운 전이로 노이즈 강건성 ↑. ProbabilityUpdater/Tile의 이진 분기 분기점 제거 | Phase C의 P2-2: "연속형으로 통일"로 단정 |
| Q-D1 | Phase D 범위 | **(c) A안(Python vectorize) 결과 보고 후 B안(C++ 이관) 진행 여부 결정** | Phase D 시작 시 A안 plan만 작성, B안은 측정 후 사용자 합의 |
| Q-Data | 회귀 측정용 odometry | **(b) bag 재생 + `fast_lio` 함께 launch** — bag에 odometry 토픽 없음. lidar+IMU 있음 → fast_lio mapping 모드로 odom 생성 | §4.1 dataset 표 + §4.2 회귀 스크립트 launch 구성 반영 |

---

## 11. References

### 패키지 내부
- `README.md`, `CHANGELOG.md`
- `docs/source/design/iwlo_design.md`
- `docs/source/design/octree_mapping.md`
- `docs/source/design/outofcore_design.md`
- `docs/source/design/depth_estimation_filter.md`
- (협의 후) `docs/source/design/slam_quality_gating_design.md`

### PKRC 컨벤션
- `/workspace/CLAUDE.md`
- `.claude/rules/coding-style.md`
- `.claude/rules/refactor-workflow.md`
- `.claude/rules/agent-delegation.md`
- `.claude/rules/git-workflow.md`

### 회귀 인프라 참고 패턴
- `src/lidar_slam/fast_lio/scripts/regression_test.sh`
- `src/lidar_slam/fast_lio/scripts/regression_compare.py`
- `src/lidar_slam/fast_lio/scripts/regression_plot.py`
