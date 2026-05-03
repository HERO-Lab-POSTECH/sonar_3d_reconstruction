# SLAM Quality Gating (Sonar 3D Mapper)

> 작성일: 2026-05-03
> 상태: Design (구현 전)
> 관련 노드: `scripts/3d_mapper_node.py`, `launch/3d_mapping.launch.py`, `launch/robot_3d_mapping.launch.py`
> 의존: `fast_lio` localization 모드 (이미 `/fast_lio/localization/confidence` publish 중)

---

## 1. 개요

### 문제
`3d_mapper_node`는 `topics.odometry`로 들어오는 위치 추정값을 그대로 사용해 sonar voxel 맵을 누적한다. SLAM의 위치 추정 품질이 낮은 구간(예: ICP fitness 0.3)에서도 sonar frame이 정상 frame과 동일하게 누적되어 맵이 오염된다.

### 해결 방법
`fast_lio`가 이미 publish 중인 `/fast_lio/localization/confidence`(Open3D ICP fitness, `std_msgs/Float32`, ~200Hz)를 sonar mapper가 구독해 임계치 이하의 confidence 시 해당 sonar frame을 drop한다. 기존 `_sync_drop_count` 패턴(time-sync 불일치 frame 처리)과 동일한 형태로 통합한다.

### 비결정 사항(이 phase 범위 외)
- **Localization 모드의 맵 자체 업데이트(Q1)**: 현재 phase에서 하지 않는다. 근거 §2.
- **Cartographer 모드의 confidence gating(Q3)**: cartographer는 per-frame scalar score를 publish하지 않는다. fork 수정이 필요하므로 별 phase로 분리. §6.

---

## 2. Q1 — 맵 업데이트 미수행 결정

### 결정
`localization_node` 측 `pcd_map_ori_/fine_/global_`은 audit 메모(2026-05-03 기준 `feedback_fast_lio_localization_audit`)대로 **read-only로 유지**한다. "달라진 점만 누적"하는 lifelong-mapping 기능은 도입하지 않는다.

### 근거
1. **audit 무효화 비용**: 모든 컨테이너가 bounded라는 audit 결과(2026-05-03)가 깨진다. 점 추가 path가 생기면 매 phase마다 재audit 필요.
2. **회귀 결정성 손실**: ICP target(`pcd_submap_cached_`)이 동적이 되어, 같은 bag 재생에서도 baseline-vs-candidate 비교가 흔들린다.
3. **자료구조 invariant**: Open3D `PointCloud`는 `std::vector<Eigen::Vector3d>` contiguous storage. append 시 vector resize → fragmentation. FPFH index가 묶인 `pcd_map_global_`은 점 추가 시 O(N) 재계산.
4. **유령 점 risk**: ICP fitness가 높을 때 새 점이 발견됐다는 것은 (a) 진짜 환경 변화 또는 (b) 동적 객체(다른 보트, 사람, ROV 자신) 둘 중 하나. LiDAR scan만으로 둘을 구분 어렵다.
5. **측정 데이터 부재**: 지금까지의 cross-bag localization 실패(audit 메모 1d 항목)는 "환경 변화" 때문이 아니라 "보트 trajectory가 다른 영역을 지나서 lidar가 본 환경이 다름" 때문. 시간 경과에 따라 fitness가 단조 감소하는 데이터는 미수집.
6. **PKRC 환경 특성**: 수조/현장 시험 — 환경 변화 빈도가 낮고, 변화가 있을 경우 mapping 모드 재실행 비용이 낮다.

### 미래 trigger 조건
다음 중 하나가 측정 데이터로 확인되면 별 phase에서 "frozen base + delta map" 분리 구조로 진행한다:
- 동일 환경 multi-day 운영에서 시간당 fitness가 monotonically 하락하는 plot
- 환경 변화 빈도가 운영 보고로 정량화됨

---

## 3. Q2 — SLAM Quality Gate 디자인

### 3.1 데이터 흐름

```
fast_lio (loc 모드)
   │
   ├── /fast_lio/localization/odometry      (nav_msgs/Odometry, ~200Hz)
   │       │
   │       └── 3d_mapper_node._odom_callback → _latest_odom_msg
   │
   └── /fast_lio/localization/confidence    (std_msgs/Float32, ~200Hz)
           │
           └── 3d_mapper_node._confidence_callback
                  → _latest_confidence
                  → _latest_confidence_wall_time = time.time()

sonar_msg 도착 (~14Hz)
   │
   └── _sonar_callback
        │
        ├── (기존) time-sync gate: |sonar_t - odom_t| > 0.1s → drop
        │
        └── (NEW) quality gate:
              if gate_enabled:
                  age = wall_now - _latest_confidence_wall_time
                  if _latest_confidence is None and age < grace_period_sec:
                      pass  # warmup
                  elif _latest_confidence is None or age > stale_timeout_sec:
                      if fail_mode == 'closed': drop
                      else: pass + throttled WARN
                  elif _latest_confidence < threshold:
                      drop + log
                  else:
                      pass
```

### 3.2 임계치 결정 (default = 0.4)

`fast_lio` 내부 동작과의 관계:
- `localization_engine.cpp:542-546`: `fitness > fitness_threshold_(0.5)` 일 때만 `mat_odom2map_` 보정. 즉 fitness 0.4~0.5 구간은 **마지막 신뢰 보정값을 freeze한 상태로 odom publish 계속**.
- `localization_engine.cpp:435`: `fitness < 0.3` → "LOCALIZATION LOST" ERROR 로그.

| 임계치 | 의미 | 운영 효과 |
|---|---|---|
| 0.5 (보수적) | fast_lio 내부 gate와 동일. fast_lio가 보정 안 하는 모든 구간 drop | 정상 운영에서 false-drop 가능 (보트 정지·일시 fitness 하락) |
| **0.4 (기본)** | fast_lio가 freeze 중이지만 LOST는 아닌 구간까지 허용 | 정상 운영 frame 99%+ 통과, 진짜 LOST 직전 구간만 drop |
| 0.3 (관대) | LOST 임계까지 허용 | drop이 거의 안 됨 — gate 효과 미미 |

audit 메모 데이터 인용: same-bag init 0.994, 정상 tracking 0.5~0.9, cross-bag 실패 0.05~0.12. → **0.4가 합리적 중간점**, 운영 데이터 누적 후 조정 가능하도록 파라미터화.

### 3.3 Fail mode 정책 (default = `open`)

confidence 토픽이 안 올 때(예: cartographer 모드, fast_lio mapping 모드, fast_lio_loc인데 confidence publisher 미동작):

| 모드 | 동작 | 적합 케이스 |
|---|---|---|
| **`open` (기본)** | 첫 `grace_period_sec(=1.0)` 동안 confidence 없어도 통과. 그 후 stale(>`stale_timeout_sec(=5.0)`) 또는 None이면 throttled WARN + frame 통과 | cartographer/fast_lio mapping 모드 backward-compat. 새 기능이 다른 운영 모드를 깨지 않는다 |
| `closed` | 같은 조건에서 frame drop | 안전 우선 운영, fast_lio_loc만 쓰는 환경 |

**default가 `open`인 이유**: 한 노드의 새 기능이 다른 운영 모드를 깨면 안 된다. cartographer 모드에서 confidence 토픽 미발행은 정상 상태이므로 `open`이 backward-compat.

### 3.4 Stale 검출

`std_msgs/Float32`는 timestamp가 없다. 두 옵션:
- **(채택)** confidence callback 시점의 wall time(`time.time()`) 기록. `stale_timeout_sec` 경과 시 stale 판정.
- (미채택) fast_lio 측을 stamped 메시지로 변경. cross-repo 변경 → 별 phase로 분리.

### 3.5 SLAM 소스별 호환성

`launch/3d_mapping.launch.py`의 기존 `ODOMETRY_CONFIG` dict와 같은 패턴으로 confidence topic도 source-specific 매핑:

```python
CONFIDENCE_CONFIG = {
    'cartographer':  '',                                 # confidence 토픽 없음
    'fast_lio':      '',                                 # mapping 모드는 fitness 의미 없음
    'fast_lio_loc':  '/fast_lio/localization/confidence',
}
```

빈 문자열이 들어오면 `3d_mapper_node`는 confidence subscription을 만들지 않고, gate를 자동으로 disabled로 간주한다(`_latest_confidence`가 영원히 None인 상태에서 fail mode = open이면 모든 frame 통과).

### 3.6 로깅 패턴

기존 `_sync_drop_count` 형식과 동형:

```python
# 기능 ON 시 첫 INFO 한 번
self.get_logger().info(
    f'[SlamQuality] Gate enabled: threshold={th:.2f}, fail_mode={fm}, '
    f'topic={confidence_topic}'
)

# Frame drop 시 throttled WARN
if dropped:
    self._quality_drop_count += 1
    if self._quality_drop_count % 10 == 1:
        self.get_logger().warn(
            f'[SlamQuality] Drop: confidence={c:.3f} < {th:.3f} '
            f'(dropped {self._quality_drop_count} total)'
        )

# Stale 시 throttled WARN (5초 throttle, fail_mode=open)
self.get_logger().warn_throttle(
    5.0, f'[SlamQuality] Stale confidence (age={age:.1f}s > {timeout:.1f}s) - passing frame'
)
```

### 3.7 파라미터 인터페이스 (`config.py`의 `MAPPER_PARAMS`에 추가)

```python
# Topics (topics.* 카테고리)
ParameterDef('topics.slam_confidence', '',
    'SLAM confidence topic (Float32). Empty = gate disabled.'),

# SLAM Quality (slam_quality.* 카테고리)
ParameterDef('slam_quality.threshold', 0.4,
    'Minimum SLAM confidence to accept sonar frame. Below this → drop.'),
ParameterDef('slam_quality.fail_mode', 'open',
    "Behavior when confidence is None or stale: 'open' (pass) or 'closed' (drop)."),
ParameterDef('slam_quality.grace_period_sec', 1.0,
    'Initial period after node start where missing confidence is tolerated.'),
ParameterDef('slam_quality.stale_timeout_sec', 5.0,
    'Confidence is considered stale if last message older than this (seconds).'),
```

런타임 변경 가능 항목(`parameter_callback`에서 처리):
- `slam_quality.threshold` (즉시 반영)
- `slam_quality.fail_mode` (즉시 반영)

런타임 변경 불가(노드 재기동 필요):
- `topics.slam_confidence` (subscription 재생성 비용 vs YAGNI)

### 3.8 변경 파일 목록

| 파일 | 변경 |
|---|---|
| `scripts/3d_mapper_node.py` | confidence subscriber, `_confidence_callback`, gate logic in `_sonar_callback`, `parameter_callback` 확장. ~40줄 |
| `scripts/config.py` | `MAPPER_PARAMS`에 위 4개 ParameterDef 추가. ~15줄 |
| `launch/3d_mapping.launch.py` | `CONFIDENCE_CONFIG` dict, parameter passthrough. ~20줄 |
| `launch/robot_3d_mapping.launch.py` | 동일 패턴 대칭 update. ~10줄 |
| `CHANGELOG.md` | `[Unreleased]` 항목 추가 (Added: SLAM quality gating) |

**수정 대상 외**: `fast_lio`(이미 confidence publish 중), `cartographer_slam`(이 phase 범위 외).

---

## 4. Q3 — Cartographer 비교 (정보, 작업 없음)

| 항목 | fast_lio (이 repo) | cartographer (이 repo) |
|---|---|---|
| Per-frame fitness scalar | `current_fitness_` → Float32 publish | **없음** |
| 매칭 임계치 (config) | `fitness_threshold=0.5` (yaml) | `min_score=0.55`, `global_localization_min_score=0.6` (slam_2d.lua:134-135) |
| Quality 시각화 | (없음, scalar만) | `/cartographer_2d/constraints` MarkerArray (node.cpp:118, 395-397) |
| Pure localization 모드 | 별도 launch + map_path | `pure_localization_trimmer` (slam_2d.lua:125) |
| 맵 업데이트 정책 | read-only | graph-SLAM 본질상 새 submap 계속 생성 |

cartographer 모드에서도 quality gating을 활성화하려면 `cartographer_slam` fork에 confidence publisher 추가 필요(예: `map_builder_bridge.cpp`에서 latest constraint score를 Float32로 publish). **이 phase 범위 외, future work**.

---

## 5. 검증 절차 (Manual)

sonar repo에 자동화 회귀 인프라 없음(`scripts/regression_*.sh` 부재). 이번 phase에서는 manual 검증으로 진행하고, 영구 회귀 인프라는 별 phase로 분리.

### 5.1 Acceptance test

1. **Backward compat (cartographer/fast_lio 모드)**:
   ```bash
   ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=cartographer
   ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio
   ```
   기대: 시작 시 `[SlamQuality] Gate disabled (no confidence topic)` INFO 한 번. 이후 frame drop 0건.

2. **fast_lio_loc 모드 + 정상 bag**:
   ```bash
   ros2 launch fast_lio localization.launch.py map_path:=<map>.pcd
   ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio_loc
   ros2 bag play <good_bag>
   ```
   기대: `[SlamQuality] Gate enabled: threshold=0.40, fail_mode=open, topic=/fast_lio/localization/confidence` INFO 한 번. drop count < 5%.

3. **fast_lio_loc 모드 + low-fitness 구간 포함 bag**:
   같은 setup, fitness가 낮은 구간이 있는 bag으로 재생.
   기대: low-fitness 구간 timestamp ≈ `[SlamQuality] Drop` warn timestamp. 그 구간의 voxel 추가 차단됨(gate disabled run 대비 voxel count 감소).

4. **Threshold dynamic param**:
   실행 중 `ros2 param set /sonar_3d_mapper slam_quality.threshold 0.7` → 즉시 throttled drop log 빈도 증가 확인.

### 5.2 PASS 기준

- [ ] Test 1: drop count = 0 (backward-compat 깨짐 없음)
- [ ] Test 2: drop count < 5% (정상 bag false-drop 적음)
- [ ] Test 3: low-fitness 구간 voxel diff > 0 (gate가 실제로 차단)
- [ ] Test 4: threshold 변경이 dynamic param으로 즉시 반영

---

## 6. Future Work

1. **Cartographer confidence publisher** (별 phase): `cartographer_slam`에 per-frame match score를 `/cartographer_2d/confidence` Float32로 publish하는 fork 변경. 그 후 `CONFIDENCE_CONFIG['cartographer']`를 채워 통합.
2. **Stamped confidence message** (cross-repo): fast_lio 측을 `Float32` → 새 stamped 메시지로 바꾸면 wall-time fallback 불필요.
3. **Quality 통계 publish** (모니터링): `_quality_drop_count`, 시간당 평균 confidence를 별도 토픽으로 publish. 운영 모니터링 필요시 추가.
4. **Lifelong mapping** (Q1 미래 trigger 조건 충족 시): frozen base + delta map 분리 구조. §2 참조.
5. **자동화 회귀 인프라** (sonar repo): `scripts/regression_*.sh` 패턴을 fast_lio에서 차용해 `regression_test_quality_gate.sh` 작성.

---

## 7. 참고

- fast_lio audit: `~/.claude/projects/-workspace/memory/reference_fast_lio_localization_audit.md` (2026-05-03)
- fast_lio 회귀 함정: `~/.claude/projects/-workspace/memory/reference_fast_lio_regression_gotchas.md`
- fast_lio confidence publisher: `src/lidar_slam/fast_lio/src/localization/localization_node.cpp:180, 277-279`
- fast_lio 내부 fitness gate: `src/lidar_slam/fast_lio/src/localization/localization_engine.cpp:542-546`
- 기존 sonar drop counter 패턴: `src/sonar_3d_reconstruction/scripts/3d_mapper_node.py:510-520`
- launch source-specific 매핑 패턴: `src/sonar_3d_reconstruction/launch/3d_mapping.launch.py:120-122`
