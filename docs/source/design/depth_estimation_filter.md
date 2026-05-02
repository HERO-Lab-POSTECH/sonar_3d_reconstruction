# 깊이 추정 기반 로봇 탐지 필터링 (Depth Estimation Filter)

## 1. 개요

### 문제
`robot_3d_mapping_v3`는 소나로 스캔한 모든 관측값을 detection map에 기록한다.
이 경우 이미 존재하는 **벽, 바닥 등 기존 환경**도 detection map에 함께 나타나서
새로운 물체(로봇)만 구분하기 어렵다.

### 해결 방법
기존 복셀맵(reference map)에 **레이캐스트**하여 각 방위각별 **예상 깊이**를 구하고,
실제 소나 측정값과 비교하여 **실제 깊이가 예상보다 유의미하게 가까운 방위각만**
detection map에 기록한다.

→ 기존 환경은 자동 필터링되고, 새로운 물체만 탐지 맵에 나타남.

---

## 2. 핵심 원리

### 비교 방향 (단방향 필터)

로봇은 기존 환경(벽, 바닥) **앞에** 나타난다.
따라서 **실제 깊이 < 예상 깊이** (expected - actual > threshold) 인 경우만 새 물체로 판단한다.

```
케이스 1: 로봇이 벽 앞에 있는 경우
  예상 깊이 (프라이어맵) = 8.0m  ← 벽이 8m 거리에 있었음
  실제 깊이 (소나 측정)  = 3.0m  ← 근데 지금 3m에서 뭔가 감지됨
  차이 = expected - actual = 8.0 - 3.0 = 5.0m > threshold(1.0m)
  → 새 물체! (로봇) → detection map에 기록

케이스 2: 기존 벽만 있는 경우
  예상 깊이 (프라이어맵) = 5.2m  ← 벽이 5.2m
  실제 깊이 (소나 측정)  = 5.1m  ← 지금도 5.1m에서 감지
  차이 = expected - actual = 5.2 - 5.1 = 0.1m < threshold(1.0m)
  → 기존 환경 → 스킵

케이스 3: 소나가 레퍼런스보다 더 멀리 보는 경우
  예상 깊이 (프라이어맵) = 4.0m
  실제 깊이 (소나 측정)  = 8.3m  ← 더 멀리 보임
  차이 = expected - actual = 4.0 - 8.3 = -4.3m (음수)
  → 기존 환경 (노이즈) → 스킵
```

### 왜 단방향인가?

처음에는 `abs(actual - expected) > threshold` (양방향)을 사용했으나,
소나가 레퍼런스보다 더 멀리 보는 경우(act > exp)도 "새 물체"로 잡아서
바닥 노이즈가 필터링되지 않는 문제가 있었다.

로봇은 기존 환경 **앞에** 나타나므로, `(expected - actual) > threshold` (단방향)으로
수정하여 실제가 더 가까운 경우만 잡도록 변경했다.

---

## 3. 데이터 흐름

```
소나 이미지 수신
    │
    ▼
[기존] Phase 1: 첫 번째 히트 수집 (_collect_first_hits)
    │
    ▼
[NEW] Phase 1.5: 깊이 추정 필터 (compute_depth_estimation)
    │
    ├─ 1. 각 방위각의 레이 방향 계산 (소나 프레임 → 월드 프레임 변환)
    ├─ 2. 소나 이미지에서 각 방위각의 실제 첫 번째 히트 깊이 추출
    ├─ 3. reference_map.batch_ray_cast_depth() → 예상 깊이 배열
    ├─ 4. (expected - actual) > threshold 비교
    └─ 5. bearing_mask 생성 (True = 새 물체 가능성, False = 기존 환경)
    │
    ▼
[기존] Phase 2: 레이 처리 + 쉐도우 체크 (_process_rays_with_shadow)
    │ bearing_mask[b_idx] == False → 해당 방위각 전체 스킵
    │ bearing_mask[b_idx] == True  → 기존대로 처리
    │
    ▼
[기존] Phase 3: 옥트리 업데이트 (_apply_updates_to_octree)
    │
    ▼
detection map에 새 물체만 기록됨
```

---

## 4. 파라미터

### 런치 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `depth_estimation` | `true` | 깊이 필터 활성화/비활성화 |
| `depth_diff_threshold` | `1.0` | 새 물체 판정 최소 거리 차이 [m] |

**사용 예시:**

```bash
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/home/hero/Data/20260122/map/sonar \
    detection_map_path:=/home/hero/Data/20260122/map/robot_detection_map \
    sonar_model:=m3000d sonar_pitch:=90.0 show_opencv:=true \
    use_sim_time:=true qos_reliability:=best_effort marker_min_depth:=7.0 \
    depth_estimation:=true \
    depth_diff_threshold:=1.0
```

### depth_diff_threshold 조절 가이드

| 값 | 효과 |
|----|------|
| `0.5` | 더 민감 (작은 물체도 검출, 오탐 가능성 증가) |
| `1.0` | 기본값 (일반적 환경에 적합) |
| `2.0` | 덜 민감 (확실한 물체만 검출, 바닥 노이즈 더 제거) |
| `3.0` | 매우 둔감 (큰 물체만 검출) |

### 내부 파라미터 (런치 파일 하드코딩)

| 파라미터 | 값 | 설명 |
|---------|-----|------|
| `ray_step_multiplier` | `2.0` | 레이 스텝 크기 = voxel_resolution × 이 값 |
| `min_confidence` | `0.7` | 레퍼런스 맵 점유 확률 최소값 |

이 값들은 `robot_3d_mapping_v3.launch.py` 내부에 설정되어 있다:
```python
'depth_estimation.ray_step_multiplier': 2.0,
'depth_estimation.min_confidence': 0.7,
```

---

## 5. 비교 결과 판정 로직

`compute_depth_estimation()` 메서드 내부 로직:

| 조건 | 판정 | bearing_mask | 의미 |
|------|------|-------------|------|
| `expected < 0` | no_ref | `True` | 레퍼런스 없음 → 처리 (새 영역일 수 있음) |
| `actual < 0` | matched | `False` | 소나 히트 없음, 레퍼런스 있음 → 스킵 |
| `(expected - actual) > threshold` | new | `True` | 실제가 더 가까움 → 새 물체 |
| 그 외 | matched | `False` | 깊이 일치 또는 더 멀음 → 기존 환경 |

---

## 6. C++ 레이캐스트 구현

### API

```cpp
// 단일 레이캐스트
double ray_cast_depth(
    const Eigen::Vector3d& origin,      // 소나 원점 (월드 좌표)
    const Eigen::Vector3d& direction,    // 레이 방향 (정규화됨)
    double max_range,                     // 최대 탐색 거리 [m]
    double step_size,                     // 스텝 크기 [m]
    double min_probability = 0.7          // 점유 판정 최소 확률
);
// 반환: 첫 번째 점유 복셀까지의 거리, 또는 -1.0 (미스)

// 배치 레이캐스트 (N개 방향 한번에)
Eigen::VectorXd batch_ray_cast_depth(
    const Eigen::Vector3d& origin,
    const Eigen::MatrixXd& directions,   // Nx3 행렬
    double max_range,
    double step_size,
    double min_probability = 0.7
);
// 반환: N개의 거리 벡터 (-1.0 = 미스)
```

### 동작 방식

```
ray_cast_depth(origin, direction, max_range, step_size, min_probability):

  1. min_probability → log-odds 변환:  L = log(p / (1-p))
  2. direction 정규화
  3. for t = 0 → max_range (step_size 간격):
     a. point = origin + t × direction
     b. tile_index 계산 (타일 경계 자동 처리)
     c. 타일 캐시에서 로드 (LRU 캐시 활용)
     d. octree->search(point) 로 복셀 조회
     e. node의 log_odds > min_log_odds 이면 → return t (히트!)
  4. return -1.0 (미스)
```

### 타일 경계 처리

레이가 타일 경계를 넘어갈 때 자동으로 새 타일을 로드한다.
LRU 캐시를 사용하므로 인접 타일은 대부분 메모리에 있다.

```
타일 A (0~10m)        타일 B (10~20m)
┌──────────────┐  ┌──────────────┐
│  ●──────────────────●          │
│  origin      │  │  hit!        │
└──────────────┘  └──────────────┘
    레이가 타일 경계를 넘어도 자동으로 타일 B 로드
```

### Python에서 호출

```python
import numpy as np
from sonar_3d_reconstruction_cpp import OutofcoreTileMapper

mapper = OutofcoreTileMapper("/path/to/map", resolution=0.2, tile_size=10.0)

# 단일 레이캐스트
origin = np.array([1.0, 2.0, 3.0])
direction = np.array([1.0, 0.0, 0.0])  # x 방향
depth = mapper.ray_cast_depth(origin, direction, max_range=30.0, step_size=0.4)
# depth = 5.3 (5.3m 거리에 점유 복셀) 또는 -1.0 (없음)

# 배치 레이캐스트 (N개 방향)
directions = np.array([
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.7, 0.7, 0.0],
], dtype=np.float64)
depths = mapper.batch_ray_cast_depth(origin, directions, max_range=30.0, step_size=0.4)
# depths = [5.3, -1.0, 8.1]  (각 방향의 깊이)
```

---

## 7. 수정된 파일 목록

| # | 파일 | 변경 내용 |
|---|------|----------|
| 1 | `cpp/outofcore_tile_mapper.h` | `ray_cast_depth()`, `batch_ray_cast_depth()` 선언 |
| 2 | `cpp/outofcore_tile_mapper.cpp` | 레이캐스트 C++ 구현 |
| 3 | `cpp/python_bindings.cpp` | pybind11 바인딩 추가 |
| 4 | `scripts/config.py` | `depth_estimation.*` 파라미터 5개 정의 |
| 5 | `scripts/3d_mapper.py` | reference map 로드 + 깊이 필터 로직 |
| 6 | `scripts/3d_mapper_node.py` | 로그 메시지 추가 |
| 7 | `launch/robot_3d_mapping_v3.launch.py` | 런치 인자 + 파라미터 연결 |

### 기존 코드 영향

- `depth_estimation.enabled` 기본값은 `False` → **기존 런치 파일에 영향 없음**
- `3d_mapping.launch.py`, `robot_3d_mapping_v2.launch.py` 등 **미영향**
- `_process_rays_with_shadow()`에 `depth_filter_mask=None` 파라미터 추가
  → `None`이면 기존과 동일하게 동작

---

## 8. 로그 출력

### 시작 시

```
[INFO] [DepthEstimation] ENABLED (ref=/home/hero/Data/20260122/map/sonar)
```

### 매 20프레임마다

```
[INFO] [DepthEst] matched=180 new=12 no_ref=64 |
  [-39° act=9.6 exp=10.0 skip]
  [+13° act=8.4 exp=8.4 skip]
  [+39° act=3.0 exp=8.0 NEW]
```

| 필드 | 의미 |
|------|------|
| `matched` | 기존 환경과 일치하여 필터링된 방위각 수 |
| `new` | 새 물체 가능성으로 처리된 방위각 수 |
| `no_ref` | 레퍼런스 데이터 없어서 처리된 방위각 수 |
| `act` | 실제 소나 측정 깊이 [m] (-1.0 = 히트 없음) |
| `exp` | 레퍼런스 맵 레이캐스트 깊이 [m] (-1.0 = 데이터 없음) |
| `skip` | 필터링됨 (detection map에 안 들어감) |
| `NEW` | 새 물체로 판단됨 (detection map에 들어감) |

---

## 9. 성능

- 방위각 수: ~256개 (bearing_step에 의해)
- 프레임 레이트: ~2.8 fps (frame_skip=5 적용)
- 초당 레이캐스트: ~256 × 2.8 = ~717회
- 레이당 최대 스텝: max_range / step_size = 10m / 0.4m = 25 스텝
- 각 스텝: `octree->search()` = O(log N) 트리 조회
- 타일 캐시: LRU 캐시(16개)로 대부분 메모리 히트
- **예상 추가 오버헤드**: 프레임당 5~15ms

---

## 10. 테스트 방법

### 깊이 필터 ON (기본)

```bash
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/home/hero/Data/20260122/map/sonar \
    detection_map_path:=/home/hero/Data/20260122/map/robot_detection_map \
    sonar_model:=m3000d sonar_pitch:=90.0 show_opencv:=true \
    use_sim_time:=true qos_reliability:=best_effort marker_min_depth:=7.0
```

### 깊이 필터 OFF (비교용)

```bash
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/home/hero/Data/20260122/map/sonar \
    detection_map_path:=/home/hero/Data/20260122/map/robot_detection_map_nofilter \
    sonar_model:=m3000d sonar_pitch:=90.0 show_opencv:=true \
    use_sim_time:=true qos_reliability:=best_effort marker_min_depth:=7.0 \
    depth_estimation:=false
```

### threshold 조절 테스트

```bash
# 더 민감하게 (0.5m)
... depth_diff_threshold:=0.5

# 더 둔감하게 (2.0m)
... depth_diff_threshold:=2.0
```

### 확인 포인트

1. 로그에 `[DepthEstimation] ENABLED` 메시지 확인
2. `matched` 수가 높을수록 기존 환경이 잘 필터링됨
3. `new` 수가 실제 새 물체 수와 비슷한지 확인
4. RViz MarkerArray에서 기존 벽/바닥이 줄었는지 시각적 비교
