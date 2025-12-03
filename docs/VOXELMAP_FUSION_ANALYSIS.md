# Voxelmap Fusion vs Sonar 3D Reconstruction 비교 분석

## 1. 개요

이 문서는 `voxelmap_fusion` (2024.02.05)과 `sonar_3d_reconstruction` 패키지의 소나 기반 3D 매핑 알고리즘을 비교 분석합니다.

| 항목 | voxelmap_fusion | sonar_3d_reconstruction |
|------|-----------------|-------------------------|
| **ROS 버전** | ROS1 (kinetic) | ROS2 (humble) |
| **언어** | Python | Python + C++ |
| **데이터 구조** | Dense Voxel Grid | Sparse Octree |
| **확률 모델** | Weighted Average | Log-Odds Bayesian |
| **좌표 변환** | Spherical Segment | Ray-based |

---

## 2. 핵심 알고리즘 비교

### 2.1 확률 업데이트 방식

#### voxelmap_fusion: Weighted Average (가중 평균)
```python
# Voxel.py:71-72
old_OP_scaled = (num_sonar_constraints + num_optical_constraints) * old_OP
new_OP = (old_OP_scaled + OP) / (num_sonar_constraints + num_optical_constraints + 1)
```

**특징:**
- 모든 제약조건을 동등하게 취급
- 새로운 관측이 추가될 때마다 평균 재계산
- 각 제약조건의 영향력이 균등 (1/N)
- 제약조건별 점유도 추적 (역추적 가능)

**장점:**
- 직관적이고 이해하기 쉬움
- 제약조건 추적으로 업데이트 전파 가능
- 이상치(outlier)에 덜 민감

**단점:**
- 관측 횟수가 많아지면 새 관측의 영향력 감소
- 수렴 속도가 느림
- 초기 관측에 과도한 가중치

#### sonar_3d_reconstruction: Log-Odds Bayesian (로그-오즈 베이지안)
```python
# 3d_mapper.py:109-136
log_odds_new = log_odds_old + delta_log_odds
probability = 1 / (1 + exp(-log_odds))
```

**특징:**
- 확률의 곱셈을 로그 공간에서 덧셈으로 변환
- 각 관측이 독립적으로 기여
- Clamping으로 극단값 제한 (-10 ~ 10)
- 적응형 스케일링으로 노이즈 감소

**장점:**
- 빠른 수렴
- 관측 횟수와 무관한 업데이트 강도
- 수학적으로 견고한 확률 모델
- 실시간 처리에 적합

**단점:**
- 이상치에 민감할 수 있음
- 개별 관측 추적 불가

### 2.2 점유도 값 할당

#### voxelmap_fusion: Binary (이진값)
```python
# SonarConstraint.py:277, 286
# Occupied: OP = 1.0
# Empty: OP = 0.0
```

**문제점:**
- 소나 강도값(0~1)을 활용하지 않음
- 강한 반사와 약한 반사를 구분하지 못함
- 임계값 의존성이 높음

#### sonar_3d_reconstruction: Intensity-Based (강도 기반)
```python
# 3d_mapper.py:514-525
if intensity > threshold:
    log_odds_update = log_odds_occupied  # +1.5
else:
    log_odds_update = log_odds_free      # -2.0
```

**개선 가능:**
- 현재도 이진 분류이나, 강도값 연속 반영 가능
- 예: `log_odds = base_log_odds * (intensity / max_intensity)`

### 2.3 데이터 구조

#### voxelmap_fusion: Dense Voxel Grid
```python
# VoxelMap.py:81-102
voxels = []  # 모든 복셀 객체 저장
voxel_positions = np.zeros((num_X * num_Y * num_Z, 3))

for i in range(num_voxels_X):
    for j in range(num_voxels_Y):
        for k in range(num_voxels_Z):
            voxels.append(Voxel())  # 모든 복셀 생성
```

**메모리 사용량:**
- 81 x 81 x 81 = 531,441 복셀
- 각 Voxel 객체: ~200+ bytes (리스트, 배열 포함)
- 총: ~100MB 이상

**문제점:**
- 미관측 영역도 메모리 점유
- 해상도 증가 시 O(N³) 메모리 증가
- 초기화 시간 길음

#### sonar_3d_reconstruction: Sparse Octree
```python
# 3d_mapper.py:45-77
class SimpleOctree:
    voxels = defaultdict(float)  # {(ix, iy, iz): log_odds}
```

**메모리 사용량:**
- 관측된 복셀만 저장
- 29~93배 메모리 절감 (실험 결과)
- 동적 경계 확장

**장점:**
- 메모리 효율적
- 무한 맵 크기 지원
- 빠른 초기화

### 2.4 좌표 변환

#### voxelmap_fusion: Spherical Segment (구면 세그먼트)
```python
# SonarConstraint.py:138-150
R = Roll Matrix (X축 회전)
P = Pitch Matrix (Y축 회전)
Y = Yaw Matrix (Z축 회전)
T = Tilt Matrix (소나 틸트)
Rot = Y @ P @ R @ T

# 복셀이 구면 세그먼트 내부인지 검사
# SonarConstraint.py:175-180
if voxel_vector @ Rot @ boundary_normal > 0:
    continue  # FOV 외부
```

**특징:**
- 각 픽셀을 구면 세그먼트로 모델링
- 반평면(half-plane) 검사로 FOV 판정
- 정확한 기하학적 검사

**장점:**
- 수학적으로 정확한 FOV 검사
- 임의 방향의 소나 지원

**단점:**
- 픽셀당 개별 세그먼트 생성 (비효율)
- O(N × K) 복잡도 (N=픽셀, K=복셀)

#### sonar_3d_reconstruction: Ray-based (광선 기반)
```python
# 3d_mapper.py:544-551
x_sonar = range_m * cos(vertical_angle) * cos(bearing_angle)
y_sonar = -range_m * cos(vertical_angle) * sin(bearing_angle)
z_sonar = range_m * sin(vertical_angle)

pt_world = T_sonar_to_world @ pt_sonar
```

**특징:**
- 각 빔(bearing)을 따라 광선 추적
- 수직 확산(vertical spread) 계산
- 변환 행렬 사전 계산

**장점:**
- 직관적인 광선 모델
- 수직 어퍼처 완전 지원
- 효율적인 레이 처리

### 2.5 제약조건 전파 (Constraint Propagation)

#### voxelmap_fusion: Chain Update (연쇄 업데이트)
```python
# SonarConstraint.py:290-331
# 복셀 상태 변화 시 같은 세그먼트의 다른 복셀에 분배
target_voxel_OP_diff = new_OP - old_OP
OP_to_update = target_voxel_OP_diff / (num_voxels - 1)

for each other_voxel:
    other_voxel.OP += OP_to_update
```

**개념:**
- 구면 세그먼트 내 복셀들은 상호 의존적
- 한 복셀이 점유로 확정되면 다른 복셀은 비점유 가능성 증가
- 반복적 수렴 (현재 비활성화: num_max_loops = 0)

**장점:**
- 물리적으로 더 정확한 모델링
- 불확실성 감소

**단점:**
- 계산 비용 높음
- 수렴 보장 어려움
- 현재 구현에서 비활성화됨

#### sonar_3d_reconstruction: Independent Update (독립 업데이트)
- 각 복셀 독립적으로 업데이트
- 광선 경로 상 free space 업데이트
- 제약조건 전파 없음

---

## 3. 성능 비교

| 항목 | voxelmap_fusion | sonar_3d_reconstruction |
|------|-----------------|-------------------------|
| **처리 속도** | ~0.5 fps (추정) | ~15 fps (C++) |
| **메모리 효율** | 낮음 (Dense) | 높음 (Sparse) |
| **해상도 확장성** | O(N³) | O(관측 복셀) |
| **초기화 시간** | 느림 | 빠름 |
| **실시간 처리** | 어려움 | 가능 |

---

## 4. voxelmap_fusion의 고급 기능

### 4.1 제약조건 추적 시스템
```python
# Voxel.py:16-21
num_sonar_constraints = 0
sonar_constraint_IDs = []  # [SC_ID, col_ID, row_ID]
sonar_constraint_occupancy_probabilities = []
```

**활용:**
- 어떤 관측이 복셀에 영향을 미쳤는지 추적
- 관측 제거/수정 시 역업데이트 가능
- 센서 융합에서 기여도 분석

### 4.2 상태 변화 감지
```python
# VoxelMap.py:357-373
if (old_OP <= thres_empty and new_OP > thres_empty) or \
   (old_OP > thres_empty and new_OP <= thres_empty) or \
   (old_OP < thres_occ and new_OP >= thres_occ) or \
   (old_OP >= thres_occ and new_OP < thres_occ):
    state_changed = True
```

**활용:**
- 점유/비점유 상태 전환 감지
- 선택적 업데이트로 성능 향상
- 맵 변화 이벤트 발생

### 4.3 복셀 축소 최적화
```python
# VoxelMap.py:153-158
reduced_voxel_IDs = sonar_constraint.find_overlapping_voxels(
    voxel_positions, r_lims, theta_lims, phi_lims
)
# "Number of voxels -- org. vs. reduced: 531441 vs 1234"
```

**효과:**
- FOV 내 복셀만 처리
- 90% 이상 복셀 제외 가능

---

## 5. sonar_3d_reconstruction 개선 권장사항

### 5.1 높은 우선순위

#### 1) 제약조건 추적 시스템 도입
```python
class Voxel:
    def __init__(self):
        self.observation_history = []  # [(frame_id, bearing_id, log_odds), ...]
        self.total_observations = 0
```

**효과:**
- 관측 이력 추적으로 디버깅 용이
- 관측 제거/수정 지원
- 센서별 기여도 분석

#### 2) 강도 기반 연속 확률 업데이트
```python
# 현재: 이진 분류
log_odds = log_odds_occupied if intensity > threshold else log_odds_free

# 개선: 연속 강도 반영
intensity_normalized = (intensity - threshold) / (max_intensity - threshold)
intensity_normalized = np.clip(intensity_normalized, 0, 1)
log_odds = log_odds_free + (log_odds_occupied - log_odds_free) * intensity_normalized
```

**효과:**
- 소나 강도 정보 완전 활용
- 강한 반사 vs 약한 반사 구분
- 더 정밀한 확률 추정

#### 3) 다중 임계값 세분화
```python
# voxelmap_fusion 방식
threshold_occupied = 0.5  # 점유로 간주
threshold_empty = 0.3     # 비점유로 간주

# 3단계 분류: empty, unknown, occupied
```

### 5.2 중간 우선순위

#### 4) 구면 세그먼트 기반 FOV 검증
현재 `is_bearing_in_valid_fov()`는 단순 각도 검사만 수행. voxelmap_fusion의 반평면 검사 방식 도입 고려:

```python
def is_point_in_fov(self, point, sensor_pose):
    """반평면 기반 정확한 FOV 검사"""
    relative_vector = point - sensor_pose.position

    # 방위각 경계 검사
    theta_min_normal = [sin(theta_min), cos(theta_min), 0]
    theta_max_normal = [sin(theta_max), cos(theta_max), 0]

    if relative_vector @ Rot @ theta_min_normal > 0:
        return False
    if relative_vector @ Rot @ theta_max_normal < 0:
        return False

    # 고도각 경계 검사 (유사하게)
    return True
```

#### 5) 연쇄 업데이트 메커니즘 (선택적)
```python
def propagate_constraints(self, changed_voxels, max_iterations=3):
    """상태 변화 복셀의 영향 전파"""
    for iteration in range(max_iterations):
        newly_changed = []
        for voxel in changed_voxels:
            affected_voxels = self.get_related_voxels(voxel)
            for affected in affected_voxels:
                old_prob = affected.probability
                # 확률 재계산
                new_prob = self.recalculate_probability(affected)
                if abs(new_prob - old_prob) > threshold:
                    newly_changed.append(affected)

        if not newly_changed:
            break
        changed_voxels = newly_changed
```

### 5.3 낮은 우선순위

#### 6) 픽셀 연결 기반 세그먼트 병합
voxelmap_fusion 주석에서 언급된 미구현 기능:
```python
# SonarConstraint.py:226-227
# "This should be accelerated by concatenating the neighboring spherical segments."
```

인접한 동일 상태 픽셀을 하나의 큰 세그먼트로 병합하여 처리 효율화.

#### 7) 센서 융합 가중치 시스템
```python
# 센서별 신뢰도 가중치
sensor_weights = {
    'oculus': 1.0,
    'ping360': 0.8,
    'lidar': 1.2
}

log_odds_update = base_update * sensor_weights[sensor_type]
```

---

## 6. 알고리즘 수식 비교

### 6.1 확률 업데이트 수식

**voxelmap_fusion (Weighted Average):**
$$P_{new} = \frac{n \cdot P_{old} + P_{obs}}{n + 1}$$

여기서:
- $n$ = 기존 제약조건 수
- $P_{old}$ = 기존 점유 확률
- $P_{obs}$ = 새 관측의 점유 확률

**sonar_3d_reconstruction (Log-Odds):**
$$L_{new} = L_{old} + \Delta L$$
$$P = \frac{1}{1 + e^{-L}}$$

여기서:
- $L$ = log-odds = $\log\frac{P}{1-P}$
- $\Delta L$ = 관측에 따른 log-odds 변화량

### 6.2 수렴 특성 비교

| 관측 횟수 | Weighted Avg 새 관측 기여 | Log-Odds 새 관측 기여 |
|-----------|---------------------------|----------------------|
| 1 | 50% | 100% |
| 2 | 33% | 100% |
| 5 | 17% | 100% |
| 10 | 9% | 100% |
| 100 | 1% | 100% |

Log-Odds 방식이 일관된 업데이트 강도를 유지함.

---

## 7. 결론

### voxelmap_fusion 장점 (도입 권장)
1. **제약조건 추적**: 관측 이력 관리 및 역추적
2. **상태 변화 감지**: 효율적인 선택적 업데이트
3. **연쇄 업데이트**: 물리적으로 정확한 모델링 (구현 복잡)
4. **구면 세그먼트 검증**: 정확한 FOV 검사

### sonar_3d_reconstruction 장점 (유지)
1. **Log-Odds 확률 모델**: 빠른 수렴, 견고한 수학적 기반
2. **Sparse Octree**: 메모리 효율성
3. **C++ 백엔드**: 실시간 처리 성능
4. **ROS2 통합**: 현대적 로봇 시스템 호환

### 권장 개선 로드맵
```
Phase 1: 강도 기반 연속 확률 업데이트
Phase 2: 제약조건 추적 시스템 (선택적)
Phase 3: 다중 임계값 세분화
Phase 4: 구면 세그먼트 FOV 검증 (선택적)
```

---

## 참고 파일 경로

**voxelmap_fusion:**
- `/workspace/tmp/Voxelmap_fusion/03_20240205/src/voxelmap_fusion/src/Voxel.py`
- `/workspace/tmp/Voxelmap_fusion/03_20240205/src/voxelmap_fusion/src/SonarConstraint.py`
- `/workspace/tmp/Voxelmap_fusion/03_20240205/src/voxelmap_fusion/src/VoxelMap.py`

**sonar_3d_reconstruction:**
- `/workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/3d_mapper.py`
- `/workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/3d_mapper_node.py`
- `/workspace/ros2_ws/src/sonar_3d_reconstruction/sonar_3d_reconstruction/cpp/`

---

*작성일: 2024-12-04*
