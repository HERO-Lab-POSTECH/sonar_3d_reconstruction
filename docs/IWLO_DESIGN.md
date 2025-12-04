# Intensity-Weighted Log-Odds (IWLO) 확률 업데이트 방법

## 개요

IWLO는 Log-Odds Bayesian과 Weighted Average 방식의 장점을 융합한 새로운 확률 업데이트 방법입니다.

**핵심 아이디어**: 강도 정보를 시그모이드 함수로 변환하여 log-odds 업데이트 강도를 조절하고, 관측 횟수에 따라 학습률을 감쇠시켜 자연 수렴을 유도합니다.

---

## 1. 배경

### 기존 방법의 한계

| 방법 | 장점 | 한계 |
|------|------|------|
| **Log-Odds** | 수학적 정당성, 빠른 변화 감지 | 강도 정보 무시, saturation 위험 |
| **Weighted Average** | 강도 활용, 자연 수렴 | Bayesian 정당성 약화, 변화 감지 느림 |

### IWLO 설계 목표

1. 강도 정보를 연속적으로 활용
2. Log-odds 기반 Bayesian 업데이트 유지
3. 관측 횟수에 따른 신뢰도 반영
4. 환경 변화에 대한 적절한 반응성 유지
5. Saturation 방지

---

## 2. 수학적 공식

### 2.1 강도-가중치 변환

```
w(I) = sigmoid(sharpness × (normalized - 0.5))

where:
  normalized = (I - threshold) / (max_intensity - threshold)
  sigmoid(x) = 1 / (1 + exp(-x))
```

**파라미터**:
- `threshold`: 강도 임계값 (기본: 35)
- `max_intensity`: 최대 강도값 (기본: 255)
- `sharpness`: 시그모이드 경사도 (기본: 3.0)

**특성**:
- 임계값 이하: w = 0 (무시)
- 중간 강도: 부드러운 전환
- 최대 강도: w → 1

### 2.2 학습률 감쇠

```
α(n) = max(min_alpha, 1 / (1 + decay_rate × n))
```

**파라미터**:
- `decay_rate`: 감쇠율 (기본: 0.1)
- `min_alpha`: 최소 학습률 (기본: 0.1)

**특성**:
- 첫 관측 (n=0): α = 1.0
- 10번 관측 (n=10): α = 0.5
- 수렴 후에도 min_alpha로 변화 감지 가능

### 2.3 적응형 스케일링

```
if current_prob < adaptive_threshold:
    scale = (current_prob / adaptive_threshold) × adaptive_max_ratio
else:
    scale = 1.0
```

**목적**: 이미 free로 판정된 영역에 노이즈성 점유 업데이트 억제

### 2.4 최종 업데이트 공식

**점유 업데이트** (intensity > threshold):
```
ΔL = L_occ × w(I) × α(n) × scale
```

**Free 업데이트** (intensity ≤ threshold):
```
ΔL = L_free × α(n)
```

**적용**:
```
new_log_odds = clip(old_log_odds + ΔL, L_min, L_max)
```

---

## 3. 파라미터 설명

| 파라미터 | 기본값 | 범위 | 설명 |
|----------|--------|------|------|
| `L_occ` | 1.5 | [0.5, 3.0] | 최대 점유 log-odds 증분 |
| `L_free` | -2.0 | [-3.0, -0.5] | Free space log-odds 증분 |
| `L_min` | -2.0 | [-5.0, 0.0] | Saturation 하한 (P≈0.12) |
| `L_max` | 3.5 | [2.0, 5.0] | Saturation 상한 (P≈0.97) |
| `sharpness` | 3.0 | [1.0, 5.0] | 시그모이드 경사도 |
| `decay_rate` | 0.1 | [0.05, 0.5] | 학습률 감쇠율 |
| `min_alpha` | 0.1 | [0.01, 0.3] | 최소 학습률 |

### 파라미터 튜닝 가이드

**sharpness 조정**:
- 높은 값 (4-5): 강도에 민감, 이진 분류에 가까움
- 낮은 값 (1-2): 강도에 둔감, 부드러운 전환

**decay_rate 조정**:
- 높은 값 (0.3-0.5): 빠른 수렴, 변화 감지 약화
- 낮은 값 (0.05-0.1): 느린 수렴, 변화 감지 유지

**min_alpha 조정**:
- 높은 값 (0.2-0.3): 환경 변화에 빠른 반응
- 낮은 값 (0.01-0.05): 안정적이지만 변화 감지 느림

---

## 4. 예상 동작

### 시나리오별 비교

| 시나리오 | Log-Odds | Weighted Avg | IWLO |
|----------|----------|--------------|------|
| 강한 신호 첫 관측 | P=0.82 | P=0.95 | P=0.88 |
| 약한 신호 누적 | 노이즈 축적 | 느린 수렴 | 안정적 수렴 |
| 환경 변화 | 빠른 반응 | 느린 반응 | 적절한 반응 |
| 장기 안정성 | saturation | 안정 | 범위 제한 안정 |

### 수렴 특성

```
관측 횟수 vs 확률 변화:
n=1:  강한 신호 → P=0.7~0.8
n=5:  반복 관측 → P=0.85~0.90
n=10: 안정화 → P=0.90~0.95
n=20: 최종 수렴 → P=0.95~0.97 (L_max 제한)
```

---

## 5. 사용법

### Launch 파일
```bash
# IWLO 방법 사용
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py method:=iwlo

# 기존 방법들
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py method:=log_odds
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py method:=weighted_average
```

### 파라미터 오버라이드
```bash
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
    method:=iwlo \
    sharpness:=4.0 \
    decay_rate:=0.2
```

### YAML 설정
```yaml
# config/method_iwlo.yaml
sonar_3d_mapper:
  ros__parameters:
    L_occ: 1.5
    L_free: -2.0
    L_min: -2.0
    L_max: 3.5
    sharpness: 3.0
    decay_rate: 0.1
    min_alpha: 0.1
```

---

## 6. 구현 상세

### Python 코드 (3d_mapper.py)

```python
def _intensity_to_weight(self, intensity):
    """시그모이드 기반 강도→가중치 변환"""
    if intensity <= self.intensity_threshold:
        return 0.0
    normalized = (intensity - self.intensity_threshold) / \
                 (self.intensity_max - self.intensity_threshold)
    x = self.sharpness * (normalized - 0.5)
    return 1.0 / (1.0 + np.exp(-x))

def _compute_alpha(self, observation_count):
    """관측 횟수 기반 학습률 계산"""
    return max(self.min_alpha, 1.0 / (1.0 + self.decay_rate * observation_count))
```

### C++ 코드 (probability_updater.cpp)

```cpp
double ProbabilityUpdater::intensity_to_weight(double intensity) const {
    if (intensity <= intensity_threshold_) return 0.0;
    double normalized = (intensity - intensity_threshold_) /
                        (intensity_max_ - intensity_threshold_);
    double x = sharpness_ * (normalized - 0.5);
    return 1.0 / (1.0 + std::exp(-x));
}

double ProbabilityUpdater::compute_alpha(int observation_count) const {
    return std::max(min_alpha_, 1.0 / (1.0 + decay_rate_ * observation_count));
}
```

---

## 7. 참고 문헌

1. **Moravec & Elfes (1985)**: Log-odds Bayesian 점유 격자 매핑의 원조
2. **OctoMap (Hornung et al., 2013)**: Saturation limits로 dead-locking 방지
3. **VoxelMap (Yuan et al., 2022)**: Weighted average 기반 실시간 매핑
4. **CMU Sonar (Teixeira et al., 2016)**: 수중 소나 특화 센서 모델
5. **BEVFusion DOGM (Kim et al., 2024)**: 딥러닝 기반 센서 융합

---

## 8. 변경 이력

| 날짜 | 버전 | 변경 내용 |
|------|------|-----------|
| 2024-12-04 | 1.0 | 초기 설계 및 구현 |
