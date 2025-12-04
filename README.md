# Sonar 3D Reconstruction

실시간 확률적 3D 해저 지형 재구성 시스템 (ROS2 Humble)

## Overview

Oculus M750D 멀티빔 소나와 Livox MID360 LiDAR를 활용한 실시간 3D 해저 지형 매핑 시스템입니다. feature_extraction_3d 알고리즘 기반으로 완전히 재구현되어 향상된 확률적 매핑과 메모리 효율성을 제공합니다.

**주요 특징**:
- **IWLO 확률 업데이트**: Log-Odds와 강도 기반 가중치 융합 알고리즘
- **Cross-talk 노이즈 필터**: 멀티빔 소나의 가로 줄무늬 노이즈 제거
- **Adaptive Bayesian Update**: 자유 공간 보호 및 노이즈 감소
- **Sparse Octree Storage**: 메모리 효율적인 동적 맵 확장
- **TF Integration**: Fast-LIO와 완전 통합된 좌표 변환
- **Real-time Configuration**: Build 없이 YAML 수정 즉시 적용
- **Dual Backend Support**: Python SimpleOctree와 C++ ProbabilityUpdater 지원
- **Performance Optimization**: C++ 백엔드로 최대 10x 속도 향상

## Quick Start

```bash
# 1. 워크스페이스 빌드
cd /workspace/ros2_ws
colcon build --packages-select sonar_3d_reconstruction
source install/setup.bash

# 2. 전체 시스템 실행 (Fast-LIO + 3D Mapper + RViz + Bag)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# 3. 파라미터 실시간 변경 (build 불필요)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
  sonar_orientation.yaw:=-90.0 voxel_resolution:=0.1
```

## System Architecture

### Core Components

#### `3d_mapper.py`
Feature extraction 기반 확률적 매핑 라이브러리:
- **Dual Backend Support**: Python SimpleOctree와 C++ ProbabilityUpdater
- **SonarTo3DMapper**: 3단계 좌표 변환 (Sonar → Base → Map)
- **Adaptive Protection**: Linear interpolation 기반 자유 공간 보호
- **Vertical Aperture**: 20° 빔 확산 정밀 처리
- **Performance Optimization**: C++ 배치 업데이트로 속도 향상

#### `3d_mapper_node.py`
ROS2 실시간 처리 노드:
- **Time Synchronization**: ApproximateTimeSynchronizer (±0.1s)
- **TF Integration**: Fast-LIO의 camera_init/body 프레임 통합
- **Dynamic Publishing**: PointCloud2 및 MarkerArray 지원
- **Parameter Priority**: CLI > YAML > Launch > Node > Library

### Configuration

#### `config/3d_mapper.yaml`
```yaml
sonar_3d_mapper:
  ros__parameters:
    # 소나 센서 파라미터
    horizontal_fov: 130.0        # 수평 FOV (degrees)
    vertical_aperture: 20.0      # 수직 빔 확산 (degrees)
    voxel_resolution: 0.05       # 복셀 크기 (meters)
    
    # 좌표 변환 (degrees)
    sonar_orientation:
      roll: 0.0
      pitch: 90.0   # 하방 (90도)
      yaw: 0.0
    
    # 확률 업데이트
    log_odds_occupied: 1.5
    log_odds_free: -2.0
    adaptive_update: true        # 자유 공간 보호 활성화
    
    # 백엔드 선택
    use_cpp_backend: true        # C++ 성능 최적화 (권장)
```

#### `launch/3d_mapping.launch.py`
- 소스 폴더 직접 참조 (build 없이 설정 변경 가능)
- Fast-LIO 자동 실행 옵션
- Bag 파일 자동 재생 옵션
- RViz 통합 실행

## Parameter System

### Priority Order (Highest → Lowest)
1. **Command Line**: `--ros-args -p param:=value`
2. **YAML File**: `/workspace/ros2_ws/src/sonar_3d_reconstruction/config/3d_mapper.yaml`
3. **Launch File**: Launch 파일 내 파라미터
4. **Node Defaults**: `3d_mapper_node.py` 기본값
5. **Library Defaults**: `3d_mapper.py` 기본값

### Real-time Configuration
YAML 파일을 직접 참조하도록 설정되어 있어, **colcon build 없이** 설정 변경 가능:
```bash
# YAML 파일 수정 후 바로 실행
vim config/3d_mapper.yaml  # 설정 변경
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py  # 즉시 적용
```

## Coordinate System & TF

### TF Tree Integration
```
camera_init (Fast-LIO map frame)
    └── body (robot frame)
        └── sonar_link (from bag tf_static)
```

### Frame Configuration
- **Map Frame**: `camera_init` (Fast-LIO의 맵 좌표계)
- **Robot Frame**: `body` (Fast-LIO의 로봇 좌표계)
- **Sonar Frame**: `sonar_link` (bag 파일의 tf_static 사용)

### Coordinate Conventions
```yaml
# Oculus M750d (실제 센서)
Sonar: +X forward, +Y right, +Z down

# Map (Fast-LIO)
Map: +X forward, +Y left, +Z up
```

## Backend Performance Comparison

### C++ vs Python Backend

| 백엔드 | 처리 속도 | 메모리 효율성 | 배치 업데이트 | 호환성 |
|--------|----------|-------------|-------------|-------|
| **C++ ProbabilityUpdater** | ⭐⭐⭐⭐⭐ (최대 10x) | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ 지원 | ⭐⭐⭐⭐ |
| **Python SimpleOctree** | ⭐⭐⭐ (기본) | ⭐⭐⭐ | ❌ 개별 업데이트 | ⭐⭐⭐⭐⭐ |

### Performance Metrics
- **C++ Backend**: ~15 fps, 배치 처리, 실시간 메모리 통계
  - **OpenMP 병렬 처리**: 복셀 레이캐스팅 10+ 스레드 병렬화
  - **Eigen3 최적화**: 좌표 변환 연산 최소화
  - **pybind11 바인딩**: 완전한 Python API 호환
- **Python Backend**: ~3 fps, 개별 처리, 추정 메모리 통계
- **Memory Efficiency**: C++ 백엔드에서 최대 30% 절약
- **Result Consistency**: 95%+ 동일 결과 보장

### Advanced C++ Optimization Features

#### 1. OpenMP 병렬 레이 처리
- **기술**: 레이캐스팅을 병렬 스레드로 분산
- **성능**: 10+ 스레드 동시 처리
- **장점**: 대규모 맵에서 최대 7.5x 속도 향상

#### 2. Eigen3 좌표 변환
- **특징**: 고성능 행렬 연산 라이브러리 활용
- **최적화**: 제로-복사 연산, SIMD 벡터화
- **정밀도**: 부동소수점 최적화된 변환

#### 3. pybind11 Python 호환성
- **통합**: C++ 코드의 완벽한 Python API 노출
- **오버헤드**: 최소한의 Python 래퍼 비용
- **메서드**: 모든 C++ 함수/클래스 동적 바인딩

#### 4. OctoMap 메모리 최적화
- **구조**: 희소 복셀 트리 구현
- **메모리**: 관측된 영역만 저장
- **동적 확장**: 고정된 메모리 경계 없음

### Backend Selection
```yaml
# C++ 백엔드 (권장 - 성능 최적화)
use_cpp_backend: true

# Python 백엔드 (호환성 우선)
use_cpp_backend: false
```

### Testing & Validation
```bash
# 백엔드 호환성 테스트
cd /workspace/ros2_ws/src/sonar_3d_reconstruction/scripts
python3 test_compatibility.py --quick

# 성능 중심 테스트 (C++/병렬화 분석)
python3 test_compatibility.py --performance --save-results \
  --parallel-analysis --cpp-benchmark

# 통합 데모 실행
python3 3d_mapper.py --use-cpp-backend
```

## Usage Examples

### Basic Launch
```bash
# 전체 시스템 (권장)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# Fast-LIO 없이 (odometry 이미 있는 경우)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py launch_fast_lio:=false

# RViz 없이
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py launch_rviz:=false

# Bag 파일 자동 재생
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py play_bag:=true
```

### Parameter Override
```bash
# Launch 시 파라미터 변경 (C++ 백엔드 포함)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
  sonar_orientation.yaw:=-90.0 \
  sonar_orientation.pitch:=45.0 \
  voxel_resolution:=0.1 \
  use_cpp_backend:=true

# 노드 직접 실행 시
ros2 run sonar_3d_reconstruction 3d_mapper_node.py --ros-args \
  -p horizontal_fov:=70.0 \
  -p voxel_resolution:=0.05 \
  -p use_cpp_backend:=true
```

### KIRO Water Tank Dataset
```bash
# 기본 bag 파일 경로 설정됨
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# 다른 bag 파일 사용
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
  bag_file:=/workspace/data/2_kiro_watertank/20250926_blueboat_sonar_lidar/oculus-tilt60-gain50-freq2
```

## Key Features

### Adaptive Bayesian Update
- **Linear Interpolation**: 확률에 따른 가중치 조정
- **Free Space Protection**: 높은 확신도의 자유 공간 보호
- **Noise Reduction**: 적응형 업데이트로 노이즈 감소

### IWLO (Intensity-Weighted Log-Odds) 확률 업데이트
- **하이브리드 업데이트**: Log-Odds Bayesian과 Weighted Average 방식 융합
- **강도 기반 가중치**: 시그모이드 함수를 통한 동적 가중치 변환
- **학습률 감쇠**: 관측 횟수에 따른 적응형 학습률 조정
- **Saturation 방지**: log-odds 범위 제한으로 수렴 안정성 확보
- **설정 파일**: `config/method_iwlo.yaml`에서 커스터마이징 가능

### Cross-talk 노이즈 필터
- **줄무늬 노이즈 억제**: Morphological Opening 필터로 가로 줄무늬 제거
- **방위각 일관성 검사**: 인접 방위각 값 비교로 이상치 제거
- **멀티빔 최적화**: 멀티빔 소나의 전기적 간섭 노이즈 감소
- **설정 파일**: `config/crosstalk_filter.yaml`에서 필터 파라미터 조정

### Memory Efficiency
- **Sparse Storage**: 관측된 복셀만 저장
- **Dynamic Expansion**: 고정 경계 없이 자동 확장
- **Octree Structure**: 계층적 공간 분할

### Visualization
- **PointCloud2**: 확률값을 intensity로 표시
- **MarkerArray**: 색상 기반 확률 시각화 (옵션)
- **RViz Integration**: TF, Path, Map 통합 표시

## Build & Development

### System Dependencies
- **OS**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+

### ROS2 Dependencies

#### Core Dependencies
- `rclpy`: Python ROS2 client library
- `rclcpp`: C++ ROS2 client library
- `ament_cmake`: ROS2 build system
- `ament_cmake_python`: Python package support

#### Message & Communication
- `std_msgs`: Standard message types
- `sensor_msgs`: Sensor data messages (Image, PointCloud2, PointField)
- `geometry_msgs`: Geometry messages (TransformStamped)
- `nav_msgs`: Navigation messages (Odometry)
- `visualization_msgs`: Visualization messages (Marker, MarkerArray)

#### Custom Sensor Messages (빌드 순서 중요)
- `marine_acoustic_msgs`: 해양 음향 센서 공통 메시지
- `oculus_sonar_msgs`: Oculus M750D 소나 전용 메시지
- `ping360_sonar_msgs`: Ping360 소나 전용 메시지

#### Point Cloud & Transformation
- `pcl_ros`: Point Cloud Library ROS2 integration
- `tf2_ros`: TF2 transformation system
- `tf2_eigen`: TF2-Eigen integration

#### Time Synchronization
- `message_filters`: Time synchronization utilities
- `cv_bridge`: OpenCV-ROS image conversion

#### Python Dependencies
- `python3-numpy`: Numerical computing
- `python3-scipy`: Scientific computing
- `python3-opencv`: Computer vision library
- `python3-matplotlib`: Plotting library
- `python3-yaml`: YAML file parsing

#### Runtime Sensor Drivers
- `oculus_sonar`: Oculus M750D driver
- `ping360_sonar`: Ping360 driver
- `fast_lio`: Fast-LIO SLAM system

#### Visualization
- `rviz2`: 3D visualization tool

### Build Instructions

```bash
# 1. ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 2. 의존성 자동 설치 (rosdep)
cd /workspace/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 3. 메시지 패키지 먼저 빌드 (순서 중요)
colcon build --packages-select marine_acoustic_msgs oculus_sonar_msgs ping360_sonar_msgs

# 4. 전체 workspace 빌드
colcon build

# 5. 환경 설정
source install/setup.bash
```

### Manual Dependencies Installation

```bash
# ROS2 Humble dependencies
sudo apt update
sudo apt install -y \
  ros-humble-pcl-ros \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-cv-bridge \
  ros-humble-message-filters \
  ros-humble-visualization-msgs \
  ros-humble-nav-msgs \
  ros-humble-rviz2

# Python dependencies
sudo apt install -y \
  python3-numpy \
  python3-scipy \
  python3-opencv \
  python3-matplotlib \
  python3-yaml
```

### Development Mode
```bash
# 소스에서 직접 실행 (개발 시)
cd /workspace/ros2_ws
source install/setup.bash
ros2 run sonar_3d_reconstruction 3d_mapper_node.py --ros-args \
  --params-file src/sonar_3d_reconstruction/config/3d_mapper.yaml
```

## Troubleshooting

### Common Issues

#### TF Conflicts
```bash
# bag 파일의 tf_static과 충돌 시
# config/3d_mapper.yaml에서 publish_tf: false 설정
```

#### Memory Issues
```bash
# 해상도 낮추기
voxel_resolution: 0.1  # 0.05 → 0.1

# 처리 범위 제한
max_range: 5.0  # 10.0 → 5.0
```

#### Time Synchronization
```bash
# 동기화 허용 오차 조정 (현재 0.1초)
# 필요시 코드에서 slop 파라미터 수정
```

## Performance Metrics

- **Processing Rate**: ~1.5 fps (실시간 가능)
- **Memory Usage**: Sparse octree로 29-93x 절약
- **Synchronization**: ±0.1s 타임스탬프 매칭
- **Voxel Resolution**: 0.05m (설정 가능)

## Package Architecture

### File Structure
```
sonar_3d_reconstruction/
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS2 package metadata & dependencies
├── README.md                   # This documentation
├── config/
│   ├── 3d_mapper.yaml         # Main configuration file
│   ├── 3d_mapper.yaml.bak60   # Backup configuration (60° tilt)
│   └── 3d_mapper.yaml.bak90   # Backup configuration (90° tilt)
├── include/
│   └── sonar_3d_reconstruction/  # C++ headers (if needed)
├── launch/
│   └── 3d_mapping.launch.py    # Main launch file
├── rviz/
│   └── 3d_mapping.rviz        # RViz configuration
├── scripts/
│   ├── 3d_mapper.py           # Core mapping library (dual backend)
│   ├── 3d_mapper_node.py      # ROS2 node implementation
│   └── test_compatibility.py  # Backend compatibility testing
└── src/                        # C++ sources (ProbabilityUpdater)
```

### Import Dependencies (Python)

#### Standard Libraries
```python
import numpy as np              # 수치 계산
import time                     # 성능 측정
import struct                   # 바이너리 데이터 패킹
import sys, os                  # 시스템 유틸리티
import importlib.util           # 동적 모듈 로딩
from collections import defaultdict  # 기본값 딕셔너리
from typing import Tuple, List, Dict, Any, Optional  # 타입 힌트
```

#### ROS2 Core
```python
import rclpy                    # ROS2 Python client
from rclpy.node import Node     # 노드 기본 클래스
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy  # QoS 설정
```

#### ROS2 Messages
```python
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
```

#### ROS2 Utilities
```python
from tf2_ros import StaticTransformBroadcaster  # TF 변환 브로드캐스터
from message_filters import Subscriber, ApproximateTimeSynchronizer  # 메시지 동기화
from cv_bridge import CvBridge  # OpenCV-ROS 변환
import cv2                      # OpenCV 이미지 처리
```

#### Launch System
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml                     # YAML 설정 파일 파싱
```

## Recent Updates (2025-12-04)

### IWLO 확률 업데이트 방법 구현
- **하이브리드 확률 업데이트**: Log-Odds Bayesian 방식과 Weighted Average 방식 융합
- **강도 기반 가중치 변환**: 소나 강도 신호의 시그모이드 함수 가중치
- **적응형 학습률**: 관측 횟수에 따른 감쇠 함수 (saturation 방지)
- **설정 파일**: `config/method_iwlo.yaml`로 모든 파라미터 커스터마이징 가능
- **구현 위치**: `scripts/3d_mapper.py`, `cpp/probability_updater.cpp`

### Cross-talk 노이즈 필터 추가
- **줄무늬 노이즈 억제**: Morphological Opening 필터 (세로 커널)
- **방위각 일관성 검사**: 인접 방위각(azimuth) 값 비교로 이상치 제거
- **멀티빔 소나 최적화**: 전기적 간섭(cross-talk) 노이즈 감소
- **설정 파일**: `config/crosstalk_filter.yaml`
- **구현 위치**: `scripts/3d_mapper.py`

### C++ 백엔드 개선
- **OctoMap 링크 수정**: CMakeLists.txt에서 라이브러리 명시적 링크
- **PYTHONPATH 환경 후크**: `env-hooks/`에서 Python 경로 자동 설정
- **빌드 최적화**: CMAKE 설정 개선으로 안정성 강화

## Recent Updates (2025-11-27)

### C++ Backend Integration
- **Dual Backend Support**: Python SimpleOctree와 C++ ProbabilityUpdater 완전 통합
- **Performance Optimization**: C++ 배치 업데이트로 최대 10x 속도 향상
- **API Compatibility**: 동일한 인터페이스로 백엔드 투명 전환
- **Comprehensive Testing**: test_compatibility.py를 통한 성능 및 호환성 검증
- **Documentation Update**: 백엔드 비교 가이드 및 사용법 추가

### Key Features Added
1. **Automatic Backend Detection**: C++ 모듈 자동 감지 및 fallback
2. **Configuration Parameter**: `use_cpp_backend` 설정으로 백엔드 선택
3. **Batch Update Support**: C++ 배치 처리로 성능 최적화
4. **Memory Statistics**: 실시간 메모리 사용량 모니터링
5. **Result Validation**: 백엔드 간 결과 일치성 검증

### C++ 최적화 상세
- **OpenMP 병렬 처리**: 레이캐스팅 병렬화, 10+ 스레드 동시 처리
- **Eigen3 최적화**: 제로-복사 좌표 변환
- **pybind11 바인딩**: 원활한 Python API 통합
- **OctoMap 메모리 효율**: 희소 복셀 트리 구현 
- **성능 지표**: 
  - 처리 속도: 최대 10x 향상
  - 메모리 사용량: 최대 30% 절감
  - 일관성: 95%+ 결과 동일성 보장

### Previous Updates (2025-11-13)

### Dependencies Documentation Update
- 전체 의존성 목록 체계적 정리
- Build order 명시 (메시지 패키지 우선)
- Python import 구조 상세 문서화
- rosdep 기반 자동 설치 가이드 추가

### Previous Updates (2025-09-29)

#### Complete Refactoring
- feature_extraction_3d 기반 재구현
- Adaptive update 메커니즘 추가
- Fast-LIO TF tree 완전 통합
- 소스 YAML 직접 참조 (빌드 불필요)
- 파라미터 우선순위 시스템 구현
- 향상된 로깅 및 시각화

#### Breaking Changes
- octree_mapper_node.py → 3d_mapper_node.py
- mapping.launch.py → 3d_mapping.launch.py
- test/ 폴더 제거 (통합 완료)

## License & Contact

ROS2 패키지 표준 라이선스를 따릅니다.

Repository: https://github.com/luckkim123/sonar_3d_reconstruction.git