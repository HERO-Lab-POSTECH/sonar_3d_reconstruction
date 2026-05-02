# 2026-03-28 — QoS Stabilization & Time-Sync Hardening

## Summary

- 워크스페이스 전반의 ROS2 QoS를 BEST_EFFORT로 통일 (맵 토픽 3개만 RELIABLE 유지)
- `3d_mapper_node`에 sonar↔odometry 시간차 임계값 추가 (드리프트 프레임 드롭)
- 검증 환경에서 40분 연속 실행 무결성 확인

## 1. QoS 통일 (Workspace-wide)

### 배경

소나 드라이버는 `BEST_EFFORT`로 publish하는데 일부 subscriber가 `RELIABLE`이라 DDS 매칭 실패 → 메시지 미수신. 노드별 분산된 QoS 설정을 한 정책으로 정렬.

### 정책

- **센서 · SLAM · odometry → BEST_EFFORT + VOLATILE 통일**
- **맵 토픽 3개만 RELIABLE + TRANSIENT_LOCAL 유지** (late-join subscriber 요구사항)

상세 표는 [reference/qos-policy.md](../reference/qos-policy.md) 참조.

### 수정 파일

| 패키지 | 파일 | 변경 |
|---|---|---|
| sonar_3d_reconstruction | `scripts/config.py` (L278) | `qos.reliability` 기본값: `reliable` → `best_effort` |
| sonar_3d_reconstruction | `launch/3d_mapping.launch.py` (L361) | `qos_reliability` 기본값: `reliable` → `best_effort` |
| oculus_ros2 | `src/oculus_driver_component.cpp` (L67-84) | 모든 publisher `rclcpp::SensorDataQoS()` |
| oculus_ros2 | `src/oculus_fan_imager.cpp` (L30) | `qos_reliability` 기본값 변경, fan_image_pub `sensor_data` QoS |
| oculus_ros2 | `launch/{oculus,fan_imager}.launch.py` | `qos_reliability` 기본값 변경 |
| ping1d_ros2 | `ping1d_sonar/ping1d_component.py` (L42-56) | RELIABLE → BEST_EFFORT, 변수명 `reliable_qos` → `sensor_qos` |
| ping360_ros2 | `ping360_sonar/src/ping360_node.cpp` (L97-124) | `#ifdef PING360_PUBLISH_BEST_EFFORT` 제거, `SensorDataQoS()` 고정 |
| fast_lio | `src/slam/laserMapping.cpp` (L926-955) | 모든 sub/pub `SensorDataQoS()` |
| fast_lio | `src/localization/localization_node.cpp` (L242-269) | 데이터 토픽 `SensorDataQoS()`, 맵 토픽은 RELIABLE + TRANSIENT_LOCAL 유지 |
| livox_ros2 | `livox_driver/src/lddc.cpp` (L528-553) | PointCloud2/CustomMsg/Imu publisher 전부 `SensorDataQoS()` |

> `robot_3d_mapping.launch.py`는 이미 `best_effort`가 기본값이라 변경 없음.

### 변경하지 않은 항목

| 패키지 | 사유 |
|---|---|
| `cartographer_slam/launch/slam.launch.py` | sensor sub가 이미 `best_effort` 기본값 |
| `cartographer_slam/src/occupancy_grid_node_main.cpp` | `cartographer_2d/map`은 TRANSIENT_LOCAL + RELIABLE 필수 |
| `sonoptix_ros2/sonoptix_ros2/echo.py` | 이미 BEST_EFFORT |

### 빌드

C++ 패키지 재빌드 필요:

```bash
colcon build --packages-select oculus_sonar ping360_sonar fast_lio livox_driver
```

Python 패키지(`sonar_3d_reconstruction`, `ping1d_ros2`)는 빌드 없이 즉시 적용.

## 2. Time-Sync 하드닝 (sonar_3d_reconstruction)

### 배경

`3d_mapper_node._sonar_callback`이 sonar message 도착 시 **임계값 없이** 가장 최신 odometry와 페어링 → 데이터 지연 시 큰 시차로 매핑 정확도 저하.

### 관측 로그

```
[TimeSync] sonar_stamp=1774680863.873 odom_stamp=1774680864.007 stamp_diff=-0.134s sonar_age=0.185s odom_age=0.051s
[TimeSync] sonar_stamp=1774680870.047 odom_stamp=1774680870.187 stamp_diff=-0.140s sonar_age=0.196s odom_age=0.056s
```

- `stamp_diff` ≈ −0.13~−0.14s (sonar가 odom보다 ~140ms 과거)
- `sonar_age` ≈ 190ms (소나 전송 지연)
- `odom_age` ≈ 50ms (정상)

### 조치

`scripts/3d_mapper_node.py::_sonar_callback`에 stamp_diff 임계값 체크 추가:

```python
MAX_STAMP_DIFF = 0.1  # seconds
if stamp_diff > MAX_STAMP_DIFF:
    # 프레임 드롭 + 경고 로그 (10프레임마다)
    return
```

### 운영 시 주의

현재 환경 stamp_diff(~0.14s)가 임계값(0.1s)보다 커서 다수 프레임이 드롭될 수 있습니다. 환경에 맞춰 0.15s 등으로 상향 조정 가능.

## 3. 검증 환경 (참고)

| 항목 | 값 |
|---|---|
| Local PC ↔ Jetson | USB 이더넷 어댑터 (`enx00e04c36fd07`, 100Mbps Full Duplex) |
| Local PC IP | `192.168.0.22/24` |
| Jetson IP | `192.168.0.13` |
| 무선 (백업) | Wi-Fi `192.168.10.36/24` |
| 연속 실행 | 40분 무중단, regression 없음 |
