# Deployment Runbook

실 환경(boat + Jetson + local PC) 통합 시스템 기동 절차. `ros2_ws` 빌드·소싱이 모든 노드에서 완료되어 있다고 가정합니다.

## 사전 조건

| 항목 | 요구사항 |
|---|---|
| Boat (Jetson) | ROS2 Humble, 워크스페이스 빌드 완료, Oculus 소나 전원·이더넷 연결 |
| Local PC | ROS2 Humble, 워크스페이스 빌드 완료, Joy 컨트롤러 연결 |
| 네트워크 | Jetson과 Local PC가 같은 LAN(`192.168.0.0/24`) 위에 있어야 DDS discovery 가능 |
| 맵 저장 경로 | `MAP_ROOT=/home/hero/Data/<YYYYMMDD>/map` (사전 생성) |

## 실행 순서

### Phase 1 — Boat 측 (SSH 세션)

```bash
ssh maincon
sonartilt
source install/setup.bash
ros2 launch oculus_sonar oculus.launch.py sonar_model:=m3000d
```

### Phase 2 — Local PC, Terminal 1 (입력 + Lidar)

```bash
ros2 run joy joy_node                            # 조이패드 입력
rqt                                              # 파라미터 모니터
ros2 launch livox_driver msg_MID360_launch.py    # Livox MID360 드라이버
```

### Phase 3 — Local PC, Terminal 2 (SLAM + 매핑)

세 명령은 **순서대로** 실행 (각 노드가 안정화된 후 다음 명령).

```bash
export MAP_ROOT=/home/hero/Data/$(date +%Y%m%d)/map
mkdir -p "$MAP_ROOT/cartographer" "$MAP_ROOT/sonar" "$MAP_ROOT/robot_detection"

# (1) Cartographer 2D SLAM
ros2 launch cartographer_slam slam.launch.py \
    localization:=false \
    save_state_filename:=$MAP_ROOT/cartographer/state.pbstream

# (2) Sonar 3D 매핑
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
    sonar_model:=m3000d \
    map_path:=$MAP_ROOT/sonar \
    sonar_pitch:=90.0

# (3) Robot detection
ros2 launch sonar_3d_reconstruction robot_3d_mapping.launch.py \
    map_path:=$MAP_ROOT/sonar/ \
    detection_map_path:=$MAP_ROOT/robot_detection \
    sonar_model:=m3000d \
    sonar_pitch:=90.0 \
    show_opencv:=true
```

## 종료 절차

역순으로 Ctrl+C: robot detection → sonar mapping → cartographer → livox → oculus.
종료 후 `$MAP_ROOT` 전체를 외장 스토리지로 백업 권장 (out-of-core 타일 데이터 + Cartographer pbstream).

## 트러블슈팅

상세 진단·복구 절차는 [troubleshooting.md](../troubleshooting.md) 및 최신 release-note를 참조하세요.
