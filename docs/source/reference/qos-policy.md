# QoS Policy Reference

워크스페이스 전체에서 사용하는 ROS2 QoS 표준. 2026-03-28 stabilization 기준.

## 원칙

1. **센서 데이터 · SLAM · odometry → BEST_EFFORT + VOLATILE 통일**
   실시간 스트림은 손실보다 지연이 더 큰 비용이므로 BEST_EFFORT.
2. **맵 토픽 3개만 RELIABLE + TRANSIENT_LOCAL**
   late-join subscriber가 마지막 publish된 맵을 받아야 하므로 DDS 규칙상 필수.

> 🟢 BEST_EFFORT &nbsp;&nbsp; 🟣 RELIABLE + TRANSIENT_LOCAL

## 센서 Publisher

| Topic | Package | Type | Reliability | Durability | Depth |
|---|---|---|---|---|---|
| `/sensor/sonar/oculus/*/sonar` | oculus_ros2 | SonarImage | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/oculus/*/metadata` | oculus_ros2 | OculusMetadata | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/oculus/*/raw_data` | oculus_ros2 | RawData | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/oculus/*/image` | oculus_ros2 | Image | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/oculus/*/param/*` | oculus_ros2 | Int32/Float32/String | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/oculus/*/fan_image` | oculus_ros2 (fan_imager) | Image | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/ping1d/*` | ping1d_ros2 | Range/Float32 | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/sensor/sonar/ping360/*` | ping360_ros2 | Image/SonarEcho/LaserScan | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/ping360/param/*` | ping360_ros2 | Int32/Float32 | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/sonar/sonoptix/*` | sonoptix_ros2 | Image/Int32/String/Bool | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/lidar/livox_mid360/points` | livox_ros2 | PointCloud2/CustomMsg | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/sensor/lidar/livox_mid360/imu` | livox_ros2 | Imu | 🟢 BEST_EFFORT | VOLATILE | 5 |

## SLAM · Odometry Publisher

| Topic | Package | Type | Reliability | Durability | Depth |
|---|---|---|---|---|---|
| `/fast_lio/odometry` | fast_lio | Odometry | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/fast_lio/cloud_registered_body` | fast_lio | PointCloud2 | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/fast_lio/debug/*` | fast_lio | PointCloud2/Path | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/fast_lio/localization/odometry` | fast_lio | Odometry | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/fast_lio/localization/confidence` | fast_lio | Float32 | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `cartographer_2d/odometry` | cartographer | Odometry | 🟢 BEST_EFFORT | VOLATILE | — |

## Map Publisher (RELIABLE + TRANSIENT_LOCAL)

| Topic | Package | Type | Reliability | Durability | Depth |
|---|---|---|---|---|---|
| `/fast_lio/localization/map` | fast_lio | PointCloud2 | 🟣 RELIABLE | TRANSIENT_LOCAL | 1 |
| `/fast_lio/localization/occupancy_grid` | fast_lio | OccupancyGrid | 🟣 RELIABLE | TRANSIENT_LOCAL | 1 |
| `cartographer_2d/map` | cartographer | OccupancyGrid | 🟣 RELIABLE | TRANSIENT_LOCAL | 10 |

## Sonar 3D Reconstruction (Pub/Sub 종합)

| Topic | Direction | Type | Reliability | Durability | Depth |
|---|---|---|---|---|---|
| `/sensor/sonar/oculus/*/image` | Sub | Image | 🟢 BEST_EFFORT | VOLATILE | 10 |
| Odometry topic | Sub | Odometry | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/param/range` | Sub | Float32 | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/sonar_3d_mapper/point_cloud` | Pub | PointCloud2 | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/sonar_3d_mapper/occupancy_grid` | Pub | MarkerArray | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/sonar_3d_mapper/filtered_image` | Pub | Image | 🟢 BEST_EFFORT | VOLATILE | 10 |
| `/sonar_3d_mapper/updated_tile_indices` | Pub | Int32MultiArray | 🟢 BEST_EFFORT | VOLATILE | 10 |

## SLAM Subscribers

| Topic | Package | Reliability | Durability | Depth |
|---|---|---|---|---|
| livox_points | fast_lio | 🟢 BEST_EFFORT | VOLATILE | 5 |
| imu | fast_lio | 🟢 BEST_EFFORT | VOLATILE | 5 |
| livox_points | cartographer | 🟢 BEST_EFFORT | VOLATILE | — |
| imu | cartographer | 🟢 BEST_EFFORT | VOLATILE | — |
| `/fast_lio/odometry` | fast_lio (localization) | 🟢 BEST_EFFORT | VOLATILE | 5 |
| `/fast_lio/cloud_registered_body` | fast_lio (localization) | 🟢 BEST_EFFORT | VOLATILE | 5 |

## 데이터 흐름

```
┌──────────────────────────────────────────────────────────────────┐
│  🟢 BEST_EFFORT (전체 시스템)                                     │
│                                                                  │
│  [Oculus]      ─BE─▶ [Fan Imager] ─BE─▶ [3D Mapper] ─BE─▶ [RViz]│
│  [Ping1D]      ─BE─▶ [3D Mapper]                                 │
│  [Ping360]     ─BE─▶ [3D Mapper]                                 │
│  [Sonoptix]    ─BE─▶ [3D Mapper]                                 │
│                                                                  │
│  [Livox MID360] ─BE─▶ [FAST-LIO] ─BE─▶ [Localization]            │
│                  ─BE─▶ [Cartographer]                            │
│                                                                  │
├──────────────────────────────────────────────────────────────────┤
│  🟣 RELIABLE + TRANSIENT_LOCAL (맵 3개만)                         │
│                                                                  │
│  [Localization] ─RE+TL─▶ /fast_lio/localization/map              │
│  [Localization] ─RE+TL─▶ /fast_lio/localization/occupancy_grid   │
│  [Cartographer] ─RE+TL─▶ cartographer_2d/map                     │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## 변경 이력

- 2026-03-28: 워크스페이스 전반 BEST_EFFORT 통일 — 상세 [release-notes/2026-03-28-qos-stabilization.md](../release-notes/2026-03-28-qos-stabilization.md)
