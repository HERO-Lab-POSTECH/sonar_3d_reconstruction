# Sonar 3D Reconstruction

Real-time probabilistic 3D underwater terrain mapping and robot detection system using Oculus multibeam sonar and LiDAR-Inertial odometry (Fast-LIO / Cartographer).

## Features

- **IWLO Probabilistic Mapping** - Intensity-Weighted Log-Odds Bayesian 3D map update
- **C++ Backend** - pybind11 + OpenMP parallel processing (~10x faster than pure Python)
- **Out-of-Core Storage** - Disk-based tile mapper with LRU cache for large-scale environments
- **Robot/Object Detection** - Depth estimation against a reference map to detect new underwater objects
- **Cross-talk Filter** - 2D FFT-based notch filter for multibeam sonar stripe noise removal
- **Map Diff Analysis** - Compare two maps to visualize added/removed/changed voxels
- **Multi-Sonar Support** - Oculus M750D (70° FOV) and M3000D (130° FOV)
- **Tilt Presets** - Optimized parameter sets for 30°, 60°, 90° sonar pitch angles
- **Dynamic Parameters** - Runtime tuning of thresholds, filters, and visualization via ROS2 parameter server

## System Architecture

```
Sonar Image (polar)              Odometry (pose)
       │                              │
       ▼                              ▼
  Crosstalk Filter             Transform Matrices
       │                              │
       └──── Synchronize (latest) ────┘
                     │
                     ▼
          Ray Processing (per bearing)
          ├─ First hit detection
          ├─ Free / occupied voxel sampling
          └─ Shadow region protection
                     │
                     ▼
             IWLO Batch Update (C++)
             ├─ In-memory octree   ─── PointCloud2
             └─ Out-of-core tiles  ─── Disk + LRU cache
                     │
            ┌────────┴────────┐
            ▼                 ▼
     Publish Markers    Notify Visualizer
     (MarkerArray)       (tile updates)
```

### Robot Detection Pipeline (V3)

```
Reference Map (pre-built)    Current Sonar Frame
         │                          │
         ▼                          ▼
    Ray-cast depth            Actual sonar depth
         │                          │
         └─── Compare per bearing ──┘
                     │
              depth_diff > threshold?
              ├─ Yes → Write to detection_map (new object)
              └─ No  → Skip (existing environment)
```

## Quick Start

```bash
# Build (message packages first)
colcon build --packages-select marine_acoustic_msgs oculus_sonar_msgs
colcon build --packages-select sonar_3d_reconstruction
source install/setup.bash

# 3D Mapping (basic)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# With bag playback
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
    bag_file:=/path/to/bag use_sim_time:=true

# Out-of-core mapping (disk-based, for large maps)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
    map_path:=/path/to/map_dir

# Robot detection on existing map
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/path/to/existing_map use_sim_time:=true
```

## Launch Files

### `3d_mapping.launch.py` - Main 3D Mapping

| Argument | Default | Description |
|----------|---------|-------------|
| `sonar_model` | `m3000d` | Sonar model: `m750d` (FOV=70°) or `m3000d` (FOV=130°) |
| `odometry` | `cartographer` | Odometry source: `cartographer`, `fast_lio`, `fast_lio_loc` |
| `sonar_pitch` | `90.0` | Sonar pitch angle: `30.0`, `60.0`, `90.0` (loads tilt preset) |
| `map_path` | `""` | Out-of-core map directory (empty = in-memory mode) |
| `bag_file` | `""` | Bag file for playback (empty = no playback) |
| `bag_rate` | `1.0` | Playback speed multiplier |
| `record_path` | `""` | Recording output directory (empty = no recording) |
| `use_sim_time` | `false` | Use simulation time |
| `rviz` | `false` | Launch RViz |
| `foxglove` | `false` | Launch Foxglove bridge (`ws://localhost:8765`) |
| `show_opencv` | `false` | Show OpenCV debug visualization |
| `launch_visualizer` | `true` | Launch tile visualizer node (requires out-of-core) |
| `qos_reliability` | `reliable` | QoS: `reliable` or `best_effort` (use `best_effort` for old bags) |

**Examples:**

```bash
# With Cartographer SLAM
ros2 launch cartographer_slam slam.launch.py
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# With Fast-LIO SLAM
ros2 launch fast_lio mapping.launch.py
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio

# With Fast-LIO Localization
ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio_loc

# BEST_EFFORT QoS for old bag files
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py \
    bag_file:=/path/to/bag use_sim_time:=true qos_reliability:=best_effort
```

### `robot_3d_mapping_v3.launch.py` - Robot Detection

Detects new objects (robots, ROVs) by comparing real-time sonar against a pre-built reference map. Original map is shown in normal Z-axis coloring; detections are shown in RED.

| Argument | Default | Description |
|----------|---------|-------------|
| `map_path` | `""` | **Required.** Path to existing reference map tiles |
| `detection_map_path` | `""` | Detection output path (auto: `{parent}/robot_detection_map`) |
| `sonar_pitch` | `60.0` | Sonar pitch: `30.0`, `60.0`, `90.0` |
| `depth_estimation` | `true` | Enable depth estimation filter |
| `depth_diff_threshold` | `1.0` | Depth difference threshold [m] for new object detection |
| `marker_min_depth` | `0.0` | Min depth for grayscale coloring [m] |
| `marker_max_depth` | `10.0` | Max depth for grayscale coloring [m] |
| `marker_alpha` | `0.8` | Marker transparency (0.0-1.0) |

Inherits all common arguments from the main launch file (`sonar_model`, `odometry`, `bag_file`, etc.).

**Depth estimation sensitivity:**
- `0.5` m - More sensitive (small objects, more false positives)
- `1.0` m - Balanced (default)
- `2.0` m - Less sensitive (only large/obvious objects)

```bash
# Robot detection with bag playback
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/home/hero/data/map/sonar \
    bag_file:=/path/to/bag use_sim_time:=true

# With custom detection map path
ros2 launch sonar_3d_reconstruction robot_3d_mapping_v3.launch.py \
    map_path:=/path/to/map \
    detection_map_path:=/path/to/detection use_sim_time:=true
```

### `robot_3d_mapping_v2.launch.py` - Robot Detection (V2)

Same as V3 but without depth estimation filtering. All sonar observations are written to the detection map.

### `map_visualizer.launch.py` - Standalone Map Viewer

Visualize saved out-of-core maps without running the mapper.

```bash
ros2 launch sonar_3d_reconstruction map_visualizer.launch.py \
    map_path:=/path/to/map rviz:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `map_path` | - | **Required.** Map directory path |
| `publish_rate` | `1.0` | PointCloud publish rate [Hz] |
| `vis_mode` | `0` | 0=pointcloud, 1=octomap, 2=all |

### `map_diff.launch.py` - Map Comparison

Compare two maps to visualize differences (added/removed voxels).

```bash
ros2 launch sonar_3d_reconstruction map_diff.launch.py \
    map_a:=/path/to/prior_map map_b:=/path/to/new_map
```

## Configuration

### Config Files

| File | Description |
|------|-------------|
| `config/common.yaml` | Main parameters (sonar, voxel, topics, visualization) |
| `config/presets/tilt_{30,60,90}.yaml` | Tilt-angle-specific filtering & IWLO for mapping |
| `config/presets/robot_detect_tilt_{30,60,90}.yaml` | Tilt-angle-specific presets for robot detection |
| `config/map_visualizer.yaml` | Standalone visualizer settings |

### Key Parameters (common.yaml)

```yaml
sonar:
  vertical_aperture: 20.0        # [deg] Vertical beam width

crosstalk:                        # 2D FFT stripe removal [Dynamic]
  enabled: true
  filter_width: 0.02
  filter_strength: 0.8

processing:
  frame_skip: 5                   # Process every N frames [Dynamic]

mapping:
  occupied_threshold: 0.7         # Probability threshold [Dynamic]
  angular_cone_width: 0.5         # 0.5=strict, 1.0=permissive [Dynamic]

octree:
  voxel_resolution: 0.2           # [m] [Static]
  use_cpp_backend: true           # [Static]

outofcore:
  tile_size: 0.4                  # [m] [Static]
  cache_size: 32                  # Max tiles in memory [Static]
```

### Tilt Presets

| Parameter | 90° (Downward) | 60° | 30° |
|-----------|---------------|-----|-----|
| `intensity_threshold` | 70 | Moderate | High sensitivity |
| `min_range` | 2.0 m | Medium | Longer |
| `L_occ` | 7.0 | Moderate | Conservative |
| `L_free` | -3.0 | Moderate | Conservative |

Robot detection presets use aggressive occupied updates (`L_occ: 15.0`) and conservative free updates (`L_free: -1.0`) to make new objects appear immediately while protecting existing map features.

## Nodes

| Node | Executable | Description |
|------|------------|-------------|
| `sonar_3d_mapper` | `3d_mapper_node.py` | Main 3D mapping node |
| `map_visualizer` | `map_visualizer_node.py` | Out-of-core tile visualizer |
| `robot_detection` | `3d_mapper_node.py` | Robot detection (same binary, different params) |
| `map_diff_visualizer` | `map_diff_visualizer.py` | Map comparison visualizer |

## ROS2 Topics

### Input

| Topic | Type | Description |
|-------|------|-------------|
| `/sensor/sonar/oculus/{m750d,m3000d}/image` | `sensor_msgs/Image` | Sonar polar image |
| `{cartographer_2d,/fast_lio}/odometry` | `nav_msgs/Odometry` | Robot odometry |

### Output (3D Mapping)

| Topic | Type | Description |
|-------|------|-------------|
| `/sonar_3d_mapper/point_cloud` | `sensor_msgs/PointCloud2` | Occupied voxels as 3D point cloud |
| `/sonar_3d_mapper/occupancy_grid` | `visualization_msgs/MarkerArray` | RViz cube markers |
| `/sonar_3d_mapper/filtered_image` | `sensor_msgs/Image` | Crosstalk-filtered sonar image |
| `/sonar_3d_mapper/updated_tile_indices` | `std_msgs/Int32MultiArray` | Updated tile notifications (out-of-core) |

### Output (Robot Detection)

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_detection/point_cloud` | `sensor_msgs/PointCloud2` | Detection point cloud |
| `/robot_detection/occupancy_grid` | `visualization_msgs/MarkerArray` | Detection markers (RED) |
| `/map_visualizer/octomap` | `octomap_msgs/Octomap` | Original map OctoMap visualization |

## TF Tree

```
map
└── odom
    └── base_link
        ├── livox_frame (LiDAR)
        ├── imu_link
        └── sonar_link (Oculus sonar)
```

Sonar mounting orientation is configured via the `sonar_pitch` launch argument. Position offset is managed by the URDF (`boat_description` package).

## Algorithms

### IWLO (Intensity-Weighted Log-Odds)

Combines probabilistic Bayesian updates with sonar intensity:

```
Log-odds:  L_new = L_old + α * ΔL
Weight:    α = sigmoid(sharpness * (intensity_norm - threshold)) * (1 - decay)
```

- Strong sonar returns → higher learning rate → faster convergence
- Weak returns → lower learning rate → conservative updates
- Saturation bounds `[L_min, L_max]` prevent arithmetic overflow

### Shadow Region Protection

Prevents double-counting voxels behind obstacles by checking if free-space rays pass through occupied voxels of neighboring bearings. Uses binary search for efficient neighbor lookup.

### Crosstalk Filter

Removes horizontal stripe artifacts from mechanical coupling noise via 2D FFT notch filtering along the bearing frequency axis, with Gaussian rolloff and DC preservation.

## Storage Backends

### In-Memory (default)

- Sparse octree in RAM (OctoMap-compatible)
- Fast random access, real-time pointcloud extraction
- Suitable for small-medium maps

### Out-of-Core (disk-based)

- Disk-based tiles with LRU in-memory cache
- Unlimited map scalability
- Lazy writing: tiles persisted on LRU eviction or periodic flush
- `metadata.json` stores resolution and tile_size for reproducibility

Enable by providing `map_path` launch argument.

## File Structure

```
sonar_3d_reconstruction/
├── config/
│   ├── common.yaml                        # Main parameters
│   ├── map_visualizer.yaml                # Visualizer settings
│   └── presets/
│       ├── tilt_{30,60,90}.yaml           # Mapping tilt presets
│       └── robot_detect_tilt_{30,60,90}.yaml  # Detection tilt presets
├── launch/
│   ├── 3d_mapping.launch.py              # Main mapping launch
│   ├── robot_3d_mapping_v2.launch.py     # Robot detection V2
│   ├── robot_3d_mapping_v3.launch.py     # Robot detection V3 (depth estimation)
│   ├── map_visualizer.launch.py          # Standalone map viewer
│   └── map_diff.launch.py               # Map comparison
├── rviz/
│   ├── 3d_mapping.rviz
│   ├── robot_detection_v2.rviz
│   └── map_diff.rviz
├── scripts/
│   ├── 3d_mapper_node.py                 # Main ROS2 node
│   ├── 3d_mapper.py                      # Core mapping library
│   ├── config.py                         # Parameter management
│   ├── crosstalk_filter.py               # 2D FFT noise filter
│   ├── map_visualizer_node.py            # Tile visualizer node
│   ├── map_diff_analyzer.py              # Map comparison logic
│   └── map_diff_visualizer.py            # Map diff visualization node
└── sonar_3d_reconstruction/
    └── cpp/                              # C++ backend (pybind11)
        ├── python_bindings.cpp
        ├── probability_updater.{h,cpp}   # In-memory octree mapper
        ├── outofcore_tile_mapper.{h,cpp} # Disk-based tile mapper
        ├── iwlo_updater.{h,cpp}          # IWLO algorithm
        ├── tile.{h,cpp}                  # Tile data structure
        ├── tile_manager.{h,cpp}          # Tile lifecycle
        └── lru_cache.h                   # LRU eviction cache
```

## Build Dependencies

```bash
# System
sudo apt install liboctomap-dev libomp-dev

# ROS2 packages
sudo apt install ros-humble-pcl-ros ros-humble-tf2-ros ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-visualization-msgs ros-humble-foxglove-bridge

# Build order (messages first)
colcon build --packages-select marine_acoustic_msgs oculus_sonar_msgs ping360_sonar_msgs
colcon build --packages-select sonar_3d_reconstruction
```

### Runtime Dependencies

- **SLAM**: `fast_lio` or `cartographer_slam`
- **Sonar driver**: `oculus_sonar` (M750D/M3000D)
- **Robot model**: `boat_description` (URDF with sonar_link TF)

## Coordinate System

```
Sonar frame (NED-style):  +X forward, +Y right (starboard), +Z down
Map frame (ENU):          +X forward, +Y left, +Z up
```

Sonar-to-world transform is computed from odometry and the sonar mounting configuration at each frame.
