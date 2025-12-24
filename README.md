# Sonar 3D Reconstruction

Real-time probabilistic 3D underwater terrain mapping system using Oculus multibeam sonar and Fast-LIO odometry.

**Documentation**: https://luckkim123.github.io/sonar_3d_reconstruction/

## Features

- **IWLO Probability Update**: Intensity-Weighted Log-Odds Bayesian mapping
- **C++ Backend**: OpenMP parallel processing for 10x speed improvement
- **Out-of-Core Storage**: Disk-based tile storage for large-scale mapping
- **Cross-talk Filter**: Multibeam sonar noise reduction

## Quick Start

```bash
# Build
cd /workspace/ros2_ws
colcon build --packages-select sonar_3d_reconstruction
source install/setup.bash

# Run (with Fast-LIO + RViz + Bag playback)
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

# Custom bag file
ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py bag_file:=/path/to/bag
```

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `launch_fast_lio` | true | Run Fast-LIO SLAM |
| `launch_rviz` | true | Run RViz visualization |
| `play_bag` | true | Play bag file |
| `bag_file` | (see yaml) | Bag file path |
| `bag_rate` | 1.0 | Playback speed |
| `sonar_pitch` | 90.0 | Sonar pitch angle (degrees) |

## Configuration

### Config Files

| File | Description |
|------|-------------|
| `config/common.yaml` | Main parameters (sonar, voxel, topics) |
| `config/method_iwlo.yaml` | IWLO probability update parameters |
| `config/crosstalk_filter.yaml` | Cross-talk noise filter settings |
| `config/robot_detection.yaml` | ROV detection parameters |
| `config/map_visualizer.yaml` | Map visualization settings |

### Key Parameters (common.yaml)

```yaml
# Sonar Sensor
horizontal_fov: 130.0        # degrees
vertical_aperture: 20.0      # degrees
max_range: 3.5               # meters
min_range: 1.0               # meters
intensity_threshold: 120     # 0-255

# Sonar Mounting
sonar_orientation:
  pitch: 90.0                # 90 = downward facing

# Voxel Map
voxel_resolution: 0.1        # meters
use_cpp_backend: true        # C++ optimization

# Out-of-Core Storage
use_outofcore: true
outofcore_map_path: "/workspace/map_tiles"
outofcore_tile_size: 0.4     # meters

# Processing
frame_skip: 5                # Process every N frames
occupied_threshold: 0.7      # Probability threshold
```

## Topics

### Subscribe

| Topic | Type | Description |
|-------|------|-------------|
| `/sensor/sonar/oculus/m3000d/image` | sensor_msgs/Image | Sonar polar image |
| `/fast_lio/odometry` | nav_msgs/Odometry | Robot odometry |
| `/sensor/sonar/oculus/param/range` | std_msgs/Float32 | Dynamic sonar range |

### Publish

| Topic | Type | Description |
|-------|------|-------------|
| `/sonar_3d_map` | sensor_msgs/PointCloud2 | 3D point cloud map |
| `/sonar_3d_map_markers` | visualization_msgs/MarkerArray | RViz markers |
| `/map_pointcloud` | sensor_msgs/PointCloud2 | Tile visualization (out-of-core) |

## Nodes

| Node | Script | Description |
|------|--------|-------------|
| `sonar_3d_mapper` | `3d_mapper_node.py` | Main 3D mapping node |
| `map_visualizer` | `map_visualizer_node.py` | Out-of-core tile visualizer |
| `world_init_broadcaster` | `world_init_broadcaster_node.py` | Gravity-aligned TF broadcaster |

## Build

### Dependencies

```bash
# ROS2 packages
sudo apt install ros-humble-pcl-ros ros-humble-tf2-ros ros-humble-cv-bridge \
  ros-humble-message-filters ros-humble-visualization-msgs

# Build order (messages first)
colcon build --packages-select marine_acoustic_msgs oculus_sonar_msgs
colcon build --packages-select sonar_3d_reconstruction
```

### Runtime Dependencies

- `fast_lio`: LiDAR-Inertial SLAM
- `oculus_sonar`: Oculus M750D/M3000D driver

## File Structure

```
sonar_3d_reconstruction/
├── config/                 # YAML configuration files
├── launch/                 # Launch files
│   └── 3d_mapping.launch.py
├── rviz/                   # RViz configurations
├── scripts/                # Python nodes and libraries
│   ├── 3d_mapper_node.py   # Main ROS2 node
│   ├── 3d_mapper.py        # Mapping library
│   └── config.py           # Configuration dataclass
└── sonar_3d_reconstruction/
    └── cpp/                # C++ backend (pybind11)
```

## Coordinate System

```
camera_init (Fast-LIO map frame)
└── body (robot frame)
    └── sonar_link (sonar frame)

Sonar: +X forward, +Y right, +Z down
Map:   +X forward, +Y left, +Z up
```
