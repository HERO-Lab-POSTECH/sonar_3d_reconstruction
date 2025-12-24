# 3d_mapper_node.py

ROS2 node wrapper for the 3D sonar mapping library.

**Location**: `scripts/3d_mapper_node.py`

---

## Overview

Subscribes to sonar images and odometry, processes them through `SonarTo3DMapper`, and publishes the 3D point cloud. Supports in-memory and out-of-core modes.

**Key Features**:
- Time-synchronized sonar + odometry (100ms tolerance)
- Dynamic parameter updates at runtime
- Static TF broadcasting for sonar mounting
- Optional: robot detection, crosstalk filter

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Input Topics                                                       │
│  ├── /sensor/sonar/.../image        [Image]                         │
│  ├── /fast_lio/odometry             [Odometry]                      │
│  └── /sensor/sonar/.../param/range  [Float32]                       │
└────────────────────────────┬────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────────┐
│  SonarMapperNode                                                    │
│  ├── ApproximateTimeSynchronizer (100ms slop)                       │
│  ├── synchronized_callback() → mapper.process_sonar_image()        │
│  └── publish_pointcloud() or periodic_flush_and_notify()           │
└────────────────────────────┬────────────────────────────────────────┘
                             ↓
┌─────────────────────────────────────────────────────────────────────┐
│  Output Topics                                                      │
│  In-Memory:    /sonar_3d_map [PointCloud2]                          │
│                /sonar_3d_map_markers [MarkerArray]                  │
│  Out-of-Core:  /updated_tile_indices [Int32MultiArray]              │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Parameters

All parameters are documented in [Configuration Reference](config.md).

---

## Dynamic Parameters

The node supports runtime parameter updates via `ros2 param set`.

### Registration

```python
# Line 252-253
self.add_on_set_parameters_callback(self.parameter_callback)
```

### Supported Dynamic Parameters

| Category | Parameter | Effect |
|----------|-----------|--------|
| **Filtering** | `filtering.intensity_threshold` | Updates mapper + C++ backend |
| | `filtering.min_range` | Updates mapper min range |
| **Mapping** | `mapping.occupied_threshold` | Changes visualization threshold |
| | `mapping.angular_cone_width` | Updates cone width |
| **Processing** | `processing.frame_skip` | Changes frame skip count |
| **Visualization** | `visualization.show_opencv_visualization` | Toggle OpenCV window |
| | `visualization.pointcloud_publish_rate` | Change publish rate |
| | `visualization.tile_save_interval` | Change flush interval |
| **Octree** | `octree.dynamic_expansion` | Toggle dynamic expansion |
| **Mounting** | `mounting.orientation.roll/pitch/yaw` | Re-broadcasts TF |

### Read-only Parameters (requires restart)

- `sonar.*` - Hardware specs
- `octree.voxel_resolution` - Cannot change after init
- `iwlo.*` - Algorithm params (set at startup)
- `adaptive.*` - Adaptive scaling params
- `outofcore.*` - Tile settings

### Example Usage

```bash
# Change intensity threshold at runtime
ros2 param set /sonar_3d_mapper filtering.intensity_threshold 60

# Update sonar mounting orientation
ros2 param set /sonar_3d_mapper mounting.orientation.pitch 85.0

# Toggle visualization
ros2 param set /sonar_3d_mapper visualization.show_opencv_visualization true
```

---

## Topics

### Input

| Topic | Type | Description |
|-------|------|-------------|
| `topics.sonar` | sensor_msgs/Image | Sonar polar image |
| `topics.odometry` | nav_msgs/Odometry | Robot pose |
| `{sonar}/param/range` | std_msgs/Float32 | Dynamic max range |

### Output (In-Memory Mode)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `topics.pointcloud` | PointCloud2 | 10 Hz | Occupied voxels (x, y, z, probability) |
| `topics.marker` | MarkerArray | 10 Hz | Visualization markers |
| `robot_detection.topic` | PointCloud2 | 10 Hz | Robot detections (optional) |

### Output (Out-of-Core Mode)

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/updated_tile_indices` | Int32MultiArray | 5s + eviction | Saved tile indices [x1,y1,z1,...] |

---

## Processing Flow

```
# synchronized_callback() [Line 515-576]

1. Frame skip check
2. Wait for max_range from /param/range
3. Decode image (mono8/mono16 -> uint8)
4. Extract pose from odometry
5. mapper.process_sonar_image()
6. Optional: OpenCV visualization
7. Out-of-core: notify saved tiles
```

**Dynamic Range Update**:
```
/sensor/sonar/.../image → /sensor/sonar/.../param/range (auto-generated)
```

---

## TF Broadcasting

Static transform from `base_link` to `sonar_link`.

- Published on startup
- Updated when `mounting.orientation.*` changes dynamically
- Uses `StaticTransformBroadcaster`

---

## Image Encoding

| Encoding | Handling |
|----------|----------|
| `mono8` / `8UC1` | Direct use |
| `mono16` / `16UC1` | Downsampled to 8-bit |

---

## Mode Comparison

| Aspect | In-Memory | Out-of-Core |
|--------|-----------|-------------|
| Parameter | `outofcore.use=False` | `outofcore.use=True` |
| Output | `/sonar_3d_map` | `/updated_tile_indices` |
| Publishing | Timer (10 Hz) | Event-driven (5s flush) |
| Visualizer | Built-in | `map_visualizer_node` |
| Memory | Grows with map | Fixed cache |

---

## Usage

```bash
# Default
ros2 run sonar_3d_reconstruction 3d_mapper_node

# With config file
ros2 run sonar_3d_reconstruction 3d_mapper_node \
    --ros-args --params-file config/mapper_params.yaml

# Dynamic update
ros2 param set /sonar_3d_mapper filtering.intensity_threshold 60
```

---

## Related

- [3d_mapper.py](3d_mapper.md) - Core mapping library
- [Configuration Reference](config.md) - All parameters
- [Utilities](utilities.md) - Map Visualizer Node for Out-of-Core mode
