# Utilities

Supporting components for the sonar 3D mapping system.

---

## Map Visualizer Node

**File**: `scripts/map_visualizer_node.py`

Reads tile-based map from disk and publishes for RViz visualization. Used with Out-of-Core mode.

### Visualization Modes

| Mode | Value | Output Topic | Description |
|------|-------|--------------|-------------|
| PointCloud | 0 | `/map_pointcloud` | PointCloud2 with threshold filtering |
| OctoMap | 1 | `/map_octomap` | OctoMap message for RViz plugin |
| All | 2 | Both | Publish both formats |

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `outofcore.map_path` | `/workspace/data/map_tiles` | Tile storage path |
| `octree.voxel_resolution` | 0.1 | Voxel size (m) |
| `outofcore.tile_size` | 10.0 | Tile size (m) |
| `visualization.mode` | 0 | 0=pointcloud, 1=octomap, 2=all |
| `mapping.occupied_threshold` | 0.7 | Probability threshold (dynamic) |
| `publish_rate` | 1.0 | Publishing rate (Hz) |
| `auto_refresh` | True | Reload tiles periodically |
| `refresh_interval` | 10.0 | Auto-refresh interval (s) |

### Topics

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/updated_tile_indices` | Int32MultiArray | Subscribe | Tile update notifications |
| `/map_pointcloud` | PointCloud2 | Publish | Point cloud visualization |
| `/map_octomap` | Octomap | Publish | OctoMap visualization |

### Tile Reload Strategy

1. **Selective reload**: When `/updated_tile_indices` received, reload only those tiles
2. **Fallback**: If no topic updates, full reload every `refresh_interval` seconds

### Usage

```bash
ros2 run sonar_3d_reconstruction map_visualizer_node \
    --ros-args -p outofcore.map_path:=/path/to/tiles
```

---

## C++ Python Bindings

**File**: `sonar_3d_reconstruction/cpp/python_bindings.cpp`

Pybind11 bindings exposing C++ classes to Python.

### Exposed Classes

**ProbabilityUpdater** (RAM-based)
```python
from sonar_3d_reconstruction_cpp import ProbabilityUpdater

updater = ProbabilityUpdater(resolution=0.05)
updater.set_iwlo_params(sharpness, decay_rate, min_alpha, L_min, L_max)
updater.batch_update_iwlo(points, intensities, is_occupied)
voxels = updater.get_occupied_voxels(min_probability=0.5)
```

**OutofcoreTileMapper** (Disk-based)
```python
from sonar_3d_reconstruction_cpp import OutofcoreTileMapper, TileIndex

mapper = OutofcoreTileMapper(
    map_path="/path/to/tiles",
    resolution=0.05,
    tile_size=10.0,
    cache_size=16
)
mapper.batch_update_iwlo(points, intensities, is_occupied)
mapper.flush_all()  # Write to disk
voxels = mapper.get_all_occupied_voxels(min_probability=0.5)
```

**OctreeMapper** (Low-level OctoMap wrapper)
```python
from sonar_3d_reconstruction_cpp import OctreeMapper

octree = OctreeMapper(resolution=0.05)
octree.batch_update(points, occupied_flags)
octree.save_to_file("map.bt")
```

**RayCasting** (Ray utilities)
```python
from sonar_3d_reconstruction_cpp import RayCasting

ray = RayCasting()
points = ray.generate_ray_points(origin, endpoint, resolution)
in_fov = ray.is_point_in_sonar_fov(origin, direction, point, h_fov, v_aperture, max_range)
```

### Helper Structures

```python
from sonar_3d_reconstruction_cpp import MemoryStats, TileIndex

# MemoryStats
stats = mapper.get_memory_usage()
print(f"{stats.num_nodes} nodes, {stats.memory_mb} MB")

# TileIndex
idx = TileIndex(x=0, y=1, z=0)
print(idx.to_string())  # "0_1_0"
```

### Module Info

```python
import sonar_3d_reconstruction_cpp as cpp

print(cpp.__version__)      # "1.0.0"
print(cpp.get_build_info()) # "Built with: Eigen3 OctoMap pybind11"
```

---

## Related

- [3d_mapper.py](3d_mapper.md) - Uses C++ backend
- [3d_mapper_node.py](3d_mapper_node.md) - Mode comparison
- [Configuration Reference](config.md) - Out-of-Core parameters
