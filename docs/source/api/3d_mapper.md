# 3d_mapper.py

Core library for converting sonar images to 3D point clouds with probabilistic mapping.

**Location**: `scripts/3d_mapper.py`

---

## Overview

Processes Oculus multibeam sonar images and generates 3D terrain maps using IWLO (Intensity-Weighted Log-Odds) Bayesian probability updates. C++ backend with OpenMP provides ~10x speedup.

**Key Features**:
- IWLO probability update with intensity-based confidence weighting
- Cross-talk filter for multibeam sonar noise removal
- Shadow region protection to preserve unknown states
- Out-of-Core storage for large-scale mapping

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  ROS2 Node: 3d_mapper_node.py                                       │
│  └── synchronized_callback() → process_sonar_image() → publish()   │
└──────────────────────────┬──────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────────┐
│  Library: 3d_mapper.py :: SonarTo3DMapper                           │
│  ├── process_sonar_image()    [Line 688-860]                        │
│  │   ├── CrosstalkFilter      (optional)                            │
│  │   ├── Phase 1: First hit collection per beam                     │
│  │   ├── Phase 2: Ray processing with shadow check                  │
│  │   └── batch_update_iwlo()  → C++ backend                         │
│  └── get_point_cloud()        [Line 862-898]                        │
└──────────────────────────┬──────────────────────────────────────────┘
                           ↓
┌─────────────────────────────────────────────────────────────────────┐
│  C++ Backend (pybind11)                                             │
│  ├── ProbabilityUpdater      (RAM-based)                            │
│  └── OutofcoreTileMapper     (Disk-based)                           │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Classes

### SonarTo3DMapper

Main class for sonar-to-3D conversion and probabilistic map maintenance.

```python
mapper = SonarTo3DMapper(config)
```

See [Configuration Reference](config.md) for all parameters.

### CrosstalkFilter

Removes cross-talk noise (horizontal stripes) from multibeam sonar images.

```python
CrosstalkFilter(
    kernel_size=5,
    kernel_shape="rect",        # "rect", "ellipse", "cross"
    consistency_threshold=0.5,
    morpho_enabled=True,
    azimuth_check_enabled=True
)
```

---

## Core Methods

### process_sonar_image

```python
def process_sonar_image(
    polar_image: np.ndarray,       # (range_bins, bearing_bins)
    robot_position: List[float],   # [x, y, z]
    robot_orientation: List[float] # [qx, qy, qz, qw]
) -> Dict[str, Any]
```

**Processing Steps**:
1. Apply cross-talk filter (if enabled)
2. **Phase 1**: Collect first hit per beam (for shadow detection)
3. **Phase 2**: Process each ray → generate voxel updates
4. Aggregate updates (average log-odds per voxel)
5. Call C++ `batch_update_iwlo()` for probability update

**Returns**: `{'frame_count', 'num_voxels', 'processing_time', ...}`

### get_point_cloud

```python
def get_point_cloud(include_free: bool = False) -> Dict[str, Any]
```

Extracts occupied voxels as a point cloud.

| Mode | Behavior |
|------|----------|
| In-Memory | Returns points for `/sonar_3d_map` |
| Out-of-Core | Skipped; use `map_visualizer_node` |

**Returns**: `{'points': (N,3), 'probabilities': (N,), 'num_occupied', ...}`

### Utility Methods

| Method | Description |
|--------|-------------|
| `reset_map()` | Clear all voxels |
| `get_memory_stats()` | Memory usage statistics |
| `flush_map()` | Write to disk (Out-of-Core) |
| `save_merged_octree(path)` | Export as .bt file |

---

## IWLO Algorithm

**Intensity-Weighted Log-Odds** uses sonar reflection intensity as a confidence weight.

```
Occupied:  ΔL = L_occ × w(I) × α(n)
Free:      ΔL = L_free × α(n)

w(I) = sigmoid(sharpness × (normalized_intensity - 0.5))
α(n) = max(min_alpha, 1 / (1 + decay_rate × n))
```

See [IWLO Design](../design/iwlo_design.md) for detailed algorithm explanation.

---

## Coordinate System

```
Sonar Frame              World Frame (Map)
    +X (forward)             +X (forward)
      ↑                        ↑
      │                        │
+Y ←──┼                 ───────┼───→ +Y
      │                        │
      ↓ +Z (down)              ↓ +Z (up)
```

**Transform**: `T_sonar_to_world = T_base_to_world @ T_sonar_to_base`

---

## Usage Example

```python
from scripts.mapper_3d import SonarTo3DMapper

config = {
    'voxel_resolution': 0.1,
    'intensity_threshold': 120,
    'use_outofcore': False
}

mapper = SonarTo3DMapper(config)

# Process frame
stats = mapper.process_sonar_image(polar_image, position, orientation)

# Get point cloud
cloud = mapper.get_point_cloud()
points = cloud['points']        # (N, 3)
probs = cloud['probabilities']  # (N,)
```

---

## Related

- [Configuration Reference](config.md) - All parameters
- [3d_mapper_node.py](3d_mapper_node.md) - ROS2 node wrapper
- [IWLO Design](../design/iwlo_design.md) - Algorithm details
