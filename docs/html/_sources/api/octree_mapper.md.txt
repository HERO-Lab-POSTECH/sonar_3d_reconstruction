# octree_mapper.cpp / .h

C++ low-level OctoMap wrapper for spatial octree operations.

**Location**: `sonar_3d_reconstruction/cpp/octree_mapper.cpp`

---

## Overview

`OctreeMapper` provides a thin wrapper around the OctoMap library's `OcTree` class. It handles probabilistic voxel updates, batch processing, and file I/O operations.

**Key Features**:
- OctoMap-based spatial octree structure
- Dual storage: OcTree + hash map for Python compatibility
- Batch update operations
- File persistence (binary format)

**Note**: This is a low-level class. Use `ProbabilityUpdater` for high-level IWLO updates.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  OctreeMapper                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  octomap::OcTree (unique_ptr)                                │   │
│  │  - Spatial octree structure                                  │   │
│  │  - Efficient spatial queries                                 │   │
│  │  - Native OctoMap operations                                 │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  log_odds_map_: unordered_map<point3d, double>              │   │
│  │  - Python SimpleOctree compatible                            │   │
│  │  - Direct log-odds access                                    │   │
│  │  - Used by get_occupied_voxels()                            │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  Cache System                                                │   │
│  │  - cached_num_nodes_: size_t                                │   │
│  │  - cache_valid_: bool                                       │   │
│  │  - Avoids expensive calcNumNodes() calls                    │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Struct: MemoryStats

Memory usage statistics returned by `get_memory_usage()`.

```cpp
struct MemoryStats {
    size_t num_nodes;        // Total nodes in octree
    size_t num_leaf_nodes;   // Leaf nodes only
    double memory_mb;        // Estimated memory in MB
    double memory_efficiency; // Ratio: leaf_nodes / total_nodes
};
```

---

## Class: OctreeMapper

### Constructor

```cpp
OctreeMapper(double resolution = 0.05,
             double prob_hit = 0.7,
             double prob_miss = 0.3,
             double prob_thres_min = 0.12,
             double prob_thres_max = 0.97);
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `resolution` | 0.05 | Voxel size in meters |
| `prob_hit` | 0.7 | Probability for occupied updates |
| `prob_miss` | 0.3 | Probability for free updates |
| `prob_thres_min` | 0.12 | Clamping minimum |
| `prob_thres_max` | 0.97 | Clamping maximum |

**Initialization**:
- Creates `octomap::OcTree` with given resolution
- Sets OctoMap internal threshold to 0.5 (for spatial queries only)
- Initializes log-odds bounds: `L_min = -10.0`, `L_max = 10.0`

**Note**: The actual occupied threshold used for visualization is passed as argument to `get_occupied_voxels(min_probability)`, configured via ROS2 parameter `mapping.occupied_threshold` (default: 0.7).

---

### Update Methods

#### update_point

```cpp
void update_point(const Eigen::Vector3d& point, bool occupied);
```

Update single voxel. Calls `octree_->updateNode()` internally.

#### batch_update

```cpp
void batch_update(const Eigen::MatrixXd& points, const std::vector<bool>& occupied_flags);
```

Batch update using OctoMap's native probability model.

| Parameter | Shape | Description |
|-----------|-------|-------------|
| `points` | Nx3 | Point coordinates |
| `occupied_flags` | N | Boolean flags |

**Implementation** (lines 80-124):
1. Validate coordinates (check `isfinite`)
2. Collect valid updates into vector
3. Apply updates sequentially (minimizes node expansion conflicts)

#### batch_update_with_log_odds

```cpp
void batch_update_with_log_odds(const Eigen::MatrixXd& points,
                                const Eigen::VectorXd& log_odds_updates);
```

**Primary update method** used by `ProbabilityUpdater`.

| Parameter | Shape | Description |
|-----------|-------|-------------|
| `points` | Nx3 | Point coordinates |
| `log_odds_updates` | N | Absolute log-odds values |

**Implementation** (lines 126-172):
1. Direct assignment to `log_odds_map_` (NOT incremental)
2. Clamp values to `[L_min, L_max]`
3. Update OcTree for spatial queries

**Important**: Values are absolute log-odds (not deltas). IWLO computes final values.

#### insert_ray

```cpp
void insert_ray(const Eigen::Vector3d& origin, const Eigen::Vector3d& endpoint);
```

Ray casting: marks free space along ray, occupied at endpoint.

---

### Query Methods

#### get_occupied_voxels

```cpp
Eigen::MatrixXd get_occupied_voxels(double min_probability = -1.0) const;
```

Returns Nx4 matrix `[x, y, z, probability]` for voxels above threshold.

**Implementation** (lines 185-227):
1. Convert probability threshold to log-odds
2. Iterate `log_odds_map_` (not OcTree)
3. Filter by log-odds threshold
4. Convert log-odds to probability for output

**Default threshold**: 0.5 (if `min_probability < 0`)

#### get_memory_usage

```cpp
MemoryStats get_memory_usage() const;
```

Returns memory statistics. Memory estimate: `num_nodes × 32 bytes`.

#### get_num_nodes

```cpp
size_t get_num_nodes() const;
```

Returns cached node count. Uses lazy evaluation pattern.

#### get_resolution

```cpp
double get_resolution() const;
```

Returns voxel resolution in meters.

---

### Configuration Methods

#### set_probability_params

```cpp
void set_probability_params(double prob_hit, double prob_miss);
```

Update OctoMap probability parameters.

#### set_occupancy_thresholds

```cpp
void set_occupancy_thresholds(double min_thresh, double max_thresh);
```

Update occupancy and clamping thresholds.

| Parameter | Description |
|-----------|-------------|
| `min_thresh` | Clamping minimum (probability floor) |
| `max_thresh` | **Occupancy threshold** + clamping maximum |

**Note**: `max_thresh` serves dual purpose - both occupancy threshold and clamping ceiling.

---

### Maintenance Methods

#### prune_tree

```cpp
size_t prune_tree();
```

Merge identical child nodes. Returns nodes removed.

#### clear

```cpp
void clear();
```

Clear both OcTree and log_odds_map_.

---

### File I/O

#### save_to_file

```cpp
bool save_to_file(const std::string& filename) const;
```

Save OcTree to binary file (.bt format).

#### load_from_file

```cpp
bool load_from_file(const std::string& filename);
```

Load OcTree from binary file.

**Note**: File I/O uses `SuppressOutput` RAII class to suppress OctoMap log messages.

---

## Internal Implementation

### SuppressOutput Class

RAII class to redirect stdout/stderr to `/dev/null`:

```cpp
class SuppressOutput {
public:
    SuppressOutput() {
        // Save file descriptors, redirect to /dev/null
        stdout_fd_ = dup(fileno(stdout));
        freopen("/dev/null", "w", stdout);
    }
    ~SuppressOutput() {
        // Restore original file descriptors
        dup2(stdout_fd_, fileno(stdout));
        close(stdout_fd_);
    }
};
```

**Purpose**: OctoMap prints verbose messages during file operations.

### Cache System

Expensive `calcNumNodes()` calls are cached:

```cpp
void update_cache() const {
    if (!cache_valid_) {
        cached_num_nodes_ = octree_->calcNumNodes();
        cache_valid_ = true;
    }
}

void invalidate_cache() {
    cache_valid_ = false;
}
```

Cache invalidated after any update operation.

### Point3d Hash

Custom hash for `octomap::point3d` used in `log_odds_map_`:

```cpp
struct Point3dHash {
    std::size_t operator()(const octomap::point3d& p) const {
        std::size_t h1 = std::hash<float>{}(p.x());
        std::size_t h2 = std::hash<float>{}(p.y());
        std::size_t h3 = std::hash<float>{}(p.z());
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};
```

---

## Dual Storage Rationale

| Storage | Purpose | Used By |
|---------|---------|---------|
| `octree_` (OcTree) | Spatial queries, ray casting | `insert_ray()`, native OctoMap ops |
| `log_odds_map_` | Exact log-odds values | `get_occupied_voxels()`, Python compatibility |

**Why both?** OctoMap internally converts log-odds to probability with clamping. The hash map preserves exact IWLO-computed values for visualization.

---

## Usage Example

```cpp
#include "octree_mapper.h"

// Create mapper with 5cm resolution
auto mapper = std::make_unique<OctreeMapper>(0.05);

// Batch update with log-odds
Eigen::MatrixXd points(100, 3);
Eigen::VectorXd log_odds(100);
// ... fill data ...

mapper->batch_update_with_log_odds(points, log_odds);

// Get occupied voxels (probability > 0.7)
auto occupied = mapper->get_occupied_voxels(0.7);

// Save to file
mapper->save_to_file("map.bt");
```

---

## Related

- [probability_updater.cpp](probability_updater.md) - High-level IWLO updater (uses this class)
- [IWLO Design](../design/iwlo_design.md) - Algorithm details
- [C++ Backend Refactoring Plan](../design/refactoring_plan.md) - Future architecture
