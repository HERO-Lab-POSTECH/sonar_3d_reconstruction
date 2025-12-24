# outofcore_tile_mapper.cpp / .h

C++ Out-of-Core 3D mapping with tile-based disk storage.

**Location**: `sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp`

**Implements**: [IMapperBackend](mapper_backend.md)

---

## Overview

`OutofcoreTileMapper` provides disk-based tile storage for large-scale mapping. Only active tiles are kept in memory using LRU cache, enabling unlimited map size.

**Key Features**:
- Disk-based tile storage (10m x 10m x 10m per tile)
- LRU cache for memory management
- IWLO algorithm support (via [IWLOUpdater](iwlo_updater.md))
- Implements `IMapperBackend` interface (Backend Type: `"Disk"`)

**Use Case**: Large-scale mapping (>100m x 100m) or limited RAM environments.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  OutofcoreTileMapper                                                │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  LRU Cache (size: cache_size)                                │   │
│  │  ┌─────────┐ ┌─────────┐ ┌─────────┐         ┌─────────┐    │   │
│  │  │ Tile 0  │ │ Tile 1  │ │ Tile 2  │   ...   │ Tile N  │    │   │
│  │  └─────────┘ └─────────┘ └─────────┘         └─────────┘    │   │
│  │       ↑ eviction callback: save to disk                     │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  TileManager                                                 │   │
│  │  - world_to_tile_index(): coordinate → tile index           │   │
│  │  - get_tile_directory(): tile index → file path             │   │
│  │  - list_all_tiles(): enumerate disk tiles                   │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  IWLOParams (shared with all tiles)                         │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
           │
           ▼ Disk Storage
┌─────────────────────────────────────────────────────────────────────┐
│  /map_tiles/                                                        │
│  ├── metadata.json                                                  │
│  ├── tile_0_0_0/                                                    │
│  │   ├── octree.bt                                                  │
│  │   └── iwlo_meta.bin                                              │
│  ├── tile_1_0_0/                                                    │
│  └── ...                                                            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Class: OutofcoreTileMapper

### Constructor

```cpp
OutofcoreTileMapper(const std::string& map_path,
                    double resolution = 0.05,
                    double tile_size = 10.0,
                    size_t cache_size = 16);
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `map_path` | - | Base directory for tile storage |
| `resolution` | 0.05 | Voxel resolution in meters |
| `tile_size` | 10.0 | Tile size in meters (10m x 10m x 10m) |
| `cache_size` | 16 | Maximum tiles in memory |

**Initialization**:
- Creates `TileManager` for file system operations
- Initializes LRU cache with eviction callback
- Sets default `occupied_threshold_` to 0.7

---

## ProbabilityUpdater Compatible API

These methods provide API compatibility with `ProbabilityUpdater` for easy mode switching.

### batch_update_iwlo

```cpp
void batch_update_iwlo(const Eigen::MatrixXd& points,
                       const Eigen::VectorXd& intensities,
                       const std::vector<bool>& is_occupied);
```

**Main entry point** for probability updates.

**Implementation** (lines 69-111):
1. Group points by tile using `group_points_by_tile()`
2. For each tile:
   - Get or load tile via `get_or_load_tile()`
   - Call `tile->batch_update()` with IWLO params
   - Update metadata bounds

### Configuration Methods

```cpp
void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                     double L_min, double L_max);
void set_log_odds_params(double log_odds_occupied, double log_odds_free);
void set_intensity_params(double intensity_threshold, double intensity_max);
void set_adaptive_params(bool enabled, double threshold, double max_ratio);
void set_occupied_threshold(double threshold);
```

All parameters are stored in `iwlo_params_` and passed to tiles during updates.

### Query Methods

#### get_occupied_voxels

```cpp
Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5);
```

Returns occupied voxels from **cached tiles only**. Fast but may miss tiles not in cache.

#### get_all_occupied_voxels

```cpp
Eigen::MatrixXd get_all_occupied_voxels(double min_probability = 0.5);
```

Returns occupied voxels from **ALL tiles** (loads from disk). Use for visualization.

#### get_memory_usage / get_num_nodes

```cpp
MemoryStats get_memory_usage() const;
size_t get_num_nodes() const;
```

Statistics from cached tiles only.

---

## Extended API

### Region Query

```cpp
Eigen::MatrixXd get_occupied_voxels_in_region(
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound,
    double min_probability = 0.5);
```

Get voxels within bounding box. Loads only tiles intersecting the region.

### Flush Operations

```cpp
void flush_all();
std::vector<TileIndex> flush_and_get_dirty_tiles();
bool flush_tile(const TileIndex& idx);
```

Save dirty tiles to disk.

| Method | Description |
|--------|-------------|
| `flush_all()` | Save all dirty tiles |
| `flush_and_get_dirty_tiles()` | Save and return saved tile indices |
| `flush_tile(idx)` | Save specific tile |

### Merged OcTree

```cpp
std::unique_ptr<octomap::OcTree> get_merged_octree();
std::unique_ptr<octomap::OcTree> get_full_merged_octree(double min_probability = -1.0);
bool save_merged_octree(const std::string& filepath);
std::pair<std::vector<int8_t>, std::string> get_octree_binary(double min_probability = 0.5);
```

| Method | Source | Use Case |
|--------|--------|----------|
| `get_merged_octree()` | Cached tiles | Quick visualization |
| `get_full_merged_octree()` | All disk tiles | Complete map export |
| `save_merged_octree()` | All disk tiles | Save to .bt file |
| `get_octree_binary()` | All disk tiles | ROS octomap_msgs |

### Tile Management

```cpp
size_t get_cached_tile_count() const;
size_t get_total_tile_count() const;
std::vector<TileIndex> get_all_tile_indices() const;
size_t get_disk_usage() const;
```

### Preloading

```cpp
void preload_region(const Eigen::Vector3d& min_bound,
                    const Eigen::Vector3d& max_bound);
void reload_tiles(const std::vector<TileIndex>& indices);
```

Pre-load tiles for anticipated access patterns.

### Maintenance

```cpp
void clear();
size_t prune_all();
```

| Method | Description |
|--------|-------------|
| `clear()` | Delete all tiles (cache + disk) |
| `prune_all()` | Merge homogeneous octree nodes |

### Visualizer Notification

```cpp
std::vector<TileIndex> get_and_clear_saved_tiles();
```

Get indices of tiles saved since last call. Used by `map_visualizer_node` to reload updated tiles.

---

## Internal Implementation

### LRU Cache with Eviction Callback

```cpp
// Constructor
tile_cache_(cache_size, [this](const TileIndex& idx, std::unique_ptr<Tile>& tile) {
    this->on_tile_eviction(idx, tile);
})

// Eviction handler
void on_tile_eviction(const TileIndex& idx, std::unique_ptr<Tile>& tile) {
    if (tile && tile->is_dirty()) {
        std::string tile_dir = tile_manager_.get_tile_directory(idx);
        if (tile->save(tile_dir)) {
            recently_saved_tiles_.push_back(idx);  // For visualizer notification
        }
    }
}
```

### Point Grouping

```cpp
std::unordered_map<TileIndex, std::vector<int>, TileIndexHash>
group_points_by_tile(const Eigen::MatrixXd& points) {
    for (int i = 0; i < points.rows(); ++i) {
        TileIndex idx = tile_manager_.world_to_tile_index(
            points(i, 0), points(i, 1), points(i, 2));
        groups[idx].push_back(i);
    }
    return groups;
}
```

### Thread Safety

```cpp
mutable std::mutex cache_mutex_;        // Protects tile_cache_
mutable std::mutex saved_tiles_mutex_;  // Protects recently_saved_tiles_
```

All public methods acquire appropriate locks.

---

## ROS2 Parameter Mapping

| ROS2 Parameter | C++ Variable | Description |
|----------------|--------------|-------------|
| `outofcore.map_path` | `map_path_` | Tile storage directory |
| `outofcore.tile_size` | `tile_size_` | Tile size in meters |
| `outofcore.cache_size` | `cache_size_` | LRU cache capacity |
| `iwlo.*` | `iwlo_params_` | IWLO algorithm parameters |
| `mapping.occupied_threshold` | `occupied_threshold_` | Save filtering threshold |

---

## Usage Example

```cpp
#include "outofcore_tile_mapper.h"

// Create mapper with 16-tile cache
auto mapper = std::make_unique<OutofcoreTileMapper>(
    "/workspace/data/map_tiles",  // map_path
    0.05,   // resolution
    10.0,   // tile_size
    16      // cache_size
);

// Configure IWLO
mapper->set_iwlo_params(0.1, 0.1, 0.3, -10.0, 10.0);
mapper->set_intensity_params(35.0, 255.0);

// Process sonar data
mapper->batch_update_iwlo(points, intensities, is_occupied);

// Periodic flush (called by timer)
auto saved = mapper->flush_and_get_dirty_tiles();
// Notify visualizer: saved tile indices

// Get all voxels for visualization
auto voxels = mapper->get_all_occupied_voxels(0.7);
```

---

## Comparison: ProbabilityUpdater vs OutofcoreTileMapper

| Aspect | ProbabilityUpdater | OutofcoreTileMapper |
|--------|-------------------|---------------------|
| Storage | RAM only | Disk + RAM cache |
| Map Size | Limited by RAM | Unlimited |
| Speed | Faster | Slower (I/O) |
| Visualization | Direct | Via `map_visualizer_node` |
| API | Original | Compatible |

---

## IMapperBackend Implementation

As an `IMapperBackend` implementation, `OutofcoreTileMapper` provides:

### Backend Identity

```cpp
std::string get_backend_type() const override { return "Disk"; }
bool supports_persistence() const override { return true; }
```

### Optional Methods

| Method | Behavior |
|--------|----------|
| `flush()` | Delegates to `flush_all()` |
| `preload_region()` | Loads tiles in bounding box |
| `get_disk_usage()` | Returns actual disk usage |
| `prune()` | Delegates to `prune_all()` |

See [IMapperBackend](mapper_backend.md) for the full interface specification.

---

## Related

- [IMapperBackend](mapper_backend.md) - Abstract interface implemented by this class
- [IWLOUpdater](iwlo_updater.md) - IWLO algorithm utilities
- [tile.cpp](tile.md) - Individual tile with IWLO logic
- [tile_manager.cpp](tile_manager.md) - File system operations
- [probability_updater.cpp](probability_updater.md) - In-Memory mode equivalent
- [IWLO Design](../design/iwlo_design.md) - Algorithm details
- [Out-of-Core Design](../design/outofcore_design.md) - Tile system architecture
