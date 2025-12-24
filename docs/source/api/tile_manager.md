# tile_manager.cpp / .h

C++ file system manager for tile-based storage.

**Location**: `sonar_3d_reconstruction/cpp/tile_manager.cpp`

---

## Overview

`TileManager` handles all file system operations for tile-based mapping. It manages tile paths, coordinate conversion, and global metadata.

**Key Features**:
- World coordinate to tile index conversion
- Tile directory path management
- Metadata persistence (JSON format)
- Tile enumeration and region queries

**Design Principle**: Single Responsibility - only file system operations, no IWLO logic.

---

## Data Structures

### MapMetadata

Global map metadata stored in `metadata.json`.

```cpp
struct MapMetadata {
    double resolution;      // Voxel resolution in meters
    double tile_size;       // Tile size in meters
    std::string version;    // Format version ("1.0")
    TileIndex min_tile;     // Minimum tile index (bounding box)
    TileIndex max_tile;     // Maximum tile index (bounding box)
};
```

---

## Class: TileManager

### Constructor

```cpp
TileManager(const std::string& base_path, double tile_size, double resolution);
```

| Parameter | Description |
|-----------|-------------|
| `base_path` | Root directory for tile storage |
| `tile_size` | Tile size in meters (default: 10.0) |
| `resolution` | Voxel resolution in meters (default: 0.05) |

### initialize

```cpp
bool initialize();
```

Initialize tile manager:
1. Create base directory if needed
2. Load existing metadata or save initial metadata

---

### Coordinate Conversion

#### world_to_tile_index

```cpp
TileIndex world_to_tile_index(double x, double y, double z) const;
TileIndex world_to_tile_index(const octomap::point3d& point) const;
```

Convert world coordinates to tile index using floor division.

```cpp
int ix = static_cast<int>(std::floor(x / tile_size_));
int iy = static_cast<int>(std::floor(y / tile_size_));
int iz = static_cast<int>(std::floor(z / tile_size_));
```

**Example** (tile_size = 10.0):
```
(5.0, 15.0, -3.0)  → TileIndex(0, 1, -1)
(-5.0, 0.0, 25.0)  → TileIndex(-1, 0, 2)
```

---

### Path Management

#### get_tile_directory

```cpp
std::string get_tile_directory(const TileIndex& idx) const;
```

Generate tile directory path.

```cpp
// Returns: "{base_path}/tile_{x}_{y}_{z}"
// Example: "/map_tiles/tile_0_1_-1"
```

#### tile_exists

```cpp
bool tile_exists(const TileIndex& idx) const;
```

Check if tile directory exists on disk.

---

### Tile Enumeration

#### list_all_tiles

```cpp
std::vector<TileIndex> list_all_tiles() const;
```

Enumerate all tile directories.

**Implementation**:
1. Iterate `base_path_` directory
2. Parse each subdirectory name using regex: `tile_(-?[0-9]+)_(-?[0-9]+)_(-?[0-9]+)`
3. Return valid tile indices

#### get_tiles_in_region

```cpp
std::vector<TileIndex> get_tiles_in_region(
    const Eigen::Vector3d& min_bound,
    const Eigen::Vector3d& max_bound) const;
```

Get all tiles intersecting a bounding box.

```cpp
TileIndex min_idx = world_to_tile_index(min_bound);
TileIndex max_idx = world_to_tile_index(max_bound);

for (int ix = min_idx.x; ix <= max_idx.x; ++ix)
    for (int iy = min_idx.y; iy <= max_idx.y; ++iy)
        for (int iz = min_idx.z; iz <= max_idx.z; ++iz)
            tiles.emplace_back(ix, iy, iz);
```

---

### Metadata Management

#### save_metadata / load_metadata

```cpp
bool save_metadata();
bool load_metadata();
```

Persist/load global metadata to/from `metadata.json`.

**JSON Format**:
```json
{
  "version": "1.0",
  "resolution": 0.05,
  "tile_size": 10.0,
  "bounds": {
    "min": [0, -1, 0],
    "max": [5, 3, 2]
  }
}
```

**Implementation**: Simple regex-based JSON parsing (no external library).

#### update_bounds

```cpp
void update_bounds(const TileIndex& idx);
```

Expand bounding box to include new tile.

```cpp
metadata_.min_tile.x = std::min(metadata_.min_tile.x, idx.x);
metadata_.max_tile.x = std::max(metadata_.max_tile.x, idx.x);
// ... y, z
```

---

### Tile Operations

#### delete_tile

```cpp
bool delete_tile(const TileIndex& idx);
```

Delete tile directory and all contents.

#### get_disk_usage

```cpp
size_t get_disk_usage() const;
```

Calculate total disk usage (bytes) using recursive directory iteration.

---

### Getters

```cpp
const std::string& get_base_path() const;
double get_tile_size() const;
double get_resolution() const;
const MapMetadata& get_metadata() const;
```

---

## Directory Structure

```
/map_tiles/                    ← base_path_
├── metadata.json              ← Global metadata
├── tile_0_0_0/
│   ├── octree.bt
│   └── iwlo_meta.bin
├── tile_1_0_0/
│   ├── octree.bt
│   └── iwlo_meta.bin
├── tile_0_1_-1/
│   ├── octree.bt
│   └── iwlo_meta.bin
└── ...
```

---

## Usage Example

```cpp
#include "tile_manager.h"

TileManager manager("/workspace/data/map_tiles", 10.0, 0.05);
manager.initialize();

// Convert world point to tile
TileIndex idx = manager.world_to_tile_index(15.5, -3.2, 7.8);
// Result: TileIndex(1, -1, 0)

// Get tile path
std::string path = manager.get_tile_directory(idx);
// Result: "/workspace/data/map_tiles/tile_1_-1_0"

// List all tiles
auto tiles = manager.list_all_tiles();

// Get tiles in region
auto region_tiles = manager.get_tiles_in_region(
    Eigen::Vector3d(0, 0, 0),
    Eigen::Vector3d(50, 50, 10)
);
```

---

## Related

- [Tile](tile.md) - Individual tile with IWLO logic
- [outofcore_tile_mapper.cpp](outofcore_tile_mapper.md) - High-level mapper using TileManager
