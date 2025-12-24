# octree_storage.cpp / .h

C++ OctoMap-based implementation of VoxelStorage.

**Location**: `sonar_3d_reconstruction/cpp/octree_storage.h`, `octree_storage.cpp`

---

## Overview

`OctreeStorage` is the primary implementation of the `VoxelStorage` interface. It uses OctoMap's OcTree for spatial indexing combined with a hash map for IWLO metadata (log-odds, observation counts).

**Key Features**:
- OctoMap OcTree for efficient spatial queries
- Separate metadata map preserving exact IWLO values
- Binary file format for IWLO metadata persistence
- Dirty flag for efficient save operations

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  OctreeStorage                                                       │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  iwlo_meta_: unordered_map<OcTreeKey, IWLOMeta>              │   │
│  │  - Source of truth for IWLO algorithm                        │   │
│  │  - Stores exact log-odds and observation counts              │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼ sync_to_octree()                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  octree_: unique_ptr<OcTree>                                 │   │
│  │  - For visualization (RViz)                                  │   │
│  │  - For spatial queries                                       │   │
│  │  - Discretized probability values                            │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Class: OctreeStorage

### Constructor

```cpp
explicit OctreeStorage(double resolution);
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | double | - | Voxel resolution in meters |

Creates a new OcTree with the specified resolution.

---

### Copy/Move Semantics

```cpp
OctreeStorage(const OctreeStorage&) = delete;           // No copy
OctreeStorage& operator=(const OctreeStorage&) = delete; // No copy assignment
OctreeStorage(OctreeStorage&&) = default;               // Move allowed
OctreeStorage& operator=(OctreeStorage&&) = default;    // Move assignment allowed
```

---

## VoxelStorage Interface Implementation

### Core Operations

All core operations are implemented as specified in [VoxelStorage](voxel_storage.md):
- `get_log_odds()` / `set_log_odds()`
- `get_observation_count()` / `increment_observation_count()`
- `get_or_create_meta()`

---

### Coordinate Operations

```cpp
octomap::OcTreeKey coord_to_key(const octomap::point3d& point) const override;
octomap::point3d key_to_coord(const octomap::OcTreeKey& key) const override;
```

Delegates to the underlying OcTree's coordinate system.

---

### Query Operations

```cpp
std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const override;
size_t get_num_voxels() const override;
bool has_occupied_voxels(double min_probability = 0.5) const override;
```

Queries are performed on `iwlo_meta_` map with probability conversion.

---

### Persistence Operations

```cpp
bool save(const std::string& path) override;
bool load(const std::string& path) override;
```

Saves/loads to a directory with two files:
- `octree.bt` - OctoMap binary tree
- `iwlo_meta.bin` - IWLO metadata binary

---

### Maintenance Operations

```cpp
void clear() override;
void sync_to_octree() override;
size_t prune() override;
size_t get_memory_usage() const override;
```

---

## Additional Methods

### get_resolution

```cpp
double get_resolution() const;
```

Returns the voxel resolution in meters.

---

### Dirty Flag Management

```cpp
bool is_dirty() const;
void mark_clean();
void mark_dirty();
```

Tracks whether storage has been modified since last save.

**Usage**: Tile class uses this for efficient persistence:
```cpp
if (storage_->is_dirty()) {
    storage_->save(path);
    storage_->mark_clean();
}
```

---

## File Formats

### octree.bt

Standard OctoMap binary format. Created by `octree_->writeBinary()`.

### iwlo_meta.bin

Custom binary format for IWLO metadata:

```
Header (16 bytes):
┌──────────────────────────────────────────┐
│  magic: uint32 (0x49574C4F = "IWLO")     │  4 bytes
│  version: uint32 (1)                      │  4 bytes
│  count: uint64 (number of entries)        │  8 bytes
└──────────────────────────────────────────┘

Entries (18 bytes each):
┌──────────────────────────────────────────┐
│  key[0]: uint16                           │  2 bytes
│  key[1]: uint16                           │  2 bytes
│  key[2]: uint16                           │  2 bytes
│  log_odds: double                         │  8 bytes
│  observation_count: int32                 │  4 bytes
└──────────────────────────────────────────┘
```

---

## Usage Example

```cpp
#include "octree_storage.h"

// Create storage with 5cm resolution
auto storage = std::make_unique<OctreeStorage>(0.05);

// Update a voxel
octomap::point3d point(1.0, 2.0, 3.0);
auto key = storage->coord_to_key(point);

auto& meta = storage->get_or_create_meta(key);
meta.log_odds += 1.5;  // Update log-odds
meta.observation_count++;

storage->mark_dirty();

// Sync to OcTree for visualization
storage->sync_to_octree();

// Save to disk
storage->save("/path/to/tile_dir");
storage->mark_clean();

// Query occupied voxels
auto occupied = storage->get_occupied_voxels(0.7);
for (const auto& voxel : occupied) {
    std::cout << voxel.x << ", " << voxel.y << ", " << voxel.z
              << " -> " << voxel.probability << std::endl;
}
```

---

## Design Rationale

**Why dual storage?**
- OctoMap internally converts log-odds to float with clamping
- Hash map preserves exact log-odds values for IWLO algorithm
- sync_to_octree() copies discretized values for visualization

**Memory trade-off**:
- Additional ~20 bytes per voxel for metadata
- Enables accurate probability tracking without OctoMap limitations

---

## Related

- [VoxelStorage](voxel_storage.md) - Abstract interface
- [Octree Mapping Design](../design/octree_mapping.md) - Dual storage design
- [tile.md](tile.md) - Uses OctreeStorage for per-tile storage
