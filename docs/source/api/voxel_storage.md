# voxel_storage.h

C++ abstract interface for voxel data storage.

**Location**: `sonar_3d_reconstruction/cpp/voxel_storage.h`

---

## Overview

`VoxelStorage` is an abstract interface that defines the contract for storing voxel data. It provides a common API for different storage backends, enabling consistent access to voxel data regardless of the underlying implementation.

**Key Features**:
- Abstract interface for voxel storage backends
- Support for IWLO metadata (log-odds, observation counts)
- Persistence operations (save/load)
- OcTree access for visualization

---

## Class: VoxelStorage

### Purpose

Defines the common interface for:
- Storing and retrieving log-odds values
- Managing observation counts for IWLO learning rate decay
- Coordinate transformations (world ↔ OcTree key)
- Query operations for occupied voxels
- Persistence to/from disk

---

## Interface Methods

### Core Operations

#### get_log_odds

```cpp
virtual double get_log_odds(const octomap::OcTreeKey& key) const = 0;
```

Get log-odds value for a voxel.

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key identifying the voxel |
| **Returns** | double | Log-odds value (0.0 if not found) |

---

#### set_log_odds

```cpp
virtual void set_log_odds(const octomap::OcTreeKey& key, double value) = 0;
```

Set log-odds value for a voxel.

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key identifying the voxel |
| `value` | double | Log-odds value to set |

---

#### get_observation_count

```cpp
virtual int get_observation_count(const octomap::OcTreeKey& key) const = 0;
```

Get observation count for a voxel (used in IWLO learning rate decay).

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key identifying the voxel |
| **Returns** | int | Observation count (0 if not found) |

---

#### increment_observation_count

```cpp
virtual int increment_observation_count(const octomap::OcTreeKey& key) = 0;
```

Increment observation count for a voxel.

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key identifying the voxel |
| **Returns** | int | New observation count after increment |

---

#### get_or_create_meta

```cpp
virtual IWLOMeta& get_or_create_meta(const octomap::OcTreeKey& key) = 0;
```

Get or create IWLO metadata for a voxel.

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key identifying the voxel |
| **Returns** | IWLOMeta& | Reference to metadata (created with defaults if not exists) |

---

### Coordinate Operations

#### coord_to_key

```cpp
virtual octomap::OcTreeKey coord_to_key(const octomap::point3d& point) const = 0;
```

Convert world coordinates to OcTree key.

| Parameter | Type | Description |
|-----------|------|-------------|
| `point` | point3d | World coordinates (x, y, z) |
| **Returns** | OcTreeKey | OcTree key for the voxel containing this point |

---

#### key_to_coord

```cpp
virtual octomap::point3d key_to_coord(const octomap::OcTreeKey& key) const = 0;
```

Convert OcTree key to world coordinates (center of voxel).

| Parameter | Type | Description |
|-----------|------|-------------|
| `key` | OcTreeKey | OcTree key |
| **Returns** | point3d | World coordinates at voxel center |

---

### Query Operations

#### get_occupied_voxels

```cpp
virtual std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const = 0;
```

Get all occupied voxels above probability threshold.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `min_probability` | double | 0.5 | Minimum probability threshold |
| **Returns** | vector | - | Vector of OccupiedVoxel structs |

---

#### get_num_voxels

```cpp
virtual size_t get_num_voxels() const = 0;
```

Get number of stored voxels with metadata.

---

#### has_occupied_voxels

```cpp
virtual bool has_occupied_voxels(double min_probability = 0.5) const = 0;
```

Check if storage has any occupied voxels above threshold.

---

### Persistence Operations

#### save

```cpp
virtual bool save(const std::string& path) = 0;
```

Save storage to directory.

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | string | Directory path |
| **Returns** | bool | True if successful |

---

#### load

```cpp
virtual bool load(const std::string& path) = 0;
```

Load storage from directory.

| Parameter | Type | Description |
|-----------|------|-------------|
| `path` | string | Directory path |
| **Returns** | bool | True if successful |

---

### Maintenance Operations

#### clear

```cpp
virtual void clear() = 0;
```

Clear all stored data.

---

#### sync_to_octree

```cpp
virtual void sync_to_octree() = 0;
```

Synchronize metadata to underlying OcTree for visualization.

---

#### prune

```cpp
virtual size_t prune() = 0;
```

Prune octree (merge homogeneous nodes).

| **Returns** | size_t | Number of nodes pruned |

---

#### get_memory_usage

```cpp
virtual size_t get_memory_usage() const = 0;
```

Get estimated memory usage in bytes.

---

### OcTree Access

#### get_octree

```cpp
virtual octomap::OcTree* get_octree() = 0;
virtual const octomap::OcTree* get_octree() const = 0;
```

Get underlying OcTree pointer (may be null).

Used for:
- Visualization (RViz octomap markers)
- Merging multiple storages
- Binary export

---

## Implementations

| Implementation | Backend | Description |
|----------------|---------|-------------|
| [OctreeStorage](octree_storage.md) | OctoMap | Primary implementation using OcTree + hash map |

---

## Data Types

### IWLOMeta

```cpp
struct IWLOMeta {
    double log_odds = 0.0;        // Current log-odds value
    int observation_count = 0;    // Number of observations
};
```

### OccupiedVoxel

```cpp
struct OccupiedVoxel {
    double x, y, z;              // World coordinates
    double probability;          // Occupancy probability
};
```

---

## Design Rationale

**Why an abstract interface?**
- Enables different storage backends (OctoMap, hash-only, GPU-based)
- Facilitates testing with mock implementations
- Separates storage concerns from update logic

**Dual Storage Pattern**:
- Metadata (iwlo_meta_) stores exact log-odds + observation counts
- OcTree stores discretized values for spatial queries
- sync_to_octree() bridges the two

---

## Related

- [OctreeStorage](octree_storage.md) - OctoMap-based implementation
- [Octree Mapping Design](../design/octree_mapping.md) - Data structure design
- [tile.md](tile.md) - Uses VoxelStorage via OctreeStorage
