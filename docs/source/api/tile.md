# tile.cpp / .h

C++ individual tile with IWLO algorithm and file I/O.

**Location**: `sonar_3d_reconstruction/cpp/tile.cpp`

---

## Overview

`Tile` represents a single spatial region (10m x 10m x 10m by default) containing voxel data. Each tile can be independently saved/loaded, enabling out-of-core mapping.

**Key Features**:
- IWLO probability update (via [IWLOUpdater](iwlo_updater.md))
- Voxel storage (via [OctreeStorage](octree_storage.md))
- Binary file I/O (octree.bt + iwlo_meta.bin)
- Tile boundary management

**Refactored Architecture** (2025-12-24):
- Storage logic delegated to `OctreeStorage`
- IWLO calculations delegated to `IWLOUpdater`

---

## Data Structures

### IWLOMeta

Per-voxel IWLO metadata.

```cpp
struct IWLOMeta {
    double log_odds;         // Current log-odds value
    int observation_count;   // Number of observations
};
```

### IWLOParams

Algorithm parameters (shared across all tiles).

```cpp
struct IWLOParams {
    double log_odds_occupied;   // L_occ (default: 3.5)
    double log_odds_free;       // L_free (default: -3.0)
    double sharpness;           // Sigmoid steepness (default: 0.1)
    double decay_rate;          // Learning rate decay (default: 0.1)
    double min_alpha;           // Minimum learning rate (default: 0.3)
    double L_min, L_max;        // Saturation bounds (default: -10.0, 10.0)
    double intensity_threshold; // (default: 35.0)
    double intensity_max;       // (default: 255.0)
    double adaptive_threshold;  // (default: 0.5)
    double adaptive_max_ratio;  // (default: 0.3)
    bool adaptive_enabled;      // (default: true)
};
```

### TileIndex

Spatial coordinate in tile grid.

```cpp
struct TileIndex {
    int x, y, z;
    std::string to_string() const;  // "x_y_z"
};
```

### OccupiedVoxel

Return structure for occupied voxel queries.

```cpp
struct OccupiedVoxel {
    double x, y, z;
    double probability;
};
```

---

## Class: Tile

### Constructor

```cpp
Tile(const TileIndex& index, double resolution, double tile_size);
```

| Parameter | Description |
|-----------|-------------|
| `index` | Tile position in grid |
| `resolution` | Voxel resolution (meters) |
| `tile_size` | Tile size (meters) |

**Initialization**:
- Creates OcTree with given resolution
- Sets occupancy threshold to 0.5
- Initializes dirty flag to false

---

### IWLO Update Methods

#### update_voxel

```cpp
void update_voxel(const octomap::point3d& point,
                  double intensity,
                  bool is_occupied,
                  const IWLOParams& params);
```

Update single voxel using IWLO algorithm.

**IWLO Algorithm** (lines 90-144):
```
1. Get/create IWLO metadata for voxel key
2. Compute intensity weight: w = sigmoid(sharpness × (normalized - 0.5))
3. Compute learning rate: α = max(min_alpha, 1/(1 + decay_rate × n))
4. Compute adaptive scaling (bidirectional protection)
5. Compute delta:
   - Occupied: ΔL = L_occ × w × α × scale
   - Free: ΔL = L_free × α × scale
6. Apply: new_log_odds = clamp(old + ΔL, L_min, L_max)
```

**Note**: Uses [IWLOUpdater](iwlo_updater.md) utility functions internally for algorithm calculations.

#### batch_update

```cpp
void batch_update(const Eigen::MatrixXd& points,
                  const Eigen::VectorXd& intensities,
                  const std::vector<bool>& is_occupied,
                  const IWLOParams& params);
```

Batch update - calls `update_voxel()` for each point.

---

### File I/O

#### save

```cpp
bool save(const std::string& tile_dir);
```

Save tile to directory.

**Steps**:
1. Create directory if needed
2. `sync_to_octree()` - copy IWLO log-odds to OcTree nodes
3. `octree_->prune()` - merge homogeneous nodes
4. Write `octree.bt` (binary OctoMap format)
5. Write `iwlo_meta.bin` (custom binary format)
6. Set `dirty_ = false`

#### load

```cpp
bool load(const std::string& tile_dir);
```

Load tile from directory.

**Steps**:
1. Read `octree.bt` if exists
2. Read `iwlo_meta.bin` if exists
3. Set `dirty_ = false`

---

### IWLO Metadata File Format

**File**: `iwlo_meta.bin`

```
Header (16 bytes):
  - magic: uint32 (0x49574C4F = "IWLO")
  - version: uint32 (1)
  - count: uint64

Entries (18 bytes each):
  - key[0]: uint16
  - key[1]: uint16
  - key[2]: uint16
  - log_odds: double (8 bytes)
  - observation_count: int (4 bytes)
```

---

### Query Methods

#### get_occupied_voxels

```cpp
std::vector<OccupiedVoxel> get_occupied_voxels(double min_probability = 0.5) const;
Eigen::MatrixXd get_occupied_voxels_matrix(double min_probability = 0.5) const;
```

Return voxels with probability above threshold.

#### has_occupied_voxels

```cpp
bool has_occupied_voxels(double occupied_threshold = 0.5) const;
```

Check if any voxel exceeds threshold (early exit optimization).

#### Bounds

```cpp
Eigen::Vector3d get_min_bound() const;
Eigen::Vector3d get_max_bound() const;
bool contains(const octomap::point3d& point) const;
```

---

### State Management

```cpp
bool is_dirty() const;
void mark_dirty();
void mark_clean();
const TileIndex& get_index() const;
size_t get_num_voxels() const;
```

---

### Maintenance

#### prune

```cpp
size_t prune();
```

Sync to OcTree and merge homogeneous nodes. Returns nodes removed.

#### clear

```cpp
void clear();
```

Clear OcTree and IWLO metadata.

#### get_memory_usage

```cpp
size_t get_memory_usage() const;
```

Estimate memory usage (OcTree: 32 bytes/node, metadata: 34 bytes/entry).

---

### Internal Methods

#### sync_to_octree

```cpp
void sync_to_octree();
```

Copy IWLO log-odds values to OcTree nodes for visualization/saving.

```cpp
for (const auto& [key, meta] : iwlo_meta_) {
    octomap::point3d coord = octree_->keyToCoord(key);
    octomap::OcTreeNode* node = octree_->updateNode(coord, true, true);
    if (node) {
        node->setLogOdds(static_cast<float>(meta.log_odds));
    }
}
octree_->updateInnerOccupancy();
```

#### Helper Functions

```cpp
static double log_odds_to_probability(double log_odds);
static double intensity_to_weight(double intensity, const IWLOParams& params);
static double compute_alpha(int observation_count, const IWLOParams& params);
```

---

## Storage Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│  Tile                                                               │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  storage_: unique_ptr<OctreeStorage>                        │   │
│  │  - Delegates all voxel storage operations                    │   │
│  │  - See OctreeStorage for internal dual storage               │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                              │                                      │
│                              ▼ delegates to                         │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  OctreeStorage (see octree_storage.md)                       │   │
│  │  - iwlo_meta_: unordered_map<OcTreeKey, IWLOMeta>           │   │
│  │  - octree_: OcTree (for visualization)                       │   │
│  └─────────────────────────────────────────────────────────────┘   │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐   │
│  │  IWLOUpdater (static utility)                                │   │
│  │  - IWLO algorithm calculations                               │   │
│  │  - See iwlo_updater.md                                       │   │
│  └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Disk Storage

```
tile_0_0_0/
├── octree.bt      # OctoMap binary (pruned)
└── iwlo_meta.bin  # IWLO metadata (magic + version + entries)
```

---

## Related

- [IWLOUpdater](iwlo_updater.md) - IWLO algorithm utilities (used internally)
- [OctreeStorage](octree_storage.md) - Voxel storage implementation (used internally)
- [VoxelStorage](voxel_storage.md) - Storage interface
- [outofcore_tile_mapper.cpp](outofcore_tile_mapper.md) - Manages multiple tiles
- [tile_manager.cpp](tile_manager.md) - File system operations
- [IWLO Design](../design/iwlo_design.md) - Algorithm details
