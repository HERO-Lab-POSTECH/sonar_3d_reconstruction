# Out-of-Core Tile Mapping Design

## Overview

Disk-based tile storage for large-scale mapping with **LRU cache** supporting **unlimited map size** in memory-constrained environments.

**Key Features**:
- Tile-based partitioning: Divides space into fixed-size tiles
- LRU cache: Keeps only active tiles in memory
- Transparent I/O: Automatic save on cache eviction

---

## 1. Architecture

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
│  └── ...                                                            │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 2. Tile System

### 2.1 Tile Definition

**Tile:** Fixed-size 3D region (default: $10m \times 10m \times 10m$)

**Tile Index:**

$$\text{TileIndex} = (i_x, i_y, i_z)$$

where:

$$i_x = \lfloor x / T \rfloor$$
$$i_y = \lfloor y / T \rfloor$$
$$i_z = \lfloor z / T \rfloor$$

$T$ = tile_size (default: 10.0m)

**Example** ($T = 10.0$):

| World Coordinate | Tile Index |
|------------------|------------|
| (5.0, 15.0, -3.0) | (0, 1, -1) |
| (-5.0, 0.0, 25.0) | (-1, 0, 2) |

---

### 2.2 Tile Contents

Each tile contains the following:

| Component | Description | File |
|-----------|-------------|------|
| OcTree | Spatial structure (pruned) | `octree.bt` |
| IWLO Metadata | Per-voxel log_odds + observation count | `iwlo_meta.bin` |

**Dual Storage in Tile:**

```
┌─────────────────────────────────────────────────────────────┐
│  Tile                                                       │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  iwlo_meta_: unordered_map<OcTreeKey, IWLOMeta>     │   │
│  │  - Primary storage for IWLO algorithm                │   │
│  │  - Preserves exact log-odds and observation count    │   │
│  └─────────────────────────────────────────────────────┘   │
│                      │                                      │
│                      ▼ sync_to_octree()                     │
│  ┌─────────────────────────────────────────────────────┐   │
│  │  octree_: OcTree                                     │   │
│  │  - For visualization and spatial queries             │   │
│  │  - Prunable for efficient storage                    │   │
│  └─────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

---

## 3. Storage Format

### 3.1 Directory Structure

```
/map_tiles/                    ← base_path
├── metadata.json              ← Global metadata
├── tile_0_0_0/
│   ├── octree.bt              ← OctoMap binary
│   └── iwlo_meta.bin          ← IWLO metadata
├── tile_1_0_0/
│   ├── octree.bt
│   └── iwlo_meta.bin
├── tile_0_1_-1/
│   ├── octree.bt
│   └── iwlo_meta.bin
└── ...
```

---

### 3.2 IWLO Metadata Binary Format

**File:** `iwlo_meta.bin`

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

### 3.3 Metadata JSON

**File:** `metadata.json`

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

---

## 4. LRU Cache

### 4.1 Cache Structure

| Parameter | Default | Description |
|-----------|---------|-------------|
| cache_size | 16 | Maximum tiles to keep in memory |
| tile_size | 10.0 | Tile size (meters) |

**Memory Estimation:**
- Each tile: ~10-50 MB (depends on voxel density)
- 16 tile cache: ~160-800 MB RAM

---

### 4.2 Eviction Policy

```
ALGORITHM: LRU_Eviction
INPUT: new_tile to cache, cache_size limit

1. IF cache.size() >= cache_size THEN
     lru_tile := get_least_recently_used_tile()

     IF lru_tile.is_dirty() THEN
       tile_dir := tile_manager.get_tile_directory(lru_tile.index)
       lru_tile.save(tile_dir)
       recently_saved_tiles.append(lru_tile.index)
     ENDIF

     cache.evict(lru_tile)
   ENDIF

2. cache.put(new_tile)
```

**Dirty Flag:** Tracks whether a tile has been modified since last save

---

### 4.3 Eviction Callback

```cpp
// Initialize LRU cache in constructor
tile_cache_(cache_size, [this](const TileIndex& idx,
                                std::unique_ptr<Tile>& tile) {
    this->on_tile_eviction(idx, tile);
})

// Eviction handler callback
void on_tile_eviction(const TileIndex& idx,
                      std::unique_ptr<Tile>& tile) {
    if (tile && tile->is_dirty()) {
        std::string tile_dir = tile_manager_.get_tile_directory(idx);
        if (tile->save(tile_dir)) {
            recently_saved_tiles_.push_back(idx);
        }
    }
}
```

---

## 5. Operations

### 5.1 Point Update Flow

```
ALGORITHM: Batch_Update_IWLO
INPUT: points[] (Nx3), intensities[] (N), is_occupied[] (N)

1. GROUP points by tile:
   tile_groups := {}
   FOR i := 0 to N-1:
     idx := tile_manager.world_to_tile_index(points[i])
     tile_groups[idx].append(i)
   ENDFOR

2. UPDATE each tile:
   FOR each (idx, point_indices) in tile_groups:
     tile := get_or_load_tile(idx)

     // Extract subset
     tile_points := points[point_indices]
     tile_intensities := intensities[point_indices]
     tile_is_occupied := is_occupied[point_indices]

     // Apply IWLO update
     tile.batch_update(tile_points, tile_intensities,
                       tile_is_occupied, iwlo_params)

     // Update bounds
     tile_manager.update_bounds(idx)
   ENDFOR
```

---

### 5.2 Query Flow

```
ALGORITHM: Get_All_Occupied_Voxels
INPUT: min_probability threshold
OUTPUT: list of (x, y, z, probability)

1. ENUMERATE all tiles:
   tile_indices := tile_manager.list_all_tiles()

2. COLLECT voxels:
   results := []
   FOR each idx in tile_indices:
     tile := get_or_load_tile(idx)
     voxels := tile.get_occupied_voxels(min_probability)
     results.extend(voxels)
   ENDFOR

3. RETURN results
```

**Note:** `get_occupied_voxels()` queries cached tiles only, while `get_all_occupied_voxels()` loads all tiles from disk

---

### 5.3 Flush Operation

```
ALGORITHM: Flush_All
PURPOSE: Save all dirty tiles to disk

1. FOR each tile in cache:
     IF tile.is_dirty() THEN
       tile_dir := tile_manager.get_tile_directory(tile.index)
       tile.save(tile_dir)
       tile.mark_clean()
     ENDIF
   ENDFOR
```

---

## 6. Implementation Classes

| Class | Role | Location |
|-------|------|----------|
| `OutofcoreTileMapper` | Overall coordination | [API Reference](../api/outofcore_tile_mapper.md) |
| `Tile` | Individual tile storage | [API Reference](../api/tile.md) |
| `TileManager` | File system operations | [API Reference](../api/tile_manager.md) |

---

## 7. Mode Comparison

| Aspect | ProbabilityUpdater | OutofcoreTileMapper |
|--------|-------------------|---------------------|
| Storage | RAM only | Disk + RAM cache |
| Map Size | Limited by RAM | **Unlimited** |
| Speed | Faster | Slower (I/O overhead) |
| Use Case | Small maps (<100m) | Large maps (>100m) |
| API | Original | Compatible |

**Selection Guide:**
- Sufficient RAM: `ProbabilityUpdater` (In-Memory)
- Large-scale maps or limited RAM: `OutofcoreTileMapper` (Out-of-Core)

---

## 8. Related Documents

### Design Documents
- [IWLO Probability Update](iwlo_design.md) - Probability update algorithm
- [Octree Mapping](octree_mapping.md) - Spatial data structure

### API Reference
- [outofcore_tile_mapper](../api/outofcore_tile_mapper.md) - High-level coordinator
- [tile](../api/tile.md) - Individual tile storage
- [tile_manager](../api/tile_manager.md) - File system operations
