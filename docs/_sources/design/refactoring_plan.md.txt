# C++ Backend Refactoring Plan

## Current Architecture Analysis

### Problem: Code Duplication

The IWLO probability update logic is duplicated in two places:

| File | Location | Role |
|-----|------|------|
| `probability_updater.cpp` | Lines 349-384 | IWLO for In-Memory mode |
| `tile.cpp` | Lines 106-141 | IWLO for Out-of-Core mode |

Both implementations have **identical algorithms**, differing only in parameter access:
- `ProbabilityUpdater`: Direct member variable access
- `Tile`: Passed via `IWLOParams` struct

### Current Class Relationships

```
┌─────────────────────────────────────────────────────────────┐
│  In-Memory Mode                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ProbabilityUpdater                                  │    │
│  │  - IWLO logic (duplicated)                           │    │
│  │  - voxels_log_odds_ (unordered_map<string, double>) │    │
│  │  - observation_counts_ (unordered_map<string, int>) │    │
│  │       │                                              │    │
│  │       └──▶ OctreeMapper (OctoMap wrapper, storage)   │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Out-of-Core Mode                                            │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  OutofcoreTileMapper                                 │    │
│  │  - TileManager (disk I/O)                            │    │
│  │  - TileCache (LRU cache)                             │    │
│  │       │                                              │    │
│  │       └──▶ Tile[]                                    │    │
│  │            - IWLO logic (duplicated)                 │    │
│  │            - iwlo_meta_ (unordered_map<Key, Meta>)  │    │
│  │            - octomap::OcTree (direct usage)          │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
```

### Issues

1. **IWLO logic duplication**: Same algorithm implemented twice
2. **Maintenance risk**: Changes require updates in both locations
3. **Test complexity**: Same logic must be tested twice
4. **Mixed responsibilities**: Each class handles multiple concerns

---

## Refactoring Plan

### Goal

Follow Single Responsibility Principle (SRP) so each class handles one concern.

### Proposed Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Shared Components                                           │
│                                                              │
│  ┌─────────────────────┐  ┌─────────────────────────────┐   │
│  │  IWLOUpdater        │  │  VoxelStorage (Interface)   │   │
│  │  - intensity_to_w() │  │  - get/set log_odds         │   │
│  │  - compute_alpha()  │  │  - get/set observation_cnt  │   │
│  │  - update_voxel()   │  │  - get_occupied_voxels()    │   │
│  │  - batch_update()   │  │                             │   │
│  └─────────────────────┘  └─────────────────────────────┘   │
│           │                         ▲                        │
│           │                         │ implements             │
│           ▼                         │                        │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  OctreeStorage                                       │    │
│  │  - octomap::OcTree wrapper                           │    │
│  │  - log_odds_map_ (metadata)                          │    │
│  │  - save/load .bt files                              │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────┐
│  Backend Implementations                                     │
│                                                              │
│  ┌─────────────────────────┐  ┌─────────────────────────┐   │
│  │  RamBackend             │  │  DiskBackend            │   │
│  │  (In-Memory Mode)       │  │  (Out-of-Core Mode)     │   │
│  │  - Single OctreeStorage │  │  - TileManager          │   │
│  │  - Uses IWLOUpdater     │  │  - TileCache (LRU)      │   │
│  │                         │  │  - Tile[] (OctreeStorage│   │
│  │                         │  │    + uses IWLOUpdater)  │   │
│  └─────────────────────────┘  └─────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### New Class Responsibilities

| Class | Responsibility | Notes |
|-------|----------------|-------|
| `IWLOUpdater` | Probability update algorithm | **Shared**, pure algorithm |
| `OctreeStorage` | OctoMap wrapper + metadata management | **Shared**, storage abstraction |
| `RamBackend` | RAM-based single map management | In-Memory mode |
| `DiskBackend` | Disk-based tile management | Out-of-Core mode |
| `Tile` | Per-tile OctreeStorage instance | Internal to DiskBackend |

### IWLOUpdater Interface (Draft)

```cpp
class IWLOUpdater {
public:
    struct Params {
        double log_odds_occupied = 3.5;
        double log_odds_free = -3.0;
        double sharpness = 0.1;
        double decay_rate = 0.1;
        double min_alpha = 0.3;
        double L_min = -10.0;
        double L_max = 10.0;
        double intensity_threshold = 35.0;
        double intensity_max = 255.0;
        bool adaptive_enabled = true;
        double adaptive_threshold = 0.5;
        double adaptive_max_ratio = 0.3;
    };

    // Core algorithm (stateless)
    static double intensity_to_weight(double intensity, const Params& p);
    static double compute_alpha(int observation_count, const Params& p);
    static double compute_delta_log_odds(
        double intensity,
        double current_log_odds,
        int observation_count,
        const Params& p
    );
};
```

### VoxelStorage Interface (Draft)

```cpp
class VoxelStorage {
public:
    virtual ~VoxelStorage() = default;

    // Core operations
    virtual double get_log_odds(const VoxelKey& key) const = 0;
    virtual void set_log_odds(const VoxelKey& key, double value) = 0;
    virtual int get_observation_count(const VoxelKey& key) const = 0;
    virtual void increment_observation_count(const VoxelKey& key) = 0;

    // Query
    virtual std::vector<OccupiedVoxel> get_occupied_voxels(double min_prob) const = 0;

    // Persistence
    virtual bool save(const std::string& path) = 0;
    virtual bool load(const std::string& path) = 0;
};
```

---

## Implementation Steps

### Phase 1: Extract IWLO Algorithm

1. [ ] Create `IWLOUpdater` class (stateless static methods)
2. [ ] Modify `ProbabilityUpdater` to call `IWLOUpdater`
3. [ ] Modify `Tile` to call `IWLOUpdater`
4. [ ] Verify with unit tests

### Phase 2: Storage Abstraction

1. [ ] Define `VoxelStorage` interface
2. [ ] Implement `OctreeStorage` (based on existing OctreeMapper)
3. [ ] Refactor `Tile` to use `OctreeStorage`

### Phase 3: Backend Unification

1. [ ] Implement `RamBackend` (replaces ProbabilityUpdater)
2. [ ] Implement `DiskBackend` (refactors OutofcoreTileMapper)
3. [ ] Update Python bindings
4. [ ] Integration tests

### Phase 4: Cleanup

1. [ ] Remove legacy duplicated code
2. [ ] Update documentation
3. [ ] Performance benchmarks

---

## Benefits

1. **No code duplication**: IWLO logic exists in one place only
2. **Easy maintenance**: Algorithm changes require single update
3. **Test efficiency**: Shared component tests cover both modes
4. **Extensibility**: Easy to add new storage backends (e.g., distributed)
5. **Readability**: Clear separation of concerns

---

## Notes

- Maintain existing API compatibility during refactoring (minimize Python changes)
- No performance regression (benchmarking required)
- Incremental refactoring recommended (avoid big-bang changes)

---

**Created**: 2024-12-24
**Status**: Planning
