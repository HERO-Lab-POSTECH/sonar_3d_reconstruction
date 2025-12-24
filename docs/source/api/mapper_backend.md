# mapper_backend.h

C++ abstract interface for 3D mapper backends.

**Location**: `sonar_3d_reconstruction/cpp/mapper_backend.h`

---

## Overview

`IMapperBackend` is an abstract interface that defines the common API for 3D occupancy mapping backends. It enables switching between In-Memory (RAM) and Out-of-Core (Disk) implementations without changing client code.

**Key Features**:
- Unified API for RAM and Disk backends
- IWLO (Intensity-Weighted Log-Odds) update support
- Parameter configuration methods
- Query and maintenance operations

---

## Class: IMapperBackend

### Purpose

Defines the common contract for:
- IWLO batch updates
- Parameter configuration (IWLO, intensity, adaptive)
- Voxel queries
- Resource management

---

## Implementations

| Implementation | Backend Type | Description |
|----------------|--------------|-------------|
| [ProbabilityUpdater](probability_updater.md) | `"RAM"` | In-Memory mapping for small maps |
| [OutofcoreTileMapper](outofcore_tile_mapper.md) | `"Disk"` | Disk-based tile storage for large maps |

---

## Required API

### Core Update

#### batch_update_iwlo

```cpp
virtual void batch_update_iwlo(
    const Eigen::MatrixXd& points,
    const Eigen::VectorXd& intensities,
    const std::vector<bool>& is_occupied) = 0;
```

Main entry point for probability updates using IWLO algorithm.

| Parameter | Shape | Description |
|-----------|-------|-------------|
| `points` | Nx3 | Point coordinates (x, y, z) |
| `intensities` | N | Intensity values per point |
| `is_occupied` | N | Boolean flags (used for intensity threshold) |

---

### Parameter Configuration

#### set_iwlo_params

```cpp
virtual void set_iwlo_params(double sharpness, double decay_rate, double min_alpha,
                              double L_min, double L_max) = 0;
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sharpness` | 0.1 | Sigmoid steepness for intensity-to-weight |
| `decay_rate` | 0.1 | Learning rate decay rate |
| `min_alpha` | 0.3 | Minimum learning rate |
| `L_min` | -10.0 | Saturation lower bound |
| `L_max` | 10.0 | Saturation upper bound |

---

#### set_log_odds_params

```cpp
virtual void set_log_odds_params(double log_odds_occupied, double log_odds_free) = 0;
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `log_odds_occupied` | 3.5 | Log-odds increment for occupied |
| `log_odds_free` | -3.0 | Log-odds decrement for free |

---

#### set_intensity_params

```cpp
virtual void set_intensity_params(double intensity_threshold, double intensity_max) = 0;
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `intensity_threshold` | 35.0 | Minimum intensity to consider |
| `intensity_max` | 255.0 | Maximum intensity value |

---

#### set_adaptive_params

```cpp
virtual void set_adaptive_params(bool enabled, double threshold, double max_ratio) = 0;
```

| Parameter | Default | Description |
|-----------|---------|-------------|
| `enabled` | true | Enable adaptive updating |
| `threshold` | 0.5 | Threshold for adaptive behavior |
| `max_ratio` | 0.3 | Maximum update ratio |

---

#### set_occupied_threshold

```cpp
virtual void set_occupied_threshold(double threshold) = 0;
```

Set probability threshold for occupied classification (default: 0.7).

---

### Query API

#### get_occupied_voxels

```cpp
virtual Eigen::MatrixXd get_occupied_voxels(double min_probability = 0.5) = 0;
```

Returns Nx4 matrix `[x, y, z, probability]` of voxels above threshold.

---

#### get_memory_usage

```cpp
virtual MemoryStats get_memory_usage() const = 0;
```

Returns memory statistics.

---

#### get_resolution

```cpp
virtual double get_resolution() const = 0;
```

Returns voxel resolution in meters.

---

#### get_num_nodes

```cpp
virtual size_t get_num_nodes() const = 0;
```

Returns total number of nodes.

---

#### clear

```cpp
virtual void clear() = 0;
```

Clear all data.

---

## Optional API

These methods have default implementations (no-op or return 0/false).

### flush

```cpp
virtual void flush() {}
```

Flush data to persistent storage. RAM backend: no-op.

---

### preload_region

```cpp
virtual void preload_region(const Eigen::Vector3d& min_bound,
                             const Eigen::Vector3d& max_bound) {}
```

Preload tiles in a region into cache. RAM backend: no-op.

---

### get_disk_usage

```cpp
virtual size_t get_disk_usage() const { return 0; }
```

Get disk usage in bytes. RAM backend: returns 0.

---

### prune

```cpp
virtual size_t prune() { return 0; }
```

Prune tree to reduce memory. Returns nodes removed.

---

### supports_persistence

```cpp
virtual bool supports_persistence() const { return false; }
```

Check if backend supports persistent storage.
- RAM backend: `false`
- Disk backend: `true`

---

### get_backend_type

```cpp
virtual std::string get_backend_type() const = 0;
```

Get backend type name.
- RAM backend: `"RAM"`
- Disk backend: `"Disk"`

---

## Usage Example

```cpp
#include "probability_updater.h"
#include "outofcore_tile_mapper.h"

// Choose backend based on map size
std::unique_ptr<IMapperBackend> backend;

if (use_large_map) {
    backend = std::make_unique<OutofcoreTileMapper>("/path/to/tiles", 0.05, 10.0, 16);
} else {
    backend = std::make_unique<ProbabilityUpdater>(0.05);
}

// Configure parameters (same API for both backends)
backend->set_iwlo_params(0.1, 0.1, 0.3, -10.0, 10.0);
backend->set_intensity_params(35.0, 255.0);

// Update with sonar data
Eigen::MatrixXd points(1000, 3);
Eigen::VectorXd intensities(1000);
std::vector<bool> is_occupied(1000);
// ... fill data ...

backend->batch_update_iwlo(points, intensities, is_occupied);

// Query results
auto occupied = backend->get_occupied_voxels(0.7);

// Optional: flush if using disk backend
if (backend->supports_persistence()) {
    backend->flush();
}

std::cout << "Backend type: " << backend->get_backend_type() << std::endl;
```

---

## Backend Comparison

| Feature | ProbabilityUpdater | OutofcoreTileMapper |
|---------|-------------------|---------------------|
| Backend Type | "RAM" | "Disk" |
| Storage | RAM only | Disk + RAM cache |
| Map Size | Limited by RAM | Unlimited |
| Speed | Faster | Slower (I/O) |
| supports_persistence() | false | true |
| flush() | no-op | saves tiles |
| preload_region() | no-op | loads tiles |
| get_disk_usage() | 0 | actual usage |

---

## Design Rationale

**Why a common interface?**
- Enables runtime backend selection based on map size
- Facilitates testing with mock implementations
- Provides consistent API for Python bindings

**Required vs Optional methods**:
- Required: Core functionality all backends must support
- Optional: Persistence features only applicable to Disk backend

---

## Related

- [ProbabilityUpdater](probability_updater.md) - RAM backend implementation
- [OutofcoreTileMapper](outofcore_tile_mapper.md) - Disk backend implementation
- [IWLO Design](../design/iwlo_design.md) - Algorithm details
