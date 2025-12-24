# iwlo_updater.cpp / .h

C++ stateless utility class for IWLO probability calculations.

**Location**: `sonar_3d_reconstruction/cpp/iwlo_updater.h`, `iwlo_updater.cpp`

---

## Overview

`IWLOUpdater` provides stateless static methods for the IWLO (Intensity-Weighted Log-Odds) algorithm. It eliminates code duplication between `ProbabilityUpdater` and `Tile` classes by centralizing the core probability update logic.

**Key Features**:
- Stateless design: All methods are static
- Code deduplication: Single source of truth for IWLO calculations
- Flexible API: Supports both explicit parameters and IWLOParams struct

---

## Class: IWLOUpdater

### Purpose

Provides core algorithms for:
- Intensity to weight conversion (sigmoid transformation)
- Learning rate decay calculation
- Log-odds to probability conversion
- Delta log-odds computation for voxel updates

---

## Core Conversion Methods

### log_odds_to_probability

```cpp
static double log_odds_to_probability(double log_odds);
```

Convert log-odds value to probability.

| Parameter | Type | Description |
|-----------|------|-------------|
| `log_odds` | double | Log-odds value |
| **Returns** | double | Probability in range [0, 1] |

**Formula**: $P = \frac{1}{1 + e^{-L}}$

---

### probability_to_log_odds

```cpp
static double probability_to_log_odds(double probability);
```

Convert probability to log-odds value.

| Parameter | Type | Description |
|-----------|------|-------------|
| `probability` | double | Probability in range (0, 1) |
| **Returns** | double | Log-odds value |

**Formula**: $L = \log\left(\frac{P}{1-P}\right)$

---

## IWLO Algorithm Methods

### intensity_to_weight (explicit parameters)

```cpp
static double intensity_to_weight(double intensity,
                                  double intensity_threshold,
                                  double intensity_max,
                                  double sharpness);
```

Convert raw intensity to weight using sigmoid transformation.

| Parameter | Type | Description |
|-----------|------|-------------|
| `intensity` | double | Raw intensity value |
| `intensity_threshold` | double | Minimum intensity threshold |
| `intensity_max` | double | Maximum intensity value |
| `sharpness` | double | Sigmoid curve sharpness |
| **Returns** | double | Weight in range [0, 1] |

**Algorithm**:
1. Threshold filtering: `intensity <= threshold` → return 0
2. Normalization: `[threshold, max]` → `[0, 1]`
3. Sigmoid: centered at 0.5, slope controlled by sharpness

---

### intensity_to_weight (with IWLOParams)

```cpp
static double intensity_to_weight(double intensity, const IWLOParams& params);
```

Convenience wrapper using IWLOParams struct.

---

### compute_alpha (explicit parameters)

```cpp
static double compute_alpha(int observation_count,
                            double decay_rate,
                            double min_alpha);
```

Compute learning rate with decay based on observation count.

| Parameter | Type | Description |
|-----------|------|-------------|
| `observation_count` | int | Number of observations for this voxel |
| `decay_rate` | double | Rate of learning rate decay |
| `min_alpha` | double | Minimum learning rate floor |
| **Returns** | double | Learning rate in range [min_alpha, 1] |

**Formula**: $\alpha(n) = \max\left(\alpha_{min}, \frac{1}{1 + \lambda \cdot n}\right)$

---

### compute_alpha (with IWLOParams)

```cpp
static double compute_alpha(int observation_count, const IWLOParams& params);
```

Convenience wrapper using IWLOParams struct.

---

### compute_adaptive_scale

```cpp
static double compute_adaptive_scale(double current_probability,
                                     const IWLOParams& params);
```

Compute adaptive scaling factor for noise reduction.

| Parameter | Type | Description |
|-----------|------|-------------|
| `current_probability` | double | Current voxel probability |
| `params` | IWLOParams | IWLO parameters struct |
| **Returns** | double | Scaling factor in range [adaptive_max_ratio, 1] |

**Behavior**:
- Reduces update magnitude for high-confidence voxels
- If probability is near 0 or 1: apply suppression (max_ratio scaling)
- Otherwise: full update (scale = 1.0)

---

### compute_delta_log_odds

```cpp
static double compute_delta_log_odds(double intensity,
                                     double current_log_odds,
                                     int observation_count,
                                     const IWLOParams& params);
```

Compute the complete delta log-odds for a voxel update.

| Parameter | Type | Description |
|-----------|------|-------------|
| `intensity` | double | Raw intensity value |
| `current_log_odds` | double | Current log-odds of the voxel |
| `observation_count` | int | Number of previous observations |
| `params` | IWLOParams | IWLO parameters struct |
| **Returns** | double | Delta log-odds to add to current value |

**IWLO Update Formula**:
$$\Delta L = \alpha \times scale \times (w \times L_{occ} + (1 - w) \times L_{free})$$

This is the main entry point that combines:
1. Intensity-to-weight conversion
2. Learning rate with decay
3. Adaptive scaling
4. Occupied/free log-odds weighting

---

### clamp_log_odds

```cpp
static double clamp_log_odds(double log_odds, double L_min, double L_max);
```

Clamp log-odds to prevent saturation.

| Parameter | Type | Description |
|-----------|------|-------------|
| `log_odds` | double | Log-odds value to clamp |
| `L_min` | double | Minimum log-odds (default: -10.0) |
| `L_max` | double | Maximum log-odds (default: 10.0) |
| **Returns** | double | Clamped log-odds |

---

## Usage Example

```cpp
#include "iwlo_updater.h"
#include "tile.h"  // For IWLOParams

// Create parameters
IWLOParams params;
params.intensity_threshold = 35.0;
params.intensity_max = 255.0;
params.sharpness = 0.1;
params.decay_rate = 0.1;
params.min_alpha = 0.3;
params.L_occ = 3.5;
params.L_free = -3.0;
params.L_min = -10.0;
params.L_max = 10.0;

// Compute delta log-odds for a voxel update
double intensity = 150.0;
double current_log_odds = 0.0;
int observation_count = 5;

double delta = IWLOUpdater::compute_delta_log_odds(
    intensity, current_log_odds, observation_count, params);

// Apply update with clamping
double new_log_odds = IWLOUpdater::clamp_log_odds(
    current_log_odds + delta, params.L_min, params.L_max);

// Convert to probability
double probability = IWLOUpdater::log_odds_to_probability(new_log_odds);
```

---

## Design Rationale

**Why static methods?**
- IWLO calculations are pure functions with no state
- Enables code reuse without instantiation overhead
- Clear separation of algorithm from storage

**Users of IWLOUpdater**:
- `ProbabilityUpdater::batch_update_iwlo()` - In-Memory mode
- `Tile::batch_update()` - Out-of-Core mode

---

## Related

- [IWLO Design](../design/iwlo_design.md) - Algorithm mathematical formulation
- [probability_updater.md](probability_updater.md) - In-Memory mapper using IWLOUpdater
- [tile.md](tile.md) - Tile class using IWLOUpdater
