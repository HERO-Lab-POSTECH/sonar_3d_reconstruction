# Troubleshooting

## Issue Log

### [2025-11-27] C++ Backend Probability Fixed Value Bug

**Category**: Runtime/Performance

**Situation**: All voxel probabilities fixed at 0.881 when using C++ backend

**Cause**:
- OctoMap's `getOccupancy()` function returning internal fixed value
- Actual log-odds values not being stored
- Probability calculation dependent on OctoMap internal logic

**Solution**:
1. Implemented direct log-odds storage system
2. Added `batch_update_with_log_odds` function
3. Applied same probability calculation logic as Python
4. Set accurate probability thresholds

**Modified Files**:
- `octree_mapper.h`
- `octree_mapper.cpp`
- `probability_updater.cpp`
- `python_bindings.cpp`

**Result**:
- C++ backend now produces identical results to Python
- Various probability distributions calculated correctly
- Point count optimized: 33,127 → 202
- Improved calculation accuracy

**Notes**:
- Probability calculation is log-odds based
- Threshold: ≥0.5 occupied, <0.5 free space
