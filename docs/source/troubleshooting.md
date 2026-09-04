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

---

### [2025-12-26] Fast-LIO Long-Running Session Time Sync Issue

**Category**: Sensor Fusion/Odometry

**Situation**: Fast-LIO를 장시간 실행한 상태에서 여러 번 데이터를 수집했을 때, LiDAR와 IMU 간 time synchronization이 맞지 않는 문제 발생

**Background**:
- 목표: Prior map (로봇 없음) vs Current map (로봇 있음) 비교를 통한 로봇 검출
- 소나 이미지만으로는 로봇 식별이 거의 불가능
- Fast-LIO의 initial pose 일관성 유지를 위해 세션을 계속 유지한 채 데이터 수집 시도

**Cause**:
- Fast-LIO 장시간 실행 시 데이터 누적으로 인한 처리 지연
- Buffer overflow 또는 처리 속도 저하로 센서 데이터 동기화 실패
- Initial pose 일관성 유지 목적과 실제 시스템 설계 의도의 불일치

**Solution**:
1. Fast-LIO는 **세션 단위로 실행** (매핑 시작 시 새로 시작)
2. 각 세션에서 독립적으로 3D point cloud map 생성
3. **ICP/GICP 기반 point cloud registration**으로 두 맵 정합
4. 정합된 맵에서 차이점 검출로 로봇 위치 파악

**Proposed Workflow**:
```
Session 1: Fast-LIO 시작 → map_prior.pcd 저장 (로봇 없음)
Session 2: Fast-LIO 시작 → map_current.pcd 저장 (로봇 있음)
Post-processing:
  1. Global registration (FPFH + RANSAC) → coarse alignment
  2. ICP/GICP refinement → precise transform T
  3. Transform 적용 후 두 맵 차이 검출 → 로봇 위치
```

**Registration Method Options**:
| Method | Pros | Cons | Use Case |
|--------|------|------|----------|
| ICP | High accuracy | Needs initial estimate | Feature-rich environment |
| NDT | Robust to noise | Computationally heavy | Noisy data |
| GICP | Combines ICP+NDT benefits | - | General purpose |
| Global (FPFH+RANSAC) | No initial estimate needed | Slow | Unknown initial pose |

**Considerations**:
- 도수로 환경의 기하학적 특징(벽, 구조물) 충분성 검토 필요
- 특징 부족 시 고정 마커 또는 랜드마크 활용 고려
- Point cloud 해상도와 registration 정확도 간 trade-off

**Status**: Proposed solution - 구현 및 검증 예정 (2026-01-05까지)

**Related Tasks**:
- 3D point cloud map ICP 정합 코드 구현
- Prior/Current map 비교 알고리즘 개발
- 도수로 환경 특성 분석
