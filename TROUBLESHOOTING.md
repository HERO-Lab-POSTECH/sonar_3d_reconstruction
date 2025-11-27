## 문제 기록

### [2025-11-27] C++ 백엔드 확률 고정 버그

**Category**: 런타임/성능

**상황**: C++ 백엔드에서 모든 voxel의 확률이 0.881로 고정되는 문제 발생

**원인**:
- OctoMap의 getOccupancy() 함수가 내부 고정값 반환
- 실제 log-odds 값 미저장
- 확률 계산이 OctoMap 내부 로직에 의존

**솔루션**:
1. Log-odds 직접 저장 시스템 구현
2. `batch_update_with_log_odds` 함수 추가
3. Python과 동일한 확률 계산 로직 적용
4. 정확한 probability threshold 설정

**수정된 파일**:
- `octree_mapper.h`
- `octree_mapper.cpp`
- `probability_updater.cpp`
- `python_bindings.cpp`

**결과**:
- C++ 백엔드가 Python과 동일한 결과 생성
- 다양한 확률 분포 정상 계산
- 점 생성 수: 33,127 → 202개로 최적화
- 계산 정확도 향상

**참고**:
- 확률 계산은 log-odds 기반으로 진행
- Threshold: 0.5 이상 점유, 0.5 미만 자유 공간