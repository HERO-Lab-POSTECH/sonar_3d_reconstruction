# CHANGELOG - sonar_3d_reconstruction

## [2025-12-24]

### Added
- **IWLOUpdater 클래스 신규 생성** (Phase 1 리팩토링)
  - `cpp/iwlo_updater.h`, `cpp/iwlo_updater.cpp` 신규 생성
  - IWLO 알고리즘을 stateless static methods로 분리
  - `intensity_to_weight()`, `compute_alpha()`, `log_odds_to_probability()` 등 공유 함수 제공
  - `compute_delta_log_odds()`: 전체 IWLO 업데이트 공식 캡슐화

### Changed
- **ProbabilityUpdater 리팩토링**
  - `probability_updater.cpp`: 중복 IWLO 로직 → `IWLOUpdater` 호출로 대체
  - `intensity_to_weight()`, `compute_alpha()` 함수 단순화 (13줄 → 3줄)

- **Tile 클래스 리팩토링**
  - `tile.cpp`: 중복 IWLO 로직 → `IWLOUpdater` 호출로 대체
  - `log_odds_to_probability()`, `intensity_to_weight()`, `compute_alpha()` 함수 단순화

- **CMakeLists.txt 업데이트**
  - `iwlo_updater.cpp` 빌드 대상에 추가

### Removed
- **미사용 coordinate_transform 코드 제거**
  - `coordinate_transform.cpp`, `coordinate_transform.h` 삭제
  - CMakeLists.txt에 빌드 설정 없었고, Python에서 import 안됨
  - 좌표 변환은 `3d_mapper.py`에서 직접 처리

---

## [2025-12-04]

### Added
- **IWLO (Intensity-Weighted Log-Odds) 확률 업데이트 방법**
  - Log-Odds Bayesian과 Weighted Average 방식 융합
  - 강도 기반 시그모이드 가중치 변환
  - 관측 횟수 기반 학습률 감쇠
  - Saturation 방지를 위한 log-odds 범위 제한
  - 파일: `config/method_iwlo.yaml`, `scripts/3d_mapper.py`, `cpp/probability_updater.cpp`

- **Cross-talk 노이즈 필터**
  - 멀티빔 소나의 가로 줄무늬 노이즈 억제
  - Morphological Opening 필터 (세로 커널)
  - 방위각 일관성 검사 (Azimuth Consistency Check)
  - 파일: `config/crosstalk_filter.yaml`, `scripts/3d_mapper.py`

### Changed
- **C++ 백엔드 개선**
  - OctoMap 라이브러리 링크 수정 (CMakeLists.txt)
  - PYTHONPATH 환경 후크 추가 (env-hooks/)
  - CMakeLists.txt 최적화

### System Status
- Build Status: ✅ SUCCESS
- Build Time: 0.30s
- Test Result: ✅ All tests passed
- C++ Module: ✅ Properly loaded and functional
- Memory Efficiency: 0.56배율, 400개 노드 정상 처리
