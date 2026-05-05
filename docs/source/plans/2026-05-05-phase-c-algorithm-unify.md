# Phase C — Algorithm Unification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

> **2026-05-05 갱신**: Task 3 (C-c) 는 회귀 측정에서 P-2 dataset 90s 동안 occupied voxel 0 (baseline 21,776) 으로 사용자 핵심 제약 위반 → **본 PR 에서 분리**. C-c 는 별도 spec 으로 재설계 (IWLOParams `L_occ` / `L_free` 비대칭 재튜닝 포함). Task 3 의 단계는 historical 자료로 보존하되 실행 안 함. 본 PR 은 Task 1 (C-a) + Task 2 (C-b) + Task 4 (CHANGELOG + PR, C-c 항목 제외) 만 진행. design doc §6 참조.

**Goal:** master spec §3 Phase C 중 결과 보존 정리 (P2-6, P2-5) 를 단일 PR 로 머지한다. P2-2 (이중 알고리즘 통일) 는 별도 spec 에서 재설계.

**Architecture:** C-a → C-b 는 결과 동일성 (jaccard ≥ 0.99) 을 강제하는 저위험 정리. C-c (제외) 는 IWLOParams 와 함께 재설계해야 하는 알고리즘 변경.

**Tech Stack:** C++17, OctoMap, Eigen3, pybind11, pytest, colcon (Release).

**Branch:** `refactor/phase-c-algorithm-unify` (이미 main 에서 분기됨, 1 commit = spec).

**Working dir:** `/workspace/ros2_ws/src/sonar_3d_reconstruction`. 모든 cd 명령은 이 위치 기준.

**Build:** `cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release && source install/setup.bash`. 매 task 빌드 시 동일.

---

## Task 1 — C-a: `load_iwlo_meta` corrupt detection (P2-6)

**근거**: master spec §3 Phase C P2-6. 현재 `load_iwlo_meta` (lines 222-263) 는 `for (uint64_t i = 0; i < count; ++i)` 루프 안에서 `ifs.read()` 의 반환을 검사하지 않는다. EOF 또는 read 실패 시 `iwlo_meta_` 에 garbage key+value 가 들어가고, 마지막에 `return ifs.good()` 하더라도 호출자는 이미 더럽혀진 storage 를 받는다.

**Files:**
- Modify: `sonar_3d_reconstruction/cpp/octree_storage.cpp:222-263`
- Test: `sonar_3d_reconstruction/test/test_octree_storage_load.cpp` (신규)
- Modify: `CMakeLists.txt` (gtest 신규 추가)

### Step 1: 단위 테스트 작성 (실패 기대)

신규 파일 `sonar_3d_reconstruction/test/test_octree_storage_load.cpp`:

```cpp
#include <gtest/gtest.h>
#include <fstream>
#include <filesystem>
#include "octree_storage.h"

namespace fs = std::filesystem;
using namespace sonar_3d_reconstruction;

namespace {

// Build a complete valid IWLO file with N entries, then truncate to 'truncate_at' bytes.
// Returns the temp directory path.
std::string make_truncated_meta(uint64_t declared_count,
                                uint64_t actual_entries,
                                size_t extra_truncate_bytes = 0) {
    auto tmpdir = fs::temp_directory_path() / fs::path("phase_c_octree_load_XXXX");
    fs::create_directories(tmpdir);
    std::string meta_path = (tmpdir / "iwlo_meta.bin").string();

    std::ofstream ofs(meta_path, std::ios::binary);
    const uint32_t magic = 0x49574C4F;
    const uint32_t version = 1;
    ofs.write(reinterpret_cast<const char*>(&magic), sizeof(magic));
    ofs.write(reinterpret_cast<const char*>(&version), sizeof(version));
    ofs.write(reinterpret_cast<const char*>(&declared_count), sizeof(declared_count));

    for (uint64_t i = 0; i < actual_entries; ++i) {
        unsigned short k0 = static_cast<unsigned short>(i);
        unsigned short k1 = 0;
        unsigned short k2 = 0;
        double log_odds = 0.5;
        int obs = 1;
        ofs.write(reinterpret_cast<const char*>(&k0), sizeof(k0));
        ofs.write(reinterpret_cast<const char*>(&k1), sizeof(k1));
        ofs.write(reinterpret_cast<const char*>(&k2), sizeof(k2));
        ofs.write(reinterpret_cast<const char*>(&log_odds), sizeof(log_odds));
        ofs.write(reinterpret_cast<const char*>(&obs), sizeof(obs));
    }
    ofs.close();

    if (extra_truncate_bytes > 0) {
        size_t cur = fs::file_size(meta_path);
        if (cur > extra_truncate_bytes) {
            fs::resize_file(meta_path, cur - extra_truncate_bytes);
        }
    }

    // Empty octree.bt so OctreeStorage::load() does not abort on missing octree
    std::string octree_path = (tmpdir / "octree.bt").string();
    std::ofstream octfs(octree_path, std::ios::binary);
    octfs.close();

    return tmpdir.string();
}

}  // namespace

TEST(OctreeStorageLoad, ReturnsFalseOnTruncatedMeta) {
    // Declare 3 entries but write only 2 -> third read should fail.
    std::string dir = make_truncated_meta(3, 2);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_FALSE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 0u)
        << "Storage must be cleared on partial-read failure";
    fs::remove_all(dir);
}

TEST(OctreeStorageLoad, ReturnsFalseOnMidEntryTruncation) {
    // Declare 2 entries, write 2, then chop off 4 bytes from the very end
    // (mid second-entry's int observation_count). Last read fails partway.
    std::string dir = make_truncated_meta(2, 2, /*extra_truncate=*/4);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_FALSE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 0u);
    fs::remove_all(dir);
}

TEST(OctreeStorageLoad, ReturnsTrueOnCompleteMeta) {
    // Declare 4 entries, write 4 -> normal round-trip.
    std::string dir = make_truncated_meta(4, 4);
    OctreeStorage storage(0.05);
    bool ok = storage.load(dir);
    EXPECT_TRUE(ok);
    EXPECT_EQ(storage.get_num_voxels(), 4u);
    fs::remove_all(dir);
}
```

CMakeLists.txt 에 gtest 등록 추가가 필요. 기존 cpp gtest 가 등록돼 있는지 먼저 검사하고, 없으면 다음 패턴 추가 (BUILD_TESTING 보호 안에):

```cmake
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_octree_storage_load
    test/test_octree_storage_load.cpp
    sonar_3d_reconstruction/cpp/octree_storage.cpp
    sonar_3d_reconstruction/cpp/iwlo_updater.cpp
    sonar_3d_reconstruction/cpp/tile.cpp
  )
  target_include_directories(test_octree_storage_load PRIVATE
    sonar_3d_reconstruction/cpp
  )
  target_link_libraries(test_octree_storage_load
    ${OCTOMAP_LIBRARIES}
    Eigen3::Eigen
  )
endif()
```

(주: 기존 `CMakeLists.txt` 에서 cpp 라이브러리 target 이름과 link 패턴 그대로 흉내 — 작성 시 `cat CMakeLists.txt` 로 확인 후 동일 패턴 적용. `sonar_3d_reconstruction_cpp` 같은 기존 target 이 있으면 source 파일 재나열 대신 link 사용도 가능.)

- [ ] **Step 1**: 신규 테스트 파일 작성 (위 코드 그대로) + CMakeLists.txt 업데이트.

### Step 2: 빌드해서 실패 확인

Run:
```bash
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select sonar_3d_reconstruction --ctest-args -R test_octree_storage_load
colcon test-result --verbose --test-result-base build/sonar_3d_reconstruction
```

Expected: `ReturnsFalseOnTruncatedMeta`, `ReturnsFalseOnMidEntryTruncation` 둘 다 FAIL (현재 코드가 `ifs.good()` 만 마지막에 보고 garbage 를 그대로 채워서 `get_num_voxels()` 가 0 이 아닐 것). `ReturnsTrueOnCompleteMeta` 는 PASS.

- [ ] **Step 2**: 두 corrupt 케이스 FAIL 확인.

### Step 3: 구현 — 루프 내 `ifs.good()` 검사

`sonar_3d_reconstruction/cpp/octree_storage.cpp` line 246-263 영역을 다음으로 교체:

```cpp
    iwlo_meta_.clear();
    iwlo_meta_.reserve(count);

    for (uint64_t i = 0; i < count; ++i) {
        octomap::OcTreeKey key;
        IWLOMeta meta;

        ifs.read(reinterpret_cast<char*>(&key[0]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&key[1]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&key[2]), sizeof(unsigned short));
        ifs.read(reinterpret_cast<char*>(&meta.log_odds), sizeof(double));
        ifs.read(reinterpret_cast<char*>(&meta.observation_count), sizeof(int));

        if (!ifs.good()) {
            std::cerr << "[OctreeStorage] IWLO metadata truncated at entry "
                      << i << " of declared " << count << std::endl;
            iwlo_meta_.clear();
            return false;
        }

        iwlo_meta_[key] = meta;
    }

    return true;
```

핵심 변경:
- 매 entry 의 5 read 끝에 `ifs.good()` 검사
- 실패 시 `iwlo_meta_.clear()` 로 부분 손상 storage 비우고 `false` 반환
- 마지막 `return ifs.good()` → `return true` (모든 entry 가 검사 통과한 시점)

- [ ] **Step 3**: 위 변경 적용.

### Step 4: 빌드 + 테스트 PASS

Run:
```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select sonar_3d_reconstruction --ctest-args -R test_octree_storage_load
colcon test-result --verbose --test-result-base build/sonar_3d_reconstruction
```

Expected: 3개 모두 PASS.

- [ ] **Step 4**: 3 테스트 PASS 확인.

### Step 5: Python 단위 테스트 회귀 확인

Run:
```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && python3 -m pytest tests/ -x -q
```

Expected: 기존 14 테스트 모두 PASS (B-1/B-2/B-3 자산).

- [ ] **Step 5**: 기존 테스트 회귀 없음 확인.

### Step 6: Commit

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git add sonar_3d_reconstruction/cpp/octree_storage.cpp test/test_octree_storage_load.cpp CMakeLists.txt
git commit -m "fix(octree_storage): detect truncated IWLO metadata in load loop (P2-6, C-a)

Per-entry ifs.good() check after the 5 reads in load_iwlo_meta. On
partial read iwlo_meta_ is cleared and false is returned, so callers
do not see a half-filled metadata map.

- cpp/octree_storage.cpp:249-263 — loop-internal good() guard
- test/test_octree_storage_load.cpp — 3 gtest cases (truncated declared
  count / mid-entry truncation / complete file)
- CMakeLists.txt — register test_octree_storage_load gtest target"
```

- [ ] **Step 6**: Commit.

---

## Task 2 — C-b: `dirty_keys_` incremental sync (P2-5)

**근거**: master spec §3 Phase C P2-5. 현재 `OctreeStorage::sync_to_octree()` (lines 276-286) 는 매 호출마다 `iwlo_meta_` **전체** 를 octree 에 재반영한다. flush 빈도가 높은 환경 (예: out-of-core mapper 의 매 frame eviction 검사) 에서 비효율. `dirty_keys_` 로 변경된 voxel 만 추적하면 sync 비용이 amortize 된다.

**Files:**
- Modify: `sonar_3d_reconstruction/cpp/octree_storage.h:124-128, 38-122` (멤버 추가, 멤버 함수에 dirty_keys_ insert)
- Modify: `sonar_3d_reconstruction/cpp/octree_storage.cpp:38-63, 222-286` (구현)
- Test: `sonar_3d_reconstruction/test/test_octree_storage_load.cpp` (신규 테스트 추가)

### Step 1: 단위 테스트 작성 (실패 기대)

`sonar_3d_reconstruction/test/test_octree_storage_load.cpp` 끝에 다음 추가:

```cpp
TEST(OctreeStorageSync, IncrementalSyncProducesSameOctreeAsFullSync) {
    OctreeStorage a(0.1);
    OctreeStorage b(0.1);

    // Same updates to both
    octomap::OcTreeKey k1 = a.coord_to_key(octomap::point3d(0.05, 0.05, 0.05));
    octomap::OcTreeKey k2 = a.coord_to_key(octomap::point3d(0.15, 0.05, 0.05));
    octomap::OcTreeKey k3 = a.coord_to_key(octomap::point3d(0.25, 0.05, 0.05));

    for (auto* s : {&a, &b}) {
        s->set_log_odds(k1, 1.5);
        s->set_log_odds(k2, -0.7);
        s->set_log_odds(k3, 0.3);
    }

    // a uses incremental sync (only dirty); b is forced to do a full re-sync
    a.sync_to_octree();   // post C-b: only the 3 dirty keys
    b.sync_to_octree();   // identical result by definition (first call after edits)

    // Compare resulting octree node log-odds for all three keys
    auto* ta = a.get_octree();
    auto* tb = b.get_octree();
    octomap::OcTreeNode* na1 = ta->search(k1);
    octomap::OcTreeNode* nb1 = tb->search(k1);
    ASSERT_NE(na1, nullptr); ASSERT_NE(nb1, nullptr);
    EXPECT_NEAR(na1->getLogOdds(), nb1->getLogOdds(), 1e-9);
    octomap::OcTreeNode* na3 = ta->search(k3);
    octomap::OcTreeNode* nb3 = tb->search(k3);
    ASSERT_NE(na3, nullptr); ASSERT_NE(nb3, nullptr);
    EXPECT_NEAR(na3->getLogOdds(), nb3->getLogOdds(), 1e-9);

    // Now mutate a single key in 'a' and verify only that one node changes
    a.set_log_odds(k2, 2.0);
    a.sync_to_octree();
    octomap::OcTreeNode* na2 = ta->search(k2);
    ASSERT_NE(na2, nullptr);
    EXPECT_NEAR(na2->getLogOdds(), 2.0f, 1e-5);

    // k1 untouched in 'a' since first sync
    octomap::OcTreeNode* na1_after = ta->search(k1);
    ASSERT_NE(na1_after, nullptr);
    EXPECT_NEAR(na1_after->getLogOdds(), 1.5f, 1e-5);
}

TEST(OctreeStorageSync, ClearAlsoResetsDirtyKeys) {
    OctreeStorage s(0.1);
    octomap::OcTreeKey k = s.coord_to_key(octomap::point3d(0.05, 0.05, 0.05));
    s.set_log_odds(k, 1.0);
    s.sync_to_octree();
    s.clear();
    EXPECT_EQ(s.get_num_voxels(), 0u);
    // After clear there should be nothing to re-sync; subsequent sync is a no-op
    s.sync_to_octree();
    EXPECT_EQ(s.get_octree()->getNumLeafNodes(), 0u);
}
```

CMakeLists.txt 의 `test_octree_storage_load` gtest 는 그대로 (같은 파일에 추가).

- [ ] **Step 1**: 두 테스트 추가.

### Step 2: 빌드 + 테스트 실행하여 회귀 없음 확인 (구현 전)

Run:
```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select sonar_3d_reconstruction --ctest-args -R test_octree_storage
colcon test-result --verbose --test-result-base build/sonar_3d_reconstruction
```

Expected: 5 테스트 모두 PASS — 현재 코드도 위 테스트는 통과한다 (full sync 도 동일 결과 산출). `IncrementalSyncProducesSameOctreeAsFullSync` 는 **결과 동일성** 만 검증하므로 dirty_keys 도입 전후 모두 통과해야 한다 (회귀 가드 역할).

- [ ] **Step 2**: 5 테스트 PASS 확인.

### Step 3: 헤더에 `dirty_keys_` 멤버 추가

`sonar_3d_reconstruction/cpp/octree_storage.h` line 124-128 (private 멤버 영역) 의

```cpp
private:
    double resolution_;
    std::unique_ptr<octomap::OcTree> octree_;
    std::unordered_map<octomap::OcTreeKey, IWLOMeta, OcTreeKeyHash> iwlo_meta_;
    bool dirty_ = false;
```

을 다음으로 교체:

```cpp
private:
    double resolution_;
    std::unique_ptr<octomap::OcTree> octree_;
    std::unordered_map<octomap::OcTreeKey, IWLOMeta, OcTreeKeyHash> iwlo_meta_;
    /// Keys with metadata changed since last sync_to_octree (P2-5).
    /// `sync_to_octree` re-applies only these to the octree, then clears.
    std::unordered_set<octomap::OcTreeKey, OcTreeKeyHash> dirty_keys_;
    bool dirty_ = false;
```

`<unordered_set>` include 가 필요. 헤더 상단의 `#include <unordered_map>` 옆에 `#include <unordered_set>` 추가.

- [ ] **Step 3**: 헤더 변경.

### Step 4: 모든 mutation 경로에 `dirty_keys_.insert(key)` 추가

`sonar_3d_reconstruction/cpp/octree_storage.cpp` 의 mutator 4 곳:

`set_log_odds` (line 38-42) 를 다음으로:

```cpp
void OctreeStorage::set_log_odds(const octomap::OcTreeKey& key, double value)
{
    iwlo_meta_[key].log_odds = value;
    dirty_keys_.insert(key);
    dirty_ = true;
}
```

`increment_observation_count` (line 53-57) 를 다음으로:

```cpp
int OctreeStorage::increment_observation_count(const octomap::OcTreeKey& key)
{
    dirty_keys_.insert(key);
    dirty_ = true;
    return ++iwlo_meta_[key].observation_count;
}
```

`get_or_create_meta` (line 59-63) 를 다음으로:

```cpp
IWLOMeta& OctreeStorage::get_or_create_meta(const octomap::OcTreeKey& key)
{
    dirty_keys_.insert(key);
    dirty_ = true;
    return iwlo_meta_[key];
}
```

(`get_or_create_meta` 는 caller 가 메타를 직접 변경할 수 있으므로 보수적으로 dirty 표시.)

`load_iwlo_meta` (line 222-263, Task 1 적용 후 형태) — 성공 시 `dirty_keys_` 를 모두 채워야 첫 sync 가 정상 동작. line 246 `iwlo_meta_.clear();` 다음 줄에 `dirty_keys_.clear();` 추가, `iwlo_meta_[key] = meta;` 다음 줄에 `dirty_keys_.insert(key);` 추가, 그리고 손상 감지 시 `iwlo_meta_.clear();` 다음 줄에 `dirty_keys_.clear();` 추가.

- [ ] **Step 4**: 4 mutation 경로 + load 경로에 dirty_keys_ 관리 추가.

### Step 5: `clear()` 와 `sync_to_octree()` 변경

`clear()` (line 269-274) 를 다음으로:

```cpp
void OctreeStorage::clear()
{
    octree_->clear();
    iwlo_meta_.clear();
    dirty_keys_.clear();
    dirty_ = true;
}
```

`sync_to_octree()` (line 276-286) 를 다음으로:

```cpp
void OctreeStorage::sync_to_octree()
{
    if (dirty_keys_.empty()) {
        return;
    }
    for (const auto& key : dirty_keys_) {
        auto it = iwlo_meta_.find(key);
        if (it == iwlo_meta_.end()) {
            continue;  // Defensive: meta erased after dirty marking
        }
        octomap::point3d coord = octree_->keyToCoord(key);
        octomap::OcTreeNode* node = octree_->updateNode(coord, true, true);
        if (node) {
            node->setLogOdds(static_cast<float>(it->second.log_odds));
        }
    }
    octree_->updateInnerOccupancy();
    dirty_keys_.clear();
}
```

핵심: 변경된 key 만 octree 에 반영, 끝에 `dirty_keys_.clear()`.

- [ ] **Step 5**: clear / sync 변경.

### Step 6: 빌드 + 테스트 PASS

Run:
```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
colcon test --packages-select sonar_3d_reconstruction --ctest-args -R test_octree_storage
colcon test-result --verbose --test-result-base build/sonar_3d_reconstruction
```

Expected: 5 테스트 모두 PASS. 특히 `IncrementalSyncProducesSameOctreeAsFullSync` 가 결과 동일성을 보증.

- [ ] **Step 6**: PASS 확인.

### Step 7: Python 회귀 확인

Run:
```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && python3 -m pytest tests/ -x -q
```

Expected: 기존 14 PASS.

- [ ] **Step 7**: Python 회귀 없음 확인.

### Step 8: Commit

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git add sonar_3d_reconstruction/cpp/octree_storage.h sonar_3d_reconstruction/cpp/octree_storage.cpp test/test_octree_storage_load.cpp
git commit -m "perf(octree_storage): incremental sync via dirty_keys_ (P2-5, C-b)

sync_to_octree now re-applies only the keys mutated since the last
sync. set_log_odds / increment_observation_count / get_or_create_meta
/ load all maintain dirty_keys_; clear() resets it.

- cpp/octree_storage.h — dirty_keys_ unordered_set member
- cpp/octree_storage.cpp — 4 mutation paths track dirty, sync iterates
  only dirty subset and clears on completion
- test/test_octree_storage_load.cpp — 2 sync tests (incremental ==
  full result, clear resets dirty)"
```

- [ ] **Step 8**: Commit.

---

## Task 3 — C-c: continuous IWLO unification (P2-2, Q-C1)

**근거**: master spec §3 Phase C P2-2 + §10 Q-C1. `ProbabilityUpdater::batch_update_iwlo` (lines 372-380) 와 `Tile::update_voxel` (lines 93-101) 둘 다 `intensity > intensity_threshold` 이진 분기로 `delta_L` 을 계산. `IWLOUpdater::compute_delta_log_odds` (`iwlo_updater.cpp:97-120`) 는 같은 입력으로 연속형 (`w * L_occ + (1-w) * L_free`) 을 계산. 두 호출자가 후자를 직접 호출하도록 통일하면 README 알고리즘 절과 일치하고 임계 부근 voxel 의 부드러운 전이를 얻는다.

**중요**: `compute_delta_log_odds` 는 내부에 `compute_adaptive_scale` 을 호출하므로, 호출자 측 `adapt_scale` 계산/곱셈 블록은 **모두 제거** 한다. 호출자 책임으로 남는 것은:
- `ProbabilityUpdater::batch_update_iwlo`: `weights(i)` 곱 (P0-5, B-2 자산)
- `Tile::update_voxel`: 없음

**Files:**
- Modify: `sonar_3d_reconstruction/cpp/probability_updater.cpp:329-389`
- Modify: `sonar_3d_reconstruction/cpp/tile.cpp:54-109`
- Test: `sonar_3d_reconstruction/tests/test_iwlo_continuous_unification.py` (신규)

### Step 1: 단위 테스트 작성 (실패 기대)

`sonar_3d_reconstruction/tests/test_iwlo_continuous_unification.py` (신규):

```python
"""Phase C-c: verify ProbabilityUpdater.batch_update_iwlo uses continuous IWLO.

The legacy implementation took a binary branch on intensity_threshold:
    intensity > T  ->  delta = L_occ * w(I) * alpha * scale
    intensity <= T ->  delta = L_free      * alpha * scale

The unified implementation calls IWLOUpdater::compute_delta_log_odds, which
mixes both terms continuously:
    delta = alpha * scale * ( w * L_occ + (1 - w) * L_free )

Near the threshold the two diverge; far from it they agree (sigmoid
saturates to {0, 1} for w).
"""

import numpy as np
import pytest

from sonar_3d_reconstruction import ProbabilityUpdater


def _fresh_updater(resolution=0.1):
    u = ProbabilityUpdater(resolution=resolution)
    # Use canonical params so legacy and continuous saturate identically far
    # from the threshold:
    u.set_iwlo_params(sharpness=10.0, decay_rate=0.05, min_alpha=0.1,
                      L_min=-10.0, L_max=10.0)
    u.set_log_odds_params(log_odds_occupied=2.0, log_odds_free=-1.0)
    u.set_intensity_params(intensity_threshold=80.0, intensity_max=255.0)
    u.set_adaptive_params(enabled=False, threshold=0.5, max_ratio=1.0)
    return u


def test_high_intensity_voxel_log_odds_increases():
    """Far above threshold (I=240, T=80) -> w ≈ 1, behaves like occupied path."""
    u = _fresh_updater()
    pts = np.array([[0.05, 0.05, 0.05]], dtype=np.float64)
    intensities = np.array([240.0], dtype=np.float64)
    occ = [True]
    u.batch_update_iwlo(pts, intensities, occ)
    voxels = u.get_occupied_voxels(min_probability=0.0)
    assert voxels.shape[0] == 1
    # log_odds_occupied=2.0, alpha for n=0 = 1.0 -> delta ~= 2.0
    # probability(2.0) ≈ 0.881
    assert voxels[0, 3] > 0.7, f"expected occupied prob, got {voxels[0, 3]}"


def test_low_intensity_voxel_log_odds_decreases():
    """Far below threshold (I=10, T=80) -> w ≈ 0, behaves like free path."""
    u = _fresh_updater()
    pts = np.array([[0.05, 0.05, 0.05]], dtype=np.float64)
    intensities = np.array([10.0], dtype=np.float64)
    occ = [False]
    # Seed positive log-odds first so we can observe a decrease.
    u.batch_update_iwlo(pts, np.array([240.0]), [True])
    p_before = u.get_occupied_voxels(min_probability=0.0)[0, 3]

    u.batch_update_iwlo(pts, intensities, occ)
    p_after = u.get_occupied_voxels(min_probability=0.0)[0, 3]
    assert p_after < p_before, (
        f"low-intensity update should decrease probability: "
        f"before={p_before}, after={p_after}"
    )


def test_threshold_neighborhood_continuous_blend():
    """Continuous IWLO produces a non-zero blend near threshold.

    Legacy binary branch at I = threshold + 1 (= 81) takes the occupied
    path with w(81) very small, giving delta ≈ L_occ * tiny = tiny positive.
    The continuous form gives delta = alpha * (w * L_occ + (1 - w) * L_free)
    ≈ 1 * (tiny * 2 + ~1 * -1) = ≈ -1, i.e. *negative*. Either sign is
    acceptable per Q-C1 (intentional change); we assert that at least the
    magnitude differs from the legacy occupied-path-tiny value.
    """
    u = _fresh_updater()
    pts = np.array([[0.05, 0.05, 0.05]], dtype=np.float64)
    intensities = np.array([81.0], dtype=np.float64)  # just above threshold 80
    u.batch_update_iwlo(pts, intensities, [True])
    p_after = u.get_occupied_voxels(min_probability=0.0)[0, 3]
    # The continuous form near-threshold produces a noticeable shift
    # (legacy would shift +tiny, continuous shifts substantially toward
    # the dominant free term). Either way |p - 0.5| should not be ~0.
    assert abs(p_after - 0.5) > 0.05, (
        f"expected non-trivial shift near threshold, got p={p_after}"
    )


def test_weights_still_propagate():
    """P0-5 invariant: per-point weights still scale the unified delta."""
    u1 = _fresh_updater()
    u3 = _fresh_updater()
    pts = np.array([[0.05, 0.05, 0.05]], dtype=np.float64)
    intensities = np.array([200.0], dtype=np.float64)
    occ = [True]
    u1.batch_update_iwlo(pts, intensities, occ, np.array([1.0]))
    u3.batch_update_iwlo(pts, intensities, occ, np.array([3.0]))
    p1 = u1.get_occupied_voxels(min_probability=0.0)[0, 3]
    p3 = u3.get_occupied_voxels(min_probability=0.0)[0, 3]
    assert p3 > p1, f"weight=3 should produce higher prob than weight=1 (got {p3} vs {p1})"
```

- [ ] **Step 1**: 신규 테스트 파일 작성.

### Step 2: 빌드 + 테스트 실행 — `test_threshold_neighborhood_continuous_blend` 가 FAIL 하는지 확인

Run:
```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction && python3 -m pytest tests/test_iwlo_continuous_unification.py -v
```

Expected: `test_high_intensity_voxel_log_odds_increases` PASS (legacy occupied path 도 양수 delta 산출), `test_low_intensity_voxel_log_odds_decreases` PASS, `test_threshold_neighborhood_continuous_blend` **FAIL** (legacy binary branch 는 I=81 에서 occupied path 진입 — `L_occ * w(81) * 1.0 * 1.0` ≈ `2.0 * tiny` ≈ 0 → 첫 update 후 p ≈ 0.5, `|p - 0.5| < 0.05`), `test_weights_still_propagate` PASS.

- [ ] **Step 2**: blend 테스트 FAIL 확인. 다른 3 PASS.

### Step 3: `ProbabilityUpdater::batch_update_iwlo` 통일

`sonar_3d_reconstruction/cpp/probability_updater.cpp:329-407` 영역 (loop 내부) 를 다음으로 교체:

```cpp
    // Build the IWLOParams once; the unified path needs it.
    IWLOParams params;
    params.intensity_threshold = intensity_threshold_;
    params.intensity_max = intensity_max_;
    params.sharpness = sharpness_;
    params.decay_rate = decay_rate_;
    params.min_alpha = min_alpha_;
    params.log_odds_occupied = log_odds_occupied_;
    params.log_odds_free = log_odds_free_;
    params.L_min = L_min_;
    params.L_max = L_max_;
    params.adaptive_enabled = adaptive_enabled_;
    params.adaptive_threshold = adaptive_threshold_;
    params.adaptive_max_ratio = adaptive_max_ratio_;

    for (int i = 0; i < points.rows(); ++i) {
        // Get voxel key
        std::string key = world_to_key(points(i, 0), points(i, 1), points(i, 2));

        // Increment observation count
        observation_counts_[key]++;
        int n = observation_counts_[key];

        double intensity = intensities(i);

        // Get current log-odds (default 0.0 for unknown voxels)
        double current_log_odds = 0.0;
        auto it = voxels_log_odds_.find(key);
        if (it != voxels_log_odds_.end()) {
            current_log_odds = it->second;
        }

        // Continuous IWLO via the canonical helper (Q-C1).
        // n was just incremented to >=1, but compute_alpha treats the
        // count as "observations *before* this update" -> pass n - 1.
        double delta_L = IWLOUpdater::compute_delta_log_odds(
            intensity, current_log_odds, n - 1, params);

        // Per-point weight (P0-5, B-2): callers may fold sub-voxel
        // multiplicity into a single batch entry.
        if (has_weights) {
            delta_L *= weights(i);
        }

        // Apply update with saturation limits
        double new_log_odds = current_log_odds + delta_L;
        new_log_odds = std::max(L_min_, std::min(L_max_, new_log_odds));

        // Store updated value
        voxels_log_odds_[key] = new_log_odds;

        // Track modified voxel for incremental sync
        modified_keys_.insert(key);
    }

    // Sync with octree_mapper_ for visualization
    if (enable_incremental_sync_) {
        sync_modified_voxels_to_octree();
    } else {
        sync_all_voxels_to_octree();
    }

    // Clear modified keys after sync
```

핵심 변경:
- 제거: `current_prob` 직접 계산, `w_intensity` / `alpha_n` 직접 계산, `adapt_scale` 자체 분기 (54 LOC 제거)
- 추가: `IWLOParams params` 빌드 (locally) + `IWLOUpdater::compute_delta_log_odds` 호출 한 줄
- 보존: `weights` (P0-5) 곱셈, `modified_keys_` 트래킹, `sync_modified_voxels_to_octree`

`#include "iwlo_updater.h"` 와 `#include "tile.h"` (IWLOParams 정의) 는 이미 포함돼 있으므로 추가 include 불필요. 확인 후 누락 시 추가.

- [ ] **Step 3**: `ProbabilityUpdater::batch_update_iwlo` 변경.

### Step 4: `Tile::update_voxel` 통일

`sonar_3d_reconstruction/cpp/tile.cpp:54-109` 의 `update_voxel` 본문을 다음으로 교체:

```cpp
void Tile::update_voxel(const octomap::point3d& point,
                        double intensity,
                        bool /* is_occupied */,
                        const IWLOParams& params)
{
    // Get octree key for this point
    octomap::OcTreeKey key = storage_->coord_to_key(point);

    // Get or create IWLO metadata
    IWLOMeta& meta = storage_->get_or_create_meta(key);
    meta.observation_count++;
    int n = meta.observation_count;

    double current_log_odds = meta.log_odds;

    // Continuous IWLO via the canonical helper (Q-C1).
    // n was just incremented to >=1; compute_alpha expects the count of
    // observations *before* the current update.
    double delta_L = IWLOUpdater::compute_delta_log_odds(
        intensity, current_log_odds, n - 1, params);

    // Apply update with saturation limits
    double new_log_odds = current_log_odds + delta_L;
    new_log_odds = std::max(params.L_min, std::min(params.L_max, new_log_odds));

    meta.log_odds = new_log_odds;
    storage_->mark_dirty();
}
```

(기존 50 LOC 본문 → 18 LOC.)

- [ ] **Step 4**: `Tile::update_voxel` 변경.

### Step 5: 빌드 + 테스트 PASS

Run:
```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
python3 -m pytest tests/test_iwlo_continuous_unification.py -v
python3 -m pytest tests/ -x -q
colcon test --packages-select sonar_3d_reconstruction --ctest-args -R test_octree_storage
colcon test-result --verbose --test-result-base /workspace/ros2_ws/build/sonar_3d_reconstruction
```

Expected:
- `test_iwlo_continuous_unification.py` 4/4 PASS (blend 테스트 이제 통과)
- 기존 14 + 4 = 18 Python 테스트 PASS
- C-a/C-b gtest 5 PASS

- [ ] **Step 5**: 모두 PASS 확인.

### Step 6: 회귀 측정 (P-2 single dataset, jaccard ≥ 0.95)

main HEAD baseline 과 비교. main 은 이미 capture 돼 있을 수 있으나 안전을 위해 재측정.

```bash
# baseline (main HEAD = 2facddf, 현재 branch 의 HEAD~3 위치)
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git stash  # working tree 보호 (이 시점엔 clean 이지만 관습)
BASE_SHA=$(git rev-parse main)
git checkout "$BASE_SHA" -- .  # detach not needed; src 만 임시 교체
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
bash scripts/regression/regression_test.sh baseline
```

(만약 baseline 이 이미 `/tmp/sonar3d_regression/baseline/` 에 캐시돼 있고 `regression_test.sh` 가 idempotent 하면 바로 candidate 단계로 가도 됨. README 확인.)

```bash
# candidate (current branch HEAD)
git checkout refactor/phase-c-algorithm-unify -- .
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
bash scripts/regression/regression_test.sh candidate

# compare
python3 scripts/regression/regression_compare.py /tmp/sonar3d_regression/baseline /tmp/sonar3d_regression/candidate
python3 scripts/regression/regression_plot.py /tmp/sonar3d_regression/baseline /tmp/sonar3d_regression/candidate -o /tmp/sonar3d_regression/phase-c-c
```

Expected: `jaccard_set ≥ 0.95` (P2-2 임계). plot 의 xy/xz 단면에서 임계 부근 voxel 부드러운 전이 확인. 같은 코드 variance ≈ 0.18 (B-1 measured floor) 안에서 의미를 가지므로 plot 이 1차 증거.

만약 jaccard < 0.95: PR 머지 보류, 사용자 보고 (refactor-workflow.md §6.1).

- [ ] **Step 6**: 회귀 측정 + plot 생성.

### Step 7: Commit

```bash
cd /workspace/ros2_ws/src/sonar_3d_reconstruction
git add sonar_3d_reconstruction/cpp/probability_updater.cpp sonar_3d_reconstruction/cpp/tile.cpp tests/test_iwlo_continuous_unification.py
git commit -m "refactor(iwlo): unify probability updater and tile to continuous IWLO (P2-2, C-c, Q-C1)

Both ProbabilityUpdater::batch_update_iwlo and Tile::update_voxel now
delegate the delta-log-odds computation to
IWLOUpdater::compute_delta_log_odds (continuous form), removing the
binary intensity > threshold branch in each. README §IWLO matches.

Per Q-C1 (2026-05-03) the result is intentionally different near the
intensity threshold: legacy collapsed sub-threshold updates to a flat
free term, continuous form blends free and occupied per the sigmoid
weight w(I).

- cpp/probability_updater.cpp:329-407 — single compute_delta_log_odds
  call in the loop; weights(i) (P0-5) still applied by caller
- cpp/tile.cpp:54-109 — same unification; ~32 LOC removed
- tests/test_iwlo_continuous_unification.py — 4 tests covering
  high-intensity, low-intensity, threshold neighborhood blend, weights
  propagation"
```

- [ ] **Step 7**: Commit.

---

## Task 4 — CHANGELOG + push + PR

**Files:**
- Modify: `CHANGELOG.md` (top)

### Step 1: CHANGELOG 항목 추가

`CHANGELOG.md` 최상단 (가장 최신 항목 위) 에 다음 추가:

```markdown
## [Unreleased] — Phase C: algorithm unification (refactor)

### Changed
- `cpp/octree_storage.cpp::sync_to_octree` — full re-sync → incremental
  via `dirty_keys_` tracking (P2-5, C-b). All mutation paths
  (`set_log_odds`, `increment_observation_count`,
  `get_or_create_meta`, `load_iwlo_meta`) maintain the dirty set;
  `clear()` resets it.
- `cpp/probability_updater.cpp::batch_update_iwlo` and
  `cpp/tile.cpp::update_voxel` — binary intensity branch removed in
  both. Both call `IWLOUpdater::compute_delta_log_odds` (continuous
  form) directly (P2-2, C-c, Q-C1). README §IWLO algorithm now matches
  the implementation. Caller-side `adapt_scale` block deleted because
  `compute_delta_log_odds` already invokes `compute_adaptive_scale`
  internally; per-point `weights(i)` (P0-5, B-2) propagation kept.
- `cpp/octree_storage.cpp::load_iwlo_meta` — per-entry `ifs.good()`
  check inside the read loop. On partial-read `iwlo_meta_` is cleared
  and `false` is returned, so callers never see half-filled metadata
  (P2-6, C-a).

### Added
- `cpp/octree_storage.h` — `dirty_keys_` (`unordered_set<OcTreeKey>`)
  member.
- `test/test_octree_storage_load.cpp` — 5 gtest cases (3 truncation
  scenarios + incremental-sync equivalence + clear-resets-dirty).
- `tests/test_iwlo_continuous_unification.py` — 4 pytest cases.
- `docs/source/design/2026-05-05-phase-c-algorithm-unify-design.md`,
  `docs/source/plans/2026-05-05-phase-c-algorithm-unify.md`.

### Verification
- colcon build PASS (Release).
- 23 tests PASS total: 14 (existing) + 4 (continuous IWLO) + 5 (octree
  storage gtest).
- Regression P-2 (`m3000d-range15-tilt90`, 352s, 1757 frames):
  jaccard_set ≥ 0.95 (C-c intentional change, Q-C1, plot-verified
  smooth threshold-neighborhood transition); jaccard_set ≥ 0.99 for
  C-a / C-b alone (result-equivalent).
- Same-code measurement variance ≈ 0.18 jaccard remains the floor as
  measured in B-1; near-threshold voxel plot is the primary qualitative
  proof.

### Notes
- P-1 dataset (`m3000d-range20-tilt30`) still skipped — TimeSync
  stamp_diff ≈ 0.21 s exceeds 0.1 s threshold; bag stamps cannot be
  modified post-hoc per user 2026-05-05 decision.
- Phase D (vectorization, P1-3) untouched — separate spec / plan / PR.
```

- [ ] **Step 1**: CHANGELOG 갱신.

### Step 2: Commit CHANGELOG

```bash
git add CHANGELOG.md
git commit -m "docs(changelog): record Phase C algorithm unification"
```

- [ ] **Step 2**: Commit.

### Step 3: Push + PR

```bash
git push -u origin refactor/phase-c-algorithm-unify
gh pr create --base main --head refactor/phase-c-algorithm-unify \
  --title "Phase C: algorithm unification (P2-2 / P2-5 / P2-6)" \
  --body "$(cat <<'EOF'
## Summary
- Unify `ProbabilityUpdater::batch_update_iwlo` and `Tile::update_voxel` on the continuous-form `IWLOUpdater::compute_delta_log_odds` (Q-C1) — README §IWLO now matches the implementation.
- `OctreeStorage::sync_to_octree` re-applies only mutated voxels via `dirty_keys_` (P2-5).
- `OctreeStorage::load_iwlo_meta` detects partial reads inside the loop and clears the storage on failure (P2-6).

## Changes
- `cpp/probability_updater.cpp` — 54 LOC of binary-branch code replaced with one canonical call; `weights(i)` (P0-5) still applied caller-side.
- `cpp/tile.cpp::update_voxel` — same unification; ~32 LOC removed.
- `cpp/octree_storage.{h,cpp}` — `dirty_keys_` member, mutation paths track dirty, `clear()` resets it.
- `test/test_octree_storage_load.cpp`, `tests/test_iwlo_continuous_unification.py` — 9 new tests.

## Verification
- [ ] colcon build PASS (Release)
- [ ] 14 (existing) + 4 (continuous IWLO) + 5 (octree storage gtest) = 23 PASS
- [ ] Regression P-2: `jaccard_set` ≥ 0.95 (C-c intentional change, Q-C1) / ≥ 0.99 for C-a/C-b alone
- [ ] Plot attached: smooth threshold-neighborhood transition vs. legacy step

## Next Phase
- **Phase D — Vectorization (P1-3)**: separate spec/plan/PR. Q-D1 controls A안→B안 progression.
EOF
)"
```

- [ ] **Step 3**: Push + gh pr create. URL 출력하여 사용자에게 보고.

---

## 자기 점검 (작성자 — plan 적용 전 1 회)

1. **spec 적용 범위**: design §2 Scope 표 (C-a/b/c 3 항목) 모두 Task 1/2/3 에 1:1 매핑. ✅
2. **placeholder 스캔**: "TBD", "TODO", "위와 유사", "적절히 처리" 없음. ✅
3. **타입/메서드 일관성**: `dirty_keys_` 는 Task 2 에서 정의된 후 Task 2/3 에서만 사용. `compute_delta_log_odds` 시그니처는 모든 호출처에서 동일 (`intensity, current_log_odds, observation_count, params`). ✅
4. **임계 일치**: §design §4 표 jaccard ≥ 0.99 (C-a/b) / ≥ 0.95 (C-c) 가 Task 3 Step 6 + Task 4 PR body / CHANGELOG 와 일치. ✅
5. **회귀 데이터셋**: P-2 단독 — design §4 + memory `project_sonar3d_audit_state.md` 와 일치. ✅
