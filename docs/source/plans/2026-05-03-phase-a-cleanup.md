# Phase A — Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use `superpowers:subagent-driven-development` (recommended) or `superpowers:executing-plans` to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **Spec**: `docs/source/design/2026-05-03-quality-perf-uplift-design.md` §3 Phase A
> **Risk**: 0% — 알고리즘 영향 없음. 회귀 측정 의무 없음. `colcon build` PASS + launch smoke test로 검증.
> **Branch**: `refactor/phase-a-cleanup` (이미 생성됨, spec commit 완료)

**Goal:** sonar_3d_reconstruction 패키지의 죽은 코드·중복 정의·dead config 5개 항목을 제거하여 약 100-150 LOC 감소시키고, Phase B-1 이후의 변경 추적성을 향상시킨다.

**Architecture:** 5개 독립 변경(P0-6, P3-2, P3-3, P3-5, P3-7)을 commit 단위로 분리. 각 변경은 다른 파일 또는 다른 책임 영역을 만지므로 충돌 없이 순차 적용 가능. 마지막에 CHANGELOG 갱신 + 통합 빌드 검증.

**Tech Stack:** ROS2 Humble (rclpy + ament_cmake), Python 3.10, C++17 (pybind11/OctoMap), `colcon`.

---

## File Structure (변경 대상)

| 파일 | 변경 종류 | 책임 |
|------|----------|------|
| `config/qos_override.yaml` | **삭제** | (제거 대상 — launch 미참조 dead file) |
| `sonar_3d_reconstruction/cpp/suppress_output.h` | **신규** | RAII stdout/stderr 억제 클래스 단일 정의 |
| `sonar_3d_reconstruction/cpp/octree_mapper.cpp` | **수정** | SuppressOutput 정의 제거, 헤더 include로 전환 |
| `sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp` | **수정** | SuppressOutput 정의 제거, 헤더 include로 전환 |
| `sonar_3d_reconstruction/cpp/CMakeLists.txt` | **수정 가능** | 신규 헤더 install 처리 (필요 시) |
| `scripts/config.py` | **수정** | 1) `from_ros_params` dead code 삭제 (~65 LOC), 2) `crosstalk.gaussian_sigma` ParameterDef 삭제, 3) `update_visualization`/`update_orientation` handler 매핑 제거 |
| `scripts/crosstalk_filter.py` | **수정** | `gaussian_sigma` 생성자 인자·멤버·핸들러 메서드 삭제 |
| `scripts/3d_mapper_node.py` | **수정** | `CrosstalkFilter()` 생성자 호출에서 `gaussian_sigma` 인자 제거 |
| `scripts/3d_mapper.py` | **수정** | `update_visualization`(451-453), `update_orientation`(459-464) stub 메서드 삭제 |
| `CHANGELOG.md` | **수정** | Phase A 항목 추가 (refactor-workflow.md 양식) |

---

## Pre-Flight 검증 (Task 0)

### Task 0: 작업 환경 확인

**Files:** (read-only)

- [ ] **Step 1: 현재 브랜치 확인 — `refactor/phase-a-cleanup`이어야 함**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction branch --show-current
```

Expected output:
```
refactor/phase-a-cleanup
```

- [ ] **Step 2: working tree clean 확인 (spec commit 외 변경 없음)**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction status -s
```

Expected output (empty — 공백):
```
```

- [ ] **Step 3: baseline build PASS 확인 (변경 전 정상 동작 검증)**

```bash
cd /workspace/ros2_ws && source /opt/ros/humble/setup.bash && \
  colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
```

Expected: `Summary: 1 package finished` (no errors). 만약 실패 시 phase 진행 중지하고 사용자 보고.

---

## Task 1: P0-6 — `qos_override.yaml` dead config 삭제

**근거**: launch 파일들 어디서도 `qos_override`를 참조하지 않고(`grep -r qos_override launch/` → 0 hits), 토픽 키도 실제 `/sensor/sonar/oculus/m750d/image`와 다른 `/sonar/image`를 가리킨다. 사용자 결정 (Q-A1) = 삭제.

**Files:**
- Delete: `config/qos_override.yaml`

- [ ] **Step 1: 삭제 전 launch에 미참조 재확인**

```bash
grep -rn "qos_override" /workspace/ros2_ws/src/sonar_3d_reconstruction/launch/ \
  /workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/
```

Expected output (empty):
```
```

만약 어떤 매칭이 나오면 plan 일시 중지하고 사용자 보고.

- [ ] **Step 2: 파일 삭제**

```bash
rm /workspace/ros2_ws/src/sonar_3d_reconstruction/config/qos_override.yaml
```

- [ ] **Step 3: package install 영향 점검 (CMakeLists에서 config 디렉토리 install 시 누락 영향 없음)**

```bash
grep -n "qos_override\|config/" /workspace/ros2_ws/src/sonar_3d_reconstruction/CMakeLists.txt
```

Expected: `install(DIRECTORY config DESTINATION share/...)` 형태만 보이고 명시 참조 없음. (전체 디렉토리 install이라 단순 삭제로 OK.)

- [ ] **Step 4: build 검증**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 5: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add -u config/qos_override.yaml
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
chore(qos): remove dead qos_override.yaml (P0-6)

launch 파일 어디에서도 --qos-overrides-file로 참조되지 않으며
토픽 키(/sonar/image)도 실제 토픽(/sensor/sonar/oculus/m750d/image)과
일치하지 않는 dead file. 노드는 launch arg qos_reliability를 통해
QoS를 일관 적용 중이므로 surgical 제거.

- config/qos_override.yaml 삭제 (-11 LOC)
EOF
)"
```

---

## Task 2: P3-2 — `SuppressOutput` RAII 헤더 추출

**근거**: `octree_mapper.cpp:14-37`과 `outofcore_tile_mapper.cpp:14-37`에 24줄 동일 정의가 중복. Audit P3-2.

**Files:**
- Create: `sonar_3d_reconstruction/cpp/suppress_output.h`
- Modify: `sonar_3d_reconstruction/cpp/octree_mapper.cpp:14-37` (정의 제거 + include)
- Modify: `sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp:14-37` (정의 제거 + include)

- [ ] **Step 1: `suppress_output.h` 신규 작성**

전체 파일 내용:

```cpp
#ifndef SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H
#define SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H

#include <iostream>
#include <cstdio>
#include <unistd.h>

namespace sonar_3d_reconstruction
{

// RAII helper to suppress stdout/stderr at file descriptor level.
// Used to silence verbose third-party (OctoMap) output during operations.
class SuppressOutput {
public:
    SuppressOutput() {
        std::cout.flush();
        std::cerr.flush();
        fflush(stdout);
        fflush(stderr);
        stdout_fd_ = dup(fileno(stdout));
        stderr_fd_ = dup(fileno(stderr));
        freopen("/dev/null", "w", stdout);
        freopen("/dev/null", "w", stderr);
    }

    ~SuppressOutput() {
        fflush(stdout);
        fflush(stderr);
        dup2(stdout_fd_, fileno(stdout));
        dup2(stderr_fd_, fileno(stderr));
        close(stdout_fd_);
        close(stderr_fd_);
    }

    // Non-copyable, non-movable (manages OS resources)
    SuppressOutput(const SuppressOutput&) = delete;
    SuppressOutput& operator=(const SuppressOutput&) = delete;
    SuppressOutput(SuppressOutput&&) = delete;
    SuppressOutput& operator=(SuppressOutput&&) = delete;

private:
    int stdout_fd_;
    int stderr_fd_;
};

}  // namespace sonar_3d_reconstruction

#endif  // SONAR_3D_RECONSTRUCTION_SUPPRESS_OUTPUT_H
```

작성:
```bash
# Use Write tool with the content above to create:
# /workspace/ros2_ws/src/sonar_3d_reconstruction/sonar_3d_reconstruction/cpp/suppress_output.h
```

- [ ] **Step 2: `octree_mapper.cpp`의 SuppressOutput 정의 제거 + include 추가**

`octree_mapper.cpp` 라인 1-7 변경:

변경 전:
```cpp
#include "octree_mapper.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unistd.h>
```

변경 후:
```cpp
#include "octree_mapper.h"
#include "suppress_output.h"
#include <algorithm>
#include <cmath>
```

라인 13-37 (SuppressOutput 클래스 전체) **삭제**:
```cpp
// RAII class to suppress stdout/stderr at file descriptor level
class SuppressOutput {
    // ... 24줄 ...
};
```

- [ ] **Step 3: `outofcore_tile_mapper.cpp`의 SuppressOutput 정의 제거 + include 추가**

`outofcore_tile_mapper.cpp` 라인 1-8 변경:

변경 전:
```cpp
#include "outofcore_tile_mapper.h"
#include <iostream>
#include <algorithm>
#include <sstream>
#include <cstdio>
#include <unistd.h>
#include <limits>
#include <cmath>
```

변경 후:
```cpp
#include "outofcore_tile_mapper.h"
#include "suppress_output.h"
#include <algorithm>
#include <sstream>
#include <limits>
#include <cmath>
```

라인 13-37 (SuppressOutput 클래스 전체) **삭제**.

- [ ] **Step 4: CMakeLists 영향 점검**

```bash
grep -n "octree_mapper.cpp\|outofcore_tile_mapper.cpp\|cpp/" /workspace/ros2_ws/src/sonar_3d_reconstruction/CMakeLists.txt | head -20
```

Expected: `.cpp` 파일들이 `add_library`/`pybind11_add_module`에 등록돼 있고, 헤더는 별도 등록 안 됨(`include_directories(...cpp)`로 디렉토리 단위 인식). 따라서 신규 헤더 추가 시 CMakeLists 변경 **불필요**.

- [ ] **Step 5: build 검증 (`colcon` Release)**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -10
```

Expected: 컴파일 에러 없음, `Summary: 1 package finished`. 만약 `'iostream' file not found` 등 헤더 누락 에러 발생 시: 두 `.cpp`에서 `<iostream>`은 코드 본문에서 직접 사용되는지 grep으로 재확인 후 필요 시 다시 include.

```bash
# Trouble-shooting (필요 시):
grep -c "std::cout\|std::cerr" /workspace/ros2_ws/src/sonar_3d_reconstruction/sonar_3d_reconstruction/cpp/octree_mapper.cpp
grep -c "std::cout\|std::cerr" /workspace/ros2_ws/src/sonar_3d_reconstruction/sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp
```

만약 카운트 > 0이면 `#include <iostream>`을 해당 cpp에 다시 추가 (suppress_output.h가 이미 include하지만 cpp의 다른 코드 경로에서 직접 사용 시 명시적으로 두는 게 가독성).

- [ ] **Step 6: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add \
  sonar_3d_reconstruction/cpp/suppress_output.h \
  sonar_3d_reconstruction/cpp/octree_mapper.cpp \
  sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
refactor(cpp): extract SuppressOutput RAII to shared header (P3-2)

octree_mapper.cpp와 outofcore_tile_mapper.cpp에 24줄 동일 정의로
중복돼 있던 SuppressOutput 클래스를 cpp/suppress_output.h로 추출.
non-copyable·non-movable 제약을 명시적으로 추가하여 OS 리소스
관리 의도를 강제.

- sonar_3d_reconstruction/cpp/suppress_output.h (신규, +47 LOC)
- octree_mapper.cpp: 정의 제거 + include (-24 LOC)
- outofcore_tile_mapper.cpp: 정의 제거 + include (-24 LOC)
- 순 LOC: -1 (헤더 가드/네임스페이스/상속 제약으로 약간 증가)
EOF
)"
```

---

## Task 3: P3-3 — `from_ros_params` dead code 삭제

**근거**: `grep` 결과 `from_ros_params`는 어디에서도 호출되지 않음. `3d_mapper_node.py:94-95`는 이미 `from_params_dict(ParameterManager.get_all(self, MAPPER_PARAMS))` 패턴을 사용. 따라서 위임이 아닌 **순수 삭제**가 surgical.

**Files:**
- Modify: `scripts/config.py:410-475` (66줄 삭제)

- [ ] **Step 1: 호출자 부재 재확인**

```bash
grep -rn "from_ros_params" /workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/ \
  /workspace/ros2_ws/src/sonar_3d_reconstruction/launch/ \
  /workspace/ros2_ws/src/sonar_3d_reconstruction/sonar_3d_reconstruction/
```

Expected output (정의만 1건):
```
.../scripts/config.py:411:    def from_ros_params(cls, node) -> 'SonarMapperConfig':
```

만약 다른 호출이 있으면 plan 중지 + 사용자 보고.

- [ ] **Step 2: `config.py:410-475` 메서드 삭제**

`config.py:410-475`의 다음 블록 전체 삭제 (`@classmethod` 데코레이터부터 `return config` + 빈 줄까지):

```python
    @classmethod
    def from_ros_params(cls, node) -> 'SonarMapperConfig':
        """
        Create configuration from ROS2 node parameters

        Args:
            node: ROS2 node with declared parameters

        Returns:
            SonarMapperConfig instance
        """
        # Create main config with namespaced parameters
        config = cls(
            # Sonar hardware (sonar.*)
            ...
            # Processing (processing.*)
            frame_skip=node.get_parameter('processing.frame_skip').value
        )
        return config

```

삭제 후 `from_params_dict`(이전 477)이 즉시 따라오도록 한다.

- [ ] **Step 3: 삭제 후 라인 카운트 검증**

```bash
wc -l /workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/config.py
```

Expected: 이전 약 601줄 → 약 535줄 (-66).

- [ ] **Step 4: import 정리 점검 (사용 안 하는 import 제거 안 함, P3 범위 외)**

`config.py`의 import에서 `from rclpy.node import Node`는 ParameterManager에서 여전히 사용 중이므로 유지. 변경 없음.

- [ ] **Step 5: build 검증**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
```

Expected: `Summary: 1 package finished`. Python은 build에서 단순 install이므로 syntax error만 검출.

- [ ] **Step 6: import smoke test**

```bash
cd /workspace/ros2_ws && source install/setup.bash && \
  python3 -c "
import sys
sys.path.insert(0, '/workspace/ros2_ws/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction')
from config import SonarMapperConfig, MAPPER_PARAMS, ParameterManager
print('imports OK, MAPPER_PARAMS count:', len(MAPPER_PARAMS))
print('from_ros_params present:', hasattr(SonarMapperConfig, 'from_ros_params'))
print('from_params_dict present:', hasattr(SonarMapperConfig, 'from_params_dict'))
"
```

Expected output:
```
imports OK, MAPPER_PARAMS count: <some number, unchanged>
from_ros_params present: False
from_params_dict present: True
```

- [ ] **Step 7: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add scripts/config.py
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
refactor(config): remove dead from_ros_params classmethod (P3-3)

3d_mapper_node.py는 이미 from_params_dict(ParameterManager.get_all(...))
패턴을 사용 중이고, from_ros_params는 어떤 호출자도 없는 dead code.
또한 from_ros_params는 depth_estimation 5개 파라미터를 누락하여
호출됐다면 잠재적 버그였음. 위임 대신 순수 삭제로 surgical.

- scripts/config.py: from_ros_params 메서드 제거 (-66 LOC)
EOF
)"
```

---

## Task 4: P3-5 — `gaussian_sigma` dead config 제거

**근거**: `crosstalk_filter.py:90`의 `_create_notch_mask` 본문은 `filter_width`만 Gaussian σ로 사용. `gaussian_sigma` 멤버는 생성자/setter에서만 만지고 실제 마스크 계산에 미참조. dead config.

**Files:**
- Modify: `scripts/crosstalk_filter.py` (생성자 인자, 멤버, setter 제거)
- Modify: `scripts/config.py` (`crosstalk.gaussian_sigma` ParameterDef 제거)
- Modify: `scripts/3d_mapper_node.py` (CrosstalkFilter 생성자 호출에서 인자 제거)

- [ ] **Step 1: `crosstalk_filter.py`에서 `gaussian_sigma` 인자·멤버 제거**

`crosstalk_filter.py:21-27` 변경:

변경 전:
```python
    def __init__(self, enabled: bool, filter_width: float, filter_strength: float,
                 dc_preserve_ratio: float, gaussian_sigma: float):
        self.enabled = enabled
        self.filter_width = filter_width
        self.filter_strength = filter_strength
        self.dc_preserve_ratio = dc_preserve_ratio
        self.gaussian_sigma = gaussian_sigma
```

변경 후:
```python
    def __init__(self, enabled: bool, filter_width: float, filter_strength: float,
                 dc_preserve_ratio: float):
        self.enabled = enabled
        self.filter_width = filter_width
        self.filter_strength = filter_strength
        self.dc_preserve_ratio = dc_preserve_ratio
```

`crosstalk_filter.py:125-127` (setter 메서드) **삭제**:

변경 전:
```python
    def update_crosstalk_gaussian_sigma(self, value: float):
        self.gaussian_sigma = float(value)
        self._invalidate_cache()
```

변경 후: (위 3줄 통째로 삭제)

- [ ] **Step 2: `config.py`에서 `crosstalk.gaussian_sigma` ParameterDef 제거**

`config.py:129-131` **삭제**:

변경 전:
```python
    ParameterDef('crosstalk.gaussian_sigma', 0.5,
                 'Gaussian rolloff sigma for smooth notch transition',
                 handler='update_crosstalk_gaussian_sigma'),
```

변경 후: (위 3줄 통째로 삭제. 앞뒤 ParameterDef는 그대로 유지.)

- [ ] **Step 3: `3d_mapper_node.py`의 `CrosstalkFilter()` 생성자 호출 인자 제거**

```bash
grep -n "CrosstalkFilter(" /workspace/ros2_ws/src/sonar_3d_reconstruction/scripts/3d_mapper_node.py
```

Expected: 1개 매칭. 그 부분의 호출에서 `gaussian_sigma=...` 또는 `params_dict.get('crosstalk.gaussian_sigma', ...)` 인자를 제거. 위치별 호출이면 4개 인자만 남도록.

호출 형태 예시(실제 호출 본문은 grep 결과에 따라 정확히 적용):

변경 전 (예시):
```python
        self.crosstalk_filter = CrosstalkFilter(
            enabled=params_dict['crosstalk.enabled'],
            filter_width=params_dict['crosstalk.filter_width'],
            filter_strength=params_dict['crosstalk.filter_strength'],
            dc_preserve_ratio=params_dict['crosstalk.dc_preserve_ratio'],
            gaussian_sigma=params_dict['crosstalk.gaussian_sigma'],
        )
```

변경 후:
```python
        self.crosstalk_filter = CrosstalkFilter(
            enabled=params_dict['crosstalk.enabled'],
            filter_width=params_dict['crosstalk.filter_width'],
            filter_strength=params_dict['crosstalk.filter_strength'],
            dc_preserve_ratio=params_dict['crosstalk.dc_preserve_ratio'],
        )
```

만약 위치 인자(positional)로 호출되어 있으면 마지막 인자만 제거.

- [ ] **Step 4: `config/common.yaml`/`config/presets/*.yaml`의 `gaussian_sigma` 키 처리 점검**

```bash
grep -rn "gaussian_sigma" /workspace/ros2_ws/src/sonar_3d_reconstruction/config/
```

Expected output (있을 수 있음):
```
config/common.yaml: ...   gaussian_sigma: 0.5
config/presets/tilt_30.yaml: ...   gaussian_sigma: ...
```

만약 yaml에 키가 있으면 ROS2가 "unused parameter" 경고만 띄우고 무시. 그러나 깨끗함을 위해 yaml에서도 키 삭제.

각 yaml 파일에서 `gaussian_sigma` 라인 제거 (들여쓰기·구분자 주의).

- [ ] **Step 5: build + import smoke test**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
```

```bash
cd /workspace/ros2_ws && source install/setup.bash && \
  python3 -c "
import sys
sys.path.insert(0, '/workspace/ros2_ws/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction')
from crosstalk_filter import CrosstalkFilter
import inspect
sig = inspect.signature(CrosstalkFilter.__init__)
params = list(sig.parameters.keys())
assert 'gaussian_sigma' not in params, f'gaussian_sigma still in init: {params}'
print('CrosstalkFilter.__init__ params:', params)
"
```

Expected: `CrosstalkFilter.__init__ params: ['self', 'enabled', 'filter_width', 'filter_strength', 'dc_preserve_ratio']`

- [ ] **Step 6: launch smoke test (노드 기동 → 수 초 후 자동 종료)**

```bash
cd /workspace/ros2_ws && source install/setup.bash && \
  timeout 8 ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py 2>&1 | \
  grep -i "error\|exception\|gaussian_sigma\|CrosstalkFilter" | head -20
```

Expected: `gaussian_sigma` 또는 `error` 라인 없음. (입력 토픽 미발행으로 mapper는 idle 상태이지만 노드 기동 자체는 성공해야 함.)

- [ ] **Step 7: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add \
  scripts/crosstalk_filter.py scripts/config.py scripts/3d_mapper_node.py \
  config/
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
refactor(crosstalk): remove unused gaussian_sigma parameter (P3-5)

CrosstalkFilter._create_notch_mask는 filter_width만 Gaussian σ로
사용하고 gaussian_sigma 멤버는 생성자/setter에서만 갱신될 뿐
실제 마스크 계산에 미참조였음 (dead config).

- crosstalk_filter.py: 생성자 인자·멤버·update_crosstalk_gaussian_sigma 제거
- config.py: crosstalk.gaussian_sigma ParameterDef 제거 (-3 LOC)
- 3d_mapper_node.py: CrosstalkFilter() 생성자 호출에서 인자 제거
- config/*.yaml: gaussian_sigma 키 제거 (해당 시)
EOF
)"
```

---

## Task 5: P3-7 — Stub 메서드(`update_visualization`, `update_orientation`) 제거

**근거**: `3d_mapper.py:451-453`의 `update_visualization`과 `:459-464`의 `update_orientation`은 본문이 `pass`인 stub. `3d_mapper_node.py:327-341`이 `mounting.orientation.*`을 직접 처리하고 `mapper.update_sonar_orientation()`(살아있는 별개 메서드)을 호출하므로 stub은 호출 경로 무관 dead code.

**Files:**
- Modify: `scripts/3d_mapper.py:451-464` (메서드 2개 삭제)
- Modify: `scripts/config.py:142-144, 162-170` (handler 매핑 제거 — `handler=` 키워드만 제거, ParameterDef 자체는 유지)

- [ ] **Step 1: `3d_mapper.py:451-464`의 두 stub 메서드 삭제**

변경 전:
```python
    def update_visualization(self, value: bool) -> None:
        """Update visualization flag (handled by node)"""
        pass  # Node-level parameter

    def update_dynamic_expansion(self, value: bool) -> None:
        """Update dynamic expansion flag"""
        self.dynamic_expansion = bool(value)

    def update_orientation(self, value: float) -> None:
        """
        Update sonar orientation (roll/pitch/yaw)
        Note: Requires node-level coordination for TF update
        """
        pass  # Node handles this with update_sonar_orientation()
```

변경 후:
```python
    def update_dynamic_expansion(self, value: bool) -> None:
        """Update dynamic expansion flag"""
        self.dynamic_expansion = bool(value)
```

(`update_visualization`과 `update_orientation` 메서드만 통째로 삭제. 그 사이에 살아있는 `update_dynamic_expansion`은 유지.)

- [ ] **Step 2: `config.py:142-144`의 visualization handler 제거**

변경 전:
```python
    ParameterDef('visualization.show_opencv_visualization', False,
                 'Show OpenCV visualization window',
                 handler='update_visualization'),
```

변경 후:
```python
    ParameterDef('visualization.show_opencv_visualization', False,
                 'Show OpenCV visualization window'),
```

(handler 인자만 제거. ParameterDef 자체는 유지하여 ROS2 파라미터 declaration은 보존.)

- [ ] **Step 3: `config.py:162-170`의 orientation handler 제거 (3건)**

변경 전:
```python
    ParameterDef('mounting.orientation.roll', 0.0,
                 'Sonar roll angle in degrees',
                 handler='update_orientation'),
    ParameterDef('mounting.orientation.pitch', 90.0,
                 'Sonar pitch angle in degrees (90 = pointing down)',
                 handler='update_orientation'),
    ParameterDef('mounting.orientation.yaw', 0.0,
                 'Sonar yaw angle in degrees',
                 handler='update_orientation'),
```

변경 후:
```python
    ParameterDef('mounting.orientation.roll', 0.0,
                 'Sonar roll angle in degrees'),
    ParameterDef('mounting.orientation.pitch', 90.0,
                 'Sonar pitch angle in degrees (90 = pointing down)'),
    ParameterDef('mounting.orientation.yaw', 0.0,
                 'Sonar yaw angle in degrees'),
```

- [ ] **Step 4: build + import smoke test**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -5
```

```bash
cd /workspace/ros2_ws && source install/setup.bash && \
  python3 -c "
import sys
sys.path.insert(0, '/workspace/ros2_ws/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction')
import importlib.util
spec = importlib.util.spec_from_file_location('mapper3d', '/workspace/ros2_ws/install/sonar_3d_reconstruction/lib/sonar_3d_reconstruction/3d_mapper.py')
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
assert not hasattr(m.SonarTo3DMapper, 'update_visualization'), 'stub still present'
assert not hasattr(m.SonarTo3DMapper, 'update_orientation'), 'stub still present'
assert hasattr(m.SonarTo3DMapper, 'update_sonar_orientation'), 'live method missing'
print('stub methods removed; live update_sonar_orientation present.')
"
```

Expected: `stub methods removed; live update_sonar_orientation present.`

- [ ] **Step 5: launch smoke test — 동적 파라미터 변경이 살아있는 경로로 처리되는지 확인**

```bash
cd /workspace/ros2_ws && source install/setup.bash && \
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py &
LAUNCH_PID=$!
sleep 5
ros2 param set /sonar_3d_mapper mounting.orientation.pitch 60.0
sleep 2
kill -INT $LAUNCH_PID 2>/dev/null
wait $LAUNCH_PID 2>/dev/null
```

Expected: ros2 param set이 `Set parameter successful` 응답. 노드 로그에 "update_sonar_orientation" 호출 흔적 또는 정상 종료. 만약 "no handler for update_orientation" 같은 경고 떠도 OK (handler 매핑 제거된 의도된 결과).

- [ ] **Step 6: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add scripts/3d_mapper.py scripts/config.py
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
refactor(mapper_3d): remove dead stub methods (P3-7)

3d_mapper.py의 update_visualization과 update_orientation은 본문이
pass인 stub. mounting.orientation.*은 3d_mapper_node.py:327-341의
parameter_callback에서 직접 처리되며 mapper.update_sonar_orientation()
(살아있는 별개 메서드)이 호출되므로 stub은 호출 경로 무관 dead code.

- 3d_mapper.py: update_visualization, update_orientation 메서드 제거 (-13 LOC)
- config.py: 4개 ParameterDef에서 handler='update_(visualization|orientation)' 제거
- 동적 파라미터 declaration은 유지 (노드 측 callback이 처리 중)
EOF
)"
```

---

## Task 6: CHANGELOG.md 갱신

**근거**: `refactor-workflow.md`의 4축(branch+commit+CHANGELOG+PR) 동기 갱신 강제.

**Files:**
- Modify: `CHANGELOG.md` (선두에 `[Unreleased] — Phase A` 항목 추가)

- [ ] **Step 1: 현재 CHANGELOG 선두 확인**

```bash
head -10 /workspace/ros2_ws/src/sonar_3d_reconstruction/CHANGELOG.md
```

- [ ] **Step 2: 선두에 Phase A 항목 추가**

`CHANGELOG.md` 1행(`# CHANGELOG - sonar_3d_reconstruction`) 아래에 다음 블록을 삽입 (기존 `## [2026-03-28]` 위에):

```markdown

## [Unreleased] — Phase A: Cleanup (refactor)

> Master design: `docs/source/design/2026-05-03-quality-perf-uplift-design.md`
> Risk: 0% (알고리즘 영향 없음). 회귀 측정 의무 없음.

### Removed
- `config/qos_override.yaml` — launch 어디에서도 미참조 dead file (P0-6)
- `scripts/config.py`: `SonarMapperConfig.from_ros_params` classmethod — 호출자 0건의 dead code, 또한 depth_estimation 파라미터 5개 누락 (잠재 버그) (P3-3, -66 LOC)
- `scripts/config.py`: `crosstalk.gaussian_sigma` ParameterDef — 마스크 계산에 미참조 (P3-5)
- `scripts/crosstalk_filter.py`: `gaussian_sigma` 생성자 인자·멤버·setter 제거 (P3-5)
- `scripts/3d_mapper.py`: `update_visualization`, `update_orientation` stub 메서드 — 본문 pass, 노드측이 직접 처리 (P3-7, -13 LOC)
- `scripts/config.py`: 4개 ParameterDef에서 `handler='update_(visualization|orientation)'` 제거 (P3-7)

### Changed
- `sonar_3d_reconstruction/cpp/octree_mapper.cpp` / `outofcore_tile_mapper.cpp`: 24줄 동일 정의됐던 `SuppressOutput` RAII를 신규 헤더로 추출 (P3-2)

### Added
- `sonar_3d_reconstruction/cpp/suppress_output.h` — RAII stdout/stderr 억제 클래스 단일 정의. non-copyable·non-movable 제약 명시 (P3-2)

### Verification
- colcon build PASS (Release)
- launch smoke test PASS (`3d_mapping.launch.py` 5초 기동 후 정상 종료)
- 동적 파라미터 set smoke test PASS (`mounting.orientation.pitch` 60° 변경)
- 회귀 측정 의무 없음 (algorithm 영향 0%)

### Notes
- Phase B-1에서 회귀 인프라(`scripts/regression/`)를 신규 작성 예정. 그 후 모든 phase는 baseline-vs-candidate metric 통과 후 머지.
```

- [ ] **Step 3: build (CHANGELOG는 빌드에 영향 없으나 일관성)**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction 2>&1 | tail -3
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 4: commit**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction add CHANGELOG.md
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction commit -m "$(cat <<'EOF'
docs(changelog): add Phase A cleanup entry

refactor-workflow.md의 4축 패턴(branch+commit+CHANGELOG+PR) 동기.
P0-6, P3-2, P3-3, P3-5, P3-7 변경 요약.

- CHANGELOG.md: [Unreleased] Phase A 항목 추가
EOF
)"
```

---

## Task 7: Phase A 통합 검증 + Phase B-1 게이트

**Files:** (read-only)

- [ ] **Step 1: 최종 build 통과 확인 (모든 Task 변경 누적)**

```bash
cd /workspace/ros2_ws && colcon build --packages-select sonar_3d_reconstruction \
  --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tee /tmp/phase_a_build.log | tail -10
```

Expected: `Summary: 1 package finished`. 만약 실패 시 어느 Task에서 회귀 도입됐는지 `git log --oneline refactor/phase-a-cleanup ^origin/main`로 확인 후 해당 commit revert.

- [ ] **Step 2: launch 풀 smoke test (`3d_mapping.launch.py` + `robot_3d_mapping.launch.py`)**

```bash
cd /workspace/ros2_ws && source install/setup.bash

for launch in 3d_mapping.launch.py robot_3d_mapping.launch.py; do
    echo "=== smoke test: $launch ==="
    timeout 8 ros2 launch sonar_3d_reconstruction $launch 2>&1 | \
      grep -iE "error|exception|traceback" | grep -v "no message" || \
      echo "  PASS"
done
```

Expected: 두 launch 모두 `PASS` (관련 error/traceback 없음). "no message" 류는 입력 토픽 미발행으로 정상.

- [ ] **Step 3: commit history 확인 (Phase A는 5+1+1 = 7 commit, 머지 시 squash로 1 commit)**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction log --oneline origin/main..HEAD
```

Expected (예시):
```
<sha7> docs(changelog): add Phase A cleanup entry
<sha6> refactor(mapper_3d): remove dead stub methods (P3-7)
<sha5> refactor(crosstalk): remove unused gaussian_sigma parameter (P3-5)
<sha4> refactor(config): remove dead from_ros_params classmethod (P3-3)
<sha3> refactor(cpp): extract SuppressOutput RAII to shared header (P3-2)
<sha2> chore(qos): remove dead qos_override.yaml (P0-6)
<sha1> docs(design): add quality-perf uplift master design
```

- [ ] **Step 4: 누적 LOC 변화 확인**

```bash
git -C /workspace/ros2_ws/src/sonar_3d_reconstruction diff origin/main..HEAD --stat | tail -5
```

Expected: 약 -120 ~ -150 LOC (spec 추가 +481 분 제외하면 코드만으로 -120~-150).

- [ ] **Step 5: Phase B-1 게이트 — 사용자 보고 + push 지시 대기**

이 시점에 Phase A 변경이 완료됐다. 다음 두 작업은 **사용자 명시 지시 필요**:
1. `git push -u origin refactor/phase-a-cleanup` (push 정책: 명시 지시 필수)
2. PR 생성 (`gh pr create ...`)

사용자에게 보고:
- Phase A 7 commit, ~150 LOC 정리 완료
- 모든 Task의 build/smoke test PASS
- 회귀 측정 의무 없음 (알고리즘 영향 0%)
- 다음 단계: push + PR → Phase B-1 plan 작성

---

## Self-Review

### Spec coverage 점검

| Spec §3 Phase A 항목 | 본 plan Task |
|---------------------|--------------|
| P0-6 qos_override.yaml 삭제 | Task 1 ✅ |
| P3-2 SuppressOutput 추출 | Task 2 ✅ |
| P3-3 from_ros_params 위임/삭제 | Task 3 (위임 → **삭제**로 정정. 호출자 0건 발견) ✅ |
| P3-5 gaussian_sigma 제거 | Task 4 ✅ |
| P3-7 stub 메서드 제거 | Task 5 ✅ |
| CHANGELOG 갱신 | Task 6 ✅ |
| 통합 build PASS | Task 7 ✅ |

### Placeholder 점검
- "TBD"/"TODO"/"implement later" 없음 ✅
- 모든 step에 실제 코드 또는 명령 ✅
- "Add error handling" 같은 모호한 지시 없음 ✅
- launch.py의 `CrosstalkFilter()` 호출은 grep 결과로 위치 확인 후 정확히 적용하도록 안내 ✅

### Type 일관성
- `SuppressOutput`은 두 cpp에서 동일 시그니처(non-copyable/movable)로 표준화 ✅
- `CrosstalkFilter.__init__` 인자 4개로 통일 (caller 동시 변경) ✅

### Risk 재평가
- 모든 변경이 dead/duplicate/unused 코드 제거 → 회귀 의무 없음
- launch smoke test로 노드 기동 확인이 충분
- Q-A1, Q-B1, Q-C1, Q-D1, Q-Data 5개 사용자 결정사항은 Phase A 범위 외(Phase B-2/C/D 전 다시 확인)

---

## Execution Handoff

**Plan complete and saved to `docs/source/plans/2026-05-03-phase-a-cleanup.md`.**

다음 두 가지 실행 모드 중 선택:

1. **Subagent-Driven (권장)** — 각 Task별 독립 subagent, Task 사이에 사용자/메인이 검토. Phase A는 5 Task만이라 빠른 반복.
2. **Inline Execution** — 본 세션에서 직접 7 Task 일괄 실행. 각 Task의 Step 5(commit) 시점에 자연스러운 checkpoint.

Phase A는 위험 0%이고 회귀 측정 의무 없으므로 **Inline Execution**이 효율적이지만, 사용자가 각 Task별 결과 확인을 원하면 Subagent-Driven도 적합.
