# Parameter Centralization Plan

## Current Problem

Parameter declarations are scattered across multiple nodes:

| File | Lines | Parameters |
|------|-------|------------|
| `3d_mapper_node.py` | 80-184, 348-430 | ~30 params + callback |
| `map_visualizer_node.py` | 77-91, 166-180 | ~10 params + callback |
| `world_init_broadcaster_node.py` | 64-67 | 4 params |

### Issues

1. **Duplication**: Same parameter patterns repeated in each node
2. **Inconsistency**: Default values may differ between nodes
3. **Maintenance burden**: Adding a parameter requires changes in multiple places
4. **Documentation sync**: config.md must be manually kept in sync
5. **Naming inconsistency**: ROS2 parameter names differ from C++ internal variable names

---

## Proposed Solution

Centralize all parameter definitions in `config.py` with:
- Parameter metadata (name, type, default, description, read_only)
- Dynamic update handlers
- Automatic declare/callback registration

### Proposed Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  config.py                                                   │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ParameterDef                                        │    │
│  │  - name: str                                         │    │
│  │  - type: ParameterType                               │    │
│  │  - default: Any                                      │    │
│  │  - description: str                                  │    │
│  │  - read_only: bool                                   │    │
│  │  - handler: Callable (for dynamic updates)          │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  MAPPER_PARAMS: List[ParameterDef]                   │    │
│  │  VISUALIZER_PARAMS: List[ParameterDef]               │    │
│  │  WORLD_INIT_PARAMS: List[ParameterDef]               │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  ParameterManager                                    │    │
│  │  - declare_all(node, param_list)                     │    │
│  │  - create_callback(node, param_list)                 │    │
│  │  - get_all_values(node, param_list) -> dict          │    │
│  └─────────────────────────────────────────────────────┘    │
└─────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────┐
│  Nodes (simplified)                                          │
│                                                              │
│  class SonarMapperNode(Node):                               │
│      def __init__(self):                                    │
│          ParameterManager.declare_all(self, MAPPER_PARAMS)  │
│          self.add_on_set_parameters_callback(               │
│              ParameterManager.create_callback(self, ...)    │
│          )                                                  │
└─────────────────────────────────────────────────────────────┘
```

### ParameterDef Structure (Draft)

```python
from dataclasses import dataclass
from typing import Any, Callable, Optional
from rcl_interfaces.msg import ParameterDescriptor

@dataclass
class ParameterDef:
    name: str
    default: Any
    description: str = ""
    read_only: bool = False
    # Handler receives (node, new_value) -> None
    handler: Optional[Callable] = None

    def to_descriptor(self) -> ParameterDescriptor:
        return ParameterDescriptor(
            description=self.description,
            read_only=self.read_only
        )
```

### Parameter Definition Example

```python
# config.py

MAPPER_PARAMS = [
    # Filtering
    ParameterDef(
        name='filtering.intensity_threshold',
        default=35,
        description='Minimum intensity to consider as occupied',
        handler=lambda node, v: setattr(node.mapper, 'intensity_threshold', int(v))
    ),
    ParameterDef(
        name='filtering.min_range',
        default=0.5,
        description='Minimum range filter (m)',
        handler=lambda node, v: setattr(node.mapper, 'min_range', float(v))
    ),

    # Read-only parameters
    ParameterDef(
        name='octree.voxel_resolution',
        default=0.05,
        description='Voxel size in meters',
        read_only=True
    ),
    # ... more parameters
]
```

### ParameterManager Implementation (Draft)

```python
class ParameterManager:
    @staticmethod
    def declare_all(node: Node, params: List[ParameterDef]):
        """Declare all parameters from definition list."""
        for p in params:
            node.declare_parameter(p.name, p.default, p.to_descriptor())

    @staticmethod
    def get_all(node: Node, params: List[ParameterDef]) -> dict:
        """Get all parameter values as dict."""
        return {p.name: node.get_parameter(p.name).value for p in params}

    @staticmethod
    def create_callback(node: Node, params: List[ParameterDef]):
        """Create parameter callback that dispatches to handlers."""
        handlers = {p.name: p.handler for p in params if p.handler}

        def callback(param_list):
            for param in param_list:
                if param.name in handlers and handlers[param.name]:
                    handlers[param.name](node, param.value)
            return SetParametersResult(successful=True)

        return callback
```

### Node Usage (After Refactoring)

```python
# 3d_mapper_node.py (simplified)

from config import MAPPER_PARAMS, ParameterManager

class SonarMapperNode(Node):
    def __init__(self):
        super().__init__('sonar_3d_mapper')

        # One-line parameter setup
        ParameterManager.declare_all(self, MAPPER_PARAMS)
        self.add_on_set_parameters_callback(
            ParameterManager.create_callback(self, MAPPER_PARAMS)
        )

        # Get all params as dict
        params = ParameterManager.get_all(self, MAPPER_PARAMS)
        self.mapper = SonarTo3DMapper(**params)
```

---

## Implementation Steps

### Phase 1: Create config.py Infrastructure

1. [ ] Create `ParameterDef` dataclass
2. [ ] Create `ParameterManager` class
3. [ ] Add unit tests for ParameterManager

### Phase 2: Migrate 3d_mapper_node.py

1. [ ] Define `MAPPER_PARAMS` in config.py
2. [ ] Refactor 3d_mapper_node.py to use ParameterManager
3. [ ] Verify all parameters work correctly
4. [ ] Test dynamic parameter updates

### Phase 3: Migrate Other Nodes

1. [ ] Define `VISUALIZER_PARAMS` and migrate map_visualizer_node.py
2. [ ] Define `WORLD_INIT_PARAMS` and migrate world_init_broadcaster_node.py
3. [ ] Integration tests

### Phase 4: Documentation Sync

1. [ ] Auto-generate config.md from config.py (optional)
2. [ ] Update existing documentation
3. [ ] Remove deprecated parameter declarations

---

## Benefits

1. **Single Source of Truth**: All parameters defined in one file
2. **Easy Maintenance**: Add/modify parameters in one place
3. **Consistent Defaults**: No risk of different defaults in different nodes
4. **Self-Documenting**: Parameter definitions include descriptions
5. **Reduced Boilerplate**: Nodes become simpler
6. **Documentation Sync**: Can auto-generate docs from code

---

## Parameter Naming Inconsistency

### Current Problem

IWLO parameter names differ between ROS2 interface and C++ implementation:

| ROS2 Parameter | C++ Variable | Documentation |
|----------------|--------------|---------------|
| `iwlo.L_occ` | `log_odds_occupied_` | L_occ |
| `iwlo.L_free` | `log_odds_free_` | L_free |
| `iwlo.L_min` | `L_min_` | L_min |
| `iwlo.L_max` | `L_max_` | L_max |

### Recommendation

**Option 1: Unify to short names (L_occ, L_free)**
- Pros: Matches documentation, shorter, consistent with academic notation
- Cons: Less descriptive

**Option 2: Unify to descriptive names (log_odds_occupied, log_odds_free)**
- Pros: Self-documenting, clearer meaning
- Cons: Longer, requires documentation update

### Proposed Standard

Use **short names** for consistency with IWLO design document:
- ROS2: `iwlo.L_occ`, `iwlo.L_free`, `iwlo.L_min`, `iwlo.L_max`
- C++: Rename internal variables to `L_occ_`, `L_free_`, `L_min_`, `L_max_`
- Python: Use same names in dataclass

### Implementation

1. [ ] Rename C++ variables in `probability_updater.cpp/.h`
2. [ ] Rename C++ variables in `tile.cpp/.h`
3. [ ] Update Python config.py to use consistent names
4. [ ] Verify all documentation uses same naming

---

## Notes

- Maintain backward compatibility during migration
- Consider using YAML-based parameter files for default values
- Handler functions should be pure and testable

---

**Created**: 2024-12-24
**Status**: Planning
