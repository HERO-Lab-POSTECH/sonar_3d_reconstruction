# sonar_3d_reconstruction package

"""
Sonar 3D Reconstruction Package

This package provides C++ backend modules for 3D sonar mapping.
"""

import warnings

# C++ module loading
CPP_MODULE_AVAILABLE = False
OUTOFCORE_AVAILABLE = False
ProbabilityUpdater = None
MemoryStats = None
OutofcoreTileMapper = None
TileIndex = None

try:
    import sys
    import importlib.util

    install_path = "/workspace/ros2_ws/install/sonar_3d_reconstruction/local/lib/python3.10/dist-packages"
    cpp_file = f"{install_path}/sonar_3d_reconstruction/sonar_3d_reconstruction_cpp.cpython-310-x86_64-linux-gnu.so"

    spec = importlib.util.spec_from_file_location("sonar_3d_reconstruction_cpp", cpp_file)
    cpp_module = importlib.util.module_from_spec(spec)
    sys.modules["sonar_3d_reconstruction_cpp"] = cpp_module
    spec.loader.exec_module(cpp_module)

    ProbabilityUpdater = cpp_module.ProbabilityUpdater
    MemoryStats = cpp_module.MemoryStats
    OutofcoreTileMapper = cpp_module.OutofcoreTileMapper
    TileIndex = cpp_module.TileIndex

    CPP_MODULE_AVAILABLE = True
    OUTOFCORE_AVAILABLE = True
except Exception as e:
    warnings.warn(f"[sonar_3d_reconstruction] C++ module unavailable: {e}")

__all__ = [
    'CPP_MODULE_AVAILABLE',
    'OUTOFCORE_AVAILABLE',
    'ProbabilityUpdater',
    'MemoryStats',
    'OutofcoreTileMapper',
    'TileIndex',
]
