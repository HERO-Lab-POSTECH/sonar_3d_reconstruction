cmake_minimum_required(VERSION 3.8)
project(sonar_3d_reconstruction)

# Default to C++17 (required for structured bindings)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2 -fno-strict-aliasing)
endif()

# Add debugging symbols for better error reporting
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g -O0)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(octomap REQUIRED)

# Build C++ extension module
add_library(sonar_3d_reconstruction_cpp MODULE
  sonar_3d_reconstruction/cpp/python_bindings.cpp
  sonar_3d_reconstruction/cpp/octree_mapper.cpp
  sonar_3d_reconstruction/cpp/iwlo_updater.cpp
  sonar_3d_reconstruction/cpp/octree_storage.cpp
  sonar_3d_reconstruction/cpp/probability_updater.cpp
  sonar_3d_reconstruction/cpp/tile.cpp
  sonar_3d_reconstruction/cpp/tile_manager.cpp
  sonar_3d_reconstruction/cpp/outofcore_tile_mapper.cpp
)

target_include_directories(sonar_3d_reconstruction_cpp PRIVATE
  sonar_3d_reconstruction/cpp
  ${EIGEN3_INCLUDE_DIR}
  ${pybind11_INCLUDE_DIR}
  ${Python3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

target_link_libraries(sonar_3d_reconstruction_cpp PRIVATE
  Eigen3::Eigen
  Python3::Python
  ${OCTOMAP_LIBRARIES}
)

# Set proper extension for Python modules
execute_process(
  COMMAND ${Python3_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_config_var('EXT_SUFFIX'))"
  OUTPUT_VARIABLE PYTHON_EXT_SUFFIX
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set_target_properties(sonar_3d_reconstruction_cpp PROPERTIES
  PREFIX ""
  SUFFIX "${PYTHON_EXT_SUFFIX}"
)

# Install to Python package directory
install(TARGETS sonar_3d_reconstruction_cpp
  LIBRARY DESTINATION local/lib/python3.10/dist-packages/sonar_3d_reconstruction
)

# Install __init__.py from source (contains C++ module loader)
install(FILES
  sonar_3d_reconstruction/__init__.py
  DESTINATION local/lib/python3.10/dist-packages/sonar_3d_reconstruction
)

# Add environment hook to set PYTHONPATH
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.dsv.in"
)

# Install Python executable scripts (install(PROGRAMS) automatically sets executable permissions)
install(PROGRAMS
  scripts/3d_mapper_node.py
  scripts/map_visualizer_node.py
  scripts/map_diff_analyzer.py
  scripts/map_diff_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (non-executable Python files)
install(FILES
  scripts/3d_mapper.py
  scripts/config.py
  scripts/crosstalk_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz configuration files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()