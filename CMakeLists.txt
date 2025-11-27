cmake_minimum_required(VERSION 3.8)
project(sonar_3d_reconstruction)

# Default to C++17 (required for structured bindings)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Find pybind11 for C++ optimization modules
find_package(pybind11 REQUIRED)

# Find Eigen3 for coordinate transformations
find_package(Eigen3 REQUIRED)

# Find OctoMap for octree mapping
find_package(octomap REQUIRED)

# Find OpenMP for parallel processing
find_package(OpenMP)

# Include directories
include_directories(
  sonar_3d_reconstruction/cpp
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# Install Python executable scripts (install(PROGRAMS) automatically sets executable permissions)
install(PROGRAMS
  scripts/3d_mapper_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules (non-executable Python files)
install(FILES
  scripts/3d_mapper.py
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

# Install header files (if any C++ headers exist)
install(DIRECTORY
  sonar_3d_reconstruction/cpp/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING PATTERN "*.h"
)

# Build pybind11 optimization modules
# Octree Mapper module (OctoMap-based)
if(pybind11_FOUND AND EIGEN3_FOUND AND octomap_FOUND)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # First create a static library for OctreeMapping (for use by other modules)
  add_library(octree_mapper_core STATIC sonar_3d_reconstruction/cpp/octree_mapper.cpp)
  target_link_libraries(octree_mapper_core PUBLIC
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )
  target_include_directories(octree_mapper_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${OCTOMAP_INCLUDE_DIRS}
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )
  # Enable Position Independent Code for linking into shared library
  set_target_properties(octree_mapper_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(octree_mapper_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_options(octree_mapper_core PUBLIC ${OpenMP_CXX_FLAGS})
  endif()

  # Then create Python module that wraps the static library
  add_library(octree_mapper MODULE sonar_3d_reconstruction/cpp/octree_mapper.cpp)

  target_link_libraries(octree_mapper PRIVATE
    pybind11::module
    pybind11::lto
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )

  # Add OpenMP if available
  if(OpenMP_CXX_FOUND)
    target_link_libraries(octree_mapper PRIVATE OpenMP::OpenMP_CXX)
    target_compile_options(octree_mapper PRIVATE ${OpenMP_CXX_FLAGS})
    message(STATUS "Octree Mapper: OpenMP enabled for parallel sonar processing")
  else()
    message(WARNING "Octree Mapper: OpenMP not found, sonar processing will be single-threaded")
  endif()

  target_include_directories(octree_mapper PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )

  set_target_properties(octree_mapper PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS octree_mapper
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "Octree Mapper module will be built with OctoMap and Python ${Python3_VERSION}")
else()
  message(WARNING "pybind11, Eigen3, or OctoMap not found, Octree Mapper C++ module will not be built")
endif()

# Ray Processor module (High-performance sonar ray processing)
if(pybind11_FOUND AND EIGEN3_FOUND AND octomap_FOUND AND TARGET octree_mapper_core)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Manual library creation for ROS2
  add_library(ray_processor MODULE sonar_3d_reconstruction/cpp/ray_processor.cpp)

  target_link_libraries(ray_processor PRIVATE
    pybind11::module
    pybind11::lto
    octree_mapper_core
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )

  # Add OpenMP if available
  if(OpenMP_CXX_FOUND)
    target_link_libraries(ray_processor PRIVATE OpenMP::OpenMP_CXX)
    target_compile_options(ray_processor PRIVATE ${OpenMP_CXX_FLAGS})
    message(STATUS "Ray Processor: OpenMP enabled for parallel bearing processing")
  else()
    message(WARNING "Ray Processor: OpenMP not found, bearing processing will be single-threaded")
  endif()

  target_include_directories(ray_processor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )

  set_target_properties(ray_processor PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS ray_processor
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "Ray Processor module will be built with Eigen, OctoMap and Python ${Python3_VERSION}")
else()
  message(WARNING "pybind11, Eigen3, or OctoMap not found, Ray Processor C++ module will not be built")
endif()

# ProbabilityUpdater module (OctoMap-based memory efficient probability updater)
if(pybind11_FOUND AND EIGEN3_FOUND AND octomap_FOUND)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Create static library for probability updater core (for reuse)
  add_library(probability_updater_core STATIC sonar_3d_reconstruction/cpp/probability_updater.cpp)
  target_link_libraries(probability_updater_core PUBLIC
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )
  target_include_directories(probability_updater_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
  )
  # Enable Position Independent Code for linking into shared library
  set_target_properties(probability_updater_core PROPERTIES
    POSITION_INDEPENDENT_CODE ON
  )
  if(OpenMP_CXX_FOUND)
    target_link_libraries(probability_updater_core PUBLIC OpenMP::OpenMP_CXX)
    target_compile_options(probability_updater_core PUBLIC ${OpenMP_CXX_FLAGS})
  endif()

  message(STATUS "ProbabilityUpdater core library built with OctoMap and Eigen3")
else()
  message(WARNING "pybind11, Eigen3, or OctoMap not found, ProbabilityUpdater C++ module will not be built")
endif()

# Sonar 3D Reconstruction C++ module (unified module with all components)
if(pybind11_FOUND AND EIGEN3_FOUND AND octomap_FOUND AND TARGET probability_updater_core)
  # Get Python include directories
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

  # Create static core library for unified module
  add_library(sonar_3d_reconstruction_core STATIC
    sonar_3d_reconstruction/cpp/python_bindings.cpp
    sonar_3d_reconstruction/cpp/octree_mapper.cpp
    sonar_3d_reconstruction/cpp/ray_casting.cpp
    sonar_3d_reconstruction/cpp/probability_updater.cpp
  )

  target_link_libraries(sonar_3d_reconstruction_core PUBLIC
    probability_updater_core
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )

  target_include_directories(sonar_3d_reconstruction_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )

  # Create unified Python module
  add_library(sonar_3d_reconstruction_cpp MODULE sonar_3d_reconstruction/cpp/python_bindings.cpp)

  target_link_libraries(sonar_3d_reconstruction_cpp PRIVATE
    pybind11::module
    pybind11::lto
    sonar_3d_reconstruction_core
    Eigen3::Eigen
    ${OCTOMAP_LIBRARIES}
    Python3::Module
  )

  # Add OpenMP if available
  if(OpenMP_CXX_FOUND)
    target_link_libraries(sonar_3d_reconstruction_core PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(sonar_3d_reconstruction_cpp PRIVATE OpenMP::OpenMP_CXX)
    target_compile_options(sonar_3d_reconstruction_core PUBLIC ${OpenMP_CXX_FLAGS})
    target_compile_options(sonar_3d_reconstruction_cpp PRIVATE ${OpenMP_CXX_FLAGS})
    message(STATUS "Sonar 3D Reconstruction: OpenMP enabled for parallel processing")
  else()
    message(WARNING "Sonar 3D Reconstruction: OpenMP not found, processing will be single-threaded")
  endif()

  target_include_directories(sonar_3d_reconstruction_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sonar_3d_reconstruction/cpp
    ${pybind11_INCLUDE_DIRS}
    ${Python3_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
  )

  # Add compile definitions for pybind11 support
  target_compile_definitions(sonar_3d_reconstruction_cpp PRIVATE
    PYBIND11_DETAILED_ERROR_MESSAGES
  )

  set_target_properties(sonar_3d_reconstruction_cpp PROPERTIES
    PREFIX "${PYTHON_MODULE_PREFIX}"
    SUFFIX "${PYTHON_MODULE_EXTENSION}"
    CXX_VISIBILITY_PRESET "hidden"
  )

  # Install to Python package directory
  install(TARGETS sonar_3d_reconstruction_cpp
    LIBRARY DESTINATION local/lib/python3.10/dist-packages/${PROJECT_NAME}
  )

  message(STATUS "Sonar 3D Reconstruction unified module will be built")
  message(STATUS "  - ProbabilityUpdater: OctoMap-based memory efficient updates")
  message(STATUS "  - Python version: ${Python3_VERSION}")
else()
  message(WARNING "Dependencies not found, Sonar 3D Reconstruction C++ module will not be built")
  message(WARNING "  Required: pybind11, Eigen3, OctoMap, and probability_updater_core")
endif()

# Temporarily disable coordinate_transform module due to conflicts with pybind11/std
message(STATUS "Coordinate Transform module temporarily disabled due to std::chrono conflicts")
message(STATUS "ProbabilityUpdater is the primary optimized component")

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
