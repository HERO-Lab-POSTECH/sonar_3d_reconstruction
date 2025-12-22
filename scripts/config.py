#!/usr/bin/env python3
"""
Configuration management for SonarTo3DMapper
Provides dataclass-based parameter management with ROS2 integration

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional
import numpy as np


@dataclass
class TerrainDetectionConfig:
    """Terrain detection parameters for robot detection mode"""
    min_threshold: int = 80
    max_threshold: int = 180


@dataclass
class RobotDetectionConfig:
    """Robot detection parameters"""
    min_threshold: int = 180
    topic: str = '/sonar_robot_detections'


@dataclass
class CrosstalkConfig:
    """Crosstalk filter parameters"""
    enabled: bool = False
    morpho_enabled: bool = True
    morpho_kernel_size: int = 5
    morpho_kernel_shape: str = 'rect'
    azimuth_enabled: bool = True
    azimuth_threshold: float = 0.5


@dataclass
class SonarMapperConfig:
    """Complete configuration for SonarTo3DMapper"""

    # Sonar parameters
    horizontal_fov: float = 130.0
    vertical_aperture: float = 20.0
    min_range: float = 0.5
    intensity_threshold: int = 35
    # max_range is received dynamically from /param/range topic

    # Terrain/robot detection
    terrain_detection: TerrainDetectionConfig = field(default_factory=TerrainDetectionConfig)
    enable_robot_detection: bool = False
    robot_detection: RobotDetectionConfig = field(default_factory=RobotDetectionConfig)

    # Sonar mounting (relative to base_link)
    sonar_position: List[float] = field(default_factory=lambda: [0.0, 0.0, -0.5])
    sonar_orientation: List[float] = field(default_factory=lambda: [0.0, 1.5708, 0.0])

    # Octree parameters
    voxel_resolution: float = 0.05
    dynamic_expansion: bool = True

    # Adaptive update
    adaptive_update: bool = True
    adaptive_threshold: float = 0.5
    adaptive_max_ratio: float = 0.3

    # Probability threshold
    occupied_threshold: float = 0.7

    # Shadow region protection
    angular_cone_width: float = 0.5

    # IWLO parameters
    sharpness: float = 3.0
    decay_rate: float = 0.1
    min_alpha: float = 0.1
    L_occ: float = 3.5
    L_free: float = -3.0
    L_min: float = -10.0
    L_max: float = 10.0
    intensity_max: int = 255

    # Backend selection
    use_cpp_backend: bool = True

    # Out-of-Core parameters
    use_outofcore: bool = False
    outofcore_map_path: str = '/workspace/data/map_tiles'
    outofcore_tile_size: float = 10.0
    outofcore_cache_size: int = 16

    # Cross-talk filter
    crosstalk: CrosstalkConfig = field(default_factory=CrosstalkConfig)

    # Processing parameters
    frame_skip: int = 1

    @classmethod
    def from_ros_params(cls, node) -> 'SonarMapperConfig':
        """
        Create configuration from ROS2 node parameters

        Args:
            node: ROS2 node with declared parameters

        Returns:
            SonarMapperConfig instance
        """
        # Check conditional feature flags
        enable_robot_detection = node.get_parameter('robot_detection.enabled').value
        enable_crosstalk = node.get_parameter('crosstalk.enabled').value

        # Extract robot detection config (conditional)
        if enable_robot_detection:
            terrain_config = TerrainDetectionConfig(
                min_threshold=node.get_parameter('terrain_detection.min_threshold').value,
                max_threshold=node.get_parameter('terrain_detection.max_threshold').value
            )
            robot_config = RobotDetectionConfig(
                min_threshold=node.get_parameter('robot_detection.min_threshold').value,
                topic=node.get_parameter('robot_detection.topic').value
            )
        else:
            terrain_config = TerrainDetectionConfig()
            robot_config = RobotDetectionConfig()

        # Extract crosstalk config (conditional)
        if enable_crosstalk:
            crosstalk_config = CrosstalkConfig(
                enabled=True,
                morpho_enabled=node.get_parameter('crosstalk.morpho_enabled').value,
                morpho_kernel_size=node.get_parameter('crosstalk.morpho_kernel_size').value,
                morpho_kernel_shape=node.get_parameter('crosstalk.morpho_kernel_shape').value,
                azimuth_enabled=node.get_parameter('crosstalk.azimuth_enabled').value,
                azimuth_threshold=node.get_parameter('crosstalk.azimuth_threshold').value,
            )
        else:
            crosstalk_config = CrosstalkConfig(enabled=False)

        # Create main config with namespaced parameters
        config = cls(
            # Sonar hardware (sonar.*)
            horizontal_fov=node.get_parameter('sonar.horizontal_fov').value,
            vertical_aperture=node.get_parameter('sonar.vertical_aperture').value,

            # Filtering (filtering.*)
            min_range=node.get_parameter('filtering.min_range').value,
            intensity_threshold=node.get_parameter('filtering.intensity_threshold').value,

            terrain_detection=terrain_config,
            enable_robot_detection=enable_robot_detection,
            robot_detection=robot_config,

            # Mounting (mounting.position.*, mounting.orientation.*)
            sonar_position=[
                node.get_parameter('mounting.position.x').value,
                node.get_parameter('mounting.position.y').value,
                node.get_parameter('mounting.position.z').value
            ],
            sonar_orientation=[
                np.radians(node.get_parameter('mounting.orientation.roll').value),
                np.radians(node.get_parameter('mounting.orientation.pitch').value),
                np.radians(node.get_parameter('mounting.orientation.yaw').value)
            ],

            # Octree (octree.*)
            voxel_resolution=node.get_parameter('octree.voxel_resolution').value,
            dynamic_expansion=node.get_parameter('octree.dynamic_expansion').value,
            use_cpp_backend=node.get_parameter('octree.use_cpp_backend').value,

            # Adaptive (adaptive.*)
            adaptive_update=node.get_parameter('adaptive.update').value,
            adaptive_threshold=node.get_parameter('adaptive.threshold').value,
            adaptive_max_ratio=node.get_parameter('adaptive.max_ratio').value,

            # Mapping (mapping.*)
            occupied_threshold=node.get_parameter('mapping.occupied_threshold').value,
            angular_cone_width=node.get_parameter('mapping.angular_cone_width').value,

            # IWLO (iwlo.*)
            sharpness=node.get_parameter('iwlo.sharpness').value,
            decay_rate=node.get_parameter('iwlo.decay_rate').value,
            min_alpha=node.get_parameter('iwlo.min_alpha').value,
            L_occ=node.get_parameter('iwlo.L_occ').value,
            L_free=node.get_parameter('iwlo.L_free').value,
            L_min=node.get_parameter('iwlo.L_min').value,
            L_max=node.get_parameter('iwlo.L_max').value,

            # Outofcore (outofcore.*)
            use_outofcore=node.get_parameter('outofcore.use').value,
            outofcore_map_path=node.get_parameter('outofcore.map_path').value,
            outofcore_tile_size=node.get_parameter('outofcore.tile_size').value,
            outofcore_cache_size=node.get_parameter('outofcore.cache_size').value,

            # Crosstalk
            crosstalk=crosstalk_config,

            # Processing (processing.*)
            frame_skip=node.get_parameter('processing.frame_skip').value
        )
        return config

    def to_mapper_dict(self) -> Dict[str, Any]:
        """
        Convert to dictionary format compatible with SonarTo3DMapper constructor

        Returns:
            Configuration dictionary
        """
        return {
            'horizontal_fov': self.horizontal_fov,
            'vertical_aperture': self.vertical_aperture,
            'min_range': self.min_range,
            'intensity_threshold': self.intensity_threshold,
            # max_range is set dynamically from /param/range topic

            'terrain_detection': {
                'min_threshold': self.terrain_detection.min_threshold,
                'max_threshold': self.terrain_detection.max_threshold
            },

            'enable_robot_detection': self.enable_robot_detection,
            'robot_detection': {
                'min_threshold': self.robot_detection.min_threshold,
                'topic': self.robot_detection.topic
            },

            'sonar_position': self.sonar_position,
            'sonar_orientation': self.sonar_orientation,

            'voxel_resolution': self.voxel_resolution,
            'dynamic_expansion': self.dynamic_expansion,

            'adaptive_update': self.adaptive_update,
            'adaptive_threshold': self.adaptive_threshold,
            'adaptive_max_ratio': self.adaptive_max_ratio,

            'occupied_threshold': self.occupied_threshold,
            'angular_cone_width': self.angular_cone_width,

            'sharpness': self.sharpness,
            'decay_rate': self.decay_rate,
            'min_alpha': self.min_alpha,
            'L_occ': self.L_occ,
            'L_free': self.L_free,
            'L_min': self.L_min,
            'L_max': self.L_max,
            'intensity_max': self.intensity_max,

            'use_cpp_backend': self.use_cpp_backend,

            'use_outofcore': self.use_outofcore,
            'outofcore_map_path': self.outofcore_map_path,
            'outofcore_tile_size': self.outofcore_tile_size,
            'outofcore_cache_size': self.outofcore_cache_size,

            'crosstalk_filter_enabled': self.crosstalk.enabled,
            'morpho_filter_enabled': self.crosstalk.morpho_enabled,
            'morpho_kernel_size': self.crosstalk.morpho_kernel_size,
            'morpho_kernel_shape': self.crosstalk.morpho_kernel_shape,
            'azimuth_check_enabled': self.crosstalk.azimuth_enabled,
            'azimuth_consistency_threshold': self.crosstalk.azimuth_threshold,

            'frame_skip': self.frame_skip
        }
