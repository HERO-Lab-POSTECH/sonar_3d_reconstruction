#!/usr/bin/env python3
"""
Configuration management for SonarTo3DMapper
Provides dataclass-based parameter management with ROS2 integration

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Callable
import numpy as np
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult


@dataclass
class ParameterDef:
    """Parameter definition for ROS2 parameter declaration and management"""
    name: str                           # Parameter name (e.g., "iwlo.sharpness")
    default: Any                        # Default value (fallback when YAML not present)
    description: str = ""               # Description for ROS2 ParameterDescriptor
    read_only: bool = False             # Immutable at runtime (applied to ParameterDescriptor)
    handler: Optional[str] = None       # Dynamic update handler method name (e.g., "update_intensity")


class ParameterManager:
    """Utility class for managing ROS2 parameters with ParameterDef"""

    @staticmethod
    def declare_all(node: Node, params: List[ParameterDef]) -> None:
        """
        Declare all parameters in the node

        Args:
            node: ROS2 node
            params: List of ParameterDef
        """
        for param in params:
            descriptor = ParameterDescriptor(
                description=param.description,
                read_only=param.read_only
            )
            node.declare_parameter(param.name, param.default, descriptor)

    @staticmethod
    def get_all(node: Node, params: List[ParameterDef]) -> Dict[str, Any]:
        """
        Get all parameter values from the node

        Args:
            node: ROS2 node
            params: List of ParameterDef

        Returns:
            Dictionary mapping parameter names to their values
        """
        result = {}
        for param in params:
            result[param.name] = node.get_parameter(param.name).value
        return result

    @staticmethod
    def create_callback(node: Node, params: List[ParameterDef], target: Any) -> Callable:
        """
        Create a parameter callback function that handles dynamic updates

        Args:
            node: ROS2 node
            params: List of ParameterDef with handler information
            target: Target object (e.g., mapper instance) that has handler methods

        Returns:
            Callback function compatible with add_on_set_parameters_callback
        """
        # Create mapping from parameter name to handler
        handlers_map = {}
        for param in params:
            if param.handler is not None and not param.read_only:
                handlers_map[param.name] = param.handler

        def callback(ros_params):
            """Handle parameter updates"""
            for ros_param in ros_params:
                if ros_param.name in handlers_map:
                    handler_name = handlers_map[ros_param.name]
                    if hasattr(target, handler_name):
                        handler = getattr(target, handler_name)
                        handler(ros_param.value)
                        node.get_logger().debug(f'{ros_param.name} updated: {ros_param.value}')
            return SetParametersResult(successful=True)

        return callback


# Parameter definitions for 3d_mapper_node
MAPPER_PARAMS: List[ParameterDef] = [
    # === Dynamic Parameters (can be changed at runtime) ===

    # Filtering (filtering.*)
    ParameterDef('filtering.min_range', 0.5,
                 'Minimum sonar range in meters',
                 handler='update_min_range'),
    ParameterDef('filtering.intensity_threshold', 35,
                 'Intensity threshold for voxel classification (0-255)',
                 handler='update_intensity'),

    # Mapping (mapping.*)
    ParameterDef('mapping.occupied_threshold', 0.7,
                 'Probability threshold for occupied classification',
                 handler='update_occupied_threshold'),
    ParameterDef('mapping.angular_cone_width', 0.5,
                 'Angular cone width for shadow region protection',
                 handler='update_angular_cone'),

    # Processing (processing.*)
    ParameterDef('processing.frame_skip', 1,
                 'Process every N frames',
                 handler='update_frame_skip'),

    # Visualization (visualization.*)
    ParameterDef('visualization.show_opencv_visualization', False,
                 'Show OpenCV visualization window',
                 handler='update_visualization'),
    ParameterDef('visualization.pointcloud_publish_rate', 10.0,
                 'PointCloud2 publishing rate in Hz (in-memory mode)'),
    ParameterDef('visualization.tile_save_interval', 5.0,
                 'Dirty tile save interval in seconds (out-of-core mode)'),

    # Octree (dynamic)
    ParameterDef('octree.dynamic_expansion', True,
                 'Enable dynamic octree expansion',
                 handler='update_dynamic_expansion'),

    # Mounting orientation (dynamic - can change at runtime)
    ParameterDef('mounting.orientation.roll', 0.0,
                 'Sonar roll angle in degrees',
                 handler='update_orientation'),
    ParameterDef('mounting.orientation.pitch', 90.0,
                 'Sonar pitch angle in degrees (90 = pointing down)',
                 handler='update_orientation'),
    ParameterDef('mounting.orientation.yaw', 0.0,
                 'Sonar yaw angle in degrees',
                 handler='update_orientation'),

    # === Read-only Parameters (cannot change at runtime) ===

    # Sonar hardware (sonar.*)
    ParameterDef('sonar.horizontal_fov', 130.0,
                 'Sonar horizontal field of view in degrees',
                 read_only=True),
    ParameterDef('sonar.vertical_aperture', 20.0,
                 'Sonar vertical aperture in degrees',
                 read_only=True),

    # Mounting position (read-only)
    ParameterDef('mounting.position.x', 0.0,
                 'Sonar X position relative to base_link',
                 read_only=True),
    ParameterDef('mounting.position.y', 0.0,
                 'Sonar Y position relative to base_link',
                 read_only=True),
    ParameterDef('mounting.position.z', -0.5,
                 'Sonar Z position relative to base_link',
                 read_only=True),

    # Octree structure (octree.*)
    ParameterDef('octree.voxel_resolution', 0.05,
                 'Octree voxel resolution in meters',
                 read_only=True),
    ParameterDef('octree.use_cpp_backend', True,
                 'Use C++ backend for octree operations',
                 read_only=True),

    # Adaptive (adaptive.*)
    ParameterDef('adaptive.update', True,
                 'Enable adaptive probability updates',
                 read_only=True),
    ParameterDef('adaptive.threshold', 0.5,
                 'Adaptive update threshold',
                 read_only=True),
    ParameterDef('adaptive.max_ratio', 0.3,
                 'Maximum adaptive ratio',
                 read_only=True),

    # IWLO (iwlo.*) - Unified with 3d_mapper.py and C++ backend
    ParameterDef('iwlo.sharpness', 0.1,
                 'IWLO sigmoid steepness for intensity-to-weight mapping',
                 read_only=True),
    ParameterDef('iwlo.decay_rate', 0.1,
                 'IWLO alpha decay rate for learning rate annealing',
                 read_only=True),
    ParameterDef('iwlo.min_alpha', 0.3,
                 'IWLO minimum learning rate for change detection',
                 read_only=True),
    ParameterDef('iwlo.L_occ', 3.5,
                 'IWLO max occupied log-odds increment',
                 read_only=True),
    ParameterDef('iwlo.L_free', -3.0,
                 'IWLO free space log-odds decrement',
                 read_only=True),
    ParameterDef('iwlo.L_min', -10.0,
                 'IWLO lower saturation bound (P ~ 0.00005)',
                 read_only=True),
    ParameterDef('iwlo.L_max', 10.0,
                 'IWLO upper saturation bound (P ~ 0.99995)',
                 read_only=True),

    # Out-of-Core settings (outofcore.*)
    ParameterDef('outofcore.use', False,
                 'Use out-of-core disk-based storage',
                 read_only=True),
    ParameterDef('outofcore.map_path', '/workspace/data/map_tiles',
                 'Tile storage directory path',
                 read_only=True),
    ParameterDef('outofcore.tile_size', 10.0,
                 'Tile size in meters',
                 read_only=True),
    ParameterDef('outofcore.cache_size', 16,
                 'Maximum tiles in memory cache',
                 read_only=True),

    # Frame IDs (frames.*)
    ParameterDef('frames.sonar', 'sonar_link',
                 'Sonar frame ID',
                 read_only=True),
    ParameterDef('frames.base', 'base_link',
                 'Base frame ID',
                 read_only=True),
    ParameterDef('frames.map', 'map',
                 'Map frame ID',
                 read_only=True),
    ParameterDef('frames.publish_tf', True,
                 'Publish TF transform from base to sonar',
                 read_only=True),

    # Topics (topics.*)
    ParameterDef('topics.sonar', '/sensor/sonar/oculus/m750d/image',
                 'Sonar image topic',
                 read_only=True),
    ParameterDef('topics.odometry', '/fast_lio/odometry',
                 'Odometry topic',
                 read_only=True),
    ParameterDef('topics.pointcloud', '/sonar_3d_map',
                 'PointCloud2 output topic',
                 read_only=True),
    ParameterDef('topics.marker', '/sonar_3d_map_markers',
                 'Marker array output topic',
                 read_only=True),

    # Recording (recording.*)
    ParameterDef('recording.bag', False,
                 'Enable bag recording',
                 read_only=True),
    ParameterDef('recording.base_path', '/workspace/data/experiments',
                 'Base directory for recordings',
                 read_only=True),
    ParameterDef('recording.prefix', 'test',
                 'Recording folder prefix',
                 read_only=True),
]

# Map visualizer parameters
VISUALIZER_PARAMS: List[ParameterDef] = [
    # === Dynamic Parameters ===
    ParameterDef('mapping.occupied_threshold', 0.7,
                 'Probability threshold for occupied voxels',
                 handler='update_occupied_threshold'),
    ParameterDef('visualization.mode', 0,
                 '0=pointcloud, 1=octomap, 2=all',
                 handler='update_visualization_mode'),

    # === Read-only Parameters ===
    ParameterDef('outofcore.map_path', '/workspace/data/map_tiles',
                 'Tile storage directory path',
                 read_only=True),
    ParameterDef('frames.map', 'camera_init',
                 'Map frame ID',
                 read_only=True),
    # Fallback values when metadata.json not found
    ParameterDef('octree.voxel_resolution', 0.1,
                 'Fallback voxel resolution (reads from metadata.json if available)',
                 read_only=True),
    ParameterDef('outofcore.tile_size', 10.0,
                 'Fallback tile size (reads from metadata.json if available)',
                 read_only=True),
    ParameterDef('publish_rate', 1.0,
                 'Map publishing rate in Hz',
                 read_only=True),
    ParameterDef('refresh_interval', 5.0,
                 'Tile refresh interval in seconds',
                 read_only=True),
    ParameterDef('auto_refresh', True,
                 'Enable automatic tile refresh',
                 read_only=True),
]



@dataclass
class SonarMapperConfig:
    """Complete configuration for SonarTo3DMapper"""

    # Sonar parameters
    horizontal_fov: float = 130.0
    vertical_aperture: float = 20.0
    min_range: float = 0.5
    intensity_threshold: int = 35
    # max_range is received dynamically from /param/range topic

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
        # Create main config with namespaced parameters
        config = cls(
            # Sonar hardware (sonar.*)
            horizontal_fov=node.get_parameter('sonar.horizontal_fov').value,
            vertical_aperture=node.get_parameter('sonar.vertical_aperture').value,

            # Filtering (filtering.*)
            min_range=node.get_parameter('filtering.min_range').value,
            intensity_threshold=node.get_parameter('filtering.intensity_threshold').value,

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

            # Processing (processing.*)
            frame_skip=node.get_parameter('processing.frame_skip').value
        )
        return config

    @classmethod
    def from_params_dict(cls, params_dict: Dict[str, Any]) -> 'SonarMapperConfig':
        """
        Create configuration from parameter dictionary

        Args:
            params_dict: Dictionary from ParameterManager.get_all()

        Returns:
            SonarMapperConfig instance
        """
        # Create main config
        config = cls(
            # Sonar hardware
            horizontal_fov=params_dict.get('sonar.horizontal_fov', 130.0),
            vertical_aperture=params_dict.get('sonar.vertical_aperture', 20.0),

            # Filtering
            min_range=params_dict.get('filtering.min_range', 0.5),
            intensity_threshold=params_dict.get('filtering.intensity_threshold', 35),

            # Mounting
            sonar_position=[
                params_dict.get('mounting.position.x', 0.0),
                params_dict.get('mounting.position.y', 0.0),
                params_dict.get('mounting.position.z', -0.5)
            ],
            sonar_orientation=[
                np.radians(params_dict.get('mounting.orientation.roll', 0.0)),
                np.radians(params_dict.get('mounting.orientation.pitch', 90.0)),
                np.radians(params_dict.get('mounting.orientation.yaw', 0.0))
            ],

            # Octree
            voxel_resolution=params_dict.get('octree.voxel_resolution', 0.05),
            dynamic_expansion=params_dict.get('octree.dynamic_expansion', True),
            use_cpp_backend=params_dict.get('octree.use_cpp_backend', True),

            # Adaptive
            adaptive_update=params_dict.get('adaptive.update', True),
            adaptive_threshold=params_dict.get('adaptive.threshold', 0.5),
            adaptive_max_ratio=params_dict.get('adaptive.max_ratio', 0.3),

            # Mapping
            occupied_threshold=params_dict.get('mapping.occupied_threshold', 0.7),
            angular_cone_width=params_dict.get('mapping.angular_cone_width', 0.5),

            # IWLO (using method_iwlo.yaml defaults)
            sharpness=params_dict.get('iwlo.sharpness', 0.1),
            decay_rate=params_dict.get('iwlo.decay_rate', 0.1),
            min_alpha=params_dict.get('iwlo.min_alpha', 0.3),
            L_occ=params_dict.get('iwlo.L_occ', 2.0),
            L_free=params_dict.get('iwlo.L_free', -4.0),
            L_min=params_dict.get('iwlo.L_min', -12.0),
            L_max=params_dict.get('iwlo.L_max', 8.0),

            # Outofcore
            use_outofcore=params_dict.get('outofcore.use', False),
            outofcore_map_path=params_dict.get('outofcore.map_path', '/workspace/data/map_tiles'),
            outofcore_tile_size=params_dict.get('outofcore.tile_size', 10.0),
            outofcore_cache_size=params_dict.get('outofcore.cache_size', 16),

            # Processing
            frame_skip=params_dict.get('processing.frame_skip', 1)
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

            'frame_skip': self.frame_skip
        }
