#!/usr/bin/env python3
"""
ROS2 Node for 3D Sonar Mapping
Subscribes to sonar images and odometry, publishes 3D map as PointCloud2

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import time
import struct

# ROS2 message imports
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int32MultiArray, Float32
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import MarkerArray, Marker
from tf2_ros import StaticTransformBroadcaster

# Message filters for time synchronization
from message_filters import Subscriber, ApproximateTimeSynchronizer

# Parameter callback and descriptor
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

# OpenCV for image processing
from cv_bridge import CvBridge
import cv2

# Import core mapping class and configuration
import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import importlib.util
spec = importlib.util.spec_from_file_location("mapper_3d",
    os.path.join(os.path.dirname(__file__), "3d_mapper.py"))
mapper_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mapper_module)
SonarTo3DMapper = mapper_module.SonarTo3DMapper

spec_config = importlib.util.spec_from_file_location("config",
    os.path.join(os.path.dirname(__file__), "config.py"))
config_module = importlib.util.module_from_spec(spec_config)
spec_config.loader.exec_module(config_module)
SonarMapperConfig = config_module.SonarMapperConfig


def get_next_test_number(base_path: str, prefix: str) -> int:
    """Find existing test folders and return next available number."""
    if not os.path.exists(base_path):
        return 1

    existing = [d for d in os.listdir(base_path)
                if d.startswith(f'{prefix}_') and os.path.isdir(os.path.join(base_path, d))]

    numbers = []
    for d in existing:
        try:
            num = int(d[len(prefix)+1:])  # +1 for underscore
            numbers.append(num)
        except ValueError:
            continue

    return max(numbers, default=0) + 1


class SonarMapperNode(Node):
    """ROS2 node for 3D sonar mapping with probabilistic octree"""
    
    def __init__(self):
        super().__init__('sonar_3d_mapper')

        # === Step 1: Declare conditional feature flags first (read-only) ===
        self.declare_parameter('robot_detection.enabled', False,
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('crosstalk.enabled', False,
                               ParameterDescriptor(read_only=True))
        self.enable_robot_detection = self.get_parameter('robot_detection.enabled').value
        self.enable_crosstalk = self.get_parameter('crosstalk.enabled').value

        # Declare parameters with default values (lowest priority - priority 4)
        # These defaults will be overridden by YAML, launch file, or command line args
        self.declare_parameters(
            namespace='',
            parameters=[
                # === Dynamic Parameters (can be changed at runtime) ===
                # Filtering (filtering.*)
                ('filtering.min_range', 0.5),
                ('filtering.intensity_threshold', 35),

                # Mapping (mapping.*)
                ('mapping.occupied_threshold', 0.7),
                ('mapping.angular_cone_width', 0.5),

                # Processing (processing.*)
                ('processing.frame_skip', 1),

                # Visualization (visualization.*)
                ('visualization.show_opencv_visualization', False),
                ('visualization.pointcloud_publish_rate', 10.0),
                ('visualization.tile_save_interval', 5.0),

                # Octree (octree.*)
                ('octree.dynamic_expansion', True),

                # Mounting orientation (dynamic - can change at runtime)
                ('mounting.orientation.roll', 0.0),
                ('mounting.orientation.pitch', 90.0),
                ('mounting.orientation.yaw', 0.0),

                # === Read-only Parameters (cannot change at runtime) ===
                # Sonar hardware (sonar.*)
                ('sonar.horizontal_fov', 130.0, ParameterDescriptor(read_only=True)),
                ('sonar.vertical_aperture', 20.0, ParameterDescriptor(read_only=True)),

                # Mounting position (read-only)
                ('mounting.position.x', 0.0, ParameterDescriptor(read_only=True)),
                ('mounting.position.y', 0.0, ParameterDescriptor(read_only=True)),
                ('mounting.position.z', -0.5, ParameterDescriptor(read_only=True)),

                # Octree structure (octree.*)
                ('octree.voxel_resolution', 0.05, ParameterDescriptor(read_only=True)),
                ('octree.use_cpp_backend', True, ParameterDescriptor(read_only=True)),

                # Adaptive (adaptive.*)
                ('adaptive.update', True, ParameterDescriptor(read_only=True)),
                ('adaptive.threshold', 0.5, ParameterDescriptor(read_only=True)),
                ('adaptive.max_ratio', 0.3, ParameterDescriptor(read_only=True)),

                # IWLO (iwlo.*)
                ('iwlo.sharpness', 3.0, ParameterDescriptor(read_only=True)),
                ('iwlo.decay_rate', 0.1, ParameterDescriptor(read_only=True)),
                ('iwlo.min_alpha', 0.1, ParameterDescriptor(read_only=True)),
                ('iwlo.L_occ', 3.5, ParameterDescriptor(read_only=True)),
                ('iwlo.L_free', -3.0, ParameterDescriptor(read_only=True)),
                ('iwlo.L_min', -10.0, ParameterDescriptor(read_only=True)),
                ('iwlo.L_max', 10.0, ParameterDescriptor(read_only=True)),

                # Out-of-Core settings (outofcore.*)
                ('outofcore.use', False, ParameterDescriptor(read_only=True)),
                ('outofcore.map_path', '/workspace/data/map_tiles', ParameterDescriptor(read_only=True)),
                ('outofcore.tile_size', 10.0, ParameterDescriptor(read_only=True)),
                ('outofcore.cache_size', 16, ParameterDescriptor(read_only=True)),

                # Frame IDs (frames.*)
                ('frames.sonar', 'sonar_link', ParameterDescriptor(read_only=True)),
                ('frames.base', 'base_link', ParameterDescriptor(read_only=True)),
                ('frames.map', 'map', ParameterDescriptor(read_only=True)),
                ('frames.publish_tf', True, ParameterDescriptor(read_only=True)),

                # Topics (topics.*)
                ('topics.sonar', '/sensor/sonar/oculus/m750d/image', ParameterDescriptor(read_only=True)),
                ('topics.odometry', '/fast_lio/odometry', ParameterDescriptor(read_only=True)),
                ('topics.pointcloud', '/sonar_3d_map', ParameterDescriptor(read_only=True)),
                ('topics.marker', '/sonar_3d_map_markers', ParameterDescriptor(read_only=True)),

                # Recording (recording.*)
                ('recording.bag', False, ParameterDescriptor(read_only=True)),
                ('recording.base_path', '/workspace/data/experiments', ParameterDescriptor(read_only=True)),
                ('recording.prefix', 'test', ParameterDescriptor(read_only=True))
            ]
        )

        # === Step 2: Conditional parameters (robot_detection) ===
        if self.enable_robot_detection:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('terrain_detection.min_threshold', 80),
                    ('terrain_detection.max_threshold', 180),
                    ('robot_detection.min_threshold', 180),
                    ('robot_detection.topic', '/sonar_robot_detections', ParameterDescriptor(read_only=True)),
                ]
            )
            self.get_logger().info('Robot detection enabled')

        # === Step 3: Conditional parameters (crosstalk) ===
        if self.enable_crosstalk:
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('crosstalk.morpho_enabled', True),
                    ('crosstalk.morpho_kernel_size', 5),
                    ('crosstalk.morpho_kernel_shape', 'rect'),
                    ('crosstalk.azimuth_enabled', True),
                    ('crosstalk.azimuth_threshold', 0.5),
                ]
            )
            self.get_logger().info('Crosstalk filter enabled')
        
        # Load configuration from ROS2 parameters using dataclass
        config_dataclass = SonarMapperConfig.from_ros_params(self)
        config = config_dataclass.to_mapper_dict()
        
        # Get other parameters (with namespaced names)
        self.sonar_frame_id = self.get_parameter('frames.sonar').value
        self.base_frame_id = self.get_parameter('frames.base').value
        self.map_frame_id = self.get_parameter('frames.map').value
        self.publish_tf = self.get_parameter('frames.publish_tf').value
        self.show_opencv_visualization = self.get_parameter('visualization.show_opencv_visualization').value
        self.pointcloud_publish_rate = self.get_parameter('visualization.pointcloud_publish_rate').value
        self.tile_save_interval = self.get_parameter('visualization.tile_save_interval').value

        # Get topic names (with namespaced names)
        sonar_topic = self.get_parameter('topics.sonar').value
        odometry_topic = self.get_parameter('topics.odometry').value
        pointcloud_topic = self.get_parameter('topics.pointcloud').value
        marker_topic = self.get_parameter('topics.marker').value

        # Auto-generate range_topic from sonar_topic: /sensor/sonar/.../image -> /sensor/sonar/.../param/range
        if sonar_topic.endswith('/image'):
            range_topic = sonar_topic.rsplit('/image', 1)[0] + '/param/range'
        else:
            range_topic = sonar_topic + '/param/range'
        self.get_logger().info(f'Auto-generated range_topic: {range_topic}')
        
        # Store robot detection settings (topic only available when enabled)
        # Note: self.enable_robot_detection already set at Step 1
        self.robot_detection_topic = config['robot_detection']['topic'] if self.enable_robot_detection else None

        # Store out-of-core mode flag
        self.use_outofcore = config['use_outofcore']

        # Initialize mapper
        self.mapper = SonarTo3DMapper(config)
        
        # Backend info (silent - available via get_memory_stats())
        
        # Create CV bridge for image conversion
        self.bridge = CvBridge()
        
        # Initialize static TF broadcaster if enabled
        if self.publish_tf:
            from tf2_ros import StaticTransformBroadcaster
            self.tf_static_broadcaster = StaticTransformBroadcaster(self)
            self.sonar_position = config['sonar_position']
            self.sonar_orientation = config['sonar_orientation']
            # Publish static transform once
            self.publish_static_tf()
        
        # Initialize latest_odometry to None
        self.latest_odometry = None
        
        # Frame counter
        self.frame_count = 0
        self.frame_skip = config['frame_skip']
        self.last_publish_time = time.time()

        # Register parameter change callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)
        
        # QoS profile for best effort subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create synchronized subscribers using message_filters
        self.sonar_sub = Subscriber(
            self,
            Image,
            sonar_topic,
            qos_profile=qos_profile
        )
        
        self.odom_sub = Subscriber(
            self,
            Odometry,
            odometry_topic,
            qos_profile=qos_profile
        )
        
        # Create time synchronizer with 0.1 second tolerance
        self.time_sync = ApproximateTimeSynchronizer(
            [self.sonar_sub, self.odom_sub],
            queue_size=10,
            slop=0.1  # 100ms tolerance for time synchronization
        )
        self.time_sync.registerCallback(self.synchronized_callback)
        
        # Create publishers
        self.pc_pub = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            10
        )
        
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10
        )
        
        # Create robot detection publisher if enabled
        if self.enable_robot_detection:
            self.robot_pub = self.create_publisher(
                PointCloud2,
                self.robot_detection_topic,
                10
            )
        else:
            self.robot_pub = None

        # Subscribe to dynamic range topic from sonar driver
        self.range_sub = self.create_subscription(
            Float32,
            range_topic,
            self.range_callback,
            qos_profile
        )
        self.get_logger().info(f'Subscribed to range topic: {range_topic}')

        # Create timer for periodic publishing
        if not self.use_outofcore:
            # In-memory mode: configurable pointcloud publishing rate
            publish_interval = 1.0 / self.pointcloud_publish_rate
            self.timer = self.create_timer(publish_interval, self.publish_pointcloud)
        else:
            # Out-of-core mode: eviction-based + periodic saving
            self.timer = None
            self.tile_update_pub = self.create_publisher(
                Int32MultiArray,
                '/updated_tile_indices',
                10
            )
            # Periodically save dirty tiles + notify visualizer
            self.flush_timer = self.create_timer(self.tile_save_interval, self.periodic_flush_and_notify)
        
        # No need for TF timer anymore since we use static transform
        
        # Initialization summary (single line)
        mode_str = "out-of-core" if self.use_outofcore else "in-memory"
        self.get_logger().info(
            f'Mapper initialized: {config["voxel_resolution"]}m res, '
            f'{config["horizontal_fov"]}°x{config["vertical_aperture"]}° FOV, '
            f'{mode_str} mode'
        )
    
    def parameter_callback(self, params):
        """
        Handle dynamic parameter updates at runtime.
        All non-read-only parameters are supported.
        """
        # Track which C++ backend updates are needed
        update_intensity = False
        update_orientation = False

        for param in params:
            name = param.name
            value = param.value

            # === Filtering ===
            if name == 'filtering.intensity_threshold':
                self.mapper.intensity_threshold = int(value)
                update_intensity = True
            elif name == 'filtering.min_range':
                self.mapper.min_range = float(value)

            # === Mapping ===
            elif name == 'mapping.occupied_threshold':
                self.mapper.occupied_threshold = float(value)
            elif name == 'mapping.angular_cone_width':
                self.mapper.angular_cone_width = float(value)

            # === Processing ===
            elif name == 'processing.frame_skip':
                self.frame_skip = int(value)
                self.mapper.frame_skip = int(value)

            # === Visualization ===
            elif name == 'visualization.show_opencv_visualization':
                self.show_opencv_visualization = bool(value)
            elif name == 'visualization.pointcloud_publish_rate':
                self.pointcloud_publish_rate = float(value)
                # Note: Timer recreation would require more complex logic
            elif name == 'visualization.tile_save_interval':
                self.tile_save_interval = float(value)

            # === Octree ===
            elif name == 'octree.dynamic_expansion':
                self.mapper.dynamic_expansion = bool(value)
                # Note: currently stored but not actively used in mapping logic

            # === Mounting Orientation ===
            elif name == 'mounting.orientation.roll':
                self.mapper.sonar_orientation[0] = np.radians(float(value))
                update_orientation = True
            elif name == 'mounting.orientation.pitch':
                self.mapper.sonar_orientation[1] = np.radians(float(value))
                update_orientation = True
            elif name == 'mounting.orientation.yaw':
                self.mapper.sonar_orientation[2] = np.radians(float(value))
                update_orientation = True

            # === Terrain/Robot Detection (conditional) ===
            elif name == 'terrain_detection.min_threshold':
                self.mapper.terrain_min_threshold = int(value)
            elif name == 'terrain_detection.max_threshold':
                self.mapper.terrain_max_threshold = int(value)
            elif name == 'robot_detection.min_threshold':
                self.mapper.robot_min_threshold = int(value)

            # === Crosstalk (conditional) - update actual filter object ===
            elif name == 'crosstalk.morpho_enabled':
                if self.mapper.crosstalk_filter is not None:
                    self.mapper.crosstalk_filter.morpho_enabled = bool(value)
            elif name == 'crosstalk.morpho_kernel_size':
                if self.mapper.crosstalk_filter is not None:
                    self.mapper.crosstalk_filter.kernel_size = int(value)
            elif name == 'crosstalk.morpho_kernel_shape':
                if self.mapper.crosstalk_filter is not None:
                    self.mapper.crosstalk_filter.kernel_shape = str(value)
            elif name == 'crosstalk.azimuth_enabled':
                if self.mapper.crosstalk_filter is not None:
                    self.mapper.crosstalk_filter.azimuth_check_enabled = bool(value)
            elif name == 'crosstalk.azimuth_threshold':
                if self.mapper.crosstalk_filter is not None:
                    self.mapper.crosstalk_filter.consistency_threshold = float(value)

            self.get_logger().debug(f'{name} updated: {value}')

        # Apply C++ backend updates
        if update_intensity and hasattr(self.mapper, 'octree') and self.mapper.octree is not None:
            self.mapper.octree.set_intensity_params(
                self.mapper.intensity_threshold,
                getattr(self.mapper, 'intensity_max', 255)
            )

        # Update transform matrix and TF if orientation changed
        if update_orientation:
            self.mapper.update_sonar_orientation()
            if self.publish_tf:
                self.sonar_orientation = self.mapper.sonar_orientation.copy()
                self.publish_static_tf()
            self.get_logger().info('Sonar orientation updated')

        return SetParametersResult(successful=True)

    def _decode_sonar_image(self, sonar_msg: Image) -> np.ndarray:
        """
        Decode ROS Image message to numpy array

        Args:
            sonar_msg: ROS Image message

        Returns:
            Decoded image as numpy array, or None on error
        """
        try:
            if sonar_msg.encoding == 'mono8' or sonar_msg.encoding == '8UC1':
                return self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding='mono8')
            elif sonar_msg.encoding == 'mono16' or sonar_msg.encoding == '16UC1':
                img16 = self.bridge.imgmsg_to_cv2(sonar_msg, desired_encoding='mono16')
                return (img16 / 256).astype(np.uint8)
            else:
                self.get_logger().error(f'Unsupported encoding: {sonar_msg.encoding}')
                return None
        except Exception as e:
            self.get_logger().error(f'Image decode failed: {e}')
            return None

    def _visualize_sonar_frame(self, sonar_image: np.ndarray):
        """
        Visualize sonar frame with threshold overlay

        Args:
            sonar_image: Grayscale sonar image
        """
        threshold = self.mapper.intensity_threshold
        thresholded = np.where(sonar_image > threshold, 255, 0).astype(np.uint8)

        original_colored = cv2.cvtColor(sonar_image, cv2.COLOR_GRAY2BGR)

        thresholded_colored = np.zeros_like(original_colored)
        thresholded_colored[:, :, 2] = thresholded

        overlay = cv2.addWeighted(original_colored, 0.6, thresholded_colored, 0.4, 0)

        cv2.putText(overlay, f"Intensity Threshold: {threshold}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(overlay, f"Frame: {self.frame_count}", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        combined = np.hstack([original_colored, overlay])

        cv2.imshow("Sonar: Original | Threshold Applied", combined)
        cv2.imshow("Binary Threshold", thresholded)
        cv2.waitKey(1)

    def range_callback(self, msg: Float32):
        """
        Update max_range dynamically from sonar driver

        Args:
            msg: Float32 message containing current sonar range in meters
        """
        new_range = msg.data
        if new_range > 0 and new_range != self.mapper.max_range:
            old_range = self.mapper.max_range
            self.mapper.update_max_range(new_range)
            if old_range is None:
                self.get_logger().info(f'max_range received: {new_range:.1f}m')
            else:
                self.get_logger().info(f'max_range updated: {old_range:.1f}m -> {new_range:.1f}m')

    def synchronized_callback(self, sonar_msg: Image, odom_msg: Odometry):
        """
        Process synchronized sonar image and odometry data

        Args:
            sonar_msg: Sonar image message
            odom_msg: Odometry message
        """
        self.frame_count += 1

        # Frame skipping logic - check BEFORE decoding
        if self.frame_count % self.frame_skip != 0:
            return

        # Check if max_range has been received from /param/range topic
        if self.mapper.max_range is None:
            if self.frame_count % 50 == 1:  # Log warning every 50 frames
                self.get_logger().warn(
                    'Waiting for max_range from /param/range topic. '
                    'Sonar frames will be skipped until range is received.'
                )
            return

        # Decode image (only for processed frames)
        sonar_image = self._decode_sonar_image(sonar_msg)
        if sonar_image is None:
            return
        
        # Extract odometry position and orientation
        position = [
            odom_msg.pose.pose.position.x,
            odom_msg.pose.pose.position.y,
            odom_msg.pose.pose.position.z
        ]
        
        orientation = [
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        ]
        
        # Process the sonar image
        stats = self.mapper.process_sonar_image(sonar_image, position, orientation)

        # Show visualization if enabled
        if self.show_opencv_visualization:
            self._visualize_sonar_frame(sonar_image)

        # Store latest odometry for TF publishing
        self.latest_odometry = odom_msg
        
        # Log statistics periodically (every 100 frames to reduce log noise)
        if not stats.get('skipped', False) and self.frame_count % 100 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {stats["num_voxels"]} voxels, '
                f'{stats["processing_time"]*1000:.1f}ms'
            )

        # Out-of-core mode: notify tiles saved via eviction
        if self.use_outofcore:
            self.notify_saved_tiles()
    
    def publish_static_tf(self):
        """Publish static TF transform from base_link to sonar_link"""
        if not self.publish_tf:
            return
        
        # Create static transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame_id
        t.child_frame_id = self.sonar_frame_id
        
        # Set translation
        t.transform.translation.x = self.sonar_position[0]
        t.transform.translation.y = self.sonar_position[1]
        t.transform.translation.z = self.sonar_position[2]
        
        # Convert RPY to quaternion
        roll, pitch, yaw = self.sonar_orientation
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)
        
        t.transform.rotation.w = cr * cp * cy + sr * sp * sy
        t.transform.rotation.x = sr * cp * cy - cr * sp * sy
        t.transform.rotation.y = cr * sp * cy + sr * cp * sy
        t.transform.rotation.z = cr * cp * sy - sr * sp * cy
        
        # Send static transform (silent)
        self.tf_static_broadcaster.sendTransform(t)
    
    def notify_saved_tiles(self):
        """Notify visualizer of tile indices saved via eviction (out-of-core mode)"""
        if not self.use_outofcore:
            return

        if hasattr(self.mapper, 'get_and_clear_saved_tiles'):
            # Get tiles that were saved via eviction
            saved_tiles = self.mapper.get_and_clear_saved_tiles()

            # Publish saved tile indices if any (silent)
            if len(saved_tiles) > 0 and hasattr(self, 'tile_update_pub'):
                msg = Int32MultiArray()
                # Pack as [x1, y1, z1, x2, y2, z2, ...]
                for tile_idx in saved_tiles:
                    msg.data.extend([tile_idx.x, tile_idx.y, tile_idx.z])
                self.tile_update_pub.publish(msg)

    def periodic_flush_and_notify(self):
        """Periodically save all dirty tiles and notify visualizer (out-of-core mode)"""
        if not self.use_outofcore:
            return

        # Flush all dirty tiles to disk and get their indices
        if hasattr(self.mapper, 'flush_map_and_get_dirty_tiles'):
            flushed_tiles = self.mapper.flush_map_and_get_dirty_tiles()

            # Publish flushed tile indices
            if len(flushed_tiles) > 0 and hasattr(self, 'tile_update_pub'):
                msg = Int32MultiArray()
                for tile_idx in flushed_tiles:
                    msg.data.extend([tile_idx.x, tile_idx.y, tile_idx.z])
                self.tile_update_pub.publish(msg)

    def publish_pointcloud(self):
        """Publish accumulated point cloud"""
        # Skip in out-of-core mode (use map_visualizer_node instead)
        if self.use_outofcore:
            return

        # Get point cloud from mapper (occupied voxels only)
        result = self.mapper.get_point_cloud(include_free=False)

        # Publish as PointCloud2
        if result['num_occupied'] > 0:
            self.publish_pointcloud2(result['points'], result['probabilities'])
        
        # Publish robot detections if enabled
        if self.enable_robot_detection:
            robot_detections = result.get('robot_detections', [])
            if len(robot_detections) > 0:
                self.publish_robot_detections(robot_detections)
    
    def publish_pointcloud2(self, points: np.ndarray, probabilities: np.ndarray):
        """
        Publish PointCloud2 message with intensity as probability
        
        Args:
            points: Nx3 array of points
            probabilities: N array of probabilities
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame_id
        
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        
        # Pack data
        data = []
        for i in range(len(points)):
            data.append(struct.pack('ffff',
                                   points[i, 0], points[i, 1], points[i, 2],
                                   probabilities[i]))
        
        cloud.data = b''.join(data)
        
        # Publish
        self.pc_pub.publish(cloud)
    
    def publish_robot_detections(self, robot_points):
        """
        Publish robot detection PointCloud2 message
        
        Args:
            robot_points: List of tuples (point, intensity) for robot detections
        """
        if not self.robot_pub or len(robot_points) == 0:
            return
        
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame_id
        
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]
        
        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(robot_points)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True
        
        # Pack data
        data = []
        for point, intensity in robot_points:
            data.append(struct.pack('ffff',
                                   point[0], point[1], point[2], intensity))
        
        cloud.data = b''.join(data)
        
        # Publish
        self.robot_pub.publish(cloud)
    
    def publish_marker_array(self, result: dict):
        """
        Publish MarkerArray with colored voxels
        
        Args:
            result: Dictionary with classified voxels
        """
        marker_array = MarkerArray()
        marker_id = 0
        
        # Create marker for occupied voxels (red)
        if len(result['occupied']) > 0:
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.mapper.voxel_resolution
            marker.scale.y = self.mapper.voxel_resolution
            marker.scale.z = self.mapper.voxel_resolution
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            for point, prob in result['occupied']:
                p = marker.points.add()
                p.x, p.y, p.z = point
            
            marker_array.markers.append(marker)
            marker_id += 1

        # Create marker for unknown voxels (yellow)
        if len(result.get('unknown', [])) > 0:
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.mapper.voxel_resolution
            marker.scale.y = self.mapper.voxel_resolution
            marker.scale.z = self.mapper.voxel_resolution
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.5
            
            for point, prob in result['unknown']:
                p = marker.points.add()
                p.x, p.y, p.z = point
            
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = SonarMapperNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final statistics and flush tiles
        if node:
            try:
                result = node.mapper.get_point_cloud()
                memory_stats = node.mapper.get_memory_stats()

                node.get_logger().info(
                    f'Shutdown: {result["processed_count"]}/{result["frame_count"]} frames, '
                    f'{result["num_occupied"]}/{result["num_voxels"]} occupied voxels'
                )

                # Flush all dirty tiles to disk on shutdown (out-of-core mode)
                if hasattr(node.mapper, 'flush_map'):
                    node.mapper.flush_map()
                    node.notify_saved_tiles()

                node.destroy_node()
            except Exception as e:
                print(f"Shutdown error: {e}")

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()