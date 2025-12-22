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
        
        # Declare parameters with default values (lowest priority - priority 4)
        # These defaults will be overridden by YAML, launch file, or command line args
        self.declare_parameters(
            namespace='',
            parameters=[
                # Sonar parameters
                ('horizontal_fov', 130.0),
                ('vertical_aperture', 20.0),
                ('max_range', 10.0),
                ('min_range', 0.5),
                ('intensity_threshold', 35),
                
                # Terrain detection (for robot detection mode)
                ('terrain_detection.min_threshold', 80),
                ('terrain_detection.max_threshold', 180),
                
                # Robot detection mode
                ('enable_robot_detection', False),
                ('robot_detection.min_threshold', 180),
                ('robot_detection.topic', '/sonar_robot_detections'),
                
                # Sonar mounting (relative to base_link)
                ('sonar_position.x', 0.0),
                ('sonar_position.y', 0.0),
                ('sonar_position.z', -0.5),
                ('sonar_orientation.roll', 0.0),  # degrees
                ('sonar_orientation.pitch', 90.0),  # degrees (90 = pointing down)
                ('sonar_orientation.yaw', 0.0),  # degrees
                
                # Octree parameters
                ('voxel_resolution', 0.05),
                ('dynamic_expansion', True),
                ('z_filter_min', -5.0),
                ('z_filter_enabled', True),
                
                # Adaptive update parameters
                ('adaptive_update', True),
                ('adaptive_threshold', 0.5),
                ('adaptive_max_ratio', 0.3),

                # Probability threshold (2-class: occupied vs free)
                ('occupied_threshold', 0.7),  # Probability >= 0.7 = occupied, < 0.7 = free

                # Shadow region protection
                ('angular_cone_width', 0.5),  # 0.5 = 0% overlap, 1.0 = full overlap

                # IWLO (Intensity-Weighted Log-Odds) parameters
                ('sharpness', 3.0),      # Sigmoid steepness for intensity-to-weight (1.0~5.0)
                ('decay_rate', 0.1),     # Learning rate decay rate (0.05~0.5)
                ('min_alpha', 0.1),      # Minimum learning rate for change detection (0.01~0.3)
                ('L_occ', 3.5),          # Log-odds occupied increment
                ('L_free', -3.0),        # Log-odds free decrement
                ('L_min', -10.0),        # Saturation lower bound
                ('L_max', 10.0),         # Saturation upper bound

                # Backend selection
                ('use_cpp_backend', True),  # Use high-performance C++ hierarchical octree by default

                # Out-of-Core parameters (disk-based storage for large maps)
                ('use_outofcore', False),   # Enable disk-based tile storage
                ('outofcore_map_path', '/workspace/data/map_tiles'),  # Tile storage directory
                ('outofcore_tile_size', 10.0),   # Tile size in meters
                ('outofcore_cache_size', 16),    # Max tiles in memory

                # Cross-talk filter parameters
                ('crosstalk_filter_enabled', False),
                ('morpho_filter_enabled', True),
                ('morpho_kernel_size', 5),
                ('morpho_kernel_shape', 'rect'),
                ('azimuth_check_enabled', True),
                ('azimuth_consistency_threshold', 0.5),

                # Processing parameters
                ('frame_skip', 1),  # Process every N frames

                # Publishing parameters
                ('show_free_space', False),
                
                # Frame IDs
                ('sonar_frame_id', 'sonar_link'),
                ('base_frame_id', 'base_link'),
                ('map_frame_id', 'map'),
                ('publish_tf', True),
                
                # Topics
                ('sonar_topic', '/sensor/sonar/oculus/m750d/image'),
                ('odometry_topic', '/fast_lio/odometry'),
                ('pointcloud_topic', '/sonar_3d_map'),
                ('marker_topic', '/sonar_3d_map_markers'),
                ('range_topic', '/sensor/sonar/oculus/param/range'),  # Dynamic range from sonar

                # Visualization
                ('show_opencv_visualization', False),
                ('pointcloud_publish_rate', 10.0),  # Hz
                ('tile_save_interval', 5.0),        # seconds

                # Bag recording (auto-increment)
                ('record_bag', False),
                ('record_base_path', '/workspace/data/experiments'),
                ('record_prefix', 'test')
            ]
        )
        
        # Load configuration from ROS2 parameters using dataclass
        config_dataclass = SonarMapperConfig.from_ros_params(self)
        config = config_dataclass.to_mapper_dict()
        
        # Get other parameters
        self.show_free_space = self.get_parameter('show_free_space').value
        self.sonar_frame_id = self.get_parameter('sonar_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.map_frame_id = self.get_parameter('map_frame_id').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.show_opencv_visualization = self.get_parameter('show_opencv_visualization').value
        self.pointcloud_publish_rate = self.get_parameter('pointcloud_publish_rate').value
        self.tile_save_interval = self.get_parameter('tile_save_interval').value

        # Get topic names
        sonar_topic = self.get_parameter('sonar_topic').value
        odometry_topic = self.get_parameter('odometry_topic').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        range_topic = self.get_parameter('range_topic').value
        
        # Store robot detection settings
        self.enable_robot_detection = config['enable_robot_detection']
        self.robot_detection_topic = config['robot_detection']['topic']

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
            # Out-of-core mode: eviction 기반 + 주기적 저장
            self.timer = None
            self.tile_update_pub = self.create_publisher(
                Int32MultiArray,
                '/updated_tile_indices',
                10
            )
            # 주기적으로 dirty 타일 저장 + visualizer 알림
            self.flush_timer = self.create_timer(self.tile_save_interval, self.periodic_flush_and_notify)
        
        # No need for TF timer anymore since we use static transform
        
        # Initialization summary (single line)
        mode_str = "out-of-core" if self.use_outofcore else "in-memory"
        self.get_logger().info(
            f'Mapper initialized: {config["voxel_resolution"]}m res, '
            f'{config["horizontal_fov"]}°x{config["vertical_aperture"]}° FOV, '
            f'{mode_str} mode'
        )
    
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

        # Out-of-core 모드: eviction으로 저장된 타일 알림
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
        """Eviction으로 저장된 타일 인덱스를 visualizer에 알림 (out-of-core 모드)"""
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
        """5초 주기로 모든 dirty 타일을 저장하고 visualizer에 알림 (out-of-core 모드)"""
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

        # Get point cloud from mapper
        result = self.mapper.get_point_cloud(include_free=self.show_free_space)
        
        if self.show_free_space:
            # Publish as marker array with colors
            self.publish_marker_array(result)
        else:
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
        
        # Create marker for free voxels (blue) if enabled
        if self.show_free_space and len(result['free']) > 0:
            marker = Marker()
            marker.header.frame_id = self.map_frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = marker_id
            marker.type = Marker.CUBE_LIST
            marker.action = Marker.ADD
            marker.scale.x = self.mapper.voxel_resolution
            marker.scale.y = self.mapper.voxel_resolution
            marker.scale.z = self.mapper.voxel_resolution
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 0.3
            
            for point, prob in result['free']:
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