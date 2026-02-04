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
ParameterManager = config_module.ParameterManager
MAPPER_PARAMS = config_module.MAPPER_PARAMS
ROBOT_DETECTION_PARAMS = config_module.ROBOT_DETECTION_PARAMS


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

        # === Step 1: Declare all parameters using ParameterManager ===
        ParameterManager.declare_all(self, MAPPER_PARAMS)

        # === Step 2: Create configuration from parameters ===
        params_dict = ParameterManager.get_all(self, MAPPER_PARAMS)
        config_dataclass = SonarMapperConfig.from_params_dict(params_dict)
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
        
        # Create publishers (use same QoS for consistency)
        self.pc_pub = self.create_publisher(
            PointCloud2,
            pointcloud_topic,
            qos_profile
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            qos_profile
        )

        # Subscribe to dynamic range topic from sonar driver
        self.range_sub = self.create_subscription(
            Float32,
            range_topic,
            self.range_callback,
            qos_profile
        )

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
                qos_profile
            )
            # Periodically save dirty tiles + notify visualizer
            self.flush_timer = self.create_timer(self.tile_save_interval, self.periodic_flush_and_notify)
        
        # No need for TF timer anymore since we use static transform
        
        # Initialization summary (single line)
        mode_str = "out-of-core" if self.use_outofcore else "in-memory"
        self.get_logger().info(
            f'Mapper: {config["voxel_resolution"]}m, '
            f'{config["horizontal_fov"]}°x{config["vertical_aperture"]}° FOV, '
            f'{mode_str}'
        )
    
    def parameter_callback(self, params):
        """
        Handle dynamic parameter updates at runtime.
        All non-read-only parameters are supported.
        """
        # Track special cases that need additional processing
        update_orientation = False
        orientation_changed = False

        for param in params:
            name = param.name
            value = param.value

            # === Standard parameters (delegated to mapper) ===
            if name == 'filtering.min_range':
                self.mapper.update_min_range(value)
            elif name == 'filtering.intensity_threshold':
                self.mapper.update_intensity(value)
            elif name == 'mapping.occupied_threshold':
                self.mapper.update_occupied_threshold(value)
            elif name == 'mapping.angular_cone_width':
                self.mapper.update_angular_cone(value)
            elif name == 'processing.frame_skip':
                self.frame_skip = int(value)
                self.mapper.update_frame_skip(value)
            elif name == 'octree.dynamic_expansion':
                self.mapper.update_dynamic_expansion(value)

            # === Node-level parameters ===
            elif name == 'visualization.show_opencv_visualization':
                self.show_opencv_visualization = bool(value)
            elif name == 'visualization.pointcloud_publish_rate':
                new_rate = float(value)
                if new_rate != self.pointcloud_publish_rate and not self.use_outofcore:
                    self.pointcloud_publish_rate = new_rate
                    # Recreate timer with new rate
                    if self.timer is not None:
                        self.timer.cancel()
                    publish_interval = 1.0 / self.pointcloud_publish_rate
                    self.timer = self.create_timer(publish_interval, self.publish_pointcloud)
                    self.get_logger().info(f'Publish rate changed to {new_rate}Hz')
            elif name == 'visualization.tile_save_interval':
                new_interval = float(value)
                if new_interval != self.tile_save_interval and self.use_outofcore:
                    self.tile_save_interval = new_interval
                    # Recreate flush timer with new interval
                    if hasattr(self, 'flush_timer') and self.flush_timer is not None:
                        self.flush_timer.cancel()
                    self.flush_timer = self.create_timer(self.tile_save_interval, self.periodic_flush_and_notify)
                    self.get_logger().info(f'Tile save interval changed to {new_interval}s')

            # === Mounting Orientation (requires TF update) ===
            elif name == 'mounting.orientation.roll':
                self.mapper.sonar_orientation[0] = np.radians(float(value))
                orientation_changed = True
            elif name == 'mounting.orientation.pitch':
                self.mapper.sonar_orientation[1] = np.radians(float(value))
                orientation_changed = True
            elif name == 'mounting.orientation.yaw':
                self.mapper.sonar_orientation[2] = np.radians(float(value))
                orientation_changed = True

            self.get_logger().debug(f'{name} updated: {value}')

        # Update transform matrix and TF if orientation changed
        if orientation_changed:
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