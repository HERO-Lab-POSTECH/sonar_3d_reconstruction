#!/usr/bin/env python3
"""
ROS2 Node for 3D Sonar Mapping
Subscribes to sonar images and odometry, publishes 3D map as PointCloud2

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sonar_3d_reconstruction.qos import SENSOR_QOS, RELIABLE_QOS, LATCHED_QOS
from std_srvs.srv import Trigger
from sonar_3d_reconstruction.map_save import resolve_map_save_path, update_latest_symlink
from sonar_3d_reconstruction.odom_buffer import OdomBuffer
from sonar_3d_reconstruction.timesync_diagnostics import TimesyncDiagnostics
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
import numpy as np
import time
from copy import deepcopy

# ROS2 message imports
from sensor_msgs.msg import Image, PointCloud2, PointField
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Int32MultiArray, Float32
from geometry_msgs.msg import TransformStamped, Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from diagnostic_msgs.msg import DiagnosticArray
from tf2_ros import StaticTransformBroadcaster

# Message filters (kept for reference, using latest-odometry sync instead)
# from message_filters import Subscriber, ApproximateTimeSynchronizer
import threading

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

spec_filter = importlib.util.spec_from_file_location("crosstalk_filter",
    os.path.join(os.path.dirname(__file__), "crosstalk_filter.py"))
filter_module = importlib.util.module_from_spec(spec_filter)
spec_filter.loader.exec_module(filter_module)
CrosstalkFilter = filter_module.CrosstalkFilter


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
        self.marker_min_depth = self.get_parameter('visualization.marker_min_depth').value
        self.marker_max_depth = self.get_parameter('visualization.marker_max_depth').value
        self.marker_alpha = self.get_parameter('visualization.marker_alpha').value

        # Get topic names (with namespaced names)
        sonar_topic = self.get_parameter('topics.sonar').value
        odometry_topic = self.get_parameter('topics.odometry').value
        slam_confidence_topic = self.get_parameter('topics.slam_confidence').value

        # Auto-generate range_topic from sonar_topic: /sensor/sonar/.../image -> /sensor/sonar/.../param/range
        if sonar_topic.endswith('/image'):
            range_topic = sonar_topic.rsplit('/image', 1)[0] + '/param/range'
        else:
            range_topic = sonar_topic + '/param/range'

        # Initialize crosstalk filter
        self.crosstalk_filter = CrosstalkFilter(
            enabled=self.get_parameter('crosstalk.enabled').value,
            filter_width=self.get_parameter('crosstalk.filter_width').value,
            filter_strength=self.get_parameter('crosstalk.filter_strength').value,
            dc_preserve_ratio=self.get_parameter('crosstalk.dc_preserve_ratio').value,
        )
        self.publish_crosstalk_filtered = self.get_parameter('crosstalk.publish_filtered').value

        # Store out-of-core mode flag
        self.use_outofcore = config['use_outofcore']

        # Initialize mapper
        self.mapper = SonarTo3DMapper(config)

        # Log depth estimation status
        if config.get('depth_estimation_enabled', False):
            ref_path = config.get('depth_estimation_reference_map_path', '')
            if self.mapper.reference_map is not None:
                self.get_logger().info(f'[DepthEstimation] ENABLED (ref={ref_path})')
            else:
                self.get_logger().warn(f'[DepthEstimation] ENABLED but reference map NOT loaded (ref={ref_path})')

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

        # === SLAM quality gate state ===
        # _wall_* fields use time.monotonic() intentionally (NTP-immune
        # wall clock). They measure elapsed time for grace-period and
        # stale-confidence checks — not bag-clock-correlated; never replace
        # with self.get_clock().now().
        self._latest_confidence = None  # float | None
        self._latest_confidence_wall_time = None  # float | None — wall clock
        self._quality_drop_count = 0
        self._quality_stale_warn_count = 0
        self._quality_closed_no_confidence_warned = False
        self._quality_closed_stale_warned = False
        self._node_start_wall_time = time.monotonic()  # wall clock
        self._quality_threshold = float(self.get_parameter('slam_quality.threshold').value)
        self._quality_fail_mode = str(self.get_parameter('slam_quality.fail_mode').value)
        self._quality_grace_period_sec = float(self.get_parameter('slam_quality.grace_period_sec').value)
        self._quality_stale_timeout_sec = float(self.get_parameter('slam_quality.stale_timeout_sec').value)
        self._slam_quality_enabled = bool(slam_confidence_topic)

        # Frame counter
        self.frame_count = 0
        self.frame_skip = config['frame_skip']
        self._max_stamp_diff_sec = float(self.get_parameter('time_sync.max_stamp_diff_sec').value)
        self._max_odom_age_sec = float(self.get_parameter('time_sync.max_odom_age_sec').value)
        self._sync_policy = str(self.get_parameter('time_sync.policy').value)
        self._compensate_rotation = bool(self.get_parameter('time_sync.compensate_rotation').value)

        self._odom_buffer = OdomBuffer(max_size=50)
        self._timesync_diag = TimesyncDiagnostics()
        self._timesync_diag.policy = self._sync_policy
        self._diag_pub = self.create_publisher(
            DiagnosticArray, '/perception/sonar_3d/diagnostics', SENSOR_QOS)
        self._diag_timer = self.create_timer(1.0, self._publish_diagnostics)

        # Register parameter change callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Get QoS reliability setting (reliable or best_effort)
        qos_reliability_str = self.get_parameter('qos.reliability').value
        if qos_reliability_str == 'best_effort':
            reliability = ReliabilityPolicy.BEST_EFFORT
            self.get_logger().info('QoS reliability: BEST_EFFORT')
        else:
            reliability = ReliabilityPolicy.RELIABLE
            self.get_logger().info('QoS reliability: RELIABLE')

        # QoS profile for subscription (default: RELIABLE)
        qos_profile = QoSProfile(
            reliability=reliability,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Latest-odometry synchronization (robust against clock skew between machines)
        # Odometry arrives at ~200Hz, so the latest message is always <5ms old.
        # When a sonar frame arrives (~14Hz), we pair it with the most recent odometry.
        self._latest_odom_msg = None
        self._odom_lock = threading.Lock()

        # Callback groups (P1-1, B-3b):
        # - odom: ReentrantCallbackGroup so the 200Hz odom topic never
        #   waits for the slower sonar pipeline; odom callbacks only
        #   touch _latest_odom_msg behind _odom_lock.
        # - sonar: MutuallyExclusiveCallbackGroup so range/confidence/
        #   sonar/timer callbacks that all reach into `self.mapper`
        #   never run concurrently and corrupt internal state.
        self.odom_cbg = ReentrantCallbackGroup()
        self.sonar_cbg = MutuallyExclusiveCallbackGroup()

        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self._odom_callback,
            RELIABLE_QOS,
            callback_group=self.odom_cbg,
        )

        self.sonar_sub = self.create_subscription(
            Image,
            sonar_topic,
            self._sonar_callback,
            SENSOR_QOS,
            callback_group=self.sonar_cbg,
        )

        # Create publishers (hardcoded per spec §2.3.3 rule 1)
        self.pc_pub = self.create_publisher(
            PointCloud2,
            '/perception/sonar_3d/points',
            SENSOR_QOS
        )

        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/sonar_3d/markers',
            SENSOR_QOS
        )

        # Crosstalk filtered image publisher (for debugging)
        self.filtered_image_pub = self.create_publisher(
            Image,
            '/sonar_3d_mapper/debug/crosstalk_filtered',
            SENSOR_QOS
        )

        # Subscribe to dynamic range topic from sonar driver
        self.range_sub = self.create_subscription(
            Float32,
            range_topic,
            self.range_callback,
            LATCHED_QOS,
            callback_group=self.sonar_cbg,
        )

        # SLAM quality confidence subscription (only if topic configured)
        if self._slam_quality_enabled:
            self.slam_confidence_sub = self.create_subscription(
                Float32,
                slam_confidence_topic,
                self._confidence_callback,
                RELIABLE_QOS,
                callback_group=self.sonar_cbg,
            )
            self.get_logger().info(
                f'[SlamQuality] Gate enabled: threshold={self._quality_threshold:.2f}, '
                f'fail_mode={self._quality_fail_mode}, topic={slam_confidence_topic}'
            )
        else:
            self.get_logger().info('[SlamQuality] Gate disabled (no confidence topic)')

        # Create timer for periodic publishing
        if not self.use_outofcore:
            # In-memory mode: configurable pointcloud publishing rate
            publish_interval = 1.0 / self.pointcloud_publish_rate
            self.timer = self.create_timer(
                publish_interval, self.publish_pointcloud,
                callback_group=self.sonar_cbg,
            )
        else:
            # Out-of-core mode: eviction-based + periodic saving
            self.timer = None
            self.tile_update_pub = self.create_publisher(
                Int32MultiArray,
                '/perception/sonar_3d/tile_indices',
                LATCHED_QOS
            )
            # Periodically save dirty tiles + notify visualizer
            self.flush_timer = self.create_timer(
                self.tile_save_interval, self.periodic_flush_and_notify,
                callback_group=self.sonar_cbg,
            )
        
        # No need for TF timer anymore since we use static transform

        # Map save service (spec §2.9)
        self._save_map_srv = self.create_service(
            Trigger, '~/save_map', self._save_map_callback)

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

            # === Crosstalk filter parameters ===
            elif name == 'crosstalk.enabled':
                self.crosstalk_filter.update_crosstalk_enabled(value)
            elif name == 'crosstalk.filter_width':
                self.crosstalk_filter.update_crosstalk_filter_width(value)
            elif name == 'crosstalk.filter_strength':
                self.crosstalk_filter.update_crosstalk_filter_strength(value)
            elif name == 'crosstalk.dc_preserve_ratio':
                self.crosstalk_filter.update_crosstalk_dc_preserve_ratio(value)
            elif name == 'crosstalk.gaussian_sigma':
                self.crosstalk_filter.update_crosstalk_gaussian_sigma(value)
            elif name == 'crosstalk.publish_filtered':
                self.publish_crosstalk_filtered = bool(value)

            # === SLAM Quality Gate parameters ===
            elif name == 'slam_quality.threshold':
                try:
                    self._quality_threshold = float(value)
                    self.get_logger().info(
                        f'[SlamQuality] threshold updated: {self._quality_threshold:.3f}'
                    )
                except (TypeError, ValueError):
                    return SetParametersResult(
                        successful=False,
                        reason=f'invalid threshold value: {value!r}'
                    )
            elif name == 'slam_quality.fail_mode':
                if value in ('open', 'closed'):
                    self._quality_fail_mode = value
                    self.get_logger().info(f'[SlamQuality] fail_mode updated: {value}')
                else:
                    return SetParametersResult(
                        successful=False,
                        reason=f"invalid fail_mode {value!r}; must be 'open' or 'closed'"
                    )

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
                    self.timer = self.create_timer(
                        publish_interval, self.publish_pointcloud,
                        callback_group=self.sonar_cbg,
                    )
                    self.get_logger().info(f'Publish rate changed to {new_rate}Hz')
            elif name == 'visualization.tile_save_interval':
                new_interval = float(value)
                if new_interval != self.tile_save_interval and self.use_outofcore:
                    self.tile_save_interval = new_interval
                    # Recreate flush timer with new interval
                    if hasattr(self, 'flush_timer') and self.flush_timer is not None:
                        self.flush_timer.cancel()
                    self.flush_timer = self.create_timer(
                        self.tile_save_interval, self.periodic_flush_and_notify,
                        callback_group=self.sonar_cbg,
                    )
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

    def _viz_thread_loop(self):
        """Run OpenCV visualization in a separate thread to avoid blocking callbacks."""
        while rclpy.ok():
            img = getattr(self, '_viz_latest_image', None)
            if img is not None:
                self._visualize_sonar_frame(img)
            else:
                cv2.waitKey(30)
            time.sleep(0.03)  # ~30fps max

    def _setup_opencv_trackbars(self):
        """Create OpenCV trackbars for real-time parameter tuning."""
        win = "Sonar: Original | Threshold Applied"
        cv2.namedWindow(win, cv2.WINDOW_AUTOSIZE)

        # depth_diff_threshold: 0~50 (x0.1 = 0.0~5.0m)
        init_thresh = int(self.mapper.depth_estimation_threshold * 10)
        cv2.createTrackbar("DepthDiffThr(x0.1m)", win, init_thresh, 50, self._on_depth_diff_threshold)

        # depth_estimation enabled: 0 or 1
        init_enabled = 1 if self.mapper.depth_estimation_enabled else 0
        cv2.createTrackbar("DepthEstimation", win, init_enabled, 1, self._on_depth_estimation_toggle)

        # marker_min_depth: 0~200 (x0.1 = 0.0~20.0m)
        init_min = int(self.marker_min_depth * 10)
        cv2.createTrackbar("MarkerMinDepth(x0.1m)", win, init_min, 200, self._on_marker_min_depth)

        # marker_max_depth: 0~200 (x0.1 = 0.0~20.0m)
        init_max = int(self.marker_max_depth * 10)
        cv2.createTrackbar("MarkerMaxDepth(x0.1m)", win, init_max, 200, self._on_marker_max_depth)


        self._trackbars_created = True

    def _on_depth_diff_threshold(self, val):
        self.mapper.depth_estimation_threshold = val * 0.1

    def _on_depth_estimation_toggle(self, val):
        self.mapper.depth_estimation_enabled = bool(val)

    def _on_marker_min_depth(self, val):
        self.marker_min_depth = val * 0.1

    def _on_marker_max_depth(self, val):
        self.marker_max_depth = val * 0.1

    def _on_marker_alpha(self, val):
        self.marker_alpha = val * 0.1

    def _visualize_sonar_frame(self, sonar_image: np.ndarray):
        """
        Visualize sonar frame with threshold overlay

        Args:
            sonar_image: Grayscale sonar image
        """
        if not hasattr(self, '_trackbars_created'):
            self._setup_opencv_trackbars()

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

        # Show current depth estimation status
        de_status = "ON" if self.mapper.depth_estimation_enabled else "OFF"
        de_thresh = self.mapper.depth_estimation_threshold
        cv2.putText(overlay, f"DepthEst: {de_status} (thr={de_thresh:.1f}m)", (10, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2)

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

    def _odom_callback(self, msg: Odometry):
        """Store the latest odometry message (thread-safe)."""
        self._odom_buffer.push(msg)
        with self._odom_lock:
            self._latest_odom_msg = msg

    def _confidence_callback(self, msg: Float32):
        """Store the latest SLAM confidence value with arrival wall time."""
        self._latest_confidence = float(msg.data)
        self._latest_confidence_wall_time = time.monotonic()

    def _quality_gate_passes(self) -> bool:
        """
        Decide whether the current sonar frame should be processed based on the
        latest SLAM confidence value and freshness.

        Fresh = non-None and age <= stale_timeout_sec; passes iff confidence >= threshold.
        Stale or missing apply fail_mode ('open' = pass with throttled WARN, 'closed' = drop).
        Uses time.monotonic() for elapsed-time comparisons (immune to NTP corrections).
        """
        wall_now = time.monotonic()

        # Case 1: never received any confidence yet
        if self._latest_confidence is None:
            elapsed = wall_now - self._node_start_wall_time
            if elapsed < self._quality_grace_period_sec:
                return True  # warmup grace
            if self._quality_fail_mode == 'closed':
                if not self._quality_closed_no_confidence_warned:
                    self._quality_closed_no_confidence_warned = True
                    self.get_logger().warn(
                        f'[SlamQuality] Entering closed-mode silent drop '
                        f'(no confidence received in {elapsed:.1f}s). '
                        f'Check that the SLAM confidence publisher is running.'
                    )
                return False
            self._quality_stale_warn_count += 1
            if self._quality_stale_warn_count % 50 == 1:
                self.get_logger().warn(
                    f'[SlamQuality] No confidence received '
                    f'(elapsed={elapsed:.1f}s) - passing frame (fail_mode=open)'
                )
            return True

        # Case 2: confidence received but stale
        age = wall_now - self._latest_confidence_wall_time
        if age > self._quality_stale_timeout_sec:
            if self._quality_fail_mode == 'closed':
                if not self._quality_closed_stale_warned:
                    self._quality_closed_stale_warned = True
                    self.get_logger().warn(
                        f'[SlamQuality] Entering closed-mode stale drop '
                        f'(confidence age={age:.1f}s > {self._quality_stale_timeout_sec:.1f}s). '
                        f'Check SLAM publisher liveness.'
                    )
                return False
            self._quality_stale_warn_count += 1
            if self._quality_stale_warn_count % 50 == 1:
                self.get_logger().warn(
                    f'[SlamQuality] Stale confidence (age={age:.1f}s '
                    f'> {self._quality_stale_timeout_sec:.1f}s) - passing frame (fail_mode=open)'
                )
            return True

        # Case 3: fresh confidence — apply threshold
        if self._latest_confidence < self._quality_threshold:
            self._quality_drop_count += 1
            if self._quality_drop_count % 10 == 1:
                self.get_logger().warn(
                    f'[SlamQuality] Drop: confidence={self._latest_confidence:.3f} '
                    f'< {self._quality_threshold:.3f} '
                    f'(dropped {self._quality_drop_count} total)'
                )
            return False

        return True

    def _sonar_callback(self, sonar_msg: Image):
        """
        Pair each sonar frame with odometry using the configured policy.
        Policy options: 'latest', 'interpolate', 'nearest'.
        """
        sonar_t = sonar_msg.header.stamp.sec + sonar_msg.header.stamp.nanosec * 1e-9

        # Policy-based odom selection
        if self._sync_policy == 'interpolate':
            odom_msg = self._odom_buffer.interpolate(sonar_t)
            if odom_msg is None:
                odom_msg = self._odom_buffer.nearest(sonar_t)
        elif self._sync_policy == 'nearest':
            odom_msg = self._odom_buffer.nearest(sonar_t)
        else:  # 'latest'
            odom_msg = self._odom_buffer.latest()

        if odom_msg is None:
            if not hasattr(self, '_odom_warn_logged'):
                self.get_logger().warn('Waiting for first odometry message...')
                self._odom_warn_logged = True
            return

        odom_t = odom_msg.header.stamp.sec + odom_msg.header.stamp.nanosec * 1e-9
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        stamp_diff = abs(sonar_t - odom_t)

        # Log time info periodically for debugging.
        # ros_now uses ROS clock — when use_sim_time:=true (bag replay), this
        # tracks bag time, so age values stay meaningful.
        if not hasattr(self, '_sync_log_count'):
            self._sync_log_count = 0
        self._sync_log_count += 1
        if self._sync_log_count % 50 == 1:
            self.get_logger().info(
                f'[TimeSync] sonar_stamp={sonar_t:.3f} odom_stamp={odom_t:.3f} '
                f'stamp_diff={sonar_t - odom_t:.3f}s '
                f'sonar_age={ros_now - sonar_t:.3f}s odom_age={ros_now - odom_t:.3f}s'
            )

        # Reject frames where sonar-odom time difference exceeds threshold
        if stamp_diff > self._max_stamp_diff_sec:
            self._timesync_diag.dropped_stamp_diff += 1
            if not hasattr(self, '_sync_drop_count'):
                self._sync_drop_count = 0
            self._sync_drop_count += 1
            if self._sync_drop_count % 10 == 1:
                self.get_logger().warn(
                    f'[TimeSync] Dropping frame: stamp_diff={stamp_diff:.3f}s > '
                    f'{self._max_stamp_diff_sec}s (dropped {self._sync_drop_count} frames total)'
                )
            return

        # Odom freshness check: positive when odom is older than sonar
        odom_age = sonar_t - odom_t
        if odom_age > self._max_odom_age_sec:
            self._timesync_diag.dropped_stale_odom += 1
            if self._timesync_diag.dropped_stale_odom % 10 == 1:
                self.get_logger().warn(
                    f'[TimeSync] odom_age={odom_age:.3f}s > {self._max_odom_age_sec}s '
                    f'(stale odom), dropping '
                    f'(total {self._timesync_diag.dropped_stale_odom})'
                )
            return

        # Update diagnostics rolling means
        self._timesync_diag.stamp_diff_mean.add(stamp_diff)
        self._timesync_diag.odom_age_mean.add(odom_age)

        # SLAM quality gate (skip frame if confidence too low / stale per fail_mode)
        if self._slam_quality_enabled:
            if not self._quality_gate_passes():
                self._timesync_diag.dropped_quality_gate += 1
                return

        # Optional rotation compensation
        if self._compensate_rotation:
            odom_msg = self._apply_rotation_compensation(odom_msg, dt=odom_age)

        self._timesync_diag.paired_count += 1
        self.synchronized_callback(sonar_msg, odom_msg)

    def _apply_rotation_compensation(self, odom_msg: Odometry, dt: float) -> Odometry:
        """Apply angular_vel × dt to odom orientation. Small-angle approx."""
        import math
        out = deepcopy(odom_msg)
        w = odom_msg.twist.twist.angular
        half_dt = dt * 0.5
        qd_x = w.x * half_dt
        qd_y = w.y * half_dt
        qd_z = w.z * half_dt
        qd_w = 1.0
        norm = math.sqrt(qd_x**2 + qd_y**2 + qd_z**2 + qd_w**2)
        qd_x, qd_y, qd_z, qd_w = qd_x/norm, qd_y/norm, qd_z/norm, qd_w/norm
        q = odom_msg.pose.pose.orientation
        out.pose.pose.orientation.w = q.w*qd_w - q.x*qd_x - q.y*qd_y - q.z*qd_z
        out.pose.pose.orientation.x = q.w*qd_x + q.x*qd_w + q.y*qd_z - q.z*qd_y
        out.pose.pose.orientation.y = q.w*qd_y - q.x*qd_z + q.y*qd_w + q.z*qd_x
        out.pose.pose.orientation.z = q.w*qd_z + q.x*qd_y - q.y*qd_x + q.z*qd_w
        return out

    def _publish_diagnostics(self):
        """Publish timesync DiagnosticArray at 1Hz."""
        msg = self._timesync_diag.to_msg(self.get_clock().now())
        self._diag_pub.publish(msg)

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

        # Apply crosstalk noise filter
        sonar_image = self.crosstalk_filter.apply(sonar_image)

        # Publish filtered image for debugging
        if self.publish_crosstalk_filtered and self.crosstalk_filter.enabled:
            filtered_msg = self.bridge.cv2_to_imgmsg(sonar_image, encoding='mono8')
            filtered_msg.header = sonar_msg.header
            self.filtered_image_pub.publish(filtered_msg)

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

        # Publish current frame's occupied voxels as MarkerArray (auto-expire)
        occupied_points = stats.get('occupied_points', None)
        if occupied_points is not None and len(occupied_points) > 0:
            if len(occupied_points) > 0:
                self.publish_detection_markers(occupied_points)

        # Show visualization if enabled (non-blocking, separate thread)
        if self.show_opencv_visualization:
            self._viz_latest_image = sonar_image.copy()
            if not hasattr(self, '_viz_thread_started'):
                self._viz_thread_started = True
                self._viz_latest_image = sonar_image.copy()
                t = threading.Thread(target=self._viz_thread_loop, daemon=True)
                t.start()

        # Store latest odometry for TF publishing
        self.latest_odometry = odom_msg

        # Log depth estimation results periodically
        depth_est = stats.get('depth_estimation')
        if depth_est and self.frame_count % 20 == 0:
            samples_str = '  '.join(
                f'[{s["bearing_deg"]:+.0f}° act={s["actual"]:.1f} exp={s["expected"]:.1f} {"NEW" if s["is_new"] else "skip"}]'
                for s in depth_est.get('samples', [])
            )
            self.get_logger().info(
                f'[DepthEst] matched={depth_est["num_matched"]} '
                f'new={depth_est["num_new"]} no_ref={depth_est["num_no_ref"]} | '
                f'{samples_str}'
            )

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
        
        packed = np.empty(len(points), dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
        ])
        packed['x'] = points[:, 0]
        packed['y'] = points[:, 1]
        packed['z'] = points[:, 2]
        packed['intensity'] = probabilities
        cloud.data = packed.tobytes()
        
        # Publish
        self.pc_pub.publish(cloud)

    def _save_map_callback(self, request, response):
        """Trigger service: save current map to .bt file (outofcore mode).

        In-memory mode is not supported by the existing mapper API and
        returns success=False with a guidance message.
        """
        try:
            user_path = self.get_parameter('output.map_dir').value if \
                self.has_parameter('output.map_dir') else ''
            if not self.use_outofcore:
                response.success = False
                response.message = (
                    'In-memory mode: save_map service not yet supported. '
                    'Run with map_path:=<dir> for outofcore mode.'
                )
                return response

            save_path = resolve_map_save_path(user_path, 'sonar_3d', 'map.bt')
            # Flush dirty tiles to disk first
            self.mapper.flush_map()
            # Then merge all tiles into a single .bt file
            ok = self.mapper.save_merged_octree(str(save_path))
            if ok:
                update_latest_symlink(save_path, 'sonar_3d')
                response.success = True
                response.message = f'Saved: {save_path}'
            else:
                response.success = False
                response.message = f'save_merged_octree returned False (path: {save_path})'
        except Exception as e:
            response.success = False
            response.message = f'Save failed: {e}'
        return response

    def publish_detection_markers(self, points: np.ndarray):
        """
        Publish current frame's occupied voxels as MarkerArray with Z-axis coloring.
        Each frame gets a unique marker ID so markers expire independently after 5s.
        """
        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.map_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'detection'
        marker.id = self.frame_count  # Unique ID per frame → independent lifetime
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=5)
        marker.scale.x = self.mapper.voxel_resolution
        marker.scale.y = self.mapper.voxel_resolution
        marker.scale.z = self.mapper.voxel_resolution

        # Depth-based grayscale (min_depth=black, max_depth=white)
        min_depth = self.marker_min_depth
        max_depth = self.marker_max_depth
        depth_range = max_depth - min_depth if max_depth > min_depth else 1.0

        for i in range(len(points)):
            p = Point()
            p.x = float(points[i, 0])
            p.y = float(points[i, 1])
            p.z = float(points[i, 2])
            marker.points.append(p)

            depth = abs(float(points[i, 2]))
            gray = max(0.0, min((depth - min_depth) / depth_range, 1.0))
            c = ColorRGBA()
            c.r = gray
            c.g = gray
            c.b = gray
            c.a = self.marker_alpha
            marker.colors.append(c)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = SonarMapperNode()

    # Drive the node with a MultiThreadedExecutor (P1-1, B-3b). odom
    # callbacks run on a Reentrant group so the 200Hz odom topic does
    # not back up behind sonar processing; the sonar/range/timer
    # callbacks share a MutuallyExclusive group so per-frame mapper
    # state stays consistent.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
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