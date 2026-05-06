#!/usr/bin/env python3
"""
ROS2 Node for Octree Map Visualization
Reads tile-based map from disk and publishes as OctoMap/PointCloud2 for RViz

Supports:
- OctoMap message (octomap_msgs/Octomap) for RViz OctoMap plugin
- PointCloud2 for point-based visualization
- Periodic refresh from disk

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sonar_3d_reconstruction.qos import SENSOR_QOS, LATCHED_QOS
import numpy as np
import os
import time

# ROS2 message imports
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int32MultiArray, ColorRGBA
from visualization_msgs.msg import MarkerArray, Marker
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point

# OctoMap message import
try:
    from octomap_msgs.msg import Octomap
    OCTOMAP_MSGS_AVAILABLE = True
except ImportError:
    OCTOMAP_MSGS_AVAILABLE = False

# Import parameter management
from config import ParameterManager, VISUALIZER_PARAMS

# Import C++ module from package
from sonar_3d_reconstruction import (
    CPP_MODULE_AVAILABLE,
    OutofcoreTileMapper,
    TileIndex,
)


class MapVisualizerNode(Node):
    """ROS2 Node for map visualization from tile files"""

    # Visualization mode constants
    VIS_MODE_POINTCLOUD = 0  # Default - supports occupied_threshold
    VIS_MODE_OCTOMAP = 1
    VIS_MODE_ALL = 2
    VIS_MODE_NAMES = ['pointcloud', 'octomap', 'all']

    def __init__(self):
        super().__init__('map_visualizer')

        # Declare all parameters using ParameterManager
        ParameterManager.declare_all(self, VISUALIZER_PARAMS)

        # Get all parameter values
        params = ParameterManager.get_all(self, VISUALIZER_PARAMS)

        # Extract parameters
        self.map_path = params['outofcore.map_path']
        self.frame_id = params['frames.map']
        self.occupied_threshold = params['mapping.occupied_threshold']
        self.publish_rate = params['publish_rate']
        self.visualization_mode = params['visualization.mode']
        self.auto_refresh = params['auto_refresh']
        self.refresh_interval = params['refresh_interval']
        self.last_refresh_time = 0.0
        self.marker_lifetime = params['marker_lifetime']

        # Read resolution and tile_size from metadata.json (fallback to params if not found)
        self.resolution, self.tile_size = self._load_metadata(
            fallback_resolution=params['octree.voxel_resolution'],
            fallback_tile_size=params['outofcore.tile_size']
        )

        # Register parameter callback for dynamic updates
        self.add_on_set_parameters_callback(
            ParameterManager.create_callback(self, VISUALIZER_PARAMS, self)
        )

        # Fallback to pointcloud if octomap_msgs not available
        if self.visualization_mode == self.VIS_MODE_OCTOMAP and not OCTOMAP_MSGS_AVAILABLE:
            self.get_logger().warn('octomap_msgs not available, using pointcloud mode')
            self.visualization_mode = self.VIS_MODE_POINTCLOUD

        # Initialize mapper (read-only mode)
        self.mapper = None

        if CPP_MODULE_AVAILABLE and os.path.exists(self.map_path):
            try:
                self.mapper = OutofcoreTileMapper(
                    self.map_path,
                    self.resolution,
                    self.tile_size,
                    4  # Minimal cache - visualizer loads all tiles via get_all_occupied_voxels()
                )

                # Preload all tiles (required for visualization)
                tile_indices = self.mapper.get_all_tile_indices()
                if len(tile_indices) > 0:
                    min_x = min(t.x for t in tile_indices) * self.tile_size - self.tile_size
                    min_y = min(t.y for t in tile_indices) * self.tile_size - self.tile_size
                    min_z = min(t.z for t in tile_indices) * self.tile_size - self.tile_size
                    max_x = (max(t.x for t in tile_indices) + 1) * self.tile_size + self.tile_size
                    max_y = (max(t.y for t in tile_indices) + 1) * self.tile_size + self.tile_size
                    max_z = (max(t.z for t in tile_indices) + 1) * self.tile_size + self.tile_size

                    min_bound = np.array([min_x, min_y, min_z])
                    max_bound = np.array([max_x, max_y, max_z])
                    self.mapper.preload_region(min_bound, max_bound)
            except Exception:
                pass

        # Publishers (hardcoded per spec §2.3.3 rule 1)
        if OCTOMAP_MSGS_AVAILABLE:
            self.octomap_pub = self.create_publisher(Octomap, '/perception/sonar_3d_visualizer/octomap', SENSOR_QOS)
        else:
            self.octomap_pub = None

        self.pc_pub = self.create_publisher(PointCloud2, '/perception/sonar_3d_visualizer/points', SENSOR_QOS)

        # MarkerArray publisher for auto-expire mode
        if self.marker_lifetime > 0:
            self.marker_pub = self.create_publisher(MarkerArray, '/perception/sonar_3d_visualizer/markers', SENSOR_QOS)
            self.last_tile_update_time = 0.0

        # Subscribe to tile update notifications from mapper node
        self.tile_update_sub = self.create_subscription(
            Int32MultiArray,
            '/perception/sonar_3d/tile_indices',
            self.tile_update_callback,
            LATCHED_QOS
        )
        self.pending_tile_updates = []  # Pending tile updates
        # Cached stamp from latest tile_update_callback. publish_callback uses
        # this so all messages emitted while processing one callback share an
        # identical stamp (visual consistency across octomap/pointcloud/markers).
        # None until first callback arrives → publish_callback falls back to fresh.
        self._last_callback_stamp = None

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Single-line initialization summary
        if self.resolution is not None:
            self.get_logger().info(
                f'Visualizer: {self.VIS_MODE_NAMES[self.visualization_mode]} mode, {self.publish_rate}Hz, '
                f'res={self.resolution}m, tile={self.tile_size}m'
            )
        else:
            self.get_logger().warn(f'Waiting for map at {self.map_path}')

    def _load_metadata(self, fallback_resolution=0.1, fallback_tile_size=10.0):
        """Load resolution and tile_size from metadata.json, fallback to params if not found"""
        import json
        metadata_path = os.path.join(self.map_path, 'metadata.json')

        if not os.path.exists(metadata_path):
            self.get_logger().info(f'metadata.json not found, using params: res={fallback_resolution}, tile={fallback_tile_size}')
            return fallback_resolution, fallback_tile_size

        try:
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
            resolution = metadata.get('resolution', fallback_resolution)
            tile_size = metadata.get('tile_size', fallback_tile_size)
            return resolution, tile_size
        except Exception as e:
            self.get_logger().error(f'Failed to load metadata: {e}')
            return fallback_resolution, fallback_tile_size

    # Parameter update handlers (called by ParameterManager)
    def update_occupied_threshold(self, value):
        """Update occupied threshold dynamically"""
        self.occupied_threshold = float(value)
        self.get_logger().info(f'occupied_threshold: {value}')

    def update_visualization_mode(self, value):
        """Update visualization mode dynamically"""
        new_mode = int(value)
        if 0 <= new_mode <= 2:
            if new_mode == self.VIS_MODE_OCTOMAP and not OCTOMAP_MSGS_AVAILABLE:
                self.get_logger().warn('octomap_msgs not available')
            else:
                self.visualization_mode = new_mode
                self.get_logger().info(f'visualization.mode: {self.VIS_MODE_NAMES[new_mode]}')

    def tile_update_callback(self, msg: Int32MultiArray):
        """Receive updated tile indices from mapper node"""
        if len(msg.data) == 0 or len(msg.data) % 3 != 0:
            return

        # Parse [x1, y1, z1, x2, y2, z2, ...] format
        tile_indices = []
        for i in range(0, len(msg.data), 3):
            tile_indices.append(TileIndex(msg.data[i], msg.data[i+1], msg.data[i+2]))

        # Add to pending list (processed in publish_callback)
        self.pending_tile_updates.extend(tile_indices)
        # Cache stamp at callback entry: messages republished as a result of
        # this callback all share this stamp (visual consistency).
        self._last_callback_stamp = self.get_clock().now().to_msg()
        if self.marker_lifetime > 0:
            self.last_tile_update_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().debug(f'Received {len(tile_indices)} tile update(s)')

    def reload_specific_tiles(self, tile_indices):
        """Reload only specific tiles from disk (selective reload)"""
        if self.mapper is None or len(tile_indices) == 0:
            return

        try:
            self.mapper.reload_tiles(tile_indices)
        except Exception:
            pass

    def reload_tiles(self):
        """Reload tiles from disk (for auto_refresh)"""
        if self.mapper is None:
            return

        try:
            # Create new mapper instance to load latest tiles
            self.mapper = OutofcoreTileMapper(
                self.map_path,
                self.resolution,
                self.tile_size,
                4  # Minimal cache - visualizer loads all tiles via get_all_occupied_voxels()
            )

            # Preload all tiles
            tile_indices = self.mapper.get_all_tile_indices()
            if len(tile_indices) > 0:
                min_x = min(t.x for t in tile_indices) * self.tile_size - self.tile_size
                min_y = min(t.y for t in tile_indices) * self.tile_size - self.tile_size
                min_z = min(t.z for t in tile_indices) * self.tile_size - self.tile_size
                max_x = (max(t.x for t in tile_indices) + 1) * self.tile_size + self.tile_size
                max_y = (max(t.y for t in tile_indices) + 1) * self.tile_size + self.tile_size
                max_z = (max(t.z for t in tile_indices) + 1) * self.tile_size + self.tile_size

                min_bound = np.array([min_x, min_y, min_z])
                max_bound = np.array([max_x, max_y, max_z])
                self.mapper.preload_region(min_bound, max_bound)
        except Exception:
            pass

    def publish_callback(self):
        """Periodically publish map visualization"""
        if self.mapper is None:
            # Retry map loading (silent)
            if os.path.exists(self.map_path) and CPP_MODULE_AVAILABLE:
                try:
                    self.mapper = OutofcoreTileMapper(
                        self.map_path,
                        self.resolution,
                        self.tile_size,
                        4  # Minimal cache
                    )
                except Exception:
                    pass
            return

        # Marker lifetime mode: publish MarkerArray only when new tile updates arrive
        # Marker disappears automatically after lifetime (no re-publishing)
        if self.marker_lifetime > 0:
            if len(self.pending_tile_updates) > 0:
                self.reload_specific_tiles(self.pending_tile_updates)
                self.pending_tile_updates = []
                try:
                    # Use cached callback stamp for consistency across this
                    # tile-update batch; fall back to fresh stamp if callback
                    # not yet received (defensive — should not happen here).
                    stamp = self._last_callback_stamp or self.get_clock().now().to_msg()
                    self.publish_marker_array(stamp)
                except Exception as e:
                    self.get_logger().error(f'Marker publish error: {e}')
            return

        # Normal mode: continuous OctoMap/PointCloud2 publishing
        # Selective reload: reload only tiles updated from mapper (priority).
        # last_refresh_time uses ROS clock so refresh throttle stays correct
        # under bag replay (use_sim_time:=true).
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        callback_driven = len(self.pending_tile_updates) > 0
        if callback_driven:
            self.reload_specific_tiles(self.pending_tile_updates)
            self.pending_tile_updates = []
            self.last_refresh_time = ros_now

        # Fallback: full reload if auto_refresh enabled and no topic updates
        if self.auto_refresh and (ros_now - self.last_refresh_time) >= self.refresh_interval:
            self.reload_tiles()
            self.last_refresh_time = ros_now

        try:
            # Callback-driven publish: use cached stamp so octomap+pointcloud
            # share identical stamp. Timer-driven publish: fresh stamp.
            if callback_driven and self._last_callback_stamp is not None:
                stamp = self._last_callback_stamp
            else:
                stamp = self.get_clock().now().to_msg()

            if self.visualization_mode == self.VIS_MODE_ALL:
                # Publish both
                if self.octomap_pub is not None:
                    self.publish_octomap(stamp)
                self.publish_pointcloud(stamp)
            elif self.visualization_mode == self.VIS_MODE_OCTOMAP and self.octomap_pub is not None:
                self.publish_octomap(stamp)
            else:
                self.publish_pointcloud(stamp)

        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')

    def publish_octomap(self, stamp):
        """Publish OctoMap message (for RViz OctoMap plugin)"""
        try:
            # Get binary octree data from C++ module (with threshold filtering)
            data, tree_id = self.mapper.get_octree_binary(self.occupied_threshold)

            if len(data) == 0:
                return

            # Create OctoMap message
            msg = Octomap()
            msg.header.stamp = stamp
            msg.header.frame_id = self.frame_id
            msg.binary = True  # Binary octree (free/occupied only)
            msg.id = tree_id   # e.g., "OcTree"
            msg.resolution = self.resolution

            # Convert bytes to signed int8 array (ROS2 int8 range: -128 to 127)
            msg.data = np.frombuffer(data, dtype=np.int8).tolist()

            self.octomap_pub.publish(msg)

        except Exception:
            pass

    def publish_marker_array(self, stamp):
        """Publish MarkerArray with auto-expire lifetime (for detection visualization)"""
        voxels = self.mapper.get_all_occupied_voxels(self.occupied_threshold)

        marker_array = MarkerArray()

        if len(voxels) == 0:
            # Publish empty array to clear existing markers
            self.marker_pub.publish(marker_array)
            return

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.frame_id
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        marker.lifetime = Duration(sec=int(self.marker_lifetime))
        marker.scale.x = self.resolution
        marker.scale.y = self.resolution
        marker.scale.z = self.resolution
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8

        # Z-axis coloring for depth perception
        z_values = voxels[:, 2]
        z_min, z_max = z_values.min(), z_values.max()
        z_range = z_max - z_min if z_max > z_min else 1.0

        for i in range(len(voxels)):
            p = Point()
            p.x = float(voxels[i, 0])
            p.y = float(voxels[i, 1])
            p.z = float(voxels[i, 2])
            marker.points.append(p)

            # Z-axis rainbow coloring (low=blue → mid=green → high=red)
            z_ratio = (voxels[i, 2] - z_min) / z_range
            c = ColorRGBA()
            if z_ratio < 0.5:
                t = z_ratio * 2.0
                c.r = 0.0
                c.g = t
                c.b = 1.0 - t
            else:
                t = (z_ratio - 0.5) * 2.0
                c.r = t
                c.g = 1.0 - t
                c.b = 0.0
            c.a = 0.8
            marker.colors.append(c)

        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def publish_pointcloud(self, stamp):
        """Publish PointCloud2"""
        # get_all_occupied_voxels: loads voxels from all tiles (no cache limit)
        voxels = self.mapper.get_all_occupied_voxels(self.occupied_threshold)

        if len(voxels) == 0:
            return

        header = Header()
        header.stamp = stamp
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        cloud = PointCloud2()
        cloud.header = header
        cloud.height = 1
        cloud.width = len(voxels)
        cloud.fields = fields
        cloud.is_bigendian = False
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width
        cloud.is_dense = True

        packed = np.empty(len(voxels), dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
        ])
        packed['x'] = voxels[:, 0]
        packed['y'] = voxels[:, 1]
        packed['z'] = voxels[:, 2]
        packed['intensity'] = voxels[:, 3]
        cloud.data = packed.tobytes()
        self.pc_pub.publish(cloud)


def main(args=None):
    rclpy.init(args=args)

    node = MapVisualizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
