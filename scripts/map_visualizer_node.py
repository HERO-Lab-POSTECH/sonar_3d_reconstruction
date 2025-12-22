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
import numpy as np
import os
import struct
import time

# ROS2 message imports
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Int32MultiArray
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor

# OctoMap message import
try:
    from octomap_msgs.msg import Octomap
    OCTOMAP_MSGS_AVAILABLE = True
except ImportError:
    OCTOMAP_MSGS_AVAILABLE = False

# Import C++ module
try:
    import sys
    import importlib.util

    install_path = "/workspace/ros2_ws/install/sonar_3d_reconstruction/local/lib/python3.10/dist-packages"
    cpp_file = f"{install_path}/sonar_3d_reconstruction/sonar_3d_reconstruction_cpp.cpython-310-x86_64-linux-gnu.so"

    spec = importlib.util.spec_from_file_location("sonar_3d_reconstruction_cpp", cpp_file)
    cpp_module = importlib.util.module_from_spec(spec)
    sys.modules["sonar_3d_reconstruction_cpp"] = cpp_module
    spec.loader.exec_module(cpp_module)

    OutofcoreTileMapper = cpp_module.OutofcoreTileMapper
    TileIndex = cpp_module.TileIndex
    CPP_MODULE_AVAILABLE = True
except Exception:
    CPP_MODULE_AVAILABLE = False


class MapVisualizerNode(Node):
    """ROS2 Node for map visualization from tile files"""

    def __init__(self):
        super().__init__('map_visualizer')

        # Read-only descriptor
        read_only = ParameterDescriptor(read_only=True)

        # Declare parameters individually (avoids type inference issues)
        # String parameters
        self.declare_parameter('outofcore.map_path', '/workspace/data/map_tiles', read_only)
        self.declare_parameter('frames.map', 'camera_init', read_only)
        self.declare_parameter('visualization.mode', 'octomap')

        # Float parameters
        self.declare_parameter('octree.voxel_resolution', 0.1, read_only)
        self.declare_parameter('outofcore.tile_size', 10.0, read_only)
        self.declare_parameter('publish_rate', 1.0, read_only)
        self.declare_parameter('refresh_interval', 10.0, read_only)
        self.declare_parameter('mapping.occupied_threshold', 0.7)

        # Bool parameters
        self.declare_parameter('auto_refresh', True, read_only)

        # Get parameters
        self.map_path = self.get_parameter('outofcore.map_path').value
        self.resolution = self.get_parameter('octree.voxel_resolution').value
        self.tile_size = self.get_parameter('outofcore.tile_size').value
        self.frame_id = self.get_parameter('frames.map').value
        self.occupied_threshold = self.get_parameter('mapping.occupied_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.visualization_mode = self.get_parameter('visualization.mode').value
        self.auto_refresh = self.get_parameter('auto_refresh').value
        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.last_refresh_time = 0.0

        # Register parameter callback for dynamic updates
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Fallback to pointcloud if octomap_msgs not available
        if self.visualization_mode == 'octomap' and not OCTOMAP_MSGS_AVAILABLE:
            self.get_logger().warn('octomap_msgs not available, using pointcloud mode')
            self.visualization_mode = 'pointcloud'

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

        # Publishers
        if OCTOMAP_MSGS_AVAILABLE:
            self.octomap_pub = self.create_publisher(Octomap, '/map_octomap', 10)
        else:
            self.octomap_pub = None

        self.pc_pub = self.create_publisher(PointCloud2, '/map_pointcloud', 10)

        # Subscribe to tile update notifications from mapper node
        self.tile_update_sub = self.create_subscription(
            Int32MultiArray,
            '/updated_tile_indices',
            self.tile_update_callback,
            10
        )
        self.pending_tile_updates = []  # Pending tile updates

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Single-line initialization summary
        self.get_logger().info(
            f'Visualizer: {self.visualization_mode} mode, {self.publish_rate}Hz'
        )

    def parameter_callback(self, params):
        """Handle dynamic parameter updates"""
        for param in params:
            if param.name == 'mapping.occupied_threshold':
                self.occupied_threshold = float(param.value)
                self.get_logger().info(f'occupied_threshold: {param.value}')
            elif param.name == 'visualization.mode':
                new_mode = str(param.value)
                if new_mode in ('octomap', 'pointcloud', 'all'):
                    if new_mode == 'octomap' and not OCTOMAP_MSGS_AVAILABLE:
                        self.get_logger().warn('octomap_msgs not available')
                    else:
                        self.visualization_mode = new_mode
                        self.get_logger().info(f'visualization.mode: {new_mode}')
        return SetParametersResult(successful=True)

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

        # Selective reload: reload only tiles updated from mapper (priority)
        if len(self.pending_tile_updates) > 0:
            self.reload_specific_tiles(self.pending_tile_updates)
            self.pending_tile_updates = []
            self.last_refresh_time = time.time()  # Reset refresh timer

        # Fallback: full reload if auto_refresh enabled and no topic updates
        current_time = time.time()
        if self.auto_refresh and (current_time - self.last_refresh_time) >= self.refresh_interval:
            self.reload_tiles()
            self.last_refresh_time = current_time

        try:
            stamp = self.get_clock().now().to_msg()

            if self.visualization_mode == 'all':
                # Publish both
                if self.octomap_pub is not None:
                    self.publish_octomap(stamp)
                self.publish_pointcloud(stamp)
            elif self.visualization_mode == 'octomap' and self.octomap_pub is not None:
                self.publish_octomap(stamp)
            else:
                self.publish_pointcloud(stamp)

        except Exception as e:
            self.get_logger().error(f'Visualization error: {e}')

    def publish_octomap(self, stamp):
        """Publish OctoMap message (for RViz OctoMap plugin)"""
        try:
            # Get binary octree data from C++ module
            data, tree_id = self.mapper.get_octree_binary()

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

        # Pack data
        data = []
        for i in range(len(voxels)):
            data.append(struct.pack('ffff',
                                   voxels[i, 0], voxels[i, 1], voxels[i, 2],
                                   voxels[i, 3]))

        cloud.data = b''.join(data)
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
