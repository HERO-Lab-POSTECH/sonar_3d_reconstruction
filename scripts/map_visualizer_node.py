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

# OctoMap message import
try:
    from octomap_msgs.msg import Octomap
    OCTOMAP_MSGS_AVAILABLE = True
except ImportError:
    OCTOMAP_MSGS_AVAILABLE = False

# C++ 모듈 임포트
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

        # Declare parameters (common.yaml 파라미터명 + visualizer 전용 파라미터)
        self.declare_parameters(
            namespace='',
            parameters=[
                # common.yaml 파라미터명 (우선 사용)
                ('outofcore_map_path', ''),
                ('voxel_resolution', 0.0),
                ('outofcore_tile_size', 0.0),
                ('map_frame_id', ''),
                # visualizer 전용 파라미터 (fallback)
                ('map_path', '/workspace/data/map_tiles'),
                ('resolution', 0.1),
                ('tile_size', 10.0),
                ('frame_id', 'camera_init'),
                ('occupied_threshold', 0.7),
                ('publish_rate', 1.0),
                ('visualization_mode', 'octomap'),  # 'octomap', 'pointcloud', or 'all'
                ('auto_refresh', True),
                ('refresh_interval', 10.0),
            ]
        )

        # Get parameters (common.yaml 파라미터 우선, 없으면 visualizer 파라미터 사용)
        def get_with_fallback(primary, fallback):
            val = self.get_parameter(primary).value
            if val == '' or val == 0 or val == 0.0:
                return self.get_parameter(fallback).value
            return val

        self.map_path = get_with_fallback('outofcore_map_path', 'map_path')
        self.resolution = get_with_fallback('voxel_resolution', 'resolution')
        self.tile_size = get_with_fallback('outofcore_tile_size', 'tile_size')
        self.frame_id = get_with_fallback('map_frame_id', 'frame_id')
        self.occupied_threshold = self.get_parameter('occupied_threshold').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.visualization_mode = self.get_parameter('visualization_mode').value
        self.auto_refresh = self.get_parameter('auto_refresh').value
        self.refresh_interval = self.get_parameter('refresh_interval').value
        self.last_refresh_time = 0.0

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

                # 모든 타일 preload (시각화를 위해 필수)
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
        self.pending_tile_updates = []  # 대기 중인 타일 업데이트

        # Timer for periodic publishing
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_callback)

        # Single-line initialization summary
        self.get_logger().info(
            f'Visualizer: {self.visualization_mode} mode, {self.publish_rate}Hz'
        )

    def tile_update_callback(self, msg: Int32MultiArray):
        """mapper 노드에서 업데이트된 타일 인덱스 수신"""
        if len(msg.data) == 0 or len(msg.data) % 3 != 0:
            return

        # Parse [x1, y1, z1, x2, y2, z2, ...] format
        tile_indices = []
        for i in range(0, len(msg.data), 3):
            tile_indices.append(TileIndex(msg.data[i], msg.data[i+1], msg.data[i+2]))

        # 대기 리스트에 추가 (publish_callback에서 처리)
        self.pending_tile_updates.extend(tile_indices)
        self.get_logger().debug(f'Received {len(tile_indices)} tile update(s)')

    def reload_specific_tiles(self, tile_indices):
        """특정 타일만 디스크에서 다시 로드 (선택적 리로드)"""
        if self.mapper is None or len(tile_indices) == 0:
            return

        try:
            self.mapper.reload_tiles(tile_indices)
        except Exception:
            pass

    def reload_tiles(self):
        """디스크에서 타일 다시 로드 (auto_refresh용)"""
        if self.mapper is None:
            return

        try:
            # 새 mapper 인스턴스 생성하여 최신 타일 로드
            self.mapper = OutofcoreTileMapper(
                self.map_path,
                self.resolution,
                self.tile_size,
                4  # Minimal cache - visualizer loads all tiles via get_all_occupied_voxels()
            )

            # 모든 타일 preload
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
        """주기적으로 맵 시각화 발행"""
        if self.mapper is None:
            # 맵 로드 재시도 (silent)
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

        # 선택적 리로드: mapper에서 업데이트된 타일만 리로드 (우선)
        if len(self.pending_tile_updates) > 0:
            self.reload_specific_tiles(self.pending_tile_updates)
            self.pending_tile_updates = []
            self.last_refresh_time = time.time()  # Reset refresh timer

        # Fallback: auto_refresh가 활성화되고 토픽 업데이트가 없으면 전체 리로드
        current_time = time.time()
        if self.auto_refresh and (current_time - self.last_refresh_time) >= self.refresh_interval:
            self.reload_tiles()
            self.last_refresh_time = current_time

        try:
            stamp = self.get_clock().now().to_msg()

            if self.visualization_mode == 'all':
                # 둘 다 발행
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
        """OctoMap 메시지 발행 (RViz OctoMap 플러그인용)"""
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
        """PointCloud2 발행"""
        # get_all_occupied_voxels: 모든 타일에서 복셀 로드 (캐시 제한 없음)
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
