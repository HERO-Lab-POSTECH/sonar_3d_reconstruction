#!/usr/bin/env python3
"""
Map Diff Visualizer - RViz2 PointCloud Publisher

Publishes map difference results as PointCloud2 for interactive 3D visualization in RViz2.

Usage:
    ros2 run sonar_3d_reconstruction map_diff_visualizer.py \
        --map_a /path/to/map_a/map_tile \
        --map_b /path/to/map_b/map_tile

Topics published:
    - /perception/map_diff/map_a (PointCloud2): Red - Map A voxels
    - /perception/map_diff/map_b (PointCloud2): Blue - Map B voxels
    - /perception/map_diff/only_a (PointCloud2): Orange - Only in Map A
    - /perception/map_diff/only_b (PointCloud2): Cyan - Only in Map B
    - /perception/map_diff/common (PointCloud2): Green - Common voxels
"""

import argparse
import struct
import numpy as np
from pathlib import Path
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from sonar_3d_reconstruction.qos import SENSOR_QOS
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class IWLOMetaLoader:
    """Load IWLO metadata from iwlo_meta.bin files"""

    def load(self, filepath: str, resolution: float = 0.2) -> List[List[float]]:
        """Load occupied voxels from IWLO metadata file."""
        voxels = []
        try:
            with open(filepath, 'rb') as f:
                header = f.read(4)
                if header != b'OLWI':
                    return voxels

                version = struct.unpack('I', f.read(4))[0]
                count = struct.unpack('Q', f.read(8))[0]

                if version != 1:
                    return voxels

                center = 32768
                for _ in range(count):
                    kx = struct.unpack('H', f.read(2))[0]
                    ky = struct.unpack('H', f.read(2))[0]
                    kz = struct.unpack('H', f.read(2))[0]
                    log_odds = struct.unpack('d', f.read(8))[0]
                    _ = struct.unpack('i', f.read(4))[0]  # obs_count

                    if log_odds > 0:
                        ix, iy, iz = kx - center, ky - center, kz - center
                        x = ix * resolution
                        y = iy * resolution
                        z = iz * resolution
                        prob = 1.0 / (1.0 + np.exp(-log_odds)) if -100 < log_odds < 100 else 1.0
                        voxels.append([x, y, z, prob])
        except Exception:
            pass
        return voxels


class MapDiffVisualizer(Node):
    """ROS2 node for visualizing map differences in RViz2"""

    def __init__(self, map_a_path: str, map_b_path: str, tolerance: float = 0.4):
        super().__init__('map_diff_visualizer')

        self.map_a_path = Path(map_a_path)
        self.map_b_path = Path(map_b_path)
        self.tolerance = tolerance

        # Parameters
        self.declare_parameter('frame_id', 'map')
        self.frame_id = self.get_parameter('frame_id').value

        # Publishers
        self.pub_map_a = self.create_publisher(PointCloud2, '/perception/map_diff/map_a', SENSOR_QOS)
        self.pub_map_b = self.create_publisher(PointCloud2, '/perception/map_diff/map_b', SENSOR_QOS)
        self.pub_only_a = self.create_publisher(PointCloud2, '/perception/map_diff/only_a', SENSOR_QOS)
        self.pub_only_b = self.create_publisher(PointCloud2, '/perception/map_diff/only_b', SENSOR_QOS)
        self.pub_common = self.create_publisher(PointCloud2, '/perception/map_diff/common', SENSOR_QOS)

        # Load and compute diff
        self.get_logger().info('Loading maps...')
        self.voxels_a, self.voxels_b, self.only_a, self.only_b, self.common = self._compute_diff()

        self.get_logger().info(f'Map A: {len(self.voxels_a)} voxels')
        self.get_logger().info(f'Map B: {len(self.voxels_b)} voxels')
        self.get_logger().info(f'Only A: {len(self.only_a)} voxels')
        self.get_logger().info(f'Only B: {len(self.only_b)} voxels')
        self.get_logger().info(f'Common: {len(self.common)} voxels')

        # Timer for publishing (1 Hz)
        self.timer = self.create_timer(1.0, self._publish_clouds)
        self.get_logger().info('Publishing to /perception/map_diff/* topics. Open RViz2 to visualize.')

    def _load_map(self, map_path: Path) -> np.ndarray:
        """Load all occupied voxels from a map"""
        metadata_path = map_path / 'metadata.json'
        resolution = 0.2
        if metadata_path.exists():
            import json
            with open(metadata_path) as f:
                meta = json.load(f)
                resolution = meta.get('resolution', 0.2)

        all_voxels = []
        loader = IWLOMetaLoader()

        for tile_dir in map_path.iterdir():
            if not tile_dir.is_dir() or not tile_dir.name.startswith('tile_'):
                continue
            iwlo_path = tile_dir / 'iwlo_meta.bin'
            if iwlo_path.exists():
                voxels = loader.load(str(iwlo_path), resolution)
                if voxels:
                    all_voxels.extend(voxels)

        if not all_voxels:
            return np.array([]).reshape(0, 4)
        return np.array(all_voxels)

    def _compute_diff(self) -> Tuple[np.ndarray, ...]:
        """Compute map difference using voxel grid hashing"""
        voxels_a = self._load_map(self.map_a_path)
        voxels_b = self._load_map(self.map_b_path)

        if len(voxels_a) == 0 or len(voxels_b) == 0:
            return voxels_a, voxels_b, voxels_a, voxels_b, np.array([]).reshape(0, 4)

        # Use voxel grid hashing for fast matching (tolerance-based)
        def to_grid_key(points, tolerance):
            """Convert points to grid keys"""
            return np.floor(points[:, :3] / tolerance).astype(np.int32)

        # Create set of grid keys for each map
        keys_a = to_grid_key(voxels_a, self.tolerance)
        keys_b = to_grid_key(voxels_b, self.tolerance)

        # Convert to tuples for set operations
        set_a = set(map(tuple, keys_a))
        set_b = set(map(tuple, keys_b))

        # Find matches (check neighboring cells too for robustness)
        def find_matches(keys, target_set):
            """Find which keys have a match in target_set (including neighbors)"""
            matched = np.zeros(len(keys), dtype=bool)
            for i, key in enumerate(keys):
                # Check current cell and 26 neighbors
                for dx in [-1, 0, 1]:
                    for dy in [-1, 0, 1]:
                        for dz in [-1, 0, 1]:
                            neighbor = (key[0] + dx, key[1] + dy, key[2] + dz)
                            if neighbor in target_set:
                                matched[i] = True
                                break
                        if matched[i]:
                            break
                    if matched[i]:
                        break
            return matched

        matched_a = find_matches(keys_a, set_b)
        matched_b = find_matches(keys_b, set_a)

        common = voxels_a[matched_a]
        only_a = voxels_a[~matched_a]
        only_b = voxels_b[~matched_b]

        return voxels_a, voxels_b, only_a, only_b, common

    def _create_pointcloud(self, points: np.ndarray, r: int, g: int, b: int) -> PointCloud2:
        """Create PointCloud2 message with RGB color"""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        if len(points) == 0:
            msg = PointCloud2()
            msg.header = header
            return msg

        rgb_uint32 = np.frombuffer(
            np.array([b, g, r, 255], dtype=np.uint8).tobytes(), dtype='<u4'
        )[0]

        packed = np.empty(len(points), dtype=[
            ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('rgb', '<u4'),
        ])
        packed['x'] = points[:, 0]
        packed['y'] = points[:, 1]
        packed['z'] = points[:, 2]
        packed['rgb'] = rgb_uint32

        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.data = packed.tobytes()
        msg.is_dense = True

        return msg

    def _publish_clouds(self):
        """Publish all point clouds"""
        # Map A - Red
        self.pub_map_a.publish(self._create_pointcloud(self.voxels_a, 255, 100, 100))
        # Map B - Blue
        self.pub_map_b.publish(self._create_pointcloud(self.voxels_b, 100, 100, 255))
        # Only A - Orange
        self.pub_only_a.publish(self._create_pointcloud(self.only_a, 255, 165, 0))
        # Only B - Cyan
        self.pub_only_b.publish(self._create_pointcloud(self.only_b, 0, 255, 255))
        # Common - Green
        self.pub_common.publish(self._create_pointcloud(self.common, 100, 255, 100))


def main():
    parser = argparse.ArgumentParser(description='Visualize map differences in RViz2')
    parser.add_argument('--map_a', type=str, required=True, help='Path to map A (map_tile directory)')
    parser.add_argument('--map_b', type=str, required=True, help='Path to map B (map_tile directory)')
    parser.add_argument('--tolerance', type=float, default=0.4, help='Matching tolerance (m)')

    # Parse known args to allow ROS2 args
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = MapDiffVisualizer(args.map_a, args.map_b, args.tolerance)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
