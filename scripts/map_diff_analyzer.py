#!/usr/bin/env python3
"""
Map Difference Analyzer for ROV Detection Feasibility Test

Compare two occupancy maps and visualize differences.
Used to validate prior map effectiveness for change detection.
"""

import argparse
import json
import os
import struct
import numpy as np
from pathlib import Path
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dataclasses import dataclass
from typing import List, Tuple, Dict, Optional


@dataclass
class TileIndex:
    """Tile spatial index"""
    x: int
    y: int
    z: int

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z

    @classmethod
    def from_dirname(cls, dirname: str) -> Optional['TileIndex']:
        """Parse tile index from directory name (e.g., 'tile_-10_0_-4')"""
        if not dirname.startswith('tile_'):
            return None
        parts = dirname[5:].split('_')
        if len(parts) != 3:
            return None
        try:
            return cls(int(parts[0]), int(parts[1]), int(parts[2]))
        except ValueError:
            return None


class IWLOMetaLoader:
    """Load IWLO metadata from iwlo_meta.bin files"""

    def load(self, filepath: str, resolution: float = 0.2) -> List[List[float]]:
        """
        Load IWLO metadata file and return occupied voxels.

        Format (C++ OctreeStorage):
        - Magic: 'OLWI' (4 bytes, little-endian for IWLO)
        - Version: uint32 (4 bytes)
        - Count: uint64 (8 bytes)
        - Entries: key (3 x uint16) + log_odds (float64) + obs_count (int32) = 18 bytes each

        Keys are Octomap OcTreeKey values centered around 32768.

        Returns:
            List of [x, y, z, probability] for occupied voxels
        """
        voxels = []

        try:
            with open(filepath, 'rb') as f:
                # Check header
                header = f.read(4)
                if header != b'OLWI':
                    return voxels

                # Read version (uint32) and count (uint64)
                version = struct.unpack('I', f.read(4))[0]
                count = struct.unpack('Q', f.read(8))[0]

                if version != 1:
                    print(f"Warning: Unsupported IWLO version {version}")
                    return voxels

                # Read voxel data (18 bytes per entry)
                for _ in range(count):
                    # Key: 3 x uint16 (6 bytes) - Octomap key format
                    kx = struct.unpack('H', f.read(2))[0]
                    ky = struct.unpack('H', f.read(2))[0]
                    kz = struct.unpack('H', f.read(2))[0]

                    # Log-odds (float64, 8 bytes)
                    log_odds = struct.unpack('d', f.read(8))[0]

                    # Observation count (int32, 4 bytes)
                    obs_count = struct.unpack('i', f.read(4))[0]

                    # Only keep occupied voxels (log_odds > 0 means prob > 0.5)
                    if log_odds > 0:
                        # Convert key to world coordinates
                        # Octomap keys are centered at 32768
                        center = 32768
                        ix = kx - center
                        iy = ky - center
                        iz = kz - center

                        x = ix * resolution
                        y = iy * resolution
                        z = iz * resolution

                        # Convert log_odds to probability
                        if log_odds > 100:
                            prob = 1.0
                        elif log_odds < -100:
                            prob = 0.0
                        else:
                            prob = 1.0 / (1.0 + np.exp(-log_odds))

                        voxels.append([x, y, z, prob])

        except Exception as e:
            print(f"Error loading IWLO meta {filepath}: {e}")

        return voxels


class MapLoader:
    """Load map tiles from disk"""

    def __init__(self, map_path: str):
        self.map_path = Path(map_path)
        self.metadata = self._load_metadata()

    def _load_metadata(self) -> dict:
        """Load map metadata.json"""
        meta_path = self.map_path / 'metadata.json'
        if meta_path.exists():
            with open(meta_path) as f:
                return json.load(f)
        return {'resolution': 0.2, 'tile_size': 1.0}

    def list_tiles(self) -> List[TileIndex]:
        """List all tile indices in the map"""
        tiles = []
        for item in self.map_path.iterdir():
            if item.is_dir() and item.name.startswith('tile_'):
                idx = TileIndex.from_dirname(item.name)
                if idx:
                    tiles.append(idx)
        return tiles

    def load_tile_voxels(self, tile_idx: TileIndex,
                         min_prob: float = 0.5) -> np.ndarray:
        """
        Load occupied voxels from a tile.

        Uses IWLO metadata for accurate probability values.

        Returns:
            Nx4 array [x, y, z, probability]
        """
        tile_dir = self.map_path / f'tile_{tile_idx.x}_{tile_idx.y}_{tile_idx.z}'

        if not tile_dir.exists():
            return np.array([]).reshape(0, 4)

        resolution = self.metadata.get('resolution', 0.2)

        # Load from IWLO metadata (stores all voxels with log-odds)
        iwlo_path = tile_dir / 'iwlo_meta.bin'
        if iwlo_path.exists():
            loader = IWLOMetaLoader()
            voxels = loader.load(str(iwlo_path), resolution)

            if voxels:
                # Filter by min_prob if needed
                voxels_arr = np.array(voxels)
                if min_prob > 0.5:
                    mask = voxels_arr[:, 3] >= min_prob
                    voxels_arr = voxels_arr[mask]
                return voxels_arr

        return np.array([]).reshape(0, 4)

    def load_all_voxels(self, min_prob: float = 0.5,
                        progress: bool = True) -> np.ndarray:
        """Load all occupied voxels from all tiles"""
        tiles = self.list_tiles()
        all_voxels = []

        for i, tile_idx in enumerate(tiles):
            if progress and i % 50 == 0:
                print(f"  Loading tiles: {i}/{len(tiles)}")

            voxels = self.load_tile_voxels(tile_idx, min_prob)
            if len(voxels) > 0:
                all_voxels.append(voxels)

        if not all_voxels:
            return np.array([]).reshape(0, 4)

        return np.vstack(all_voxels)


class MapDiffAnalyzer:
    """Analyze differences between two maps"""

    def __init__(self, map_a_path: str, map_b_path: str, tolerance: float = 0.2):
        self.map_a = MapLoader(map_a_path)
        self.map_b = MapLoader(map_b_path)
        self.tolerance = tolerance

    def compute_diff(self, min_prob: float = 0.5) -> Dict:
        """
        Compute difference between two maps.

        Returns dict with:
            - voxels_a: all voxels from map A
            - voxels_b: all voxels from map B
            - common: voxels present in both maps
            - only_a: voxels only in map A
            - only_b: voxels only in map B
            - stats: statistics dictionary
        """
        print(f"\n=== Loading Map A: {self.map_a.map_path} ===")
        voxels_a = self.map_a.load_all_voxels(min_prob)
        print(f"  Loaded {len(voxels_a)} voxels from {len(self.map_a.list_tiles())} tiles")

        print(f"\n=== Loading Map B: {self.map_b.map_path} ===")
        voxels_b = self.map_b.load_all_voxels(min_prob)
        print(f"  Loaded {len(voxels_b)} voxels from {len(self.map_b.list_tiles())} tiles")

        if len(voxels_a) == 0 or len(voxels_b) == 0:
            print("Warning: One or both maps have no voxels!")
            return {
                'voxels_a': voxels_a,
                'voxels_b': voxels_b,
                'common': np.array([]).reshape(0, 4),
                'only_a': voxels_a,
                'only_b': voxels_b,
                'stats': {'error': 'Empty map'}
            }

        print(f"\n=== Computing Difference (tolerance={self.tolerance}m) ===")

        # Build KDTree for map B
        tree_b = cKDTree(voxels_b[:, :3])

        # Find matches for each voxel in A
        distances, indices = tree_b.query(voxels_a[:, :3], k=1)
        matched_a = distances <= self.tolerance

        # Build KDTree for map A
        tree_a = cKDTree(voxels_a[:, :3])

        # Find matches for each voxel in B
        distances_b, indices_b = tree_a.query(voxels_b[:, :3], k=1)
        matched_b = distances_b <= self.tolerance

        # Compute sets
        common_a = voxels_a[matched_a]
        only_a = voxels_a[~matched_a]
        only_b = voxels_b[~matched_b]

        # Statistics
        stats = {
            'total_a': len(voxels_a),
            'total_b': len(voxels_b),
            'common': len(common_a),
            'only_a': len(only_a),
            'only_b': len(only_b),
            'match_ratio_a': len(common_a) / len(voxels_a) if len(voxels_a) > 0 else 0,
            'match_ratio_b': np.sum(matched_b) / len(voxels_b) if len(voxels_b) > 0 else 0,
            'diff_ratio_a': len(only_a) / len(voxels_a) if len(voxels_a) > 0 else 0,
            'diff_ratio_b': len(only_b) / len(voxels_b) if len(voxels_b) > 0 else 0,
        }

        return {
            'voxels_a': voxels_a,
            'voxels_b': voxels_b,
            'common': common_a,
            'only_a': only_a,
            'only_b': only_b,
            'stats': stats
        }

    def print_stats(self, result: Dict):
        """Print statistics"""
        stats = result['stats']

        print("\n" + "="*60)
        print("MAP DIFFERENCE ANALYSIS RESULTS")
        print("="*60)
        print(f"\nMap A: {self.map_a.map_path.name}")
        print(f"  Total voxels: {stats['total_a']:,}")
        print(f"  Matched: {stats['common']:,} ({stats['match_ratio_a']*100:.1f}%)")
        print(f"  Only in A: {stats['only_a']:,} ({stats['diff_ratio_a']*100:.1f}%)")

        print(f"\nMap B: {self.map_b.map_path.name}")
        print(f"  Total voxels: {stats['total_b']:,}")
        print(f"  Matched: {int(stats['match_ratio_b']*stats['total_b']):,} ({stats['match_ratio_b']*100:.1f}%)")
        print(f"  Only in B: {stats['only_b']:,} ({stats['diff_ratio_b']*100:.1f}%)")

        print("\n" + "-"*60)
        print("FEASIBILITY ASSESSMENT")
        print("-"*60)

        avg_diff_ratio = (stats['diff_ratio_a'] + stats['diff_ratio_b']) / 2
        if avg_diff_ratio < 0.1:
            print(f"✓ EXCELLENT: Avg diff ratio {avg_diff_ratio*100:.1f}% < 10%")
            print("  Prior map approach is highly feasible for ROV detection")
        elif avg_diff_ratio < 0.3:
            print(f"△ MODERATE: Avg diff ratio {avg_diff_ratio*100:.1f}%")
            print("  Prior map approach may work with additional filtering")
        else:
            print(f"✗ POOR: Avg diff ratio {avg_diff_ratio*100:.1f}% > 30%")
            print("  High noise/misalignment - need better registration")
        print("="*60)

    def visualize(self, result: Dict, output_path: Optional[str] = None):
        """Visualize map differences"""
        voxels_a = result['voxels_a']
        voxels_b = result['voxels_b']
        only_a = result['only_a']
        only_b = result['only_b']
        common = result['common']

        # Subsample for visualization if too many points
        max_points = 50000

        def subsample(arr, max_n):
            if len(arr) > max_n:
                idx = np.random.choice(len(arr), max_n, replace=False)
                return arr[idx]
            return arr

        fig = plt.figure(figsize=(16, 6))

        # Plot 1: Both maps overlaid
        ax1 = fig.add_subplot(131, projection='3d')

        va_sub = subsample(voxels_a, max_points)
        vb_sub = subsample(voxels_b, max_points)

        if len(va_sub) > 0:
            ax1.scatter(va_sub[:, 0], va_sub[:, 1], va_sub[:, 2],
                       c='red', alpha=0.3, s=1, label=f'Map A ({len(voxels_a):,})')
        if len(vb_sub) > 0:
            ax1.scatter(vb_sub[:, 0], vb_sub[:, 1], vb_sub[:, 2],
                       c='blue', alpha=0.3, s=1, label=f'Map B ({len(voxels_b):,})')
        ax1.set_title('Both Maps Overlaid')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.legend()

        # Plot 2: Differences only
        ax2 = fig.add_subplot(132, projection='3d')

        oa_sub = subsample(only_a, max_points // 2)
        ob_sub = subsample(only_b, max_points // 2)

        if len(oa_sub) > 0:
            ax2.scatter(oa_sub[:, 0], oa_sub[:, 1], oa_sub[:, 2],
                       c='red', alpha=0.5, s=2, label=f'Only A ({len(only_a):,})')
        if len(ob_sub) > 0:
            ax2.scatter(ob_sub[:, 0], ob_sub[:, 1], ob_sub[:, 2],
                       c='blue', alpha=0.5, s=2, label=f'Only B ({len(only_b):,})')
        ax2.set_title('Differences Only\n(Potential ROV Candidates)')
        ax2.set_xlabel('X (m)')
        ax2.set_ylabel('Y (m)')
        ax2.set_zlabel('Z (m)')
        ax2.legend()

        # Plot 3: Common voxels
        ax3 = fig.add_subplot(133, projection='3d')

        com_sub = subsample(common, max_points)
        if len(com_sub) > 0:
            ax3.scatter(com_sub[:, 0], com_sub[:, 1], com_sub[:, 2],
                       c='green', alpha=0.3, s=1, label=f'Common ({len(common):,})')
        ax3.set_title('Common Voxels\n(Static Environment)')
        ax3.set_xlabel('X (m)')
        ax3.set_ylabel('Y (m)')
        ax3.set_zlabel('Z (m)')
        ax3.legend()

        plt.tight_layout()

        if output_path:
            os.makedirs(output_path, exist_ok=True)
            fig_path = os.path.join(output_path, 'map_diff_visualization.png')
            plt.savefig(fig_path, dpi=150, bbox_inches='tight')
            print(f"\nVisualization saved to: {fig_path}")
        else:
            plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Analyze differences between two occupancy maps')
    parser.add_argument('--map_a', type=str, required=True,
                        help='Path to first map (map_tile directory)')
    parser.add_argument('--map_b', type=str, required=True,
                        help='Path to second map (map_tile directory)')
    parser.add_argument('--tolerance', type=float, default=0.2,
                        help='Voxel matching tolerance in meters (default: 0.2)')
    parser.add_argument('--min_prob', type=float, default=0.5,
                        help='Minimum probability threshold (default: 0.5)')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory for results')
    parser.add_argument('--no_viz', action='store_true',
                        help='Skip visualization')

    args = parser.parse_args()

    # Create analyzer
    analyzer = MapDiffAnalyzer(args.map_a, args.map_b, args.tolerance)

    # Compute differences
    result = analyzer.compute_diff(args.min_prob)

    # Print statistics
    analyzer.print_stats(result)

    # Visualize
    if not args.no_viz:
        analyzer.visualize(result, args.output)

    # Save results if output path specified
    if args.output:
        os.makedirs(args.output, exist_ok=True)

        # Save statistics
        stats_path = os.path.join(args.output, 'stats.json')
        with open(stats_path, 'w') as f:
            json.dump(result['stats'], f, indent=2)
        print(f"Statistics saved to: {stats_path}")

        # Save difference voxels
        if len(result['only_a']) > 0:
            np.save(os.path.join(args.output, 'only_a.npy'), result['only_a'])
        if len(result['only_b']) > 0:
            np.save(os.path.join(args.output, 'only_b.npy'), result['only_b'])


if __name__ == '__main__':
    main()
