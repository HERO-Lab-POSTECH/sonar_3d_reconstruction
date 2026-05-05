#!/usr/bin/env python3
"""
3D Sonar Mapping Library with Probabilistic Octree
Based on feature_extraction_3d.py with adaptations for real data

This module provides octree-based sparse storage and probabilistic mapping
for 3D sonar reconstruction using log-odds Bayesian updates.

Author: Sonar 3D Reconstruction Team
Date: 2025
"""

import numpy as np
from collections import defaultdict
from typing import Tuple, List, Dict, Any, Optional
import time
import warnings

# Import C++ module from package
from sonar_3d_reconstruction import (
    CPP_MODULE_AVAILABLE,
    OUTOFCORE_AVAILABLE,
    ProbabilityUpdater,
    MemoryStats,
    OutofcoreTileMapper,
)


# Python SimpleOctree class removed - using C++ backend only


class SonarTo3DMapper:
    """
    Convert sonar images to 3D point clouds with probabilistic mapping
    Accumulates multiple frames and updates voxel probabilities
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """
        Initialize mapper with configuration
        
        Args:
            config: Configuration dictionary (overrides defaults)
        """
        # Default configuration
        default_config = {
            # Sonar parameters
            'horizontal_fov': 130.0,       # degrees
            'vertical_aperture': 20.0,     # degrees
            # max_range is received dynamically from /param/range topic
            'min_range': 0.5,              # meters
            'intensity_threshold': 35,     # 0-255 scale

            # Sonar mounting (relative to base_link)
            'sonar_position': [0.0, 0.0, -0.5],  # xyz
            'sonar_orientation': [0.0, 1.5708, 0.0],  # rpy (0, 90deg, 0)

            # Octree parameters
            'voxel_resolution': 0.05,      # meters
            'dynamic_expansion': True,

            # Probability threshold (2-class: occupied vs free)
            'occupied_threshold': 0.7,     # prob >= this = occupied, < this = free

            # Adaptive update
            'adaptive_update': True,
            'adaptive_threshold': 0.5,
            'adaptive_max_ratio': 0.3,

            # IWLO parameters
            'L_occ': 3.5,          # Log-odds occupied increment
            'L_free': -3.0,        # Log-odds free decrement
            'L_min': -10.0,        # Saturation lower bound
            'L_max': 10.0,         # Saturation upper bound
            'intensity_max': 255,  # Maximum intensity value for normalization

            # Shadow region protection
            'angular_cone_width': 0.5,     # 0.5 = no overlap, 1.0 = full overlap

            # Processing parameters
            'frame_skip': 1,  # Process every N frames

            # Out-of-Core parameters (disk-based storage)
            'use_outofcore': False,        # Enable disk-based tile storage
            'outofcore_map_path': '/workspace/data/map_tiles',  # Tile storage directory
            'outofcore_tile_size': 10.0,   # Tile size in meters
            'outofcore_cache_size': 16,    # Max tiles in memory
        }
        
        # Update with provided config
        if config:
            default_config.update(config)

        # Store parameters
        self.horizontal_fov = np.radians(default_config['horizontal_fov'])
        self.vertical_aperture = np.radians(default_config['vertical_aperture'])
        self.max_range = None  # Set dynamically from /param/range topic
        self.min_range = default_config['min_range']
        self.intensity_threshold = default_config['intensity_threshold']

        # Processing parameters
        self.frame_skip = default_config['frame_skip']

        # Image dimensions - dynamically set from first frame
        self.image_width = None
        self.image_height = None
        self.voxel_resolution = default_config['voxel_resolution']
        self.occupied_threshold = default_config['occupied_threshold']
        self.dynamic_expansion = default_config['dynamic_expansion']

        # Shadow region protection
        self.angular_cone_width = default_config.get('angular_cone_width', 0.5)

        # Sonar mounting transform
        self.sonar_position = np.array(default_config['sonar_position'])
        self.sonar_orientation = np.array(default_config['sonar_orientation'])
        
        # Pre-compute sonar to base_link transform
        self.T_sonar_to_base = self.create_transform_matrix(
            self.sonar_position,
            self.sonar_orientation
        )
        
        # Initialize C++ backend
        self.use_cpp = default_config.get('use_cpp_backend', CPP_MODULE_AVAILABLE)
        self.use_outofcore = default_config.get('use_outofcore', False)

        # Store log-odds values for ray processing
        self.log_odds_occupied = default_config['L_occ']
        self.log_odds_free = default_config['L_free']

        # Store IWLO parameters for dynamic updates
        self.sharpness = default_config.get('sharpness', 3.0)
        self.decay_rate = default_config.get('decay_rate', 0.1)
        self.min_alpha = default_config.get('min_alpha', 0.1)
        self.L_min = default_config.get('L_min', -10.0)
        self.L_max = default_config.get('L_max', 10.0)
        self.intensity_max = default_config.get('intensity_max', 255)

        # Store adaptive parameters for dynamic updates
        self.adaptive_update = default_config.get('adaptive_update', True)
        self.adaptive_threshold = default_config.get('adaptive_threshold', 0.5)
        self.adaptive_max_ratio = default_config.get('adaptive_max_ratio', 0.3)

        # Initialize octree backend
        self.octree = self._create_octree_backend(default_config)
        self._configure_octree(default_config)

        # Depth estimation from reference map
        self.depth_estimation_enabled = default_config.get('depth_estimation_enabled', False)
        self.depth_estimation_threshold = default_config.get('depth_estimation_depth_diff_threshold', 1.0)
        self.depth_estimation_ray_step_multiplier = default_config.get('depth_estimation_ray_step_multiplier', 2.0)
        self.depth_estimation_min_confidence = default_config.get('depth_estimation_min_confidence', 0.7)
        self.reference_map = None
        ref_map_path = default_config.get('depth_estimation_reference_map_path', '')
        if self.depth_estimation_enabled and ref_map_path and OUTOFCORE_AVAILABLE:
            self._load_reference_map(ref_map_path, default_config)
        
        # Bearing angles - initialized dynamically in process_sonar_image
        self.bearing_angles = None
        
        # Frame counter
        self.frame_count = 0
        self.processed_frame_count = 0

        # Processing statistics
        self.last_processing_time = 0.0
        self.total_processing_time = 0.0

    def _load_reference_map(self, ref_map_path: str, config: Dict[str, Any]):
        """Load reference map as read-only OutofcoreTileMapper for depth estimation."""
        import os
        if not os.path.exists(ref_map_path):
            return
        try:
            tile_size = config.get('outofcore_tile_size', 10.0)
            cache_size = config.get('outofcore_cache_size', 16)
            self.reference_map = OutofcoreTileMapper(
                ref_map_path, self.voxel_resolution, tile_size, cache_size
            )
        except Exception:
            self.reference_map = None

    def compute_depth_estimation(self, polar_image: np.ndarray,
                                  bearing_step: int,
                                  T_sonar_to_world: np.ndarray,
                                  sonar_origin_world: np.ndarray,
                                  range_resolution: float) -> Optional[Dict[str, Any]]:
        """
        Ray-cast into reference map and compare with actual sonar depths.
        Returns depth estimation results for logging/filtering.
        Returns None if depth estimation is disabled or reference map not loaded.
        """
        if self.reference_map is None or not self.depth_estimation_enabled:
            return None
        if self.max_range is None:
            return None

        bearing_bins = polar_image.shape[1]
        active_indices = []
        directions = []
        actual_depths = []

        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            # Ray direction in sonar frame → world frame
            dir_sonar = np.array([
                np.cos(bearing_angle),
                -np.sin(bearing_angle),
                0.0,
            ])
            dir_world = T_sonar_to_world[:3, :3] @ dir_sonar
            dir_norm = np.linalg.norm(dir_world)
            if dir_norm < 1e-10:
                continue
            dir_world /= dir_norm

            # Actual first hit from sonar image
            intensity_profile = polar_image[:, b_idx]
            first_hit_idx = self._first_hit_index(intensity_profile, range_resolution)
            actual_depth = first_hit_idx * range_resolution if first_hit_idx >= 0 else -1.0

            active_indices.append(b_idx)
            directions.append(dir_world)
            actual_depths.append(actual_depth)

        if not directions:
            return None

        # Batch ray-cast into reference map
        directions_array = np.array(directions, dtype=np.float64)
        origin = np.array(sonar_origin_world[:3], dtype=np.float64)
        step_size = self.voxel_resolution * self.depth_estimation_ray_step_multiplier

        expected_depths = self.reference_map.batch_ray_cast_depth(
            origin, directions_array, self.max_range, step_size,
            self.depth_estimation_min_confidence
        )

        # Build results
        threshold = self.depth_estimation_threshold
        bearing_mask = np.ones(bearing_bins, dtype=bool)
        num_matched = 0
        num_new = 0
        num_no_ref = 0
        sample_results = []  # For logging

        for i, b_idx in enumerate(active_indices):
            actual = actual_depths[i]
            expected = expected_depths[i]
            bearing_deg = np.degrees(self.bearing_angles[b_idx])

            if expected < 0:
                # No reference data → process normally
                bearing_mask[b_idx] = True
                num_no_ref += 1
            elif actual < 0:
                # No sonar hit but reference has data → skip
                bearing_mask[b_idx] = False
                num_matched += 1
            elif (expected - actual) > threshold:
                # Actual is CLOSER than expected → new object in front
                bearing_mask[b_idx] = True
                num_new += 1
            else:
                # Actual matches or is further → existing environment, skip
                bearing_mask[b_idx] = False
                num_matched += 1

            # Collect samples for logging (every ~50 bearings)
            if len(sample_results) < 5 and i % max(1, len(active_indices) // 5) == 0:
                sample_results.append({
                    'bearing_deg': round(bearing_deg, 1),
                    'actual': round(actual, 2) if actual >= 0 else -1,
                    'expected': round(expected, 2) if expected >= 0 else -1,
                    'is_new': bearing_mask[b_idx],
                })

        return {
            'bearing_mask': bearing_mask,
            'num_matched': num_matched,
            'num_new': num_new,
            'num_no_ref': num_no_ref,
            'total_bearings': len(active_indices),
            'samples': sample_results,
        }

    def _create_octree_backend(self, config: Dict[str, Any]):
        """Create octree backend instance based on configuration."""
        if self.use_outofcore and OUTOFCORE_AVAILABLE:
            map_path = config.get('outofcore_map_path', '/workspace/data/map_tiles')
            tile_size = config.get('outofcore_tile_size', 10.0)
            cache_size = config.get('outofcore_cache_size', 16)
            return OutofcoreTileMapper(
                map_path, self.voxel_resolution, tile_size, cache_size
            )
        elif self.use_cpp and CPP_MODULE_AVAILABLE:
            return ProbabilityUpdater(self.voxel_resolution)
        else:
            raise RuntimeError(
                "C++ backend is required but not available. "
                "Please build the C++ module: colcon build --packages-select sonar_3d_reconstruction"
            )

    def _configure_octree(self, config: Dict[str, Any]) -> None:
        """Configure octree parameters for both RAM and disk-based backends."""
        # Common parameters for all backends
        self.octree.set_log_odds_params(self.log_odds_occupied, self.log_odds_free)
        self.octree.set_adaptive_params(
            config['adaptive_update'],
            config['adaptive_threshold'],
            config['adaptive_max_ratio']
        )
        self.octree.set_iwlo_params(
            config.get('sharpness', 0.1),
            config.get('decay_rate', 0.1),
            config.get('min_alpha', 0.3),
            config.get('L_min', -10.0),
            config.get('L_max', 10.0)
        )
        self.octree.set_intensity_params(
            self.intensity_threshold,
            config.get('intensity_max', 255)
        )

        # RAM-based backend specific: clamping thresholds
        if not self.use_outofcore and hasattr(self.octree, 'set_clamping_thresholds'):
            min_prob = 1.0 / (1.0 + np.exp(-config['L_min']))
            max_prob = 1.0 / (1.0 + np.exp(-config['L_max']))
            self.octree.set_clamping_thresholds(min_prob, max_prob)

    def create_transform_matrix(self, position: np.ndarray, rpy: np.ndarray) -> np.ndarray:
        """
        Create 4x4 homogeneous transform matrix from position and RPY
        
        Args:
            position: [x, y, z] translation
            rpy: [roll, pitch, yaw] rotation in radians
            
        Returns:
            4x4 numpy array transform matrix
        """
        # Create rotation matrix from RPY
        cr = np.cos(rpy[0])
        sr = np.sin(rpy[0])
        cp = np.cos(rpy[1])
        sp = np.sin(rpy[1])
        cy = np.cos(rpy[2])
        sy = np.sin(rpy[2])
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp, cp*sr, cp*cr]
        ])
        
        # Build 4x4 homogeneous transformation matrix
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = position
        
        return T
    
    def quaternion_to_matrix(self, quaternion: List[float]) -> np.ndarray:
        """
        Convert quaternion to rotation matrix
        
        Args:
            quaternion: [x, y, z, w] quaternion
            
        Returns:
            3x3 rotation matrix
        """
        x, y, z, w = quaternion
        
        R = np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - w*z), 2*(x*z + w*y)],
            [2*(x*y + w*z), 1 - 2*(x**2 + z**2), 2*(y*z - w*x)],
            [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x**2 + y**2)]
        ])
        
        return R

    def update_sonar_orientation(self):
        """
        Recompute sonar-to-base transform after orientation change.
        Call this after modifying self.sonar_orientation.
        """
        self.T_sonar_to_base = self.create_transform_matrix(
            self.sonar_position,
            self.sonar_orientation
        )

    def create_odometry_transform(self, position: List[float], quaternion: List[float]) -> np.ndarray:
        """
        Create transformation matrix from odometry data
        
        Args:
            position: [x, y, z] position
            quaternion: [x, y, z, w] orientation
            
        Returns:
            4x4 transformation matrix
        """
        T = np.eye(4)
        T[:3, :3] = self.quaternion_to_matrix(quaternion)
        T[:3, 3] = position
        return T

    def update_max_range(self, new_range: float) -> None:
        """
        Update max_range dynamically (e.g., from sonar driver topic)

        Args:
            new_range: New maximum range in meters
        """
        if new_range > 0:
            self.max_range = new_range

    # === Parameter update handlers for ParameterManager ===

    def update_min_range(self, value: float) -> None:
        """Update minimum range parameter"""
        self.min_range = float(value)

    def update_intensity(self, value: int) -> None:
        """Update intensity threshold and propagate to C++ backend"""
        self.intensity_threshold = int(value)
        if hasattr(self, 'octree') and self.octree is not None:
            self.octree.set_intensity_params(self.intensity_threshold, self.intensity_max)

    def update_occupied_threshold(self, value: float) -> None:
        """Update occupied threshold parameter"""
        self.occupied_threshold = float(value)

    def update_angular_cone(self, value: float) -> None:
        """Update angular cone width parameter"""
        self.angular_cone_width = float(value)

    def update_frame_skip(self, value: int) -> None:
        """Update frame skip parameter"""
        self.frame_skip = int(value)

    def update_dynamic_expansion(self, value: bool) -> None:
        """Update dynamic expansion flag"""
        self.dynamic_expansion = bool(value)

    def is_bearing_in_valid_fov(self, bearing_angle: float) -> bool:
        """Check if bearing angle is within valid FOV"""
        half_fov = self.horizontal_fov / 2
        return abs(bearing_angle) <= half_fov

    def _first_hit_index(self, intensity_profile: np.ndarray, range_resolution: float) -> int:
        hits = (intensity_profile > self.intensity_threshold) & (
            np.arange(len(intensity_profile)) * range_resolution >= self.min_range
        )
        if not hits.any():
            return -1
        return int(np.argmax(hits))

    def is_in_shadow_region(self, voxel_range: float, bearing_angle: float,
                            bearing_first_hits: List[Tuple[float, float]]) -> bool:
        """
        Check if a voxel is in another bearing's shadow region (optimized)

        Shadow region: area behind first hit of adjacent bearings.
        Uses binary search on sorted bearing list for O(log N) complexity.

        Args:
            voxel_range: Range of the voxel from sensor origin
            bearing_angle: Bearing angle of the current ray
            bearing_first_hits: Sorted list of (bearing, first_hit_range) tuples

        Returns:
            True if voxel is in shadow region (should skip free update)
        """
        if not bearing_first_hits:
            return False

        bearing_resolution = self.horizontal_fov / self.image_width
        tolerance = bearing_resolution * self.angular_cone_width * 2

        # Binary search for nearest bearing
        left, right = 0, len(bearing_first_hits) - 1
        while left <= right:
            mid = (left + right) // 2
            mid_bearing = bearing_first_hits[mid][0]

            if abs(mid_bearing - bearing_angle) < tolerance:
                # Found adjacent bearing - check shadow
                if mid_bearing != bearing_angle:
                    first_hit = bearing_first_hits[mid][1]
                    if first_hit > 0 and voxel_range > first_hit:
                        return True

                # Check immediate neighbors
                if mid > 0:
                    prev_bearing, prev_hit = bearing_first_hits[mid - 1]
                    if abs(prev_bearing - bearing_angle) < tolerance and prev_bearing != bearing_angle:
                        if prev_hit > 0 and voxel_range > prev_hit:
                            return True

                if mid < len(bearing_first_hits) - 1:
                    next_bearing, next_hit = bearing_first_hits[mid + 1]
                    if abs(next_bearing - bearing_angle) < tolerance and next_bearing != bearing_angle:
                        if next_hit > 0 and voxel_range > next_hit:
                            return True
                return False

            elif mid_bearing < bearing_angle:
                left = mid + 1
            else:
                right = mid - 1

        return False

    def world_to_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """
        Convert world coordinates to voxel key (compatible with both C++ and Python)
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            Tuple (ix, iy, iz) as voxel index
        """
        i = int(np.floor(x / self.voxel_resolution))
        j = int(np.floor(y / self.voxel_resolution))
        k = int(np.floor(z / self.voxel_resolution))
        return (i, j, k)
    
    def key_to_world(self, key: Tuple[int, int, int]) -> np.ndarray:
        """
        Convert voxel key to world coordinates (center of voxel)
        
        Args:
            key: Tuple (ix, iy, iz) voxel index
            
        Returns:
            numpy array [x, y, z] world coordinates
        """
        x = (key[0] + 0.5) * self.voxel_resolution
        y = (key[1] + 0.5) * self.voxel_resolution
        z = (key[2] + 0.5) * self.voxel_resolution
        return np.array([x, y, z])
    
    def process_sonar_ray(self, bearing_angle: float, intensity_profile: np.ndarray,
                          T_sonar_to_world: np.ndarray) -> List[Tuple[np.ndarray, float, str, Optional[float]]]:
        """
        Process a single sonar ray and return voxel updates

        Args:
            bearing_angle: Horizontal angle in radians
            intensity_profile: 1D array of intensities along range
            T_sonar_to_world: 4x4 transform matrix

        Returns:
            List of (point, log_odds_update, type, intensity) tuples
        """
        updates = []
        
        # Find first hit
        range_resolution = self.max_range / len(intensity_profile)
        first_hit_idx = self._first_hit_index(intensity_profile, range_resolution)

        # If no hit, skip this ray (no update)
        if first_hit_idx == -1:
            return updates  # Return empty updates - no information available

        # Calculate vertical aperture parameters
        half_aperture = self.vertical_aperture / 2

        # Process free space before first hit (sparse) - always enabled for carving
        # Note: min_range only affects first hit detection, not free space carving
        # All voxels before first hit are free (including those within min_range)
        free_sampling_step = 10
        for r_idx in range(0, first_hit_idx, free_sampling_step):
            range_m = r_idx * range_resolution

            # Calculate vertical spread and sample count
            # Sparse sampling (4x voxel) for free space - sufficient for carving
            vertical_spread = range_m * np.tan(half_aperture)
            num_vertical = max(1, int(vertical_spread / (self.voxel_resolution * 4)))

            for v_step in range(-num_vertical, num_vertical + 1):
                # Normalize to [-1, 1] for exact aperture coverage: ±half_aperture
                vertical_angle = (v_step / num_vertical) * half_aperture

                # Sonar coordinate system (NED-style, right-handed):
                #   X = forward (range direction at bearing=0)
                #   Y = right (starboard, positive bearing direction)
                #   Z = down (positive vertical angle = below horizontal)
                #
                # Spherical to Cartesian conversion:
                #   - bearing_angle: horizontal angle from forward (+X axis)
                #   - vertical_angle: elevation from horizontal plane
                #   - Y uses negative sin() because positive bearing is clockwise
                #     when viewed from above, but sin() assumes CCW positive
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)

                # Transform to world
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar

                updates.append((pt_world[:3], self.log_odds_free, 'free', None))
        
        # Process occupied regions (dense)
        if first_hit_idx < len(intensity_profile):
            # Find all high intensity regions
            for r_idx in range(first_hit_idx, min(first_hit_idx + 50, len(intensity_profile))):
                intensity = intensity_profile[r_idx]

                # Simple threshold check
                is_occupied = intensity > self.intensity_threshold

                if is_occupied:
                    range_m = r_idx * range_resolution

                    # Check both min and max range
                    if range_m < self.min_range:
                        continue
                    if range_m > self.max_range:
                        break

                    # Calculate vertical spread and sample count
                    # Dense sampling (1.5x voxel) for occupied - higher precision for surfaces
                    vertical_spread = range_m * np.tan(half_aperture)
                    num_vertical = max(1, int(vertical_spread / (self.voxel_resolution * 1.5)))

                    for v_step in range(-num_vertical, num_vertical + 1):
                        # Normalize to [-1, 1] for exact aperture coverage: ±half_aperture
                        vertical_angle = (v_step / num_vertical) * half_aperture

                        # Sonar coordinates - see free space section for coordinate system details
                        x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                        y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                        z_sonar = range_m * np.sin(vertical_angle)

                        # Transform to world
                        pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                        pt_world = T_sonar_to_world @ pt_sonar

                        # Add to regular map updates with intensity value
                        updates.append((pt_world[:3], self.log_odds_occupied, 'occupied', float(intensity)))

        return updates
    
    def _collect_first_hits(self, polar_image: np.ndarray, bearing_step: int,
                            range_resolution: float) -> List[Tuple[float, float]]:
        """
        Collect first hit information for shadow region calculation.

        Args:
            polar_image: 2D sonar image (range_bins x bearing_bins)
            bearing_step: Step size for bearing iteration
            range_resolution: Range resolution in meters per bin

        Returns:
            Sorted list of (bearing_angle, first_hit_range) tuples
        """
        bearing_first_hits = []
        bearing_bins = polar_image.shape[1]

        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            intensity_profile = polar_image[:, b_idx]
            first_hit_idx = self._first_hit_index(intensity_profile, range_resolution)
            if first_hit_idx >= 0:
                bearing_first_hits.append((bearing_angle, first_hit_idx * range_resolution))

        bearing_first_hits.sort(key=lambda x: x[0])
        return bearing_first_hits

    def _process_rays_with_shadow(self, polar_image: np.ndarray, bearing_step: int,
                                   T_sonar_to_world: np.ndarray, sonar_origin_world: np.ndarray,
                                   bearing_first_hits: List[Tuple[float, float]],
                                   depth_filter_mask: np.ndarray = None) -> Dict:
        """
        Process rays and accumulate voxel updates with shadow checking.

        Args:
            polar_image: 2D sonar image
            bearing_step: Step size for bearing iteration
            T_sonar_to_world: Transform matrix from sonar to world
            sonar_origin_world: Sonar origin in world coordinates
            bearing_first_hits: Sorted list of first hits for shadow check
            depth_filter_mask: Boolean array per bearing (True=process, False=skip)

        Returns:
            Dictionary of voxel updates keyed by voxel index
        """
        voxel_updates = defaultdict(lambda: {'sum': 0.0, 'count': 0, 'type': 'unknown', 'intensity': None})
        bearing_bins = polar_image.shape[1]

        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            # Depth estimation filter: skip bearings matching existing environment
            if depth_filter_mask is not None and not depth_filter_mask[b_idx]:
                continue

            intensity_profile = polar_image[:, b_idx]
            ray_updates = self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world)

            for point, log_odds, update_type, intensity in ray_updates:
                key = self.world_to_key(point[0], point[1], point[2])

                # Shadow check: skip free updates in shadow regions
                if update_type == 'free':
                    voxel_range = np.sqrt((point[0] - sonar_origin_world[0])**2 +
                                          (point[1] - sonar_origin_world[1])**2)
                    if self.is_in_shadow_region(voxel_range, bearing_angle, bearing_first_hits):
                        continue

                if voxel_updates[key]['type'] != 'occupied':
                    voxel_updates[key]['type'] = update_type
                    if intensity is not None:
                        voxel_updates[key]['intensity'] = intensity
                voxel_updates[key]['sum'] += log_odds
                voxel_updates[key]['count'] += 1

        return voxel_updates

    def _apply_updates_to_octree(self, voxel_updates: Dict) -> Tuple[int, int]:
        """
        Apply accumulated voxel updates to the octree backend.

        Args:
            voxel_updates: Dictionary of voxel updates

        Returns:
            Tuple of (num_occupied, num_free) counts
        """
        points_list = []
        intensities_list = []
        is_occupied_list = []
        num_occupied = 0
        num_free = 0

        for key, update_info in voxel_updates.items():
            if update_info['count'] > 0:
                world_point = self.key_to_world(key)
                points_list.append(world_point)

                intensity_val = update_info.get('intensity', 0.0)
                intensities_list.append(intensity_val if intensity_val else 0.0)

                is_occupied = update_info['type'] == 'occupied'
                is_occupied_list.append(is_occupied)

                if is_occupied:
                    num_occupied += 1
                else:
                    num_free += 1

        if points_list:
            points_array = np.array(points_list, dtype=np.float64)
            intensities_array = np.array(intensities_list, dtype=np.float64)
            is_occupied_array = np.array(is_occupied_list, dtype=bool)
            self.octree.batch_update_iwlo(points_array, intensities_array, is_occupied_array)

        return num_occupied, num_free

    def process_sonar_image(self, polar_image: np.ndarray,
                           robot_position: List[float],
                           robot_orientation: List[float]) -> Dict[str, Any]:
        """
        Process sonar image and update probabilistic map.

        Args:
            polar_image: 2D numpy array (height x width) with intensity values
            robot_position: [x, y, z] position from odometry
            robot_orientation: [x, y, z, w] quaternion from odometry

        Returns:
            Processing statistics dictionary
        """
        self.frame_count += 1
        start_time = time.time()
        self.processed_frame_count += 1

        if not isinstance(polar_image, np.ndarray):
            polar_image = np.array(polar_image)

        range_bins, bearing_bins = polar_image.shape

        # Update bearing angles if needed
        if self.bearing_angles is None or bearing_bins != self.image_width:
            self.bearing_angles = np.linspace(
                -self.horizontal_fov/2, self.horizontal_fov/2, bearing_bins
            )
            self.image_width = bearing_bins

        if self.image_height is None or range_bins != self.image_height:
            self.image_height = range_bins

        # Create transforms
        T_base_to_world = self.create_odometry_transform(robot_position, robot_orientation)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base
        sonar_origin_world = T_sonar_to_world[:3, 3]

        # Processing parameters
        bearing_step = max(1, bearing_bins // 256)
        range_resolution = self.max_range / range_bins

        # Phase 1: Collect first hits
        bearing_first_hits = self._collect_first_hits(polar_image, bearing_step, range_resolution)

        # Phase 1.5: Depth estimation (reference map comparison)
        depth_estimation_result = self.compute_depth_estimation(
            polar_image, bearing_step, T_sonar_to_world, sonar_origin_world, range_resolution
        )

        # Phase 2: Process rays with shadow-aware updates + depth filter
        depth_filter_mask = None
        if depth_estimation_result is not None:
            depth_filter_mask = depth_estimation_result.get('bearing_mask')
        voxel_updates = self._process_rays_with_shadow(
            polar_image, bearing_step, T_sonar_to_world, sonar_origin_world,
            bearing_first_hits, depth_filter_mask
        )

        # Collect current frame's occupied voxel positions (before octree update)
        current_occupied_points = []
        for key, update_info in voxel_updates.items():
            if update_info['count'] > 0 and update_info['type'] == 'occupied':
                current_occupied_points.append(self.key_to_world(key))

        # Phase 3: Apply updates to octree
        num_occupied, num_free = self._apply_updates_to_octree(voxel_updates)

        # Calculate processing time
        processing_time = time.time() - start_time
        self.last_processing_time = processing_time
        self.total_processing_time += processing_time

        return {
            'frame_count': self.frame_count,
            'processed_count': self.processed_frame_count,
            'num_occupied': num_occupied,
            'num_free': num_free,
            'num_voxels': self.octree.get_num_nodes(),
            'processing_time': processing_time,
            'avg_processing_time': self.total_processing_time / max(1, self.processed_frame_count),
            'occupied_points': np.array(current_occupied_points) if current_occupied_points else np.empty((0, 3)),
            'depth_estimation': depth_estimation_result,
        }
    
    def get_point_cloud(self, include_free: bool = False) -> Dict[str, Any]:
        """
        Get current point cloud from probabilistic map

        Args:
            include_free: Whether to include free space voxels (not supported - ignored)

        Returns:
            Dictionary containing point cloud data and statistics
        """
        if include_free:
            warnings.warn("Free space voxel retrieval is not supported. Only occupied voxels returned.")

        # Query occupied voxels from C++
        occupied_data = self.octree.get_occupied_voxels(self.occupied_threshold)

        if len(occupied_data) > 0:
            points = occupied_data[:, :3]  # x, y, z
            probabilities = occupied_data[:, 3]  # probability
        else:
            points = np.empty((0, 3))
            probabilities = np.empty(0)

        # Memory usage statistics
        memory_stats = self.octree.get_memory_usage()

        return {
            'points': points,
            'probabilities': probabilities,
            'num_voxels': memory_stats.num_nodes,
            'num_occupied': len(points),
            'frame_count': self.frame_count,
            'processed_count': self.processed_frame_count,
            'memory_mb': memory_stats.memory_mb,
            'memory_efficiency': memory_stats.memory_efficiency,
        }

    def reset_map(self):
        """Reset the probabilistic map"""
        self.octree.clear()
        self.frame_count = 0
        self.processed_frame_count = 0
        self.total_processing_time = 0.0

    def get_memory_stats(self) -> Dict[str, Any]:
        """Get detailed memory usage statistics"""
        # C++ memory statistics
        stats = self.octree.get_memory_usage()

        if self.use_outofcore:
            return {
                'backend': 'C++ OutofcoreTileMapper (Disk)',
                'num_nodes': stats.num_nodes,
                'num_leaf_nodes': stats.num_leaf_nodes,
                'memory_mb': stats.memory_mb,
                'memory_efficiency': stats.memory_efficiency,
                'resolution': self.octree.get_resolution(),
                'cached_tiles': self.octree.get_cached_tile_count(),
                'total_tiles': self.octree.get_total_tile_count(),
                'disk_usage_bytes': self.octree.get_disk_usage()
            }
        else:
            return {
                'backend': 'C++ ProbabilityUpdater (RAM)',
                'num_nodes': stats.num_nodes,
                'num_leaf_nodes': stats.num_leaf_nodes,
                'memory_mb': stats.memory_mb,
                'memory_efficiency': stats.memory_efficiency,
                'resolution': self.octree.get_resolution()
            }

    def flush_map(self):
        """Flush all dirty tiles to disk (only for OutofcoreTileMapper)"""
        if self.use_outofcore:
            self.octree.flush_all()

    def flush_map_and_get_dirty_tiles(self):
        """
        Flush dirty tiles and return their indices (for visualization sync)

        Returns:
            List of TileIndex objects that were flushed, or empty list if not outofcore mode
        """
        if self.use_outofcore:
            return self.octree.flush_and_get_dirty_tiles()
        return []

    def get_and_clear_saved_tiles(self):
        """
        Get tiles saved via LRU eviction and clear the list (for visualizer sync)

        Returns:
            List of TileIndex objects that were saved since last call
        """
        if self.use_outofcore:
            return self.octree.get_and_clear_saved_tiles()
        return []

    def save_merged_octree(self, filepath: str) -> bool:
        """
        Save merged octree to .bt file (only for OutofcoreTileMapper)

        Args:
            filepath: Output file path (.bt format)

        Returns:
            True if successful
        """
        if self.use_outofcore:
            return self.octree.save_merged_octree(filepath)
        return False
    
    def prune_map(self) -> int:
        """
        Prune unnecessary nodes from the map (merge homogeneous octree nodes)

        Returns:
            Number of nodes removed
        """
        if self.use_outofcore:
            return self.octree.prune_all()
        return self.octree.prune_tree()


if __name__ == "__main__":
    """
    3D Mapper basic test
    """
    print("3D Mapper basic test")

    # Basic configuration
    config = {
        'voxel_resolution': 0.1,
        'occupied_threshold': 0.6,
        'intensity_threshold': 30,
    }

    # Initialize mapper
    print(f"\nMapper init - C++ module: {'available' if CPP_MODULE_AVAILABLE else 'unavailable'}")
    mapper = SonarTo3DMapper(config)

    # Create test image
    test_image = np.zeros((500, 256), dtype=np.uint8)
    test_image[100:150, 120:140] = 80  # Object

    # Frame processing test
    print("\nFrame processing test...")
    stats = mapper.process_sonar_image(test_image, [0, 0, 0], [0, 0, 0, 1])
    print(f"Result: {stats['num_occupied']} occupied voxels, {stats['processing_time']:.3f}s")

    # Query point cloud
    point_cloud = mapper.get_point_cloud()
    print(f"Point cloud: {point_cloud['num_occupied']} points")

    # Memory statistics
    memory_stats = mapper.get_memory_stats()
    print(f"Memory usage: {memory_stats}")

    print("\nTest completed")