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

# Try to import C++ module
try:
    # Import module directly from ROS2 install path
    import sys
    import importlib.util

    install_path = "/workspace/ros2_ws/install/sonar_3d_reconstruction/local/lib/python3.10/dist-packages"
    cpp_file = f"{install_path}/sonar_3d_reconstruction/sonar_3d_reconstruction_cpp.cpython-310-x86_64-linux-gnu.so"

    # Direct loading method
    spec = importlib.util.spec_from_file_location("sonar_3d_reconstruction_cpp", cpp_file)
    cpp_module = importlib.util.module_from_spec(spec)
    sys.modules["sonar_3d_reconstruction_cpp"] = cpp_module
    spec.loader.exec_module(cpp_module)

    ProbabilityUpdater = cpp_module.ProbabilityUpdater
    MemoryStats = cpp_module.MemoryStats
    OutofcoreTileMapper = cpp_module.OutofcoreTileMapper

    CPP_MODULE_AVAILABLE = True
    OUTOFCORE_AVAILABLE = True
except Exception as e:
    CPP_MODULE_AVAILABLE = False
    OUTOFCORE_AVAILABLE = False
    warnings.warn(f"[3D Mapper] C++ module unavailable, using Python fallback: {e}")


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

        if self.use_outofcore and OUTOFCORE_AVAILABLE:
            # Initialize OutofcoreTileMapper (disk-based)
            map_path = default_config.get('outofcore_map_path', '/workspace/data/map_tiles')
            tile_size = default_config.get('outofcore_tile_size', 10.0)
            cache_size = default_config.get('outofcore_cache_size', 16)

            self.octree = OutofcoreTileMapper(
                map_path,
                self.voxel_resolution,
                tile_size,
                cache_size
            )

            # Configure IWLO parameters
            self.octree.set_iwlo_params(
                default_config.get('sharpness', 0.1),
                default_config.get('decay_rate', 0.1),
                default_config.get('min_alpha', 0.3),
                default_config.get('L_min', -10.0),
                default_config.get('L_max', 10.0)
            )
            self.octree.set_log_odds_params(
                self.log_odds_occupied,
                self.log_odds_free
            )
            self.octree.set_intensity_params(
                self.intensity_threshold,
                default_config.get('intensity_max', 255)
            )
            self.octree.set_adaptive_params(
                default_config['adaptive_update'],
                default_config['adaptive_threshold'],
                default_config['adaptive_max_ratio']
            )

        elif self.use_cpp and CPP_MODULE_AVAILABLE:
            # Initialize C++ ProbabilityUpdater (RAM-based)
            self.octree = ProbabilityUpdater(self.voxel_resolution)

            # Configure C++ octree parameters
            self.octree.set_log_odds_params(
                self.log_odds_occupied,
                self.log_odds_free
            )
            self.octree.set_adaptive_params(
                default_config['adaptive_update'],
                default_config['adaptive_threshold'],
                default_config['adaptive_max_ratio']
            )

            # Set clamping thresholds based on log-odds
            min_prob = 1.0 / (1.0 + np.exp(-default_config['L_min']))
            max_prob = 1.0 / (1.0 + np.exp(-default_config['L_max']))
            self.octree.set_clamping_thresholds(min_prob, max_prob)

            # Configure IWLO parameters
            self.octree.set_iwlo_params(
                default_config.get('sharpness', 0.1),
                default_config.get('decay_rate', 0.1),
                default_config.get('min_alpha', 0.3),
                default_config.get('L_min', -10.0),
                default_config.get('L_max', 10.0)
            )
            self.octree.set_intensity_params(
                self.intensity_threshold,
                default_config.get('intensity_max', 255)
            )
        else:
            # C++ module required but not available
            raise RuntimeError(
                "C++ backend is required but not available. "
                "Please build the C++ module: colcon build --packages-select sonar_3d_reconstruction"
            )
        
        # Bearing angles - initialized dynamically in process_sonar_image
        self.bearing_angles = None
        
        # Frame counter
        self.frame_count = 0
        self.processed_frame_count = 0

        # Processing statistics
        self.last_processing_time = 0.0
        self.total_processing_time = 0.0

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

    def update_visualization(self, value: bool) -> None:
        """Update visualization flag (handled by node)"""
        pass  # Node-level parameter

    def update_dynamic_expansion(self, value: bool) -> None:
        """Update dynamic expansion flag"""
        self.dynamic_expansion = bool(value)

    def update_orientation(self, value: float) -> None:
        """
        Update sonar orientation (roll/pitch/yaw)
        Note: Requires node-level coordination for TF update
        """
        pass  # Node handles this with update_sonar_orientation()

    def is_bearing_in_valid_fov(self, bearing_angle: float) -> bool:
        """Check if bearing angle is within valid FOV"""
        half_fov = self.horizontal_fov / 2
        return abs(bearing_angle) <= half_fov

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
        first_hit_idx = -1
        range_resolution = self.max_range / len(intensity_profile)
        
        for r_idx, intensity in enumerate(intensity_profile):
            range_m = r_idx * range_resolution
            if intensity > self.intensity_threshold and range_m >= self.min_range:
                first_hit_idx = r_idx
                break
        
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
    
    def process_sonar_image(self, polar_image: np.ndarray,
                           robot_position: List[float],
                           robot_orientation: List[float]) -> Dict[str, Any]:
        """
        Process sonar image and update probabilistic map

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

        # Ensure image is numpy array
        if not isinstance(polar_image, np.ndarray):
            polar_image = np.array(polar_image)

        # Get image dimensions dynamically from image shape
        range_bins, bearing_bins = polar_image.shape

        # Update bearing angles if first frame or dimension changed
        # Bearing angle convention for Oculus M750D polar image:
        #   - Column 0 = left side of FOV = -horizontal_fov/2 (negative angle)
        #   - Column N-1 = right side of FOV = +horizontal_fov/2 (positive angle)
        #   - Positive bearing = clockwise from forward (starboard/right)
        #   - Negative bearing = counter-clockwise from forward (port/left)
        if self.bearing_angles is None or bearing_bins != self.image_width:
            self.bearing_angles = np.linspace(
                -self.horizontal_fov/2,
                self.horizontal_fov/2,
                bearing_bins
            )
            self.image_width = bearing_bins

        # Update image height if first frame or range changed
        if self.image_height is None or range_bins != self.image_height:
            self.image_height = range_bins

        # Create transformation matrices
        T_base_to_world = self.create_odometry_transform(robot_position, robot_orientation)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base

        # Extract sonar origin in world coordinates for shadow range calculation
        sonar_origin_world = T_sonar_to_world[:3, 3]

        # Accumulate updates per voxel
        voxel_updates = defaultdict(lambda: {'sum': 0.0, 'count': 0, 'type': 'unknown', 'intensity': None})

        # Process subset of bearings for efficiency
        bearing_step = max(1, bearing_bins // 256)
        range_resolution = self.max_range / range_bins

        # Phase 1: Collect first hit information (sorted list for O(log N) lookup)
        bearing_first_hits = []
        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            intensity_profile = polar_image[:, b_idx]
            first_hit_idx = -1
            for r_idx, intensity in enumerate(intensity_profile):
                range_m = r_idx * range_resolution
                if intensity > self.intensity_threshold and range_m >= self.min_range:
                    first_hit_idx = r_idx
                    break

            if first_hit_idx >= 0:
                bearing_first_hits.append((bearing_angle, first_hit_idx * range_resolution))

        # Sort for binary search (already sorted due to iteration order, but explicit)
        bearing_first_hits.sort(key=lambda x: x[0])

        # Phase 2: Process rays with shadow-aware updates
        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]

            # Skip bearings outside valid FOV
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            # Process this ray
            intensity_profile = polar_image[:, b_idx]
            ray_updates = self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world)

            # Accumulate updates with shadow check
            for point, log_odds, update_type, intensity in ray_updates:
                key = self.world_to_key(point[0], point[1], point[2])

                # Shadow check: skip free updates in shadow regions
                if update_type == 'free':
                    # Calculate range from sonar origin (not world origin) for correct shadow comparison
                    # bearing_first_hits stores ranges in sonar coordinate system
                    voxel_range = np.sqrt((point[0] - sonar_origin_world[0])**2 +
                                          (point[1] - sonar_origin_world[1])**2)
                    if self.is_in_shadow_region(voxel_range, bearing_angle, bearing_first_hits):
                        continue  # Skip - preserve unknown state in shadow

                if voxel_updates[key]['type'] != 'occupied':  # Occupied has priority
                    voxel_updates[key]['type'] = update_type
                    # Store intensity for weighted average method
                    if intensity is not None:
                        voxel_updates[key]['intensity'] = intensity
                voxel_updates[key]['sum'] += log_odds
                voxel_updates[key]['count'] += 1
        
        # Apply averaged updates to octree
        num_occupied = 0
        num_free = 0
        
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            # Use C++ batch update
            points_list = []
            log_odds_list = []
            intensities_list = []  # Intensity array for IWLO
            is_occupied_list = []

            for key, update_info in voxel_updates.items():
                if update_info['count'] > 0:
                    avg_update = update_info['sum'] / update_info['count']

                    # Convert key to world coordinates
                    world_point = self.key_to_world(key)
                    points_list.append(world_point)
                    log_odds_list.append(avg_update)

                    # Collect intensity for IWLO (default to 0 if not present)
                    intensity_val = update_info.get('intensity', 0.0)
                    intensities_list.append(intensity_val if intensity_val else 0.0)

                    is_occupied = update_info['type'] == 'occupied'
                    is_occupied_list.append(is_occupied)

                    if is_occupied:
                        num_occupied += 1
                    else:
                        num_free += 1

            # Convert to NumPy arrays for batch update
            if points_list:
                points_array = np.array(points_list, dtype=np.float64)
                log_odds_array = np.array(log_odds_list, dtype=np.float64)
                intensities_array = np.array(intensities_list, dtype=np.float64)
                is_occupied_array = np.array(is_occupied_list, dtype=bool)

                # Execute C++ batch update (IWLO only)
                self.octree.batch_update_iwlo(
                    points_array, intensities_array, is_occupied_array
                )
        
        # Calculate processing time
        processing_time = time.time() - start_time
        self.last_processing_time = processing_time
        self.total_processing_time += processing_time
        
        # Get voxel count
        num_voxels = self.octree.get_num_nodes()
        
        return {
            'frame_count': self.frame_count,
            'processed_count': self.processed_frame_count,
            'num_occupied': num_occupied,
            'num_free': num_free,
            'num_voxels': num_voxels,
            'processing_time': processing_time,
            'avg_processing_time': self.total_processing_time / max(1, self.processed_frame_count)
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