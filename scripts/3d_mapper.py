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

# C++ 모듈 임포트 시도
try:
    # ROS2 install 경로에서 직접 모듈 임포트
    import sys
    import importlib.util
    
    install_path = "/workspace/ros2_ws/install/sonar_3d_reconstruction/local/lib/python3.10/dist-packages"
    cpp_file = f"{install_path}/sonar_3d_reconstruction/sonar_3d_reconstruction_cpp.cpython-310-x86_64-linux-gnu.so"
    
    # 직접 로드 방식
    spec = importlib.util.spec_from_file_location("sonar_3d_reconstruction_cpp", cpp_file)
    cpp_module = importlib.util.module_from_spec(spec)
    sys.modules["sonar_3d_reconstruction_cpp"] = cpp_module
    spec.loader.exec_module(cpp_module)
    
    ProbabilityUpdater = cpp_module.ProbabilityUpdater
    MemoryStats = cpp_module.MemoryStats
    
    CPP_MODULE_AVAILABLE = True
    print("[3D Mapper] C++ ProbabilityUpdater 모듈 로드 성공")
except Exception as e:
    CPP_MODULE_AVAILABLE = False
    print("[3D Mapper] C++ 모듈 없음 - Python SimpleOctree 사용")
    print(f"  Error: {e}")


class SimpleOctree:
    """
    Sparse voxel storage using dictionary with dynamic expansion
    Stores log-odds values for each voxel with adaptive updating
    """

    def __init__(self, resolution: float = 0.03, dynamic_expansion: bool = True,
                 probability_update_method: str = 'log_odds', intensity_max: int = 255):
        """
        Initialize octree with given resolution

        Args:
            resolution: Size of each voxel in meters
            dynamic_expansion: Enable dynamic map expansion
            probability_update_method: 'log_odds' or 'weighted_average'
            intensity_max: Maximum intensity value for normalization (default 255)
        """
        self.resolution = resolution
        self.voxels = defaultdict(float)  # Store log-odds values
        self.observation_counts = defaultdict(int)  # Track observation counts per voxel
        self.dynamic_expansion = dynamic_expansion

        # Probability update method
        self.probability_update_method = probability_update_method
        self.intensity_max = intensity_max

        # Map bounds (for dynamic expansion)
        self.min_bounds = np.array([float('inf')] * 3)
        self.max_bounds = np.array([-float('inf')] * 3)

        # Log-odds parameters (will be set from config)
        self.log_odds_occupied = 1.5      # Log-odds increment for occupied
        self.log_odds_free = -2.0         # Log-odds decrement for free space
        self.log_odds_min = -10.0         # Minimum log-odds (clamping)
        self.log_odds_max = 10.0          # Maximum log-odds (clamping)
        self.log_odds_threshold = 0.0     # Threshold for considering occupied

        # Probability threshold (2-class classification)
        self.occupied_threshold = 0.7     # Probability threshold for occupied (< this = free)

        # Convert threshold to log-odds
        self.log_odds_occupied_thresh = np.log(self.occupied_threshold / (1.0 - self.occupied_threshold))

        # Adaptive update parameters
        self.adaptive_update = True       # Enable adaptive updating
        self.adaptive_threshold = 0.5     # Protection threshold
        self.adaptive_max_ratio = 0.5     # Maximum update ratio at threshold

        # IWLO (Intensity-Weighted Log-Odds) parameters
        self.sharpness = 3.0              # Sigmoid steepness for intensity-to-weight
        self.decay_rate = 0.1             # Learning rate decay rate
        self.min_alpha = 0.1              # Minimum learning rate for change detection
        self.L_min = -2.0                 # Saturation lower bound (P ~ 0.12)
        self.L_max = 3.5                  # Saturation upper bound (P ~ 0.97)
    
    def world_to_key(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        """
        Convert world coordinates to voxel key
        
        Args:
            x, y, z: World coordinates in meters
            
        Returns:
            Tuple (ix, iy, iz) as voxel index
        """
        i = int(np.floor(x / self.resolution))
        j = int(np.floor(y / self.resolution))
        k = int(np.floor(z / self.resolution))
        return (i, j, k)
    
    def key_to_world(self, key: Tuple[int, int, int]) -> np.ndarray:
        """
        Convert voxel key to world coordinates (center of voxel)
        
        Args:
            key: Tuple (ix, iy, iz) voxel index
            
        Returns:
            numpy array [x, y, z] world coordinates
        """
        x = (key[0] + 0.5) * self.resolution
        y = (key[1] + 0.5) * self.resolution
        z = (key[2] + 0.5) * self.resolution
        return np.array([x, y, z])
    
    def update_voxel(self, point: np.ndarray, log_odds_update: float, adaptive: bool = True,
                     intensity: Optional[float] = None):
        """
        Update voxel log-odds value with optional adaptive updating

        Args:
            point: [x, y, z] numpy array in world coordinates
            log_odds_update: Log-odds increment/decrement (ignored for weighted_average method)
            adaptive: If True, use adaptive updating for occupied updates
            intensity: Observed intensity value (for weighted_average method)
        """
        key = self.world_to_key(point[0], point[1], point[2])

        # Get current values
        old_log_odds = self.voxels.get(key, 0.0)
        old_prob = 1.0 / (1.0 + np.exp(-old_log_odds))
        n = self.observation_counts.get(key, 0)

        # Use weighted average method if enabled and intensity provided
        if self.probability_update_method == 'weighted_average' and intensity is not None:
            # Convert intensity to probability
            threshold = self.intensity_threshold if hasattr(self, 'intensity_threshold') else 35
            intensity_clamped = np.clip(intensity, threshold, self.intensity_max)

            # Normalize: [threshold, intensity_max] -> [0.7, 0.95]
            # More aggressive mapping: threshold intensity -> 0.7 prob, max intensity -> 0.95 prob
            normalized_ratio = (intensity_clamped - threshold) / (self.intensity_max - threshold)
            obs_prob = 0.7 + 0.25 * normalized_ratio

            # Weighted average: new_prob = (n * old_prob + obs_prob) / (n + 1)
            new_prob = (n * old_prob + obs_prob) / (n + 1)

            # Convert back to log-odds
            if new_prob >= 0.9999:
                new_log_odds = self.log_odds_max
            elif new_prob <= 0.0001:
                new_log_odds = self.log_odds_min
            else:
                new_log_odds = np.log(new_prob / (1.0 - new_prob))

            # Set the new value (not increment)
            self.voxels[key] = np.clip(new_log_odds, self.log_odds_min, self.log_odds_max)

        elif self.probability_update_method == 'iwlo' and intensity is not None:
            # IWLO: Intensity-Weighted Log-Odds
            # Combines Log-Odds Bayesian with Weighted Average approach
            threshold = self.intensity_threshold if hasattr(self, 'intensity_threshold') else 35

            # 1. Compute intensity-based weight using sigmoid
            w_intensity = self._intensity_to_weight(intensity, threshold)

            # 2. Compute learning rate based on observation count
            alpha_n = self._compute_alpha(n)

            # 3. Compute adaptive scaling (for occupied updates)
            if w_intensity > 0 and old_prob < self.adaptive_threshold:
                adapt_scale = (old_prob / self.adaptive_threshold) * self.adaptive_max_ratio
            else:
                adapt_scale = 1.0

            # 4. Compute log-odds update
            if intensity > threshold:
                # Occupied update: ΔL = L_occ × w(I) × α(n) × scale
                delta_L = self.log_odds_occupied * w_intensity * alpha_n * adapt_scale
            else:
                # Free space update: ΔL = L_free × α(n)
                delta_L = self.log_odds_free * alpha_n

            # 5. Apply update with saturation limits
            if key not in self.voxels:
                self.voxels[key] = 0.0
            self.voxels[key] += delta_L
            self.voxels[key] = np.clip(self.voxels[key], self.L_min, self.L_max)

        else:
            # Standard log-odds update (additive)
            # Adaptive update: reduce occupied updates for voxels that are likely free
            if adaptive and self.adaptive_update and log_odds_update > 0:
                current_prob = 1.0 / (1.0 + np.exp(-old_log_odds))

                # Linear interpolation for adaptive update
                if current_prob <= self.adaptive_threshold:
                    update_scale = (current_prob / self.adaptive_threshold) * self.adaptive_max_ratio
                    log_odds_update *= update_scale

            # Apply update
            if key not in self.voxels:
                self.voxels[key] = 0.0
            self.voxels[key] += log_odds_update

            # Clamp to prevent overflow
            self.voxels[key] = np.clip(self.voxels[key], self.log_odds_min, self.log_odds_max)

        # Increment observation count after update
        self.observation_counts[key] = n + 1

        # Update bounds for dynamic expansion
        if self.dynamic_expansion:
            self.min_bounds = np.minimum(self.min_bounds, point)
            self.max_bounds = np.maximum(self.max_bounds, point)
    
    def calculate_weighted_average_update(self, intensity: float, observation_count: int) -> float:
        """
        Calculate weighted average log-odds update based on intensity and observation count
        Based on voxelmap_fusion approach (Voxel.py:71-72)

        Args:
            intensity: Observed intensity value (threshold ~ intensity_max range)
            observation_count: Number of times this voxel has been observed

        Returns:
            Log-odds update value
        """
        # Normalize intensity to probability range [0.5, 1.0]
        # intensity_threshold is stored as self.intensity_threshold in parent mapper
        threshold = self.intensity_threshold if hasattr(self, 'intensity_threshold') else 35

        # Clamp intensity to valid range
        intensity = np.clip(intensity, threshold, self.intensity_max)

        # Normalize: [threshold, intensity_max] -> [0.5, 1.0]
        normalized_prob = 0.5 + 0.5 * (intensity - threshold) / (self.intensity_max - threshold)

        # Calculate weight: decreases with more observations
        # weight = 1/(n+1) means first observation has weight 1, second has 1/2, etc.
        weight = 1.0 / observation_count

        # Convert normalized probability to log-odds
        if normalized_prob >= 1.0:
            normalized_log_odds = self.log_odds_occupied
        elif normalized_prob <= 0.0:
            normalized_log_odds = self.log_odds_free
        else:
            normalized_log_odds = np.log(normalized_prob / (1.0 - normalized_prob))

        # Apply weight to the update
        log_odds_update = normalized_log_odds * weight

        return log_odds_update

    def _intensity_to_weight(self, intensity: float, threshold: float) -> float:
        """
        Convert intensity to weight using sigmoid transformation (IWLO method)

        Args:
            intensity: Observed intensity value (0~255)
            threshold: Minimum intensity threshold

        Returns:
            Weight in range [0, 1]
        """
        if intensity <= threshold:
            return 0.0

        # Normalize: [threshold, intensity_max] -> [0, 1]
        normalized = (intensity - threshold) / (self.intensity_max - threshold)
        normalized = np.clip(normalized, 0.0, 1.0)

        # Sigmoid transformation centered at 0.5
        x = self.sharpness * (normalized - 0.5)
        return 1.0 / (1.0 + np.exp(-x))

    def _compute_alpha(self, observation_count: int) -> float:
        """
        Compute learning rate based on observation count (IWLO method)

        Args:
            observation_count: Number of times this voxel has been observed

        Returns:
            Learning rate alpha in range [min_alpha, 1.0]
        """
        return max(self.min_alpha, 1.0 / (1.0 + self.decay_rate * observation_count))

    def _classify_state(self, log_odds: float) -> str:
        """
        Classify voxel state based on log-odds value

        Args:
            log_odds: Log-odds value

        Returns:
            'free', 'unknown', or 'occupied'
        """
        if log_odds < self.log_odds_free_thresh:
            return 'free'
        elif log_odds > self.log_odds_occupied_thresh:
            return 'occupied'
        else:
            return 'unknown'

    def update_voxel_with_state_tracking(self, point: np.ndarray, log_odds_update: float,
                                         adaptive: bool = True, intensity: Optional[float] = None) -> str:
        """
        Update voxel with state change tracking

        Args:
            point: [x, y, z] numpy array in world coordinates
            log_odds_update: Log-odds increment/decrement
            adaptive: If True, use adaptive updating
            intensity: Observed intensity value

        Returns:
            State change: 'free->occupied', 'occupied->free', or 'no_change'
        """
        key = self.world_to_key(point[0], point[1], point[2])

        # Get previous state
        old_log_odds = self.voxels.get(key, 0.0)
        old_state = self._classify_state(old_log_odds)

        # Update voxel
        self.update_voxel(point, log_odds_update, adaptive, intensity)

        # Get new state
        new_log_odds = self.voxels[key]
        new_state = self._classify_state(new_log_odds)

        # Detect state change
        if old_state != new_state:
            return f"{old_state}->{new_state}"
        else:
            return "no_change"

    def get_log_odds(self, x: float, y: float, z: float) -> float:
        """Get log-odds value for a voxel"""
        key = self.world_to_key(x, y, z)
        return self.voxels.get(key, 0.0)

    def get_probability(self, x: float, y: float, z: float) -> float:
        """Get probability from log-odds value"""
        log_odds = self.get_log_odds(x, y, z)
        return 1.0 / (1.0 + np.exp(-log_odds))
    
    def get_occupied_voxels(self, min_probability: float = 0.5) -> List[Tuple[np.ndarray, float]]:
        """
        Get all occupied voxels above probability threshold
        
        Args:
            min_probability: Minimum probability to consider occupied
            
        Returns:
            List of (point, probability) tuples
        """
        occupied = []
        
        # Convert probability to log-odds threshold
        if min_probability >= 1.0:
            min_log_odds = self.log_odds_max - 0.01
        elif min_probability <= 0.0:
            min_log_odds = self.log_odds_min
        else:
            min_log_odds = np.log(min_probability / (1.0 - min_probability))
        
        for key, log_odds in self.voxels.items():
            if log_odds > min_log_odds:
                point = self.key_to_world(key)
                probability = 1.0 / (1.0 + np.exp(-log_odds))
                occupied.append((point, probability))
        
        return occupied
    
    def get_all_voxels_classified(self, occupied_threshold: Optional[float] = None) -> Dict[str, List]:
        """
        Get all voxels classified as free or occupied (2-class)

        Args:
            occupied_threshold: Probability threshold for occupied (uses self.occupied_threshold if None)

        Returns:
            Dictionary with 'free', 'occupied' lists
        """
        free = []
        occupied = []

        # Use instance threshold if not provided
        if occupied_threshold is None:
            occupied_threshold = self.occupied_threshold

        # Convert probability threshold to log-odds
        log_odds_occupied_thresh = np.log(occupied_threshold / (1.0 - occupied_threshold))

        for key, log_odds in self.voxels.items():
            point = self.key_to_world(key)
            probability = 1.0 / (1.0 + np.exp(-log_odds))

            if log_odds >= log_odds_occupied_thresh:
                occupied.append((point, probability))
            else:
                free.append((point, probability))

        return {
            'free': free,
            'occupied': occupied
        }
    
    def clear(self):
        """Clear all voxels and observation counts"""
        self.voxels.clear()
        self.observation_counts.clear()
        self.min_bounds = np.array([float('inf')] * 3)
        self.max_bounds = np.array([-float('inf')] * 3)


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
            'max_range': 10.0,             # meters
            'min_range': 0.5,              # meters
            'intensity_threshold': 35,     # 0-255 scale

            # Terrain detection parameters (for robot detection mode)
            'terrain_detection': {
                'min_threshold': 80,
                'max_threshold': 180
            },

            # Robot detection parameters
            'enable_robot_detection': False,
            'robot_detection': {
                'min_threshold': 180,
                'topic': '/sonar_robot_detections'
            },

            'image_width': 512,            # bearings
            'image_height': 500,           # ranges

            # Sonar mounting (relative to base_link)
            'sonar_position': [0.0, 0.0, -0.5],  # xyz
            'sonar_orientation': [0.0, 1.5708, 0.0],  # rpy (0, 90deg, 0)

            # Octree parameters
            'voxel_resolution': 0.05,      # meters
            'dynamic_expansion': True,

            # Probability update method
            'probability_update_method': 'log_odds',  # 'log_odds' or 'weighted_average'
            'intensity_max': 255,          # Maximum intensity value for normalization

            # Probability threshold (2-class: occupied vs free)
            'occupied_threshold': 0.7,     # prob >= this = occupied, < this = free

            # Adaptive update
            'adaptive_update': True,
            'adaptive_threshold': 0.5,
            'adaptive_max_ratio': 0.3,

            # Log-odds parameters
            'log_odds_occupied': 1.5,
            'log_odds_free': -2.0,
            'log_odds_min': -10.0,
            'log_odds_max': 10.0,

            # Processing parameters
            'frame_skip': 1,  # Process every N frames
        }
        
        # Update with provided config
        if config:
            default_config.update(config)
        
        # Store parameters
        self.horizontal_fov = np.radians(default_config['horizontal_fov'])
        self.vertical_aperture = np.radians(default_config['vertical_aperture'])
        self.max_range = default_config['max_range']
        self.min_range = default_config['min_range']
        self.intensity_threshold = default_config['intensity_threshold']
        
        # Terrain detection parameters  
        self.terrain_min_threshold = default_config['terrain_detection']['min_threshold']
        self.terrain_max_threshold = default_config['terrain_detection']['max_threshold']
        
        # Robot detection parameters
        self.enable_robot_detection = default_config['enable_robot_detection']
        self.robot_min_threshold = default_config['robot_detection']['min_threshold']
        
        # Processing parameters
        self.frame_skip = default_config['frame_skip']
        
        self.image_width = default_config['image_width']
        self.image_height = default_config['image_height']
        self.voxel_resolution = default_config['voxel_resolution']
        self.occupied_threshold = default_config['occupied_threshold']
        self.dynamic_expansion = default_config['dynamic_expansion']
        
        # Z-axis filtering
        self.z_filter_min = default_config.get('z_filter_min', -5.0)
        self.z_filter_enabled = default_config.get('z_filter_enabled', False)
        
        # Sonar mounting transform
        self.sonar_position = np.array(default_config['sonar_position'])
        self.sonar_orientation = np.array(default_config['sonar_orientation'])
        
        # Pre-compute sonar to base_link transform
        self.T_sonar_to_base = self.create_transform_matrix(
            self.sonar_position,
            self.sonar_orientation
        )
        
        # Initialize octree - Try C++ first, fallback to Python
        self.use_cpp = default_config.get('use_cpp_backend', CPP_MODULE_AVAILABLE)
        
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            # Initialize C++ ProbabilityUpdater
            self.octree = ProbabilityUpdater(self.voxel_resolution)

            # Store log-odds values for ray processing
            self.log_odds_occupied = default_config['log_odds_occupied']
            self.log_odds_free = default_config['log_odds_free']

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
            min_prob = 1.0 / (1.0 + np.exp(-default_config['log_odds_min']))
            max_prob = 1.0 / (1.0 + np.exp(-default_config['log_odds_max']))
            self.octree.set_clamping_thresholds(min_prob, max_prob)

            print(f"[3D Mapper] C++ ProbabilityUpdater 사용 (해상도: {self.voxel_resolution}m)")
        else:
            # Initialize Python SimpleOctree
            self.octree = SimpleOctree(
                self.voxel_resolution,
                self.dynamic_expansion,
                default_config['probability_update_method'],
                default_config['intensity_max']
            )

            # Store log-odds values for both approaches
            self.log_odds_occupied = default_config['log_odds_occupied']
            self.log_odds_free = default_config['log_odds_free']

            # Configure Python octree parameters
            self.octree.log_odds_occupied = self.log_odds_occupied
            self.octree.log_odds_free = self.log_odds_free
            self.octree.log_odds_min = default_config['log_odds_min']
            self.octree.log_odds_max = default_config['log_odds_max']
            self.octree.occupied_threshold = default_config['occupied_threshold']
            self.octree.adaptive_update = default_config['adaptive_update']
            self.octree.adaptive_threshold = default_config['adaptive_threshold']
            self.octree.adaptive_max_ratio = default_config['adaptive_max_ratio']

            # Store intensity threshold for weighted average method
            self.octree.intensity_threshold = self.intensity_threshold

            # IWLO parameters
            self.octree.sharpness = default_config.get('sharpness', 3.0)
            self.octree.decay_rate = default_config.get('decay_rate', 0.1)
            self.octree.min_alpha = default_config.get('min_alpha', 0.1)
            self.octree.L_min = default_config.get('L_min', -2.0)
            self.octree.L_max = default_config.get('L_max', 3.5)

            # Recalculate log-odds threshold
            self.octree.log_odds_occupied_thresh = np.log(
                default_config['occupied_threshold'] / (1.0 - default_config['occupied_threshold'])
            )

            print(f"[3D Mapper] Python SimpleOctree 사용 (해상도: {self.voxel_resolution}m)")
            print(f"  업데이트 방식: {default_config['probability_update_method']}")
            if self.use_cpp:
                print("  주의: C++ 모듈을 요청했지만 사용할 수 없음")
        
        # Pre-compute bearing angles
        self.bearing_angles = np.linspace(
            -self.horizontal_fov/2,
            self.horizontal_fov/2,
            self.image_width
        )
        
        # Frame counter
        self.frame_count = 0
        self.processed_frame_count = 0
        
        # Robot detection storage
        self.robot_detections = []  # List of (point, intensity, timestamp) tuples
        self.robot_detection_timeout = 10.0  # seconds
        
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
    
    def is_bearing_in_valid_fov(self, bearing_angle: float) -> bool:
        """Check if bearing angle is within valid FOV"""
        half_fov = self.horizontal_fov / 2
        return abs(bearing_angle) <= half_fov
    
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
        
        # Process free space before first hit (sparse)
        free_sampling_step = 10
        for r_idx in range(0, first_hit_idx, free_sampling_step):
            range_m = r_idx * range_resolution
            if range_m < self.min_range:
                continue
            
            # Calculate vertical spread
            vertical_spread = range_m * np.tan(half_aperture)
            num_vertical = max(1, int(vertical_spread / (self.voxel_resolution * 4)))
            
            for v_step in range(-num_vertical, num_vertical + 1):
                vertical_angle = (v_step / max(1, num_vertical)) * half_aperture
                
                # Sonar coordinates (X=forward, Y=right, Z=down)
                x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                z_sonar = range_m * np.sin(vertical_angle)
                
                # Transform to world
                pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                pt_world = T_sonar_to_world @ pt_sonar
                
                # Apply Z-axis filter if enabled
                if self.z_filter_enabled and pt_world[2] < self.z_filter_min:
                    continue

                updates.append((pt_world[:3], self.log_odds_free, 'free', None))
        
        # Process occupied regions (dense)
        if first_hit_idx < len(intensity_profile):
            # Find all high intensity regions
            for r_idx in range(first_hit_idx, min(first_hit_idx + 50, len(intensity_profile))):
                intensity = intensity_profile[r_idx]
                
                # Multi-threshold processing
                is_robot = (self.enable_robot_detection and intensity >= self.robot_min_threshold)
                is_terrain = (self.terrain_min_threshold <= intensity <= self.terrain_max_threshold if self.enable_robot_detection 
                             else intensity > self.intensity_threshold)
                
                if is_robot or is_terrain:
                    range_m = r_idx * range_resolution
                    
                    # Check both min and max range
                    if range_m < self.min_range:
                        continue
                    if range_m > self.max_range:
                        break
                    
                    # Calculate vertical spread
                    vertical_spread = range_m * np.tan(half_aperture)
                    num_vertical = max(2, int(vertical_spread / (self.voxel_resolution * 1.5)))
                    
                    for v_step in range(-num_vertical, num_vertical + 1):
                        vertical_angle = (v_step / max(1, num_vertical)) * half_aperture
                        
                        # Sonar coordinates (X=forward, Y=right, Z=down)
                        x_sonar = range_m * np.cos(vertical_angle) * np.cos(bearing_angle)
                        y_sonar = -range_m * np.cos(vertical_angle) * np.sin(bearing_angle)
                        z_sonar = range_m * np.sin(vertical_angle)
                        
                        # Transform to world
                        pt_sonar = np.array([x_sonar, y_sonar, z_sonar, 1.0])
                        pt_world = T_sonar_to_world @ pt_sonar
                        
                        # Apply Z-axis filter if enabled
                        if self.z_filter_enabled and pt_world[2] < self.z_filter_min:
                            continue

                        # Add to regular map updates with intensity value
                        updates.append((pt_world[:3], self.log_odds_occupied, 'occupied', float(intensity)))

                        # Store robot detection separately if enabled
                        if is_robot:
                            current_time = time.time()
                            self.robot_detections.append((pt_world[:3].copy(), intensity, current_time))
        
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
        
        # Clean up old robot detections (older than 10 seconds)
        if self.enable_robot_detection and self.robot_detections:
            current_time = start_time
            self.robot_detections = [
                (point, intensity, timestamp) for point, intensity, timestamp in self.robot_detections
                if current_time - timestamp <= self.robot_detection_timeout
            ]
        
        # Ensure image is numpy array
        if not isinstance(polar_image, np.ndarray):
            polar_image = np.array(polar_image)
        
        # Get image dimensions
        range_bins, bearing_bins = polar_image.shape
        
        # Update bearing angles if needed
        if bearing_bins != self.image_width:
            self.bearing_angles = np.linspace(
                -self.horizontal_fov/2,
                self.horizontal_fov/2,
                bearing_bins
            )
            self.image_width = bearing_bins
        
        # Create transformation matrices
        T_base_to_world = self.create_odometry_transform(robot_position, robot_orientation)
        T_sonar_to_world = T_base_to_world @ self.T_sonar_to_base
        
        # Accumulate updates per voxel
        voxel_updates = defaultdict(lambda: {'sum': 0.0, 'count': 0, 'type': 'unknown', 'intensity': None})

        # Process subset of bearings for efficiency
        bearing_step = max(1, bearing_bins // 256)

        for b_idx in range(0, bearing_bins, bearing_step):
            bearing_angle = self.bearing_angles[b_idx]

            # Skip bearings outside valid FOV
            if not self.is_bearing_in_valid_fov(bearing_angle):
                continue

            # Process this ray
            intensity_profile = polar_image[:, b_idx]
            ray_updates = self.process_sonar_ray(bearing_angle, intensity_profile, T_sonar_to_world)

            # Accumulate updates
            for point, log_odds, update_type, intensity in ray_updates:
                key = self.world_to_key(point[0], point[1], point[2])
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
            # C++ 배치 업데이트 사용
            points_list = []
            log_odds_list = []
            is_occupied_list = []
            
            for key, update_info in voxel_updates.items():
                if update_info['count'] > 0:
                    avg_update = update_info['sum'] / update_info['count']
                    
                    # 키를 월드 좌표로 변환
                    world_point = self.key_to_world(key)
                    points_list.append(world_point)
                    log_odds_list.append(avg_update)
                    is_occupied = update_info['type'] == 'occupied'
                    is_occupied_list.append(is_occupied)
                    
                    if is_occupied:
                        num_occupied += 1
                    else:
                        num_free += 1
            
            # NumPy 배열로 변환하여 배치 업데이트
            if points_list:
                points_array = np.array(points_list, dtype=np.float64)
                log_odds_array = np.array(log_odds_list, dtype=np.float64)
                is_occupied_array = np.array(is_occupied_list, dtype=bool)
                
                # C++ 배치 업데이트 실행
                self.octree.batch_update(
                    points_array, log_odds_array, is_occupied_array
                )
                
        else:
            # Python 개별 업데이트 사용
            for key, update_info in voxel_updates.items():
                if update_info['count'] > 0:
                    avg_update = update_info['sum'] / update_info['count']
                    point = self.key_to_world(key)
                    intensity_val = update_info.get('intensity', None)

                    if update_info['type'] == 'occupied':
                        self.octree.update_voxel(point, avg_update, adaptive=True, intensity=intensity_val)
                        num_occupied += 1
                    elif update_info['type'] == 'free':
                        self.octree.update_voxel(point, avg_update, adaptive=False, intensity=None)
                        num_free += 1
        
        # Calculate processing time
        processing_time = time.time() - start_time
        self.last_processing_time = processing_time
        self.total_processing_time += processing_time
        
        # Get voxel count (compatible with both C++ and Python)
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            num_voxels = self.octree.get_num_nodes()
        else:
            num_voxels = len(self.octree.voxels)
        
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
            include_free: Whether to include free space voxels
            
        Returns:
            Dictionary containing point cloud data and statistics
        """
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            # C++ ProbabilityUpdater 사용
            if include_free:
                warnings.warn("C++ 백엔드는 free space 복셀 조회를 지원하지 않습니다. occupied만 반환합니다.")
                include_free = False
            
            # C++에서 점유 복셀 조회
            occupied_data = self.octree.get_occupied_voxels(self.occupied_threshold)
            
            if len(occupied_data) > 0:
                points = occupied_data[:, :3]  # x, y, z
                probabilities = occupied_data[:, 3]  # probability
            else:
                points = np.empty((0, 3))
                probabilities = np.empty(0)
            
            # 메모리 사용량 통계
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
                'robot_detections': [(point, intensity) for point, intensity, timestamp in self.robot_detections] if self.enable_robot_detection else []
            }
        else:
            # Python SimpleOctree 사용
            if include_free:
                classified = self.octree.get_all_voxels_classified()

                return {
                    'occupied': classified['occupied'],
                    'free': classified['free'],
                    'unknown': classified['unknown'],
                    'num_voxels': len(self.octree.voxels),
                    'num_occupied': len(classified['occupied']),
                    'num_free': len(classified['free']),
                    'num_unknown': len(classified['unknown']),
                    'frame_count': self.frame_count,
                    'processed_count': self.processed_frame_count,
                    'bounds': {
                        'min': self.octree.min_bounds.copy() if self.octree.dynamic_expansion else None,
                        'max': self.octree.max_bounds.copy() if self.octree.dynamic_expansion else None
                    },
                    'robot_detections': [(point, intensity) for point, intensity, timestamp in self.robot_detections] if self.enable_robot_detection else []
                }
            else:
                occupied_voxels = self.octree.get_occupied_voxels(self.occupied_threshold)
                
                if occupied_voxels:
                    points = np.array([v[0] for v in occupied_voxels])
                    probabilities = np.array([v[1] for v in occupied_voxels])
                else:
                    points = np.empty((0, 3))
                    probabilities = np.empty(0)
                
                return {
                    'points': points,
                    'probabilities': probabilities,
                    'num_voxels': len(self.octree.voxels),
                    'num_occupied': len(occupied_voxels),
                    'frame_count': self.frame_count,
                    'processed_count': self.processed_frame_count,
                    'robot_detections': [(point, intensity) for point, intensity, timestamp in self.robot_detections] if self.enable_robot_detection else []
                }
    
    def reset_map(self):
        """Reset the probabilistic map"""
        self.octree.clear()
        self.robot_detections.clear()  # Clear robot detections as well
        self.frame_count = 0
        self.processed_frame_count = 0
        self.total_processing_time = 0.0
        print("Map reset")
    
    def get_memory_stats(self) -> Dict[str, Any]:
        """Get detailed memory usage statistics"""
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            # C++ 메모리 통계
            stats = self.octree.get_memory_usage()
            return {
                'backend': 'C++ ProbabilityUpdater',
                'num_nodes': stats.num_nodes,
                'num_leaf_nodes': stats.num_leaf_nodes,
                'memory_mb': stats.memory_mb,
                'memory_efficiency': stats.memory_efficiency,
                'resolution': self.octree.get_resolution()
            }
        else:
            # Python 메모리 추정
            num_voxels = len(self.octree.voxels)
            # 각 복셀당 대략적인 메모리 사용량 추정 (키 + 값)
            estimated_mb = num_voxels * (3 * 8 + 8) / (1024 * 1024)  # 32바이트 per voxel
            
            return {
                'backend': 'Python SimpleOctree',
                'num_voxels': num_voxels,
                'estimated_memory_mb': estimated_mb,
                'resolution': self.voxel_resolution,
                'dynamic_expansion': self.octree.dynamic_expansion
            }
    
    def prune_map(self) -> int:
        """
        Prune unnecessary nodes from the map
        
        Returns:
            Number of nodes removed (C++ only)
        """
        if self.use_cpp and CPP_MODULE_AVAILABLE:
            removed_nodes = self.octree.prune_tree()
            print(f"트리 정리 완료: {removed_nodes} 노드 제거")
            return removed_nodes
        else:
            print("Python 백엔드는 트리 정리를 지원하지 않습니다")
            return 0


if __name__ == "__main__":
    """
    3D Mapper 기본 테스트
    """
    print("🚀 3D Mapper 기본 테스트")
    
    # 기본 설정
    config = {
        'voxel_resolution': 0.1,
        'occupied_threshold': 0.6,
        'intensity_threshold': 30,
        'max_range': 8.0
    }
    
    # 매퍼 초기화
    print(f"\n매퍼 초기화 - C++ 모듈: {'사용 가능' if CPP_MODULE_AVAILABLE else '사용 불가'}")
    mapper = SonarTo3DMapper(config)
    
    # 테스트 이미지 생성
    test_image = np.zeros((500, 256), dtype=np.uint8)
    test_image[100:150, 120:140] = 80  # 객체
    
    # 프레임 처리 테스트
    print("\n프레임 처리 테스트...")
    stats = mapper.process_sonar_image(test_image, [0, 0, 0], [0, 0, 0, 1])
    print(f"처리 결과: {stats['num_occupied']}개 점유 복셀, {stats['processing_time']:.3f}s")
    
    # 포인트 클라우드 조회
    point_cloud = mapper.get_point_cloud()
    print(f"포인트 클라우드: {point_cloud['num_occupied']}개 점")
    
    # 메모리 통계
    memory_stats = mapper.get_memory_stats()
    print(f"메모리 사용량: {memory_stats}")
    
    print("\n✅ 테스트 완료")