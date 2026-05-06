#!/usr/bin/env python3
"""
Launch file for Robot Detection - Dual Map Visualization

Detects robots/objects by scanning with sonar. The original map is shown in
normal colors; new scan data written to detection_map shown in RED.

Arguments:
  sonar_model        : Sonar model [m750d, m3000d]                     (default: m3000d)
  odometry           : Odometry source [cartographer, fast_lio, fast_lio_loc] (default: cartographer)
  sonar_pitch        : Sonar tilt angle [deg] (preset: 30, 60, 90)    (default: 60.0)
  map_path           : Path to existing map tiles (required)           (default: '')
  detection_map_path : Detection map path (empty = auto-derived)       (default: '')
  bag_path           : Bag file path for playback                      (default: '')
  bag_rate           : Bag playback rate                               (default: 1.0)
  use_opencv_window  : Show OpenCV sonar visualization                 (default: true)
  use_sim_time       : Use simulation time (set true for bag)          (default: false)
  use_rviz           : Launch RViz                                     (default: false)
  use_visualizer     : Launch original map visualizer node             (default: true)

Preset selection:
  sonar_pitch=30 -> config/presets/robot_detect_tilt_30.yaml
  sonar_pitch=60 -> config/presets/robot_detect_tilt_60.yaml (default)
  sonar_pitch=90 -> config/presets/robot_detect_tilt_90.yaml

Output topics:
  /map_visualizer/octomap   - original map (OctoMap, Z-axis coloring)
  /robot_detection/octomap  - detection map (OctoMap, RED)

Examples:
  ros2 launch sonar_3d_reconstruction robot_3d_mapping.launch.py map_path:=/path/to/sonar use_sim_time:=true
  ros2 launch sonar_3d_reconstruction robot_3d_mapping.launch.py map_path:=/path/to/map bag_path:=/path/to/bag use_sim_time:=true
"""

import os
import yaml
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


# =============================================================================
# SONAR MODEL CONFIGURATION
# =============================================================================
SONAR_CONFIG = {
    'm750d': {
        'fov': 70.0,
        'topic': '/sensor/sonar/oculus/m750d/image',
    },
    'm3000d': {
        'fov': 130.0,
        'topic': '/sensor/sonar/oculus/m3000d/image',
    },
}

# =============================================================================
# ODOMETRY CONFIGURATION
# =============================================================================
ODOMETRY_CONFIG = {
    'cartographer': '/localization/cartographer/odometry',
    'fast_lio': '/localization/fast_lio/odometry',
    'fast_lio_loc': '/localization/fast_lio_loc/odometry',
}

CONFIDENCE_CONFIG = {
    'cartographer': '',
    'fast_lio': '',
    'fast_lio_loc': '/localization/fast_lio_loc/confidence',
}

# Valid sonar pitch angles for robot detection
VALID_PITCHES = [30, 60, 90]


def _flatten_dict(d, parent_key='', sep='.'):
    """Flatten nested dict: {'a': {'b': 1}} -> {'a.b': 1}"""
    items = []
    for k, v in d.items():
        new_key = f'{parent_key}{sep}{k}' if parent_key else k
        if isinstance(v, dict):
            items.extend(_flatten_dict(v, new_key, sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def _auto_force_sim_time(context):
    """Force use_sim_time=true when bag_path is set (spec §2.6.1).

    Silent — overrides any prior value. Bag replay always implies sim time;
    distinguishing user-explicit-false from default-false is unreliable
    via LaunchContext, so we always force. Mutates
    context.launch_configurations so downstream OpaqueFunctions see the
    corrected value.
    """
    bag_path = context.launch_configurations.get('bag_path', '')
    if not bag_path:
        return
    prev = context.launch_configurations.get('use_sim_time', 'false').lower()
    if prev != 'true':
        print(f"[launch] bag_path='{bag_path}' → forcing use_sim_time:=true (was: {prev})")
    context.launch_configurations['use_sim_time'] = 'true'


def setup_nodes(context, *args, **kwargs):
    """Setup mapper, original map visualizer, and detection map visualizer"""
    _auto_force_sim_time(context)

    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')
    map_visualizer_config = os.path.join(pkg_dir, 'config', 'map_visualizer.yaml')

    # Get launch configurations
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
    sonar_model = context.launch_configurations.get('sonar_model', 'm3000d')
    odometry = context.launch_configurations.get('odometry', 'cartographer')
    map_path = context.launch_configurations.get('map_path', '')
    use_opencv_window = context.launch_configurations.get('use_opencv_window', 'false')
    use_visualizer = context.launch_configurations.get('use_visualizer', 'true')
    detection_map_arg = context.launch_configurations.get('detection_map_path', '')
    sonar_pitch = float(context.launch_configurations.get('sonar_pitch', '60.0'))
    pitch_int = int(sonar_pitch)  # For preset file lookup

    # Robot detection preset (selected by sonar_pitch)
    tilt_preset = os.path.join(
        pkg_dir, 'config', 'presets', f'robot_detect_tilt_{pitch_int}.yaml'
    )
    if not os.path.exists(tilt_preset):
        raise RuntimeError(
            f'[Robot Detection V2] Preset not found: {tilt_preset}. '
            f'Valid pitches: {VALID_PITCHES}'
        )

    if not map_path:
        raise RuntimeError(
            '[Robot Detection V2] map_path is required! '
            'Provide the path to the existing sonar map.'
        )

    # =========================================================================
    # Detection map path (mapper writes here, starts empty)
    # =========================================================================
    original_map_path = map_path

    if detection_map_arg:
        detection_map_path = detection_map_arg
    else:
        detection_map_path = os.path.join(
            os.path.dirname(map_path), 'robot_detection_map'
        )

    # Create empty detection map directory if not exists
    os.makedirs(detection_map_path, exist_ok=True)

    # Load common.yaml for default values
    with open(common_config, 'r') as f:
        yaml_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # Load tilt preset yaml (node name differs, so load manually)
    with open(tilt_preset, 'r') as f:
        preset_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # Resolve sonar configuration
    sonar_cfg = SONAR_CONFIG.get(sonar_model, SONAR_CONFIG['m3000d'])
    sonar_fov = sonar_cfg['fov']
    sonar_topic = sonar_cfg['topic']

    # Resolve odometry topic
    odometry_topic = ODOMETRY_CONFIG.get(odometry, ODOMETRY_CONFIG['cartographer'])
    slam_confidence_topic = CONFIDENCE_CONFIG.get(odometry, '')

    # Print configuration
    print(f'[Robot Detection V2] Sonar: {sonar_model} (FOV={sonar_fov})')
    print(f'[Robot Detection V2] Preset: robot_detect_tilt_{pitch_int}.yaml (pitch={sonar_pitch}°)')
    print(f'[Robot Detection V2] Odometry: {odometry} (topic={odometry_topic})')
    print(f'[Robot Detection V2] SLAM confidence topic: {slam_confidence_topic or "(disabled)"}')
    print(f'[Robot Detection V2] Original map: {original_map_path}')
    print(f'[Robot Detection V2] Detection map: {detection_map_path}')

    nodes = []

    # =========================================================================
    # Node 1: Mapper (writes to detection_map)
    # =========================================================================
    # Flatten preset params and merge (node name differs from yaml key)
    preset_flat = _flatten_dict(preset_params)

    mapper_overrides = {
        'use_sim_time': use_sim_time == 'true',
        'sonar.horizontal_fov': sonar_fov,
        'topics.sonar': sonar_topic,
        'topics.odometry': odometry_topic,
        'topics.slam_confidence': slam_confidence_topic,
        'topics.pointcloud': '/robot_detection/point_cloud',
        'topics.marker': '/robot_detection/occupancy_grid',
        'outofcore.use': True,
        'outofcore.map_path': detection_map_path,
        'visualization.show_opencv_visualization': use_opencv_window == 'true',
        'visualization.tile_save_interval': 3.0,
        'mounting.orientation.roll': 0.0,
        'mounting.orientation.pitch': float(sonar_pitch),
        'mounting.orientation.yaw': 0.0,
        'depth_estimation.enabled': True,
        'depth_estimation.reference_map_path': original_map_path,
    }

    # Flatten common params too (yaml key is sonar_3d_mapper, node name is robot_detection)
    common_flat = _flatten_dict(yaml_params)

    # Common first, then preset, then overrides (later takes priority)
    merged_params = {**common_flat, **preset_flat, **mapper_overrides}

    nodes.append(Node(
        package='sonar_3d_reconstruction',
        executable='3d_mapper_node.py',
        name='robot_detection',
        parameters=[
            merged_params
        ],
        remappings=[
            ('/sonar_3d_mapper/debug/crosstalk_filtered', '/robot_detection/filtered_image'),
            ('/perception/sonar_3d/tile_indices', '/robot_detection/updated_tile_indices'),
        ],
        output='screen'
    ))

    # =========================================================================
    # Node 2: Original Map Visualizer (shows existing map, normal coloring)
    # =========================================================================
    if use_visualizer.lower() == 'true':
        visualizer_overrides = {
            'use_sim_time': use_sim_time == 'true',
            'outofcore.map_path': original_map_path,
            'frames.map': yaml_params.get('frames', {}).get('map', 'map'),
            'mapping.occupied_threshold': yaml_params.get('mapping', {}).get('occupied_threshold', 0.7),
            'octree.voxel_resolution': yaml_params.get('octree', {}).get('voxel_resolution', 0.1),
            'outofcore.tile_size': yaml_params.get('outofcore', {}).get('tile_size', 10.0),
            'visualization.mode': 2,
            'refresh_interval': 3.0,
        }
        nodes.append(Node(
            package='sonar_3d_reconstruction',
            executable='map_visualizer_node.py',
            name='map_visualizer',
            parameters=[
                map_visualizer_config,
                visualizer_overrides
            ],
            output='screen'
        ))

    # =========================================================================
    # Node 3: Detection visualization (MarkerArray with 5s auto-expire)
    # Published by robot_detection node: /robot_detection/occupancy_grid
    # =========================================================================

    return nodes


def setup_bag_playback(context, *args, **kwargs):
    """Setup bag playback if bag_path is provided"""
    bag_path = context.launch_configurations.get('bag_path', '')
    bag_rate = context.launch_configurations.get('bag_rate', '1.0')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')

    if not bag_path:
        return []

    print(f'[Robot Detection V2] Bag playback: {bag_path} (rate={bag_rate}x)')

    cmd = ['ros2', 'bag', 'play', bag_path, '--rate', bag_rate]
    if use_sim_time == 'true':
        cmd.append('--clock')

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen'
        )
    ]


def setup_bag_recording(context, *args, **kwargs):
    """Setup bag recording if output_bag_path is provided"""
    output_bag_path = context.launch_configurations.get('output_bag_path', '')

    if not output_bag_path:
        return []

    # Create timestamped subfolder
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_output = os.path.join(output_bag_path, timestamp)

    # QoS override: ros2 bag record defaults to RELIABLE, which silently
    # drops messages from BEST_EFFORT publishers (livox, oculus, ping360, etc.)
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    qos_override_path = os.path.join(pkg_dir, 'config', 'bag_record_qos_override.yaml')

    print(f'[Robot Detection V2] Bag recording: {bag_output}/')

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-a',
                '--qos-profile-overrides-path', qos_override_path,
                '-o', bag_output,
            ],
            output='screen'
        )
    ]


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')

    # RViz config
    rviz_config = os.path.join(pkg_dir, 'rviz', 'robot_detection_v2.rviz')

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    # Build launch description
    ld = LaunchDescription([
        # =====================================================================
        # SONAR & ODOMETRY
        # =====================================================================
        DeclareLaunchArgument('sonar_model',
            default_value='m3000d',
            choices=['m750d', 'm3000d'],
            description='Sonar model: m750d (FOV=70) or m3000d (FOV=130)'),
        DeclareLaunchArgument('odometry',
            default_value='cartographer',
            choices=['cartographer', 'fast_lio', 'fast_lio_loc'],
            description='Odometry source'),
        DeclareLaunchArgument('sonar_pitch',
            default_value='60.0',
            description='Sonar tilt angle in degrees (preset: 30, 60, 90)'),

        # =====================================================================
        # MAP PATHS
        # =====================================================================
        DeclareLaunchArgument('map_path',
            default_value='',
            description='Path to existing map tiles (original, read-only)'),
        DeclareLaunchArgument('detection_map_path',
            default_value='',
            description='Detection map path (empty = auto: {parent}/robot_detection_map)'),

        # =====================================================================
        # BAG PLAYBACK & RECORDING
        # =====================================================================
        DeclareLaunchArgument('bag_path',
            default_value='',
            description='Bag file path for playback (empty = no playback)'),
        DeclareLaunchArgument('bag_rate',
            default_value='1.0',
            description='Bag playback rate'),
        DeclareLaunchArgument('output_bag_path',
            default_value='',
            description='Recording output directory (empty = no recording)'),

        # =====================================================================
        # VISUALIZATION & LAUNCH OPTIONS
        # =====================================================================
        DeclareLaunchArgument('use_opencv_window',
            default_value='true',
            description='Show OpenCV visualization'),
        DeclareLaunchArgument('use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument('use_rviz',
            default_value='false',
            description='Launch RViz'),

        # =====================================================================
        # ADVANCED OPTIONS
        # =====================================================================
        DeclareLaunchArgument('use_visualizer',
            default_value='true',
            description='Launch original map visualizer node'),
    ])

    # Mapper + Visualizers
    ld.add_action(OpaqueFunction(function=setup_nodes))

    # RViz (conditional)
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    ))

    # Bag playback (conditional on bag_path)
    ld.add_action(OpaqueFunction(function=setup_bag_playback))

    # Bag recording (conditional on output_bag_path)
    ld.add_action(OpaqueFunction(function=setup_bag_recording))

    return ld
