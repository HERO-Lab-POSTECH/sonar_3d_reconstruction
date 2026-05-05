#!/usr/bin/env python3
"""
Launch file for 3D Sonar Mapping System

Arguments:
  sonar_model       : Sonar model [m750d, m3000d]                     (default: m3000d)
  odometry          : Odometry source [cartographer, fast_lio, fast_lio_loc] (default: cartographer)
  map_path          : Out-of-core map directory (empty = in-memory)   (default: '')
  bag_path          : Bag file path for playback                      (default: '')
  bag_rate          : Bag playback rate                               (default: 1.0)
  output_bag_path   : Recording output directory (empty = no record)  (default: '')
  use_opencv_window : Show OpenCV sonar visualization                 (default: true)
  use_sim_time      : Use simulation time (set true for bag)          (default: false)
  use_rviz          : Launch RViz                                     (default: false)
  sonar_pitch       : Sonar pitch angle [deg] [30.0, 60.0, 90.0]     (default: 90.0)
  use_visualizer    : Launch map visualizer node (needs out-of-core)  (default: true)

TF tree (provided by SLAM launch or bag file):
  map -> odom -> base_link -> sonar_link / livox_frame / imu_link
  Legacy aliases: body->base_link, oculus->sonar_link

Examples:
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py bag_path:=/path/to.bag use_sim_time:=true
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py map_path:=/path/to/map_dir use_rviz:=true
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
    'cartographer': '',                                  # no per-frame scalar published
    'fast_lio': '',                                      # mapping mode: fitness undefined
    'fast_lio_loc': '/localization/fast_lio_loc/confidence',
}


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


def setup_mapper_nodes(context, *args, **kwargs):
    """Setup 3D mapper and visualizer nodes with resolved launch arguments"""
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
    sonar_pitch = context.launch_configurations.get('sonar_pitch', '90.0')
    use_visualizer = context.launch_configurations.get('use_visualizer', 'false')

    # Resolve tilt preset config
    tilt_preset = os.path.join(
        pkg_dir, 'config', 'presets', f'tilt_{int(float(sonar_pitch))}.yaml'
    )
    if not os.path.exists(tilt_preset):
        raise RuntimeError(
            f'[3D Mapping] No tilt preset for sonar_pitch={sonar_pitch}. '
            f'Available: config/presets/tilt_30.yaml, tilt_60.yaml, tilt_90.yaml'
        )

    # Load common.yaml for default values
    with open(common_config, 'r') as f:
        yaml_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # Resolve sonar configuration
    sonar_cfg = SONAR_CONFIG.get(sonar_model, SONAR_CONFIG['m3000d'])
    sonar_fov = sonar_cfg['fov']
    sonar_topic = sonar_cfg['topic']

    # Resolve odometry topic
    odometry_topic = ODOMETRY_CONFIG.get(odometry, ODOMETRY_CONFIG['cartographer'])
    slam_confidence_topic = CONFIDENCE_CONFIG.get(odometry, '')

    # Resolve out-of-core settings
    use_outofcore = bool(map_path)

    # Print configuration
    print(f'[3D Mapping] Sonar: {sonar_model} (FOV={sonar_fov}°, topic={sonar_topic})')
    print(f'[3D Mapping] Tilt preset: tilt_{int(float(sonar_pitch))}.yaml')
    print(f'[3D Mapping] Odometry: {odometry} (topic={odometry_topic})')
    print(f'[3D Mapping] SLAM confidence topic: {slam_confidence_topic or "(disabled)"}')
    print(f'[3D Mapping] Out-of-core: {"enabled" if use_outofcore else "disabled (in-memory)"}')
    if use_outofcore:
        print(f'[3D Mapping] Map path: {map_path}')

    nodes = []

    # Build parameter overrides
    mapper_overrides = {
        'use_sim_time': use_sim_time == 'true',
        'sonar.horizontal_fov': sonar_fov,
        'topics.sonar': sonar_topic,
        'topics.odometry': odometry_topic,
        'topics.slam_confidence': slam_confidence_topic,
        'outofcore.use': use_outofcore,
        'visualization.show_opencv_visualization': use_opencv_window == 'true',
    }

    if use_outofcore:
        mapper_overrides['outofcore.map_path'] = map_path
        os.makedirs(map_path, exist_ok=True)

    # Mounting configuration
    # NOTE: Position (x, y, z) is managed via TF (base_link -> sonar_link in boat_description URDF)
    #       Only orientation is configured here since it's sonar-specific
    mapper_overrides['mounting.orientation.roll'] = 0.0
    mapper_overrides['mounting.orientation.pitch'] = float(sonar_pitch)
    mapper_overrides['mounting.orientation.yaw'] = 0.0

    # 3D Mapper node
    nodes.append(Node(
        package='sonar_3d_reconstruction',
        executable='3d_mapper_node.py',
        name='sonar_3d_mapper',
        parameters=[
            common_config,
            tilt_preset,
            mapper_overrides
        ],
        output='screen'
    ))

    # Map Visualizer node (if enabled and using out-of-core)
    if use_visualizer.lower() == 'true' and use_outofcore:
        visualizer_overrides = {
            'use_sim_time': use_sim_time == 'true',
            'outofcore.map_path': map_path,
            'frames.map': yaml_params.get('frames', {}).get('map', 'map'),
            'mapping.occupied_threshold': yaml_params.get('mapping', {}).get('occupied_threshold', 0.7),
            'octree.voxel_resolution': yaml_params.get('octree', {}).get('voxel_resolution', 0.1),
            'outofcore.tile_size': yaml_params.get('outofcore', {}).get('tile_size', 10.0),
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

    return nodes


def setup_bag_playback(context, *args, **kwargs):
    """Setup bag playback if bag_path is provided"""
    bag_path = context.launch_configurations.get('bag_path', '')
    bag_rate = context.launch_configurations.get('bag_rate', '1.0')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')

    if not bag_path:
        return []

    print(f'[3D Mapping] Bag playback: {bag_path} (rate={bag_rate}x)')

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

    print(f'[3D Mapping] Bag recording: {bag_output}/')

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_output],
            output='screen'
        )
    ]


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')

    # Config files
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')

    # Load defaults from common.yaml
    with open(common_config, 'r') as f:
        common_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # RViz config
    rviz_config_name = common_params.get('rviz_config', '3d_mapping.rviz')
    rviz_config = os.path.join(pkg_dir, 'rviz', rviz_config_name)

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

        # =====================================================================
        # STORAGE
        # =====================================================================
        DeclareLaunchArgument('map_path',
            default_value='',
            description='Out-of-core map path (empty = in-memory mode)'),

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
        DeclareLaunchArgument('sonar_pitch',
            default_value='90.0',
            choices=['30.0', '60.0', '90.0'],
            description='Sonar pitch angle [deg] - loads matching tilt preset config'),
        DeclareLaunchArgument('use_visualizer',
            default_value='true',
            description='Launch map visualizer node (requires out-of-core mode)'),
    ])

    # 3D Mapper + Visualizer nodes
    ld.add_action(OpaqueFunction(function=setup_mapper_nodes))

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
