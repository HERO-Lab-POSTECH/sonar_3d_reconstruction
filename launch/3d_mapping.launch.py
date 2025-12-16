#!/usr/bin/env python3
"""
Launch file for 3D Sonar Mapping System

Usage:
  # Default (IWLO method):
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

  # Override bag file:
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py bag_file:=/path/to/bag

  # Custom map output path:
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py map_path:=/custom/path
"""

import os
import yaml
from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_timestamped_map_path(context, *args, **kwargs):
    """Generate timestamped map path and setup mapper/visualizer nodes"""
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')
    method_iwlo = os.path.join(pkg_dir, 'config', 'method_iwlo.yaml')
    robot_detection_config = os.path.join(pkg_dir, 'config', 'robot_detection.yaml')
    crosstalk_config = os.path.join(pkg_dir, 'config', 'crosstalk_filter.yaml')
    map_visualizer_config = os.path.join(pkg_dir, 'config', 'map_visualizer.yaml')

    use_sim_time = context.launch_configurations.get('use_sim_time', 'true')
    sonar_pitch = context.launch_configurations.get('sonar_pitch', '90.0')
    launch_visualizer = context.launch_configurations.get('launch_visualizer', 'false')
    map_path_arg = context.launch_configurations.get('map_path', '')

    # Generate timestamped path if not specified
    if map_path_arg:
        map_path = map_path_arg
    else:
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_path = f'/workspace/data/map_tiles/{timestamp}'

    # Create directory
    os.makedirs(map_path, exist_ok=True)
    print(f'[3D Mapping] Map output path: {map_path}')

    nodes = []

    # 3D Mapper node
    nodes.append(Node(
        package='sonar_3d_reconstruction',
        executable='3d_mapper_node.py',
        name='sonar_3d_mapper',
        parameters=[
            common_config,
            method_iwlo,
            robot_detection_config,
            crosstalk_config,
            {
                'use_sim_time': use_sim_time == 'true',
                'sonar_orientation.pitch': float(sonar_pitch),
                'outofcore_map_path': map_path,  # Override map path
            }
        ],
        output='screen'
    ))

    # Map Visualizer node (if enabled)
    if launch_visualizer.lower() == 'true':
        # Read common params for visualizer
        with open(common_config, 'r') as f:
            vis_common_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

        nodes.append(Node(
            package='sonar_3d_reconstruction',
            executable='map_visualizer_node.py',
            name='map_visualizer',
            parameters=[
                map_visualizer_config,
                {
                    'use_sim_time': use_sim_time == 'true',
                    'outofcore_map_path': map_path,
                    'map_path': map_path,
                    # Pass common.yaml parameters explicitly (different namespace)
                    'outofcore_tile_size': vis_common_params.get('outofcore_tile_size', 10.0),
                    'voxel_resolution': vis_common_params.get('voxel_resolution', 0.1),
                    'outofcore_cache_size': vis_common_params.get('outofcore_cache_size', 16),
                }
            ],
            output='screen'
        ))

    return nodes


def setup_bag_recording(context, *args, **kwargs):
    """Setup bag recording with timestamp-based folder names"""
    from datetime import datetime

    record_bag = context.launch_configurations.get('record_bag', 'false')
    if record_bag.lower() != 'true':
        return []

    base_path = context.launch_configurations.get('record_base_path')

    # Create timestamp-based folder name: YYYYMMDD_HHMMSS
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_path = os.path.join(base_path, timestamp)

    print(f'[Bag Recording] Output: {bag_path}/')

    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-a', '-o', bag_path],
            output='screen'
        )
    ]


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    fast_lio_pkg = get_package_share_directory('fast_lio')

    # Config files
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')

    # Load defaults from common.yaml
    with open(common_config, 'r') as f:
        common_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # RViz config (from common.yaml or default)
    rviz_config_name = common_params.get('rviz_config', '3d_mapping.rviz')
    rviz_config = os.path.join(pkg_dir, 'rviz', rviz_config_name)

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fast_lio = LaunchConfiguration('launch_fast_lio')
    launch_rviz = LaunchConfiguration('launch_rviz')
    play_bag = LaunchConfiguration('play_bag')
    bag_file = LaunchConfiguration('bag_file')
    bag_rate = LaunchConfiguration('bag_rate')
    record_bag = LaunchConfiguration('record_bag')
    record_base_path = LaunchConfiguration('record_base_path')

    # Declare arguments (defaults from common.yaml)
    ld = LaunchDescription([
        DeclareLaunchArgument('use_sim_time',
            default_value=str(common_params.get('use_sim_time', True)).lower()),
        DeclareLaunchArgument('launch_fast_lio',
            default_value=str(common_params.get('launch_fast_lio', True)).lower()),
        DeclareLaunchArgument('launch_rviz',
            default_value=str(common_params.get('launch_rviz', True)).lower()),
        DeclareLaunchArgument('play_bag',
            default_value=str(common_params.get('play_bag', True)).lower()),
        DeclareLaunchArgument('bag_file',
            default_value=common_params.get('bag_file', '')),
        DeclareLaunchArgument('bag_rate',
            default_value=str(common_params.get('bag_playback_rate', 1.0))),
        DeclareLaunchArgument('sonar_pitch',
            default_value=str(common_params.get('sonar_orientation', {}).get('pitch', 90.0)),
            description='Sonar pitch angle in degrees'),
        DeclareLaunchArgument('record_bag',
            default_value=str(common_params.get('record_bag', False)).lower(),
            description='Enable bag recording with auto-increment'),
        DeclareLaunchArgument('record_base_path',
            default_value=common_params.get('record_base_path', '/workspace/data/experiments'),
            description='Base directory for bag recordings'),
        DeclareLaunchArgument('launch_visualizer',
            default_value='true',
            description='Launch map visualizer node for out-of-core tile visualization'),
        DeclareLaunchArgument('map_path',
            default_value='',
            description='Map output path (default: timestamped folder in /workspace/data/map_tiles/)'),
    ])

    # Fast-LIO
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'false'}.items(),
        condition=IfCondition(launch_fast_lio)
    ))

    # 3D Mapper + Visualizer nodes (with timestamped map path)
    ld.add_action(OpaqueFunction(function=generate_timestamped_map_path))

    # RViz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz)
    ))

    # Bag player
    ld.add_action(ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file, '--clock', '--rate', bag_rate],
        output='screen',
        condition=IfCondition(play_bag)
    ))

    # Bag recorder with auto-increment
    ld.add_action(OpaqueFunction(function=setup_bag_recording))

    return ld
