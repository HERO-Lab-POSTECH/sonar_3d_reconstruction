#!/usr/bin/env python3
"""
Launch file for Map Visualizer (standalone)

Visualize a saved out-of-core map without running the 3d_mapper node.
Runs only map_visualizer_node.

================================================================================
LAUNCH ARGUMENTS
================================================================================
  map_path      : Out-of-core map directory path               (required)
  use_sim_time  : Use simulation time                          (default: false)
  rviz          : Launch RViz                                  (default: false)
  foxglove      : Launch Foxglove bridge                       (default: true)
  publish_rate  : Point cloud publish rate [Hz]                (default: 1.0)
  vis_mode      : Visualization mode (0=pointcloud, 1=octomap, 2=all) (default: 0)

================================================================================
EXAMPLES
================================================================================
  # Basic usage (map_path required):
  ros2 launch sonar_3d_reconstruction map_visualizer.launch.py map_path:=/path/to/map

  # With RViz:
  ros2 launch sonar_3d_reconstruction map_visualizer.launch.py map_path:=/path/to/map rviz:=true

  # With bag playback:
  ros2 launch sonar_3d_reconstruction map_visualizer.launch.py map_path:=/path/to/map use_sim_time:=true

  # Custom publish rate:
  ros2 launch sonar_3d_reconstruction map_visualizer.launch.py map_path:=/path/to/map publish_rate:=5.0
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def setup_visualizer(context, *args, **kwargs):
    """Setup map visualizer node with resolved launch arguments"""
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')
    map_visualizer_config = os.path.join(pkg_dir, 'config', 'map_visualizer.yaml')

    # Get launch configurations
    map_path = context.launch_configurations.get('map_path', '')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
    publish_rate = context.launch_configurations.get('publish_rate', '1.0')
    vis_mode = context.launch_configurations.get('vis_mode', '0')

    if not map_path:
        print('[Map Visualizer] ERROR: map_path is required')
        return []

    if not os.path.isdir(map_path):
        print(f'[Map Visualizer] WARNING: map_path does not exist: {map_path}')

    # Load common.yaml for shared defaults
    with open(common_config, 'r') as f:
        yaml_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    print(f'[Map Visualizer] Map path: {map_path}')
    print(f'[Map Visualizer] Publish rate: {publish_rate} Hz')
    print(f'[Map Visualizer] Vis mode: {vis_mode} (0=pointcloud, 1=octomap, 2=all)')

    visualizer_overrides = {
        'use_sim_time': use_sim_time == 'true',
        'outofcore.map_path': map_path,
        'publish_rate': float(publish_rate),
        'visualization.mode': int(vis_mode),
        'frames.map': yaml_params.get('frames', {}).get('map', 'map'),
        'mapping.occupied_threshold': yaml_params.get('mapping', {}).get('occupied_threshold', 0.7),
        'octree.voxel_resolution': yaml_params.get('octree', {}).get('voxel_resolution', 0.1),
        'outofcore.tile_size': yaml_params.get('outofcore', {}).get('tile_size', 10.0),
    }

    return [Node(
        package='sonar_3d_reconstruction',
        executable='map_visualizer_node.py',
        name='map_visualizer',
        parameters=[
            map_visualizer_config,
            visualizer_overrides
        ],
        output='screen'
    )]


def generate_launch_description():
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')

    # Load defaults from common.yaml for rviz config
    with open(common_config, 'r') as f:
        common_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    rviz_config_name = common_params.get('rviz_config', '3d_mapping.rviz')
    rviz_config = os.path.join(pkg_dir, 'rviz', rviz_config_name)

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')
    foxglove = LaunchConfiguration('foxglove')

    return LaunchDescription([
        DeclareLaunchArgument('map_path',
            description='Out-of-core map directory path (required)'),
        DeclareLaunchArgument('use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument('rviz',
            default_value='false',
            description='Launch RViz'),
        DeclareLaunchArgument('foxglove',
            default_value='true',
            description='Launch Foxglove bridge'),
        DeclareLaunchArgument('publish_rate',
            default_value='1.0',
            description='PointCloud publish rate [Hz]'),
        DeclareLaunchArgument('vis_mode',
            default_value='0',
            description='Visualization mode: 0=pointcloud, 1=octomap, 2=all'),

        # Map Visualizer node
        OpaqueFunction(function=setup_visualizer),

        # RViz (conditional)
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(rviz)
        ),

        # Foxglove bridge (conditional)
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(foxglove)
        ),
    ])
