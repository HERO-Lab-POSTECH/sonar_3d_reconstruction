#!/usr/bin/env python3
"""
Launch file for 3D Sonar Mapping System

Usage:
  # Default (IWLO method):
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

  # Override bag file:
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py bag_file:=/path/to/bag
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    fast_lio_pkg = get_package_share_directory('fast_lio')

    # Config files
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')
    method_iwlo = os.path.join(pkg_dir, 'config', 'method_iwlo.yaml')
    robot_detection_config = os.path.join(pkg_dir, 'config', 'robot_detection.yaml')
    crosstalk_config = os.path.join(pkg_dir, 'config', 'crosstalk_filter.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', '3d_mapping.rviz')

    # Load defaults from common.yaml
    with open(common_config, 'r') as f:
        common_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_fast_lio = LaunchConfiguration('launch_fast_lio')
    launch_rviz = LaunchConfiguration('launch_rviz')
    play_bag = LaunchConfiguration('play_bag')
    bag_file = LaunchConfiguration('bag_file')
    bag_rate = LaunchConfiguration('bag_rate')
    sonar_pitch = LaunchConfiguration('sonar_pitch')

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
    ])

    # Fast-LIO
    ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_pkg, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'rviz': 'false'}.items(),
        condition=IfCondition(launch_fast_lio)
    ))

    # 3D Mapper node (IWLO only)
    ld.add_action(Node(
        package='sonar_3d_reconstruction',
        executable='3d_mapper_node.py',
        name='sonar_3d_mapper',
        parameters=[
            common_config,
            method_iwlo,
            robot_detection_config,
            crosstalk_config,
            {'use_sim_time': use_sim_time, 'sonar_orientation.pitch': sonar_pitch}
        ],
        output='screen'
    ))

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

    return ld
