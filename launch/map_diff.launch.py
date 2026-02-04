#!/usr/bin/env python3
"""
Launch file for Map Difference Visualizer

Usage:
    ros2 launch sonar_3d_reconstruction map_diff.launch.py \
        map_a:=/path/to/map_a/map_tile \
        map_b:=/path/to/map_b/map_tile
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')

    # Default paths for KHNP data
    default_map_a = '/workspace/data/5_khnp/20251219_seafloor_mapping/m750d_lawnmower_small_v1.1/map_tile'
    default_map_b = '/workspace/data/5_khnp/20251219_seafloor_mapping/m750d_lawnmower_small_v1.2/map_tile'

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('map_a', default_value=default_map_a,
                              description='Path to prior map (map_tile directory)'),
        DeclareLaunchArgument('map_b', default_value=default_map_b,
                              description='Path to new map (map_tile directory)'),
        DeclareLaunchArgument('tolerance', default_value='0.4',
                              description='Voxel matching tolerance in meters'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz2'),

        # Map Diff Visualizer Node
        Node(
            package='sonar_3d_reconstruction',
            executable='map_diff_visualizer.py',
            name='map_diff_visualizer',
            output='screen',
            arguments=[
                '--map_a', LaunchConfiguration('map_a'),
                '--map_b', LaunchConfiguration('map_b'),
                '--tolerance', LaunchConfiguration('tolerance'),
            ]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_dir, 'rviz', 'map_diff.rviz')],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
