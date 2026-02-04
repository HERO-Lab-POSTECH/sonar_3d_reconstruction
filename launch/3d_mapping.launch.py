#!/usr/bin/env python3
"""
Launch file for 3D Sonar Mapping System

================================================================================
LAUNCH ARGUMENTS
================================================================================
  sonar_model   : Sonar model selection [m750d, m3000d]              (default: m3000d)
                  - m750d:  FOV=70deg,  topic=/sensor/sonar/oculus/m750d/image
                  - m3000d: FOV=130deg, topic=/sensor/sonar/oculus/m3000d/image

  odometry      : Odometry source [cartographer, fast_lio, fast_lio_loc] (default: cartographer)
                  - cartographer:  topic=cartographer_2d/odometry
                  - fast_lio:      topic=/fast_lio/odometry
                  - fast_lio_loc:  topic=/fast_lio/localization/odometry

  map_path      : Out-of-core map directory path                     (default: "" = in-memory)
                  If provided, enables disk-based mapping. Reuses existing map.

  bag_file      : Bag file path for playback                         (default: "" = no playback)
  bag_rate      : Bag playback rate                                  (default: 1.0)
  record_path   : Recording output directory                         (default: "" = no recording)

  show_opencv   : Show OpenCV visualization                          (default: false)
  use_sim_time  : Use simulation time                                (default: false)
  rviz          : Launch RViz                                        (default: false)
  foxglove      : Launch Foxglove bridge (ws://localhost:8765)       (default: false)

  sonar_pitch   : Sonar pitch angle [deg]                            (default: 90.0)
  launch_visualizer : Launch map visualizer node                     (default: true)

================================================================================
TF TREE (provided by SLAM launch or bag file)
================================================================================
  map
  └── odom
      └── base_link
          ├── livox_frame (LiDAR)
          ├── imu_link
          └── sonar_link (Oculus sonar)

  Legacy alias (for old bag compatibility):
    body -> base_link
    oculus -> sonar_link

================================================================================
TOPICS
================================================================================
  Input:
    - /sensor/sonar/oculus/{m750d,m3000d}/image (sensor_msgs/Image)
    - {cartographer_2d,/fast_lio}/odometry (nav_msgs/Odometry)
  Output:
    - /sonar_3d_mapper/point_cloud (sensor_msgs/PointCloud2)
    - /sonar_3d_mapper/occupancy_grid (visualization_msgs/MarkerArray)

================================================================================
EXAMPLES
================================================================================
  # With Cartographer SLAM (separate terminal):
  ros2 launch cartographer_slam slam.launch.py
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py

  # With Fast-LIO SLAM (separate terminal):
  ros2 launch fast_lio mapping.launch.py
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio

  # With Fast-LIO Localization:
  ros2 launch fast_lio localization.launch.py map_path:=/path/to/map.pcd
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py odometry:=fast_lio_loc

  # Bag playback (TF from bag):
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py bag_file:=/path/to/bag use_sim_time:=true

  # Out-of-core mapping (disk-based, for large maps):
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py map_path:=/path/to/map_dir

  # Disable Foxglove bridge:
  ros2 launch sonar_3d_reconstruction 3d_mapping.launch.py foxglove:=false
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
    'cartographer': 'cartographer_2d/odometry',
    'fast_lio': '/fast_lio/odometry',
    'fast_lio_loc': '/fast_lio/localization/odometry',
}


def setup_mapper_nodes(context, *args, **kwargs):
    """Setup 3D mapper and visualizer nodes with resolved launch arguments"""
    pkg_dir = get_package_share_directory('sonar_3d_reconstruction')
    common_config = os.path.join(pkg_dir, 'config', 'common.yaml')
    method_iwlo = os.path.join(pkg_dir, 'config', 'method_iwlo.yaml')
    map_visualizer_config = os.path.join(pkg_dir, 'config', 'map_visualizer.yaml')

    # Get launch configurations
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')
    sonar_model = context.launch_configurations.get('sonar_model', 'm3000d')
    odometry = context.launch_configurations.get('odometry', 'cartographer')
    map_path = context.launch_configurations.get('map_path', '')
    show_opencv = context.launch_configurations.get('show_opencv', 'false')
    sonar_pitch = context.launch_configurations.get('sonar_pitch', '90.0')
    launch_visualizer = context.launch_configurations.get('launch_visualizer', 'false')

    # Load common.yaml for default values
    with open(common_config, 'r') as f:
        yaml_params = yaml.safe_load(f)['sonar_3d_mapper']['ros__parameters']

    # Resolve sonar configuration
    sonar_cfg = SONAR_CONFIG.get(sonar_model, SONAR_CONFIG['m3000d'])
    sonar_fov = sonar_cfg['fov']
    sonar_topic = sonar_cfg['topic']

    # Resolve odometry topic
    odometry_topic = ODOMETRY_CONFIG.get(odometry, ODOMETRY_CONFIG['cartographer'])

    # Resolve out-of-core settings
    use_outofcore = bool(map_path)

    # Print configuration
    print(f'[3D Mapping] Sonar: {sonar_model} (FOV={sonar_fov}°, topic={sonar_topic})')
    print(f'[3D Mapping] Odometry: {odometry} (topic={odometry_topic})')
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
        'outofcore.use': use_outofcore,
        'visualization.show_opencv_visualization': show_opencv == 'true',
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
            method_iwlo,
            mapper_overrides
        ],
        output='screen'
    ))

    # Map Visualizer node (if enabled and using out-of-core)
    if launch_visualizer.lower() == 'true' and use_outofcore:
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
    """Setup bag playback if bag_file is provided"""
    bag_file = context.launch_configurations.get('bag_file', '')
    bag_rate = context.launch_configurations.get('bag_rate', '1.0')
    use_sim_time = context.launch_configurations.get('use_sim_time', 'false')

    if not bag_file:
        return []

    print(f'[3D Mapping] Bag playback: {bag_file} (rate={bag_rate}x)')

    cmd = ['ros2', 'bag', 'play', bag_file, '--rate', bag_rate]
    if use_sim_time == 'true':
        cmd.append('--clock')

    return [
        ExecuteProcess(
            cmd=cmd,
            output='screen'
        )
    ]


def setup_bag_recording(context, *args, **kwargs):
    """Setup bag recording if record_path is provided"""
    record_path = context.launch_configurations.get('record_path', '')

    if not record_path:
        return []

    # Create timestamped subfolder
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_output = os.path.join(record_path, timestamp)

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
    rviz = LaunchConfiguration('rviz')
    foxglove = LaunchConfiguration('foxglove')

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
        DeclareLaunchArgument('bag_file',
            default_value='',
            description='Bag file path for playback (empty = no playback)'),
        DeclareLaunchArgument('bag_rate',
            default_value='1.0',
            description='Bag playback rate'),
        DeclareLaunchArgument('record_path',
            default_value='',
            description='Recording output directory (empty = no recording)'),

        # =====================================================================
        # VISUALIZATION & LAUNCH OPTIONS
        # =====================================================================
        DeclareLaunchArgument('show_opencv',
            default_value='false',
            description='Show OpenCV visualization'),
        DeclareLaunchArgument('use_sim_time',
            default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument('rviz',
            default_value='false',
            description='Launch RViz'),
        DeclareLaunchArgument('foxglove',
            default_value='false',
            description='Launch Foxglove bridge (connect via ws://localhost:8765)'),

        # =====================================================================
        # ADVANCED OPTIONS
        # =====================================================================
        DeclareLaunchArgument('sonar_pitch',
            default_value='90.0',
            description='Sonar pitch angle [deg] (90 = pointing down)'),
        DeclareLaunchArgument('launch_visualizer',
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
        condition=IfCondition(rviz)
    ))

    # Foxglove bridge (conditional)
    ld.add_action(Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(foxglove)
    ))


    # Bag playback (conditional on bag_file)
    ld.add_action(OpaqueFunction(function=setup_bag_playback))

    # Bag recording (conditional on record_path)
    ld.add_action(OpaqueFunction(function=setup_bag_recording))

    return ld
