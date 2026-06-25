"""Bring up PCT→ART→Pure Pursuit navigation without Nav2 components."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('pct_art_local_navigation')

    pct_params = os.path.join(package_share, 'config', 'pct_global_planner.yaml')
    fastdem_params = os.path.join(package_share, 'config', 'fastdem_local_mapping.yaml')
    traversability_robot_params = os.path.join(
        package_share, 'config', 'traversability_robot.yaml'
    )
    traversability_footprint_params = os.path.join(
        package_share, 'config', 'traversability_robot_footprint.yaml'
    )
    traversability_filter_params = os.path.join(
        package_share, 'config', 'traversability_robot_filter.yaml'
    )
    coordinator_params = os.path.join(package_share, 'config', 'coordinator.yaml')
    art_params = os.path.join(package_share, 'config', 'art_local.yaml')
    pursuit_params = os.path.join(package_share, 'config', 'pure_pursuit_local.yaml')
    bridge_params = os.path.join(package_share, 'config', 'go2_bridge_local.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_pct_planner = LaunchConfiguration('start_pct_planner')
    pct_params_file = LaunchConfiguration('pct_params_file')
    network_interface = LaunchConfiguration('network_interface')

    unitree_runtime_environment = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [
            '/opt/unitree_robotics/lib',
            ':',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )

    pct_planner = Node(
        package='pct_planner',
        executable='run_ros2_global_planner',
        name='pct_global_planner',
        output='screen',
        parameters=[pct_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_pct_planner),
    )

    fastdem = Node(
        package='fastdem_ros2',
        executable='fastdem_node',
        name='fastdem',
        output='screen',
        parameters=[
            {
                'config_file': fastdem_params,
                'base_frame': 'base_link',
                'map_frame': 'map',
                'use_sim_time': use_sim_time,
                # Kept here so bag/live input can be changed without touching
                # the FastDEM package launch file.
                'input_scan': '/scan_base_link',
            }
        ],
    )

    traversability = Node(
        package='traversability_estimation',
        executable='traversability_estimation_node',
        name='traversability_estimation',
        output='screen',
        parameters=[
            traversability_robot_params,
            traversability_footprint_params,
            traversability_filter_params,
            {'use_sim_time': use_sim_time},
        ],
    )

    art_planner = Node(
        package='art_planner_ros',
        executable='art_planner_ros_node',
        name='art_planner',
        output='screen',
        parameters=[art_params, {'use_sim_time': use_sim_time}],
        remappings=[('~/elevation_map', '/traversability_map')],
    )

    coordinator = Node(
        package='pct_art_local_navigation',
        executable='pct_art_coordinator',
        name='pct_art_coordinator',
        output='screen',
        parameters=[coordinator_params, {'use_sim_time': use_sim_time}],
    )

    pure_pursuit = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner',
        name='pure_pursuit_node',
        output='screen',
        parameters=[pursuit_params, {'use_sim_time': use_sim_time}],
        remappings=[('/pct_path', '/local_path')],
    )

    go2_bridge = Node(
        package='pure_pursuit_planner',
        executable='go2_cmd_vel_bridge',
        name='go2_cmd_vel_bridge',
        output='screen',
        parameters=[
            bridge_params,
            {'network_interface': network_interface, 'use_sim_time': use_sim_time},
        ],
    )

    return LaunchDescription([
        # DeclareLaunchArgument(
        #     'network_interface',
        #     default_value='eth0',
        #     description='Network interface connected to the Go2, for example enp2s0',
        # ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_pct_planner', default_value='true'),
        DeclareLaunchArgument('pct_params_file', default_value=pct_params),
        unitree_runtime_environment,
        pct_planner,
        fastdem,
        traversability,
        art_planner,
        coordinator,
        pure_pursuit,
        # go2_bridge,
    ])
