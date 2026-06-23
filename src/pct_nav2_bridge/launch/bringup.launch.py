# =============================================================================
# Top-level bringup launch — everything needed for PCT→Nav2→robot control
#
# Starts:
#   1. pct_nav2_bridge    — path subscriber + FollowPath action client
#   2. controller_server   — RPP path tracking
#   3. velocity_smoother   — smooth cmd_vel output
#   4. lifecycle_manager   — auto-activate Nav2 nodes
#
# Assumes already running:
#   - open3d_loc / FAST-LIO (publishes /tf and /odom)
#   - PCT planner (publishes /pct_path)
#
# Usage:
#   ros2 launch pct_nav2_bridge bringup.launch.py
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pct_nav2_bridge')

    # ── arguments ───────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    clip_nearby = LaunchConfiguration('clip_nearby', default='false')
    zero_vel_on_failure = LaunchConfiguration('zero_vel_on_failure', default='true')
    segment_cross_floor = LaunchConfiguration('segment_cross_floor', default='false')
    controller_id = LaunchConfiguration('controller_id', default='FollowPath')
    goal_checker_id = LaunchConfiguration('goal_checker_id', default='goal_checker')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true')

    declare_clip_nearby = DeclareLaunchArgument(
        'clip_nearby', default_value='false',
        description='Clip path points behind the robot (requires TF base_link)')

    declare_zero_vel = DeclareLaunchArgument(
        'zero_vel_on_failure', default_value='true',
        description='Publish zero velocity on FollowPath failure')

    declare_segment = DeclareLaunchArgument(
        'segment_cross_floor', default_value='false',
        description='Enable stair/flat path segmentation')

    declare_controller_id = DeclareLaunchArgument(
        'controller_id', default_value='FollowPath')

    declare_goal_checker_id = DeclareLaunchArgument(
        'goal_checker_id', default_value='goal_checker')

    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel')

    # ── Nav2 controller (standalone) ────────────────────────────────────
    nav2_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'nav2_controller.launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items())

    # ── pct_nav2_bridge node ────────────────────────────────────────────
    bridge_node = Node(
        package='pct_nav2_bridge',
        executable='bridge_node',
        name='pct_nav2_bridge',
        output='screen',
        parameters=[{
            'controller_id': controller_id,
            'goal_checker_id': goal_checker_id,
            'path_frame': 'map',
            'zero_vel_on_failure': zero_vel_on_failure,
            'clip_nearby': clip_nearby,
            'clip_radius_m': 0.3,
            'segment_cross_floor': segment_cross_floor,
            'cmd_vel_topic': cmd_vel_topic,
        }])

    return LaunchDescription([
        # arguments
        declare_use_sim_time,
        declare_autostart,
        declare_clip_nearby,
        declare_zero_vel,
        declare_segment,
        declare_controller_id,
        declare_goal_checker_id,
        declare_cmd_vel_topic,
        # nodes
        nav2_controller_launch,
        bridge_node,
    ])
