# =============================================================================
# Minimal test launch: only bridge node + Nav2 controller + velocity smoother.
# Use this for initial bring-up / debugging.
#
# Requires: TF (map→odom→base_link), /odom, /pct_path
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('pct_nav2_bridge')

    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'params', 'nav2_controller_params.yaml'))

    declare_params = DeclareLaunchArgument(
        'params_file', default_value=params_file)

    # Bridge node only (assumes controller_server already running)
    bridge_node = Node(
        package='pct_nav2_bridge',
        executable='bridge_node',
        name='pct_nav2_bridge',
        output='screen',
        parameters=[{
            'controller_id': 'FollowPath',
            'goal_checker_id': 'goal_checker',
            'path_frame': 'map',
            'zero_vel_on_failure': True,
            'clip_nearby': True,
            'clip_radius_m': 0.3,
            'segment_cross_floor': False,
            'cmd_vel_topic': '/cmd_vel',
        }])

    return LaunchDescription([
        declare_params,
        bridge_node,
    ])
