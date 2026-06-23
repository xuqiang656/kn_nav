# =============================================================================
# PCT + Pure Pursuit Planner — integrated launch
#
# Architecture:
#   PCT planner → /pct_path (nav_msgs/Path, frame_id=map)
#   open3d_loc → /Odometry_open3d (nav_msgs/Odometry, map→base_link)
#        ↓
#   pure_pursuit_planner
#        ↓
#   /cmd_vel → 机器人
#
# Usage:
#   ros2 launch pure_pursuit_planner pct_pure_pursuit.launch.py
# =============================================================================

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('pure_pursuit_planner')
    config_path = os.path.join(pkg_dir, 'config', 'pct_params.yaml')

    pure_pursuit_node = Node(
        package='pure_pursuit_planner',
        executable='pure_pursuit_planner',
        name='pure_pursuit_planner',
        output='screen',
        parameters=[config_path],
    )

    return LaunchDescription([pure_pursuit_node])
