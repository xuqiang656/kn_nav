# =============================================================================
# Plan A: Pure Pursuit Follower — direct /pct_path → /cmd_vel
#
# Bypasses Nav2 ControllerServer entirely.
# Requires only: TF (map→base_link), /pct_path (nav_msgs/Path, frame_id=map)
#
# Usage:
#   ros2 launch pct_nav2_bridge pure_pursuit.launch.py
#
# Override params:
#   ros2 launch pct_nav2_bridge pure_pursuit.launch.py max_linear_vel:=0.15 lookahead_dist:=0.4
# =============================================================================

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ── tunable parameters ──────────────────────────────────────────────
    lookahead_dist = LaunchConfiguration('lookahead_dist', default='0.5')
    min_lookahead_dist = LaunchConfiguration('min_lookahead_dist', default='0.3')
    max_lookahead_dist = LaunchConfiguration('max_lookahead_dist', default='0.8')
    max_linear_vel = LaunchConfiguration('max_linear_vel', default='0.2')
    min_linear_vel = LaunchConfiguration('min_linear_vel', default='0.05')
    max_angular_vel = LaunchConfiguration('max_angular_vel', default='0.5')
    goal_tolerance = LaunchConfiguration('goal_tolerance', default='0.3')
    slowdown_yaw_error = LaunchConfiguration('slowdown_yaw_error', default='0.7')
    global_frame = LaunchConfiguration('global_frame', default='map')
    robot_frame = LaunchConfiguration('robot_frame', default='base_link')
    path_timeout = LaunchConfiguration('path_timeout', default='2.0')
    control_rate = LaunchConfiguration('control_rate', default='20.0')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')

    declare_lookahead = DeclareLaunchArgument(
        'lookahead_dist', default_value='0.5',
        description='Pure pursuit lookahead distance (m)')
    declare_min_lookahead = DeclareLaunchArgument(
        'min_lookahead_dist', default_value='0.3')
    declare_max_lookahead = DeclareLaunchArgument(
        'max_lookahead_dist', default_value='0.8')
    declare_max_v = DeclareLaunchArgument(
        'max_linear_vel', default_value='0.2',
        description='Maximum forward velocity (m/s)')
    declare_min_v = DeclareLaunchArgument(
        'min_linear_vel', default_value='0.05')
    declare_max_w = DeclareLaunchArgument(
        'max_angular_vel', default_value='0.5',
        description='Maximum angular velocity (rad/s)')
    declare_goal_tol = DeclareLaunchArgument(
        'goal_tolerance', default_value='0.3',
        description='Goal arrival radius (m)')
    declare_slowdown = DeclareLaunchArgument(
        'slowdown_yaw_error', default_value='0.7',
        description='Yaw error threshold to begin slowdown (rad)')
    declare_frame = DeclareLaunchArgument(
        'global_frame', default_value='map')
    declare_robot_frame = DeclareLaunchArgument(
        'robot_frame', default_value='base_link')
    declare_timeout = DeclareLaunchArgument(
        'path_timeout', default_value='2.0',
        description='Max age of last path before timeout (s)')
    declare_rate = DeclareLaunchArgument(
        'control_rate', default_value='20.0')
    declare_cmd_vel_topic = DeclareLaunchArgument(
        'cmd_vel_topic', default_value='/cmd_vel')

    # ── pure pursuit follower ───────────────────────────────────────────
    follower = Node(
        package='pct_nav2_bridge',
        executable='pure_pursuit_follower',
        name='pure_pursuit_follower',
        output='screen',
        parameters=[{
            'global_frame': global_frame,
            'robot_frame': robot_frame,
            'lookahead_dist': lookahead_dist,
            'min_lookahead_dist': min_lookahead_dist,
            'max_lookahead_dist': max_lookahead_dist,
            'goal_tolerance': goal_tolerance,
            'max_linear_vel': max_linear_vel,
            'min_linear_vel': min_linear_vel,
            'max_angular_vel': max_angular_vel,
            'slowdown_yaw_error': slowdown_yaw_error,
            'path_timeout': path_timeout,
            'control_rate': control_rate,
            'cmd_vel_topic': cmd_vel_topic,
        }])

    return LaunchDescription([
        declare_lookahead,
        declare_min_lookahead,
        declare_max_lookahead,
        declare_max_v,
        declare_min_v,
        declare_max_w,
        declare_goal_tol,
        declare_slowdown,
        declare_frame,
        declare_robot_frame,
        declare_timeout,
        declare_rate,
        declare_cmd_vel_topic,
        follower,
    ])
