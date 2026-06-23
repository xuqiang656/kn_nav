# =============================================================================
# Nav2 Controller Server — standalone launch
#
# Starts ONLY controller_server (lifecycle node) with RPP plugin.
# No planner_server, bt_navigator, map_server, or AMCL.
#
# Usage:
#   ros2 launch pct_nav2_bridge nav2_controller.launch.py
# =============================================================================

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_dir = get_package_share_directory('pct_nav2_bridge')

    # ── arguments ───────────────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    autostart = LaunchConfiguration('autostart', default='true')
    params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_dir, 'params', 'nav2_controller_params.yaml'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup Nav2 lifecycle nodes')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'params', 'nav2_controller_params.yaml'),
        description='Path to Nav2 controller params YAML')

    # ── rewrite YAML for selected namespace (if any) ────────────────────
    param_substitutions = {'use_sim_time': use_sim_time}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # ── lifecycle manager ───────────────────────────────────────────────
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': ['controller_server', 'velocity_smoother']}])

    # ── controller server ───────────────────────────────────────────────
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
        ])

    # ── velocity smoother (optional) ────────────────────────────────────
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params],
        remappings=[
            ('/cmd_vel', '/cmd_vel_nav'),
            ('/cmd_vel_smoothed', '/cmd_vel'),
        ])

    # ── collision monitor (optional, disabled by default) ───────────────
    # Uncomment to enable — requires pointcloud/scan input.
    # collision_monitor = Node(
    #     package='nav2_collision_monitor',
    #     executable='collision_monitor',
    #     name='collision_monitor',
    #     output='screen',
    #     parameters=[configured_params],
    #     remappings=[
    #         ('/cmd_vel', '/cmd_vel'),
    #     ])

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        lifecycle_manager,
        controller_server,
        velocity_smoother,
    ])
