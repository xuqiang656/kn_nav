"""Local test launch entry for PCT ART navigation."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


CONFIG_NAME = 'local'


def log_process_start(node, label):
    return RegisterEventHandler(
        OnProcessStart(
            target_action=node,
            on_start=[LogInfo(msg=f'[pct_art bringup] started {label}')],
        )
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_open3d_loc = LaunchConfiguration('start_open3d_loc')
    start_pct_planner = LaunchConfiguration('start_pct_planner')
    start_go2_bridge = LaunchConfiguration('start_go2_bridge')
    pct_params_file = LaunchConfiguration('pct_params_file')
    network_interface = LaunchConfiguration('network_interface')
    package_share = FindPackageShare('pct_art_local_navigation')

    def config_file(filename):
        return PathJoinSubstitution([package_share, 'config', CONFIG_NAME, filename])

    open3d_loc_params = config_file('open3d_loc.yaml')
    fastdem_params = config_file('fastdem_local_mapping.yaml')
    traversability_robot_params = config_file('traversability_robot.yaml')
    traversability_footprint_params = config_file(
        'traversability_robot_footprint.yaml'
    )
    traversability_filter_params = config_file('traversability_robot_filter.yaml')
    coordinator_params = config_file('coordinator.yaml')
    art_params = config_file('art_local.yaml')
    pursuit_params = config_file('pure_pursuit_local.yaml')
    bridge_params = config_file('go2_bridge_local.yaml')

    unitree_runtime_environment = SetEnvironmentVariable(
        'LD_LIBRARY_PATH',
        [
            '/opt/unitree_robotics/lib',
            ':',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ],
    )

    open3d_global_localization = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='both',
        parameters=[open3d_loc_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
    )

    open3d_localization_service = Node(
        package='open3d_loc',
        executable='localization_service_node',
        name='localization_service_node',
        output='both',
        parameters=[open3d_loc_params, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_open3d_loc),
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
        condition=IfCondition(start_go2_bridge),
    )

    startup_summary = LogInfo(
        msg=[
            '[pct_art bringup] launching PCT -> FastDEM -> Traversability -> '
            'ART -> Coordinator -> Pure Pursuit with config/',
            CONFIG_NAME,
            '. Open3D localization start=',
            start_open3d_loc,
            ', Go2 bridge start=',
            start_go2_bridge,
            '.',
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_open3d_loc', default_value='true'),
        DeclareLaunchArgument('start_pct_planner', default_value='true'),
        DeclareLaunchArgument('start_go2_bridge', default_value='false'),
        DeclareLaunchArgument(
            'network_interface',
            default_value='enp2s0',
            description='Network interface connected to the Go2',
        ),
        DeclareLaunchArgument(
            'pct_params_file',
            default_value=config_file('pct_global_planner.yaml'),
        ),
        unitree_runtime_environment,
        startup_summary,
        log_process_start(
            open3d_global_localization,
            'open3d global localization (/Odometry_open3d)',
        ),
        log_process_start(open3d_localization_service, 'open3d localization service'),
        log_process_start(pct_planner, 'pct_global_planner (/pct_path)'),
        log_process_start(fastdem, 'fastdem (/fastdem/mapping/gridmap)'),
        log_process_start(
            traversability,
            'traversability_estimation (/traversability_map)',
        ),
        log_process_start(
            art_planner,
            'art_planner (/art_planner/path, /art_planner/plan_to_goal)',
        ),
        log_process_start(
            coordinator,
            'pct_art_coordinator (/local_goal, /local_path)',
        ),
        log_process_start(pure_pursuit, 'pure_pursuit_node (/cmd_vel)'),
        log_process_start(
            go2_bridge,
            'go2_cmd_vel_bridge (/go2_cmd_vel_bridge/enable)',
        ),
        open3d_global_localization,
        open3d_localization_service,
        pct_planner,
        fastdem,
        traversability,
        art_planner,
        coordinator,
        pure_pursuit,
        # go2_bridge,
    ])
