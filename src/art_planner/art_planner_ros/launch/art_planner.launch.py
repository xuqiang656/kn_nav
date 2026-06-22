from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    planner_params = os.path.join(
        get_package_share_directory('art_planner_ros'),
        'config',
        'params.yaml',
    )
    motion_cost_params = os.path.join(
        get_package_share_directory('art_planner_motion_cost'),
        'config',
        'config.yaml',
    )

    return LaunchDescription([
        Node(
            package='art_planner_ros',
            executable='art_planner_ros_node',
            name='art_planner',
            output='screen',
            parameters=[planner_params],
            remappings=[
                ('~/elevation_map', '/traversability_map'),
                ('~/cost_query', '/art_planner/motion_cost_server/cost_query'),
                ('~/cost_query_no_update', '/art_planner/motion_cost_server/cost_query_no_update'),
            ],
        ),
        GroupAction([
            PushRosNamespace('art_planner'),
            # Node(
            #     package='art_planner_motion_cost',
            #     executable='cost_query_server.py',
            #     name='motion_cost_server',
            #     output='screen',
            #     parameters=[motion_cost_params],
            #     remappings=[
            #         ('~/map', '/elevation_mapping_node/elevation_map_raw'),
            #     ],
            # ),
            Node(
                package='art_planner_ros',
                executable='plan_to_goal_client.py',
                name='plan_to_goal_client',
                output='screen',
            ),
        ]),
    ])
