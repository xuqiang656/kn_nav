from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('art_planner_motion_cost'),
        'config',
        'config.yaml',
    )

    return LaunchDescription([
        Node(
            package='art_planner_motion_cost',
            executable='cost_query_server.py',
            name='motion_cost_server',
            output='screen',
            parameters=[params],
            remappings=[
                ('~/map', '/elevation_mapping/elevation_map_raw'),
            ],
        ),
    ])
