# maze_robot/launch/maze_navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    maze_share = get_package_share_directory('maze_robot')
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(maze_share, 'launch', 'world_robot.launch.py')
        )
    )

    map_node = Node(
        package='maze_robot',
        executable='maze_map_publisher',
        parameters=[{
            'sdf_path': os.path.join(maze_share, 'worlds', 'maze.world'),
            'model_name': 'Maze',
            'resolution': 0.1,
        }],
        output='screen',
    )

    planner = Node(
        package='maze_robot',
        executable='path_planner_node',
        output='screen',
    )

    follower = Node(
        package='maze_robot',
        executable='path_follower_node',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
    )

    return LaunchDescription([
        world_launch,
        map_node,
        planner,
        follower,
        rviz,
    ])
