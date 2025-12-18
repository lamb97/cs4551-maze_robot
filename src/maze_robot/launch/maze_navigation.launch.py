# maze_robot/launch/maze_navigation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    maze_share = get_package_share_directory('maze_robot')
    maze_world_arg = DeclareLaunchArgument(
        'maze_world',
        default_value='maze2.world',
        choices=['maze.world', 'maze2.world'],
        description='Which world file to load from the package `worlds/` directory.',
    )
    maze_world = LaunchConfiguration('maze_world')
    world_path = PathJoinSubstitution([maze_share, 'worlds', maze_world])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(maze_share, 'launch', 'world_robot.launch.py')
        ),
        launch_arguments={
            'maze_world': maze_world,
        }.items(),
    )

    map_node = Node(
        package='maze_robot',
        executable='maze_map_publisher',
        parameters=[{
            'sdf_path': ParameterValue(world_path, value_type=str),
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
        maze_world_arg,
        world_launch,
        map_node,
        planner,
        follower,
        rviz,
    ])
