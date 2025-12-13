# maze_robot/launch/world_turtlebot3.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command


def generate_launch_description():
    maze_robot_share = get_package_share_directory('maze_robot')
    world_path = os.path.join(maze_robot_share, 'worlds', 'maze.world')
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )
    tb3_xacro = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_waffle.urdf'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', tb3_xacro]),
            'use_sim_time': True,
        }]
    )
    spawn_tb3 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'turtlebot3',     
            '-topic', 'robot_description', 
            '-x', '0.0', '-y', '0.0', '-z', '0.1',
        ],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_tb3,
    ])
