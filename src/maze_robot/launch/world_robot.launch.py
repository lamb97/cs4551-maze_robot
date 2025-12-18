# maze_robot/launch/world_robot.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    maze_robot_share = get_package_share_directory('maze_robot')
    world_path = PathJoinSubstitution([maze_robot_share, 'worlds', 'maze.world'])
    local_models = os.path.join(maze_robot_share, 'models')

    demos_share = get_package_share_directory('ros_gz_sim_demos')
    demos_models = os.path.join(demos_share, 'models')
    tb3_share = get_package_share_directory('turtlebot3_gazebo')
    tb3_models = os.path.join(tb3_share, 'models')
    tb3_description_share = get_package_share_directory('turtlebot3_description')
    tb3_urdf = os.path.join(
        tb3_description_share,
        'urdf',
        'turtlebot3_burger.urdf'
    )
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    resource_path = ':'.join(
        p for p in [
            existing,
            maze_robot_share,
            local_models,
            demos_models,
            tb3_models,
        ] if p
    )

    set_gz_resource = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path,
    )

    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', world_path],
        output='screen'
    )

    with open(tb3_urdf, 'r') as urdf_file:
        tb3_urdf_content = urdf_file.read()
    robot_description = ParameterValue(tb3_urdf_content, value_type=str)
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )
    static_map_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(maze_robot_share, 'config', 'tb3_bridge.yaml'),
            'expand_gz_topic_names': True,
        }],
        output='screen',
    )
    odom_tf = Node(
        package='maze_robot',
        executable='odom_tf_broadcaster',
        output='screen',
    )

    return LaunchDescription([
        set_gz_resource,
        gz_sim,
        state_publisher,
        static_map_tf,
        bridge,
        odom_tf,
    ])
