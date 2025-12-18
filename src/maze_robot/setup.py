import os
from setuptools import find_packages, setup

package_name = 'maze_robot'

def collect_model_data():
    model_entries = []
    for root, _, files in os.walk('models'):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        sources = [os.path.join(root, f) for f in files]
        model_entries.append((install_dir, sources))
    return model_entries

model_data_files = collect_model_data()

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        ('share/' + package_name + '/launch', [
            'launch/world_robot.launch.py',
            'launch/maze_navigation.launch.py',
        ]),

        ('share/' + package_name + '/worlds', [
            'worlds/maze.world',
        ]),
        ('share/' + package_name + '/config', [
            'config/tb3_bridge.yaml',
            'config/maze_nav.rviz',
        ]),
    ] + model_data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='liu03222',
    maintainer_email='liu03222@umn.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'maze_map_publisher = maze_robot.maze_map_publisher:main',
            'path_planner_node = maze_robot.path_planner_node:main',
            'path_follower_node = maze_robot.path_follower_node:main',
            'odom_tf_broadcaster = maze_robot.odom_tf_broadcaster:main',
        ],
    },
)
