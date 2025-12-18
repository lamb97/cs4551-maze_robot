# CSCI 4551 Project

## Member:
Yang Liu, Haotian Zhai, Brian Vo

## Building
```
mkdir -p ~/mazebot_ws 
cd ~/mazebot_ws
mkdir src
cd src 
git clone https://github.com/lamb97/cs4551-maze_robot.git
cd ..
colcon build
<<<<<<< HEAD
source ~/mazebot_ws/install/setup.bash
=======
source install/setup.bash
```
## Mazesolve
```
#Terminal 1
ros2 run maze_robot maze_map_publisher --ros-args   -p sdf_path:="your world path"   -p resolution:=0.1   -p model_name:="Maze or maze_2"
#Terminal 2
ros2 launch maze_robot world_turtlebot3.launch.py
#Terminal 3 
ros2 run maze_robot path_planner_node
#Terminal 4 
#open a new terminal
rviz2
 
>>>>>>> cb5a2361f2081ee4f9de5db2b20f69b0d3f4655c
```

## World2Grid
```
# Terminal 1
# Default (maze2.world)
ros2 launch maze_robot maze_navigation.launch.py

# Choose maze.world
ros2 launch maze_robot maze_navigation.launch.py maze_world:=maze.world

# Choose maze2.world
ros2 launch maze_robot maze_navigation.launch.py maze_world:=maze2.world

#Terminal 2 
#open a new terminal
rviz2
 
```

![Demo](./Images/world2grid.png)
