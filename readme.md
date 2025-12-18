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
source ~/mazebot_ws/install/setup.bash
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
