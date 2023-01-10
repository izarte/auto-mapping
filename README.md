# Auto mapping

Auto-mapping is a ros noetic package for auto exploration and mapping of enviroment based on front barrier

# Prerequisites
    - Docker

# Usage
Create a new worskspace in this folder or add src content an existing src workspace.
In this case, it is assumed repository is inside a folder named catkin_ws.

In order to let container display graphical interface such as stage or rviz, it may be necessary to execute this command before container up:
```sh
xhost +
```

```sh
docker run -v <FOLDER_PATH>/catkin_ws:/catkin_ws --volume=/tmp/.X11-unix:/tmp/.X11-unix --env="DISPLAY" --net=host -it --rm --name ros_noetic inigo183/ros_noetic:odom /bin/bash
```
## Installation
In first use it is necesary to compile new packages, to do so, jut run this command inside docker container
```sh
cd catkin_ws
catkin make
```

## Execution
In each tab is needed to execute in catkin_ws workspace to load packages

### Simulation
```sh
source devel/setup.bash
```

Tab 1:
```sh
roslaunch auto_mapping stage.launch
```

Tab 2:
```sh
rosrun auto_mapping explore.py
```

### Real robot
Tab 1:
```sh
turtlebot_bringup.launch
```

Tab 2:
```sh
roslaunch turtlebot_bringup hokuyo_ust10lx.launch
```

Tab 3:
```sh
export TURTLEBOT_3D_SENSOR=astra
roslaunch autom_mapping amcl.launch
```

Tab 4:
```sh
roslaunch turtlebot_navigation gmapping_demo.launch
```

Tab 5:
```sh
rosrun auto_mapping explore.py
```
