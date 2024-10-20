# ros2_ws

# Description

ROS2 workspace to visulize the robot. 
I has nodes to hear topics and publish to simulator


# From root workspace ROS 2 folder:

source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

# Launch RViz:
```
ros2 launch quadruped_simulation rviz_launch.py
```