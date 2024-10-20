# ros2_ws description

ROS2 workspace to visualize the robot.

* /quadruped_simulation package:
  * I has node to hear topics from the robot and publish to simulator
  * .rviz config file
  * .urdf model file


## Launch RViz:

```
ros2 launch quadruped_simulation rviz_launch.py
```


## From root workspace ROS 2 folder:

```
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

#
