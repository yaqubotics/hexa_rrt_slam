# hexa_rrt_slam

Do these following commands:
1. Clone this project to your ROS catkin folder then compile it:
https://github.com/yaqubotics/px4_simulation_stack/tree/hexa_rrt_slam

2. Clone and setup PX4 Firmware project outside your ROS catkin folder:
https://github.com/yaqubotics/Firmware/tree/cc_uavs

3. Run the Gazebo and spawn the hexacopter:
```
roslaunch px4_simulation_stack hexa_j153.launch
```

4. Run the obstacle avoidance node:
```
rosrun obstacle_avoidance obstacle_avoidance.py 1
```

[![Watch the video](https://img.youtube.com/vi/PnuEB_RsXnw/maxresdefault.jpg)](https://youtu.be/PnuEB_RsXnw)
