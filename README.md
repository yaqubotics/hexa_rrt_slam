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

The program should look like this:

[![Watch the video](https://img.youtube.com/vi/PnuEB_RsXnw/maxresdefault.jpg)](https://youtu.be/PnuEB_RsXnw)

References:
[1] LaValle, Steven M. "Rapidly-exploring random trees: A new tool for path planning." (1998).
[2] Kohlbrecher, Stefan, Oskar Von Stryk, Johannes Meyer, and Uwe Klingauf. "A flexible and scalable slam system with full 3d motion estimation." In 2011 IEEE International Symposium on Safety, Security, and Rescue Robotics, pp. 155-160. IEEE, 2011.
