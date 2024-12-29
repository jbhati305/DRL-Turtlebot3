# DRL-Turtlebot3
I have created this project to implement deep reinforcement learning for autonomous navigation and exploration using turtlebot3_waffle and ROS2-Jazzy with Gazebo Harmonic. 



```
cd ~/DRL-Turtlebot3
colcon build --symlink-install
. install/setup.bash
```

For spawning Robot:
```
ros2 launch tb3_simulation spawn_robot.launch.py
```
For initiating Navigation:
```
ros2 launch tb3_navigation navigation_with_slam.launch.py
```

![Alt text](images/gazebo.png)
![Alt text](images/rviz1.png)
![Alt text](images/rviz2.png)

### TO-DO:
1. Implementing EKF by myself in cpp.
2. Implement SLAM in CPP.

