# ER_ROBO(using Hospital Robot)
- ROS noetic(Ubuntu 20.04)


---
## Slam
### Terminal 1
    roslaunch myrobot_slam myrobot_slam.launch
### Terminal 2
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
Done
### Terminal 2
    rosrun map_server map_saver
---
## Navigation
    roslaunch myrobot_navigation myrobot_static_navigation.launch
