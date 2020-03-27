# RoboticsND-ChaseIt
Implement robot with camera sensor in ROS/Gazebo and make it chase white ball placed in simulation.

# Setup

```
mkdir ChaseIt
mkdir -p catkin_ws/src
cd catkin_ws
git clone https://github.com/max-kazak/RoboticsND-ChaseIt.git catkin_ws/src
catkin_make
```

# Run code
Terminal1 (creates simulation):
```
cd catkin_ws
source devel/setup.bash
roslaunch my_robot world.launch
```

Terminal2 (runs robot logic):
```
cd catkin_ws
source devel/setup.bash
roslaunch ball_chaser ball_chaser.launch
```

Terminal3 (for visualization purposes):
```
cd catkin_ws
source devel/setup.bash
rosrun rqt_image_view rqt_image_view 
```
