# RoboticsND-ChaseIt
Implement robot with camera sensor in ROS/Gazebo and make it chase white ball placed in simulation.

# Setup

```
mkdir -p ChaseIt/catkin_ws/src
cd ChaseIt/catkin_ws/src
git clone https://github.com/max-kazak/RoboticsND-ChaseIt.git .
catkin_init_workspace
cd ..
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
