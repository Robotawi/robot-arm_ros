# Prepare the workspace
```sh
catkin_make 
source ./sevel/setup.bash
```

# Use the robot
```
roslaunch simple_arm robot_spawn.launch
```
The system uses two topics to track the image seem by the arm camera, and to detect of the arm is stopped. The topics are /rgb_camera/image_raw and /simple_arm/joint_states

To view the camera output
```
rqt_image_view /rgb_camera/image_raw
```

If the arm is seeing a unifrom grey image, and it is stopped (meaning looing up at the sky of gazebo), then it is tasked to move in another way. 

To make the arm to look up at the sky, we can call the service from the command linse as follows
```sh
rosservice call /arm_mover/safe_move "joint_1: 0.0
joint_2: 0.0"
```

Upon this look_up move, the sky is detected and the arm is moved not to look at the sky.


**Resources about how to make the robot model from scratch**
- [Gazebo ROS urdf](http://gazebosim.org/tutorials?tut=ros_urdf)
- [Gazebo in 5 mins] 001 - How To Launch Your [First Gazebo World Using ROS](https://www.youtube.com/watch?v=qi2A32WgRqI)

## To learn how to build world files
1. [http://gazebosim.org/tutorials?cat=build_world](http://gazebosim.org/tutorials?cat=build_world).
2. [Tutorial: Using roslaunch to start Gazebo, world files and URDF models](http://gazebosim.org/tutorials?tut=ros_roslaunch&cat=connect_ros).
3. Check what is inside gazebo [empty_world.launch](https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_ros/launch/empty_world.launch) file. This file is has important defintions that are inherited by our created world. 