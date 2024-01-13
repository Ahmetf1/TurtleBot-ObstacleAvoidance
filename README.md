# EE 451 Final Project - Velocity Kinematics of a PPP Robot

## How to Run in Simulation ?

```
cd <package_path>
catkin_make
source devel/setup.bash
roslaunch turtlebot spawn_robot.launch
```

Specify the color you want to follow and avoid in the launch files.
Then move the robot !

```
cd <package_path>
source devel/setup.bash
rosrun turtlebot test_BSF
```


## How to Run in a Real Robot ?
Specify the color you want to follow and avoid in the launch files.
Then move the robot !

```
cd <package_path>
catkin_make
source devel/setup.bash
roslaunch turtlebot real_robot.launch
```

```
roslaunch freenect_launch freenect.launch
```

```
cd <package_path>
source devel/setup.bash
rosrun turtlebot create_bringup create_1.launch
```

```
cd <package_path>
source devel/setup.bash
rosrun turtlebot follow_person
```


## Dependecy
Run the commands below to install depencesies
```
sudo apt-get install ros-<ROS_DISTRO>-ros-control ros-noetic-ros-controllers
sudo apt-get install ros-<ROS_DISTRO>-controller-manager
sudo apt-get install ros-<ROS_DISTRO>-gazebo-ros-control
sudo apt-get install ros-<ROS_DISTRO>-gazebo-ros
```

## Tested Platform
The project is tested on Ubuntu 20.04 with Gazebo 11 and ROS Noetic in simulation
The project is tested on Ubuntu 14 with Turtlebot 1.0
