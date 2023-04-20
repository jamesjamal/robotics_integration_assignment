# robotics_integration_assignment
# TII
## This repository contains MPC controller for polaris gem austonamous vehicle
### Under developemnt

### Test video of the simulation [Testing video] (https://drive.google.com/drive/folders/1hIaL2AcyNDBDJBECRpzFxI7WqM6XHQgE?usp=share_link)

### Requirements for mpc controller

Check the reuirement.txt files for python depedencies

### Polaris GEM e2 Simulator
This repository provides a simulated vehicle model of Polaris GEM e2 Electric Cart in the Gazebo simulation environment as well as ROS based sensors and controllers for autonomous-driving. The Polaris GEM e2 vehicle model was measured and modeled using Solidworks by Hang Cui and Jiaming Zhang. Hang Cui further constructed the URDF files of the vehicle model compatible with ROS, RViz, and Gazebo.
The simulator was initially developed for personal research with ROS Melodic and Gazebo 9 in Ubuntu 18.04 in Fall 2019. This simulator then became an essential teaching material for the course, Principles of Safe Autonomy @ Illinois, and the project subsequently received funding from the Center for Autonomy at University of Illinois at Urbana-Champaign. Hang Cui further developed and upgraded the simulator to use ROS Noetic and Gazebo 11 in Summer 2021. This simulator is currently under active development for research and teaching.

### Requirements
Our simulation setup is currently tested only with the following system and ROS packages.
System: Ubuntu 20.04 + ROS Noetic (Gazebo 11)
We refer readers to http://wiki.ros.org/noetic/Installation/Ubuntu and follow the instructions to install ROS noetic and Gazebo 11.
We also recommend Desktop-Full Install as suggested in the instructions.
Required ROS Packages:

**ackermann_msgs
geometry2
hector_gazebo
hector_models
jsk_rviz_plugins
ros_control
ros_controllers
velodyne_simulator**

After the installation of ROS Noetic and Gazebo 11 on Ubuntu 20.04, we recommend installing ROS packages using APT as follows
```
 $ sudo apt install ros-noetic-ackermann-msgs ros-noetic-geometry2 \
    ros-noetic-hector-gazebo ros-noetic-hector-models ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-velodyne-simulator
```
Compile Polaris GEM e2 Simulator
We assume the Catkin workspace is under ~/gem_ws. We first clone this repository to ~/gem_ws/src.
For example,

```
$ mkdir -p ~/gem_ws/src
$ cd ~/gem_ws/src
$ git clone (https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2.git)
Then we compile the whole workspace use catkin_make
```

```
$ source /opt/ros/noetic/setup.bash
$ cd ~/gem_ws
$ catkin_make
```

For more detail about Catkin workspace, please refer to the tutorials at (http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Usage

Simple Track Environment

```
$ source devel/setup.bash
$ roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true"
```



