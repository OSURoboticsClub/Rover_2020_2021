# Rover Arm IK Packages Overview

This document provides a brief overview of the main software packages responsible for autonomous operation of the arm. This document aims to inform members about the structure an purpose of each package, as well as how to launch different files for simulation and control. Two main libraries allow us the functionality to preform autonomous operations with the arm: ROS_Control and Moveit!. Moveit! provides us with a number of different plugins and packages for simulating motion, motion planning, and improved kinematics on our arm. ROS Control provides us with the framework to connect the planned motions from Moveit! libraries with the actual hardware of the arm. In the sections below, a more in depth overview of how these two libraries function is presented.

## Package Descriptions

### rover_arm_control

The ```rover_arm_control``` package encompasses the code responsible for linking simulated/planned motions to real robot hardware. It is the implementation of the ROS Control library. The main pieces of software in this package are the hardware interface, and the joint controllers. Joint controllers are the ROS nodes responsible for keeping track of information about the joints of the arm. Joint controllers in the simplest sense, are what are responsible for taking commands from simulated/planned motion and executing it on the physical arm. The hardware interface is repsonsible for providing these controllers with the capability to interface with the real hardware of the arm.

### rover_arm_description

The ```rover_arm_description``` package is what stores the URDF model for the arm. The URDF model file is what contains information about limits and physical characteristics of every joint on the arm. It is what tells the simulator and the control package how the joints are linked together and how they interact with one another. The files in this package are used by many of the packages for arm autonomy.

### rover_arm_gazebo

The ```rover_arm_gazebo``` package is responsible for taking the URDF model of the arm and launching it in Gazebo- a full 3D physics simulator. Gazebo provides a good approximation of real life behavior of the arm through simulation. This package contains joint controllers that publish false joint data for simulation purposes, so the simulation here is not connected to actual hardware. 

### rover_arm_moveit_config

### stomp_core

### stomp_moveit

### joint_parser

## Simulating Arm Control Using RViz and Gazebo

## Running the Arm With ROS Control

## References

### General References for Moveit! and Gazebo

[Official Moveit! Documentation](https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/index.html)
[Official Gazebo Documentation for ROS](http://gazebosim.org/tutorials?cat=connect_ros)


### ROS Control Resources

[ROS Control Boilerplate Code](https://github.com/PickNikRobotics/ros_control_boilerplate/tree/kinetic-devel) by PikNik Robotics 
[ROS Control Tutorial](https://medium.com/%40slaterobotics/how-to-implement-ros-control-on-a-custom-robot-748b52751f2e) by Slate Robotics 
[Official ROS Control Repository](https://github.com/ros-controls/ros_control)

### Inverse Kinematics Resources

[Documentation for using Moveit! and IKFAST](https://docs.ros.org/en/kinetic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html)
[Offical OpenRAVE documentation](http://www.openrave.org/docs/latest_stable/)
