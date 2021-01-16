#ifndef ARM_ROS_CONTROL_ARM_HW_INTERFACE_H
#define ARM_ROS_CONTROL_ARM_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <vector>
#include "arm_state.h"

namespace arm_ros_control {

class ArmHWInterface : public hardware_interface::RobotHW {
public:
    ArmHWInterface(ros::NodeHandle& nh); //constructor for hw interface- registers controllers
    ~ArmHWInterface(); //destructor for hw interface

    /* might have to edit the types/args of these functions depending what's in them- for now they're void */
    void write();
    void read();

private:
    //Node handle
    ros::NodeHandle nh_;

    //Interfaces for ROS Control
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface pos_joint_interface_;

    unsigned int n_joints_;

    std::vector<std::string> joint_names_;
	std::vector<double> joint_pos_;
	std::vector<double> joint_vel_;
	std::vector<double> joint_eff_;
	std::vector<double> joint_pos_comm_;

    //arm state variable
    ArmState arm_;

};

}

#endif