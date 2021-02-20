#ifndef ARM_ROS_CONTROL_ARM_HW_INTERFACE_H
#define ARM_ROS_CONTROL_ARM_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include "arm_state.h"

namespace arm_hw_interface {

class ArmHWInterface : public hardware_interface::RobotHW {
public:
    ArmHWInterface(); //default constructor hw interface
    ArmHWInterface(ros::NodeHandle& nh); //constructor for hw interface- registers controllers
    ~ArmHWInterface(); //destructor for hw interface

    void write();
    void read();
    void update(const ros::TimerEvent& e); //function responsible for calling read/write

protected:
    //Node handle
    ros::NodeHandle nh_;

    //Interfaces for ROS Control
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface pos_joint_interface_;

    //Constants
    unsigned int n_joints_;
    unsigned int start_joint_ = 1;
    double loop_hz; //variable for controlling freq of control loop

    //vectors for storing joint information
    std::vector<std::string> joint_names_;
	std::vector<double> joint_pos_;
	std::vector<double> joint_vel_;
	std::vector<double> joint_eff_;
	std::vector<double> joint_pos_comm_;

    //variable that allows us to interface with ionis
    ArmState arm_;

    //variables for controller manager/timing
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::Timer arm_control_loop;
	ros::Duration control_period_;
	ros::Duration elapsed_time_;

};

}

#endif