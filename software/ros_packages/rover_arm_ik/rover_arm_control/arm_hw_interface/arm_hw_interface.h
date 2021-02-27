#ifndef ARM_ROS_CONTROL_ARM_HW_INTERFACE_H
#define ARM_ROS_CONTROL_ARM_HW_INTERFACE_H

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>
#include <ros/ros.h>
#include <urdf/model.h>
#include "arm_state.h"
#include <time.h>

namespace arm_hw_interface {

class ArmHWInterface : public hardware_interface::RobotHW {
public:
    ArmHWInterface(); //default constructor hw interface
    ArmHWInterface(ros::NodeHandle& nh); //constructor for hw interface- registers controllers
    ~ArmHWInterface(); //destructor for hw interface

    void write(ros::Time &Time, ros::Duration &elapsed_time);
    void read(ros::Time &Time, ros::Duration &elapsed_time);
    void run(); //function that runs the main loop
    void update(); //function responsible for calling read/write
    void registerJointLim(const hardware_interface::JointHandle &joint_handle_position, std::size_t joint_names); //function that ensures joints are limited
    void enforceLimits(ros::Duration &period); //function to enforce all joint limits before writing out

protected:
    //Node handle
    ros::NodeHandle nh_;

    //Interfaces for ROS Control
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface pos_joint_interface_;

    //Interfaces for Joint Limits - Saturation
    joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;

    //Interfaces for Joint Limits - Soft
    joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;

    //Constants
    unsigned int n_joints_;
    unsigned int start_joint_ = 1;
    double BILLION = 1000000000.0; //convert seconds elapsed to nanoseconds
    
    //vectors for storing joint information
    std::vector<std::string> joint_names_;
	std::vector<double> joint_pos_;
	std::vector<double> joint_vel_;
	std::vector<double> joint_eff_;
	std::vector<double> joint_pos_comm_;

    //vectors for storing joint limits
    std::vector<double> joint_position_lower_limits_;
    std::vector<double> joint_position_upper_limits_;


    //variable that allows us to interface with ionis
    ArmState arm_;

    //variables for controller manager/timing
    double loop_hz; //variable for controlling freq of control loop
    double error_threshold; //variable for timeout for timer
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::Duration update_freq;
    ros::Duration elapsed_time;
    struct timespec last_time_;
    struct timespec current_time_;
};

}

#endif