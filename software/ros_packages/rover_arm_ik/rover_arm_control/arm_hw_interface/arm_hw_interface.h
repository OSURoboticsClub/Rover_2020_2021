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
#include <rover_arm_control/IKControlMessage.h>

namespace arm_hw_interface {

class ArmHWInterface : public hardware_interface::RobotHW {
public:
    ArmHWInterface(); //default constructor hw interface
    ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL); //constructor for hw interface - sets up basic params
    ~ArmHWInterface(); //destructor for hw interface

    void init(); //main function for setting up + registering controllers
    void write(ros::Time &Time, ros::Duration &elapsed_time);
    void read(ros::Time &Time, ros::Duration &elapsed_time);
    void run(); //function that runs the main loop
    void stop(); //function that stops the hw interface + shuts down controllers
    void update(); //function responsible for calling read/write
    void registerJointLim(const hardware_interface::JointHandle &joint_handle_position, int jn); //function that ensures joints are limited
    void enforceLimits(ros::Duration &period); //function to enforce all joint limits before writing out

protected:
    //Node handle
    ros::NodeHandle nh_;

    //Interfaces for ROS Control
    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::PositionJointInterface pos_joint_interface_;
    hardware_interface::JointHandle pos_jnt_handle_;

    //Interfaces for Joint Limits - Saturation
    joint_limits_interface::PositionJointSaturationInterface pos_jnt_sat_interface_;

    //Interfaces for Joint Limits - Soft
    joint_limits_interface::PositionJointSoftLimitsInterface pos_jnt_soft_limits_;

    //Constants
    unsigned int n_joints_;
    unsigned int start_joint_ = 1;
    double BILLION = 1000000000.0; //convert seconds elapsed to nanoseconds
    bool has_joint_limits;
    
    //vectors for storing joint information
    std::vector<std::string> joint_names_;
	std::vector<double> joint_pos_;
	std::vector<double> joint_vel_;
	std::vector<double> joint_eff_;
	std::vector<double> joint_pos_comm_;

    //vectors for storing joint limits
    std::vector<double> joint_pos_ll;
    std::vector<double> joint_pos_ul;

    //variable that allows us to interface with ionis
    ArmState arm_;

    //urdf variable for grabbing pos limits + function
    urdf::Model *rover_arm_urdf_ = NULL;
    void getURDF(const ros::NodeHandle& nh, std::string param_name);

    //variables for controller manager/timing
    double loop_hz; //variable for controlling freq of control loop
    double error_threshold; //variable for timeout for timer
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
    ros::Duration update_freq;
    ros::Duration elapsed_time;
    struct timespec last_time_;
    struct timespec current_time_;

    //msg string
    const default_ik_controls_topic = "IKControl/control_status";
    const default_ik_status_topic = "IKControl/button_status";

    //booleans for start and stop
    bool controllers_started, controllers_stopped;
    bool start_button_pushed, stop_button_pushed;
};

}

#endif