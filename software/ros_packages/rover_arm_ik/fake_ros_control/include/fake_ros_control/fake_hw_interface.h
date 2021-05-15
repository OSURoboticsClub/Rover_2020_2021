#ifndef FAKE_ROS_CONTROL_FAKE_HW_INTERFACE
#define FAKE_ROS_CONTROL_FAKE_HW_INTERFACE

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
#include <time.h>

class FakeHWInterface : public hardware_interface::RobotHW {
public:
    FakeHWInterface(); //default constructor
    FakeHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model = NULL);
    ~FakeHWInterface();

    void init();
    void write(ros::Time &Time, ros::Duration &elapsed_time);
    void registerJointLim(const hardware_interface::JointHandle &joint_handle_position, int jn); //function that ensures joints are limited
    void update(); //function responsible for calling read/write
    void run(); //function that runs the main loop
protected:
    void fakePosControl(ros::Duration &elapsed_time, int jn); //function for spoofing "joint feedback"- takes care of "read" as well

    /*begin copied values from real hw interface */
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
    double p_error; //double for diff between prev + current joint pos
    bool has_joint_limits;

    //vectors for storing joint information
    std::vector<std::string> joint_names_;
	std::vector<double> joint_pos_;
	std::vector<double> joint_vel_;
	std::vector<double> joint_eff_;
	std::vector<double> joint_pos_comm_;
    std::vector<double> joint_pos_prev_;

    //vectors for storing joint limits
    std::vector<double> joint_pos_ll;
    std::vector<double> joint_pos_ul;

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

};
#endif
