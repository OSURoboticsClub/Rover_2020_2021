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
    void write();
protected:
    void fakePosControl(); //function for spoofing "joint feedback"