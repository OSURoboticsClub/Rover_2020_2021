#include <fake_ros_control/fake_hw_interface.h>

//constructors + deconstructor

FakeHWInterface::FakeHWInterface() {
    //default constructor- not used
}

FakeHWInterface::FakeHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model) : nh_(nh) {

    if (rover_arm_urdf_ == NULL) {
        getURDF(nh_, "robot_description");
    }
    else {
        rover_arm_urdf_ = urdf_model;
    }
    
    nh_.getParam("/rover_arm/fake_hw_interface/joints", joint_names_); //get list of joints on the arm

    if (joint_names_.size() == 0){ //checks to see if joint list is empty/empty file
        ROS_FATAL_STREAM_NAMED("init", "Cannot find required parameter '/rover_arm/fake_hw_interface/joints' "
        "on the parameter server.");
    }
}

FakeHWInterface::~FakeHWInterface() {
    //destructor 
}