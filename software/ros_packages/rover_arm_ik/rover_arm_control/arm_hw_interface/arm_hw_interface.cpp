#include "arm_hw_interface.h"
#include "arm_state.h"

namespace arm_ros_control {

ArmHWInterface::ArmHWInterface(ros::NodeHandle& nh) : nh_(nh) {
    nh_.getParam("arm_joint_names.yaml", joint_names_); //get list of joints on the arm

    if (joint_names_.size() == 0){ //checks to see if joint list is empty/empty file
        ROS_ERROR("Cannot find required parameter 'arm_joint_names' "
        "on the parameter server.");
        exit(-1);
    }
    
    n_joints_ = joint_names_.size(); //sets number of joints in list to variable

    /* resize vectors to be the size of how many joints are on the robot */
    joint_pos_.resize(n_joints_);
    joint_eff_.resize(n_joints_);
    joint_vel_.resize(n_joints_);
    joint_pos_comm_.resize(n_joints_);

    /* initializing controllers for each joint */

    for(unsigned int i = 0; i < n_joints_; ++i) {
        /* init joint state interface for each joint */
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i])); 

        /* init position interface for each joint */
        pos_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_pos_comm_[i])); 

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&pos_joint_interface_);
    }
}

void ArmHWInterface::write() {
    /* add write functionality + hw specific code here */
}

void ArmHWInterface::read() {
    std::vector<double> pos, vel, current;
}

}