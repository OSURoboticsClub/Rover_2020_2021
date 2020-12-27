#include "arm_hw_interface.h"

ArmHWInterface::ArmHWInterface(ros::NodeHandle& nh) {
    nh_.get_param("arm_joint_names.yaml", joint_names_); //get list of joints on the arm

    if (joint_names_.size() == 0){ //checks to see if joint list is empty/empty file
        ROS_ERROR("Cannot find required parameter 'arm_joint_names' "
        "on the parameter server.");
        exit(-1);
    }
    
    n_joints_ = joint_names_.size(); //sets number of joints in list to variable

    /* initializing controllers for each joint */

    for(unsigned int i = 0; i < num_joints_; ++i) {
        /* init joint state interface for each joint */
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i])); /* TODO: add vectors for pos, vel, effort */

        /* init position interface for each joint */
        pos_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]))); /* TODO add vector for cmds */

        /* register interfaces */
        registerInterface(&joint_state_interface_);
        registerInterface(&pos_joint_interface_);
    }
}

void ArmHWInterface::write() {
    /* add write functionality + hw specific code here */
}

void ArmHWInterface::read() {
    /* add read functionality + hw specific code here */
}