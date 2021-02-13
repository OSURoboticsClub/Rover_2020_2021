#include "arm_hw_interface.h"
#include "arm_state.h"

namespace arm_hw_interface {

ArmHWInterface::ArmHWInterface() {
    //default constructor- not used
}

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

ArmHWInterface::~ArmHWInterface() {
    //destructor 
}

void ArmHWInterface::write() {
    arm_.set_joint_positions(joint_pos_comm_); /* send joint positions off to hardware */
    arm_.constrain_set_positions(); /* makes sure joint positions are within constraints */
}

void ArmHWInterface::read() {
    std::vector<double> pos, vel, torque;
    /* read in current joint values into vectors */
    arm_.get_joint_velocities(vel); 
    arm_.get_joint_effort(torque);
    arm_.get_joint_positions(pos);

    for(int i = start_joint_; i < n_joints_; ++i){
        for(int j = 0; j < 6; j++) {
            joint_pos_[i] = pos[j];
            joint_eff_[i] = torque[j];
            joint_vel_[i] = vel[j];
        }
    }
}

}