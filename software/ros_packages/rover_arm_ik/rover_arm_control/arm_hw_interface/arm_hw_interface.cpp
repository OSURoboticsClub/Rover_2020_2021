#include "arm_hw_interface.h"
#include "arm_state.h"

namespace arm_hw_interface {

ArmHWInterface::ArmHWInterface() {
    //default constructor- not used
}

ArmHWInterface::ArmHWInterface(ros::NodeHandle& nh) : nh_(nh) {
    
    nh_.getParam("/rover_arm/arm_hw_interface/joints", joint_names_); //get list of joints on the arm

    if (joint_names_.size() == 0){ //checks to see if joint list is empty/empty file
        ROS_FATAL_STREAM_NAMED("init", "Cannot find required parameter '/rover_arm/arm_hw_interface/joints' "
        "on the parameter server.");
    }

    n_joints_ = joint_names_.size(); //sets number of joints in list to variable

    /* resize vectors to be the size of how many joints are on the robot */
    joint_pos_.resize(n_joints_);
    joint_eff_.resize(n_joints_);
    joint_vel_.resize(n_joints_);
    joint_pos_comm_.resize(n_joints_);

    /* initializing controllers for each joint */
    for(int i = 0; i < n_joints_; ++i) {
        /* init joint state interface for each joint */
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i])); 

        /* init position interface for each joint */
        pos_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_pos_comm_[i])); 
    }

    /* register interfaces */
    registerInterface(&joint_state_interface_);
    registerInterface(&pos_joint_interface_);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_)); //create new controller manager

    //Set the frequency of the control loop + error threshold for timeout.
    loop_hz = 10;
    error_threshold = 0.01;

    // Get current time for use with first update
    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    
    //set update frequency for control loop
    update_freq = ros::Duration(1/loop_hz);
}

ArmHWInterface::~ArmHWInterface() {
    //destructor 
}

void ArmHWInterface::write(ros::Time &Time, ros::Duration &elapsed_time) {
    arm_.set_joint_positions(joint_pos_comm_); /* send joint positions off to hardware */
    arm_.constrain_set_positions(); /* makes sure joint positions are within constraints */
}

void ArmHWInterface::read(ros::Time &Time, ros::Duration &elapsed_time) {
    std::vector<double> pos, vel, torque;
    /* read in current joint values into vectors */
    arm_.get_joint_velocities(vel); 
    arm_.get_joint_effort(torque);
    arm_.get_joint_positions(pos);

    for(int i = 0; i < n_joints_; ++i){
        joint_pos_[i] = pos[i];
        joint_eff_[i] = torque[i];
        joint_vel_[i] = vel[i];
    }
}

void ArmHWInterface::update() {
    /* gets elapsed time difference */
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    elapsed_time =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
    last_time_ = current_time_; /* updates timing variables for next loop */
    ros::Time now = ros::Time::now();

    read(now, elapsed_time); /* read in the joint states */
    controller_manager_->update(now, elapsed_time); /* update controller manager */
    write(now, elapsed_time); /* write out new joint states */
}

void ArmHWInterface::run()
{
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    update();
    rate.sleep();
  }
}

}