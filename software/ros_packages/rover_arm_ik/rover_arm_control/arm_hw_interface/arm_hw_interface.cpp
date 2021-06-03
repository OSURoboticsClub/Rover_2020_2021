#include "arm_hw_interface.h"
#include "arm_state.h"

namespace arm_hw_interface {

/* constructors and destructor */

ArmHWInterface::ArmHWInterface() {
    //default constructor- not used
}

ArmHWInterface::ArmHWInterface(ros::NodeHandle& nh, urdf::Model* urdf_model) : nh_(nh) {
    if (rover_arm_urdf_ == NULL) {
        getURDF(nh_, "robot_description");
    }
    else {
        rover_arm_urdf_ = urdf_model;
    }
    
    nh_.getParam("/rover_arm/arm_hw_interface/joints", joint_names_); //get list of joints on the arm

    if (joint_names_.size() == 0){ //checks to see if joint list is empty/empty file
        ROS_FATAL_STREAM_NAMED("init", "Cannot find required parameter '/rover_arm/arm_hw_interface/joints' "
        "on the parameter server.");
    }
}

ArmHWInterface::~ArmHWInterface() {
    //destructor 
}

void ArmHWInterface::init() {
    n_joints_ = joint_names_.size(); //sets number of joints in list to variable

    /* resize vectors to be the size of how many joints are on the robot */
    joint_pos_.resize(n_joints_);
    joint_eff_.resize(n_joints_);
    joint_vel_.resize(n_joints_);
    joint_pos_comm_.resize(n_joints_);
    joint_pos_ll.resize(n_joints_);
    joint_pos_ul.resize(n_joints_);

    /* initializing controllers for each joint */
    for(int i = 0; i < n_joints_; ++i) {
        /* init joint state interface for each joint */
        joint_state_interface_.registerHandle(hardware_interface::JointStateHandle(joint_names_[i], &joint_pos_[i], &joint_vel_[i], &joint_eff_[i])); 

        /* init position interface for each joint */
        hardware_interface::JointHandle pos_jnt_handle_ = hardware_interface::JointHandle(joint_state_interface_.getHandle(joint_names_[i]), &joint_pos_comm_[i]);
        pos_joint_interface_.registerHandle(pos_jnt_handle_);
        

        /* register the joint limits of each joint */
        registerJointLim(pos_jnt_handle_, i);
    }

    /* register interfaces */
    registerInterface(&joint_state_interface_);
    registerInterface(&pos_joint_interface_);

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_)); //create new controller manager

    //Set the frequency of the control loop + error threshold for timeout.
    loop_hz = 300;
    error_threshold = 0.01;

    // Get current time for use with first update
    clock_gettime(CLOCK_MONOTONIC, &last_time_);
    
    //set update frequency for control loop
    update_freq = ros::Duration(1/loop_hz);

    ROS_INFO_STREAM_NAMED("Hardware Interface", "Arm HW interface ready");
}

/* write/read functions */

void ArmHWInterface::write(ros::Time &Time, ros::Duration &elapsed_time) {
    enforceLimits(elapsed_time);
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

/* joint limit functions */
void ArmHWInterface::registerJointLim(const hardware_interface::JointHandle &pos_joint_interface_, int jn){
  /*set default limits */
  joint_pos_ll[jn] = -std::numeric_limits<double>::max();
  joint_pos_ul[jn] = std::numeric_limits<double>::max();

  /* create data structures */
  joint_limits_interface::JointLimits joint_lim;
  //TODO Add functionality for soft limits as needed

  if(rover_arm_urdf_ == NULL){
        ROS_WARN_STREAM_NAMED("URDF: ", "No URDF model loaded, unable to get joint limits");
        return;
  }

  /* Get limits from URDF */
  urdf::JointConstSharedPtr arm_joint = rover_arm_urdf_->getJoint(joint_names_[jn]);

  if(arm_joint == NULL){
    ROS_ERROR_STREAM_NAMED("URDF: ", "URDF joint not found " << joint_names_[jn]);
    return;
  }

  if(joint_limits_interface::getJointLimits(arm_joint, joint_lim)){
    has_joint_limits = true;
    ROS_DEBUG_STREAM_NAMED("URDF: ", "Joint " << joint_names_[jn] << " has URDF position limits [" << joint_lim.min_position << ", " << joint_lim.max_position << "]");                          
  }
  else {
    if (arm_joint->type != urdf::Joint::CONTINUOUS){
      ROS_WARN_STREAM_NAMED("URDF: ", "Joint " << joint_names_[jn] << " does not have a URDF position limit");
    }
  }

  /* if we haven't found any joints, quit */
  if (!has_joint_limits){
    return;
  }

  /* Copy position limits if available */
  if (joint_lim.has_position_limits)
  {
    // Slighly reduce the joint limits to prevent floating point errors
    joint_lim.min_position += std::numeric_limits<double>::epsilon();
    joint_lim.max_position -= std::numeric_limits<double>::epsilon();

    joint_pos_ll[jn] = joint_lim.min_position;
    joint_pos_ul[jn] = joint_lim.max_position;
  }

    ROS_DEBUG_STREAM_NAMED("URDF", "Using saturation limits (not soft limits)");

    joint_limits_interface::PositionJointSaturationHandle sat_handle_position(pos_joint_interface_, joint_lim);
    pos_jnt_sat_interface_.registerHandle(sat_handle_position);
}

void ArmHWInterface::enforceLimits(ros::Duration &period){
    pos_jnt_sat_interface_.enforceLimits(period);
    pos_jnt_soft_limits_.enforceLimits(period);
}


/* urdf loading function */
void ArmHWInterface::getURDF(const ros::NodeHandle &nh, std::string param_name){
    std::string urdf_string;
    rover_arm_urdf_ = new urdf::Model();

    while(urdf_string.empty() && ros::ok()) {
        std::string search_param_name;    
        if (nh.searchParam(param_name, search_param_name)){
        ROS_INFO_STREAM_NAMED("URDF: ", "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << search_param_name);
            nh.getParam(search_param_name, urdf_string);
        }
        else{
        ROS_INFO_STREAM_NAMED("URDF: ", "Waiting for model URDF on the ROS param server at location: " << nh.getNamespace() << param_name);
            nh.getParam(param_name, urdf_string);
        }

        usleep(100000);
    }

    if (!rover_arm_urdf_->initString(urdf_string)) {
        ROS_ERROR_STREAM_NAMED("URDF: ", "Unable to load URDF model");
    }
    else{
        ROS_DEBUG_STREAM_NAMED("URDF: ", "Received URDF from param server");
    }
}

/* main implementation functions */

void ArmHWInterface::update() {
    /* gets elapsed time difference */
    clock_gettime(CLOCK_MONOTONIC, &current_time_);
    elapsed_time =
      ros::Duration(current_time_.tv_sec - last_time_.tv_sec + (current_time_.tv_nsec - last_time_.tv_nsec) / BILLION);
    last_time_ = current_time_; /* updates timing variables for next loop */
    ros::Time now = ros::Time::now();

    /* check for cycle timeout */
    const double time_error = (elapsed_time - update_freq).toSec();
    if(time_error > error_threshold){
        ROS_WARN_STREAM_NAMED("Cycle Time Error:", "Cycle time exceed error threshold by: " << time_error << ", cycle time: " << elapsed_time << 
        ", threshold:" << error_threshold);
    }

    read(now, elapsed_time); /* read in the joint states */
    controller_manager_->update(now, elapsed_time); /* update controller manager */
    write(now, elapsed_time); /* write out new joint states */
}

void ArmHWInterface::run(bool start, bool start_status)
{
  ros::Rate rate(loop_hz);
  while (ros::ok())
  {
    start_status = true;
    update();
    rate.sleep();
  }
}

void ArmHWInterface::stop(bool stop, bool stop_status, const ros::Time& time){
    if(stop == true){
        controller_manager_->stopControllers(const ros::Time& time);
        stop_status = true;
    }
}

}