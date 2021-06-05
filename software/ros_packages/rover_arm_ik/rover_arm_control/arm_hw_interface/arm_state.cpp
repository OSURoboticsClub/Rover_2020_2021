#include "arm_state.h"

/* global vars taken from rover_arm.cpp */
#define SMP_SERIAL_ENC_OFFSET 575 //This was missing from simplemotion_defs.h

const std::string default_port = "/dev/rover/ttyARM";

// Base
const smuint8 base_address = 1;
const smint32 base_counts_per_rev = 5725807;
const double base_min_rev = -0.5;
const double base_max_rev = 0.5;

smint32 base_min_rev_counts;
smint32 base_max_rev_counts;

// Shoulder
const smuint8 shoulder_address = 2;
const smint32 shoulder_counts_per_rev = 2620130;
const double shoulder_min_rev = -0.25;
const double shoulder_max_rev = 0.25;

smint32 shoulder_min_rev_counts;
smint32 shoulder_max_rev_counts;

//Elbow
const smuint8 elbow_address = 3;
const smint32 elbow_counts_per_rev = 4917661;
const double elbow_min_rev = -0.5;
const double elbow_max_rev = 0.5;

smint32 elbow_min_rev_counts;
smint32 elbow_max_rev_counts;

//Roll
const smuint8 roll_address = 4;
const smint32 roll_counts_per_rev = 1637581;
const double roll_min_rev = -0.25;
const double roll_max_rev = 0.25;

smint32 roll_min_rev_counts;
smint32 roll_max_rev_counts;

//Wrist Pitch
const smuint8 wrist_pitch_address = 5;
const smint32 wrist_pitch_counts_per_rev = 4096000;
const double wrist_pitch_min_rev = -0.25;
const double wrist_pitch_max_rev = 0.25;

smint32 wrist_pitch_min_rev_counts;
smint32 wrist_pitch_max_rev_counts;

//Wrist Roll
const smuint8 wrist_roll_address = 6;
const smint32 wrist_roll_counts_per_rev = 1638400;

ArmState::ArmState() {
    node_handle = new ros::NodeHandle("~");

    node_handle->param("port", arm_port, default_port);
    arm_bus_handle = smOpenBus(arm_port.c_str());

    // if (arm_bus_handle < 0) {
    //     ROS_ERROR("Could not connect to arm");
    //     return;
    // } else {
    //     arm_successfully_connected = true;
    // }

    base_min_rev_counts = smint32(base_min_rev * base_counts_per_rev);
    base_max_rev_counts = smint32(base_max_rev * base_counts_per_rev);

    shoulder_min_rev_counts = smint32(shoulder_min_rev * shoulder_counts_per_rev);
    shoulder_max_rev_counts = smint32(shoulder_max_rev * shoulder_counts_per_rev);

    elbow_min_rev_counts = smint32(elbow_min_rev * elbow_counts_per_rev);
    elbow_max_rev_counts = smint32(elbow_max_rev * elbow_counts_per_rev);

    roll_min_rev_counts = smint32(roll_min_rev * roll_counts_per_rev);
    roll_max_rev_counts = smint32(roll_max_rev * roll_counts_per_rev);

    wrist_pitch_min_rev_counts = smint32(wrist_pitch_min_rev * wrist_pitch_counts_per_rev);
    wrist_pitch_max_rev_counts = smint32(wrist_pitch_max_rev * wrist_pitch_counts_per_rev);
}

ArmState::~ArmState() {
    //arm state destructor
}

void ArmState::get_joint_velocities(std::vector<double> &vel_state) {
    /* read in joint velocities */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS, &base_faults, SMP_ACTUAL_VELOCITY_FB, &base_current_velocity);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_VELOCITY_FB, &shoulder_current_velocity);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_VELOCITY_FB, &elbow_current_velocity);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_VELOCITY_FB, &roll_current_velocity);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_VELOCITY_FB, &wrist_pitch_current_velocity);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_VELOCITY_FB, &wrist_roll_current_velocity);

    /* cast joint velocities to doubles so controller can work with them */
    base_curr_vel = double(base_current_velocity);
    shoulder_curr_vel = double(shoulder_current_velocity);
    elbow_curr_vel = double(elbow_current_velocity);
    roll_curr_vel = double(roll_current_velocity);
    wrist_pitch_curr_vel = double(wrist_pitch_current_velocity);
    wrist_roll_curr_vel = double(wrist_roll_current_velocity);

    /* put velocity states into vector */
    vel_state.push_back(base_curr_vel);
    vel_state.push_back(shoulder_curr_vel);
    vel_state.push_back(elbow_curr_vel);
    vel_state.push_back(roll_curr_vel);
    vel_state.push_back(wrist_pitch_curr_vel);
    vel_state.push_back(wrist_roll_curr_vel);
}

void ArmState::get_joint_effort(std::vector<double> &eff_state) {
    /* read in joint efforts */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS, &base_faults, SMP_ACTUAL_TORQUE, &base_current_effort);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_TORQUE, &shoulder_current_effort);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_TORQUE, &elbow_current_effort);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_TORQUE, &roll_current_effort);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_TORQUE, &wrist_pitch_current_effort);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_TORQUE, &wrist_roll_current_effort);

    /* cast joint efforts to doubles for controllers */
    base_curr_eff = double(base_current_effort);
    shoulder_curr_eff = double(shoulder_current_effort);
    elbow_curr_eff = double(elbow_current_effort);
    roll_curr_eff = double(roll_current_effort);
    wrist_pitch_curr_eff = double(wrist_pitch_current_effort);
    wrist_roll_curr_eff = double(wrist_roll_current_effort);

    /* put effort states into vector */
    eff_state.push_back(base_curr_eff);
    eff_state.push_back(shoulder_curr_eff);
    eff_state.push_back(elbow_curr_eff);
    eff_state.push_back(roll_curr_eff);
    eff_state.push_back(wrist_pitch_curr_eff);
    eff_state.push_back(wrist_roll_curr_eff);

}

void ArmState::get_joint_positions(std::vector<double> &pos_state) {
    /* read in joint positions */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS, &base_faults, SMP_ACTUAL_POSITION_FB, &base_current_position);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_POSITION_FB, &shoulder_current_position);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_POSITION_FB, &elbow_current_position);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_POSITION_FB, &roll_current_position);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_POSITION_FB, &wrist_pitch_current_position);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_POSITION_FB, &wrist_roll_current_position);
    
    wrist_pitch_last_set_position = wrist_pitch_current_position;
    /*cast smint32 to double bc that's the data type required for controllers in ros */
    base_curr_pos = double(base_current_position);
    shoulder_curr_pos = double(shoulder_current_position);
    elbow_curr_pos = double(elbow_current_position);
    roll_curr_pos = double(roll_current_position);
    wrist_pitch_curr_pos = double(wrist_pitch_current_position);
    wrist_roll_curr_pos = double(wrist_roll_current_position);

    /* put position states into vector */
    pos_state.push_back(base_curr_pos);
    pos_state.push_back(shoulder_curr_pos);
    pos_state.push_back(elbow_curr_pos);
    pos_state.push_back(roll_curr_pos);
    pos_state.push_back(wrist_pitch_curr_pos);
    pos_state.push_back(wrist_roll_curr_pos);
}

void ArmState::constrain_set_positions(){
    base_set_position = std::min(std::max(base_set_position, base_min_rev_counts), base_max_rev_counts);
    shoulder_set_position = std::min(std::max(shoulder_set_position, shoulder_min_rev_counts), shoulder_max_rev_counts);
    elbow_set_position = std::min(std::max(elbow_set_position, elbow_min_rev_counts), elbow_max_rev_counts);
    roll_set_position = std::min(std::max(roll_set_position, roll_min_rev_counts), roll_max_rev_counts);
    wrist_pitch_set_position = std::min(std::max(wrist_pitch_set_position, wrist_pitch_min_rev_counts), wrist_pitch_max_rev_counts);
}

void ArmState::set_joint_positions(std::vector<double> &joint_cmds){
    /* get joint position cmds from cmd vector */
    base_cmd = joint_cmds.at(0);
    shoulder_cmd = joint_cmds.at(1);
    elbow_cmd = joint_cmds.at(2);
    roll_cmd = joint_cmds.at(3);
    wrist_pitch_cmd = joint_cmds.at(4);
    wrist_roll_cmd = joint_cmds.at(5);

    /*cast commands to long ints/smints so ionis can read them */
    base_set_position = long(base_cmd);
    shoulder_set_position = long(shoulder_cmd);
    elbow_set_position = long(elbow_cmd);
    roll_set_position = long(roll_cmd);
    wrist_pitch_set_position = long(wrist_pitch_cmd);
    wrist_roll_set_position = long(wrist_roll_cmd);

    /* actually write/send cmds to IONI */
    smSetParameter(arm_bus_handle, base_address, SMP_ABSOLUTE_SETPOINT, base_set_position);
    smSetParameter(arm_bus_handle, shoulder_address, SMP_ABSOLUTE_SETPOINT, shoulder_set_position);
    smSetParameter(arm_bus_handle, elbow_address, SMP_ABSOLUTE_SETPOINT, elbow_set_position);
    smSetParameter(arm_bus_handle, roll_address, SMP_ABSOLUTE_SETPOINT, roll_set_position);
    smSetParameter(arm_bus_handle, wrist_pitch_address, SMP_ABSOLUTE_SETPOINT, wrist_pitch_set_position);
    smSetParameter(arm_bus_handle, wrist_roll_address, SMP_ABSOLUTE_SETPOINT, wrist_roll_set_position);
}