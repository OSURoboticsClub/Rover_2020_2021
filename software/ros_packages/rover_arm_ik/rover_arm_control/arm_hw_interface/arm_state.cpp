#include "arm_state.h"
#include <vector>

ArmState::ArmState() {
    arm_bus_handle = smOpenBus(arm_port.c_str());

    if (arm_bus_handle < 0) {
        ROS_ERROR("Could not connect to arm");
        return;
    } else {
        arm_successfully_connected = true;
    }

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

void ArmState::get_joint_velocities(std::vector<double> &vel_state) {
    /* read in joint velocities */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS, &base_faults, SMP_ACTUAL_VELOCITY_FB, &base_current_velocity);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_VELOCITY_FB, &shoulder_current_velocity);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_VELOCITY_FB, &elbow_current_velocity);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_VELOCITY_FB, &roll_current_velocity);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_VELOCITY_FB, &wrist_pitch_current_velocity);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_VELOCITY_FB, &wrist_roll_current_velocity);

    /* put velocity states into vector */
    vel_state.insert(base_current_velocity, 0);
    vel_state.insert(shoulder_current_velocity, 1);
    vel_state.insert(elbow_current_velocity, 2);
    vel_state.insert(roll_current_velocity, 3);
    vel_state.insert(wrist_pitch_current_velocity, 4);
    vel_state.insert(wrist_roll_current_velocity, 5);
}

void ArmState::get_joint_effort(std::vector<double> &eff_state) {
    /* read in joint efforts */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, SMP_FAULTS, &base_faults, SMP_ACTUAL_TORQUE_FB, &base_current_effort);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_TORQUE_FB, &shoulder_current_effort);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_TORQUE_FB, &elbow_current_effort);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_TORQUE_FB, &roll_current_effort);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_TORQUE_FB, &wrist_pitch_current_effort);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_TORQUE_FB, &wrist_roll_current_effort);

    /* put velocity states into vector */
    eff_state.insert(base_current_effort, 0);
    eff_state.insert(shoulder_current_effort, 1);
    eff_state.insert(elbow_current_effort, 2);
    eff_state.insert(roll_current_effort, 3);
    eff_state.insert(wrist_pitch_current_effort, 4);
    eff_state.insert(wrist_roll_current_effort, 5);

}

void ArmState::get_joint_positions(std::vector<double> &pos_state) {
    /* read in joint positions */
    smRead3Parameters(arm_bus_handle, base_address, SMP_STATUS, &base_status, &base_faults, SMP_ACTUAL_POSITION_FB, &base_current_position);
    smRead3Parameters(arm_bus_handle, shoulder_address, SMP_STATUS, &shoulder_status, SMP_FAULTS, &shoulder_faults, SMP_ACTUAL_POSITION_FB, &shoulder_curent_position);
    smRead3Parameters(arm_bus_handle, elbow_address, SMP_STATUS, &elbow_status, SMP_FAULTS, &elbow_faults, SMP_ACTUAL_POSITION_FB, &elbow_current_position);
    smRead3Parameters(arm_bus_handle, roll_address, SMP_STATUS, &roll_status, SMP_FAULTS, &roll_faults, SMP_ACTUAL_POSITION_FB, &roll_current_position);
    smRead3Parameters(arm_bus_handle, wrist_pitch_address, SMP_STATUS, &wrist_pitch_status, SMP_FAULTS, &wrist_pitch_faults, SMP_ACTUAL_POSITION_FB, &wrist_pitch_current_position);
    smRead3Parameters(arm_bus_handle, wrist_roll_address, SMP_STATUS, &wrist_roll_status, SMP_FAULTS, &wrist_roll_faults, SMP_ACTUAL_POSITION_FB, &wrist_roll_current_position);
    
    wrist_pitch_last_set_position = wrist_pitch_current_position;
    /* put position states into vector */
    pos_state.insert(base_current_position, 0);
    pos_state.insert(shoulder_current_position, 1);
    pos_state.insert(elbow_current_position, 2);
    pos_state.insert(roll_current_position, 3);
    pos_state.insert(wrist_pitch_current_position, 4);
    pos_state.insert(wrist_roll_current_position, 5);
}