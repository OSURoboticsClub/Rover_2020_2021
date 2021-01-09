#include "arm_state.h"

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

void ArmState::get_joint_positions() {

}

void ArmState::get_joint_velocities() {

}

void ArmState::get_joint_effort() {
    
}