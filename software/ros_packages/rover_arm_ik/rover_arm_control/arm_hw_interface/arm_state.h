/* psuedo code/plan below here

* get_joint_positions():
- see first half of 'get_joint_status' code in rover_arm.cpp for ref
- write individual joint positions to a vector of joint positions
- assign this vector of joint pos to pos vector in hw interface

* get_joint_velocities();
- use first half of joint status code, but modify it to take in velocity
- write indiv joint velocities to a vector of joint velocities
- assign this vector of joint vel to vel vector in hw interface

*get_joint_effort();
- use first half of joint status code but modify it to take in effort
- write indiv joint efforts to vector of joint efforts
- assign this vector of joint efforts to effort vector in hw interface

*/

#ifndef ARM_STATE_H
#define ARM_STATE_H

#include "simplemotion/simplemotion.h"
#include <vector>

/* global vars taken from rover_arm.cpp */
#define SMP_SERIAL_ENC_OFFSET 575 //This was missing from simplemotion_defs.h

// Base
const smuint8 base_address = 1;
const smint32 base_counts_per_rev = 5725807;
const double base_min_rev = -0.5;
const double base_max_rev.5;

smint32 base_min_rev_counts;
smint32 base_max_rev_counts;

// Shoulder
const smuint8 shoulder_address = 2;
const smint32 shoulder_counts_per_rev = 2620130;
const double shoulder_min_rev = -0.25;
const double shoulder_max_rev.25;

smint32 shoulder_min_rev_counts;
smint32 shoulder_max_rev_counts;

//Elbow
const smuint8 elbow_address = 3;
const smint32 elbow_counts_per_rev = 4917661;
const double elbow_min_rev = -0.5;
const double elbow_max_rev.5;

smint32 elbow_min_rev_counts;
smint32 elbow_max_rev_counts;

//Roll
const smuint8 roll_address = 4;
const smint32 roll_counts_per_rev = 1637581;
const double roll_min_rev = -0.25;
const double roll_max_rev.25;

smint32 roll_min_rev_counts;
smint32 roll_max_rev_counts;

//Wrist Pitch
const smuint8 wrist_pitch_address = 5;
const smint32 wrist_pitch_counts_per_rev = 4096000;
const double wrist_pitch_min_rev = -0.25;
const double wrist_pitch_max_rev.25;

smint32 wrist_pitch_min_rev_counts;
smint32 wrist_pitch_max_rev_counts;

//Wrist Roll
const smuint8 wrist_roll_address = 6;
const smint32 wrist_roll_counts_per_rev = 1638400;

class ArmState {
    public:
    ArmState();
    ~ArmState();

    void get_joint_positions(std::vector<double> &pos_state);
    void get_joint_effort(std::vector<double> &vel_state);
    void get_joint_velocities(std::vector<double> &eff_state);
    
    private:
    /* sm variables */
    smbus arm_bus_handle;
    bool arm_successfully_connected = false;

    smint32 base_set_position = 0;
    smint32 base_current_position = 0;
    smint32 base_current_velocity = 0;
    smint32 base_current_effort = 0;
    smint32 base_pos_state = 0;
    smint32 base_status = 0;
    smint32 base_faults = 0;

    smint32 shoulder_set_position = 0;
    smint32 shoulder_current_position = 0;
    smint32 shoulder_current_velocity = 0;
    smint32 shoulder_current_effort = 0;
    smint32 shoulder_comm_state = 0;
    smint32 shoulder_status = 0;
    smint32 shoulder_faults = 0;

    smint32 elbow_set_position = 0;
    smint32 elbow_current_position = 0;
    smint32 elbow_current_velocity = 0;
    smint32 elbow_current_effort = 0;
    smint32 elbow_comm_state = 0;
    smint32 elbow_status = 0;
    smint32 elbow_faults = 0;

    smint32 roll_set_position = 0;
    smint32 roll_current_position = 0;
    smint32 roll_current_velocity = 0;
    smint32 roll_current_effort = 0;
    smint32 roll_comm_state = 0;
    smint32 roll_status = 0;
    smint32 roll_faults = 0;

    smint32 wrist_pitch_last_set_position = 0;
    smint32 wrist_pitch_set_position = 0;
    smint32 wrist_pitch_current_position = 0;
    smint32 wrist_pitch_current_velocity = 0;
    smint32 writst_pitch_current_effort = 0;
    smint32 wrist_pitch_comm_state = 0;
    smint32 wrist_pitch_status = 0;
    smint32 wrist_pitch_faults = 0;

    smint32 wrist_roll_set_position = 0;
    smint32 wrist_roll_current_position = 0;
    smint32 wrist_roll_current_velocity = 0;
    smint32 wrist_roll_current_effort = 0;
    smint32 wrist_roll_comm_state = 0;
    smint32 wrist_roll_status = 0;
    smint32 wrist_roll_faults = 0;
}
#endif