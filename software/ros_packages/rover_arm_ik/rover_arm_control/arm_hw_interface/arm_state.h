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

#include "src/simplemotion/simplemotion.h"
#include <ros/console.h>
#include <ros/ros.h>
#include <string>
#include <unistd.h>
#include <algorithm>

class ArmState {
    public:
    ArmState();
    ~ArmState();

    void get_joint_positions(std::vector<double> &pos_state);
    void get_joint_effort(std::vector<double> &vel_state);
    void get_joint_velocities(std::vector<double> &eff_state);

    /* setters for positions */
    void set_joint_positions(std::vector<double> &joint_cmds);
    void constrain_set_positions();
    
    private:
    /* sm variables */
    smbus arm_bus_handle;
    std::string arm_port;
    bool arm_successfully_connected = false;

    smint32 base_set_position = 0;
    smint32 base_current_position = 0;
    smint32 base_current_velocity = 0;
    smint32 base_current_effort = 0;
    smint32 base_pos_state = 0;
    smint32 base_status = 0;
    smint32 base_faults = 0;
    double base_cmd = 0;
    double base_curr_pos = 0;
    double base_curr_vel = 0;
    double base_curr_eff = 0;

    smint32 shoulder_set_position = 0;
    smint32 shoulder_current_position = 0;
    smint32 shoulder_current_velocity = 0;
    smint32 shoulder_current_effort = 0;
    smint32 shoulder_comm_state = 0;
    smint32 shoulder_status = 0;
    smint32 shoulder_faults = 0;
    double shoulder_cmd = 0;
    double shoulder_curr_pos = 0;
    double shoulder_curr_vel = 0;
    double shoulder_curr_eff = 0;

    smint32 elbow_set_position = 0;
    smint32 elbow_current_position = 0;
    smint32 elbow_current_velocity = 0;
    smint32 elbow_current_effort = 0;
    smint32 elbow_comm_state = 0;
    smint32 elbow_status = 0;
    smint32 elbow_faults = 0;
    double elbow_cmd = 0;
    double elbow_curr_pos = 0;
    double elbow_curr_vel = 0;
    double elbow_curr_eff = 0;

    smint32 roll_set_position = 0;
    smint32 roll_current_position = 0;
    smint32 roll_current_velocity = 0;
    smint32 roll_current_effort = 0;
    smint32 roll_comm_state = 0;
    smint32 roll_status = 0;
    smint32 roll_faults = 0;
    double roll_cmd = 0;
    double roll_curr_pos = 0;
    double roll_curr_vel = 0;
    double roll_curr_eff = 0;

    smint32 wrist_pitch_last_set_position = 0;
    smint32 wrist_pitch_set_position = 0;
    smint32 wrist_pitch_current_position = 0;
    smint32 wrist_pitch_current_velocity = 0;
    smint32 wrist_pitch_current_effort = 0;
    smint32 wrist_pitch_comm_state = 0;
    smint32 wrist_pitch_status = 0;
    smint32 wrist_pitch_faults = 0;
    double wrist_pitch_cmd = 0;
    double wrist_pitch_curr_pos = 0;
    double wrist_pitch_curr_vel = 0;
    double wrist_pitch_curr_eff = 0;

    smint32 wrist_roll_set_position = 0;
    smint32 wrist_roll_current_position = 0;
    smint32 wrist_roll_current_velocity = 0;
    smint32 wrist_roll_current_effort = 0;
    smint32 wrist_roll_comm_state = 0;
    smint32 wrist_roll_status = 0;
    smint32 wrist_roll_faults = 0;
    double wrist_roll_cmd = 0;
    double wrist_roll_curr_pos = 0;
    double wrist_roll_curr_vel = 0;
    double wrist_roll_curr_eff = 0;
};
#endif