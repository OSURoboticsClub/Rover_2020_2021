#include "arm_state.h"

ArmState::ArmState() {
    arm_bus_handle = smOpenBus(arm_port.c_str());

    if (arm_bus_handle < 0) {
        ROS_ERROR("Could not connect to arm");
        return;
    } else {
        arm_successfully_connected = true;
    }
}