#include "arm_hw_interface.h"

//timing libraries
#include <time.h>
#include <chrono>


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_hw_interface");
    ros::NodeHandle nh; //set up node handle for this node
    ros::spin(); //start spinning node (might multithread later)

    /* setup timers */
    ros::Time timestamp;
    ros::Duration period;
    auto stopwatch_last = std::chrono::steady_clock::now();
    auto stopwatch_now = stopwatch_last;

    arm_ros_control::ArmHWInterface arm_hw_interface(nh); //create hw interface object
    ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");
    controller_manager::ControllerManager controller_manager(&arm_hw_interface, nh); //create controller manager

    /* get time since last read */
    timestamp = ros::Time::now();
    stopwatch_now = std::chrono::steady_clock::now();
    period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
    stopwatch_last = stopwatch_now;

    while(ros::ok) {
        /* read current state from arm */
        arm_hw_interface.read();

        /* get time since last read */
        timestamp = ros::Time::now();
        stopwatch_now = std::chrono::steady_clock::now();
        period.fromSec(std::chrono::duration_cast<std::chrono::duration<double>>(stopwatch_now - stopwatch_last).count());
        stopwatch_last = stopwatch_now;

        /* update controllers */
        controller_manager.update(timestamp, period);

        /* write to arm */
        arm_hw_interface.write();
    }

    return 0;
}