#include "arm_hw_interface.h"


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_hw_interface");
    ros::NodeHandle nh; //set up node handle for this node

    arm_hw_interface::ArmHWInterface arm_hw_interface(nh); //create hw interface object
    ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");
    arm_hw_interface.run(); /* run hw interface */
    
    ros::spin(); //start spinning node (TODO: Implement safe multithreading)

    return 0;
}