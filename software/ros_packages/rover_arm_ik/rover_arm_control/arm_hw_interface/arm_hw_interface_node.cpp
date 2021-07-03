#include "arm_hw_interface.h"

//global variables
//msg string
const std::string default_ik_controls_topic = "IKControl/control_status";
const std::string default_ik_status_topic = "IKControl/button_status";

//booleans for start and stop
bool controllers_started, start_button_pushed, stop_button_pushed;
//bool controller_stopped /* for later use once we have the stop function in melodic maybe? */


int main(int argc, char** argv) {
    ros::init(argc, argv, "arm_hw_interface");
    ros::NodeHandle nh; //set up node handle for this node

    // NOTE: We run the ROS loop in a separate thread as external calls such
    // as service callbacks to load controllers can block the (main) control loop
    ros::AsyncSpinner spinner(3);
    spinner.start();

    arm_hw_interface::ArmHWInterface arm_hw_interface(nh); //create hw interface object
    arm_hw_interface.init(default_ik_controls_topic, default_ik_status_topic);
    ROS_INFO_STREAM_NAMED("hardware_interface", "Starting hardware interface...");
    arm_hw_interface.run(start_button_pushed, stop_button_pushed, controllers_started); /* run hw interface */
    
    return 0;
}