#!/usr/bin/env python
import rospy

def hello_ros_node():
    rospy.init_node('hello_ros') #establishes the program as a ros node

    rospy.loginfo("Hello ROS!") #prints a message to the terminal using the ROS_INFO stream

if __name__ == '__main__':
    try:
        hello_ros_node()
    except rospy.ROSInterruptException:
        pass
