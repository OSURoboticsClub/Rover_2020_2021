#!/usr/bin/env python
import rospy

def hello_ros_node():
    rospy.init_node('hello_ros')

    rospy.loginfo("Hello ROS!")

if __name__ == '__main__':
    try:
        hello_ros_node()
    except rospy.ROSInterruptException:
        pass



