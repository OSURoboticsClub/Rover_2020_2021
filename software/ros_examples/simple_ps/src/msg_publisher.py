#!/usr/bin/env python
import rospy
from simple_ps.msg import SimpleMessage
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('msg_pub', SimpleMessage, queue_size=10) #sets up the publisher object to publish a value of type String on the 'simple_pub' topic
    rospy.init_node('msg_publisher') #establishes program as a ros node
    rate = rospy.Rate(10) #10Hz
    msg = SimpleMessage()

    while not rospy.is_shutdown():
        #initializing each msg field
        msg.a = 32
        msg.b = 7.54
        msg.c = False
        msg.d = "Hello"

        rospy.loginfo("Publishing Sample Topic....") #prints to terminal
        pub.publish(msg) #publishes to 'simple_pub'
        rate.sleep() #keeps desired rate of message every 1/10th of a second

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass