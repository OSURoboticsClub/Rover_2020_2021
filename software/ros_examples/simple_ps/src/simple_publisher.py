#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('simple_pub', String, queue_size=10) #sets up the publisher object to publish a value of type String on the 'simple_pub' topic
    rospy.init_node('simple_publisher') #establishes program as a ros node
    rate = rospy.Rate(10) #10Hz

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing Sample Topic....") #prints to terminal
        pub.publish("I am a sample topic!") #publishes to 'simple_pub'
        rate.sleep() #keeps desired rate of message every 1/10th of a second

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass