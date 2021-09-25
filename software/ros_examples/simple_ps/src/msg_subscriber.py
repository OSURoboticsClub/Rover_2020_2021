#!/usr/bin/env python
import rospy
from simple_ps.msg import SimpleMessage
from std_msgs.msg import String

def callback(data):
    msg = SimpleMessage()
    rospy.loginfo(rospy.get_name() + "Field a is a=%d, Field b is b=%f, Field c is c=%s, Field d is d=%s", data.a, data.b, data.c, data.d)

def subscriber():
    rospy.init_node('msg_sub', anonymous=True)
    rospy.Subscriber("msg_pub", SimpleMessage, callback)

    rospy.spin()

if __name__ == '__main__':
    subscriber()