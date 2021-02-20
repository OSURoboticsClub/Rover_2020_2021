#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import asyncio
import apsw
from matplotlib import pyplot as plt 
import matplotlib.animation as animation
import time
from pylive import live_plotter
import numpy as np
from datetime import datetime


def callback(data):
	rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", data.x,data.y)
	#setup.x_vec.append(data.x)
	#setup.y_vec.append(data.y)
	#setup.line1 = []
	
	#setup.line1 = live_plotter(setup.x_vec,setup.y_vec,setup.line1)

	setup.x_vec.append(0)
	setup.y_vec.append(data.y)
	if time.time() > setup.timer+1:
		print(len(setup.x_vec))
		print(len(setup.y_vec))
		setup.line1 = live_plotter(setup.x_vec,setup.y_vec,setup.line1)
		setup.timer = time.time()



	

def listener():

	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.
	rospy.init_node('listener', anonymous=True)

	rospy.Subscriber("science_sensor/temp", Point, callback)

	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()

def setup():
	#setup.x_vec = []
	#setup.y_vec = []
	#setup.line1 = []

	size = 1000
	setup.x_vec = [0]*size
	setup.y_vec = [0]*size
	setup.line1 = []
	setup.timer = time.time()


 
if __name__ == '__main__':
	setup()
	listener()
	