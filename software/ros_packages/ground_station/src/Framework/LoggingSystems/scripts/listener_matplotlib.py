#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import asyncio
import apsw
#from matplotlib import pyplot as plt 
#import matplotlib.animation as animation
import time
from pylive import live_plotter
import numpy as np
from datetime import datetime

#sqlite functions
def create_temp_db(cursor):
	#creates the table in the db. Part of setup.
	cursor.execute('''CREATE TABLE IF NOT EXISTS temp 
	([id] INTEGER PRIMARY KEY, [arg2] float, [arg3] float)''')
	return

def append_value(cursor,arg2,arg3):
	#adds the data point (id,x,y) to the db
	statement = 'INSERT INTO temp VALUES (?,?,?)'
	Tuple = (None,arg2,arg3)
	cursor.execute(statement,Tuple)
	return










#rosfunctions
def callback(data):
	rospy.loginfo(rospy.get_caller_id() + ',' + str(data.x-setup.starttime))
	#setup.x_vec.append(data.x)
	#setup.y_vec.append(data.y)
	#setup.line1 = []
	
	#setup.line1 = live_plotter(setup.x_vec,setup.y_vec,setup.line1)

	#setup.x_vec = np.append(setup.x_vec[1:],data.x-setup.starttime)
	#setup.y_vec = np.append(setup.y_vec[1:],data.y)
	append_value(setup.cursor,data.x,data.y)
	setup.timer2 += 1
	if setup.timer2 > 1000 or time.time() > setup.timer + 1:
		setup.cursor.execute('COMMIT')
		setup.cursor.execute('BEGIN TRANSACTION')
		setup.timer2 = 0
		setup.timer = time.time()

		#setup.line1 = live_plotter(setup.x_vec,setup.y_vec,setup.line1)
		#setup.timer = time.time()
		



	

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
	#setup processes. Creating variables and tables
	setup.starttime = time.time()
	size = 1000
	setup.x_vec = [time.time()-setup.starttime]*size
	setup.y_vec = [0]*size
	setup.line1 = []
	setup.timer = time.time()
	setup.timer1 = 0
	setup.timer2 = 0
	c = apsw.Connection('GROUND.db')
	setup.cursor = c.cursor()
	setup.cursor.execute('BEGIN TRANSACTION')
	create_temp_db(setup.cursor)
	setup.cursor.execute('COMMIT')
	setup.cursor.execute('BEGIN TRANSACTION')


 
if __name__ == '__main__':
	setup()
	listener()