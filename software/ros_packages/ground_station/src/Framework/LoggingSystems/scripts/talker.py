import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
import apsw
import sys
import time

def talker(rows):
	if not rospy.is_shutdown():
		pub = rospy.Publisher('science_sensor/temp',Point,queue_size=10)
		rospy.init_node('talker', anonymous=True)
		rate = rospy.Rate(100)
		for x in range(1,len(rows)):
		
			talker_point = Point()
			talker_point.x = 0#float(time.time())
			talker_point.y = rows[x][2]
			talker_point.z = 0
			#message_str = ''
			#for y in range(len(rows[x])):
			#	message_str = message_str + str(rows[x][y])
			rospy.loginfo(str(talker_point.x)+str(talker_point.y))
			pub.publish(talker_point)
			rate.sleep()
	else:
		return

def db_grab_all(cursor):
	cursor.execute("SELECT * FROM TEST1")

	rows = cursor.fetchall()
	return rows


if __name__=='__main__':
	c = apsw.Connection('TEST.db')
	cursor = c.cursor()
	cursor.execute('BEGIN TRANSACTION')
	rows = db_grab_all(cursor)
	cursor.execute('COMMIT')
	print(type(rows[0]))

	try: 
		talker(rows)
	except rospy.ROSInterruptException:
		pass