import apsw
from pylive import live_plotter
import numpy as np
import time
import sys

def grab_100(cursor):
	#begin sqlite transaction
	cursor.execute('BEGIN TRANSACTION')
	#exectues a prepared statement to grab the last 1000 entries into the db
	statement = 'SELECT * FROM temp ORDER BY id DESC LIMIT 1000'
	cursor.execute(statement)
	#returns the last 1000 rows as the list named "rows"
	rows = cursor.fetchall()
	if len(rows) < 1000 and len(rows) > 0:
		setup.x_vec = [rows[0][1]]*1000
		setup.y_vec = [rows[0][2]]*1000
	#inserts "rows" onto the end of the x and y values for the live plotter. x_vec and y_vec CANNOT change in length
	for i in range(len(rows)):
			setup.x_vec = np.append(setup.x_vec[1:],rows[i][1])
			setup.y_vec = np.append(setup.y_vec[1:],rows[i][2])
	
	#ends sqlite transaction
	cursor.execute('COMMIT')
	return

def grapher(cursor):
	while True:
		#graphs every second to conserve resources
		if time.time() > setup.timer+1:
			#exception used to prevent trying to use the database while it's being edited
			try:
				#grabs the last 1000 rows from the db
				grab_100(cursor)
				#uses pylive.py to plot the graph live
				setup.line1 = live_plotter(setup.x_vec,setup.y_vec,setup.line1)
				#resets the timer
				setup.timer = time.time()
			except: 
				print(sys.exc_info()[0])

def setup():
	size = 1000
	setup.x_vec = [0]*size
	setup.y_vec = [0]*size
	setup.line1 = []
	setup.timer = time.time()

if __name__ == '__main__':
	setup()
	c = apsw.Connection('GROUND.db')
	cursor = c.cursor()
	grapher(cursor)