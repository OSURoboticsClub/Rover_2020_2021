import sqlite3
from sqlite3 import Error
import logging
import datetime
import csv
import sys
import apsw

logging.basicConfig(filename='./logs/bot.log',level=logging.INFO)
logging.info(datetime.datetime.utcnow().isoformat())
logging.info('Logging initialized successfully')

# Hijack output to logger
class StreamToLogger(object):
	"""
	Fake file-like stream object that redirects writes to a logger instance.
	"""
	def __init__(self, logger, log_level=logging.INFO):
		self.logger = logger
		self.log_level = log_level
		self.linebuf = ''

	def write(self, buf):
		for line in buf.rstrip().splitlines():
			self.logger.log(self.log_level, line.rstrip())

	def flush(self):
		pass

stdout_logger = logging.getLogger('STDOUT')
sl = StreamToLogger(stdout_logger, logging.INFO)
sys.stdout = sl

stderr_logger = logging.getLogger('STDERR')
sl = StreamToLogger(stderr_logger, logging.ERROR)
sys.stderr = sl

def create_test_db(cursor):
	cursor.execute('''CREATE TABLE IF NOT EXISTS TEST1 
	([id] INTEGER PRIMARY KEY, [arg2] text, [arg3] float)''')
	return

def create_test(cursor,arg2,arg3):
	#verifying that the message is not a duplicate of an existing scrim
	statement = 'INSERT INTO TEST1 VALUES (?,?,?)'
	Tuple = (None,arg2,arg3)
	#time1 = datetime.datetime.now()
	cursor.execute(statement,Tuple)
	#logging.info('cursor.execute')
	#logging.info(datetime.datetime.now()-time1)
	#time1 = datetime.datetime.now()
	#logging.info('c.commit')
	#logging.info(datetime.datetime.now()-time1)
	return

def insert_test(cursor,arg2,arg3,id):
	statement = 'UPDATE TEST1 SET arg2 = ?, arg3 = ? WHERE id = ?'
	Tuple = (arg2,arg3,id)
	#time1 = datetime.datetime.utcnow().isoformat()
	cursor.execute(statement,Tuple)
	#logging.info('cursor.execute')
	#logging.info(datetime.datetime.utcnow().isoformat()-time1)
	#time1 = datetime.datetime.utcnow().isoformat()
	#logging.info('c.commit')
	#logging.info(datetime.datetime.utcnow().isoformat()-time1)
	return

def print_all(cursor):
	cursor.execute("SELECT * FROM TEST")

	rows = cursor.fetchall()

	for row in rows:
		logging.info(row)

def main():
	c = apsw.Connection('TEST.db')
	cursor = c.cursor()
	cursor.execute('BEGIN TRANSACTION')
	create_test_db(cursor)
	with open('monthly_csv.csv', newline='') as csvfile:
		spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
		for row in spamreader:
			#logging.info(type(row))
			#logging.info(row[0].split(',')[2])
			create_test(cursor,row[0].split(',')[1],row[0].split(',')[2])
	cursor.execute('COMMIT')

if __name__=='__main__':
	main()
	#logging.info(datetime.datetime.utcnow().isoformat())
