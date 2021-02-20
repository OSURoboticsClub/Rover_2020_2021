import sqlite3
from sqlite3 import Error
import logging
import datetime
import csv
import sys
import apsw

def apsw_create_table(cursor,name,*args):
	statement = 'CREATE TABLE IF NOT EXISTS ' + str(name) +'''
	([id] INTEGER PRIMARY KEY'''

	for x in range(int(len(args)/2)):
		statement = statement + ', [' + str(args[x]) + '] ' + str(args[x+1]) 
	statement = statement + ')'
	print(statement)
	cursor.execute(statement)
	return

def apsw_append(cursor,arg2,arg3):
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

def apsw_replace(cursor,arg2,arg3,id):
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

def print_all(cursor,name):
	cursor.execute("SELECT * FROM " + str(name) + ')'

	rows = cursor.fetchall()

	for row in rows:
		print(row)

def main():
	c = apsw.Connection('TEST2.db')
	cursor = c.cursor()
	cursor.execute('BEGIN TRANSACTION')
	apsw_create_table(cursor,'table_test','a','string')
	cursor.execute('COMMIT')

	cursor.execute('BEGIN TRANSACTION')
	print_all(cursor,'table_test')
	cursor.execute('COMMIT')

if __name__=='__main__':
	main()
	#logging.info(datetime.datetime.utcnow().isoformat())
