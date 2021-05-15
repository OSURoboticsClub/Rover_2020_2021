from PyQt5 import QtWidgets, uic, QtCore
from pyqtgraph import PlotWidget, plot
from PyQt5.QtCore import QTimer,QDateTime
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
import random
import time


class MainWindow(QtWidgets.QMainWindow):


    def __init__(self, app, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load the UI Page
        uic.loadUi('MainWindow.ui', self)

        self.plot(app)

    def plot(self, app):
        hour = []
        temperature = []
        for i in range(5):
            hour.append(i)
            temperature.append(random.randint(0,100)/100)
        print(temperature)
        print("------------------------------------------------------------")
        self.graphWidget.plot(hour, temperature)
        self.show()
        

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow(app)
    sys.exit(app.exec_())
    # main.show()
    while True:
        print("It's in the loop!")
        main.plot(app)
        time.sleep(1)
        main.show()
    
    #timer = QTimer()
    #timer.timeout.connect(main.plot())
    #timer.start(500)
    

if __name__ == '__main__':
    main()