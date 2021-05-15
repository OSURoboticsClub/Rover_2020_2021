from PyQt5 import QtWidgets, uic
from pyqtgraph import PlotWidget, plot
from PyQt5.QtCore import QTimer,QDateTime
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os
import random
import time

class MainWindow(QtWidgets.QMainWindow):

    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load the UI Page
        uic.loadUi('MainWindow.ui', self)

        self.plot()

    

    def plot(self):
        hour = []
        temperature = []
        for i in range(100):
            hour.append(i)
            temperature.append(random.randint(0,100)/100)
            print(temperature)
        self.graphWidget.plot(hour, temperature)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    #timer = QTimer()
    #timer.timeout.connect(main.plot())
    #timer.start(500)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()