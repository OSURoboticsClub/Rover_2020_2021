from PyQt5 import QtWidgets, uic
import sys
import rviz
import mumu

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)
        uic.loadUi('interface.ui', self)
        #self.window = QtWidgets.QWidget()
        #self.layout = QtWidgets.QGridLayout()
        #self.setCentralWidget(self.window)
        #self.window.setLayout(self.layout)

        self.setWindowTitle("This is the mumu")
        #self.MoveIt = mumu.MoveItInterface()
        #self.layout.addWidget(self.MoveIt)

def main():
    app = QtWidgets.QApplication(sys.argv)
    main = MainWindow()
    main.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
