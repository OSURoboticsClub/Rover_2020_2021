import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy
import logging
import qdarkstyle

import test_right_screen as test2


class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(ApplicationWindow, self).__init__(parent)

        ui = test2.Ui_MainWindow()
        ui.setupUi(self)  # Make a window in this application

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


class GroundStation(QtCore.QObject):
    RIGHT_SCREEN_ID = 0

    

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        self.shared_objects = {
            "screens": {},
            "regular_classes": {},
            "threaded_classes": {}
        }

        self.shared_objects["screens"]["right_screen"] = \
                self.create_application_window("Rover Ground Station Right Screen",
                                            self.RIGHT_SCREEN_ID)  # type: ApplicationWindow
        
        

    @staticmethod
    def create_application_window(title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        

        app_window = ApplicationWindow(parent=None)
        
        app_window.setWindowTitle(title)  # Sets the window title
        #QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), app_window, app_window.exit_requested_signal.emit)

        #app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
        #                        QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
        #                        QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
        #                        QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        #app_window.showFullScreen()  # Shows the window in full screen mode
        app_window.show()

        return app_window





if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    #MainWindow = QtWidgets.QMainWindow()
    gnd = GroundStation()
    #ui = ApplicationWindow()
    #ui.imported.setupUi(MainWindow)
    #MainWindow.show()
    app.exec_()