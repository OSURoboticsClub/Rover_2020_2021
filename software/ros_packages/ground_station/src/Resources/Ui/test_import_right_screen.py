import sys
from PyQt5 import QtWidgets, QtCore, QtGui, uic
import signal
import rospy
import logging
import qdarkstyle

import test2 as test2

class ApplicationWindow(QtWidgets.QMainWindow):
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None, ui_file_path=None):
        super(ApplicationWindow, self).__init__(parent)

        uic.loadUi(ui_file_path, self)

        QtWidgets.QShortcut(QtGui.QKeySequence("Ctrl+Q"), self, self.exit_requested_signal.emit)


class GroundStation(QtCore.QObject):

    def __init__(self, parent=None,):
        # noinspection PyArgumentList
        super(GroundStation, self).__init__(parent)

        self.shared_objects["screens"]["right_screen"] = \
                self.create_application_window(UI_FILE_RIGHT, "Rover Ground Station Right Screen",
                                            self.RIGHT_SCREEN_ID)  # type: ApplicationWindow

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()  # This gets us access to the desktop geometry

        app_window = ApplicationWindow(parent=None, ui_file_path=ui_file_path)  # Make a window in this application
        app_window.setWindowTitle(title)  # Sets the window title

        app_window.setWindowFlags(app_window.windowFlags() |  # Sets the windows flags to:
                                QtCore.Qt.FramelessWindowHint |  # remove the border and frame on the application,
                                QtCore.Qt.WindowStaysOnTopHint |  # and makes the window stay on top of all others
                                QtCore.Qt.X11BypassWindowManagerHint)  # This is needed to show fullscreen in gnome

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))  # Sets the window to be on the first screen

        app_window.showFullScreen()  # Shows the window in full screen mode

        return app_window





if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = test2.Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())