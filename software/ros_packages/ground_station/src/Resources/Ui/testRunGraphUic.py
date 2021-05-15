import test_graph_uic as test_graph_uic
import time
import signal
from PyQt5.QtCore import QTimer,QDateTime
from PyQt5 import QtWidgets, uic, QtCore
from pyqtgraph import PlotWidget, plot
from PyQt5.QtCore import QTimer,QDateTime
import pyqtgraph as pg
import sys  # We need sys so that we can pass argv to QApplication
import os

# app = QtWidgets.QApplication(sys.argv)
# main = graph.MainWindow(app) 

UI_FILE_RIGHT = "Resources/Ui/MainWindow.ui"

# import Resources.Ui.test_graph_uic as test_graph_uic

class MainWindow(QtWidgets.QMainWindow):
    
    exit_requested_signal = QtCore.pyqtSignal()

    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, app, *args, **kwargs):
        super(MainWindow, self).__init__(*args, **kwargs)

        #Load the UI Page
        uic.loadUi('MainWindow.ui', self)

        self.plot(app)


class ScienceSystems(QtCore.QObject):
    
    RIGHT_SCREEN_ID = 0

    exit_requested_signal = QtCore.pyqtSignal()

    start_threads_signal = QtCore.pyqtSignal()
    connect_signals_and_slots_signals = QtCore.pyqtSignal()
    kill_threads_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super(ScienceSystems, self).__init__(parent)

        self.shared_objects = {
            "screens": {},
            "regular_classes": {},
            "threaded_classes": {}
        }

        self.shared_objects["screens"]["right_screen"] = \
            self.create_application_window(UI_FILE_RIGHT, "Rover Testing Science Plotting", self.RIGHT_SCREEN_ID)

        self.__add_thread("Science Plotting", test_graph_uic.test_graph_uic(self.shared_objects))

        self.connect_signals_and_slots_signal.emit()
        self.__connect_signals_to_slots()
        self.start_threads_signal.emit()

    def __add_thread(self, thread_name, instance):
        self.shared_objects["threaded_classes"][thread_name] = instance
        instance.setup_signals(self.start_threads_signal, self.connect_signals_and_slots_signals, self.kill_threads_signal)

    def __connect_signals_to_slots(self):
        self.sjared_objects["screens"]["main_window"].exit_requested_signal.connect(self.on_exit_requested__slot)

    def on_exit_requested__slot(self):
        self.kill_threads_signal.emit()

        for thread in self.shared_objects["threaded_classes"]:
            self.shared_objects["threaded_classes]"][thread].wait()

        QtGui.QGuiApplication.exit()

    @staticmethod
    def create_application_window(ui_file_path, title, display_screen):
        system_desktop = QtWidgets.QDesktopWidget()

        app_window = ApplicationWindow(parent = None, ui_file_path = ui_file_path)
        app_window.setWindowTitle(title)

        app_window.setWindowFlags(app_window.windowFlags() | QtCore.Qt.FramelessWindowHint | QtCore.Qt.WindowStaysOnTopHint | QtCore.Qt.X11BypassWindowManagerHint)

        app_window.setGeometry(
            system_desktop.screenGeometry(display_screen))
        
        app_window.showFullScreen()

        return app_window


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    application = QtWidgets.QApplication(sys.argv)
    
    application.exec_()