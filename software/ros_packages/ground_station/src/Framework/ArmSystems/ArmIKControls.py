# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import sys
import signal
import logging
import rospy

from rover_arm_control.msg import IKControlMessage

#####################################
# Global Variables
#####################################
ARM_CONTROLLER_STATUS = "/rover_arm_control/IKControl/control_status"
ARM_TOGGLE_STATUS =  "/rover_arm_control/IKControl/button_status"


class ArmIKControls(QtCore.QObject):
    def __init__(self, shared_objects):
        controller_start__signal = QtCore.pyqtSignal(str)
        controller_status__signal = QtCore.pyqtSignal(str)

        # ########## Reference to class init variables ##########
        super(ArmIKControls, self).__init__()
        self.shared_objects = shared_objects
        self.main_screen = self.shared_objects["screens"]["main_screen"]

        self.ik_control_start_button = self.main_screen.ik_control_start_button #type: QtWidgets.QButton
        self.ik_control_status_label = self.main_screen.ik_control_status_label #type : QtWidgets.QLabel

        # ########## Class Variables ##########
        self.ik_status_subscriber = rospy.Subscriber(ARM_CONTROLLER_STATUS, IKControlMessage, self.arm_ik_status__callback)
        self.ik_status_publisher = rospy.Publisher(ARM_TOGGLE_STATUS, IKControlMessage, queue_size =1)

        self.controllers_status = False
        self.button_status = False

    def arm_ik_status__callback(self,data):
        self.controllers_status = data.controllers_started
        self.button_status = data.start_button
        
        if self.controllers_started is True:
            ##Indicate that arm IK is started on groundstation controller_start__signal.emit(COLOR_GREEN)
        else:
            ##Indicate that are IK is off on groundstation controller_start__signal.emit(COLOR_RED)

        if self.button_status is True:
            self.ik_status_publisher.publish(message)
        else:
            message.start_button = False
            self.ik_status_publisher.publish(message)

    def connect_signals_and_slots(self):
        self.ik_control_start_button.clicked.connect(self.on_ik_start_button_pressed__slot)

    def on_ik_start_button_pressed__slot(self):
        message = IKControlMessage()
        message.start_button = True
        self.button_status = True



