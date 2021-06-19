# coding=utf-8
#####################################
# Imports
#####################################
# Python native imports
from PyQt5 import QtCore, QtWidgets, QtGui
import logging
import rospy

# put msg files here when ready
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

        super(ArmIKControls, self).__init__()

        # ########## Get the settings instance ##########
        self.settings = QtCore.QSettings()

        # ########## Class Variables ##########
        self.button_status_subscriber = rospy.Subscriber(ARM_TOGGLE_STATUS, IKControlMessage, self.arm_ik_status__callback)
        self.ik_status_publisher = rospy.Publisher(ARM_CONTROLLER_STATUS, IKControlMessage, queue_size =1)

        self.controllers_status = False
        self.button_status = False

    def arm_ik_status__callback(self,data):
        self.controllers_status = data.controllers_started
        self.button_status = data.start_button
        
        if self.controllers_started is True:
            ##Indicate that arm IK is started on groundstation
        else:
            ##Indicate that are IK is off on groundstation

        if self.button_status is True:
            ##Send true value across so HW interface can be launched
        else:
            ##Send false value across to stop HW interface

    def connect_signals_and_slots(self):


