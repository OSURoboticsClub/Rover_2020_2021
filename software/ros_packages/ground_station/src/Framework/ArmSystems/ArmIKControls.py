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
        super(ArmIKControls, self).__init__()

        # ########## Class Variables ##########
        self.button_status_subscriber = rospy.Subscriber(ARM_TOGGLE_STATUS, IKControlMessage, self.new_arm_ik_status__callback)
        self.ik_status_publisher = rospy.Publisher(ARM_CONTROLLER_STATUS, IKControlMessage, queue_size =1)

        self.controllers_started = False
        self.start_button = False



