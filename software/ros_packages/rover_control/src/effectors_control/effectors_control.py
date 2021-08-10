#!/usr/bin/env python
#####################################
# Imports
#####################################
# Python native imports
import rospy
from time import time, sleep

import serial.rs485
import minimalmodbus

# from std_msgs.msg import UInt8, UInt16

# Custom Imports
from rover_control.msg import MiningControlMessage, MiningStatusMessage, GripperControlMessage, GripperStatusMessage, CameraControlMessage, DrillControlMessage
# from rover20_comms import rover20_comms as pothos # falling back from pothos control for now #

#####################################
# Global Variables
#####################################
NODE_NAME = "effectors_control"

# ##### Communication Defines #####
DEFAULT_GRIPPER_PORT = "/dev/rover/ttyEffectors"
DEFAULT_MINING_PORT = "/dev/rover/ttyMining"
# DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200

GRIPPER_TIMEOUT = 0.5
MINING_TIMEOUT = 0.3
DRILL_TIMEOUT = 0.3

MINING_HALF_REG_LIMIT = 15
MINING_REMAINING_REGS = 17

FAILED_GRIPPER_MODBUS_LIMIT = 20
#FAILED_MINING_MODBUS_LIMIMT = 20
FAILED_SCIENCE_MECH_MODBUS_LIMIT = 20
#FAILED_DRILL_MODBUS_LIMIMT = 20

RX_DELAY = 0.01
TX_DELAY = 0.01

DEFAULT_HERTZ = 40

GRIPPER_CONTROL_SUBSCRIBER_TOPIC = "gripper/control"
GRIPPER_STATUS_PUBLISHER_TOPIC = "gripper/status"

MINING_CONTROL_SUBSCRIBER_TOPIC = "mining/control"
MINING_STATUS_PUBLISHER_TOPIC = "mining/status"

DRILL_CONTROL_SUBSCRIBER_TOPIC = "mining/drill/control"

CAMERA_CONTROL_SUBSCRIBER_TOPIC = "camera/control"

GRIPPER_UNIVERSAL_POSITION_MAX = 10000
MINING_POSITIONAL_THRESHOLD = 20

# ##### Mining Node Defines ##### #
"""
mining_nodes = {
    "ScienceMech": 1
}
"""

# ##### Mining Register Defines ##### #
"""
mining_pothos_registers = {
    # For Motor 1 #
    "SPEED_1": 0,
    "DIR_1": 1,
    "TMP_1": 2,
    "CURRENT_1": 3,

    # For Motor 2 #
    "SPEED_2": 4,
    "DIR_2": 5,
    "TMP_2": 6,
    "CURRENT_2": 7,

    # For Video MUX #
    "VID_SELECT": 8,

    # For Stepper Motor #
    "DIR_3": 9,
    "STEP": 10
}
"""

# ##### Gripper Node Defines ##### #

# ##### Gripper Register Defines ##### #


# ##### Node IDS ##### #

GRIPPER_NODE_ID = 1
#DRILL_NODE_ID = 2
#MINING_NODE_ID = 3
SCIENCE_MECH_NODE_ID = 2 #mining_nodes.get("ScienceMech")

# ##### Setup for pothos comms ##### #
dataTypes = [['chr', 'chr', 'float', 'float'], ['chr', 'int', 'int', 'float']] # data types for pothos
comms = pothos(len(dataTypes), dataTypes, 'COM25', 0.030, 500000)

# ##### Gripper Defines #####
GRIPPER_MODBUS_REGISTERS = {
    "POSITION": 0,
    "HOME": 1,
    "TARGET": 2,
    "SPEED": 3,
    "DIRECTION": 4,
    "LASER": 5,
    "LED": 6,
    "TEMP": 7,
    "DISTANCE": 8,
    "CURRENT": 9,
    "IS_HOMED": 10
}


DEFAULT_GRIPPER_REGISTERS = [
    0,  # No positional update
    0,  # Do not home
    0,  # No target
    0,  # 0 speed
    0,  # No direction
    0,  # No laser
    0,  # Light off
    0,  # 0 temp
    0,  # 0 distance
    0,  # 0 current
    0,  # Not homed
]

# ##### Mining Defines #####
# ##### These are divided into two parts to avoid CRC errors over modbus. #####
MINING_MODBUS_REGISTERS = {
    """
    "CAM_ZOOM_IN_FULL": 0,
    #"CAM_ZOOM_OUT_FULL": 1,
    #"CAM_ZOOM_IN": 2,
    #"CAM_ZOOM_OUT": 3,
    #"CAM_SHOOT": 4,
    #"CAM_CHANGE_VIEW": 5,

    "MOTOR_SET_POSITION_POSITIVE": 6,
    "MOTOR_SET_POSITION_NEGATIVE": 7,

    "MOTOR_SET_POSITION_ABSOLUTE": 8,
    "MOTOR_GO_HOME": 9,
    """

    ### Registers for Motor Controller 1 ###
    "SPEED_1": 0,
    "DIR_1": 1,
    "TMP_1": 2,
    "CURRENT_1": 3,

    ### Registers for Motor Controller 2 ###
    "SPEED_2": 4,
    "DIR_2": 5,
    "TMP_2": 6,
    "CURRENT_2": 7,

    ### Register for Camera MUX ###
    "CAM_MUX": 8,

    ### Registers for Stepper Motor ###
    "INCREMENT_STEP": 9,
    "SET_DIRECTION": 10,

    ### Registers for Limit Switches ###
    "LINEAR_LIM_TOP": 11,
    "LINEAR_LIM_BASE": 12,

    ### Additional Registers for Linear Motor ###
    "LINEAR_SET_POSITION_TARGET": 13
    "LINEAR_CURRENT_POSITION": 14

}

"""
MINING_MODBUS_REGISTERS_PART_2 = {
    "LINEAR_SET_POSITION_POSITIVE": 0,
    "LINEAR_SET_POSITION_NEGATIVE": 1,
    "LINEAR_SET_POSITION_ABSOLUTE": 2,

    "LINEAR_CURRENT_POSITION": 3,
    "MOTOR_CURRENT_POSITION": 4,

    "TEMP1": 5,
    "TEMP2": 6,

    "MOTOR_CURRENT": 7,
    "LINEAR_CURRENT": 8,

    "SERVO1_TARGET": 9,
    "SERVO2_TARGET": 10,

    "SWITCH1_OUT": 11,
    "SWITCH2_OUT": 12,

    "HOMING_NEEDED": 13,
#}
"""

"""
DRILL_MODBUS_REGISTERS = {
    "DIRECTION": 0,
    "SPEED": 1
}
"""

# ##### Science Defines #####

# ##### Misc Defines #####
NODE_LAST_SEEN_TIMEOUT = 2  # seconds

INT16_MAX = 32767
INT16_MIN = -32768
UINT16_MAX = 65535


#####################################
# DriveControl Class Definition
#####################################
class EffectorsControl(object):
    EFFECTORS = [
        "GRIPPER",
        "MINING"
    ]

    def __init__(self):
        rospy.init_node(NODE_NAME)

        self.gripper_port = rospy.get_param("~port", DEFAULT_GRIPPER_PORT)
        self.mining_port = rospy.get_param("~port", DEFAULT_MINING_PORT)
        self.drill_port = rospy.get_param("~port", DEFAULT_MINING_PORT)
        self.baud = rospy.get_param("~baud", DEFAULT_BAUD)

        self.gripper_node_id = rospy.get_param("~gripper_node_id", GRIPPER_NODE_ID)
        #self.mining_node_id = rospy.get_param("~mining_node_id", MINING_NODE_ID)
        #self.drill_node_id = rospy.get_param("~drill_node_id", DRILL_NODE_ID)
        self.science_mech_node_id = rospy.get_param("~science_mech_node_id", SCIENCE_MECH_NODE_ID)

        self.gripper_control_subscriber_topic = rospy.get_param("~gripper_control_subscriber_topic",
                                                                GRIPPER_CONTROL_SUBSCRIBER_TOPIC)
        self.gripper_status_publisher_topic = rospy.get_param("~gripper_status_publisher_topic",
                                                              GRIPPER_STATUS_PUBLISHER_TOPIC)

        self.mining_control_subscriber_topic = rospy.get_param("~mining_control_subscriber_topic",
                                                             MINING_CONTROL_SUBSCRIBER_TOPIC)
        
        self.drill_control_subscriber_topic = rospy.get_param("~drill_control_subscriber_topic",
                                                               DRILL_CONTROL_SUBSCRIBER_TOPIC)

        self.mining_status_publisher_topic = rospy.get_param("~mining_status_publisher_topic",
                                                             MINING_STATUS_PUBLISHER_TOPIC)

        self.camera_control_subscriber_topic = rospy.get_param("~camera_control_subscriber_topic",
                                                               CAMERA_CONTROL_SUBSCRIBER_TOPIC)

        self.wait_time = 1.0 / rospy.get_param("~hertz", DEFAULT_HERTZ)

        self.gripper_node = None  # type:minimalmodbus.Instrument
        #self.mining_node = None  # type:minimalmodbus.Instrument
        #self.drill_node = None  # type:minimalmodbus.Instrument
        self.science_mech_node = None #type:minimalmodbus.Instrument

        self.gripper_node_present = False
        #self.mining_node_present = True
        #self.drill_node_present = True
        self.science_mech_node_present = True

        self.connect_to_nodes()
        self.check_which_nodes_present()

        # ##### Subscribers #####
        self.gripper_control_subscriber = rospy.Subscriber(self.gripper_control_subscriber_topic, GripperControlMessage, self.gripper_control_message_received__callback)

        self.mining_control_subscriber = rospy.Subscriber(self.mining_control_subscriber_topic, MiningControlMessage, self.mining_control_message_received__callback)
        
        self.drill_control_subscriber = rospy.Subscriber(self.drill_control_subscriber_topic, DrillControlMessage, self.drill_control_message_received__callback)
        
        self.camera_control_subscriber = rospy.Subscriber(self.camera_control_subscriber_topic, CameraControlMessage, self.camera_control_message_received__callback)

        # ##### Publishers #####
        self.gripper_status_publisher = rospy.Publisher(self.gripper_status_publisher_topic, GripperStatusMessage, queue_size=1)

        self.mining_status_publisher = rospy.Publisher(self.mining_status_publisher_topic, MiningStatusMessage, queue_size=1)

        # ##### Misc #####
        self.modbus_nodes_seen_time = time()

        # ##### Mining Variables #####
        self.mining_registers = [0] * MINING_HALF_REG_LIMIT  # Weird stuff to read twice to resolve CRC errors
        self.mining_registers_part_2 = [0] * MINING_REMAINING_REGS  # For reading the last 17 registers
        self.gripper_registers = None
        
        self.drill_registers = None

        self.mining_control_message = None  # type:MiningControlMessage
        self.new_mining_control_message = False

        self.gripper_control_message = None
        self.new_gripper_control_message = False
        
        self.drill_control_message = None  # type:DrillControlMessage
        self.new_drill_control_message = False

        self.camera_control_message = None  # type: CameraControlMessage
        self.new_camera_control_message = False

        self.failed_gripper_modbus_count = 0
        #self.failed_mining_modbus_count = 0
        #self.failed_dril_modbus_count = 0
        self.failed_science_mech_modbus_count = 0

        self.which_effector = self.EFFECTORS.index("GRIPPER")

        self.gripper_position_status = 0

        self.linear_curr_position = 0
        self.motor_curr_position = 0
        self.rack_number_steps = 0

        self.run()

    def __setup_minimalmodbus_for_485(self):
        self.gripper_node.serial = serial.rs485.RS485(self.gripper_port, baudrate=self.baud, timeout=GRIPPER_TIMEOUT)
        self.gripper_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.mining_node.serial = serial.rs485.RS485(self.mining_port, baudrate=self.baud, timeout=MINING_TIMEOUT)
        self.mining_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)

        self.drill_node.serial = serial.rs485.RS485(self.drill_port, baudrate=self.baud, timeout=DRILL_TIMEOUT)
        self.drill_node.serial.rs485_mode = serial.rs485.RS485Settings(rts_level_for_rx=1, rts_level_for_tx=0, delay_before_rx=RX_DELAY, delay_before_tx=TX_DELAY)


    def run(self):
        while not rospy.is_shutdown():
            if self.which_effector == self.EFFECTORS.index("GRIPPER"):
                try:
                    self.run_arm()
                    self.failed_gripper_modbus_count = 0
                except Exception, e:
                    print e
                    self.failed_gripper_modbus_count += 1

                if self.failed_gripper_modbus_count == FAILED_GRIPPER_MODBUS_LIMIT:
                    print "Gripper not present. Trying mining."
                    self.which_effector = self.EFFECTORS.index("MINING")

            elif self.which_effector == self.EFFECTORS.index("MINING"):
                try:
                    self.run_mining()
                    self.failed_mining_modbus_count = 0
                except Exception, e:
                    print e
                    self.failed_mining_modbus_count += 1

                if self.failed_mining_modbus_count == FAILED_MINING_MODBUS_LIMIMT:
                    print "No effectors present. Exiting...."
                    return

    def run_arm(self):
        self.process_gripper_control_message()
        self.send_gripper_status_message()

    def run_mining(self):
        self.process_mining_control_message()
        self.send_mining_status_message()
        self.process_drill_control_messages()
        self.process_camera_control_message()

    def connect_to_nodes(self):
        self.gripper_node = minimalmodbus.Instrument(self.gripper_port, int(self.gripper_node_id))
        #self.mining_node = minimalmodbus.Instrument(self.mining_port, int(self.mining_node_id))
        #self.drill_node = minimalmodbus.Instrument(self.drill_port, int(self.drill_node_id))
        self.science_mech_node = minimalmodbus.Instrument(self.mining_port, int(self.science_mech_node_id))

        self.__setup_minimalmodbus_for_485()

    def process_mining_control_message(self):
        print("process mining control entered")
        if not self.mining_registers or not self.mining_registers_part_2:
            # Read around half of registers first to avoid reading 32 registers at a time. 
            # This seems to prevent CRC errors.
            self.mining_registers = self.mining_node.read_registers(0, MINING_HALF_REG_LIMIT)   
            self.mining_registers_part_2 = self.mining_node.read_registers(MINING_HALF_REG_LIMIT, MINING_REMAINING_REGS)
        """
        #not using pothos for comp
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["SPEED_1"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["DIR_1"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["TMP_1"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["CURRENT_1"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["SPEED_2"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["DIR_2"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["TMP_2"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["CURRENT_2"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["DIR_3"])
        comms.read(mining_nodes["ScienceMech"], mining_pothos_registers["STEP"])
        """

        print("mining registers read")
        if self.new_mining_control_message: # and self.mining_node_present :
            print(self.mining_control_message)
            motor_go_home = self.mining_control_message.linear_go_home
            linear_set_direction = self.mining_control_message.linear_set_direction
            rack_set_direction = self.mining_control_message.rack_set_direction
            using_linear = self.mining_control_message.using_linear
            using_rack = self.mining_control_message.using_rack
            linear_pos_target = self.mining_control_message.linear_target
            linear_at_base = self.mining_control_message.linear_at_base
            linear_homed = self.mining_control_message.linear_homed

            ### Linear actuator controls ###
            if using_linear is True:
                
                ### Manual control of linear actuator ###
                if linear_pos_target >= 0 && linear_at_base == False && linear_set_direction = 0:
                    new_linear_target = linear_current_pos + linear_pos_target
                elif linear_pos_target >= 0 && linear_at_base == True && linear_set_direction = 0:
                    print("motor needs to go home, it is at the base")
                    motor_go_home = True
                else 
                    new_linear_target = 0

                if linear_pos_target >= 0 && linear_homed == False && linear_set_direction = 1:
                    new_linear_target = linear_current_pos + linear_pos_target
                elif linear_pos_target >= 0 && linear_homed == True && linear_set_direction = 1
                    print("Motor is already at the top of the actuator!")
                    new_linear_target = 0
                else 
                    new_linear_target = 0

                linear_stop = self.mining_control_message.linear_stop

                if motor_go_home:
                    self.mining_registers[MINING_MODBUS_REGISTERS["MOTOR_GO_HOME"]] = 1
                    print("MOTOR_GO_HOME is TRUE")
                    self.science_mech_node.write_registers(0, self.mining_registers)

                if linear_pos_target != 0 
                    self.mining_registers[MINING_MODBUS_REGISTERS["LINEAR_SET_POSITION_TARGET"]] = new_linear_target
                
                if linear_set_direction = 0:
                    self.mining_registers[MINING_MODBUS_REGISTERS["DIR_2"]] = linear_set_direction
                elif linear_set_direction = 1
                    self.mining_registers[MINING_MODBUS_REGISTERS["DIR_2"]] = linear_set_direction
            
            if using_rack is True:
            
                

                


            #print(self.mining_registers_part_2)
            self.mining_node.write_registers(0, self.mining_registers)
            #self.mining_node.write_registers(MINING_HALF_REG_LIMIT, self.mining_registers_part_2)
            print("wrote registers...")

            self.modbus_nodes_seen_time = time()
            self.new_mining_control_message = False

    def process_camera_control_message(self):
        if self.new_camera_control_message:
            self.mining_registers[MINING_MODBUS_REGISTERS_PART_2["MOTOR_GO_HOME"]] = 0
            self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["OVERTRAVEL"]] = 0

            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_CHANGE_VIEW"]] = self.camera_control_message.cam_change_view
            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_ZOOM_IN"]] = self.camera_control_message.cam_zoom_in
            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_ZOOM_OUT"]] = self.camera_control_message.cam_zoom_out
            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_ZOOM_IN_FULL"]] = self.camera_control_message.cam_zoom_in_full
            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_ZOOM_OUT_FULL"]] = self.camera_control_message.cam_zoom_out_full
            self.mining_registers[MINING_MODBUS_REGISTERS["CAM_SHOOT"]] = self.camera_control_message.cam_shoot

            self.mining_node.write_registers(0, self.mining_registers)
            self.mining_node.write_registers(MINING_HALF_REG_LIMIT, self.mining_registers_part_2)
            self.modbus_nodes_seen_time = time()

            self.new_camera_control_message = False

    def send_mining_status_message(self):
        print("send mining statuses entered")
        if self.mining_node_present:
            self.mining_registers = self.mining_node.read_registers(0, MINING_HALF_REG_LIMIT)
            self.mining_registers_part_2 = self.mining_node.read_registers(MINING_HALF_REG_LIMIT, MINING_REMAINING_REGS)
            print("read mining registers")
            
            print("making status message")
            message = MiningStatusMessage()
            print("made status message")

            self.linear_curr_position = message.linear_current_position
            self.motor_curr_position = message.motor_current_position

            message.linear_current_position = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["LINEAR_CURRENT_POSITION"]]

            message.motor_current_position = self.mining_registers[MINING_MODBUS_REGISTERS_PART_2["MOTOR_CURRENT_POSITION"]]

            message.temp1 = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["TEMP1"]]
            message.temp2 = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["TEMP2"]]

            message.motor_current = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["MOTOR_CURRENT"]]
            message.linear_current = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["LINEAR_CURRENT"]]

            message.switch1_out = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["SWITCH1_OUT"]]
            message.switch2_out = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["SWITCH2_OUT"]]
            message.homing_needed = self.mining_registers_part_2[MINING_MODBUS_REGISTERS_PART_2["HOMING_NEEDED"]]

            print(message)
            print("publishing message...")
            self.mining_status_publisher.publish(message)

            self.modbus_nodes_seen_time = time()

    def process_gripper_control_message(self):
        if not self.gripper_registers:
            self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))
            print(self.gripper_registers)

        if self.new_gripper_control_message:
            if self.gripper_control_message.should_home:
                print("GRIPPER SHOULD_HOME TRUE")
                self.gripper_registers[GRIPPER_MODBUS_REGISTERS["HOME"]] = 1
                self.gripper_node.write_registers(0, self.gripper_registers)

                homing_complete = False

                #gripper_homing_time = time()
                while not homing_complete:
                    print("entered homing while")
                    #time_elapsed = time() - gripper_homing_time
                    self.gripper_registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))
                    #self.send_gripper_status_message()
                    #print("time elapsed: ", time_elapsed, "homing start time: ", self.gripper_homing_time)

                    #if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]] or time_elapsed >= 1000:
                    #print(GRIPPER_MODBUS_REGISTERS["LED"])
                    #print(GRIPPER_MODBUS_REGISTERS["LASER"])
                    #print(GRIPPER_MODBUS_REGISTERS["IS_HOMED"])
                    #print(self.gripper_registers[10])
                    #print(self.gripper_registers[6])
                    #print(self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]])
                    #print(self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]])

                    print self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]]
                    if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]]:
                        homing_complete = True
                        self.gripper_registers = None
                        print("GRIPPER HOMING COMPLETE")
                        if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["IS_HOMED"]]:
                            print("is_homed true")
                        #gripper_homing_time = 0

            else:
                if self.gripper_control_message.toggle_light:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LED"]] = 0 if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LED"]] else 1
                    self.gripper_control_message.toggle_light = False

                if self.gripper_control_message.toggle_laser:
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]] = 0 if self.gripper_registers[GRIPPER_MODBUS_REGISTERS["LASER"]] else 1
                    self.gripper_control_message.toggle_laser = False

                gripper_target = self.gripper_control_message.target

                if INT16_MIN < gripper_target < INT16_MAX:
                    new_position = self.gripper_position_status + gripper_target
                    self.gripper_registers[GRIPPER_MODBUS_REGISTERS["TARGET"]] = min(max(new_position, 0), INT16_MAX)

                self.gripper_node.write_registers(0, self.gripper_registers)
                print(self.gripper_registers)

        self.gripper_control_message = None
        self.new_gripper_control_message = False

    def send_gripper_status_message(self):
        registers = self.gripper_node.read_registers(0, len(GRIPPER_MODBUS_REGISTERS))

        message = GripperStatusMessage()
        message.position_raw = registers[GRIPPER_MODBUS_REGISTERS["POSITION"]]
        self.gripper_position_status = message.position_raw
        message.temp = registers[GRIPPER_MODBUS_REGISTERS["TEMP"]]
        message.light_on = registers[GRIPPER_MODBUS_REGISTERS["LED"]]
        message.laser_on = registers[GRIPPER_MODBUS_REGISTERS["LASER"]]
        message.current = registers[GRIPPER_MODBUS_REGISTERS["CURRENT"]]
        message.distance = registers[GRIPPER_MODBUS_REGISTERS["DISTANCE"]]

        self.gripper_status_publisher.publish(message)

    def process_drill_control_messages(self):
        if self.new_drill_control_message and self.drill_node_present:
            self.drill_registers[DRILL_MODBUS_REGISTERS["DIRECTION"]] = self.drill_control_message.direction
            self.drill_registers[DRILL_MODBUS_REGISTERS["SPEED"]] = self.drill_control_message.speed
 
            self.drill_node.write_registers(0, self.drill_registers)
            self.gripper_control_message = None
            self.modbus_nodes_seen_time = time()
            self.new_drill_control_message = False

    def gripper_control_message_received__callback(self, control_message):
        self.gripper_control_message = control_message
        self.new_gripper_control_message = True

    def mining_control_message_received__callback(self, control_message):
        self.mining_control_message = control_message
        self.new_mining_control_message = True

    def drill_control_message_received__callback(self, control_message):
        self.drill_control_message = control_message
        self.new_drill_control_message = True

    def camera_control_message_received__callback(self, control_message):
        self.camera_control_message = control_message
        self.new_camera_control_message = True


if __name__ == "__main__":
    EffectorsControl()
