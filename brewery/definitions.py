# encoding: utf-8

import os
import math
import platform
import inspect
import sys


########################################################################
# Constants

# Brewery execution host
import collections

IS_HOST_DEVICE_ARM                     = platform.machine() == "armv5tel"
IS_HOST_DEVICE_PC                      = not IS_HOST_DEVICE_ARM

# Field
FIELD_Y_SIZE                           = 3.0
FIELD_X_SIZE                           = 2.0

MAIN_ROBOT_GYRATION_RADIUS = 0.208
SECONDARY_ROBOT_GYRATION_RADIUS = 0.100535

def get_servos_commands():
    all_commands=collections.defaultdict(dict)
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        if not inspect.isfunction(obj) and not inspect.isclass(obj):
            try:
                if len(obj) == 4:
                    servo_id = obj[0]
                    servo_value = obj[2]
                    all_commands[servo_id][servo_value]=name
            except TypeError: pass

    return all_commands



def setup_definitions(is_main_robot):
    globals()["IS_MAIN_ROBOT"]         = is_main_robot

    if is_main_robot:
        globals()["ROBOT_X_SIZE"]          = 0.300
        globals()["ROBOT_Y_SIZE"]          = 0.290
        globals()["ROBOT_CENTER_X"]        = 0.150
        globals()["ROBOT_CENTER_Y"]        = 0.145
        globals()["ROBOT_GYRATION_RADIUS"] = MAIN_ROBOT_GYRATION_RADIUS
        globals()["INPUT"]                 = MAIN_INPUT
        globals()["SERVOS_IDS"]            = MAIN_SERVO_IDS
    else:
        globals()["ROBOT_X_SIZE"]          = 0.155
        globals()["ROBOT_Y_SIZE"]          = 0.165
        globals()["ROBOT_CENTER_X"]        = 0.0775
        globals()["ROBOT_CENTER_Y"]        = 0.0825
        globals()["ROBOT_GYRATION_RADIUS"] = SECONDARY_ROBOT_GYRATION_RADIUS
        globals()["INPUT"]                 = SECONDARY_INPUT
        globals()["SERVOS_IDS"]            = SECONDARY_SERVO_IDS

    globals()["ROBOT_VMAX_LIMIT"]                  = 88.0
    globals()["ALL_SERVO_COMMANDS"]=get_servos_commands()


# Rule specific
MATCH_DURATION_MS                      = 90000
FUNNY_ACTION_DURATION_MS               = 0
FULL_DURATION_MS                       = MATCH_DURATION_MS + FUNNY_ACTION_DURATION_MS
BREWERY_LIFETIME_MS                    = FULL_DURATION_MS + 5000
TEAM_LEFT_COLOR                        = "#f9c001"
TEAM_RIGHT_COLOR                       = "#00aa1e"

# Timing
KEEP_ALIVE_DELAY_MS                    = 250
KEEP_ALIVE_MINIMUM_AGE_S               = (KEEP_ALIVE_DELAY_MS * 4.0 / 5.0) / 1000.0

#Teammate collaboration
TEAMMATE_INFO_DELAY_MS                 = 100 #Time between two position information sent to teammate
TEAMMATE_POSITION_IN_MAP               = True

# Remote device connection
if IS_HOST_DEVICE_ARM:
    REMOTE_IP                          = "pic"
else:
    REMOTE_IP                          = "localhost"
REMOTE_PORT                            = 7001
REMOTE_LOG_PORT                        = 23

if IS_HOST_DEVICE_ARM:
    MAIN_INTERBOT_IP                   = "main"
else:
    MAIN_INTERBOT_IP                   = "localhost"
MAIN_INTERBOT_PORT                     = 7002

MEDIAPLAYER_IP                         = "mediaplayer"
MEDIAPLAYER_PORT                       = 7003

# Serial port
if IS_HOST_DEVICE_ARM:
    SERIAL_PORT_PATH                   = "/dev/ttyUSB0"
else:
    SERIAL_PORT_PATH                   = None
SERIAL_PORT_SPEED                      = 115200

# Leds:
if IS_HOST_DEVICE_ARM:
    ORANGE_LED_DEVICE_PATH             = "/sys/class/leds/dockstar:orange:misc/brightness"
    GREEN_LED_DEVICE_PATH              = "/sys/class/leds/dockstar:green:health/brightness"
else:
    ORANGE_LED_DEVICE_PATH             = None
    GREEN_LED_DEVICE_PATH              = None

# Log directory
LOG_DIR                                = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "logs")

# Blocking opponent handling
DEFAULT_OPPONENT_WAIT_MS               = 2000
DEFAULT_OPPONENT_DISAPPEAR_RETRIES     = -1

# Turret detection ranges
TURRET_DETECTION_RANGE_START = 28
TURRET_DETECTION_RANGE_END   = 88

TURRET_SHORT_DISTANCE_DETECTION_RANGE  = 0.55
TURRET_LONG_DISTANCE_DETECTION_RANGE   = 1.0
TURRET_SHORT_DISTANCE_DETECTION_ID     = 250
TURRET_LONG_DISTANCE_DETECTION_ID      = 255

# For security reasons
UNDEFINED_ACTUATOR_VALUE = None

# Asser settings

ROBOT_DEFAULT_RATIO_DECC = 1.8


########################################################################
# Enums


class Enum(object):

    def __init__(self, description, **kwargs):
        self.description = description
        self.lookup_by_name = {}
        self.lookup_by_value = {}
        for enum_item_name, enum_item_value in list(kwargs.items()):
            globals()[enum_item_name] = enum_item_value
            self.lookup_by_name[enum_item_name] = enum_item_value
            self.lookup_by_value[enum_item_value] = enum_item_name

    def __iter__(self):
        yield from self.lookup_by_name.items()


    def values(self):
        return self.lookup_by_name.values()




REMOTE_DEVICE = Enum("Remote hardware type",
    REMOTE_DEVICE_PIC       = 0,
    REMOTE_DEVICE_SIMULATOR = 1,
    REMOTE_DEVICE_UNKNOWN   = 2,
)

CONTROLLER_STATUS = Enum("Controller status",
    CONTROLLER_STATUS_BUSY  = 0,
    CONTROLLER_STATUS_READY = 1,
)

TEAM = Enum("Team color",
    TEAM_RIGHT   = 0,
    TEAM_LEFT    = 1,
    TEAM_UNKNOWN = 2,
)

DIRECTION = Enum("Direction",
    DIRECTION_AUTO      =  0,
    DIRECTION_FORWARD  =   1,
    DIRECTION_BACKWARDS = -1,
)

REASON = Enum("Goto finished reason",
    REASON_DESTINATION_REACHED = 0,
    REASON_BLOCKED_FRONT       = 1,
    REASON_BLOCKED_BACK        = 2,
    REASON_STOP_REQUESTED      = 3,
)

AXIS = Enum("Axis",
    AXIS_X = 0,
    AXIS_Y = 1,
)

ACTUATOR_TYPE = Enum("Actuator type",
    ACTUATOR_TYPE_SERVO_AX = 0,
    ACTUATOR_TYPE_SERVO_RX = 1,
    ACTUATOR_TYPE_OUTPUT   = 2,
    ACTUATOR_TYPE_PWM      = 3,
)

SERVO_COMMAND = Enum("Servo subcommand",
    SERVO_COMMAND_MOVE          = 0,
    SERVO_COMMAND_SETUP_SPEED   = 1,
    SERVO_COMMAND_POSITION      = 2,
    SERVO_COMMAND_TORQUE_ENABLE = 3,
)

SERVO_STATUS = Enum("Servo status",
    SERVO_STATUS_TIMED_OUT = 0,
    SERVO_STATUS_SUCCESS   = 1,
)

ACTION = Enum("Action",
    ACTION_OFF = 0,
    ACTION_ON  = 1,
)

KIND = Enum("Input kind",
    KIND_EVENT = 0,
    KIND_READ  = 1,
)

TRAJECTORY = Enum("Navigation result",
    TRAJECTORY_DESTINATION_REACHED     = 0,
    TRAJECTORY_BLOCKED                 = 1,
    TRAJECTORY_OPPONENT_DETECTED       = 2,
    TRAJECTORY_DESTINATION_UNREACHABLE = 3,
    TRAJECTORY_STOP_REQUESTED          = 4,
)

OPPONENT_ROBOT = Enum("Detected opponent robot",
    OPPONENT_ROBOT_MAIN      = 0,
    OPPONENT_ROBOT_SECONDARY = 1,
    OPPONENT_ROBOT_TEAMMATE  = 2,
)

TURRET_INIT_MODE = Enum("Turret initialization read/write mode",
    TURRET_INIT_MODE_READ  = 0,
    TURRET_INIT_MODE_WRITE = 1,
)

GOAL_STATUS = Enum("Goal status",
    GOAL_AVAILABLE  = 0,
    GOAL_DOING      = 1,
    GOAL_DONE       = 2,
    GOAL_FAILED     = 3,
    GOAL_DISABLED   = 4,
)

SIDE = Enum("Side",
    SIDE_LEFT  = 0,
    SIDE_RIGHT = 1,
)

MAIN_INPUT = Enum("Inputs",
    MAIN_INPUT_START                = 16,
    MAIN_INPUT_TEAM                 = 3,
    MAIN_INPUT_ROBOT_INIT           = 7,
    MAIN_INPUT_LEFT_BULB_PRESENCE   = 14,
    MAIN_INPUT_LEFT_STAND_PRESENCE  = 0,
    MAIN_INPUT_RIGHT_BULB_PRESENCE  = 2,
    MAIN_INPUT_RIGHT_STAND_PRESENCE = 10,
    MAIN_INPUT_LEFT_SCANNER         = 12,
    MAIN_INPUT_RIGHT_SCANNER        = 13,
)

SECONDARY_INPUT = Enum("Inputs",
    SECONDARY_INPUT_START        = 16,
    SECONDARY_INPUT_TEAM         = 3,
    SECONDARY_INPUT_ROBOT_INIT   = 7,
    SECONDARY_INPUT_CUP_PRESENCE = 12,
)

def makeServoMoveCommand(servo, value, timeout=None):
    return servo[0], SERVO_COMMAND_MOVE, value, timeout or servo[1]

def makeServoSetupCommand(servo, value):
    return servo[0], SERVO_COMMAND_SETUP_SPEED, value, servo[1]

def makeServoReadCommand(servo):
    return servo[0], SERVO_COMMAND_POSITION, 0, servo[1]

def makeServoTorqueControl(servo, status):
    return servo[0], SERVO_COMMAND_TORQUE_ENABLE, 1 if status else 0, servo[1]




DEFAULT_SERVOS_TIMEOUT_MS = 2000

DEFAULT_RX_SERVOS_TIMEOUT_MS = DEFAULT_SERVOS_TIMEOUT_MS
DEFAULT_AX_SERVOS_TIMEOUT_MS = DEFAULT_SERVOS_TIMEOUT_MS


# Main robot actuators

MAIN_SERVO_IDS = Enum("Main robot servos",
    LEFT_BUILDER_PLIERS_LEFT_ID    = (ACTUATOR_TYPE_SERVO_AX, 101),
    LEFT_BUILDER_PLIERS_RIGHT_ID   = (ACTUATOR_TYPE_SERVO_AX, 102),
    LEFT_BUILDER_GRIPPER_LEFT_ID   = (ACTUATOR_TYPE_SERVO_AX, 103),
    LEFT_BUILDER_GRIPPER_RIGHT_ID  = (ACTUATOR_TYPE_SERVO_AX, 104),
    LEFT_BUILDER_LIGHTER_ID        = (ACTUATOR_TYPE_SERVO_AX, 105),
    LEFT_BUILDER_ELEVATOR_ID       = (ACTUATOR_TYPE_SERVO_RX, 107),
    RIGHT_BUILDER_PLIERS_LEFT_ID   = (ACTUATOR_TYPE_SERVO_AX, 201),
    RIGHT_BUILDER_PLIERS_RIGHT_ID  = (ACTUATOR_TYPE_SERVO_AX, 202),
    RIGHT_BUILDER_GRIPPER_LEFT_ID  = (ACTUATOR_TYPE_SERVO_AX, 203),
    RIGHT_BUILDER_GRIPPER_RIGHT_ID = (ACTUATOR_TYPE_SERVO_AX, 204),
    RIGHT_BUILDER_LIGHTER_ID       = (ACTUATOR_TYPE_SERVO_AX, 205),
    RIGHT_BUILDER_ELEVATOR_ID      = (ACTUATOR_TYPE_SERVO_RX, 207),
    LIGHTER_GRIPPER_ID             = (ACTUATOR_TYPE_SERVO_AX, 11),
    LIGHTER_ELEVATOR_ID            = (ACTUATOR_TYPE_SERVO_RX, 5),
    LEFT_CLAPMAN_ID                = (ACTUATOR_TYPE_SERVO_AX, 106),
    RIGHT_CLAPMAN_ID               = (ACTUATOR_TYPE_SERVO_AX, 206),
)

MAIN_OUTPUT_IDS = Enum("Main robot outputs")

MAIN_PWM_IDS = Enum("Main robot PWMs")

PLIERS_HOLD_TIMEOUT = 500
GRIPPERS_HOLD_TIMEOUT = 500
SERVO_OPEN_TIMEOUT = 500

#### LEFT BUILDER ####

LEFT_BUILDER_PLIERS_LEFT         = (LEFT_BUILDER_PLIERS_LEFT_ID, 700)
LEFT_BUILDER_PLIERS_LEFT_INIT    = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 777)
LEFT_BUILDER_PLIERS_LEFT_CLOSE   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 587)
LEFT_BUILDER_PLIERS_LEFT_HOLD    = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 620, PLIERS_HOLD_TIMEOUT)
LEFT_BUILDER_PLIERS_LEFT_OPEN    = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 418, SERVO_OPEN_TIMEOUT)

LEFT_BUILDER_PLIERS_RIGHT        = (LEFT_BUILDER_PLIERS_RIGHT_ID, 700)
LEFT_BUILDER_PLIERS_RIGHT_INIT   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 245)
LEFT_BUILDER_PLIERS_RIGHT_CLOSE  = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 415)
LEFT_BUILDER_PLIERS_RIGHT_HOLD   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 395, PLIERS_HOLD_TIMEOUT)
LEFT_BUILDER_PLIERS_RIGHT_OPEN   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 630, SERVO_OPEN_TIMEOUT)

LEFT_BUILDER_GRIPPER_LEFT         = (LEFT_BUILDER_GRIPPER_LEFT_ID, 700)
LEFT_BUILDER_GRIPPER_LEFT_INIT    = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 800)
LEFT_BUILDER_GRIPPER_LEFT_CLOSE   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 621, GRIPPERS_HOLD_TIMEOUT)
LEFT_BUILDER_GRIPPER_LEFT_GUIDE   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 560)
LEFT_BUILDER_GRIPPER_LEFT_LIGHT   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 530)
LEFT_BUILDER_GRIPPER_LEFT_DEPOSIT = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 439)

LEFT_BUILDER_GRIPPER_RIGHT         = (LEFT_BUILDER_GRIPPER_RIGHT_ID, 700)
LEFT_BUILDER_GRIPPER_RIGHT_INIT    = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 223)
LEFT_BUILDER_GRIPPER_RIGHT_CLOSE   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 420, GRIPPERS_HOLD_TIMEOUT)
LEFT_BUILDER_GRIPPER_RIGHT_GUIDE   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 465)
LEFT_BUILDER_GRIPPER_RIGHT_LIGHT   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 495)
LEFT_BUILDER_GRIPPER_RIGHT_DEPOSIT = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 555)

LEFT_BUILDER_LIGHTER               = (LEFT_BUILDER_LIGHTER_ID, 700)
LEFT_BUILDER_LIGHTER_WAIT          = makeServoMoveCommand(LEFT_BUILDER_LIGHTER, 220)
LEFT_BUILDER_LIGHTER_DEPOSIT       = makeServoMoveCommand(LEFT_BUILDER_LIGHTER, 531)

LEFT_BUILDER_ELEVATOR              = (LEFT_BUILDER_ELEVATOR_ID, 700)
LEFT_BUILDER_ELEVATOR_DOWN         = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 871)
LEFT_BUILDER_ELEVATOR_PLATFORM     = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 794)
LEFT_BUILDER_ELEVATOR_UP           = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 500)

#### RIGHT BUILDER ####

RIGHT_BUILDER_PLIERS_LEFT          = (RIGHT_BUILDER_PLIERS_LEFT_ID, 700)
RIGHT_BUILDER_PLIERS_LEFT_INIT     = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 777)
RIGHT_BUILDER_PLIERS_LEFT_CLOSE    = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 592)
RIGHT_BUILDER_PLIERS_LEFT_HOLD     = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 612, PLIERS_HOLD_TIMEOUT)
RIGHT_BUILDER_PLIERS_LEFT_OPEN     = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 418, SERVO_OPEN_TIMEOUT)

RIGHT_BUILDER_PLIERS_RIGHT         = (RIGHT_BUILDER_PLIERS_RIGHT_ID, 700)
RIGHT_BUILDER_PLIERS_RIGHT_INIT    = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 245)
RIGHT_BUILDER_PLIERS_RIGHT_CLOSE   = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 423)
RIGHT_BUILDER_PLIERS_RIGHT_HOLD    = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 403, PLIERS_HOLD_TIMEOUT)
RIGHT_BUILDER_PLIERS_RIGHT_OPEN    = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 630, SERVO_OPEN_TIMEOUT)

RIGHT_BUILDER_GRIPPER_LEFT         = (RIGHT_BUILDER_GRIPPER_LEFT_ID, 700)
RIGHT_BUILDER_GRIPPER_LEFT_INIT    = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 800)
RIGHT_BUILDER_GRIPPER_LEFT_CLOSE   = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 621, GRIPPERS_HOLD_TIMEOUT)
RIGHT_BUILDER_GRIPPER_LEFT_GUIDE   = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 560)
RIGHT_BUILDER_GRIPPER_LEFT_LIGHT   = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 530)
RIGHT_BUILDER_GRIPPER_LEFT_DEPOSIT = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 439)

RIGHT_BUILDER_GRIPPER_RIGHT        = (RIGHT_BUILDER_GRIPPER_RIGHT_ID, 700)
RIGHT_BUILDER_GRIPPER_RIGHT_INIT   = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 223)
RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 420, GRIPPERS_HOLD_TIMEOUT)
RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 465)
RIGHT_BUILDER_GRIPPER_RIGHT_LIGHT  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 495)
RIGHT_BUILDER_GRIPPER_RIGHT_DEPOSIT= makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 555)

RIGHT_BUILDER_LIGHTER              = (RIGHT_BUILDER_LIGHTER_ID, 700)
RIGHT_BUILDER_LIGHTER_WAIT         = makeServoMoveCommand(RIGHT_BUILDER_LIGHTER, 270)
RIGHT_BUILDER_LIGHTER_DEPOSIT      = makeServoMoveCommand(RIGHT_BUILDER_LIGHTER, 528)

RIGHT_BUILDER_ELEVATOR             = (RIGHT_BUILDER_ELEVATOR_ID, 700)
RIGHT_BUILDER_ELEVATOR_DOWN        = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 85)
RIGHT_BUILDER_ELEVATOR_PLATFORM    = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 185)
RIGHT_BUILDER_ELEVATOR_UP          = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 481)


LIGHTER_GRIPPER       = (LIGHTER_GRIPPER_ID, 500)
LIGHTER_GRIPPER_CLOSE = makeServoMoveCommand(LIGHTER_GRIPPER, 570)
LIGHTER_GRIPPER_OPEN  = makeServoMoveCommand(LIGHTER_GRIPPER, 499)

LIGHTER_ELEVATOR      = (LIGHTER_ELEVATOR_ID, 1500)
LIGHTER_ELEVATOR_DOWN = makeServoMoveCommand(LIGHTER_ELEVATOR, 725)
LIGHTER_ELEVATOR_BULB = makeServoMoveCommand(LIGHTER_ELEVATOR, 660)
LIGHTER_ELEVATOR_UP   = makeServoMoveCommand(LIGHTER_ELEVATOR, 114)


LEFT_CLAPMAN       = (LEFT_CLAPMAN_ID, 700)
LEFT_CLAPMAN_CLOSE = makeServoMoveCommand(LEFT_CLAPMAN, 498)
LEFT_CLAPMAN_OPEN  = makeServoMoveCommand(LEFT_CLAPMAN, 206)

RIGHT_CLAPMAN       = (RIGHT_CLAPMAN_ID, 700)
RIGHT_CLAPMAN_CLOSE = makeServoMoveCommand(RIGHT_CLAPMAN, 511)
RIGHT_CLAPMAN_OPEN  = makeServoMoveCommand(RIGHT_CLAPMAN, 820)


# Secondary robot actuators

SECONDARY_SERVO_IDS = Enum("Main robot servos",
    LEFT_CARPET_DROPPER_ID  = (ACTUATOR_TYPE_SERVO_AX, 22),
    RIGHT_CARPET_DROPPER_ID = (ACTUATOR_TYPE_SERVO_AX, 21),
    CUP_GRIPPER_ID          = (ACTUATOR_TYPE_SERVO_AX, 23),
    CLAPMAN_ID              = (ACTUATOR_TYPE_SERVO_AX, 24)
)

SECONDARY_OUTPUT_IDS = Enum("Secondary robot outputs",
    LEFT_CARPET_EJECTOR_ID  = 0,
    RIGHT_CARPET_EJECTOR_ID = 2,
)

SECONDARY_PWM_IDS = Enum("Secondary robot PWMs")

LEFT_CARPET_DROPPER       = (LEFT_CARPET_DROPPER_ID, 500)
LEFT_CARPET_DROPPER_CLOSE = makeServoMoveCommand(LEFT_CARPET_DROPPER, 723)
LEFT_CARPET_DROPPER_OPEN  = makeServoMoveCommand(LEFT_CARPET_DROPPER, 510)

RIGHT_CARPET_DROPPER       = (RIGHT_CARPET_DROPPER_ID, 500)
RIGHT_CARPET_DROPPER_CLOSE = makeServoMoveCommand(RIGHT_CARPET_DROPPER, 291)
RIGHT_CARPET_DROPPER_OPEN  = makeServoMoveCommand(RIGHT_CARPET_DROPPER, 521)

LEFT_CARPET_EJECTOR_HOLD  = ((ACTUATOR_TYPE_OUTPUT, LEFT_CARPET_EJECTOR_ID), ACTION_OFF)
LEFT_CARPET_EJECTOR_THROW = ((ACTUATOR_TYPE_OUTPUT, LEFT_CARPET_EJECTOR_ID), ACTION_ON)

RIGHT_CARPET_EJECTOR_HOLD  = ((ACTUATOR_TYPE_OUTPUT, RIGHT_CARPET_EJECTOR_ID), ACTION_OFF)
RIGHT_CARPET_EJECTOR_THROW = ((ACTUATOR_TYPE_OUTPUT, RIGHT_CARPET_EJECTOR_ID), ACTION_ON)

CUP_GRIPPER           = (CUP_GRIPPER_ID, 500)
CUP_GRIPPER_CLOSE     = makeServoMoveCommand(CUP_GRIPPER, 796)
CUP_GRIPPER_ON_CUP    = makeServoMoveCommand(CUP_GRIPPER, 740)
CUP_GRIPPER_HALF_OPEN = makeServoMoveCommand(CUP_GRIPPER, 684)
CUP_GRIPPER_OPEN      = makeServoMoveCommand(CUP_GRIPPER, 550)

CLAPMAN = (CLAPMAN_ID, 500)
CLAPMAN_CLOSE = makeServoMoveCommand(CLAPMAN, 504)
CLAPMAN_OPEN  = makeServoMoveCommand(CLAPMAN, 191)

STAND_GOAL_OFFSET = -0.25
