# encoding: utf-8

import os
import math
import platform


########################################################################
# Constants

# Brewery execution host
IS_HOST_DEVICE_ARM                     = platform.machine() == "armv5tel"
IS_HOST_DEVICE_PC                      = not IS_HOST_DEVICE_ARM

# Field
FIELD_Y_SIZE                           = 3.0
FIELD_X_SIZE                           = 2.0

MAIN_ROBOT_GYRATION_RADIUS = 0.208
SECONDARY_ROBOT_GYRATION_RADIUS = 0.100535

def setup_definitions(is_main_robot):
    globals()["IS_MAIN_ROBOT"]         = is_main_robot

    if is_main_robot:
        globals()["ROBOT_X_SIZE"]          = 0.300
        globals()["ROBOT_Y_SIZE"]          = 0.290
        globals()["ROBOT_CENTER_X"]        = 0.150
        globals()["ROBOT_CENTER_Y"]        = 0.145
        globals()["ROBOT_GYRATION_RADIUS"] = MAIN_ROBOT_GYRATION_RADIUS
        globals()["LEFT_START_Y"]          = 0.07 + ROBOT_CENTER_X
        globals()["LEFT_START_X"]          = 1.0
        globals()["LEFT_START_ANGLE"]      = 0.0
        globals()["INPUT"]                 = MAIN_INPUT
    else:
        globals()["ROBOT_X_SIZE"]          = 0.155
        globals()["ROBOT_Y_SIZE"]          = 0.165
        globals()["ROBOT_CENTER_X"]        = 0.0775
        globals()["ROBOT_CENTER_Y"]        = 0.0825
        globals()["ROBOT_GYRATION_RADIUS"] = SECONDARY_ROBOT_GYRATION_RADIUS
        globals()["LEFT_START_Y"]          = 0.55
        globals()["LEFT_START_X"]          = 1.0
        globals()["LEFT_START_ANGLE"]      = math.pi / 2.0
        globals()["INPUT"]                 = SECONDARY_INPUT

    globals()["ROBOT_VMAX_LIMIT"]                  = 88.0

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
TURRET_SHORT_DISTANCE_DETECTION_RANGE  = 0.55
TURRET_LONG_DISTANCE_DETECTION_RANGE   = 1.0
TURRET_SHORT_DISTANCE_DETECTION_ID     = 250
TURRET_LONG_DISTANCE_DETECTION_ID      = 255


########################################################################
# Enums


class Enum(object):

    def __init__(self, description, **kwargs):
        self.description = description
        self.lookup_by_name = {}
        self.lookup_by_value = {}
        auto_value = 0
        for enum_item_name, enum_item_value in list(kwargs.items()):
            if enum_item_value is None:
                enum_item_value = auto_value
            auto_value = enum_item_value + 1
            globals()[enum_item_name] = enum_item_value
            self.lookup_by_name[enum_item_name] = enum_item_value
            self.lookup_by_value[enum_item_value] = enum_item_name




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

TRAJECTORY = Enum("Navigation result",
    TRAJECTORY_DESTINATION_REACHED     = 0,
    TRAJECTORY_BLOCKED                 = 1,
    TRAJECTORY_OPPONENT_DETECTED       = 2,
    TRAJECTORY_DESTINATION_UNREACHABLE = 3,
)

OPPONENT_ROBOT = Enum("Detected opponent robot",
    OPPONENT_ROBOT_MAIN      = 0,
    OPPONENT_ROBOT_SECONDARY = 1,
    OPPONENT_ROBOT_TEAMMATE  = 2,
)

OPPONENT_DISTANCE = Enum("Opponent distance",
    OPPONENT_DISTANCE_NEAR = 0,
    OPPONENT_DISTANCE_FAR  = 1,
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
    MAIN_INPUT_START                = 0,
    MAIN_INPUT_TEAM                 = 1,
    MAIN_INPUT_ROBOT_INIT           = 2,
    MAIN_INPUT_LEFT_BULB_PRESENCE   = 4,
    MAIN_INPUT_LEFT_STAND_PRESENCE  = 5,
    MAIN_INPUT_RIGHT_BULB_PRESENCE  = 6,
    MAIN_INPUT_RIGHT_STAND_PRESENCE = 7,
    MAIN_INPUT_LEFT_SCANNER         = 8,
    MAIN_INPUT_RIGHT_SCANNER        = 9,
)

SECONDARY_INPUT = Enum("Inputs",
    SECONDARY_INPUT_START        = 0,
    SECONDARY_INPUT_TEAM         = 1,
    SECONDARY_INPUT_ROBOT_INIT   = 2,
    SECONDARY_INPUT_CUP_PRESENCE = 3,
)

def makeServoMoveCommand(servo, value):
    return (servo[0], servo[1], SERVO_COMMAND_MOVE, value, servo[2])

def makeServoSetupCommand(servo, value):
    return (servo[0], servo[1], SERVO_COMMAND_SETUP_SPEED, value, servo[2])


DEFAULT_SERVOS_TIMEOUT_MS = 2000

DEFAULT_RX_SERVOS_TIMEOUT_MS = DEFAULT_SERVOS_TIMEOUT_MS
DEFAULT_AX_SERVOS_TIMEOUT_MS = DEFAULT_SERVOS_TIMEOUT_MS


# Main robot actuators

LEFT_BUILDER_PLIERS_LEFT         = (ACTUATOR_TYPE_SERVO_AX, 0, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_PLIERS_LEFT_CLOSE  = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 1234)
LEFT_BUILDER_PLIERS_LEFT_OPEN   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 1234)

LEFT_BUILDER_PLIERS_RIGHT        = (ACTUATOR_TYPE_SERVO_AX, 1, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_PLIERS_RIGHT_CLOSE  = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 1234)
LEFT_BUILDER_PLIERS_RIGHT_OPEN   = makeServoMoveCommand(LEFT_BUILDER_PLIERS_RIGHT, 1234)

LEFT_BUILDER_GRIPPER_LEFT        = (ACTUATOR_TYPE_SERVO_AX, 2, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_GRIPPER_LEFT_CLOSE  = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 1234)
LEFT_BUILDER_GRIPPER_LEFT_GUIDE  = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 1234)
LEFT_BUILDER_GRIPPER_LEFT_OPEN   = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_LEFT, 1234)

LEFT_BUILDER_GRIPPER_RIGHT       = (ACTUATOR_TYPE_SERVO_AX, 3, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_GRIPPER_RIGHT_CLOSE = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 1234)
LEFT_BUILDER_GRIPPER_RIGHT_GUIDE = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 1234)
LEFT_BUILDER_GRIPPER_RIGHT_OPEN  = makeServoMoveCommand(LEFT_BUILDER_GRIPPER_RIGHT, 1234)

LEFT_BUILDER_LIGHTER             = (ACTUATOR_TYPE_SERVO_AX, 4, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_LIGHTER_CLOSE       = makeServoMoveCommand(LEFT_BUILDER_LIGHTER, 1234)
LEFT_BUILDER_LIGHTER_OPEN        = makeServoMoveCommand(LEFT_BUILDER_LIGHTER, 1234)

LEFT_BUILDER_ELEVATOR            = (ACTUATOR_TYPE_SERVO_RX, 0, DEFAULT_RX_SERVOS_TIMEOUT_MS)
LEFT_BUILDER_ELEVATOR_DOWN       = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 1234)
LEFT_BUILDER_ELEVATOR_PLATFORM   = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 1234)
LEFT_BUILDER_ELEVATOR_UP         = makeServoMoveCommand(LEFT_BUILDER_ELEVATOR, 1234)


RIGHT_BUILDER_PLIERS_LEFT         = (ACTUATOR_TYPE_SERVO_AX, 5, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_PLIERS_LEFT_CLOSE   = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 1234)
RIGHT_BUILDER_PLIERS_LEFT_OPEN    = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_LEFT, 1234)

RIGHT_BUILDER_PLIERS_RIGHT        = (ACTUATOR_TYPE_SERVO_AX, 6, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_PLIERS_RIGHT_CLOSE  = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 1234)
RIGHT_BUILDER_PLIERS_RIGHT_OPEN   = makeServoMoveCommand(RIGHT_BUILDER_PLIERS_RIGHT, 1234)

RIGHT_BUILDER_GRIPPER_LEFT        = (ACTUATOR_TYPE_SERVO_AX, 7, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_GRIPPER_LEFT_CLOSE  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 1234)
RIGHT_BUILDER_GRIPPER_LEFT_GUIDE  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 1234)
RIGHT_BUILDER_GRIPPER_LEFT_OPEN   = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_LEFT, 1234)

RIGHT_BUILDER_GRIPPER_RIGHT       = (ACTUATOR_TYPE_SERVO_AX, 8, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 1234)
RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 1234)
RIGHT_BUILDER_GRIPPER_RIGHT_OPEN  = makeServoMoveCommand(RIGHT_BUILDER_GRIPPER_RIGHT, 1234)

RIGHT_BUILDER_LIGHTER             = (ACTUATOR_TYPE_SERVO_AX, 9, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_LIGHTER_CLOSE       = makeServoMoveCommand(RIGHT_BUILDER_LIGHTER, 1234)
RIGHT_BUILDER_LIGHTER_OPEN        = makeServoMoveCommand(RIGHT_BUILDER_LIGHTER, 1234)

RIGHT_BUILDER_ELEVATOR            = (ACTUATOR_TYPE_SERVO_RX, 1, DEFAULT_RX_SERVOS_TIMEOUT_MS)
RIGHT_BUILDER_ELEVATOR_DOWN       = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 1234)
RIGHT_BUILDER_ELEVATOR_PLATFORM   = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 1234)
RIGHT_BUILDER_ELEVATOR_UP         = makeServoMoveCommand(RIGHT_BUILDER_ELEVATOR, 1234)


LIGHTER_GRIPPER       = (ACTUATOR_TYPE_SERVO_AX, 10, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LIGHTER_GRIPPER_CLOSE = makeServoMoveCommand(LIGHTER_GRIPPER, 1234)
LIGHTER_GRIPPER_OPEN  = makeServoMoveCommand(LIGHTER_GRIPPER, 1234)

LIGHTER_ELEVATOR      = (ACTUATOR_TYPE_SERVO_RX, 2, DEFAULT_RX_SERVOS_TIMEOUT_MS)
LIGHTER_ELEVATOR_DOWN = makeServoMoveCommand(LIGHTER_ELEVATOR, 1234)
LIGHTER_ELEVATOR_BULB = makeServoMoveCommand(LIGHTER_ELEVATOR, 1234)
LIGHTER_ELEVATOR_UP   = makeServoMoveCommand(LIGHTER_ELEVATOR, 1234)


LEFT_CLAPMAN       = (ACTUATOR_TYPE_SERVO_AX, 11, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_CLAPMAN_CLOSE = makeServoMoveCommand(LEFT_CLAPMAN, 1234)
LEFT_CLAPMAN_OPEN  = makeServoMoveCommand(LEFT_CLAPMAN, 1234)

RIGHT_CLAPMAN       = (ACTUATOR_TYPE_SERVO_AX, 12, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_CLAPMAN_CLOSE = makeServoMoveCommand(RIGHT_CLAPMAN, 1234)
RIGHT_CLAPMAN_OPEN  = makeServoMoveCommand(RIGHT_CLAPMAN, 1234)


# Secondary robot actuators

LEFT_CARPET_DROPPER       = (ACTUATOR_TYPE_SERVO_AX, 0, DEFAULT_AX_SERVOS_TIMEOUT_MS)
LEFT_CARPET_DROPPER_CLOSE = makeServoMoveCommand(LEFT_CARPET_DROPPER, 1234)
LEFT_CARPET_DROPPER_OPEN  = makeServoMoveCommand(LEFT_CARPET_DROPPER, 1234)

RIGHT_CARPET_DROPPER       = (ACTUATOR_TYPE_SERVO_AX, 1, DEFAULT_AX_SERVOS_TIMEOUT_MS)
RIGHT_CARPET_DROPPER_CLOSE = makeServoMoveCommand(RIGHT_CARPET_DROPPER, 1234)
RIGHT_CARPET_DROPPER_OPEN  = makeServoMoveCommand(RIGHT_CARPET_DROPPER, 1234)

LEFT_CARPET_EJECTOR_HOLD  = (ACTUATOR_TYPE_OUTPUT, 0, ACTION_OFF)
LEFT_CARPET_EJECTOR_THROW = (ACTUATOR_TYPE_OUTPUT, 0, ACTION_ON)

RIGHT_CARPET_EJECTOR_HOLD  = (ACTUATOR_TYPE_OUTPUT, 1, ACTION_OFF)
RIGHT_CARPET_EJECTOR_THROW = (ACTUATOR_TYPE_OUTPUT, 1, ACTION_ON)

CUP_GRIPPER           = (ACTUATOR_TYPE_SERVO_AX, 2, DEFAULT_AX_SERVOS_TIMEOUT_MS)
CUP_GRIPPER_CLOSE     = makeServoMoveCommand(CUP_GRIPPER, 1234)
CUP_GRIPPER_ON_CUP    = makeServoMoveCommand(CUP_GRIPPER, 1234)
CUP_GRIPPER_HALF_OPEN = makeServoMoveCommand(CUP_GRIPPER, 1234)
CUP_GRIPPER_OPEN      = makeServoMoveCommand(CUP_GRIPPER, 1234)
