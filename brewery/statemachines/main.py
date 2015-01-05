# encoding: utf-8

import collections
import math

import statemachine
import packets
import position
import logger
import goalmanager

from statemachine import *
from definitions import *
from commonstates import *
from position import *
from tools import *

import statemachines.testscommon as testscommon
import statemachines.testsmain as testsmain




class Main(State):

    def on_enter(self):
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield GetInputStatus(MAIN_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")




class CalibratePosition(State):

    def on_enter(self):
        if IS_HOST_DEVICE_ARM:
            yield DefinePosition(FIELD_X_SIZE - ROBOT_CENTER_X, 2.0, math.pi)
            yield MoveLineTo(LEFT_START_X, 2.0)
            yield RotateTo(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(LEFT_START_X, 0.0)
            yield DefinePosition(None, LEFT_START_Y, math.pi / 2.0)
            yield SpeedControl()
        else:
            yield DefinePosition(LEFT_START_X, LEFT_START_Y, math.pi / 2.0)




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield Stop()
