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
import statemachines.testssecondary as testssecondary




class Main(State):

    def on_enter(self):
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield GetInputStatus(SECONDARY_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")




class CalibratePosition(State):

    def on_enter(self):
        if IS_HOST_DEVICE_ARM:
            yield SpeedControl(0.2)
            xinit = 0.778 - ROBOT_CENTER_X
            xmove = xinit - 0.05
            yield DefinePosition(xinit, 1.0, -math.pi)
            yield MoveLineTo(xmove, 1.0)
            yield RotateTo(math.pi / 2.0)
            yield MoveLineTo(xmove, 0.0)
            yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
            yield SpeedControl()
            yield MoveLineTo(xmove, LEFT_START_Y)
            yield RotateTo(0.0)
            yield MoveLineTo(LEFT_START_X, LEFT_START_Y)
        else:
            yield DefinePosition(LEFT_START_X, LEFT_START_Y, 0.0)




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield Stop()
