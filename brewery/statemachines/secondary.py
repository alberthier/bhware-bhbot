# encoding: utf-8

import collections
import math

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
        # TODO : maybe created at the wrong place
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        self.fsm.cup_grabber = StateMachine(self.event_loop, "cupgrabber")


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield ServoTorqueControl(SERVOS_IDS.values(), False)
            yield AntiBlocking(True)
            yield GetInputStatus(SECONDARY_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(90000, EndOfMatch())
            logger.log("Starting ...")




class Initialize(State):

    def on_enter(self):
        yield None




class CalibratePosition(State):

    def on_enter(self):
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl(SERVOS_IDS.values(), False)
