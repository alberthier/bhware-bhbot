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
        	self.send_packet(packets.InputStatusRequest(INPUT_TEAM))


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield Stop()
