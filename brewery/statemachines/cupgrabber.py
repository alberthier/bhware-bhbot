# encoding: utf-8

import math

import packets
import position
import logger

from statemachine import *
from definitions import *
from commonstates import *
from position import *
from tools import *




class Main(State):

    def on_enter(self):
        self.fsm.enabled = True

    def on_start(self, packet):
        self.robot.holding_cup = False
        if packet.value == 0:
            self.robot.grabbing_in_progress = False
            self.yield_at(90000, EndOfMatch())
            yield Trigger(CUP_GRIPPER_SEEKING)
            yield Timer(500)
            yield GrabCup()




class GrabCup(State):

    def on_cup_presence(self, packet):
        if packet.value == 0 and not self.robot.holding_cup and self.fsm.enabled:
            yield Timer(50)
            self.robot.grabbing_in_progress = True
            self.send_packet(packets.Stop())
            yield Trigger(CUP_GRIPPER_ON_CUP)
            yield Timer(150)
            self.robot.holding_cup = True
            self.robot.grabbing_in_progress = False
            self.send_packet(packets.CupGrabbed())




##################################################
# End of match




class EndOfMatch(statemachine.State):
    pass
