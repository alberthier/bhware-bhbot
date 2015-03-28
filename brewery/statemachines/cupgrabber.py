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

    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(90000, EndOfMatch())
            yield Trigger(CUP_GRIPPER_OPEN)
            yield GrabStand()




class GrabStand(State):

    def on_cup_presence(self, packet):
        if packet.value == 1 and not self.robot.holding_cup:
            self.send_packet(packets.Stop())
            yield Trigger(CUP_GRIPPER_ON_CUP)
            self.robot.holding_cup = True
            self.send_packet(packets.CupGrabbed())




##################################################
# End of match




class EndOfMatch(statemachine.State):
    pass
