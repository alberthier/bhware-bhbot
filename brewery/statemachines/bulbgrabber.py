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
             yield Trigger(LIGHTER_ELEVATOR_BULB)
             yield Trigger(LIGHTER_GRIPPER_CLOSE)
             yield Timer(500)
             self.send_packet(packets.BulbGrabbed())
             yield Trigger(LIGHTER_ELEVATOR_UP)
             yield Trigger(LIGHTER_GRIPPER_OPEN)
             yield Timer(500)
             yield Trigger(LIGHTER_ELEVATOR_DOWN)
             yield None