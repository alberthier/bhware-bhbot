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

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Trigger(LIGHTER_GRIPPER_OPEN, LIGHTER_ELEVATOR_DOWN)
            yield ServoTorqueControl([LIGHTER_GRIPPER_ID, LIGHTER_ELEVATOR_ID], False)


    def on_start(self, packet):
        if packet.value == 0:
             self.yield_at(90000, EndOfMatch())
             yield Trigger(LIGHTER_ELEVATOR_BULB)
             yield Timer(200)
             yield Trigger(LIGHTER_GRIPPER_CLOSE)
             yield Timer(250)
             self.send_packet(packets.BulbGrabbed())


    def on_lift_bulb(self, packet):
             yield Trigger(LIGHTER_ELEVATOR_UP)
             yield Trigger(LIGHTER_GRIPPER_OPEN)
             yield Timer(500)
             bulb_presence = yield GetInputStatus(MAIN_INPUT_LEFT_BULB_PRESENCE)
             self.robot.has_left_bulb = bulb_presence == 0
             bulb_presence = yield GetInputStatus(MAIN_INPUT_RIGHT_BULB_PRESENCE)
             self.robot.has_right_bulb = bulb_presence == 0
             yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
         yield ServoTorqueControl([LIGHTER_ELEVATOR_ID, LIGHTER_GRIPPER_ID], False)
