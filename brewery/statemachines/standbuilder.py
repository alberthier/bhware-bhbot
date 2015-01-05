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




class Main(State):

    def on_enter(self):
        self.fsm.stand_count = 0

        if self.fsm.side == SIDE_LEFT:
            self.fsm.PLIERS_LEFT_CLOSE   = LEFT_BUILDER_PLIERS_LEFT_CLOSE
            self.fsm.PLIERS_LEFT_OPEN    = LEFT_BUILDER_PLIERS_LEFT_OPEN
            self.fsm.PLIERS_RIGHT_CLOSE  = LEFT_BUILDER_PLIERS_RIGHT_CLOSE
            self.fsm.PLIERS_RIGHT_OPEN   = LEFT_BUILDER_PLIERS_RIGHT_OPEN
            self.fsm.GRIPPER_LEFT_CLOSE  = LEFT_BUILDER_GRIPPER_LEFT_CLOSE
            self.fsm.GRIPPER_LEFT_OPEN   = LEFT_BUILDER_GRIPPER_LEFT_OPEN
            self.fsm.GRIPPER_RIGHT_CLOSE = LEFT_BUILDER_GRIPPER_RIGHT_CLOSE
            self.fsm.GRIPPER_RIGHT_OPEN  = LEFT_BUILDER_GRIPPER_RIGHT_OPEN
            self.fsm.LIGHTER_CLOSE       = LEFT_BUILDER_LIGHTER_CLOSE
            self.fsm.LIGHTER_OPEN        = LEFT_BUILDER_LIGHTER_OPEN
            self.fsm.ELEVATOR_DOWN       = LEFT_BUILDER_ELEVATOR_DOWN
            self.fsm.ELEVATOR_PLATFORM   = LEFT_BUILDER_ELEVATOR_PLATFORM
            self.fsm.ELEVATOR_UP         = LEFT_BUILDER_ELEVATOR_UP

            self.fsm.INPUT_BULB_PRESENCE   = MAIN_INPUT_LEFT_BULB_PRESENCE
            self.fsm.INPUT_STAND_PRESENCE  = MAIN_INPUT_LEFT_STAND_PRESENCE
        else:
            self.fsm.PLIERS_LEFT_CLOSE   = RIGHT_BUILDER_PLIERS_LEFT_CLOSE
            self.fsm.PLIERS_LEFT_OPEN    = RIGHT_BUILDER_PLIERS_LEFT_OPEN
            self.fsm.PLIERS_RIGHT_CLOSE  = RIGHT_BUILDER_PLIERS_RIGHT_CLOSE
            self.fsm.PLIERS_RIGHT_OPEN   = RIGHT_BUILDER_PLIERS_RIGHT_OPEN
            self.fsm.GRIPPER_LEFT_CLOSE  = RIGHT_BUILDER_GRIPPER_LEFT_CLOSE
            self.fsm.GRIPPER_LEFT_OPEN   = RIGHT_BUILDER_GRIPPER_LEFT_OPEN
            self.fsm.GRIPPER_RIGHT_CLOSE = RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE
            self.fsm.GRIPPER_RIGHT_OPEN  = RIGHT_BUILDER_GRIPPER_RIGHT_OPEN
            self.fsm.LIGHTER_CLOSE       = RIGHT_BUILDER_LIGHTER_CLOSE
            self.fsm.LIGHTER_OPEN        = RIGHT_BUILDER_LIGHTER_OPEN
            self.fsm.ELEVATOR_DOWN       = RIGHT_BUILDER_ELEVATOR_DOWN
            self.fsm.ELEVATOR_PLATFORM   = RIGHT_BUILDER_ELEVATOR_PLATFORM
            self.fsm.ELEVATOR_UP         = RIGHT_BUILDER_ELEVATOR_UP

            self.fsm.INPUT_BULB_PRESENCE  = MAIN_INPUT_RIGHT_BULB_PRESENCE
            self.fsm.INPUT_STAND_PRESENCE = MAIN_INPUT_RIGHT_STAND_PRESENCE


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN)
        yield Build()




class Build(State):

    def on_input_status(self, packet):
        # Dispatch manually
        if packet.id == self.fsm.INPUT_BULB_PRESENCE:
            yield from self.on_bulb_presence(packet)
        elif packet.id == self.fsm.INPUT_STAND_PRESENCE:
            yield from self.on_stand_presence(packet)


    def on_bulb_presence(self, packet):
        pass


    def on_stand_presence(self, packet):
        if self.fsm.stand_count < 4:
            yield Trigger(self.fsm.PLIERS_LEFT_CLOSE, self.fsm.PLIERS_RIGHT_CLOSE)
            if self.fsm.stand_count < 3:
                yield Trigger(self.fsm.GRIPPER_LEFT_OPEN, self.fsm.GRIPPER_RIGHT_OPEN)
                yield Trigger(self.fsm.ELEVATOR_UP)
                yield Trigger(self.fsm.GRIPPER_LEFT_CLOSE, self.fsm.GRIPPER_RIGHT_CLOSE)
                yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN, ELEVATOR_DOWN)
            self.fsm.stand_count += 1




##################################################
# End of match




class EndOfMatch(statemachine.State):
    pass
