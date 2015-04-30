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
        self.setup()

    def setup(self):
        if self.fsm.side == SIDE_LEFT and self.robot.team == TEAM_LEFT:
            self.fsm.PLIERS_LEFT_INIT      = LEFT_BUILDER_PLIERS_LEFT_INIT
            self.fsm.PLIERS_LEFT_CLOSE     = LEFT_BUILDER_PLIERS_LEFT_CLOSE
            self.fsm.PLIERS_LEFT_HOLD      = LEFT_BUILDER_PLIERS_LEFT_HOLD
            self.fsm.PLIERS_LEFT_OPEN      = LEFT_BUILDER_PLIERS_LEFT_OPEN
            self.fsm.PLIERS_RIGHT_INIT     = LEFT_BUILDER_PLIERS_RIGHT_INIT
            self.fsm.PLIERS_RIGHT_CLOSE    = LEFT_BUILDER_PLIERS_RIGHT_CLOSE
            self.fsm.PLIERS_RIGHT_HOLD     = LEFT_BUILDER_PLIERS_RIGHT_HOLD
            self.fsm.PLIERS_RIGHT_OPEN     = LEFT_BUILDER_PLIERS_RIGHT_OPEN
            self.fsm.GRIPPER_LEFT_INIT     = LEFT_BUILDER_GRIPPER_LEFT_INIT
            self.fsm.GRIPPER_LEFT_CLOSE    = LEFT_BUILDER_GRIPPER_LEFT_CLOSE
            self.fsm.GRIPPER_LEFT_GUIDE    = LEFT_BUILDER_GRIPPER_LEFT_GUIDE
            self.fsm.GRIPPER_LEFT_LIGHT    = LEFT_BUILDER_GRIPPER_LEFT_LIGHT
            self.fsm.GRIPPER_LEFT_DEPOSIT  = LEFT_BUILDER_GRIPPER_LEFT_DEPOSIT
            self.fsm.GRIPPER_RIGHT_INIT    = LEFT_BUILDER_GRIPPER_RIGHT_INIT
            self.fsm.GRIPPER_RIGHT_CLOSE   = LEFT_BUILDER_GRIPPER_RIGHT_CLOSE
            self.fsm.GRIPPER_RIGHT_GUIDE   = LEFT_BUILDER_GRIPPER_RIGHT_GUIDE
            self.fsm.GRIPPER_RIGHT_LIGHT   = LEFT_BUILDER_GRIPPER_RIGHT_LIGHT
            self.fsm.GRIPPER_RIGHT_DEPOSIT = LEFT_BUILDER_GRIPPER_RIGHT_DEPOSIT
            self.fsm.LIGHTER_WAIT          = LEFT_BUILDER_LIGHTER_WAIT
            self.fsm.LIGHTER_DEPOSIT       = LEFT_BUILDER_LIGHTER_DEPOSIT
            self.fsm.ELEVATOR_DOWN         = LEFT_BUILDER_ELEVATOR_DOWN
            self.fsm.ELEVATOR_PLATFORM     = LEFT_BUILDER_ELEVATOR_PLATFORM
            self.fsm.ELEVATOR_UP           = LEFT_BUILDER_ELEVATOR_UP

            self.fsm.PLIERS_LEFT_ID        = LEFT_BUILDER_PLIERS_LEFT_ID
            self.fsm.PLIERS_RIGHT_ID       = LEFT_BUILDER_PLIERS_RIGHT_ID
            self.fsm.GRIPPER_LEFT_ID       = LEFT_BUILDER_GRIPPER_LEFT_ID
            self.fsm.GRIPPER_RIGHT_ID      = LEFT_BUILDER_GRIPPER_RIGHT_ID
            self.fsm.LIGHTER_ID            = LEFT_BUILDER_LIGHTER_ID
            self.fsm.ELEVATOR_ID           = LEFT_BUILDER_ELEVATOR_ID

            self.fsm.INPUT_BULB_PRESENCE   = MAIN_INPUT_LEFT_BULB_PRESENCE
            self.fsm.INPUT_STAND_PRESENCE  = MAIN_INPUT_LEFT_STAND_PRESENCE
        else:
            self.fsm.PLIERS_LEFT_INIT      = RIGHT_BUILDER_PLIERS_LEFT_INIT
            self.fsm.PLIERS_LEFT_CLOSE     = RIGHT_BUILDER_PLIERS_LEFT_CLOSE
            self.fsm.PLIERS_LEFT_HOLD      = RIGHT_BUILDER_PLIERS_LEFT_HOLD
            self.fsm.PLIERS_LEFT_OPEN      = RIGHT_BUILDER_PLIERS_LEFT_OPEN
            self.fsm.PLIERS_RIGHT_INIT     = RIGHT_BUILDER_PLIERS_RIGHT_INIT
            self.fsm.PLIERS_RIGHT_CLOSE    = RIGHT_BUILDER_PLIERS_RIGHT_CLOSE
            self.fsm.PLIERS_RIGHT_HOLD     = RIGHT_BUILDER_PLIERS_RIGHT_HOLD
            self.fsm.PLIERS_RIGHT_OPEN     = RIGHT_BUILDER_PLIERS_RIGHT_OPEN
            self.fsm.GRIPPER_LEFT_INIT     = RIGHT_BUILDER_GRIPPER_LEFT_INIT
            self.fsm.GRIPPER_LEFT_CLOSE    = RIGHT_BUILDER_GRIPPER_LEFT_CLOSE
            self.fsm.GRIPPER_LEFT_GUIDE    = RIGHT_BUILDER_GRIPPER_LEFT_GUIDE
            self.fsm.GRIPPER_LEFT_LIGHT    = RIGHT_BUILDER_GRIPPER_LEFT_LIGHT
            self.fsm.GRIPPER_LEFT_DEPOSIT  = RIGHT_BUILDER_GRIPPER_LEFT_DEPOSIT
            self.fsm.GRIPPER_RIGHT_INIT    = RIGHT_BUILDER_GRIPPER_RIGHT_INIT
            self.fsm.GRIPPER_RIGHT_CLOSE   = RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE
            self.fsm.GRIPPER_RIGHT_GUIDE   = RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE
            self.fsm.GRIPPER_RIGHT_LIGHT   = RIGHT_BUILDER_GRIPPER_RIGHT_LIGHT
            self.fsm.GRIPPER_RIGHT_DEPOSIT = RIGHT_BUILDER_GRIPPER_RIGHT_DEPOSIT
            self.fsm.LIGHTER_WAIT          = RIGHT_BUILDER_LIGHTER_WAIT
            self.fsm.LIGHTER_DEPOSIT       = RIGHT_BUILDER_LIGHTER_DEPOSIT
            self.fsm.ELEVATOR_DOWN         = RIGHT_BUILDER_ELEVATOR_DOWN
            self.fsm.ELEVATOR_PLATFORM     = RIGHT_BUILDER_ELEVATOR_PLATFORM
            self.fsm.ELEVATOR_UP           = RIGHT_BUILDER_ELEVATOR_UP

            self.fsm.PLIERS_LEFT_ID        = RIGHT_BUILDER_PLIERS_LEFT_ID
            self.fsm.PLIERS_RIGHT_ID       = RIGHT_BUILDER_PLIERS_RIGHT_ID
            self.fsm.GRIPPER_LEFT_ID       = RIGHT_BUILDER_GRIPPER_LEFT_ID
            self.fsm.GRIPPER_RIGHT_ID      = RIGHT_BUILDER_GRIPPER_RIGHT_ID
            self.fsm.LIGHTER_ID            = RIGHT_BUILDER_LIGHTER_ID
            self.fsm.ELEVATOR_ID           = RIGHT_BUILDER_ELEVATOR_ID

            self.fsm.INPUT_BULB_PRESENCE   = MAIN_INPUT_RIGHT_BULB_PRESENCE
            self.fsm.INPUT_STAND_PRESENCE  = MAIN_INPUT_RIGHT_STAND_PRESENCE


    def on_controller_status(self, packet):
        #TODO: handle not empty
        if packet.status == CONTROLLER_STATUS_READY:
            yield InitialPosition()


    def on_start(self, packet):
        self.setup() # Ensure we take the color correctly into account (we may change it after the robot is booted)
        if packet.value == 0:
            self.fsm.stand_count = 0
            self.fsm.building = False
            self.fsm.enabled = True

            self.yield_at(89500, EndOfMatch())
            yield IdlePosition()
            yield Build()




class InitialPosition(State):

    def on_enter(self):
        yield Trigger(self.fsm.PLIERS_LEFT_INIT, self.fsm.PLIERS_RIGHT_INIT,
                      self.fsm.GRIPPER_LEFT_INIT, self.fsm.GRIPPER_RIGHT_INIT,
                      self.fsm.ELEVATOR_DOWN, self.fsm.LIGHTER_WAIT)
        yield ServoTorqueControl([self.fsm.PLIERS_LEFT_ID, self.fsm.PLIERS_RIGHT_ID,
                                  self.fsm.GRIPPER_LEFT_ID, self.fsm.GRIPPER_RIGHT_ID,
                                  self.fsm.ELEVATOR_ID, self.fsm.LIGHTER_ID], False)
        yield None




class IdlePosition(State):

    def on_enter(self):
        yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN,
                      self.fsm.GRIPPER_LEFT_GUIDE, self.fsm.GRIPPER_RIGHT_GUIDE,
                      self.fsm.ELEVATOR_DOWN)
        yield ServoTorqueControl([self.fsm.PLIERS_LEFT_ID, self.fsm.PLIERS_RIGHT_ID,
                                  self.fsm.GRIPPER_LEFT_ID, self.fsm.GRIPPER_RIGHT_ID,
                                  self.fsm.ELEVATOR_ID], False)
        yield None




class Build(State):

    def on_enter(self):
        self.building = False

    def on_input_status(self, packet):
        # Dispatch manually because there are 2 bulb detectors and 2 stand presence detectors
        # if packet.id == self.fsm.INPUT_BULB_PRESENCE:
        #     yield from self.on_bulb_presence(packet)
        if packet.id == self.fsm.INPUT_STAND_PRESENCE and packet.kind == KIND_EVENT and packet.value == 0:
            yield from self.on_stand_presence(packet)


    def on_ensure_build(self, packet):
        if packet.side == self.fsm.side:
            yield from self.grab_stand()

    def on_stand_presence(self, packet):
        yield from self.grab_stand()


    def grab_stand(self):
        if not self.fsm.enabled:
            return
        if self.building:
            return
        if self.fsm.stand_count < 4:
            self.fsm.building = True
            if self.fsm.stand_count < 3:
                yield Trigger(self.fsm.PLIERS_LEFT_HOLD, self.fsm.PLIERS_RIGHT_HOLD)
                yield Trigger(self.fsm.GRIPPER_LEFT_GUIDE, self.fsm.GRIPPER_RIGHT_GUIDE)
                yield Trigger(self.fsm.ELEVATOR_UP)
                self.send_packet(packets.StandGrabbed(self.fsm.side))
                yield Trigger(self.fsm.GRIPPER_LEFT_CLOSE, self.fsm.GRIPPER_RIGHT_CLOSE)
                yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN, self.fsm.ELEVATOR_DOWN)
                yield ServoTorqueControl([self.fsm.PLIERS_LEFT_ID, self.fsm.PLIERS_RIGHT_ID, self.fsm.ELEVATOR_ID], False)
            else:
                yield Trigger(self.fsm.PLIERS_LEFT_CLOSE, self.fsm.PLIERS_RIGHT_CLOSE)
                self.send_packet(packets.StandGrabbed(self.fsm.side))
            self.fsm.stand_count += 1
        self.fsm.building = False
        yield Timer(300)
        self.send_packet(packets.StandStored(self.fsm.side))


    def on_build_spotlight(self, packet):
        if packet.side == self.fsm.side:
            yield BuildSpotlight()


    def on_standbuilder_idle(self, packet):
        if packet.side == self.fsm.side:
            yield IdlePosition()




class BuildSpotlight(State):

    def on_enter(self):
        bulb_presence = yield GetInputStatus(self.fsm.INPUT_BULB_PRESENCE)
        if bulb_presence.value == 0:
            yield Trigger(self.fsm.LIGHTER_DEPOSIT)
            yield ServoTorqueControl([self.fsm.LIGHTER_ID], False)
            yield Timer(200)
        yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN)
        yield MoveLineRelative(0.05)
        yield Trigger(self.fsm.GRIPPER_LEFT_GUIDE, self.fsm.GRIPPER_RIGHT_GUIDE)
        yield Timer(200)
        yield Trigger(self.fsm.GRIPPER_LEFT_LIGHT, self.fsm.GRIPPER_RIGHT_LIGHT)
        yield Timer(500)
        yield Trigger(self.fsm.GRIPPER_LEFT_DEPOSIT, self.fsm.GRIPPER_RIGHT_DEPOSIT)
        self.fsm.stand_count = 0
        self.send_packet(packets.BuildSpotlight(self.fsm.side))
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        yield Trigger(self.fsm.GRIPPER_LEFT_DEPOSIT, self.fsm.GRIPPER_RIGHT_DEPOSIT)
        yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN)
        yield ServoTorqueControl([self.fsm.PLIERS_LEFT_ID, self.fsm.PLIERS_RIGHT_ID,
                                  self.fsm.GRIPPER_LEFT_ID, self.fsm.GRIPPER_RIGHT_ID,
                                  self.fsm.ELEVATOR_ID, self.fsm.LIGHTER_ID], False)
