# encoding: utf-8

import collections
import math
from contextlib import contextmanager

import packets
import position
import logger
import goalmanager

from statemachine import *
from definitions import *
from commonstates import *
from position import *

from tools import *

def increment_stand_count(state):
    state.fsm.stand_count+=1
    if state.fsm.side == SIDE_LEFT:
        state.robot.left_stand_count+=1
    else:
        state.robot.right_stand_count+=1

    logger.log("Robot new builder count: {}, {}".format(state.robot.left_stand_count, state.robot.right_stand_count))

def reset_stand_count(state):
    state.fsm.stand_count=0
    if state.fsm.side == SIDE_LEFT:
        state.robot.left_stand_count=0
    else:
        state.robot.right_stand_count=0

class Main(State):

    def on_enter(self):
        self.setup()

    def setup(self):
        if self.robot.team != TEAM_UNKNOWN:
            virtual_left = self.fsm.side == SIDE_LEFT and self.robot.team == TEAM_LEFT
            virtual_left |= self.fsm.side == SIDE_RIGHT and self.robot.team == TEAM_RIGHT
        else:
            virtual_left = self.fsm.side == SIDE_LEFT

        if virtual_left:
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
            reset_stand_count(self)
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

@contextmanager
def building_guard(self):
    self.fsm.building = True
    yield
    self.fsm.building = False

class NoStandFoundException(Exception):
    pass

class Build(State):

    def on_enter(self):
        self.building = False
        self.is_holding = False

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


    def hold_and_check(self):
        yield Trigger(self.fsm.PLIERS_LEFT_HOLD, self.fsm.PLIERS_RIGHT_HOLD)

        previous_value_left = None
        previous_value_right = None

        for i in range(4):
            value_left = yield ReadServoPosition(self.fsm.PLIERS_LEFT_ID)
            value_right = yield ReadServoPosition(self.fsm.PLIERS_RIGHT_ID)

            if previous_value_left:
                if previous_value_left.value == value_left.value \
                    and previous_value_right == value_right.value:
                    break

            previous_value_left = value_left
            previous_value_right = value_right

            yield Timer(50)

        logger.log("Got stable value for servos after {} attempts".format(i+1))

        #check position

        expected_left  = self.fsm.PLIERS_LEFT_HOLD[-2]
        expected_right = self.fsm.PLIERS_RIGHT_HOLD[-2]

        delta_expected = expected_left - expected_right

        value_left = yield ReadServoPosition(self.fsm.PLIERS_LEFT_ID)
        value_right = yield ReadServoPosition(self.fsm.PLIERS_RIGHT_ID)

        delta_real = value_left.value - value_right.value

        self.is_holding = False

        logger.log("Left: real: {} expected: {}".format(value_left.value, expected_left))
        logger.log("Right: real: {} expected: {}".format(value_right.value, expected_right))

        logger.log("Delta real: {} expected: {}".format(delta_real, delta_expected))

        if delta_real < delta_expected - 10 :
            self.is_holding = True

        if IS_HOST_DEVICE_PC:
            self.is_holding = True

        logger.log("Holding: {}".format("YES" if self.is_holding else "NO"))

        if not self.is_holding :
            raise NoStandFoundException()


    def grab_stand(self):
        if not self.fsm.enabled:
            return
        if self.building:
            return

        with building_guard(self):
            try:
                if self.fsm.stand_count < 4:
                    if self.fsm.stand_count < 3:
                        yield from self.hold_and_check()

                        yield Trigger(self.fsm.GRIPPER_LEFT_GUIDE, self.fsm.GRIPPER_RIGHT_GUIDE)
                        yield Trigger(self.fsm.ELEVATOR_UP)
                        self.send_packet(packets.StandGrabbed(self.fsm.side))
                        yield Trigger(self.fsm.GRIPPER_LEFT_CLOSE, self.fsm.GRIPPER_RIGHT_CLOSE)
                        yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN, self.fsm.ELEVATOR_DOWN)
                        yield ServoTorqueControl([self.fsm.PLIERS_LEFT_ID, self.fsm.PLIERS_RIGHT_ID, self.fsm.ELEVATOR_ID], False)
                    else:
                        yield from self.hold_and_check()
                yield Timer(300)
                increment_stand_count(self)
                self.send_packet(packets.StandStored(self.fsm.side))
            except NoStandFoundException as e:
                logger.error("No stand found, resetting pliers position")
                yield Trigger(self.fsm.PLIERS_LEFT_OPEN, self.fsm.PLIERS_RIGHT_OPEN)
                self.send_packet(packets.StandGrabbed(self.fsm.side))
                self.send_packet(packets.StandStored(self.fsm.side))
                return



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
        reset_stand_count(self)
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
