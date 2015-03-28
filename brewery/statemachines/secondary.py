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
from sysinfo import *
import statemachines.testscommon as testscommon
import statemachines.testssecondary as testssecondary




CUP_OFFSET = ROBOT_CENTER_X + (0.095 / 2.0)




class GrabCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_FORWARD, handler_state, ctor_parameters)


    def is_available(self):
        return not self.goal_manager.event_loop.robot.holding_cup and super().is_available()




class DepositCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_FORWARD, handler_state, ctor_parameters)


    def is_available(self):
        return self.goal_manager.event_loop.robot.holding_cup and super().is_available()




class Main(State):

    def on_enter(self):
        # TODO : maybe created at the wrong place
        SysInfo(self.event_loop)
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        StateMachine(self.event_loop, "cupgrabber")

        self.robot.holding_cup = False

        self.robot.goal_manager.add(
            GrabCupGoal("GRAB_SOUTH_MINE_CUP", 1, 1.75, 0.25, -CUP_OFFSET, GrabCup),
            DepositCupGoal("DEPOSIT_CUP_SOUTH", 2, 1.1, 0.5, 0, DepositCup, (0.28,)),
            GrabCupGoal("GRAB_STAIRS_CUP", 3, 0.80, 0.91, -CUP_OFFSET, GrabCup),
            DepositCupGoal("DEPOSIT_CUP_NORTH", 4, 0.9, 0.5, 0, DepositCup, (0.28,)),
            GrabCupGoal("GRAB_PLATFORM_CUP", 5, 1.65, 1.50, -CUP_OFFSET, GrabCup),
            DepositCupGoal("DEPOSIT_CUP_CENTER", 6, 1.0, 0.5, 0, DepositCup, (0.36,)),
            goalmanager.Goal("DEPOSIT_CARPETS", 7, 0.7, 1.1, 0, DIRECTION_BACKWARDS, DepositCarpets),
        )

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield GetInputStatus(SECONDARY_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(90000, EndOfMatch())
            logger.log("Starting ...")
            self.send_packet(packets.ServoControl(*CUP_GRIPPER_OPEN))
            yield MoveLineTo(1.32, LEFT_START_Y)
            yield ExecuteGoals()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CARPET_DROPPER_CLOSE, RIGHT_CARPET_DROPPER_CLOSE,
                      LEFT_CARPET_EJECTOR_HOLD, RIGHT_CARPET_EJECTOR_HOLD,
                      CUP_GRIPPER_CLOSE)
        yield None




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
        yield None




class GrabCup(State):

    def on_enter(self):
        if self.robot.holding_cup:
            self.exit_reason = GOAL_DONE
            yield None


    def on_cup_grabbed(self, packet):
        self.exit_reason = GOAL_DONE
        yield None




class DepositCup(State):

    def __init__(self, y):
        self.y = y


    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(-math.pi / 2.0)
        yield MoveLineTo(goal.x, self.y)
        yield Trigger(CUP_GRIPPER_HALF_OPEN)
        yield Timer(300)
        yield MoveLineRelative(-0.02)
        yield Trigger(CUP_GRIPPER_OPEN)
        yield MoveLineTo(goal.x, goal.y)
        self.robot.holding_cup = False
        self.exit_reason = GOAL_DONE
        yield None




class DepositCarpets(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(0.0)
        yield MoveLineTo(0.58 + ROBOT_CENTER_X, goal.y)
        yield Trigger(LEFT_CARPET_DROPPER_OPEN, RIGHT_CARPET_DROPPER_OPEN)
        yield Trigger(LEFT_CARPET_EJECTOR_THROW, RIGHT_CARPET_EJECTOR_THROW)
        #FIXME : don't throw carpets at the same place
        yield Trigger(LEFT_CARPET_DROPPER_CLOSE, RIGHT_CARPET_DROPPER_CLOSE)
        yield Trigger(LEFT_CARPET_EJECTOR_HOLD, RIGHT_CARPET_EJECTOR_HOLD)
        self.exit_reason = GOAL_DONE
        yield None


##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
