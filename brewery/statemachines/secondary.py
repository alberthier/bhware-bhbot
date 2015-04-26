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




class GrabCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_BACKWARDS, handler_state, ctor_parameters)


    def is_available(self):
        return not self.goal_manager.event_loop.robot.holding_cup and super().is_available()




class DepositCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_BACKWARDS, handler_state, ctor_parameters)


    def is_available(self):
        return self.goal_manager.event_loop.robot.holding_cup and super().is_available()




class Main(State):

    def on_enter(self):
        # TODO : maybe created at the wrong place
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        StateMachine(self.event_loop, "cupgrabber")

        self.robot.holding_cup = False

        G=goalmanager.GoalBuilder
        GCG=functools.partial(goalmanager.GoalBuilder, ctor=GrabCupGoal)
        DCG=functools.partial(goalmanager.GoalBuilder, ctor=DepositCupGoal)

        self.robot.goal_manager.add(
            #GrabCupGoal("GRAB_STAIRS_CUP", 1, 0.80, 0.91, -ROBOT_CENTER_X, GrabCup),
            GCG("GRAB_STAIRS_CUP")
            .weight(1)
            .coords(0.80, 0.91)
            .offset(-ROBOT_CENTER_X)
            .state(GrabCup)
            .build(),
            #goalmanager.Goal("DEPOSIT_CARPET_LEFT" , 2, 0.725, 1.08, 0, DIRECTION_FORWARD, DepositCarpet, (SIDE_LEFT,)),
            G("DEPOSIT_CARPET_LEFT")
            .weight(2)
            .coords(0.725, 1.08)
            .direction(DIRECTION_FORWARD)
            .state(DepositCarpet, (SIDE_LEFT,))
            .build(),
            #goalmanager.Goal("DEPOSIT_CARPET_RIGHT", 3, 0.725, 1.40, 0, DIRECTION_FORWARD, DepositCarpet, (SIDE_RIGHT,)),
            G("DEPOSIT_CARPET_RIGHT")
            .weight(3)
            .coords(0.725, 1.40)
            .direction(DIRECTION_FORWARD)
            .state(DepositCarpet, (SIDE_RIGHT,))
            .build(),
            #DepositCupGoal("DEPOSIT_CUP_HOME", 4, 1.0, 0.5, 0, DepositCup, (True,)),
            DCG("DEPOSIT_CUP_HOME")
            .weight(4)
            .coords(1.0, 0.5)
            .state(DepositCup, (True,))
            .build(),
            #GrabCupGoal("GRAB_SOUTH_MINE_CUP", 5, 1.75, 0.25, -ROBOT_CENTER_X, GrabCup),
            GCG("GRAB_SOUTH_MINE_CUP")
            .weight(5)
            .coords(1.75, 0.25)
            .offset(-ROBOT_CENTER_X)
            .state(GrabCup)
            .build(),
            #DepositCupGoal("DEPOSIT_OPP_NORTH", 6, 0.67, 2.70, 0, DepositCup, (False,)),
            DCG("DEPOSIT_OPP_NORTH")
            .weight(6)
            .coords(0.67, 2.70)
            .state(DepositCup, (False,))
            .build(),
            #GrabCupGoal("GRAB_PLATFORM_CUP", 7, 1.65, 1.50, -ROBOT_CENTER_X, GrabCup),
            GCG("GRAB_PLATFORM_CUP")
            .weight(7)
            .coords(1.65, 1.50)
            .offset(-ROBOT_CENTER_X)
            .state(GrabCup)
            .build(),
            #DepositCupGoal("DEPOSIT_OPP_SOUTH", 8, 1.33, 2.70, 0, DepositCup, (False,)),
            DCG("DEPOSIT_OPP_SOUTH")
            .weight(8)
            .coords(1.33, 2.70)
            .state(DepositCup, (False,))
            .build(),
        )

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
            self.send_packet(packets.ServoControl(*CUP_GRIPPER_OPEN))
            yield ExecuteGoals()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CARPET_DROPPER_CLOSE, RIGHT_CARPET_DROPPER_CLOSE,
                      LEFT_CARPET_EJECTOR_HOLD, RIGHT_CARPET_EJECTOR_HOLD,
                      CUP_GRIPPER_CLOSE)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        start_x = 1.0
        start_y = 0.54

        apose = position.Pose(0.80, 0.91, None, True)
        start_angle = math.atan2(apose.y - start_y, apose.x - start_x) + math.pi
        start_angle = tools.normalize_angle(start_angle)

        if IS_HOST_DEVICE_ARM:
            wedge_size = 0.055
            setup_x = 0.8 + wedge_size + ROBOT_CENTER_X
            setup_y = 0.07 + 0.30 + ROBOT_CENTER_X
            yield DefinePosition(setup_x, setup_y, math.pi / 2.0)
            yield MoveLineTo(setup_x, start_y)
            yield RotateTo(0.0)
            yield MoveLineTo(start_x, start_y)
            yield RotateTo(start_angle)
        else:
            yield DefinePosition(start_x, start_y, start_angle)
        yield None




class GrabCup(State):

    def on_enter(self):
        if self.robot.holding_cup or not self.robot.grabbing_in_progress:
            self.exit_reason = GOAL_DONE
            yield None


    def on_cup_grabbed(self, packet):
        self.exit_reason = GOAL_DONE
        yield None




class DepositCup(State):

    def __init__(self, home):
        self.home = home


    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        if self.home:
            yield RotateTo(math.pi / 2.0)
            yield MoveLineTo(goal.x, 0.30)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_HALF_OPEN)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_OPEN)
        if self.home:
            yield MoveLineTo(goal.x, goal.y)
        else:
            yield MoveLineRelative(0.05)
        self.robot.holding_cup = False
        self.exit_reason = GOAL_DONE
        yield None




class DepositCarpet(State):

    def __init__(self, side):
        if side == SIDE_LEFT:
            self.dropper_open  = LEFT_CARPET_DROPPER_OPEN
            self.dropper_close = LEFT_CARPET_DROPPER_CLOSE
            self.ejector_throw = LEFT_CARPET_EJECTOR_THROW
            self.ejector_hold  = LEFT_CARPET_EJECTOR_HOLD
        else:
            self.dropper_open  = RIGHT_CARPET_DROPPER_OPEN
            self.dropper_close = RIGHT_CARPET_DROPPER_CLOSE
            self.ejector_throw = RIGHT_CARPET_EJECTOR_THROW
            self.ejector_hold  = RIGHT_CARPET_EJECTOR_HOLD


    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(math.pi)
        yield Trigger(self.dropper_open)
        yield Trigger(self.ejector_throw)
        yield Trigger(self.dropper_close)
        yield Trigger(self.ejector_hold)
        self.exit_reason = GOAL_DONE
        yield None


##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl(SERVOS_IDS.values(), False)
