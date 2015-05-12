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


CUP_GRAB_RATION_DECC=1.0
GRAB_OFFSET = -(ROBOT_CENTER_X + 0.05)




class GrabCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_BACKWARDS, handler_state, ctor_parameters)
        self.ratio_decc = CUP_GRAB_RATION_DECC


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
            GCG("GRAB_STAIRS_CUP")
                .weight(1)
                .coords(0.80, 0.91)
                .offset(GRAB_OFFSET)
                .state(GrabCup)
                .build(),
            G("DEPOSIT_CARPET_LEFT")
                .weight(2)
                .coords(0.725, 1.08)
                .direction(DIRECTION_FORWARD)
                .state(DepositCarpet, (SIDE_LEFT,))
                .build(),
            G("DEPOSIT_CARPET_RIGHT")
                .weight(3)
                .coords(0.725, 1.40)
                .direction(DIRECTION_FORWARD)
                .state(DepositCarpet, (SIDE_RIGHT,))
                .build(),
            DCG("DEPOSIT_CUP_HOME")
                .weight(4)
                .coords(1.0, 0.5)
                .state(DepositCup, (True,))
                .build(),
            G("GRAB_SOUTH_MINE_CUP")
                .weight(5)
                .coords(1.40, 0.50)
                .direction(DIRECTION_BACKWARDS)
                .state(GrabSouthCornerCup)
                .build(),
            G("GRAB_SOUTH_THEIRS_CUP")
                .weight(10)
                .coords(1.40, 2.50)
                .direction(DIRECTION_BACKWARDS)
                .state(GrabSouthCornerCup)
                .build(),
            DCG("DEPOSIT_OPP_NORTH")
                .weight(8)
                .coords(0.67, 2.70)
                .state(DepositCup, (False,))
                .build(),
            GCG("GRAB_PLATFORM_CUP")
                .weight(7)
                .coords(1.65, 1.28)
                .offset(GRAB_OFFSET)
                .state(GrabPlatformCup)
                .build(),
            GCG("GRAB_PLATFORM_CUP")
                .weight(7)
                .coords(1.65, 1.72)
                .offset(GRAB_OFFSET)
                .state(GrabPlatformCup)
                .build(),
            DCG("DEPOSIT_OPP_SOUTH")
                .weight(6)
                .coords(1.40, 2.80)
                .state(DepositCup, (False,))
                .build(),
            G("KICK_THEIRS_CLAP")
                .weight(6)
                .coords(1.86, 2.65)
                .direction(DIRECTION_FORWARD)
                .state(KickTheirsClap)
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
            yield StaticStrategy()
            yield ExecuteGoals()
            # yield ExecuteGoalsV2()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CARPET_DROPPER_CLOSE, RIGHT_CARPET_DROPPER_CLOSE,
                      LEFT_CARPET_EJECTOR_HOLD, RIGHT_CARPET_EJECTOR_HOLD,
                      CUP_GRIPPER_CLOSE, CLAPMAN_CLOSE)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        start_x = 1.0
        start_y = 0.54

        start_angle = math.atan2(0.91 - start_y, 0.80 - start_x) + math.pi
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




class StaticStrategy(State):

    def on_enter(self):
        try:
            self.send_packet(packets.InterbotLock("NORTH_ZONE"))
            yield from self.execute_goal("GRAB_STAIRS_CUP")
            yield from self.execute_goal("DEPOSIT_CARPET_RIGHT", DIRECTION_BACKWARDS)
            yield from self.execute_goal("DEPOSIT_CARPET_LEFT")
            yield LookAtOpposite(1.0, 0.55)
            yield SafeMoveLineTo(1.0, 0.55)
            self.send_packet(packets.InterbotUnlock("NORTH_ZONE"))
            yield from self.execute_goal("DEPOSIT_CUP_HOME")
            yield WaitForUnlock("SOUTH_ZONE", 5000)
            yield from self.execute_goal("GRAB_SOUTH_MINE_CUP")
        except OpponentInTheWay:
            pass
        yield None


    def execute_goal(self, name, forced_direction = None):
        goal = self.robot.goal_manager.get_goals(name)[0]
        x, y = get_offset_position(self.robot.pose, goal.x, goal.y, goal.offset)
        direction = forced_direction if forced_direction is not None else goal.direction
        if direction == DIRECTION_FORWARD:
            yield LookAt(x, y)
        else:
            yield LookAtOpposite(x, y)
        yield SafeMoveLineTo(x, y)
        goal.doing()
        state = yield goal.get_state()
        if state.exit_reason == GOAL_DONE :
            goal.done()
        else:
            goal.available()
        self.robot.goal_manager.update_goal_status(name, GOAL_DONE)




class GrabCup(Timer):

    def __init__(self):
        super().__init__(300)
        self.exit_reason = GOAL_DONE


    def on_cup_grabbed(self, packet):
        yield None




class GrabSouthCornerCup(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        mine = goal.y < 1.5
        cup_y = 0.250 if mine else 3.0 - 0.250

        # Recalibrate
        a = math.pi / 2.0 if mine else -math.pi / 2.0
        yield RotateTo(a)
        yield SpeedControl(0.2)
        y = 0.0 if mine else 3.0
        yield MoveLineTo(goal.x, y)
        ry = ROBOT_CENTER_X if mine else 3.0 - ROBOT_CENTER_X
        yield DefinePosition(None, ry, a)
        yield SpeedControl()
        yield MoveLineTo(goal.x, cup_y)
        yield RotateTo(math.pi)
        yield SpeedControl(0.2)
        yield MoveLineTo(1.0, cup_y)
        yield DefinePosition(1.222 + ROBOT_CENTER_X, None, math.pi)
        yield SpeedControl()

        # Grab cup
        yield SafeMoveLineTo(1.750 + GRAB_OFFSET, cup_y)
        grab = yield GrabCup()
        self.exit_reason = grab.exit_reason
        yield None




class GrabPlatformCup(State):

    def on_enter(self):
        xc = 1.650
        if self.robot.pose.virt.y < 1.500:
            yc = 1.500 - GRAB_OFFSET
        else:
            yc = 1.500 + GRAB_OFFSET
        yield LookAtOpposite(xc, yc)
        yield SafeMoveLineTo(xc, yc)
        grab = yield GrabCup()
        self.exit_reason = grab.exit_reason



class DepositCup(State):

    def __init__(self, home):
        self.home = home


    def on_enter(self):
        if self.home:
            yield RotateTo(math.pi / 2.0)
            yield MoveLineTo(1.0, 0.30)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_HALF_OPEN)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_OPEN)
        if self.home:
            yield MoveLineTo(1.0, 0.65)
        else:
            yield MoveLineRelative(0.05)
        self.robot.holding_cup = False
        self.exit_reason = GOAL_DONE
        yield None




class DepositCarpet(State):

    def __init__(self, side):
        self.side = side


    def on_enter(self):
        if self.robot.team == TEAM_LEFT and self.side == SIDE_LEFT or self.robot.team == TEAM_RIGHT and self.side == SIDE_RIGHT:
            real_side = SIDE_LEFT
        else:
            real_side = SIDE_RIGHT
        if real_side == SIDE_LEFT:
            self.dropper_open  = LEFT_CARPET_DROPPER_OPEN
            self.dropper_close = LEFT_CARPET_DROPPER_CLOSE
            self.ejector_throw = LEFT_CARPET_EJECTOR_THROW
            self.ejector_hold  = LEFT_CARPET_EJECTOR_HOLD
        else:
            self.dropper_open  = RIGHT_CARPET_DROPPER_OPEN
            self.dropper_close = RIGHT_CARPET_DROPPER_CLOSE
            self.ejector_throw = RIGHT_CARPET_EJECTOR_THROW
            self.ejector_hold  = RIGHT_CARPET_EJECTOR_HOLD

        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(math.pi)
        yield MoveLineTo(0.600, goal.y)
        yield DefinePosition(0.580 + ROBOT_CENTER_X, None, math.pi)
        yield Trigger(self.dropper_open)
        yield Trigger(self.ejector_throw)
        yield Trigger(self.dropper_close)
        yield Trigger(self.ejector_hold)
        yield MoveLineTo(goal.x, goal.y)
        self.exit_reason = GOAL_DONE
        yield None




class KickTheirsClap(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield Trigger(CLAPMAN_OPEN)
        if self.robot.team == TEAM_LEFT:
            yield RotateTo(math.pi / 2)
        else:
            yield RotateTo(-math.pi / 2)
        yield SafeMoveLineTo(goal.x, 2.330)
        yield RotateTo(math.pi)
        yield Trigger(CLAPMAN_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl(SERVOS_IDS.values(), False)
