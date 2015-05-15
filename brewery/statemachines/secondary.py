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
HIGH_BORDER_OFFSET = 0.009



class GrabCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_BACKWARDS, handler_state, ctor_parameters)
        self.ratio_decc = CUP_GRAB_RATION_DECC


    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return len(robot.handled_cup_zones) < 3 and not robot.holding_cup and super().is_available()




class DepositCupGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, x, y, offset, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, offset, DIRECTION_BACKWARDS, handler_state, ctor_parameters)


    def is_available(self):
        robot = self.goal_manager.event_loop.robot
        return robot.holding_cup and self.identifier not in robot.handled_cup_zones and super().is_available()




class Main(State):

    def on_enter(self):
        # TODO : maybe created at the wrong place
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        self.fsm.cup_grabber = StateMachine(self.event_loop, "cupgrabber")

        self.robot.holding_cup = False
        self.robot.handled_cup_zones = []

        G=goalmanager.GoalBuilder
        GCG=functools.partial(goalmanager.GoalBuilder, ctor=GrabCupGoal)
        DCG=functools.partial(goalmanager.GoalBuilder, ctor=DepositCupGoal)

        self.robot.goal_manager.add(
            GCG("GRAB_STAIRS_CUP")
                .weight(1)
                .coords(0.830, 0.910)
                .offset(GRAB_OFFSET)
                .state(GrabCup)
                .build(),
            G("DEPOSIT_CARPET_LEFT")
                .weight(2)
                .coords(0.740, 1.08)
                .direction(DIRECTION_FORWARD)
                .state(DepositCarpet, (SIDE_LEFT,))
                .build(),
            G("DEPOSIT_CARPET_RIGHT")
                .weight(3)
                .coords(0.740, 1.40)
                .direction(DIRECTION_FORWARD)
                .state(DepositCarpet, (SIDE_RIGHT,))
                .build(),
            DCG("DEPOSIT_CUP_HOME")
                .weight(4)
                .coords(1.0, 0.5)
                .state(DepositCup)
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
                .coords(0.55, 2.70)
                .state(DepositCup)
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
                .coords(1.45, 2.70)
                .state(DepositCup)
                .build(),
            G("KICK_THEIRS_CLAP")
                .weight(7)
                .coords(2.0 - 0.0825 - 0.070, 2.55)
                .direction(DIRECTION_FORWARD)
                .state(KickTheirsClap)
                .build(),
            GCG("GRAB_THEIRS_STAIRS_CUP")
                .weight(15)
                .coords(0.830, 3.0 - 0.910)
                .offset(GRAB_OFFSET)
                .state(GrabCup)
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
        start_y = 0.51

        start_angle = math.atan2(0.91 - start_y, 0.80 - start_x) + math.pi
        start_angle = tools.normalize_angle(start_angle)

        if IS_HOST_DEVICE_ARM:
            setup_x = 0.8 - 0.022 - ROBOT_CENTER_X
            turn_x = 0.63
            yield DefinePosition(setup_x, 1.0, math.pi)
            yield MoveLineTo(turn_x, 1.0)
            yield RotateTo(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(turn_x, 0.0)
            yield SpeedControl()
            yield DefinePosition(None, ROBOT_CENTER_X + HIGH_BORDER_OFFSET, math.pi / 2.0)
            yield MoveLineTo(turn_x, start_y)
            yield RotateTo(0)
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
            yield from self.execute_goal("DEPOSIT_CUP_HOME", None, "NORTH_ZONE")
            yield WaitForUnlock("SOUTH_ZONE", 5000)
            yield SafeMoveLineTo(1.0, 0.55)
            yield from self.execute_goal("GRAB_SOUTH_MINE_CUP")
            yield MoveLineTo(1.40, 0.2)
            yield WaitForUnlock("CROSS_FIELD", 20000)
        except OpponentInTheWay:
            pass
        yield None


    def execute_goal(self, name, forced_direction = None, unlock_zone = None):
        goal = self.robot.goal_manager.get_goals(name)[0]
        x, y = get_offset_position(self.robot.pose, goal.x, goal.y, goal.offset)
        direction = forced_direction if forced_direction is not None else goal.direction
        if direction == DIRECTION_FORWARD:
            yield LookAt(x, y)
        else:
            yield LookAtOpposite(x, y)
        yield SafeMoveLineTo(x, y)
        if unlock_zone is not None:
            self.send_packet(packets.InterbotUnlock(unlock_zone))
        goal.doing()
        state = yield goal.get_state()
        if state.exit_reason == GOAL_DONE :
            goal.done()
        else:
            goal.available()
        self.robot.goal_manager.update_goal_status(name, GOAL_DONE)




class GrabCup(Timer):

    def __init__(self):
        super().__init__(500)


    def on_enter(self):
        self.exit_reason = GOAL_DONE
        super_on_enter = super().on_enter()
        if super_on_enter:
            yield from super_on_enter


    def on_cup_grabbed(self, packet):
        yield None




class GrabSouthCornerCup(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        if goal.identifier == "GRAB_SOUTH_THEIRS_CUP" and "DEPOSIT_OPP_SOUTH" in self.robot.handled_cup_zones:
            yield from self.grab_simple()
        else:
            yield from self.grab_with_recalibrate()


    def grab_with_recalibrate(self):
        goal = self.robot.goal_manager.get_current_goal()
        mine = goal.y < 1.5
        cup_y = 0.250 if mine else 3.0 - 0.250

        # Recalibrate
        a = math.pi / 2.0 if mine else -math.pi / 2.0
        yield RotateTo(a)
        yield SpeedControl(0.2)
        y = 0.0 if mine else 3.0
        yield MoveLineTo(goal.x, y)
        offset = ROBOT_CENTER_X + HIGH_BORDER_OFFSET
        ry = offset if mine else 3.0 - offset
        yield DefinePosition(None, ry, a)
        yield SpeedControl()
        yield MoveLineTo(goal.x, cup_y)
        yield RotateTo(0)
        yield SpeedControl(0.2)
        yield MoveLineTo(1.0, cup_y)
        yield DefinePosition(1.222 + ROBOT_CENTER_X, None, 0.0)
        yield SpeedControl()

        yield MoveLineTo(1.4, cup_y)
        yield RotateTo(math.pi)

        # Grab cup
        yield SafeMoveLineTo(1.750 + GRAB_OFFSET, cup_y)
        grab = yield GrabCup()
        self.exit_reason = grab.exit_reason
        yield None


    def grab_simple(self):
        yield LookAtOpposite(1.750 + GRAB_OFFSET, cup_y)
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
        yield None




class DepositCup(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        if goal.identifier == "DEPOSIT_CUP_HOME":
            yield RotateTo(math.pi / 2.0)
            yield MoveLineTo(1.0, 0.30)
        else:
            yield RotateTo(-math.pi / 2.0)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_HALF_OPEN)
        yield Timer(500)
        yield Trigger(CUP_GRIPPER_OPEN)
        cup_presence = yield GetInputStatus(SECONDARY_INPUT_CUP_PRESENCE)
        yield MoveLineRelative(0.05)
        self.fsm.cup_grabber.enabled = False
        yield Trigger(CUP_GRIPPER_SEEKING)
        yield Timer(500)
        self.fsm.cup_grabber.enabled = True
        self.robot.holding_cup = False
        if cup_presence.value == 0:
            self.robot.handled_cup_zones.append(goal.identifier)
            self.exit_reason = GOAL_DONE
        else:
            self.log("We had  no cup :(")
            self.exit_reason = GOAL_FAILED
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
        yield Trigger(self.dropper_open)
        yield Trigger(self.dropper_close)
        yield Trigger(self.dropper_open)
        yield Trigger(self.dropper_close)
        yield Trigger(self.dropper_open)
        yield Trigger(self.dropper_close)
        yield MoveLineTo(goal.x, goal.y)
        self.exit_reason = GOAL_DONE
        yield None




class KickTheirsClap(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        offset = 0
        if self.robot.team == TEAM_RIGHT:
            offset = 0.1
            yield RotateTo(-math.pi / 2)
            yield MoveLineTo(goal.x, goal.y + offset)
        yield RotateTo(math.pi)
        yield SpeedControl(0.2)
        yield MoveLineTo(2.50, goal.y + offset)
        yield DefinePosition(2.0 - (ROBOT_CENTER_X + HIGH_BORDER_OFFSET), None, math.pi)
        yield SpeedControl()
        yield MoveLineTo(goal.x, goal.y + offset)
        if self.robot.team == TEAM_LEFT:
            yield RotateTo(3 * math.pi / 4)
            yield Trigger(CLAPMAN_OPEN)
            yield RotateTo(math.pi / 2)
        else:
            yield RotateTo(-3 * math.pi / 4)
            yield Trigger(CLAPMAN_OPEN)
            yield RotateTo(-math.pi / 2)
        yield SafeMoveLineTo(goal.x, 2.300 + offset)
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
