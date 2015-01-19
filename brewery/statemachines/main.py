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
import statemachines.testsmain as testsmain




STAND_GRAB_OFFSET = 0.01
STAND_GOAL_OFFSET = -0.25




def left_builder_at_pose(x, y, angle):
    return get_center_pose_for_point(0.150, 0.0725, x, y, angle)


def right_builder_at_pose(x, y, angle):
    return get_center_pose_for_point(0.150, -0.0725, x, y, angle)


def builder_at_point(side, robot_pose, x, y):
    px = 0.150
    py = 0.0725 if side == SIDE_LEFT else -0.0725

    ra = math.atan2(robot_pose.virt.y, robot_pose.virt.x)
    da = math.atan2(y, x)

    ra = angle_between(robot_pose.virt.x, robot_pose.virt.y, x, y)
    d = distance(robot_pose.virt.x, robot_pose.virt.y, x, y)
    angle = ra - math.asin(py / d)
    return get_center_pose_for_point(px, py, x, y, angle)


def left_builder_at_point(robot_pose, x, y):
    return builder_at_point(SIDE_LEFT, robot_pose, x, y)


def right_builder_at_point(robot_pose, x, y):
    return builder_at_point(SIDE_RIGHT, robot_pose, x, y)




class StandGoal(goalmanager.Goal):

    def __init__(self, identifier, weight, side, x, y, handler_state, ctor_parameters = None):
        super().__init__(identifier, weight, x, y, STAND_GOAL_OFFSET, DIRECTION_FORWARD, handler_state, ctor_parameters)
        self.side = side




class Main(State):

    def on_enter(self):
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        self.fsm.builders = {}
        self.fsm.builders[SIDE_LEFT]  = StateMachine(self.event_loop, "standbuilder", side = SIDE_LEFT)
        self.fsm.builders[SIDE_RIGHT] = StateMachine(self.event_loop, "standbuilder", side = SIDE_RIGHT)

        gm = self.robot.goal_manager
        gm.add(
                           # identifier, order, x, y, offset, direction, handler_state
            goalmanager.Goal("GRAB_NORTH_MINE_STAND", 2, 0.42, 0.30, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 0.200, 0.090, False)),
            StandGoal("GRAB_PLATFORM_1_STAND", 3, SIDE_LEFT, 1.355, 0.870, GoalGrabStand),
            goalmanager.Goal("GRAB_PLATFORM_2_STAND", 4, 1.600, 0.900, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 1.770, 1.100, False)),
            StandGoal("GRAB_PLATFORM_3_STAND", 5, SIDE_LEFT, 1.400, 1.300, GoalGrabStand),
            goalmanager.Goal("GRAB_SOUTH_MINE_STANDS", 6, 1.45, 0.22, 0, DIRECTION_FORWARD, GrabSouthMineStands),
        )

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield GetInputStatus(MAIN_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        self.yield_at(90000, EndOfMatch())
        logger.log("Starting ...")
        yield PickupBulb()
        yield StaticStrategy()
        yield ExecuteGoals()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_BUILDER_PLIERS_LEFT_CLOSE, LEFT_BUILDER_PLIERS_RIGHT_CLOSE,
                      RIGHT_BUILDER_PLIERS_LEFT_CLOSE, RIGHT_BUILDER_PLIERS_RIGHT_CLOSE,
                      LEFT_BUILDER_ELEVATOR_DOWN,
                      RIGHT_BUILDER_ELEVATOR_DOWN,
                      LEFT_BUILDER_GRIPPER_LEFT_CLOSE, LEFT_BUILDER_GRIPPER_RIGHT_CLOSE,
                      RIGHT_BUILDER_GRIPPER_LEFT_CLOSE, RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE,
                      LEFT_BUILDER_LIGHTER_CLOSE,
                      RIGHT_BUILDER_LIGHTER_CLOSE,
                      LIGHTER_GRIPPER_OPEN,
                      LIGHTER_ELEVATOR_DOWN,
                      LEFT_CLAPMAN_CLOSE,
                      RIGHT_CLAPMAN_CLOSE)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        if IS_HOST_DEVICE_ARM:
            yield DefinePosition(FIELD_X_SIZE - ROBOT_CENTER_X, 2.0, math.pi)
            yield MoveLineTo(LEFT_START_X, 2.0)
            yield RotateTo(math.pi / 2.0)
            yield SpeedControl(0.2)
            yield MoveLineTo(LEFT_START_X, 0.0)
            yield DefinePosition(None, LEFT_START_Y, math.pi / 2.0)
            yield SpeedControl()
        else:
            yield DefinePosition(LEFT_START_X, LEFT_START_Y, math.pi / 2.0)
        yield None




class WaitForStandStored(Timer):

    def __init__(self, side):
        super().__init__(2000)
        self.side = side

    def on_enter(self):
        builder = self.fsm.builders[self.side]
        if not builder.building:
            yield None


    def on_stand_stored(self, packet):
        if packet.side == self.side:
            yield None


    def on_timeout(self):
        self.log("{}({}) timed out !!!!!!!!!!!!!!".format(type(self).__name__, SIDE.lookup_by_value[self.side]))
        yield None




class WaitForStandGrabbed(WaitForStandStored):

    def on_stand_grabbed(self, packet):
        if packet.side == self.side:
            yield None




class PickupBulb(State):

    def on_enter(self):
        yield Trigger(LIGHTER_ELEVATOR_BULB)
        yield Trigger(LIGHTER_GRIPPER_CLOSE)
        yield Trigger(LIGHTER_ELEVATOR_UP)
        yield Trigger(LIGHTER_GRIPPER_OPEN)
        yield Trigger(LIGHTER_ELEVATOR_DOWN)
        yield None




class StaticStrategy(State):

    def on_enter(self):
        # GRAB_NORTH_STAIRS_STANDS
        x, y, angle = right_builder_at_pose(0.200 - STAND_GRAB_OFFSET, 0.850, math.pi)
        yield MoveLineTo(LEFT_START_X, y)
        yield RotateTo(angle)
        yield MoveLineTo(x, y)
        yield WaitForStandStored(SIDE_RIGHT)
        x, y, angle = right_builder_at_pose(0.100 - STAND_GRAB_OFFSET, 0.850, math.pi)
        yield MoveLineTo(x, y)
        yield WaitForStandGrabbed(SIDE_RIGHT)
        yield LookAtOpposite(0.42, 0.73)
        yield MoveLineTo(0.42, 0.73)

        # GRAB_NORTH_MINE_STAND
        yield RotateTo(-math.pi / 2.0)
        yield MoveLineTo(0.42, 0.30)
        yield from self.grab_stand("GRAB_NORTH_MINE_STAND", SIDE_LEFT, 0.200, 0.090, False)
        x, y = get_crossing_point(self.robot.pose.virt.x, self.robot.pose.virt.y, self.robot.pose.virt.angle, 0.5, 1.5, math.pi / 2.0)
        yield MoveLineTo(x, y)

        # GRAB_PLATFORM_1_STAND
        yield LookAt(0.5, 0.62)
        yield MoveLineTo(0.5, 0.62)
        yield from self.grab_stand("GRAB_PLATFORM_1_STAND", SIDE_LEFT, 1.355, 0.870, False)

        # GRAB_PLATFORM_2_STAND
        yield from self.grab_stand("GRAB_PLATFORM_2_STAND", SIDE_LEFT, 1.770, 1.100, False)

        # GRAB_PLATFORM_3_STAND
        yield from self.grab_stand("GRAB_PLATFORM_3_STAND", SIDE_LEFT, 1.400, 1.300, False)

        # GRAB_SOUTH_MINE_STANDS
        x, y = 1.45, 0.22
        yield LookAt(x, y)
        yield MoveLineTo(x, y)
        yield from self.grab_stand(None, SIDE_RIGHT, 1.75, 0.09, True)
        yield from self.grab_stand("GRAB_SOUTH_MINE_STANDS", SIDE_RIGHT, 1.85, 0.09, True)

        yield None


    def grab_stand(self, goal_name, side, x, y, store_stand):
        grab = yield GrabStand(side, x, y, store_stand)
        if goal_name is not None:
            if grab.exit_reason == GOAL_DONE:
                self.robot.goal_manager.update_goal_status(goal_name, GOAL_DONE)
            else:
                yield None




class GrabStand(State):

    def __init__(self, side, x, y, store_stand):
        super().__init__()
        self.side = side
        self.x = x
        self.y = y
        self.store_stand = store_stand


    def on_enter(self):
        x, y, angle = builder_at_point(self.side, self.robot.pose, self.x, self.y)
        yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, STAND_GRAB_OFFSET)
        move = yield MoveLineTo(x, y)
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            if self.store_stand:
                yield WaitForStandStored(self.side)
            else:
                yield WaitForStandGrabbed(self.side)
            self.exit_reason = GOAL_DONE
        else:
            self.exit_reason = GOAL_FAILED
        yield None



class GoalGrabStand(GrabStand):

    def __init__(self):
        super().__init__(SIDE_LEFT, 0, 0, False)

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        self.x = goal.x
        self.y = goal.y
        yield from super().on_enter()




class GrabSouthMineStands(State):

    def on_enter(self):
        grab = yield GrabStand(SIDE_RIGHT, 1.75, 0.09, True)
        if grab.exit_reason == GOAL_DONE:
            grab = yield GrabStand(SIDE_RIGHT, 1.85, 0.09, False)
        self.exit_reason = grab.exit_reason
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
