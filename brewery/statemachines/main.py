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




class Main(State):

    def on_enter(self):
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        self.fsm.builders = {}
        self.fsm.builders[SIDE_LEFT]  = StateMachine(self.event_loop, "standbuilder", side = SIDE_LEFT)
        self.fsm.builders[SIDE_RIGHT] = StateMachine(self.event_loop, "standbuilder", side = SIDE_RIGHT)

        gm = self.robot.goal_manager
#        gm.add(
                           # identifier, order, x, y, offset, direction, handler_state
            #goalmanager.Goal("GRAB_NORTH_STANDS", 1, LEFT_START_X, p1y, 0.020, DIRECTION_FORWARD, GrabNorthStands),
            #goalmanager.Goal("GRAB_NORTH_MINE_STAND", 2, 0.42, 0.30, 0.0, DIRECTION_FORWARD, GrabNorthMineStand),
            #goalmanager.Goal("GRAB_SOUTH_CENTER_STAND_1", 2, 0.42, 0.30, 0.0, DIRECTION_FORWARD, GrabNorthMineStand),
#        )


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


    def on_stand_construction_done(self, packet):
        if packet.side == self.side:
            yield None


    def on_timeout(self):
        self.log("{} timed out !!!!!!!!!!!!!!".format(type(self).__name__))
        yield None




class WaitForStandGrabbed(Timer):

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
        stand_grab_offset = 0.01

        # GRAB_NORTH_STAIRS_STANDS
        x, y, angle = right_builder_at_pose(0.190, 0.850, math.pi)
        yield MoveLineTo(LEFT_START_X, y)
        yield RotateTo(angle)
        yield MoveLineTo(x, y)
        yield WaitForStandStored(SIDE_RIGHT)
        x, y, angle = right_builder_at_pose(0.090, 0.850, math.pi)
        yield MoveLineTo(x, y)
        yield WaitForStandGrabbed(SIDE_RIGHT)
        yield LookAtOpposite(0.42, 0.73)
        yield MoveLineTo(0.42, 0.73)

        # GRAB_NORTH_MINE_STAND
        yield RotateTo(-math.pi / 2.0)
        yield MoveLineTo(0.42, 0.30)
        x, y, angle = left_builder_at_point(self.robot.pose, 0.200, 0.090)
        yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, stand_grab_offset)
        yield MoveLineTo(x, y)
        yield WaitForStandGrabbed(SIDE_LEFT)
        x, y = get_crossing_point(self.robot.pose.virt.x, self.robot.pose.virt.y, self.robot.pose.virt.angle, 0.5, 1.5, math.pi / 2.0)
        yield MoveLineTo(x, y)

        # GRAB_SOUTH_PLATFORM_STANDS
        yield LookAt(0.5, 0.62)
        yield MoveLineTo(0.5, 0.62)
        stands_angle = angle_between(1.355, 0.870, 1.770, 1.100)
        self.log("stands_angle = {}".format(stands_angle))
        sx, sy, angle = left_builder_at_pose(1.770, 1.100, stands_angle)
        x, y = get_crossing_point(self.robot.pose.virt.x, self.robot.pose.virt.y, 0.0, sx, sy, stands_angle)
        self.log("cx, cy = {} {}".format(x, y))
        yield LookAt(x, y)
        yield MoveLineTo(x, y)
        yield RotateTo(angle)
        sx, sy = get_offset_position(self.robot.pose, sx, sy, stand_grab_offset)
        yield MoveLineTo(sx, sy)
        yield WaitForStandGrabbed(SIDE_LEFT)

        # GRAB_PLATFORM_CENTER_STAND
        x, y, angle = left_builder_at_point(self.robot.pose, 1.4, 1.3)
        yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, stand_grab_offset)
        yield MoveLineTo(x, y)
        yield WaitForStandGrabbed(SIDE_LEFT)

        # GRAB_SOUTH_MINE_STANDS
        yield RotateTo(-math.pi / 2.0)
        yield MoveLineTo(x, 0.27)
        x, y, angle = right_builder_at_point(self.robot.pose, 1.75, 0.09)
        yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, stand_grab_offset)
        yield MoveLineTo(x, y)
        yield WaitForStandStored(SIDE_RIGHT)
        x, y, angle = right_builder_at_point(self.robot.pose, 1.85, 0.09)
        yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, stand_grab_offset)
        yield MoveLineTo(x, y)
        yield WaitForStandStored(SIDE_RIGHT)

        yield None




class GrabNorthStairsStands(State):

    def on_enter(self):
        x, y, angle = right_builder_at_pose(0.080, 0.850, math.pi)

        yield MoveLineTo(LEFT_START_X, y)
        yield RotateTo(angle)
        yield MoveLineTo(x, y)
        yield LookAtOpposite(0.42, 0.73)
        yield MoveLineTo(0.42, 0.73)

        yield None




class GrabNorthMineStand(State):

    def on_enter(self):
        x, y, angle = left_builder_at_point(self.robot.pose, 0.200, 0.090)
        yield RotateTo(angle)
        yield MoveLineTo(x, y)
        yield MoveLineRelative(0.2, DIRECTION_BACKWARDS)
        self.exit_reason = GOAL_DONE
        yield None




class WaitForStandGrabbing(State):

    def on_enter(self):
        yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
