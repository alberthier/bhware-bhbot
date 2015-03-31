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
import statemachines.testsmain as testsmain




STAND_GRAB_OFFSET = 0.01
STAND_GOAL_OFFSET = -0.25

STAND_PRESENCE_SENSOR_OFFSET = 0.0




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
        # TODO : maybe created at the wrong place
        SysInfo(self.event_loop)
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        StateMachine(self.event_loop, "bulbgrabber")
        self.fsm.builders = {
            SIDE_LEFT: StateMachine(self.event_loop, "standbuilder", side=SIDE_LEFT),
            SIDE_RIGHT: StateMachine(self.event_loop, "standbuilder", side=SIDE_RIGHT)
        }

        self.robot.goal_manager.add(
                           # identifier, order, x, y, offset, direction, handler_state
            goalmanager.Goal("GRAB_NORTH_MINE_STAND", 2, 0.42, 0.30, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 0.200, 0.090, False)),
            StandGoal("GRAB_PLATFORM_1_STAND", 3, SIDE_LEFT, 1.355, 0.870, GoalGrabStand),
            goalmanager.Goal("GRAB_PLATFORM_2_STAND", 4, 1.600, 0.900, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 1.770, 1.100, False)),
            StandGoal("GRAB_PLATFORM_3_STAND", 5, SIDE_LEFT, 1.400, 1.300, GoalGrabStand),
            goalmanager.Goal("GRAB_SOUTH_MINE_STANDS", 6, 1.45, 0.22, 0, DIRECTION_FORWARD, GrabSouthMineStands),
            goalmanager.Goal("KICK_MINE_CLAPS", 7, 1.77, 0.22, 0, DIRECTION_FORWARD, KickMineClaps),
            goalmanager.Goal("SCAN_AND_BUILD_SPOTLIGHT", 8, 1.67, 1.2 - STAND_PRESENCE_SENSOR_OFFSET, 0, DIRECTION_FORWARD, ScanAndBuildSpotlight),
            goalmanager.Goal("KICK_THEIR_CLAP", 9, 1.77, 2.62, 0, DIRECTION_FORWARD, KickTheirClap),
        )

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield GetInputStatus(MAIN_INPUT_TEAM)
            # yield CalibratePosition()


    def on_bulb_grabbed(self, packet):
        logger.log("Starting ...")
        # yield StaticStrategy()
        # yield ExecuteGoals()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LIGHTER_GRIPPER_OPEN,
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




class StaticStrategy(State):

    def on_enter(self):
        # GRAB_NORTH_STAIRS_STANDS
        yield MoveLineTo(LEFT_START_X, 0.53)
        yield GrabStand(SIDE_RIGHT, 0.200, 0.850, True)
        yield GrabStand(SIDE_RIGHT, 0.100, 0.850, False)
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
        yield ResettleAfterSouthMineStands()

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
        yield ResettleAfterSouthMineStands()
        yield None




class ResettleAfterSouthMineStands(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_goals("KICK_MINE_CLAPS")[0]
        yield MoveCurve(0.0, [(1.4, goal.y), (1.32, goal.y)])
        yield DefinePosition(1.222 + ROBOT_CENTER_X, None, 0.0)
        yield None




class KickMineClaps(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(math.pi / 2.0)
        yield MoveLineTo(goal.x, 0.10)
        yield DefinePosition(None, ROBOT_CENTER_X, math.pi / 2.0)
        if self.robot.team == TEAM_LEFT:
            self.commands = [RIGHT_CLAPMAN_OPEN, RIGHT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_OPEN, RIGHT_CLAPMAN_CLOSE]
        else:
            self.commands = [LEFT_CLAPMAN_OPEN, LEFT_CLAPMAN_CLOSE, LEFT_CLAPMAN_OPEN, LEFT_CLAPMAN_CLOSE]
        self.commands.reverse()
        yield Trigger(self.commands.pop())
        move = MoveLine([(goal.x, 0.22), (goal.x, 0.60), (goal.x, 0.88)])
        move.on_waypoint_reached = self.on_waypoint_reached
        yield move
        yield Trigger(self.commands[-1]) # Use last command in any case as the move may have failed
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            self.exit_reason = GOAL_DONE
        yield None


    def on_waypoint_reached(self, packet):
        self.send_packet(packets.ServoControl(*self.commands.pop()))




class KickTheirClap(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()

        yield RotateTo(-math.pi / 2.0)
        if self.robot.team == TEAM_LEFT:
            yield Trigger(LEFT_CLAPMAN_OPEN)
        else:
            yield Trigger(RIGHT_CLAPMAN_OPEN)
        yield MoveLineTo(goal.x, 2.40)
        if self.robot.team == TEAM_LEFT:
            yield Trigger(LEFT_CLAPMAN_CLOSE)
        else:
            yield Trigger(RIGHT_CLAPMAN_CLOSE)
        self.exit_reason = GOAL_DONE
        yield None


class BuildSpotlight(State):

    def on_enter(self):
        # yield MoveLineTo(goal.x, center_y)
        # yield RotateTo(0.0)

        self.send_packet(packets.StandAction(side=SIDE_LEFT,action=STAND_ACTION_START))
        self.send_packet(packets.StandAction(side=SIDE_RIGHT,action=STAND_ACTION_START))

        # yield MoveLineTo(1.77, center_y)
        # yield DefinePosition(1.9 - ROBOT_CENTER_X, None, 0.0)

        # self.exit_reason = GOAL_DONE
        # yield None


    def on_start(self, packet):
        if packet.value == 1:
            self.send_packet(packets.StandAction(side=SIDE_LEFT,action=STAND_ACTION_DEPOSIT))
            self.send_packet(packets.StandAction(side=SIDE_RIGHT,action=STAND_ACTION_DEPOSIT))
        else:
            self.send_packet(packets.StandAction(side=SIDE_LEFT,action=STAND_ACTION_END))
            self.send_packet(packets.StandAction(side=SIDE_RIGHT,action=STAND_ACTION_END))




class ScanAndBuildSpotlight(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        yield RotateTo(math.pi / 2.0)
        self.start_pose = self.robot.pose
        self.stop_pose = None
        move = MoveLineTo(goal.x, 1.8 - STAND_PRESENCE_SENSOR_OFFSET)
        move.on_keep_alive = self.on_keep_alive
        if self.robot.team == TEAM_LEFT:
            move.on_right_scanner = self.on_scanner
        else:
            move.on_left_scanner = self.on_scanner
        yield move
        if self.start_pose is not None:
            if self.stop_pose is None:
                self.stop_pose = self.robot.pose
            center_y = STAND_PRESENCE_SENSOR_OFFSET + (self.start_pose.virt.y + self.stop_pose.virt.y) / 2.0
            yield MoveLineTo(goal.x, center_y)
            yield RotateTo(0.0)
            self.fsm.builders[SIDE_LEFT].enabled = False
            self.fsm.builders[SIDE_RIGHT].enabled = False
            # yield Trigger(LEFT_BUILDER_LIGHTER_OPEN, RIGHT_BUILDER_LIGHTER_OPEN)
            # yield Trigger(LEFT_BUILDER_LIGHTER_CLOSE, RIGHT_BUILDER_LIGHTER_CLOSE)
            # yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_GUIDE, LEFT_BUILDER_GRIPPER_RIGHT_GUIDE,
            #               RIGHT_BUILDER_GRIPPER_LEFT_GUIDE, RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE)
            # yield Trigger(LEFT_BUILDER_ELEVATOR_UP, RIGHT_BUILDER_ELEVATOR_UP)

            self.send_packet(packets.StandAction(side=SIDE_RIGHT,action=STAND_ACTION_START))

            yield MoveLineTo(1.77, center_y)
            yield DefinePosition(1.9 - ROBOT_CENTER_X, None, 0.0)
            # yield Trigger(LEFT_BUILDER_ELEVATOR_PLATFORM, RIGHT_BUILDER_ELEVATOR_PLATFORM)
            # yield Trigger(LEFT_BUILDER_PLIERS_LEFT_OPEN, LEFT_BUILDER_PLIERS_RIGHT_OPEN,
            #               RIGHT_BUILDER_PLIERS_LEFT_OPEN, RIGHT_BUILDER_PLIERS_RIGHT_OPEN,
            #               LEFT_BUILDER_GRIPPER_LEFT_OPEN, LEFT_BUILDER_GRIPPER_RIGHT_OPEN,
            #               RIGHT_BUILDER_GRIPPER_LEFT_OPEN, RIGHT_BUILDER_GRIPPER_RIGHT_OPEN)
            # yield MoveLineTo(1.68, center_y)
            # self.send_packet(packets.ServoControl(*LEFT_BUILDER_GRIPPER_LEFT_CLOSE))
            # self.send_packet(packets.ServoControl(*LEFT_BUILDER_GRIPPER_RIGHT_CLOSE))
            # self.send_packet(packets.ServoControl(*RIGHT_BUILDER_GRIPPER_LEFT_CLOSE))
            # self.send_packet(packets.ServoControl(*RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE))
            self.fsm.builders[SIDE_LEFT].stand_count = 0
            self.fsm.builders[SIDE_RIGHT].stand_count = 0
            self.fsm.builders[SIDE_LEFT].enabled = True
            self.fsm.builders[SIDE_RIGHT].enabled = True

        self.exit_reason = GOAL_DONE
        yield None


    def on_scanner(self, packet):
        if packet.value == 1:
            if not self.check_distance():
                self.start_pose = None
        else:
            self.start_pose = self.robot.pose


    def on_keep_alive(self, packet):
        self.check_distance()


    def check_distance(self):
        required_space = ROBOT_CENTER_Y * 2
        if self.start_pose is not None:
            d = distance(self.start_pose.x, self.start_pose.y, self.robot.pose.x, self.robot.pose.y)
            if d >= required_space:
                self.log("Deposit position found (distance = {})".format(d))
                self.stop_pose = self.robot.pose
                self.send_packet(packets.Stop())
                return True
        return False




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
