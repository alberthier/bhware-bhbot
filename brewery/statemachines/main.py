# encoding: utf-8

import collections
import math

import packets
import position
import logger
import goalmanager
import tools
import robot

from statemachine import *
from definitions import *
from commonstates import *
from position import *
from tools import *

import statemachines.testscommon as testscommon
import statemachines.testsmain as testsmain




LEFT_START_X = 1.000
LEFT_START_Y = 0.070 + ROBOT_CENTER_Y
LEFT_START_ANGLE = math.pi / 2.0


def left_builder_at_pose(x, y, angle):
    return get_center_pose_for_point(0.150, 0.0725, x, y, angle)


def right_builder_at_pose(x, y, angle):
    return get_center_pose_for_point(0.150, -0.0725, x, y, angle)


def builder_at_point(side, robot_pose, x, y):
    px = 0.150
    py = 0.0725 if side == SIDE_LEFT else -0.0725

    da = math.atan2(y, x)

    ra = angle_between(robot_pose.virt.x, robot_pose.virt.y, x, y)
    d = distance(robot_pose.virt.x, robot_pose.virt.y, x, y)
    angle = ra - math.asin(py / d)
    return get_center_pose_for_point(px, py, x, y, angle)


def left_builder_at_point(robot_pose, x, y):
    return builder_at_point(SIDE_LEFT, robot_pose, x, y)


def right_builder_at_point(robot_pose, x, y):
    return builder_at_point(SIDE_RIGHT, robot_pose, x, y)


class ScoreEstimator:
    def __init__(self):
        self.temporary_score = 0

    def before_goal(self, goal, robot):
        self.temporary_score=0

        if "KICK" in goal.tags:
            self.temporary_score=5
        elif "BUILD" in goal.tags:
            #TODO: only one construction per zone counts !!!
            if goal.builder_action[0]<0:
                if robot.left_stand_count:
                    if robot.has_left_bulb:
                        self.temporary_score=robot.left_stand_count*5
                    else:
                        self.temporary_score=robot.left_stand_count*2
            else:
                if robot.right_stand_count:
                    if robot.has_right_bulb:
                        self.temporary_score=robot.right_stand_count*5
                    else:
                        self.temporary_score=robot.right_stand_count*2

    def after_goal_success(self, goal_id, robot):
        robot.score+=self.temporary_score
        retval=self.temporary_score
        self.temporary_score=0
        return retval

class Main(State):

    def init_platform_build_position(self):
        cx, cy, ca = get_center_pose_for_point(0.150, -0.0725, 2.00 - 0.05, 1.20 - 0.05, math.pi / 4.0)
        self.fsm.build_spotlight_platform_x = cx - 0.1
        self.fsm.build_spotlight_platform_y = cy - 0.1


    def on_enter(self):
        self.init_platform_build_position()

        self.robot.has_left_bulb = False
        self.robot.has_right_bulb = False

        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        StateMachine(self.event_loop, "bulbgrabber")
        self.fsm.builders = {
            SIDE_LEFT: StateMachine(self.event_loop, "standbuilder", side=SIDE_LEFT),
            SIDE_RIGHT: StateMachine(self.event_loop, "standbuilder", side=SIDE_RIGHT)
        }

        self.send_packet(packets.Text("Main robot initializing"))
        self.send_packet(packets.Say("Le docteur d'enfer est prêt à en découdre"))

        gm=self.robot.goal_manager
        gm.score_estimator=ScoreEstimator()

        G = goalmanager.GoalBuilder

        gm.add(
            G("GRAB_NORTH_CORNER_STAND")
                .weight(2)
                .coords(0.420, 0.300)
                .direction(DIRECTION_FORWARD)
                .state(GrabNorthCornerStand)
                .builder_action(1,0)
                .estimated_duration(5)
                .build(),
            G("GRAB_STAIRS_STAND")
                .weight(2)
                .coords(0.400, 0.770)
                .direction(DIRECTION_FORWARD)
                .state(GrabStairsStands)
                .builder_action(0,2)
                .estimated_duration(10)
                .build(),
            G("GRAB_CENTER_WEST_STAND")
                .weight(3)
                .coords(1.100, 0.870 - 0.0725)
                .direction(DIRECTION_FORWARD)
                .state(GrabCenterWestStand)
                .builder_action(1,0)
                .estimated_duration(5)
                .disabled()
                .build(),
            G("GRAB_CENTER_EAST_STAND")
                .weight(4)
                .coords(1.150, 1.300 - 0.0725)
                .direction(DIRECTION_FORWARD)
                .state(GrabCenterEastStand)
                .builder_action(1,0)
                .estimated_duration(5)
                .disabled()
                .build(),
            G("GRAB_CENTER_SOUTH_STAND")
                .weight(5)
                .coords(1.500, 1.100 - 0.0725)
                .direction(DIRECTION_FORWARD)
                .state(GrabCenterSouthStand)
                .builder_action(1,0)
                .disabled()
                .build(),
            G("GRAB_SOUTH_CORNER_STANDS")
                .weight(6)
                .coords(1.400, 0.730)
                .direction(DIRECTION_FORWARD)
                .state(GrabSouthCornerStandsDirect)
                .builder_action(1,1)
                .estimated_duration(10)
                .build(),
            G("KICK_MINE_CLAPS")
                .weight(7)
                .coords(1.76, 0.25)
                .direction(DIRECTION_FORWARD)
                .state(KickMineClaps)
                .not_before(["GRAB_SOUTH_CORNER_STANDS"])
                .build(),
            G("BUILD_SPOTLIGHT_HOME")
                .weight(10)
                .coords(1.0, 0.58)
                .direction(DIRECTION_FORWARD)
                .state(BuildSpotlightHome)
                .builder_action(-1,0)
                .estimated_duration(15)
                .build(),
#            G("KICK_THEIR_CLAP")
#                .weight(9)
#                .coords(1.77, 2.62)
#                .direction(DIRECTION_FORWARD)
#                .state(KickTheirClap)
#                .build(),
            G("BUILD_SPOTLIGHT_PLATFORM")
                .weight(8)
                .coords(self.fsm.build_spotlight_platform_x, self.fsm.build_spotlight_platform_y)
                .direction(DIRECTION_FORWARD)
                .state(BuildSpotlightPlatform)
                .builder_action(0,-1)
                .estimated_duration(15)
                .build(),
            G("THE_BITCHY_WAY")
                .weight(20)
                .coords(1.0, 0.73)
                .direction(DIRECTION_BACKWARDS)
                .state(TheBitchyWay)
                .estimated_duration(15)
                .not_before(["ALL", "BUILD_SPOTLIGHT_PLATFORM", "BUILD_SPOTLIGHT_HOME"])
                .build(),
            # G("BUILD_SPOTLIGHT_PLATFORM_ALTERNATE_LEFT")
            #     .weight(15)
            #     .coords(1.73, 1.96)
            #     .direction(DIRECTION_FORWARD)
            #     .state(BuildSpotlightGeneric, (-math.pi/4, SIDE_LEFT))
            #     .builder_action(-1,0)
            #     .estimated_duration(15)
            #     .build(),
            # G("BUILD_SPOTLIGHT_PLATFORM_ALTERNATE_RIGHT")
            #     .weight(15)
            #     .coords(1.74, 1.96)
            #     .direction(DIRECTION_FORWARD)
            #     .state(BuildSpotlightGeneric, (-math.pi/4, SIDE_RIGHT))
            #     .builder_action(0,-1)
            #     .estimated_duration(15)
            #     .build()
            )

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield AntiBlocking(True)
            yield GetInputStatus(MAIN_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        if packet.value == 0:
            self.send_packet(packets.Say("C'est parti ! Faites entrer le clone !"))
            self.send_packet(packets.Text("Match started"))
            self.yield_at(89500, EndOfMatch())


    def on_bulb_grabbed(self, packet):
        logger.log("Starting ...")

        if self.robot.team == TEAM_LEFT:
            build_spotlight_platform = self.robot.goal_manager.get_goals("BUILD_SPOTLIGHT_PLATFORM")[0]
            self.fsm.build_spotlight_platform_x -= 0.01
            self.fsm.build_spotlight_platform_y += 0.03
            build_spotlight_platform.x = self.fsm.build_spotlight_platform_x
            build_spotlight_platform.y = self.fsm.build_spotlight_platform_y

        first_move = False
        while not first_move:
            try:
                yield SafeMoveLineTo(LEFT_START_X, 0.48)
                first_move = True
            except OpponentInTheWay:
                yield Timer(100)
        self.send_packet(packets.LiftBulb())
        yield StaticStrategy()
        # yield ExecuteGoals()
        while True:
            yield ExecuteGoalsV2()
            yield Timer(1000)




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_CLOSE)
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        yield DefinePosition(1.0, 0.07 + ROBOT_CENTER_X, math.pi / 2.0)
        yield None




class WaitForStandStored(Timer):

    def __init__(self, side, timeout_ms=4000):
        super().__init__(timeout_ms)
        self.side = side


    def on_stand_stored(self, packet):
        if packet.side == self.side:
            yield None


    def on_timeout(self):
        self.log("{}({}) timed out !!!!!!!!!!!!!!".format(type(self).__name__, SIDE.lookup_by_value[self.side]))
        yield None




class WaitForStandGrabbed(WaitForStandStored):

    def __init__(self, side, timeout_ms=2000):
        super().__init__(side, timeout_ms)

    def on_stand_grabbed(self, packet):
        if packet.side == self.side:
            yield None




class StaticStrategy(State):

    def first_stands_with_curve(self):
        p1 = Pose(*right_builder_at_point(self.robot.pose, 1.355, 0.870))
        x1, y1 = get_offset_position(self.robot.pose, p1.x, p1.y, 0.03)
        p2 = Pose(*left_builder_at_point(Pose(x1, y1), 1.400, 1.300))
        x2, y2 = get_offset_position(p1, p2.x, p2.y, 0.03)
        yield LookAt(p1.x, p1.y)
        yield SafeMoveCurve(None, 0.50, [ p1, Pose(x1, y1), p2, Pose(x2, y2) ], DIRECTION_FORWARD)


    def first_stands_with_line(self):
        adjust = math.radians(-2)
        if self.robot.team == TEAM_RIGHT:
            adjust = -adjust

        p1 = Pose(*left_builder_at_point(self.robot.pose, 1.400, 1.300))
        p2 = Pose(*get_offset_position(self.robot.pose, p1.x, p1.y, 0.05))
        r = distance(self.robot.pose.x, self.robot.pose.y, p2.x, p2.y)
        a1 = math.atan2(p2.y - self.robot.pose.y, p2.x - self.robot.pose.x)

        xr = self.robot.pose.x + r * math.cos(a1 + adjust)
        yr = self.robot.pose.y + r * math.sin(a1 + adjust)

        yield LookAt(xr, yr)
        yield SafeMoveLineTo(xr, yr)


    def first_stands_with_two_lines(self):
        adjust = math.radians(-2)
        if self.robot.team == TEAM_RIGHT:
            adjust = -adjust

        p1 = Pose(*left_builder_at_point(self.robot.pose, 1.400, 1.300))
        p2 = Pose(*get_offset_position(self.robot.pose, p1.virt.x, p1.virt.y, 0.05))
        r = distance(self.robot.pose.virt.x, self.robot.pose.virt.y, p2.virt.x, p2.virt.y)
        a1 = math.atan2(p2.virt.y - self.robot.pose.virt.y, p2.virt.x - self.robot.pose.virt.x)

        xr2 = self.robot.pose.virt.x + r * math.cos(a1 + adjust)
        yr2 = self.robot.pose.virt.y + r * math.sin(a1 + adjust)
        xr1, yr1 = get_offset_position(self.robot.pose, xr2, yr2, -0.3)

        yield LookAt(xr1, yr1)
        yield SafeMoveLineTo(xr1, yr1)
        yield WaitForStandStored(SIDE_RIGHT)
        yield SafeMoveLineTo(xr2, yr2)


    def on_enter(self):
        try:
            self.send_packet(packets.InterbotLock("SOUTH_ZONE"))
            self.send_packet(packets.InterbotLock("CROSS_FIELD"))
            try:
                yield from self.first_stands_with_line()
                self.robot.goal_manager.update_goal_status("GRAB_CENTER_WEST_STAND", GOAL_DONE)
                self.robot.goal_manager.update_goal_status("GRAB_CENTER_EAST_STAND", GOAL_DONE)
                yield WaitForStandStored(SIDE_LEFT)
            except OpponentInTheWay:
                self.log("GRAB_CENTER_WEST_STAND aborted")
                yield MoveLineRelative(-0.100)
            try:
                yield GrabCenterSouthStand()
                self.robot.goal_manager.update_goal_status("GRAB_CENTER_SOUTH_STAND", GOAL_DONE)
            except OpponentInTheWay:
                self.log("GRAB_CENTER_SOUTH_STAND aborted")
                yield MoveLineRelative(-0.100)

            yield RotateTo(-math.pi / 2.0)
            x, y, a = right_builder_at_pose(0.200, 0.850, math.pi)
            yield SafeMoveLineTo(self.robot.pose.x, y)
            yield RotateTo(math.pi)

            yield WaitForUnlock("NORTH_ZONE", 5000)

            yield GrabStairsStands()

            self.robot.goal_manager.update_goal_status("GRAB_STAIRS_STAND", GOAL_DONE)

            yield LookAt(0.42, 0.30)
            yield SafeMoveLineTo(0.42, 0.30)
            yield GrabNorthCornerStand()
            self.robot.goal_manager.update_goal_status("GRAB_NORTH_CORNER_STAND", GOAL_DONE)

            yield LookAt(1.400, 0.730)
            yield SafeMoveLineTo(1.400, 0.730)
            yield GrabSouthCornerStandsDirect()
            self.robot.goal_manager.update_goal_status("GRAB_SOUTH_CORNER_STANDS", GOAL_DONE)
            yield LookAtOpposite(1.76, 0.25)

            yield MoveLineTo(1.76, 0.25)
            kick = yield KickMineClaps()
            if kick.exit_reason == GOAL_DONE:
                self.robot.goal_manager.update_goal_status("KICK_MINE_CLAPS", GOAL_DONE)
            yield LookAt(self.fsm.build_spotlight_platform_x, self.fsm.build_spotlight_platform_y)
            yield MoveLineTo(self.fsm.build_spotlight_platform_x, self.fsm.build_spotlight_platform_y)
            yield BuildSpotlightPlatform()
            self.robot.goal_manager.update_goal_status("BUILD_SPOTLIGHT_PLATFORM", GOAL_DONE)

            yield LookAt(1.00, 0.60)
            yield SafeMoveLineTo(1.00, 0.60)
            yield BuildSpotlightHome()
            self.robot.goal_manager.update_goal_status("BUILD_SPOTLIGHT_HOME", GOAL_DONE)
            yield TheBitchyWay()
            self.robot.goal_manager.update_goal_status("THE_BITCHY_WAY", GOAL_DONE)

        except OpponentInTheWay:
            pass
        yield None




class GrabStand(State):

    def __init__(self, side, x, y, stand_grab_offset, store_stand, raise_on_error = True, skip_rotate = False, unlock_zone = None):
        super().__init__()
        self.side = side
        self.x = x
        self.y = y
        self.stand_grab_offset = stand_grab_offset
        self.store_stand = store_stand
        self.raise_on_error = raise_on_error
        self.skip_rotate = skip_rotate
        self.unlock_zone = unlock_zone


    def on_enter(self):
        x, y, angle = builder_at_point(self.side, self.robot.pose, self.x, self.y)
        if not self.skip_rotate:
            yield RotateTo(angle)
        x, y = get_offset_position(self.robot.pose, x, y, self.stand_grab_offset)
        self.exit_reason = GOAL_FAILED
        try:
            move = yield SafeMoveLineTo(x, y)
            goal = self.robot.goal_manager.get_current_goal()
            if self.unlock_zone is not None:
                self.log("Unlocking " + self.unlock_zone)
                self.send_packet(packets.InterbotUnlock(self.unlock_zone))
            else:
                self.log("Nothing to unlock")
            self.send_packet(packets.EnsureBuild(self.side))
            if self.store_stand:
                yield WaitForStandStored(self.side)
            else:
                yield WaitForStandGrabbed(self.side)
            self.exit_reason = GOAL_DONE
        except OpponentInTheWay as e:
            if self.raise_on_error:
                raise e
        yield None




class GrabStairsStands(State):

    def on_enter(self):
        grab = yield GrabStand(SIDE_RIGHT, 0.200, 0.850, 0.04, True, True, False, "SOUTH_ZONE")
        if grab.exit_reason == GOAL_DONE:
            grab = yield GrabStand(SIDE_RIGHT, 0.100, 0.850, 0.04, False, True)

        self.exit_reason = grab.exit_reason

        yield SafeMoveLineTo(0.68, 0.850 - 0.0725)

        yield None




class GrabNorthCornerStand(State):

    def on_enter(self):
        grab = yield GrabStand(SIDE_LEFT, 0.200, 0.090, 0.01, False)
        self.exit_reason = grab.exit_reason
        yield MoveLineRelative(-0.10)
        yield LookAtOpposite(0.60, 0.60)
        yield SafeMoveLineTo(0.60, 0.60)
        yield None




class GrabCenterWestStand(GrabStand):

    def __init__(self):
        super().__init__(SIDE_RIGHT, 1.355, 0.870, 0.01, False)




class GrabCenterSouthStand(GrabStand):

    def __init__(self):
        super().__init__(SIDE_LEFT, 1.770, 1.100, 0.01, False)




class GrabCenterEastStand(GrabStand):

    def __init__(self):
        super().__init__(SIDE_LEFT, 1.400, 1.300, 0.04, False)




class GrabSouthCornerStands(State):

    def on_enter(self):
        grab = yield GrabStand(SIDE_RIGHT, 1.75, 0.09, 0.01, True)
        yield MoveLineRelative(-0.20)
        if grab.exit_reason == GOAL_DONE:
            grab = yield GrabStand(SIDE_RIGHT, 1.85, 0.09, 0.02, False)
        self.exit_reason = grab.exit_reason
        yield ResettleAfterSouthCornerStands()

        yield None




class GrabSouthCornerStandsDirect(State):

    def on_enter(self):
        self.stored_stands = 0

        self.x1, self.y1 = 1.400, 0.730
        self.x2, self.y2 = 1.800, 0.330
        self.x3, self.y3 = 1.800, 0.180
        self.cx, self.cy = self.x1, self.y2
        self.r = self.x2 - self.x1

        yield RotateTo(0)
        yield SafeMoveLineTo(self.x1, self.y1)
        yield SafeMoveArc(self.cx, self.cy, self.r, [ 0.0 ], DIRECTION_FORWARD)
        self.send_packet(packets.InterbotUnlock("CROSS_FIELD"))
        yield SafeMoveLineTo(self.x3, self.y3)
        self.send_packet(packets.EnsureBuild(SIDE_LEFT))
        self.send_packet(packets.EnsureBuild(SIDE_RIGHT))

        self.exit_reason = GOAL_DONE


    def escape(self):
        yield SafeMoveLineTo(self.x2, self.y2)
        yield SafeMoveArc(self.cx, self.cy, self.r, [ math.pi / 6.0 ], DIRECTION_BACKWARDS)
        yield None


    def on_stand_stored(self, packet):
        self.stored_stands += 1
        if self.stored_stands == 2:
            yield from self.escape()




class ResettleAfterSouthCornerStands(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_goals("KICK_MINE_CLAPS")[0]
        #yield MoveCurve(0.0, 0.10, [(1.4, goal.y), (1.32, goal.y)])
        rp = self.robot.pose
        xc, yc = get_crossing_point(rp.x, rp.y, rp.angle, 1.3, 0.25, 0.0)
        yield LookAtOpposite(xc, yc)
        yield MoveLineTo(xc, yc)
        yield RotateTo(0.0)
        yield MoveLineTo(1.3, 0.25)
        yield DefinePosition(1.222 + ROBOT_CENTER_X, None, 0.0)
        yield None




class KickMineClaps(State):

    def on_enter(self):
        self.last_servo_command = None

        goal = self.robot.goal_manager.get_goals("KICK_MINE_CLAPS")[0]
        yield RotateTo(math.pi / 2.0)

        if self.robot.team == TEAM_LEFT:
            self.commands = [RIGHT_CLAPMAN_OPEN, RIGHT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_OPEN, RIGHT_CLAPMAN_CLOSE]
        else:
            self.commands = [LEFT_CLAPMAN_OPEN, LEFT_CLAPMAN_CLOSE, LEFT_CLAPMAN_OPEN, LEFT_CLAPMAN_CLOSE]
        points = [(goal.x, 0.28), (goal.x, 0.65), (goal.x, 0.88)]

        finish_command = self.commands[-1]

        self.commands.reverse()
        yield Trigger(self.commands.pop())

        yield SpeedControl(0.3)

        move = MoveLine(points)
        move.on_waypoint_reached = self.on_waypoint_reached
        yield move
        yield Trigger(self.commands[-1]) # Use last command in any case as the move may have failed
        if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
            self.exit_reason = GOAL_DONE
        else:
            if self.last_servo_command != finish_command:
                yield Trigger(finish_command)
            self.exit_reason = GOAL_FAILED

        yield SpeedControl()

        yield None


    def on_waypoint_reached(self, packet):
        self.last_servo_command = self.commands.pop()
        self.send_packet(packets.ServoControl(*self.last_servo_command))




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


class BuildSpotlightGeneric(State):

    def __init__(self, angle, side):
        self.side = side
        self.angle = angle

    def on_enter(self):
        yield RotateTo(self.angle)
        self.fsm.builders[self.side].enabled = False
        self.send_packet(packets.BuildSpotlight(self.side, platform_mode=False, finished=False))


    def on_build_spotlight(self, packet):
        if packet.side == self.side:
            if not packet.finished:
                self.send_packet(packets.BuildSpotlight(self.side, platform_mode=False, finished=True))
            else:
                yield MoveLineRelative(-0.15)
                self.fsm.builders[self.side].enabled = True
                self.send_packet(packets.StandbuilderIdle(self.side))
                self.exit_reason = GOAL_DONE
                yield None


class BuildSpotlightPlatform(State):

    def on_enter(self):
        yield RotateTo(math.pi / 4.0)
        self.fsm.builders[SIDE_RIGHT].enabled = False
        self.send_packet(packets.BuildSpotlight(SIDE_RIGHT, platform_mode=True, finished=False))


    def on_build_spotlight(self, packet):
        if packet.side == SIDE_RIGHT:
            if not packet.finished:
                a = math.pi / 4.0
                dist = 0.1
                x = self.fsm.build_spotlight_platform_x + math.cos(a) * dist
                y = self.fsm.build_spotlight_platform_y + math.sin(a) * dist
                yield MoveLineTo(x, y)
                self.send_packet(packets.BuildSpotlight(SIDE_RIGHT, platform_mode=True, finished=True))
            else:
                a = (math.pi / 4.0) + math.pi
                dist = 0.15
                x = self.fsm.build_spotlight_platform_x + math.cos(a) * dist
                y = self.fsm.build_spotlight_platform_y + math.sin(a) * dist
                while True:
                    move = yield SafeMoveLineTo(x, y)
                    if move.exit_reason == TRAJECTORY_DESTINATION_REACHED:
                        break
                self.fsm.builders[SIDE_RIGHT].enabled = True
                self.send_packet(packets.StandbuilderIdle(SIDE_RIGHT))
                self.exit_reason = GOAL_DONE
                yield None




class BuildSpotlightHome(State):

    def on_enter(self):
        yield RotateTo(-math.pi / 2)
        yield MoveLineTo(1.0, 0.5)
        self.fsm.builders[SIDE_LEFT].enabled = False
        self.send_packet(packets.BuildSpotlight(SIDE_LEFT, platform_mode=False, finished=False))


    def on_build_spotlight(self, packet):
        if packet.side == SIDE_LEFT:
            if not packet.finished:
                self.send_packet(packets.BuildSpotlight(SIDE_LEFT, platform_mode=False, finished=True))
            else:
                yield MoveLineRelative(-0.15)
                self.fsm.builders[SIDE_LEFT].enabled = True
                self.send_packet(packets.StandbuilderIdle(SIDE_LEFT))
                self.exit_reason = GOAL_DONE
                yield None




class TheBitchyWay(State):

    def on_enter(self):
        cdp_x1 = 0.57
        cdp_y1 = 0.73
        cdp_x2 = 0.57
        cdp_y2 = 0.6
        cdp_x3 = 0.57
        cdp_y3 = 0.25

        try:
            yield LookAt(cdp_x1, cdp_y1)
            yield SafeMoveLineTo(cdp_x1, cdp_y1)
            yield LookAt(cdp_x2, cdp_y2)
            yield Trigger(LEFT_BUILDER_PLIERS_LEFT_INIT,
                          LEFT_BUILDER_PLIERS_RIGHT_INIT,
                          RIGHT_BUILDER_PLIERS_LEFT_INIT,
                          RIGHT_BUILDER_PLIERS_RIGHT_INIT,
                          LEFT_BUILDER_GRIPPER_LEFT_DEPOSIT,
                          LEFT_BUILDER_GRIPPER_RIGHT_DEPOSIT,
                          RIGHT_BUILDER_GRIPPER_LEFT_DEPOSIT,
                          RIGHT_BUILDER_GRIPPER_RIGHT_DEPOSIT,
                          )
            yield SafeMoveLineTo(cdp_x3, cdp_y3)
            yield SafeMoveLineTo(cdp_x1, cdp_y1)
        except:
            yield SafeMoveLineTo(cdp_x2, cdp_y2)

        cdp_x1 = 1.425
        cdp_y1 = 0.73
        cdp_x2 = 1.425
        cdp_y2 = 0.6
        cdp_x3 = 1.425
        cdp_y3 = 0.25

        try:
            yield LookAt(1.00, cdp_y1)
            yield SafeMoveLineTo(1.00, cdp_y1)
            yield LookAt(cdp_x1, cdp_y1)
            yield SafeMoveLineTo(cdp_x1, cdp_y1)
            yield LookAt(cdp_x2, cdp_y2)
            yield Trigger(LEFT_BUILDER_PLIERS_LEFT_INIT,
                          LEFT_BUILDER_PLIERS_RIGHT_INIT,
                          RIGHT_BUILDER_PLIERS_LEFT_INIT,
                          RIGHT_BUILDER_PLIERS_RIGHT_INIT,
                          LEFT_BUILDER_GRIPPER_LEFT_DEPOSIT,
                          LEFT_BUILDER_GRIPPER_RIGHT_DEPOSIT,
                          RIGHT_BUILDER_GRIPPER_LEFT_DEPOSIT,
                          RIGHT_BUILDER_GRIPPER_RIGHT_DEPOSIT,
                          )
            yield SafeMoveLineTo(cdp_x3, cdp_y3)
            yield SafeMoveLineTo(cdp_x1, cdp_y1)
        except:
            yield SafeMoveLineTo(cdp_x2, cdp_y2)
        # YEAAAAHH BITCHES !!!!!!!!




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        goalmanager.on_end_of_match(self.robot.goal_manager, self.robot)
        tools.on_end_of_match()
