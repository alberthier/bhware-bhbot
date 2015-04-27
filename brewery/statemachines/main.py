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




LEFT_START_X = 1.0
LEFT_START_Y = 0.07 + ROBOT_CENTER_Y
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




class Main(State):

    def on_enter(self):
        self.fsm.interbot_fsm = StateMachine(self.event_loop, "interbot")
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        StateMachine(self.event_loop, "bulbgrabber")
        self.fsm.builders = {
            SIDE_LEFT: StateMachine(self.event_loop, "standbuilder", side=SIDE_LEFT),
            SIDE_RIGHT: StateMachine(self.event_loop, "standbuilder", side=SIDE_RIGHT)
        }

        G=goalmanager.GoalBuilder
        SG=functools.partial(goalmanager.GoalBuilder, ctor=goalmanager.StandGoal)

        self.robot.goal_manager.add(
            #goalmanager.Goal("GRAB_NORTH_MINE_STAND", 2, 0.42, 0.30, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 0.200, 0.090, False, False)),
            G("GRAB_NORTH_MINE_STAND")
                .weight(2)
                .coords(0.42, 0.30)
                .direction(DIRECTION_FORWARD)
                .state(GrabStand, (SIDE_LEFT, 0.200, 0.090, 0.04, False, False))
                .build(),
            G("GRAB_STAIRS_STAND")
                .weight(2)
                .coords(0.4, 0.77)
                .direction(DIRECTION_FORWARD)
                .state(GrabNorthMineStands)
                .build(),
            #StandGoal("GRAB_PLATFORM_1_STAND", 3, SIDE_LEFT, 1.355, 0.870, GoalGrabStand),
            SG("GRAB_PLATFORM_1_STAND")
                .weight(3)
                .side(SIDE_LEFT)
                .coords(1.355, 0.870)
                .state(GoalGrabStand, (0.04,))
                .build(),
            #goalmanager.Goal("GRAB_PLATFORM_2_STAND", 4, 1.600, 0.900, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 1.770, 1.100, False, False)),,
            G("GRAB_PLATFORM_2_STAND")
                .weight(4)
                .coords(1.600, 0.900)
                .direction(DIRECTION_FORWARD)
                .state(GrabStand, (SIDE_LEFT, 1.770, 1.100, 0.04, False, False))
                .build(),
            #goalmanager.StandGoal("GRAB_PLATFORM_3_STAND", 5, SIDE_LEFT, 1.400, 1.300, GoalGrabStand),
            SG("GRAB_PLATFORM_3_STAND")
                .weight(5)
                .side(SIDE_LEFT)
                .coords(1.400, 1.300)
                .state(GoalGrabStand, (0.04,))
                .build(),
            #goalmanager.Goal("GRAB_SOUTH_MINE_STANDS", 6, 1.45, 0.22, 0, DIRECTION_FORWARD, GrabSouthMineStands),
            G("GRAB_SOUTH_MINE_STANDS")
                .weight(6)
                .coords(1.45, 0.22)
                .direction(DIRECTION_FORWARD)
                .state(GrabSouthMineStands)
                .build(),
            #goalmanager.Goal("KICK_MINE_CLAPS", 7, 1.77, 0.22, 0, DIRECTION_FORWARD, KickMineClaps),
            G("KICK_MINE_CLAPS")
                .weight(7)
                .coords(1.77, 0.22)
                .direction(DIRECTION_FORWARD)
                .state(KickMineClaps)
                .build(),
            #goalmanager.Goal("DEPOSIT_SPOTLIGHT_HOME", 10, 1.0, 0.58, 0, DIRECTION_FORWARD, BuildSpotlightHome),
            G("DEPOSIT_SPOTLIGHT_HOME")
                .weight(10)
                .coords(1.0, 0.58)
                .direction(DIRECTION_FORWARD)
                .state(BuildSpotlightHome)
                .build(),
            #goalmanager.Goal("KICK_THEIR_CLAP", 9, 1.77, 2.62, 0, DIRECTION_FORWARD, KickTheirClap),
            G("KICK_THEIR_CLAP")
                .weight(9)
                .coords(1.77, 2.62)
                .direction(DIRECTION_FORWARD)
                .state(KickTheirClap)
                .build(),
            #goalmanager.Goal("DEPOSIT_SPOTLIGHT_PLATFORM", 8, 1.69, 1.3, 0, DIRECTION_FORWARD, BuildSpotlightPlatform),
            G("DEPOSIT_SPOTLIGHT_PLATFORM")
                .weight(8)
                .coords(1.64, 1.3)
                .direction(DIRECTION_FORWARD)
                .state(BuildSpotlightPlatform)
                .build()


        )

    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield AntiBlocking(True)
            yield GetInputStatus(MAIN_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(89500, EndOfMatch())


    def on_bulb_grabbed(self, packet):
        logger.log("Starting ...")
        yield SafeMoveLineTo(LEFT_START_X, 0.53)
        yield StaticStrategy()
        # yield ExecuteGoals()
        yield ExecuteGoalsV2()




class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_CLOSE)
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        yield DefinePosition(1.0, 0.07 + ROBOT_CENTER_Y, math.pi / 2.0)
        yield None




class WaitForStandStored(Timer):

    def __init__(self, side):
        super().__init__(3000)
        self.side = side


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
        try:
            # GRAB_NORTH_STAIRS_STANDS
            yield GrabStand(SIDE_RIGHT, 0.200, 0.850, 0.04, True)
            yield GrabStand(SIDE_RIGHT, 0.100, 0.850, 0.04, False)
            self.robot.goal_manager.update_goal_status("GRAB_STAIRS_STAND", GOAL_DONE)
            yield LookAtOpposite(0.42, 0.73)
            yield SafeMoveLineTo(0.42, 0.73)

            # GRAB_NORTH_MINE_STAND
            yield RotateTo(-math.pi / 2.0)
            yield SafeMoveLineTo(0.42, 0.30)
            yield from self.grab_stand("GRAB_NORTH_MINE_STAND", SIDE_LEFT, 0.200, 0.090, 0.04, False)
            x, y = get_crossing_point(self.robot.pose.virt.x, self.robot.pose.virt.y, self.robot.pose.virt.angle, 0.5, 1.5, math.pi / 2.0)
            yield SafeMoveLineTo(x, y)

            # GRAB_PLATFORM_1_STAND
            yield LookAt(0.5, 0.62)
            yield SafeMoveLineTo(0.5, 0.62)
            yield from self.grab_stand("GRAB_PLATFORM_1_STAND", SIDE_LEFT, 1.355, 0.870, 0.01, False)

            # GRAB_PLATFORM_2_STAND
            yield from self.grab_stand("GRAB_PLATFORM_2_STAND", SIDE_LEFT, 1.770, 1.100, 0.01, False)

            # GRAB_PLATFORM_3_STAND
            yield from self.grab_stand("GRAB_PLATFORM_3_STAND", SIDE_LEFT, 1.400, 1.300, 0.04, False)

            # GRAB_SOUTH_MINE_STANDS
            x, y = 1.45, 0.22
            yield LookAt(x, y)
            yield SafeMoveLineTo(x, y)
            yield from self.grab_stand(None, SIDE_RIGHT, 1.750, 0.090, 0.03, True)
            yield from self.grab_stand("GRAB_SOUTH_MINE_STANDS", SIDE_RIGHT, 1.850, 0.090, 0.04, True)
            yield ResettleAfterSouthMineStands()
            yield MoveLineTo(1.77, 0.22)
            kick = yield KickMineClaps()
            if kick.exit_reason == GOAL_DONE:
                self.robot.goal_manager.update_goal_status("KICK_MINE_CLAPS", GOAL_DONE)

        except OpponentInTheWay:
            pass
        yield None


    def grab_stand(self, goal_name, side, x, y, stand_grab_offset, store_stand):
        grab = yield GrabStand(side, x, y, stand_grab_offset, store_stand)
        if goal_name is not None:
            if grab.exit_reason == GOAL_DONE:
                self.robot.goal_manager.update_goal_status(goal_name, GOAL_DONE)
            else:
                yield None




class GrabStand(State):

    def __init__(self, side, x, y, stand_grab_offset, store_stand, raise_on_error = True):
        super().__init__()
        self.side = side
        self.x = x
        self.y = y
        self.stand_grab_offset = stand_grab_offset
        self.store_stand = store_stand
        self.raise_on_error = raise_on_error


    def on_enter(self):
        x, y, angle = builder_at_point(self.side, self.robot.pose, self.x, self.y)
        ref_pose = Pose(self.robot.pose.x, self.robot.pose.y, angle, True)
        yield RotateTo(angle)
        x, y = get_offset_position(ref_pose, x, y, self.stand_grab_offset)
        self.exit_reason = GOAL_FAILED
        try:
            move = yield SafeMoveLineTo(x, y)
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




class GoalGrabStand(GrabStand):

    def __init__(self, stand_grab_offset):
        super().__init__(SIDE_LEFT, 0, 0, stand_grab_offset, False, False)


    def on_enter(self):
        goal = self.robot.goal_manager.get_current_goal()
        self.x = goal.x
        self.y = goal.y
        yield from super().on_enter()




class GrabNorthMineStands(State):

    def on_enter(self):

        grab = yield GrabStand(SIDE_RIGHT, 0.200, 0.850, 0.04, True)
        if grab.exit_reason == GOAL_DONE:
            yield GrabStand(SIDE_RIGHT, 0.100, 0.850, 0.04, False)

        self.exit_reason = grab.exit_reason
        yield None




class GrabSouthMineStands(State):

    def on_enter(self):
        grab = yield GrabStand(SIDE_RIGHT, 1.75, 0.09, 0.03, True)
        if grab.exit_reason == GOAL_DONE:
            grab = yield GrabStand(SIDE_RIGHT, 1.85, 0.09, 0.04, False)
        self.exit_reason = grab.exit_reason
        yield ResettleAfterSouthMineStands()

        yield None




class ResettleAfterSouthMineStands(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_goals("KICK_MINE_CLAPS")[0]
        yield MoveCurve(0.0, 0.10, [(1.4, goal.y), (1.32, goal.y)])
        yield DefinePosition(1.222 + ROBOT_CENTER_X, None, 0.0)
        yield None




class KickMineClaps(State):

    def on_enter(self):
        goal = self.robot.goal_manager.get_goals("KICK_MINE_CLAPS")[0]
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




class BuildSpotlightPlatform(State):

    def on_enter(self):
        yield RotateTo(0.0)
        self.fsm.builders[SIDE_LEFT].enabled = False
        self.send_packet(packets.BuildSpotlight(SIDE_LEFT))


    def on_build_spotlight(self, packet):
        if packet.side == SIDE_LEFT:
            yield MoveLineRelative(-0.15)
            self.fsm.builders[SIDE_LEFT].enabled = True
            self.send_packet(packets.StandbuilderIdle(SIDE_LEFT))
            self.exit_reason = GOAL_DONE
            yield None




class BuildSpotlightHome(State):

    def on_enter(self):
        yield RotateTo(-math.pi / 2)
        yield MoveLineTo(1.0, 0.5)
        self.fsm.builders[SIDE_RIGHT].enabled = False
        self.send_packet(packets.BuildSpotlight(SIDE_RIGHT))


    def on_build_spotlight(self, packet):
        if packet.side == SIDE_RIGHT:
            yield MoveLineRelative(-0.15)
            self.fsm.builders[SIDE_RIGHT].enabled = True
            self.send_packet(packets.StandbuilderIdle(SIDE_RIGHT))
            self.exit_reason = GOAL_DONE
            yield None




##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
