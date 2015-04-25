# encoding: utf-8

import pytest
import run_tests
import unittest
from unittest.mock import MagicMock, patch

run_tests.patch_tests(True)

from definitions import *

import goalmanager
from goalmanager import Goal, StandGoal
import goaldecider



########################################################################
# Packet test infrastructure


class GrabStand(object): pass

class GrabSouthMineStands(object): pass

class KickMineClaps(object): pass

class KickTheirClap(object): pass

class GoalGrabStand(object): pass

class ScanAndBuildSpotlight(object): pass

@pytest.fixture
def robot():
    import robot
    import eventloop

    patcher1 = patch("eventloop.EventLoop")
    ev = patcher1.start()
    ev.get_remaining_match_time.return_value = 25

    robotobj = robot.Robot(ev)
    return robotobj

@pytest.fixture
def logs():
    import logger
    logger.initialize()

def test_basic(logs,robot):
    STAND_GRAB_OFFSET = 0.01
    STAND_GOAL_OFFSET = -0.25

    STAND_PRESENCE_SENSOR_OFFSET = 0.0

    G=goalmanager.GoalBuilder

    gm = robot.goal_manager

    gm.add(
            # Goal("GRAB_NORTH_MINE_STAND", 2, 0.42, 0.30, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 0.200, 0.090, False)),
            #identifier, weight, x, y, offset, direction, handler_state, ctor_parameters = None, shared = False, navigate = True
            G("GRAB_NORTH_MINE_STAND")
                .weight(2)
                .coords(0.42, 0.30)
                .offset(0)
                .direction(DIRECTION_FORWARD)
                .state(GrabStand, (SIDE_LEFT, 0.200, 0.090, False))
                .build(),
            goalmanager.StandGoal("GRAB_PLATFORM_1_STAND", 3, SIDE_LEFT, 1.355, 0.870, GoalGrabStand),
            goalmanager.Goal("GRAB_PLATFORM_2_STAND", 4, 1.600, 0.900, 0, DIRECTION_FORWARD, GrabStand, (SIDE_LEFT, 1.770, 1.100, False)),
            goalmanager.StandGoal("GRAB_PLATFORM_3_STAND", 5, SIDE_LEFT, 1.400, 1.300, GoalGrabStand),
            goalmanager.Goal("GRAB_SOUTH_MINE_STANDS", 6, 1.45, 0.22, 0, DIRECTION_FORWARD, GrabSouthMineStands),
            goalmanager.Goal("KICK_MINE_CLAPS", 7, 1.77, 0.22, 0, DIRECTION_FORWARD, KickMineClaps),
            goalmanager.Goal("SCAN_AND_BUILD_SPOTLIGHT", 8, 1.67, 1.2 - STAND_PRESENCE_SENSOR_OFFSET, 0, DIRECTION_FORWARD, ScanAndBuildSpotlight),
            goalmanager.Goal("KICK_THEIR_CLAP", 9, 1.77, 2.62, 0, DIRECTION_FORWARD, KickTheirClap),
        )

    goaldecider.get_best_goal(gm.goals, map_=None, robot=robot, max_duration=30, max_depth=3)

