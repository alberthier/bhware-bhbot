# encoding: utf-8

import math

import goalmanager
import position
import goaldecider
from definitions import *




class Robot(object):

    def __init__(self, event_loop):
        self.pose = position.Pose(0.0, 0.0, 0.0)
        self._team = TEAM_UNKNOWN
        self.event_loop = event_loop
        self.destination = None
        self.main_opponent_direction = None
        self.secondary_opponent_direction = None
        self.goal_manager = goalmanager.GoalManager(event_loop)
        self.goal_decider = goaldecider.GoalDecider(event_loop, self.goal_manager)
        self.locks = set()
        self.used_storage_spaces = []
        self.score=0
        self.is_holding_module = True


    def is_looking_at(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        angle = math.atan2(dy, dx) % (2.0 * math.pi)
        current_angle = self.pose.angle % (2.0 * math.pi)
        return abs(current_angle - angle) < (math.pi / 32.0)


    def is_looking_at_opposite(self, pose):
        dx = pose.x - self.pose.x
        dy = pose.y - self.pose.y
        angle = (math.atan2(dy, dx) + math.pi) % (2.0 * math.pi)
        current_angle = self.pose.angle % (2.0 * math.pi)
        return abs(current_angle - angle) < (math.pi / 32.0)


    def on_team_read(self, packet):
        self.team = packet.value


    def on_goto_finished(self, packet):
        self.pose = packet.current_pose


    def on_keep_alive(self, packet):
        self.pose = packet.current_pose


    def on_packet(self, packet):
        packet.dispatch(self.goal_manager)


    def set_team(self, team):
        position.Pose.match_team = team
        self._team = team

    def get_team(self):
        return self._team

    team = property(get_team, set_team)


    def lock(self, name):
        self.locks.add(name)


    def unlock(self, name):
        self.locks.discard(name)


    def is_locked(self, name):
        return name in self.locks
