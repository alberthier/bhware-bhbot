# encoding: utf-8

import datetime
import itertools
import random
import sys

import logger
import packets
import position
import statemachine
import tools

from definitions import *

import greplin
import greplin.scales as scales

class GoalStats(object):
    real_duration = scales.StateTimeStat('real_duration')

    def __init__(self,id):
        scales.init(self, '/goals/'+id)

class Goal:
    __slots__ = ["identifier",
                 "weight",
                 "x",
                 "y",
                 "offset",
                 "direction",
                 "handler_state",
                 "navigation_cost",
                 "ctor_parameters",
                 "score",
                 "penality",
                 "status", "shared", "navigate", "trial_count","last_try",
                 "goal_manager", "is_current", "is_blacklisted", "uid", "estimated_duration",
                 "builder_action",
                 "ratio_decc", "cached_pose",
                 "stats"
                 ]

    def __init__(self, identifier, weight, x, y, offset, direction, handler_state, ctor_parameters = None, shared = False, navigate = True):
        self.identifier = identifier
        self.weight = weight
        self.x = x
        self.y = y
        self.offset = offset
        self.direction = direction
        self.handler_state = handler_state
        self.navigation_cost = 0.0
        self.ctor_parameters = ctor_parameters
        self.score = 0.0
        self.penality = 0.0
        self.status = GOAL_AVAILABLE
        self.shared = shared
        self.navigate = navigate
        self.trial_count = 0
        self.last_try = None
        self.goal_manager = None
        self.is_current = False
        self.is_blacklisted = False
        self.uid = self.identifier
        self.ratio_decc = None
        self.cached_pose = None
        self.estimated_duration = None
        self.builder_action = None
        self.stats=GoalStats(identifier)



    def clone(self):
        n = Goal(self.identifier, self.weight, self.x, self.y, self.offset, self.direction,
                 self.handler_state, self.ctor_parameters, self.shared, self.navigate)
        n.score = self.score
        n.penality = self.penality
        n.status = self.status
        n.trial_count = self.trial_count
        if self.last_try:
            l=self.last_try
            n.last_try = datetime.datetime(l.year,l.month,l.day,l.hour,l.minute,l.second,l.microsecond)
        else:
            n.last_try = None
        n.goal_manager = self.goal_manager
        n.is_current = self.is_current
        n.is_blacklisted = self.is_blacklisted
        n.uid = self.uid
        n.estimated_duration = self.estimated_duration
        n.builder_action = self.builder_action
        n.ratio_decc = self.ratio_decc
        return n

    @property
    def pose(self):
        if not self.cached_pose:
            self.cached_pose = position.Pose(self.x, self.y)
        return self.cached_pose


    def increment_trials(self):
        self.trial_count += 1
        logger.log('Goal {} : increment trials : {}'.format(self.identifier, self.trial_count))


    def get_state(self):
        if isinstance(self.handler_state, statemachine.State):
            return self.handler_state
        else :
            if self.ctor_parameters is not None:
                try :
                    logger.log('Next state : {}{}'.format(self.handler_state.__name__, self.ctor_parameters))
                    return self.handler_state(*self.ctor_parameters)
                except Exception as e :
                    logger.dbg("Exception while calling constructor for {} with parameters".format(self.handler_state, self.ctor_parameters))
                    logger.log_exception(e)
                    raise
            else:
                try :
                    logger.log('Next state : {}()'.format(self.handler_state.__name__))
                    return self.handler_state()
                except Exception as e :
                    logger.dbg("Exception while calling constructor for {}".format(self.handler_state))
                    logger.log_exception(e)
                    raise


    def available(self):
        self.is_current = False
        self.goal_manager.update_goal_status(self, GOAL_AVAILABLE)


    def doing(self):
        self.is_current = True
        self.last_try = datetime.datetime.now()
        self.goal_manager.update_goal_status(self, GOAL_DOING)


    def done(self):
        self.is_current = False
        self.goal_manager.update_goal_status(self, GOAL_DONE)


    def is_available(self):
        return self.status == GOAL_AVAILABLE


    def before_evaluation(self):
        pass




class GoalManager:

    GOAL_EVALUATION_USES_PATHFINDING = True

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.goals = []
        self.last_goal = None
        self.goal_ids = set()
        self.score_estimator = None

    @property
    def available_goals(self):
        return [ g for g in self.goals if g.is_available() ]

    @property
    def remaining_goals(self):
        return [ g for g in self.goals if g.status != GOAL_DONE ]

    @property
    def doable_goals(self):
        return [ g for g in self.goals if g.is_available() and not g.is_blacklisted ]

    def add(self, *args):
        for goal in args :
            goal.goal_manager = self
            added = False
            add_int = 1
            while not added :
                add_int+=1
                if goal.uid in self.goal_ids :
                    goal.uid = "{}_{}".format(goal.identifier,add_int)
                else :
                    self.goals.append(goal)
                    self.goal_ids.add(goal.uid)
                    added = True



    def has_available_goal(self, identifier):
        return any((g.is_available() for g in self.all_goals if g.identifier == identifier))


    def get_candidate_goals(self):
        return [ goal for goal in self.goals if goal.is_available() and not goal.is_blacklisted ]


    def whitelist_all(self):
        for goal in self.goals:
            goal.is_blacklisted = False


    def has_blacklisted_goals(self):
        for goal in self.goals:
            if goal.is_available() and goal.is_blacklisted:
                return True
        return False


    def get_simple_next_goal(self):
        candidates = self.get_candidate_goals()

        if len(candidates) == 0:
            return None

        for goal in candidates :
            goal.before_evaluation()

        candidates.sort(key = lambda goal : goal.weight)

        logger.log('Candidate goals : {}'.format(["{}({})".format(goal.uid, goal.weight) for goal in candidates]))

        best_weight = candidates[0].weight
        best_candidates = []
        for goal in candidates:
            if goal.weight == best_weight:
                best_candidates.append(goal)
            else:
                break

        if len(best_candidates) == 1:
            return best_candidates[0]
        else:
            best_goal = None
            for goal in best_candidates:
                pose = position.Pose(goal.x, goal.y, virtual = True)
                logger.log("Evaluate goal {}".format(goal.uid))
                if GoalManager.GOAL_EVALUATION_USES_PATHFINDING:
                    goal.navigation_cost = self.event_loop.map.evaluate(self.event_loop.robot.pose, pose)
                else:
                    goal.navigation_cost = tools.distance(self.event_loop.robot.pose.x, self.event_loop.robot.pose.y, pose.x, pose.y)
                if best_goal is None or best_goal.navigation_cost > goal.navigation_cost:
                    best_goal = goal
            return best_goal


    def get_next_goal(self):
        candidates = self.get_candidate_goals()

        for goal in candidates :
            goal.before_evaluation()

        for goal in candidates:
            pose = position.Pose(goal.x, goal.y, virtual = True)
            logger.log("Evaluate goal {}".format(goal.uid))
            if GoalManager.GOAL_EVALUATION_USES_PATHFINDING:
                goal.navigation_cost = self.event_loop.map.evaluate(self.event_loop.robot.pose, pose)
            else:
                goal.navigation_cost = tools.distance(self.event_loop.robot.pose.x, self.event_loop.robot.pose.y, pose.x, pose.y)
        # Remove unreachable goals
        candidates = [ goal for goal in candidates if goal.navigation_cost is not None ]

        if len(candidates) == 0:
            return None
        if len(candidates) == 1:
            return candidates[0]

        candidates.sort(key = lambda goal : goal.navigation_cost)
        nearest_cost = candidates[0].navigation_cost
        farest_cost = candidates[-1].navigation_cost
        total_range = farest_cost - nearest_cost

        logger.log("Nearest cost: {}    Farest cost: {}    Range: {}".format(nearest_cost, farest_cost, total_range))

        for goal in candidates:
            current = goal.navigation_cost - nearest_cost
            k = (total_range - current) / total_range
            goal.score = goal.weight * (1.0 / 3.0 + k * 2.0 / 3.0)
            logger.log("Goal '{}'     Navigation cost: {}    Score: {}".format(goal.identifier, goal.navigation_cost, goal.score))

        return max(candidates, key = lambda goal : goal.score)



    def get_least_recent_tried_goal(self):


        never_tried = [ g for g in self.available_goals if g.last_try is None ]

        if len(never_tried) > 0:
            goals = never_tried
        else :
            oldest = datetime.datetime.now()
            goals = [ None ]
            for g in self.available_goals:
                if g.last_try is not None and g.last_try < oldest:
                    oldest = g.last_try
                    goals[0] = g

        return random.choice(goals)


    def get_best_goal(self):
        """
        :type goals:  list of Goal
        """

        available_goals = [ g for g in self.goals if g.is_available() ]

        if self.last_goal in available_goals:
            available_goals.remove(self.last_goal)

        logger.log('available goals : {}'.format([g.identifier for g in available_goals]))

        if not available_goals :
            return None

        for goal in available_goals:
            pose = position.Pose(goal.x, goal.y, virtual=True)
            logger.log("Evaluate goal {}".format(goal.identifier))
            if GoalManager.GOAL_EVALUATION_USES_PATHFINDING:
                goal.navigation_cost = self.event_loop.map.evaluate(self.event_loop.robot.pose, pose)
            else:
                goal.navigation_cost = tools.distance(self.event_loop.robot.pose.x, self.event_loop.robot.pose.y, pose.x, pose.y)
            if goal.navigation_cost is None:
                goal.navigation_cost = 999999.0
            goal.score = goal.penality
            goal.penality = 0.0

        logger.log("Scoring distance")
        for order, goal in enumerate(sorted(available_goals, key = lambda x : x.navigation_cost, reverse = True)):
            goal.score += (order + 1) * 2
            logger.log("Goal {} nav cost = {}, score = {}".format(goal.identifier, goal.navigation_cost, goal.score))

        logger.log("Adding weights")
        for goal in available_goals:
            goal.score += goal.weight
            logger.log("Goal {} score = {}".format(goal.identifier, goal.score))

        logger.log("Scoring tentatives")
        order = 0
        last_value = None
        for goal in sorted(available_goals, key = lambda x : x.trial_count, reverse = True):
            if last_value != goal.trial_count:
                last_value = goal.trial_count
                order += 1
            goal.score += order
            logger.log("Goal {} score = {}".format(goal.identifier, goal.score))

        logger.log("available_goals by score : {}".format(["{}:{}".format(g.identifier, g.score) for g in available_goals ] ))

        best_goal = max(available_goals, key = lambda g : g.score)

        logger.log("Best goal is {} with score {}".format(best_goal.identifier, best_goal.score))

        self.last_goal = best_goal

        return best_goal


    def get_current_goal(self):
        for g in self.goals:
            if g.is_current:
                return g
        return None


    def penalize_goal(self, goal):
        for g in self.harvesting_goals + self.emptying_goals:
            if g.identifier == goal.identifier:
                g.penality = 100.0


    def get_goals(self, identifier):
        goals = []
        for goal in self.goals:
            if goal.identifier == identifier:
                goals.append(goal)
        return goals

    def is_done(self, goal_id):
        for goal in self.goals :
            if goal.identifier == goal_id and goal.status == GOAL_DONE :
                return True
        return False


    def update_goal_status(self, goal, new_status):
        if not isinstance(goal, Goal):
            for g in self.goals:
                if g.identifier == goal:
                    goal = g
                    break
        logger.log("Goal {} : {}".format(GOAL_STATUS.lookup_by_value[new_status], goal.identifier))

        self.internal_goal_update(goal.identifier, new_status)

        if goal.shared :
            logger.log('A shared goal status changed, notifying my buddy : {} -> {}'.format(goal.identifier, goal.status))
            packet = packets.InterbotGoalStatus(goal_identifier = goal.identifier, goal_status = goal.status)
            self.event_loop.send_packet(packet)


    def internal_goal_update(self, identifier, status):
        for g in self.goals:
            if g.identifier == identifier:
                old = g.status
                g.status = status
                logger.log('updated goal {} status : {} -> {}'.format(identifier,
                                                                      GOAL_STATUS.lookup_by_value[old],
                                                                      GOAL_STATUS.lookup_by_value[status])
                           )


    def on_interbot_goal_status(self, packet):
        logger.log('Got goal status : {} = {}'.format(packet.goal_identifier,GOAL_STATUS.lookup_by_value[packet.goal_status]))
        self.internal_goal_update(packet.goal_identifier, packet.goal_status)

import inspect

class GoalBuilder:
    def __init__(self, goal_id, ctor=Goal):
        self.goal_id = goal_id
        self.ctor = ctor
        self._estimated_time = None
        self._disabled = False
        self.params = {"identifier":goal_id}
        self.default_values={"offset": 0}
        self._builder_action=None

    @tools.newobj
    def identifier(self, identifier):
        self.params["identifier"]=identifier

    @tools.newobj
    def weight(self, weight):
        self.params["weight"]=weight

    @tools.newobj
    def coords(self, x, y):
        self.params["x"]=x
        self.params["y"]=y

    @tools.newobj
    def offset(self, offset):
        self.params["offset"]=offset

    @tools.newobj
    def direction(self, direction):
        self.params["direction"]=direction

    @tools.newobj
    def side(self, side):
        self.params["side"]=side

    @tools.newobj
    def builder_action(self, left,right):
        self._builder_action=left,right

    @tools.newobj
    def state(self, handler_state, ctor_parameters=None):
        self.params["handler_state"]=handler_state
        if ctor_parameters:
            self.params["ctor_parameters"]=ctor_parameters

    @tools.newobj
    def estimated_duration(self, estimated_duration):
        self._estimated_duration = estimated_time

    @tools.newobj
    def disabled(self):
        self._disabled = True

    def build(self):
        logger.log("Building goal {}".format(self.goal_id))

        #inspecting params
        ctor_params=inspect.getargspec(self.ctor)
        passed_params={}
        passed_params.update(self.params)

        for p in ctor_params[0]:
            if p not in passed_params.keys() and p in self.default_values:
                passed_params[p]=self.default_values[p]

        logger.log("Calling {} with {}".format(self.ctor, passed_params))
        g = self.ctor(**passed_params)
        if self._estimated_time:
            g.estimated_duration = self._estimated_duration
            logger.log("estimated_duration={}".format(g.estimated_duration))

        if self._disabled:
            g.status = GOAL_DISABLED
            logger.log("status={}".format(g.status))

        if self._builder_action:
            g.builder_action = self._builder_action
        return g

def on_end_of_match(gm: GoalManager, robot):
    logger.log("End of match statistics")
    logger.log("Remaining goals: {}".format([g.identifier for g in gm.remaining_goals]))
    logger.log("Our score is: {}".format(robot.score))
