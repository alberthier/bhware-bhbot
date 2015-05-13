import copy
import datetime
import goalmanager
import random
import position
# import logger
import robot
import goalmanager


from definitions import *

MEAN_SPEED_PER_S=0.1
DEFAULT_GOAL_ESTIMATED_DURATION_S=15
MAX_STAND_PER_BUILDER=4

class GoalDeciderException(Exception):
    def __init__(self, goal, message=None):
        if not message:
            super(GoalDeciderException, self).__init__("{}: {}".format(self.__class__.__name__, goal.identifier))
        else:
            super(GoalDeciderException, self).__init__("{}: {}: {}".format(self.__class__.__name__, goal.identifier, message))


class NoTimeToAttain(GoalDeciderException):
    pass


class NoTimeToExecute(GoalDeciderException):
    pass


class GoalImpossibleToExecute(GoalDeciderException):
    pass

class UnneededAction(GoalDeciderException):
    pass


def goals_to_str(goals):
    return "->".join((g.uid for g in goals)) if goals else None

def goals_to_list(goals):
    return [g.uid for g in goals] if goals else None

class WorldState:
    def __init__(self, goals, map_, remaining_time):
        """

        :type goals: list(brewery.goalmanager.Goal)
        :type remaining_time: float
        """
        import logger
        self.score = 0
        self.right_builder_count = 0
        self.left_builder_count = 0
        self.remaining_goals = goals
        self.executed_goals = []
        ":type executed_goals: list(brewery.goalmanager.Goal)"
        self.depth = 0
        self.remaining_time = remaining_time
        self.map_ = map_
        self.robot_pose = position.Pose(0,0,0)
        self.logger=logger
        self.traveled_distance = 0
        self.has_right_bulb = False
        self.has_left_bulb = False
        self._remaining_deposit_goals = None

    def clone(self):
        """

        :rtype : WorldState
        """
        new_inst = WorldState([g.clone() for g in self.remaining_goals], self.map_, self.remaining_time)
        new_inst.score = self.score
        new_inst.right_builder_count = self.right_builder_count
        new_inst.left_builder_count = self.left_builder_count
        new_inst.executed_goals = [g.clone() for g in self.executed_goals]
        new_inst.depth = self.depth
        new_inst.remaining_time = self.remaining_time
        new_inst.robot_pose = self.robot_pose.clone()
        new_inst.traveled_distance = self.traveled_distance
        new_inst.has_left_bulb = self.has_left_bulb
        new_inst.has_right_bulb = self.has_right_bulb
        new_inst._remaining_deposit_goals = self._remaining_deposit_goals
        return new_inst


    @property
    def remaining_deposit_goals(self):
        if self._remaining_deposit_goals is None:
            self._remaining_deposit_goals = len([ g for g in self.remaining_goals if "BUILD" in g.tags ])
        return self._remaining_deposit_goals

    def decrement_deposit_goals(self):
        if self._remaining_deposit_goals:
            self._remaining_deposit_goals-=1

    @property
    def last_goal(self) -> goalmanager.Goal:
        return self.executed_goals[-1] if self.executed_goals else None


    def goal_trace(self):
        return goals_to_str(self.executed_goals)


    def execute_goal(self, goal: goalmanager.Goal):
        self.executed_goals.append(goal)
        self.remaining_goals=[g for g in self.remaining_goals if g.identifier != goal.identifier]

        self.logger.trace("Executing goal : {}".format(self.goal_trace()))

        if goal.tags.issuperset({"BUILD", "SPOTLIGHT"}):
            self.decrement_deposit_goals()

        if goal.tags.issuperset({"GRAB", "STAND"}):
            if self.remaining_deposit_goals <=0:
                raise UnneededAction(goal, "Abort: Impossible to deposit these stand(s)")

        distance = goal.pose - self.robot_pose
        elapsed_time = distance / MEAN_SPEED_PER_S
        self.traveled_distance+=distance

        if self.remaining_time - elapsed_time <= 0.0 :
            raise NoTimeToAttain(goal)

        self.remaining_time-=elapsed_time

        if self.remaining_time - (goal.estimated_duration or DEFAULT_GOAL_ESTIMATED_DURATION_S) <= 0.0 :
            raise NoTimeToExecute(goal)

        self.robot_pose = goal.pose.clone()

        if goal.tags.issuperset({"GRAB", "STAND"}):
            if goal.builder_action[0]>0:
                if self.left_builder_count+goal.builder_action[0]> MAX_STAND_PER_BUILDER:
                    raise GoalImpossibleToExecute(goal, "No space left in left builder")
                self.left_builder_count+=goal.builder_action[0]
            elif goal.builder_action[1]>0:
                if self.right_builder_count+goal.builder_action[1]> MAX_STAND_PER_BUILDER:
                    raise GoalImpossibleToExecute(goal, "No space left in right builder")
                self.right_builder_count+=goal.builder_action[1]

        self.logger.log("After execute goal {}: {}".format(goal.identifier, self.to_str()))

    def after_goal(self, goal):
        if "BUILD" in goal.tags :
            if goal.builder_action[0]<0:
                self.left_builder_count = 0
                self.has_left_bulb=False
            else:
                self.right_builder_count = 0
                self.has_right_bulb=False

    def to_str(self):
        return "Builders: {},{} Bulbs: {}, {} Time: {} Traveled: {}".format(
            self.left_builder_count,
            self.right_builder_count,
            self.has_left_bulb,
            self.has_right_bulb,
            self.remaining_time,
            self.traveled_distance
        )


def compare_world_by_score_and_dist(w1 : WorldState, w2 : WorldState):
    if w1.score > w2.score:
        return 1
    elif w1.score < w2.score:
        return -1
    else:
        return cmp(w1.traveled_distance, w2.traveled_distance)

MAX_DIST=3.0*2.0*3

def world_to_key_distance_score(w : WorldState):
    return w.score * 1000 + (MAX_DIST - w.traveled_distance)

def world_to_key_distance_score_decay(w : WorldState):
    alteration = max(1 - (( w.traveled_distance / 0.75 ) * 0.15), 0.0)

    return alteration * w.score * 1000 + (MAX_DIST - w.traveled_distance)

def world_to_key_distance_decay(w : WorldState):
    alteration = max(1 - (( w.traveled_distance / 0.75 ) * 0.15), 0.0)

    return alteration * w.score * 1000 + (MAX_DIST - w.traveled_distance)

world_comparison_func = world_to_key_distance_score

class Explorer:

    def __init__(self):
        import logger
        self.max_result=100
        self.max_duration=5
        self.start_time = datetime.datetime.now()
        self.logger=logger
        self.evaluated_count=0

    def set_max_result(self, param):
        self.max_result = param

    def set_max_duration(self, max_duration):
        self.max_duration = max_duration

    def set_max_depth(self, max_depth):
        self.max_depth = max_depth

    def explore(self, world):

        self.logger.log("Finding best goal combination. Max depth: {} Max duration: {}"
                          .format(self.max_depth, self.max_duration))

        self.logger.log("Considered goals are: {}".format([g.identifier for g in world.remaining_goals]))

        best = self.explore_recursive(world)

        if best:
            self.logger.log("Found best world: {}".format(best.goal_trace()))
            self.logger.log(best.to_str())
            self.logger.log("Score: {} Remaining time: {}".format(best.score, best.remaining_time))
            self.logger.log("Remaining goals: {}".format(len(best.remaining_goals)))
            self.logger.log(best.goal_trace())
            return best.executed_goals, best

    def set_world(self, w):
        self.world = w

    # breadth-first exploration
    def explore_recursive(self, world: WorldState):
        new_worlds=[]

        for goal in sorted(world.remaining_goals, key=lambda g: g.weight) :
            try:
                if self.time_expired():
                    self.log("Time has expired")
                    return self.best(new_worlds)

                self.evaluated_count+=1

                new_world = world.clone()
                new_world.depth+=1

                new_world.execute_goal(goal)

                self.score_world(new_world)

                new_world.after_goal(goal)

                self.logger.trace("Score: {} Remaining time: {} Remaining goals: {} Traveled distance: {}".format(new_world.score, new_world.remaining_time, len(new_world.remaining_goals), new_world.traveled_distance))

                new_worlds.append(new_world)
            except GoalDeciderException as e:
                self.log(str(e))

        if world.depth == self.max_depth:
            return self.best(new_worlds)

        allret=[]

        new_worlds.sort(key=world_comparison_func, reverse=True)

        for world in new_worlds :
            ret=self.explore_recursive(world)
            if not ret :
                continue
            allret.append(ret)

        return self.best(allret) if allret else world

    def time_expired(self):
        return (datetime.datetime.now() - self.start_time).total_seconds() >= self.max_duration

    def log(self, param):
        print(param)

    def score_world(self, new_world: WorldState):
        last = new_world.last_goal
        action_score = 0

        if last:
            if "BUILD" in last.tags :
                if last.builder_action[0]<0 and new_world.left_builder_count:
                    if new_world.has_left_bulb:
                        action_score+=new_world.left_builder_count*3
                    else:
                        action_score+=new_world.left_builder_count*2

                if last.builder_action[1]<0 and new_world.right_builder_count:
                    if new_world.has_right_bulb:
                        action_score+=new_world.right_builder_count*3
                    else:
                        action_score+=new_world.right_builder_count*2

                if not new_world.left_builder_count and not new_world.right_builder_count:
                    raise UnneededAction(last, "No stand to deposit")


            elif "KICK" in last.tags :
                action_score+=5

        if action_score>0:
            self.logger.log("This action scored: {}".format(action_score))
            new_world.score += action_score

    def best(self, worlds):
        if worlds :
            # self.logger.log("Choosing best of {} worlds".format(len(worlds)))
            if len(worlds) > 1:
                worlds.sort(key=world_comparison_func, reverse=True)
            return worlds[0]

    def evaluate(self, base_world: WorldState, last_solution: WorldState):

        world = base_world.clone()

        for goal in last_solution.executed_goals:
            try:
                if self.time_expired():
                    self.log("Time has expired")
                    break

                if not goal.is_available():
                    continue

                self.evaluated_count+=1

                new_world = world.clone()
                new_world.depth+=1

                new_world.execute_goal(goal)

                self.score_world(new_world)

                new_world.after_goal(goal)

                self.logger.trace("Score: {} Remaining time: {} Remaining goals: {} Traveled distance: {}".format(new_world.score, new_world.remaining_time, len(new_world.remaining_goals), new_world.traveled_distance))

                world = new_world
            except GoalDeciderException as e:
                self.log(str(e))

        return world




class GoalDecider:
    def __init__(self, event_loop, goal_manager):
        import logger
        self.last_solution = None
        ":type self.last_solution: None or WorldState"
        self.logger = logger
        self.event_loop = event_loop
        self.gm = goal_manager

    def adapt_world(self, goals, map_, robot):
        w = WorldState(goals, map_, robot.event_loop.get_remaining_match_time())
        w.remaining_goals = goals
        w.map_ = map_
        w.robot_pose = robot.pose
        w.has_left_bulb = robot.has_left_bulb
        w.has_right_bulb = robot.has_right_bulb
        w.left_builder_count = robot.left_stand_count
        w.right_builder_count = robot.right_stand_count

        self.logger.log("Initial world situation: {}".format(w.to_str()))

        return w

    def init_context(self, goals, map_, robot, max_duration: float, max_depth: int):
        w = self.adapt_world(goals, map_, robot)

        ex = Explorer()

        ex.set_max_result(10)
        ex.set_max_duration(max_duration)
        ex.set_max_depth(max_depth)
        ex.set_world(w)

        return ex, w

    def explore_for_best_goal(self, goals, map_, robot, max_duration: float, max_depth: int):
        ex, base_world = self.init_context(goals, map_, robot, max_duration, max_depth)

        goals, best_world = ex.explore(base_world)

        self.logger.log("Evaluated count : {}".format(ex.evaluated_count))
        if goals:
            identifier=goals[0].identifier
            self.logger.log("Best goal is: {}".format(identifier))
            return identifier, best_world
        else:
            self.logger.log("Best goal was not found")
            return None, None

    def get_best_goal(self, goals, map_, robot, max_duration: float, max_depth: int):
        best = None
        if self.last_solution:
            self.logger.log("Trying to validate latest solution: {}".format(self.last_solution.goal_trace()))
            best = self.validate_latest_solution(goals, map_, robot, max_duration, max_depth)

        if not best:
            self.logger.log("Doing full exploration")
            best, self.last_solution = self.explore_for_best_goal(goals, map_, robot, max_duration, max_depth)

        return best

    def handle_navigation_failure(self):
        self.logger.log("Navigation failed, invalidating latest solution")
        self.last_solution = None

    def validate_latest_solution(self, goals, map_, robot, max_duration, max_depth):
        available_goals = self.current_available_goals

        if not available_goals:
            return

        ex, base_world = self.init_context(goals, map_, robot, max_duration, max_depth)

        evaluated_solution = ex.evaluate(base_world, self.last_solution)

        self.logger.log("Previous solution score: {} Re-evaluated to: {}".format(self.last_solution.score, evaluated_solution.score))

        if evaluated_solution.score == self.last_solution.score :
            available_ids = set(g.identifier for g in goals)
            for goal in evaluated_solution.executed_goals:
                if goal.identifier in available_ids:
                    return goal.identifier

    def get_current_goals(self):
        if self.last_solution:
            return [ g.uid for g in self.last_solution.executed_goals ]

    @property
    def current_available_goals(self):
        if self.last_solution:
            return (g for g in (self.gm.get_goal_by_uid(g.uid) for g in self.last_solution.executed_goals ) if g.is_available() )

    @property
    def done_goals(self):
        return (g for g in self.gm.goals if g.is_done())

    def get_sysinfo(self):
        curr = self.gm.get_current_goal()
        return {
            "current_goals": self.get_current_goals(),
            "current_available_goals": goals_to_list(self.current_available_goals),
            "done_goals": goals_to_list(self.done_goals),
            "doing_goal": curr.uid if curr else None,
            "remaining_time": self.event_loop.get_remaining_match_time()
        }