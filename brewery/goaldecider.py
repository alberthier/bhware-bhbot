import copy
import datetime
import goalmanager
import random
import position
# import logger
import robot


from definitions import *

MEAN_SPEED_PER_S=0.3
DEFAULT_GOAL_ESTIMATED_DURATION_S=15
MAX_STAND_PER_BUILDER=4

class GoalDeciderException(Exception):
    def __init__(self, goal):
         super(GoalDeciderException, self).__init__(goal.identifier)


class NoTimeToAttain(GoalDeciderException):
    pass


class NoTimeToExecute(GoalDeciderException):
    pass


class GoalImpossibleToExecute(GoalDeciderException):
    pass


class WorldState:
    def __init__(self, goals, map_, remaining_time):
        """

        :type goals: list(goalmanager.Goal)
        """
        import logger
        self.score = 0
        self.right_builder_count = 0
        self.left_builder_count = 0
        self.remaining_goals = goals
        self.executed_goals = []
        self.depth = 0
        self.remaining_time = remaining_time
        self.map_ = map_
        self.robot_pose = position.Pose(0,0,0)
        self.logger=logger
        self.traveled_distance = 0
        self.has_right_bulb = False
        self.has_left_bulb = False

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
        return new_inst




    @property
    def last_goal(self) -> goalmanager.Goal:
        return self.executed_goals[-1] if self.executed_goals else None




    def goal_trace(self):
        return "->".join((g.identifier for g in self.executed_goals))

    def execute_goal(self, goal: goalmanager.Goal):
        self.executed_goals.append(goal)
        self.remaining_goals=[g for g in self.remaining_goals if g.identifier != goal.identifier]

        self.logger.trace("Executing goal : {}".format(self.goal_trace()))

        distance = goal.pose - self.robot_pose
        elapsed_time = distance / MEAN_SPEED_PER_S
        self.traveled_distance+=distance

        if self.remaining_time - elapsed_time <= 0.0 :
            raise NoTimeToAttain(goal)

        self.remaining_time-=elapsed_time

        if self.remaining_time - (goal.estimated_duration or DEFAULT_GOAL_ESTIMATED_DURATION_S) <= 0.0 :
            raise NoTimeToExecute(goal)

        self.robot_pose = goal.pose.clone()

        if "STAND" in goal.identifier :
            if self.left_builder_count>= MAX_STAND_PER_BUILDER and self.right_builder_count>= MAX_STAND_PER_BUILDER:
                raise GoalImpossibleToExecute(goal)
            if self.left_builder_count>= MAX_STAND_PER_BUILDER:
                self.right_builder_count+=1
            elif self.right_builder_count>= MAX_STAND_PER_BUILDER:
                self.left_builder_count+=1
            else:
                if random.randint(0,1) == 0 :
                    #TODO : check how to affect builder
                    self.left_builder_count+=1
                else:
                    self.right_builder_count+=1

    def after_goal(self, goal):
        if "SPOTLIGHT" in goal.identifier :
            self.left_builder_count = 0
            self.right_builder_count = 0


def compare_world_by_score_and_dist(w1 : WorldState, w2 : WorldState):
    if w1.score > w2.score:
        return 1
    elif w1.score < w2.score:
        return -1
    else:
        return cmp(w1.traveled_distance, w2.traveled_distance)

def world_to_key(w : WorldState):
    return w.score * 1000 + w.traveled_distance


class Explorer:

    def __init__(self):
        import logger
        self.max_result=100
        self.max_duration=5
        self.start_time=None
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

        self.start_time = datetime.datetime.now()

        best = self.explore_recursive(world)

        if best:
            self.logger.log("Found best world")
            self.logger.log("Score: {} Remaining time: {}".format(best.score, best.remaining_time))
            self.logger.log("Remaining goals: {}".format(len(best.remaining_goals)))
            self.logger.log("Traveled distance: {}".format(best.traveled_distance))
            self.logger.log(best.goal_trace())
            return best.executed_goals

    def set_world(self, w):
        self.world = w

    # breadth-first exploration
    def explore_recursive(self, world: WorldState):
        new_worlds=[]

        for goal in world.remaining_goals :
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

                self.logger.trace("Score: {} Remaining time: {}".format(new_world.score, new_world.remaining_time))
                self.logger.trace("Remaining goals: {}".format(len(new_world.remaining_goals)))
                self.logger.trace("Traveled distance: {}".format(new_world.traveled_distance))

                new_worlds.append(new_world)
            except GoalDeciderException as e:
                self.log(str(e))

        if world.depth == self.max_depth:
            return self.best(new_worlds)

        allret=[]

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
            if "SPOTLIGHT" in last.identifier :
                if new_world.left_builder_count:
                    action_score+=new_world.left_builder_count*2
                    if new_world.has_left_bulb:
                        action_score+=3
                        new_world.has_left_bulb=False
                if new_world.right_builder_count:
                    action_score+=new_world.right_builder_count*2
                    if new_world.has_right_bulb:
                        action_score+=3
                        new_world.has_right_bulb=False


            elif "CLAP" in last.identifier :
                action_score+=5

        if action_score>0:
            self.logger.log("This action scored: {}".format(action_score))
            new_world.score += action_score

    def best(self, worlds):
        self.logger.log("Choosing best of {} worlds".format(len(worlds) if worlds else 0))
        if worlds :
            if len(worlds) > 1:
                worlds.sort(key=world_to_key, reverse=True)
            return worlds[0]


def adapt_world(goals, map_, robot):
    w = WorldState(goals, map_, robot.event_loop.get_remaining_match_time())
    w.remaining_goals = goals
    w.map_ = map_
    w.robot_pose = robot.pose
    w.has_left_bulb = robot.has_left_bulb
    w.has_right_bulb = robot.has_right_bulb

    return w


def get_best_goal(goals, map_, robot, max_duration: float, max_depth: int):
    import logger
    w = adapt_world(goals, map_, robot)

    ex = Explorer()

    ex.set_max_result(10)
    ex.set_max_duration(max_duration)
    ex.set_max_depth(max_depth)
    ex.set_world(w)

    goals = ex.explore(w)

    logger.log("Evaluated count : {}".format(ex.evaluated_count))
    if goals:
        identifier=goals[0].identifier
        logger.log("Best goal is: {}".format(identifier))
        return identifier
    else:
        logger.log("Best goal was not found")

