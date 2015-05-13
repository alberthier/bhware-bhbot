# encoding: utf-8
import inspect

import math
import functools
import sys

import logger

EPSILON = 1e-6

@functools.lru_cache()
def quasi_equal(f1, f2):
    return abs(f1 - f2) < EPSILON


@functools.lru_cache()
def quasi_null(f1):
    return quasi_equal(f1, 0.0)

@functools.lru_cache()
def is_between(a, b, x):
    if a < b:
        ok = a < x and x < b
    else:
        ok = b < x and x < a
    return ok or quasi_equal(a, x) or quasi_equal(b, x)

@functools.lru_cache()
def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

@functools.lru_cache()
def manathan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)

@functools.lru_cache()
def angle_between(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)

@functools.lru_cache()
def normalize_angle(a):
    """
    :param a: angle
    :return: normalized angle between -math.pi and math.pi
    """
    na = a % (2.0 * math.pi)
    if na > math.pi:
        na -= 2.0 * math.pi
    return na

def sym_x(coord_x):
    return 2.0 - coord_x

def sym_y(coord_y):
    return 3.0 - coord_y

def sym_angle(angle):
    return -angle

@functools.lru_cache()
def compare_angles(a1, a2, epsilon):
    return abs(normalize_angle((angle_minus_pi_to_pi(a1) - normalize_angle(a2)))) < epsilon

@functools.lru_cache()
def get_center_pose_for_point(robot_point_x, robot_point_y, dest_x, dest_y, dest_angle):
    """
        Returns the pose of the center of the robots that brings the point (robot_point_x, robot_point_y)
        at (dest_x, dest_y)
    """
    cos_a = math.cos(dest_angle)
    sin_a = math.sin(dest_angle)
    center_x = dest_x - (cos_a * robot_point_x - sin_a * robot_point_y)
    center_y = dest_y - (sin_a * robot_point_x + cos_a * robot_point_y)
    return center_x, center_y, dest_angle


def get_direction(robot_pose, x, y):
    dx = x - robot_pose.virt.x
    dy = y - robot_pose.virt.y
    a = math.atan2(dy, dx) - robot_pose.virt.angle
    a = normalize_angle(a)
    if -math.pi / 2.0 <= a and a <= math.pi / 2.0:
        return 1 # DIRECTION_FORWARD
    else:
        return -1 # DIRECTION_BACKWARDS


def get_offset_position(robot_pose, x, y, offset):
    """
    Returns the (x, y) position offseted by 'offset' mm with robot_pose as start reference
    """
    a = angle_between(robot_pose.virt.x, robot_pose.virt.y, x, y)
    dist = distance(robot_pose.virt.x, robot_pose.virt.y, x, y)
    dist += offset
    x = robot_pose.virt.x + math.cos(a) * dist
    y = robot_pose.virt.y + math.sin(a) * dist
    return x, y

@functools.lru_cache()
def get_crossing_point(x1, y1, angle1, x2, y2, angle2):
    if quasi_equal(abs(angle1), math.pi / 2.0):
        a2 = math.tan(angle2)
        b2 = y2 - a2 * x2
        xc = x1
        yc = a2 * xc + b2
    elif quasi_equal(abs(angle2), math.pi / 2.0):
        a1 = math.tan(angle1)
        b1 = y1 - a1 * x1
        xc = x2
        yc = a1 * xc + b1
    else:
        a1 = math.tan(angle1)
        b1 = y1 - a1 * x1
        a2 = math.tan(angle2)
        b2 = y2 - a2 * x2
        xc = (b2 - b1) / (a1 - a2)
        yc = a1 * xc + b1
    return xc, yc

# from http://kracekumar.com/post/100897281440/fluent-interface-in-python
from functools import wraps

def newobj(method):
    @wraps(method)
    # Well, newobj can be decorated with function, but we will cover the case
    # where it decorated with method
    def inner(self, *args, **kwargs):
        obj = self.__class__.__new__(self.__class__)
        obj.__dict__ = self.__dict__.copy()
        method(obj, *args, **kwargs)
        return obj
    return inner

def on_end_of_match():
    logger.log("LRU cache info")
    for name, obj in inspect.getmembers(sys.modules[__name__]):
        if inspect.isfunction(obj):
            if hasattr(obj, "cache_info"):
                info=obj.cache_info()
                efficiency=0.0
                if info.currsize > 0:
                    efficiency=info.hits*100/(info.hits+info.misses)
                logger.log("{}: efficiency: {}% {}".format(name, efficiency, info))