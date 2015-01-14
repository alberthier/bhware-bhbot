# encoding: utf-8

import math


EPSILON = 1e-6


def quasi_equal(f1, f2):
    return abs(f1 - f2) < EPSILON


def quasi_null(f1):
    return quasi_equal(f1, 0.0)


def is_between(a, b, x):
    if a < b:
        ok = a < x and x < b
    else:
        ok = b < x and x < a
    return ok or quasi_equal(a, x) or quasi_equal(b, x)


def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def manathan_distance(x1, y1, x2, y2):
    return abs(x2 - x1) + abs(y2 - y1)


def angle_between(x1, y1, x2, y2):
    return math.atan2(y2 - y1, x2 - x1)


def normalize_angle(a):
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


def angle_minus_pi_to_pi(a):
    if a > math.pi :
        return a - 2 * math.pi
    if a <= -math.pi :
        return a + 2 * math.pi
    return a


def compare_angles(a1, a2, epsilon):
    return abs(angle_minus_pi_to_pi((angle_minus_pi_to_pi(a1) - angle_minus_pi_to_pi(a2)))) < epsilon


def get_center_pose_for_point(robot_point_x, robot_point_y, dest_x, dest_y, dest_angle):
    """
        Returns the pose of the center of the robots that brings the point (robot_point_x, robot_point_y)
        at (dest_x, dest_y)
    """
    cos_a = math.cos(dest_angle)
    sin_a = math.sin(dest_angle)
    center_x = dest_x - (cos_a * robot_point_x - sin_a * robot_point_y)
    center_y = dest_y + sin_a * robot_point_x + cos_a * robot_point_y
    return (center_x, center_y, dest_angle)


def get_direction(robot_pose, x, y):
    dx = x - robot_pose.virt.x
    dy = y - robot_pose.virt.y
    a = math.atan2(dy, dx) - robot_pose.virt.angle
    if -math.pi / 2.0 <= a and a <= math.pi / 2.0:
        return 1 # DIRECTION_FORWARD
    else:
        return -1 # DIRECTION_BACKWARDS


def get_offset_position(robot_pose, x, y, offset):
    a = angle_between(robot_pose.x, robot_pose.y, x, y)
    dist = distance(robot_pose.x, robot_pose.y, x, y)
    dist += offset
    x = robot_pose.x + math.cos(a) * dist
    y = robot_pose.y + math.sin(a) * dist
    return (x, y)
