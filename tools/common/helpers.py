# encoding: utf-8

from PyQt4.Qt import Qt
from PyQt4.QtGui import *

import os
import subprocess
import collections

from definitions import *



def get_last_remote_logfile():
    drunkstar_ip = "192.168.1.42"
    drunkstar_bhware = "/root/logs"
    ls = subprocess.Popen(["ssh", "root@{}".format(drunkstar_ip), "ls {}/brewerylog*".format(drunkstar_bhware)], stdout = subprocess.PIPE, stderr = subprocess.PIPE)
    (out, err) = ls.communicate()
    logs = out.split("\n")
    if len(logs) != 0:
        logs.sort()
        log = logs[-1]
        if not os.path.exists("/tmp/bh"):
            os.makedirs("/tmp/bh")
        local_file = "/tmp/bh/remote_{}".format(os.path.basename(log))
        subprocess.call(["scp", "root@{}:{}".format(drunkstar_ip, log), local_file])
        return local_file

    return None




def get_last_logfile():
    brewery_root_path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))
    log_dir = os.path.join(brewery_root_path, "logs")
    index = 0
    if os.path.exists(log_dir):
        while True:
            filepath = os.path.join(log_dir, "brewerylog_{:=#04}.py".format(index))
            if os.path.exists(filepath):
                index += 1
            else:
                index -= 1
                return os.path.join(log_dir, "brewerylog_{:=#04}.py".format(index))
    return None




def create_main_robot_base_item(pen, brush, gyration_pen):
    robot = QGraphicsItemGroup()

    path = QPainterPath()
    path.moveTo(-150, -145)
    path.lineTo(-150, 145)
    path.lineTo(150, 145)
    path.lineTo(150, 102.5)
    path.arcTo(180, 102.5, -60.0, -60.0, 90.0, -180.0)
    path.lineTo(150, -42.5)
    path.arcTo(180, -42.5, -60.0, -60.0, 90.0, -180.0)
    path.lineTo(150, -145)
    path.closeSubpath()
    base = QGraphicsPathItem(path)
    base.setPen(pen)
    base.setBrush(brush)
    robot.addToGroup(base)

    gyration_radius = MAIN_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return all_group, robot, gyration




def create_secondary_robot_base_item(pen, brush, gyration_pen):
    robot = QGraphicsItemGroup()

    path = QPainterPath()
    path.moveTo(-64.5, -77.5)
    path.lineTo(64.5, -77.5)
    path.lineTo(82.5, -58)
    path.lineTo(82.5, 58)
    path.lineTo(64.5, 77.5)
    path.lineTo(-64.5, 77.5)
    path.lineTo(-82.5, 58)
    path.lineTo(-82.5, -58)
    path.closeSubpath()
    base = QGraphicsPathItem(path)
    base.setPen(pen)
    base.setBrush(brush)
    robot.addToGroup(base)

    robot.addToGroup(create_arrow(pen, -20.0, -60.0, 40.0, 135.0))

    gyration_radius = SECONDARY_ROBOT_GYRATION_RADIUS * 1000.0
    gyration = QGraphicsEllipseItem(-gyration_radius, -gyration_radius, 2.0 * gyration_radius, 2.0 * gyration_radius)
    gyration.setPen(gyration_pen)

    all_group = QGraphicsItemGroup()
    all_group.addToGroup(robot)
    all_group.addToGroup(gyration)

    return all_group, robot, gyration




def create_arrow(pen, x, y, width, length):
    path = QPainterPath()
    path.moveTo(y, x)
    path.lineTo(y + length - width * 2.0, x)
    path.lineTo(y + length, x + width / 2.0)
    path.lineTo(y + length - width * 2.0, x + width)
    path.lineTo(y, x + width)
    path.closeSubpath()

    item = QGraphicsPathItem(path)
    item.setPen(pen)
    return item
