# encoding: utf-8

import collections
import math

import statemachine
import packets
import mediahelpers

from definitions import *




class Main(statemachine.Timer):

    OPPONENT_DETECTION_DISAPPEARING_MS = 800

    MAIN_IN_FRONT_IDS = [ 0, 1, 2, 16, 17 ]
    MAIN_IN_BACK_IDS  = [ 7, 8, 9, 10, 11 ]
    MAIN_ANGLES = [ 0, -20, -40, -60, -80, -100, -120, -140, -160, 180, 160, 140, 120, 100, 80, 60, 40, 20 ]

    SECONDARY_IN_FRONT_IDS = [ 0, 1, 2, 16, 17 ]
    SECONDARY_IN_BACK_IDS  = [ 7, 8, 9, 10, 11 ]
    SECONDARY_ANGLES = [ 0, -20, -40, -60, -80, -100, -120, -140, -160, 180, 160, 140, 120, 100, 80, 60, 40, 20 ]

    PACKET_BUFFER_SIZE = 6

    def __init__(self):
        super().__init__(self.OPPONENT_DETECTION_DISAPPEARING_MS, False)
        self.in_front_ids = self.MAIN_IN_FRONT_IDS if IS_MAIN_ROBOT else self.SECONDARY_IN_FRONT_IDS
        self.in_back_ids  = self.MAIN_IN_BACK_IDS  if IS_MAIN_ROBOT else self.SECONDARY_IN_BACK_IDS
        self.angles       = self.MAIN_ANGLES       if IS_MAIN_ROBOT else self.SECONDARY_ANGLES
        self.detections = collections.deque()


    def on_enter(self):
        # Do not call super().on_enter() as we don't want to start the timer immediately
        self.opponent_direction = None
        self.x = None
        self.y = None
        if not hasattr(self.fsm, "enabled"):
            self.fsm.enabled = True
        # The timer is started only once an opponent has been detected
        self.stop()


    def on_turret_detect(self, packet):
        if not self.fsm.enabled:
            return
        if packet.robot != self.fsm.opponent_type:
            return

        self.detections.append(packet)
        if len(self.detections) < self.PACKET_BUFFER_SIZE:
            return
        elif len(self.detections) > self.PACKET_BUFFER_SIZE:
            self.detections.popleft()

        nearest_detection = None
        for detection in self.detections:
            if nearest_detection is None or detection.distance < nearest_detection.distance:
                nearest_detection = detection

        distance_m = nearest_detection.distance / 100.0

        robot_pose = self.robot.pose
        real_angle = math.radians(self.angles[nearest_detection.angle]) + robot_pose.angle
        self.x = robot_pose.x + distance_m * math.cos(real_angle)
        self.y = robot_pose.y + distance_m * math.sin(real_angle)

        previous_direction = self.opponent_direction
        if nearest_detection.angle in self.in_front_ids:
            self.opponent_direction = DIRECTION_FORWARD
        elif nearest_detection.angle in self.in_back_ids:
            self.opponent_direction = DIRECTION_BACKWARDS
        else:
            self.opponent_direction = None

        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, distance = nearest_detection.distance, x = self.x, y = self.y))
        self.restart()

        if self.opponent_direction is not None:
            if self.opponent_direction != previous_direction:
                self.log("{} opponent detected at ({:.2f}, {:.2f})".format(self.opponent_name(), self.x, self.y))
                self.set_detected(self.opponent_direction)
                self.send_packet(packets.OpponentDetected(robot = self.fsm.opponent_type, direction = self.opponent_direction, x = self.x, y = self.y))
                mediahelpers.safe_say(self, "OPPONENT_DETECTED")
        elif self.opponent_direction is None and previous_direction is not None:
            self.opponent_disappeared()


    def on_timeout(self):
        self.opponent_disappeared()


    def opponent_disappeared(self):
        self.log("{} opponent disappeared".format(self.opponent_name()))
        self.set_detected(None)
        self.stop()
        self.detections.clear()
        self.send_packet(packets.OpponentPosition(robot = self.fsm.opponent_type, x = None, y = None))
        if self.opponent_direction is not None:
            self.send_packet(packets.OpponentDisappeared(robot = self.fsm.opponent_type, direction = self.opponent_direction))
        self.opponent_direction = None


    def opponent_name(self):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            return "Main"
        else:
            return "Secondary"


    def set_detected(self, direction):
        if self.fsm.opponent_type == OPPONENT_ROBOT_MAIN:
            self.robot.main_opponent_direction = direction
        else:
            self.robot.secondary_opponent_direction = direction
