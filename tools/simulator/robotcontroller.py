# encoding: utf-8

import os
import random

from PyQt5.QtCore import *
from PyQt5.QtWidgets import *
from PyQt5.QtNetwork import *

import packets
import binarizer
from definitions import *

import simulatorfieldview




class RobotController(object):

    def __init__(self, game_controller, main_window, output_view, offset):
        self.game_controller = game_controller
        self.output_view = output_view
        self.offset = offset

        self.team = None
        self.team_name = "?"
        self.team_color = "#555753"
        self.is_main = None

        self.layers = []
        self.robot_layer = simulatorfieldview.RobotLayer(main_window.field_view_controller, self)
        self.layers.append(self.robot_layer)
        self.graph_routing_layer = simulatorfieldview.GraphRoutingLayer(main_window.field_view_controller, self)
        self.layers.append(self.graph_routing_layer)

        self.process = None
        self.socket = None
        self.incoming_packet_buffer = ""
        self.incoming_packet = None
        self.ready = False
        self.resettle_count = 0
        self.stop_requested = False
        self.servo_positions={}


    def get_input_id(self, name):
        if self.is_main:
            return MAIN_INPUT.lookup_by_name["MAIN_" + name]
        else:
            return SECONDARY_INPUT.lookup_by_name["SECONDARY_" + name]


    def is_process_started(self):
        return self.process is not None


    def is_connected(self):
        return self.socket is not None


    def is_ready(self):
        return self.ready


    def hide_all(self):
        for layer in self.layers:
            layer.hide()


    def setup(self, team, is_main):
        if self.process is None:
            self.incoming_packet_buffer = ""
            self.incoming_packet = None
            self.resettle_count = 0
            self.team = team
            self.is_main = is_main

            self.battery_left=15.5
            self.battery_right=15.5

            if team == TEAM_RIGHT:
                self.team_name = "green"
                self.team_color = TEAM_RIGHT_COLOR
            else:
                self.team_name = "yellow"
                self.team_color = TEAM_LEFT_COLOR
            if is_main:
                self.inputs = MAIN_INPUT
                self.team_name = "Main " + self.team_name
            else:
                self.inputs = SECONDARY_INPUT
                self.team_name = "Secondary " + self.team_name
                self.team_color = QColor(self.team_color).lighter().name()

            self.output_view.setup(team, is_main)
            for layer in self.layers:
                layer.show()
                layer.setup()

            brewery = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))), "brewery", "brewery.py")
            self.process = QProcess()
            self.process.setProcessChannelMode(QProcess.MergedChannels)
            self.process.readyRead.connect(self.read_output)
            args = ["--color", "always"]

            if self.is_main:
                args.append("--hostname")
                args.append("main")
            else:
                args.append("--hostname")
                args.append("secondary")

            if self.offset == 0 and self.game_controller.args.pydev_debug is not None:
                args.append("--pydev-debug")
                args+=self.game_controller.args.pydev_debug

            if is_main:
                if self.game_controller.args.main_fsm is not None:
                    args.append(self.game_controller.args.main_fsm)
                else:
                    args.append("main")
            else :
                if self.game_controller.args.secondary_fsm is not None:
                    args.append(self.game_controller.args.secondary_fsm)
                else:
                    args.append("secondary")

            self.process.start(brewery, args)


    def shutdown(self):
        if self.process is not None:
            if self.socket is not None:
                self.socket.disconnected.disconnect(self.shutdown)
            self.process.terminate()
            self.process.waitForFinished()
            self.process = None
            self.socket = None
            self.ready = False


    def connected(self, socket):
        self.socket = socket
        self.socket.setSocketOption(QAbstractSocket.LowDelayOption, 1)
        self.socket.disconnected.connect(self.shutdown)
        self.socket.readyRead.connect(self.read_packet)
        self.try_device_ready()


    def read_output(self):
        while self.process.canReadLine():
            log = str(self.process.readLine(), "utf-8")[:-1]
            self.output_view.add_log(log)


    def read_packet(self):
        if self.socket is not None:
            while self.socket.bytesAvailable() > 0 :
                if self.incoming_packet is None:
                    self.incoming_packet = packets.create_packet(self.socket.peek(1))
                if self.socket.bytesAvailable() >= self.incoming_packet.get_size():
                    buf = self.socket.read(self.incoming_packet.get_size())
                    packet = self.incoming_packet
                    self.incoming_packet = None
                    packet.deserialize(buf)
                    packet.dispatch(self)
                    packet.dispatch(self.robot_layer)
                    packet.dispatch(self.graph_routing_layer)


    def on_enable_anti_blocking(self, packet):
        self.send_packet(packet)


    def on_disable_anti_blocking(self, packet):
        self.send_packet(packet)


    def on_position_control_config(self, packet):
        self.send_packet(packet)


    def on_stop(self, packet):
        self.stop()


    def on_resettle(self, packet):
        self.send_packet(packet)
        self.resettle_count += 1
        if self.resettle_count == 2:
            self.ready = True
            self.game_controller.try_start()


    def on_servo_control(self, packet):
        """

        :type packet: packets.ServoControl
        """
        # if packet.type
        # packet.status = SERVO_STATUS_SUCCESS
        if packet.command == SERVO_COMMAND_POSITION:
            packet.value=self.servo_positions.setdefault(packet.id,random.randint(0,1023))
        if packet.command == SERVO_COMMAND_MOVE:
            self.servo_positions[packet.id]=packet.value
        packet.status = SERVO_STATUS_SUCCESS
        self.send_packet(packet)


    def on_output_control(self, packet):
        self.send_packet(packet)


    def on_pwm_control(self, packet):
        self.send_packet(packet)


    def on_input_status_request(self, packet):
        status = packets.InputStatus()
        status.id = packet.id
        status.kind = KIND_READ
        if packet.id == self.get_input_id("INPUT_TEAM"):
            status.value = self.team
        else:
            status.value = 1 #random.choice([0, 1])
        self.send_packet(status)


    def on_simulator_data(self, packet):
        self.output_view.handle_led(packet.leds)


    def send_packet(self, packet):
        if self.is_connected():
            self.socket.write(packet.serialize())


    def try_device_ready(self):
        if random.choice([True, False]):
            # OK, ready
            self.send_status()
        else:
            # Still busy
            self.send_status(True)
            QTimer.singleShot(500, self.send_status)


    def send_status(self, busy = None):
        packet = packets.ControllerStatus()
        packet.status = CONTROLLER_STATUS_BUSY if busy else CONTROLLER_STATUS_READY
        packet.remote_device = REMOTE_DEVICE_SIMULATOR
        self.send_packet(packet)


    def send_keep_alive(self):
        if self.robot_layer.robot.item is not None:
            packet = packets.KeepAlive()
            packet.current_pose = self.robot_layer.get_pose()
            packet.left_battery_voltage = self.battery_left
            packet.right_battery_voltage = self.battery_right
            self.send_packet(packet)
            self.battery_left-=0.0001*random.random()
            self.battery_right-=0.0001*random.random()


    def send_start_signal(self):
        packet = packets.InputStatus()
        packet.id = self.get_input_id("INPUT_START")
        packet.value = 0
        self.send_packet(packet)


    def send_goto_finished(self, reason, current_point_index):
        self.goto_packet = None
        packet = packets.GotoFinished()
        packet.reason = reason
        packet.current_pose = self.robot_layer.get_pose()
        packet.current_point_index = current_point_index
        self.send_packet(packet)


    def pause(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.pause_animation()


    def stop(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Running:
            self.robot_layer.robot.stop_animation()


    def resume(self):
        if self.robot_layer.robot.move_animation.state() == QAbstractAnimation.Paused:
            self.robot_layer.robot.resume_animation()
