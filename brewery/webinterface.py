# encoding: utf-8


import json
import os
import socket
import traceback
import urllib.parse

import statemachine

from definitions import *




class WebInterface:

    def __init__(self, event_loop):
        self.event_loop = event_loop

    def __call__(self, environ, start_response):
        response = None
        path = environ["PATH_INFO"]
        if path == "/hostname":
            response = socket.gethostname()
        elif path == "/statemachines":
            response = self.statemachines(environ)
        elif path == "/remotecontrol":
            response = self.remotecontrol(environ)
        elif path == "/eval":
            response = self.eval(environ)

        if response is None:
            code = "404 Not Found"
            response = ""
        else:
            code = "200 OK"

        start_response(code, [('Content-type','text/plain; charset=utf-8')])
        return [bytes(response, "utf-8")]


    def hostname(self, environ):
        return socket.gethostname()


    def statemachines(self, environ):
        result = {}
        for fsm in self.event_loop.fsms:
            states = []
            result[fsm.name] = states
            for state in fsm.state_stack:
                states.append(state.name)
        return json.dumps(result)


    def remotecontrol(self, environ):
        result = []

        if IS_MAIN_ROBOT:
            actions = []
            actions.append("yield Trigger(LEFT_BUILDER_PLIERS_LEFT_CLOSE, LEFT_BUILDER_PLIERS_RIGHT_CLOSE)")
            actions.append("yield Trigger(LEFT_BUILDER_PLIERS_LEFT_OPEN, LEFT_BUILDER_PLIERS_RIGHT_OPEN)")
            actions.append("yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_CLOSE, LEFT_BUILDER_GRIPPER_RIGHT_CLOSE)")
            actions.append("yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_GUIDE, LEFT_BUILDER_GRIPPER_RIGHT_GUIDE)")
            actions.append("yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_OPEN, LEFT_BUILDER_GRIPPER_RIGHT_OPEN)")
            actions.append("yield Trigger(LEFT_BUILDER_LIGHTER_CLOSE)")
            actions.append("yield Trigger(LEFT_BUILDER_LIGHTER_OPEN)")
            actions.append("yield Trigger(LEFT_BUILDER_ELEVATOR_DOWN)")
            actions.append("yield Trigger(LEFT_BUILDER_ELEVATOR_PLATFORM)")
            actions.append("yield Trigger(LEFT_BUILDER_ELEVATOR_UP)")
            result.append(["Left Builder", actions])

            actions = []
            actions.append("yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_CLOSE, RIGHT_BUILDER_PLIERS_RIGHT_CLOSE)")
            actions.append("yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_OPEN, RIGHT_BUILDER_PLIERS_RIGHT_OPEN)")
            actions.append("yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_CLOSE, RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE)")
            actions.append("yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_GUIDE, RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE)")
            actions.append("yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_OPEN, RIGHT_BUILDER_GRIPPER_RIGHT_OPEN)")
            actions.append("yield Trigger(RIGHT_BUILDER_LIGHTER_CLOSE)")
            actions.append("yield Trigger(RIGHT_BUILDER_LIGHTER_OPEN)")
            actions.append("yield Trigger(RIGHT_BUILDER_ELEVATOR_DOWN)")
            actions.append("yield Trigger(RIGHT_BUILDER_ELEVATOR_PLATFORM)")
            actions.append("yield Trigger(RIGHT_BUILDER_ELEVATOR_UP)")
            result.append(["Right Builder", actions])

            actions = []
            actions.append("yield Trigger(LIGHTER_GRIPPER_CLOSE)")
            actions.append("yield Trigger(LIGHTER_GRIPPER_OPEN)")
            actions.append("yield Trigger(LIGHTER_ELEVATOR_DOWN)")
            actions.append("yield Trigger(LIGHTER_ELEVATOR_BULB)")
            actions.append("yield Trigger(LIGHTER_ELEVATOR_UP)")
            result.append(["Lighter", actions])

            actions = []
            actions.append("yield Trigger(LEFT_CLAPMAN_CLOSE)")
            actions.append("yield Trigger(LEFT_CLAPMAN_OPEN)")
            result.append(["Left clapman", actions])

            actions = []
            actions.append("yield Trigger(RIGHT_CLAPMAN_CLOSE)")
            actions.append("yield Trigger(RIGHT_CLAPMAN_OPEN)")
            result.append(["Right clapman", actions])
        else:
            actions = []
            actions.append("yield Trigger(LEFT_CARPET_DROPPER_CLOSE)")
            actions.append("yield Trigger(LEFT_CARPET_DROPPER_OPEN)")
            actions.append("yield Trigger(LEFT_CARPET_EJECTOR_HOLD)")
            actions.append("yield Trigger(LEFT_CARPET_EJECTOR_THROW)")
            result.append(["Left carpet", actions])

            actions = []
            actions.append("yield Trigger(RIGHT_CARPET_DROPPER_CLOSE)")
            actions.append("yield Trigger(RIGHT_CARPET_DROPPER_OPEN)")
            actions.append("yield Trigger(RIGHT_CARPET_EJECTOR_HOLD)")
            actions.append("yield Trigger(RIGHT_CARPET_EJECTOR_THROW)")
            result.append(["Right carpet", actions])

            actions = []
            actions.append("yield Trigger(CUP_GRIPPER_CLOSE)")
            actions.append("yield Trigger(CUP_GRIPPER_ON_CUP)")
            actions.append("yield Trigger(CUP_GRIPPER_HALF_OPEN)")
            actions.append("yield Trigger(CUP_GRIPPER_OPEN)")
            result.append(["Cup gripper", actions])

        actions = []
        actions.append("yield GotoHome()")
        actions.append("yield MoveLineRelative(0.2)")
        actions.append("yield MoveLineRelative(-0.2, DIRECTION_BACKWARDS)")
        actions.append("yield RotateRelative(math.radians(45))")
        actions.append("yield RotateRelative(math.pi / 4.0)")
        actions.append("yield Timer(1000)")
        actions.append("yield DefinePosition(0.0, 0.0, math.pi / 2.0)")
        actions.append("yield AntiBlocking(True)")
        actions.append("yield SpeedControl(0.3)")
        actions.append("yield Rotate(math.pi / 2.0)")
        actions.append("yield LookAt(1.0, 1.0)")
        actions.append("yield LookAtOpposite(1.0, 1.0)")
        actions.append("yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)])")
        actions.append("yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)], DIRECTION_BACKWARDS)")
        actions.append("yield MoveLineTo(0.1, 0.0)")
        actions.append("yield MoveLineTo(0.1, 0.0, DIRECTION_BACKWARDS)")
        actions.append("yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)])")
        actions.append("yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)], DIRECTION_BACKWARDS)")
        result.append(["Misc", actions])

        return json.dumps(result)


    def eval(self, environ):
        encoding = "iso-8859-1"
        if "CONTENT_TYPE" in environ:
            for key, value in urllib.parse.parse_qsl(environ["CONTENT_TYPE"]):
                if key == "charset":
                    encoding = value
                    break
        code = str(environ["wsgi.input"].read(), encoding)
        fsm = statemachine.StateMachine(self.event_loop, "eval", code = code)
        text = ""
        if fsm.error is not None:
            for l in traceback.format_exception(type(fsm.error), fsm.error, None):
                text += l
        else:
            text = "OK"
        return text
