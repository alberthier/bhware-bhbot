# encoding: utf-8


import json
import logger
import os
import socket
import traceback
import urllib.parse

import statemachine

from definitions import *




LAST_WEB_FSM = None




class WebInterface:

    def __init__(self, event_loop):
        self.event_loop = event_loop

    def __call__(self, environ, start_response):
        response = None
        code = None
        path = environ["PATH_INFO"]
        handler = path[1:]
        if hasattr(self, handler):
            try:
                response = getattr(self, handler)(environ)
                code = "200 OK"
            except Exception as e:
                response = "Internal error:\n{}".format(traceback.format_exc())
                code = "500 Internal error"

        if response is None:
            code = "404 Not Found"
            response = ""

        content_type='text/plain; charset=utf-8'

        if isinstance(response, dict):
            content_type='application/json; charset=utf-8'
            response=json.dumps(response)

        start_response(code, [('Content-type',content_type)])
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
            actions = [
                "yield Trigger(LEFT_BUILDER_PLIERS_LEFT_INIT, LEFT_BUILDER_PLIERS_RIGHT_INIT)",
                "yield Trigger(LEFT_BUILDER_PLIERS_LEFT_CLOSE, LEFT_BUILDER_PLIERS_RIGHT_CLOSE)",
                "yield Trigger(LEFT_BUILDER_PLIERS_LEFT_HOLD, LEFT_BUILDER_PLIERS_RIGHT_HOLD)",
                "yield Trigger(LEFT_BUILDER_PLIERS_LEFT_OPEN, LEFT_BUILDER_PLIERS_RIGHT_OPEN)",
                "yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_INIT, LEFT_BUILDER_GRIPPER_RIGHT_INIT)",
                "yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_CLOSE, LEFT_BUILDER_GRIPPER_RIGHT_CLOSE)",
                "yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_GUIDE, LEFT_BUILDER_GRIPPER_RIGHT_GUIDE)",
                "yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_LIGHT, LEFT_BUILDER_GRIPPER_RIGHT_LIGHT)",
                "yield Trigger(LEFT_BUILDER_GRIPPER_LEFT_DEPOSIT, LEFT_BUILDER_GRIPPER_RIGHT_DEPOSIT)",
                "yield Trigger(LEFT_BUILDER_LIGHTER_CLOSE)",
                "yield Trigger(LEFT_BUILDER_LIGHTER_OPEN)",
                "yield Trigger(LEFT_BUILDER_ELEVATOR_DOWN)",
                "yield Trigger(LEFT_BUILDER_ELEVATOR_PLATFORM)",
                "yield Trigger(LEFT_BUILDER_ELEVATOR_UP)"
            ]
            result.append(["Left Builder", actions])

            actions = [
                "yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_INIT, RIGHT_BUILDER_PLIERS_RIGHT_INIT)",
                "yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_CLOSE, RIGHT_BUILDER_PLIERS_RIGHT_CLOSE)",
                "yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_HOLD, RIGHT_BUILDER_PLIERS_RIGHT_HOLD)",
                "yield Trigger(RIGHT_BUILDER_PLIERS_LEFT_OPEN, RIGHT_BUILDER_PLIERS_RIGHT_OPEN)",
                "yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_INIT, RIGHT_BUILDER_GRIPPER_RIGHT_INIT)",
                "yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_CLOSE, RIGHT_BUILDER_GRIPPER_RIGHT_CLOSE)",
                "yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_GUIDE, RIGHT_BUILDER_GRIPPER_RIGHT_GUIDE)",
                "yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_LIGHT, RIGHT_BUILDER_GRIPPER_RIGHT_LIGHT)",
                "yield Trigger(RIGHT_BUILDER_GRIPPER_LEFT_DEPOSIT, RIGHT_BUILDER_GRIPPER_RIGHT_DEPOSIT)",
                "yield Trigger(RIGHT_BUILDER_LIGHTER_CLOSE)",
                "yield Trigger(RIGHT_BUILDER_LIGHTER_OPEN)",
                "yield Trigger(RIGHT_BUILDER_ELEVATOR_DOWN)",
                "yield Trigger(RIGHT_BUILDER_ELEVATOR_PLATFORM)",
                "yield Trigger(RIGHT_BUILDER_ELEVATOR_UP)"
            ]
            result.append(["Right Builder", actions])

            actions = [
                "yield Trigger(LIGHTER_GRIPPER_CLOSE)",
                "yield Trigger(LIGHTER_GRIPPER_OPEN)",
                "yield Trigger(LIGHTER_ELEVATOR_DOWN)",
                "yield Trigger(LIGHTER_ELEVATOR_BULB)",
                "yield Trigger(LIGHTER_ELEVATOR_UP)"
            ]
            result.append(["Lighter", actions])

            actions = [
                "yield Trigger(LEFT_CLAPMAN_CLOSE)",
                "yield Trigger(LEFT_CLAPMAN_OPEN)"
            ]
            result.append(["Left clapman", actions])

            actions = [
                "yield Trigger(RIGHT_CLAPMAN_CLOSE)",
                "yield Trigger(RIGHT_CLAPMAN_OPEN)"
            ]
            result.append(["Right clapman", actions])
        else:
            actions = [
                "yield Trigger(LEFT_CARPET_DROPPER_CLOSE)",
                "yield Trigger(LEFT_CARPET_DROPPER_OPEN)",
                "yield Trigger(LEFT_CARPET_EJECTOR_HOLD)",
                "yield Trigger(LEFT_CARPET_EJECTOR_THROW)"
            ]
            result.append(["Left carpet", actions])

            actions = [
                "yield Trigger(RIGHT_CARPET_DROPPER_CLOSE)",
                "yield Trigger(RIGHT_CARPET_DROPPER_OPEN)",
                "yield Trigger(RIGHT_CARPET_EJECTOR_HOLD)",
                "yield Trigger(RIGHT_CARPET_EJECTOR_THROW)"
            ]
            result.append(["Right carpet", actions])

            actions = [
                "yield Trigger(CUP_GRIPPER_CLOSE)",
                "yield Trigger(CUP_GRIPPER_ON_CUP)",
                "yield Trigger(CUP_GRIPPER_HALF_OPEN)",
                "yield Trigger(CUP_GRIPPER_SEEKING)",
                "yield Trigger(CUP_GRIPPER_OPEN)"
            ]
            result.append(["Cup gripper", actions])

            actions = [
                "yield Trigger(CLAPMAN_CLOSE)",
                "yield Trigger(CLAPMAN_OPEN)"
            ]
            result.append(["Clapman", actions])

        actions = [
            "yield GotoHome()",
            "yield MoveLineRelative(0.2)",
            "yield MoveLineRelative(-0.2, DIRECTION_BACKWARDS)",
            "yield RotateRelative(math.radians(45))",
            "yield RotateRelative(math.pi / 4.0)",
            "yield Timer(1000)",
            "yield DefinePosition(0.0, 0.0, math.pi / 2.0)",
            "yield AntiBlocking(True)",
            "yield SpeedControl(0.3)",
            "yield Rotate(math.pi / 2.0)",
            "yield LookAt(1.0, 1.0)",
            "yield LookAtOpposite(1.0, 1.0)",
            "yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)])",
            "yield MoveCurve(math.pi / 2.0, [(0.1, 0.2), (0.4, 0.5)], DIRECTION_BACKWARDS)",
            "yield MoveLineTo(0.1, 0.0)",
            "yield MoveLineTo(0.1, 0.0, DIRECTION_BACKWARDS)",
            "yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)])",
            "yield MoveLine([(0.1, 0.0), (0.2, 0.0), (0.3, 0.0)], DIRECTION_BACKWARDS)"
        ]
        result.append(["Misc", actions])

        result.append(
            [
                "Calibration",
                [

                    [
                        "yield DefinePosition(0.0, 0.0, 0.0)",
                        "yield MoveLineTo(1.3,0.0)"
                    ],
                    [
                        "yield DefinePosition(0.0, 0.0, 0.0)",
                        "yield MoveLineTo(1.7,0.0)"
                    ],
                    [
                        "yield DefinePosition(0.0, 0.0, 0.0)"
                    ],
                    [
                        "for i in range(10): yield RotateTo(2*math.pi/3); yield RotateTo(-2*math.pi/3); yield RotateTo(0.0)"
                    ]

                ]
            ],

        )

        return json.dumps(result)


    def eval(self, environ):
        encoding = "iso-8859-1"
        if "CONTENT_TYPE" in environ:
            for key, value in urllib.parse.parse_qsl(environ["CONTENT_TYPE"]):
                if key == "charset":
                    encoding = value
                    break
        code = str(environ["wsgi.input"].read(), encoding)
        global LAST_WEB_FSM
        LAST_WEB_FSM = statemachine.StateMachine(self.event_loop, "eval", code = code)
        text = ""
        if LAST_WEB_FSM.error is not None:
            for l in traceback.format_exception(type(LAST_WEB_FSM.error), LAST_WEB_FSM.error, None):
                text += l
        else:
            text = "OK"
        return text


    def inputs(self, environ):
        result = {}

        inputs = MAIN_INPUT if IS_MAIN_ROBOT else SECONDARY_INPUT
        result = inputs.lookup_by_value

        return result


    def getevaloutput(self, environ):
        global LAST_WEB_FSM
        if LAST_WEB_FSM is not None:
            return LAST_WEB_FSM.output
        else:
            return ""


    def sysinfo(self, environ):
        return self.event_loop.sysinfo.get_data()
