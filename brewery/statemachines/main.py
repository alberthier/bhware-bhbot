# encoding: utf-8

import collections
import math

import packets
import position
import logger
import goalmanager
import tools
import robot

from statemachine import *
from definitions import *
from commonstates import *
from position import *
from tools import *

import statemachines.testscommon as testscommon
import statemachines.testsmain as testsmain




class Main(State):

    def on_enter(self):
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            yield Initialize()
            yield AntiBlocking(True)
            yield GetInputStatus(MAIN_INPUT_TEAM)
            yield CalibratePosition()


    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(89500, EndOfMatch())
            yield MoveLineTo(1.0, 1.5)

            #yield Trigger(RIGHT_BUILDER_PLIERS_RIGHT_INIT, RIGHT_BUILDER_GRIPPER_LEFT_INIT)
            #yield Trigger(makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 500))
            #rp = yield ReadServoPosition(LEFT_BUILDER_PLIERS_LEFT_ID)
            #print(rp.value)

            l_servoID = [LEFT_BUILDER_PLIERS_LEFT_ID, LEFT_BUILDER_PLIERS_RIGHT_ID, LEFT_BUILDER_GRIPPER_LEFT_ID, LEFT_BUILDER_GRIPPER_RIGHT_ID, LEFT_BUILDER_LIGHTER_ID]

            yield ServoTorqueControl(l_servoID, False)

            ArmPosition = yield ReadArmServoPosition(l_servoID)
            
            yield MoveArmServoPosition(ArmPosition.l_servoPosition)






class Initialize(State):

    def on_enter(self):
        yield Trigger(LEFT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_CLOSE)
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        yield None




class CalibratePosition(State):

    def on_enter(self):
        yield DefinePosition(1.0, 0.07 + ROBOT_CENTER_X, math.pi / 2.0)
        yield None



class ReadArmServoPosition(State):
    def __init__(self, l_servoID):
        self.l_servoID = l_servoID

    def on_enter(self):
        l_servoPosition = []
        for servoID in self.l_servoID :
            ret = yield ReadServoPosition(servoID)
            l_servoPosition.append([servoID, ret.value])
        self.l_servoPosition = l_servoPosition

        print("Positions of the arm:")
        print(l_servoPosition)
        
        yield None


class MoveArmServoPosition(State):

    def __init__(self, l_servoPosition):
        self.l_servoPosition = l_servoPosition

    def on_enter(self):
        l_ServoCommand =[]
        for servoPosition in self.l_servoPosition :
            servoID = servoPosition[0]
            position = servoPosition[1]
            l_ServoCommand.append(makeServoMoveCommand((servoID, SERVO_TIMEOUT), position))
            
        yield Trigger(l_ServoCommand)
        yield None


##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        tools.on_end_of_match()
