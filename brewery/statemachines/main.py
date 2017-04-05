# encoding: utf-8

import collections
import math
import os.path

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
        #StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        #StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        #self.l_armPosition = []
        if os.path.isfile("/tmp/traj.txt"):
            f_listArmPosition = open("/tmp/traj.txt", "r")
            self.l_armPosition = eval(f_listArmPosition.read())
            f_listArmPosition.close()
        else:
            self.l_armPosition = []
        None


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            #yield Initialize()
            #yield AntiBlocking(True)
            rt = yield GetInputStatus(MAIN_INPUT_TEAM)
            self.rec = rt.value

            self.l_servoID = [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]

            #yield CalibratePosition()

            #rp = yield ReadServoPosition(STORAGE_FINGER_LEFT_ID)
            #print("STORAGE_FINGER_LEFT_ID")
            #print(rp.value)
            None
            
    def on_input_status(self, packet):
        if packet.id == MAIN_INPUT_TEAM: # and packet.kind == KIND_READ:
            self.rec = packet.value
            #if packet.rec == 0:
            #    print("No recording")

            #if packet.value == 0:
            #    yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
            #else:
            #    yield Trigger(STORAGE_FINGER_RIGHT_INIT)



            if packet.value == 0:
                print("Recording")
                #yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
                #self.l_armPosition = []
                #for i_rec in range(40):
                ArmPosition = yield ReadArmServoPosition(self.l_servoID)
                self.l_armPosition.append(ArmPosition.l_servoPosition)
                    #yield Timer(100)
                #yield Trigger(STORAGE_FINGER_RIGHT_INIT)

                f_listArmPosition = open("/tmp/traj.txt", "w")
                f_listArmPosition.write(str(self.l_armPosition))
                f_listArmPosition.close()


    def on_start(self, packet):
        if packet.value == 0:
            #self.yield_at(89500, EndOfMatch())
            #yield MoveLineTo(1.0, 1.5)

            #yield Trigger(RIGHT_BUILDER_PLIERS_RIGHT_INIT, RIGHT_BUILDER_GRIPPER_LEFT_INIT)
            #yield Trigger(makeServoMoveCommand(LEFT_BUILDER_PLIERS_LEFT, 500))

            ## Position reading
            ###################
            #print("STORAGE_FINGER_LEFT")
            #rp = yield ReadServoPosition(STORAGE_FINGER_LEFT_ID)
            #print(rp.value)
            #print("STORAGE_FINGER_LEFT_FRONT")
            #rp = yield ReadServoPosition(STORAGE_FINGER_LEFT_FRONT_ID)
            #print(rp.value)
            #print("STORAGE_FINGER_RIGHT_FRONT")
            #rp = yield ReadServoPosition(STORAGE_FINGER_RIGHT_FRONT_ID)
            #print(rp.value)
            #print("STORAGE_FINGER_RIGHT")
            #rp = yield ReadServoPosition(STORAGE_FINGER_RIGHT_ID)
            #print(rp.value)

            #print("ARM_3")
            #rp = yield ReadServoPosition(ARM_3_ID)
            #print(rp.value)
            #print("STONE_GUIDE")
            #rp = yield ReadServoPosition(STONE_GUIDE_ID)
            #print(rp.value)

            ## Moves
            #######
            #yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
            #yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), 800))
            #yield Trigger(STONE_GUIDE_HOLD)
            #yield Trigger(STORAGE_FINGER_LEFT_INIT, STORAGE_FINGER_LEFT_FRONT_INIT, STORAGE_FINGER_RIGHT_FRONT_INIT, STORAGE_FINGER_RIGHT_INIT)
            #yield Trigger(STORAGE_FINGER_LEFT_OPEN, STORAGE_FINGER_LEFT_FRONT_OPEN, STORAGE_FINGER_RIGHT_FRONT_OPEN, STORAGE_FINGER_RIGHT_OPEN)

            #l_servoID = [LEFT_BUILDER_PLIERS_LEFT_ID, LEFT_BUILDER_PLIERS_RIGHT_ID, LEFT_BUILDER_GRIPPER_LEFT_ID, LEFT_BUILDER_GRIPPER_RIGHT_ID, LEFT_BUILDER_LIGHTER_ID]

            #yield ServoTorqueControl(l_servoID, False)

            #ArmPosition = yield ReadArmServoPosition(l_servoID)
            
            #yield MoveArmServoPosition(ArmPosition.l_servoPosition)

            rt = yield GetInputStatus(MAIN_INPUT_TEAM)
            self.rec = rt.value

            if self.rec == 1:
                for servoID in self.l_servoID:
                    yield Trigger(makeServoSetupCommand((servoID, 1000), 200))
                #yield Trigger(STONE_GUIDE_HOLD)

                f_listArmPosition = open("/tmp/traj.txt", "r")
                trajArm_fromFile = eval(f_listArmPosition.read())
                f_listArmPosition.close()

                yield Trigger(STORAGE_FINGER_LEFT_HOLD)
                #for l_servoPosition in self.l_armPosition:
                for l_servoPosition in trajArm_fromFile:
                    yield MoveArmServoPosition(l_servoPosition)
                    yield Timer(1000)
                yield Trigger(STORAGE_FINGER_LEFT_INIT)
            else:
                print("Erase arm traj")
                self.l_armPosition = []
            None






class Initialize(State):

    def on_enter(self):
        #yield Trigger(LEFT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_CLOSE)
        #yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        #yield None
        None




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
            l_ServoCommand.append(makeServoMoveCommand((servoID, ARM_SERVO_TIMEOUT), position))
            
        yield Trigger(l_ServoCommand)
        yield None


##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        #yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        tools.on_end_of_match()
