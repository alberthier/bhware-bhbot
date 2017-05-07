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

TEAM_BLUE = TEAM_LEFT
TEAM_YELLOW = TEAM_RIGHT

class ReadArmTrajTxt(State):
    def on_enter(self):
        rt = yield GetInputStatus(MAIN_INPUT_TEAM)
        rec = rt.value

        if rec == 1:
            l_armServoID = [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]
            for servoID in l_armServoID:
                # Speed control
                yield Trigger(makeServoSetupCommand((servoID, 1000), 200))
            #yield Trigger(STONE_GUIDE_HOLD)
            
            f_listArmPosition = open("/tmp/traj.txt", "r")
            trajArm_fromFile = eval(f_listArmPosition.read())
            f_listArmPosition.close()
            
            #yield Trigger(STORAGE_FINGER_LEFT_HOLD)
            for l_servoPosition in trajArm_fromFile:
                yield MoveArmServoPosition(l_servoPosition)
                yield Timer(1000)
            #yield Trigger(STORAGE_FINGER_LEFT_INIT)
        yield None
                

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
            
        # Arm tests
        self.cnt_arm_action = 0
            
        gm=self.robot.goal_manager

        G = goalmanager.GoalBuilder

        gm.add(
            G("POLY_ROCKET")
                .weight(20)
                .coords(1.25, 0.3+0.15)
                .direction(DIRECTION_FORWARD)
                .state(PolyRocket, [True])
                .build(),
            G("MONO_ROCKET")
                .weight(20)
                .coords(0.350+0.15, 1.150)
                .direction(DIRECTION_FORWARD)
                .state(MonoRocket, [True])
                .build(),
            G("CENTRAL_MOON_BASE_LAT_BRANCH")
                .weight(20)
                .coords(1.210, 1.130)
                .direction(DIRECTION_AUTO)
                .state(CentralMoonBaseLatBranch, [True])
                .build(),
            G("CRATER")
                .weight(20)
                .coords(0.795, 0.843)
                .direction(DIRECTION_BACKWARDS)
                .state(Crater, [True])
                .build(),
            G("STONE_DROP")
                .weight(1)
                .coords(0.450, 0.944)
                .direction(DIRECTION_BACKWARDS)
                .state(StoneDrop, [True])
                .build()
                )
                
        None


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_READY:
            #yield Initialize()
            #yield AntiBlocking(True)
            rt = yield GetInputStatus(MAIN_INPUT_TEAM)
            self.rec = rt.value

            self.l_servoID = [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]
            
            for servoID in self.l_servoID:
                # Speed control
                servo_speed = 200
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))
            
            yield CalibratePosition()

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

            #readArmTrajTxt()
            
            # Test of arm traj sequence start by start
            
            #~ yield Trigger(ARM_7_HOLD)
            #~ self.cnt_arm_action = 10
            
            if self.cnt_arm_action == 0:
                yield InitArm()
            elif self.cnt_arm_action == 1:
                #~ yield TestGrabModuleFromInit()
                #~ yield TestStockModuleFromGrabbedModuleRightFront()
                #~ yield TestTakeModuleFromStorageReturn()
                #~ yield TestStockModuleFromGrabbedModuleRight()
                #~ yield TestTakeModuleFromStorageReturn()
                #~ yield TestStockModuleFromGrabbedModuleLeftFront()
                #~ yield TestTakeModuleFromStorageReturn()
                #~ yield TestStockModuleFromGrabbedModuleLeft()
                
                team = TEAM_YELLOW
                
                yield GrabPolyModuleFromInit(team)
                yield OutputPolyModuleFromRocket(team)
                yield DropPolyModule(team, kick=False)
                
                yield GrabPolyModuleFromDropZone(team, previousModuleTurned=False)
                yield OutputPolyModuleFromRocket(team)
                yield DropTurnedPolyModule(team, finalKick=False)
                
                yield GrabPolyModuleFromDropZone(team, previousModuleTurned=True)
                yield OutputPolyModuleFromRocket(team)
                yield DropPolyModule(team, kick=True)
                
                yield GrabPolyModuleFromDropZone(team, previousModuleTurned=False)
                yield OutputPolyModuleFromRocket(team)
                yield DropTurnedPolyModule(team, finalKick=True)
                
            elif self.cnt_arm_action == 2:
                #yield TestTakeModuleFromStorageReturn()
                #yield TestStockModuleFromGrabbedModuleRight()
                #yield TestStockModuleFromGrabbedModuleRightFront()
                #yield DropPolyModule()
                #yield GrabPolyModuleFromDropZone()
                yield DropTurnedPolyModule()
            elif self.cnt_arm_action == 3:
                None
                
            self.cnt_arm_action = self.cnt_arm_action + 1
            
                
            #~ yield ExecuteGoals()
                
            #~ yield StaticStrategy()
                
            None

class ReadArmTraj(State):
    def __init__(self, arm_traj, delay = 1000, reverse = False):
        self.arm_traj = arm_traj
        self.delay = delay
        self.reverse = reverse
        
    def on_enter(self):
        cnt_pos = len(self.arm_traj)
        for l_servoPosition in self.arm_traj:
            if self.reverse == True:
                l_servoPosition.reverse()
            yield MoveArmServoPosition(l_servoPosition)
            cnt_pos = cnt_pos - 1
            if cnt_pos != 0:
                yield Timer(self.delay)
        yield None

class ArmSpeed(State):
    def __init__(self, speed):
        self.speed = speed
        
    def on_enter(self):
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
            # Speed control
            yield Trigger(makeServoSetupCommand((servoID, 1000), self.speed))
        yield None

class InitArm(State):
    def on_enter(self):
        yield ArmSpeed(110)
        
        #~ arm_traj_r0 = [
        #~ [[(1, 5), 425], [(1, 207), 712], [(1, 107), 650], [(0, 204), 512], [(0, 206), 317], [(0, 105), 519]], 
        #~ [[(1, 5), 425], [(1, 207), 679], [(1, 107), 650], [(0, 204), 512], [(0, 206), 194], [(0, 105), 519]], 
        #~ [[(1, 5), 423], [(1, 207), 366], [(1, 107), 336], [(0, 204), 519], [(0, 206), 200], [(0, 105), 518]], 
        #~ [[(1, 5), 424], [(1, 207), 358], [(1, 107), 405], [(0, 204), 517], [(0, 206), 578], [(0, 105), 518]], 
        #~ [[(1, 5), 423], [(1, 207), 315], [(1, 107), 422], [(0, 204), 495], [(0, 206), 741], [(0, 105), 518]], 
        #~ [[(1, 5), 423], [(1, 207), 282], [(1, 107), 392], [(0, 204), 497], [(0, 206), 770], [(0, 105), 519]]
        #~ ]
        
        arm_traj_r1 = [
        [[(1, 5), 408], [(1, 207), 642], [(1, 107), 494], [(0, 204), 511], [(0, 206), 257], [(0, 105), 547]], 
        [[(1, 5), 409], [(1, 207), 558-50], [(1, 107), 422], [(0, 204), 511], [(0, 206), 260], [(0, 105), 547]], 
        [[(1, 5), 412], [(1, 207), 466-50], [(1, 107), 350], [(0, 204), 511], [(0, 206), 309], [(0, 105), 547]], 
        [[(1, 5), 412], [(1, 207), 369-30], [(1, 107), 293], [(0, 204), 510], [(0, 206), 309], [(0, 105), 547]], 
        [[(1, 5), 412], [(1, 207), 359], [(1, 107), 308], [(0, 204), 510], [(0, 206), 396], [(0, 105), 546]], 
        [[(1, 5), 412], [(1, 207), 342], [(1, 107), 412], [(0, 204), 511], [(0, 206), 519], [(0, 105), 546]], 
        [[(1, 5), 412], [(1, 207), 351], [(1, 107), 479], [(0, 204), 510], [(0, 206), 518], [(0, 105), 546]], 
        [[(1, 5), 414], [(1, 207), 362], [(1, 107), 480], [(0, 204), 497], [(0, 206), 801], [(0, 105), 541]]
        ]
        
        arm_traj = [
        [[(1, 5), 433], [(1, 207), 400], [(1, 107), 437], [(0, 204), 512], [(0, 206), 702], [(0, 105), 462]], 
        [[(1, 5), 436], [(1, 207), 334], [(1, 107), 401], [(0, 204), 512], [(0, 206), 720], [(0, 105), 462]], 
        [[(1, 5), 436], [(1, 207), 306], [(1, 107), 419], [(0, 204), 512], [(0, 206), 778], [(0, 105), 462]], 
        #[[(1, 5), 436], [(1, 207), 299], [(1, 107), 458], [(0, 204), 512], [(0, 206), 814], [(0, 105), 462]],
        [[(1, 5), 414], [(1, 207), 362], [(1, 107), 480], [(0, 204), 497], [(0, 206), 801], [(0, 105), 541]]
        ]
        yield ReadArmTraj(arm_traj, 700)
        yield Trigger(makeServoSetupCommand(((0,205), 1000), 1000))
        yield Trigger(ARM_7_INIT)
        yield None

class TestGrabModuleFromInit(State):
    def on_enter(self):
        arm_traj_by_side = [
        [[(1, 5), 301], [(1, 207), 273], [(1, 107), 391], [(0, 204), 495], [(0, 206), 764], [(0, 105), 517]], 
        [[(1, 5), 301], [(1, 207), 253], [(1, 107), 382], [(0, 204), 496], [(0, 206), 471], [(0, 105), 518]], 
        [[(1, 5), 301], [(1, 207), 347], [(1, 107), 320], [(0, 204), 496], [(0, 206), 315], [(0, 105), 518]], 
        [[(1, 5), 301], [(1, 207), 445], [(1, 107), 303], [(0, 204), 496], [(0, 206), 207], [(0, 105), 518]], 
        [[(1, 5), 335], [(1, 207), 370], [(1, 107), 167], [(0, 204), 495], [(0, 206), 161], [(0, 105), 518], [(0, 205), 200]], 
        [[(1, 5), 389], [(1, 207), 371], [(1, 107), 162], [(0, 204), 499], [(0, 206), 161], [(0, 105), 528]], 
        [[(1, 5), 416], [(1, 207), 553], [(1, 107), 340], [(0, 204), 500], [(0, 206), 173], [(0, 105), 512]], 
        [[(1, 5), 447], [(1, 207), 603], [(1, 107), 420], [(0, 204), 500], [(0, 206), 185], [(0, 105), 477], [(0, 205), 505]]
        ]
        
        yield Trigger(ARM_7_OPEN)
        yield ArmSpeed(110)
                
        ARM_2_shift_up = 10
        arm_traj_1 = [
        [[(1, 5), 423], [(1, 207), 339], [(1, 107), 511], [(0, 204), 494], [(0, 206), 718], [(0, 105), 516]], 
        [[(1, 5), 415], [(1, 207), 283], [(1, 107), 359], [(0, 204), 481], [(0, 206), 617], [(0, 105), 512]], 
        #[[(1, 5), 415], [(1, 207), 296], [(1, 107), 352], [(0, 204), 480], [(0, 206), 500], [(0, 105), 512]], 
        [[(1, 5), 415], [(1, 207), 296-ARM_2_shift_up], [(1, 107), 352+10], [(0, 204), 480], [(0, 206), 420], [(0, 105), 512]], 
        #[[(1, 5), 413], [(1, 207), 351], [(1, 107), 316], [(0, 204), 481], [(0, 206), 388], [(0, 105), 512]], 
        [[(1, 5), 413], [(1, 207), 340-ARM_2_shift_up], [(1, 107), 325+40], [(0, 204), 481], [(0, 206), 350], [(0, 105), 512]], 
        #[[(1, 5), 410], [(1, 207), 371], [(1, 107), 275], [(0, 204), 481], [(0, 206), 301], [(0, 105), 512]], 
        [[(1, 5), 410], [(1, 207), 350], [(1, 107), 300+25], [(0, 204), 481], [(0, 206), 280], [(0, 105), 512]], 
        #[[(1, 5), 410], [(1, 207), 463], [(1, 107), 324], [(0, 204), 481], [(0, 206), 260], [(0, 105), 512]], 
        [[(1, 5), 410], [(1, 207), 450], [(1, 107), 335+20], [(0, 204), 481], [(0, 206), 260], [(0, 105), 512]]
        ###[[(1, 5), 406], [(1, 207), 559], [(1, 107), 348], [(0, 204), 484], [(0, 206), 168], [(0, 105), 512]] ,
        ]
        yield ReadArmTraj(arm_traj_1, delay=200, reverse=True)
        
        yield ArmSpeed(200)
        yield Timer(100)
        arm_traj_2 = [
        ####[[(1, 5), 441], [(1, 207), 618], [(1, 107), 400], [(0, 204), 484], [(0, 206), 161], [(0, 105), 472]]
        [[(1, 5), 406], [(1, 207), 600], [(1, 107), 423], [(0, 204), 481], [(0, 206), 238], [(0, 105), 511]], 
        #[[(1, 5), 408], [(1, 207), 674], [(1, 107), 511], [(0, 204), 480], [(0, 206), 237], [(0, 105), 512]], 
        [[(1, 5), 409], [(1, 207), 711], [(1, 107), 560], [(0, 204), 480], [(0, 206), 239], [(0, 105), 512]], 
        [[(1, 5), 427], [(1, 207), 738], [(1, 107), 597], [(0, 204), 491], [(0, 206), 245], [(0, 105), 506]]
        ]
        yield ReadArmTraj(arm_traj_2, delay=200, reverse=True)
        yield Timer(50)
        yield Trigger(ARM_7_HOLD)
        yield Timer(100)
        yield ArmSpeed(200)
        yield ReadArmTraj([
        [[(1, 5), 423], [(1, 207), 631], [(1, 107), 441], [(0, 204), 498], [(0, 206), 208], [(0, 105), 506]],
        [[(1, 5), 426], [(1, 207), 498], [(1, 107), 279], [(0, 204), 497], [(0, 206), 160], [(0, 105), 494]]
        ], 100)
        yield None
        
class GrabPolyModuleFromInit(State):
    def __init__(self, team):
        self.team = team
        
    def on_enter(self):
        if self.team == TEAM_BLUE:
            yield ArmSpeed(250)
                    
            yield ReadArmTraj([[[(0, 206), 682]]])
            yield Trigger(ARM_7_OPEN)
            arm_traj_1 = [
            [[(1, 5), 419], [(1, 207), 290], [(1, 107), 381+40], [(0, 204), 500], [(0, 206), 682-50], [(0, 105), 529]],
            [[(1, 5), 423], [(1, 207), 463], [(1, 107), 227+40], [(0, 204), 500], [(0, 206), 169-20], [(0, 105), 512]],
            ]
            yield ReadArmTraj(arm_traj_1, delay=300, reverse=True)
            
            yield Timer(100)
            yield ArmSpeed(110+20)
            
            arm_traj_2 = [
            [[(1, 5), 506], [(1, 207), 565], [(1, 107), 344], [(0, 204), 501], [(0, 206), 174], [(0, 105), 427]], 
            [[(1, 5), 529], [(1, 207), 612], [(1, 107), 402], [(0, 204), 511], [(0, 206), 175], [(0, 105), 410]], 
            [[(1, 5), 537], [(1, 207), 656], [(1, 107), 472], [(0, 204), 511], [(0, 206), 204], [(0, 105), 410]], 
            [[(1, 5), 559], [(1, 207), 668], [(1, 107), 498], [(0, 204), 511], [(0, 206), 222], [(0, 105), 403]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=200, reverse=True)
            
            yield Timer(50)
            yield Trigger(ARM_7_HOLD)
          
        if self.team == TEAM_YELLOW:
            yield ArmSpeed(250+50)
                    
            yield ReadArmTraj([[[(0, 206), 682]]])
            yield Trigger(ARM_7_OPEN)
            arm_traj = [
            [[(1, 5), 419], [(1, 207), 290], [(1, 107), 381+40+20], [(0, 204), 500], [(0, 206), 169-20], [(0, 105), 512]],
            [[(1, 5), 423], [(1, 207), 386], [(1, 107), 183], [(0, 204), 499], [(0, 206), 161], [(0, 105), 512]],
            [[(1, 5), 320]]
            ]
            yield ReadArmTraj(arm_traj, delay=300, reverse=True)
            
            yield Timer(100)
            
            arm_traj = [
            [[(1, 5), 326], [(1, 207), 557], [(1, 107), 338], [(0, 204), 496], [(0, 206), 163], [(0, 105), 512]], 
            [[(1, 5), 333], [(1, 207), 638], [(1, 107), 460], [(0, 204), 496], [(0, 206), 210], [(0, 105), 512]]
            ]
            yield ReadArmTraj(arm_traj, delay=50, reverse=True)
            yield Timer(50)
            arm_traj = [
            [[(1, 5), 304], [(1, 207), 678], [(1, 107), 504], [(0, 204), 495], [(0, 206), 207], [(0, 105), 617]],
            [[(1, 5), 311], [(1, 207), 663], [(1, 107), 476], [(0, 204), 492], [(0, 206), 203], [(0, 105), 614]]
            ]
            yield ReadArmTraj(arm_traj, delay=200, reverse=True)
            
            yield Timer(100)
            yield ArmSpeed(110)
            
            yield Trigger(ARM_7_HOLD)
            
        yield None
        
        
class OutputPolyModuleFromRocket(State):
    def __init__(self, team):
        self.team = team
        
    def on_enter(self):
        if self.team == TEAM_BLUE:
            yield ArmSpeed(110)
            arm_traj = [
            [[(1, 5), 574], [(1, 207), 581], [(1, 107), 378], [(0, 204), 511], [(0, 206), 195], [(0, 105), 383]], 
            [[(1, 5), 618-10], [(1, 207), 470], [(1, 107), 259], [(0, 204), 509], [(0, 206), 166], [(0, 105), 322]], 
            ]
            yield ReadArmTraj(arm_traj, delay=700, reverse=True)
            yield Timer(550)
            yield ArmSpeed(400)
            
            yield ReadArmTraj([[[(1, 5), 451+30]]], reverse=True)
            
            yield Timer(200)
            
        if self.team == TEAM_YELLOW:
            #~ yield ArmSpeed(110)
            arm_traj = [
            #[[(1, 5), 301], [(1, 207), 537], [(1, 107), 319], [(0, 204), 495], [(0, 206), 160], [(0, 105), 597]]
            [[(1, 5), 301], [(1, 207), 594], [(1, 107), 365], [(0, 204), 511], [(0, 206), 160], [(0, 105), 647]]
            ]
            yield ReadArmTraj(arm_traj, delay=700, reverse=True)
            
            yield Timer(400)
            yield ArmSpeed(400)
            
            arm_traj = [
            [[(1, 5), 380]]
            ]
            yield ReadArmTraj(arm_traj)
            
            yield Timer(200)
            
        yield None
        
class DropPolyModule(State):
    def __init__(self, team, kick=True):
        self.team = team
        self.kick = kick
        
    def on_enter(self): 
        if self.team == TEAM_BLUE:
            yield ArmSpeed(180)
            arm_traj = [
                [[(1, 5), 377], [(1, 207), 435], [(1, 107), 313], [(0, 204), 496], [(0, 206), 252-5], [(0, 105), 421]], 
                [[(1, 5), 280], [(1, 207), 612], [(1, 107), 473], [(0, 204), 494], [(0, 206), 233], [(0, 105), 590]],
                [[(1, 5), 287-20], [(1, 207), 699], [(1, 107), 617], [(0, 204), 494], [(0, 206), 296], [(0, 105), 478+20+20]]
                ]
            if self.kick == True:
                yield ReadArmTraj(arm_traj, delay=300-100-50)#, reverse=True)
                yield Timer(500)
            else:
                yield ReadArmTraj([arm_traj[0]])#, reverse=True)
                
            yield Timer(500-200)
            yield ArmSpeed(200)
                
            # Reach drop position
            arm_traj = [
            [[(1, 5), 313], [(1, 207), 519], [(1, 107), 435], [(0, 204), 507], [(0, 206), 282-30], [(0, 105), 610]]
            ]
            yield ReadArmTraj(arm_traj, delay=400, reverse=True)
            
            yield Timer(400-100  -100)
            
            yield ArmSpeed(110-50*0)
            
            ARM_7_DROP_int = [(0, 205), 162+140]
            arm_traj = [
            [ARM_7_DROP_int],
            [[(1, 5), 306-30], [(1, 207), 470], [(1, 107), 498], [(0, 204), 509], [(0, 206), 395], [(0, 105), 619], ARM_7_DROP_int]
            ]
            yield ReadArmTraj(arm_traj, delay=400, reverse=True)
            yield Timer(200-100)
            
        if self.team == TEAM_YELLOW:
            yield ArmSpeed(110+40)
            arm_traj_1 = [
            [[(1, 207), 441], [(1, 107), 323+20+20], [(0, 204), 484], [(0, 206), 240], [(0, 105), 560]]
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)
            yield Timer(300)
            arm_traj_1 = [
            [[(1, 5), 412], [(0, 105), 450]], 
            [[(1, 5), 529], [(1, 207), 474], [(1, 107), 342+25+10 -5], [(0, 204), 494], [(0, 206), 231], [(0, 105), 397-24]], 
            #[[(1, 5), 551-10], [(1, 207), 612], [(1, 107), 481+30], [(0, 204), 494], [(0, 206), 233], [(0, 105), 376]]
            [[(1, 5), 546], [(1, 207), 590], [(1, 107), 455+30 -20], [(0, 204), 494], [(0, 206), 245], [(0, 105), 373]] 
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)
            if self.kick == True:
                arm_traj_2 = [
                [[(1, 5), 596], [(1, 207), 740], [(1, 107), 678], [(0, 204), 491], [(0, 206), 327], [(0, 105), 372]], 
                [[(1, 5), 581], [(1, 207), 616], [(1, 107), 511], [(0, 204), 499], [(0, 206), 275], [(0, 105), 380]], 
                #[[(1, 5), 546], [(1, 207), 577], [(1, 107), 454], [(0, 204), 498], [(0, 206), 245], [(0, 105), 402]]
                arm_traj_1[-1]
                ]
                yield ReadArmTraj(arm_traj_2, delay=1000)#, reverse=True)
                yield Timer(1000)
                
                
            yield Timer(500)
            yield Trigger(ARM_7_OPEN)
            #~ ARM_7_OPEN_int = [(0, 205), 162]
            arm_traj = [
            #~ [ARM_7_OPEN_int],
            [[(0, 105), 376+50]]
            ]
            #~ yield ReadArmTraj(arm_traj, delay=200, reverse=True)
            
        yield None
        
class DropTurnedPolyModule(State):
    def __init__(self, team, finalKick=False):
        self.team = team
        self.finalKick = finalKick
        
    def on_enter(self):        
        if self.team == TEAM_BLUE:
            yield ArmSpeed(110+20+30)
            # Kick dropped modules
            arm_traj = [
            [[(1, 5), 377], [(1, 207), 435], [(1, 107), 313], [(0, 204), 496], [(0, 206), 252-5], [(0, 105), 421]], 
            [[(1, 5), 280], [(1, 207), 612], [(1, 107), 473], [(0, 204), 494], [(0, 206), 233], [(0, 105), 590]],
            [[(1, 5), 287], [(1, 207), 699], [(1, 107), 617], [(0, 204), 494], [(0, 206), 296], [(0, 105), 478]]
            ]
            yield ReadArmTraj(arm_traj, delay=300-100-50)#, reverse=True)
            
            yield Timer(500-100)
            
            yield ArmSpeed(110+40)
            ARM_7_DROP_int = [(0, 205), 438]
            arm_traj = [
            [[(1, 5), 251], [(1, 207), 656], [(1, 107), 585], [(0, 204), 424], [(0, 206), 310-2], [(0, 105), 548]],
            [[(1, 5), 235-10], [(1, 207), 728], [(1, 107), 730], [(0, 204), 286], [(0, 206), 294], [(0, 105), 542]],
            [ARM_7_DROP_int],
            [[(1, 5), 260], [(1, 207), 527], [(1, 107), 525], [(0, 204), 338], [(0, 206), 298], [(0, 105), 556], ARM_7_DROP_int],
            [[(1, 5), 260-40]]
            ]
            yield ReadArmTraj(arm_traj, delay=500-200, reverse=True)
            yield Timer(300-200)
            
        if self.team == TEAM_YELLOW:
            yield ArmSpeed(150)
            # Kick dropped modules and then drop
            #~ ARM_7_DROP_int = [(0, 205), 438]
            #~ arm_traj = [
            #~ [[(1, 5), 391], [(1, 207), 411], [(1, 107), 340+20], [(0, 204), 511], [(0, 206), 298], [(0, 105), 619]],
            #~ [[(1, 5), 586], [(1, 207), 656], [(1, 107), 575], [(0, 204), 491], [(0, 206), 306], [(0, 105), 396]], 
            #~ [[(1, 5), 587], [(1, 207), 693], [(1, 107), 635], [(0, 204), 491], [(0, 206), 324], [(0, 105), 511]], 
            #~ [[(1, 5), 605], [(1, 207), 694], [(1, 107), 642], [(0, 204), 530], [(0, 206), 336], [(0, 105), 372]], 
            #~ [[(1, 5), 636], [(1, 207), 646], [(1, 107), 620], [(0, 204), 772], [(0, 206), 294], [(0, 105), 547]], 
            #~ [[(1, 5), 597], [(1, 207), 740], [(1, 107), 723], [(0, 204), 836], [(0, 206), 312], [(0, 105), 722]], 
            #~ [ARM_7_DROP_int],
            #~ [[(1, 5), 586], [(1, 207), 679], [(1, 107), 595], [(0, 204), 797], [(0, 206), 211], [(0, 105), 720]]
            #~ ]
            arm_traj_1 = [
            [[(1, 207), 441], [(1, 107), 323+20+7], [(0, 204), 484], [(0, 206), 240], [(0, 105), 560]],
            [[(1, 5), 412], [(0, 105), 450]], 
            [[(1, 5), 529], [(1, 207), 474], [(1, 107), 342+25+1], [(0, 204), 494], [(0, 206), 231], [(0, 105), 397-24]], 
            #[[(1, 5), 551-10], [(1, 207), 612], [(1, 107), 481+30], [(0, 204), 494], [(0, 206), 233], [(0, 105), 376]]
            [[(1, 5), 546], [(1, 207), 590], [(1, 107), 455+30], [(0, 204), 494], [(0, 206), 245], [(0, 105), 373]] 
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)#, reverse=True)
            
            #~ if self.kick == True:
                #~ arm_traj_2 = [
                #~ [[(1, 5), 598], [(1, 207), 698], [(1, 107), 636], [(0, 204), 511], [(0, 206), 340], [(0, 105), 348]], 
                #~ #[[(1, 5), 546], [(1, 207), 565], [(1, 107), 444], [(0, 204), 510], [(0, 206), 264], [(0, 105), 412]]
                #~ arm_traj_1[-1]
                #~ ]
                #~ yield ReadArmTraj(arm_traj_2, delay=1000)#, reverse=True)
                
            yield Timer(1000)
            shift_up = 30
            arm_traj_2 = [
            [[(0, 105), 360-5]],
            [[(1, 5), 579], [(1, 207), 662], [(1, 107), 560+shift_up], [(0, 204), 494], [(0, 206), 285]],# [(0, 105), 360]]
            #~ [[(1, 5), 587], [(1, 207), 637], [(1, 107), 544+shift_up], [(0, 204), 541], [(0, 206), 296], [(0, 105), 383]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=1000, reverse=True)
            
            yield Timer(1000)
            yield Trigger(ARM_7_DROP)
            
            yield Timer(1000)
            yield ArmSpeed(100)
            arm_traj_2 = [
            [[(1, 5), 576], [(1, 207), 592], [(1, 107), 544+shift_up], [(0, 204), 504], [(0, 206), 330], [(0, 105), 362]],
            #[[(1, 5), 529], [(1, 207), 466], [(1, 107), 422+shift_up], [(0, 204), 518], [(0, 206), 317], [(0, 105), 431]],
            [[(1, 5), 535], [(1, 207), 469], [(1, 107), 426], [(0, 204), 504], [(0, 206), 329], [(0, 105), 387]],
            [[(1, 107), 426+shift_up+20]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=100, reverse=True)
            
            yield Timer(200)
            # yield Trigger(ARM_7_OPEN)
            # yield Timer(500)
            arm_traj_2 = [
            #[[(1, 5), 443], [(1, 207), 387], [(1, 107), 400+shift_up+40+20], [(0, 204), 512], [(0, 206), 367], [(0, 105), 497]]
            [[(1, 207), 387], [(1, 107), 400+shift_up+40], [(0, 204), 512], [(0, 206), 367], [(0, 105), 497]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=100, reverse=True)
            
            if self.finalKick == True:
                yield Timer(1000)
                yield Trigger(ARM_7_INIT)
                #~ yield Timer(500)
                arm_traj = [
                [[(1, 5), 433]],#[(1, 207), 366], [(1, 107), 425], [(0, 204), 511], [(0, 206), 386], [(0, 105), 504]
                [[(1, 207), 437], [(1, 107), 366+40], [(0, 204), 512], [(0, 206), 363], [(0, 105), 503]], #[(1, 5), 433],
                #[[(1, 5), 496+20]], #[(1, 207), 496], [(1, 107), 419], [(0, 204), 512], [(0, 206), 363], [(0, 105), 502]], 
                [[(1, 5), 551], [(1, 207), 546], [(1, 107), 542], [(0, 204), 571], [(0, 206), 450], [(0, 105), 412]],
                [[(1, 5), 434]] #[(1, 207), 434], [(1, 107), 369], [(0, 204), 511], [(0, 206), 363], [(0, 105), 502]]
                ]
                yield ReadArmTraj(arm_traj, delay=1000)
            
        yield None
        
class GrabPolyModuleFromDropZone(State):
    def __init__(self, team, previousModuleTurned=False):
        self.team = team
        self.previousModuleTurned = previousModuleTurned
        
    def on_enter(self):
        if self.team == TEAM_BLUE:
            yield Trigger(ARM_7_OPEN)
            yield ArmSpeed(110+40+10)
            arm_traj = [
            [[(1, 5), 284], [(1, 207), 371], [(1, 107), 336], [(0, 204), 476], [(0, 206), 345], [(0, 105), 634]], 
            [[(1, 5), 356], [(1, 207), 364], [(1, 107), 231], [(0, 204), 476], [(0, 206), 250], [(0, 105), 555+30]], 
            [[(1, 5), 425], [(1, 207), 411], [(1, 107), 235], [(0, 204), 476], [(0, 206), 203], [(0, 105), 479+30]], 
            [[(1, 5), 480], [(1, 207), 567], [(1, 107), 329], [(0, 204), 475], [(0, 206), 160], [(0, 105), 422+30]], 
            [[(1, 5), 525], [(1, 207), 644], [(1, 107), 427], [(0, 204), 476], [(0, 206), 166], [(0, 105), 380+20]]
            ]
            yield ReadArmTraj(arm_traj, delay=100, reverse=True)
            yield Timer(200)
            arm_traj = [
            [[(1, 5), 546], [(1, 207), 696], [(1, 107), 529], [(0, 204), 483], [(0, 206), 214], [(0, 105), 369]]
            ]
            yield ReadArmTraj(arm_traj, delay=100, reverse=True)
            yield Trigger(ARM_7_HOLD)
            
        if self.team == TEAM_YELLOW:
            yield Trigger(ARM_7_OPEN)
            if self.previousModuleTurned == False:
                yield ArmSpeed(300)
                arm_traj_1 = [
                [[(1, 5), 561], [(1, 207), 519], [(1, 107), 386], [(0, 204), 493], [(0, 206), 257], [(0, 105), 397-20]], 
                [[(1, 5), 546], [(1, 207), 353], [(1, 107), 201], [(0, 204), 473], [(0, 206), 159], [(0, 105), 426]], 
                [[(1, 5), 284]],# [(1, 207), 354], [(1, 107), 205], [(0, 204), 473], [(0, 206), 159], [(0, 105), 518]], 
                ]
                yield ReadArmTraj(arm_traj_1, delay=100, reverse=True)
                yield Timer(500)
                arm_traj_1 = [
                [[(1, 5), 286], [(1, 207), 609], [(1, 107), 382], [(0, 204), 494], [(0, 206), 159], [(0, 105), 571]], 
                [[(1, 5), 296], [(1, 207), 644], [(1, 107), 446], [(0, 204), 494], [(0, 206), 191], [(0, 105), 629]]
                ]
                yield ReadArmTraj(arm_traj_1, delay=100, reverse=True)
                yield Timer(50)
                
            if self.previousModuleTurned == True:
                yield ArmSpeed(150)
                arm_traj_1 = [
                [[(1, 5), 550], [(1, 207), 272], [(1, 107), 380], [(0, 204), 511], [(0, 206), 481], [(0, 105), 418]], 
                [[(1, 5), 416], [(1, 207), 255], [(1, 107), 375], [(0, 204), 510], [(0, 206), 497], [(0, 105), 440]], 
                [[(1, 5), 232], [(1, 207), 257], [(1, 107), 373], [(0, 204), 495], [(0, 206), 499], [(0, 105), 640]], 
                [[(1, 5), 241], [(1, 207), 345], [(1, 107), 320], [(0, 204), 484], [(0, 206), 361], [(0, 105), 656]], 
                [[(1, 5), 241], [(1, 207), 507], [(1, 107), 308], [(0, 204), 483], [(0, 206), 201], [(0, 105), 656]]
                ]
                yield ReadArmTraj(arm_traj_1, delay=1000, reverse=True)
                yield Timer(1000)
                
            arm_traj = [
            [[(1, 5), 304], [(1, 207), 678], [(1, 107), 504], [(0, 204), 495], [(0, 206), 207], [(0, 105), 617]],
            [[(1, 5), 311], [(1, 207), 663], [(1, 107), 476], [(0, 204), 492], [(0, 206), 203], [(0, 105), 614]]
            ]
            yield ReadArmTraj(arm_traj, delay=200, reverse=True)
            
            yield Timer(100)
            yield ArmSpeed(110)
            
            yield Trigger(ARM_7_HOLD)

        yield None
        

class TestTakeModuleFromStorageReturn(State):
    def on_enter(self):
        yield ArmSpeed(200)
                
        arm_traj = [
        [[(1, 5), 414], [(1, 207), 444], [(1, 107), 225+20], [(0, 204), 488], [(0, 206), 177], [(0, 105), 512]], 
        [[(1, 5), 414], [(1, 207), 526], [(1, 107), 299+20], [(0, 204), 488], [(0, 206), 160], [(0, 105), 512]], 
        [[(1, 5), 413], [(1, 207), 587], [(1, 107), 376+20], [(0, 204), 488], [(0, 206), 176], [(0, 105), 512]],
        [[(1, 5), 417], [(1, 207), 693], [(1, 107), 545], [(0, 204), 488], [(0, 206), 244], [(0, 105), 502]]
        ]
        yield ReadArmTraj(arm_traj, delay=50)
        yield Timer(100)
        yield ArmSpeed(110)
        #yield ReadArmTraj([[[(1, 5), 441], [(1, 207), 626], [(1, 107), 418], [(0, 204), 488], [(0, 206), 176], [(0, 105), 469]]], delay=50)
        yield ReadArmTraj([[[(1, 5), 427], [(1, 207), 701], [(1, 107), 551], [(0, 204), 488], [(0, 206), 241], [(0, 105), 502]]])
        yield Timer(100)
        yield Trigger(ARM_7_HOLD)
        yield Timer(100)
        yield ArmSpeed(200)
        yield ReadArmTraj([
        [[(1, 5), 423], [(1, 207), 631], [(1, 107), 441], [(0, 204), 498], [(0, 206), 208], [(0, 105), 506]],
        [[(1, 5), 426], [(1, 207), 498], [(1, 107), 279], [(0, 204), 497], [(0, 206), 160], [(0, 105), 494]]
        ], 100)
        yield None


class TestStockModuleFromGrabbedModuleRightFront(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_OPEN)
        
        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(200)
        yield ReadArmTraj([[[ARM_1_ID, 317]]], delay=100)
        
        # Montee du module a vitesse plus lente
        yield ArmSpeed(110)
        
        arm_traj_up = [
        [[(1, 5), 312], [(1, 207), 500], [(1, 107), 376], [(0, 204), 500], [(0, 206), 240], [(0, 105), 487]], 
        [[(1, 5), 286], [(1, 207), 442], [(1, 107), 358], [(0, 204), 484], [(0, 206), 297], [(0, 105), 553]]
        ]
        yield ReadArmTraj(arm_traj_up, 100)
        
        arm_traj_up = [
        [[(1, 5), 280], [(1, 207), 381], [(1, 107), 369], [(0, 204), 484], [(0, 206), 357], [(0, 105), 590]], 
        [[(1, 5), 277], [(1, 207), 356], [(1, 107), 424], [(0, 204), 485], [(0, 206), 434], [(0, 105), 641]], 
        [[(1, 5), 272], [(1, 207), 339], [(1, 107), 469], [(0, 204), 485], [(0, 206), 496], [(0, 105), 668]],
        [[(1, 5), 266], [(1, 207), 366], [(1, 107), 533], [(0, 204), 490], [(0, 206), 524], [(0, 105), 680]], 
        [[(1, 5), 260], [(1, 207), 409], [(1, 107), 596], [(0, 204), 491], [(0, 206), 551], [(0, 105), 690]], 
        [[(1, 5), 260], [(1, 207), 423], [(1, 107), 619], [(0, 204), 490], [(0, 206), 557], [(0, 105), 690]]
        ]
        yield ReadArmTraj(arm_traj_up, 10)
        
        yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), arm_traj_up[-1][2][1] + 35))
        
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_HOLD)
        
        yield Timer(300)
        yield Trigger(ARM_7_DROP)
                
        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 257], [(1, 207), 397], [(1, 107), 585], [(0, 204), 421], [(0, 206), 555], [(0, 105), 703]], 
        [[(1, 5), 250], [(1, 207), 315], [(1, 107), 460], [(0, 204), 421], [(0, 206), 555], [(0, 105), 703]],
        [[(1, 5), 250], [(1, 207), 278], [(1, 107), 361-20], [(0, 204), 411], [(0, 206), 555], [(0, 105), 668]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 100, reverse = True)
        yield Timer(100)
        arm_traj_down_2 = [
        [[(1, 5), 429], [(1, 207), 275], [(1, 107), 364], [(0, 204), 492], [(0, 206), 500], [(0, 105), 512]], 
        [[(1, 5), 421], [(1, 207), 319], [(1, 107), 334], [(0, 204), 494], [(0, 206), 320], [(0, 105), 525]], 
        [[(1, 5), 416], [(1, 207), 367], [(1, 107), 227], [(0, 204), 494], [(0, 206), 261], [(0, 105), 523]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))
        
        yield None

        
class TestStockModuleFromGrabbedModuleRight(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_OPEN)
        
        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(200)
        yield ReadArmTraj([[[ARM_1_ID, 317]]], delay=100)
        
        yield ArmSpeed(110)
        yield Timer(200)
        
        arm_traj_up = [
        [[(1, 5), 302], [(1, 207), 541], [(1, 107), 366], [(0, 204), 488], [(0, 206), 201], [(0, 105), 476]], 
        [[(1, 5), 261], [(1, 207), 466], [(1, 107), 335], [(0, 204), 486], [(0, 206), 237], [(0, 105), 549]], 
        [[(1, 5), 216], [(1, 207), 436], [(1, 107), 350], [(0, 204), 493], [(0, 206), 275], [(0, 105), 601]], 
        [[(1, 5), 175], [(1, 207), 392], [(1, 107), 352], [(0, 204), 520], [(0, 206), 304], [(0, 105), 617]], 
        [[(1, 5), 174], [(1, 207), 309], [(1, 107), 342], [(0, 204), 511], [(0, 206), 370], [(0, 105), 504]], 
        [[(1, 5), 197], [(1, 207), 320], [(1, 107), 431], [(0, 204), 511], [(0, 206), 453], [(0, 105), 449]], 
        #[[(1, 5), 197], [(1, 207), 379], [(1, 107), 560], [(0, 204), 502], [(0, 206), 526], [(0, 105), 435]]
        [[(1, 5), 210], [(1, 207), 353], [(1, 107), 522], [(0, 204), 497], [(0, 206), 509], [(0, 105), 433]]
        ]
        yield ReadArmTraj(arm_traj_up, delay = 100)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), arm_traj_up[-1][2][1] + 50))
        
        yield Timer(100)
        yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
        
        yield Timer(300)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 197], [(1, 207), 303], [(1, 107), 438], [(0, 204), 499], [(0, 206), 479], [(0, 105), 435]], 
        [[(1, 5), 197], [(1, 207), 255], [(1, 107), 405], [(0, 204), 499], [(0, 206), 483], [(0, 105), 436]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 100, reverse = True)
        yield Timer(500)
        arm_traj_down_2 = [
        [[(1, 5), 201], [(1, 207), 310], [(1, 107), 342], [(0, 204), 500], [(0, 206), 422], [(0, 105), 437]],
        [[(1, 5), 358], [(1, 207), 308], [(1, 107), 343], [(0, 204), 500], [(0, 206), 423], [(0, 105), 461]], 
        [[(1, 5), 433], [(1, 207), 371], [(1, 107), 275], [(0, 204), 499], [(0, 206), 299], [(0, 105), 491]], 
        [[(1, 5), 430], [(1, 207), 375], [(1, 107), 215], [(0, 204), 500], [(0, 206), 224], [(0, 105), 491]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))
        
        yield None

class TestStockModuleFromGrabbedModuleLeftFront(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_LEFT_FRONT_OPEN)
        
        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(200)
        yield ReadArmTraj([[[ARM_1_ID, 517]]], delay=100)
        
        # Montee du module a vitesse plus lente
        yield ArmSpeed(110)
        
        arm_traj_up = [
        [[(1, 5), 521], [(1, 207), 339], [(1, 107), 323], [(0, 204), 496], [(0, 206), 342], [(0, 105), 480]], 
        [[(1, 5), 589], [(1, 207), 327], [(1, 107), 330], [(0, 204), 497], [(0, 206), 357], [(0, 105), 354]]
        ]
        yield ReadArmTraj(arm_traj_up, 120)
        
        arm_traj_up = [
        [[(1, 5), 594], [(1, 207), 326], [(1, 107), 402], [(0, 204), 497], [(0, 206), 438], [(0, 105), 354]], 
        [[(1, 5), 606], [(1, 207), 375], [(1, 107), 515], [(0, 204), 504], [(0, 206), 498], [(0, 105), 348]], 
        [[(1, 5), 617], [(1, 207), 460], [(1, 107), 650], [(0, 204), 504], [(0, 206), 561], [(0, 105), 315]], 
        #[[(1, 5), 619], [(1, 207), 460], [(1, 107), 650], [(0, 204), 504], [(0, 206), 561], [(0, 105), 313]]
        [[(1, 5), 629], [(1, 207), 465], [(1, 107), 645], [(0, 204), 504], [(0, 206), 554], [(0, 105), 289]]
        ]
        yield ReadArmTraj(arm_traj_up, 10)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), arm_traj_up[-1][2][1] + 35))
        
        yield Timer(1000)
        yield Trigger(STORAGE_FINGER_LEFT_FRONT_HOLD)
        
        yield Timer(300)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 670], [(1, 207), 379], [(1, 107), 563], [(0, 204), 510], [(0, 206), 559], [(0, 105), 264]], 
        [[(1, 5), 658], [(1, 207), 348], [(1, 107), 497], [(0, 204), 531], [(0, 206), 656], [(0, 105), 272]],
        [[(1, 5), 670], [(1, 207), 272], [(1, 107), 398], [(0, 204), 629], [(0, 206), 512], [(0, 105), 265]], 
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 300, reverse = True)
        
        yield Timer(300)
        arm_traj_down_2 = [
        [[(1, 5), 505], [(1, 207), 278], [(1, 107), 373], [(0, 204), 628], [(0, 206), 493], [(0, 105), 265]], 
        [[(1, 5), 450], [(1, 207), 291], [(1, 107), 354], [(0, 204), 517], [(0, 206), 478], [(0, 105), 488]], 
        [[(1, 5), 442], [(1, 207), 371], [(1, 107), 233], [(0, 204), 518], [(0, 206), 266], [(0, 105), 490]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))
        
        yield None
        
class TestStockModuleFromGrabbedModuleLeft(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_LEFT_OPEN)
        
        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(200)
        yield ReadArmTraj([[[ARM_1_ID, 541]]], delay=100)
        
        # Montee du module a vitesse plus lente
        yield ArmSpeed(110)
        
        arm_traj_up = [
        [[(1, 5), 675], [(1, 207), 475], [(1, 107), 331], [(0, 204), 495], [(0, 206), 233], [(0, 105), 498]], 
        [[(1, 5), 675], [(1, 207), 336], [(1, 107), 358], [(0, 204), 496], [(0, 206), 376], [(0, 105), 518]]
        ]
        yield ReadArmTraj(arm_traj_up, 120)
        
        arm_traj_up = [
        [[(1, 5), 671], [(1, 207), 353], [(1, 107), 457], [(0, 204), 496], [(0, 206), 454], [(0, 105), 558]], 
        [[(1, 5), 670], [(1, 207), 415], [(1, 107), 582], [(0, 204), 496], [(0, 206), 525], [(0, 105), 576]], 
        [[(1, 5), 667], [(1, 207), 411], [(1, 107), 587], [(0, 204), 496], [(0, 206), 536], [(0, 105), 581]]
        ]
        yield ReadArmTraj(arm_traj_up, 10)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), arm_traj_up[-1][2][1] + 40))
        
        yield Timer(1000)
        yield Trigger(STORAGE_FINGER_LEFT_HOLD)
        
        yield Timer(300)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 664], [(1, 207), 353], [(1, 107), 527], [(0, 204), 495], [(0, 206), 535], [(0, 105), 580]], 
        [[(1, 5), 664], [(1, 207), 271], [(1, 107), 366], [(0, 204), 503], [(0, 206), 463], [(0, 105), 580]], 
        [[(1, 5), 664], [(1, 207), 314], [(1, 107), 340], [(0, 204), 502], [(0, 206), 463], [(0, 105), 579]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 300, reverse = True)
        
        yield Timer(300)
        arm_traj_down_2 = [
        [[(1, 5), 436], [(1, 207), 314], [(1, 107), 339], [(0, 204), 501], [(0, 206), 465], [(0, 105), 579]], 
        [[(1, 5), 435], [(1, 207), 364], [(1, 107), 225], [(0, 204), 512], [(0, 206), 245], [(0, 105), 512]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))
        
        yield None
        

class TestGrabModuleLeftFromInit(State):
    def on_enter(self):
        arm_traj = [
        [[(1, 5), 419], [(1, 207), 285], [(1, 107), 406], [(0, 204), 495], [(0, 206), 489], [(0, 105), 518]], 
        [[(1, 5), 419], [(1, 207), 364], [(1, 107), 238], [(0, 204), 504], [(0, 206), 247], [(0, 105), 517]], 
        [[(1, 5), 414], [(1, 207), 449], [(1, 107), 233], [(0, 204), 504], [(0, 206), 160], [(0, 105), 517], [(0, 205), 200]], 
        [[(1, 5), 517], [(1, 207), 549], [(1, 107), 334], [(0, 204), 533], [(0, 206), 163], [(0, 105), 443]], 
        [[(1, 5), 566], [(1, 207), 692], [(1, 107), 548], [(0, 204), 534], [(0, 206), 247], [(0, 105), 411], [(0, 205), 505]]
        ]
        yield ReadArmTraj(arm_traj)
        yield None


class PolyRocket(State):
    
    def __init__(self, depl = True):
        self.depl = depl
    
    def on_enter(self):
        if self.depl == True:
            yield RotateTo(-math.pi/2.0)
            yield MoveLineTo(1.25, 0.3)
            
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None

class MonoRocket(State):
    def __init__(self, depl = True):
        self.depl = depl
        
    def on_enter(self):
        if self.depl == True:
            yield RotateTo(math.pi)
            yield MoveLineTo(0.350, 1.150)
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None

class CentralMoonBaseLatBranch(State):
    def __init__(self, depl = True):
        self.depl = depl
        
    def on_enter(self):
        if self.depl == True:
            yield RotateTo(-math.pi/4.0)
            yield MoveLineTo(1.238, 1.101)
            
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None
        
class Crater(State):
    def __init__(self, depl = True):
        self.depl = depl
        
    def on_enter(self):
        if self.depl == True:
            yield LookAtOpposite(0.540, 0.650)
            angle_from_crater_center_to_robot = angle_between(0.54, 0.65, self.robot.pose.x, self.robot.pose.y)
            yield MoveLineTo(0.54 + (0.112 + ROBOT_CENTER_X) * math.cos(angle_from_crater_center_to_robot), 0.65 + (0.112 + ROBOT_CENTER_X) * math.sin(angle_from_crater_center_to_robot), direction = DIRECTION_BACKWARDS)
            
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None
        
class StoneDrop(State):
    def __init__(self, depl = True):
        self.depl = depl
        
    def on_enter(self):
        if self.depl == True:
            None
        self.exit_reason = GOAL_DONE
        yield None


class Initialize(State):

    def on_enter(self):
        #yield Trigger(LEFT_CLAPMAN_CLOSE, RIGHT_CLAPMAN_CLOSE)
        #yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        #yield None
        None




class CalibratePosition(State):

    def on_enter(self):
        yield DefinePosition(0.15, 0.923, 0.0)
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


class StaticStrategy(State):
    def on_enter(self):
        # Deplacement vers fusee polychrome bleu
        yield MoveLineTo(0.927, 0.923)
        yield RotateTo(-math.pi/4.0)
        yield MoveLineTo(1.25, 0.6)
        yield RotateTo(-math.pi/2.0)
        yield MoveLineTo(1.25, 0.3)
        
        # Travail sur fusee polychrome bleu pour dépose 4 elements
        yield PolyRocket(False)
        
        # Deplacement vers fusee bleu
        yield MoveLineTo(1.250, 0.600)
        yield RotateTo(-math.pi/4.0)
        yield MoveLineTo(0.700, 1.150)
        yield RotateTo(math.pi)
        yield MoveLineTo(0.350, 1.150)
        
        # Travail sur fusee bleu pour stockage de 3 elements et prise de 1 element
        yield MonoRocket(False)
        
        # Deplacement vers base lunaire centrale
        yield MoveLineTo(1.210, 1.130)
        yield RotateTo(-math.pi/4.0)
        yield MoveLineTo(1.238, 1.101)
        
        # Depose des elements dans branche bleu base lunaire centrale
        yield CentralMoonBaseLatBranch(False)
        
        # Deplacement vers cratere bleu
        yield MoveLineTo(1.210, 1.130)
        yield RotateTo(math.radians(37.0))
        yield MoveLineTo(0.765, 0.811)
        
        # Prise des roches dans cratere
        yield Crater(False)
        
        # Deplacement vers zone de depart bleu
        yield MoveLineTo(0.959, 0.950)
        yield RotateTo(0.0)
        yield MoveLineTo(0.450, 0.944)
        
        # Depose roche dans zone de depart bleu
        yield StoneDrop(False)
        
        # Fin du match
        
        yield None


##################################################
# End of match




class EndOfMatch(statemachine.State):

    def on_enter(self):
        self.send_packet(packets.Stop())
        #yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        tools.on_end_of_match()
