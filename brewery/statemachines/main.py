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
            if self.cnt_arm_action == 0:
                yield InitArm()
            elif self.cnt_arm_action == 1:
                yield TestGrabModuleFromInit()
                #yield TestGrabModuleLeftFromInit()
                yield TestStockModuleFromGrabbedModuleRightFront()
            elif self.cnt_arm_action == 2:
                #yield TestStockModuleFromGrabbedModuleRightFront()
                #yield TestStockModuleFromGrabbedModuleRight()
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

class InitArm(State):
    def on_enter(self):
        yield Trigger(ARM_7_INIT)
        arm_traj = [
        [[(1, 5), 425], [(1, 207), 712], [(1, 107), 650], [(0, 204), 512], [(0, 206), 317], [(0, 105), 519]], 
        [[(1, 5), 425], [(1, 207), 679], [(1, 107), 650], [(0, 204), 512], [(0, 206), 194], [(0, 105), 519]], 
        [[(1, 5), 423], [(1, 207), 366], [(1, 107), 336], [(0, 204), 519], [(0, 206), 200], [(0, 105), 518]], 
        [[(1, 5), 424], [(1, 207), 358], [(1, 107), 405], [(0, 204), 517], [(0, 206), 578], [(0, 105), 518]], 
        [[(1, 5), 423], [(1, 207), 315], [(1, 107), 422], [(0, 204), 495], [(0, 206), 741], [(0, 105), 518]], 
        [[(1, 5), 423], [(1, 207), 282], [(1, 107), 392], [(0, 204), 497], [(0, 206), 770], [(0, 105), 519]]
        ]
        yield ReadArmTraj(arm_traj, 700)
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
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
                # Speed control
                servo_speed = 110
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))
                
        arm_traj = [
        [[(1, 5), 423], [(1, 207), 339], [(1, 107), 511], [(0, 204), 494], [(0, 206), 718], [(0, 105), 516]], 
        [[(1, 5), 415], [(1, 207), 283], [(1, 107), 359], [(0, 204), 481], [(0, 206), 617], [(0, 105), 512]], 
        #[[(1, 5), 415], [(1, 207), 296], [(1, 107), 352], [(0, 204), 480], [(0, 206), 500], [(0, 105), 512]], 
        [[(1, 5), 415], [(1, 207), 296], [(1, 107), 352+10], [(0, 204), 480], [(0, 206), 420], [(0, 105), 512]], 
        #[[(1, 5), 413], [(1, 207), 351], [(1, 107), 316], [(0, 204), 481], [(0, 206), 388], [(0, 105), 512]], 
        [[(1, 5), 413], [(1, 207), 340], [(1, 107), 325+40], [(0, 204), 481], [(0, 206), 350], [(0, 105), 512]], 
        #[[(1, 5), 410], [(1, 207), 371], [(1, 107), 275], [(0, 204), 481], [(0, 206), 301], [(0, 105), 512]], 
        [[(1, 5), 410], [(1, 207), 350], [(1, 107), 300+25], [(0, 204), 481], [(0, 206), 280], [(0, 105), 512]], 
        #[[(1, 5), 410], [(1, 207), 463], [(1, 107), 324], [(0, 204), 481], [(0, 206), 260], [(0, 105), 512]], 
        [[(1, 5), 410], [(1, 207), 450], [(1, 107), 335+20], [(0, 204), 481], [(0, 206), 260], [(0, 105), 512]], 
        [[(1, 5), 406], [(1, 207), 559], [(1, 107), 348], [(0, 204), 484], [(0, 206), 168], [(0, 105), 512]], 
        [[(1, 5), 441], [(1, 207), 618], [(1, 107), 400], [(0, 204), 484], [(0, 206), 161], [(0, 105), 472]]
        ]

        yield ReadArmTraj(arm_traj, delay=200, reverse=True)
        yield Trigger(ARM_7_HOLD)
        yield Timer(300)
        yield None

class TestStockModuleFromGrabbedModuleRightFront(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_OPEN)
        
        # Rotation seule et rapide de la base du bras
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
                # Speed control
                servo_speed = 200
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))
        
        yield ReadArmTraj([[[(1, 5), 317]]], 100)
        
        # Montee du module a vitesse plus lente
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
                # Speed control
                servo_speed = 110
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))
        
        arm_traj_up = [
        [[(1, 5), 312], [(1, 207), 500], [(1, 107), 376], [(0, 204), 500], [(0, 206), 240], [(0, 105), 487]], 
        [[(1, 5), 286], [(1, 207), 442], [(1, 107), 358], [(0, 204), 484], [(0, 206), 297], [(0, 105), 553]], 
        [[(1, 5), 280], [(1, 207), 381], [(1, 107), 369], [(0, 204), 484], [(0, 206), 357], [(0, 105), 590]], 
        [[(1, 5), 277], [(1, 207), 356], [(1, 107), 424], [(0, 204), 485], [(0, 206), 434], [(0, 105), 641]], 
        [[(1, 5), 272], [(1, 207), 339], [(1, 107), 469], [(0, 204), 485], [(0, 206), 496], [(0, 105), 668]],
        [[(1, 5), 266], [(1, 207), 366], [(1, 107), 533], [(0, 204), 490], [(0, 206), 524], [(0, 105), 680]], 
        [[(1, 5), 260], [(1, 207), 409], [(1, 107), 596], [(0, 204), 491], [(0, 206), 551], [(0, 105), 690]], 
        [[(1, 5), 260], [(1, 207), 423], [(1, 107), 619], [(0, 204), 490], [(0, 206), 557], [(0, 105), 690]]
        ]
        yield ReadArmTraj(arm_traj_up, 10)
        
        yield Trigger(makeServoMoveCommand((ARM_3_ID, 700), arm_traj_up[-1][2][1] + 20))
        
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_HOLD)
        
        yield Timer(100)
        yield Trigger(ARM_7_DROP)
            
                
        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 236], [(1, 207), 384], [(1, 107), 574], [(0, 204), 486], [(0, 206), 555], [(0, 105), 704]], 
        [[(1, 5), 223], [(1, 207), 306], [(1, 107), 465], [(0, 204), 486], [(0, 206), 518], [(0, 105), 704]], 
        [[(1, 5), 226], [(1, 207), 253], [(1, 107), 398], [(0, 204), 487], [(0, 206), 508], [(0, 105), 700]], 
        [[(1, 5), 261], [(1, 207), 272], [(1, 107), 365], [(0, 204), 487], [(0, 206), 462], [(0, 105), 626]],
        [[(1, 5), 277], [(1, 207), 298], [(1, 107), 350], [(0, 204), 487], [(0, 206), 415], [(0, 105), 550]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 100, reverse = True)
        
        arm_traj_down_2 = [
        [[(1, 5), 293], [(1, 207), 342], [(1, 107), 323], [(0, 204), 487], [(0, 206), 350], [(0, 105), 518]], 
        [[(1, 5), 279], [(1, 207), 370], [(1, 107), 265], [(0, 204), 490], [(0, 206), 254], [(0, 105), 513]], 
        [[(1, 5), 280], [(1, 207), 354], [(1, 107), 201], [(0, 204), 488], [(0, 206), 180], [(0, 105), 514]], 
        [[(1, 5), 280], [(1, 207), 371], [(1, 107), 163], [(0, 204), 490], [(0, 206), 145], [(0, 105), 514]],
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))
        
        yield None

        
class TestStockModuleFromGrabbedModuleRight(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_OPEN)
        
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
                # Speed control
                servo_speed = 50
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))
        
        arm_traj = [
        [[(1, 5), 270], [(1, 207), 542], [(1, 107), 390], [(0, 204), 494], [(0, 206), 205], [(0, 105), 510]], 
        [[(1, 5), 200], [(1, 207), 406], [(1, 107), 381], [(0, 204), 511], [(0, 206), 328], [(0, 105), 525]], 
        [[(1, 5), 196], [(1, 207), 323], [(1, 107), 391], [(0, 204), 511], [(0, 206), 412], [(0, 105), 465]], 
        [[(1, 5), 200], [(1, 207), 384], [(1, 107), 569], [(0, 204), 493], [(0, 206), 546], [(0, 105), 431]]
        ]
        yield ReadArmTraj(arm_traj)
        
        yield Timer(1000)
        
        ARM_3         = (ARM_3_ID, 700)
        ARM_3_MOVE    = makeServoMoveCommand(ARM_3, arm_traj[-1][2][1] + 25)
        yield Trigger(ARM_3_MOVE)
        
        yield Timer(1000)
        yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
        
        yield Timer(1000)
        yield Trigger(ARM_7_DROP)
        
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
