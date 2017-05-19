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

ARM_LEARNING = 0
CENTRAL_BASE_X = 1.288 + 0.014
CENTRAL_BASE_Y = 1.151 - 0.014
CENTRAL_BASE_ANGLE = -math.pi / 4.0
RECALIBRATE_DIST = 0.02




class StartArmSequence(State):

    def __init__(self, id, *args, **kwargs):
        self.id = id
        self.args = args
        self.kwargs = kwargs


    def on_enter(self):
        packet = packets.ArmSequenceStart(self.id)
        packet.args = self.args
        packet.kwargs = self.kwargs
        self.send_packet(packet)
        yield None




class WaitForArmSequence(Timer):

    def __init__(self):
        Timer.__init__(self, 0)


    def on_timeout(self):
        if not self.fsm.arm.is_moving:
            yield None

    def on_arm_sequence_end(self, packet):
        yield None




class ArmSequence(State):

    def __init__(self, id, *args, **kwargs):
        self.id = id
        self.args = args
        self.kwargs = kwargs


    def on_enter(self):
        yield StartArmSequence(self.id, *self.args, **self.kwargs)
        yield WaitForArmSequence()
        yield None




class Main(State):

    def on_enter(self):
        self.fsm.main_opponent_detector = StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_MAIN)
        self.fsm.secondary_opponent_detector = StateMachine(self.event_loop, "opponentdetector", opponent_type = OPPONENT_ROBOT_SECONDARY)
        self.fsm.arm = StateMachine(self.event_loop, "arm", opponent_type = OPPONENT_ROBOT_SECONDARY)
        if ARM_LEARNING == 1:
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
            yield Initialize()

            #yield AntiBlocking(True)
            rt = yield GetInputStatus(MAIN_INPUT_TEAM)
            self.rec = rt.value

            self.l_servoID = [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]

            for servoID in self.l_servoID:
                # Speed control
                servo_speed = 200
                yield Trigger(makeServoSetupCommand((servoID, 1000), servo_speed))

            yield CalibratePosition()

            None

    def on_input_status(self, packet):
        if packet.id == MAIN_INPUT_TEAM: # and packet.kind == KIND_READ:
            self.rec = packet.value

            if packet.value == 0 and ARM_LEARNING == 1:
                print("Recording")
                ArmPosition = yield ReadArmServoPosition(self.l_servoID)
                self.l_armPosition.append(ArmPosition.l_servoPosition)

                f_listArmPosition = open("/tmp/traj.txt", "w")
                f_listArmPosition.write(str(self.l_armPosition))
                f_listArmPosition.close()


    def on_start(self, packet):
        if packet.value == 0:
            self.yield_at(89500, EndOfMatch())
            yield SpeedControl(88)

            if self.robot.team == TEAM_RIGHT:
                global CENTRAL_BASE_X
                global CENTRAL_BASE_Y
                CENTRAL_BASE_X += 0.010
                CENTRAL_BASE_Y -= 0.010

            if 1:
                if self.cnt_arm_action == 0:
                    #~ yield InitArm()
                    #~ yield InitForMonoColorModuleToDrop()
                    #~ yield Initialize()

                    #~ yield DefinePosition(0.0, 0.0, 0.0)
                    #~ yield MoveLineTo(1.0, 0.0)
                    #~ yield RotateTo(-math.pi/2.0)

                    yield StaticStrategy()


                elif self.cnt_arm_action == 1:
                    yield PolyRocket(False)
                    pass

                    #~ yield PolyRocket(False)
                    #~ yield MonoRocket(False)

                    #~ yield Trigger(ARM_7_HOLD)
                    #~ yield Trigger(STORAGE_FINGER_LEFT_HOLD)

                    #~ yield GrabModuleFromInit()
                    #~ yield StockModuleFromGrabbedModuleRight()
                    #~ yield GrabModuleFromStorageReturn()
                    #~ yield StockModuleFromGrabbedModuleRightFront()
                    #~ yield GrabModuleFromStorageReturn()
                    #~ yield StockModuleFromGrabbedModuleLeft()
                    #~ yield GrabModuleFromStorageReturn()
                    #~ yield StockModuleFromGrabbedModuleLeftFront()

                    #~ yield GrabBackModuleRight()

                    #~ team = TEAM_YELLOW
                    #~
                    #~ yield GrabPolyModuleFromInit()
                    #~ yield OutputPolyModuleFromRocket()
                    #~ yield DropPolyModule(kick=False)
                    #~
                    #~ yield GrabPolyModuleFromDropZone(previousModuleTurned=False)
                    #~ yield OutputPolyModuleFromRocket()
                    #~ yield DropTurnedPolyModule(finalKick=False)
                    #~
                    #~ yield GrabPolyModuleFromDropZone(previousModuleTurned=True)
                    #~ yield OutputPolyModuleFromRocket()
                    #~ yield DropPolyModule(kick=True)
                    #~
                    #~ yield GrabPolyModuleFromDropZone(previousModuleTurned=False)
                    #~ yield OutputPolyModuleFromRocket()
                    #~ yield DropTurnedPolyModule(finalKick=True)

                elif self.cnt_arm_action == 2:
                    #yield TestTakeModuleFromStorageReturn()
                    #yield TestStockModuleFromGrabbedModuleRight()
                    #yield TestStockModuleFromGrabbedModuleRightFront()
                    #yield DropPolyModule()
                    #yield GrabPolyModuleFromDropZone()
                    #~ yield DropTurnedPolyModule()

                    #~ yield GrabBackModuleRightFront()
                    #~ yield GrabBackModuleLeft()
                    #~ yield GrabBackModuleRight()
                    #~ yield GrabBackModuleLeftFront()

                    #~ yield Trigger(STORAGE_FINGER_LEFT_FRONT_HOLD)

                    #~ team = TEAM_YELLOW
                    #~ yield DropModuleFromStorage()
                    #~ yield InitForMonoColorModuleToDrop()

                    team = TEAM_YELLOW
                    yield CentralMoonBaseLatBranch(depl=False)
                    #~ None

                elif self.cnt_arm_action == 3:
                    #~ yield Trigger(STORAGE_FINGER_RIGHT_FRONT_HOLD)

                    #~ team = TEAM_YELLOW
                    #~ yield CentralMoonBaseLatBranch(depl=False)
                    None

                elif self.cnt_arm_action == 4:
                    yield Trigger(STORAGE_FINGER_RIGHT_HOLD)
                    #~ None

                elif self.cnt_arm_action == 5:
                    #~ team = TEAM_YELLOW
                    #~ yield CentralMoonBaseLatBranch(depl=False)
                    None

                self.cnt_arm_action = self.cnt_arm_action + 1


            #~ yield ExecuteGoals()

            #~ yield StaticStrategy()
            yield None




class PolyRocket(State):

    def __init__(self, depl = True):
        self.depl = depl

    def on_enter(self):
        if self.depl == True:
            yield RotateTo(-math.pi/2.0)
            yield MoveLineTo(1.25, 0.3)
        #---

        shift_1 = -0.005
        yield ArmSequence('GrabPolyModuleFromInit')

        yield ArmSequence('OutputPolyModuleFromRocket')

        yield StartArmSequence('DropPolyModule', kick=False)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield StartArmSequence('GrabPolyModuleFromDropZone', previousModuleTurned=False)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield ArmSequence('OutputPolyModuleFromRocket')

        yield StartArmSequence('DropTurnedPolyModule', finalKick=True)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield None # Skip the rest

        yield StartArmSequence('GrabPolyModuleFromDropZone', previousModuleTurned=True)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield ArmSequence('OutputPolyModuleFromRocket')

        yield StartArmSequence('DropPolyModule', kick=True)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield StartArmSequence('GrabPolyModuleFromDropZone', previousModuleTurned=False)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield ArmSequence('OutputPolyModuleFromRocket')

        yield StartArmSequence('DropTurnedPolyModule', finalKick=True)
        yield MoveLineTo(1.25, 0.3 + RECALIBRATE_DIST)
        yield MoveLineTo(1.25, 0.3 + shift_1)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        yield StartArmSequence('InitArm')
        #---
        if self.depl == True:
            yield MoveLineRelative(-0.05)

        self.exit_reason = GOAL_DONE
        yield None



class ReadModuleHolderPresence(State):
    def on_enter(self):

        self.robot.used_storage_spaces=[]

        for label, position, storage_space in [    ("LEFT FRONT", STORAGE_FINGER_LEFT_FRONT_HOLD, STORAGE_MODULE_LEFT_FRONT),
                                    ("LEFT", STORAGE_FINGER_LEFT_HOLD, STORAGE_MODULE_LEFT),
                                    ("RIGHT FRONT", STORAGE_FINGER_RIGHT_FRONT_HOLD, STORAGE_MODULE_RIGHT_FRONT),
                                    ("RIGHT", STORAGE_FINGER_RIGHT_HOLD, STORAGE_MODULE_RIGHT)
                                ]:
            logger.log("Read servo {} position {}")
            srv_id = position[0] # type: int
            expected_value = position[2] # type: int
            real_value = (yield ReadServoPosition(srv_id)).value # type:int

            delta = abs(expected_value - real_value) < 10

            logger.log("Servo {} expected position: {} real position: {} delta: {}".format(label, expected_value, real_value, delta))

            if delta < 10:
                logger.log("Storage empty")
            else:
                logger.log("Storage used")
                self.robot.used_storage_spaces.append(storage_space)

        logger.log("Used storage spaces: {}".format(self.robot.used_storage_spaces))


class MonoRocket(State):
    def __init__(self, depl = True):
        self.depl = depl

    def on_enter(self):
        if self.depl == True:
            yield RotateTo(math.pi)
            yield MoveLineTo(0.350, 1.150)
        #---
        yield ArmSequence('GrabModuleFromInit')
        if self.robot.team == TEAM_RIGHT:
            yield StartArmSequence('StockModuleFromGrabbedModuleRight')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield StartArmSequence('StockModuleFromGrabbedModuleRightFront')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield StartArmSequence('StockModuleFromGrabbedModuleLeft')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield ArmSequence('StockModuleFromGrabbedModuleLeftFront')
        else:
            yield StartArmSequence('StockModuleFromGrabbedModuleLeft')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield StartArmSequence('StockModuleFromGrabbedModuleLeftFront')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield StartArmSequence('StockModuleFromGrabbedModuleRight')
            yield MoveLineTo(0.350 - 0.005 + RECALIBRATE_DIST, 1.150)
            yield MoveLineTo(0.350 - 0.005, 1.150)
            yield RotateTo(math.pi)
            yield WaitForArmSequence()
            yield ArmSequence('GrabModuleFromStorageReturn')

            yield ArmSequence('StockModuleFromGrabbedModuleRightFront')
        yield StartArmSequence('InitArm')
        #---
        if self.depl == True:
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None


class PickNextModuleToDrop(State):
    def on_enter(self):
        self.module_to_drop = False

        DROP_STATE_MAPPING = {
            STORAGE_MODULE_LEFT: "GrabBackModuleLeft",
            STORAGE_MODULE_LEFT_FRONT: "GrabBackModuleLeftFront"
            STORAGE_MODULE_RIGHT: "GrabBackModuleRight"
            STORAGE_MODULE_RIGHT_FRONT: "GrabBackModuleRightFront"
        }


        if self.robot.team == TEAM_LEFT:
            DROP_ORDER = [STORAGE_MODULE_LEFT_FRONT, STORAGE_MODULE_LEFT, STORAGE_MODULE_RIGHT_FRONT, STORAGE_MODULE_RIGHT]
        else:
            DROP_ORDER = [STORAGE_MODULE_RIGHT_FRONT, STORAGE_MODULE_RIGHT, STORAGE_MODULE_LEFT_FRONT, STORAGE_MODULE_LEFT]

        logger.log("Drop order: {}".format([STORAGE_MODULE.lookup_by_value[x] for x in DROP_ORDER)))

        for storage_name in DROP_ORDER:
            if storage_name in self.robot.used_storage_spaces:
                storage_label = STORAGE_MODULE.lookup_by_value[storage_name]
                logger.log("Storage {} is in use".format(storage_label))
                state_to_use = DROP_STATE_MAPPING[storage_name]
                yield StartArmSequence(state_to_use)
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                self.robot.used_storage_spaces.remove(storage_name)
                self.module_to_drop = True


class CentralMoonBaseLatBranch(State):
    def __init__(self, depl = True):
        self.depl = depl

    def on_enter(self):
        if self.depl == True:
            yield RotateTo(-math.pi/4.0)
            yield MoveLineTo(1.238, 1.101)
        #---

        yield ArmSequence('DropModuleFromStorage')

        for i in range(4):
            if (yield PickNextModuleToDrop()).module_to_drop:
                yield ArmSequence('DropModuleFromStorage')
            else:
                logger.log("No module to drop")

        yield StartArmSequence('InitArm')
        #---
        if self.depl == True:
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None



class CentralMoonBaseLatBranchBackup(State):
    def __init__(self, depl = True):
        self.depl = depl

    def on_enter(self):
        if self.depl == True:
            yield RotateTo(-math.pi/4.0)
            yield MoveLineTo(1.238, 1.101)
        #---
        if self.robot.team == TEAM_LEFT:

            #TODO: dynamic pick order
            if self.robot.is_holding_module:
                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_LEFT in self.robot.used_storage_spaces:
                yield StartArmSequence('GrabBackModuleLeft')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_RIGHT_FRONT in self.robot.used_storage_spaces:
                yield StartArmSequence('GrabBackModuleRightFront')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_RIGHT in self.robot.used_storage_spaces:
                yield StartArmSequence('GrabBackModuleRight')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

        if self.robot.team == TEAM_RIGHT:
            if self.robot.is_holding_module:
                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_RIGHT in self.robot.used_storage_spaces:

                yield StartArmSequence('GrabBackModuleRight')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_LEFT_FRONT in self.robot.used_storage_spaces:
                yield StartArmSequence('GrabBackModuleLeftFront')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

            if STORAGE_MODULE_LEFT in self.robot.used_storage_spaces:

                yield StartArmSequence('GrabBackModuleLeft')
                yield MoveLineRelative(-RECALIBRATE_DIST)
                yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
                yield RotateTo(CENTRAL_BASE_ANGLE)
                yield WaitForArmSequence()

                yield ArmSequence('DropModuleFromStorage')

        yield StartArmSequence('InitArm')
        #---
        if self.depl == True:
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
        #---
        yield Trigger(STONE_GUIDE_HOLD)
        yield Timer(500)
        #---
        if self.depl == True:
            yield MoveLineRelative(-0.05)
        self.exit_reason = GOAL_DONE
        yield None



class StoneDrop(State):
    def __init__(self, depl = True):
        self.depl = depl

    def on_enter(self):
        if self.depl == True:
            None
        #---
        yield Trigger(STONE_GUIDE_INIT)
        #---
        self.exit_reason = GOAL_DONE
        yield None




class Initialize(State):

    def on_enter(self):
        yield StartArmSequence('InitArm')
        yield Trigger(STORAGE_FINGER_RIGHT_INIT,
                        STORAGE_FINGER_RIGHT_FRONT_INIT,
                        STORAGE_FINGER_LEFT_FRONT_INIT,
                        STORAGE_FINGER_LEFT_INIT,
                        STONE_GUIDE_INIT
                        )
        #yield ServoTorqueControl([LEFT_CLAPMAN_ID, RIGHT_CLAPMAN_ID], False)
        yield None
        #~ None




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




class StaticStrategy(State):
    def on_enter(self):
        yield StartArmSequence('ClearFirstModule')

        # Deplacement vers fusee polychrome bleu
        yield MoveLineTo(0.927 - 0.05, 0.923 + 0.05)
        yield StartArmSequence('InitArm')
        yield RotateTo(-math.pi/4.0)
        yield MoveLineTo(1.25, 0.6)
        yield RotateTo(-math.pi/2.0)
        yield StartArmSequence('InitGrabPolyModuleFromInit')
        yield MoveLineTo(1.25, 0.3 -0.005)
        yield RotateTo(-math.pi/2.0)
        yield WaitForArmSequence()

        # Travail sur fusee polychrome bleu pour dépose 4 elements
        yield PolyRocket(False)

        # Deplacement vers fusee bleu
        yield MoveLineTo(1.250, 0.600)
        yield RotateTo(-math.pi/4.0)
        yield MoveLineTo(0.700, 1.150)
        yield RotateTo(math.pi)
        yield StartArmSequence('InitGrabModuleFromInit')
        yield MoveLineTo(0.350 - 0.005, 1.150)
        yield RotateTo(math.pi)
        yield WaitForArmSequence()

        # Travail sur fusee bleu pour stockage de 3 elements et prise de 1 element
        yield MonoRocket(False)

        # Deplacement vers base lunaire centrale
        yield MoveLineTo(1.260, 1.180)
        yield RotateTo(-math.pi/4.0)
        yield WaitForArmSequence()
        # if self.robot.team == TEAM_RIGHT:
        #     yield StartArmSequence('GrabBackModuleRightFront')
        # else:
        #     yield StartArmSequence('GrabBackModuleLeftFront')
        # yield MoveLineTo(CENTRAL_BASE_X, CENTRAL_BASE_Y)
        # yield RotateTo(CENTRAL_BASE_ANGLE)
        # yield WaitForArmSequence()

        yield ReadModuleHolderPresence()

        yield PickNextModuleToDrop()

        # Depose des elements dans branche bleu base lunaire centrale
        yield CentralMoonBaseLatBranch(False)

        # Deplacement vers cratere bleu
        yield MoveLineTo(1.260, 1.180)
        #yield RotateTo(math.radians(37.0))
        yield LookAtOpposite(0.765, 0.811)
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
        tools.on_end_of_match()
