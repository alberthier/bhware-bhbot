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
        self.fsm.is_moving = False

    def on_start(self, packet):
        self.yield_at(89500, EndOfMatch())

    def on_arm_sequence_start(self, packet):
        state = getattr(sys.modules[__name__], packet.id)
        if not state:
            self.log("State not found: '{}'".format(packet.id))
        else:
            self.fsm.is_moving = True
            yield state(*packet.args, **packet.kwargs)
            self.fsm.is_moving = False
        self.send_packet(packets.ArmSequenceEnd(packet.id))




class EndOfMatch(State):

    def on_enter(self):
        yield ServoTorqueControl([ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID, ARM_7_ID], False)




class ArmSpeed(State):
    def __init__(self, speed):
        self.speed = speed

    def on_enter(self):
        for servoID in [ARM_1_ID, ARM_2_ID, ARM_3_ID, ARM_4_ID, ARM_5_ID, ARM_6_ID]:
            # Speed control
            yield Trigger(makeServoSetupCommand((servoID, 1000), self.speed))
        yield None




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




class GrabBackModuleRightFront(State):
    def on_enter(self):
        yield Trigger(ARM_7_OPEN)
        yield ArmSpeed(350)
        shift_up = 25
        arm_traj = [
        [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]]
        ]
        yield ReadArmTraj(arm_traj, delay=30, reverse=False)
        yield Timer(30)
        arm_traj = [
        [[(1, 5), 266], [(1, 207), 315], [(1, 107), 368+shift_up], [(0, 204), 474], [(0, 206), 443], [(0, 105), 668]],
        [[(1, 5), 223], [(1, 207), 272], [(1, 107), 368+shift_up], [(0, 204), 497-23], [(0, 206), 454], [(0, 105), 712]],
        [[(1, 5), 204], [(1, 207), 249], [(1, 107), 394+shift_up], [(0, 204), 477], [(0, 206), 501], [(0, 105), 728]],
        [[(1, 5), 211], [(1, 207), 320], [(1, 107), 516+shift_up], [(0, 204), 488], [(0, 206), 551], [(0, 105), 728]],
        [[(1, 5), 233], [(1, 207), 360], [(1, 107), 544+shift_up], [(0, 204), 493], [(0, 206), 546], [(0, 105), 708]],
        [[(1, 5), 275], [(1, 207), 392], [(1, 107), 578+shift_up], [(0, 204), 493], [(0, 206), 546], [(0, 105), 690]]
        ]
        yield ReadArmTraj(arm_traj, delay=30, reverse=False)

        yield Timer(100)
        yield Trigger(ARM_7_HOLD)
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_OPEN)

        yield Timer(100)

        arm_traj = [
        [[(1, 5), 265], [(1, 207), 343], [(1, 107), 392], [(0, 204), 491], [(0, 206), 418], [(0, 105), 684]],
        [[(1, 5), 423], [(1, 207), 415], [(1, 107), 478], [(0, 204), 500], [(0, 206), 414], [(0, 105), 523]]
        ]
        yield ReadArmTraj(arm_traj, delay=300)
        yield None


class GrabBackModuleLeft(State):
    def on_enter(self):
        yield Trigger(makeServoMoveCommand(ARM_7, 347))
        yield ArmSpeed(350)
        shift_up = 12
        arm_traj = [
        [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]],
        [[(1, 5), 682]]
        ]
        yield ReadArmTraj(arm_traj, delay=30)
        yield Timer(200)
        arm_traj = [
        [[(1, 5), 650], [(1, 207), 261], [(1, 107), 385+shift_up], [(0, 204), 507], [(0, 206), 497], [(0, 105), 601]],
        [[(1, 5), 649], [(1, 207), 320], [(1, 107), 480+shift_up], [(0, 204), 510], [(0, 206), 511], [(0, 105), 601]],
        [[(1, 5), 645], [(1, 207), 364], [(1, 107), 540+shift_up], [(0, 204), 506], [(0, 206), 536], [(0, 105), 608]],
        [[(1, 5), 651], [(1, 207), 415], [(1, 107), 585+shift_up+5], [(0, 204), 507], [(0, 206), 535], [(0, 105), 614]],
        [[(1, 5), 655], [(1, 207), 427], [(1, 107), 594+shift_up+5], [(0, 204), 507], [(0, 206), 533], [(0, 105), 614]]
        ]
        yield ReadArmTraj(arm_traj, delay=30, reverse=False)

        yield Timer(100)
        yield Trigger(ARM_7_HOLD)
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_LEFT_OPEN)

        yield Timer(100)

        arm_traj = [
        [[(1, 5), 628], [(1, 207), 496], [(1, 107), 548], [(0, 204), 504], [(0, 206), 438], [(0, 105), 561]],
        [[(1, 5), 423], [(1, 207), 415], [(1, 107), 478], [(0, 204), 500], [(0, 206), 414], [(0, 105), 523]]
        ]
        yield ReadArmTraj(arm_traj, delay=300)
        yield None

class GrabBackModuleRight(State):
    def on_enter(self):
        yield Trigger(ARM_7_OPEN)
        yield ArmSpeed(350)
        shift_up = 12
        arm_traj = [
        [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]],
        [[(1, 5), 152]]
        ]
        yield ReadArmTraj(arm_traj, delay=30)

        yield Timer(200)
        arm_traj = [
        [[(1, 5), 165], [(1, 207), 225], [(1, 107), 398+shift_up], [(0, 204), 495], [(0, 206), 550], [(0, 105), 494]],
        [[(1, 5), 173], [(1, 207), 310], [(1, 107), 527+shift_up], [(0, 204), 503], [(0, 206), 602], [(0, 105), 472]],
        [[(1, 5), 197], [(1, 207), 397], [(1, 107), 632+shift_up], [(0, 204), 500], [(0, 206), 629], [(0, 105), 447]],
        [[(1, 5), 197], [(1, 207), 445], [(1, 107), 678+shift_up], [(0, 204), 503], [(0, 206), 616], [(0, 105), 443]],
        [[(1, 5), 211], [(1, 207), 455], [(1, 107), 680+shift_up], [(0, 204), 502], [(0, 206), 602], [(0, 105), 429]]
        ]
        yield ReadArmTraj(arm_traj, delay=30)

        yield Timer(100)
        yield Trigger(ARM_7_HOLD)
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_RIGHT_OPEN)

        yield Timer(100)

        arm_traj = [
        [[(1, 5), 232], [(1, 207), 575], [(1, 107), 729], [(0, 204), 500], [(0, 206), 509], [(0, 105), 490]],
        [[(1, 5), 423], [(1, 207), 415], [(1, 107), 478], [(0, 204), 500], [(0, 206), 414], [(0, 105), 523]]
        ]
        yield ReadArmTraj(arm_traj, delay=300)
        yield None

class GrabBackModuleLeftFront(State):
    def on_enter(self):
        #~ yield Trigger(ARM_7_OPEN)
        yield Trigger(makeServoMoveCommand(ARM_7, 347))

        yield ArmSpeed(300)

        arm_traj = [
        [[(1, 5), 548], [(1, 207), 278], [(1, 107), 362], [(0, 204), 501], [(0, 206), 513], [(0, 105), 515]],
        [[(1, 5), 548], [(1, 207), 232], [(1, 107), 408], [(0, 204), 502], [(0, 206), 584], [(0, 105), 515]],
        [[(1, 5), 546], [(1, 207), 238], [(1, 107), 436], [(0, 204), 495], [(0, 206), 561], [(0, 105), 513]]
        ]
        yield ReadArmTraj(arm_traj, delay=50)
        yield Timer(100)
        arm_traj = [
        [[(1, 5), 548], [(1, 207), 375], [(1, 107), 568], [(0, 204), 501], [(0, 206), 540], [(0, 105), 514]]
        ]
        yield ReadArmTraj(arm_traj)

        yield Timer(200)
        yield Trigger(ARM_7_HOLD)
        yield Timer(200)
        yield Trigger(STORAGE_FINGER_LEFT_FRONT_OPEN)

        yield Timer(100)

        arm_traj = [
        [[(1, 5), 545], [(1, 207), 302], [(1, 107), 405], [(0, 204), 500], [(0, 206), 440], [(0, 105), 513]],
        [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]]
        ]
        yield ReadArmTraj(arm_traj, delay=300)

        yield None


class DropModuleFromStorage(State):
    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            yield ArmSpeed(150)
            arm_traj = [
            [[(1, 5), 318], [(1, 207), 491], [(1, 107), 579], [(0, 204), 770], [(0, 206), 592], [(0, 105), 450]],
            [[(1, 5), 322], [(1, 207), 600], [(1, 107), 681], [(0, 204), 757], [(0, 206), 605], [(0, 105), 450]],
            [[(1, 5), 426], [(1, 207), 531], [(1, 107), 560+30], [(0, 204), 810], [(0, 206), 517], [(0, 105), 451]],
            [[(1, 5), 566], [(1, 207), 678], [(1, 107), 735+30], [(0, 204), 866], [(0, 206), 391], [(0, 105), 436]],
            # new
            [[(1, 5), 575], [(1, 207), 457], [(1, 107), 512+10], [(0, 204), 869], [(0, 206), 404], [(0, 105), 435]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield Timer(500)
            yield ArmSpeed(200)
            #~ yield Trigger(ARM_7_DROP)
            yield Trigger(ARM_7_OPEN)
            yield Timer(150)

            arm_traj = [
            [[(1, 5), 576], [(1, 207), 519], [(1, 107), 554], [(0, 204), 880], [(0, 206), 391], [(0, 105), 466]],
            [[(1, 5), 441], [(1, 207), 397], [(1, 107), 435], [(0, 204), 817], [(0, 206), 501], [(0, 105), 468]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield Timer(150)
            yield ArmSpeed(600)
            arm_traj = [
            # Position finale pour enchainement recup + drop
            [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield Timer(150)
            yield ArmSpeed(200)

        if self.robot.team == TEAM_RIGHT:
            yield ArmSpeed(250)

            arm_traj = [
            [[(1, 5), 585], [(1, 207), 410], [(1, 107), 431], [(0, 204), 563], [(0, 206), 462], [(0, 105), 393]],
            [[(1, 5), 591], [(1, 207), 544], [(1, 107), 597+10], [(0, 204), 902], [(0, 206), 375], [(0, 105), 486]],
            [[(1, 5), 575], [(1, 207), 600], [(1, 107), 655+10], [(0, 204), 890], [(0, 206), 378], [(0, 105), 483]],
            [[(1, 5), 485], [(1, 207), 498], [(1, 107), 520+10], [(0, 204), 853], [(0, 206), 457], [(0, 105), 485]],
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield ArmSpeed(150)
            yield Timer(120)
            arm_traj = [
            [[(1, 5), 378], [(1, 207), 509], [(1, 107), 525+10], [(0, 204), 783], [(0, 206), 555], [(0, 105), 484]],
            [[(1, 5), 294], [(1, 207), 631], [(1, 107), 681+10], [(0, 204), 751], [(0, 206), 629], [(0, 105), 450]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield ArmSpeed(250)
            yield Timer(120)
            arm_traj = [
            [[(1, 5), 274], [(1, 207), 507], [(1, 107), 558+10], [(0, 204), 728], [(0, 206), 646], [(0, 105), 478]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)
            yield Timer(150)
            yield Trigger(ARM_7_OPEN)
            yield Timer(150)

            arm_traj = [
            [[(1, 5), 436], [(1, 207), 376], [(1, 107), 415+10], [(0, 204), 822], [(0, 206), 509], [(0, 105), 485]]
            ]
            yield ReadArmTraj(arm_traj)
            yield Timer(150)
            yield ArmSpeed(600)
            arm_traj = [
            [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]]
            ]
            yield ReadArmTraj(arm_traj)
            yield ArmSpeed(250)

        yield None

class InitForMonoColorModuleToDrop(State):
    def on_enter(self):
        # Init for test
        yield ArmSpeed(110)
        arm_traj = [
        # Init en test
        #[[(1, 5), 423], [(1, 207), 415], [(1, 107), 448+30], [(0, 204), 500], [(0, 206), 414], [(0, 105), 523]]

        # Init pour enchainement recup + drop
        [[(1, 5), 405], [(1, 207), 321], [(1, 107), 402], [(0, 204), 495], [(0, 206), 532], [(0, 105), 541]]
        ]
        yield ReadArmTraj(arm_traj)
        yield Timer(1000)
        yield Trigger(ARM_7_OPEN)
        yield None

class InitArm(State):
    def on_enter(self):
        yield ArmSpeed(110)

        arm_traj = [
        [[(1, 5), 433], [(1, 207), 400], [(1, 107), 437], [(0, 204), 512], [(0, 206), 702], [(0, 105), 462]],
        [[(1, 5), 436], [(1, 207), 334], [(1, 107), 401], [(0, 204), 512], [(0, 206), 720], [(0, 105), 462]],
        [[(1, 5), 436], [(1, 207), 306], [(1, 107), 419], [(0, 204), 512], [(0, 206), 778], [(0, 105), 462]],
        # 1
        [[(1, 5), 414], [(1, 207), 362], [(1, 107), 480], [(0, 204), 497], [(0, 206), 801], [(0, 105), 541]]
        # 2
        #[[(1, 5), 420], [(1, 207), 271], [(1, 107), 507], [(0, 204), 481], [(0, 206), 750], [(0, 105), 542]]
        ]
        yield ReadArmTraj(arm_traj, 700)
        yield Trigger(makeServoSetupCommand(((0,205), 1000), 1000))
        yield Trigger(ARM_7_INIT)

        yield None


class InitGrabModuleFromInit(State):
    def on_enter(self):
        yield ArmSpeed(350)

        yield ReadArmTraj([[[(0, 206), 682]]])
        yield Trigger(ARM_7_OPEN)
        arm_traj = [
        [[(1, 5), 419], [(1, 207), 290], [(1, 107), 381+60], [(0, 204), 500], [(0, 206), 149], [(0, 105), 512]],
        [[(1, 5), 413], [(1, 207), 386], [(1, 107), 183], [(0, 204), 499], [(0, 206), 161], [(0, 105), 512-10]]
        ]
        yield ReadArmTraj(arm_traj, delay=150, reverse=True)
        yield Timer(100)
        yield None


class GrabModuleFromInit(State):
    def on_enter(self):
        arm_traj = [
        [[(1, 5), 414], [(1, 207), 551], [(1, 107), 340], [(0, 204), 500], [(0, 206), 192], [(0, 105), 502]]
        ]
        yield ReadArmTraj(arm_traj, delay=50, reverse=True)
        yield Timer(50)
        arm_traj = [
        [[(1, 5), 413], [(1, 207), 716], [(1, 107), 570], [(0, 204), 498], [(0, 206), 242], [(0, 105), 503]],
        [[(1, 5), 425], [(1, 207), 758], [(1, 107), 632], [(0, 204), 496], [(0, 206), 259], [(0, 105), 518]]
        ]
        yield ReadArmTraj(arm_traj, delay=50, reverse=True)

        yield Timer(50)
        yield Trigger(ARM_7_HOLD)

        yield ArmSpeed(250)

        yield ReadArmTraj([
        [[(1, 5), 423], [(1, 207), 631], [(1, 107), 441], [(0, 204), 498], [(0, 206), 208], [(0, 105), 506]],
        [[(1, 5), 426], [(1, 207), 498], [(1, 107), 279], [(0, 204), 497], [(0, 206), 160], [(0, 105), 494]]
        ], delay=50)

        yield Timer(50)

        yield None


class InitGrabPolyModuleFromInit(State):

    def on_enter(self):
        yield ArmSpeed(350)
        yield ReadArmTraj([[[(0, 206), 682]]])
        yield Timer(500)
        arm_traj = [
        [[(1, 5), 419], [(1, 207), 290], [(1, 107), 381+60], [(0, 204), 500], [(0, 206), 149], [(0, 105), 512]],
        [[(1, 5), 423], [(1, 207), 386], [(1, 107), 183], [(0, 204), 499], [(0, 206), 161], [(0, 105), 512]]
        ]
        yield ReadArmTraj(arm_traj, delay=150, reverse=True)
        yield ArmSpeed(500)
        yield Trigger(ARM_7_OPEN)
        yield None




class GrabPolyModuleFromInit(State):
    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            arm_traj = [
            [[(1, 5), 530]]
            ]
            yield ReadArmTraj(arm_traj, delay=150, reverse=True)

            yield Timer(50)

            yield ArmSpeed(400)

            arm_traj = [
            [[(1, 5), 561], [(1, 207), 495], [(1, 107), 279], [(0, 204), 499], [(0, 206), 166], [(0, 105), 350]],
            [[(1, 5), 549], [(1, 207), 667], [(1, 107), 477], [(0, 204), 498], [(0, 206), 198], [(0, 105), 370]],
            [[(1, 5), 564], [(1, 207), 696], [(1, 107), 530], [(0, 204), 505], [(0, 206), 222], [(0, 105), 369]]
            ]
            yield ReadArmTraj(arm_traj, delay=30, reverse=True)
            yield Timer(50)
            yield Trigger(ARM_7_HOLD)

        if self.robot.team == TEAM_RIGHT:
            arm_traj = [
            [[(1, 5), 320]]
            ]
            yield ReadArmTraj(arm_traj, delay=150, reverse=True)

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
    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            yield ArmSpeed(110)
            arm_traj = [
            [[(1, 5), 574], [(1, 207), 581], [(1, 107), 378], [(0, 204), 511], [(0, 206), 195], [(0, 105), 383]],
            [[(1, 5), 608], [(1, 207), 470], [(1, 107), 259], [(0, 204), 509], [(0, 206), 166], [(0, 105), 322]],
            ]
            yield ReadArmTraj(arm_traj, delay=700, reverse=True)
            yield Timer(550)
            yield ArmSpeed(400)

            yield ReadArmTraj([[[(1, 5), 451+30]]], reverse=True)

            yield Timer(200)

        if self.robot.team == TEAM_RIGHT:
            yield ArmSpeed(110)
            arm_traj = [
            [[(1, 5), 301], [(1, 207), 594], [(1, 107), 365], [(0, 204), 511], [(0, 206), 160], [(0, 105), 647]]
            ]
            yield ReadArmTraj(arm_traj, delay=700, reverse=True)

            yield Timer(400)
            yield ArmSpeed(400)

            arm_traj = [
            [[(1, 5), 380]]
            ]
            yield ReadArmTraj(arm_traj)

            yield Timer(100)

        yield None

class DropPolyModule(State):
    def __init__(self, kick=True):
        self.kick = kick

    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            yield ArmSpeed(180)
            arm_traj = [
                [[(1, 5), 377], [(1, 207), 435], [(1, 107), 313], [(0, 204), 496], [(0, 206), 247], [(0, 105), 421]],
                [[(1, 5), 280], [(1, 207), 612], [(1, 107), 473], [(0, 204), 494], [(0, 206), 233], [(0, 105), 590]],
                [[(1, 5), 267], [(1, 207), 699], [(1, 107), 617], [(0, 204), 494], [(0, 206), 296], [(0, 105), 518]]
                ]
            if self.kick == True:
                yield ReadArmTraj(arm_traj, delay=150)#, reverse=True)
                yield Timer(500)
            else:
                yield ReadArmTraj([arm_traj[0]])#, reverse=True)

            yield Timer(300)
            yield ArmSpeed(200)

            # Reach drop position
            arm_traj = [
            [[(1, 5), 313], [(1, 207), 519], [(1, 107), 435], [(0, 204), 507], [(0, 206), 252], [(0, 105), 610]]
            ]
            yield ReadArmTraj(arm_traj, delay=400, reverse=True)

            yield Timer(200)

            yield ArmSpeed(110)

            ARM_7_DROP_int = [(0, 205), 162+140]
            arm_traj = [
            [ARM_7_DROP_int],
            [[(1, 5), 276], [(1, 207), 470], [(1, 107), 498], [(0, 204), 509], [(0, 206), 395], [(0, 105), 619], ARM_7_DROP_int]
            ]
            yield ReadArmTraj(arm_traj, delay=400, reverse=True)
            yield Timer(100)

        if self.robot.team == TEAM_RIGHT:
            yield ArmSpeed(150)
            arm_traj_1 = [
            [[(1, 207), 441], [(1, 107), 323+40], [(0, 204), 484], [(0, 206), 240], [(0, 105), 560]]
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)
            yield Timer(300)
            arm_traj_1 = [
            [[(1, 5), 412], [(0, 105), 450]],
            [[(1, 5), 529], [(1, 207), 474], [(1, 107), 342+30], [(0, 204), 494], [(0, 206), 231], [(0, 105), 373]],
            [[(1, 5), 546], [(1, 207), 590], [(1, 107), 455+10], [(0, 204), 494], [(0, 206), 245], [(0, 105), 373]]
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)
            yield Timer(100)
            if self.kick == True:
                yield ArmSpeed(200)
                arm_traj_2 = [
                [[(1, 5), 596], [(1, 207), 740], [(1, 107), 678], [(0, 204), 491], [(0, 206), 327], [(0, 105), 372]],
                [[(1, 5), 581], [(1, 207), 616], [(1, 107), 511], [(0, 204), 499], [(0, 206), 275], [(0, 105), 380]],
                arm_traj_1[-1]
                ]
                yield ReadArmTraj(arm_traj_2, delay=200)#, reverse=True)
                yield Timer(200)
                yield ArmSpeed(150)


            yield Timer(200)
            yield Trigger(ARM_7_OPEN)

        yield None

class DropTurnedPolyModule(State):
    def __init__(self, finalKick=False):
        self.finalKick = finalKick

    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            yield ArmSpeed(150)
            # Kick dropped modules
            arm_traj = [
            [[(1, 5), 377], [(1, 207), 435], [(1, 107), 313], [(0, 204), 496], [(0, 206), 247], [(0, 105), 421]],
            [[(1, 5), 280], [(1, 207), 612], [(1, 107), 473], [(0, 204), 494], [(0, 206), 233], [(0, 105), 590]],
            [[(1, 5), 287], [(1, 207), 699], [(1, 107), 617], [(0, 204), 494], [(0, 206), 296], [(0, 105), 478]]
            ]
            yield ReadArmTraj(arm_traj, delay=150)#, reverse=True)

            yield Timer(400)

            yield ArmSpeed(150)
            ARM_7_DROP_int = [(0, 205), 438]
            arm_traj = [
            [[(1, 5), 251], [(1, 207), 656], [(1, 107), 585], [(0, 204), 424], [(0, 206), 308], [(0, 105), 548]],
            [[(1, 5), 225], [(1, 207), 728], [(1, 107), 730], [(0, 204), 286], [(0, 206), 294], [(0, 105), 542]],
            [ARM_7_DROP_int],
            [[(1, 5), 260], [(1, 207), 527], [(1, 107), 525], [(0, 204), 338], [(0, 206), 298], [(0, 105), 556], ARM_7_DROP_int],
            [[(1, 5), 260-40]]
            ]
            yield ReadArmTraj(arm_traj, delay=300, reverse=True)
            yield Timer(100)

        if self.robot.team == TEAM_RIGHT:
            yield ArmSpeed(200)

            arm_traj_1 = [
            [[(1, 207), 441], [(1, 107), 323+27], [(0, 204), 484], [(0, 206), 240], [(0, 105), 560]],
            [[(1, 5), 412], [(0, 105), 450]],
            [[(1, 5), 529], [(1, 207), 474], [(1, 107), 342+26], [(0, 204), 494], [(0, 206), 231], [(0, 105), 373]],
            [[(1, 5), 546], [(1, 207), 590], [(1, 107), 455+30], [(0, 204), 494], [(0, 206), 245], [(0, 105), 373]]
            ]
            yield ReadArmTraj(arm_traj_1, delay=100)#, reverse=True)

            yield Timer(300)
            # Kick dropped modules and then drop
            shift_up = 30
            arm_traj_2 = [
            [[(0, 105), 345]],
            [[(1, 5), 586], [(1, 207), 662], [(1, 107), 560+shift_up], [(0, 204), 494], [(0, 206), 285]],
            ]
            yield ReadArmTraj(arm_traj_2, delay=50, reverse=True)

            yield Timer(100)
            yield Trigger(ARM_7_DROP)

            yield Timer(100)
            yield ArmSpeed(100)
            arm_traj_2 = [
            [[(1, 5), 576], [(1, 207), 592], [(1, 107), 544+shift_up], [(0, 204), 504], [(0, 206), 330], [(0, 105), 362]],
            [[(1, 5), 535], [(1, 207), 469], [(1, 107), 426], [(0, 204), 504], [(0, 206), 329], [(0, 105), 387]],
            [[(1, 107), 426+shift_up+20]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=100, reverse=True)

            yield Timer(200)

            arm_traj_2 = [
            [[(1, 207), 387], [(1, 107), 400+shift_up+40], [(0, 204), 512], [(0, 206), 367], [(0, 105), 497]]
            ]
            yield ReadArmTraj(arm_traj_2, delay=100, reverse=True)
            yield Timer(200)

            if self.finalKick == True:
                yield ArmSpeed(300)
                yield Timer(100)
                yield Trigger(ARM_7_INIT)
                arm_traj = [
                [[(1, 5), 433]],
                [[(1, 207), 437], [(1, 107), 366+40], [(0, 204), 512], [(0, 206), 363], [(0, 105), 503]]
                ]
                yield ReadArmTraj(arm_traj, delay=100)
                yield Timer(200)
                yield ArmSpeed(150)
                arm_traj = [
                [[(1, 5), 541], [(1, 207), 546], [(1, 107), 542], [(0, 204), 571], [(0, 206), 450], [(0, 105), 402]],
                ]
                yield ReadArmTraj(arm_traj)

                yield Timer(300)
                yield ArmSpeed(300)
                arm_traj = [
                [[(1, 5), 434]]
                ]
                yield ReadArmTraj(arm_traj)
                yield ArmSpeed(150)

        yield None

class GrabPolyModuleFromDropZone(State):
    def __init__(self, previousModuleTurned=False):
        self.previousModuleTurned = previousModuleTurned

    def on_enter(self):
        if self.robot.team == TEAM_LEFT:
            yield Trigger(ARM_7_OPEN)
            yield ArmSpeed(300)
            arm_traj = [
            [[(1, 5), 284], [(1, 207), 371], [(1, 107), 336], [(0, 204), 476], [(0, 206), 345], [(0, 105), 634]],
            [[(1, 5), 356], [(1, 207), 364], [(1, 107), 231], [(0, 204), 476], [(0, 206), 250], [(0, 105), 555+30]],
            [[(1, 5), 425], [(1, 207), 411], [(1, 107), 235], [(0, 204), 476], [(0, 206), 203], [(0, 105), 479+30]],
            [[(1, 5), 480], [(1, 207), 567], [(1, 107), 329], [(0, 204), 475], [(0, 206), 160], [(0, 105), 422+30]],
            [[(1, 5), 525], [(1, 207), 644], [(1, 107), 427], [(0, 204), 476], [(0, 206), 166], [(0, 105), 380+20]]
            ]
            yield ReadArmTraj(arm_traj, delay=20, reverse=True)
            yield Timer(50)
            arm_traj = [
            [[(1, 5), 546], [(1, 207), 696], [(1, 107), 529], [(0, 204), 483], [(0, 206), 214], [(0, 105), 369]]
            ]
            yield ReadArmTraj(arm_traj, delay=100, reverse=True)
            yield Trigger(ARM_7_HOLD)

        if self.robot.team == TEAM_RIGHT:
            yield Trigger(ARM_7_OPEN)
            if self.previousModuleTurned == False:
                yield ArmSpeed(300)
                arm_traj_1 = [
                [[(1, 5), 561], [(1, 207), 519], [(1, 107), 386], [(0, 204), 493], [(0, 206), 257], [(0, 105), 377]],
                [[(1, 5), 546], [(1, 207), 353], [(1, 107), 201], [(0, 204), 473], [(0, 206), 159], [(0, 105), 426]],
                [[(1, 5), 284]],
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
                yield ArmSpeed(300)
                arm_traj_1 = [
                [[(1, 5), 550], [(1, 207), 272], [(1, 107), 380], [(0, 204), 511], [(0, 206), 481], [(0, 105), 418]],
                [[(1, 5), 416], [(1, 207), 255], [(1, 107), 375], [(0, 204), 510], [(0, 206), 497], [(0, 105), 440]],
                [[(1, 5), 232], [(1, 207), 257], [(1, 107), 373], [(0, 204), 495], [(0, 206), 499], [(0, 105), 640]],
                [[(1, 5), 241], [(1, 207), 345], [(1, 107), 320], [(0, 204), 484], [(0, 206), 361], [(0, 105), 656]]
                ]
                yield ReadArmTraj(arm_traj_1, delay=50, reverse=True)
                yield Timer(100)
                yield ArmSpeed(150)

            arm_traj = [
            [[(1, 5), 241], [(1, 207), 507], [(1, 107), 308], [(0, 204), 483], [(0, 206), 201], [(0, 105), 656]],

            [[(1, 5), 304], [(1, 207), 678], [(1, 107), 504], [(0, 204), 495], [(0, 206), 207], [(0, 105), 617]],
            [[(1, 5), 311], [(1, 207), 663], [(1, 107), 476], [(0, 204), 492], [(0, 206), 203], [(0, 105), 614]]
            ]
            yield ReadArmTraj(arm_traj, delay=100, reverse=True)

            yield Timer(100)
            yield ArmSpeed(110)

            yield Trigger(ARM_7_HOLD)

        yield None


class GrabModuleFromStorageReturn(State):
    def on_enter(self):
        yield ArmSpeed(300)

        arm_traj = [
        [[(1, 5), 414], [(1, 207), 526], [(1, 107), 299+20], [(0, 204), 488], [(0, 206), 160], [(0, 105), 512]],
        [[(1, 5), 417], [(1, 207), 693], [(1, 107), 545], [(0, 204), 488], [(0, 206), 244], [(0, 105), 502]]
        ]
        yield ReadArmTraj(arm_traj, delay=50)

        yield Timer(50)

        arm_traj = [
        [[(1, 5), 413], [(1, 207), 716], [(1, 107), 570], [(0, 204), 498], [(0, 206), 242], [(0, 105), 503]],
        [[(1, 5), 425], [(1, 207), 758], [(1, 107), 632], [(0, 204), 496], [(0, 206), 259], [(0, 105), 518]]
        ]
        yield ReadArmTraj(arm_traj, delay=50, reverse=True)

        yield Timer(50)

        yield Trigger(ARM_7_HOLD)

        yield Timer(100)
        yield ArmSpeed(250)

        yield ReadArmTraj([
        [[(1, 5), 423], [(1, 207), 631], [(1, 107), 441], [(0, 204), 498], [(0, 206), 208], [(0, 105), 506]],
        [[(1, 5), 426], [(1, 207), 498], [(1, 107), 279], [(0, 204), 497], [(0, 206), 160], [(0, 105), 494]]
        ], 50)

        yield Timer(50)

        yield None


class StockModuleFromGrabbedModuleRightFront(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_OPEN)

        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(300)
        yield ReadArmTraj([[[ARM_1_ID, 317]]])

        # Montee du module a vitesse plus lente
        yield ArmSpeed(200)

        arm_traj_up = [
        [[(1, 5), 312], [(1, 207), 500], [(1, 107), 376], [(0, 204), 500], [(0, 206), 240], [(0, 105), 487]],
        [[(1, 5), 286], [(1, 207), 442], [(1, 107), 358], [(0, 204), 484], [(0, 206), 297], [(0, 105), 553]]
        ]
        yield ReadArmTraj(arm_traj_up, 100)

        arm_traj_up = [
        [[(1, 5), 280], [(1, 207), 381], [(1, 107), 369], [(0, 204), 484], [(0, 206), 357], [(0, 105), 590]],
        [[(1, 5), 277], [(1, 207), 356], [(1, 107), 424], [(0, 204), 485], [(0, 206), 434], [(0, 105), 641]],
        [[(1, 5), 272], [(1, 207), 339], [(1, 107), 469], [(0, 204), 485], [(0, 206), 496], [(0, 105), 668]],
        [[(1, 5), 266], [(1, 207), 366], [(1, 107), 533+35], [(0, 204), 490], [(0, 206), 524], [(0, 105), 680]],
        [[(1, 5), 260], [(1, 207), 409], [(1, 107), 596+35], [(0, 204), 491], [(0, 206), 551], [(0, 105), 690]],
        [[(1, 5), 260], [(1, 207), 423], [(1, 107), 619+35], [(0, 204), 490], [(0, 206), 557], [(0, 105), 690]]
        ]
        yield ReadArmTraj(arm_traj_up, 30)

        yield Timer(100)
        yield Trigger(STORAGE_FINGER_RIGHT_FRONT_HOLD)

        yield Timer(200)
        yield Trigger(ARM_7_DROP)

        yield ArmSpeed(150)

        arm_traj_down_1 = [
        [[(1, 5), 257], [(1, 207), 397], [(1, 107), 585], [(0, 204), 421], [(0, 206), 555], [(0, 105), 703]],
        [[(1, 5), 250], [(1, 207), 315], [(1, 107), 460], [(0, 204), 421], [(0, 206), 555], [(0, 105), 703]],
        [[(1, 5), 250], [(1, 207), 278], [(1, 107), 361-20], [(0, 204), 411], [(0, 206), 555], [(0, 105), 668]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 80, reverse = True)

        yield Timer(20)
        yield ArmSpeed(200)

        arm_traj_down_2 = [
        [[(1, 5), 429], [(1, 207), 275], [(1, 107), 364], [(0, 204), 492], [(0, 206), 500], [(0, 105), 512]],
        [[(1, 5), 421], [(1, 207), 319], [(1, 107), 334], [(0, 204), 494], [(0, 206), 320], [(0, 105), 525]],
        [[(1, 5), 416], [(1, 207), 367], [(1, 107), 227], [(0, 204), 494], [(0, 206), 261], [(0, 105), 523]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 30, reverse = True)
        yield Trigger(ARM_7_OPEN)

        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))

        yield None


class StockModuleFromGrabbedModuleRight(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_RIGHT_OPEN)

        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(300)
        yield ReadArmTraj([[[ARM_1_ID, 317]]], delay=100)

        yield ArmSpeed(300)

        arm_traj_up = [
        [[(1, 5), 314], [(1, 207), 435], [(1, 107), 437 -30], [(0, 204), 497], [(0, 206), 386], [(0, 105), 494]],
        [[(1, 5), 183], [(1, 207), 366], [(1, 107), 399], [(0, 204), 505], [(0, 206), 388], [(0, 105), 486]]
        ]
        yield ReadArmTraj(arm_traj_up, delay = 100)
        yield Timer(200)
        arm_traj_up = [
        [[(1, 5), 183], [(1, 207), 321], [(1, 107), 418], [(0, 204), 505], [(0, 206), 449], [(0, 105), 485]],
        [[(1, 5), 184], [(1, 207), 392], [(1, 107), 575 + 30], [(0, 204), 505], [(0, 206), 534], [(0, 105), 483]]
        ]
        yield ReadArmTraj(arm_traj_up, delay = 100)

        yield Timer(500)
        yield Trigger(STORAGE_FINGER_RIGHT_HOLD)

        yield Timer(200)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 197], [(1, 207), 303], [(1, 107), 438], [(0, 204), 499], [(0, 206), 479], [(0, 105), 435]],
        [[(1, 5), 197], [(1, 207), 255], [(1, 107), 405], [(0, 204), 499], [(0, 206), 483], [(0, 105), 436]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 100, reverse = True)

        yield Timer(100)
        yield ArmSpeed(200)

        arm_traj_down_2 = [
        [[(1, 5), 201], [(1, 207), 310], [(1, 107), 342], [(0, 204), 500], [(0, 206), 422], [(0, 105), 437]],
        [[(1, 5), 358], [(1, 207), 308], [(1, 107), 343], [(0, 204), 500], [(0, 206), 423], [(0, 105), 461]],
        [[(1, 5), 433], [(1, 207), 371], [(1, 107), 275], [(0, 204), 499], [(0, 206), 299], [(0, 105), 491]],
        [[(1, 5), 430], [(1, 207), 375], [(1, 107), 215], [(0, 204), 500], [(0, 206), 224], [(0, 105), 491]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))

        yield None

class StockModuleFromGrabbedModuleLeftFront(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_LEFT_FRONT_OPEN)

        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(300)
        yield ReadArmTraj([[[ARM_1_ID, 517]]], delay=100)

        # Montee du module a vitesse plus lente
        yield ArmSpeed(300)

        arm_traj_up = [
        [[(1, 5), 523], [(1, 207), 472], [(1, 107), 513], [(0, 204), 507], [(0, 206), 412], [(0, 105), 494]],
        [[(1, 5), 563], [(1, 207), 360], [(1, 107), 447], [(0, 204), 506], [(0, 206), 453], [(0, 105), 424]]
        ]
        yield ReadArmTraj(arm_traj_up, 100)
        yield Timer(200)
        arm_traj_up = [
        [[(1, 5), 596], [(1, 207), 351], [(1, 107), 498 +20], [(0, 204), 506], [(0, 206), 509], [(0, 105), 398]],
        [[(1, 5), 596 + 10], [(1, 207), 394], [(1, 107), 583 +40], [(0, 204), 506], [(0, 206), 554], [(0, 105), 398]]
        ]
        yield ReadArmTraj(arm_traj_up, 100)

        yield Timer(500)
        yield Trigger(STORAGE_FINGER_LEFT_FRONT_HOLD)

        yield Timer(200)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 617], [(1, 207), 317], [(1, 107), 529], [(0, 204), 510], [(0, 206), 573], [(0, 105), 397]],
        [[(1, 5), 658], [(1, 207), 348], [(1, 107), 497], [(0, 204), 531], [(0, 206), 656], [(0, 105), 272]],
        [[(1, 5), 670], [(1, 207), 272], [(1, 107), 398], [(0, 204), 629], [(0, 206), 512], [(0, 105), 265]],
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 300, reverse = True)

        yield Timer(300)
        arm_traj_down_2 = [
        [[(1, 5), 505], [(1, 207), 278], [(1, 107), 373], [(0, 204), 628], [(0, 206), 493], [(0, 105), 265]],
        [[(1, 5), 450], [(1, 207), 291], [(1, 107), 354], [(0, 204), 517], [(0, 206), 478], [(0, 105), 488]],
        [[(1, 5), 430+12], [(1, 207), 375], [(1, 107), 215], [(0, 204), 500], [(0, 206), 224], [(0, 105), 491]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 10, reverse = True)
        yield Trigger(ARM_7_OPEN)
        yield Timer(100)
        yield Trigger(makeServoMoveCommand((ARM_1_ID, 700), 416))

        yield None

class StockModuleFromGrabbedModuleLeft(State):
    def on_enter(self):
        yield Trigger(STORAGE_FINGER_LEFT_OPEN)

        # Rotation seule et rapide de la base du bras
        yield ArmSpeed(300)
        yield ReadArmTraj([[[ARM_1_ID, 541]]], delay=100)

        yield ArmSpeed(300)

        arm_traj_up = [
        [[(1, 5), 536], [(1, 207), 401], [(1, 107), 402-15], [(0, 204), 505], [(0, 206), 376], [(0, 105), 494]],
        [[(1, 5), 694], [(1, 207), 393], [(1, 107), 427], [(0, 204), 513], [(0, 206), 404], [(0, 105), 547]],
        [[(1, 5), 687], [(1, 207), 360], [(1, 107), 486 + 30], [(0, 204), 511], [(0, 206), 483], [(0, 105), 547]],
        [[(1, 5), 685], [(1, 207), 414], [(1, 107), 599 + 30], [(0, 204), 505], [(0, 206), 546], [(0, 105), 548]]
        ]
        yield ReadArmTraj(arm_traj_up, delay=100, reverse=True)

        yield Timer(500)
        yield Trigger(STORAGE_FINGER_LEFT_HOLD)

        yield Timer(200)
        yield Trigger(ARM_7_DROP)

        yield Timer(100)
        arm_traj_down_1 = [
        [[(1, 5), 664], [(1, 207), 353], [(1, 107), 527], [(0, 204), 495], [(0, 206), 535], [(0, 105), 580]],
        [[(1, 5), 664], [(1, 207), 271], [(1, 107), 366], [(0, 204), 503], [(0, 206), 463], [(0, 105), 580]],
        [[(1, 5), 664], [(1, 207), 314], [(1, 107), 340], [(0, 204), 502], [(0, 206), 463], [(0, 105), 579]]
        ]
        yield ReadArmTraj(arm_traj_down_1, delay = 300, reverse = True)

        yield Timer(100)
        arm_traj_down_2 = [
        [[(1, 5), 436], [(1, 207), 314], [(1, 107), 339], [(0, 204), 501], [(0, 206), 465], [(0, 105), 579]],
        [[(1, 5), 430], [(1, 207), 375], [(1, 107), 215], [(0, 204), 500], [(0, 206), 224], [(0, 105), 491]]
        ]
        yield ReadArmTraj(arm_traj_down_2, delay = 50, reverse = True)
        yield Trigger(ARM_7_OPEN)

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


class ClearFirstModule(State):
    def on_enter(self):
        yield ArmSpeed(200)
        arm_traj = [
        [[(1, 5), 411], [(1, 207), 400], [(1, 107), 401 +30], [(0, 204), 495], [(0, 206), 427], [(0, 105), 0]], 
        [[(1, 5), 401], [(1, 207), 450], [(1, 107), 313 +30], [(0, 204), 495], [(0, 206), 237], [(0, 105), 0]], 
        [[(1, 5), 299], [(1, 207), 584], [(1, 107), 434 +30], [(0, 204), 435], [(0, 206), 228], [(0, 105), 0]]
        ]
        yield ReadArmTraj(arm_traj, delay=300)

        yield None
