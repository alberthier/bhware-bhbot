# encoding: utf-8


import functools
import math
import random

import logger
import packets
import position
import statemachine
import tools
import metrics
import checks
from definitions import *


class StateChain(statemachine.State):

    def __init__(self, *args):
        self.states = args
        self.dbg('StateChain : {}'.format(self.states))


    def __getattr__(self, item):
        if item.startswith('on_'):
            method = functools.partial(self.handle_event, item)
            setattr(self, item, method)
            return method
        raise AttributeError()


    def on_enter(self, *args, **kwargs):
        for s in self.states :
            self.fsm.init_state(s)

        real_ret = None
        for state in self.states :
            ret = state.on_enter(*args, **kwargs)
            if ret :
                real_ret = ret
        return real_ret


    def handle_event(self, event_name, *args, **kwargs):
        self.dbg('StateChain.handle_event : {}'.format(event_name))
        for state in self.states :
            method = getattr(state, event_name, None)
            if method :
                ret = method(*args, **kwargs)
                if ret :
                    return ret
        return None




class WaitPackets(statemachine.State):

    def __init__(self, *args):
        self.packet_classes = [ pc for pc in args ]


    def on_packet(self, packet):
        for i in range(len(self.packet_classes)):
            if isinstance(packet, self.packet_classes[i]):
                del self.packet_classes[i]
                if len(self.packet_classes) == 0:
                    yield None
                    return
                break




class SendPacketAndWait(statemachine.State):

    def __init__(self, packet_to_send, packet_class_to_wait = None):
        super().__init__()
        self.packet_to_send = packet_to_send
        if packet_class_to_wait is None:
            self.packet_class_to_wait = type(packet_to_send)
        else:
            self.packet_class_to_wait = packet_class_to_wait


    def on_enter(self):
        self.dbg("Sending packet {}".format(self.packet_to_send))
        self.send_packet(self.packet_to_send)
        self.dbg('Waiting for packet type {}'.format(self.packet_class_to_wait.__name__))


    def on_packet(self, packet):
        if isinstance(packet, self.packet_class_to_wait):
            self.dbg("Got expected packet, exiting state")
            yield None




class SendPacketsAndWaitAnswer(statemachine.State):

    def __init__(self, *packets):
        super().__init__()
        self.packets = set(packets)
        """ :type self.packets: set of packets.BasePacket"""


    def on_enter(self):
        for p in self.packets :
            self.dbg("Sending packet {}".format(p))
            self.send_packet(p)
            self.dbg('Waiting for packet type {}'.format(p.name))


    def on_packet(self, packet):
        for p in self.packets :
            if type(p) == type(packet):
                self.dbg("Got expected packet {}".format(packet.name))
                self.packets.remove(p)
                break
        if not self.packets :
            self.dbg('No more packets to wait, exiting state')
            yield None




class DefinePosition(statemachine.State):

    def __init__(self, x = None, y = None, angle = None):
        statemachine.State.__init__(self)
        self.has_x = x is not None
        self.has_y = y is not None
        px = x if self.has_x else 0.0
        py = y if self.has_y else 0.0
        pangle = angle if angle is not None else 0.0
        self.pose = position.Pose(px, py, pangle, True)


    def on_enter(self):
        if self.has_x:
            packet = packets.Resettle()
            packet.axis = AXIS_X
            packet.position = self.pose.x
            packet.angle = self.pose.angle
            yield SendPacketAndWait(packet, packets.Resettle)
            self.robot.pose.x = self.pose.x
            self.robot.pose.angle = self.pose.angle

        if self.has_y:
            packet = packets.Resettle()
            packet.axis = AXIS_Y
            packet.position = self.pose.y
            packet.angle = self.pose.angle
            yield SendPacketAndWait(packet, packets.Resettle)
            self.robot.pose.y = self.pose.y
            self.robot.pose.angle = self.pose.angle

        yield None




class AntiBlocking(statemachine.State):

    def __init__(self, desired_status):
        if desired_status :
            self.packet = packets.EnableAntiBlocking()
        else :
            self.packet = packets.DisableAntiBlocking()


    def on_enter(self):
        self.send_packet(self.packet)


    def on_enable_anti_blocking(self, packet):
        yield None


    def on_disable_anti_blocking(self, packet):
        yield None




class SpeedControl(statemachine.State):

    def __init__(self, speed = None):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()
        if speed is not None:
            self.packet.vmax_limit = speed
        else:
            self.packet.vmax_limit = ROBOT_VMAX_LIMIT


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_config(self, packet):
        yield None


class RatioDeccControl(statemachine.State):

    def __init__(self, ratio = None):
        statemachine.State.__init__(self)
        self.packet = packets.PositionControlConfig()
        if ratio is not None:
            self.packet.ratio_decc = ratio
        else:
            self.packet.ratio_decc = ROBOT_DEFAULT_RATIO_DECC


    def on_enter(self):
        self.send_packet(self.packet)


    def on_position_control_config(self, packet):
        yield None




class SetupTurret(statemachine.State):

    def __init__(self, short_distance, long_distance):
        self.short_distance = short_distance
        self.long_distance = long_distance


    def on_enter(self):
        packet = packets.TurretInit(TURRET_INIT_MODE_WRITE, self.short_distance, self.long_distance)
        self.send_packet(packet)


    def on_turret_distances(self, packet):
        yield None




class WaitForOpponentLeave(statemachine.Timer):

    TIMEOUT       = 0
    OPPONENT_LEFT = 100

    def __init__(self, opponent, miliseconds, move_direction, retries):
        if miliseconds is None :
            miliseconds = DEFAULT_OPPONENT_WAIT_MS
        self.wait_milis = miliseconds
        statemachine.Timer.__init__(self, miliseconds)
        self.opponent = opponent
        self.move_direction = move_direction
        self.retries = retries
        if not self.retries :
            self.retries = DEFAULT_OPPONENT_DISAPPEAR_RETRIES


    def on_enter(self):
        logger.log('WaitForOpponentLeave : time={}, retries={}'.format(self.wait_milis, self.retries))
        statemachine.Timer.on_enter(self)
        self.goto_finished = False
        self.opponent_disappeared = False
        self.timer_expired = False
        self.exit_reason = None
        if self.move_direction == DIRECTION_FORWARD:
            direction = DIRECTION_BACKWARDS
            distance = -0.150
        else:
            direction = DIRECTION_FORWARD
            distance = 0.150

        current_pose = self.robot.pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * distance
        packet = packets.MoveLine()
        packet.direction = direction
        packet.points = [ position.Pose(x, y, None, True) ]
        self.send_packet(packet)


    def on_timeout(self):
        logger.log('WaitForOpponentLeave : on_timeout')
        self.timer_expired = True
        if self.exit_reason is None:
            self.exit_reason = self.TIMEOUT
        return self.try_leave()


    def on_goto_finished(self, packet):
        logger.log('WaitForOpponentLeave : on_goto_finished')
        self.goto_finished = True
        return self.try_leave()


    def on_opponent_disappeared(self, packet):
        logger.log('WaitForOpponentLeave : on_opponent_disappeared {}'.format(packet.robot == self.opponent))
        if packet.robot == self.opponent:
            self.exit_reason = self.OPPONENT_LEFT
            self.opponent_disappeared = True
            if not self.goto_finished:
                self.send_packet(packets.Stop())
            return self.try_leave()


    def try_leave(self):
        logger.log('WaitForOpponentLeave : try_leave goto_finished={} timer_expired={} opponent_disappeared={}'.format(self.goto_finished, self.timer_expired, self.opponent_disappeared))
        if self.goto_finished and (self.timer_expired or self.opponent_disappeared):
            yield None
        if self.retries >= 0 :
            if self.timer_expired :
                self.retries-=1
                if self.retries < 0 :
                    logger.log('WaitForOpponentLeave : retries exceeded')
                    self.exit_reason = TRAJECTORY_BLOCKED
                    yield None
                else :
                    logger.log('WaitForOpponentLeave : retries remaining = {}'.format(self.retries))
        logger.log('WaitForOpponentLeave : not leaving')




class OpponentHandlingConfig:
    def __init__(self, stop, raise_exception, backout, retries: int or None=None, wait_delay: float or None=None):
        self.stop = stop
        self.raise_exception = raise_exception
        self.backout = backout
        self.retries_count = retries
        self.wait_delay = wait_delay




class OpponentInTheWay(Exception):

    def __init__(self, opponent):
        self.opponent = opponent




OPPONENT_HANDLING_NONE = OpponentHandlingConfig(False, False, False, None, None)
OPPONENT_HANDLING_STOP = OpponentHandlingConfig(True, False, True, 0, None)
OPPONENT_HANDLING_RAISE = OpponentHandlingConfig(True, True, False, 0, None)




class AbstractMove(statemachine.State):

    def __init__(self, opponent_handling_config: OpponentHandlingConfig):
        self.current_opponent = None
        self.opponent_handling_config = opponent_handling_config


    def on_enter(self):
        if self.opponent_handling_config.stop and self.packet.direction in [ self.robot.main_opponent_direction, self.robot.secondary_opponent_direction ]:
            if self.robot.main_opponent_direction is not None:
                self.current_opponent = OPPONENT_ROBOT_MAIN
            else:
                self.current_opponent = OPPONENT_ROBOT_SECONDARY
            yield from self.handle_opponent_detected(None)
        else:
            self.send_packet(self.packet)


    def on_opponent_detected(self, packet):
        if self.opponent_handling_config.stop and self.packet.direction == packet.direction and self.current_opponent is None:
            self.log("Opponent detected. direction = {}. Stop robot".format(packet.direction))
            self.send_packet(packets.Stop())
            self.current_opponent = packet.robot


    def on_goto_finished(self, packet):
        if packet.reason == REASON_DESTINATION_REACHED:
            self.exit_reason = TRAJECTORY_DESTINATION_REACHED
            yield None
        elif packet.reason == REASON_BLOCKED_FRONT or packet.reason == REASON_BLOCKED_BACK:
            self.exit_reason = TRAJECTORY_BLOCKED
            yield None
        elif packet.reason == REASON_STOP_REQUESTED:
            if self.current_opponent is not None:
                if hasattr(self.packet, "points"):
                    index = packet.current_point_index
                else:
                    index = None
                yield from self.handle_opponent_detected(index)
            else:
                self.exit_reason = TRAJECTORY_STOP_REQUESTED
                yield None


    def handle_opponent_detected(self, point_index):
        if self.opponent_handling_config.backout:
            leave_state = yield WaitForOpponentLeave(self.current_opponent, self.opponent_handling_config.wait_delay,
                                                     self.packet.direction, self.opponent_handling_config.retries_count)
            reason = leave_state.exit_reason
            self.current_opponent = None
            self.log("AbstractMove: handle_opponent_detected {} {}".format(reason, reason in (WaitForOpponentLeave.TIMEOUT, TRAJECTORY_BLOCKED)))
            if reason in (WaitForOpponentLeave.TIMEOUT, TRAJECTORY_BLOCKED) :
                self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
                self.log("AbstractMove: handle_opponent_detected leaving state")
                yield None
            else:
                if point_index is not None:
                    self.packet.points = self.packet.points[point_index:]
                self.log("AbstractMove: handle_opponent_detected replay")
                self.send_packet(self.packet)
        else:
            self.log("Opponent detected, cancelling log")
            self.exit_reason = TRAJECTORY_OPPONENT_DETECTED
            if self.opponent_handling_config.raise_exception:
                raise OpponentInTheWay(self.current_opponent)
            else:
                yield None


    def get_direction(self, x, y):
        return tools.get_direction(self.robot.pose, x, y)




class RotateTo(AbstractMove):

    def __init__(self, angle, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_NONE):
        super().__init__(opponent_handling_config)
        pose = position.Pose(0.0, 0.0, angle, virtual)
        self.packet = packets.Rotate(direction = direction, angle = pose.angle)




class SafeRotateTo(RotateTo):

    def __init__(self, angle, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(angle, direction, virtual, OPPONENT_HANDLING_RAISE)




class LookAt(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_NONE):
        super().__init__(opponent_handling_config)
        self.pose = position.Pose(x, y, None, virtual)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx)
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class SafeLookAt(LookAt):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(x, y, direction, virtual, OPPONENT_HANDLING_RAISE)




class LookAtOpposite(AbstractMove):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_NONE):
        super().__init__(opponent_handling_config)
        self.pose = position.Pose(x, y, None, virtual)
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        dx = self.pose.x - current_pose.x
        dy = self.pose.y - current_pose.y
        angle = math.atan2(dy, dx) + math.pi
        self.packet = packets.Rotate(direction = self.direction, angle = angle)
        return AbstractMove.on_enter(self)




class SafeLookAtOpposite(LookAtOpposite):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(x, y, direction, virtual, OPPONENT_HANDLING_RAISE)




class MoveCurve(AbstractMove):

    def __init__(self, angle, min_curve_radius, points, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_STOP):
        super().__init__(opponent_handling_config)
        apose = position.Pose(0.0, 0.0, angle, virtual)
        poses = []
        for pt in points:
            if isinstance(pt, tuple):
                poses.append(position.Pose(pt[0], pt[1], None, virtual))
            else:
                poses.append(pt)
        self.packet = packets.MoveCurve(direction, apose.angle, min_curve_radius, poses)


    def on_enter(self):
        if self.packet.direction == DIRECTION_AUTO:
            first_point = self.packet.points[0]
            self.packet.direction = self.get_direction(first_point.virt.x, first_point.virt.y)
        yield from super().on_enter()




class SafeMoveCurve(MoveCurve):

    def __init__(self, angle, min_curve_radius, points, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(angle, min_curve_radius, points, direction, virtual, OPPONENT_HANDLING_RAISE)




class MoveCurveTo(MoveCurve):

    def __init__(self, angle, min_curve_radius, pose, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_STOP):
        super().__init__(angle, min_curve_radius, [pose], direction, virtual, opponent_handling)




class SafeMoveCurveTo(MoveCurveTo):

    def __init__(self, angle, min_curve_radius, pose, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(angle, min_curve_radius, pose, direction, virtual, OPPONENT_HANDLING_RAISE)




class MoveLine(AbstractMove):

    def __init__(self, points, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_STOP):
        checks.check_type(direction, int)
        super().__init__(opponent_handling_config)
        poses = []
        for pt in points:
            if type(pt) == tuple:
                poses.append(position.Pose(pt[0], pt[1], None, virtual))
            else:
                poses.append(pt)
        self.packet = packets.MoveLine(direction = direction, points = poses)


    def on_enter(self):
        if self.packet.direction == DIRECTION_AUTO:
            first_point = self.packet.points[0]
            self.packet.direction = self.get_direction(first_point.virt.x, first_point.virt.y)
        yield from super().on_enter()




class SafeMoveLine(MoveLine):

    def __init__(self, points, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(points, direction, virtual, OPPONENT_HANDLING_RAISE)




class MoveLineTo(MoveLine):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_STOP):
        super().__init__([position.Pose(x, y, None, virtual)], direction, virtual, opponent_handling_config)




class SafeMoveLineTo(MoveLineTo):

    def __init__(self, x, y, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(x, y, direction, virtual, OPPONENT_HANDLING_RAISE)




class MoveLineRelative(statemachine.State):

    def __init__(self, distance, direction = DIRECTION_AUTO, opponent_handling_config = OPPONENT_HANDLING_STOP):
        self.opponent_handling_config = opponent_handling_config
        if direction != DIRECTION_AUTO:
            self.distance = distance * direction
        else:
            self.distance = distance
        self.direction = direction


    def on_enter(self):
        current_pose = self.robot.pose
        x = current_pose.virt.x + math.cos(current_pose.virt.angle) * self.distance
        y = current_pose.virt.y + math.sin(current_pose.virt.angle) * self.distance
        move = yield MoveLineTo(x, y, DIRECTION_AUTO, True, self.opponent_handling_config)
        self.exit_reason = move.exit_reason
        yield None




class SafeMoveLineRelative(MoveLineRelative):

    def __init__(self, distance, direction = DIRECTION_AUTO):
        super().__init__(distance, direction, OPPONENT_HANDLING_RAISE)




class RotateRelative(statemachine.State):

    def __init__(self, relative_angle, direction = DIRECTION_AUTO, opponent_handling_config = OPPONENT_HANDLING_NONE):
        self.relative_angle = relative_angle
        self.direction = direction
        self.opponent_handling_config = opponent_handling_config


    def on_enter(self):
        current_pose = self.robot.pose
        move = yield RotateTo(current_pose.angle + self.relative_angle, self.direction, self.opponent_handling_config)
        self.exit_reason = move.exit_reason
        yield None




class SafeRotateRelative(RotateRelative):

    def __init__(self, relative_angle, direction = DIRECTION_AUTO):
        super().__init__(relative_angle, direction, OPPONENT_HANDLING_RAISE)




class MoveArc(AbstractMove):

    def __init__(self, center_x, center_y, radius, points, direction = DIRECTION_AUTO, virtual = True, opponent_handling_config = OPPONENT_HANDLING_STOP):
        super().__init__(opponent_handling_config)
        cpose = position.Pose(center_x, center_y, None, virtual)
        angles = []
        for a in points:
            apose = position.Pose(0.0, 0.0, a, virtual)
            angles.append(apose.angle)
        self.packet = packets.MoveArc(direction = direction, center = cpose, radius = radius, points = angles)


    def on_enter(self):
        if self.packet.direction == DIRECTION_AUTO:
            first_angle = self.packet.points[0].virt.angle
            first_point = position.Pose(center_x + math.cos(first_angle) * self.packet.radius,
                                        center_y + math.sin(first_angle) * self.packet.radius,
                                        None, True)
            self.packet.direction = self.get_direction(first_point.virt.x, first_point.virt.y)
        yield from super().on_enter()




class SafeMoveArc(MoveArc):

    def __init__(self, center_x, center_y, radius, points, direction = DIRECTION_AUTO, virtual = True):
        super().__init__(center_x, center_y, radius, points, direction, virtual, OPPONENT_HANDLING_RAISE)




class FollowPath(statemachine.State):

    def __init__(self, path, direction = DIRECTION_AUTO):
        super().__init__()
        self.path = path
        self.direction = direction
        self.exit_reason = None


    def on_enter(self):
        dest = self.path[-1]
        self.robot.destination = position.Pose(dest.x, dest.y, 0.0)
        if self.direction == DIRECTION_AUTO:
            first_point = self.path[0]
            self.direction = tools.get_direction(self.robot.pose, first_point.virt.x, first_point.virt.y)
        for pose in self.path:
            assert isinstance(pose, position.Pose)
            if self.direction == DIRECTION_FORWARD:
                if not self.robot.is_looking_at(pose):
                    move = yield LookAt(pose.virt.x, pose.virt.y, DIRECTION_FORWARD)
            else:
                if not self.robot.is_looking_at_opposite(pose):
                    move = yield LookAtOpposite(pose.virt.x, pose.virt.y, DIRECTION_FORWARD)
            move = self.create_move_line(pose, self.direction)
            yield move
            if move.exit_reason != TRAJECTORY_DESTINATION_REACHED:
                break
        self.robot.destination = None
        self.exit_reason = move.exit_reason
        yield None


    def create_move_line(self, pose, direction):
        return MoveLine([pose], direction)




class SafeFollowPath(FollowPath):

    def create_move_line(self, pose, direction):
        return SafeMoveLine(pose, direction)




class Navigate(statemachine.State):

    def __init__(self, x, y, direction = DIRECTION_AUTO, offset = 0):
        statemachine.State.__init__(self)
        self.destination = position.Pose(x, y, None, True)
        self.direction = direction
        self.exit_reason = TRAJECTORY_DESTINATION_REACHED
        self.offset = offset


    def create_path(self):
        cost, path = self.event_loop.map.route(self.robot.pose, self.destination)
        if len(path) == 0 or self.offset == 0:
            return path
        else:
            p = self.robot.pose if len(path) == 1 else path[-2]
            a = tools.angle_between(p.x, p.y, self.destination.x, self.destination.y)
            dist = tools.distance(p.x, p.y, self.destination.x, self.destination.y)
            dist += self.offset
            last_pose = path[-1]
            last_pose.x = p.x + math.cos(a) * dist
            last_pose.y = p.y + math.sin(a) * dist
            return path


    def on_enter(self):
        path = self.create_path()
        self.log(str(path))
        if len(path) == 0:
            self.exit_reason = TRAJECTORY_DESTINATION_UNREACHABLE
            yield None
            return
        move = self.create_follow_path(path, self.direction)
        yield move
        self.direction = move.direction # fetch the real move direction in case of DIRECTION_AUTO for the caller
        self.exit_reason = move.exit_reason
        yield None


    def create_follow_path(self, path, direction):
        return FollowPath(path, direction)




class SafeNavigate(Navigate):

    def create_follow_path(self, path, direction):
        return SafeFollowPath(path, direction)




class GotoHome(Navigate):

    def __init__(self):
        Navigate.__init__(self, LEFT_START_X, LEFT_START_Y)




class Trigger(statemachine.Timer):

    TYPE               = 0
    ID                 = 1
    TYPED_ID           = 0
    SERVO_COMMAND      = 1
    SERVO_VALUE        = 2
    SERVO_TIMEOUT      = 3
    OUTPUT_ACTION      = 1
    PWM_VALUE          = 1

    def __init__(self, *args):
        """
            args can be 3 or 4 arguments: actuator_type, id, args... for a actuator control or
            a list of tuples (actuator_type, id, args...) for multiple motor control
            ex:
                Trigger(ACTUATOR_TYPE_SERVO_AX, 1, 154, 1000)
                Trigger((ACTUATOR_TYPE_SERVO_AX, 1, 154, 1000), (ACTUATOR_TYPE_OUTPUT, 2, ACTION_ON))
        """
        if len(args) > 0:
            if type(args[0]) == list:
                self.commands = args[0]
            elif type(args[0]) == tuple:
                self.commands = list(args)
            elif len(args) == 3 or len(args) == 4:
                self.commands = [ args ]
            else:
                raise TypeError("Invalid arguments")
        else:
            raise TypeError("Invalid arguments")

        self.status = False
        self.statuses = {}
        max_timeout = 0
        for cmd in self.commands:
            self.statuses[cmd[self.TYPED_ID]] = False
            if cmd[self.TYPED_ID][self.TYPE] in [ ACTUATOR_TYPE_SERVO_AX, ACTUATOR_TYPE_SERVO_RX ]:
                max_timeout = max(max_timeout, cmd[self.SERVO_TIMEOUT])
        super().__init__(max_timeout + 2000)


    def on_enter(self):
        for cmd in self.commands:
            actuator_typed_id = cmd[self.TYPED_ID]
            actuator_type = actuator_typed_id[self.TYPE]
            actuator_id = actuator_typed_id[self.ID]

            if actuator_type in [ ACTUATOR_TYPE_SERVO_AX, ACTUATOR_TYPE_SERVO_RX ]:
                servo_value = cmd[self.SERVO_VALUE]
                timeout_value = cmd[self.SERVO_TIMEOUT]

                logger.log("Trigger: set servo {} value: {} (tm: {})".format(
                    SERVOS_IDS.lookup_by_value[actuator_typed_id],
                    ALL_SERVO_COMMANDS[actuator_typed_id].get(servo_value, servo_value),
                    timeout_value)
                )
                self.send_packet(packets.ServoControl(*cmd))
            elif actuator_type == ACTUATOR_TYPE_OUTPUT:
                self.send_packet(packets.OutputControl(id = actuator_id, action = cmd[self.OUTPUT_ACTION]))
            elif actuator_type == ACTUATOR_TYPE_PWM:
                self.send_packet(packets.PwmControl(id = actuator_id, value = cmd[self.PWM_VALUE]))
            else:
                self.log_error("Unknown actuator type in command: {}".format(cmd))
                yield from self.cleanup(actuator_id)


    def on_servo_control(self, packet):
        if packet.id not in self.statuses:
            return
        if packet.status != SERVO_STATUS_SUCCESS:
            self.log_error("Servo #{} (id={}) timed out".format(SERVOS_IDS.lookup_by_value[packet.id], packet.id))
        self.statuses[packet.id] = packet.status == SERVO_STATUS_SUCCESS
        yield from self.cleanup(packet.id, packet.command)


    def on_output_control(self, packet):
        typed_id = (ACTUATOR_TYPE_OUTPUT, packet.id)
        self.statuses[typed_id] = True
        yield from self.cleanup(typed_id)


    def on_pwm_control(self, packet):
        typed_id = (ACTUATOR_TYPE_PWM, packet.id)
        self.statuses[typed_id] = True
        yield from self.cleanup(typed_id)


    def cleanup(self, typed_id, subcommand = None):
        for i, cmd in enumerate(self.commands):
            if cmd[self.TYPED_ID] == typed_id:
                if subcommand is None or subcommand == cmd[self.SERVO_COMMAND]:
                    del self.commands[i]
                break
        if len(self.commands) == 0:
            self.status = not (False in self.statuses)
            yield None


    def on_timeout(self):
        self.log_error("Trigger timed out !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        yield from super().on_timeout()




class ServoTorqueControl(statemachine.State):

    def __init__(self, servos, enabled):
        self.servos = servos
        self.enabled = enabled


    def on_enter(self):
        cmds = []
        for servo in self.servos:
            cmds.append(makeServoTorqueControl((servo, 0), self.enabled))
        if len(cmds) != 0:
            yield Trigger(*cmds)
        yield None




class ReadServoPosition(statemachine.State):

    def __init__(self, servo_id):
        self.servo_id = servo_id
        self.value = -1


    def on_enter(self):
        cmd = makeServoReadCommand((self.servo_id, 0))
        self.send_packet(packets.ServoControl(*cmd))


    def on_servo_control(self, packet):
        if packet.id == self.servo_id:
            self.value = packet.value
            yield None




class GetInputStatus(statemachine.State):

    def __init__(self, id):
        self.id = id


    def on_enter(self):
        self.send_packet(packets.InputStatusRequest(self.id))


    def on_input_status(self, packet):
        if packet.id == self.id and packet.kind == KIND_READ:
            self.value = packet.value
            yield None




class WaitForUnlock(statemachine.Timer):

    def __init__(self, lock_name, timeout):
        super().__init__(timeout)
        self.lock_name = lock_name


    def on_enter(self):
        if not self.robot.is_locked(self.lock_name):
            yield None


    def on_interbot_unlock(self, packet):
        if packet.lock_name == self.lock_name:
            yield None




##################################################
# GOAL MANAGEMENT
##################################################


class ExecuteGoalsBase(statemachine.State):

    def get_next_goal_simple(self, gm):
        pass

    def handle_navigation_failure(self):
        pass

    def on_enter(self):
        gm = self.robot.goal_manager

        navigation_failures = 0

        while True:

            if navigation_failures < 10:
                logger.log("Choosing the best goal")
                goal = self.get_next_goal_simple(gm)
            else:
                logger.log("Escaping to anywhere !!")
                yield EscapeToAnywhere()
                gm.whitelist_all()
                navigation_failures = 0
                continue

            if goal:
                logger.log('Next goal is {}'.format(goal.identifier))

                if gm.score_estimator:
                    gm.score_estimator.before_goal(goal, self.robot)

                goal.doing()

                current_navigation_succeeded = True
                if goal.navigate :
                    nav_timer=metrics.Timer("Navigation to {}".format(goal.identifier))
                    start_pose=self.robot.pose.clone()

                    with nav_timer:
                        logger.log('Navigating to goal')

                        if goal.ratio_decc:
                            logger.log("Goal has ratio_decc set to: {}".format(goal.ratio_decc))
                            yield RatioDeccControl(goal.ratio_decc)

                        move = yield Navigate(goal.x, goal.y, goal.direction, goal.offset)
                        logger.log('End of navigation : {}'.format(TRAJECTORY.lookup_by_value[move.exit_reason]))

                        if goal.ratio_decc:
                            yield RatioDeccControl()

                    current_navigation_succeeded = move.exit_reason in [ TRAJECTORY_DESTINATION_REACHED, TRAJECTORY_STOP_REQUESTED ]
                    if current_navigation_succeeded:
                        navigation_failures = 0
                        navigation_distance=self.robot.pose-start_pose
                        navigation_speed=navigation_distance/nav_timer.duration
                        logger.log('Navigation was successful, Robot speed: {} m/s'.format(navigation_speed))
                    else:
                        navigation_failures += 1
                        logger.log('Cannot navigate to goal -> picking another')
                        goal.is_blacklisted = True
                        goal.available()
                        self.handle_navigation_failure()
                    if move.exit_reason == TRAJECTORY_BLOCKED:
                        direction = DIRECTION_BACKWARDS if move.direction == DIRECTION_FORWARD else DIRECTION_FORWARD
                        dist = ROBOT_GYRATION_RADIUS - ROBOT_CENTER_X + 0.02
                        yield MoveLineRelative(dist, direction)
                        self.event_loop.map.robot_blocked(goal.direction)



                if current_navigation_succeeded:
                    gm.whitelist_all()
                    state = goal.get_state()
                    state.goal = goal
                    state.exit_reason = GOAL_FAILED

                    with goal.stats.real_duration.acquire(), metrics.simple_timer("Goal {}".format(goal.identifier)):
                        try:
                            yield state
                        except:
                            pass

                    logger.log('State exit reason : {}'.format(GOAL_STATUS.lookup_by_value[state.exit_reason]))

                    if state.exit_reason == GOAL_DONE :
                        goal.done()
                        if gm.score_estimator:
                            increment=gm.score_estimator.after_goal_success(goal, self.robot)
                            if increment:
                                logger.log("YES !!! our score was increased by {}, now {}".format(increment, self.robot.score))
                    else :
                        goal.increment_trials()
                        goal.available()

            else:
                navigation_failures += 1
                if not gm.has_blacklisted_goals():
                    break
            self.log('Goal statuses: {}'.format({ g.identifier : g.is_available() for g in gm.goals}))

        self.log('No more goals available')

        yield None


class ExecuteGoals(ExecuteGoalsBase):

    def get_next_goal_simple(self, gm):
        return gm.get_simple_next_goal()

import graphmap
class ExecuteGoalsV2(ExecuteGoalsBase):

    def get_next_goal_simple(self,gm):
        with metrics.STATS.duration.time():
            identifier = self.robot.goal_decider.get_best_goal(gm.doable_goals, gm.done_goals, map_=graphmap, robot=self.robot, max_duration=2, max_depth=10)
            if identifier:
                goal=gm.get_goals(identifier)[0]
                return goal

    def handle_navigation_failure(self):
        self.robot.goal_decider.handle_navigation_failure()



class EscapeToAnywhere(statemachine.Timer):

    def __init__(self):
        super().__init__(50)


    def on_enter(self):
        exit_reason = TRAJECTORY_OPPONENT_DETECTED


    def on_timeout(self):
        # We cannot use a simple while here. We have to give control back to the eventloop at each try
        # otherwise we will block all communications / statemachines
        x = random.randrange(300, 1700) / 1000.0
        if IS_MAIN_ROBOT:
            y = random.randrange(600, 1400) / 1000.0
        else:
            y = random.randrange(1600, 2700) / 1000.0
        move = yield Navigate(x, y)
        exit_reason = move.exit_reason
        if exit_reason == TRAJECTORY_BLOCKED:
            move = yield MoveLineRelative(0.1, DIRECTION_BACKWARDS)
            exit_reason = move.exit_reason
        yield None




class MoveArmServoPosition(statemachine.State):

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