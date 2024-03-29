# encoding: utf-8

import datetime
import itertools
import sys

import builder
import eventloop
import logger
import math
import packets
import position

from definitions import *




class ZoneData:

    def __init__(self, id):
        self.id = id
        self.is_detected = False
        self.is_enabled = True
        self.x = 0.0
        self.y = 0.0




class Map:

    def __init__(self, event_loop):
        self.event_loop = event_loop
        self.build_module()
        import graphpathfinding
        field_offset = 0.05
        self.pathfinder = graphpathfinding.PathFinder(field_offset,                # min x
                                                      field_offset,                # min y
                                                      FIELD_X_SIZE - field_offset, # max x
                                                      FIELD_Y_SIZE - field_offset, # max y
                                                      0.02,                        # zone escape increment
                                                      1.00)                        # zone max escape increment
        self.teammate_zone_timer = eventloop.Timer(self.event_loop, TEAMMATE_INFO_DELAY_MS * 2, self.disable_teammate_zone)
        self.ubo_zone_timer = eventloop.Timer(self.event_loop, 2000, self.disable_ubo_zone)
        self.use_interbot_position = TEAMMATE_POSITION_IN_MAP

        self.main_opponent_zone = None
        self.secondary_opponent_zone = None
        self.teammate_zone = None
        self.ubo_zone = None


    def on_controller_status(self, packet):
        if packet.status == CONTROLLER_STATUS_BUSY:
            return
        if packet.remote_device == REMOTE_DEVICE_SIMULATOR:
            self.event_loop.send_packet(packets.SimulatorClearGraphMapZones())


    def setup_zones(self, team):
        if self.main_opponent_zone is not None:
            logger.log("Map already initialized. Skipping.")
            return
        self.main_opponent_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.130 + ROBOT_GYRATION_RADIUS))
        self.secondary_opponent_zone = self.add_zone(self.create_circular_coords(0.0, 0.0, 0.080 + ROBOT_GYRATION_RADIUS))
        self.teammate_zone = self.add_zone(self.create_segment_coords(0.0, 0.0, 0.0, 0.0, self.get_teammate_radius()))
        self.ubo_zone = self.add_zone(self.create_rect_coords(0.0, 0.0, 0.0, 0.0, 0.0))

        # Add Field obstacles
        self.add_zone([(0.0, 0.0),
                       (0.0, 0.70),
                       (0.45, 0.94),
                       (0.65, 0.94),
                       (0.85, 0.77),
                       (0.85, 0.0)])
                       
        self.add_zone([(0.0,  FIELD_Y_SIZE - 0.0),
                       (0.0,  FIELD_Y_SIZE - 0.70),
                       (0.45, FIELD_Y_SIZE - 0.94),
                       (0.65, FIELD_Y_SIZE - 0.94),
                       (0.85, FIELD_Y_SIZE - 0.77),
                       (0.85, FIELD_Y_SIZE - 0.0)])
                       
        self.add_zone([(FIELD_X_SIZE, 0.0),
                       (FIELD_X_SIZE - 0.68, 0.0),
                       (FIELD_X_SIZE - 0.68, FIELD_Y_SIZE),
                       (FIELD_X_SIZE, FIELD_Y_SIZE)])
                       
        self.add_zone([(FIELD_X_SIZE, 0.67),
                       (FIELD_X_SIZE - 0.60, 0.67),
                       (FIELD_X_SIZE - 0.83, 0.90),
                       (FIELD_X_SIZE - 0.72, 1.22),
                       (FIELD_X_SIZE, 1.22)])
                       
        self.add_zone([(FIELD_X_SIZE, FIELD_Y_SIZE - 1.22),
                       (FIELD_X_SIZE - 0.72, FIELD_Y_SIZE - 1.22),
                       (FIELD_X_SIZE - 0.83, FIELD_Y_SIZE - 0.90),
                       (FIELD_X_SIZE - 0.60, FIELD_Y_SIZE - 0.67),
                       (FIELD_X_SIZE, FIELD_Y_SIZE - 0.67)])
                       
        self.add_zone([(FIELD_X_SIZE, 1.22),
                       (1.1, 1.22),
                       (1.0, 1.32),
                       (1.0, 1.68),
                       (1.1, 1.78),
                       (FIELD_X_SIZE, 1.78)])
        
        
        #~ offset = ROBOT_GYRATION_RADIUS * math.cos(math.pi / 4.0) + 0.01
        #~ if not IS_MAIN_ROBOT:
            #~ offset += 0.03
#~ 
        #~ popcorn_loc = 0.300 - 0.035 - offset
        #~ self.add_zone([(0.0, popcorn_loc),
                       #~ (0.07 + offset, popcorn_loc),
                       #~ (0.07 + offset, FIELD_Y_SIZE - popcorn_loc),
                       #~ (0.0, FIELD_Y_SIZE - popcorn_loc)])
        #~ steps_loc = 0.967 - offset
        #~ self.add_zone([(0.0, steps_loc),
                       #~ (0.580 + offset, steps_loc),
                       #~ (0.580 + offset, FIELD_Y_SIZE - steps_loc),
                       #~ (0.0, FIELD_Y_SIZE - steps_loc)])
        #~ start_loc = 0.8 - 0.022 - offset
        #~ if team == TEAM_LEFT:
            #~ self.add_zone([(start_loc, 0.0),
                           #~ (start_loc, 0.4 + offset),
                           #~ (FIELD_X_SIZE - start_loc, 0.4 + offset),
                           #~ (FIELD_X_SIZE - start_loc, 0.0)])
        #~ else:
            #~ self.add_zone([(start_loc, 0.0),
                           #~ (start_loc, 0.4 + offset + 0.25),
                           #~ (FIELD_X_SIZE - start_loc, 0.4 + offset + 0.25),
                           #~ (FIELD_X_SIZE - start_loc, 0.0)])
        #~ if team == TEAM_LEFT:
            #~ self.add_zone([(start_loc, FIELD_Y_SIZE),
                           #~ (start_loc, FIELD_Y_SIZE - 0.4 - offset - 0.25),
                           #~ (FIELD_X_SIZE - start_loc, FIELD_Y_SIZE - 0.4 - offset - 0.25),
                           #~ (FIELD_X_SIZE - start_loc, FIELD_Y_SIZE)])
        #~ else:
            #~ self.add_zone([(start_loc, FIELD_Y_SIZE),
                           #~ (start_loc, FIELD_Y_SIZE - 0.4 - offset),
                           #~ (FIELD_X_SIZE - start_loc, FIELD_Y_SIZE - 0.4 - offset),
                           #~ (FIELD_X_SIZE - start_loc, FIELD_Y_SIZE)])
        #~ platfrom_loc = 1.2 - offset
        #~ self.add_zone([(FIELD_X_SIZE, platfrom_loc),
                       #~ (FIELD_X_SIZE - 0.1 - offset, platfrom_loc),
                       #~ (FIELD_X_SIZE - 0.1 - offset, FIELD_Y_SIZE - platfrom_loc),
                       #~ (FIELD_X_SIZE, FIELD_Y_SIZE - platfrom_loc)])
#~ 
        #~ if not IS_MAIN_ROBOT:
            #~ zones = []
            #~ off = 0.10 + offset
            #~ zones.append([(1.650 - off, 1.500 - off),
                          #~ (1.650 - off, 1.500 + off),
                          #~ (1.650 + off, 1.500 + off),
                          #~ (1.650 + off, 1.500 - off)])
            #~ for coords in zones:
                #~ if team == TEAM_RIGHT:
                    #~ coords = list(map(lambda c: (c[0], 3.0 - c[1]), coords))
                #~ self.add_zone(coords)

        self.pathfinder.field_config_done()

        self.enable_zone(self.main_opponent_zone, False)
        self.enable_zone(self.secondary_opponent_zone, False)
        self.enable_zone(self.teammate_zone, False)
        self.enable_zone(self.ubo_zone, False)


    def get_teammate_radius(self):
        return MAIN_ROBOT_GYRATION_RADIUS + SECONDARY_ROBOT_GYRATION_RADIUS


    def create_quarter_coords(self, x, y, radius):
        coords = [(x, y)]
        npoints = 4
        for i in range(npoints):
            a = float(i) * (math.pi / 2.0) / float(npoints - 1)
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x - cx, y + cy))
        return coords


    def create_half_circle_coords(self, x, y, radius, vertical):
        coords = []
        npoints = 5
        for i in range(npoints):
            a = float(i) * math.pi / float(npoints - 1)
            if vertical:
                a += math.pi / 2.0
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x + cx, y + cy))
        return coords


    def create_circular_coords(self, x, y, radius):
        coords = []
        npoints = 8
        for i in range(npoints):
            a = float(i) * 2.0 * math.pi / float(npoints)
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x + cx, y + cy))
        return coords


    def create_segment_coords(self, x1, y1, x2, y2, radius):
        coords = []
        npoints = 4
        angle = math.atan2(y2 - y1, x2 - x1)
        for i in range(npoints):
            a = float(i) * math.pi / float(npoints - 1) + math.pi / 2.0 + angle
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x1 + cx, y1 + cy))
        for i in range(npoints):
            a = float(i) * math.pi / float(npoints - 1) - math.pi / 2.0 + angle
            cx = math.cos(a) * radius
            cy = math.sin(a) * radius
            coords.append((x2 + cx, y2 + cy))
        return coords


    def create_rect_coords(self, x, y, width, height, angle):
        w_2 = width / 2.0
        h_2 = height / 2.0
        cos_a = math.cos(angle)
        sin_a = math.sin(angle)
        coords = []
        for dx, dy in [(w_2, h_2), (-w_2, h_2), (-w_2, -h_2), (w_2, -h_2)]:
            coords.append((x - dx * sin_a + dy * cos_a, y + dx * cos_a + dy * sin_a))
        return coords


    def add_zone(self, coords):
        id = self.pathfinder.add_zone(coords)
        if IS_HOST_DEVICE_PC:
            flattened_coords = list(itertools.chain.from_iterable(coords))
            self.event_loop.send_packet(packets.SimulatorAddGraphMapZone(id = id, points = flattened_coords))
        return ZoneData(id)


    def update_zone(self, zone, coords):
        self.pathfinder.update_zone(zone.id, coords)
        if IS_HOST_DEVICE_PC:
            flattened_coords = list(itertools.chain.from_iterable(coords))
            self.event_loop.send_packet(packets.SimulatorAddGraphMapZone(id = zone.id, points = flattened_coords))


    def enable_zone(self, zone, enabled):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorEnableGraphMapZone(id = zone.id, enabled = enabled))
        self.pathfinder.enable_zone(zone.id, enabled)
        zone.is_enabled = enabled


    def move_zone(self, id, dx, dy):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorMoveGraphMapZone(id = id, dx = dx, dy = dy))
        self.pathfinder.move_zone(id, dx, dy)


    def route(self, start, end):
        logger.log("Compute route from ({}, {}) to ({}, {})".format(start.x, start.y, end.x, end.y))
        start_date = datetime.datetime.now()
        (cost, path) = self.pathfinder.find_path(start.x, start.y, end.x, end.y)
        delta = datetime.datetime.now() - start_date
        if len(path) == 0:
            logger.log("No route found. Cost: {}. computation time: {}".format(cost, delta.total_seconds()))
            return None, []
        else:
            logger.log("Route computed. Cost: {}. computation time: {}".format(cost, delta.total_seconds()))
        pose_path = []
        # remove start node and convert to poses
        for (x, y) in path[1:]:
            pose_path.append(position.Pose(x, y))
        self.send_to_simulator(pose_path)
        return cost, pose_path


    def evaluate(self, start, end):
        # When evaluating a path we consider far opponents
        for zone in [ self.main_opponent_zone, self.secondary_opponent_zone, self.teammate_zone ]:
            if zone.is_detected and not zone.is_enabled:
                self.pathfinder.enable_zone(zone.id, True)
        cost = self.route(start, end)[0]
        for zone in [ self.main_opponent_zone, self.secondary_opponent_zone, self.teammate_zone ]:
            if zone.is_detected and not zone.is_enabled:
                self.pathfinder.enable_zone(zone.id, False)
        return cost


    def send_to_simulator(self, path):
        if IS_HOST_DEVICE_PC:
            self.event_loop.send_packet(packets.SimulatorClearGraphMapEdges())
            packet = packets.SimulatorGraphMapEdges()
            for v in self.pathfinder.get_edges():
                packet.points.append(v)
                if len(packet.points) == 60:
                    self.event_loop.send_packet(packet)
                    packet.points = []
            if len(packet.points) != 0:
                self.event_loop.send_packet(packet)
            if path is not None:
                points = []
                for p in path:
                    points.append(p.x)
                    points.append(p.y)
                packet = packets.SimulatorGraphMapRoute(points = points)
                self.event_loop.send_packet(packet)


    def on_opponent_position(self, packet):
        if packet.robot == OPPONENT_ROBOT_MAIN:
            zone = self.main_opponent_zone
        else:
            zone = self.secondary_opponent_zone

        if zone is not None:
            if packet.x is not None and packet.y is not None:
                self.enable_zone(zone, True)
                zone.is_detected = True
                dx = packet.x - zone.x
                dy = packet.y - zone.y
                zone.x = packet.x
                zone.y = packet.y
                self.move_zone(zone.id, dx, dy)
            else:
                self.enable_zone(zone, False)
                zone.is_detected = False


    def build_module(self):
        pyversion = "python{}.{}{}".format(sys.version_info.major, sys.version_info.minor, sys.abiflags)
        include_dir = sys.exec_prefix + "/include/" + pyversion
        lib_dir = sys.exec_prefix + "/lib"

        working_dir = os.path.dirname(__file__)
        source_file = "graphpathfinding.c"
        output_file = "graphpathfinding.so"
        exe = "gcc"

        params = ["-O2", "-shared", "-Wall", "-fPIC", "-o", output_file, "-I" + include_dir, "-L" + lib_dir, source_file, "-l" + pyversion]

        if sys.platform == "darwin" :
            params = ["-dynamiclib"] + params

        commands = [exe] + params

        bld = builder.Builder(source_file, output_file, commands, working_dir)
        bld.build()


#    def on_interbot_position(self, packet):
#        """
#        :type packet: packets.InterbotPosition
#        """
#        if self.use_interbot_position:
#            if packet.is_moving:
#                coords = self.create_segment_coords(packet.pose.x, packet.pose.y, packet.destination.x, packet.destination.y, self.get_teammate_radius())
#            else:
#                coords = self.create_segment_coords(packet.pose.x, packet.pose.y, packet.pose.x, packet.pose.y, self.get_teammate_radius())
#            self.teammate_zone.x = packet.pose.x
#            self.teammate_zone.y = packet.pose.y
#            self.enable_zone(self.teammate_zone, not packet.is_moving)
#            self.teammate_zone.is_detected = True
#            self.update_zone(self.teammate_zone, coords)
#            self.teammate_zone_timer.restart()


    def robot_blocked(self, direction):
        pose = self.event_loop.robot.pose
        width = 0.4
        height = 0.3
        if direction == DIRECTION_FORWARD:
            angle = pose.angle
        else:
            angle = pose.angle + math.pi
        d = ROBOT_CENTER_X + height / 2.0
        x = pose.x + d * math.cos(angle)
        y = pose.y + d * math.sin(angle)
        coords = self.create_rect_coords(x, y, width, height, angle)
        logger.log("Place UBO zone: {}".format(coords))
        self.enable_zone(self.ubo_zone, True)
        self.update_zone(self.ubo_zone, coords)
        self.ubo_zone_timer.restart()


    def disable_teammate_zone(self):
        self.enable_zone(self.teammate_zone, False)


    def disable_ubo_zone(self):
        self.enable_zone(self.ubo_zone, False)
