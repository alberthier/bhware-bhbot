import datetime
import json
import collections
from definitions import *


# class DotNotation:
#     RESERVED_FIELDS=["name","data","value"]
#     def __init__(self, name=None):
#         self.name=name
#         self.data={}
#         self.value=None
#
#     def __getattr__(self, name):
#         if not name in self.data and not name in DotNotation.RESERVED_FIELDS:
#             self.data[name]=DotNotation(name)
#
#         return self.data.get(name)
#
#     def to_json_dict(self):
#         if self.value:
#             return self.value
#         return { it.name: it.to_json_dict() for it in self.data }
#
#     def __setattr__(self, name, value):
#         if name in self.data and not name in DotNotation.RESERVED_FIELDS:
#             self.data[name]=value
#         else:
#             object.__setattr__(self, name, value)
#
# class DefaultDictOfDicts(collections.defaultdict(lambda k: {})):
#     pass
#
# class AutoVivification(dict):
#
#     #__getattr__= dict.__getitem__
#     #__setattr__= dict.__setitem__
#     #__delattr__= dict.__delitem__
#
#     """Implementation of perl's autovivification feature."""
#     def __getitem__(self, item):
#         try:
#             return dict.__getitem__(self, item)
#         except KeyError:
#             value = self[item] = type(self)()
#             return value
#
# #DEFAULT_DATA

class SysInfo:
    def __init__(self, event_loop, destination_filename="/tmp/sysinfo.json"):
        # self.data=AutoVivification()
        self.last_write=None
        self.destination_filename=destination_filename
        self.event_loop = event_loop
        self.event_loop.fsms.append(self)

        self.data = {
            "pose" : {
                "x": -0.0,
                "y": -0.0,
                "angle": -0.0
            },
            "battery": {
                "left": {
                    "voltage": 0
                },
                "right": {
                    "voltage": 0
                },
            },
            "servo": {
            }
        }

        for name, servo_id in SERVOS_IDS:
            self.init_servo_data(servo_id)


    def on_keep_alive(self, packet):
        self.data["battery"]["left"]["voltage"]=packet.left_battery_voltage
        self.data["battery"]["right"]["voltage"]=packet.right_battery_voltage
        self.data["pose"]={"x": packet.current_pose.x, "y": packet.current_pose.y, "angle": packet.current_pose.angle}


        if not self.last_write or (datetime.datetime.now() - self.last_write).total_seconds() > 1 :
            with open(self.destination_filename,"w") as f:
                json.dump(self.data,f,indent=True)
                f.flush()
                self.last_write=datetime.datetime.now()

    def init_servo_data(self, servo_id):
        lk = SERVOS_IDS.lookup_by_value[servo_id]
        self.data["servo"].setdefault(lk, {"value": None, "torque": None, "speed": None})
        self.data["servo"][lk]["typed_id"] = servo_id
        return lk

    def on_servo_control(self, packet):
        lk = self.init_servo_data(packet.id)

        if packet.command in [SERVO_COMMAND_MOVE, SERVO_COMMAND_POSITION]:
            self.data["servo"][lk]["value"]=packet.value

        elif packet.command == SERVO_COMMAND_TORQUE_ENABLE:
            self.data["servo"][lk]["torque"]=True if packet.value==1 else False

        elif packet.command == SERVO_COMMAND_SETUP_SPEED:
            self.data["servo"][lk]["speed"]=packet.value

        print(self.data["servo"][lk])

# d=DotNotation()