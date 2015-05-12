#!/usr/bin/python3

import struct
import socket
import sys

if len(sys.argv) < 5:
    print("Usage: {} <server> <port> <say|play|text> <text|path>".format(sys.argv[0]))
    sys.exit(1)

sock = socket.create_connection((sys.argv[1], int(sys.argv[2])))

if sys.argv[3] == 'play':
    msg_type = 250
elif sys.argv[3] == 'say':
    msg_type = 251
else:
    msg_type = 252

msg = " ".join(sys.argv[4:])
data = struct.pack("<BB254s", msg_type, 254, bytes(msg,'utf-8','ignore'))
sock.send(data)
sock.close()
