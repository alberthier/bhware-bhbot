#!/usr/bin/env python3
# encoding: utf-8

import signal
import argparse
import socket
import sys
import os

def patch_sys_path():
    current_dir=os.path.abspath(os.path.dirname(__file__))
    sys.path.append(current_dir+"/../libs")
    import glob
    for p in glob.glob1(current_dir+"/../libs","*.egg"):
        sys.path.append(current_dir+"/../libs/"+p)

patch_sys_path()

loop = None


def signal_handler(signum, frame):
    global loop
    if loop is not None:
        loop.stop()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    import definitions

    parser = argparse.ArgumentParser(description = "BH Team's main strategy program.", add_help = True)
    parser.add_argument("--webserver-port", action = "store", type = int, default = 8080, metavar = "PORT", help = "Internal web server port")
    parser.add_argument("--pydev-debug", nargs=2)
    parser.add_argument("--disable-interbot", action="store_true", default=False)
    parser.add_argument("--hostname", action = "store", default = socket.gethostname())
    parser.add_argument("--color", action = "store", default = "auto", help = "Colorize ouput (never, always, auto)")
    parser.add_argument('statemachine', action="store", nargs='?', default = None)

    args = parser.parse_args()

    if args.statemachine is None:
        args.statemachine = args.hostname

    if args.pydev_debug :
        import sys
        if sys.platform=="darwin":
            sys.path.append('/Applications/PyCharm.app/Contents/debug-eggs/pycharm-debug-py3k.egg')
            print(sys.path)
            import pydevd
            pydevd.settrace(args.pydev_debug[0], port=int(args.pydev_debug[1]), stdoutToServer=True,
                                                          stderrToServer=True, suspend=False)

    definitions.setup_definitions(args.hostname == "main")

    print('''      _
     ( )
      H
      H
     _H_
  .-'-.-'-.
 /         \\       ####   ####   #####  #   #  #####  ####   #   #
|           |      #   #  #   #  #      #   #  #      #   #  #   #
|   .-------'._    ####   ####   ###    # # #  ###    ####    # #
|  / /  '.' '. \\   #   #  #  #   #      # # #  #      #  #     #
|  \\ \\ @   @ / /   ####   #   #  #####   # #   #####  #   #    #
|   '---------'
|    _______|                       = 2015 =
|  .'-+-+-+|
|  '.-+-+-+|
|    """""" |
'-.__   __.-'
     """''')

    import eventloop
    import logger
    import leds

    logger.initialize(args)

    loop = eventloop.EventLoop(args.statemachine, args.webserver_port, not args.disable_interbot)
    leds.initialize(loop)
    loop.start()
    logger.close()

    if args.pydev_debug:
        pydevd.stoptrace()

    sys.exit(loop.exit_value)


