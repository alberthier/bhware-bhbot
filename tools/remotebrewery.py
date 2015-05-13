#!/usr/bin/env python3

import argparse
import os
import sys
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument("-p", "--push", action = "store_true", help = "push the local source code with rsync")
parser.add_argument("-r", "--run", action = "store_true", help = "execute the remote brewery")
parser.add_argument("remote", help = "IP or hostname of the robot")
args = parser.parse_args()

if args.push:
    bhware = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    params=["rsync",
                     "--archive",
                     "--verbose",
                     "--ignore-times",
                     "--exclude-from", os.path.join(bhware, ".hgignore"),
                     "brewery",
                     "root@{}:/root/bhware".format(args.remote)]
    ret = subprocess.call(" ".join(params), cwd = bhware, shell=True)

    if ret == 0:
        print("rsync OK")
    else:
        print("rsync FAILED ({})".format(ret))

    subprocess.call(["/usr/bin/ssh",
             "-t",
             "root@{}".format(args.remote),
             "sync"])

    if ret == 0:
        print("sync OK")
    else:
        print("sync FAILED ({})".format(ret))

if args.run:
    os.execl("/usr/bin/ssh",
             "-t",
             "root@{}".format(args.remote),
             "source /root/.profile && /root/bhware/brewery/brewery.py")
