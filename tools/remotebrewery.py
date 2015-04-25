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
    subprocess.call(["rsync",
                     "--archive",
                     "--verbose",
                     "--ignore-times",
                     "--exclude-from", os.path.join(bhware, ".hgignore"),
                     "brewery",
                     "root@{}:/root/bhware".format(args.remote)], cwd = bhware)

if args.run:
    os.execl("/usr/bin/ssh",
             "-t",
             "root@{}".format(args.remote),
             "source /root/.profile && /root/bhware/brewery/brewery.py")
