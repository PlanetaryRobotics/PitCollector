#! /usr/bin/env python
import rospy
import numpy as np

from rig import Rig
from camera import Camera
import time

import json

def run_collection_sequence(seq_file):
    print("Collecting Images.")
    print("Sequence File: {}".format(seq_file))

    with open(seq_file, 'r') as json_file:
        seq_def = json.loads(seq_file)


def kill_ros():
    import subprocess
    subprocess.call(["rosnode", "kill", "--all"])
    subprocess.call(["killall", "-9", "rosmaster"])
    subprocess.call(["killall", "-9", "roscore"])

if __name__ == "__main__":
    rospy.init_node('collector')
    seq_file = rospy.get_param('~seq_file')
    run_collection_sequence(seq_file)

    if not rospy.is_shutdown():
        kill_ros()
