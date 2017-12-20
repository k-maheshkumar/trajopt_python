#!/usr/bin/env python

import sys
import argparse

from urdf_parser_py.urdf import URDF

# location_prefix = '/home/mahesh/libraries/bullet3/data/'
location_prefix = '/home/mahe/masterThesis/bullet3/data/'

robot = URDF.from_xml_file(location_prefix + "kuka_iiwa/model.urdf")

# print(robot.joint_map)
# for joint in robot.joints:
#     print robot.joint_map[joint.name]

# robot.add