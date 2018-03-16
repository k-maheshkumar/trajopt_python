# import time
import math
import pybullet as p
import time
import pybullet_data

import numpy as np

# __all__ = [pybullet, math, datetime]

# clid = p.connect(p.SHARED_MEMORY)

location_prefix = '/home/mahesh/catkin_ws/src/iai_robots/'
# p.connect(p.GUI)
p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF("plane.urdf", [0, 0, -0.3], useFixedBase=True)

robot_id = p.loadURDF(location_prefix + "iai_donbot_description/robots/donbot2.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(robot_id, [0, 0.25, 0.4], [0, 0, 0, 1])
p.setGravity(0, 0, -10)

numJoints = p.getNumJoints(robot_id)


# lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
# upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
# joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
# restposes for null space
rp = [0, 0, 0, -0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
# joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

# for i in range(numJoints):
#     p.resetJointState(kukaId, i, rp[i])

nJoints = p.getNumJoints(robot_id)
print "------------", nJoints

# jointNameToId = {}
#
#
# motordir = [-1, 1, -1, -1, 1, 1, 1]
# halfpi = 1.57079632679
# kneeangle = -2.1834
#
# useRealTimeSimulation = 1
# if useRealTimeSimulation:
#     p.setRealTimeSimulation(useRealTimeSimulation)
#
# start = p.addUserDebugParameter("start", 0.0, 1.0, 0.0)
# end = p.addUserDebugParameter("end", 0.0, 1.0, 0.0)
#
#
# startState = [-0.49197958189616936, 1.4223062659337982, -1.5688299779644697, -1.3135004031364736, 1.5696229411153653,
#               1.5749627479696093, 1.5708037563007493]
# endState = [-2.0417782994426674, 0.9444594031189716, -1.591006403858707, -1.9222844444479184, 1.572303282659756,
#             1.5741716208788483, 1.5716145442929421]
#
# wroteStartState = False
# wroteEndState = False
#
# for i in range(numJoints):
#     jointInfo = p.getJointInfo(kukaId, i)
#     jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
#
#
#
# joint_names = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3', 'lbr_iiwa_joint_4', 'lbr_iiwa_joint_5',
#           'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
#
# for i, joint in enumerate(joint_names):
#     p.resetJointState(kukaId, jointNameToId[joint], motordir[i] * halfpi)

# while 1:
#
#
#     time.sleep(0.2)
#
#
#
#     if not wroteStartState:
#         if p.readUserDebugParameter(start):
#             for i in range(numJoints):
#                 startState.append(p.getJointState(kukaId, request["joints"][i]["id"])[0])
#             file = open('jointStateStart.txt', 'w')
#             file.write(str(startState))
#             file.close()
#             wroteStartState = True
#     if not wroteEndState:
#         if p.readUserDebugParameter(end):
#             for i in range(numJoints):
#                 endState.append(p.getJointState(kukaId, request["joints"][i]["id"])[0])
#             file = open('jointStateEnd.txt', 'w')
#             file.write(str(endState))
#             file.close()
#             wroteEndState = True





