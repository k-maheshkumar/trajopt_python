import pybullet as p
# import time
import math
from datetime import datetime
import matplotlib.pyplot as plt

# __all__ = [pybullet, math, datetime]

# clid = p.connect(p.SHARED_MEMORY)
location_prefix = '/home/mahesh/libraries/bullet3/data/'
p.connect(p.GUI)
p.loadURDF(location_prefix + "plane.urdf", [0, 0, -0.3], useFixedBase=True)
kukaId = p.loadURDF(location_prefix + "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kukaId, [0, 0, 0], [0, 0, 0, 1])
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
    exit()

p.loadURDF(location_prefix + "cube.urdf", [2, 2, 5])
p.loadURDF(location_prefix + "cube.urdf", [-2, -2, 5])
p.loadURDF(location_prefix + "cube.urdf", [2, -2, 5])

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

for i in range(numJoints):
    p.resetJointState(kukaId, i, rp[i])

nJoints = p.getNumJoints(kukaId)

jointNameToId = {}
request = {"joints":[]}
reduceJoints = 4

for i in range(nJoints-reduceJoints):
    jointInfo = p.getJointInfo(kukaId, i)
    # print jointInfo
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    joint = {}
    joint["id"] = jointInfo[0]
    joint["lower_joint_limit"] = jointInfo[8]
    joint["upper_joint_limit"] = jointInfo[9]
    request["joints"].append(joint)

request.update({'samples' : 100, 'duration' : 15, 'maxIteration': 100})

# print "request"
# print request
    # print jointInfo[1], jointNameToId[jointInfo[1]]

lbr_iiwa_joint_1 = jointNameToId['lbr_iiwa_joint_1']
lbr_iiwa_joint_2 = jointNameToId['lbr_iiwa_joint_2']
lbr_iiwa_joint_3 = jointNameToId['lbr_iiwa_joint_3']
# lbr_iiwa_joint_4 = jointNameToId['lbr_iiwa_joint_4']
# lbr_iiwa_joint_5 = jointNameToId['lbr_iiwa_joint_5']
# lbr_iiwa_joint_6 = jointNameToId['lbr_iiwa_joint_6']
# lbr_iiwa_joint_7 = jointNameToId['lbr_iiwa_joint_7']
# #
# motordir = [-1, -1, -1, -1, 1, 1, 1, -1]
# halfpi = 1.57079632679
# kneeangle = -2.1834
#
# p.resetJointState(kukaId, lbr_iiwa_joint_1, motordir[0] * halfpi)
# p.resetJointState(kukaId, lbr_iiwa_joint_2, motordir[1] * kneeangle)
# p.resetJointState(kukaId, lbr_iiwa_joint_3, motordir[2] * halfpi)
# p.resetJointState(kukaId, lbr_iiwa_joint_4, motordir[3] * kneeangle)
# p.resetJointState(kukaId, lbr_iiwa_joint_5, motordir[4] * kneeangle)
# p.resetJointState(kukaId, lbr_iiwa_joint_6, motordir[5] * kneeangle)
# p.resetJointState(kukaId, lbr_iiwa_joint_7, motordir[6] * kneeangle)
# p.setRealTimeSimulation(1)
#
# p.setGravity(0, 0, -10)
#
# cubePos, cubeOrn = p.getBasePositionAndOrientation(kukaId)
# p.loadURDF(location_prefix + "cube.urdf", cubePos, cubeOrn)

# p.changeConstraint(cid, maxForce=10000)
# cid = p.createConstraint(kukaId, -1, kukaId, lbr_iiwa_joint_1, p.JOINT_POINT2POINT,
#                          [0, 0, 1], [0, 0.005, 0.2], [0, 0.01, 0.2])
# print p.getConstraintInfo(1)
# print "get contact points"
# print p.getContactPoints()

t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 0

count = 0
useOrientation = 1
useSimulation = 1
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
# trailDuration is duration (in seconds) after debug lines will be removed automatically
# use 0 for no-removal
trailDuration = 15

p.setGravity(0, 0, -10)

# logId1 = p.startStateLogging(p.STATE_LOGGING_GENERIC_ROBOT, "LOG0001.txt", [0, 1, 2])
# logId2 = p.startStateLogging(p.STATE_LOGGING_CONTACT_POINTS, "LOG0002.txt", bodyUniqueIdA=2)

# for i in range(5):
    # print("Body %d's name is %s." % (i, p.getBodyInfo(i)[1]))

# print "get contact points"
# print contactpts[0][5]

last_time = datetime.now()

while 1:
    t = 0
    if (useRealTimeSimulation):
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
        # print dt - last_time
        last_time = dt
    else:
        t = t + 0.1

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    for i in range(1):
        # contactpts = p.getClosestPoints(kukaId, 2, 100, 6)
        # print contactpts
        # pos = contactpts[0][6]
        # pos = [0.4, 0.2 * math.cos(t), -0.2 + 0.2 * math.sin(t)]
        pos = [0.4, 0.2 * math.cos(t), 0 + 0.2 * math.sin(t)]
        # cur_pos = p.getLinkState(kukaId, kukaEndEffectorIndex)
        # print cur_pos[0]
        # # fig, ax = plt.subplots()


        # p.addUserDebugLine(pos, contactpts[0][6], [0, 0, 0.3], 1, trailDuration)
        # end effector points down, not up (in case useOrientation==1)
        orn = p.getQuaternionFromEuler([0, -math.pi, 0])
        if (useNullSpace == 1):
            if (useOrientation == 1):
                jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, ll, ul, jr, rp)
            else:
                jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, lowerLimits=ll,
                                                          upperLimits=ul, jointRanges=jr, restPoses=rp)
        else:
            if (useOrientation == 1):
                jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos, orn, jointDamping=jd)
            else:
                jointPoses = p.calculateInverseKinematics(kukaId, kukaEndEffectorIndex, pos)

        if (useSimulation):
            for i in range(numJoints-reduceJoints):
                request["joints"][i].update({"start" : p.getJointState(kukaId, request["joints"][i]["id"])[0], "end": jointPoses[i], "min_velocity" : -0.2, "max_velocity" : 0.2,})
                    # .append(p.getJointState(kukaId, request["joints"][i]["id"]))

                # p.setJointMotorControl2(bodyIndex=kukaId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                #                         targetPosition=jointPoses[i], targetVelocity=0, force=500, positionGain=0.03,
                #                         velocityGain=1)

            # print request
            from trajPlanner import trajPlanner

            sp = trajPlanner.TrajectoryPlanner(request, "osqp")
            # sp.displayProblem()
            jointPoses = sp.solveProblem()
            # print jointPoses
            for i in range(numJoints-4):
                for j in range(len(jointPoses[i])):
                    # print len(jointPoses[i])
                    # print "setting pose: ", j
                    # print jointPoses[i][j]
                    p.setJointMotorControl2(bodyIndex=kukaId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                            targetPosition=jointPoses[i][j], targetVelocity=0, force=500, positionGain=0.03,
                                            velocityGain=.5)

        else:
            # reset the joint state (ignoring all dynamics, not recommended to use during simulation)
            for i in range(numJoints):
                p.resetJointState(kukaId, i, jointPoses[i])
        # colors = ['bo', 'go', 'ro', 'co', 'mo', 'yo', 'ko', 'wo']


    ls = p.getLinkState(kukaId, kukaEndEffectorIndex)
    # if (hasPrevPose):
    # p.addUserDebugLine(prevPose,pos,[0,0,0.3],1,trailDuration)
    # p.addUserDebugLine(prevPose1,ls[4],[1,0,0],1,trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = 1
    # eef = p.getJointInfo(kukaId, 6)
    # print eef


