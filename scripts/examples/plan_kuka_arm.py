import math
import pybullet as p
import time

import numpy as np

from scripts.Planner import Planner

# __all__ = [pybullet, math, datetime]

# clid = p.connect(p.SHARED_MEMORY)
location_prefix = '/home/mahesh/libraries/bullet3/data/'
p.connect(p.GUI)
p.loadURDF(location_prefix + "plane.urdf", [0, 0, -0.3], useFixedBase=True)
p.loadURDF(location_prefix + "table/table.urdf", [0, 0, -0.3], useFixedBase=True)

kukaId = p.loadURDF(location_prefix + "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
p.resetBasePositionAndOrientation(kukaId, [0, 0.25, 0.4], [0, 0, 0, 1])
p.setGravity(0, 0, -10)
kukaEndEffectorIndex = 6
numJoints = p.getNumJoints(kukaId)
if (numJoints != 7):
    exit()

# p.loadURDF(location_prefix + "cube.urdf", [2, 2, 5])
# p.loadURDF(location_prefix + "cube.urdf", [-2, -2, 5])
# p.loadURDF(location_prefix + "cube.urdf", [2, -2, 5])

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
request1 = {"joints":[]}

reduceJoints = 0
samples = 30
duration = 30
max_iteration = 100
max_penalty = 1e6
max_delta = 5
request.update({'samples': samples, 'duration': duration, 'max_iteration': max_iteration, 'max_penalty': max_penalty,
                "max_delta": max_delta})
request1.update({'samples': samples, 'duration': duration, 'max_iteration': max_iteration, 'max_penalty': max_penalty,
                 "max_delta": max_delta})

# print request
    # print jointInfo[1], jointNameToId[jointInfo[1]]

motordir = [-1, 1, -1, -1, 1, 1, 1]
halfpi = 1.57079632679
kneeangle = -2.1834


useRealTimeSimulation = 1
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)

start = p.addUserDebugParameter("start", 0.0, 1.0, 0.0)
end = p.addUserDebugParameter("end", 0.0, 1.0, 0.0)
# startState = [-1.5708022241650113, 1.5711988957726704, -1.57079632679,
#               -1.5707784259568982, 1.5713463278825928, 1.5719498333358852, 1.5707901876998593]
# endState = [-0.43397739069763064, 1.4723163745550858, -1.5581591136925037,
#             -1.0268873087087633, 1.567033026684058, 1.5814537807610403, 1.571342501955761]

# startState = [-0.49197958189616936, 1.4223062659337982, -1.5688299779644697, -1.3135004031364736, 1.5696229411153653, 1.5749627479696093, 1.5708037563007493]
# endState = [-2.0417782994426674, 0.9444594031189716, -1.591006403858707, -1.9222844444479184, 1.572303282659756, 1.5741716208788483, 1.5716145442929421]

# startState = [0] * (samples)
# endState = [0] * (samples)

startState = []
endState = []

wroteStartState = False
wroteEndState = False

for i in range(numJoints):
    jointInfo = p.getJointInfo(kukaId, i)
    # print jointInfo
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    joint = {}
    joint["id"] = jointInfo[0]
    joint["lower_joint_limit"] = jointInfo[8]
    joint["upper_joint_limit"] = jointInfo[9]

    joint["min_velocity"] = -jointInfo[11]
    joint["max_velocity"] = jointInfo[11]

    request["joints"].append(joint)

    joint1 = {}
    joint1["id"] = jointInfo[0]
    joint1["lower_joint_limit"] = jointInfo[8]
    joint1["upper_joint_limit"] = jointInfo[9]

    joint1["min_velocity"] = -jointInfo[11]
    joint1["max_velocity"] = jointInfo[11]

    request1["joints"].append(joint1)



    # print "request:", request
    # print "request1:", request1


lbr_iiwa_joint_1 = jointNameToId['lbr_iiwa_joint_1']
lbr_iiwa_joint_2 = jointNameToId['lbr_iiwa_joint_2']
lbr_iiwa_joint_3 = jointNameToId['lbr_iiwa_joint_3']
lbr_iiwa_joint_4 = jointNameToId['lbr_iiwa_joint_4']
lbr_iiwa_joint_5 = jointNameToId['lbr_iiwa_joint_5']
lbr_iiwa_joint_6 = jointNameToId['lbr_iiwa_joint_6']
lbr_iiwa_joint_7 = jointNameToId['lbr_iiwa_joint_7']

p.resetJointState(kukaId, lbr_iiwa_joint_1, motordir[0] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_2, motordir[1] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_3, motordir[2] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_4, motordir[3] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_5, motordir[4] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_6, motordir[5] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_7, motordir[6] * halfpi)
jointPoses = []
jointPoses1 = []
# updateJoints()

def spline_interpolate(y_points, x):
    from scipy import interpolate

    x_points = np.linspace(0, 10, num=11, endpoint=True)

    tck = interpolate.splrep(x_points, y_points)
    return interpolate.splev(x, tck)

def updatePoses():

    plan = Planner.TrajectoryOptimizationPlanner(request, "SCS")
    # sp.displayProblem()
    result, jointPoses = plan.get_trajectory(None)
    plan1 = Planner.TrajectoryOptimizationPlanner(request1, "SCS")
    # sp.displayProblem()
    result1, jointPoses1 = plan1.get_trajectory(None)

    # print jointPoses
    # print jointPoses1
    return jointPoses, jointPoses1


def updateJoints(startState = [0], endState = [0]):
    for i in range(numJoints):
        joint = {}
        joint["start"] = startState[i]
        joint["end"] = endState[i]
        request["joints"][i].update(joint)
        joint1 = {}
        joint1["end"] = startState[i]
        joint1["start"] = endState[i]
        request1["joints"][i].update(joint1)
useSimulation = 1
# goStart = p.addUserDebugParameter("go to start", 0.0, 1.0, 0.0)
getNewStates = p.addUserDebugParameter("get New States", 0.0, 1.0, 1.0)

def moveArm(jointPoses2=[0]):

    for j in range(len(jointPoses2[0])):
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=kukaId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses2[i][j], targetVelocity=0, force=500, positionGain=0.03,
                                        velocityGain=.5)
        # time.sleep(duration/float(samples))
        # time.sleep(duration / float(samples) * 4)

        time.sleep(0.3)

gotostart = 0
gotoend = 1
updateNewStates  = 0
canMove = 0
# getNewStates = 0
# for i in range(numJoints):
#     request["joints"][i].update(
#             {"start": startState[i], "end": endState[i]})
# sp = trajPlanner.TrajectoryPlanner(request, "osqp")
# # sp.displayProblem()
# result, jointPoses = sp.solveProblem()
#
# print jointPoses
# print jointPoses1
while 1:
#     pass
#     print canMove
    temp = p.readUserDebugParameter(getNewStates)
    # print "temp", temp
    if canMove and not updateNewStates:
        if gotoend:
            # for i in range(numJoints):
            #
            #     request["joints"][i].update(
            #         {"start": startState[i], "end": endState[i]})
            # sp = trajPlanner.TrajectoryPlanner(request, "osqp")
            # # sp.displayProblem()
            # result, jointPoses = sp.solveProblem()
            moveArm(jointPoses)
            gotostart = 1
            gotoend = 0
            time.sleep(1)

            # print "gotostart"
            # print jointPoses


        # time.sleep(2)
        if gotostart:
            # for i in range(numJoints):
            #     request1["joints"][i].update(
            #         {"start": endState[i], "end": startState[i]})
            #
            # sp = trajPlanner.TrajectoryPlanner(request, "osqp")
            # # sp.displayProblem()
            # result, jointPoses1 = sp.solveProblem()
            moveArm(jointPoses1)
            gotostart = 0
            gotoend = 1
            # print "gotoend"
            # print jointPoses1


    # time.sleep(2)

    if temp and not canMove:
        if not wroteStartState:
            if p.readUserDebugParameter(start) == 1:
                for i in range(numJoints):
                    startState.append(p.getJointState(kukaId, request["joints"][i]["id"])[0])
                # file = open('jointStateStart.txt', 'w')
                # file.write(str(startState))
                # file.close()
                wroteStartState = True
        if not wroteEndState:
            if p.readUserDebugParameter(end)  == 1:
                for i in range(numJoints):
                    endState.append(p.getJointState(kukaId, request["joints"][i]["id"])[0])
                # file = open('jointStateEnd.txt', 'w')
                # file.write(str(endState))
                # file.close()
                wroteEndState = True

            updateNewStates = 1
    # print updateNewStates , wroteStartState , wroteEndState
    if updateNewStates and wroteStartState and wroteEndState:
        # print "startState", startState
        # print "endState", endState
        updateJoints(startState, endState)
        jointPoses, jointPoses1 = updatePoses()
        canMove = 1
        updateNewStates = 0


    # move = p.readUserDebugParameter(goStart)
    # if move:
    #     useSimulation = 0
    # else:
    #     useSimulation = 1
    #
    # if (useSimulation):
    #     moveArm()
    # else:
    #     # reset the joint state (ignoring all dynamics, not recommended to use during simulation)
    #     for i in range(numJoints):
    #         p.resetJointState(kukaId, lbr_iiwa_joint_1, motordir[0] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_2, motordir[1] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_3, motordir[2] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_4, motordir[3] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_5, motordir[4] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_6, motordir[5] * halfpi)
    #         p.resetJointState(kukaId, lbr_iiwa_joint_7, motordir[6] * halfpi)





