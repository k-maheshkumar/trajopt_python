import pybullet as p
import time
import numpy as np
import sys

def create_constraint(shape, mass, position, size=None, radius=None, height=None, orientation=None):
    if position is not None:
        if radius is not None:
            col_id = p.createCollisionShape(shape, radius=radius)
            vis_id = p.createCollisionShape(shape, radius=radius)
        if radius is not None and height is not None:
            col_id = p.createCollisionShape(shape, radius=radius, height=height)
            vis_id = p.createCollisionShape(shape, radius=radius, height=height)
        if size is not None:
            col_id = p.createCollisionShape(shape, halfExtents=size)
            vis_id = p.createCollisionShape(shape, halfExtents=size)

        if orientation is not None:
            shape_id = p.createMultiBody(mass, col_id, vis_id, position, orientation)
        else:
            shape_id = p.createMultiBody(mass, col_id, vis_id, position)


    return shape_id

def interpolate(start, end, samples=5, decimals_to_round=3):
    data = []
    step_size = (end - start) / (samples - 1)
    intermediate = start
    for i in range(samples):
        data.append(intermediate)
        intermediate += step_size
    return np.round(data, decimals_to_round)

def step_simulation_for(seconds):
    start = time.time()
    while time.time() < start + seconds:
        p.stepSimulation()


def moveArm(jointPoses):
    for j in range(len(jointPoses[0])):
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=kukaId, jointIndex=i, controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i][j], targetVelocity=0, force=500, positionGain=0.03,
                                    velocityGain=.5)
            time.sleep(0.2)


def reset_joint_states_to(robot_id, trajectory, joints, joint_name_to_id):

    if len(trajectory) == len(joints):
        for i in range(len(trajectory)):
            p.resetJointState(robot_id, joint_name_to_id[joints[i]], trajectory[i])

def get_link_states_at(robot_id, trajectory, joints, joint_name_to_id):
    link_states = []
    reset_joint_states_to(robot_id, trajectory, joints, joint_name_to_id)
    for link_index in joint_name_to_id.items():
        state = p.getLinkState(robot_id, link_index, computeLinkVelocity=1,
                                 computeForwardKinematics=1)
        link_states.append(state)
    return link_states


p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, -10)

location_prefix = '/home/mahesh/libraries/bullet3/data/'
p.loadURDF(location_prefix + "plane.urdf", [0, 0, 0.0], useFixedBase=True)
useRealTimeSimulation = 0
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)
start = [0.28, 1.0, 0.2]
end = [0.28, 2.0, 0.2]

wall = create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.18], position=[0.95, 0.3, 0], mass=1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
kukaId = p.loadURDF(location_prefix + "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)

useRealTimeSimulation = 0
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)
else:
    step_simulation_for(0.05)

trajectory = []
# print jointInfo[1], jointNameToId[jointInfo[1]]

motordir = [1, 1, 1, 1, -1, 1, 1]
motordir1 = [0.7, 1, 1, 1, -1, 1, 1]
# motordir = [1, 1, 1, 1, -1, 1, 1]
halfpi = 1.57079632679
kneeangle = -2.1834

motordir = [x * halfpi for x in motordir]
motordir1 = [x * halfpi for x in motordir1]

start_state = [1.5708024764979394, 1.5712326366904628, 1.57079632708463, 1.5707855978401637, -1.5701336236638301,
               1.5719725305810723, 1.570793057646014]
end_state = [0.6799090616158451, 1.5729501835585853, 1.5704022441549654, 1.6398254945800432, -1.5625212113651783,
             1.596406342019127, 1.568985184095934]
for i in range(len(start_state)):
    trajectory.append(interpolate(start_state[i], end_state[i], 3, 5))

trajectory = np.asarray(trajectory).T


useRealTimeSimulation = 0
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)

jointNameToId = {}
numJoints = p.getNumJoints(kukaId)

joints = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3', 'lbr_iiwa_joint_4',
          'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']

for i in range(numJoints):


    jointInfo = p.getJointInfo(kukaId, i)
    # print jointInfo
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]



lbr_iiwa_joint_1 = jointNameToId['lbr_iiwa_joint_1']
lbr_iiwa_joint_2 = jointNameToId['lbr_iiwa_joint_2']
lbr_iiwa_joint_3 = jointNameToId['lbr_iiwa_joint_3']
lbr_iiwa_joint_4 = jointNameToId['lbr_iiwa_joint_4']
lbr_iiwa_joint_5 = jointNameToId['lbr_iiwa_joint_5']
lbr_iiwa_joint_6 = jointNameToId['lbr_iiwa_joint_6']
lbr_iiwa_joint_7 = jointNameToId['lbr_iiwa_joint_7']

p.resetJointState(kukaId, lbr_iiwa_joint_1, motordir[0])
p.resetJointState(kukaId, lbr_iiwa_joint_2, motordir[1])
p.resetJointState(kukaId, lbr_iiwa_joint_3, motordir[2])
p.resetJointState(kukaId, lbr_iiwa_joint_4, motordir[3])
p.resetJointState(kukaId, lbr_iiwa_joint_5, motordir[4])
p.resetJointState(kukaId, lbr_iiwa_joint_6, motordir[5])
p.resetJointState(kukaId, lbr_iiwa_joint_7, motordir[6])



for i in range(len(trajectory)):

    print ("------------------trajectory["+str(i)+"]: ", trajectory[i])
    next_link_state = None
    if i < (len(trajectory) - 1):
        reset_joint_states_to(kukaId, trajectory[i+1], joints, jointNameToId)
        next_link_state = p.getLinkState(kukaId, 6, computeLinkVelocity=1,
                                                 computeForwardKinematics=1)

    reset_joint_states_to(kukaId, trajectory[i], joints, jointNameToId)
    current_link_state = p.getLinkState(kukaId, 6, computeLinkVelocity=1,
                                              computeForwardKinematics=1)
    time.sleep(0.5)



    closest_points = p.getClosestPoints(kukaId, wall, linkIndexA=6, distance=0.10)

    for j in range(len(closest_points)):
        print("-------------closest points -----------------")
        print("link A index: ", closest_points[j][3])
        print("link B index: ", closest_points[j][4])
        print("point on A: ", closest_points[j][5])
        print("point on B: ", closest_points[j][6])
        print("contact normal on B: ", closest_points[j][7])
        print("contact distance: ", closest_points[j][8])
        print("-------------************ -----------------")
    if next_link_state is not None:

        start = time.time()

        cast_points = p.getConvexSweepClosestPoints(kukaId, wall, distance=0.10, linkIndexA=6,
                                                   bodyAfromPosition=current_link_state[0],
                                                   bodyAtoPosition=next_link_state[0],
                                                   bodyAfromOrientation=current_link_state[1],
                                                   bodyAtoOrientation=next_link_state[1]
                                                   )
        print ("time :--------------------- ", time.time() - start)

        for k in range(len(cast_points)):
            print("-------------cast_points -----------------")
            print("link A index: ", cast_points[k][3])
            print("link B index: ", cast_points[k][4])
            print("point on A(t): ", cast_points[k][5])
            print("point on A(t+1): ", cast_points[k][6])
            print("point on B: ", cast_points[k][7])
            print("contact normal on B: ", cast_points[k][8])
            print("contact distance: ", cast_points[k][9])
            print("contact fraction: ", cast_points[k][10])
            print("-------------************ -----------------")

while True:
    pass


# startState = [-1.5708022241650113, 1.5711988957726704, -1.57079632679,
#               -1.5707784259568982, 1.5713463278825928, 1.5719498333358852, 1.5707901876998593]
# endState = [-0.43397739069763064, 1.4723163745550858, -1.5581591136925037,
#             -1.0268873087087633, 1.567033026684058, 1.5814537807610403, 1.571342501955761]
#
# startState = [-0.49197958189616936, 1.4223062659337982, -1.5688299779644697, -1.3135004031364736, 1.5696229411153653,
#               1.5749627479696093, 1.5708037563007493]
# endState = [-2.0417782994426674, 0.9444594031189716, -1.591006403858707, -1.9222844444479184, 1.572303282659756,
#             1.5741716208788483, 1.5716145442929421]
# startState =[]
# endState =[]
# wroteStartState = False
# wroteEndState = False
#
#
#
# start = p.addUserDebugParameter("start", 0.0, 1.0, 0.0)
# end = p.addUserDebugParameter("end", 0.0, 1.0, 0.0)
# # print jointPoses1
# while 1:
#
#     if not wroteStartState:
#         if p.readUserDebugParameter(start):
#             for i in range(numJoints):
#                 startState.append(p.getJointState(kukaId, jointNameToId[joints[i]])[0])
#             file = open('jointStateStart.txt', 'w')
#             file.write(str(startState))
#             file.close()
#             wroteStartState = True
#     if not wroteEndState:
#         if p.readUserDebugParameter(end):
#             for i in range(numJoints):
#                 endState.append(p.getJointState(kukaId, jointNameToId[joints[i]])[0])
#             file = open('jointStateEnd.txt', 'w')
#             file.write(str(endState))
#             file.close()
#             wroteEndState = True
#
#
#     time.sleep(2)






