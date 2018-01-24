import  pybullet as p
import os
import numpy as np
import time

home = os.path.expanduser('~')

location_prefix = home + '/masterThesis/bullet3/data/'
urdf_file = location_prefix + "kuka_iiwa/model.urdf"

# gui = p.connect(p.GUI)
gui = p.connect(p.DIRECT)
p.setGravity(0, 0, -10)

p.setRealTimeSimulation(0)



table = p.loadURDF(location_prefix + "table/table.urdf", basePosition=[0, 0, 0.0], useFixedBase=1)
p.loadURDF(location_prefix + "plane.urdf", basePosition=[0, 0, 0.0], useFixedBase=1)
kukaId = p.loadURDF(urdf_file, basePosition=[0, 0.25, 0.6], useFixedBase=1)
jointNameToId = {}
numJoints = p.getNumJoints(kukaId)

# motor_dir = np.random.uniform(-1, 1, size=len(joints))
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
motordir = [-1, 1, -1, -1, 1, 1, 1]
halfpi = 1.57079632679
p.resetJointState(kukaId, lbr_iiwa_joint_1, motordir[0] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_2, motordir[1] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_3, motordir[2] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_4, motordir[3] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_5, motordir[4] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_6, motordir[5] * halfpi)
p.resetJointState(kukaId, lbr_iiwa_joint_7, motordir[6] * halfpi)

def reset_joints_to(joints, states):
    for joint, state in zip(joints, states):
        p.resetJointState(kukaId, joint, state * halfpi)

group = ['lbr_iiwa_joint_1','lbr_iiwa_joint_2','lbr_iiwa_joint_3','lbr_iiwa_joint_4','lbr_iiwa_joint_5','lbr_iiwa_joint_6','lbr_iiwa_joint_7']

def get_jacobian():

    iteration_count = 0
    # p.stepSimulation()
    while iteration_count < 1000:
        motor_dir = np.random.uniform(-1, 1, size=numJoints)
        reset_joints_to(list(jointNameToId.values()), motor_dir)
        joint_states = p.getJointStates(kukaId, [jointNameToId[joint_name] for joint_name in group])
        joint_positions = [state[0] for state in joint_states]
        for link_index in list(jointNameToId.values()):
            link_state = p.getLinkState(kukaId, link_index, computeLinkVelocity=1,
                                          computeForwardKinematics=1)

            zero_vec = [0.0] * numJoints

            closest_points = p.getClosestPoints(kukaId, table,
                                                      linkIndexA=link_index, distance=0.2)
            if len(closest_points) > 0:
                if closest_points[0][8] < 0:
                    jac_t, jac_r = p.calculateJacobian(kukaId, link_index, closest_points[0][5],
                                                       joint_positions,
                                                         zero_vec, zero_vec)

        iteration_count += 1


if __name__ == '__main__':

    iteration_count = 0
    while True:
        if iteration_count < 1:
            iteration_count += 1
            start = time.time()
            get_jacobian()
            end = time.time()
            print "time: ", end -start
            import sys
            sys.exit(0)

