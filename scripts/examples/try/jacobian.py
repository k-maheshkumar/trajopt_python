import pybullet as p
import pybullet_data


def getJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques


def getMotorJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
    joint_infos = [p.getJointInfo(robot, i) for i in range(p.getNumJoints(robot))]
    joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
    joint_infos1 = [i for i in  joint_infos if i[3] > -1]
    joint_positions = [state[0] for state in joint_states]
    joint_velocities = [state[1] for state in joint_states]
    joint_torques = [state[3] for state in joint_states]
    return joint_positions, joint_velocities, joint_torques


def setJointPosition(robot, position, kp=1.0, kv=0.3):
    num_joints = p.getNumJoints(robot)
    zero_vec = [0.0] * num_joints
    if len(position) == num_joints:
        p.setJointMotorControlArray(robot, range(num_joints), p.POSITION_CONTROL,
            targetPositions=position, targetVelocities=zero_vec,
            positionGains=[kp] * num_joints, velocityGains=[kv] * num_joints)

def multiplyJacobian(robot, jacobian, vector):
    result = [0.0, 0.0, 0.0]
    i = 0
    for c in range(len(vector)):
        if p.getJointInfo(robot, c)[3] > -1:
            for r in range(3):
                result[r] += jacobian[r][i] * vector[c]
            i += 1
    return result


clid = p.connect(p.SHARED_MEMORY)
if (clid<0):
    p.connect(p.DIRECT)

time_step = 0.001
gravity_constant = -9.81
p.resetSimulation()
p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, gravity_constant)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF("plane.urdf",[0,0,-0.3])

kukaId = p.loadURDF("husky/husky.urdf",[0,0,0], useFixedBase=True)
kukaId1 = p.loadURDF("husky/husky.urdf",[1,1,0], useFixedBase=False)
p.resetBasePositionAndOrientation(kukaId,[0,0,0],[0,0,0,1])
p.resetBasePositionAndOrientation(kukaId1,[1,1,0],[0,0,0,1])
numJoints = p.getNumJoints(kukaId)
kukaEndEffectorIndex = numJoints - 1

# Set a joint target for the position control and step the sim.
setJointPosition(kukaId, [0.1] * numJoints)
p.stepSimulation()

# Get the joint and link state directly from Bullet.
pos, vel, torq = getJointStates(kukaId)
pos1, vel1, torq1 = getJointStates(kukaId1)
mpos, mvel, mtorq = getMotorJointStates(kukaId)
mpos1, mvel1, mtorq1 = getMotorJointStates(kukaId1)

result = p.getLinkState(kukaId, kukaEndEffectorIndex, computeLinkVelocity=1, computeForwardKinematics=1)
link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
# Get the Jacobians for the CoM of the end-effector link.
# Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
# The localPosition is always defined in terms of the link frame coordinates.
print("\n")
print ("mpos", mpos)
print ("mpos1", mpos1)
print ("pos", pos)
print ("pos1", pos1)
zero_vec = [0.0] * len(mpos)
jac_t, jac_r = p.calculateJacobian(kukaId, kukaEndEffectorIndex, com_trn, mpos, zero_vec, zero_vec)
jac_t1, jac_r1 = p.calculateJacobian(kukaId1, kukaEndEffectorIndex, com_trn, mpos, zero_vec, zero_vec)

print ("jac_t")
print (jac_t)
print ("jac_r")
print (jac_r)

# number of columns expected to match number of DOFS
print ("jac_t1")
print (jac_t1)
print ("jac_r1")
print (jac_r1)
