import pybullet as p
import pybullet_data
import os


def getJointStates(robot):
    joint_states = p.getJointStates(robot, range(p.getNumJoints(robot)))
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
    else:
        print("Not setting torque. "
              "Expected torque vector of "
              "length {}, got {}".format(num_joints, len(torque)))


def multiplyJacobian(jacobian, vector):
    result = [0.0, 0.0, 0.0]
    for c in range(len(vector)):
        for r in range(3):
            result[r] += jacobian[r][c] * vector[c]
    return result


clid = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

if (clid < 0):
    p.connect(p.DIRECT)
time_step = 0.001
gravity_constant = -9.81
p.resetSimulation()
p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, gravity_constant)
p.loadURDF("plane.urdf", [0, 0, -0.3])

home = os.path.expanduser('~')

location_prefix = home + "/catkin_ws/src/iai_robots/"

urdf_file = location_prefix + "iai_donbot_description/robots/don_bot.urdf"
# kukaId = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])
robot_id = p.loadURDF(urdf_file, [0, 0, 0])
p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], [0, 0, 0, 1])
ur5_wrist_3_joint = 13
numJoints = p.getNumJoints(robot_id)
if (numJoints != 37):
    exit()
# Set a joint target for the position control and step the sim.
setJointPosition(robot_id, [0.1] * p.getNumJoints(robot_id))
p.stepSimulation()
# Get the joint and link state directly from Bullet.
pos, vel, torq = getJointStates(robot_id)
print "gfdlhg", len(pos)
result = p.getLinkState(robot_id, ur5_wrist_3_joint, computeLinkVelocity=1, computeForwardKinematics=1)
link_trn, link_rot, com_trn, com_rot, frame_pos, frame_rot, link_vt, link_vr = result
# Get the Jacobians for the CoM of the end-effector link.
# Note that in this example com_rot = identity, and we would need to use com_rot.T * com_trn.
# The localPosition is always defined in terms of the link frame coordinates.
zero_vec = [0.0] * numJoints
jac_t, jac_r = p.calculateJacobian(robot_id, ur5_wrist_3_joint, com_trn, pos, zero_vec, zero_vec)

# print ("Link linear velocity of CoM from getLinkState:")
# print (link_vt)
# print ("Link linear velocity of CoM from linearJacobian * q_dot:")
# print (multiplyJacobian(jac_t, vel))
# print ("Link angular velocity of CoM from getLinkState:")
# print (link_vr)
# print ("Link angular velocity of CoM from angularJacobian * q_dot:")
# print (multiplyJacobian(jac_r, vel))
print ("Jacobian:")
print jac_t