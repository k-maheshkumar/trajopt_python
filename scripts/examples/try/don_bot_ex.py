import pybullet as p
import os

home = os.path.expanduser('~')

location_prefix = home + "/catkin_ws/src/iai_robots/"

urdf_file = location_prefix + "iai_donbot_description/robots/donbot2.urdf"

p.connect(p.GUI)

robot_id = p.loadURDF(urdf_file)
joint_states = []

joints = [i for i in range(p.getNumJoints(robot_id))]
print joints
print p.getJointStates(robot_id, joints)
# for i in range(p.getNumJoints(robot_id)):
#     print p.getJointInfo(robot_id, i)
#     joint_states.append(p.getJointState())
# ur5_ee_fixed_joint = 14
# zero_vec = [0] * p.getNumJoints(robot_id)
# current_robot_state = p.getJointStates()
# current_position_jacobian, _ = p.calculateJacobian(robot_id, ur5_ee_fixed_joint,
#                                                      # closest_pt_on_A_at_t,
#                                                      [0, 0, 0],
#                                                      current_robot_state,
#                                                      zero_vec, zero_vec)
#

while True:
    pass