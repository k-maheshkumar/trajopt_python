import os
from collections import OrderedDict
import pybullet as p
import pybullet_data



class Example:
    def __init__(self):

        home = os.path.expanduser('~')
        p.connect(p.SHARED_MEMORY, "localhost")

        urdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/urdf/kuka_husky.urdf"
        urdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/urdf/kuka.urdf"
        urdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/urdf/husky.urdf"

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        plane_id = p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(urdf_file, useFixedBase=False)
        self.joint_name_to_id = OrderedDict()
        self.setup_joint_id_to_joint_name()


    def run(self):


        start_state = OrderedDict()

        start_state["front_left_wheel"] = 0.0
        start_state["front_right_wheel"] = 0.0
        start_state["rear_left_wheel"] = 0.0
        start_state["rear_right_wheel"] = 0.0

        # start_state["lbr_iiwa_joint_1"] = -1.8933
        # start_state["lbr_iiwa_joint_2"] = 1.5694
        # start_state["lbr_iiwa_joint_3"] = 0.9404
        # start_state["lbr_iiwa_joint_4"] = -1.0499
        # start_state["lbr_iiwa_joint_5"] = -0.5409
        # start_state["lbr_iiwa_joint_6"] = 1.2149
        # start_state["lbr_iiwa_joint_7"] = 0.0

        index = 2

        current_robot_state = self.get_joint_states_at(start_state.keys())
        zero_vec = [0] * len(current_robot_state[0])

        current_position_jacobian, _ = p.calculateJacobian(self.robot_id, index,
                                                           # closest_pt_on_A_at_t,
                                                           [0, 0, 0],
                                                           current_robot_state[0],
                                                           zero_vec, zero_vec)

        print (current_position_jacobian)

    def get_joint_states_at(self, group):
        joint_ids = [self.joint_name_to_id[joint] for joint in group]
        joint_states = p.getJointStates(self.robot_id, joint_ids)
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def setup_joint_id_to_joint_name(self):
        for i in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, i)
            self.joint_name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]


def main():
    example = Example()
    example.run()
    while True:
        p.stepSimulation()
if __name__ == '__main__':
    main()
