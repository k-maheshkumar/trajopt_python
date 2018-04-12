import os
import time
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from scripts.utils.dict import DefaultOrderedDict
from srdfdom.srdf import SRDF

home = os.path.expanduser('~')

class PlannerExample:
    def __init__(self):

        location_prefix = home + "/catkin_ws/src/iai_robots/"

        urdf_file = location_prefix + "iai_donbot_description/robots/don_bot.urdf"

        config = {
            "use_gui": True,
            "verbose": True,
            "log_file": True,
            "robot_config": "robot_config_don_bot.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(use_gui=False)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_gravity(0, 0, -10)
        plane_id = self.planner.add_constraint_from_urdf("plane", "plane.urdf", position=[0, 0, 0.0])
        self.robot_id = self.planner.load_robot(urdf_file)
        #
        # # table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf",
        # #                                                  position=[0, 0, 0.0])
        # #
        # # self.box_id = self.planner.add_constraint(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        # #                                           position=[0.28, -0.43, 0.9], mass=100)
        #
        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.02)

    def run(self):
        from collections import OrderedDict
        start_state = OrderedDict()
        goal_state = OrderedDict()

        start_state["odom_x_joint"] = 0.01
        start_state["odom_y_joint"] = 0.01
        start_state["odom_z_joint"] = 0.01

        start_state["ur5_shoulder_pan_joint"] = 2.3823357809267463
        start_state["ur5_shoulder_lift_joint"] = 2.9299975516996142
        start_state["ur5_elbow_joint"] = -1.9762726255540713
        start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state["ur5_wrist_2_joint"] = 1.5855963769735366
        start_state["ur5_wrist_3_joint"] = -1.5770985888989753

        start_state["gripper_joint"] = 0
        start_state["gripper_base_gripper_left_joint"] = 0
        # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918

        goal_state["odom_x_joint"] = 0.01
        goal_state["odom_y_joint"] = 0.01
        goal_state["odom_z_joint"] = 0.01

        goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        goal_state["ur5_elbow_joint"] = 1.5873548294514912
        goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        goal_state["ur5_wrist_3_joint"] = 1.5857854098720727

        goal_state["gripper_joint"] = 0
        goal_state["gripper_base_gripper_left_joint"] = 0

        # start_state["ur5_shoulder_pan_joint"] = -1.4823357809267463
        # start_state["ur5_shoulder_lift_joint"] = -2.9299975516996142
        # start_state["ur5_elbow_joint"] = 1.9762726255540713
        # start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        # start_state["ur5_wrist_2_joint"] = -1.5855963769735366
        # start_state["ur5_wrist_3_joint"] = -1.5770985888989753
        # # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918
        #
        # goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        # goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        # goal_state["ur5_elbow_joint"] = -1.5873548294514912
        # goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        # goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        # goal_state["ur5_wrist_3_joint"] = 1.5857854098720727
        # goal_state["ur5_ee_fixed_joint"] = 1.5726221954434347

        # start_state = "place"
        # goal_state = "pick"
        # group = "full_arm"
        group = goal_state.keys()

        duration = 10
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1

        self.planner.world.reset_joint_states(self.robot_id, start_state.values(), start_state.keys())


        # import pybullet as p
        # p.connect(p.SHARED_MEMORY, "localhost")
        # joints = [i for i in range(p.getNumJoints(self.robot_id))]
        #
        # ur5_wrist_3_joint = 13
        # # print "ghdghrdklhg", p.getNumJoints(self.robot_id)
        # # zero_vec = [0] * 11
        # zero_vec = [0] * p.getNumJoints(self.robot_id)
        # # for i in range(p.getNumJoints(self.robot_id)):
        # #     print p.getJointInfo(self.robot_id, i)
        # current_robot_state = self.planner.world.get_joint_states_at(self.robot_id, start_state.values(), goal_state.keys())
        #
        # print "current_robot_state*------------------", len(current_robot_state[0])
        # print current_robot_state[0]
        #
        # current_position_jacobian, _ = p.calculateJacobian(self.robot_id, ur5_wrist_3_joint,
        #                                                      # closest_pt_on_A_at_t,
        #                                                      [0, 0, 0],
        #                                                    current_robot_state[0],
        #                                                      zero_vec, zero_vec)
        #
        # print current_position_jacobian

        _, status, trajectory = self.planner.get_trajectory(group=group, start_state=start_state,
                                                         goal_state=goal_state, samples=samples, duration=duration,
                                                         collision_safe_distance=collision_safe_distance,
                                                         collision_check_distance=collision_check_distance)
        print("is trajectory free from collision: ", status)
        # print trajectory.final
        self.planner.execute_trajectory()
        # self.planner.world.step_simulation_for(5)
        time.sleep(5)
        # import sys
        # sys.exit(0)

    def load_srdf(self):
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_iiwa_description/moveit_config/config/lbr_iiwa.srdf"

        stream = open(srdf_file, 'r')
        srdf = SRDF.from_xml_string(stream.read())

        ignored_collisions = DefaultOrderedDict(bool)
        shape = len(self.planner.world.joint_ids)

        # ignored_collisions_matrix = np.zeros((shape, shape))
        # joints = self.planner.world.link_name_to_id

        for collision in srdf.disable_collisionss:
            ignored_collisions[collision.link1, collision.link2] = True
            ignored_collisions[collision.link2, collision.link1] = True
            # if collision.link1 in joints and collision.link2 in joints:
            #     ignored_collisions_matrix[joints[collision.link1], joints[collision.link2]] = 1
            #     ignored_collisions_matrix[joints[collision.link2], joints[collision.link1]] = 1
        # print ignored_collisions
        self.planner.world.ignored_collisions = ignored_collisions


def main():
    example = PlannerExample()
    example.load_srdf()
    example.run()
    while True:
        pass


if __name__ == '__main__':
    main()
