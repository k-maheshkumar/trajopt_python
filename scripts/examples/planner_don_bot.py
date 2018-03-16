import os
import time
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner


class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + "/catkin_ws/src/iai_robots/"

        urdf_file = location_prefix + "iai_donbot_description/robots/donbot2.urdf"

        config = {
            # "use_gui": True,
            "use_gui": True,
            "verbose": False,
            "log_file": True,
            "robot_config": "robot_config_don_bot.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(use_gui=False)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_gravity(0, 0, -10)
        plane_id = self.planner.add_constraint_from_urdf("plane.urdf", position=[0, 0, 0.0])
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
        start_state = {}
        goal_state = {}

        start_state["ur5_shoulder_pan_joint"] = 2.3823357809267463
        start_state["ur5_shoulder_lift_joint"] = 2.9299975516996142
        start_state["ur5_elbow_joint"] = -1.9762726255540713
        start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state["ur5_wrist_2_joint"] = -1.5855963769735366
        start_state["ur5_wrist_3_joint"] = -1.5770985888989753
        # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918

        goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        goal_state["ur5_elbow_joint"] = 1.5873548294514912
        goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        goal_state["ur5_wrist_3_joint"] = 1.5857854098720727

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

        self.planner.world.reset_joint_states(self.robot_id, goal_state)


        # import pybullet as p
        # p.connect(p.SHARED_MEMORY, "localhost")
        # joints = [i for i in range(p.getNumJoints(self.robot_id))]
        #
        # ur5_ee_fixed_joint = 14
        # zero_vec = [0] * p.getNumJoints(self.robot_id)
        # current_robot_state = p.getJointStates(self.robot_id, joints)
        #
        # print current_robot_state
        #
        # current_position_jacobian, _ = p.calculateJacobian(self.robot_id, ur5_ee_fixed_joint,
        #                                                      # closest_pt_on_A_at_t,
        #                                                      [0, 0, 0],
        #                                                      current_robot_state,
        #                                                      zero_vec, zero_vec)
        #
        # print current_position_jacobian
        # #
        #
        status, trajectory = self.planner.get_trajectory(group=group, start_state=start_state,
                                                         goal_state=goal_state, samples=samples, duration=duration,
                                                         collision_safe_distance=collision_safe_distance,
                                                         collision_check_distance=collision_check_distance)
        # print("is trajectory free from collision: ", status)
        # self.planner.execute_trajectory()
        # self.planner.world.step_simulation_for(5)
        # time.sleep(5)
        # import sys
        # sys.exit(0)


def main():
    example = PlannerExample()
    example.run()
    while True:
        pass


if __name__ == '__main__':
    main()
