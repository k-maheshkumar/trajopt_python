from scripts.simulation.SimulationWorld import SimulationWorld
import os
from scripts.Robot import Robot
from scripts.utils.utils import Utils as utils
import numpy as np
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptImizationPlanner1 import TrajectoryOptimizationPlanner
import collections

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"
        # self.planner = Planner(urdf_file, position=[0, 0.25, 0.6], use_gui=True)
        self.planner = TrajectoryOptimizationPlanner(urdf_file, position=[0, 0.25, 0.6], use_gui=False, verbose=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_robot(self.planner.robot)
        self.planner.world.load_urdf(urdf_file=location_prefix + "plane.urdf",
                             position=[0, 0, 0.0])

        table_id = self.planner.world.load_urdf(urdf_file=location_prefix + "table/table.urdf",
                                        position=[0, 0, 0.0])

        # self.cylinder_id = self.world.create_constraint(shape=self.world.CYLINDER,
        #                                                 height=0.70, radius=0.12, position=[0.50, 0.30, 0.9], mass=100)
        box_id = self.planner.add_collision_constraints(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                             # position=[-0.17, -0.42, 0.9], mass=100)
                                             position=[0.28, -0.43, 0.9], mass=100)
        # self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.2, 0.45],
        #                                      position=[-0.17, -0.42, 0.9], mass=100)
        # position=[0.28, -0.43, 0.9], mass=100)
        # self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.2, 0.45],
        #                                      position=[-0.50, -0.42, 0.9], mass=100)
        # position=[0.28, -0.43, 0.9], mass=100)

        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)
        self.world.add_collision_constraints(table_id)

    def run_simulation(self):
        iteration_count = 0
        while 1:
            if iteration_count < 1:
                iteration_count += 1
                start_state = collections.OrderedDict()
                goal_state = collections.OrderedDict()

                start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
                start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
                start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
                start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
                start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
                start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
                start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

                goal_state["lbr_iiwa_joint_1"] = -0.08180533826032865
                goal_state["lbr_iiwa_joint_2"] = 1.5474152457596664
                goal_state["lbr_iiwa_joint_3"] = -1.5873548294514912
                goal_state["lbr_iiwa_joint_4"] = -0.5791571346767671
                goal_state["lbr_iiwa_joint_5"] = 1.5979105177314896
                goal_state["lbr_iiwa_joint_6"] = 1.5857854098720727
                goal_state["lbr_iiwa_joint_7"] = 1.5726221954434347

                duration = 10
                samples = 20
                self.planner.set_robot_state_to(start_state)
                self.planner.world.step_simulation_for(0.2)
                collision_check_distance = 0.10
                collision_safe_distance = 0.05
                # group = goal_state.keys()
                groups = [
                    {"name":"full_arm", "base": "lbr_iiwa_link_0", "tip": "lbr_iiwa_link_7"},
                    # {"name":"full_arm1", "base": "lbr_iiwa_joint_3", "tip": "lbr_iiwa_joint_7"},
                          ]
                self.planner.init_robot_groups(groups)
                self.planner.get_trajectory(group="full_arm",
                                                goal_state=goal_state, samples=int(samples),
                                                duration=int(duration),
                                                collision_safe_distance=collision_safe_distance,
                                                collision_check_distance=collision_check_distance)
                # self.world.toggle_rendering_while_planning(False)
                #
                # self.robot.calulate_trajecotory(self.world.get_collision_infos)
                #
                # print ("if trajectory has collision: ", \
                # self.world.check_for_collision_in_trajectory(self.robot.get_trajectory().final, goal_state.keys(),
                #                                            collision_safe_distance))
                #
                # self.world.toggle_rendering_while_planning(True)
                #
                # # self.robot.get_trajectory().final = \
                # #     np.asarray(utils.interpolate_list(self.robot.planner.get_trajectory().final.T, 10)).T.tolist()
                # self.planner.execute_trajectory()
                self.planner.world.step_simulation_for(2)

                import sys
                sys.exit(0)



if __name__ == '__main__':
    example = PlannerExample()
    example.run_simulation()