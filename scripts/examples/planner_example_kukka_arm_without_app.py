from scripts.simulation.SimulationWorld import SimulationWorld
import os
from scripts.Robot import Robot
from scripts.utils.utils import Utils as utils
import numpy as np
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"

        self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=True, verbose="DEBUG", log_file=True)
        # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        plane_id = self.planner.load_from_urdf(urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[0.28, -0.43, 0.9], mass=100)


        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)

    def run(self):
        # start_state = {}
        # goal_state = {}
        #
        # start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
        # start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
        # start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
        # start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
        # start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
        # start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
        # start_state["lbr_iiwa_joint_7"] = 1.5704531145724918
        #
        # goal_state["lbr_iiwa_joint_1"] = -0.08180533826032865
        # goal_state["lbr_iiwa_joint_2"] = 1.5474152457596664
        # goal_state["lbr_iiwa_joint_3"] = -1.5873548294514912
        # goal_state["lbr_iiwa_joint_4"] = -0.5791571346767671
        # goal_state["lbr_iiwa_joint_5"] = 1.5979105177314896
        # goal_state["lbr_iiwa_joint_6"] = 1.5857854098720727
        # goal_state["lbr_iiwa_joint_7"] = 1.5726221954434347

        start_state = "place"
        goal_state = "pick"
        group = "full_arm"

        duration = 10
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1

        status, trajectory = self.planner.get_trajectory(group=group, start_state= start_state, goal_state=goal_state, samples=samples, duration=duration,
             collision_safe_distance=collision_safe_distance,
             collision_check_distance=collision_check_distance)
        print("is trajectory free from collision: ", status)
        self.planner.execute_trajectory()
        self.planner.world.step_simulation_for(5)
        import sys
        sys.exit(0)


def main():
    example = PlannerExample()
    example.run()


if __name__ == '__main__':
    main()
