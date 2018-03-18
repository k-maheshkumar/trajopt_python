from scripts.simulation.SimulationWorld import SimulationWorld
import os
from scripts.Robot import Robot
from scripts.utils.utils import Utils as utils
import numpy as np
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
import itertools
from collections import OrderedDict

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"

        self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=True, verbose=False, log_file=True)
        # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        plane_id = self.planner.load_from_urdf(urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.35],
                                                  position=[0.28, -0.43, 0.9], mass=100)


        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)

    def run(self):
        start_state = {}
        goal_state1 = OrderedDict()

        start_state["lbr_iiwa_joint_1"] = -0.4823357809267463
        start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
        start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
        start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
        start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
        start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
        start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

        goal_state1["lbr_iiwa_joint_1"] = -2.08180533826032865
        goal_state1["lbr_iiwa_joint_2"] = 1.5474152457596664
        goal_state1["lbr_iiwa_joint_3"] = -1.5873548294514912
        goal_state1["lbr_iiwa_joint_4"] = -0.5791571346767671
        goal_state1["lbr_iiwa_joint_5"] = 1.5979105177314896
        goal_state1["lbr_iiwa_joint_6"] = 1.5857854098720727
        goal_state1["lbr_iiwa_joint_7"] = 1.5726221954434347
        group1 = goal_state1.keys()


        start_state = [-2.4823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103,
                       1.5855963769735366, 1.5770985888989753, 1.5704531145724918]
        goal_state = [-0.08180533826032865, 1.5474152457596664, -1.5873548294514912, -0.5791571346767671,
                      1.5979105177314896, 1.5857854098720727, 1.5726221954434347]

        # start_state = [-1.1823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103,
        #                1.5855963769735366, 1.5770985888989753, 1.5704531145724918]
        # goal_state = [-0.54180533826032865, 1.5474152457596664, -1.5873548294514912, -0.5791571346767671,
        #               1.5979105177314896, 1.5857854098720727, 1.5726221954434347]

        # start_state = "place"
        # goal_state = "pick"
        group = "full_body"

        duration = 10
        samples = 20
        collision_check_distance = 0.10
        collision_safe_distance = 0.05


        # b = [-0.71352,  1.53639, -1.58443, -0.65476,  1.59987,  1.58676,  1.57208]
        # a = [-0.58718,  1.53859, -1.58502, -0.63964,  1.59947,  1.58657,  1.57218]
        #
        #
        # # self.planner.world.reset_joint_states(self.planner.robot.id, goal_state1.keys(), a)
        #
        # zero_vec = [0]  * 7
        #
        # import pybullet as p
        #
        # p.connect(p.SHARED_MEMORY)
        #
        # l_index = 4
        #
        # current_position_jacobian, _ = p.calculateJacobian(self.planner.robot.id, l_index,
        #                                                      # closest_pt_on_A_at_t,
        #                                                      [0, 0, 0],
        #                                                    start_state,
        #                                                      zero_vec, zero_vec)
        # start = self.planner.world.get_link_states_at(self.planner.robot.id, group1, a)
        # end = self.planner.world.get_link_states_at(self.planner.robot.id, group1, b)
        #
        #
        # print "jacobian"
        # print current_position_jacobian
        # print start[l_index][0]
        # print end[l_index][0]
        #
        # # cast_closest_points = []
        #
        # cast_closest_points = p.getConvexSweepClosestPoints(self.planner.robot.id, self.box_id,
        #                                                       linkIndexA=l_index, distance=0.1,
        #                                                       bodyAfromPosition=start[l_index][0],
        #                                                       bodyAfromOrientation=start[l_index][1],
        #                                                       # bodyAfromOrientation=[0, 0, 0, 1],
        #                                                       bodyAtoPosition=end[l_index][0],
        #                                                       bodyAtoOrientation=end[l_index][1],
        #                                                       # bodyAtoOrientation=[0, 0, 0, 1],
        #                                                       )
        #
        # if len(cast_closest_points) > 0:
        #
        #     closest_pt_on_A_at_t = cast_closest_points[0][5]
        #     closest_pt_on_A_at_t_plus_1 = cast_closest_points[0][6]
        #     closest_pt_on_B = cast_closest_points[0][7]
        #     normal_ = - np.vstack(cast_closest_points[0][8]).reshape(3, 1)
        #     normal_ = utils.normalize_vector(normal_)
        #     dist = cast_closest_points[0][9]
        #     fraction = cast_closest_points[0][10]
        #
        #     if dist < 0:
        #         print "-----------cast points--------------"
        #         print "A(t)", closest_pt_on_A_at_t
        #         print "A(t+1)", closest_pt_on_A_at_t_plus_1
        #         print "B", closest_pt_on_B
        #         print "normal", normal_.T
        #         print "Distance ", dist
        #         print "fraction ", fraction
        #         print "*************************************"

        # while True:
        #     pass

        status, trajectory = self.planner.get_trajectory(group=group, start_state=start_state,
                                                         goal_state=goal_state, samples=samples, duration=duration,
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
