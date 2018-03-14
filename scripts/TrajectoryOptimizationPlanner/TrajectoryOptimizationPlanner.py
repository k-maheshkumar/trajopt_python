from scripts.simulation.SimulationWorld import SimulationWorld
from scripts.TrajectoryOptimizationPlanner.Planner import OptimizationPlanner
from scripts.Robot.Robot import Robot
import numpy as np
import logging
from scripts.utils.utils import Utils as utils


class TrajectoryOptimizationPlanner():
    def __init__(self, urdf_file, use_gui=False):
        self.robot = Robot(urdf_file)
        self.world = SimulationWorld(urdf_file, use_gui)
        self.world.toggle_rendering(0)
        self.robot.id = self.world.load_urdf(urdf_file, position=[0, 0.25, 0.6])
        self.world.setup_joint_id_to_joint_name(self.robot.id)


    def add_constraint_from_urdf(self, urdf_file, position, orientation=None, use_fixed_base=True):
        urdf_id = self.world.load_urdf(urdf_file, position, orientation, use_fixed_base)
        self.world.add_collision_constraints(urdf_id)
        return urdf_id

    def add_constraint(self, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        shape_id = self.world.create_constraint(shape, mass, position, size, radius, height, orientation)
        self.world.add_collision_constraints(shape_id)
        return shape_id

    def get_trajectory(self, group, goal_state, samples, duration, collision_safe_distance,
                       collision_check_distance):
        current_robot_state = self.world.get_current_states_for_given_joints(self.robot.id, group)
        self.robot.init_plan_trajectory(group=group, current_state=current_robot_state,
                                        goal_state=goal_state, samples=samples, duration=duration,
                                        collision_safe_distance=collision_safe_distance,
                                        collision_check_distance=collision_check_distance)
        self.world.toggle_rendering_while_planning(False)

        self.robot.calulate_trajecotory(self.callback_function_from_solver)

        status = self.world.check_for_collision_in_trajectory(self.robot.id, self.robot.get_trajectory().final,
                                                                    goal_state.keys(),
                                                                    collision_safe_distance)

        self.world.toggle_rendering_while_planning(True)

        return status, self.robot.planner.get_trajectory()

    def execute_trajectory(self):

        # self.robot.get_trajectory().final = \
        #     np.asarray(utils.interpolate_list(self.robot.planner.get_trajectory().final.T, 10)).T.tolist()
        self.world.execute_trajectory(self.robot, self.robot.planner.get_trajectory())

    def callback_function_from_solver(self, new_trajectory, delta_trajectory=None):
        constraints, lower_limit, upper_limit = None, None, None
        trajectory = np.split(new_trajectory, self.robot.planner.no_of_samples)
        self.robot.planner.trajectory.add_trajectory(trajectory)

        collision_infos = self.world.get_collision_infos(self.robot.id, trajectory, self.robot.planner.current_planning_joint_group,
                                                         distance=self.robot.planner.collision_check_distance)

        if len(collision_infos[2]) > 0:
            constraints, lower_limit, upper_limit = \
                self.robot.planner.problem_model.update_collision_infos(collision_infos, self.robot.planner.collision_safe_distance)
            self.robot.planner.update_prob()

        return constraints, lower_limit, upper_limit
