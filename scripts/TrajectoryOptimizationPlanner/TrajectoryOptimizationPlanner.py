from scripts.simulation.SimulationWorld import SimulationWorld
from scripts.Robot.Robot import Robot
import numpy as np
import scripts.utils.yaml_paser as yaml
from scripts.utils.utils import Utils as utils
import logging
import os

class TrajectoryOptimizationPlanner():
    def __init__(self, **kwargs):

        if "logger_name" in kwargs:
            main_logger_name = kwargs["logger_name"]
        else:
            main_logger_name = "Trajectory_Planner"

        if "verbose" in kwargs:
            verbose = kwargs["verbose"]
        else:
            verbose = False

        if "log_file" in kwargs:
            log_file = kwargs["log_file"]
        else:
            log_file = False

        if "robot_config" in kwargs:
            robot_config = kwargs["robot_config"]

        self.logger = logging.getLogger(main_logger_name)
        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)
        self.robot = Robot(main_logger_name, verbose, log_file)
        self.world = SimulationWorld(**kwargs)
        self.world.toggle_rendering(0)
        self.load_configs(robot_config)

    def load_configs(self, config_file=None):
        file_path_prefix = os.path.join(os.path.dirname(__file__), '../../config/')
        self.default_config = yaml.ConfigParser(file_path_prefix + 'default_config.yaml')
        self.config = self.default_config.get_by_key("config")

        self.sqp_config_file = file_path_prefix + self.config["solver"]

        self.sqp_yaml = yaml.ConfigParser(self.sqp_config_file)
        self.sqp_config = self.sqp_yaml.get_by_key("sqp")
        if config_file is not None:
            robot_config_file = file_path_prefix + config_file
        else:
            robot_config_file = file_path_prefix + self.config["robot"]["config"]
        self.robot_default_config_params = self.config["robot"]["default_paramaters"]
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")

    def load_robot(self, urdf_file, position=[0, 0, 0], orientation=[0, 0, 0, 1], use_fixed_base=True):
        self.robot.id = self.world.load_robot(urdf_file, position, orientation, use_fixed_base)
        self.robot.load_robot_model(urdf_file)

        return self.robot.id

    def load_from_urdf(self, urdf_file, position, orientation=None, use_fixed_base=True):
        urdf_id = self.world.load_urdf(urdf_file, position, orientation, use_fixed_base)
        return urdf_id

    def add_constraint_from_urdf(self, urdf_file, position, orientation=None, use_fixed_base=True):
        urdf_id = self.world.load_urdf(urdf_file, position, orientation, use_fixed_base)
        self.world.add_collision_constraints(urdf_id)
        return urdf_id

    def add_constraint(self, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        shape_id = self.world.create_constraint(shape, mass, position, size, radius, height, orientation)
        self.world.add_collision_constraints(shape_id)
        return shape_id

    def add_constraint_with_id(self, constraint_id):
        self.world.add_collision_constraints(constraint_id)

    # def get_trajectory(self, group, start_state, goal_state, samples, duration, collision_safe_distance,
    #                    collision_check_distance):
    def get_trajectory(self, **kwargs):

        if "group" in kwargs:
            group = kwargs["group"]
            if type(group) is str:
                group = self.robot_config["joints_groups"][kwargs["group"]]
        if "start_state" in kwargs:
            start_state = kwargs["start_state"]
            if type(start_state) is str:
                start_state = self.robot_config["joint_configurations"][start_state]

            if type(start_state)is dict:
                start_state = start_state.values()

            self.world.reset_joint_states(self.robot.id, start_state, group)
            self.world.step_simulation_for(0.2)

        if "goal_state" in kwargs:
            goal_state = kwargs["goal_state"]
            if type(goal_state) is str:
                goal_state = self.robot_config["joint_configurations"][goal_state]

            if type(goal_state)is dict:
                goal_state = goal_state.values()
        if "samples" in kwargs:
            samples = kwargs["samples"]
        else:
            samples = 20
        if "duration" in kwargs:
            duration = kwargs["duration"]
        else:
            duration = 10
        if "collision_safe_distance" in kwargs:
            collision_safe_distance = kwargs["collision_safe_distance"]
        else:
            collision_safe_distance = 0.05
        if "collision_check_distance" in kwargs:
            collision_check_distance = kwargs["collision_check_distance"]
        else:
            collision_check_distance = 0.1

        current_robot_state = self.world.get_current_states_for_given_joints(self.robot.id, group)
        self.robot.init_plan_trajectory(group=group, current_state=current_robot_state,
                                        goal_state=goal_state, samples=samples, duration=duration,
                                        collision_safe_distance=collision_safe_distance,
                                        collision_check_distance=collision_check_distance)
        self.world.toggle_rendering_while_planning(False)
        #
        self.robot.calulate_trajecotory(self.callback_function_from_solver)
        #
        status = self.world.is_trajectory_collision_free(self.robot.id, self.robot.get_trajectory().final,
                                                         group,
                                                         collision_safe_distance)

        self.world.toggle_rendering_while_planning(True)

        return status, self.robot.planner.get_trajectory()

    def plan_trajectory(self, planning_group, goal_state, samples=20, duration=10,
                        collision_safe_distance=0.1,
                       collision_check_distance=0.05, solver_config=None):
        current_robot_state = self.world.get_current_states_for_given_joints(self.robot.id, planning_group)
        self.robot.init_plan_trajectory(group=planning_group, current_state=current_robot_state,
                                        goal_state=goal_state, samples=samples, duration=duration,
                                        collision_safe_distance=collision_safe_distance,
                                        collision_check_distance=collision_check_distance,
                                        solver_config=solver_config)
        self.world.toggle_rendering_while_planning(False)

        status, _ = self.robot.calulate_trajecotory(self.callback_function_from_solver)

        can_execute_trajectory = self.world.is_trajectory_collision_free(self.robot.id, self.robot.get_trajectory().final,
                                                                         goal_state.keys(),
                                                                         collision_safe_distance)

        self.world.toggle_rendering_while_planning(True)

        return status, can_execute_trajectory

    def execute_trajectory(self):

        # self.robot.get_trajectory().final = \
        #     np.asarray(utils.interpolate_list(self.robot.planner.get_trajectory().final.T, 10)).T.tolist()
        self.world.execute_trajectory(self.robot, self.robot.planner.get_trajectory())

        return "Trajectory execution completed"

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
