from scripts.simulation.SimulationWorld import SimulationWorld
from scripts.TrajectoryOptimizationPlanner.Planner import OptimizationPlanner
from scripts.Robot.Robot import Robot
import numpy as np
import logging
from scripts.utils.utils import Utils as utils

class TrajectoryOptimizationPlanner:
    def __init__(self, urdf_file, position=[0, 0, 0], orientation=[0, 0, 0, 1], use_fixed_base=True, use_gui=False,
                 verbose=False, log_file=False):
        # verbose = "DEBUG"
        main_logger_name = "Trajectory_Optimization_Planner."
        self.logger = logging.getLogger(main_logger_name)

        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)

        self.robot = Robot(urdf_file, main_logger_name, verbose, log_file)
        self.world = SimulationWorld(self.robot, use_gui, logger_name= main_logger_name, verbose=verbose, log_file=log_file)
        self.robot.id = self.load_from_urdf(urdf_file, position, orientation, use_fixed_base)
        self.world.setup_joint_id_to_joint_name(self.robot.id)
        self.current_planning_joint_group = None
        self.collision_safe_distance = 0.1
        self.collision_check_distance = 0.05



    def get_trajectory(self, group, goal_state, samples, duration,
                             collision_safe_distance, collision_check_distance):
        self.collision_safe_distance = collision_safe_distance
        self.collision_check_distance = collision_check_distance
        self.world.toggle_rendering_while_planning(False)
        self.current_planning_joint_group = self.robot.get_joints_of_group(group)
        # print "in trajopt planner get traj"
        # print self.current_planning_joint_group
        self.robot.init_plan_trajectory(group=self.current_planning_joint_group,
                                    current_state=self.world.get_current_states_for_given_joints(
                                        self.current_planning_joint_group, self.robot.id),
                                    goal_state=goal_state, samples=int(samples),
                                    duration=int(duration),
                                    collision_safe_distance=collision_safe_distance,
                                    collision_check_distance=collision_check_distance)
        self.world.toggle_rendering_while_planning(False)

        self.robot.calulate_trajecotory(self.callback_function_from_solver)
        self.world.toggle_rendering_while_planning(True)
        # print ("if trajectory has collision: ", \
        # self.world.check_for_collision_in_trajectory(self.robot.get_trajectory().final, goal_state.keys(),
        #                                            collision_safe_distance))

    def execute_trajectory(self):
        self.world.execute_trajectory(self.robot.id, self.robot.planner.get_trajectory())

    def init_robot_groups(self, groups):
        self.robot.init_robot_groups(groups)

    def add_collision_constraints(self, shape, mass, position, orientation=None, size=None, radius=None, height=None):
        constraint = self.world.create_constraint(shape, mass, position, orientation, size, radius, height)
        self.world.add_collision_constraints(constraint)

    def load_from_urdf(self, urdf_file, position, orientation=None, use_fixed_base=True):
        urdf_id = self.world.load_urdf(urdf_file, position, orientation, use_fixed_base)

        self.world.add_collision_constraints(urdf_id)

        return urdf_id

    def set_robot_state_to(self, robot_state):
        self.world.reset_joint_states(self.robot.id, robot_state)

    def callback_function_from_solver(self, new_trajectory, delta_trajectory=None):
        constraints, lower_limit, upper_limit = None, None, None
        trajectory = np.split(new_trajectory, self.robot.planner.no_of_samples)
        self.robot.planner.trajectory.add_trajectory(trajectory)

        collision_infos = self.world.get_collision_infos(self.robot.id, trajectory, self.current_planning_joint_group,
                                                                        distance=self.collision_check_distance)

        if len(collision_infos[2]) > 0:
            constraints, lower_limit, upper_limit = \
                self.robot.planner.problem_model.update_collision_infos(collision_infos, self.collision_safe_distance)
            self.robot.planner.update_prob()

        return constraints, lower_limit, upper_limit

    def setup_logger(self, main_logger_name, verbose=False, log_file=False):

        # creating a formatter
        formatter = logging.Formatter('-%(asctime)s - %(name)s - %(levelname)-8s: %(message)s')

        # create console handler with a debug log level
        log_console_handler = logging.StreamHandler()
        if log_file:
            # create file handler which logs info messages
            logger_file_handler = logging.FileHandler(main_logger_name + '.log', 'w', 'utf-8')
            logger_file_handler.setLevel(logging.INFO)
            # setting handler format
            logger_file_handler.setFormatter(formatter)
            # add the file logging handlers to the logger
            self.logger.addHandler(logger_file_handler)

        if verbose == "WARN":
            self.logger.setLevel(logging.WARN)
            log_console_handler.setLevel(logging.WARN)

        elif verbose == "INFO" or verbose is True:
            self.logger.setLevel(logging.INFO)
            log_console_handler.setLevel(logging.INFO)

        elif verbose == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
            log_console_handler.setLevel(logging.DEBUG)

        # setting console handler format
        log_console_handler.setFormatter(formatter)
        # add the handlers to the logger
        self.logger.addHandler(log_console_handler)