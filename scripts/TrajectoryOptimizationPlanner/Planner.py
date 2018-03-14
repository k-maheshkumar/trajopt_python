import logging
import time

import numpy as np

import scripts.sqp_solver.ProblemModelling as model
from scripts.Robot import Trajectory
from scripts.sqp_solver import SQPsolver
from scripts.sqpproblem import SQPproblem
import collections
from scripts.utils.utils import Utils as utils


class OptimizationPlanner:

    def __init__(self, logger_name=__name__, verbose=False, log_file=False):
        self.sqp = collections.OrderedDict()
        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbG = []
        self.ubG = []
        self.b = []

        self.initial_guess = []
        self.solver_status = []
        self.joint_names = []
        self.problem = collections.OrderedDict()
        self.max_penalty = -1
        self.delta_max = -1
        self.joints = -1
        self.num_of_joints = -1
        self.solver = -1
        self.solver =  -1
        self.duration = -1
        self.no_of_samples = -1
        self.max_no_of_Iteration = -1
        self.max_no_of_Iteration = -1
        self.decimals_to_round = 5
        self.collision_check_distance = 0.1
        self.collision_safe_distance = 0.05
        self.solver_class = 0

        self.solver_config = None
        self.trajectory = Trajectory.Trajectory()
        self.problem_model = model.ProblemModelling()
        self.sqp_solver = SQPsolver.SQPsolver(logger_name, verbose, log_file)
        # self.planning_groups = collections.OrderedDict()
        self.planning_groups = collections.OrderedDict()
        self.current_planner_group = None
        self.callback_function_to_get_collision_infos = None

        logger_name = logger_name + __name__
        self.logger = logging.getLogger(logger_name)
        utils.setup_logger(self.logger, logger_name, verbose, log_file)
    def __clear_all_data(self):
        self.sqp = collections.OrderedDict()
        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbG = []
        self.ubG = []
        self.b = []

        self.initial_guess = []
        self.solver_status = []
        self.joint_names = []

    def init(self, **kwargs):
        self.__clear_all_data()
        if kwargs is not None:
            if "problem" in kwargs:
                self.problem = kwargs["problem"]
                if "samples" in self.problem:
                    self.no_of_samples = self.problem["samples"]
                else:
                    self.logger.critical("attribute: number of samples is missing")
                if "duration" in self.problem:
                    self.duration = self.problem["duration"]
                else:
                    self.logger.critical("attribute: duration is missing")
                if "max_iteration" in self.problem:
                    self.max_no_of_Iteration = self.problem["max_iteration"]
                else:
                    self.max_no_of_Iteration = 10
                if "joints" in self.problem:
                    self.joints = self.problem["joints"]
                    self.num_of_joints = len(self.problem["joints"])
                else:
                    self.logger.critical("attribute: joints is missing")
                if "max_penalty" in self.problem:
                    self.max_penalty = self.problem["max_penalty"]
                else:
                    self.max_penalty = 1e4
                if "max_delta" in self.problem:
                    self.delta_max = self.problem["max_delta"]
                else:
                    self.delta_max = 5
                if "collision_safe_distance" in self.problem:
                    self.collision_safe_distance = self.problem["collision_safe_distance"]
                if "collision_check_distance" in self.problem:
                    self.collision_check_distance = self.problem["collision_check_distance"]
                if "joint_group" in self.problem:
                    self.current_planner_group = self.problem["joint_group"]

            if "joints" in kwargs:
                self.joints = kwargs["joints"]
                self.num_of_joints = len(self.joints)
            if "solver" in kwargs:
                self.solver = kwargs["solver"]
            else:
                self.solver = None
            if "duration" in kwargs:
                self.duration = kwargs["duration"]
            if "samples" in kwargs:
                self.no_of_samples = kwargs["samples"]
            if "max_iteration" in kwargs:
                self.max_no_of_Iteration = kwargs["max_iteration"]
            else:
                self.max_no_of_Iteration = 20
            if "decimals_to_round" in kwargs:
                self.decimals_to_round = int(kwargs["decimals_to_round"])
            else:
                self.decimals_to_round = 5


            if "joint_group" in kwargs:
                self.current_planner_group = kwargs["joint_group"]

            if "collision_safe_distance" in kwargs:
                self.collision_safe_distance = float(kwargs["collision_safe_distance"])
            if "collision_check_distance" in kwargs:
                self.collision_check_distance = float(kwargs["collision_check_distance"])

            if "solver_config" in kwargs:
                self.solver_config = kwargs["solver_config"]

            self.problem_model.init(self.joints, self.no_of_samples, self.duration, self.decimals_to_round,
                              self.collision_safe_distance, self.collision_check_distance)
            self.sqp_solver.init(P=self.problem_model.cost_matrix_P, q=self.problem_model.cost_matrix_q,
                                 G=self.problem_model.robot_constraints_matrix,
                                 lbG=self.problem_model.constraints_lower_limits, ubG=self.problem_model.constraints_upper_limits,
                                 A=self.problem_model.start_and_goal_matrix, b=self.problem_model.start_and_goal_limits,
                                 initial_guess=self.problem_model.initial_guess, solver_config=self.solver_config)
            self.trajectory.init(np.array((np.split(self.problem_model.initial_guess, self.no_of_samples))),
                                 self.problem_model.samples, self.problem_model.duration, self.current_planner_group)
            # self.trajectory.init(np.array(self.problem_model.initial_guess), self.problem_model.samples, self.problem_model.duration)


    def update_prob(self):
        self.sqp_solver.update_prob(G=self.problem_model.robot_constraints_matrix,
                                    lbG=self.problem_model.constraints_lower_limits, ubG=self.problem_model.constraints_upper_limits)

    def display_problem(self):
        self.sqp_solver.display_problem()

    def calculate_trajectory(self, robot_id, initial_guess=None, callback_function=None):
        can_execute_trajectory = False
        self.logger.info("getting trajectory")
        start = time.time()
        self.solver_status, trajectory = self.sqp_solver.solve(initial_guess, callback_function)
        end = time.time()
        trajectory = np.array((np.split(trajectory, self.no_of_samples)))
        self.trajectory.update(trajectory, self.joints.keys())
        self.trajectory.plot_trajectories()
        status = "-1"
        if self.solver_status == "Solved":
            can_execute_trajectory = True
            print "Optimal Trajectory has been found in " + str(end - start) + " secs"
            status = "Optimal Trajectory has been found in " + str(end - start) + " secs"
            self.logger.info(status)
        else:
            status = "Couldn't find the trajectory for the input problem"
            self.logger.info(status)
        return status, can_execute_trajectory

    def get_trajectory(self):
        return self.trajectory

    def callback_function_from_solver(self, new_trajectory, delta_trajectory=None):
        constraints, lower_limit, upper_limit = None, None, None
        trajectory = np.split(new_trajectory, self.no_of_samples)
        self.trajectory.add_trajectory(trajectory)

        collision_infos = self.callback_function_to_get_collision_infos(trajectory, self.current_planner_group,
                                                                        distance=self.collision_check_distance)

        if len(collision_infos[2]) > 0:
            constraints, lower_limit, upper_limit = \
                self.problem_model.update_collision_infos(collision_infos, self.collision_safe_distance)
            self.update_prob()

        return constraints, lower_limit, upper_limit

