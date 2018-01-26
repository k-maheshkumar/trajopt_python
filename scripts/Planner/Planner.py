import logging
import time

import numpy as np

import scripts.sqp_solver.ProblemModelling as model
from scripts.Robot import Trajectory
from scripts.sqp_solver import SQPsolver
from scripts.sqpproblem import SQPproblem


class TrajectoryOptimizationPlanner:

    def __init__(self, *args, **kwargs):
        self.sqp = {}
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
        self.problem = {}
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
        self.lower_safe_distance_threshold = 0.5
        self.upper_safe_distance_threshold = 2
        self.solver_class = 0

        self.solver_config = None
        self.trajectory = Trajectory.Trajectory()
        self.sqp_solver = None
        self.planner_group = None

        self.verbose = False
        self.logger = logging.getLogger("Trajectory_Planner."+__name__)

    def __clear_all_data(self):
        self.sqp = {}
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
                if "lower_safe_distance_threshold" in self.problem:
                    self.lower_safe_distance_threshold = self.problem["lower_safe_distance_threshold"]
                if "upper_safe_distance_threshold" in self.problem:
                    self.upper_safe_distance_threshold = self.problem["upper_safe_distance_threshold"]
                if "joint_group" in self.problem:
                    self.planner_group = self.problem["joint_group"]

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

            if "lower_safe_distance_threshold" in kwargs:
                self.lower_safe_distance_threshold = float(kwargs["lower_safe_distance_threshold"])
            if "upper_safe_distance_threshold" in kwargs:
                self.upper_safe_distance_threshold = float(kwargs["upper_safe_distance_threshold"])

            if "solver_class" in kwargs:
                self.solver_class = kwargs["solver_class"]

            if "solver_config" in kwargs:
                self.solver_config = kwargs["solver_config"]

            if self.solver_class:
                self.sqp_solver = SQPsolver.SQPsolver()
            else:
                self.sqp_solver = SQPproblem.SQPProblem()

            self.problem = model.ProblemModelling()
            self.problem.init(self.joints, self.no_of_samples, self.duration, self.decimals_to_round,
                              self.lower_safe_distance_threshold, self.upper_safe_distance_threshold)
            self.sqp_solver.init(P=self.problem.cost_matrix_P, q=self.problem.cost_matrix_q,
                                 G=self.problem.robot_constraints_matrix,
                                 lbG=self.problem.constraints_lower_limits, ubG=self.problem.constraints_upper_limits,
                                 A=self.problem.start_and_goal_matrix, b=self.problem.start_and_goal_limits,
                                 initial_guess=self.problem.initial_guess, solver_config=self.solver_config)
            self.trajectory.init(np.array((np.split(self.problem.initial_guess[-self.no_of_samples:], self.no_of_samples)))
                                 , self.problem.samples, self.problem.duration, self.planner_group)
            # self.trajectory.init(np.array(self.problem.initial_guess), self.problem.samples, self.problem.duration)


    def display_problem(self):
        self.sqp_solver.display_problem()

    def calculate_trajectory(self, initial_guess= None, callback_function=None):
        can_execute_trajectory = False
        self.logger.info("getting trajectory")
        start = time.time()
        self.solver_status, trajectory = self.sqp_solver.solve(initial_guess, callback_function)
        end = time.time()
        trajectory = np.array((np.split(trajectory, self.no_of_samples)))
        self.trajectory.update(trajectory, self.joints.keys())
        status = "-1"
        if self.solver_status == "Solved":
            can_execute_trajectory = True
            status = "Optimal Trajectory has been found in " + str(end - start) + " secs"
            self.logger.info(status)
        else:
            status = "Couldn't find the trajectory for the input problem"
            self.logger.info(status)
        return status, can_execute_trajectory

    def get_trajectory(self):
        return self.trajectory
