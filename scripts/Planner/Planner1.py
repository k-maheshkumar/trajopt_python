import numpy as np
from scripts.sqpproblem import SQPproblem
from scripts.sqpproblem import SQPsolver

import Trajectory
from scripts.sqpproblem import ProblemBuilder as sqp
import logging
import time
import scripts.sqpproblem.ProblemModelling as model

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
        self.solver_class = 0

        self.solver_config = None
        self.trajectory = Trajectory.Trajectory()
        # self.sqp_solver1 = SQPproblem.SQPProblem()
        # self.sqp_solver = SQPsolver.SQPsolver()
        self.sqp_solver1 = None
        self.sqp_solver = None

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

            if "solver_class" in kwargs:
                self.solver_class = kwargs["solver_class"]

            if "solver_config" in kwargs:
                self.solver_config = kwargs["solver_config"]

            if self.solver_class:
                self.sqp_solver = SQPsolver.SQPsolver()
            else:
                self.sqp_solver = SQPproblem.SQPProblem()

            self.problem = model.ProblemModelling()
            self.problem.init(self.joints, self.no_of_samples, self.duration, self.decimals_to_round)
            self.sqp_solver.init(P=self.problem.cost_matrix_P, q=self.problem.cost_matrix_q,
                                 G=self.problem.constraints_matrix,
                                 lbG=self.problem.constraints_lower_limits, ubG=self.problem.constraints_upper_limits,
                                 A=self.problem.start_and_goal_matrix, b=self.problem.start_and_goal_limits,
                                 initial_guess=self.problem.initial_guess, solver_config=self.solver_config)
            self.trajectory.init(np.array((np.split(self.problem.initial_guess, self.no_of_samples))), self.problem.samples, self.problem.duration)
            # self.trajectory.init(np.array(self.problem.initial_guess), self.problem.samples, self.problem.duration)


    def display_problem(self):
        self.sqp_solver.display_problem()

    # def calculate_trajectory(self):
    #     status, trajectory = self.sqp_solver1.solveSQP()
    #
    #     return
    #     # print trajectory
    #     # print problem.cost_matrix
        # print problem.velocity_matrix

    def calculate_trajectory(self, initial_guess= None):
        can_execute_trajectory = False
        self.logger.info("getting trajectory")
        # if self.solver_class:
        #     self.sqp_solver.init(self, self.solver, self.solver_config, self.verbose)
        #     start = time.time()
        #     self.solver_status, self.trajectory = self.sqp_solver.solveSQP(initial_guess)
        #     end = time.time()
        #
        # else:
        #     # self.sqp_solver1.init(self, self.solver, self.solver_config, self.verbose)
        #     self.sqp_solver1.init(P=self.problem.cost_matrix_P, q=self.problem.cost_matrix_q, G=self.problem.constraints_matrix,
        #                           lbG=self.problem.constraints_lower_limits, ubG=self.problem.constraints_upper_limits,
        #                           A=self.problem.start_and_goal_matrix, b=self.problem.start_and_goal_limits,
        #                           initial_guess=self.problem.initial_guess, solver_config=self.solver_config)
        start = time.time()
        self.solver_status, trajectory = self.sqp_solver.solve(initial_guess)
        end = time.time()
        trajectory = np.array((np.split(trajectory, self.no_of_samples)))

        # trajectory = dict(zip(self.joint_names, trajectory))
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
