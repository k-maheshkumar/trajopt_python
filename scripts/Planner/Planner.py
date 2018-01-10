import numpy as np
from scripts.sqpproblem import SQPproblem
from scripts.sqpproblem import SQPsolver

import Trajectory
from scripts.sqpproblem import ProblemBuilder as sqp
import logging
import time

class TrajectoryOptimizationPlanner:
    # def __init__(self, problem, solver, temp= False):
    #
    #     self.problem = problem
    #     self.samples = problem["samples"]
    #     self.duration = problem["duration"]
    #     self.max_no_of_Iteration = problem["max_iteration"]
    #     self.joints = problem["joints"]
    #     self.num_of_joints = len(problem["joints"])
    #     self.solver = solver
    #     self.max_penalty = problem["max_penalty"]
    #     self.delta_max = problem["max_delta"]
    #     self.initialise(temp)
    #
    #
    # def __init__(self, joints, samples, duration, max_no_of_Iteration=20, max_penalty=1e6,
    #              delta_max=5, solver="SCS", temp= True):
    #     self.samples = samples
    #     self.duration = duration
    #     self.max_no_of_Iteration = max_no_of_Iteration
    #     self.joints = joints
    #     self.num_of_joints = len(joints)
    #     self.solver = solver
    #     self.max_penalty = max_penalty
    #     self.delta_max = delta_max
    #
    #     self.initialise(temp)

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
        self.decimals_to_round = -1
        self.decimals_to_round = 3
        self.solver_class = -1

        self.solver_config = {}
        self.trajectory = Trajectory.Trajectory()
        self.sqp_solver = SQPproblem.SQPProblem()
        self.sqp_solver1 = SQPsolver.SQPsolver()

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
                self.decimals_to_round = 3

            if "solver_class" in kwargs:
                self.solver_class = kwargs["solver_class"]
                self.initialise(self.solver_class)

            if "solver_config" in kwargs:
                self.solver_config = kwargs["solver_config"]

    def initialise1(self, solver_class):
        for i in range(self.num_of_joints):
            sp = sqp.ProblemBuilder(self.no_of_samples, self.duration, self.joints[i], self.decimals_to_round)

            self.sqp.append(sp)

            # self.initial_guess.append(self.interpolate(sp.start, sp.end, self.samples))
            self.initial_guess.append(self.interpolate(sp.start + 0.02, sp.end + 0.02, self.no_of_samples))

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)

            if solver_class:
                self.A.append(self.sqp[i].A)
                self.b.append(self.sqp[i].b.tolist())

                self.G.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.no_of_samples)]))
                self.lbG.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
                self.ubG.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))

                # self.G.append(np.vstack([self.sqp[i].G, self.sqp[i].G,
                #                          self.sqp[i].A, self.sqp[i].A, self.sqp[i].A, self.sqp[i].A, self.sqp[i].A,
                #                          np.identity(self.samples), np.identity(self.samples)]))
                # self.lbG.append(np.hstack([self.sqp[i].lbG, self.sqp[i].lbG,
                #                            self.sqp[i].b, self.sqp[i].b, self.sqp[i].b, self.sqp[i].b, self.sqp[i].b,
                #                            self.sqp[i].lb, self.sqp[i].lb]))
                # self.ubG.append(np.hstack([self.sqp[i].ubG, self.sqp[i].ubG,
                #                            self.sqp[i].b, self.sqp[i].b, self.sqp[i].b, self.sqp[i].b, self.sqp[i].b,
                #                            self.sqp[i].ub, self.sqp[i].ub]))
            else:

                self.A.append(np.vstack([self.sqp[i].A, self.sqp[i].A, self.sqp[i].A]))
                self.b.append(np.hstack([self.sqp[i].b.tolist(), self.sqp[i].b.tolist(), self.sqp[i].b.tolist()]))

                self.G.append(np.vstack([self.sqp[i].G, np.identity(self.no_of_samples)]))
                self.lbG.append(np.hstack([self.sqp[i].lbG, self.sqp[i].lb]))
                self.ubG.append(np.hstack([self.sqp[i].ubG, self.sqp[i].ub]))

            self.lb.append(self.sqp[i].lb.tolist())
            self.ub.append(self.sqp[i].ub.tolist())

        self.initial_guess = np.hstack(self.initial_guess).flatten()
        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.ub = np.hstack(self.ub)
        self.lbG = np.hstack(self.lbG)
        self.ubG = np.hstack(self.ubG)

        self.P = self.__diagonal_block_mat_slicing(self.P)
        self.A = self.__diagonal_block_mat_slicing(self.A)
        self.G = self.__diagonal_block_mat_slicing(self.G)

        self.b = np.hstack(self.b)
        # self.q = [item for sublist in self.q for item in sublist]
        # self.q = np.asarray(self.q)

        self.G = self.G.astype(float)

        self.P = 2.0 * self.P + 1e-08 * np.eye(self.P.shape[1])

    def initialise(self, solver_class):
        for joint_name in self.joints:
            self.joint_names.append(joint_name)
            self.sqp[joint_name] = sqp.ProblemBuilder(self.no_of_samples, self.duration, self.joints[joint_name], self.decimals_to_round)
            # self.trajectory[joint_name] = -1
            # self.initial_guess.append(self.interpolate(sp.start, sp.end, self.samples))
            self.initial_guess.append(self.interpolate(self.sqp[joint_name].start + 0.02, self.sqp[joint_name].end + 0.02, self.no_of_samples))

            self.P.append(self.sqp[joint_name].P)
            self.q.append(self.sqp[joint_name].q)

            if solver_class:
                self.A.append(self.sqp[joint_name].A)
                self.b.append(self.sqp[joint_name].b.tolist())

                self.G.append(np.vstack([self.sqp[joint_name].G, self.sqp[joint_name].A, self.sqp[joint_name].A, np.identity(self.no_of_samples)]))
                self.lbG.append(np.hstack([self.sqp[joint_name].lbG, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].lb]))
                self.ubG.append(np.hstack([self.sqp[joint_name].ubG, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].ub]))

                # self.G.append(np.vstack([self.sqp[joint_name].G, self.sqp[joint_name].G,
                #                          self.sqp[joint_name].A, self.sqp[joint_name].A, self.sqp[joint_name].A, self.sqp[joint_name].A, self.sqp[joint_name].A,
                #                          np.identity(self.samples), np.identity(self.samples)]))
                # self.lbG.append(np.hstack([self.sqp[joint_name].lbG, self.sqp[joint_name].lbG,
                #                            self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b,
                #                            self.sqp[joint_name].lb, self.sqp[joint_name].lb]))
                # self.ubG.append(np.hstack([self.sqp[joint_name].ubG, self.sqp[joint_name].ubG,
                #                            self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b, self.sqp[joint_name].b,
                #                            self.sqp[joint_name].ub, self.sqp[joint_name].ub]))
            else:

                self.A.append(np.vstack([self.sqp[joint_name].A, self.sqp[joint_name].A, self.sqp[joint_name].A,
                                         self.sqp[joint_name].A, self.sqp[joint_name].A]))
                self.b.append(np.hstack([self.sqp[joint_name].b.tolist(), self.sqp[joint_name].b.tolist(),
                                         self.sqp[joint_name].b.tolist(), self.sqp[joint_name].b.tolist(),
                                         self.sqp[joint_name].b.tolist()]))
                if self.sqp[joint_name].collision_matrix is None:
                    self.G.append(np.vstack([self.sqp[joint_name].G, np.identity(self.no_of_samples)]))
                    self.lbG.append(np.hstack([self.sqp[joint_name].lbG, self.sqp[joint_name].lb]))
                    self.ubG.append(np.hstack([self.sqp[joint_name].ubG, self.sqp[joint_name].ub]))
                else:
                    self.G.append(np.vstack([self.sqp[joint_name].G, np.identity(self.no_of_samples),
                                             self.sqp[joint_name].collision_matrix, self.sqp[joint_name].collision_matrix]))
                    self.lbG.append(np.hstack([self.sqp[joint_name].lbG, self.sqp[joint_name].lb,
                                               self.sqp[joint_name].lower_collision_limit,
                                               self.sqp[joint_name].lower_collision_limit]))
                    self.ubG.append(np.hstack([self.sqp[joint_name].ubG, self.sqp[joint_name].ub,
                                               self.sqp[joint_name].upper_collision_limit,
                                               self.sqp[joint_name].upper_collision_limit]))

            self.lb.append(self.sqp[joint_name].lb.tolist())
            self.ub.append(self.sqp[joint_name].ub.tolist())

        self.initial_guess = np.hstack(self.initial_guess).flatten()
        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.ub = np.hstack(self.ub)
        self.lbG = np.hstack(self.lbG)
        self.ubG = np.hstack(self.ubG)

        self.P = self.__diagonal_block_mat_slicing(self.P)
        self.A = self.__diagonal_block_mat_slicing(self.A)
        self.G = self.__diagonal_block_mat_slicing(self.G)

        self.b = np.hstack(self.b)
        # self.q = [item for sublist in self.q for item in sublist]
        # self.q = np.asarray(self.q)

        self.G = self.G.astype(float)

        self.P = 2.0 * self.P + 1e-08 * np.eye(self.P.shape[1])

    # noinspection PyMethodMayBeStatic
    def __diagonal_block_mat_slicing(self, matrix):
        shape = matrix[0].shape
        length = len(matrix)
        length_range = range(length)
        out = np.zeros((length, shape[0], length, shape[1]), dtype=int)
        out[length_range, :, length_range, :] = matrix
        return out.reshape(np.asarray(shape) * length)

    def display_problem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.G)
        print ("lbG")
        print (self.lbG)
        print ("ubG")
        print (self.ubG)
        print (self.A)
        print ("b")
        print (self.b)
        print ("lb")
        print (self.lb)
        print ("ub")
        print (self.ub)

    def interpolate(self, start, end, samples):
        data = []
        step_size = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += step_size
        return np.round(data, self.decimals_to_round)

    def calculate_trajectory(self, initial_guess= None):
        can_execute_trajectory = False
        self.logger.info("getting trajectory")
        if self.solver_class:
            self.sqp_solver.init(self, self.solver, self.solver_config, self.verbose)
            start = time.time()
            self.solver_status, self.trajectory = self.sqp_solver.solveSQP(initial_guess)
            end = time.time()

        else:
            self.sqp_solver1.init(self, self.solver, self.solver_config, self.verbose)
            start = time.time()
            self.solver_status, trajectory = self.sqp_solver1.solve(initial_guess)
            end = time.time()

        trajectory = np.array((np.split(trajectory, self.num_of_joints)))

        trajectory = dict(zip(self.joint_names, trajectory))
        self.trajectory.update(trajectory, self.no_of_samples, self.duration)
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
