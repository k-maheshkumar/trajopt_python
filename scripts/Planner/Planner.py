import numpy as np
from scripts.sqpproblem import SQPproblem
from scripts.sqpproblem import SQPsolver

import Trajectory as traj
from scripts.sqpproblem import ProblemBuilder as sqp

'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbC <= C * x <= ubC
            # lb <= x <= ub
            # A * x == b

'''


class TrajectoryOptimizationPlanner:
    def __init__(self, problem, solver, temp= False):

        self.problem = problem
        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.max_no_of_Iteration = problem["max_iteration"]
        self.joints = problem["joints"]
        self.num_of_joints = len(problem["joints"])
        self.solver = solver
        self.max_penalty = problem["max_penalty"]
        self.deltaMax = problem["max_delta"]

        self.sqp = []
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

        for i in range(self.num_of_joints):
            sp = sqp.ProblemBuilder(self.samples, self.duration, self.joints[i], self.max_no_of_Iteration)

            self.sqp.append(sp)

            self.initial_guess.append(self.interpolate(sp.start, sp.end, self.samples))

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)

            if temp == 1:
                self.A.append(self.sqp[i].A)
                self.b.append(self.sqp[i].b.tolist())

                self.G.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.samples)]))
                self.lbG.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
                self.ubG.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))
            else:

                self.A.append(np.vstack([self.sqp[i].A, self.sqp[i].A, self.sqp[i].A]))
                self.b.append(np.hstack([self.sqp[i].b.tolist(), self.sqp[i].b.tolist(), self.sqp[i].b.tolist()]))

                self.G.append(np.vstack([self.sqp[i].G, np.identity(self.samples)]))
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

        self.P = self.diag_block_mat_slicing(self.P)
        self.A = self.diag_block_mat_slicing(self.A)
        self.G = self.diag_block_mat_slicing(self.G)

        self.b = np.hstack(self.b)
        # self.q = [item for sublist in self.q for item in sublist]
        # self.q = np.asarray(self.q)

        self.G = self.G.astype(float)

        self.P = 2.0 * self.P + 1e-08 * np.eye(self.P.shape[1])

    def diag_block_mat_slicing(self, L):
        shp = L[0].shape
        N = len(L)
        r = range(N)
        out = np.zeros((N, shp[0], N, shp[1]), dtype=int)
        out[r, :, r, :] = L
        return out.reshape(np.asarray(shp) * N)

    def displayProblem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.G)
        print ("lb")
        print (self.lb)
        print ("ub")
        print (self.ub)
        print ("lbG")
        print (self.lbG)
        print ("ubG")
        print (self.ubG)
        print ("b")
        print (self.b)
        print ("A")
        print (self.A)

        print ("maxNoOfIteration")
        print (self.max_no_of_Iteration)

    def interpolate(self, start, end, samples):
        data = []
        stepSize = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += stepSize
        return np.round(data, 3)

    def get_trajectory(self, initial_guess= None, equality= False):
        print "getting trajectory"
        if equality:
            sp = SQPproblem.SQPProblem(self, self.solver)
        else:
            sp = SQPsolver.SQPsolver(self, self.solver)
        solver_status, trajectory = sp.solveSQP(initial_guess)
        trajectory = np.array((np.split(trajectory, self.num_of_joints)))
        trajectory = traj.Trajectory(trajectory)

        return solver_status, trajectory.get_trajectory()


