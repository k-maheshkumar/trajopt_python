from sqpproblem import SQPproblem as sqp
from trajPlanner import trajPlanner
import time

import numpy as np
# from sqpproblem import SQPproblem.SQPproblem as sp
from scipy.sparse import csc_matrix

request = {
    "samples" : 3,
    "duration" : 6,
    "maxIteration" : 1000,
    "joints" : [
        # { "end": 0.7, 'initialGuess': 0.2, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},

        {"start": 0.2, "end": 0.7, 'initialGuess': 0.2,"lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        {"start": 0.3, "end": 0.9, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.9, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.3, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.22, "end": 0.6, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.6, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.3, "end": 0.6, 'initialGuess': 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},

    ]
}

# request = {'duration': 15,
#            'joints': [
#                {'end': -0.0546, 'max_velocity': 0.2, 'start': 1.937e-05, 'upper_joint_limit': 2.96, 'min_velocity': -0.2, 'id': 0, 'lower_joint_limit': -2.967},
#                {'end': -0.00098, 'max_velocity': 0.2, 'start': 0.000238, 'upper_joint_limit': 2.09, 'min_velocity': -0.2, 'id': 1, 'lower_joint_limit': -2.0943},
#                {'end': -0.0558, 'max_velocity': 0.2, 'start': -0.0011, 'upper_joint_limit': 2.96, 'min_velocity': -0.2, 'id': 2, 'lower_joint_limit': -2.967},
#                {'end': -2.0, 'max_velocity': 0.2, 'start': -1.5708, 'upper_joint_limit': 2.094, 'min_velocity': -0.2, 'id': 3, 'lower_joint_limit': -2.094},
#                {'end': -0.047293, 'max_velocity': 0.2, 'start': 1.3125e-07, 'upper_joint_limit': 2.967, 'min_velocity': -0.2, 'id': 4, 'lower_joint_limit': -2.967},
#                {'end': -0.3398, 'max_velocity': 0.2, 'start': -1.036, 'upper_joint_limit': 2.094, 'min_velocity': -0.2, 'id': 5, 'lower_joint_limit': -2.094},
#                {'end': 0.1073, 'max_velocity': 0.2, 'start': 0.00016, 'upper_joint_limit': 3.05, 'min_velocity': -0.2, 'id': 6, 'lower_joint_limit': -3.054}
#            ],
#            'maxIteration': 100,
#            'samples': 10}



'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbG <= G * x <= ubG
            lb <= x <= ub
            A * x == b

'''
# P = np.array([
#     [1., - 2.,  0.],
#     [0.,  2., - 2.],
#     [0.,  0., 1.]])

# q = np.array([ 0.,  0.,  0.])


# G = np.array([
#     [-1.,  1.,  0.],
#     [ 0., -1.,  1.]]).reshape((2,3))

# A = np.array([
#     [1., 0., 0.],
#     [0., 0., 1.]]).reshape((2,3))

# l = array([3., 2., -2.]).reshape((3,))
#
#
# h = array([3., 2., -2.]).reshape((3,))
#
# lb = np.array([-0.3, -0.3, -0.3])
# ub = np.array([1.1, 1.1,  1.1])
#
# lbG = np.array([-0.25, -0.25])
# ubG = np.array([0.25,  0.25]).reshape((2,))
#
# b = np.array([0.2,  0.7]).reshape((2,))


# nwsr = array([100])

# sp = trajPlanner.TrajectoryPlanner(request, "osqp")
# start = time.time()
# result, sol = sp.solveProblem()
# end = time.time()
# print("osqp",end - start)
# print sol
# sp = trajPlanner.TrajectoryPlanner(request, "osqp1")
# start = time.time()
# sp.solveProblem()
# end = time.time()
# print("qpoases",end - start)
sp = trajPlanner.TrajectoryPlanner(request, "OSQP")
start = time.time()
sp.solveProblem()
end = time.time()
print("cvxopt",end - start)
# sp.displayProblem()
#
# result, sol = sp.solveProblem()
# # sp.solveQpProb1()
# print (sol)
# print (result.info.status)
#


# from qpsolvers.qpsolvers import qpoases_ as qp
#
# sol = qp.qpoases_solve_qp(P, q, G, lb, ub, lbG, ubG, A, b, initvals=None,
#                      max_wsr=100)
#
# print sol

