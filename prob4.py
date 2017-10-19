from sqpproblem import SQPproblem as sqp
from trajPlanner import trajPlanner
import numpy as np
# from sqpproblem import SQPproblem.SQPproblem as sp
from scipy.sparse import csc_matrix

request = {
    "samples" : 3,
    "duration" : 6,
    "maxIteration" : 1000,
    "joints" : [
        {"start": 0.2, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        # {"start": 0.3, "end": 0.9, 'xOPt': 0.1, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},


    ]
}


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

lb = np.array([-0.3, -0.3, -0.3])
ub = np.array([1.1, 1.1,  1.1])

lbG = np.array([-0.25, -0.25])
ubG = np.array([0.25,  0.25]).reshape((2,))

b = np.array([0.2,  0.7]).reshape((2,))


# nwsr = array([100])

sp = trajPlanner.TrajectoryPlanner(request, "osqp")
# sp.displayProblem()
# sp.solveProblem()
sp.solveQpProb1()



# from qpsolvers.qpsolvers import qpoases_ as qp
#
# sol = qp.qpoases_solve_qp(P, q, G, lb, ub, lbG, ubG, A, b, initvals=None,
#                      max_wsr=100)
#
# print sol

