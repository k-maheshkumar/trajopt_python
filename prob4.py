from sqpproblem import SQPproblem as sqp
from trajPlanner import trajPlanner
import numpy as np
# from sqpproblem import SQPproblem.SQPproblem as sp

request = {
    "samples" : 8,
    "duration" : 20,
    "maxIteration" : 10000,
    "joints" : [
        {"start": 0.2, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        {"start": 0.3, "end": 0.9, 'xOPt': 0.1, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},

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
P = np.array([
    [1., - 2.,  0.],
    [0.,  2., - 2.],
    [0.,  0., 1.]])

q = np.array([ 0.,  0.,  0.])


G = np.array([
    [-1.,  1.,  0.],
    [ 0., -1.,  1.]])

A = np.array([
    [1., 0., 0.],
    [0., 0., 1.]])

# l = array([3., 2., -2.]).reshape((3,))
#
#
# h = array([3., 2., -2.]).reshape((3,))

lb = np.array([-0.3, -0.3, -0.3])
ub = np.array([1.1, 1.1,  1.1])

lbG = np.array([-0.25, -0.25])
ubG = np.array([0.25,  0.25])

b = np.array([0.2,  0.7])


# nwsr = array([100])


# from qpsolvers.qpsolvers import qpoases_ as qp
#
# sol = qp.qpoases_solve_qp(P, q, G, lb, ub, lbG, ubG, A, b, initvals=None,
#                      max_wsr=100)
#
# print sol


sp = trajPlanner.TrajectoryPlanner(request)

# sp = trajPlanner.solveQp()

# sp.displayProblem()
sp.solveProblem()
#
# example, num = sp.solveProblem()
# # print num
# xOpt = np.zeros(num)
# example.getPrimalSolution(xOpt)
# #
# # # analyser = SolutionAnalysis()
# # # maxStat = np.zeros(1)
# # # maxFeas = np.zeros(1)
# # # maxCmpl = np.zeros(1)
# # # analyser.getKktViolation(example, maxStat, maxFeas, maxCmpl)
# # # print("maxStat: %e, maxFeas:%e, maxCmpl: %e\n"%(maxStat, maxFeas, maxCmpl))
# #
# # print "solution"
# # # print xOpt , example.getObjVal()
# #
# numJoints = len(request["joints"])
# print "numJoints", numJoints
# print np.split(xOpt, numJoints)
# # print("\nxOpt = [ %e, %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],xOpt[2],example.getObjVal()))
# # example.printOptions()