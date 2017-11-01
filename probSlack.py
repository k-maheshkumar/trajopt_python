import osqp
import numpy as np
from scipy.sparse import csc_matrix
import cvxpy
import scipy.sparse as sparse

P = np.array([[ 2., -4.,  0.,  0.,  0.,  0.],
             [ 0.,  4., -4.,  0.,  0.,  0.],
             [ 0.,  0.,  2.,  0.,  0.,  0.],
             [ 0.,  0.,  0.,  2., -4.,  0.],
             [ 0.,  0.,  0.,  0.,  4., -4.],
             [ 0.,  0.,  0.,  0.,  0. , 2.]])

q = np.array([ 0.,  0.,  0.,  0.,  0.,  0.])

A = np.array([[-1.,  1.,  0.,  0.,  0., 0.],
             [ 0., -1.,  1.,  0.,  0.,  0.],
             [ 1.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  1.,  0.,  0.,  0.],
             [ 1.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  1.,  0.,  0.,  0.],
             [ 1.,  0.,  0.,  0.,  0.,  0.],
             [ 0.,  1.,  0.,  0.,  0.,  0.],
             [ 0.,  0.,  1.,  0.,  0.,  0.],
             [ 0.,  0.,  0., -1.,  1.,  0.],
             [ 0.,  0.,  0.,  0., -1.,  1.],
             [ 0.,  0.,  0.,  1.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  1.],
             [ 0.,  0.,  0.,  1.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  1.],
             [ 0.,  0.,  0.,  1.,  0.,  0.],
             [ 0.,  0.,  0.,  0.,  1.,  0.],
             [ 0.,  0.,  0.,  0.,  0.,  1.],
             [ 0,  1.,  0.,  0,  1.,  0.]])

lbA =   np.array([-0.3,-0.3, 0.2,0.7,0.2,0.7,-0.3,-0.3,-0.3,-0.3,-0.3,0.3,0.9,0.3,0.9,-0.3,-0.3,-0.3, 1.7])
ubA =   np.array([ 0.3, 0.3,0.2,0.7,0.2,0.7,1.1,1.1,1.1,0.3,0.3,0.3,0.9,0.3,0.9,1.1,1.1,1.1, 1.8])
# P =  .5 * (P + P.transpose())
P_csc = csc_matrix(P)
P = .5 * (P + P.T) + 1e-08 * sparse.eye(P.shape[0])

# G_csc = csc_matrix(G)
A_csc = csc_matrix(A)
# initialX = np.array([0.1, 0.2, 0.5])

# from qpsolvers.qpsolvers import osqp_ as qp
#
# sol = qp.osqp_solve_qp(P_csc, q, G_csc, h, A, b, initvals=None)
#
# print sol
#

# print "P"
# print P_csc.todense()
# print "q"
# print q
# print "A"
# print A_csc.todense()
# # print "lb"
# # print lb
# # print "ub"
# # print ub
# print "lbA"
# print lbA
# print "ubA"
# print ubA
# P_csc = .5 * (P_csc + P_csc.transpose())
# m = osqp.OSQP()
# m.setup(P=P_csc, q=q, A=A_csc, l=lbA, u=ubA, max_iter = 1000, verbose=True)
# # m.warm_start(x=initialX)
#
# results = m.solve()
# print results.x

mu = 15  # Penalty parameter


m = osqp.OSQP()
m.setup(P=P_csc, q=q, A=A_csc, l=lbA, u=ubA, verbose=True)
results_osqp = m.solve()

# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])

# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

# Penalty part of the objective
objective += mu * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))

problem = cvxpy.Problem(cvxpy.Minimize(objective))
objective_osqp_cvxpy = problem.solve(solver=cvxpy.OSQP, verbose=True)


# print "Result OSQP Python x =", np.round(results_osqp.x, 3)
print "Results OSQP CVXPY x =", np.round(x.value, 3)
print "Result OSQP Python x =", np.round(results_osqp.x, 3)