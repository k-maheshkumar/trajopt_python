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
             [ 0,  1.,  0.,  0,  1.,  0.]
                ])

lbA =   np.array([-0.3,-0.3, 0.2,0.7,0.2,0.7,-0.3,-0.3,-0.3,-0.3,-0.3,0.3,0.9,0.3,0.9,-0.3,-0.3,-0.3
                     , 1.7
                  ])
ubA =   np.array([ 0.3, 0.3,0.2,0.7,0.2,0.7,1.1,1.1,1.1,0.3,0.3,0.3,0.9,0.3,0.9,1.1,1.1,1.1
                     , 1.8
                   ])
# P =  .5 * (P + P.transpose())
# P_csc = csc_matrix(P)
P = .5 * (P + P.T) + 1e-08 * sparse.eye(P.shape[0])

# A_csc = csc_matrix(A)

print cvxpy.installed_solvers()

mu = 10  # Penalty parameter


# m = osqp.OSQP()
# m.setup(P=P, q=q, A=A, l=lbA, u=ubA, verbose=True)
# results_osqp = m.solve()

# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])

# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

# Penalty part of the objective
objective += mu * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))

problem = cvxpy.Problem(cvxpy.Minimize(objective))
objective_osqp_cvxpy = problem.solve(solver=cvxpy.SCS, verbose=True)


# print "Result OSQP Python x =", np.round(results_osqp.x, 3)
print "Results OSQP CVXPY x =", np.round(x.value, 3)
# print results_osqp.info,Status
# print "Result OSQP Python x =", np.round(results_osqp.x, 3)
