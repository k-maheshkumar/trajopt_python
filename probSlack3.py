
import cvxpy
import osqp
import scipy.sparse as sparse
import numpy as np

# Define problem data
P = sparse.csc_matrix([[2., -4., 0.],
                       [0., 4., -4.],
                       [0., 0., 2.]])
# Make symmetric and not indefinite
P = .5 * (P + P.T) + 1e-08 * sparse.eye(3)
print P.todense()

q = np.array([1., 1., 0.])
A = sparse.csc_matrix([[-1., 1., 0.],
                       [0., -1., 1.],
                       [1., 0., 0.],
                       [0., 0., 1.],
                       [1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., 1.],
                       [1., 1., 0.]])
l = np.array([-0.3, -0.3, 0.2, 0.7, -0.3, -0.3, -0.3, 1.2])
u = np.array([0.3, 0.3, 0.2, 0.7, 1.1, 1.1, 1.1, 1.8])

mu = 15  # Penalty parameter

# m = osqp.OSQP()
# m.setup(P=P, q=q, A=A, l=l, u=u, verbose=True)
# results_osqp = m.solve()  # this works

# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])

# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

# Penalty part of the objective
objective += mu * (cvxpy.norm1(A * x - u) + cvxpy.norm1(-A * x + l))

problem = cvxpy.Problem(cvxpy.Minimize(objective))
objective_osqp_cvxpy = problem.solve(solver=cvxpy.OSQP, verbose=True)


# print "Result OSQP Python x =", np.round(results_osqp.x, 3)
print "Results OSQP CVXPY x =", np.round(x.value, 3)