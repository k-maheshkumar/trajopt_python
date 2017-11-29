import cvxpy
import osqp
import scipy.sparse as sparse
import numpy as np

# Define problem data
P = np.array([[2., -4., 0.],
                       [0., 4., -4.],
                       [0., 0., 2.]])
# Make symmetric and not indefinite
P = .5 * (P + P.T) + 1e-08 * np.eye(3)

q = np.array([1., 1., 0.])
G = np.array([[-1., 1., 0.],
                       [0., -1., 1.],
                       [1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., 1.]])
l = np.array([-0.3, -0.3, -0.3, -0.3, -0.3])
u = np.array([0.3, 0.3,  1.1, 1.1, 1.1])

A = np.array([[1., 0., 0.],
              [0., 0., 1.]])
b = np.array([0.2, 0.7])

mu = 15  # Penalty parameter

# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])

# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

# Penalty part of the objective
objective += mu * (cvxpy.norm1(G * x - u) + cvxpy.norm1(-G * x + l) + cvxpy.norm1(A * x - b))

problem = cvxpy.Problem(cvxpy.Minimize(objective))
objective_osqp_cvxpy = problem.solve(solver=cvxpy.ECOS, verbose=False)

print "Results x =", np.round(x.value, 3)