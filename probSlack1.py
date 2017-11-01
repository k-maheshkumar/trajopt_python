import osqp
import numpy as np
from scipy.sparse import csc_matrix
import cvxpy

"""
    minimize x,   0.5 * xT P x + qT x
    subject to  lA <= Ax <= uA
    
    minimize x, (x[i, t+1] - x[i, t]) ** 2
    
    subject to,
        -0.3 <= x <= 0.3
        -0.3 <= (x[i,t] - x[i, t+1]) * (duration/ (Num of samples  - 1)) <= 1.1   
        
        with x[0, 0] = 0.2, x[0, t] = 0.7,
             x[1, 0] = 0.2, x[1, t] = 0.7
    
    where, 
        Num of samples = 3,
        duration = 6
        x = [x(0,0), x(1,0), . . . . ., x(i-1, t-1), x(i, t)]
        i = joints for i = 0 to Num of joints
        t = time for t = 0 to (duration  - 1)
        
    
    
    
"""


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
             [ 0.,  0.,  0.,  0.,  0.,  1.]])
lbA =   np.array([-0.3,-0.3, 0.2,0.7,0.2,0.7,-0.3,-0.3,-0.3,-0.3,-0.3,0.3,0.9,0.3,0.9,-0.3,-0.3,-0.3])
ubA =   np.array([ 0.3, 0.3,0.2,0.7,0.2,0.7,1.1,1.1,1.1,0.3,0.3,0.3,0.9,0.3,0.9,1.1,1.1,1.1])
# P =  .5 * (P + P.transpose())
# P_csc = csc_matrix(P)
# A_csc = csc_matrix(A)
# initialX = np.array([0.1, 0.2, 0.2, 0.1, 0.2, 0.2])
# m = osqp.OSQP()
# m.setup(P=P_csc, q=q, A=A_csc, l=lbA, u=ubA, max_iter = 1000, verbose=True)
# m.warm_start(x=initialX)
#
# results = m.solve()
# print results.x



import cvxpy
import scipy.sparse as sparse
import numpy as np

# Define problem data
# P = sparse.csc_matrix([[4., 1.], [1., 2.]])
# q = np.array([1., 1.])
# A = sparse.csc_matrix([[1., 0.], [0., 1.], [1., 0.], [0., 1.], [1., 1.]])
# l = np.array([0., 0., 0.2, 0.9, -0.2])
# u = np.array([1., 1., 0.2, 0.9, 1.5])
mu = 15  # Penalty parameter


# Define CVXPY problem
x = cvxpy.Variable(P.shape[0])
# Standard objective
objective = cvxpy.quad_form(x, P) + q * x
# Penalty part of the objective
objective += mu * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))
problem = cvxpy.Problem(cvxpy.Minimize(objective), [])
results = problem.solve(solver=cvxpy.OSQP, verbose=False)
print results
print x.value
