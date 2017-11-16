def hessian_and_grad(x):
    """
    Calculate the hessian matrix with finite differences
    Parameters:
       - x : ndarray
    Returns:
       an array of shape (x.dim, x.ndim) + x.shape
       where the array[i, j, ...] corresponds to the second derivative x_ij
    """
    x_grad = np.gradient(x)
    hessian = np.empty((x.ndim, x.ndim) + x.shape, dtype=x.dtype)
    for k, grad_k in enumerate(x_grad):
        # iterate over dimensions
        # apply gradient again to every component of the first derivative.
        tmp_grad = np.gradient(grad_k)
        for l, grad_kl in enumerate(tmp_grad):
            hessian[k, l, :, :] = grad_kl
    return np.array(x_grad), hessian

def dispShape(P,grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2):
    print "P"
    print P.shape
    print "grad_P"

    print grad_P.shape
    print "hess_P"

    print hess_P.shape
    print "cons1"

    print cons1.shape
    print "grad_cons1"

    print grad_cons1.shape
    print "hess_cons1"

    print hess_cons1.shape
    print "grad_cons2"

    print grad_cons2.shape
    print "hess_cons2"

    print hess_cons2.shape

    print  "cons1 + grad_cons1 * p"
    print cons1 + grad_cons1

    print  "cons2 + grad_cons2"
    print cons2 + grad_cons2

def disp(P,grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2):
    print "P"
    print P
    print "grad_P"

    print grad_P
    print "hess_P"

    print hess_P
    print "cons1"

    print cons1
    print "grad_cons1"

    print grad_cons1
    print "hess_cons1"

    print hess_cons1
    print "grad_cons2"

    print grad_cons2
    print "hess_cons2"

    print hess_cons2

    print  "cons1 + grad_cons1 * p"
    print cons1 + grad_cons1

    print  "cons2 + grad_cons2"
    print cons2 + grad_cons2

import osqp
import numpy as np
from scipy.sparse import csc_matrix
import cvxpy
import scipy.sparse as sparse

P = np.array([[ 2., -4.,  0.],
             [ 0.,  4., -4.],
             [ 0.,  0.,  2.]])

q = np.array([ 0.,  0.,  0.])

A = np.array([[-1.,  1.,  0.],
             [ 0., -1.,  1.],
             [ 1.,  0.,  0.],
             [ 0.,  0.,  1.],
             [ 1.,  0.,  0.],
             [ 0.,  0.,  1.],
             [ 1.,  0.,  0.],
             [ 0.,  1.,  0.],
             [ 0.,  0.,  1.]])

lbA =   np.array([-0.3,-0.3, 0.2,0.7,0.2,0.7,-0.3,-0.3,-0.3])
ubA =   np.array([ 0.3, 0.3,0.2,0.7,0.2,0.7,1.1,1.1,1.1])
# P =  .5 * (P + P.transpose())
# P_csc = csc_matrix(P)
P = .5 * (P + P.T) + 1e-08 * sparse.eye(P.shape[0])

# A_csc = csc_matrix(A)

print cvxpy.installed_solvers()

mu = 1  # Penalty parameter


# m = osqp.OSQP()
# m.setup(P=P, q=q, A=A, l=lbA, u=ubA, verbose=True)
# results_osqp = m.solve()

# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])
x_0 =   np.array([ 0.2, 0.5, 0.7])
# p = cvxpy.Variable(P.shape[1])

grad_P, hess_P = hessian_and_grad(P)
cons1 = np.subtract(np.matmul(A, x_0), ubA)
cons2 = np.add(np.matmul(-A, x_0), lbA)
grad_cons1, hess_cons1 = hessian_and_grad(cons1)
grad_cons2, hess_cons2 = hessian_and_grad(cons2)

hess_cons1 =hess_cons1.reshape((hess_cons1.shape[2],))
hess_cons2 =hess_cons1.reshape((hess_cons2.shape[2],))
print hess_P
hess_P =  hess_P.reshape((hess_P.shape[0] * hess_P.shape[1]) * hess_P.shape[2], hess_P.shape[3] )
grad_P =  grad_P.reshape((grad_P.shape[0] * grad_P.shape[1]), grad_P.shape[2] )

# disp(P,grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2)

# objective = cvxpy.quad_form(x, P) + q * x
p = cvxpy.Variable(12)

if hess_P.shape[0] != hess_P.shape[1]:
    print hess_P.ndim
    print hess_P.shape
    print p.shape
# objective = cvxpy.quad_form(p, hess_P) + grad_P * p
# objective_model = objective + mu * (cvxpy.norm1(cons1 + grad_cons1 * p) + cvxpy.norm1(cons2 + grad_cons2 * p))

# print objective_model

# # Penalty part of the objective
# # objective += mu * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))
# # objective_model = objective + mu * (cvxpy.norm1((A * x + np.gradient(A)) - ubA ) + cvxpy.norm1((-A * x - np.gradient(A)) + lbA ))
# # print objective_model
# problem = cvxpy.Problem(cvxpy.Minimize(objective_model))
# objective_osqp_cvxpy = problem.solve(solver=cvxpy.SCS, verbose=True)


# print "Results OSQP CVXPY x =", np.round(x.value, 3)
