import numpy as np
import cvxpy


# from : https://stackoverflow.com/questions/31206443/numpy-second-derivative-of-a-ndimensional-array
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


def get_grad(x):
    return np.array(np.gradient(x))

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

def evaluate_objective(x):
    # print x
    # objective = np.matmul(x.T, P)
    # objective = np.matmul(objective,x) + np.matmul(q, x)
    # cons1 = np.subtract(np.matmul(A, x), ubA)
    # cons1 = np.linalg.norm(cons1, ord=1)
    # cons2 = np.subtract(np.matmul(-A, x), lbA)
    # cons2 = np.linalg.norm(cons2, ord=1)
    # objective += mu * (cons1 + cons2)
    # return objective
    objective = np.matmul(P, x_0)
    objective = 0.5 * np.matmul(x_0.T, objective) + np.matmul(q, x_0)
    return  objective

def evaluate_constraints(x):
    cons1 = np.matmul(A,x)
    cons1 += ubA
    cons2 = np.matmul(-A,x)
    cons2 += lbA
    return cons1, cons2



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
P = .5 * (P + P.T) + 1e-08 * np.eye(P.shape[0])


def evaluate_objective_model(x, p, mu):
    objective_model = evaluate_objective(x, mu)
    # cons1_m = np.matmul(np.matmul(get_grad(A), x), p)
    grad_A = get_grad(A)
    cons1 = np.matmul(np.matmul(grad_A, x), p)
    print cons1

    return objective_model




# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])
p = cvxpy.Variable(P.shape[0] * 1)

x_0 =   np.array([ 0.2, 0.5, 0.7])
p_0 = np.zeros(p.shape[0])
# p = cvxpy.Variable(P.shape[1])

trust_box_size = 0.1
merit = 1  # Penalty parameter


objective_at_xk = evaluate_objective(x_0)
cons1_at_xk, cons2_at_xk = evaluate_constraints(x_0)
# cons2_at_xk = cons1_at_xk.reshape(cons1_at_xk.shape[0], 1)
# cons2_at_xk = cons2_at_xk.reshape(cons2_at_xk.shape[0], 1)
cons1_grad = A
cons2_grad = -A
print objective_at_xk
print cons1_at_xk, cons2_at_xk
objective_jacob =  0.5 * (P + P.T)
# objective_jacob  = objective_jacob.reshape((x.shape[0], 1))
print objective_jacob
objective_hess = np.matmul(objective_jacob.T, objective_jacob)
# print objective_hess
objective_hess1 = P + P.T
# print objective_hess1

print cons1_at_xk.T.shape
cons1_model = cons1_at_xk +cons1_grad * p
cons2_model = cons2_at_xk + cons2_grad * p
objective_jacob_at_xk = np.matmul(objective_jacob, x_0)
objective_hess_at_xk = np.matmul(objective_hess, x_0)

print "objective_hess_at_xk", objective_hess_at_xk
# temp = cvxpy.quad_form(p, objective_hess_at_xk)
# objective_model += merit * (cons1_at_xk)
objective_jacob_at_xk = objective_jacob_at_xk.reshape(objective_jacob_at_xk.shape[0], 1)

objective_model = objective_at_xk + objective_jacob_at_xk.T * p + cvxpy.quad_form(p, objective_hess)

objective_model1 = objective_at_xk + objective_jacob_at_xk.T * p + cvxpy.quad_form(p, objective_hess1)


# constraints = cvxpy.constraints.constraint.Constraint([p <= delta, -delta <= p])
constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
# objective_model += mu * (cvxpy.norm1(A * p - ubA) + cvxpy.norm1(-A * p + lbA))
objective_model += merit * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))
objective_model1 += merit * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))

#
problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
objective_osqp_cvxpy = problem.solve(solver=cvxpy.ECOS, verbose=False)
print "p", p.value

problem1 = cvxpy.Problem(cvxpy.Minimize(objective_model1), constraints)
objective_osqp_cvxpy = problem1.solve(solver=cvxpy.ECOS, verbose=False)
print "p", p.value
print x_0 + p.value