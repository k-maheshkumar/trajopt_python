import cvxpy
import osqp
import scipy.sparse as sparse
import numpy as np
import  copy
# Define problem data
P = np.array([[2., -4., 0.],
                       [0., 4., -4.],
                       [0., 0., 2.]])
# Make symmetric and not indefinite
P = .5 * (P + P.T) + 1e-08 * np.eye(3)

q = np.array([1., 1., 0.])
A = np.array([[-1., 1., 0.],
                       [0., -1., 1.],
                       [1., 0., 0.],
                       [0., 0., 1.],
                       [1., 0., 0.],
                       [0., 1., 0.],
                       [0., 0., 1.]])
l = np.array([-0.3, -0.3, 0.2, 0.7, -0.3, -0.3, -0.3])
u = np.array([0.3, 0.3, 0.2, 0.7, 1.1, 1.1, 1.1])

def evaluate_constraints(x_k):
    cons1 = np.subtract(np.matmul(A, x_k), u)
    cons2 = np.add(np.matmul(-A, x_k), l)
    return cons1, cons2


def get_constraints_gradients():
    cons1_grad = A
    cons2_grad= -A
    return cons1_grad, cons2_grad


def get_objective_model_gradient_and_hessian(x_k):

    model_grad = 0.5 * np.matmul((P + P.T), x_k)
    model_hess = 0.5 * (P + P.T)
    return model_grad, model_hess

mu = 1  # Penalty parameter
# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])
p = cvxpy.Variable(P.shape[1])

x.value = np.array([0.1, 0.6, 0.9])

penalty = cvxpy.Parameter(nonneg= True)
penalty.value = 1

# Standard objective
objective = 0.5 * cvxpy.quad_form(x, P) + q * x
# Penalty part of the objective
objective += penalty * (cvxpy.norm1(A * x - u) + cvxpy.norm1(-A * x + l))

problem = cvxpy.Problem(cvxpy.Minimize(objective))


max_penalty = 1e0

x_0 = np.array([0.3, 0.6, 0.9])
x_k = copy.copy(x_0)
while penalty.value <= max_penalty:
    penalty.value *= 10
    # objective_osqp_cvxpy = problem.solve(solver= cvxpy.SCS, verbose= False, warm_start= True)

    cons1_at_x_k, cons2_at_x_k = evaluate_constraints(x_k)
    cons1_grad, cons2_grad = get_constraints_gradients()

    cons1_model = cons1_at_x_k + cons1_grad * p
    cons2_model = cons2_at_x_k + cons2_grad * p
    objective_grad_at_x_k, objective_hess_at_x_k = get_objective_model_gradient_and_hessian(x_k)

    objective_model = objective.value + objective_grad_at_x_k.T * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_x_k) + \
                      penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))
    problem_model = cvxpy.Problem(cvxpy.Minimize(objective_model))
    result_model = problem_model.solve(solver=cvxpy.SCS, verbose=False, warm_start=True)

    p_k = copy.copy(p.value)
    print "p_k ", p_k, objective_model.value, objective.value
    print "x_k + p_k", x_k + p_k , penalty.value


