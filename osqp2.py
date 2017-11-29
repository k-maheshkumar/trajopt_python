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
                       [0., 0., 1.],
                       [0., 1., 1.]])
l = np.array([-0.3, -0.3, 0.2, 0.7, -0.3, -0.3, -0.3, 1.2])
u = np.array([0.3, 0.3, 0.2, 0.7, 1.1, 1.1, 1.1, 1.8])


def evaluate_constraints(x_k):
    cons1 = np.subtract(np.matmul(A, x_k), u)
    cons2 = np.add(np.matmul(-A, x_k), l)
    return cons1, cons2


def get_constraints_gradients():
    cons1_gradi = A
    cons2_gradi = -A
    return cons1_gradi, cons2_gradi


def get_objective_model_gradient_and_hessian(x_k):

    model_grad = 0.5 * np.matmul((P + P.T), x_k)
    model_grad = model_grad.reshape((1, x_k.shape[0]))
    # model_hess = 0.5 * (P + P.T)
    model_hess = np.matmul(model_grad.T, model_grad)
    # print model_grad
    return model_grad, model_hess

mu = 1  # Penalty parameter
# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])
p = cvxpy.Variable(P.shape[1])

x.value = np.array([0.1, 0.6, 0.9])
# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

penalty = cvxpy.Parameter(nonneg= True)
penalty.value = 1
# Penalty part of the objective
objective += penalty * (cvxpy.norm1(A * x - u) + cvxpy.norm1(-A * x + l))

problem = cvxpy.Problem(cvxpy.Minimize(objective))


max_penalty = 1e10
# initial guess value . . . . .. . . . .. . . . . . . .
x_0 = np.array([1.2, 1.6, 1.8])
p_0 = np.zeros(x.shape[0])
x_k = copy.copy(x_0)

max_iteration = 20
iteration_count = 0
is_penalty_adjust = False
trust_box_size = 0.5
min_trust_box_size = 1e-4
trust_box_size_shrink_ratio = 0.25
trust_box_size_expand_ratio = 2
min_model_improve = 1e-4;

is_converged = False
while penalty.value <= max_penalty:
    while iteration_count < max_iteration:
        iteration_count += 1
        while trust_box_size >= min_trust_box_size:
            # print iteration_count
            # objective_osqp_cvxpy = problem.solve(solver= cvxpy.SCS, verbose= False, warm_start= True)

            cons1_at_x_k, cons2_at_x_k = evaluate_constraints(x_k)
            cons1_grad, cons2_grad = get_constraints_gradients()

            cons1_model = cons1_at_x_k + cons1_grad * p
            cons2_model = cons2_at_x_k + cons2_grad * p
            objective_grad_at_x_k, objective_hess_at_x_k = get_objective_model_gradient_and_hessian(x_k)

            objective_model = objective.value + objective_grad_at_x_k * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_x_k) + \
                              penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))
            constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
            problem_model = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
            result_model = problem_model.solve(solver=cvxpy.SCS, verbose=False, warm_start=True)

            p_k = copy.copy(p.value)
            # print "p_k ", p_k
            # print "x_k + p_k", x_k + p_k

            objective_model_at_p_k = copy.copy(objective_model.value)
            objective_at_x_k = copy.copy(objective.value)

            # print objective_at_x_k
            # print objective_model_at_p_k

            x.value = x_k + p_k
            objective_at_x_k_plus_p_k = copy.copy(objective.value)

            p.value = p_0
            objective_model_at_p_0 = copy.copy(objective_model.value)

            # print objective_model_at_p_0
            # print objective_at_x_k_plus_p_k

            predicted_reduction = objective_model_at_p_0[0] - objective_model_at_p_k[0]
            actual_reduction = objective_at_x_k - objective_at_x_k_plus_p_k

            rho_k = actual_reduction / predicted_reduction
            max_p_k = np.linalg.norm(p_k, np.inf)
            print "p k", p_k
            print "x_k + p_k", x_k + p_k , trust_box_size, penalty.value
            # print "evaluating constraints... ", evaluate_constraints(x_k + p_k)
            print "max_p_k", max_p_k
            print "rho_k", rho_k, trust_box_size
            # print "x_k + p_k", x_k + p_k

            if predicted_reduction < min_model_improve:
                # warnings.warn("converged because improvement was small (%.3e < %.3e)", modelImprove,
                #               min_model_improve)
                print("converged because improvement was small")
                isAdjustPenalty = True
                break
            if predicted_reduction / objective_model_at_p_k[0] < -float("inf"):
                print "need to adjust penalty"
                isAdjustPenalty = True
                break

            if rho_k == 1 and problem_model.status == cvxpy.OPTIMAL:
                is_converged = True
                break

            if actual_reduction <= 0:
                is_penalty_adjust = True
                break

            if rho_k <= 0.25:
                print "shrinking trust region", rho_k
                trust_box_size *= 0.25
            elif rho_k >= 0.75 or max_p_k == trust_box_size:
                print "expanding trust region", rho_k
                trust_box_size *= 2
                x_k += p_k

            if iteration_count >= max_iteration:
                break
        if is_penalty_adjust:
            is_penalty_adjust = False
            break
        if is_converged:
            break
    if is_converged:
        break
    trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_box_size_shrink_ratio * 0.5)
    penalty.value *= 10

print x_k + p_k
