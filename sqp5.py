import numpy as np
import cvxpy
import warnings
import copy

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
lbA = np.array([-0.3, -0.3, 0.2, 0.7, -0.3, -0.3, -0.3])
ubA = np.array([0.3, 0.3, 0.2, 0.7, 1.1, 1.1, 1.1])

p_A = np.array([[1., 0., 0.],
                [0., 0., 1.]])
p_b = np.array([0.2, 0.7])


def evaluate_constraints(x_k):
    cons1 = np.subtract(np.matmul(A, x_k), ubA)
    cons2 = np.add(np.matmul(-A, x_k), lbA)
    return cons1, cons2


def get_constraints_grad():
    cons1_grad = A
    cons2_grad = -A
    return cons1_grad, cons2_grad


def get_objective_grad_and_hess(g):
    model_grad = 0.5 * np.matmul((P + P.T), g)
    model_hess = 0.5 * (P + P.T)
    return model_grad, model_hess


def get_model_objective(xk, penalty, p):
    cons1_at_xk, cons2_at_xk = evaluate_constraints(xk)
    cons1_grad_at_xk, cons2_grad_at_xk = get_constraints_grad()
    cons1_model = cons1_at_xk + cons1_grad_at_xk * p
    cons2_model = cons2_at_xk + cons2_grad_at_xk * p

    objective_grad_at_xk, objective_hess_at_xk = get_objective_grad_and_hess(xk)
    objective_at_xk = get_actual_objective(xk, penalty)
    model = objective_at_xk.value + objective_grad_at_xk * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_xk)

    model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))

    return model, objective_at_xk


def get_actual_objective(xk, penalty):
    x = cvxpy.Variable(P.shape[0])
    x.value = copy.copy(xk)
    objective = 0.5 * cvxpy.quad_form(x, P) + q * x
    objective += penalty * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))

    return objective


def sovle_problem(xk, penalizer, p, delta):
    model_objective, actual_objective = get_model_objective(xk, penalizer, p)
    constraints = [cvxpy.norm(p, "inf") <= delta]
    problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
    result = problem.solve(solver=cvxpy.SCS, warm_start=True, verbose=False)
    return p.value, model_objective, actual_objective, problem.status


def main():
    x = cvxpy.Variable(P.shape[0])
    p = cvxpy.Variable(x.shape[0])
    penalty = cvxpy.Parameter(nonneg= True)
    penalty.value = 1
    x_0 = np.array([2, 1, 2.0])
    p_0 = np.zeros(p.shape[0])
    trust_box_size = 0.5
    max_penalty = 1e4
    min_trust_box_size = 1e-4
    x_k = copy.copy(x_0)
    max_trust_box_size = 5

    trust_shrink_ratio = 0.25
    min_model_improve = 1e-4;
    trust_expand_ratio = 2

    trust_good_region_ratio = 0.75
    max_iteration = 20
    iteration_count = 0

    min_model_improve = 1e-4;
    improve_ratio_threshold = .25;
    min_approx_improve_frac = - float('inf')
    is_converged = False
    isAdjustPenalty = False

    old_rho_k = 0
    new_x_k = copy.copy(x_0)
    min_actual_redution = 1e-1
    while penalty.value <= max_penalty:
        print "penalty ", penalty.value
        while iteration_count < max_iteration:
            iteration_count += 1
            print "iteration_count", iteration_count
            while trust_box_size >= min_trust_box_size:
                print "iteration_count in trust loop", iteration_count

                p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = sovle_problem(x_k, penalty, p, trust_box_size)

                print "pk ", p_k, solver_status

                actual_objective_at_x_plus_p_k = get_actual_objective(x_k + p_k, penalty)
                model_objective_at_p_0 = get_actual_objective(p_0, penalty)

                # print "objective_at_x_plus_p_k", actual_objective_at_x_plus_p_k.value
                # print "model_objective_at_x_plus_p_k", model_objective_at_p_0.value
                print "actual xk, xk1", actual_objective_at_x_plus_p_k.value, actual_objective_at_x_k.value
                print "model p0, pk",model_objective_at_p_0.value, model_objective_at_p_k.value

                actual_reduction = actual_objective_at_x_plus_p_k.value - actual_objective_at_x_k.value
                predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                rho_k = actual_reduction / predicted_reduction

                print "rho_k", rho_k, abs(actual_reduction)
                print "x_k + pk ",x_k + p_k, trust_box_size, penalty.value, abs(actual_reduction)
                # print evaluate_constraints(x_k)

                max_p_k = (np.linalg.norm(p_k, np.inf))
                if solver_status == cvxpy.INFEASIBLE or solver_status == cvxpy.INFEASIBLE_INACCURATE or solver_status == cvxpy.UNBOUNDED or solver_status == cvxpy.UNBOUNDED_INACCURATE:
                    # print problem.status
                    # Todo: throw error when problem is not solved
                    break

                if old_rho_k == rho_k:
                    print "rho_k are same"
                    isAdjustPenalty = True
                    break

                if abs(actual_reduction) <= min_actual_redution:
                    print "actual reduction is very small"
                    is_converged = True
                    break
                # if rho_k > trust_good_region_ratio or max_p_k == trust_box_size:
                #     trust_box_size *= trust_expand_ratio
                #     print "expanding trust region", trust_box_size
                #     x_k += p_k
                #     break
                # elif rho_k <= 0.25:
                #     print "shrinking trust region", trust_box_size
                #     trust_box_size *= trust_shrink_ratio


                # if predicted_reduction < 0.00001:
                #     # warnings.warn("approximate merit function got worse. (convexification is probably wrong to zeroth order)",
                #     #               modelImprove)
                #     print("approximate merit function got worse. (convexification is probably wrong to zeroth order)",
                #           predicted_reduction, iteration_count)
                #     isAdjustPenalty = True
                #     break
                # if predicted_reduction < min_model_improve and iteration_count != 1:
                #     # warnings.warn("converged because improvement was small (%.3e < %.3e)", modelImprove,
                #     #               min_model_improve)
                #     print("converged because improvement was small", predicted_reduction,
                #                   min_model_improve, iteration_count, penalty.value)
                #     is_converged = True
                #     break
                if predicted_reduction / model_objective_at_p_k.value < -float("inf"):
                    print "need to adjust penalty"
                    isAdjustPenalty = True
                    break
                if rho_k <= 0.25:
                    trust_box_size *= trust_shrink_ratio
                    print "shrinking trust region", trust_box_size
                    break
                    x_k = copy.copy(new_x_k)
                else:
                # elif rho_k >= 0.75:
                    trust_box_size = min(trust_box_size * trust_expand_ratio, max_trust_box_size)
                    print "expanding trust region", trust_box_size
                    x_k += p_k
                    new_x_k = copy.copy(x_k)
                    break


                if iteration_count >= max_iteration:
                    # print "max iterations reached"
                    break
                old_rho_k = rho_k
            if is_converged:
                break
            trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
        if is_converged or isAdjustPenalty:
            break
        penalty.value *= 10
        iteration_count = 0
    print "initial x_0 ", x_0
    print "final x_k", x_k, trust_box_size
    print "final x_k + p_k", x_k + p_k



    # print get_objective_at_xk(x_0, penalty)
    # print sovle_problem(x_0, penalty, p, trust_box_size)
    # print evaluate_constraints(x_0)


main()

