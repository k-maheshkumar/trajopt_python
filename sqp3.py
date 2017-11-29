import numpy as np
import cvxpy
import warnings
import copy


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


def dispShape(P, grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2):
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


def disp(P, grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2):
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
    #     print x
    #     objective = np.matmul(x.T, P)
    #     objective = np.matmul(objective,x) + np.matmul(q, x)
    #     cons1 = np.subtract(np.matmul(A, x), ubA)
    #     cons1 = np.linalg.norm(cons1, ord=1)
    #     cons2 = np.subtract(np.matmul(-A, x), lbA)
    #     cons2 = np.linalg.norm(cons2, ord=1)
    #     objective += mu * (cons1 + cons2)
    #     return objective
    objective = np.matmul(P, x)
    objective = 0.5 * np.matmul(x.T, objective) + np.matmul(q, x)
    return objective


def evaluate_constraints(x):
    cons1 = np.matmul(A, x)
    cons1 += ubA
    cons2 = np.matmul(-A, x)
    cons2 += lbA
    return cons1, cons2


P = np.array([[2., -4., 0.],
              [0., 4., -4.],
              [0., 0., 2.]])

q = np.array([0., 0., 0.])

A = np.array([[-1., 1., 0.],
              [0., -1., 1.],
              [1., 0., 0.],
              [0., 0., 1.],
              [1., 0., 0.],
              [0., 0., 1.],
              [1., 0., 0.],
              [0., 1., 0.],
              [0., 0., 1.]])

lbA = np.array([-0.3, -0.3, 0.2, 0.7, 0.2, 0.7, -0.3, -0.3, -0.3])
ubA = np.array([0.3, 0.3, 0.2, 0.7, 0.2, 0.7, 1.1, 1.1, 1.1])
P = .5 * (P + P.T) + 1e-08 * np.eye(P.shape[0])

p_A = np.array([[1., 0., 0.],
                [0., 0., 1.]])
p_b = np.array([0.2, 0.7])


def convexifyProblem(objective_at_xk, x_0, penalty):
    cons1_grad = A
    cons2_grad = -A
    cons1_at_xk, cons2_at_xk = evaluate_constraints(x_0)

    # objective_at_xk = evaluate_objective(x_0)
    objective_jacob = 0.5 * (P + P.T)
    objective_hess = 0.5 * (P + P.T)
    # objective_hess = np.matmul(objective_jacob.T, objective_jacob)

    objective_jacob_at_xk = np.matmul(objective_jacob, x_0)

    objective_jacob_at_xk = objective_jacob_at_xk.reshape(objective_jacob_at_xk.shape[0], 1)

    objective_model = objective_at_xk + objective_jacob_at_xk.T * p + 0.5 * cvxpy.quad_form(p, objective_hess)

    cons1_model = cons1_at_xk + cons1_grad * p
    cons2_model = cons2_at_xk + cons2_grad * p

    objective_model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))

    # objective_model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))
    return objective_model


def evaluate_objective_model(objective_at_xk, objective_jacob, objective_hess, x, p, merit):
    # objective_model = evaluate_objective(x, mu)
    # # cons1_m = np.matmul(np.matmul(get_grad(A), x), p)
    # grad_A = get_grad(A)
    # cons1 = np.matmul(np.matmul(grad_A, x), p)
    # print cons1

    # return objective_model
    # print "p", p
    objective_hess_at_pk = np.matmul(objective_hess, p)
    objective_jacob_at_xk1 = np.matmul(objective_jacob, x)
    objective_model1 = objective_at_xk + np.matmul(objective_jacob_at_xk1.T, p) + 0.5 * np.matmul(p.T,
                                                                                                  objective_hess_at_pk)

    # cons1_model = cons1_at_xk + cons1_grad * p
    # cons2_model = cons2_at_xk + cons2_grad * p
    # cons1_model = cons1_at_xk
    p = p.reshape((p.shape[0], 1))
    cons1_at_xk = np.matmul(A, x) - ubA
    cons2_at_xk = np.matmul(-A, x) + lbA

    cons1_grad_at_pk = np.matmul(A, p)
    cons2_grad_at_pk = np.matmul(-A, p)
    # cons1_model += cons1_grad * p
    # print cons1_grad.shape, p.shape
    print "cons1_at_xk", cons1_at_xk
    print "cons2_at_xk", cons2_at_xk
    print "cons1_at_pk", cons1_grad_at_pk
    print "cons2_at_pk", cons2_grad_at_pk
    cons1_model = cons1_at_xk + cons1_grad_at_pk
    cons2_model = cons1_at_xk + cons2_grad_at_pk
    print "cons1_model", cons1_model
    print "cons2_model", cons2_model
    print "cvxpy.norm1(cons1_model)", cvxpy.norm1(cons1_model)

    # objective_model1 += merit * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))
    print "objective_model1", objective_model1
    return objective_model1


# Define CVXPY problem
x = cvxpy.Variable(P.shape[1])
p = cvxpy.Variable(P.shape[0] * 1)

x_0 = np.array([0.2, 0.8, 0.7])
p_0 = np.zeros(p.shape[0])
# p = cvxpy.Variable(P.shape[1])
penalty = 1
max_penalty = 10000000000000
min_trust_box_size = 1e-4
trust_box_size = 0.1
max_iteration = 20
iterationCount = 0
x_k = copy.deepcopy(x_0)
max_wsr = 2000
# Standard objective
min_improve_rhok_threshold = .25;

gamma = cvxpy.Parameter(nonneg=True)

isAdjustPenalty = False
trust_shrink_ratio = 0.25
min_model_improve = 1e-4;
trust_expand_ratio = 1.5
trust_good_region_ratio = 0.75

actual_objective = cvxpy.quad_form(x, P) + q * x
trust_box_shrink_ratio = 0.25
trust_box_expand_ratio = 1.5
p.value = p_0
gamma.value = penalty

actual_objective = gamma * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))
is_xk_changed = True
old_rho_k = 0
delta = 1
while penalty <= max_penalty:
    print ("penalty iteration", iterationCount)
    iterationCount = 0
    while iterationCount < max_iteration and trust_box_size >= min_trust_box_size:
        print ("iteration loop")
        iterationCount += 1
        while trust_box_size >= min_trust_box_size:
            # if is_xk_changed:
            is_xk_changed = False
            x.value = copy.copy(x_k)

            objective_at_xk = actual_objective.value
            gamma.value = penalty
            objective_model = convexifyProblem(objective_at_xk, x_k, gamma)
            # constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
            constraints = [cvxpy.norm2(p) <= trust_box_size]

            problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
            result = problem.solve(solver=cvxpy.SCS, verbose=False, max_iters=max_wsr, warm_start=True)

            p_k = copy.copy(p.value)
            x.value = x_k + p_k
            print "x_k + p_k ", x_k + p_k
            objective_at_xk_plus_pk = actual_objective.value
            x.value = x_k # restoring old x_k after getting x_k + p_k

            actual_prediction_reduction = objective_at_xk - objective_at_xk_plus_pk
            # print "actual_prediction ", actual_prediction_reduction
            objective_model_at_pk = objective_model.value
            p.value = p_0
            objective_model_at_p0 = objective_model.value
            p.value = p_k  # restoring old p_k after getting x_k + p_k
            model_prediction_reduction = objective_model_at_p0[0] - objective_model_at_pk[0]
            # print "model_prediction ", model_prediction_reduction

            rho_k = actual_prediction_reduction / model_prediction_reduction
            print "rho_k ", rho_k
            print "rho_k old ", old_rho_k
            print "evaluating constraints: "
            con1, con2 = evaluate_constraints(x_k)
            print "cons: ", np.linalg.norm(con1, 1), np.linalg.norm(con2, 1)

            if old_rho_k == rho_k:
                print "probably converged . .  .. . . . "
                isAdjustPenalty = True
                break

            old_rho_k = copy.copy(rho_k)

            if model_prediction_reduction < min_model_improve:
                # warnings.warn("converged because improvement was small (%.3e < %.3e)", modelImprove,
                #               min_model_improve)
                # print("converged because improvement was small", modelImprove,
                #               min_model_improve, iterationCount, penalty)
                isAdjustPenalty = True
                break
            if model_prediction_reduction / objective_model_at_pk[0] < -float("inf"):
                print "need to adjust penalty"
                isAdjustPenalty = True
                break

            if rho_k < 0 or rho_k < 0.25:
                print "shrinking trust region"
                trust_box_size *= trust_box_shrink_ratio
            elif rho_k >= 0.75:
                print "expanding trust region"
                trust_box_size *= trust_box_expand_ratio
                x_k += p_k

            if iterationCount >= max_iteration:
                break
        if isAdjustPenalty:
            break
    trust_box_size = np.fmin(min_trust_box_size * 1.1, trust_box_size / trust_box_shrink_ratio * 1.5)
    penalty *= 10

print penalty, trust_box_size, iterationCount






# while penalty <= max_penalty:
#     print "penalty iteration"
#     gamma.value = penalty
#     while iterationCount < max_iteration:
#         iterationCount += 1
#         print "iteration loop: ", iterationCount
#         while trust_box_size >= min_trust_box_size:
#             x.value = x_k
#
#             objective_at_xk = actual_objective.value
#
#             objective_model = convexifyProblem(objective_at_xk, x_k, gamma)
#             constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
#
#             problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
#             result = problem.solve(solver=cvxpy.SCS, verbose=False, max_iters=max_wsr, warm_start=True)
#
#             p_k = p.value
#             x.value = x_k + p_k
#             objective_at_xk_plus_pk = actual_objective.value
#             x.value = x_k # restoring old x_k after getting x_k + p_k
#
#             actual_prediction_reduction = objective_at_xk - objective_at_xk_plus_pk
#             # print "actual_prediction ", actual_prediction
#             objective_model_at_pk = objective_model.value
#             p.value = p_0
#             objective_model_at_p0 = objective_model.value
#             p.value = p_k  # restoring old p_k after getting x_k + p_k
#             model_prediction_reduction = objective_model_at_p0[0] - objective_model_at_pk[0]
#             # print "model_prediction ", model_prediction
#
#             rho_k = actual_prediction_reduction / model_prediction_reduction
#             print "rho_k ", rho_k
#             if rho_k < 0 or rho_k < 0.25:
#                 print "shrinking trust region: ", trust_box_size
#                 trust_box_size *= trust_box_shrink_ratio
#             elif rho_k > 0.5:
#                 print "expanding trust region: ", trust_box_size
#                 trust_box_size *= trust_box_expand_ratio
#                 x_k += p_k
#             print x_k + p_k, iterationCount
#             if iterationCount >= max_iteration:
#                 print "max iteration reached"
#                 break
#     print "increasing penalty", trust_box_size <= min_trust_box_size
#     if trust_box_size < min_trust_box_size:
#         trust_box_size = np.fmin(min_trust_box_size * 1.1, trust_box_size / trust_box_shrink_ratio * 1.5)
#         print "resetting trust box size to: ", trust_box_size, min_trust_box_size
#
#     penalty *= 10





# while penalty <= max_penalty:
#
#     iterationCount = 0
#     gamma.value = penalty
#     # if trust_box_size < min_trust_box_size:
#     trust_box_size *= 2
#
#     # print "new penalty:", gamma.value, trust_box_size
#     while iterationCount < max_iteration:
#         iterationCount += 1
#         # print "iteration limiter", iterationCount, trust_box_size
#         # if trust_box_size >= min_trust_box_size:
#         # print "trust box size is larger than min trust box size"
#         # else:
#         # print "fdsf"
#         # trust_box_size = 0.25
#
#         while trust_box_size >= min_trust_box_size:
#             # print "trust iteration"
#
#             x.value = x_k
#             # Penalty part of the objective
#             objective += gamma * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))
#
#             objective_model = convexifyProblem(objective.value, x_k, gamma)
#             constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
#             # constraints = [-trust_box_size <= cvxpy.norm(p, 1), cvxpy.norm(p, 1) <= trust_box_size]
#             # constraints = [-trust_box_size <= p, p <= trust_box_size]
#
#             problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
#             result = problem.solve(solver=cvxpy.SCS, verbose=False, max_iters=max_wsr, warm_start=True)
#
#             # problem1 = cvxpy.Problem(cvxpy.Minimize(objective))
#             # result1 = problem1.solve(solver=cvxpy.ECOS, verbose=False, max_iters=max_wsr, warm_start=True)
#             # print "Results: ", np.round(x.value, 3)
#             # print problem.status
#
#             if problem.status == cvxpy.INFEASIBLE or problem.status == cvxpy.INFEASIBLE_INACCURATE or problem.status == cvxpy.UNBOUNDED or problem.status == cvxpy.UNBOUNDED_INACCURATE:
#                 # print problem.status
#                 break
#
#                 # Todo: throw error when problem is not solved
#
#             p_k = p.value
#             # x_k += p_k
#
#             predicted_model_obj_at_pk = objective_model.value
#             # print "predicted_model_obj_at_pk: ", predicted_model_obj_at_pk
#             # predicted_model_obj_at_p0 = evaluate_objective_model(objective_at_xk, objective_jacob, objective_hess, x_k, p_0, merit)
#             p.value = p_0
#             predicted_model_obj_at_p0 = objective_model.value
#             # print "predicted_model_obj_at_p0: ", predicted_model_obj_at_p0
#
#             x.value = x_k
#             actual_obj_at_xk = objective.value
#
#             x.value = x_k + p_k
#             actual_obj_at_xk_plus_pk = objective.value
#
#             # actual_obj_at_xk = evaluate_objective(x_k)
#             # print "actual_obj_at_xk ", actual_obj_at_xk
#             # actual_obj_at_xk_plus_pk = evaluate_objective(x_k + p_k)
#             # print "actual_obj_at_xk_plus_pk ", actual_obj_at_xk_plus_pk
#
#             trueImprove = actual_obj_at_xk - actual_obj_at_xk_plus_pk  # actual reduction
#             # print "trueImprove", trueImprove
#
#             modelImprove = predicted_model_obj_at_p0[0] - predicted_model_obj_at_pk[0]  # predicted reduction
#             # print "model improve", modelImprove
#             rhoK = trueImprove / modelImprove
#             print rhoK
#             print x_k
#             temp = (np.linalg.norm(p_k, np.inf))
#             # print "temp", temp, trust_box_size
#
#             # if modelImprove < -1e-5:
#             #     # warnings.warn("approximate merit function got worse. (convexification is probably wrong to zeroth order)",
#             #     #               modelImprove)
#             #     # print("approximate merit function got worse. (convexification is probably wrong to zeroth order)",
#             #     #     modelImprove, iterationCount)
#             #     isAdjustPenalty = True
#             #     break
#             if modelImprove < min_model_improve:
#                 # warnings.warn("converged because improvement was small (%.3e < %.3e)", modelImprove,
#                 #               min_model_improve)
#                 # print("converged because improvement was small", modelImprove,
#                 #               min_model_improve, iterationCount, penalty)
#                 isAdjustPenalty = True
#                 break
#             if modelImprove / predicted_model_obj_at_pk < -float("inf"):
#                 print "need to adjust penalty"
#                 isAdjustPenalty = True
#                 break
#
#             if trueImprove < 0 or rhoK < min_improve_rhok_threshold:
#                 print "shrinking trust region", trust_box_size
#                 trust_box_size *= trust_shrink_ratio
#                 # x_k -= p_k
#                 break
#             # else:
#             if rhoK > trust_good_region_ratio or temp == trust_box_size:
#                 print "expanding trust region", trust_box_size
#                 trust_box_size *= trust_expand_ratio
#                 x_k += p_k
#                 break
#             elif rhoK <= 0.25:
#                 print "shrinking trust region", trust_box_size
#                 trust_box_size *= trust_shrink_ratio
#                 # x_k -= p_k
#
#
#                 # if trust_box_size <
#
#         if isAdjustPenalty:
#             isAdjustPenalty = False
#             penalty *= 10
#             # print "trust_box_size", trust_box_size
#
#             break
#
#         # for cons in problem.constraints:
#         #     print cons.residual
#
#
#         if iterationCount >= max_iteration:
#             print "max iterations reached"
#             break
#
# # trust_box_size_ = np.fmax(trust_box_size, min_trust_box_size / trust_shrink_ratio * 1.5)
# #         print "after fmax", trust_box_size
# #
# #             # print iterationCount
# #
# print ("x k", x_k)
# print ("x 0", x_0)
#

