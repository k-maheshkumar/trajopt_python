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


def convexifyProblem(objective_at_xk, x_0, penalty):
    cons1_grad = A
    cons2_grad = -A
    cons1_at_xk, cons2_at_xk = evaluate_constraints(x_0)

    # objective_at_xk = evaluate_objective(x_0)
    objective_jacob = 0.5 * (P + P.T)
    objective_hess = P + P.T
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
    objective_model1 = objective_at_xk + np.matmul(objective_jacob_at_xk1.T, p) + 0.5 * np.matmul(p.T, objective_hess_at_pk)

    # cons1_model = cons1_at_xk + cons1_grad * p
    # cons2_model = cons2_at_xk + cons2_grad * p
    # cons1_model = cons1_at_xk
    p = p.reshape((p.shape[0], 1))
    cons1_at_xk = np.matmul(A, x) - ubA
    cons2_at_xk = np.matmul(-A, x) +lbA

    cons1_grad_at_pk = np.matmul(A, p)
    cons2_grad_at_pk = np.matmul(-A, p)
    # cons1_model += cons1_grad * p
    # print cons1_grad.shape, p.shape
    print "cons1_at_xk",  cons1_at_xk
    print "cons2_at_xk",  cons2_at_xk
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

x_0 =   np.array([ 0.1, 0.3, 0.6])
p_0 = np.zeros(p.shape[0])
# p = cvxpy.Variable(P.shape[1])
penalty = 1
trust_box_size = 0.5
max_penalty = 10000
minDelta = 0.01
max_iter_ = 20
iterationCount = 0
x_k = x_0
max_wsr = 2000
# Standard objective
objective = cvxpy.quad_form(x, P) + q * x

gamma = cvxpy.Parameter(nonneg=True)

min_model_improve= 1e-4;
improve_ratio_threshold = .25;
min_approx_improve_frac = - float('inf')
while penalty <= max_penalty:
    penalty *= 10
    gamma.value = penalty
    # print "penalty iteration"
    while iterationCount <= max_iter_:
        iterationCount += 1
        # print "iteration limiter"
        # Penalty part of the objective

        objective += gamma * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))

        objective_model = convexifyProblem(objective.value, x_k, gamma)
        constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
        problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)

        while trust_box_size >= minDelta:
            #         print "trust iteration"

            result = problem.solve(solver=cvxpy.ECOS_BB, verbose=False, max_iters = max_wsr, warm_start=True)

            # problem1 = cvxpy.Problem(cvxpy.Minimize(objective))
            # result1 = problem1.solve(solver=cvxpy.ECOS, verbose=False, max_iters=max_wsr, warm_start=True)
            # print "Results: ", np.round(x.value, 3)
            # print problem.value

            if problem.status == cvxpy.INFEASIBLE or problem.status == cvxpy.INFEASIBLE_INACCURATE or problem.status == cvxpy.UNBOUNDED or problem.status == cvxpy.UNBOUNDED_INACCURATE:
                print problem.status
                breakz

                # Todo: throw error when problem is not solved
            else:
                p_k = p.value
                print "pk: ", p_k
                predicted_model_obj_at_pk = objective_model.value
                print "predicted_model_obj_at_pk: ", predicted_model_obj_at_pk
                p.value = p_0
                predicted_model_obj_at_p0 = objective_model.value
                print "predicted_model_obj_at_p0: ", predicted_model_obj_at_p0
                # predicted_model_obj_at_p0 = evaluate_objective_model(objective_at_xk, objective_jacob, objective_hess, x_k, p_0, merit)

                x.value = x_k
                actual_obj_at_xk = objective.value

                x.value = x_k + p_k
                actual_obj_at_xk_plus_pk = objective.value

                # actual_obj_at_xk = evaluate_objective(x_k)
                # print "actual_obj_at_xk ", actual_obj_at_xk
                # actual_obj_at_xk_plus_pk = evaluate_objective(x_k + p_k)
                # print "actual_obj_at_xk_plus_pk ", actual_obj_at_xk_plus_pk

                trueImprove = actual_obj_at_xk - actual_obj_at_xk_plus_pk # actual reduction
                # print "trueImprove", trueImprove

                # modelImprove = predicted_model_obj_at_p0[0] - predicted_model_obj_at_pk[0] # predicted reduction
                modelImprove = predicted_model_obj_at_p0 - predicted_model_obj_at_pk # predicted reduction
                modelImprove = modelImprove[0]

                # print "model improve", modelImprove

                rhoK = trueImprove / modelImprove

                if modelImprove < -1e-5:
                    print ("approximate merit function got worse (%.3e). (convexification is probably wrong to zeroth order)", modelImprove)

                if modelImprove < min_model_improve:
                    print("converged because improvement was small (%.3e < %.3e)", modelImprove, min_model_improve)
                # retval = OPT_CONVERGED;
                # goto penaltyadjustment;
                print modelImprove
                temp = modelImprove / actual_obj_at_xk
                print temp
                if temp  < min_approx_improve_frac:
                    print("converged because improvement ratio was small ")
                    # (%.3e < %.3e),approx_merit_improve / old_merit, min_approx_improve_frac_)
                # retval = OPT_CONVERGED
                # goto penaltyadjustment;

                else:
                    if trueImprove < 0 or merit_improve_ratio < improve_ratio_threshold:
                    # adjustTrustRegion(trust_shrink_ratio_)
                        trust_box_size = 0.25 * trust_box_size
                        print("shrunk trust region. new box size: ",  trust_box_size)

                    else:
                        x_k += p_k;
                        results_.cost_vals = new_cost_vals;
                        # results_.cnt_viols = new_cnt_viols;
                        # adjustTrustRegion(trust_expand_ratio_);
                        trust_box_size = 2 * trust_box_size
                        print("expanded trust region. new box size: %.4f", trust_box_size)
                        break;


                        # print modelImprove, trueImprove, rhoK
                        print "rho",  rhoK, problem.status, trust_box_size
                        print "x_new", x_k
                        # for constraint in problem.constraints:
                        #     print constraint.violation()
                        if rhoK < 0.25:
                            print "shrinking .....", rhoK, iterationCount
                            trust_box_size = 0.25 * trust_box_size
                            x_k -= p_k

                        else:
                            if rhoK > 0.75:
                                print "expanding... .. ... . .", rhoK, iterationCount
                                # delta = min(2 * delta, )
                                trust_box_size = 2 * trust_box_size
                                # trust_box_size_ = np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5)

                                x_k += p_k
                                print "inside else",x_k

                        # if modelImprove < min_model_improve:
                        #     print "les"
                        #     break
                        if iterationCount >= max_iter_:
                            print "last print ",trust_box_size, penalty
                            break

    print "finally", x_k
