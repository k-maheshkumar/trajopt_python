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


def convexifyProblem(x_0, penalty):
    cons1_grad = A
    cons2_grad = -A
    cons1_at_xk, cons2_at_xk = evaluate_constraints(x_0)

    objective_at_xk = evaluate_objective(x_0)
    objective_jacob = 0.5 * (P + P.T)
    objective_hess = P + P.T

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

x_0 =   np.array([ 0.2, 0.4, 0.7])
p_0 = np.zeros(p.shape[0])
# p = cvxpy.Variable(P.shape[1])
penalty = 10
trust_box_size = 0.1
max_penalty = 100000
minDelta = 0.00001
max_iter_ = 10
iterationCount = 0
x_k = x_0
max_wsr = 2000
while penalty <= max_penalty:
    penalty *= 10
    # print "penalty iteration"
    while iterationCount <= max_iter_:
        iterationCount += 1
        # print "iteration limiter"
    #     while trust_box_size >= minDelta:
    #         print "trust iteration"


        # Standard objective
        objective = cvxpy.quad_form(x, P) + q * x

        # Penalty part of the objective
        objective += penalty * (cvxpy.norm1(A * x - ubA) + cvxpy.norm1(-A * x + lbA))

        objective_model = convexifyProblem(x_k, penalty)
        constraints = [cvxpy.norm(p, "inf") <= trust_box_size]

        problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
        result = problem.solve(solver=cvxpy.ECOS, verbose=False, max_iters = max_wsr, warm_start=True)
        # print "Results: ", np.round(p.value, 3)
        # print problem.value

        if problem.status == cvxpy.INFEASIBLE or problem.status == cvxpy.INFEASIBLE_INACCURATE or problem.status == cvxpy.UNBOUNDED or problem.status == cvxpy.UNBOUNDED_INACCURATE:
            # print problem.status
            break

            # Todo: throw error when problem is not solved
        else:
            p_k = p.value
            x_k += p_k
            predicted_model_obj_at_pk = objective_model.value
            # print "predicted_model_obj_at_pk: ", predicted_model_obj_at_pk
            # predicted_model_obj_at_p0 = evaluate_objective_model(objective_at_xk, objective_jacob, objective_hess, x_k, p_0, merit)
            p.value = p_0
            predicted_model_obj_at_p0 = objective_model.value
            # print "predicted_model_obj_at_p0: ", predicted_model_obj_at_p0

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

            modelImprove = predicted_model_obj_at_p0 - predicted_model_obj_at_pk # predicted reduction
            # print "model improve", modelImprove
            rhoK = trueImprove / modelImprove[0]
            # print modelImprove, trueImprove, rhoK
            print "rho",  rhoK, problem.status
            print "x_new", x_k + p_k
            # for constraint in problem.constraints:
            #     print constraint.violation()
            if rhoK < 0.25:
                # print "shrinking .....", iterationCount
                trust_box_size = 0.25 * trust_box_size
                x_k -= p_k

            else:
                if rhoK > 0.75:
                    print "expanding... .. ... . .", iterationCount
                    # delta = min(2 * delta, )
                    trust_box_size = 2 * trust_box_size
                    # trust_box_size_ = np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5)

                    x_k += p_k
                    # print "inside else",x_k

            # if modelImprove < min_model_improve:
            #     print "les"
            #     break
            if iterationCount >= max_iter_:
                print "last print ",trust_box_size, penalty
                break

    print "finally", x_k
