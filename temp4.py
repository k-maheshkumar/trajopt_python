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

def evaluate_objective(x, mu):
    print x
    objective = np.matmul(x.T, P)
    objective = np.matmul(objective,x) + np.matmul(q, x)
    cons1 = np.subtract(np.matmul(A, x), ubA)
    cons1 = np.linalg.norm(cons1, ord=1)
    cons2 = np.subtract(np.matmul(-A, x), lbA)
    cons2 = np.linalg.norm(cons2, ord=1)
    objective += mu * (cons1 + cons2)
    return objective





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
merit = 1
trust_box_size = 0.01
max_merit_increases = 100
minDelta = 0.001
max_iter_ = 100
iterationCount = 0
x_k = x_0
while merit <= max_merit_increases:
    merit *= 5
    while iterationCount <= max_iter_:
        iterationCount += 1
        while trust_box_size >= minDelta:

            grad_objective, hess_P = hessian_and_grad(P)
            # cons1 = np.subtract(np.matmul(A, x_0), ubA)
            # cons2 = np.add(np.matmul(-A, x_0), lbA)
            # cons1 = np.subtract(np.matmul(A, x_0), ubA)
            # cons2 = np.add(np.matmul(-A, x_0), lbA)
            grad_cons1 = get_grad(-A)
            grad_cons2 = get_grad(A)
            # dispShape(P,grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2)

            # hess_cons1 =hess_cons1.reshape((hess_cons1.shape[2],))
            # hess_cons2 =hess_cons1.reshape((hess_cons2.shape[2],))
            # print grad_P
            grad_objective =  grad_objective.reshape((grad_objective.shape[0] * grad_objective.shape[1]), grad_objective.shape[2])
            # print grad_P

            # disp(P,grad_P, hess_P, cons1, grad_cons1, hess_cons1, cons2, grad_cons2, hess_cons2)

            objective_at_x0 = np.matmul(x_0.T, P)
            objective_at_x0 = np.matmul(objective_at_x0, x_0)

            objective_model = objective_at_x0

            objective_model = grad_objective * p
            hess_objective = np.matmul(grad_objective.T, grad_objective)

            objective_model = cvxpy.quad_form(p, 2 * hess_objective)

            grad_cons1 = grad_cons1.reshape(A.shape[0] * 2, x.shape[0])
            grad_cons2 = grad_cons2.reshape(A.shape[0] * 2, x.shape[0])
            grad_lbA = np.gradient(lbA)
            grad_ubA = np.gradient(ubA)

            cons1_model = grad_cons1
            cons1_model = np.matmul(grad_cons1, x_0) + (grad_cons1 * x_0) * p
            cons2_model = np.matmul(grad_cons2, x_0) + (grad_cons2 * x_0) * p

            trust_box_size = 0.1
            merit = 1  # Penalty parameter

            # constraints = cvxpy.constraints.constraint.Constraint([p <= delta, -delta <= p])
            constraints = [cvxpy.norm(p, "inf") <= trust_box_size]
            # objective_model += mu * (cvxpy.norm1(A * p - ubA) + cvxpy.norm1(-A * p + lbA))
            objective_model += merit * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))

            problem = cvxpy.Problem(cvxpy.Minimize(objective_model), constraints)
            objective_osqp_cvxpy = problem.solve(solver=cvxpy.SCS, verbose=False)
            # print "Results: ", np.round(p.value, 3)
            # print problem.value

            if problem.status == cvxpy.INFEASIBLE or problem.status == cvxpy.INFEASIBLE_INACCURATE or problem.status == cvxpy.UNBOUNDED or problem.status == cvxpy.UNBOUNDED_INACCURATE:
                print problem.status
                break

                # Todo: throw error when problem is not solved
            else:
                p_k = p.value
                # x_k += p_k
                predicted_model_obj_at_pk = problem.value
                predicted_model_obj_at_p0 = evaluate_objective_model(x_k, p_k, merit)
                modelImprove = predicted_model_obj_at_p0 - predicted_model_obj_at_pk
                actual_obj_at_xk = evaluate_objective(x_k, merit)
                actual_obj_at_xk_plus_pk = evaluate_objective(x_k + p_k, merit)
                trueImprove = actual_obj_at_xk - actual_obj_at_xk_plus_pk
                rhoK = trueImprove / modelImprove
                # print modelImprove, trueImprove, rhoK
                print "rho",  rhoK, problem.status
                print "x_new", x_k + p_k

                if rhoK < 0.25:
                    print "shrinking ....."
                    trust_box_size = 0.25 * trust_box_size
                else:
                    if rhoK > 0.75:
                        print "expanding... .. ... . .", iterationCount
                        # delta = min(2 * delta, )
                        trust_box_size = 2 * trust_box_size
                        # trust_box_size_ = np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5)

                        x_k += p_k
                # if modelImprove < min_model_improve:
                #     print "les"
                #     break
                if iterationCount >= max_iter_:
                    print "last print ",trust_box_size, merit
                    break

        print p_k
