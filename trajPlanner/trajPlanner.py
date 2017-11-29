import numpy as np
from sqpproblem import SQPproblem as sqp
from warnings import warn
import cvxpy
import copy

'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbC <= C * x <= ubC
            # lb <= x <= ub
            # A * x == b

'''


class TrajectoryPlanner:

    def __init__(self, problem, solver):

        self.problem = problem
        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.max_no_of_Iteration = problem["max_iteration"]
        self.joints = problem["joints"]
        self.num_of_joints = len(problem["joints"])
        self.solver = solver
        self.max_penalty = problem["max_penalty"]
        self.deltaMax = problem["max_delta"]

        self.sqp = []
        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbG = []
        self.ubG = []
        self.b = []

        self.initial_guess = []

        for i in range(self.num_of_joints):

            sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.max_no_of_Iteration)

            self.sqp.append(sp)

            self.initial_guess.append(self.interpolate(sp.start, sp.end, self.samples))

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)

            self.A.append(self.sqp[i].A)
            self.b.append(self.sqp[i].b.tolist())

            self.G.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.samples)]))
            self.lbG.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
            self.ubG.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))

            self.lb.append(self.sqp[i].lb.tolist())
            self.ub.append(self.sqp[i].ub.tolist())

        self.initial_guess = np.hstack(self.initial_guess).flatten()
        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.ub = np.hstack(self.ub)
        self.lbG = np.hstack(self.lbG)
        self.ubG = np.hstack(self.ubG)

        self.P = self.diag_block_mat_slicing(self.P)
        self.A = self.diag_block_mat_slicing(self.A)
        self.G = self.diag_block_mat_slicing(self.G)


        # self.lb = [item for sublist in self.lb for item in sublist]
        self.b = np.hstack(self.b)
        # self.q = [item for sublist in self.q for item in sublist]

        # self.lb = np.asarray(self.lb)
        # self.ub = np.asarray(self.ub)
        # self.lbC = np.asarray(self.lbC)
        # self.ubC = np.asarray(self.ubC)
        # self.b = np.asarray(self.b)

        # self.q = np.asarray(self.q)

        self.G = self.G.astype(float)

        self.P = 2.0 * self.P + 1e-08 * np.eye(self.P.shape[1])




    def diag_block_mat_slicing(self, L):
        shp = L[0].shape
        N = len(L)
        r = range(N)
        out = np.zeros((N, shp[0], N, shp[1]), dtype=int)
        out[r, :, r, :] = L
        return out.reshape(np.asarray(shp) * N)


    def displayProblem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.G)
        print ("lb")
        print (self.lb)
        print ("ub")
        print (self.ub)
        print ("lbG")
        print (self.lbG)
        print ("ubG")
        print (self.ubG)
        print ("b")
        print (self.b)
        print ("A")
        print (self.A)

        print ("maxNoOfIteration")
        print (self.max_no_of_Iteration)


    def interpolate(self, start, end, samples):
        data = []
        stepSize = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += stepSize
        return np.round(data, 3)

    def evaluate_constraints(self, x_k):
        cons1 = np.subtract(np.matmul(self.G, x_k), self.ubG)
        cons2 = np.add(np.matmul(-self.G, x_k), self.lbG)
        return cons1.flatten(), cons2.flatten()

    def get_constraints_gradients(self):
        cons1_grad = self.G
        cons2_grad = -self.G
        return cons1_grad, cons2_grad

    def get_objective_gradient_and_hessian(self, x_k):
        model_grad = 0.5 * np.matmul((self.P + self.P.T), x_k)
        model_hess = 0.5 * (self.P + self.P.T)
        return model_grad, model_hess

    def get_model_objective(self, x_k, penalty, p):
        cons1_at_xk, cons2_at_xk = self.evaluate_constraints(x_k)
        cons1_grad_at_xk, cons2_grad_at_xk = self.get_constraints_gradients()

        cons1_model = cons1_at_xk + cons1_grad_at_xk * p
        cons2_model = cons2_at_xk + cons2_grad_at_xk * p

        objective_grad_at_xk, objective_hess_at_xk = self.get_objective_gradient_and_hessian(x_k)
        objective_at_xk = self.get_actual_objective(x_k, penalty)
        model = objective_at_xk.value + objective_grad_at_xk * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_xk)

        model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model))

        return model, objective_at_xk

    def get_actual_objective(self, x_k, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(x_k)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        objective += penalty * (
        cvxpy.norm1(self.G * x - self.ubG.flatten()) + cvxpy.norm1(-self.G * x + self.lbG.flatten()))

        return objective

    def sovle_problem(self, x_k, penalizer, p, delta):
        model_objective, actual_objective = self.get_model_objective(x_k, penalizer, p)
        constraints = [cvxpy.norm(p, "inf") <= delta]
        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        result = problem.solve(solver=cvxpy.SCS, warm_start=True, verbose=False)
        return p.value, model_objective, actual_objective, problem.status

    def get_constraints_norm(self, x_k):
        con1, con2 = self.evaluate_constraints(x_k)
        max_con1 = (np.linalg.norm(con1, np.inf))
        max_con2 = (np.linalg.norm(con2, np.inf))
        return max_con1, max_con2

    def solveSQP(self):
        x = cvxpy.Variable(self.P.shape[0])
        p = cvxpy.Variable(x.shape[0])
        penalty = cvxpy.Parameter(nonneg=True)
        penalty.value = 1
        # x_0 = np.array([10.2, 10.7, 10.1])
        # print self.initialX
        # x_0 = np.array([2, 2, 2.0, 2, 2, 2.0, 2, 2, 2.0, 2, 2, 2.0])
        # x_0 = np.full((1, self.P.shape[0]), 1.0).flatten()
        x_0 = self.initial_guess
        # x_0 = self.initialX
        p_0 = np.zeros(p.shape[0])
        trust_box_size = 1
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
        min_x_redution = 1e-3

        min_actual_worse_redution = -100
        min_const_violation = 2.4
        con1_norm, con2_norm = self.get_constraints_norm(x_k)
        same_trust_region_count = 0
        old_trust_region = 0
        while con1_norm + con2_norm >= 2 or penalty.value <= max_penalty:
            # print "penalty ", penalty.value
            while iteration_count < max_iteration:
                iteration_count += 1
                # print "iteration_count", iteration_count
                while trust_box_size >= min_trust_box_size:
                    p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = self.sovle_problem(x_k, penalty, p,
                                                                                                             trust_box_size)

                    actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, penalty)
                    model_objective_at_p_0 = self.get_actual_objective(p_0, penalty)

                    actual_reduction = actual_objective_at_x_plus_p_k.value - actual_objective_at_x_k.value
                    predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                    rho_k = actual_reduction / predicted_reduction
                    con1_norm, con2_norm = self.get_constraints_norm(x_k)

                    if solver_status == cvxpy.INFEASIBLE or solver_status == cvxpy.INFEASIBLE_INACCURATE or solver_status == cvxpy.UNBOUNDED or solver_status == cvxpy.UNBOUNDED_INACCURATE:
                        # print problem.status
                        # Todo: throw error when problem is not solved
                        break

                    if old_rho_k == rho_k:
                        # print "rho_k are same"
                        isAdjustPenalty = True
                        break

                    if abs(actual_reduction) <= min_actual_redution:
                        if con1_norm + con2_norm >= min_const_violation:
                            print ("infeasible intial guess and actual reduction is very small")
                            is_converged = True  # to force loop exit
                            break
                        print ("actual reduction is very small, so converged to optimal solution")
                        x_k += p_k
                        is_converged = True
                        break

                    if con1_norm + con2_norm <= min_const_violation:
                        print ("constraint violations are satisfied, so converged to optimal solution")
                        x_k += p_k
                        is_converged = True
                        break

                    if abs((np.linalg.norm(x_k - (x_k + p_k), np.inf))) <= min_x_redution:
                        if con1_norm + con2_norm >= min_const_violation:
                            print ("infeasible intial guess and improvement in x is very small")
                            is_converged = True  # to force loop exit
                            break
                        print ("improvement in x is very small, so converged to optimal solution")
                        x_k += p_k
                        is_converged = True
                        break


                    if actual_reduction <= min_actual_worse_redution:
                        print ("infeasible intial guess, because actual reduction",  actual_reduction," is worser than ", min_actual_worse_redution)
                        is_converged = True  # to force loop exit
                        break
                    if predicted_reduction / model_objective_at_p_k.value < -float("inf"):
                        # print "need to adjust penalty"
                        isAdjustPenalty = True
                        break
                    if rho_k <= 0.25:
                        trust_box_size *= trust_shrink_ratio
                        # print "shrinking trust region", trust_box_size
                        # x_k = copy.copy(new_x_k)
                        break
                    else:
                        # elif rho_k >= 0.75:
                        trust_box_size = min(trust_box_size * trust_expand_ratio, max_trust_box_size)
                        # print "expanding trust region", trust_box_size
                        x_k += p_k
                        break
                        new_x_k = copy.copy(x_k)

                    if trust_box_size < 0.01:
                        isAdjustPenalty = True
                        break
                    if iteration_count >= max_iteration:
                        print ("max iterations reached")
                        break

                    if old_trust_region == trust_box_size:
                        same_trust_region_count += 1
                    if same_trust_region_count >= 5:
                        # print "resetting trust region, since trust region size is same for last ", same_trust_region_count, " iterations"
                        trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                    old_rho_k = rho_k
                    old_trust_region = copy.copy(trust_box_size)
                if is_converged:
                    break
                trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                # if con1_norm + con2_norm <= min_const_violation:
                #     print "constraint violations are satisfied, so converged to optimal solution"
                #     x_k += p_k
                #     is_converged = True
                #     break
            if is_converged or isAdjustPenalty:
                break
            penalty.value *= 10
            iteration_count = 0
        print ("initial x_0", x_0)
        # print "final x_k", x_k, trust_box_size, penalty.value
        print ("final x: ", (np.split(x_k, self.num_of_joints)))
