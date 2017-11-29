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
    def diag_block_mat_slicing(self, L):
        shp = L[0].shape
        N = len(L)
        r = range(N)
        out = np.zeros((N, shp[0], N, shp[1]), dtype=int)
        out[r, :, r, :] = L
        return out.reshape(np.asarray(shp) * N)

    def __init__(self, problem, solver):
        self.problem = problem

        # self.check_input()

        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.maxNoOfIteration = problem["maxIteration"]
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])
        self.solver = solver
        self.penaltyMax = problem["penaltyMax"]
        self.deltaMax = problem["deltaMax"]

        self.sqp = []

        self.P = []
        self.q = []
        self.G = []
        self.A = []
        self.lb = []
        self.ub = []
        self.lbA = []
        self.ubA = []
        self.lbC = []
        self.ubC = []
        self.b = []
        self.C = []
        self.initialX = []

        if 'initialGuess' in self.joints[0]:
            self.initialGuess = []
            self.isInitialGuessAvailable = True
        else:
            self.isInitialGuessAvailable = False

        for i in range(self.numJoints):

            # sp.display()
            if self.joints[i].has_key("initialGuess"):
                sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration,
                                    self.joints[i]["initialGuess"])

                self.initialGuess.append(sp.initVals)
                self.initialX.append(self.interpolate(sp.start, sp.end, self.samples))
            else:
                sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration)

            self.sqp.append(sp)

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)
            # self.G.append(self.sqp[i].G)
            # self.A.append(self.sqp[i].A)

            if solver == "qpoases":
                self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A]))
                # self.C.append(self.sqp[i].G)
                self.A.append(self.sqp[i].A.tolist())

                # print np.array([self.sqp[i].lbG[0][0]])
                # self.lb.append(np.hstack([self.sqp[i].lbG]))
                self.lb.append(self.sqp[i].lb)

                # self.lb = np.append(self.lb, self.sqp[i].lbG[0][0])
                # self.ub.append(self.sqp[i].ub.tolist())
                self.ub.append(np.hstack([self.sqp[i].ub]))
                # self.ub = np.dstack([self.ub, self.sqp[i].ub[0][0]])
            else:
                # self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.samples)]))
                self.C.append(np.vstack([self.sqp[i].G, np.identity(self.samples)]))

                self.A.append(np.vstack([self.sqp[i].A, self.sqp[i].A, self.sqp[i].A]))

                # self.A.append(self.sqp[i].A)
                # self.lbA.append(self.sqp[i].b)
                self.b.append(np.hstack([self.sqp[i].b, self.sqp[i].b, self.sqp[i].b]))

                # self.lb.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
                self.lb.append(np.hstack([self.sqp[i].lbG, self.sqp[i].lb]))

                # self.ub.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))
                self.ub.append(np.hstack([self.sqp[i].ubG, self.sqp[i].ub]))


                # self.lbC.append(self.sqp[i].lbG.tolist())
                # self.ubC.append(self.sqp[i].ubG.tolist())
                # self.b.append(self.sqp[i].b.tolist())
                # sp.display()
                # print (self.lb)

        # print self.C
        # print self.A

        # print self.G
        # self.q = self.q[0]

        self.initialX = np.hstack(self.initialX)

        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.b = np.hstack(self.b).flatten()

        self.ub = np.hstack(self.ub)
        # print  self.initialGuess
        # print  len(self.initialGuess)
        self.P = self.diag_block_mat_slicing(self.P)
        # self.q = self.diag_block_mat_slicing(self.q)

        self.A = self.diag_block_mat_slicing(self.A)
        if solver == "qpoases":
            self.A = self.diag_block_mat_slicing(self.A)
        # self.G = self.diag_block_mat_slicing(self.G)
        self.C = self.diag_block_mat_slicing(self.C)
        # print self.C


        # self.lb = [item for sublist in self.lb for item in sublist]
        # self.ub = [item for sublist in self.ub for item in sublist]
        # self.lbC = [item for sublist in self.lbC for item in sublist]
        # self.ubC = [item for sublist in self.ubC for item in sublist]
        # self.b = [item for sublist in self.b for item in sublist]

        # self.q = [item for sublist in self.q for item in sublist]

        self.lb = np.asarray(self.lb)
        self.ub = np.asarray(self.ub)
        # self.lbC = np.asarray(self.lbC)
        # self.ubC = np.asarray(self.ubC)
        # self.b = np.asarray(self.b)

        self.q = np.asarray(self.q)
        # if self.joints[i].has_key("initialGuess"):
        if 'initialGuess' in self.joints[i]:
            self.initialGuess = np.hstack(self.initialGuess)
            self.initialGuess = np.asarray(self.initialGuess)

        else:
            self.initialGuess = None
        # print "q.shape", self.q.shape
        # self.H = self.H.astype(float)
        # self.q = self.q.astype(float)
        # self.G = self.G.astype(float)


        self.C = self.C.astype(float)

        self.P = 2.0 * self.P + 1e-08 * np.eye(self.P.shape[1])


        # self.A = 1.0 * self.A
        # self.G = 1.0 * self.G
        # self.G = self.G.flatten()


        # print self.initialGuess

        # example, num = sp.solveQp()
        # # print num
        # initialGuess = np.zeros(num)
        # example.getPrimalSolution(initialGuess)

        # print "solution"
        # print initialGuess, example.getObjVal()

        # self.displayProblem()


    def displayProblem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.C)
        print ("lbA")
        print (self.lbA)
        print ("ubA")
        print (self.ubA)
        print ("lbG")
        print (self.lb)
        print ("ubG")
        print (self.ub)
        print ("b")
        print (self.b)
        print ("A")
        print (self.A)

        print ("maxNoOfIteration")
        print (self.maxNoOfIteration)



    def interpolate(self, start, end, samples):
        guess = []
        stepSize = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            guess.append(intermediate)
            intermediate += stepSize
        return np.round(guess, 3)

    def evaluate_constraints(self, x_k):
        cons1 = np.subtract(np.matmul(self.C, x_k), self.ub).flatten()
        cons2 = np.add(np.matmul(-self.C, x_k), self.lb).flatten()
        cons3 = np.subtract(np.matmul(self.A, x_k), self.b).flatten()
        return cons1, cons2, cons3

    def get_constraints_grad(self):
        cons1_grad = self.C
        cons2_grad = -self.C
        cons3_grad = self.A
        return cons1_grad, cons2_grad, cons3_grad

    def get_objective_grad_and_hess(self, g):
        model_grad = 0.5 * np.matmul((self.P + self.P.T), g)
        model_hess = 0.5 * (self.P + self.P.T)
        return model_grad, model_hess

    def get_model_objective(self, xk, penalty, p):
        cons1_at_xk, cons2_at_xk, cons3_at_xk = self.evaluate_constraints(xk)
        cons1_grad_at_xk, cons2_grad_at_xk, cons3_grad_at_xk = self.get_constraints_grad()

        cons1_model = cons1_at_xk + cons1_grad_at_xk * p
        cons2_model = cons2_at_xk + cons2_grad_at_xk * p
        cons3_model = cons3_at_xk + cons3_grad_at_xk * p

        # print "A", self.A
        # cons3_model = np.subtract(np.matmul(self.A, xk), self.lbA).flatten()
        # cons3_model += self.A * p
        # print "xk", xk, self.lbA
        # print cons3_model
        objective_grad_at_xk, objective_hess_at_xk = self.get_objective_grad_and_hess(xk)
        objective_at_xk = self.get_actual_objective(xk, penalty)
        model = objective_at_xk.value + objective_grad_at_xk * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_xk)

        model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model) + cvxpy.norm1(cons3_model))

        return model, objective_at_xk

    def get_actual_objective(self, xk, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(xk)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        objective += penalty * (
        cvxpy.norm1(self.C * x - self.ub.flatten()) + cvxpy.norm1(-self.C * x + self.lb.flatten())
        + cvxpy.norm1(self.A * x - self.b))

        return objective

    def sovle_problem(self, xk, penalizer, p, delta):
        model_objective, actual_objective = self.get_model_objective(xk, penalizer, p)
        constraints = [cvxpy.norm(p, "inf") <= delta]
        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        result = problem.solve(solver=cvxpy.SCS, warm_start=True, verbose=False)
        return p.value, model_objective, actual_objective, problem.status

    def get_constraints_norm(self, x_k):
        con1, con2, con3 = self.evaluate_constraints(x_k)
        # print con3
        max_con1 = (np.linalg.norm(con1, np.inf))
        max_con2 = (np.linalg.norm(con2, np.inf))
        max_con3 = (np.linalg.norm(con3, np.inf))

        return max_con1, max_con2, max_con3

    def solveSQP(self):
        x = cvxpy.Variable(self.P.shape[0])
        p = cvxpy.Variable(x.shape[0])
        penalty = cvxpy.Parameter(nonneg=True)
        penalty.value = 1
        # x_0 = np.array([10.2, 10.7, 10.1])
        # print self.initialX
        # x_0 = np.array([2, 2, 2.0, 2, 2, 2.0, 2, 2, 2.0, 2, 2, 2.0])
        # x_0 = np.full((1, self.P.shape[0]), 2.0).flatten()
        # print x_0
        # x_0 = self.initialGuess
        x_0 = (self.initialX + np.full((1, self.P.shape[0]), 0.2)).flatten()
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
        con1_norm, con2_norm, con3_norm = self.get_constraints_norm(x_k)
        same_trust_region_count = 0
        old_trust_region = 0
        while con1_norm + con2_norm >= 2 or penalty.value <= max_penalty:
            # print "penalty ", penalty.value
            while iteration_count < max_iteration:
                iteration_count += 1
                # print "iteration_count", iteration_count
                while trust_box_size >= min_trust_box_size:
                    p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = self.sovle_problem(x_k,
                                                                                                             penalty, p,
                                                                                                             trust_box_size)
                    # print "iteration_count in trust loop", iteration_count


                    # print "pk ", p_k, solver_status

                    actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, penalty)
                    model_objective_at_p_0 = self.get_actual_objective(p_0, penalty)

                    # print "objective_at_x_plus_p_k", actual_objective_at_x_plus_p_k.value
                    # print "model_objective_at_x_plus_p_k", model_objective_at_p_0.value
                    # print "actual xk, xk1", actual_objective_at_x_plus_p_k.value, actual_objective_at_x_k.value
                    # print "model p0, pk",model_objective_at_p_0.value, model_objective_at_p_k.value

                    actual_reduction = actual_objective_at_x_plus_p_k.value - actual_objective_at_x_k.value
                    predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                    rho_k = actual_reduction / predicted_reduction

                    # print "rho_k", rho_k, abs(actual_reduction)
                    print"x_k  ", x_k
                    print "x_k + pk ", x_k + p_k, trust_box_size, penalty.value, abs(actual_reduction)
                    con1_norm, con2_norm, con3_norm = self.get_constraints_norm(x_k)
                    print "con3_norm", con3_norm
                    # max_con1 = (np.linalg.norm(con1, np.inf))
                    # max_con2 = (np.linalg.norm(con2, np.inf))
                    # print "max_con1, max_con2", con1_norm, con2_norm, actual_reduction
                    max_p_k = (np.linalg.norm(p_k, np.inf))
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
                            print "infeasible intial guess"
                            is_converged = True  # to force loop exit
                            break
                        print "actual reduction is very small, so converged to optimal solution"
                        x_k += p_k
                        is_converged = True
                        break

                    # if con1_norm + con2_norm <= min_const_violation:
                    #     print "constraint violations are satisfied, so converged to optimal solution"
                    #     x_k += p_k
                    #     is_converged = True
                    #     break

                    if abs((np.linalg.norm(x_k - (x_k + p_k), np.inf))) <= min_x_redution:
                        if con1_norm + con2_norm >= min_const_violation:
                            print "infeasible intial guess"
                            is_converged = True  # to force loop exit
                            break
                        print "improvement in x is very small, so converged to optimal solution"
                        x_k += p_k
                        is_converged = True
                        break


                    if actual_reduction <= min_actual_worse_redution:
                        print "infeasible intial guess"
                        is_converged = True  # to force loop exit
                        break
                    if predicted_reduction / model_objective_at_p_k.value < -float("inf"):
                        # print "need to adjust penalty"
                        isAdjustPenalty = True
                        break
                    if rho_k <= 0.25:
                        trust_box_size *= trust_shrink_ratio
                        print "shrinking trust region", trust_box_size
                        x_k = copy.copy(new_x_k)
                        break
                    else:
                        # elif rho_k >= 0.75:
                        trust_box_size = min(trust_box_size * trust_expand_ratio, max_trust_box_size)
                        print "expanding trust region", trust_box_size
                        print "test . .. . .", x_k
                        x_k += p_k
                        print "test after. .. . .", x_k
                        new_x_k = copy.copy(x_k)
                        break

                    if trust_box_size < 0.01:
                        isAdjustPenalty = True
                        break
                    if iteration_count >= max_iteration:
                        print "max iterations reached"
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
        print "initial x_0", x_0
        print "final x_k", x_k, trust_box_size, penalty.value
