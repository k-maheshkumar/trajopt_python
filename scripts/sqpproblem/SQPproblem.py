import copy
import cvxpy
import numpy as np
from scripts.utils import yaml_paser as yaml
import logging

'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbG <= G * x <= ubG
            # lb <= x <= ub
'''


class SQPProblem:
    def __init__(self, problem, solver, verbose=False):
        self.P = problem.P
        self.q = problem.q
        self.G = problem.G
        self.lb = problem.lb
        self.ub = problem.ub
        self.lbG = problem.lbG
        self.ubG = problem.ubG
        self.A = problem.A
        self.b = problem.b
        self.initial_guess = problem.initial_guess
        self.status = "-1"
        self.norm_ = 1

        file_path_prefix = '../../config/'
        sqp_config_file = file_path_prefix + 'sqp_config.yaml'

        sqp_yaml = yaml.ConfigParser(sqp_config_file)
        self.solver_config = sqp_yaml.get_by_key("sqp")

        if solver is not None:
            self.solver = solver
        else:
            self.solver = self.solver_config["solver"][1]

        self.logger = logging.getLogger("SQP Solver")
        ch = logging.StreamHandler()
        # create formatter
        formatter = logging.Formatter("%(asctime)s - %(name)s - %(levelname)s - %(message)s")
        # add formatter to ch
        ch.setFormatter(formatter)

        if verbose == "WARN":
            self.logger.setLevel(logging.WARN)
            ch.setLevel(logging.WARN)

        elif verbose == "INFO" or verbose is True:
            self.logger.setLevel(logging.INFO)
            ch.setLevel(logging.INFO)

        elif verbose == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
            ch.setLevel(logging.DEBUG)

        # add ch to logger
        self.logger.addHandler(ch)

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

    def is_constraints_satisfied(self, x_k):
        cons1_cond = np.less(np.matmul(self.G, x_k), self.ubG)
        cons2_cond = np.less(self.lbG, np.matmul(self.G, x_k))
        return cons1_cond.all() and cons2_cond.all()

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

        model += penalty * (cvxpy.norm(cons1_model, self.norm_) + cvxpy.norm(cons2_model, self.norm_))

        return model, objective_at_xk

    def get_actual_objective(self, x_k, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(x_k)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        objective += penalty * (
            cvxpy.norm(self.G * x - self.ubG.flatten(), self.norm_) + cvxpy.norm(-self.G * x + self.lbG.flatten(),
                                                                                 self.norm_))

        return objective

    def sovle_problem(self, x_k, penalizer, p, delta):
        model_objective, actual_objective = self.get_model_objective(x_k, penalizer, p)
        constraints = [cvxpy.norm(p, "inf") <= delta]
        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        result = problem.solve(solver=self.solver, warm_start=True, verbose=False)
        return p.value, model_objective, actual_objective, problem.status

    def get_constraints_norm(self, x_k):
        con1, con2 = self.evaluate_constraints(x_k)
        max_con1 = (np.linalg.norm(con1, np.inf))
        max_con2 = (np.linalg.norm(con2, np.inf))
        return max_con1, max_con2

    def solveSQP(self, initial_guess=None):
        self.logger.info("Starting SQP solver . . . . . . .")
        x = cvxpy.Variable(self.P.shape[0])
        p = cvxpy.Variable(x.shape[0])
        penalty = cvxpy.Parameter(nonneg=True)
        penalty.value = 1
        x_0 = np.full((1, self.P.shape[0]), 3.0).flatten()
        # if initial_guess is None:
        #     x_0 = self.initial_guess
        # else:
        #     x_0 = initial_guess
        p_0 = np.zeros(p.shape[0])
        trust_box_size = float(self.solver_config["trust_region_size"])
        max_penalty = float(self.solver_config["max_penalty"])
        min_trust_box_size = float(self.solver_config["min_trust_box_size"])
        x_k = copy.copy(x_0)
        max_trust_box_size = float(self.solver_config["max_trust_box_size"])

        trust_shrink_ratio = float(self.solver_config["trust_shrink_ratio"])
        trust_expand_ratio = float(self.solver_config["trust_expand_ratio"])

        trust_good_region_ratio = float(self.solver_config["trust_good_region_ratio"])

        max_iteration = float(self.solver_config["max_iteration"])
        iteration_count = 0

        min_model_improve = float(self.solver_config["min_model_improve"])
        improve_ratio_threshold = float(self.solver_config["improve_ratio_threshold"])
        min_approx_improve_frac = - float('inf')
        is_converged = False
        isAdjustPenalty = False

        old_rho_k = 0
        new_x_k = copy.copy(x_0)
        min_actual_redution = float(self.solver_config["min_actual_redution"])
        min_x_redution = float(self.solver_config["min_x_redution"])

        min_actual_worse_redution = float(self.solver_config["min_actual_worse_redution"])
        const_violation_tolerance = float(self.solver_config["const_violation_tolerance"])
        con1_norm, con2_norm = self.get_constraints_norm(x_k)
        same_trust_region_count = 0
        old_trust_region = copy.copy(trust_box_size)

        good_rho_k = 0.2
        is_improved = False
        p_k = 0

        while penalty.value <= max_penalty:
            # print "penalty ", penalty.value
            self.logger.debug("penalty " + str(penalty.value))
            while iteration_count <= max_iteration:
                iteration_count += 1
                # print("iteration count", iteration_count)
                self.logger.debug("iteration_count " + str(iteration_count))
                while trust_box_size >= min_trust_box_size:
                    p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = self.sovle_problem(x_k,
                                                                                                             penalty, p,
                                                                                                             trust_box_size)

                    actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, penalty)
                    model_objective_at_p_0 = self.get_actual_objective(p_0, penalty)

                    actual_reduction = actual_objective_at_x_plus_p_k.value - actual_objective_at_x_k.value
                    # predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value
                    predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                    rho_k = actual_reduction / predicted_reduction
                    x_k += p_k

                    self.logger.debug("x_k " + str(x_k))
                    self.logger.debug("rho_k " + str(rho_k))

                    if solver_status == cvxpy.INFEASIBLE or solver_status == cvxpy.INFEASIBLE_INACCURATE or solver_status == cvxpy.UNBOUNDED or solver_status == cvxpy.UNBOUNDED_INACCURATE:
                        self.logger.warn("Infeasible problem cannot be solved")
                        self.status = "Infeasible"
                        is_converged = True
                        break

                    if old_rho_k == rho_k:
                        # print "rho_k are same"
                        isAdjustPenalty = True
                        break

                    if predicted_reduction / model_objective_at_p_k.value < -float("inf"):
                        # print "need to adjust penalty"
                        isAdjustPenalty = True
                        break
                    # if rho_k > good_rho_k:
                        # x_k += p_k

                    if trust_box_size < min_trust_box_size:
                        isAdjustPenalty = True
                        trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                        break
                    if iteration_count >= max_iteration:
                        # print ("max iterations reached")
                        isAdjustPenalty = True
                        break

                    if old_trust_region == trust_box_size:
                        same_trust_region_count += 1
                    if same_trust_region_count >= 5:
                        # print "resetting trust region, since trust region size is same for last ",
                        #  same_trust_region_count, " iterations"
                        trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)


                    if rho_k <= 0.25:
                        trust_box_size *= trust_shrink_ratio
                        self.logger.debug("shrinking trust region " + str(trust_box_size))
                        break
                    else:
                        # if rho_k >= 0.75:
                        trust_box_size = min(trust_box_size * trust_expand_ratio, max_trust_box_size)
                        # print "expanding trust region", trust_box_size, rho_k
                        self.logger.debug("expanding trust region" + str(trust_box_size))
                        break
                trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                if abs(actual_reduction) <= min_actual_redution:
                    is_converged = True
                    inter_status = "actual reduction is very small"
                    self.logger.info(inter_status + ", so problem has probably converged")

                    self.status = "Solved"
                    break
                if abs((np.linalg.norm(new_x_k - x_k, np.inf))) <= min_x_redution:
                    is_converged = True
                    inter_status = "reduction in x is very small"
                    self.logger.info(inter_status + ", so problem has probably converged")
                    self.status = "Solved"
                if self.is_constraints_satisfied(x_k):
                    is_converged = True
                    self.logger.info(inter_status + " and constraints violations are satisfied")
                    self.status = "Solved"
                    break

                new_x_k = copy.copy(x_k)
            if is_converged:
                break
            penalty.value *= 10
            iteration_count = 0
        print "final result", self.status, x_k

        print("sqp problem")

        return self.status, x_k


