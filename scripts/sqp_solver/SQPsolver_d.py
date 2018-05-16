import copy
import cvxpy
import numpy as np
from scripts.utils import yaml_paser as yaml
from scripts.utils.utils import Utils as utils
import logging
import os
import time
from collections import OrderedDict


'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbC <= G * x <= ubC
            A * x == b
            lb <= x <= ub

'''


class SQPsolver:
    def __init__(self, main_logger_name=__name__, verbose=False, log_file=False):
        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbG = []
        self.ubG = []
        self.b = []
        self.D = None
        self.lbD = None
        self.ubD = None

        self.initial_guess = []
        self.status = "-1"
        self.norm_ = 1

        self.solver_config = OrderedDict()

        self.solver = []

        self.penalty_norm = 1
        self.trust_region_norm = np.inf

        self.num_qp_iterations = 0
        self.num_sqp_iterations = 0
        self.solving_time = 0
        self.initial_cost = 0
        self.final_cost = 0
        self.initial_cost1 = 0
        self.final_cost1 = 0
        self.initial_cost2 = 0
        self.final_cost2 = 0
        self.initial_cost3 = 0
        self.final_cost3 = 0

        self.initial_costs = []
        self.final_costs = []
        self.initial_costs1 = []
        self.final_costs1 = []
        self.initial_costs2 = []
        self.final_costs2 = []
        self.initial_costs3 = []
        self.final_costs3 = []

        self.is_initialised = False

        self.logger = logging.getLogger(main_logger_name + __name__)
        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)

    def init(self, **kwargs):

        self.D = None
        self.lbD = None
        self.ubD = None
        if "P" in kwargs:
            self.P = kwargs["P"]
        if "q" in kwargs:
            self.q = kwargs["q"]
        if "G" in kwargs:
            self.G = kwargs["G"]
        if "lbG" in kwargs:
            self.lbG = kwargs["lbG"]
        else:
            self.lbG = None
        if "ubG" in kwargs:
            self.ubG = kwargs["ubG"]
        else:
            self.ubG = None
        if "A" in kwargs:
            self.A = kwargs["A"]
        else:
            self.A = None
        if "b" in kwargs:
            self.b = kwargs["b"]
        else:
            self.A = None
        if "initial_guess" in kwargs:
            self.initial_guess = kwargs["initial_guess"]
        else:
            self.initial_guess = np.zeros((self.P.shape[0], 1)).flatten()

        if "solver_config" in kwargs:
            solver_config = kwargs["solver_config"]
        else:
            solver_config = None
        if "solver" in kwargs:
            solver = kwargs["solver"]
        else:
            solver = None

        if solver_config is not None:
            self.solver_config = solver_config
        else:
            file_path_prefix = os.path.join(os.path.dirname(__file__), '../../config/')

            sqp_config_file = file_path_prefix + 'sqp_config.yaml'

            sqp_yaml = yaml.ConfigParser(sqp_config_file)
            self.solver_config = sqp_yaml.get_by_key("sqp")

        self.penalty_norm = self.solver_config["penalty_norm"]

        self.trust_region_norm = self.solver_config["trust_region_norm"]

        self.analyse_inputs()

        if solver is not None:
            self.solver = solver
        else:
            self.solver = self.solver_config["solver"][0]

    def display_problem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.G)
        print ("lbG")
        print (self.lbG)
        print ("ubG")
        print (self.ubG)
        print ("A")
        print (self.A)
        print ("b")
        print (self.b)
        print ("lb")
        print (self.lb)
        print ("ub")
        print (self.ub)
        print ("initial guess")
        print (self.initial_guess)

    def update_prob(self, G=None, lbG=None, ubG=None, A=None, b=None):
        if G is not None:
            self.G = G
        if lbG is not None:
            self.lbG = lbG
        if ubG is not None:
            self.ubG = ubG
        if A is not None:
            self.A = A
        if b is not None:
            self.b = b
        self.analyse_inputs()

    def analyse_inputs(self):
        if self.lbG is not None:
            self.lbG = np.array([utils.replace_none(lb, float(self.solver_config["replace_none_with"]), negate=True)
                                 for lb in self.lbG])
        if self.ubG is not None:
            self.ubG = np.array([utils.replace_none(ub, float(self.solver_config["replace_none_with"]))
                                 for ub in self.ubG])

        if self.G is not None and self.lbG is None and self.ubG is not None:
            self.lbG = -self.ubG
        if self.G is not None and self.ubG is None and self.lbG is not None:
            self.G = np.vstack([-self.G, -self.G])
            self.lbG = np.hstack([-self.ubG, -self.ubG])
            self.ubG = np.hstack([self.ubG, -self.ubG])

    def is_x_converged(self, x_k, p_k, tolerance=1e-3):
        return abs((np.linalg.norm(x_k - (x_k + p_k), np.inf))) <= tolerance

    def is_objective_function_converged(self, objective, tolerance=1e-3):

        return abs(objective) <= tolerance

    def is_constraints_satisfied(self, x_k, p, tolerance=1e-3):
        cons1_cond = np.isclose(np.matmul(self.G, x_k) <= self.ubG, 1, rtol=tolerance, atol=tolerance)
        cons2_cond = np.isclose(np.matmul(self.G, x_k) >= self.lbG, 1, rtol=tolerance, atol=tolerance)
        cons3_cond = np.isclose(np.matmul(self.A, x_k), self.b, rtol=tolerance, atol=tolerance)
        cons4_cond = True
        if self.D is not None:
            p_k = np.hstack([x_k] * (self.D.shape[1] / p.shape[0]))
            cons4_cond = np.isclose(np.matmul(self.D, p_k) >= self.lbD, 1, rtol=tolerance, atol=tolerance).all()

        return cons1_cond.all() and cons2_cond.all() and cons3_cond.all() and cons4_cond

    def evaluate_constraints(self, x_k, p):
        cons1 = np.subtract(np.matmul(self.G, x_k), self.ubG)
        cons2 = np.add(np.matmul(-self.G, x_k), self.lbG)
        cons3 = np.subtract(np.matmul(self.A, x_k), self.b)
        cons4 = 0
        if self.D is not None:
            p_k = np.hstack([x_k] * (self.D.shape[1] / p.shape[0]))
            cons4 = self.lbD - cvxpy.matmul(self.D, p_k)

        return cons1.flatten(), cons2.flatten(), cons3.flatten(), cons4

    def get_constraints_gradients(self):
        cons1_grad = self.G
        cons2_grad = -self.G
        cons3_grad = self.A
        cons4_grad = 0
        if self.D is not None:
            cons4_grad = -self.D
        return cons1_grad, cons2_grad, cons3_grad, cons4_grad

    def get_objective_gradient_and_hessian(self, x_k):
        model_grad = 0.5 * np.matmul((self.P + self.P.T), x_k)
        model_hess = 0.5 * (self.P + self.P.T)
        return model_grad, model_hess

    def get_model_objective(self, x_k, p, penalty):
        cons1_at_xk, cons2_at_xk, cons3_at_xk, cons4_at_xk = self.evaluate_constraints(x_k, p)
        cons1_grad_at_xk, cons2_grad_at_xk, cons3_grad_at_xk, cons4_grad_at_xk = self.get_constraints_gradients()
        cons1_model = cons1_at_xk + cons1_grad_at_xk * p
        cons2_model = cons2_at_xk + cons2_grad_at_xk * p
        cons3_model = cons3_at_xk + cons3_grad_at_xk * p

        cons4_model = 0
        if self.D is not None:
            p1 = cvxpy.hstack([p] * (self.D.shape[1] / p.shape[0]))
            cons4_model = cons4_at_xk + cvxpy.matmul(cons4_grad_at_xk, p1)

        objective_grad_at_xk, objective_hess_at_xk = self.get_objective_gradient_and_hessian(x_k)
        objective_at_xk = self.get_actual_objective(x_k, p, penalty)
        model = objective_at_xk.value + objective_grad_at_xk * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_xk)

        model += penalty * (cvxpy.norm(cons1_model, self.penalty_norm) + cvxpy.norm(cons2_model, self.penalty_norm)
                            + cvxpy.norm(cons3_model, self.penalty_norm)
                            + cvxpy.norm(cons4_model, self.penalty_norm))

        return model, objective_at_xk

    def get_actual_objective(self, xk, p, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(xk)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        constraints1 = cvxpy.norm(self.G * x - self.ubG.flatten(), self.penalty_norm)
        constraints2 = cvxpy.norm(-self.G * x + self.lbG.flatten(), self.penalty_norm)
        constraints3 = cvxpy.norm(self.A * x - self.b.flatten(), self.penalty_norm)

        constraints4 = 0
        if self.D is not None:
            p1 = np.hstack([xk] * (self.D.shape[1] / p.shape[0]))
            constraints4 = cvxpy.norm(self.lbD - cvxpy.matmul(self.D, p1), self.penalty_norm)

        objective += penalty * (constraints1 + constraints2 + constraints3 + constraints4)
        return objective

    def solve_problem(self, x_k, penalizer, p, delta, constraints=None, lower_limit=None, upper_limit=None):
        model_objective, actual_objective = self.get_model_objective(x_k, p, penalizer)
        if self.D is not None:
            print self.D.shape, p.shape, delta, penalizer.value
        constraints = [cvxpy.norm(p, self.trust_region_norm) <= delta]

        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        # print problem.get_problem_data(self.solver)[0]
        if self.solver == "CVXOPT":
            start = time.time()
            result = problem.solve(solver=self.solver, warm_start=True, kktsolver=cvxpy.ROBUST_KKTSOLVER, verbose=False)
            end = time.time()
        else:
            start = time.time()
            result = problem.solve(solver=self.solver, warm_start=True, verbose=False, max_iters=100)
            end = time.time()
        self.solving_time += end - start

        return p.value, model_objective, actual_objective, problem.status, problem.value

    def approx_equal(self, x, y, tolerance=0.001):
        return abs(x - y) <= 0.5 * tolerance * (x + y)

    def get_constraints_norm(self, x_k):
        con1, con2, con3 = self.evaluate_constraints(x_k)
        max_con1 = (np.linalg.norm(con1, np.inf))
        max_con2 = (np.linalg.norm(con2, np.inf))
        max_con3 = (np.linalg.norm(con3, np.inf))

        return max_con1, max_con2, max_con3

    def test_prob(self, p, problem, constraints, penalty, delta, solver, trust_region_norm, penalty_norm):
        from scripts.cvxpy_optimizer.solver_cvxpy2 import ConvexOptimizer
        opt = ConvexOptimizer(p, problem, constraints, penalty, delta, solver, trust_region_norm, penalty_norm)

    def solve(self, initial_guess=None, callback_function=None):
        self.logger.info("Starting SQP solver . . . . . . .")
        x = cvxpy.Variable(self.P.shape[0])
        p = cvxpy.Variable(x.shape[0])
        penalty = cvxpy.Parameter(nonneg=True)

        if initial_guess is None:
            x_0 = self.initial_guess
        else:
            x_0 = initial_guess

        p_0 = cvxpy.Variable(x.shape[0])
        p_0.value = np.zeros(p.shape[0])

        penalty.value = float(self.solver_config["initial_penalty"])
        trust_box_size = float(self.solver_config["trust_region_size"])
        max_penalty = float(self.solver_config["max_penalty"])
        min_trust_box_size = float(self.solver_config["min_trust_region_size"])
        max_trust_box_size = float(self.solver_config["max_trust_region_size"])

        trust_shrink_ratio = float(self.solver_config["trust_shrink_ratio"])
        trust_expand_ratio = float(self.solver_config["trust_expand_ratio"])

        trust_good_region_ratio = float(self.solver_config["trust_good_region_ratio"])
        trust_bad_region_ratio = float(self.solver_config["trust_bad_region_ratio"])
        max_iteration = float(self.solver_config["max_iteration"])


        min_actual_redution = float(self.solver_config["min_actual_redution"])
        min_x_redution = float(self.solver_config["min_x_redution"])

        const_violation_tolerance = float(self.solver_config["const_violation_tolerance"])

        x_k = copy.copy(x_0)

        iteration_count = 0
        check_for_constraints = False

        is_adjust_penalty = False

        same_trust_region_count = 0
        old_trust_region = copy.copy(trust_box_size)

        actual_reduction = 1000

        rho_k = 0
        old_rho_k = 0
        inter_status = "-1"
        p_k = [0] * len(x_0)
        last_p_k = [0] * len(x_0)

        dynamic_constraints_satisfied = False

        p.value = copy.deepcopy(p_0.value)


        while penalty.value <= max_penalty:
            # print "penalty ", penalty.value
            self.logger.debug("penalty " + str(penalty.value))
            self.num_qp_iterations += 1
            self.num_sqp_iterations += 1
            while iteration_count < max_iteration:
                # self.is_initialised = False
                iteration_count += 1
                self.num_qp_iterations += 1
                # print "iteration_count", iteration_count
                self.logger.debug("iteration_count " + str(iteration_count))
                if callback_function is not None:
                    # constraints, lower_limit, upper_limit = callback_function(x_k, p_k)
                    self.D, self.lbD, self.ubD = callback_function(x_k, p_k)
                while trust_box_size >= min_trust_box_size:
                    self.num_qp_iterations += 1
                    if callback_function is not None:
                        if self.D is not None:
                            p_k, model_objective_at_p_k, \
                            actual_objective_at_x_k, solver_status, prob_value = self.solve_problem(x_k, penalty, p, trust_box_size,
                                                                                        self.D, self.lbD, self.ubD)

                        else:
                            dynamic_constraints_satisfied = True
                            inter_status = "dynamic constrained satisfied "
                            self.logger.info(inter_status)
                            self.status = "Solved"
                            break
                    else:
                        p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status, prob_value = self.solve_problem(x_k,
                                                                                                                 penalty,
                                                                                                                 p,
                                                                                                                 trust_box_size)

                    if p_k is None:
                        x_k -= last_p_k
                        p.value = last_p_k
                        p_k = copy.deepcopy(last_p_k)
                    if p_k is not None:
                        actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, p, penalty)
                        model_objective_at_p_0 = self.get_model_objective(x_k, p_0, penalty)[0]

                        actual_reduction = actual_objective_at_x_k.value - actual_objective_at_x_plus_p_k.value
                        predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                        if predicted_reduction == 0:
                            predicted_reduction = 0.0000001
                        rho_k = actual_reduction / predicted_reduction
                        self.initial_costs.append(predicted_reduction)
                        self.initial_costs1.append(actual_objective_at_x_k.value)
                        self.initial_costs2.append(actual_reduction)
                        self.initial_costs3.append(prob_value)

                        if not self.is_initialised:
                            self.is_initialised = True
                            self.initial_cost = predicted_reduction
                            self.initial_cost1 = actual_objective_at_x_k.value
                            self.initial_cost2 = actual_reduction
                            self.initial_cost3 = prob_value


                        else:
                            self.final_cost = predicted_reduction
                            self.final_cost1 = actual_objective_at_x_k.value
                            self.final_cost2 = actual_reduction
                            self.final_cost3 = prob_value

                            self.final_costs.append(predicted_reduction)
                            self.final_costs1.append(actual_objective_at_x_k.value)
                            self.final_costs2.append(actual_reduction)
                            self.final_costs3.append(prob_value)

                        self.logger.debug("\n x_k " + str(x_k))
                        self.logger.debug("rho_k " + str(rho_k))

                        if solver_status == cvxpy.INFEASIBLE or solver_status == cvxpy.INFEASIBLE_INACCURATE or solver_status == cvxpy.UNBOUNDED or solver_status == cvxpy.UNBOUNDED_INACCURATE:
                            penalty.value *= trust_expand_ratio
                            break

                        if rho_k >= trust_good_region_ratio:
                            trust_box_size = np.fmin(trust_box_size * trust_expand_ratio, max_trust_box_size)
                            self.logger.debug("expanding trust region" + str(trust_box_size))
                            x_k += p_k
                            break
                        elif rho_k <= trust_bad_region_ratio:
                            trust_box_size *= trust_shrink_ratio
                            self.logger.debug("shrinking trust region " + str(trust_box_size))
                            x_k -= p_k

                        if trust_box_size < min_x_redution:
                            check_for_constraints = True
                            break

                        last_p_k = p_k

                trust_box_size = np.fmax(trust_box_size, min_trust_box_size / (trust_shrink_ratio * 0.5))

                if is_adjust_penalty or dynamic_constraints_satisfied:
                    break

                if check_for_constraints:
                    break

                if self.is_objective_function_converged(actual_reduction, min_actual_redution):
                    is_objective_converged = True
                    inter_status = "actual reduction is very small"
                    self.logger.info(inter_status)
                    self.status = "Solved"
                    break
                # else:
                #     is_converged = False

                if self.is_x_converged(x_k, p_k, min_x_redution):
                    is_x_converged = True
                    inter_status = "reduction in x is very small"
                    self.logger.info(inter_status)
                    self.status = "Solved"
                    break
            if self.is_constraints_satisfied(x_k, p, const_violation_tolerance):
                if callback_function is not None:
                    if dynamic_constraints_satisfied:
                        is_converged = True
                        if inter_status != "-1":
                            inter_status += " and"
                        else:
                            inter_status = ""
                        self.logger.info(inter_status + " constraints violations are satisfied")

                        self.status = "Solved"
                        break
                    else:
                        is_converged = False
                else:
                    is_converged = True
                    if inter_status != "-1":
                        inter_status += " and"
                    else:
                        inter_status = ""
                    self.logger.info(inter_status + " constraints violations are satisfied")

                    self.status = "Solved"
                    break
            else:
                is_converged = False
                is_objective_converged = False
                is_x_converged = False
                check_for_constraints = False
                dynamic_constraints_satisfied = False

            if is_converged or dynamic_constraints_satisfied:
                break
            penalty.value *= 10
            iteration_count = 0
            is_adjust_penalty = False
            trust_box_size = float(self.solver_config["trust_region_size"])
        self.logger.debug("\n initial x_0 " + str(x_0))
        self.logger.debug("\n final x_k " + str(x_k))
        self.logger.debug("solver status: " + self.status)

        return self.status, x_k
