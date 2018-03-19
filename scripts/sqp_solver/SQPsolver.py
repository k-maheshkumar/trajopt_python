import copy
import cvxpy
import numpy as np
from scripts.utils import yaml_paser as yaml
from scripts.utils.utils import Utils as utils
import logging
import os
import time


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

        self.initial_guess = []
        self.status = "-1"
        self.norm_ = 1

        self.solver_config = {}

        self.solver = []

        self.penalty_norm = 1
        self.trust_region_norm = np.inf

        self.logger = logging.getLogger(main_logger_name + __name__)
        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)

    def init(self, **kwargs):
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

        self.analyse_inputs()

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
        print self.initial_guess

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

    def analyse_inputs(self):
        if self.G is not None and self.lbG is None and self.ubG is not None:
            # self.lbG = np.ones(self.ubG.shape) * -1000
            self.lbG = -self.ubG
            # self.G = np.vstack([self.G])
            # self.lbG = np.hstack([-self.ubG])
            # self.ubG = np.hstack([self.ubG])
        if self.G is not None and self.ubG is None and self.lbG is not None:
            self.G = np.vstack([-self.G, -self.G])
            self.lbG = np.hstack([-self.ubG, -self.ubG])
            self.ubG = np.hstack([self.ubG, -self.ubG])

    def is_constraints_satisfied(self, x_k, tolerance=1e-3):
        cons1_cond = np.isclose(np.matmul(self.G, x_k) <= self.ubG, 1, rtol=tolerance, atol=tolerance)
        cons2_cond = np.isclose(np.matmul(-self.G, x_k) >= self.lbG, 1, rtol=tolerance, atol=tolerance)
        cons3_cond = np.isclose(np.matmul(self.A, x_k), self.b, rtol=tolerance, atol=tolerance)
        # return cons2_cond.all() and cons3_cond.all() or cons1_cond.all() and cons3_cond.all()

        return cons1_cond.all() and cons2_cond.all() and cons3_cond.all()

    def is_x_converged(self, x_k, p_k, tolerance=1e-3):
        return abs((np.linalg.norm(x_k - (x_k + p_k), np.inf))) <= tolerance

    def is_objective_function_converged(self, objective, tolerance=1e-3):

        return abs(objective) <= tolerance

    def evaluate_constraints(self, x_k):
        cons1 = np.subtract(np.matmul(self.G, x_k), self.ubG)
        cons2 = np.add(np.matmul(-self.G, x_k), self.lbG)
        cons3 = np.subtract(np.matmul(self.A, x_k), self.b)
        # print cons1, cons2, cons3
        return cons1.flatten(), cons2.flatten(), cons3.flatten()

    def get_constraints_gradients(self):
        cons1_grad = self.G
        cons2_grad = -self.G
        cons3_grad = self.A
        return cons1_grad, cons2_grad, cons3_grad

    def get_objective_gradient_and_hessian(self, x_k):
        model_grad = 0.5 * np.matmul((self.P + self.P.T), x_k)
        model_hess = 0.5 * (self.P + self.P.T)
        return model_grad, model_hess

    def get_model_objective(self, x_k, penalty, p):
        cons1_at_xk, cons2_at_xk, cons3_at_xk = self.evaluate_constraints(x_k)
        cons1_grad_at_xk, cons2_grad_at_xk, cons3_grad_at_xk = self.get_constraints_gradients()
        cons1_model = cons1_at_xk + cons1_grad_at_xk * p
        cons2_model = cons2_at_xk + cons2_grad_at_xk * p
        cons3_model = cons3_at_xk + cons3_grad_at_xk * p

        objective_grad_at_xk, objective_hess_at_xk = self.get_objective_gradient_and_hessian(x_k)
        objective_at_xk = self.get_actual_objective(x_k, penalty)
        model = objective_at_xk.value + objective_grad_at_xk * p + 0.5 * cvxpy.quad_form(p, objective_hess_at_xk)

        model += penalty * (cvxpy.norm(cons1_model, self.penalty_norm) + cvxpy.norm(cons2_model, self.penalty_norm)
                            + cvxpy.norm(cons3_model, self.penalty_norm))

        return model, objective_at_xk

    def get_actual_objective(self, xk, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(xk)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        constraints1 = cvxpy.norm(self.G * x - self.ubG.flatten(), self.penalty_norm)
        constraints2 = cvxpy.norm(-self.G * x + self.lbG.flatten(), self.penalty_norm)
        constraints3 = cvxpy.norm(self.A * x - self.b.flatten(), self.penalty_norm)
        objective += penalty * (constraints1 + constraints2 + constraints3)
        return objective

    def solve_problem(self, x_k, penalizer, p, delta, constraints=None, lower_limit=None, upper_limit=None):
        model_objective, actual_objective = self.get_model_objective(x_k, penalizer, p)
        if constraints is not None:
            # print lower_limit, delta
            if constraints.shape[1] == 2 * p.shape[0]:
                p1 = cvxpy.hstack([p, p])
            else:
                p1 = p
            if lower_limit is not None and upper_limit is not None:
                constraints = [cvxpy.norm(p, self.trust_region_norm) <= delta,
                               lower_limit <= cvxpy.matmul(constraints, p1), cvxpy.matmul(constraints, p1) <= upper_limit]
            elif lower_limit is None:
                constraints = [cvxpy.norm(p, self.trust_region_norm) <= delta,cvxpy.matmul(constraints, p1) <= upper_limit]
            elif upper_limit is None:
                constraints = [cvxpy.norm(p, self.trust_region_norm) <= delta, lower_limit <= cvxpy.matmul(constraints, p1)]
        else:
            constraints = [cvxpy.norm(p, self.trust_region_norm) <= delta]
        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        # print problem.get_problem_data(self.solver)[0]
        if self.solver == "CVXOPT":
            result = problem.solve(solver=self.solver, warm_start=True, kktsolver=cvxpy.ROBUST_KKTSOLVER, verbose=False)
        else:
            result = problem.solve(solver=self.solver, warm_start=True, verbose=False)
        return p.value, model_objective, actual_objective, problem.status

    def approx_equal(self, x, y, tolerance=0.001):
        return abs(x - y) <= 0.5 * tolerance * (x + y)

    def get_constraints_norm(self, x_k):
        con1, con2, con3 = self.evaluate_constraints(x_k)
        max_con1 = (np.linalg.norm(con1, np.inf))
        max_con2 = (np.linalg.norm(con2, np.inf))
        max_con3 = (np.linalg.norm(con3, np.inf))

        return max_con1, max_con2, max_con3

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

        min_model_improve = float(self.solver_config["min_model_improve"])
        improve_ratio_threshold = float(self.solver_config["improve_ratio_threshold"])
        min_approx_improve_frac = - float('inf')

        min_actual_redution = float(self.solver_config["min_actual_redution"])
        min_x_redution = float(self.solver_config["min_x_redution"])

        min_actual_worse_redution = float(self.solver_config["min_actual_worse_redution"])
        const_violation_tolerance = float(self.solver_config["const_violation_tolerance"])
        min_equality_norm = float(self.solver_config["min_equality_norm"])

        x_k = copy.copy(x_0)

        iteration_count = 0
        is_converged = False
        is_objective_converged = False
        is_x_converged = False
        check_for_constraints = False

        is_adjust_penalty = False
        old_rho_k = 0
        new_x_k = copy.copy(x_0)

        same_trust_region_count = 0
        old_trust_region = copy.copy(trust_box_size)

        actual_objective_at_x_plus_p_k = 0
        model_objective_at_p_0 = 0

        actual_reduction = 1000
        predicted_reduction = 0

        rho_k = 0
        old_rho_k = 0
        inter_status = "-1"
        p_k = [0] * len(x_0)
        last_p_k = [0] * len(x_0)
        last_dynamic_constraints_count = 0
        dynamic_constraints_count = 0

        dynamic_constraints_satisfied = False

        while penalty.value <= max_penalty:
            # print "penalty ", penalty.value
            trust_box_size = np.fmax(trust_box_size, min_trust_box_size / trust_shrink_ratio * 1.5)

            self.logger.debug("penalty " + str(penalty.value))
            while iteration_count < max_iteration:
                iteration_count += 1
                # print "iteration_count", iteration_count
                self.logger.debug("iteration_count " + str(iteration_count))
                if callback_function is not None:
                    constraints, lower_limit, upper_limit = callback_function(x_k, p_k)
                while trust_box_size >= min_trust_box_size:
                    if callback_function is not None:
                        # constraints, lower_limit, upper_limit = callback_function(x_k, p_k)
                        if constraints is not None:
                            # dynamic_constraints_count = len(upper_limit)
                            # if dynamic_constraints_count > last_dynamic_constraints_count:
                            #     print "collision count increases. . . .. . ."
                            # print x_k
                            #     x_k -= p_k
                            # print x_k
                            # penalty.value *= trust_expand_ratio
                            # trust_box_size = np.fmin(max_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                            # trust_box_size = np.fmax(min_trust_box_size, np.linalg.norm(lower_limit, np.inf) * 3)
                            # trust_box_size = np.fmin(trust_box_size * trust_expand_ratio, max_trust_box_size)
                            # trust_box_size = np.fmin(trust_box_size * trust_shrink_ratio, max_trust_box_size)
                            # print "delta: ", trust_box_size

                            # last_dynamic_constraints_count = copy.copy(dynamic_constraints_count)
                            # if trust_box_size < np.linalg.norm(lower_limit, np.inf):
                            #     trust_box_size = np.fmin(np.linalg.norm(lower_limit, np.inf) * trust_expand_ratio,
                            #                              max_trust_box_size)
                            # print "delta 1 .. .: ", trust_box_size
                            start = time.time()
                            p_k, model_objective_at_p_k, \
                            actual_objective_at_x_k, solver_status = self.solve_problem(x_k, penalty, p, trust_box_size,
                                                                                        constraints, lower_limit,
                                                                                        upper_limit)
                            end = time.time()

                            # print constraints
                            # print p_k
                            print "each solve_problem time: ", end - start
                            # temp =  np.matmul(constraints, p_k)
                            # print temp
                            # print temp.shape


                        else:
                            dynamic_constraints_satisfied = True
                            inter_status = "dynamic constrained satisfied "
                            self.logger.info(inter_status)

                            self.status = "Solved"
                            # print "must check for constrains satisfaction .. . . ..  ."

                            break
                    else:
                        p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = self.solve_problem(x_k,
                                                                                                                 penalty,
                                                                                                                 p,
                                                                                                                 trust_box_size)

                    if p_k is None:
                        x_k -= last_p_k
                        p.value = last_p_k
                        p_k = last_p_k
                    if p_k is not None:
                        actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, penalty)
                        model_objective_at_p_0 = self.get_model_objective(x_k, penalty, p_0)[0]

                        actual_reduction = actual_objective_at_x_k.value - actual_objective_at_x_plus_p_k.value
                        predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                        if predicted_reduction == 0:
                            predicted_reduction = 0.0000001
                        rho_k = actual_reduction / predicted_reduction

                        self.logger.debug("\n x_k " + str(x_k))
                        self.logger.debug("rho_k " + str(rho_k))
                        # print x_k, rho_k

                        # print "objective_at_x_plus_p_k", actual_objective_at_x_plus_p_k.value
                        # print "model_objective_at_x_plus_p_k", model_objective_at_p_0.value
                        # print "actual xk, xk1", actual_objective_at_x_plus_p_k.value, actual_objective_at_x_k.value
                        # print "model p0, pk",model_objective_at_p_0.value, model_objective_at_p_k.value

                        if solver_status == cvxpy.INFEASIBLE or solver_status == cvxpy.INFEASIBLE_INACCURATE or solver_status == cvxpy.UNBOUNDED or solver_status == cvxpy.UNBOUNDED_INACCURATE:
                            # self.logger.warn("Infeasible problem cannot be solved" + str(penalty.value))
                            # self.status = "Infeasible"
                            # is_adjust_penalty = True
                            penalty.value *= trust_expand_ratio
                            break

                        if rho_k >= trust_good_region_ratio:
                            trust_box_size = np.fmin(trust_box_size * trust_expand_ratio, max_trust_box_size)
                            self.logger.debug("expanding trust region" + str(trust_box_size))
                            x_k += p_k
                            # print "expanding . .. . .. "
                            new_x_k = copy.copy(x_k)
                            break
                        elif rho_k <= trust_bad_region_ratio:
                            trust_box_size *= trust_shrink_ratio
                            self.logger.debug("shrinking trust region " + str(trust_box_size))
                            # x_k = copy.copy(new_x_k)
                            x_k -= p_k

                        # if actual_reduction > 0.2 * predicted_reduction:
                        #     trust_box_size *= trust_expand_ratio
                        #     print "expanding  .. .. . ."
                        #     x_k += p_k
                        # else:
                        #     trust_box_size *=trust_shrink_ratio
                        #     print "shrinking  .. .. . ."

                        if trust_box_size < min_x_redution:
                            check_for_constraints = True
                            break

                        if old_trust_region == trust_box_size:
                            # print "incresing same trust region count . . .. . "
                            same_trust_region_count += 1
                        if same_trust_region_count >= 5:
                            # print "resetting trust region, since trust region size is same for last ",
                            #  same_trust_region_count, " iterations"
                            trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                        if old_rho_k == rho_k:
                            # print "rho_k are same"
                            is_adjust_penalty = True
                            break

                        # if trust_box_size < min_x_redution:
                        #     is_adjust_penalty = True
                        #     break
                        old_rho_k = rho_k
                        old_trust_region = copy.copy(trust_box_size)
                        last_p_k = p_k

                # trust_box_size = np.fmin(max_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)
                # trust_box_size = np.fmax(trust_box_size, min_trust_box_size / trust_shrink_ratio * 0.5)
                # trust_box_size = np.fmax(trust_box_size, min_trust_box_size / trust_shrink_ratio * 1.5);

                # else:
                #     is_converged = False

                if is_adjust_penalty or dynamic_constraints_satisfied:
                    break

                if check_for_constraints:
                    break

                if self.is_objective_function_converged(actual_reduction, min_actual_redution):
                    is_objective_converged = True
                    inter_status = "actual reduction is very small"
                    # print "actual reduction is very small"
                    self.logger.info(inter_status)
                    self.status = "Solved"
                    break
                # else:
                #     is_converged = False

                if self.is_x_converged(x_k, p_k, min_x_redution):
                    is_x_converged = True
                    inter_status = "reduction in x is very small"
                    # print "reduction in x is very small"
                    self.logger.info(inter_status)
                    self.status = "Solved"
                    break

            if self.is_constraints_satisfied(x_k, const_violation_tolerance):
                # print "constrains satisfied .. . . ..  ."
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
        self.logger.debug("\n initial x_0 " + str(x_0))
        self.logger.debug("\n final x_k " + str(x_k))
        self.logger.debug("solver status: " + self.status)

        return self.status, x_k
