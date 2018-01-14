import copy
import cvxpy
import numpy as np
from scripts.utils import yaml_paser as yaml
import logging

'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbC <= G * x <= ubC
            A * x == b
            lb <= x <= ub

'''


class SQPsolver:
    def __init__(self):
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

        self.logger = logging.getLogger("Trajectory_Planner." + __name__)

    def init(self, **kwargs):
        if "P" in kwargs:
            self.P = kwargs["P"]
        if "q" in kwargs:
            self.q = kwargs["q"]
        if "G" in kwargs:
            self.G = kwargs["G"]
        if "lbG" in kwargs:
            self.lbG = kwargs["lbG"]
        if "ubG" in kwargs:
            self.ubG = kwargs["ubG"]
        if "A" in kwargs:
            self.A = kwargs["A"]
        if "b" in kwargs:
            self.b = kwargs["b"]
        if "initial_guess" in kwargs:
            self.initial_guess = kwargs["initial_guess"]
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
            # file_path_prefix = '../../../config/'
            file_path_prefix = '../../config/'
            sqp_config_file = file_path_prefix + 'sqp_config.yaml'

            sqp_yaml = yaml.ConfigParser(sqp_config_file)
            self.solver_config = sqp_yaml.get_by_key("sqp")
        if solver is not None:
            self.solver = solver
        else:
            self.solver = self.solver_config["solver"][0]

        if solver_config is not None:
            self.solver_config = solver_config
        else:
            file_path_prefix = '../../config/'
            # file_path_prefix = '../../../config/'
            sqp_config_file = file_path_prefix + 'sqp_config.yaml'

            sqp_yaml = yaml.ConfigParser(sqp_config_file)
            self.solver_config = sqp_yaml.get_by_key("sqp")
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

    def evaluate_constraints(self, x_k):
        cons1 = np.subtract(np.matmul(self.G, x_k), self.ubG)
        cons2 = np.add(np.matmul(-self.G, x_k), self.lbG)
        cons3 = np.subtract(np.matmul(self.A, x_k), self.b)
        return cons1.flatten(), cons2.flatten(), cons3.flatten()

    def is_constraints_satisfied(self, x_k, tolerance=1e-3):
        cons1_cond = np.matmul(self.G, x_k) <= self.ubG
        cons2_cond = np.matmul(-self.G, x_k) >= self.lbG
        cons3_cond = np.isclose(np.matmul(self.A, x_k), self.b, rtol=tolerance, atol=tolerance)
        # return cons2_cond.all() and cons3_cond.all() or cons1_cond.all() and cons3_cond.all()
        return cons2_cond.all() and cons3_cond.all() and cons1_cond.all()

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

        model += penalty * (cvxpy.norm1(cons1_model) + cvxpy.norm1(cons2_model) + cvxpy.norm1(cons3_model))

        return model, objective_at_xk

    def get_actual_objective(self, x_k, penalty):
        x = cvxpy.Variable(self.P.shape[0])
        x.value = copy.copy(x_k)
        objective = 0.5 * cvxpy.quad_form(x, self.P) + self.q * x
        objective += penalty * (
        cvxpy.norm1(self.G * x - self.ubG.flatten()) + cvxpy.norm1(-self.G * x + self.lbG.flatten()) + cvxpy.norm1(
            self.A * x + self.b.flatten()))

        return objective

    def solve_problem(self, x_k, penalizer, p, delta, constraints=None, lower_limit=None, upper_limit=None):
        model_objective, actual_objective = self.get_model_objective(x_k, penalizer, p)
        if constraints is not None:
            print lower_limit, constraints.shape
            constraints = [cvxpy.norm(p, "inf") <= delta, lower_limit <= constraints * p]
            # constraints = [cvxpy.norm(p, "inf") <= delta]
        else:
            constraints = [cvxpy.norm(p, "inf") <= delta]
        problem = cvxpy.Problem(cvxpy.Minimize(model_objective), constraints)
        result = problem.solve(solver=self.solver, warm_start=True, verbose=False)
        return p.value, model_objective, actual_objective, problem.status

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
        # x_0 = np.full((1, self.P.shape[0]), 8.0).flatten()
        if initial_guess is None:
            x_0 = self.initial_guess
        else:
            x_0 = initial_guess
        p_0 = np.zeros(p.shape[0])

        penalty.value = float(self.solver_config["initial_penalty"])
        trust_box_size = float(self.solver_config["trust_region_size"])
        max_penalty = float(self.solver_config["max_penalty"])
        min_trust_box_size = float(self.solver_config["min_trust_box_size"])
        max_trust_box_size = float(self.solver_config["max_trust_box_size"])

        trust_shrink_ratio = float(self.solver_config["trust_shrink_ratio"])
        trust_expand_ratio = float(self.solver_config["trust_expand_ratio"])

        trust_good_region_ratio = float(self.solver_config["trust_good_region_ratio"])
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
        isAdjustPenalty = False
        old_rho_k = 0
        new_x_k = copy.copy(x_0)

        con1_norm, con2_norm, con3_norm = self.get_constraints_norm(x_k)
        same_trust_region_count = 0
        old_trust_region = copy.copy(trust_box_size)

        actual_objective_at_x_plus_p_k = 0
        model_objective_at_p_0 = 0

        actual_reduction = 1000
        predicted_reduction = 0

        rho_k = 0
        inter_status = "-1"
        p_k = None
        while penalty.value <= max_penalty:
            # print "penalty ", penalty.value, trust_box_size
            self.logger.debug("penalty " + str(penalty.value))
            if penalty.value >= 1000:
                trust_box_size = 2
            while iteration_count < max_iteration:
                iteration_count += 1
                self.logger.debug("iteration_count " + str(iteration_count))
                # print "iteration_count", iteration_count, trust_box_size
                while trust_box_size >= min_trust_box_size:
                    if callback_function is not None:
                        constraints, lower_limit, upper_limit = callback_function(x_k, p_k)
                        if constraints is not None and lower_limit is not None and upper_limit is not None:
                            p_k, model_objective_at_p_k, \
                            actual_objective_at_x_k, solver_status = self.solve_problem(x_k, penalty, p, trust_box_size,
                                                                                        constraints, lower_limit,
                                                                                        upper_limit)
                        else:
                            is_converged = True
                            inter_status = "dynamic constrained satisfied "
                            self.logger.info(inter_status)

                            self.status = "Solved"
                            break
                    else:
                        p_k, model_objective_at_p_k, actual_objective_at_x_k, solver_status = self.solve_problem(x_k,
                                                                                                             penalty, p,
                                                                                                             trust_box_size,)

                    actual_objective_at_x_plus_p_k = self.get_actual_objective(x_k + p_k, penalty)
                    model_objective_at_p_0 = self.get_actual_objective(p_0, penalty)

                    actual_reduction = actual_objective_at_x_plus_p_k.value - actual_objective_at_x_k.value
                    predicted_reduction = model_objective_at_p_0.value - model_objective_at_p_k.value

                    rho_k = actual_reduction / predicted_reduction
                    x_k += p_k


                    self.logger.debug("\n x_k " + str(x_k))
                    self.logger.debug("\n rho_k " + str(rho_k))

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

                    old_rho_k = rho_k
                    old_trust_region = copy.copy(trust_box_size)

                trust_box_size = np.fmax(min_trust_box_size, trust_box_size / trust_shrink_ratio * 0.5)

                if abs(actual_reduction) <= min_actual_redution:
                    # if not self.is_constraints_satisfied(x_k, const_violation_tolerance):
                    #     self.status = "infeasible initial guess"
                    #     is_converged = True
                    #     break
                    # is_converged = True
                    inter_status = "actual reduction is very small"
                    self.logger.info(inter_status)

                    self.status = "Solved"
                    break
                if abs((np.linalg.norm(new_x_k - x_k, np.inf))) <= min_x_redution:
                    # is_converged = True
                    inter_status = "reduction in x is very small"
                    self.logger.info(inter_status)
                    self.status = "Solved"
                    break
                new_x_k = copy.copy(x_k)
                if is_converged:
                    break
            if self.is_constraints_satisfied(x_k, const_violation_tolerance):
                is_converged = True
                if inter_status != "-1":
                    inter_status += " and"
                else:
                    inter_status = ""
                self.logger.info(inter_status + " constraints violations are satisfied")

                self.status = "Solved"
                break

            if is_converged or isAdjustPenalty:
                break
            penalty.value *= 10
            iteration_count = 0
        self.logger.debug("\n initial x_0 " + str(x_0))
        self.logger.debug("\n final x_k " + str(x_k))
        self.logger.debug("solver status: " + self.status)

        return self.status, x_k