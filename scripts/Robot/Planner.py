import logging
import time
import numpy as np
from scripts.utils.utils import Utils as utils
import scripts.sqp_solver.ProblemModelling as model
from scripts.Robot import Trajectory
from scripts.sqp_solver import SQPsolver
from scripts.sqp_solver import SQPsolver_old
from collections import OrderedDict


class TrajectoryPlanner:

    # basic constructor to initialize all necessary variables
    def __init__(self, main_logger_name=__name__, verbose=False, log_file=False):
        self.sqp = OrderedDict()
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
        self.solver_status = []
        self.joint_names = []
        self.problem = OrderedDict()
        self.max_penalty = -1
        self.delta_max = -1
        self.joints = -1
        self.num_of_joints = -1
        self.solver = -1
        self.duration = -1
        self.no_of_samples = -1
        self.max_no_of_Iteration = -1
        self.max_no_of_Iteration = -1
        self.decimals_to_round = 5
        self.collision_check_distance = 0.1
        self.collision_safe_distance = 0.05
        self.solver_class = 0
        self.prob_model_time = 0

        self.solver_config = None
        self.trajectory = Trajectory.Trajectory()
        self.problem_model = model.ProblemModelling()
        self.sqp_solver = None
        self.current_planning_joint_group = None
        self.callback_function_to_get_collision_infos = None
        self.sqp_solver = SQPsolver.SQPsolver(main_logger_name, verbose, log_file)
        self.sqp_solver_old = SQPsolver_old.SQPsolver(main_logger_name, verbose, log_file)

        self.logger = logging.getLogger(main_logger_name + __name__)
        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)

    # clearing all variables so that next trajectory can be planned
    def __clear_all_data(self):
        self.sqp = OrderedDict()
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
        self.solver_status = []
        self.joint_names = []

    # method to initialize necessary variables on each trajectory plan request
    def init(self, **kwargs):
        self.__clear_all_data()
        self.joints = utils.get_var_from_kwargs("joints", **kwargs)
        if self.joints is not None:
            self.num_of_joints = len(self.joints)

        self.solver = utils.get_var_from_kwargs("solver", optional=True, **kwargs)
        self.duration = utils.get_var_from_kwargs("duration", optional=True, default=10, **kwargs)
        self.no_of_samples = utils.get_var_from_kwargs("samples", optional=True, default=20, **kwargs)
        self.max_no_of_Iteration = utils.get_var_from_kwargs("max_iteration", optional=True, default=20, **kwargs)
        self.decimals_to_round = utils.get_var_from_kwargs("decimals_to_round", optional=True, default=5, **kwargs)
        self.collision_safe_distance = utils.get_var_from_kwargs("collision_safe_distance", optional=True,
                                                                 default=0.1, **kwargs)
        self.collision_check_distance = utils.get_var_from_kwargs("collision_check_distance", optional=True,
                                                                  default=0.15, **kwargs)
        self.current_planning_joint_group = utils.get_var_from_kwargs("joint_group", **kwargs)
        self.solver_class = utils.get_var_from_kwargs("solver_class", optional=True, default="new", **kwargs)
        if self.solver_class is not None:
            # considering 1st element of the solver class list from sqp config file
            self.solver_class = self.solver_class[0]

        if self.solver_class.lower() == "old":
            self.sqp_solver = self.sqp_solver_old

        self.solver_config = utils.get_var_from_kwargs("solver_config", optional=True, **kwargs)

        start = time.time()
        # creating model of the trajectory planning problem
        self.problem_model.init(self.joints, self.no_of_samples, self.duration, self.decimals_to_round,
                          self.collision_safe_distance, self.collision_check_distance)
        end = time.time()
        self.prob_model_time += end - start
        # initializing SQP solver
        self.sqp_solver.init(P=self.problem_model.cost_matrix_P, q=self.problem_model.cost_matrix_q,
                             G=self.problem_model.robot_constraints_matrix,
                             lbG=self.problem_model.constraints_lower_limits, ubG=self.problem_model.constraints_upper_limits,
                             A=self.problem_model.start_and_goal_matrix, b=self.problem_model.start_and_goal_limits,
                             initial_guess=self.problem_model.initial_guess, solver_config=self.solver_config)

        self.trajectory.init(np.array((np.split(self.problem_model.initial_guess, self.no_of_samples)))
                             , self.problem_model.samples, self.problem_model.duration, self.current_planning_joint_group)

    def update_prob(self):
        # updating SQP solver variables on updating constraints
        self.sqp_solver.update_prob(G=self.problem_model.robot_constraints_matrix,
                                    lbG=self.problem_model.constraints_lower_limits, ubG=self.problem_model.constraints_upper_limits)

    def display_problem(self):
        # method to display SQP problem data
        self.sqp_solver.display_problem()

    # caling SQP solver to calculate the trajectory
    def calculate_trajectory(self, initial_guess= None, callback_function=None):
        can_execute_trajectory = False
        self.logger.info("getting trajectory")
        start = time.time()
        self.solver_status, trajectory = self.sqp_solver.solve(initial_guess, callback_function)
        end = time.time()
        planning_time = end - start

        trajectory = np.array((np.split(trajectory, self.no_of_samples)))
        # final solved trajectory is updated
        self.trajectory.update(trajectory, self.current_planning_joint_group)
        # updating status of the SQP solver
        if self.solver_status == "Solved":
            can_execute_trajectory = True
            status = "Total time taken to calculate Optimal Trajectory: " + str(planning_time) + " secs"
            self.logger.debug(status)
        else:
            status = "Couldn't find the trajectory for the input problem"
            self.logger.info(status)
        return status, planning_time, can_execute_trajectory

    def get_trajectory(self):
        return self.trajectory


