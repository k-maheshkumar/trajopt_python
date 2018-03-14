import logging
import time

from easydict import EasyDict as edict

from scripts.Robot.Planner import TrajectoryPlanner
from scripts.utils.yaml_paser import ConfigParser


class Example:
    def __init__(self, problem, verbose=False):


        main_logger_name = "Trajectory_Planner"
        # verbose = "DEBUG"
        self.logger = logging.getLogger(main_logger_name)
        self.setup_logger(main_logger_name, verbose)
        self.problem = problem
        self.plan = TrajectoryPlanner()

    def setup_logger(self, main_logger_name, verbose=False, log_file=False):

        # creating a formatter
        formatter = logging.Formatter('-%(asctime)s - %(name)s - %(levelname)-8s: %(message)s')

        # create console handler with a debug log level
        log_console_handler = logging.StreamHandler()
        if log_file:
            # create file handler which logs info messages
            logger_file_handler = logging.FileHandler(main_logger_name + '.log', 'w', 'utf-8')
            logger_file_handler.setLevel(logging.INFO)
            # setting handler format
            logger_file_handler.setFormatter(formatter)
            # add the file logging handlers to the logger
            self.logger.addHandler(logger_file_handler)

        if verbose == "WARN":
            self.logger.setLevel(logging.WARN)
            log_console_handler.setLevel(logging.WARN)

        elif verbose == "INFO" or verbose is True:
            self.logger.setLevel(logging.INFO)
            log_console_handler.setLevel(logging.INFO)

        elif verbose == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
            log_console_handler.setLevel(logging.DEBUG)

        # setting console handler format
        log_console_handler.setFormatter(formatter)
        # add the handlers to the logger
        self.logger.addHandler(log_console_handler)

    def init(self):
        self.plan.init(joints=self.problem.joints, samples=self.problem.samples, duration=self.problem.duration,
                  solver=None, solver_config=None, solver_class=1,
                  decimals_to_round=4, verbose=False)

    def calculate(self):
        # self.plan.calculate_trajectory()
        print self.plan.display_problem()

if __name__ == '__main__':

    # problem = {}
    # with open('./problem.json') as json_data:
    #     problem = edict(json.load(json_data))

    problem = edict(ConfigParser("./problem.yaml").config)
    example = Example(problem, verbose="DEBUG")
    example.init()

    start = time.time()
    example.calculate()
    end = time.time()
    # plan.display_problem()
    # example.plan.display_problem()
    # print example.plan.get_trajectory().trajectory_by_name
    print("computation time: ", end - start)

# # print request
# request = munchify(request)
#
# temp = 1
# plan = planner.TrajectoryOptimizationPlanner()
#
# start = time.time()
# # plan.init(problem=request)
# request["samples"] = 500
# request["duration"] = 2
# plan.init(joints=request["joints"], samples=request["samples"], duration=request["duration"],
#           solver=None, solver_config=None, solver_class=0,
#           decimals_to_round=4, verbose=False)
# plan.calculate_trajectory()
# end = time.time()
# # plan.display_problem()
# print plan.get_trajectory().trajectory_by_name
# print("computation time: ",end - start)

