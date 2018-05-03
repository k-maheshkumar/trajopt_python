import logging
import time
from scripts.utils.yaml_paser import ConfigParser
from scripts.cvxpy_optimizer.solver_cvxpy import ConvexOptimizer

class Example:
    def __init__(self, problem):

        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.joints = problem["joints"]
        self.planner = ConvexOptimizer()
        self.planner.init(self.joints, self.samples, self.duration)


    def calculate(self):
        traj = self.planner.solve()
        print traj

if __name__ == '__main__':
    # problem = {}
    # with open('./problem.json') as json_data:
    #     problem = edict(json.load(json_data))

    problem = ConfigParser("../problem.yaml").config
    example = Example(problem)

    # start = time.time()
    # example.calculate()
    # end = time.time()
    # # plan.display_problem()
    # # example.plan.display_problem()
    # # print example.plan.get_trajectory().trajectory_by_name
    # print("computation time: ", end - start)
    #
