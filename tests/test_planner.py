import sys
# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
from easydict import EasyDict as edict
import yaml
import numpy as np

from scripts.TrajectoryOptimizationPlanner import Planner as planner
from scripts.cvxpy_optimizer.solver_cvxpy import ConvexOptimizer
from collections import defaultdict
import cvxpy
import random
import itertools

import logging
#
logger = logging.getLogger("Trajectory_Planner")
logger.level = logging.WARN
stream_handler = logging.StreamHandler(sys.stdout)
logger.addHandler(stream_handler)


class Test_sqp_solver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open("problem_1_joint.yaml", 'r') as config:
            cls.problem = edict(yaml.load(config))
        cls.planner = planner.OptimizationPlanner()
        cls.cvx_optimizer = ConvexOptimizer()

        np.random.seed(0)
        length = 1
        cls.samples = np.random.random_integers(20, 30, size=(1, length)).flatten()
        cls.durations = np.random.random_integers(5, 20, size=(1, length)).flatten()
        joints_random = np.random.random_integers(7, 20, size=(1, length)).flatten()
        cls.joints = []

        for i in range(len(joints_random)):

            joints = defaultdict(lambda: defaultdict(dict))
            for j in range(joints_random[i]):
                joints[str(j)]["states"]["start"] = cls.random_float(-2, 2)
                joints[str(j)]["states"]["end"] = cls.random_float(-2, 2)
                joints[str(j)]["limit"]["lower"] = cls.random_float(-5, -2)
                joints[str(j)]["limit"]["upper"] = cls.random_float(2, 5)
                joints[str(j)]["limit"]["velocity"] = cls.random_float(10, 12)
            cls.joints.append(edict(joints))

    def setUp(self):
        pass

    def tearDown(self):
        pass

    @classmethod
    def random_float(self, low, high):
        return random.random() * (high - low) + low

    def get_actual_result(self, joints, samples, duration):
        x = cvxpy.Variable((samples, len(joints)))
        cost = 0
        constraints = []
        joints = joints.values()
        for i in range(len(joints)):
            for t in range((samples - 1)):
                min_vel = - joints[i].limit.velocity * duration / float(samples - 1)
                max_vel = joints[i].limit.velocity * duration / float(samples - 1)
                lower_limit = joints[i].limit.lower
                upper_limit = joints[i].limit.upper
                start = joints[i].states.start
                end = joints[i].states.end
                cost += cvxpy.sum_squares(x[t + 1, i] - x[t, i])
                constraints += [x[t + 1, i] - x[t, i] <= max_vel,
                                min_vel <= x[t + 1, i] - x[t, i]]
                constraints += [lower_limit <= x, x <= upper_limit]
                constraints += [x[0, i] == start, x[samples - 1, i] == end]

        problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        problem.solve(solver=cvxpy.ECOS, verbose=False)
        return x.value.T

    def test_random_joints_planning(self):
        # print joints["10"].states
        for sample, duration, joints in itertools.izip(self.samples, self.durations, self.joints):
            self.cvx_optimizer.init(joints, sample, duration)
            actual_result = self.cvx_optimizer.solve()
            self.planner.init(joints=joints, samples=sample, duration=duration,
                           solver=None, solver_config=None, solver_class=1,
                           decimals_to_round=7, verbose=False)
            # self.planner.display_problem()
            self.planner.calculate_trajectory()
            trajectory =  self.planner.get_trajectory().final.T


            self.assertEquals(np.isclose(actual_result, trajectory, atol=0.01).all(), True)

    def test_planning_from_file(self):
        # print joints["10"].states
        self.cvx_optimizer.init(self.problem.joints, self.problem.samples, self.problem.duration)
        actual_result = self.cvx_optimizer.solve()
        self.planner.init(joints=self.problem.joints, samples=self.problem.samples, duration=self.problem.duration,
                          solver=None, solver_config=None, solver_class=1,
                          decimals_to_round=7, verbose=False)
        # self.planner.display_problem()
        self.planner.calculate_trajectory()
        trajectory = self.planner.get_trajectory().final.T
        # print trajectory

        self.assertEquals(np.isclose(actual_result, trajectory, atol=0.01).all(), True)

if __name__ == '__main__':
    unittest.main()


