import sys

# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
import scripts.sqp_solver.SQPsolver as solver
from scripts.cvxpy_optimizer.solver_cvxpy import ConvexOptimizer
import numpy as np
import yaml
import os

from easydict import EasyDict as edict
from collections import defaultdict
import random
import copy


class Test_sqp_solver(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # self.q = np.asarray([0, 0])
        # self.P = 2 * np.asarray([[1., 2.], [0., 4.]])
        # self.G = np.asarray([[-1.0, 0.0], [0.0, -1.0]])
        # self.lbG = np.asarray([0.0, 0.0])
        # self.ubG = np.asarray([3.0, 2.0])
        # self.A = np.asarray([[2.0, 1.0], [0.5, 1.2]])
        # self.b = np.asarray([2.0, 0.3])
        # # self.initial_guess = None
        # self.initial_guess = [0.5, 0.0]

        cls.P = np.array([[2., -4., 0.],
                          [0., 4., -4.],
                          [0., 0., 2.]])
        # Make symmetric and not indefinite
        cls.P = .5 * (cls.P + cls.P.T) + 1e-08 * np.eye(3)

        cls.q = np.array([0., 0., 0.])

        cls.G = np.array([[-1., 1., 0.],
                          [0., -1., 1.],
                          [1., 0., 0.],
                          [0., 1., 0.],
                          [0., 0., 1.]])
        # self.start = 0.1
        # self.end = 0.9
        cls.lbG = np.array([-0.3, -0.3, -0.3, -0.3, -0.3])
        cls.ubG = np.array([0.3, 0.3, 1.1, 1.1, 1.1])

        cls.start = -0.49
        cls.end = -2.041
        # self.lbA = np.array([-10, -10, -2.96, -2.96, -2.96])
        # self.ubA = np.array([-10, 10, 2.96, 2.96, 2.96])

        cls.A = np.array([[1., 0., 0.],
                          [0., 0., 1.],
                          [1., 0., 0.],
                          [0., 0., 1.],
                          [1., 0., 0.],
                          [0., 0., 1.],
                          [1., 0., 0.],
                          [0., 0., 1.],
                          [1., 0., 0.],
                          [0., 0., 1.]
                          ])
        cls.b = np.array([cls.start, cls.end,
                          cls.start, cls.end,
                          cls.start, cls.end,
                          cls.start, cls.end,
                          cls.start, cls.end
                          ])
        cls.initial_guess = cls.interpolate(cls.start, cls.end, 3)

        cls.cvx_solver = ConvexOptimizer()
        np.random.seed(0)
        length = 1
        cls.samples = np.random.random_integers(20, 30, size=(1, length)).flatten()[0]
        cls.durations = np.random.random_integers(5, 20, size=(1, length)).flatten()[0]
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
        cls.cvx_solver.init(cls.joints[0], cls.samples, cls.durations)
        cls.x_from_cvx = cls.cvx_solver.solve()
        cls.problem_from_cvx = cls.cvx_solver.get_problem_data()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    @classmethod
    def random_float(self, low, high):
        return random.random() * (high - low) + low

    def test_solver(self):
        #  test_solver_without_config_file
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess)
        status, result = self.solver.solve()
        self.assertEquals(status, "Solved")

        # test_solver_with_config_file_none
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess, solver_config=None)
        status, result = self.solver.solve()
        self.assertEquals(status, "Solved")

        #  test_solver_with_config_file
        self.solver = solver.SQPsolver()
        file_path_prefix = os.path.join(os.path.dirname(__file__), '../config/')

        sqp_config_file = file_path_prefix + 'sqp_config.yaml'
        with open(sqp_config_file, 'r') as config:
            config_file = yaml.load(config)["sqp"]
        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess, solver_config=config_file)
        status, result = self.solver.solve()
        self.assertEquals(status, "Solved")

    def test_is_constraints_satisfied(self):
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess, solver_config=None)
        x_k = [1.0, 1.0, 1.0]
        p_k = np.ones(len(x_k)) * 1e-2
        status = self.solver.is_constraints_satisfied(x_k, p_k)
        self.assertEquals(status, False)

    def test_is_x_converged(self):
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess, solver_config=None)
        x_k = np.asarray([1.0, 1.0, 1.0])
        p_k = np.asarray([1.0, 1.0, 1.0])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, False)

        x_k = np.asarray([-0.49, -0.79, -2.040])
        p_k = np.asarray([-0.49, -0.79, -2.040])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, False)

        x_k = np.asarray([-0.49000578, -0.78985068, -1.29054165])
        p_k = np.asarray([-5.78025222e-06, 4.75149321e-01, 7.50458345e-01])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, False)

        x_k = np.asarray([-0.49027259, -0.79316194, -2.04089679])
        p_k = np.asarray([-0.00040797, -0.000409058, 0.00018375])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, True)

    def test_is_x_converged_from_cvx_solver_data(self):

        self.solver = solver.SQPsolver()

        self.solver.init(P=self.problem_from_cvx["P"], q=self.problem_from_cvx["q"], G=self.problem_from_cvx["G"],
                         lbG=self.problem_from_cvx["lbG"], ubG=self.problem_from_cvx["ubG"],
                         A=self.problem_from_cvx["A"], b=self.problem_from_cvx["b"])
        x_k = copy.copy(self.x_from_cvx)
        p_k = np.zeros(x_k.shape)
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, True)

        p_k = np.zeros(x_k.shape)
        p_k = np.ones(x_k.shape) * 1e-2
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, False)

    def test_random_is_constraints_satisfied_(self):
        self.solver = solver.SQPsolver()
        # initial_guess = np.zeros(self.x_from_cvx.shape)
        self.solver.init(P=self.problem_from_cvx["P"], q=self.problem_from_cvx["q"], G=self.problem_from_cvx["G"],
                         lbG=self.problem_from_cvx["lbG"], ubG=self.problem_from_cvx["ubG"],
                         A=self.problem_from_cvx["A"], b=self.problem_from_cvx["b"],)
        _, x = self.solver.solve()

        p_k = np.ones(x.shape) * 1e-2
        status = self.solver.is_constraints_satisfied(x, p_k)
        self.assertEquals(status, True)

        x += np.ones(x.shape) * 1e-1
        p_k = np.ones(x.shape) * 1e-2
        status = self.solver.is_constraints_satisfied(x, p_k)
        self.assertEquals(status, False)

    @classmethod
    def interpolate(self, start, end, samples):
        data = []
        stepSize = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += stepSize
        return np.round(data, 3)


if __name__ == '__main__':
    unittest.main()


