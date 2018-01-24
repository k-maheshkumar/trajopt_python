import sys
# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
import scripts.sqp_solver.SQPsolver as solver
import numpy as np
import yaml
import os



class Test_sqp_solver(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
       pass

    def setUp(self):

        # self.q = np.asarray([0, 0])
        # self.P = 2 * np.asarray([[1., 2.], [0., 4.]])
        # self.G = np.asarray([[-1.0, 0.0], [0.0, -1.0]])
        # self.lbG = np.asarray([0.0, 0.0])
        # self.ubG = np.asarray([3.0, 2.0])
        # self.A = np.asarray([[2.0, 1.0], [0.5, 1.2]])
        # self.b = np.asarray([2.0, 0.3])
        # # self.initial_guess = None
        # self.initial_guess = [0.5, 0.0]

        self.P = np.array([[2., -4., 0.],
                           [0., 4., -4.],
                           [0., 0., 2.]])
        # Make symmetric and not indefinite
        self.P = .5 * (self.P + self.P.T) + 1e-08 * np.eye(3)

        self.q = np.array([0., 0., 0.])

        self.G = np.array([[-1., 1., 0.],
                           [0., -1., 1.],
                           [1., 0., 0.],
                           [0., 1., 0.],
                           [0., 0., 1.]])
        # self.start = 0.1
        # self.end = 0.9
        self.lbG = np.array([-0.3, -0.3, -0.3, -0.3, -0.3])
        self.ubG = np.array([0.3, 0.3, 1.1, 1.1, 1.1])

        self.start = -0.49
        self.end = -2.041
        # self.lbA = np.array([-10, -10, -2.96, -2.96, -2.96])
        # self.ubA = np.array([-10, 10, 2.96, 2.96, 2.96])

        self.A = np.array([[1., 0., 0.],
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
        self.b = np.array([self.start, self.end,
                           self.start, self.end,
                           self.start, self.end,
                           self.start, self.end,
                           self.start, self.end
                           ])
        self.initial_guess = self.interpolate(self.start, self.end, 3)

    def tearDown(self):
        pass

    def test_solver_without_config_file(self):
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess)
        # self.solver.display_problem()
        status, result = self.solver.solve()
        self.assertEquals(status, "Solved")
        # self.assertEquals(status, '-1')

    def test_solver_with_config_file_none(self):
        self.solver = solver.SQPsolver()

        self.solver.init(P=self.P, q=self.q, G=self.G, lbG=self.lbG, ubG=self.ubG, A=self.A, b=self.b,
                         initial_guess=self.initial_guess, solver_config=None)
        status, result = self.solver.solve()
        self.assertEquals(status, "Solved")

    def test_solver_with_config_file(self):
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
        status = self.solver.is_constraints_satisfied(x_k)
        self.assertEquals(status, False)

        x_k = [-0.49, -0.79, -2.040]
        status = self.solver.is_constraints_satisfied(x_k)
        self.assertEquals(status, True)

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
        p_k = np.asarray([ -5.78025222e-06,   4.75149321e-01,   7.50458345e-01])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, False)

        x_k = np.asarray([-0.49027259, -0.79316194, -2.04089679])
        p_k = np.asarray([-0.00040797, -0.000409058,  0.00018375])
        status = self.solver.is_x_converged(x_k, p_k)
        self.assertEquals(status, True)

    def test_solver_from_problem_file(self):
        with open("problem_1_joint.yaml", 'r') as config:
            self.problem = yaml.load(config)


    def interpolate(self, start, end, samples):
        data = []
        stepSize = (end - start) / (samples - 1)
        intermediate = start
        # if start < 0 and end < 0:
        #     stepSize *= -1
        for i in range(samples):
            data.append(intermediate)
            intermediate += stepSize
        return np.round(data, 3)
if __name__ == '__main__':
    unittest.main()


