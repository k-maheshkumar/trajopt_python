import sys
# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
from easydict import EasyDict as edict
import yaml
import numpy as np

from scripts.Planner import Planner1 as planner

import cvxpy

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
        cls.planner = planner.TrajectoryOptimizationPlanner()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def get_actual_result(self, joints, samples, duration, lower_joint, upper_joint, velocity, start, end):
        lower_limit = lower_joint
        upper_limit = upper_joint
        problem = []
        min_vel = -velocity
        max_vel = velocity
        min_vel *= duration / float(samples - 1)
        max_vel *= duration / float(samples - 1)
        print min_vel, max_vel
        cost = 0
        constraints = []
        x = cvxpy.Variable((samples, joints))

        for i in range(joints):
            for t in range((samples - 1)):
                cost += cvxpy.sum_squares(x[t + 1, i] - x[t, i])
                constraints += [x[t + 1, i] - x[t, i] <= max_vel,
                                min_vel <= x[t + 1, i] - x[t, i]]
                pro = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
                problem.append(pro)
        constraints += [lower_limit <= x, x <= upper_limit]
        constraints += [x[0, 0] == start, x[samples - 1, 0] == end]

        problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        problem.solve(solver=cvxpy.ECOS, verbose=False)
        G = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["F"].todense()).T
        G = G[~np.all(G == 0, axis=1)]
        ubG = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["G"])

        # print G.T
        # print ubG
        return x.value.T

    def test_solver_from_problem_file(self):
        self.problem.samples = 10
        self.problem.duration = 3
        # self.actual_result =  np.array([-0.49197958, - 0.66418096,
        #                                         - 0.83638088, - 1.00858013,
        #                                         - 1.18077868, - 1.35297643,
        #                                         - 1.52517484, - 1.69737578,
        #                                         - 1.86957595, - 2.0417783])
        self.actual_result = self.get_actual_result(len(self.problem.joints), self.problem.samples, self.problem.duration,
                                                    self.problem.joints["lbr_iiwa_joint_1"].limit.lower,
                                                    self.problem.joints["lbr_iiwa_joint_1"].limit.upper,
                                                    self.problem.joints["lbr_iiwa_joint_1"].limit.velocity,
                                                    self.problem.joints["lbr_iiwa_joint_1"].states.start,
                                                    self.problem.joints["lbr_iiwa_joint_1"].states.end)

        # print self.actual_result
        self.planner.init(joints=self.problem.joints, samples=self.problem.samples, duration=self.problem.duration,
                       solver=None, solver_config=None, solver_class=1,
                       decimals_to_round=4, verbose=False)
        # print self.planner.sqp_solver.G
        # print self.planner.sqp_solver.lbG
        # print self.planner.sqp_solver.ubG
        # self.planner.display_problem()
        self.planner.calculate_trajectory()
        trajectory =  self.planner.get_trajectory().final.T
        # print np.gradient(trajectory)
        print "result ", trajectory
        # print "initial ", self.planner.get_trajectory().initial.T

        print np.ediff1d(trajectory)

        print np.isclose(self.actual_result, trajectory, atol=0.01)
        print np.linalg.norm(np.ediff1d(trajectory), np.inf)


        #
    # def testSimpleMsg(self):
    #     stream_handler.stream = sys.stdout
    #     print("AA")
    #     logging.getLogger().info("BB")

if __name__ == '__main__':
    unittest.main()


