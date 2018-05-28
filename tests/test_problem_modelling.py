import sys
# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
import scripts.sqp_solver.ProblemModelling as modelling
import numpy as np
import time


class Test_Problem_Modelling(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.problem = dict(np.load("problem.npy").item())

    def setUp(self):
        pass

    def tearDown(self):
        pass



    def test_init(self):
        samples = np.arange(500, 1000, 100, dtype=int)
        durations = np.arange(50, 100, 10, dtype=int)
        # print samples.shape
        # print durations.shape
        for sample, duration in zip(samples, durations):

            self.model = modelling.ProblemModelling()
            if sample == 0 or duration == 0:
                with self.assertRaises(ValueError):
                    start = time.time()
                    self.model.init(self.problem["joints"], sample, duration,
                                    collision_safe_distance=0.5, collision_check_distance=2)
                end = time.time()
            else:
                start = time.time()
                self.model.init(self.problem["joints"], sample, duration,
                                collision_safe_distance=0.5, collision_check_distance=2)
                end = time.time()
            init_time = end - start
            print ("Initialization for samples: %f, duration: %f completed in %f" %(sample, duration, init_time))

if __name__ == '__main__':
    unittest.main()


