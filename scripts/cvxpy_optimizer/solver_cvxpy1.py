import cvxpy
import numpy as np


class ConvexOptimizer:

    def __init__(self):
        self.joints = None
        self.samples = None
        self.duration = None
        self.x = None
        self.objective = 0
        self.constraints = []

        # self.problem = None

    def init(self, joints, samples, duration):
        self.joints = joints
        self.samples = samples
        self.duration = duration
        self.x = cvxpy.Variable((samples, len(joints)))
        self.model_problem()
        self.init_problem()


    def model_problem1(self):
        joints = self.joints.values()
        for i in range(len(joints)):
            for t in range((self.samples - 1)):
                min_vel = - joints[i]["limit"]["velocity"] * self.duration / float(self.samples - 1)
                max_vel = joints[i]["limit"]["velocity"] * self.duration / float(self.samples - 1)
                lower_limit = joints[i]["limit"]["lower"]
                upper_limit = joints[i]["limit"]["upper"]
                start = joints[i]["states"]["start"]
                end = joints[i]["states"]["end"]
                self.objective += cvxpy.sum_squares(self.x[t + 1, i] - self.x[t, i])
                self.constraints += [self.x[t + 1, i] - self.x[t, i] <= max_vel,
                                min_vel <= self.x[t + 1, i] - self.x[t, i]]
                self.constraints += [lower_limit <= self.x, self.x <= upper_limit]
                self.constraints += [self.x[0, i] == start, self.x[self.samples - 1, i] == end]

    def init_problem(self):
        self.problem = cvxpy.Problem(cvxpy.Minimize(self.objective), self.constraints)
        # self.problem = cvxpy.Problem(cvxpy.Minimize(self.objective))

    def get_problem_data(self):
        data = {}
        # data["P"]= np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["P"].todense())[:self.samples, -self.samples:]
        # data["q"]= np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["q"])[-self.samples:]
        # data["A"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["A"].todense())[::, -self.samples:]
        # data["G"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["F"].todense())[::2]
        # data["ubG"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["G"])
        # data["b"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["b"])[-self.samples:]
        # # data["G"] = np.vstack([data["G"], -data["G"]])
        # data["lbG"] = np.zeros(data["ubG"].shape)
        # # data["lbG"] = np.vstack([data["ubG"], -data["ubG"]]).flatten()
        # # data["ubG"] = np.vstack([data["ubG"], -data["ubG"]]).flatten()    `
        # data["lbG"] = -data["ubG"]

        # print data["lbG"]
        # print data["ubG"]
        #
        # print data["G"]

        data["P"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["P"].todense())
        data["q"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["q"])
        data["A"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["A"].todense())
        data["G"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["F"].todense())
        data["ubG"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["G"])
        data["b"] = np.asarray(self.problem.get_problem_data(cvxpy.OSQP)[0]["b"])
        # data["G"] = np.vstack([data["G"], -data["G"]])
        data["lbG"] = np.zeros(data["ubG"].shape)
        # data["lbG"] = np.vstack([data["ubG"], -data["ubG"]]).flatten()
        # data["ubG"] = np.vstack([data["ubG"], -data["ubG"]]).flatten()    `
        data["lbG"] = -data["ubG"]

        return data

    def solve(self, solver="ECOS"):
        self.problem.solve(solver=solver, verbose=False)
        return np.asarray(self.x.value.T)

    def get_constraints(self, x):
        # print x
        y = cvxpy.Variable((self.samples, len(self.joints)))
        y.value = x
        # print y.value
        # print self.constraints
        # cons = cvxpy.matmul(self.constraints, y)

        for con in self.problem.constraints:
            c = cvxpy.NonPos(con)
            print c.residual()

            # temp = cvxpy.matmul(c.variables(), x)


    def model_problem(self):
        joints = self.joints.values()
        penalty = cvxpy.Parameter(nonneg=True)
        penalty.value = 10
        self.penalty_norm = 1

        for i in range(len(joints)):
            for t in range((self.samples - 1)):
                min_vel = - joints[i]["limit"]["velocity"] * self.duration / float(self.samples - 1)
                max_vel = joints[i]["limit"]["velocity"] * self.duration / float(self.samples - 1)
                lower_limit = joints[i]["limit"]["lower"]
                upper_limit = joints[i]["limit"]["upper"]
                start = joints[i]["states"]["start"]
                end = joints[i]["states"]["end"]
                self.objective += cvxpy.sum_squares(self.x[t + 1, i] - self.x[t, i])
                self.constraints += [self.x[t + 1, i] - self.x[t, i] <= max_vel,
                                     min_vel <= self.x[t + 1, i] - self.x[t, i]]
                self.constraints += [lower_limit <= self.x, self.x <= upper_limit]
                self.constraints += [self.x[0, i] == start, self.x[self.samples - 1, i] == end]


                # cons_grad = self.problem.constraints.

                # self.constraints * self.x
                # prb = self.objective + penalty * cvxpy(self.constraints * self.x, self.penalty_norm)


