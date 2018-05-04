import cvxpy
import numpy as np
from cvxpy.constraints import constraint

import cvxpy.lin_ops.lin_op as lo
import copy

class ConvexOptimizer:

    def __init__(self):
        self.joints = None
        self.samples = None
        self.duration = None
        self.x = None
        self.objective = 0
        self.constraints = []

        self.problem = None

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
                # min_vel = - joints[i].limit.velocity * self.duration / float(self.samples - 1)
                # max_vel = joints[i].limit.velocity * self.duration / float(self.samples - 1)
                # lower_limit = joints[i].limit.lower
                # upper_limit = joints[i].limit.upper
                # start = joints[i].states.start
                # end = joints[i].states.end
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

    def model_problem(self):
        joints = self.joints.values()
        for i in range(len(joints)):
            for t in range((self.samples - 1)):
                # min_vel = - joints[i].limit.velocity * self.duration / float(self.samples - 1)
                # max_vel = joints[i].limit.velocity * self.duration / float(self.samples - 1)
                # lower_limit = joints[i].limit.lower
                # upper_limit = joints[i].limit.upper
                # start = joints[i].states.start
                # end = joints[i].states.end
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



    def init_problem1(self):
        self.problem = cvxpy.Problem(cvxpy.Minimize(self.objective), self.constraints)

        problem = cvxpy.Problem(cvxpy.Minimize(self.objective), self.constraints)
        expr = cvxpy.transforms.indicator(self.constraints)
        # c = cvxpy.constraints.constraints.Constraints()
        # c.
        self.x.value = [[-0.49197958, - 1.26687622, - 2.0417783]]

        cons = expr * self.x

        print cons.shape, len(self.constraints), self.x.shape
        # for c in cons:
        #     print c
        # print cons[0]
        data = self.get_problem_data()
        # cons_val = constraint.Constraint(self.constraints)
        # cons_v = cons_val.residual


        # objective = self.objective + cvxpy.norm(cons, 1)
        # self.problem = cvxpy.Problem(cvxpy.Minimize(objective))

        # t = cvxpy.lin_ops.lin_op.LinOp()
        a = 0
        for i, con in enumerate(problem.constraints):
            a += con.residual
        #     # # print con.residual
        #     # e =  con.canonicalize()[1][0]
        #     # # print cvxpy.multiply(con, self.x.value)
        #     # # print self.x.value
        #     # print e

        print a
        print data["P"]
        print self.x.value
        print self.x.value + [[0, 0]]
        # tep = np.vstack([self.x.value, [[0]] * 2 ])
        tep = self.x.value
        print tep.shape
        k =  data["P"][:3,:3]
        model_grad = 0.5 * np.matmul((k + k.T), tep)
        model_hess = 0.5 * (k + k.T)

        print model_hess.shape

        cons1_model = a + cons.T * self.p
        cd = 0.5 * cvxpy.quad_form(self.p, model_hess)
        obj = self.objective
        penalty = cvxpy.Parameter(nonneg=True)
        obj += problem.value
        obj += model_grad.T * self.p + cd
        obj += penalty * cvxpy.norm(cons1_model, 1)

        # problem1 = cvxpy.Problem(cvxpy.Minimize(obj))
        # c = problem1.solve()
        #
        # print x.value

    def init_problem(self):
        self.problem = cvxpy.Problem(cvxpy.Minimize(self.objective), self.constraints)
        self.x.value = [[0] * self.samples]

        p = cvxpy.Variable((self.samples, len(self.joints)))
        p.value = [[0] * self.samples]



        data = self.get_problem_data()
        # P = data["P"][:self.samples, :self.samples]
        P = np.asarray([
            [1, -2, 0],
            [0, 2, -2],
            [0, 0, 1]]
                     )
        P = P + + 1e-08 * np.eye(P.shape[1])
        q = data["q"]
        A = data["A"][-4:, -self.samples:]
        b = data["b"][2:]
        b = b.reshape(b.shape[0], 1)
        G = data["G"][:, -self.samples:]
        lbG = data["lbG"]
        lbG = lbG.reshape(lbG.shape[0], 1)

        #
        # print P
        # print q
        # print A
        # print b



        x_k = copy.deepcopy(self.x.value)
        # print x_k
        # print G
        # print np.matmul(G, x_k)
        cons1_at_xk = np.add(np.matmul(G, x_k), lbG)
        cons1_grad = -G
        cons2_at_xk = np.subtract(np.matmul(A, x_k), b)
        cons2_grad = A

        # print x_k.shape, lbG.shape, cons_at_xk.shape, cons_grad.shape, p.shape
        cons_model1 = cons1_at_xk
        cons_model1 += cons1_grad * p
        cons_model2 = cons2_at_xk
        cons_model2 += cons2_grad * p
        # print A.shape, x_k.shape, b.shape
        # print cons2_at_xk.shape, cons2_grad.shape, p.shape

        # cons_model11 = cons1_at_xk + np.matmul(cons1_grad, p.value)
        # print cons_model11.shape
        # expr = cvxpy.transforms.indicator(self.constraints)
        #
        # cons = expr * self.x
        #
        # print len(self.problem.constraints)
        # a = 0
        # for i, con in enumerate(self.problem.constraints):
        #     a += con.residual
        # cons1_model12 = a + cons.T * p
        # print cons

        # print P.shape
        # print P

        model_grad = 0.5 * np.matmul((P + P.T), x_k)
        model_hess = 0.5 * (P + P.T)

        model = self.problem.value
        model += model_grad.T * p
        model += 0.5 * cvxpy.quad_form(p, model_hess)
        penalty = cvxpy.Parameter(nonneg=True)
        penalty.value = 1000
        model += penalty * (cvxpy.norm(cons_model1, 1))
        model += penalty * (cvxpy.norm(cons_model2, 1))
        problem = cvxpy.Problem(cvxpy.Minimize(model))
        problem.solve()


        sol = x_k + p.value
        print "from here", sol.T



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