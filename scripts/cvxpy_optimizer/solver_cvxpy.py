import cvxpy

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


    def model_problem(self):
        joints = self.joints.values()
        for i in range(len(joints)):
            for t in range((self.samples - 1)):
                min_vel = - joints[i].limit.velocity * self.duration / float(self.samples - 1)
                max_vel = joints[i].limit.velocity * self.duration / float(self.samples - 1)
                lower_limit = joints[i].limit.lower
                upper_limit = joints[i].limit.upper
                start = joints[i].states.start
                end = joints[i].states.end
                self.objective += cvxpy.sum_squares(self.x[t + 1, i] - self.x[t, i])
                self.constraints += [self.x[t + 1, i] - self.x[t, i] <= max_vel,
                                min_vel <= self.x[t + 1, i] - self.x[t, i]]
                self.constraints += [lower_limit <= self.x, self.x <= upper_limit]
                self.constraints += [self.x[0, i] == start, self.x[self.samples - 1, i] == end]

    def init_problem(self):
        self.problem = cvxpy.Problem(cvxpy.Minimize(self.objective), self.constraints)

    def solve(self):
        self.problem.solve(solver=cvxpy.ECOS, verbose=False)
        return self.x.value.T