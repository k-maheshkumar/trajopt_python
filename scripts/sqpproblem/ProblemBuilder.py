import numpy as np



class ProblemBuilder:
    def __init__(self, samples, duration, joint, decimals_to_round=3):
        self.joint = joint
        self.samples = samples
        self.duration = duration
        self.decimals_to_round = decimals_to_round

        self.fillP()
        self.fillG()
        self.fillA()


        self.q = np.zeros((1, self.samples))

        self.getStartAndEnd()

        self.fillbounds()
        self.fillBoundsforG()
        self.fillEqualityConstraintforA()

        if "collision_constraints" in self.joint:
            self.collision_constraints = self.joint["collision_constraints"]
            if self.collision_constraints is not None:
                self.fill_collision_matrix()
                self.fill_bounds_for_collision_matrix()
        else:
            self.collision_constraints = None


    def getStartAndEnd(self):
        if type(self.joint) is dict:
            self.start = self.joint["start"]
            self.end = self.joint["end"]
        else:
            self.start = self.joint.states.start
            self.end = self.joint.states.end


    def fillbounds(self):
        if type(self.joint) is dict:
            self.lb = np.full((1, self.P.shape[0]), self.joint["lower_joint_limit"])
            self.ub = np.full((1, self.P.shape[0]), self.joint["upper_joint_limit"])
        else:
            self.lb = np.full((1, self.P.shape[0]), self.joint.limits.lower)
            self.ub = np.full((1, self.P.shape[0]), self.joint.limits.upper)

    def fillBoundsforG(self):
        if type(self.joint) is dict:
            max_vel = self.joint["max_velocity"]

            min_vel = self.joint["min_velocity"]
        else:
            max_vel = self.joint.limits.velocity

            min_vel = - self.joint.limits.velocity

        self.lbG = np.full((1, self.G.shape[0]), min_vel * self.duration / (self.samples - 1))
        self.ubG = np.full((1, self.G.shape[0]), max_vel * self.duration / (self.samples - 1))


    def fillEqualityConstraintforA(self):
        self.b = np.zeros((1, 2))

        if type(self.joint) is dict:
            self.b[0,0] = np.round(self.joint["start"], self.decimals_to_round)

            self.b[0,1] = np.round(self.joint["end"], self.decimals_to_round)
        else:
            self.b[0, 0] = np.round(self.joint.states.start, self.decimals_to_round)

            self.b[0, 1] = np.round(self.joint.states.end, self.decimals_to_round)


    def fillP(self):
        self.P = np.zeros((self.samples, self.samples))
        np.fill_diagonal(self.P, 2.0)
        self.hShape = self.P.shape
        self.P[0, 0] = 1.0
        self.P[self.hShape[0] - 1, self.hShape[0] - 1] = 1.0
        self.P[np.arange(self.hShape[0] - 1), np.arange(self.hShape[0] - 1) + 1] = -2.0


    def fillA(self):
        self.A = np.zeros((2, self.samples))

        self.A[0, 0] = 1.0
        self.A[1, self.A.shape[1] - 1] = 1.0


    def fillG(self):
        self.G = np.zeros((self.samples, self.samples))

        self.aShape = self.G.shape
        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1)] = -1

        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) + 1] = 1
        # to slice zero last row
        self.G.resize(self.samples - 1, self.samples)

    def fill_collision_matrix(self):
        self.collision_matrix = np.zeros((self.samples, self.samples))

        self.aShape = self.collision_matrix.shape
        self.collision_matrix[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1)] = -1

        self.collision_matrix[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) + 1] = 1
        # to slice zero last row
        self.collision_matrix.resize(self.samples - 1, self.samples)

    def fill_bounds_for_collision_matrix(self):

        if type(self.joint) is dict:
            lower_limit = self.collision_constraints["limits"]["lower"] - \
                          self.collision_constraints["initial_signed_distance"]
            upper_limit = self.collision_constraints["limits"]["upper"]
            normal = self.collision_constraints["normal"]
            jacobian = self.collision_constraints["jacobian"]
            max_vel = self.joint["max_velocity"]

        else:
            lower_limit = self.collision_constraints.limits.lower - \
                          self.collision_constraints.initial_signed_distance
            upper_limit = self.collision_constraints.limits.upper
            normal = self.collision_constraints.normal
            jacobian = self.collision_constraints.jacobian
            max_vel = self.joint.limits.velocity

        self.lower_collision_limit = np.full((1, self.collision_matrix.shape[0]), lower_limit)
        # self.upper_collision_limit = np.full((1, self.collision_matrix.shape[0]), upper_limit *
        #                                      self.duration / (self.samples - 1))
        self.upper_collision_limit = np.full((1, self.collision_matrix.shape[0]), max_vel *
                                             self.duration / (self.samples - 1))


        jacobian_times_normal = np.matmul(normal.T, jacobian)

        # print "self.collision_matrix before", self.collision_matrix
        self.collision_matrix = jacobian_times_normal * self.collision_matrix
        # print "self.collision_matrix after", self.collision_matrix
        # print "lower_limit", self.lower_collision_limit, lower_limit
        # print "upper_limit", self.upper_collision_limit, upper_limit
        # print "jacobian_times_normal", jacobian_times_normal

    def display(self):
        # print "Problem ", self.problem
        print "P"
        print self.P
        print "q"
        print self.q
        print "G"
        print self.G
        print "lb"
        print self.lb
        print "ub"
        print self.ub
        print "lbG"
        print self.lbG
        print "ubG"
        print self.ubG
        print "b"
        print self.b
        print "A"
        print self.A

        print "max_iteration"
        print self.max_iteration

