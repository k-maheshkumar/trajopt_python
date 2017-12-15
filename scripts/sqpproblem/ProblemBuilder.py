import numpy as np



class ProblemBuilder:
    def __init__(self, samples, duration, joint, max_iteration):
        self.joint = joint
        self.samples = samples

        self.max_iteration = max_iteration

        self.duration = duration
        self.fillP()
        self.fillG()
        self.fillA()


        self.q = np.zeros((1, self.samples))

        self.getStartAndEnd()

        self.fillbounds()
        self.fillBoundsforG()
        self.fillEqualityConstraintforA()


    def getStartAndEnd(self):
        self.start = self.joint["start"]
        self.end = self.joint["end"]

    def fillbounds(self):

        self.lb = np.full((1, self.P.shape[0]), self.joint["lower_joint_limit"])
        self.ub = np.full((1, self.P.shape[0]), self.joint["upper_joint_limit"])


    def fillBoundsforG(self):

        max_vel = self.joint["max_velocity"]

        min_vel = self.joint["min_velocity"]

        self.lbG = np.full((1, self.G.shape[0]), min_vel * self.duration / (self.samples - 1))
        self.ubG = np.full((1, self.G.shape[0]), max_vel * self.duration / (self.samples - 1))


    def fillEqualityConstraintforA(self):
        self.b = np.zeros((1, 2))

        self.b[0,0] = self.joint["start"]

        self.b[0,1] = self.joint["end"]

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
        self.G = np.zeros((self.samples , self.samples))

        self.aShape = self.G.shape
        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1)] = -1

        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) + 1] = 1
        # to slice zero last row
        self.G.resize(self.samples - 1, self.samples)


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

