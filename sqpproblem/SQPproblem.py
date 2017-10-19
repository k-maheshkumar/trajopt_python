import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from numpy import array, hstack, ones, vstack, zeros


class SQPproblem:
    def __init__(self, samples, duration, joint, maxNoOfIteration):
        self.joint = joint
        self.samples = samples
        # self.jointsVelocities = problem["jointsVelocities"]


        self.nWSR = maxNoOfIteration

        self.duration = duration
        self.fillP()
        self.fillG()
        self.fillA()


        self.q = np.zeros((1, self.samples))
        # self.q = self.G.flatten()

        # self.q = np.array([0.0, 0.0, 0.0])


        # for i in range(self.numJoints):
        self.getStartAndEnd()

        self.fillbounds()
        self.fillBoundsforG()
        self.fillEqualityConstraintforA()



    def getStartAndEnd(self):
        self.start = self.joint["start"]
        self.end = self.joint["end"]

    def fillbounds(self):

        # self.lb = np.zeros((1, self.samples))
        # self.ub = np.zeros((1, self.samples))

        self.lb = np.full((1, self.P.shape[0]), self.joint["lower_joint_limit"])
        self.ub = np.full((1, self.P.shape[0]), self.joint["upper_joint_limit"])



        # self.lb = self.lb.flatten()
        # self.ub = self.ub.flatten()


    def fillBoundsforG(self):

        max_vel = self.joint["max_velocity"]

        min_vel = self.joint["min_velocity"]

        self.lbG = np.full((1, self.G.shape[0]), min_vel * self.duration / (self.samples - 1))
        self.ubG = np.full((1, self.G.shape[0]), max_vel * self.duration / (self.samples - 1))

            # self.ubA[0, i] = max_vel / self.duration

        # self.lbG[0, self.G.shape[0] - 2] = self.start
        # self.lbG[0, self.G.shape[0] - 1] = self.end
        #
        # self.ubG[0, self.G.shape[0] - 2] = self.start
        # self.ubG[0, self.G.shape[0] - 1] = self.end

        # self.lbA = np.array([min_vel / self.duration, min_vel / self.duration, self.start, self.end])
        # self.ubA = np.array([max_vel / self.duration, max_vel / self.duration, self.start, self.end])




        # self.lbG = self.lbG.flatten()
        # self.ubG = self.ubG.flatten()

        # print self.lbA
        # print self.ubA

    def fillEqualityConstraintforA(self):



        self.b = np.zeros((1, 2))

        self.b[0,0] = self.joint["start"]

        self.b[0,1] = self.joint["end"]

            # self.ubA[0, i] = max_vel / self.duration

        # self.lbG[0, self.G.shape[0] - 2] = self.start
        # self.lbG[0, self.G.shape[0] - 1] = self.end
        #
        # self.ubG[0, self.G.shape[0] - 2] = self.start
        # self.ubG[0, self.G.shape[0] - 1] = self.end

        # self.lbA = np.array([min_vel / self.duration, min_vel / self.duration, self.start, self.end])
        # self.ubA = np.array([max_vel / self.duration, max_vel / self.duration, self.start, self.end])




        # self.lbG = self.lbG.flatten()
        # self.ubG = self.ubG.flatten()

        # print self.lbA
        # print self.ubA

    def fillP(self):
        self.P = np.zeros((self.samples, self.samples))
        np.fill_diagonal(self.P, 2.0)
        self.hShape = self.P.shape
        self.P[0, 0] = 1.0
        self.P[self.hShape[0] - 1, self.hShape[0] - 1] = 1.0
        self.P[np.arange(self.hShape[0] - 1), np.arange(self.hShape[0] - 1) + 1] = -2.0



        # self.P = 2.0 * self.P

    def fillA(self):
        self.A = np.zeros((2, self.samples))

        # self.A[0, self.A.shape[0] - 2] = 1
        # self.A[1, self.A.shape[1] - 1] = 1

        self.A[0, 0] = 1.0
        self.A[1, self.A.shape[1] - 1] = 1.0



        # self.P = 2 * self.P



    def fillG(self):
        self.G = np.zeros((self.samples , self.samples))
        # np.fill_diagonal(self.A, 2.0)
        self.aShape = self.G.shape
        # # self.A[0, 0] = -1.0
        # # self.A[self.aShape[0] - 1, self.aShape[0] - 1] = -1.0
        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1)] = -1

        self.G[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) + 1] = 1

        # self.A[np.arange(self.aShape[0] - 1) + 1, np.arange(self.aShape[0] - 1) + 1] = -1
        # self.A[np.arange(self.aShape[0] - 1) , np.arange(self.aShape[0] - 1) + 1] = -1


        self.G.resize(self.samples - 1, self.samples)
        #
        # Q = np.zeros((self.samples -1 , self.samples))
        #
        # Q[0, 0] = 1.0
        #
        # Q[1 , self.samples - 1] = 1.0
        #
        # self.A = vstack([self.A,  Q])
        # self.A.resize(self.samples + 1, self.samples)




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

        print "nwsr"
        print self.nWSR



    def solveQp(self):
        # options = Options()
        # options.printLevel = PrintLevel.MEDIUM

        # qp = QProblem(self.H.shape[0], self.A.shape[0])
        # qp.setOptions(options)

        # H = np.array([1.0, 0.0, 0.0, 0.5]).reshape((2, 2))
        # A = np.array([1.0, 1.0]).reshape((2, 1))
        # g = np.array([1.5, 1.0])
        # lb = np.array([0.5, -2.0])
        # ub = np.array([5.0, 2.0])
        # lbA = np.array([-1.0])
        # ubA = np.array([2.0])
        #
        # # Setup data of second QP.
        #
        # g_new = np.array([1.0, 1.5])
        # lb_new = np.array([0.0, -1.0])
        # ub_new = np.array([5.0, -0.5])
        # lbA_new = np.array([-2.0])
        # ubA_new = np.array([1.0])
        #
        # # Setting up QProblem object.
        # C = vstack([self.G, self.A, self.A])

        # print "self.H.shape[0], self.A.shape[0]", self.H.shape[0], self.A.shape[0]
        example = QProblem(self.P.shape[0], self.G.shape[0])
        options = Options()
        options.printLevel = PrintLevel.HIGH
        example.setOptions(options)

        # print "before init"
        # self.display()
        self.G = self.G.flatten()

        # TODO: check return value for error code; throw exception if unsuccessful
        example.init(self.P, self.G, self.G, self.lb, self.ub, self.lbG, self.ubG, np.array([self.nWSR]))

        # print "before hotstart"

        self.G = self.G.flatten()
        # TODO: check return value for error code; throw exception if unsuccessful
        print 'foo'

        self.display()
        example.hotstart(self.G, self.lb, self.ub, self.lbG, self.ubG, np.array([self.nWSR]))

        return example, self.P.shape[0]


    # def solveQp(self, H, G, A, lb, ub, lbA, ubA, nWSR):
    #     # options = Options()
    #     # options.printLevel = PrintLevel.MEDIUM
    #
    #     # qp = QProblem(self.H.shape[0], self.A.shape[0])
    #     # qp.setOptions(options)
    #
    #     # H = np.array([1.0, 0.0, 0.0, 0.5]).reshape((2, 2))
    #     # A = np.array([1.0, 1.0]).reshape((2, 1))
    #     # g = np.array([1.5, 1.0])
    #     # lb = np.array([0.5, -2.0])
    #     # ub = np.array([5.0, 2.0])
    #     # lbA = np.array([-1.0])
    #     # ubA = np.array([2.0])
    #     #
    #     # # Setup data of second QP.
    #     #
    #     # g_new = np.array([1.0, 1.5])
    #     # lb_new = np.array([0.0, -1.0])
    #     # ub_new = np.array([5.0, -0.5])
    #     # lbA_new = np.array([-2.0])
    #     # ubA_new = np.array([1.0])
    #     #
    #     # # Setting up QProblem object.
    #     # C = vstack([self.G, self.A, self.A])
    #
    #     # print "self.H.shape[0], self.A.shape[0]", self.H.shape[0], self.A.shape[0]
    #     example = QProblem(self.H.shape[0], self.A.shape[0])
    #     options = Options()
    #     options.printLevel = PrintLevel.NONE
    #     example.setOptions(options)
    #
    #     # print "before init"
    #     # self.display()
    #
    #     # TODO: check return value for error code; throw exception if unsuccessful
    #     example.init(H, G, A, lb, ub, lbA, ubA, nWSR)
    #
    #     # print "before hotstart"
    #     # self.display()
    #
    #     self.A = self.A.flatten()
    #     # TODO: check return value for error code; throw exception if unsuccessful
    #     example.hotstart(G, A, lb, ub, lbA, ubA, nWSR)
    #
    #     return example, self.H.shape[0]

