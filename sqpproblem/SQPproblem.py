import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from numpy import array, hstack, ones, vstack, zeros


class SQPproblem:
    def __init__(self, problem):
        self.problem = problem
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])
        self.samples = problem["samples"]
        # self.jointsVelocities = problem["jointsVelocities"]


        self.nWSR = np.array([problem["maxIteration"]])

        self.duration = problem["duration"]
        self.fillH()
        self.fillA()

        self.G = np.zeros((1, self.samples))
        self.G = self.G.flatten()

        # self.G = np.array([0.0, 0.0, 0.0])


        self.getStartAndEnd(0)

        self.fillbounds(0)
        self.fillBoundsforA(0)



    def getStartAndEnd(self, index):
        self.start = self.joints[index]["start"]
        self.end = self.joints[index]["end"]

    def fillbounds(self, index):

        self.lb = np.zeros((1, self.samples))
        self.ub = np.zeros((1, self.samples))

        print self.joints

        for i in range(self.lb.shape[1]):
            self.lb[0, i] = self.joints[index]["lower_joint_limit"]

        for i in range(self.ub.shape[1]):

            self.ub[0, i] = self.joints[index]["upper_joint_limit"]


        # lb = np.array([min_pos, min_pos, min_pos])
        # ub = np.array([max_pos, max_pos, max_pos])


        self.lb = self.lb.flatten()
        self.ub = self.ub.flatten()


    def fillBoundsforA(self, index):
        # print self.A.shape
        self.lbA = np.zeros((1, self.A.shape[0]))
        self.ubA = np.zeros((1, self.A.shape[0]))

        # max_vel = self.jointsVelocities[index]["upper_joint_limit"]
        #
        # min_vel = self.jointsVelocities[index]["lower_joint_limit"]

        max_vel = self.joints[index]["max_velocity"]

        min_vel = self.joints[index]["min_velocity"]

        # print self.joints
        # for i in range(self.A.shape[0] - 2):
        #     self.lbA[0, i] = self.jointsVelocities[index]["lower_joint_limit"]
        #     self.lbA[0, i] = self.jointsVelocities[index]["lower_joint_limit"]
        #
        #     self.ubA[0, i] = self.jointsVelocities[index]["upper_joint_limit"]
        #     self.ubA[0, i] = self.jointsVelocities[index]["upper_joint_limit"]
        #
        # self.lbA[0, self.A.shape[0] - 2] = self.joints[index]["lower_joint_limit"]
        # self.lbA[0, self.A.shape[0] - 1] = self.joints[index]["lower_joint_limit"]
        #
        # self.ubA[0, self.A.shape[0] - 2] = self.joints[index]["upper_joint_limit"]
        # self.ubA[0, self.A.shape[0] - 1] = self.joints[index]["upper_joint_limit"]


        for i in range(self.A.shape[0] - 2):
            self.lbA[0, i] = min_vel * self.duration / (self.samples - 1)
            self.ubA[0, i] = max_vel * self.duration / (self.samples - 1)

            # self.ubA[0, i] = max_vel / self.duration

        self.lbA[0, self.A.shape[0] - 2] = self.start
        self.lbA[0, self.A.shape[0] - 1] = self.end

        self.ubA[0, self.A.shape[0] - 2] = self.start
        self.ubA[0, self.A.shape[0] - 1] = self.end

        # self.lbA = np.array([min_vel / self.duration, min_vel / self.duration, self.start, self.end])
        # self.ubA = np.array([max_vel / self.duration, max_vel / self.duration, self.start, self.end])




        self.lbA = self.lbA.flatten()
        self.ubA = self.ubA.flatten()

        # print self.lbA
        # print self.ubA



    def fillH(self):
        self.H = np.zeros((self.samples, self.samples))
        np.fill_diagonal(self.H, 2.0)
        self.hShape = self.H.shape
        self.H[0, 0] = 1.0
        self.H[self.hShape[0] - 1, self.hShape[0] - 1] = 1.0
        self.H[np.arange(self.hShape[0]- 1), np.arange(self.hShape[0]- 1) + 1] = -2


        self.H = 2 * self.H



    def fillA(self):
        self.A = np.zeros((self.samples , self.samples))
        # np.fill_diagonal(self.A, 2.0)
        self.aShape = self.A.shape
        # # self.A[0, 0] = -1.0
        # # self.A[self.aShape[0] - 1, self.aShape[0] - 1] = -1.0
        self.A[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) ] = -1

        self.A[np.arange(self.aShape[0] - 1), np.arange(self.aShape[0] - 1) + 1] = 1

        # self.A[np.arange(self.aShape[0] - 1) + 1, np.arange(self.aShape[0] - 1) + 1] = -1
        # self.A[np.arange(self.aShape[0] - 1) , np.arange(self.aShape[0] - 1) + 1] = -1


        self.A.resize(self.samples -1, self.samples)

        Q = np.zeros((self.samples -1 , self.samples))

        Q[0, 0] = 1.0

        Q[1 , self.samples - 1] = 1.0

        self.A = vstack([self.A,  Q])
        self.A.resize(self.samples + 1, self.samples)




    def display(self):
        # print "Problem ", self.problem
        print "H"
        print self.H
        print "A"
        print self.A
        #
        print "G"

        print self.G
        print "lb"
        print self.lb
        print "ub"
        print self.ub
        print "lbA"
        print self.lbA
        print "ubA"
        print self.ubA



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

        print "self.H.shape[0], self.A.shape[0]", self.H.shape[0], self.A.shape[0]
        example = QProblem(self.H.shape[0], self.A.shape[0])
        options = Options()
        options.printLevel = PrintLevel.NONE
        example.setOptions(options)

        print "before init"
        self.display()

        # TODO: check return value for error code; throw exception if unsuccessful
        example.init(self.H, self.G, self.A, self.lb, self.ub, self.lbA, self.ubA, self.nWSR)

        # print "before hotstart"
        # self.display()

        self.A = self.A.flatten()
        # TODO: check return value for error code; throw exception if unsuccessful
        example.hotstart(self.G, self.A, self.lb, self.ub, self.lbA, self.ubA, self.nWSR)

        return example, self.H.shape[0]

