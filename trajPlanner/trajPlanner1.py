import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from sqpproblem import SQPproblem1 as sqp

from qpoases import PyBooleanType as BooleanType
from qpoases import PySubjectToStatus as SubjectToStatus
class TrajectoryPlanner:

    def diag_block_mat_slicing(self,L):
        shp = L[0].shape
        N = len(L)
        r = range(N)
        out = np.zeros((N, shp[0], N, shp[1]), dtype=int)
        out[r, :, r, :] = L
        return out.reshape(np.asarray(shp) * N)

    def __init__(self, problem):
        self.problem = problem
        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.maxNoOfIteration = problem["maxIteration"]
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])

        self.sqp = []

        self.H = []
        self.G = []
        self.A = []
        self.lb = []
        self.ub = []
        self.lbA = []
        self.ubA = []
        if self.joints[0].has_key("xOPt"):
            self.xOPtInitial = []
            self.isXOptAvailable = True
        else:
            self.isXOptAvailable = False

        for i in range(self.numJoints):
            sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration)
            self.sqp.append(sp)
            # sp.display()
            if self.joints[i].has_key("xOPt") :
                self.xOPtInitial.append(self.joints[i]["xOPt"])
            # print "h " + str(i)

            # print self.sqp[i].H

            self.H.append(self.sqp[i].H)
            self.A.append(self.sqp[i].A)
            self.G.append(self.sqp[i].G)

            self.lb.append(self.sqp[i].lb.tolist())
            self.ub.append(self.sqp[i].ub.tolist())
            self.lbA.append(self.sqp[i].lbA.tolist())
            self.ubA.append(self.sqp[i].ubA.tolist())

        # print self.G
        self.H = self.diag_block_mat_slicing(self.H)
        self.A = self.diag_block_mat_slicing(self.A)
        self.G = self.diag_block_mat_slicing(self.G)

        self.lb = [item for sublist in self.lb for item in sublist]
        self.ub = [item for sublist in self.ub for item in sublist]
        self.lbA = [item for sublist in self.lbA for item in sublist]
        self.ubA = [item for sublist in self.ubA for item in sublist]

        self.lb = np.asarray(self.lb)
        self.ub = np.asarray(self.ub)
        self.lbA = np.asarray(self.lbA)
        self.ubA = np.asarray(self.ubA)

        # self.H = self.H.astype(float)
        self.A = self.A.astype(float)
        self.G = self.G.astype(float)

        self.H = 2.0 * self.H
        # self.A = 1.0 * self.A
        # self.G = 1.0 * self.G
        self.G = self.G.flatten()


        # print self.xOPtInitial

        # example, num = sp.solveQp()
            # # print num
            # xOpt = np.zeros(num)
            # example.getPrimalSolution(xOpt)

            # print "solution"
            # print xOpt, example.getObjVal()



    def getStartAndEnd(self, index):
        self.start = self.joints[index]["start"]
        self.end = self.joints[index]["end"]


    def displayProblem(self):
        print "H"
        print self.H

        print "A"
        print self.A

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

        # print "self.H.shape[0], self.A.shape[0]", self.H.shape[0], self.A.shape[0]
        qp = QProblem(self.H.shape[0], self.A.shape[0])
        options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.LOW
        qp.setOptions(options)

        # print "before init"
        # self.display()

        # temp =  np.asarray(self.xOPtInitial)

        # print temp

        # TODO: check return value for error code; throw exception if unsuccessful

        if self.isXOptAvailable:
            status =  qp.init1(self.H, self.G, self.A, self.lb, self.ub, self.lbA, self.ubA,np.array([self.maxNoOfIteration]), 0.0, np.array([self.xOPtInitial]).flatten())
        else:
            status = qp.init(self.H, self.G, self.A, self.lb, self.ub, self.lbA, self.ubA,
                             np.array([self.maxNoOfIteration]))
        # if (status == 0):
        print "init status: ", status

        # print "before hotstart"
        # self.display()

        self.A = self.A.flatten()
        # TODO: check return value for error code; throw exception if unsuccessful
        status = qp.hotstart(self.G, self.lb, self.ub, self.lbA, self.ubA, np.array([self.maxNoOfIteration]))

        print "hotstart status: ", status

        # assert type(status) is type(int), "Cannot solver for given number of duration"

        # if not (status):
        #     print "Cannot solver for given number of duration"

        # status  = qp.getSilmpleStatus();


        return qp, self.H.shape[0]


