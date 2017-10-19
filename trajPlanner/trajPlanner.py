import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from sqpproblem import SQPproblem as sqp

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

    def __init__(self, problem, solver):
        self.problem = problem
        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.maxNoOfIteration = problem["maxIteration"]
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])
        self.solver = solver

        self.sqp = []

        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbG = []
        self.ubG = []
        self.b = []
        self.C = []


        if self.joints[0].has_key("xOPt"):
            self.xOPtInitial = []
            self.isXOptAvailable = True
        else:
            self.isXOptAvailable = False

        for i in range(self.numJoints):
            sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration)
            # sp.display()




            self.sqp.append(sp)
            # sp.display()
            # if self.joints[i].has_key("xOPt") :
            #     self.xOPtInitial.append(self.joints[i]["xOPt"])
            # print "h " + str(i)

            # print self.sqp[i].H

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)
            # print  "G in iteration"
            # print self.G
            # self.G.append(self.sqp[i].G)
            # self.A.append(self.sqp[i].A)


            if solver == "osqp":
                self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.samples)]))
                # self.lb.append(self.sqp[i].lb.tolist())
                self.lb.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
                # self.ub.append(self.sqp[i].ub.tolist())
                self.ub.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))

            else:
                self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A]))
                # print np.array([self.sqp[i].lbG[0][0]])
                self.lb.append(np.hstack([self.sqp[i].lbG]))
                self.lb = np.dstack([self.lb, self.sqp[i].lbG[0][0]])
                # self.ub.append(self.sqp[i].ub.tolist())
                self.ub.append(np.hstack([self.sqp[i].ubG]))
                self.ub = np.dstack([self.ub, self.sqp[i].ubG[0][0]])


            self.lbG.append(self.sqp[i].lbG.tolist())
            self.ubG.append(self.sqp[i].ubG.tolist())
            self.b.append(self.sqp[i].b.tolist())
            # sp.display()
            # print (self.lb)


        # print self.G
        # self.q = self.q[0]
        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.ub = np.hstack(self.ub)

        self.P = self.diag_block_mat_slicing(self.P)
        # self.q = self.diag_block_mat_slicing(self.q)

        # self.A = self.diag_block_mat_slicing(self.A)
        # self.G = self.diag_block_mat_slicing(self.G)
        self.C = self.diag_block_mat_slicing(self.C)
        # print self.C


        # self.lb = [item for sublist in self.lb for item in sublist]
        # self.ub = [item for sublist in self.ub for item in sublist]
        self.lbG = [item for sublist in self.lbG for item in sublist]
        self.ubG = [item for sublist in self.ubG for item in sublist]
        self.b = [item for sublist in self.b for item in sublist]

        # self.q = [item for sublist in self.q for item in sublist]

        self.lb = np.asarray(self.lb)
        self.ub = np.asarray(self.ub)
        self.lbG = np.asarray(self.lbG)
        self.ubG = np.asarray(self.ubG)
        self.b = np.asarray(self.b)

        self.q = np.asarray(self.q)
        # print "q.shape", self.q.shape
        # self.H = self.H.astype(float)
        # self.q = self.q.astype(float)
        # self.G = self.G.astype(float)


        self.C = self.C.astype(float)

        self.P = 2.0 * self.P




        # self.A = 1.0 * self.A
        # self.G = 1.0 * self.G
        # self.G = self.G.flatten()


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
        print "P"
        print self.P
        print "q"
        print self.q
        print "G"
        print self.C
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

        print "maxNoOfIteration"
        print self.maxNoOfIteration

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
        qp = QProblem(self.P.shape[0], self.q.shape[0])
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
            status =  qp.init(self.P, self.G, self.q, self.lb, self.ub, self.lbG, self.ubG, np.array([self.maxNoOfIteration]), 0.0, np.array([self.xOPtInitial]).flatten())
        else:
            status = qp.init1(self.P, self.G, self.q, self.lb, self.ub, self.lbG, self.ubG,
                              np.array([self.maxNoOfIteration]))
        # if (status == 0):
        print "init status: ", status

        # print "before hotstart"
        # self.display()

        self.q = self.q.flatten()
        # TODO: check return value for error code; throw exception if unsuccessful
        status = qp.hotstart(self.G, self.lb, self.ub, self.lbG, self.ubG, np.array([self.maxNoOfIteration]))

        print "hotstart status: ", status

        # assert type(status) is type(int), "Cannot solver for given number of duration"

        # if not (status):
        #     print "Cannot solver for given number of duration"

        # status  = qp.getSilmpleStatus();


        return qp, self.P.shape[0]


    def solveProblem(self):

        if self.solver == "osqp":
            from qpsolvers.qpsolvers import osqp_ as qp

            from scipy.sparse import csc_matrix

            # print "before call", self.q
            sol = qp.osqp_solve_qp1(csc_matrix(self.P), self.q, csc_matrix(self.C), self.lb, self.ub, self.lbG,
                                    self.ubG, None, self.b,
                                    initvals=None,
                                    max_wsr=np.array([self.maxNoOfIteration]))

            print np.split(sol, self.numJoints)
        else:
            from qpsolvers.qpsolvers import qpoases_ as qp
            print "before call", self.lbG
            qp = qp.qpoases_solve_qp(self.P, self.q, self.C, self.lb, self.ub, self.lbG, self.ubG, None, self.b, initvals=None,
                                      max_wsr=np.array([self.maxNoOfIteration]))

            x_opt = np.zeros(self.P.shape[0])
            ret = qp.getPrimalSolution(x_opt)
            if ret != 0:  # 0 == SUCCESSFUL_RETURN code of qpOASES
                warn("qpOASES failed with return code %d" % ret)

            print "numJoints", self.numJoints
            print np.split(x_opt, self.numJoints)


            # example = QProblem(self.P.shape[0], self.q.shape[0])
            # options = Options()
            # options.printLevel = PrintLevel.LOW
            # example.setOptions(options)

            # print "before init"
            # self.display()
            # g = g.flatten()


            # example.init(self.P, self.q.flatten(), self.C, self.lb.flatten(), self.ub.flatten(), self.lbG.flatten(), ubA.flatten(), np.array([1000]))
            # # example.hotstart(A.flatten(), lb, ub, lbA, ubA, np.array([1000]))
            # xOpt = np.zeros(g.shape[0])
            # example.getPrimalSolution(xOpt)
            # print xOpt
        # print sol


    def solveQpProb1(self):
        # import osqp
        # from scipy.sparse import csc_matrix
        #
        # m = osqp.OSQP()
        # m.setup(P=csc_matrix(self.P), q=self.q, A=csc_matrix(self.A), l=self.l, u=self.u, max_iter=10000)
        # results = m.solve()
        # print results.x

        from qpsolvers.qpsolvers import osqp_ as qp

        from scipy.sparse import csc_matrix

        # print "before call", self.q
        sol = qp.osqp_solve_qp1(csc_matrix(self.P), self.q, csc_matrix(self.C), self.lb, self.ub, self.lbG, self.ubG, None, self.b,
                                 initvals=None,
                                 max_wsr=np.array([self.maxNoOfIteration]))

        print np.split(sol, self.numJoints)