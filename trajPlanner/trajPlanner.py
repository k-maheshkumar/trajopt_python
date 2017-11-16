import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from sqpproblem import SQPproblem as sqp
from warnings import warn

'''
        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbC <= C * x <= ubC
            # lb <= x <= ub
            # A * x == b

'''

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

        # self.check_input()

        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.maxNoOfIteration = problem["maxIteration"]
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])
        self.solver = solver
        self.penaltyMax = problem["penaltyMax"]
        self.deltaMax = problem["deltaMax"]

        self.sqp = []

        self.P = []
        self.G = []
        self.A = []
        self.q = []
        self.lb = []
        self.ub = []
        self.lbC = []
        self.ubC = []
        self.b = []
        self.C = []
        self.initialX = []

        if 'initialGuess' in self.joints[0]:
            self.initialGuess = []
            self.isInitialGuessAvailable = True
        else:
            self.isInitialGuessAvailable = False

        for i in range(self.numJoints):


            # sp.display()
            if self.joints[i].has_key("initialGuess") :
                sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration,
                                    self.joints[i]["initialGuess"])

                self.initialGuess.append(sp.initVals)
                self.initialX.append(self.interpolate1(sp.start, sp.end, self.samples))
                # self.initialGuessInitial.append(np.full((1, 3), self.joints[i]["initialGuess"]))
            else:
                sp = sqp.SQPproblem(self.samples, self.duration, self.joints[i], self.maxNoOfIteration)

            self.sqp.append(sp)



            # print "h " + str(i)

            # print self.sqp[i].H

            self.P.append(self.sqp[i].P)
            self.q.append(self.sqp[i].q)
            # self.G.append(self.sqp[i].G)
            # self.A.append(self.sqp[i].A)

            if solver == "qpoases":
                self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A]))
                # self.C.append(self.sqp[i].G)
                self.A.append(self.sqp[i].A)

                # print np.array([self.sqp[i].lbG[0][0]])
                # self.lb.append(np.hstack([self.sqp[i].lbG]))
                self.lb.append(self.sqp[i].lb)

                # self.lb = np.append(self.lb, self.sqp[i].lbG[0][0])
                # self.ub.append(self.sqp[i].ub.tolist())
                self.ub.append(np.hstack([self.sqp[i].ub]))
                # self.ub = np.dstack([self.ub, self.sqp[i].ub[0][0]])
            else:
                self.C.append(np.vstack([self.sqp[i].G, self.sqp[i].A, self.sqp[i].A, np.identity(self.samples)]))
                # self.lb.append(self.sqp[i].lb.tolist())
                self.lb.append(np.hstack([self.sqp[i].lbG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].lb]))
                # self.ub.append(self.sqp[i].ub.tolist())
                self.ub.append(np.hstack([self.sqp[i].ubG, self.sqp[i].b, self.sqp[i].b, self.sqp[i].ub]))



            self.lbC.append(self.sqp[i].lbG.tolist())
            self.ubC.append(self.sqp[i].ubG.tolist())
            self.b.append(self.sqp[i].b.tolist())
            # sp.display()
            # print (self.lb)

        # print self.C
        # print self.A

        # print self.G
        # self.q = self.q[0]

        self.initialX = np.hstack(self.initialX).tolist()


        self.q = np.hstack(self.q)
        self.lb = np.hstack(self.lb)
        self.ub = np.hstack(self.ub)
        # print  self.initialGuess
        # print  len(self.initialGuess)
        self.P = self.diag_block_mat_slicing(self.P)
        # self.q = self.diag_block_mat_slicing(self.q)

        if solver == "qpoases":
            self.A = self.diag_block_mat_slicing(self.A)
        # self.G = self.diag_block_mat_slicing(self.G)
        self.C = self.diag_block_mat_slicing(self.C)
        # print self.C


        # self.lb = [item for sublist in self.lb for item in sublist]
        # self.ub = [item for sublist in self.ub for item in sublist]
        self.lbC = [item for sublist in self.lbC for item in sublist]
        self.ubC = [item for sublist in self.ubC for item in sublist]
        self.b = [item for sublist in self.b for item in sublist]

        # self.q = [item for sublist in self.q for item in sublist]

        self.lb = np.asarray(self.lb)
        self.ub = np.asarray(self.ub)
        self.lbC = np.asarray(self.lbC)
        self.ubC = np.asarray(self.ubC)
        self.b = np.asarray(self.b)

        self.q = np.asarray(self.q)
        # if self.joints[i].has_key("initialGuess"):
        if 'initialGuess' in self.joints[i]:
            self.initialGuess = np.hstack(self.initialGuess)
            self.initialGuess = np.asarray(self.initialGuess)

        else:
            self.initialGuess = None
        # print "q.shape", self.q.shape
        # self.H = self.H.astype(float)
        # self.q = self.q.astype(float)
        # self.G = self.G.astype(float)


        self.C = self.C.astype(float)

        self.P = 2.0 * self.P

        self.P_model = .5 * (self.P + self.P.T) + 1e-08 * np.eye(self.P.shape[0])



        # self.A = 1.0 * self.A
        # self.G = 1.0 * self.G
        # self.G = self.G.flatten()


        # print self.initialGuess

        # example, num = sp.solveQp()
            # # print num
            # initialGuess = np.zeros(num)
            # example.getPrimalSolution(initialGuess)

            # print "solution"
            # print initialGuess, example.getObjVal()

    # def check_input(self):
    #     print "bhfgd"
    #     if 'start' in self.problem:
    #         print "bhfgd"
    #         warn("need key: start")
    #         exit(1)

    def getStartAndEnd(self, index):
        self.start = self.joints[index]["start"]
        self.end = self.joints[index]["end"]


    def displayProblem(self):
        print ("P")
        print (self.P)
        print ("q")
        print (self.q)
        print ("G")
        print (self.C)
        print ("lb")
        print (self.lb)
        print ("ub")
        print (self.ub)
        print ("lbG")
        print (self.lbC)
        print ("ubG")
        print (self.ubC)
        print ("b")
        print (self.b)
        print ("A")
        print (self.A)

        print ("maxNoOfIteration")
        print (self.maxNoOfIteration)


    def solveProblem(self):

        if self.solver == "osqp":
            from qpsolvers import osqp_ as qp

            from scipy.sparse import csc_matrix

            # print "before call", self.q
            result = qp.osqp_solve_qp1(csc_matrix(self.P), self.q, csc_matrix(self.C), self.lb, self.ub, self.lbC,
                                    self.ubC, None, self.b,
                                    initvals= self.initialGuess,
                                    max_wsr=np.array([self.maxNoOfIteration]))

            # print np.split(sol, self.numJoints)
            sol = np.split(result.x, self.numJoints)
            return  result, sol


        elif self.solver == "osqp1":
            from qpsolvers import qpoases_ as qp
            qp = qp.qpoases_solve_qp(self.P, self.q, self.C, self.lb, self.ub, self.lbC, self.ubC, self.A, self.b, initvals=None,
                                      max_wsr=np.array([self.maxNoOfIteration]))

            x_opt = np.zeros(self.P.shape[0])
            ret = qp.getPrimalSolution(x_opt)

            if ret != 0:  # 0 == SUCCESSFUL_RETURN code of qpOASES
                warn("qpOASES failed with return code %d" % ret)

            print "numJoints", self.numJoints
            print np.split(x_opt, self.numJoints)
        elif self.solver == "osqp2":
            from qpsolvers import cvxopt_ as qp

            qp = qp.cvxopt_solve_qp1(self.P, self.q, self.C, self.lb, self.ub, self.lbC, self.ubC, self.A, self.b,
                                     initvals=None)
        else:
            from qpsolvers import osqp_ as qp

            result = qp.solve_with_cvxpy(self.P, self.q, self.C, self.lb, self.ub, self.lbC,
                                       self.ubC, None, self.b,
                                       initvals=self.initialGuess,
                                       max_wsr=np.array([self.maxNoOfIteration]), solver= self.solver)


    def solveQpProb1(self):
        # import osqp
        # from scipy.sparse import csc_matrix
        #
        # m = osqp.OSQP()
        # m.setup(P=csc_matrix(self.P), q=self.q, A=csc_matrix(self.A), l=self.l, u=self.u, max_iter=10000)
        # results = m.solve()
        # print results.x

        from qpsolvers import osqp_ as qp

        from scipy.sparse import csc_matrix

        # print "before call", self.q
        sol = qp.osqp_solve_qp1(csc_matrix(self.P), self.q, csc_matrix(self.C), self.lb, self.ub, self.lbC, self.ubC, None, self.b,
                                 initvals=None,
                                 max_wsr=np.array([self.maxNoOfIteration]))

        print (np.split(sol, self.numJoints))

    def interpolate(self, start, end, samples):

        data = []
        for i in range(samples - 1):
            percentage = 1 / (i + 1.0)
            data.append((percentage * start) + ((1 - percentage) * end))
        data.append(end)

        return np.round(data, 3)

    def interpolate1(self, start, end, samples):

        # data1 = []
        # for i in range(samples - 1):
        #     percentage = 1 / (i + 1.0)
        #     data1.append((percentage * start) + ((1 - percentage) * end))
        # data1.append(end)

        # return np.round(data1, 3)
        data = []
        stepSize = (end - start) / (samples -1)
        intermediate = start
        data.append(start)

        for i in range(samples-1):
            intermediate += stepSize
            data.append(intermediate)
        return np.round(data, 3)


    def evaluate_objective(self, p, x):

        if x is not  None:
            x = x.reshape((x.shape[0],1))
            obj = np.matmul(x.T,p)
            obj = np.matmul(obj, x)
            return obj[0, 0]

    def evaluate_Constraints(self, A, x):
        x = x.reshape((6,1))
        cons = np.matmul(A, x)

        # return obj[0, 0]






    def solveSQP(self):
        convexify = True
        trustregion = True
        mu = 0.01
        delta = 2
        INFINITY = 1e+100
        improve_ratio_threshold_ = .25;
        min_trust_box_size_ = 1e-4;
        min_model_improve = 1e-4;
        min_model_improve_frac = -INFINITY;
        max_iter_ = 50;
        trust_shrink_ratio_ = .1;
        trust_expand_ratio_ = 1.5;
        cnt_tolerance_ = 1e-4;
        max_merit_coeff_increases_ = 100;
        merit_coeff_increase_ratio_ = 10;
        max_time_ = INFINITY;

        merit_error_coeff_ = 10;
        trust_box_size_ = 1e-1;
        trustregionSize = 0.1
        x = self.initialX
        delta = 0.01
        # print self.penaltyMax
        # from qpsolvers import osqp_ as qp
        # prob, x_new = qp.solve_with_cvxpy(self.P_model, self.q, self.C, self.lb, self.ub, self.lbC,
        #                                   self.ubC, None, self.b,
        #                                   initvals=x,
        #                                   max_wsr=np.array([self.maxNoOfIteration]), solver=self.solver, mu=mu,
        #                                   delta=delta)
        # model_obj_x = self.evaluate_objective(self.P_model, np.array(x))
        # model_obj_xK = self.evaluate_objective(self.P_model, x_new)
        # modelImprove = model_obj_x- model_obj_xK
        # actual_obj_x = self.evaluate_objective(self.P, np.array(x))
        # actual_obj_xk = self.evaluate_objective(self.P, np.array(x_new))
        # trueImprove = actual_obj_x - actual_obj_xk
        # rhoK = trueImprove / modelImprove
        # print modelImprove, trueImprove, rhoK

        # trustregion = False
        # while mu <= self.penaltyMax - 400:
        #     # while convexify:
        #     # while trust_box_size_ >= min_trust_box_size_:
        #     from qpsolvers import osqp_ as qp
        #     # print trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5
        #     prob, x_new = qp.solve_with_cvxpy(self.P, self.q, self.C, self.lb, self.ub, self.lbC,
        #                                 self.ubC, None, self.b,
        #                                 initvals=x,
        #                                 max_wsr=np.array([self.maxNoOfIteration]), solver=self.solver, mu= mu,
        #                                 delta=delta)
        #     # trustregion = False
        #     # trust_box_size_ *= np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5);
        #     trust_box_size_ = np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5);
        #     # convexify = False
        #     print "trust_box_size_", trust_box_size_
        #     self.evaluate_objective(x_new)
        #
        #     x = x_new
        #     mu = mu * 10
        #     delta = delta * 10
        #     # print mu
        #
        # print "x_new: ", x_new
        # return x_new

        merit_increases = 1
        iterationCount = 0
        # x_new = x
        while merit_increases < max_merit_coeff_increases_:
            merit_increases *= 10
            iterationCount = 0

            print "merit increases .. . .."
            while iterationCount <= max_iter_:
                while trust_box_size_ >= min_trust_box_size_:
                    iterationCount += 1
                    from qpsolvers import osqp_ as qp
                    import cvxpy

                    # print trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5
                    prob, x_k = qp.solve_with_cvxpy(self.P_model, self.q, self.C, self.lb, self.ub, self.lbC,
                                                      self.ubC, None, self.b,
                                                      initvals=x,
                                                      max_wsr=np.array([self.maxNoOfIteration]), solver=self.solver, mu= merit_increases,
                                                      delta=trust_box_size_, verbose= False)

                    if prob.status == cvxpy.INFEASIBLE or prob.status == cvxpy.INFEASIBLE_INACCURATE or prob.status == cvxpy.UNBOUNDED or prob.status == cvxpy.UNBOUNDED_INACCURATE:
                        x_new = x
                        print prob.status
                        break

                        # Todo: throw error when problem is not solved
                    else:
                        x_new = x_k
                        model_obj_x = self.evaluate_objective(self.P_model, np.array(x))
                        model_obj_xK = self.evaluate_objective(self.P_model, x_new)
                        modelImprove = model_obj_x - model_obj_xK
                        actual_obj_x = self.evaluate_objective(self.P, np.array(x))
                        actual_obj_xk = self.evaluate_objective(self.P, np.array(x_new))
                        trueImprove = actual_obj_x - actual_obj_xk
                        rhoK = trueImprove / modelImprove
                        # print modelImprove, trueImprove, rhoK
                        print  rhoK, prob.status
                        print "x_new",x_new


                        if rhoK < 0.25:
                            print "shrinking ....."
                            trust_box_size_ = 0.25 * trust_box_size_
                        else:
                            if rhoK > 0.75:
                                print "expanding... .. ... . .", iterationCount
                                # delta = min(2 * delta, )
                                trust_box_size_ = 2 * trust_box_size_
                                # trust_box_size_ = np.fmax(trust_box_size_, min_trust_box_size_ / trust_shrink_ratio_ * 1.5)

                                x = x_new
                        # if modelImprove < min_model_improve:
                        #     print "les"
                        #     break
                        if iterationCount >= max_iter_:
                            print "last print ",trust_box_size_, merit_increases
                            break

        print x
        return prob

