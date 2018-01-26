from cvxpy import *
import time
import scripts.sqp_solver.SQPsolver as solver
import  numpy as np
duration = 5
samples = 5
joints = 1
x = Variable((samples, joints))
pro = []
cost = 0
constraints = []
lower_limit = -2.96705972839
upper_limit = 2.96705972839
problem = []
min_vel = -10
max_vel = 10
min_vel *= duration / float(samples - 1)
max_vel *= duration / float(samples - 1)

for t in range((samples - 1)):
    for i in range(joints):
        cost += cvxpy.sum_squares(x[t + 1, i] - x[t, i])
        cost += (x[t + 1, i] - x[t, i]) ** 2
        constraints += [x[t + 1, i] - x[t, i] <= max_vel ,
                        min_vel <= x[t + 1, i] - x[t, i]]
        # pro = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        problem.append(cost)

cost = cvxpy.sum(problem)


# print cost
constraints += [lower_limit <= x, x<= upper_limit]
constraints += [x[0, 0] == -0.49197958189616936, x[samples - 1, 0] == -2.0417782994426674,
                # x[0, 1] == 0.3, x[samples - 1, 1] == 0.9,
                # x[0, 2] == 0.5, x[samples - 1, 2] == 0.6,
                # x[0, 3] == 0.1, x[samples - 1, 3] == 0.8,
                # x[0, 4] == 0.3, x[samples - 1, 4] == 0.7,
                # x[0, 5] == 0.1, x[samples - 1, 5] == 0.5,
                # x[0, 6] == 0.2, x[samples - 1, 6] == 0.6,
                ]


# for i in range(joints):
#     for t in range((samples - 1)):
#         cost += cvxpy.sum_squares(x[t + 1, i] - x[t, i])
#         constr += [x[t + 1, i] - x[t, i] <= max_vel * duration / (samples - 1),
#                    min_vel * duration / (samples - 1) <= x[t + 1, i] - x[t, i]]
#         pro = cvxpy.Problem(cvxpy.Minimize(cost), constr)
#         problem.append(pro)
#
# prob = cost
#
# constraints = constr
# constraints += [lower_limit <= x, x <= upper_limit]
# constraints += [x[0, 0] == 0.2, x[samples - 1, 0] == 0.7,
#                 x[0, 1] == 0.3, x[samples - 1, 1] == 0.9,
#                 x[0, 2] == 0.5, x[samples - 1, 2] == 0.6,
#                 x[0, 3] == 0.1, x[samples - 1, 3] == 0.8,
#                 x[0, 4] == 0.3, x[samples - 1, 4] == 0.7,
#                 x[0, 5] == 0.1, x[samples - 1, 5] == 0.5,
#                 x[0, 6] == 0.2, x[samples - 1, 6] == 0.6,
#                 ]

problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
start_cvx = time.time()
problem.solve(solver=cvxpy.ECOS, verbose=False)
end_cvx = time.time()
# print problem.get_problem_data(cvxpy.OSQP)[0]
# print problem.get_problem_data(cvxpy.OSQP)[0].keys()

P = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["P"].todense())[:samples, :samples]
# P = np.array([[ 2., -2.,  0.,  0.,  0.],
#               [-2.,  4., -2.,  0.,  0.],
#               [ 0., -2.,  4., -2.,  0.],
#               [ 0.,  0., -2.,  4., -2.],
#               [ 0.,  0.,  0., -2.,  2.],
#                                 ])
q = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["q"])[-samples:]
A = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["A"].todense())[-joints*2:, -samples:]
G = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["F"].todense())[::, -samples:]
ubG = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["G"])
b = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["b"])[-joints*2:]
lbG = np.zeros(ubG.shape)[-samples:]



ubG = ubG[::2]
G1 = G[:(samples*2)-2:2, :]
G2 = G[-samples:, :]
G = np.vstack([G1, G2])

# print G1

# print  P
# print x.value
# print A.todense()

# P = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["P"].todense())
# q = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["q"])
# A = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["A"].todense())
# G = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["F"].todense())
# ubG = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["G"])
# b = np.asarray(problem.get_problem_data(cvxpy.OSQP)[0]["b"])
# lbG = np.zeros(ubG.shape)


sqp_solver = solver.SQPsolver()
sqp_solver.init(P=P, q=q, G=G, A=A, b=b, lbG=None, ubG=ubG)
start = time.time()
sqp_solver.display_problem()
_, x_k = sqp_solver.solve()
end = time.time()

# sqp_solver.display_problem()
# print x.T.value


# print "objective value 1", cost.value

print "SCS solver time: ", end_cvx - start_cvx
print "SQP solver time: ", end - start

# x_k = np.asarray(x_k[-(samples * joints):]).reshape((joints, samples))
print x_k
#
# print x.shape, x_k.shape
# x.value = x_k
# print x_k.T
# print "objective value 2", cost.value

# start = time.time()
# problem.solve(solver=cvxpy.OSQP, verbose=False)
# end = time.time()
# print "OSQP solver time: ", end - start
# start = time.time()
# problem.solve(solver=cvxpy.ECOS, verbose=False)
# end = time.time()
# print "ECOS solver time: ", end - start
# start = time.time()
# problem.solve(solver=cvxpy.ECOS_BB, verbose=False)
# end = time.time()
# print "ECOS_BB solver time: ", end - start
# print prob.value
# print x.value



