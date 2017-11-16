import numpy as np
from cvxpy import *

samples = 3
duration = 6
joints = 1
samples -= 1
x = Variable((joints, samples))
min_vel = -0.1
max_vel = 0.1
min_joint_limit = -0.3
max_joint_limit = 1.1
objective = []
constraints = []
states = []
cost = 0
# for t in range(samples-1):
#     for i in range(joints):
#
#         cost += sum_squares(sum_squares(x[i,t+1] - x[i, t]))
#
#         # states.append( Problem(Minimize(cost), [x <= 0.3, x >= -0.3] ) )
# # sums problem objectives and concatenates constraints.
# # prob = sum(states)
# prob  = Problem(Minimize(cost))
# prob.constraints.append([x[:,samples-1] == 0.7, x[:,0] == 0.2])
# prob.solve()
# print x.value
# print prob
for i in range(joints):
    print i
    for t in range(samples-1):
        print "t", t
        objective += (x[i,t+1] **2 + x[i, t] **2 - 2* (x[i,t+1] * x[i, t]))
        print x[i,t+1], x[i, t]
        constraints += [x[i, t + 1] - x[i, t] <= max_vel * duration / (samples - 1),
                   min_vel * duration / (samples - 1) <= x[i:, t + 1] - x[i:, t]]
constraints += [min_joint_limit <= x, x <= max_joint_limit]
x_0 = np.array([[0.2]])
x_1 = [[0.7]]

constraints += [x[0, duration-1] == x_1, x[0, 0] == x_0,
                x[1, duration -1] == 0.9, x[1, 0] == 0.3
                ]
objective +=  1e-08 * np.eye(samples)
print objective
problem = Problem(Minimize(objective))
problem.solve(solver=cvxpy.OSQP, verbose= True)
print x.value
print problem.get_problem_data(cvxpy.OSQP)
print problem.get_problem_data(cvxpy.OSQP)[0]["P"].todense()
print problem.get_problem_data(cvxpy.OSQP)[0]["q"]