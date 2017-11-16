import numpy as np
from cvxpy import *
np.random.seed(1)
# n = 8
# m = 2
# T = 50
# alpha = 0.2
# beta = 5
# A = np.eye(n) + alpha*np.random.randn(n,n)
# B = np.random.randn(n,m)
# x_0 = beta*np.random.randn(n,1)
# # Form and solve control problem.
#
# x1 = Variable((n, T+1))
# u = Variable((m, T))

#
#
# states = []
# for t in range(T):
#     cost = sum_squares(x[:,t+1]) + sum_squares(u[:,t])
#     constr = [x1[:,t+1] == A*x1[:,t] + B*u[:,t],
#               norm(u[:,t], 'inf') <= 1]
#     states.append( Problem(Minimize(cost), constr) )
duration = 5
samples = 3
joints = 2
x = Variable((joints, duration+1))
pro = []
cost = 0
states = []
constr = []
min_vel = -0.3
max_vel = 0.3
# for t in range(duration):
#     for i in range(joints):
#
#         # cost.append(sum_squares(x[i,t+1] - x[i,t]))
#         cost += sum_squares(x[i, t + 1] - x[i, t])
#         # states.append(cost)
#         constr += [x[i:, t + 1] - x[i:, t ] <= max_vel * duration / (samples - 1),
#                    min_vel * duration / (samples - 1) <= x[i:, t + 1] - x[i:, t ]]
        # states.append( Problem(Minimize(cost)) )

for t in range(duration):
    for i in range(joints):

        # cost.append(sum_squares(x[i,t+1] - x[i,t]))
        cost += sum_squares(x[i, t + 1] - x[i, t])
        # states.append(cost)
        constr += [x[i:, t + 1] - x[i:, t ] <= max_vel * duration / (samples - 1),
                   min_vel * duration / (samples - 1) <= x[i:, t + 1] - x[i:, t ]]

# sums problem objectives and concatenates constraints.
prob = cost
constraints = [-0.3 <= x, x <= 1.1]
constraints += constr

x_0 = np.array([[0.2]])
x_1 = [[0.7]]

constraints += [x[0:, duration] == x_1, x[0:, 0] == x_0,
                # x[1:, duration] == 0.9, x[1:, 0] == 0.3
                ]
x.value = np.array([[0.2,         0.30000004,  0.39999965,  0.49999954,  0.59999994,  0.7],
                    [0.2,         0.3,         0.39999993,  0.49999988 , 0.60000017 , 0.7]])
prob = Problem(Minimize(prob), constraints)
prob.solve(solver= cvxpy.ECOS, verbose= True)
print x.value
print prob.value

# X = Variable((samples, duration+1))
#
# prob = Problem(Minimize(tv(X)), constraints)
# x_0 = np.array([[0.2]]).reshape(1,)
# x_1 = np.array([[0.7]]
#
# prob.constraints += [x[:, duration] == x_1, x[:, 0] == x_0]
# prob.solve()
#
# print X.value
