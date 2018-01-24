from cvxpy import *

duration = 5
samples = 50
joints = 7
x = Variable((samples, joints))
pro = []
cost = 0
constr = []
min_vel = -10
max_vel = 10
lower_limit = -2.96
upper_limit = 2.96
problem = []
# for t in range((samples - 1)):
#     for i in range(joints):
#         cost += cvxpy.sum_squares(x[i, t+1] - x[i, t])
#         constr += [x[i:, t + 1] - x[i:, t] <= max_vel * duration / (samples - 1),
#                      min_vel * duration / (samples - 1) <= x[i:, t + 1] - x[i:, t]]
#         pro = cvxpy.Problem(cvxpy.Minimize(cost), constr)
#         problem.append(pro)
#
# prob = cost
#
# constraints = constr
# constraints += [lower_limit <= x, x<= upper_limit]
# constraints += [x[0, 0] == 0.2,
#                x[0, samples-1] == 0.7,
#                 x[1, 0] == 0.3,
#                 x[1, samples - 1] == 0.9]
#
# problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
# problem.solve(solver=cvxpy.SCS, verbose=False)
# print prob.value
# print x.value

for i in range(joints):
    for t in range((samples - 1)):
        cost += cvxpy.sum_squares(x[t + 1, i] - x[t, i])
        constr += [x[t + 1, i] - x[t, i] <= max_vel * duration / (samples - 1),
                     min_vel * duration / (samples - 1) <= x[t + 1, i] - x[t, i]]
        pro = cvxpy.Problem(cvxpy.Minimize(cost), constr)
        problem.append(pro)

prob = cost

constraints = constr
constraints += [lower_limit <= x, x<= upper_limit]
constraints += [x[0, 0] == 0.2, x[samples - 1, 0] == 0.7,
                x[0, 1] == 0.3, x[samples - 1, 1] == 0.9,
                x[0, 2] == 0.5, x[samples - 1, 2] == 0.6,
                x[0, 3] == 0.1, x[samples - 1, 3] == 0.8,
                x[0, 4] == 0.3, x[samples - 1, 4] == 0.7,
                x[0, 5] == 0.1, x[samples - 1, 5] == 0.5,
                x[0, 6] == 0.2, x[samples - 1, 6] == 0.6,
                ]

problem = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
problem.solve(solver=cvxpy.SCS, verbose=False)
print prob.value
print x.value



