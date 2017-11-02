# Generate data for control problem.
import numpy as np
np.random.seed(1)
n = 8
m = 2
T = 50
alpha = 0.2
beta = 5
A = np.eye(n) + alpha*np.random.randn(n,n)
B = np.random.randn(n,m)
x_0 = beta*np.random.randn(n,1)
# Form and solve control problem.
from cvxpy import *
x = Variable(n, T+1)
u = Variable(m, T)

states = []
for t in range(T):
    cost = sum_squares(x[:,t+1]) + sum_squares(u[:,t])
    constr = [x[:,t+1] == A*x[:,t] + B*u[:,t],
              norm(u[:,t], 'inf') <= 1]
    states.append( Problem(Minimize(cost), constr) )
# sums problem objectives and concatenates constraints.
prob = sum(states)
prob.constraints += [x[:,T] == 0, x[:,0] == x_0]
prob.solve()