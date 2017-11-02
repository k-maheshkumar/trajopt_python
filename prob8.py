from cvxopt import matrix, solvers
import numpy as np
p = 2*matrix([ [2, -2, 0], [0, 4, -2], [0, 0, 2] ])
Q = matrix([0.0, 0.0, 0.0])
Q = np.array(Q).transpose()
Q = matrix(Q)
G = matrix([[0.0,0.0, 0.0]])
h = matrix([1.1, 1.1, 1.1])
A = matrix([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
b = matrix([0.7])
sol=solvers.qp(Q, p, G, h, A, b)
print(sol['x'])