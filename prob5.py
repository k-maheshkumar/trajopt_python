import osqp
import numpy as np
from scipy.sparse import csc_matrix

P = np.array([
    [1., - 2.,  0.],
    [0.,  2., - 2.],
    [0.,  0., 1.]])
q = np.array([ 0.,  0.,  0.])
# G = np.array([
#     [-1.,  1.,  0.],
#     [ 0., -1.,  1.]]).reshape((2,3))
l = np.array([-0.3, -0.3,  0.2,  0.7])
u = np.array([0.3,  0.3,  0.2,  0.7])
lb = np.array([-0.3, -0.3, -0.3])
ub = np.array([1.1, 1.1,  1.1])
# A = np.array([
#     [1., 0., 0.],
#     [0., 0., 1.]]).reshape((2,3))
A = np.array([[-1.,  1.,  0.],
              [0., -1.,  1.],
              [ 1.,  0.,  0.],
              [ 0.,  0.,  1.]])
I =  np.identity(3)
A = np.vstack([A, I])
l = np.hstack([l, lb])
u = np.hstack([u, ub])

P_csc = csc_matrix(2 * P)
# G_csc = csc_matrix(G)
A_csc = csc_matrix(A)
# initialX = np.array([0.1, 0.2, 0.5])

# from qpsolvers.qpsolvers import osqp_ as qp
#
# sol = qp.osqp_solve_qp(P_csc, q, G_csc, h, A, b, initvals=None)
#
# print sol
#
P_csc = .5 * (P_csc + P_csc.transpose())
m = osqp.OSQP()
m.setup(P=P_csc, q=q, A=A_csc, l=l, u=u, max_iter = 10000)
# m.warm_start(x=initialX)

results = m.solve()
print "P"
print P_csc.todense()
print "q"
print q
print "A"
print A_csc.todense()
# print "lb"
# print lb
# print "ub"
# print ub
print "lbA"
print l
print "ubA"
print u

print results.x