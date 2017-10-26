from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
import numpy as np

# QP matrices
H = 2 * np.array([[ 1., -2.,  0.],
                  [ 0.,  2., -2.],
                  [ 0.,  0.,  1.]])
g = np.array([0., 0., 0.])
A = np.array([[-1.,  1.,  0.],
              [ 0., -1.,  1.],
              [ 1.,  0.,  0.],
              [ 0.,  0.,  1.]])

lb = np.array([-0.3, -0.3, -0.3])
ub = np.array([ 1.1,  1.1,  1.1])
lbA = np.array([-0.3, -0.3,  0.2,  0.7])
ubA = np.array([ 0.3,  0.3,  0.2,  0.7])


example = QProblem(H.shape[0], A.shape[0])
options = Options()
options.printLevel = PrintLevel.LOW
example.setOptions(options)

# print "before init"
# self.display()
# g = g.flatten()

print (H, g, A, lb, ub, lbA, ubA)
status = example.init(H, g, A, lb, ub, lbA, ubA, np.array([1000]))
print("status", status)
# example.hotstart(A.flatten(), lb, ub, lbA, ubA, np.array([1000]))
xOpt = np.zeros(g.shape[0])
example.getPrimalSolution(xOpt)
print xOpt