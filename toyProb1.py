import numpy as np
from qpoases import PyQProblem as QProblem
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel

#Setup data of first QP.
# H   = np.array([1, 0.0, 0.0, 1.5 ]).reshape((2,2))
# g   = np.array([-1.0, -1.0])
# A = np.array([1.0, 1.0]).reshape((2, 1))
# lb  = np.array([0.0, 0.0])
# ub  =  np.array([1000.0, 1000.0])
# lbA = np.array([6.0])
# ubA = np.array([1000.0])

# H   = np.array([1.0, -2.0, 0.0, 0.0,
#                 0.0, 2.0, -2.0, 0.0,
#                 0.0, 0.0, -2.0, -2.0,
#                 0.0, 0.0, 0.0, 1.0]).reshape((4,4))

H   = np.array([1.0, -1.0, 0.0, 0.0,
                0.0, 2.0, -1.0, 0.0,
                0.0, 0.0, -1.0, -1.0,
                0.0, 0.0, 0.0, 1.0]).reshape((4,4))

g   = np.array([0.0, 0.0, 0.0, 0.0])

lb = np.array([0.2, 0.2])
ub = np.array([0.7, 0.7])

A   = np.array([1.0, 0.0, 0.0, 0.0,
                0.0, 1.0, 0.0, 0.0,
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0,

               -1.0, 1.0, 0.0, 0.0,
                0.0, -1.0, 1.0, 0.0,
                1.0, 0.0, -1.0, -1.0,

                1.0, 0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, 1.0]).reshape((9,4))
dt = 0.4
max_vel = 0.1
start = 0.2
end = 0.7
lbA  = np.array([-max_vel/dt, -max_vel/dt, -max_vel/dt, start, end])
ubA   = np.array([max_vel/dt, max_vel/dt, -max_vel/dt, start, end])
# Setting up QProblem object.

example = QProblem(2, 1)
options = Options()
options.printLevel = PrintLevel.NONE
example.setOptions(options)

# Solve first QP.
nWSR = np.array([1000])
example.init(H, g, A, lb, ub, lbA, ubA, nWSR)

example.hotstart( g, lb, ub, lbA, ubA, nWSR)

xOpt = np.zeros(4)
example.getPrimalSolution(xOpt)
print xOpt, example.getObjVal()
# print("\nxOpt = [ %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],example.getObjVal()))
# example.printOptions()

print "H"
print H
print "A"
print A
#
print "G"

print g
print "lb"
print lb
print "ub"
print ub
print "lbA"
print lbA
print "ubA"
print ubA