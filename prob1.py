from sqpproblem import SQPproblem as sqp
import numpy as np

request = {
    "samples" : 3,
    "duration" : 4,
    "maxIteration" : 100,
    "joints" : [
        {"start": 0.2, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity" : -0.1, "max_velocity" : 0.1}
    ]
}

sp = sqp.SQPproblem(request)
sp.display()

example, num = sp.solveQp()
# print num
xOpt = np.zeros(num)
example.getPrimalSolution(xOpt)

print "solution"
print xOpt , example.getObjVal()

# print("\nxOpt = [ %e, %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],xOpt[2],example.getObjVal()))
# example.printOptions()