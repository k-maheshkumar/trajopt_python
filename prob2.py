from sqpproblem import SQPproblem as sqp
from trajPlanner import trajPlanner1
import numpy as np
from qpoases import PySolutionAnalysis as SolutionAnalysis

request = {
    "samples" : 23,
    "duration" : 50,
    "maxIteration" : 1000,
    "joints" : [
        {"start": 0.2, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        {"start": 0.3, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.3, 'xOPt': 0.21, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.5, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.3, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.5, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.3, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.7, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        #
        # {"start": 0.1, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.4, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},




    ]
}
# request = {'duration': 10,
#            'joints':
#                [{'start': -0.04, 'end': -0.014, 'max_velocity': 15, 'xOPt': 3.65e-05, 'upper_joint_limit': 2.96, 'min_velocity': -15, 'id': 0, 'lower_joint_limit': -2.9670},
#                 {'start': -0.02,'end': -0.01, 'max_velocity': 15, 'xOPt': 0.00043, 'upper_joint_limit': 2.094, 'min_velocity': -15, 'id': 1, 'lower_joint_limit': -2.094},
#                 {'start': -0.05,'end': -0.016, 'max_velocity': 15, 'xOPt': -0.0021, 'upper_joint_limit': 2.969, 'min_velocity': -15, 'id': 2, 'lower_joint_limit': -2.967},
#                 # {'start': -0.01,'end': -2.350, 'max_velocity': 15, 'xOPt': -1.57091, 'upper_joint_limit': 2.094, 'min_velocity': -15, 'id': 3, 'lower_joint_limit': -2.094},
#                 # {'start': -0.05,'end': -0.014, 'max_velocity': 15, 'xOPt': 1.3125e-01, 'upper_joint_limit': 2.967, 'min_velocity': -15, 'id': 4, 'lower_joint_limit': -2.96},
#                 # {'start': -0.07,'end': -0.251, 'max_velocity': 15, 'xOPt': -1.036, 'upper_joint_limit': 2.094, 'min_velocity': -15, 'id': 5, 'lower_joint_limit': -2.094},
#                 # {'start': -0.06,'end': 0.028, 'max_velocity': 15, 'xOPt': 0.00029, 'upper_joint_limit': 3.054, 'min_velocity': -15, 'id': 6, 'lower_joint_limit': -3.054}
#                 ],
#            'maxIteration': 1000,
#            'samples': 15}



sp = trajPlanner1.TrajectoryPlanner(request)

# sp = trajPlanner.solveQp()

sp.displayProblem()
#
example, num = sp.solveQp()
# print num
xOpt = np.zeros(num)
example.getPrimalSolution(xOpt)

# analyser = SolutionAnalysis()
# maxStat = np.zeros(1)
# maxFeas = np.zeros(1)
# maxCmpl = np.zeros(1)
# analyser.getKktViolation(example, maxStat, maxFeas, maxCmpl)
# print("maxStat: %e, maxFeas:%e, maxCmpl: %e\n"%(maxStat, maxFeas, maxCmpl))

print "solution"
# print xOpt , example.getObjVal()

numJoints = len(request["joints"])
print "numJoints", numJoints
print np.split(xOpt, numJoints)
# print("\nxOpt = [ %e, %e, %e ];  objVal = %e\n\n"%(xOpt[0],xOpt[1],xOpt[2],example.getObjVal()))
# example.printOptions()