import numpy as np
from sqpproblem import SQPproblem.SQPproblem as sqp
class SQPproblem:
    def __init__(self, problem):
        self.problem = problem
        self.samples = problem["samples"]
        self.duration = problem["duration"]
        self.nWSR = np.array([problem["maxIteration"]])
        self.joints = problem["joints"]
        self.numJoints = len(problem["joints"])


        for i in range(self.numJoints):
            sqp.()



    def getStartAndEnd(self, index):
        self.start = self.joints[index]["start"]
        self.end = self.joints[index]["end"]


