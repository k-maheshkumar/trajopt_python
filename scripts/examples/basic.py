import time

import cvxpy
import numpy as np

from scripts.Planner import Planner

request = {
    "samples" : 5,
    "duration" : 6,
    "max_iteration" : 1000,
    "max_penalty": 500,
    "max_delta": 5,
    "joints" : [
        # { "end": 0.7, 'initialGuess': 0.2, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},

        {"start": 0.2, "end": 0.7,  "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        {"start": 0.4, "end": 0.9,  "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.3, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},

    ]
}

temp = 1
plan = Planner.TrajectoryOptimizationPlanner(request, None, temp)
start = time.time()
# sp.displayProblem()
# x_0 = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
x_0 = np.full((1, request["samples"] * len(request["joints"])), 1.0).flatten()
# plan.displayProblem()
prob = plan.get_trajectory(x_0, temp)
end = time.time()
print("cvxopt",end - start)

