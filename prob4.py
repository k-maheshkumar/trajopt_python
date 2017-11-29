from sqpproblem import SQPproblem as sqp
from trajPlanner import trajPlanner
import time
import cvxpy
request = {
    "samples" : 3,
    "duration" : 6,
    "max_iteration" : 1000,
    "max_penalty": 500,
    "max_delta": 5,
    "joints" : [
        # { "end": 0.7, 'initialGuess': 0.2, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
        #  "min_velocity": -0.1, "max_velocity": 0.1},

        {"start": 0.2, "end": 0.7,  "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
        {"start": 0.3, "end": 0.9,  "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.1, "end": 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.2, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.5, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
        # {"start": 0.3, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},

    ]
}

sp = trajPlanner.TrajectoryPlanner(request, cvxpy.ECOS)
start = time.time()
# sp.displayProblem()
prob = sp.solveSQP()
end = time.time()
print("cvxopt",end - start)

