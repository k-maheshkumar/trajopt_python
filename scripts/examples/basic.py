import time

import cvxpy
import numpy as np

from scripts.Planner import Planner as planner

request = {
    "samples" : 5,
    "duration" : 6,
    "max_iteration" : 1000,
    "max_penalty": 500,
    "max_delta": 5,
    # "joints" : [
    #     # { "end": 0.7, 'initialGuess': 0.2, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1,
    #     #  "min_velocity": -0.1, "max_velocity": 0.1},
    #
    #     {"start": 0.2, "end": 0.7,  "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity" : 0.1},
    #     {"start": 0.4, "end": 0.9,  "lower_joint_limit": -0.4, "upper_joint_limit": 1.1, "min_velocity": -0.1,  "max_velocity": 0.1},
    #     # {"start": 0.5, "end": 0.9, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
    #     # {"start": 0.1, "end": 0.3, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
    #     # {"start": 0.2, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
    #     # {"start": 0.5, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
    #     # {"start": 0.3, "end": 0.6, "lower_joint_limit": -0.3, "upper_joint_limit": 1.1, "min_velocity": -0.1, "max_velocity": 0.1},
    #
    # ],
    "joints" : [
        {   "name": "lbr_iiwa_joint_1",
            "states":{"start": -0.49197958189616936, "end": -2.0417782994426674},
            # "start": -0.49, "end": -2.04,
            "lower_joint_limit": -2.96705972839, "upper_joint_limit": 2.96705972839,
            "min_velocity": -10.0, "max_velocity" : 10},
        # {"name": "lbr_iiwa_joint_2",
        #  "start": 1.4223062659337982, "end": 0.9444594031189716,
        #  "lower_joint_limit": -2.09439510239, "upper_joint_limit": 2.09439510239,
        #  "min_velocity": -10.0, "max_velocity": 10},
        # {"name": "lbr_iiwa_joint_3",
        #  "start": -1.5688299779644697, "end": -1.591006403858707,
        #  "lower_joint_limit": -2.96705972839, "upper_joint_limit": 2.96705972839,
        #  "min_velocity": -10.0, "max_velocity": 10},
        # {"name": "lbr_iiwa_joint_4",
        #  "start": -1.3135004031364736, "end": -1.9222844444479184,
        #  "lower_joint_limit": -2.09439510239, "upper_joint_limit": 2.09439510239,
        #  "min_velocity": -10.0, "max_velocity": 10},
        # {"name": "lbr_iiwa_joint_5",
        #  "start": 1.5696229411153653, "end": 1.572303282659756,
        #  "lower_joint_limit": -2.96705972839, "upper_joint_limit": 2.96705972839,
        #  "min_velocity": -10.0, "max_velocity": 10},
        # {"name": "lbr_iiwa_joint_6",
        #  "start": 1.5749627479696093, "end": 1.5741716208788483,
        #  "lower_joint_limit": -2.09439510239, "upper_joint_limit": 2.09439510239,
        #  "min_velocity": -10.0, "max_velocity": 10},
        # {"name": "lbr_iiwa_joint_7",
        #  "start": 1.5708037563007493, "end": 1.5716145442929421,
        #  "lower_joint_limit": -3.05432619099, "upper_joint_limit": 3.05432619099,
        #  "min_velocity": -10.0, "max_velocity": 10}
    ]
}

temp = 1
plan = planner.TrajectoryOptimizationPlanner(problem=request, solver="SCS", solver_class=temp)
start = time.time()
# sp.displayProblem()
# x_0 = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
x_0 = np.full((1, request["samples"] * len(request["joints"])), 1.0).flatten()
# plan.displayProblem()
prob = plan.get_trajectory()
end = time.time()
print("computation time: ",end - start)

