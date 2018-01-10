import time
import numpy as np

from scripts.Planner import Planner1 as planner
import collections


request = collections.OrderedDict()
request["samples"] = 30
request["duration"] = 6
request["joints"] = collections.OrderedDict()
request["joints"] = {
    "lbr_iiwa_joint_1": {
        "states": {"start": -0.49197958189616936, "end": -2.0417782994426674},
        "limit": {"lower": -2.96705972839, "upper": 2.96705972839, "velocity": 10},
    },
    "lbr_iiwa_joint_2": {
        "states": {"start": 1.4223062659337982, "end": 0.9444594031189716},
            "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
    },
    "lbr_iiwa_joint_3": {
        "states": {"start": -1.5688299779644697, "end": -1.591006403858707},
        "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
    },
    "lbr_iiwa_joint_4": {
        "states": {"start": -1.3135004031364736, "end": -1.9222844444479184},
        "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
     },
    "lbr_iiwa_joint_5": {
        "states": {"start": 1.5696229411153653, "end": 1.572303282659756},
        "limit": {"lower": -2.96705972839, "upper": 2.96705972839, "velocity": 10}
    },
    "lbr_iiwa_joint_6": {
     "states": {"start": 1.5749627479696093, "end": 1.5741716208788483},
     "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
    },
    "lbr_iiwa_joint_7": {
        "states": {"start": 1.5708037563007493, "end": 1.5716145442929421},
        "limit": {"lower": -3.05432619099, "upper": 3.05432619099, "velocity": 10}
    }
}

temp = 1
plan = planner.TrajectoryOptimizationPlanner()

start = time.time()
plan.init(problem=request)
plan.calculate_trajectory()
end = time.time()
print("computation time: ",end - start)

