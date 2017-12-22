from collections import namedtuple
from munch import *

class obj(object):
    def __init__(self, d):
        for a, b in d.items():
            if isinstance(b, (list, tuple)):
                setattr(self, a, [obj(x) if isinstance(x, dict) else x for x in b])
            else:
                setattr(self, a, obj(b) if isinstance(b, dict) else b)

request = {
    "samples": 5,
    "duration": 6,
    "max_iteration": 1000,
    "max_penalty": 500,
    "max_delta": 5,
    "joints": [
        {"name": "lbr_iiwa_joint_1",
         "states": {"start": -0.49197958189616936, "end": -2.0417782994426674},
         # "start": -0.49, "end": -2.04,
         "lower_joint_limit": -2.96705972839, "upper_joint_limit": 2.96705972839,
         "min_velocity": -10.0, "max_velocity": 10},

    ]
}


