import scripts.Planner.Planner1 as planner
import collections

if __name__ == '__main__':
    request = collections.OrderedDict()
    request["samples"] =  50
    request["duration"] = 6
    request["joints"] = {
            "lbr_iiwa_joint_1": {
                "states": {"start": -0.49197958189616936, "end": -2.0417782994426674},
                # "start": -0.49, "end": -2.04,
                "limit": {"lower": -2.96705972839, "upper": 2.96705972839, "velocity": 10},
            },
            "lbr_iiwa_joint_2": {
                "states": {"start": 1.4223062659337982, "end": 0.9444594031189716},
                # "start": -0.49, "end": -2.04,
                "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            },
            # "lbr_iiwa_joint_3": {
            #     "states": {"start": 1.5223062659337982, "end": 1.9444594031189716,},
            #
            #  "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            # },
            # "lbr_iiwa_joint_4":{
            #     "states": {"start": -1.3135004031364736, "end": -1.9222844444479184,},
            #
            #  "lower_joint_limit": -2.09439510239, "upper_joint_limit": 2.09439510239,
            #  "min_velocity": -10.0, "max_velocity": 10,
            #     "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            #
            # },
            # "lbr_iiwa_joint_5":{
            #     "states": {"start": 1.5696229411153653, "end": 1.572303282659756,},
            #
            #      "lower_joint_limit": -2.96705972839, "upper_joint_limit": 2.96705972839,
            #      "min_velocity": -10.0, "max_velocity": 10,
            #     "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            #
            # },
            # "lbr_iiwa_joint_6":{
            #     "states": {"start": 1.5749627479696093, "end": 1.5741716208788483,},
            #
            #      "lower_joint_limit": -2.09439510239, "upper_joint_limit": 2.09439510239,
            #      "min_velocity": -10.0, "max_velocity": 10,
            #         "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            #
            # },
            # "lbr_iiwa_joint_7":{
            #     "states": {"start": 1.5708037563007493, "end": 1.5716145442929421,},
            #
            #      "lower_joint_limit": -3.05432619099, "upper_joint_limit": 3.05432619099,
            #      "min_velocity": -10.0, "max_velocity": 10,
            #     "limit": {"lower": -2.09439510239, "upper": 2.09439510239, "velocity": 10},
            #
            # }
        }


    # prob = Problem_modelling(request)
    # # print prob.cost_matrix
    # print prob.velocity_matrix
    plan = planner.TrajectoryOptimizationPlanner()
    plan.init(problem=request, solver_class=0)
    plan.calculate_trajectory()
    # print plan.get_trajectory().trajectory
    # print plan.get_trajectory().trajectory_by_name
