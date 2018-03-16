from urdf_parser_py.urdf import URDF
from scripts.Planner import Planner1 as planner
import logging
from collections import OrderedDict

class Robot:
    def __init__(self, urdf_file):
        self.model = URDF.from_xml_file(urdf_file)
        self.planner = planner.TrajectoryOptimizationPlanner()
        self.logger = logging.getLogger("Trajectory_Planner." + __name__)

    def get_trajectory(self):
        return self.planner.trajectory

    def get_initial_trajectory(self):
        return self.planner.trajectory.initial

    def init_plan_trajectory(self, *args, **kwargs):
        joints = OrderedDict()
        status = "-1"

        if "group" in kwargs:
            joint_group = kwargs["group"]
        if "samples" in kwargs:
            samples = kwargs["samples"]
        if "duration" in kwargs:
            duration = kwargs["duration"]
        if "solver" in kwargs:
            solver = kwargs["solver"]
        else:
            solver = "SCS"
        if "solver_config" in kwargs:
            solver_config = kwargs["solver_config"]
            if solver_config is not None:
                if "decimals_to_round" in solver_config:
                    decimals_to_round = int(solver_config["decimals_to_round"])
            else:
                decimals_to_round = 5
        else:
            decimals_to_round = 5

        if "current_state" in kwargs:
            current_state = kwargs["current_state"]
        if "goal_state" in kwargs:
            goal_state = kwargs["goal_state"]

        if "current_state" in kwargs and "goal_state" in kwargs:
            states = OrderedDict()
            for joint_in_group in joint_group:
                if joint_in_group in current_state and joint_in_group in goal_state and \
                    joint_in_group in self.model.joint_map:
                    states[joint_in_group] = OrderedDict([("start", current_state[joint_in_group]),
                                                          ("end", goal_state[joint_in_group])])

                    joints[joint_in_group] = OrderedDict([("states", states[joint_in_group]),
                                                          ("limit", self.model.joint_map[joint_in_group].limit)])
        if len(joints):
            self.planner.init(joints=joints, samples=samples, duration=duration, joint_group=joint_group,
                              solver=solver, solver_config=solver_config, solver_class=1,
                              decimals_to_round=decimals_to_round, verbose=True)

    def calulate_trajecotory(self, callback_function=None):
        status, can_execute_trajectory = "No trajectory has been found", False
        status, can_execute_trajectory = self.planner.calculate_trajectory(callback_function=callback_function)
        return status, can_execute_trajectory

    # def get_robot_trajectory(self):
    #     return self.planner.trajectory.get_trajectory()