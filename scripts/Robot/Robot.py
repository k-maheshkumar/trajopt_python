from urdf_parser_py.urdf import URDF
from scripts.TrajectoryOptimizationPlanner import Planner as planner
import time
from easydict import EasyDict as edict
import munch
import collections
import logging
# import kdl_parser_py as kdl_parser

import kdl_parser_py.urdf
from scripts.utils.utils import Utils as utils
from scripts.utils.dict import DefaultOrderedDict

class Robot:
    def __init__(self, urdf_file=None, logger_name=__name__, verbose=False, log_file=False):
        self.id = -1
        if urdf_file is not None:
            self.model = URDF.from_xml_file(urdf_file)
            status, self.kdl_tree =  kdl_parser_py.urdf.treeFromFile(urdf_file)
        else:
            self.model = URDF.from_parameter_server()
            status, self.kdl_tree = kdl_parser_py.urdf.treeFromUrdfModel(self.model, quiet=True)

        if not status:
            print("cannot parse urdf to create kdl tree")

        self.__setup_get_joint_by_name()
        self.planner = planner.OptimizationPlanner(logger_name, verbose, log_file)
        logger_name = logger_name + __name__
        self.logger = logging.getLogger(logger_name)
        utils.setup_logger(self.logger, logger_name, verbose, log_file)

    def get_trajectory(self):
        return self.planner.trajectory

    def get_initial_trajectory(self):
        return self.planner.trajectory.initial

    def __replace_joints_in_model_with_map(self):
        joints = collections.OrderedDict()
        for joint in self.model.joints:
            joints[joint.name] = joint
        del self.model.joints[:]
        self.model.joints = joints

    def __setup_get_joint_by_name(self):
        joints = collections.OrderedDict()
        for joint in self.model.joints:
            joints[joint.name] = joint
        self.model.joint_by_name = joints

    def init_plan_trajectory(self, *args, **kwargs):
        joints = collections.OrderedDict()
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
            solver_config = None
            decimals_to_round = 5

        if "current_state" in kwargs:
            current_state = kwargs["current_state"]

        if "goal_state" in kwargs:
            goal_state = kwargs["goal_state"]

        if "collision_safe_distance" in kwargs:
            collision_safe_distance = kwargs["collision_safe_distance"]
        else:
            collision_safe_distance = 0.05

        if "collision_check_distance" in kwargs:
            collision_check_distance = kwargs["collision_check_distance"]
        else:
            collision_check_distance = 0.01


        if "current_state" in kwargs and "goal_state" in kwargs:

            states = collections.OrderedDict()
            for joint in self.model.joints:
                for joint_in_group in joint_group:
                    if joint_in_group in current_state and joint_in_group in goal_state:
                        states[joint_in_group] = collections.OrderedDict([('start', current_state[joint_in_group]),
                                                  ('end', goal_state[joint_in_group])])
                    if joint.name == joint_in_group and joint.limit is not None:
                        joints[joint.name] = collections.OrderedDict([('states', states[joint_in_group]),
                                                                      ('limit', joint.limit)])

        if len(joints):
            self.planner.init(joints=joints, samples=samples, duration=duration,
                              joint_group=joint_group,
                              collision_safe_distance=collision_safe_distance,
                              collision_check_distance=collision_check_distance,
                              solver=solver, solver_config=solver_config,
                              solver_class=1, decimals_to_round=decimals_to_round)

    def calulate_trajecotory(self, callback_function=None):
        status, can_execute_trajectory = "No trajectory has been found", False
        status, can_execute_trajectory = self.planner.calculate_trajectory(robot_id=self.id,
                                                                           callback_function=callback_function)
        return status, can_execute_trajectory

    # def get_robot_trajectory(self):
    #     return self.planner.trajectory.get_trajectory()

    def init_robot_groups(self, groups):
        for group in groups:
            chain =  self.kdl_tree.getChain(group["base"], group["tip"])
            sub_group = []
            for i in range(chain.getNrOfSegments()):
                sub_group.append(chain.getSegment(i).getJoint().getName())
        self.planner.planning_groups[group["name"]] = sub_group

    def get_joints_of_group(self, group):
        return self.planner.planning_groups[group]

