import logging
from scripts.utils.utils import Utils as utils
from urdf_parser_py.urdf import URDF
from scripts.Robot.Planner import TrajectoryPlanner
import itertools
from scripts.Robot.ModelandTree import RobotTree
from srdfdom.srdf import SRDF
from scripts.utils.dict import DefaultOrderedDict

class Robot:
    def __init__(self, logger_name=__name__, verbose=False, log_file=False):
        self.id = -1
        self.model = None
        self.planner = TrajectoryPlanner(logger_name, verbose, log_file)
        self.logger = logging.getLogger(logger_name + __name__)
        self.ignored_collisions = DefaultOrderedDict(bool)
        self.srdf = None
        utils.setup_logger(self.logger, logger_name, verbose, log_file)



    def load_robot_model(self, urdf_file=None):
        if urdf_file is not None:
            self.model = URDF.from_xml_file(urdf_file)
        else:
            self.model = URDF.from_parameter_server()

        # base_link, end_link = "lbr_iiwa_link_0", "lbr_iiwa_link_7"

        # self.tree = RobotTree(self.model, base_link, end_link)

    def load_srdf(self, srdf_file):

        stream = open(srdf_file, 'r')
        self.srdf = SRDF.from_xml_string(stream.read())

    def get_ignored_collsion(self):
        for collision in self.srdf.disable_collisionss:
            self.ignored_collisions[collision.link1, collision.link2] = True
            self.ignored_collisions[collision.link2, collision.link1] = True

        return self.ignored_collisions

    def get_trajectory(self):
        return self.planner.trajectory

    def get_initial_trajectory(self):
        return self.planner.trajectory.initial

    def init_plan_trajectory(self, *args, **kwargs):
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
            # current_state = OrderedDict(zip(joint_group, current_state))

        if "goal_state" in kwargs:
            goal_state = kwargs["goal_state"]
            # goal_state = OrderedDict(zip(joint_group, goal_state))


        if "collision_safe_distance" in kwargs:
            collision_safe_distance = kwargs["collision_safe_distance"]
        else:
            collision_safe_distance = 0.05

        if "collision_check_distance" in kwargs:
            collision_check_distance = kwargs["collision_check_distance"]
        else:
            collision_check_distance = 0.1

        if "verbose" in kwargs:
            verbose = kwargs["verbose"]
        else:
            verbose = False

        if verbose:
            main_logger_name = "Trajectory_Planner"
            self.logger = logging.getLogger(main_logger_name)
            self.setup_logger(main_logger_name, verbose)

        if "current_state" in kwargs and "goal_state" in kwargs:
            if type(current_state) is dict and type(current_state) is dict:
                states = {}
                joints = {}
                for joint_in_group in joint_group:
                    if joint_in_group in current_state and joint_in_group in goal_state and \
                                    joint_in_group in self.model.joint_map:
                        if self.model.joint_map[joint_in_group].type != "fixed":
                            states[joint_in_group] = {"start": current_state[joint_in_group],
                                                      "end": goal_state[joint_in_group]}
                            joints[joint_in_group] = {
                                "states": states[joint_in_group],
                                "limit": self.model.joint_map[joint_in_group].limit,
                            }
            elif type(current_state) is list and type(current_state) is list:
                joints = []
                assert len(current_state) == len(goal_state) == len(joint_group)
                for joint, current_state, next_state in itertools.izip(joint_group, current_state, goal_state):
                    if joint in self.model.joint_map:
                        joints.append([current_state, next_state, self.model.joint_map[joint].limit])
        if len(joints):
            self.planner.init(joints=joints, samples=samples, duration=duration,
                              joint_group=joint_group,
                              collision_safe_distance=collision_safe_distance,
                              collision_check_distance=collision_check_distance,
                              solver=solver, solver_config=solver_config,
                              solver_class=1, decimals_to_round=decimals_to_round, verbose=verbose)

    def calulate_trajecotory(self, callback_function=None):
        status, can_execute_trajectory = "No trajectory has been found", False
        status, planning_time, can_execute_trajectory = self.planner.calculate_trajectory(callback_function=callback_function)
        return status, planning_time, can_execute_trajectory

    # def get_robot_trajectory(self):
    #     return self.planner.trajectory.get_trajectory()


