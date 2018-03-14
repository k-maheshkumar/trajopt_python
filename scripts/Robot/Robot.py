import logging

from easydict import EasyDict as edict
from urdf_parser_py.urdf import URDF
from scripts.Robot.Planner import TrajectoryPlanner

class Robot:
    def __init__(self, urdf_file):
        self.id = -1
        self.model = URDF.from_xml_file(urdf_file)
        self.__setup_get_joint_by_name()
        self.state = self.init_state()
        self.planner = TrajectoryPlanner()
        self.logger = logging.getLogger("Trajectory_Planner." + __name__)

    def init_state(self):
        state = {}
        for joint in self.model.joints:
            state[joint.name] = {
                "current_value": 0
            }
        state = edict(state)
        return state

    def get_state(self):
        return self.state

    def get_trajectory(self):
        return self.planner.trajectory

    def get_initial_trajectory(self):
        return self.planner.trajectory.initial

    def __replace_joints_in_model_with_map(self):
        joints = {}
        for joint in self.model.joints:
            joints[joint.name] = joint
        del self.model.joints[:]
        self.model.joints = joints

    def __setup_get_joint_by_name(self):
        joints = {}
        for joint in self.model.joints:
            joints[joint.name] = joint
        self.model.joint_by_name = joints

    def init_plan_trajectory(self, *args, **kwargs):
        joints = {}
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
            self.update_robot_state(kwargs["current_state"])
        if "goal_state" in kwargs:
            goal_state = kwargs["goal_state"]

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
            # verbose = False
            self.logger = logging.getLogger(main_logger_name)
            self.setup_logger(main_logger_name, verbose)
        # else:
        #     self.logger = logging.getLogger("Trajectory_Planner." + __name__)

        if "current_state" in kwargs and "goal_state" in kwargs:
            states = {}
            for joint in self.model.joints:
                for joint_in_group in joint_group:
                    if joint_in_group in self.state and joint_in_group in goal_state:
                        states[joint_in_group] = {"start": self.state[joint_in_group]["current_value"],
                                                  "end": goal_state[joint_in_group]}
                    if joint.name == joint_in_group and joint.limit is not None:
                        joints[joint.name] = edict({
                            "states": states[joint_in_group],
                            "limit": joint.limit,
                        })
        if len(joints):
            self.planner.init(joints=joints, samples=samples, duration=duration,
                              joint_group=joint_group,
                              collision_safe_distance=collision_safe_distance,
                              collision_check_distance=collision_check_distance,
                              solver=solver, solver_config=solver_config,
                              solver_class=1, decimals_to_round=decimals_to_round, verbose=verbose)

    def calulate_trajecotory(self, callback_function=None):
        status, can_execute_trajectory = "No trajectory has been found", False
        status, can_execute_trajectory = self.planner.calculate_trajectory(callback_function=callback_function)
        return status, can_execute_trajectory

    # def get_robot_trajectory(self):
    #     return self.planner.trajectory.get_trajectory()

    def update_robot_state(self, current_state):
        for key, value in self.state.items():
            if key in current_state:
                self.state[key]["current_value"] = current_state[key]

    def setup_logger(self, main_logger_name, verbose=False, log_file=False):

        # creating a formatter
        formatter = logging.Formatter('-%(asctime)s - %(name)s - %(levelname)-8s: %(message)s')

        # create console handler with a debug log level
        log_console_handler = logging.StreamHandler()
        if log_file:
            # create file handler which logs info messages
            logger_file_handler = logging.FileHandler(main_logger_name + '.log', 'w', 'utf-8')
            logger_file_handler.setLevel(logging.INFO)
            # setting handler format
            logger_file_handler.setFormatter(formatter)
            # add the file logging handlers to the logger
            self.logger.addHandler(logger_file_handler)

        if verbose == "WARN":
            self.logger.setLevel(logging.WARN)
            log_console_handler.setLevel(logging.WARN)

        elif verbose == "INFO" or verbose is True:
            self.logger.setLevel(logging.INFO)
            log_console_handler.setLevel(logging.INFO)

        elif verbose == "DEBUG":
            self.logger.setLevel(logging.DEBUG)
            log_console_handler.setLevel(logging.DEBUG)

        # setting console handler format
        log_console_handler.setFormatter(formatter)
        # add the handlers to the logger
        self.logger.addHandler(log_console_handler)
