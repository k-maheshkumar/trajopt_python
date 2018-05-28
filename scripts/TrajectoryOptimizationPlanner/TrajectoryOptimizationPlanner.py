from scripts.simulation.SimulationWorld import SimulationWorld
from scripts.Robot.Robot import Robot
import numpy as np
import scripts.utils.yaml_paser as yaml
from scripts.utils.utils import Utils as utils
import logging
import os
from collections import OrderedDict
from scripts.DB.Mongo_driver import MongoDriver
import time


class TrajectoryOptimizationPlanner():
    def __init__(self, **kwargs):

        self.robot_config = None
        self.default_config = None
        self.config = None
        self.sqp_yaml = None
        self.sqp_config = None
        self.robot_default_config_params = None
        self.elapsed_time = 0

        main_logger_name = utils.get_var_from_kwargs("logger_name", optional=True,
                                                     default="Trajectory_Optimization_Planner.", **kwargs)
        verbose = utils.get_var_from_kwargs("verbose", optional=True, default=False, **kwargs)

        log_file = utils.get_var_from_kwargs("log_file", optional=True, default=False, **kwargs)

        robot_config = utils.get_var_from_kwargs("robot_config", optional=True, **kwargs)
        self.load_configs(robot_config)

        self.save_problem = utils.get_var_from_kwargs("save_problem", optional=True, **kwargs)
        if self.save_problem is not None:
            db_name = utils.get_var_from_kwargs("db_name", optional=True, default="Trajectory_planner_results", **kwargs)
            self.db_driver = MongoDriver(db_name)
        else:
            self.db_driver = None
            self.save_problem = None

        self.if_plot_traj = utils.get_var_from_kwargs("plot_trajectory", optional=True, default=False, **kwargs)

        self.robot = Robot(main_logger_name, verbose, log_file)
        self.world = SimulationWorld(**kwargs)
        self.logger = logging.getLogger(main_logger_name)
        utils.setup_logger(self.logger, main_logger_name, verbose, log_file)

        self.world.toggle_rendering(0)
        self.load_robot_from_config()
        self.world.toggle_rendering(1)

    def load_configs(self, config_file=None):
        file_path_prefix = os.path.join(os.path.dirname(__file__), '../../config/')
        self.default_config = yaml.ConfigParser(file_path_prefix + 'default_config.yaml')
        self.config = self.default_config.get_by_key("config")

        sqp_config_file = file_path_prefix + self.config["solver"]

        self.sqp_yaml = yaml.ConfigParser(sqp_config_file)
        self.sqp_config = self.sqp_yaml.get_by_key("sqp")
        if config_file is not None:
            robot_config_file = file_path_prefix + config_file
        else:
            robot_config_file = file_path_prefix + self.config["robot"]["config"]
        self.robot_default_config_params = self.config["robot"]["default_paramaters"]
        robot_yaml = yaml.ConfigParser(robot_config_file)
        self.robot_config = robot_yaml.get_by_key("robot")

    def load_robot(self, urdf_file, position=[0, 0, 0], orientation=[0, 0, 0, 1], use_fixed_base=True):
        self.robot.id = self.world.load_robot(urdf_file, position, orientation, use_fixed_base)
        self.robot.load_robot_model(urdf_file)
        return self.robot.id

    def load_robot_srdf(self, srdf_file):
        self.robot.load_srdf(srdf_file)
        self.world.ignored_collisions = self.robot.get_ignored_collsion()

    def load_robot_from_config(self):
        urdf_file = utils.get_var_from_kwargs("urdf", optional=True, **self.robot_config)
        srdf_file = utils.get_var_from_kwargs("srdf", optional=True, **self.robot_config)
        if urdf_file is not None:
            pos = self.robot_config["position"] if "position" in self.robot_config else [0, 0, 0]
            orn = self.robot_config["orientation"] if "orientation" in self.robot_config else [0, 0, 0]
            self.load_robot(urdf_file, position=pos, orientation=orn)
        if srdf_file is not None:
            self.load_robot_srdf(srdf_file)

    def load_from_urdf(self, name, urdf_file, position, orientation=None, use_fixed_base=False):
        urdf_id = self.world.load_urdf(name, urdf_file, position, orientation, use_fixed_base)
        return urdf_id

    def add_constraint_from_urdf(self, name, urdf_file, position, orientation=None, use_fixed_base=False):
        urdf_id = self.world.load_urdf(name, urdf_file, position, orientation, use_fixed_base)
        self.world.add_collision_constraints(urdf_id)
        return urdf_id

    def add_constraint(self, name, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        shape_id = self.world.create_constraint(name, shape, mass, position, orientation, size, radius, height)
        self.world.add_collision_constraints(shape_id)
        return shape_id

    def add_constraint_with_id(self, constraint_id):
        self.world.add_collision_constraints(constraint_id)

    def get_trajectory(self, **kwargs):
        group = []
        group_name = utils.get_var_from_kwargs("group", **kwargs)
        if group_name is not None:
            if type(group_name) is list:
                group = group_name
            if type(group_name) is str:
                group = self.robot_config["joints_groups"][group_name]
            if not len(group):
                group = self.robot.get_planning_group_from_srdf(group)

        start_state = utils.get_var_from_kwargs("start_state", optional=True, **kwargs)
        if start_state is not None and len(group):
            if type(start_state) is dict or type(start_state) is OrderedDict:
                start_state = start_state.values()
            if not type(start_state) is list:
                _, start_state = self.get_planning_group_and_corresponding_state("start_state", **kwargs)
            self.reset_robot_to(start_state, group, key="start_state")
            status, is_collision_free, trajectory = "start state in collision", False, -1
            is_start_state_in_collision = self.world.is_given_state_in_collision(self.robot.id, start_state, group)
            if is_start_state_in_collision:
                print "is_start_state_in_collision", is_start_state_in_collision
                status = "start state in collision"
                return status, is_collision_free, trajectory
        elif len(group):
            start_state = self.world.get_current_states_for_given_joints(self.robot.id, group)

        goal_state = utils.get_var_from_kwargs("goal_state", **kwargs)
        if goal_state is not None and len(group):
            if type(goal_state) is dict or type(goal_state) is OrderedDict:
                goal_state = goal_state.values()
            if not type(goal_state) is list:
                _, goal_state = self.get_planning_group_and_corresponding_state("goal_state", **kwargs)
                status, is_collision_free, trajectory = "goal state in collision", False, -1
                is_goal_in_collision = self.world.is_given_state_in_collision(self.robot.id, goal_state, group)
                if is_goal_in_collision:
                    print "is_goal_in_collision", is_goal_in_collision
                    status = "goal state in collision"
                    return status, is_collision_free, trajectory

        samples = utils.get_var_from_kwargs("samples", optional=True, default=20, **kwargs)
        duration = utils.get_var_from_kwargs("duration", optional=True, default=10, **kwargs)
        collision_safe_distance = utils.get_var_from_kwargs("collision_safe_distance", optional=True,
                                                            default=0.05, **kwargs)
        collision_check_distance = utils.get_var_from_kwargs("collision_check_distance", optional=True,
                                                             default=0.1, **kwargs)
        ignore_goal_states = utils.get_var_from_kwargs("ignore_goal_states", optional=True, **kwargs)

        self.robot.init_plan_trajectory(group=group, current_state=start_state,
                                        goal_state=goal_state, samples=samples, duration=duration,
                                        collision_safe_distance=collision_safe_distance,
                                        collision_check_distance=collision_check_distance,
                                        solver_class=self.sqp_config["solver_class"],
                                        ignore_goal_states=ignore_goal_states
                                        )

        # self.world.toggle_rendering_while_planning(False)
        _, planning_time, _ = self.robot.calulate_trajecotory(self.callback_function_from_solver)
        trajectory = self.robot.planner.get_trajectory()

        # is_collision_free = True
        is_collision_free = self.world.is_trajectory_collision_free(self.robot.id, self.robot.get_trajectory().final,
                                                                    group,
                                                                    0.02)
        self.world.toggle_rendering_while_planning(True)
        self.elapsed_time = self.robot.planner.sqp_solver.solving_time + \
                            self.world.collision_check_time + self.robot.planner.prob_model_time
        status = "Optimal Trajectory has been found in " + str(self.elapsed_time) + " secs"
        self.logger.info(status)

        if self.if_plot_traj:
            self.robot.planner.trajectory.plot_trajectories()

        if self.save_problem:
            self.save_to_db(samples, duration, current_robot_state, goal_state, group, collision_safe_distance,
                            collision_check_distance, is_collision_free)

        return status, is_collision_free, trajectory

    def plan_trajectory(self, planning_group, goal_state, samples=20, duration=10,
                        collision_safe_distance=0.1,
                       collision_check_distance=0.05, solver_config=None):

        status, is_collision_free, _ = self.get_trajectory(group=planning_group,
                                        goal_state=goal_state, samples=samples, duration=duration,
                                        collision_safe_distance=collision_safe_distance,
                                        collision_check_distance=collision_check_distance,
                                        solver_config=solver_config)
        print "is_collision_free",
        print is_collision_free
        return status, is_collision_free

    def execute_trajectory(self):
        self.world.execute_trajectory(self.robot, self.robot.planner.get_trajectory())

        return "Trajectory execution completed"

    def get_planning_group_and_corresponding_state(self, group_state, **kwargs):
        group = []
        joint_states = []

        group_name = utils.get_var_from_kwargs("group", **kwargs)
        if group_name is not None:
            if type(group_name) is str:
                group = self.robot_config["joints_groups"][kwargs["group"]]
                if not len(group):
                    group = self.robot.get_planning_group_from_srdf(group)
            if group_state in kwargs and len(group):
                joint_states = kwargs[group_state]
                if type(joint_states) is str and joint_states in self.robot_config["joint_configurations"]:
                    joint_states = self.robot_config["joint_configurations"][joint_states]
                # if not len(joint_states):
                else:
                    group, joint_states = self.robot.get_group_state_from_srdf(joint_states)
                if type(joint_states) is dict or type(joint_states) is OrderedDict:
                    joint_states = joint_states.values()

        return group, joint_states

    def reset_robot_to(self, state, group, key="reset_state"):
        group, joint_states = self.get_planning_group_and_corresponding_state(key, group=group, reset_state=state)
        self.world.reset_joint_states(self.robot.id, joint_states, group)

    def reset_robot_to_random_state(self, group):
        if type(group) is str:
            group = self.robot_config["joints_groups"][group]
        if type(group) is dict or type(group) is OrderedDict:
            group = group.values()
        status = self.world.reset_joints_to_random_states(self.robot, group)
        self.world.step_simulation_for(0.2)

        return status

    def get_group_names(self, group):
        if type(group) is str:
            group = self.robot_config["joints_groups"][group]
        if type(group) is dict or type(group) is OrderedDict:
            group = group.values()

        return group

    def callback_function_from_solver(self, new_trajectory, delta_trajectory=None):

        constraints, lower_limit, upper_limit = None, None, None
        new_trajectory = new_trajectory[:self.robot.planner.no_of_samples * self.robot.planner.num_of_joints]
        trajectory = np.split(new_trajectory, self.robot.planner.no_of_samples)
        self.robot.planner.trajectory.add_trajectory(trajectory)
        start = time.time()
        collision_infos = self.world.get_collision_infos(self.robot, trajectory, self.robot.planner.current_planning_joint_group,
                                                         distance=self.robot.planner.collision_check_distance)
        end = time.time()
        self.elapsed_time = (end - start) + self.robot.planner.sqp_solver.solving_time
        # self.elapsed_time = self.world.collision_check_time + self.robot.planner.sqp_solver.solving_time
        if len(collision_infos[2]) > 0:
            constraints, lower_limit, upper_limit = \
                self.robot.planner.problem_model.update_collision_infos(collision_infos, self.robot.planner.collision_safe_distance)
            self.robot.planner.update_prob()

        return constraints, lower_limit, upper_limit

    def get_group_and_state(self, group, state, **kwargs):
        if type(group) is str:
            group = self.robot_config["joints_groups"][kwargs["group"]]
        goal_state = kwargs["goal_state"]
        if type(goal_state) is str:
            goal_state = self.robot_config["joint_configurations"][goal_state]

        if type(goal_state) is dict or type(goal_state) is OrderedDict:
            goal_state = goal_state.values()

        return goal_state

    def save_to_db(self, samples, duration, current_robot_state, goal_state, group, d_safe, d_check, is_collision_free):

        act_redutcion =  self.robot.planner.sqp_solver.actual_reductions
        pred_reduction =  self.robot.planner.sqp_solver.predicted_reductions
        actual_costs =  self.robot.planner.sqp_solver.actual_costs
        model_costs =  self.robot.planner.sqp_solver.model_costs
        actual_reduction_improve, predicted_reduction_improve, actual_cost_improve, model_cost_improve = 0, 0, 0, 0
        if len(act_redutcion):
            # actual_reduction_improve = act_redutcion[len(act_redutcion)-2] - act_redutcion[len(act_redutcion)-1]
            actual_reduction_improve = act_redutcion[0] - act_redutcion[-1]
            actual_reduction_improve /= (act_redutcion[0] + 0.000000001)
            actual_reduction_improve *= 100
        if len(pred_reduction):
            # predicted_reduction_improve = pred_reduction[len(pred_reduction)-2] - pred_reduction[len(pred_reduction)-1]
            predicted_reduction_improve = pred_reduction[0] - pred_reduction[-1]
            predicted_reduction_improve /= (pred_reduction[0] + 0.000000001)
            predicted_reduction_improve *= 100
        if len(actual_costs):
            # actual_cost_improve = actual_costs[len(actual_costs)-2] - actual_costs[len(actual_costs)-1]
            actual_cost_improve = actual_costs[0] - actual_costs[-1]
            actual_cost_improve /= (actual_costs[0] + 0.000000001)
            actual_cost_improve *= 100
        if len(actual_costs):
            # model_cost_improve = model_costs[len(model_costs)-2] - model_costs[len(model_costs)-1]
            model_cost_improve = model_costs[0] - model_costs[-1]
            model_cost_improve /= (model_costs[0] + 0.000000001)
            model_cost_improve *= 100
        print "samples", samples
        print "no of links", len(self.world.robot_info["joint_infos"])
        print "number of qp iterations: ", self.robot.planner.sqp_solver.num_qp_iterations
        print "number of sqp iterations: ", self.robot.planner.sqp_solver.num_sqp_iterations

        print "actual_reduction: ", self.robot.planner.sqp_solver.actual_reductions
        print "predicted_reduction: ", self.robot.planner.sqp_solver.predicted_reductions
        print "actual_costs: ", self.robot.planner.sqp_solver.actual_costs
        print "model_costs: ", self.robot.planner.sqp_solver.model_costs

        print "actual reduction improvement: ", actual_reduction_improve
        print "predicted reduction improvement: ", predicted_reduction_improve
        print "actual cost improvement: ", actual_cost_improve
        print "model cost improvement: ", model_cost_improve
        print "collision check time: ", self.world.collision_check_time
        print "solving_time: ", self.robot.planner.sqp_solver.solving_time
        print "prob_model_time: ", self.robot.planner.prob_model_time
        print "total elapsed_time time: ", self.elapsed_time

        if self.save_problem and self.db_driver is not None:
            planning_request = OrderedDict()
            planning_request["samples"] = samples
            planning_request["duration"] = duration
            planning_request["start_state"] = current_robot_state
            planning_request["goal_state"] = goal_state
            planning_request["no of links"] = len(self.world.robot_info["joint_infos"])
            planning_request["collision_safe_distance"] = d_safe
            planning_request["collision_check_distance"] = d_check
            planning_request["no_scene_items"] = len(self.world.scene_items)

            result = OrderedDict()

            # result["type"] = "donbot_random_state_and_obstacles"
            result["type"] = "old_vs_new_solver"
            # result["sub_type"] = "prob_" + str(len(self.world.scene_items))
            # result["sub_type"] = "donbot_full_new_solver"
            # result["sub_type"] = "donbot_arm_new_solver"
            result["sub_type"] = "donbot_arm_old_solver"

            result["num_qp_iterations"] = self.robot.planner.sqp_solver.num_qp_iterations
            result["num_sqp_iterations"] = self.robot.planner.sqp_solver.num_sqp_iterations
            result["actual_reductions"] = self.robot.planner.sqp_solver.actual_reductions
            result["predicted_reductions"] = self.robot.planner.sqp_solver.predicted_reductions
            result["actual_costs"] = self.robot.planner.sqp_solver.actual_costs
            result["model_costs"] = self.robot.planner.sqp_solver.model_costs
            result["cost_improvement"] = actual_reduction_improve
            result["collision_check_time"] = self.world.collision_check_time
            result["solving_time"] = self.robot.planner.sqp_solver.solving_time
            result["prob_model_time"] = self.robot.planner.prob_model_time
            # result["total_time"] = elapsed_time
            result["planning_time"] = self.elapsed_time
            result["is_collision_free"] = is_collision_free
            result["planning_request"] = planning_request
            result["initial_trajectory"] = self.robot.planner.trajectory.initial.tolist()
            result["final_trajectory"] = [x.tolist() for x in self.robot.planner.trajectory.trajectory_by_name.values()]
            planning_request["group"] = self.robot.planner.trajectory.trajectory_by_name.keys()
            result["solver_config"] = self.robot.planner.sqp_solver.solver_config

            self.db_driver.insert(result)


if __name__ == '__main__':

    temp = {}
    planner = TrajectoryOptimizationPlanner(**temp)
    planner.save_problem_in_db("", "", "")