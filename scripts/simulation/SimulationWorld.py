import logging
import pybullet as sim
import time
import numpy as np
from scripts.Robot import Robot
from scripts.utils.utils import utils
import os
import PyKDL as kdl
import itertools
import collections

CYLINDER = sim.GEOM_CYLINDER
BOX = sim.GEOM_BOX


class SimulationWorld():
    def __init__(self, urdf_file=None):

        if urdf_file is None:
            main_logger_name = "Trajectory_Planner"
            # verbose = "DEBUG"
            verbose = False
            self.logger = logging.getLogger(main_logger_name)
            self.setup_logger(main_logger_name, verbose)
        else:
            self.logger = logging.getLogger("Trajectory_Planner." + __name__)

        self.gui = sim.connect(sim.GUI)
        # self.gui = sim.connect(sim.DIRECT)
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 0)
        home = os.path.expanduser('~')
        # location_prefix = '/home/mahe/masterThesis/bullet3/data/'
        # location_prefix = '/home/mahesh/libraries/bullet3/data/'
        location_prefix = home + '/masterThesis/bullet3/data/'
        # file_path_prefix = os.path.join(os.path.dirname(__file__), '../../config/')

        if urdf_file is None:
            urdf_file = location_prefix + "kuka_iiwa/model.urdf"
        self.robot = Robot.Robot(urdf_file)
        self.robot_id = -1
        self.table_id = -1
        self.joint_name_to_id = collections.OrderedDict()
        self.no_of_joints = -1
        self.start_state_for_traj_planning = collections.OrderedDict()
        self.end_state_for_traj_planning = collections.OrderedDict()

        pland_id = self.place_items_from_urdf(urdf_file=location_prefix + "plane.urdf",
                                              position=[0, 0, 0.0])

        self.table_id = self.place_items_from_urdf(urdf_file=location_prefix + "table/table.urdf",
                                                   position=[0, 0, 0.0])

        self.robot_id = self.place_items_from_urdf(urdf_file, position=[0, 0.25, 0.6])

        self.planning_group = []
        self.planning_group_ids = []

        self.planning_samples = 0
        self.collision_safe_distance = 0.4
        self.collision_check_distance = 0.2

        self.end_effector_index = 6
        use_real_time_simulation = 0
        fixed_time_step = 0.01
        if use_real_time_simulation:
            sim.setRealTimeSimulation(use_real_time_simulation)
        else:
            sim.setTimeStep(fixed_time_step)

        sim.setGravity(0, 0, -10)
        self.no_of_joints = sim.getNumJoints(self.robot_id)
        self.setup_joint_id_to_joint_name()
        self.collision_constraints = []

        # self.cylinder_id = self.create_constraint(shape=CYLINDER, height=0.28, radius=0.1,
        #                                           position=[-0.17, -0.22, 0.9], mass=1)
        self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.2, 0.25],
                                             position=[0.28, -0.43, 0.9], mass=100)
        self.collision_constraints.append(self.table_id)

        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 1)

        if urdf_file is not None:
            # start_state = {
            #     'lbr_iiwa_joint_1': -1.570,
            #     'lbr_iiwa_joint_2': 1.571,
            #     'lbr_iiwa_joint_3': -1.570,
            #     'lbr_iiwa_joint_4': -1.570,
            #     'lbr_iiwa_joint_5': 1.571,
            #     'lbr_iiwa_joint_6': 1.571,
            #     'lbr_iiwa_joint_7': 1.571
            # }
            # start_state = {'lbr_iiwa_joint_5': 1.5855963769735366, 'lbr_iiwa_joint_4': -0.8666279970481103,
            #                'lbr_iiwa_joint_7': 1.5704531145724918, 'lbr_iiwa_joint_6': 1.5770985888989753,
            #                'lbr_iiwa_joint_1': -2.4823357809267463, 'lbr_iiwa_joint_3': -1.5762726255540713,
            #                'lbr_iiwa_joint_2': 1.4999975516996142}

            start_state = collections.OrderedDict()

            start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
            start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
            start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
            start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
            start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
            start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
            start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

            self.reset_joint_states(start_state)
            self.step_simulation_for(3)


            # else:

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

    def create_constraint(self, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        if position is not None:
            if radius is not None:
                col_id = sim.createCollisionShape(shape, radius=radius, height=height)
                vis_id = sim.createCollisionShape(shape, radius=radius, height=height)
            if size is not None:
                col_id = sim.createCollisionShape(shape, halfExtents=size)
                vis_id = sim.createCollisionShape(shape, halfExtents=size)
            shape_id = sim.createMultiBody(mass, col_id, vis_id, position)

        self.collision_constraints.append(shape_id)

        return shape_id

    def place_items_from_urdf(self, urdf_file, position, orientation=None, use_fixed_base=True):

        if orientation is None:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position, useFixedBase=use_fixed_base)
        else:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position,
                                   baseOrientation=orientation, useFixedBase=use_fixed_base)

        return urdf_id

    def run_simulation(self):
        iteration_count = 0
        while 1:
            if iteration_count < 1:
                iteration_count += 1
                # goal_state = {
                #     'lbr_iiwa_joint_1': -2.0417782994426674,
                #     'lbr_iiwa_joint_2': 0.9444594031189716,
                #     'lbr_iiwa_joint_3': -1.591006403858707,
                #     'lbr_iiwa_joint_4': -1.9222844444479184,
                #     'lbr_iiwa_joint_5': 1.572303282659756,
                #     'lbr_iiwa_joint_6': 1.5741716208788483,
                #     'lbr_iiwa_joint_7': 1.5716145442929421
                # }
                # start_state = {
                #     'lbr_iiwa_joint_1': -1.570,
                #     'lbr_iiwa_joint_2': 1.571,
                #     'lbr_iiwa_joint_3': -1.570,
                #     'lbr_iiwa_joint_4': -1.570,
                #     'lbr_iiwa_joint_5': 1.571,
                #     'lbr_iiwa_joint_6': 1.571,
                #     'lbr_iiwa_joint_7': 1.571
                # }
                start_state = collections.OrderedDict()
                goal_state = collections.OrderedDict()
                # start_state = {'lbr_iiwa_joint_5': 1.5855963769735366, 'lbr_iiwa_joint_4': -0.8666279970481103,
                #                'lbr_iiwa_joint_7': 1.5704531145724918, 'lbr_iiwa_joint_6': 1.5770985888989753,
                #                'lbr_iiwa_joint_1': -2.4823357809267463, 'lbr_iiwa_joint_3': -1.5762726255540713,
                #                'lbr_iiwa_joint_2': 1.4999975516996142}
                # goal_state = {'lbr_iiwa_joint_5': 1.5979105177314896, 'lbr_iiwa_joint_4': -0.5791571346767671,
                #               'lbr_iiwa_joint_7': 1.5726221954434347, 'lbr_iiwa_joint_6': 1.5857854098720727,
                #               'lbr_iiwa_joint_1': -0.08180533826032865, 'lbr_iiwa_joint_3': -1.5873548294514912,
                #               'lbr_iiwa_joint_2': 1.5474152457596664}

                # startState = [-1.5708022241650113, 1.5711988957726704, -1.57079632679,
                #               -1.5707784259568982, 1.5713463278825928, 1.5719498333358852, 1.5707901876998593]

                start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
                start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
                start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
                start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
                start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
                start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
                start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

                goal_state["lbr_iiwa_joint_1"] = -0.08180533826032865
                goal_state["lbr_iiwa_joint_2"] = 1.5474152457596664
                goal_state["lbr_iiwa_joint_3"] = -1.5873548294514912
                goal_state["lbr_iiwa_joint_4"] = -0.5791571346767671
                goal_state["lbr_iiwa_joint_5"] = 1.5979105177314896
                goal_state["lbr_iiwa_joint_6"] = 1.5857854098720727
                goal_state["lbr_iiwa_joint_7"] = 1.5726221954434347

                group1_test = ['lbr_iiwa_joint_1']

                group1 = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3']
                group2 = ['lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
                duration = 20
                samples = 5
                full_arm = group1 + group2
                # full_arm = group1_test
                self.reset_joint_states(start_state)
                self.step_simulation_for(2)
                # time.sleep(1)
                check_distance = 0.1
                collision_safe_distance = 0.05

                self.plan_trajectory(goal_state.keys(), goal_state, samples, duration,
                                     collision_safe_distance=collision_safe_distance, collision_check_distance=check_distance)
                # self.execute_trajectory()
                # print self.robot.planner.get_trajectory().trajectory_by_name
                # self.execute_trajectories(full_arm)
                print self.robot.planner.get_trajectory().trajectory_by_name
                import sys
                sys.exit()

    def get_link_states_at(self, trajectory, group):
        link_states = []
        self.reset_joint_states_to(trajectory, group)
        for link_index in self.planning_group_ids:
            state = sim.getLinkState(self.robot_id, link_index, computeLinkVelocity=1,
                                     computeForwardKinematics=1)
            link_states.append(state)
        return link_states

    def extract_ids_from_planning_group(self, group):
        for joint in group:
            self.planning_group_ids.append(self.joint_name_to_id[joint])

    def get_collision_infos(self, initial_trajectory, group, distance=0.20):

        # print initial_trajectory
        collision_infos = self.formulate_collision_infos(initial_trajectory, group, distance)

        return collision_infos

    def get_jacobian_matrix(self, position_jacobian, trajectory_length, planning_group_length, time_step_count):
        jaco1 = np.asarray([[[0] * planning_group_length] * 3] * (time_step_count - 1))
        if len(jaco1):
            jaco1 = np.hstack(jaco1)

        jaco2 = np.asarray(
            [[[0] * planning_group_length] * 3] * (trajectory_length - time_step_count))

        if len(jaco1) > 0 and len(jaco2):
            jaco2 = np.hstack(jaco2)
            jacobian_matrix = np.hstack(
                [jaco1, np.asarray(position_jacobian), jaco2])
        elif len(jaco1) > 0 and len(jaco2) == 0:
            # jacobian_matrix = np.vstack([jaco1, np.asarray(jac_t).reshape(1, 3, 7)])
            jacobian_matrix = np.hstack([jaco1, np.asarray(position_jacobian)])
        elif len(jaco1) == 0 and len(jaco2) > 0:
            jacobian_matrix = np.vstack(
                [np.asarray(position_jacobian).reshape(1, 3, 7), jaco2])
            jacobian_matrix = np.hstack(jacobian_matrix)

        return jacobian_matrix

    def get_velocity_matrix(self, link_index, planning_group_length, time_step_count):
        increase_resolution_matrix = []

        velocity_matrix = np.zeros((planning_group_length * self.planning_samples,
                                    self.planning_samples * planning_group_length))
        np.fill_diagonal(velocity_matrix, -1.0)
        i, j = np.indices(velocity_matrix.shape)
        velocity_matrix[i == j - planning_group_length] = 1.0

        # to slice zero last row
        velocity_matrix.resize(velocity_matrix.shape[0] - planning_group_length, velocity_matrix.shape[1])

        # print velocity_matrix

        mat = velocity_matrix[((time_step_count - 2) * planning_group_length):, :]
        # print mat

        mat = mat[link_index::planning_group_length, :]
        # print mat

        mat = mat[:5:, :]

        if len(mat):
            increase_resolution_matrix.append(np.vstack(mat))

        return increase_resolution_matrix

    def formulate_collision_infos(self, trajectory, group, distance=0.2):

        normal = []
        initial_signed_distance = []
        closest_pts = []

        jacobian_matrix = []
        current_normal_T_times_jacobian = []
        next_normal_T_times_jacobian = []
        increase_resolution_matrix = []
        start_state = self.get_current_states_for_given_joints(group)
        time_step_count = 0
        # increase_resolution_matrix = np.ones((1, (len(trajectory) * len(group)))).flatten()

        for previous_time_step_of_trajectory, current_time_step_of_trajectory, \
            next_time_step_of_trajectory in utils.iterate_with_previous_and_next(trajectory):
            time_step_count += 1
            # current_time_step_of_trajectory = current_time_step_of_trajectory.reshape((current_time_step_of_trajectory.shape[0], 1))
            # self.reset_joint_states_to(current_time_step_of_trajectory, group)
            if next_time_step_of_trajectory is not None:
                current_robot_state = list(current_time_step_of_trajectory)
                next_robot_state = list(next_time_step_of_trajectory)
                zero_vec = [0.0] * len(current_robot_state)
                current_link_states = self.get_link_states_at(current_time_step_of_trajectory, group)
                next_link_states = self.get_link_states_at(next_time_step_of_trajectory, group)

                self.reset_joint_states_to(current_time_step_of_trajectory, group)

                for link_index, current_link_state, next_link_state in itertools.izip(self.planning_group_ids,
                                                                                      current_link_states,
                                                                                      next_link_states):

                    for constraint in self.collision_constraints:
                        closest_points = sim.getClosestPoints(self.robot_id, constraint,
                                                                         linkIndexA=link_index, distance=distance
                                                                         )
                        cast_closest_points = sim.getConvexSweepClosestPoints(self.robot_id, constraint,
                                                          linkIndexA=link_index, distance=distance,
                                                                         bodyAfromPosition=current_link_state[0],
                                                                         bodyAfromOrientation=current_link_state[1],
                                                                         # bodyAfromOrientation=[0, 0, 0, 1],
                                                                         bodyAtoPosition=next_link_state[0],
                                                                         bodyAtoOrientation=next_link_state[1],
                                                                         # bodyAtoOrientation=[0, 0, 0, 1],
                                                                         )

                        if len(closest_points) > 0:
                            closest_pt_on_A_at_t1 = closest_points[0][5]
                            closest_pt_on_A_at_t_plus_11 = closest_points[0][5]
                            closest_pt_on_B1 = closest_points[0][6]
                            normal_1 = np.vstack(closest_points[0][7]).reshape(3, 1)
                            dist1 = closest_points[0][8]
                            fraction1 = 0

                        if len(cast_closest_points) > 0:

                            closest_pt_on_A_at_t = cast_closest_points[0][5]
                            closest_pt_on_A_at_t_plus_1 = cast_closest_points[0][6]
                            closest_pt_on_B = cast_closest_points[0][7]
                            normal_ = np.vstack(cast_closest_points[0][8]).reshape(3, 1)
                            normal_ = utils.normalize_vector(normal_)
                            dist = cast_closest_points[0][9]
                            fraction = cast_closest_points[0][10]

                            if dist < 0:
                                if dist1 < 0:
                                    print "-----------closest points--------------"
                                    print "A(t)", closest_pt_on_A_at_t1
                                    print "B", closest_pt_on_B1
                                    print "normal", normal_1.T
                                    print "Distance ", dist1
                                    print "*************************************"

                                    print "-----------cast points--------------"
                                    print "A(t)", closest_pt_on_A_at_t
                                    print "A(t+1)", closest_pt_on_A_at_t_plus_1
                                    print "B", closest_pt_on_B
                                    print "normal", normal_.T
                                    print "Distance ", dist
                                    print "fraction ", fraction
                                    print "*************************************"
                                # if link_index == self.end_effector_index:
                                # link_state = sim.getLinkState(self.robot_id, link_index, computeLinkVelocity=1,
                                #                               computeForwardKinematics=1)
                                current_link_position_in_world_frame = current_link_state[4]
                                current_link_orentation_in_world_frame = current_link_state[5]
                                current_closest_point_on_link_in_link_frame = self.get_point_in_local_frame(
                                    current_link_position_in_world_frame, current_link_orentation_in_world_frame,
                                    closest_pt_on_A_at_t)

                                # print "closest_point_on_link_in_link_frame", closest_point_on_link_in_link_frame
                                # print "closest_point_on_link_in_world_frame", closest_point_on_link_in_world_frame
                                initial_signed_distance.append(dist)
                                closest_pts.append(closest_pt_on_A_at_t)
                                # jac_t, jac_r = sim.calculateJacobian(self.robot_id, self.end_effector_index, closest_pt_on_A_at_t,

                                current_position_jacobian, _ = sim.calculateJacobian(self.robot_id, link_index,
                                                                     # closest_pt_on_A_at_t,
                                                                     current_closest_point_on_link_in_link_frame,
                                                                     current_robot_state,
                                                                     zero_vec, zero_vec)

                                current_state_jacobian_matrix = self.get_jacobian_matrix(current_position_jacobian,
                                                                                         len(trajectory),
                                                                                         len(group),
                                                                                         time_step_count)

                                next_link_position_in_world_frame = next_link_state[4]
                                next_link_orentation_in_world_frame = next_link_state[5]
                                next_closest_point_on_link_in_link_frame = self.get_point_in_local_frame(
                                    next_link_position_in_world_frame, next_link_orentation_in_world_frame,
                                    closest_pt_on_A_at_t_plus_1)

                                next_position_jacobian, _ = sim.calculateJacobian(self.robot_id, link_index,
                                                                                     # closest_pt_on_A_at_t,
                                                                                  next_closest_point_on_link_in_link_frame,
                                                                                     current_robot_state,
                                                                                     zero_vec, zero_vec)

                                next_state_jacobian_matrix = self.get_jacobian_matrix(next_position_jacobian,
                                                                                         len(trajectory),
                                                                                         len(group),
                                                                                         time_step_count+1)

                                jacobian_matrix.append(current_state_jacobian_matrix)
                                # normal.append(np.asarray(closest_points[0][7]).reshape(3, 1))
                                normal.append(normal_)
                                current_normal_T_times_jacobian.append(np.matmul(normal_.T,
                                                                         current_state_jacobian_matrix))

                                next_normal_T_times_jacobian.append(np.matmul((1 - fraction) * normal_.T,
                                                                                 next_state_jacobian_matrix))
                                # print link_index
                                # print res


                #     else:
                #         jacobian.append(np.zeros((3, len(group))))
                # else:
                #     jacobian.append(np.zeros((3, len(group))))

            #
            #         # jacobian = np.asarray(jac_t)
            #         # normal = np.asarray(closest_points[0][7]).reshape(3, 1)
            #
            #         # normal_times_jacobian.append(np.matmul(np.asarray(normal).T, np.asarray(jacobian)))
            #
            #         # print np.asarray(jacobian).shape
            # if len(jacobian) > 0:
            #     jacobian_matrix.append(jacobian)

        if len(normal) > 0:
            normal = np.vstack(np.asarray(normal))
            # print "normal", normal.shape

        if len(initial_signed_distance) > 0:
            initial_signed_distance = np.vstack(np.asarray(initial_signed_distance))
            # print "initial_signed_distance", initial_signed_distance.shape

        if len(jacobian_matrix) > 0:
            jacobian_matrix = np.vstack(jacobian_matrix)
            # print "jacobian", jacobian_matrix.shape
            # print "jacobian"
            # print jacobian_matrix.T
            # print jacobian_matrix.shape
        if len(current_normal_T_times_jacobian) > 0:
            current_normal_T_times_jacobian = np.vstack(current_normal_T_times_jacobian)
            # print "current_normal_T_times_jacobian", current_normal_T_times_jacobian.shape

        if len(next_normal_T_times_jacobian) > 0:
            next_normal_T_times_jacobian = np.vstack(next_normal_T_times_jacobian)
            # print "next_normal_T_times_jacobian", next_normal_T_times_jacobian.shape
            # print "next_normal_T_times_jacobian"
            # print next_normal_T_times_jacobian
        if len(increase_resolution_matrix) > 0:
            increase_resolution_matrix = np.vstack(increase_resolution_matrix)
            # print "increase_resolution_matrix", increase_resolution_matrix.shape
            # print "increase_resolution_matrix \n", increase_resolution_matrix
        # print "initial_signed_distance", initial_signed_distance.shape
        # print "initial", current_time_step_of_trajectory.shape
        # print "initial_trajectory", np.asarray(trajectory.flatten()).shape
        # print "collision matrix", jacobian_matrix.shape
        # print "collision matrix", jacobian_matrix[ 0.  0.  0.  1.  0.]
        self.reset_joint_states(start_state, group)

        # collision_infos = [initial_signed_distance, current_normal_T_times_jacobian]

        # constraints, lower_limit, upper_limit = self.robot.planner.problem.update_collision_infos(collision_infos)

        return initial_signed_distance, current_normal_T_times_jacobian, next_normal_T_times_jacobian, increase_resolution_matrix

    def get_point_in_local_frame(self, frame_position, frame_orientation, point):
        # frame = kdl.Frame()
        # print frame_orientation
        rotation = kdl.Rotation.Quaternion(frame_orientation[0], frame_orientation[1], frame_orientation[2],
                                           frame_orientation[3])
        position = kdl.Vector(frame_position[0], frame_position[1], frame_position[2])
        frame = kdl.Frame(rotation, position)
        point = kdl.Vector(point[0], point[1], point[2])

        point_on_frame =  frame.Inverse() * point

        return [point_on_frame[0], point_on_frame[1], point_on_frame[2]]

    def update_collsion_infos(self, new_trajectory, delta_trajectory=None):
        trajectory = np.array((np.split(new_trajectory, self.planning_samples)))

        self.robot.planner.trajectory.add_trajectory(trajectory)
        trajectory = np.split(new_trajectory, self.planning_samples)
        collision_infos = self.get_collision_infos(trajectory, self.planning_group, distance=self.collision_check_distance)

        constraints, lower_limit, upper_limit = self.robot.planner.problem_model.update_collision_infos(collision_infos, self.collision_safe_distance)
        if len(collision_infos[2]) > 0:
            self.robot.planner.update_prob()
        # initial_signed_distance = collision_infos[0]
        # normal = collision_infos[1]
        # jacobian = collision_infos[2]
        # normal_times_jacobian = np.matmul(normal.T, jacobian)
        # lower_limit =

        return constraints, lower_limit, upper_limit


    def plan_trajectory(self, group, goal_state, samples, duration, solver_config=None, collision_safe_distance=None, collision_check_distance=0.2):
        # self.collision_constraints = collections.OrderedDict()
        self.planning_group = group
        self.planning_samples = samples
        self.collision_safe_distance = collision_safe_distance
        self.collision_check_distance = collision_check_distance
        # if (lower_collision_limit is not None or lower_collision_limit != 0) and (upper_collision_limit is not None or upper_collision_limit != 0):
            # self.get_collision_infos(group, lower_collision_limit, upper_collision_limit)
        #
        #     # print self.collision_constraints
        #     # print joint_name, jac_t
        #     # print self.jacobian_by_joint_name
        #     # print np.asarray(self.jacobian_by_joint_name).shape
        # else:
        #     self.logger.debug("ignoring collision constraints . . . . ")
        #
        # status, can_execute_trajectory = self.robot.init_plan_trajectory(group=group,
        #                                                                  current_state=self.get_current_states_for_given_joints(group),
        #                                                                  goal_state=goal_state, samples=int(samples),
        #                                                                  duration=int(duration),
        #                                                                  collision_constraints=self.collision_constraints,
        #                                                                  solver_config=solver_config)
        # return status, can_execute_trajectory

        self.extract_ids_from_planning_group(group)
        self.robot.init_plan_trajectory(group=group, current_state=self.get_current_states_for_given_joints(group),
                                        goal_state=goal_state, samples=int(samples),
                                        duration=int(duration),
                                        # collision_constraints=self.collision_constraints,
                                        solver_config=solver_config)
        # collision_infos = self.get_collision_infos(self.robot.get_trajectory().initial, group, distance=collision_check_distance)
        # self.update_collsion_infos(self.robot.get_trajectory().initial.flatten())

        # self.robot.planner.problem.update_collision_infos(collision_infos)
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_TINY_RENDERER, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_GUI, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_SHADOWS, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        sim.configureDebugVisualizer(sim.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0)

        status, can_execute_trajectory = self.robot.calulate_trajecotory(self.update_collsion_infos) # callback function
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 1)

        # status, can_execute_trajectory = self.robot.calulate_trajecotory(None)

        return status, can_execute_trajectory

    def get_current_states_for_given_joints(self, joints):
        current_state = collections.OrderedDict()
        for joint in joints:
            current_state[joint] = \
            sim.getJointState(bodyUniqueId=self.robot_id, jointIndex=self.joint_name_to_id[joint])[0]
        return current_state

    def execute_trajectory(self):
        trajectories = self.robot.get_trajectory()
        sleep_time = trajectories.duration / float(trajectories.no_of_samples)

        for i in range(int(trajectories.no_of_samples)):
            for joint_name, corresponding_trajectory in trajectories.trajectory_by_name.items():
                sim.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.joint_name_to_id[joint_name],
                                          controlMode=sim.POSITION_CONTROL,
                                          targetPosition=corresponding_trajectory[i], targetVelocity=0,
                                          force=self.robot.model.joint_by_name[joint_name].limit.effort,
                                          positionGain=0.03,
                                          velocityGain=.5,
                                          # maxVelocity=float(self.robot.model.joint_by_name[joint_name].limit.velocity)
                                          )
                # self.get_contact_points()
            # time.sleep(trajectories.no_of_samples / float(trajectories.duration))
            # sim.stepSimulation()
            self.step_simulation_for(sleep_time)
            # time.sleep(sleep_time)
            # sim.stepSimulation()
        # print trajectories.trajectory
        # for traj in trajectories.trajectory:
        #     for i in range(len(group)):
        #         sim.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.joint_name_to_id[group[i]],
        #                                   controlMode=sim.POSITION_CONTROL,
        #                                   targetPosition=traj[i], targetVelocity=0,
        #                                   force=self.robot.model.joint_by_name[group[i]].limit.effort,
        #                                   positionGain=0.03,
        #                                   velocityGain=.5,
        #                                   # maxVelocity=float(self.robot.model.joint_by_name[joint_name].limit.velocity)
        #                                   )
        #         # self.get_contact_points()
        #     # time.sleep(trajectories.no_of_samples / float(trajectories.duration))
        #     time.sleep(trajectories.duration / float(trajectories.no_of_samples))
        #     # sim.stepSimulation()

        status = "Trajectory execution has finished"
        self.logger.info(status)
        return status


    def execute_trajectories(self, group):
        trajectories = self.robot.get_trajectory()
        sleep_time = trajectories.duration / float(trajectories.no_of_samples)

        for i in range(int(trajectories.no_of_samples)):
            for trajectory in trajectories.trajectories:
                for joint_name, corresponding_trajectory in trajectory:
                    sim.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.joint_name_to_id[joint_name],
                                              controlMode=sim.POSITION_CONTROL,
                                              targetPosition=corresponding_trajectory[i], targetVelocity=0,
                                              force=self.robot.model.joint_by_name[joint_name].limit.effort,
                                              positionGain=0.03,
                                              velocityGain=.5,
                                              # maxVelocity=float(self.robot.model.joint_by_name[joint_name].limit.velocity)
                                              )
                    # self.get_contact_points()
                # time.sleep(trajectories.no_of_samples / float(trajectories.duration))
                # sim.stepSimulation()
                self.step_simulation_for(sleep_time)
            # time.sleep(sleep_time)
            # sim.stepSimulation()
        # print trajectories.trajectory
        # for traj in trajectories.trajectory:
        #     for i in range(len(group)):
        #         sim.setJointMotorControl2(bodyIndex=self.robot_id, jointIndex=self.joint_name_to_id[group[i]],
        #                                   controlMode=sim.POSITION_CONTROL,
        #                                   targetPosition=traj[i], targetVelocity=0,
        #                                   force=self.robot.model.joint_by_name[group[i]].limit.effort,
        #                                   positionGain=0.03,
        #                                   velocityGain=.5,
        #                                   # maxVelocity=float(self.robot.model.joint_by_name[joint_name].limit.velocity)
        #                                   )
        #         # self.get_contact_points()
        #     # time.sleep(trajectories.no_of_samples / float(trajectories.duration))
        #     time.sleep(trajectories.duration / float(trajectories.no_of_samples))
        #     # sim.stepSimulation()

        status = "Trajectory execution has finished"
        self.logger.info(status)
        return status

    def plan_and_execute_trajectory(self, group, goal_state, samples, duration, solver_config=None):
        status = "-1"
        status, can_execute_trajectory = self.plan_trajectory(group, goal_state, samples, duration, solver_config=None)
        status += " and "
        status += self.execute_trajectory()

        return status, can_execute_trajectory

    def setup_joint_id_to_joint_name(self):
        for i in range(self.no_of_joints):
            joint_info = sim.getJointInfo(self.robot_id, i)
            self.joint_name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]

    def reset_joint_states_to(self, trajectory, joints, motor_dir=None):
        if len(trajectory) == len(joints):
            for i in range(len(trajectory)):
                sim.resetJointState(self.robot_id, self.joint_name_to_id[joints[i]], trajectory[i])
                status = "Reset joints to start pose is complete"

        else:
            status = "cannot reset the joint states as the trajectory and joints size doesn't match"

        # self.logger.info(status)
        return status

    def reset_joint_states(self, joints, motor_dir=None):
        # if motor_dir is None:
        #     # motor_dir = [-1, -1, -1, 1, 1, 1, 1]
        #     motor_dir = np.random.uniform(-1, 1, size=len(joints))
        # half_pi = 1.57079632679
        for joint in joints:
            for j in range(len(joints)):
                sim.resetJointState(self.robot_id, self.joint_name_to_id[joint], joints[joint])
        status = "Reset joints to start pose is complete"
        # self.logger.info(status)
        return status

    def get_joint_states(self, group):
        joint_states = sim.getJointStates(self.robot_id, [self.joint_name_to_id[joint_name] for joint_name in group])
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def step_simulation_for(self, seconds):
        start = time.time()
        while time.time() < start + seconds:
            sim.stepSimulation()

    def get_contact_points(self):
        for i in range(sim.getNumJoints(self.robot_id)):
            contact_points = sim.getClosestPoints(self.robot_id, self.box_id, linkIndexA=i, distance=4)
            contact_points1 = sim.getContactPoints(self.robot_id, self.box_id, linkIndexA=i)

            if len(contact_points) > 0 and len(contact_points1) > 0:
                if contact_points[0][8] < 0:
                    print "index ", i
                    print "positionOnA ", contact_points[0][5]
                    print "positionOnB ", contact_points[0][6]
                    print "contactNormalOnB ", contact_points[0][7]
                    print "contactDistance ", contact_points[0][8]

                if contact_points1[0][8] < 0:
                    print "index 1 ", i
                    print "positionOnA 1 ", contact_points1[0][5]
                    print "positionOnB 1 ", contact_points1[0][6]
                    print "contactNormalOnB 1 ", contact_points1[0][7]
                    print "contactDistance 1 ", contact_points1[0][8]


if __name__ == "__main__":
    sim1 = SimulationWorld()
    sim1.run_simulation()