import logging
import pybullet as sim
import time
import numpy as np
from scripts.Robot import Robot
from munch import *
import os
import PyKDL as kdl
import itertools

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
        self.joint_name_to_id = {}
        self.no_of_joints = -1
        self.start_state_for_traj_planning = {}
        self.end_state_for_traj_planning = {}

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
            start_state = {'lbr_iiwa_joint_5': 1.5855963769735366, 'lbr_iiwa_joint_4': -0.8666279970481103,
                           'lbr_iiwa_joint_7': 1.5704531145724918, 'lbr_iiwa_joint_6': 1.5770985888989753,
                           'lbr_iiwa_joint_1': -2.4823357809267463, 'lbr_iiwa_joint_3': -1.5762726255540713,
                           'lbr_iiwa_joint_2': 1.4999975516996142}
            self.reset_joint_states(start_state)

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
                start_state = {'lbr_iiwa_joint_5': 1.5855963769735366, 'lbr_iiwa_joint_4': -0.8666279970481103,
                               'lbr_iiwa_joint_7': 1.5704531145724918, 'lbr_iiwa_joint_6': 1.5770985888989753,
                               'lbr_iiwa_joint_1': -2.4823357809267463, 'lbr_iiwa_joint_3': -1.5762726255540713,
                               'lbr_iiwa_joint_2': 1.4999975516996142}
                goal_state = {'lbr_iiwa_joint_5': 1.5979105177314896, 'lbr_iiwa_joint_4': -0.5791571346767671,
                              'lbr_iiwa_joint_7': 1.5726221954434347, 'lbr_iiwa_joint_6': 1.5857854098720727,
                              'lbr_iiwa_joint_1': -0.08180533826032865, 'lbr_iiwa_joint_3': -1.5873548294514912,
                              'lbr_iiwa_joint_2': 1.5474152457596664}

                # startState = [-1.5708022241650113, 1.5711988957726704, -1.57079632679,
                #               -1.5707784259568982, 1.5713463278825928, 1.5719498333358852, 1.5707901876998593]
                group1_test = ['lbr_iiwa_joint_1']

                group1 = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3']
                group2 = ['lbr_iiwa_joint_4', 'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']
                duration = 20
                samples = 20
                full_arm = group1 + group2
                # full_arm = group1_test
                self.reset_joint_states(start_state)
                self.step_simulation_for(2)
                # time.sleep(1)
                check_distance = 0.2
                collision_safe_distance = 0.05


                self.plan_trajectory(goal_state.keys(), goal_state, samples, duration,
                                     collision_safe_distance=collision_safe_distance, collision_check_distance=check_distance)
                self.execute_trajectory()
                # self.execute_trajectories(full_arm)
                # import sys
                # sys.exit()

    def extract_ids_from_planning_group(self, group):
        for joint in group:
            self.planning_group_ids.append(self.joint_name_to_id[joint])


    def get_collision_infos(self, initial_trajectory, group, distance=0.20):

        # print initial_trajectory
        collision_infos = self.formulate_collision_infos(initial_trajectory, group, distance)

        return collision_infos

    def formulate_collision_infos(self, trajectory, group, distance=0.2):

        normal = []
        initial_signed_distance = []
        closest_pts = []

        jacobian_matrix = []
        nomral_T_times_jacobian = []
        increase_resolution_matrix = []
        start_state = self.get_current_states_for_given_joints(group)
        time_step_count = 0
        # increase_resolution_matrix = np.ones((1, (len(trajectory) * len(group)))).flatten()

        for time_step_of_trajectory, delta_trajectory in itertools.izip(trajectory, trajectory):
            time_step_count += 1
            time_step_of_trajectory = time_step_of_trajectory.reshape((time_step_of_trajectory.shape[0], 1))
            self.reset_joint_states_to(time_step_of_trajectory, group)
            robot_joint_positions = list(time_step_of_trajectory)
            zero_vec = [0.0] * len(robot_joint_positions)

            for link_index in self.planning_group_ids:

                for constratint in self.collision_constraints:
                    closest_points = sim.getClosestPoints(self.robot_id, constratint,
                                                      linkIndexA=link_index, distance=distance)
                    if len(closest_points) > 0:
                        if closest_points[0][8] < 0:
                            # if link_index == self.end_effector_index:
                            link_state = sim.getLinkState(self.robot_id, link_index, computeLinkVelocity=1,
                                                          computeForwardKinematics=1)
                            link_position_in_world_frame = link_state[4]
                            link_orentation_in_world_frame = link_state[5]
                            closest_point_on_link_in_world_frame = closest_points[0][5]
                            closest_point_on_link_in_link_frame = self.get_point_in_local_frame(
                                link_position_in_world_frame, link_orentation_in_world_frame,
                                closest_point_on_link_in_world_frame)

                            # print "closest_point_on_link_in_link_frame", closest_point_on_link_in_link_frame
                            # print "closest_point_on_link_in_world_frame", closest_point_on_link_in_world_frame
                            initial_signed_distance.append(closest_points[0][8])
                            closest_pts.append(closest_points[0][5])
                            # jac_t, jac_r = sim.calculateJacobian(self.robot_id, self.end_effector_index, closest_points[0][5],

                            jac_t, jac_r = sim.calculateJacobian(self.robot_id, link_index,
                                                                 # closest_points[0][5],
                                                                 closest_point_on_link_in_link_frame,
                                                                 robot_joint_positions,
                                                                 zero_vec, zero_vec)
                            jaco1 = np.asarray([[[0] * len(group)] * 3] * (time_step_count - 1))

                            # for i in range(1, 3):
                            #     mat1 = [0] * (link_index - 1) + [1] + [0] * (len(group) - link_index)
                            #     mat2 = [0] * (link_index - 1) + [-1] + [0] * (len(group) - link_index)
                            #     mat = [[0] * len(group)] * (time_step_count-(i+1)) + [mat2] + [mat1] + [[0] * len(group)] * ((len(trajectory)) - (time_step_count) + (i-1))
                            #     # print "mat", link_index, time_step_count
                            # # print np.hstack(mat)
                            #     print mat
                            # mat = np.zeros((7  , len(group) * len(trajectory)))
                            # i, j = np.indices(mat.shape)
                            #
                            # # mat[i == j - link_index] = -1
                            # # mat[i == j - (link_index + len(trajectory))] = 1
                            #
                            # mat[i == j - (((time_step_count - 1) * self.planning_samples) + link_index)] = -1
                            # mat[i == j - ((link_index + (time_step_count ) * self.planning_samples))] = 1

                            velocity_matrix = np.zeros((len(group) * self.planning_samples, self.planning_samples * len(group)))
                            np.fill_diagonal(velocity_matrix, -1.0)
                            i, j = np.indices(velocity_matrix.shape)
                            velocity_matrix[i == j - len(group)] = 1.0

                            # to slice zero last row
                            velocity_matrix.resize(velocity_matrix.shape[0] - len(group), velocity_matrix.shape[1])

                            # print velocity_matrix

                            mat = velocity_matrix[((time_step_count - 2) * len(group)):, :]
                            # print mat

                            mat = mat[link_index::(len(group)), :]
                            # print mat

                            mat = mat[:5:, :]

                            # print "mat", link_index, time_step_count
                            # print mat.shape
                            # print np.vstack(mat)
                            if len(mat):
                                increase_resolution_matrix.append(np.vstack(mat))

                            # res1 = np.asarray([[[0] * len(group)]] * (time_step_count - 1))
                            if len(jaco1):
                                jaco1 = np.hstack(jaco1)

                            jaco2 = np.asarray([[[0] * len(group)] * 3] * (len(trajectory) - (time_step_count)))

                            if len(jaco2) > 0:
                                jaco2 = np.hstack(jaco2)
                                jaco = np.hstack([jaco1, np.asarray(jac_t), jaco2])
                            else:
                                jaco = np.hstack([jaco1, np.asarray(jac_t)])

                            res1 = np.zeros((1, (time_step_count - 1))).flatten()

                            # if len(res1):
                            #     res1 = np.hstack(res1)
                            #
                            # # res2 = np.asarray([[[0] * len(group)]] * (len(trajectory) - (time_step_count)))
                            # res2 = np.zeros((1, (len(trajectory) - time_step_count))).flatten()
                            # if len(res2) > 0:
                            #     res2 = np.hstack(res2)
                            #     # res = np.hstack([res1, np.asarray([[[0] * len(group)]]), res2])
                            #     temp = np.ones((1, (len(trajectory) - time_step_count))).flatten()
                            #     # print res1.shape, res2.shape, temp.shape
                            #     res = np.hstack([res1, temp, res2])
                            # else:
                            #     res = np.hstack([res1, np.ones((1, (len(group))))])
                            #
                            # res = np.ones((1, (len(trajectory) * len(time_step_of_trajectory)))).flatten()
                            # print np.asarray(jac_t).shape
                            # print jaco.shape, link_index, time_step_count - 1
                            jacobian_matrix.append(jaco)
                            # normal.append(np.asarray(closest_points[0][7]).reshape(3, 1))
                            normal.append(np.vstack(closest_points[0][7]).reshape(3, 1))
                            nomral_T_times_jacobian.append(np.matmul(np.asarray(closest_points[0][7]).reshape(1, 3), jaco))
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
        if len(nomral_T_times_jacobian) > 0:
            nomral_T_times_jacobian = np.vstack(nomral_T_times_jacobian)
            # print "nomral_T_times_jacobian", nomral_T_times_jacobian.shape
            # print "nomral_T_times_jacobian"
            # print nomral_T_times_jacobian
        if len(increase_resolution_matrix) > 0:
            increase_resolution_matrix = np.vstack(increase_resolution_matrix)
            # print "increase_resolution_matrix", increase_resolution_matrix.shape
            # print "increase_resolution_matrix \n", increase_resolution_matrix
        # print "initial_signed_distance", initial_signed_distance.shape
        # print "initial", time_step_of_trajectory.shape
        # print "initial_trajectory", np.asarray(trajectory.flatten()).shape
        # print "collision matrix", jacobian_matrix.shape
        # print "collision matrix", jacobian_matrix[ 0.  0.  0.  1.  0.]
        self.reset_joint_states(start_state, group)

        # collision_infos = [initial_signed_distance, nomral_T_times_jacobian]

        # constraints, lower_limit, upper_limit = self.robot.planner.problem.update_collision_infos(collision_infos)

        return initial_signed_distance, nomral_T_times_jacobian, increase_resolution_matrix

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
        # self.collision_constraints = {}
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
        current_state = {}
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
                                          force=self.robot.model.joint_map[joint_name].limit.effort,
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
                                              force=self.robot.model.joint_map[joint_name].limit.effort,
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