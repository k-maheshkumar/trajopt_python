import logging
import pybullet as sim
import pybullet_data
import time
import numpy as np
from scripts.interfaces.ISimulationWorldBase import ISimulationWorldBase
import PyKDL as kdl
import itertools
from scripts.utils.utils import Utils as utils
from scripts.utils.yaml_paser import ConfigParser as config
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
from scripts.simulation.bulletTypes import *
import os
from scripts.Robot.ModelandTree import RobotTree

home = os.path.expanduser('~')
file_name = home + '/temp/collision.log'
os.remove(file_name) if os.path.exists(file_name) else None

file_name1 = home + '/temp/collision_data.log'
os.remove(file_name) if os.path.exists(file_name1) else None


class SimulationWorld(ISimulationWorldBase):
    def __init__(self, **kwargs):

        if "use_gui" in kwargs:
            use_gui = kwargs["use_gui"]
        else:
            use_gui = False

        if "verbose" in kwargs:
            verbose = kwargs["verbose"]
        else:
            verbose = False

        if "log_file" in kwargs:
            log_file = kwargs["log_file"]
        else:
            log_file = False

        if "use_real_time_simulation" in kwargs:
            use_real_time_simulation = kwargs["use_real_time_simulation"]
        else:
            use_real_time_simulation = False

        if "fixed_time_step" in kwargs:
            fixed_time_step = kwargs["fixed_time_step"]
        else:
            fixed_time_step = 0.01

        if "logger_name" in kwargs:
            logger_name = kwargs["logger_name"]
        else:
            logger_name = __name__

        self.CYLINDER = sim.GEOM_CYLINDER
        self.BOX = sim.GEOM_BOX

        self.logger = logging.getLogger(logger_name + __name__)
        utils.setup_logger(self.logger, logger_name, verbose, log_file)

        if use_gui:
            self.gui = sim.connect(sim.GUI_SERVER)
        else:
            self.gui = sim.connect(sim.DIRECT)

        sim.setAdditionalSearchPath(pybullet_data.getDataPath())


        self.joint_name_to_id = OrderedDict()
        self.joint_id_to_name = OrderedDict()
        self.start_state_for_traj_planning = OrderedDict()
        self.end_state_for_traj_planning = OrderedDict()
        self.scene_items = OrderedDict()
        self.joint_name_to_info = OrderedDict()
        self.joint_id_to_info = OrderedDict()
        self.ignored_collisions = DefaultOrderedDict(bool)
        self.link_pairs = DefaultOrderedDict(list)

        self.planning_group = []
        self.planning_group_ids = []
        self.joint_ids = []

        self.planning_samples = 0
        self.collision_safe_distance = 0.4
        self.collision_check_distance = 0.2

        self.robot = None

        if use_real_time_simulation:
            sim.setRealTimeSimulation(use_real_time_simulation)
        else:
            sim.setTimeStep(fixed_time_step)

        self.collision_constraints = []

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

    def toggle_rendering(self, enable):
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, enable)

    def toggle_rendering_while_planning(self, enable=1):
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_TINY_RENDERER, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_GUI, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_SHADOWS, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_RGB_BUFFER_PREVIEW, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_DEPTH_BUFFER_PREVIEW, enable)
        sim.configureDebugVisualizer(sim.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, enable)

    def set_gravity(self, x=0, y=0, z=-10):
        sim.setGravity(x, y, z)

    def create_constraint(self, name, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        if position is not None:
            if radius is not None:
                col_id = sim.createCollisionShape(shape, radius=radius, height=height)
                vis_id = sim.createCollisionShape(shape, radius=radius, height=height)
            if size is not None:
                col_id = sim.createCollisionShape(shape, halfExtents=size)
                vis_id = sim.createCollisionShape(shape, halfExtents=size)
            shape_id = sim.createMultiBody(mass, col_id, vis_id, position)
        self.scene_items[name] = shape_id

        return shape_id

    def add_collision_constraints(self, constraint_id):
        self.collision_constraints.append(constraint_id)

    def load_urdf(self, name, urdf_file, position, orientation=None, use_fixed_base=False):

        if orientation is None:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position, useFixedBase=use_fixed_base)
        else:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position,
                                   baseOrientation=orientation, useFixedBase=use_fixed_base)
        self.scene_items[name] = urdf_id

        return urdf_id

    def load_robot(self, urdf_file, position, orientation=[0, 0, 0, 1], use_fixed_base=False):

        robot_id = sim.loadURDF(urdf_file, basePosition=position, baseOrientation=orientation,
                                useFixedBase=use_fixed_base,
                                    # flags=sim.URDF_USE_SELF_COLLISION
                                )

        self.setup_joint_id_to_joint_name(robot_id)
        self.joint_ids = [i for i in range(sim.getNumJoints(robot_id))]

        self.init_js_info(robot_id)

        return robot_id

    def init_js_info(self, robot_id):
        self.joint_name_to_info['base'] = JointInfo(*([-1, 'base'] + [None] * 15))
        self.joint_id_to_info[-1] = JointInfo(*([-1, 'base'] + [None] * 15))
        for joint_index in range(sim.getNumJoints(robot_id)):
            joint_info = JointInfo(*sim.getJointInfo(robot_id, joint_index))
            self.joint_name_to_info[joint_info.joint_name] = joint_info
            self.joint_id_to_info[joint_info.joint_index] = joint_info

        for body_a, body_b in itertools.combinations(self.joint_ids, 2):
            self.link_pairs[body_a].append(body_b)
            self.link_pairs[body_b].append(body_a)

        initial_distances = self.check_self_collision(robot_id, distance=0.05)
        # file_name1 = home + '/temp/initial_collision.xml'
        for (link_a, link_b) in initial_distances:
            self.ignored_collisions[link_a, link_b] = True
            self.ignored_collisions[link_b, link_a] = True
        #     with open(file_name1, 'a') as file_:
        #         file_.write( "<disable_collisions link1=\""+link_a+"\" link2=\""+ link_b + "\" reason=\"initial_collision\" />")
        #         file_.write("\n")

    def check_self_collision(self, robot_id, distance=0.001):
        contact_infos = [ClosestPointInfo(*x) for x in sim.getClosestPoints(robot_id, robot_id, distance)]
        distances = OrderedDict()
        for ci in contact_infos:
            link_a = self.joint_id_to_info[ci.link_index_a].link_name
            link_b = self.joint_id_to_info[ci.link_index_b].link_name
            if ci.link_index_a != ci.link_index_b and not self.ignored_collisions[link_a, link_b] and \
                            (link_b, link_a) not in distances:
                distances[link_a, link_b] = ci
        return distances

    def get_link_states_at(self, robot_id, trajectory, group):
        link_states = []
        self.reset_joint_states_to(robot_id, trajectory, group)
        for link_index in self.planning_group_ids:
            state = sim.getLinkState(robot_id, link_index, computeLinkVelocity=1,
                                     computeForwardKinematics=1)
            link_states.append(state)
        return link_states

    def get_joint_states_at(self, robot_id, trajectory, group):
        # self.reset_joint_states_to(robot_id, trajectory, group)
        # joint_ids = [self.joint_name_to_id[joint] for joint in group]
        # joint_states = sim.getJointStates(robot_id, joint_ids)
        # joint_positions = [state[0] for state in joint_states]
        # joint_velocities = [state[1] for state in joint_states]
        # joint_torques = [state[3] for state in joint_states]
        # return joint_positions, joint_velocities, joint_torques

        joint_states = sim.getJointStates(robot_id, range(sim.getNumJoints(robot_id)))

        joint_infos = [sim.getJointInfo(robot_id, i) for i in range(sim.getNumJoints(robot_id))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def get_joint_and_link_states_at(self, robot_id, trajectory, group):
        link_states = []
        joint_states = OrderedDict()
        joint_positions = []
        joint_velocities = []
        joint_torques = []

        self.reset_joint_states_to(robot_id, trajectory, group)

        for i in range(len(self.joint_ids)):
            if self.joint_id_to_name[i] in group:
                state = sim.getLinkState(robot_id, self.joint_ids[i], computeLinkVelocity=1,
                                     computeForwardKinematics=1)

                link_states.append(state)

            # joint_states[self.joint_id_to_name[i]] = sim.getJointState(robot_id, self.joint_ids[i])
            joint_state = sim.getJointState(robot_id, self.joint_ids[i])
            joint_info = sim.getJointInfo(robot_id, i)
            if joint_info[3] > -1:
                joint_positions.append(joint_state[0])
                joint_velocities.append(joint_state[1])
                joint_torques.append(joint_state[3])

        # for joint_name in group:
        #     joint_states[joint_name] = sim.getJointState(robot_id, self.joint_name_to_id[joint_name])
        #
        #     if joint_name in group:
        #         state = sim.getLinkState(robot_id, self.joint_name_to_id[joint_name], computeLinkVelocity=1,
        #                              computeForwardKinematics=1)
        #
        #         link_states.append(state)

        # joint_positions = [joint_states[state][0] for state in joint_states]
        # joint_velocities = [joint_states[state][1] for state in joint_states]
        # joint_torques = [joint_states[state][3] for state in joint_states]

        return [joint_positions, joint_velocities, joint_torques], link_states

    def extract_ids_from_planning_group(self, group):
        for joint in group:
            self.planning_group_ids.append(self.joint_name_to_id[joint])

    def get_collision_infos(self, robot, initial_trajectory, group, distance=0.10):
        # print initial_trajectory
        self.extract_ids_from_planning_group(group)
        collision_infos = self.formulate_collision_infos(robot, initial_trajectory, group, distance)
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
                [np.asarray(position_jacobian).reshape(1, 3, planning_group_length), jaco2])
            jacobian_matrix = np.hstack(jacobian_matrix)
        return jacobian_matrix

    def write_to_file(self):
        # print "----------------------------------------------"
        # print "time_step_count", time_step_count
        # print "link_index", link_index
        # print "current_robot_state", current_robot_state
        # print "current_link_states", current_link_state[0]
        #
        # print "next_robot_state", next_robot_state
        # print "next_link_states", next_link_state[0]
        # print "**********************************************"
        #
        print ("-----------cast points--------------")
        print ("collision pair", self.joint_id_to_info[body_a].link_name, self.joint_id_to_info[body_b].link_name)

        print ("bodyUniqueIdA", cast_closest_points[0][1])
        print ("bodyUniqueIdB", cast_closest_points[0][2])
        print ("linkIndexA", cast_closest_points[0][3])
        print ("linkIndexB", cast_closest_points[0][4])
        print ("A(t)", closest_pt_on_A_at_t)
        print ("A(t+1)", closest_pt_on_A_at_t_plus_1)
        print ("B", closest_pt_on_B)
        print ("normal", normal_.T)
        print ("Distance ", dist)
        print ("fraction ", fraction)
        print ("*************************************")

        with open(file_name, 'a') as file_:
            file_.write("---------------------------------------------------------------------")
            file_.write("\n")
            file_.write("---------------------------------------------------------------------")
            file_.write("\n")
            file_.write("collision pairs . . . . " + object_A + ", " + object_B)
            file_.write("\n")
            file_.write("each pair collision check time: " + str(end - start))
            file_.write("\n")
            file_.write("time_step_count: " + str(time_step_count))
            file_.write("\n")
            file_.write("link_index: " + str(link_index))
            file_.write("\n")
            file_.write("link_index_A: " + str(link_index_A))
            file_.write("\n")
            file_.write("link_index_B: " + str(link_index_B))
            file_.write("\n")

            # file_.write("current_robot_state", current_robot_state)
            # file_.write("current_link_states", current_link_state[0])
            # file_.write("next_robot_state", next_robot_state)
            # file_.write("next_link_states", next_link_state[0])
            file_.write("bodyUniqueIdA: " + str(cast_closest_points[0][1]))
            file_.write("\n")
            file_.write("bodyUniqueIdB: " + str(cast_closest_points[0][2]))
            file_.write("\n")
            file_.write("linkIndexA: " + str(cast_closest_points[0][3]))
            file_.write("\n")
            file_.write("linkIndexB: " + str(cast_closest_points[0][4]))
            file_.write("\n")
            file_.write("A(t): " + str(closest_pt_on_A_at_t))
            file_.write("\n")
            file_.write("A(t+1):" + str(closest_pt_on_A_at_t_plus_1))
            file_.write("\n")
            file_.write("B: " + str(closest_pt_on_B))
            file_.write("\n")
            file_.write("normal: " + str(normal_.T))
            file_.write("\n")
            file_.write("Distance: " + str(dist))
            file_.write("\n")
            file_.write("fraction: " + str(fraction))
            file_.write("\n")
            file_.write("**********************************************************************")
            file_.write("\n")
            file_.write("**********************************************************************")
            file_.write("\n \n")

    def print_contact_points(self, cp, time, index):

        link_a = self.joint_id_to_info[cp.link_index_a].link_name
        link_b = self.joint_id_to_info[cp.link_index_b].link_name

        print ("-----------cast points--------------")
        print ("time_step_count", time)
        print ("link_index", index)

        print ("collision pair", link_a, link_b)

        print ("bodyUniqueIdA", cp.body_unique_id_a)
        print ("bodyUniqueIdB", cp.body_unique_id_b)
        print ("linkIndexA", cp.link_index_a)
        print ("linkIndexB", cp.link_index_b)
        print ("A(t)", cp.position_on_a)
        print ("A(t+1)", cp.position_on_a1)
        print ("B", cp.position_on_b)
        print ("normal", cp.contact_normal_on_b)
        print ("Distance ", cp.contact_distance)
        print ("fraction ", cp.contact_fraction)
        print ("*************************************")

    def formulate_jacbian_matrix(self, robot_id, link_index, robot_state, link_state, cp, trajectory, group,
                                 time_step_count, zero_vec):
        link_position_in_world_frame = link_state[4]
        link_orentation_in_world_frame = link_state[5]
        closest_point_on_link_in_link_frame = self.get_point_in_local_frame(
            link_position_in_world_frame, link_orentation_in_world_frame,
            cp)

        position_jacobian, _ = sim.calculateJacobian( robot_id, link_index,
            # closest_pt_on_A_at_t,
            closest_point_on_link_in_link_frame,
            # [0, 0, 0],
            robot_state, zero_vec, zero_vec)

        current_position_jacobian1 = []
        current_position_jacobian1.append(
            [jac[-len(robot_state):] for jac in position_jacobian])

        jacobian_matrix = self.get_jacobian_matrix(current_position_jacobian1[0],
                                                                 len(trajectory),
                                                                 len(group),
                                                                 time_step_count)

        return jacobian_matrix

    def get_robot_self_collision_infos(self, robot_id, link_index, current_link_state, next_link_state, distance,
                                       current_robot_state, next_robot_state,
                                       zero_vec, trajectory, group, time_step_count,
                                       initial_signed_distance_,
                                       current_normal_T_times_jacobian_,
                                       next_normal_T_times_jacobian_, checked_collisions_pairs, to_plot):

        cast_closest_points = [CastClosestPointInfo(*x) for x in
                               sim.getConvexSweepClosestPoints(robot_id, robot_id,
                                                               linkIndexA=link_index,
                                                               # linkIndexB=link_index_B,
                                                               distance=distance,
                                                               bodyAfromPosition=current_link_state[
                                                                   0],
                                                               bodyAfromOrientation=
                                                               current_link_state[1],
                                                               # bodyAfromOrientation=[0, 0, 0, 1],
                                                               bodyAtoPosition=next_link_state[0],
                                                               bodyAtoOrientation=next_link_state[1],
                                                               # bodyAtoOrientation=[0, 0, 0, 1],
                                                               )]
        # print self.ignored_collisions
        # print self.joint_id_to_info

        for cp in cast_closest_points:
            if cp.link_index_a > -1 and cp.link_index_b > -1:
                link_a = self.joint_id_to_info[cp.link_index_a].link_name
                link_b = self.joint_id_to_info[cp.link_index_b].link_name
                visited = checked_collisions_pairs[time_step_count, link_a, link_b]

                if not visited and cp.link_index_a == link_index and cp.link_index_a != cp.link_index_b and not self.ignored_collisions[link_a, link_b]:

                    closest_pt_on_A_at_t = cp.position_on_a
                    closest_pt_on_A_at_t_plus_1 = cp.position_on_a1
                    normal_ = np.vstack(cp.contact_normal_on_b).reshape(3, 1)
                    cp._replace(contact_normal_on_b=utils.normalize_vector(normal_))
                    dist = cp.contact_distance
                    fraction = cp.contact_fraction

                    if dist < 0:
                        checked_collisions_pairs[time_step_count, link_a, link_b] = True

                        # collision_data = OrderedDict()
                        # collision_data["link_index"] = link_index
                        # collision_data["time_step_count"] = time_step_count
                        # collision_data["collision_data"] = cp._asdict()
                        # to_plot[link_a, link_b] = collision_data


                        current_state_jacobian_matrix = self.formulate_jacbian_matrix(robot_id, link_index,
                                                                      current_robot_state, current_link_state,
                                                                                      closest_pt_on_A_at_t,
                                                                    trajectory, group, time_step_count, zero_vec)

                        next_state_jacobian_matrix = self.formulate_jacbian_matrix(robot_id, link_index,
                                                                                      next_robot_state,
                                                                                      next_link_state,
                                                                                      closest_pt_on_A_at_t_plus_1,
                                                                                      trajectory, group,
                                                                                      time_step_count + 1, zero_vec)
                        initial_signed_distance_.append(dist)

                        current_normal_T_times_jacobian_.append(np.matmul(fraction * normal_.T,
                                                                          current_state_jacobian_matrix))

                        next_normal_T_times_jacobian_.append(np.matmul((1 - fraction) * normal_.T,
                                                                       next_state_jacobian_matrix))

    def get_collision_infos_for_constraints(self, robot, link_index, current_link_state, next_link_state, distance,
                                            current_robot_state, next_robot_state,
                                            zero_vec, trajectory, group, time_step_count,
                                            initial_signed_distance_,
                                            current_normal_T_times_jacobian_,
                                            next_normal_T_times_jacobian_):

        for constraint in self.collision_constraints:
            if robot.id != constraint:
                cast_closest_points = [CastClosestPointInfo(*x) for x in
                                       sim.getConvexSweepClosestPoints(robot.id, constraint,
                                                                       linkIndexA=link_index,
                                                                       linkIndexB=-1,
                                                                       distance=distance,
                                                                       bodyAfromPosition=current_link_state[
                                                                           0],
                                                                       bodyAfromOrientation=
                                                                       current_link_state[1],
                                                                       # bodyAfromOrientation=[0, 0, 0, 1],
                                                                       bodyAtoPosition=next_link_state[0],
                                                                       bodyAtoOrientation=next_link_state[1],
                                                                       # bodyAtoOrientation=[0, 0, 0, 1],
                                                                       )]

                # if len(cast_closest_points) > 0:
                for closest_point in cast_closest_points:
                    link_a = self.joint_id_to_info[closest_point.link_index_a].link_name
                    link_b = self.joint_id_to_info[closest_point.link_index_b].link_name
                    closest_pt_on_A_at_t = closest_point.position_on_a
                    closest_pt_on_A_at_t_plus_1 = closest_point.position_on_a1
                    closest_pt_on_B = closest_point.position_on_b
                    normal_ = np.vstack(closest_point.contact_normal_on_b).reshape(3, 1)
                    closest_point._replace(contact_normal_on_b=utils.normalize_vector(normal_))
                    dist = closest_point.contact_distance
                    fraction = closest_point.contact_fraction

                    # if dist < 0 and link_a is not None and link_b is not None:
                    if dist < 0:
                        self.print_contact_points(closest_point, time_step_count, link_index)
                        current_state_jacobian_matrix = self.formulate_jacbian_matrix(robot.id, link_index,
                                                                                      current_robot_state,
                                                                                      current_link_state,
                                                                                      closest_pt_on_A_at_t,
                                                                                      trajectory, group,
                                                                                      time_step_count, zero_vec)

                        next_state_jacobian_matrix = self.formulate_jacbian_matrix(robot.id, link_index,
                                                                                   next_robot_state,
                                                                                   next_link_state,
                                                                                   closest_pt_on_A_at_t_plus_1,
                                                                                   trajectory, group,
                                                                                   time_step_count + 1, zero_vec)

                        initial_signed_distance_.append(dist)

                        current_normal_T_times_jacobian_.append(np.matmul(fraction * normal_.T,
                                                                          current_state_jacobian_matrix))

                        next_normal_T_times_jacobian_.append(np.matmul((1 - fraction) * normal_.T,
                                                                       next_state_jacobian_matrix))



    def formulate_collision_infos(self, robot, trajectory, group, distance=0.2):

        initial_signed_distance = []
        current_normal_T_times_jacobian = []
        next_normal_T_times_jacobian = []
        start_state = self.get_current_states_for_given_joints(robot.id, group)
        time_step_count = 0
        checked_collisions_pairs = DefaultOrderedDict(bool)
        to_plot = OrderedDict()

        for previous_time_step_of_trajectory, current_time_step_of_trajectory, \
            next_time_step_of_trajectory in utils.iterate_with_previous_and_next(trajectory):
            time_step_count += 1

            if next_time_step_of_trajectory is not None:
                next_robot_state, next_link_states \
                    = self.get_joint_and_link_states_at(robot.id, next_time_step_of_trajectory, group)
                current_robot_state, current_link_states \
                    = self.get_joint_and_link_states_at(robot.id, current_time_step_of_trajectory, group)
                current_robot_state = current_robot_state[0]
                next_robot_state = next_robot_state[0]
                zero_vec = [0.0] * len(current_robot_state)

                # print "gjrg"
                # print len(zero_vec), len(current_robot_state)


                # self.reset_joint_states_to(robot_id, current_time_step_of_trajectory, group)


                for link_index, current_link_state, next_link_state in itertools.izip(self.planning_group_ids,
                                                                                      current_link_states,
                                                                                      next_link_states):
                    current_normal_T_times_jacobian_ = []
                    next_normal_T_times_jacobian_ = []
                    initial_signed_distance_ = []
                    start = time.time()

                    self.get_collision_infos_for_constraints(robot, link_index, current_link_state, next_link_state,
                                                             distance, current_robot_state, next_robot_state,
                                                             zero_vec, trajectory, group, time_step_count,
                                                             initial_signed_distance_,
                                                             current_normal_T_times_jacobian_,
                                                             next_normal_T_times_jacobian_)
                    end = time.time()

                    # print "each constraints collision check time:", str(end - start)
                    #
                    # start = time.time()
                    if True:
                        self.get_robot_self_collision_infos(robot.id, link_index, current_link_state, next_link_state,
                                                            distance, current_robot_state, next_robot_state,
                                                            zero_vec, trajectory, group, time_step_count,
                                                            initial_signed_distance_,
                                                            current_normal_T_times_jacobian_,
                                                            next_normal_T_times_jacobian_, checked_collisions_pairs,
                                                            to_plot)

                    # end = time.time()
                    # print "each self collision check time:", str(end - start)
                if len(initial_signed_distance_) > 0:
                    initial_signed_distance.append(initial_signed_distance_)
                if len(current_normal_T_times_jacobian_) > 0:
                    current_normal_T_times_jacobian.append(current_normal_T_times_jacobian_)
                if len(next_normal_T_times_jacobian_) > 0:
                    next_normal_T_times_jacobian.append(next_normal_T_times_jacobian_)

        # print to_plot
        # config.save_to_file(home + "/temp/collision_data.yaml", to_plot, mode="a")

        if len(initial_signed_distance) > 0:
            initial_signed_distance = np.vstack(itertools.chain.from_iterable(initial_signed_distance))
        if len(current_normal_T_times_jacobian) > 0:
            current_normal_T_times_jacobian = np.vstack(itertools.chain.from_iterable(current_normal_T_times_jacobian))

        if len(next_normal_T_times_jacobian) > 0:
            next_normal_T_times_jacobian = np.vstack(itertools.chain.from_iterable(next_normal_T_times_jacobian))

        self.reset_joint_states(robot.id, start_state, group)

        return initial_signed_distance, current_normal_T_times_jacobian, next_normal_T_times_jacobian

    def get_point_in_local_frame(self, frame_position, frame_orientation, point, inverse=True):
        # frame = kdl.Frame()
        # print frame_orientation
        rotation = kdl.Rotation.Quaternion(frame_orientation[0], frame_orientation[1], frame_orientation[2],
                                           frame_orientation[3])
        position = kdl.Vector(frame_position[0], frame_position[1], frame_position[2])
        frame = kdl.Frame(rotation, position)
        point = kdl.Vector(point[0], point[1], point[2])

        if inverse:
            point_on_frame = frame.Inverse() * point
        else:
            point_on_frame = frame * point

        return [point_on_frame[0], point_on_frame[1], point_on_frame[2]]

    def update_collsion_infos(self, no_of_samples, new_trajectory, delta_trajectory=None):
        trajectory = np.array((np.split(new_trajectory, no_of_samples)))

        self.robot.planner.trajectory.add_trajectory(trajectory)
        trajectory = np.split(new_trajectory, self.planning_samples)
        collision_infos = self.get_collision_infos(trajectory, self.planning_group,
                                                   distance=self.collision_check_distance)

        constraints, lower_limit, upper_limit = self.robot.planner.problem_model.update_collision_infos(collision_infos,
                                                                                                        self.collision_safe_distance)
        if len(collision_infos[2]) > 0:
            self.robot.planner.update_prob()

        return constraints, lower_limit, upper_limit

    def plan_trajectory(self, group, goal_state, samples, duration, solver_config=None, collision_safe_distance=None,
                        collision_check_distance=0.2):
        # self.collision_constraints = OrderedDict()
        self.planning_group = group
        self.planning_samples = samples
        self.collision_safe_distance = collision_safe_distance
        self.collision_check_distance = collision_check_distance

        self.extract_ids_from_planning_group(group)
        self.robot.init_plan_trajectory(group=group, current_state=self.get_current_states_for_given_joints(group),
                                        goal_state=goal_state, samples=int(samples),
                                        duration=int(duration),
                                        # collision_constraints=self.collision_constraints,
                                        solver_config=solver_config)
        self.toggle_rendering_while_planning(False)
        status, can_execute_trajectory = self.robot.calulate_trajecotory(self.update_collsion_infos)  # callback function
        self.toggle_rendering_while_planning(True)
        # status, can_execute_trajectory = self.robot.calulate_trajecotory(None)

        return status, can_execute_trajectory

    def get_current_states_for_given_joints(self, robot_id, group):
        current_state = []
        for joint in group:
            current_state.append(sim.getJointState(bodyUniqueId=robot_id, jointIndex=self.joint_name_to_id[joint])[0])
        return current_state

    def execute_trajectory(self, robot, trajectory, step_time=None):
        if step_time is None:
            sleep_time = trajectory.duration / float(trajectory.no_of_samples)
            for each_time_step_trajectory in trajectory.final:
                # print each_time_step_trajectory
                # print trajectory.trajectory_by_name.keys()
                self.reset_joint_states_to(robot.id, each_time_step_trajectory, trajectory.trajectory_by_name.keys())
                time.sleep(0.5)
            # for joint_name, corresponding_trajectory in trajectory.final:
            #     print corresponding_trajectory
            #     print trajectory.trajectory_by_name.keys()
                # self.reset_joint_states_to(robot.id, corresponding_trajectory, trajectory.trajectory_by_name.keys())
                # time.sleep(sleep_time)

        # for i in range(int(trajectory.no_of_samples)):
        #     for joint_name, corresponding_trajectory in trajectory.trajectory_by_name.items():
        #         # if robot.model.joint_map[joint_name].type == "prismatic":
        #         #     sim.setJointMotorControl2(bodyIndex=robot.id, jointIndex=self.joint_name_to_id[joint_name],
        #         #                           controlMode=sim.VELOCITY_CONTROL,
        #         #                           targetPosition=0,
        #         #                           targetVelocity=corresponding_trajectory[i],
        #         #                           force=robot.model.joint_map[joint_name].limit.effort,
        #         #                           positionGain=0.03,
        #         #                           velocityGain=.5,
        #         #                           maxVelocity=float(robot.model.joint_map[joint_name].limit.velocity)
        #         #                           )
        #         # else:
        #         sim.setJointMotorControl2(bodyIndex=robot.id, jointIndex=self.joint_name_to_id[joint_name],
        #                               controlMode=sim.POSITION_CONTROL,
        #                               targetPosition=corresponding_trajectory[i], targetVelocity=0,
        #                               force=robot.model.joint_map[joint_name].limit.effort,
        #                               positionGain=0.03,
        #                               velocityGain=.5,
        #                               maxVelocity=float(robot.model.joint_map[joint_name].limit.velocity)
        #                               )

            # self.step_simulation_for(sleep_time)

        status = "Trajectory execution has finished"
        self.logger.info(status)
        return status

    def execute_trajectories(self, robot, group, trajectories):
        sleep_time = trajectories.duration / float(trajectories.no_of_samples)

        for i in range(int(trajectories.no_of_samples)):
            for trajectory in trajectories.trajectories:
                for joint_name, corresponding_trajectory in trajectory:
                    sim.setJointMotorControl2(bodyIndex=robot.id, jointIndex=self.joint_name_to_id[joint_name],
                                              controlMode=sim.POSITION_CONTROL,
                                              targetPosition=corresponding_trajectory[i], targetVelocity=0,
                                              force=robot.model.joint_map[joint_name].limit.effort,
                                              positionGain=0.03,
                                              velocityGain=.5,
                                              # maxVelocity=float(self.robot.model.joint_map[joint_name].limit.velocity)
                                              )

                self.step_simulation_for(sleep_time)

        status = "Trajectories execution has finished"
        self.logger.info(status)
        return status

    def plan_and_execute_trajectory(self, group, goal_state, samples, duration, solver_config=None,
                                    collision_safe_distance=0.05, collision_check_distance=0.1):
        status = "-1"
        status, can_execute_trajectory = self.plan_trajectory(group, goal_state, samples, duration,
                                                              solver_config=solver_config,
                                                              collision_safe_distance=collision_safe_distance,
                                                              collision_check_distance=collision_check_distance)
        status += " and "
        status += self.execute_trajectory()

        return status, can_execute_trajectory

    def is_given_robot_state_has_collision(self, robot_id, robot_current_state_position,
                                           robot_current_state_orientation=[0, 0, 0, 1], collision_safe_distance=0.05):
        cast_closest_points = sim.getClosestPoints(robot_id, distance=collision_safe_distance
                                                              )

        if len(cast_closest_points) > 0:
            dist = cast_closest_points[0][9]
            if dist < 0:
                return True
        return False

    def is_trajectory_collision_free(self, robot_id, trajectory, group, collision_safe_distance=0.05):
        collision = True
        start_state = self.get_current_states_for_given_joints(robot_id, group)
        distance = 10
        for previous_time_step_of_trajectory, current_time_step_of_trajectory, \
            next_time_step_of_trajectory in utils.iterate_with_previous_and_next(trajectory):

            if next_time_step_of_trajectory is not None:
                next_link_states = self.get_link_states_at(robot_id, next_time_step_of_trajectory, group)
            current_link_states = self.get_link_states_at(robot_id, current_time_step_of_trajectory, group)

            self.reset_joint_states_to(robot_id, current_time_step_of_trajectory, group)

            for link_index, current_link_state, next_link_state in itertools.izip(self.planning_group_ids,
                                                                                  current_link_states,
                                                                                  next_link_states):

                for constraint in self.collision_constraints:
                    if next_link_state is not None:
                        cast_closest_points = sim.getConvexSweepClosestPoints(robot_id, constraint,
                                                                              linkIndexA=link_index,
                                                                              distance=collision_safe_distance,
                                                                              bodyAfromPosition=current_link_state[0],
                                                                              bodyAfromOrientation=current_link_state[
                                                                                  1],
                                                                              # bodyAfromOrientation=[0, 0, 0, 1],
                                                                              bodyAtoPosition=next_link_state[0],
                                                                              bodyAtoOrientation=next_link_state[1],
                                                                              # bodyAtoOrientation=[0, 0, 0, 1],
                                                                              )

                        if len(cast_closest_points) > 0:
                            distance = cast_closest_points[0][9]
                    else:
                        closest_points = sim.getClosestPoints(robot_id, constraint,
                                                              linkIndexA=link_index, distance=collision_safe_distance)
                        if len(closest_points) > 0:
                            distance = cast_closest_points[0][8]

                    if distance < 0:
                        collision = False
                        break

                cast_closest_points = [CastClosestPointInfo(*x) for x in
                                       sim.getConvexSweepClosestPoints(robot_id, robot_id,
                                                                       # linkIndexA=link_index_A,
                                                                       # linkIndexB=link_index_B,
                                                                       distance=distance,
                                                                       bodyAfromPosition=current_link_state[
                                                                           0],
                                                                       bodyAfromOrientation=
                                                                       current_link_state[1],
                                                                       # bodyAfromOrientation=[0, 0, 0, 1],
                                                                       bodyAtoPosition=next_link_state[0],
                                                                       bodyAtoOrientation=next_link_state[1],
                                                                       # bodyAtoOrientation=[0, 0, 0, 1],
                                                                       )]

                for cp in cast_closest_points:
                    if cp.link_index_a > -1 and cp.link_index_b > -1:
                        link_a = self.joint_id_to_info[cp.link_index_a].link_name
                        link_b = self.joint_id_to_info[cp.link_index_b].link_name


                        if cp.link_index_a == link_index and cp.link_index_a != cp.link_index_b and not \
                        self.ignored_collisions[link_a, link_b]:
                            dist = cp.contact_distance
                            if dist < 0:
                                collision = False
                                break

                if not collision:
                    break
            if not collision:
                break

        self.reset_joint_states(robot_id, start_state, group)

        return collision

    def setup_joint_id_to_joint_name(self, robot_id):
        for i in range(sim.getNumJoints(robot_id)):
            joint_info = sim.getJointInfo(robot_id, i)
            self.joint_name_to_id[joint_info[1].decode('UTF-8')] = joint_info[0]

        self.joint_id_to_name = OrderedDict(zip(self.joint_name_to_id.values(), self.joint_name_to_id.keys()))

    def reset_joint_states_to(self, robot_id, trajectory, joints):
        if len(trajectory) == len(joints):
            for i in range(len(trajectory)):
                sim.resetJointState(robot_id, self.joint_name_to_id[joints[i]], trajectory[i])
                status = "Reset joints to start pose is complete"

        else:
            status = "cannot reset the joint states as the trajectory and joints size doesn't match"

        # self.logger.info(status)
        return status

    def reset_joints_to_random_states(self, robot_id, joints):

        motor_dir = np.random.uniform(-1, 1, size=len(joints))
        half_pi = 1.57079632679
        for j, joint in enumerate(joints):
            # motor_dir = np.random.uniform(robot.model.joint_map[joint].limit.lower,
            #                               robot.model.joint_map[joint].limit.upper, size=1)
            sim.resetJointState(robot_id, self.joint_name_to_id[joint], motor_dir[j] * half_pi * (-(-1)**j))
        status = "Reset joints to random pose is complete"
        self.logger.info(status)
        return status

    def reset_joint_states(self, robot_id, joints, group):
        # assert len(joints) == len(group)
        for i in range(len(group)):
            sim.resetJointState(robot_id, self.joint_name_to_id[group[i]], joints[i])
        status = "Reset joints to start pose is complete"
        # self.logger.info(status)
        return status

    def reset_objects_to(self, object_id, position=None, orientation=None):
        if position is None:
            position = [0.28, -0.43, 0.98]
        if orientation is None:
            orientation = [0, 0, 0, 1]
        sim.resetBasePositionAndOrientation(object_id, position, orientation)

    def get_joint_states(self, robot_id, group):
        joint_states = sim.getJointStates(robot_id, [self.joint_name_to_id[joint_name] for joint_name in group])
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def step_simulation_for(self, seconds):
        start = time.time()
        while time.time() < start + seconds:
            sim.stepSimulation()
