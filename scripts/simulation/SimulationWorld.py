import logging
import pybullet as sim
import pybullet_data
import time
import numpy as np
from scripts.interfaces.ISimulationWorldBase import ISimulationWorldBase
import itertools
from scripts.utils.utils import Utils as utils
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
from scripts.simulation.bulletTypes import *
from scripts.utils.yaml_paser import ConfigParser


class SimulationWorld(ISimulationWorldBase):
    def __init__(self, **kwargs):

        use_gui = utils.get_var_from_kwargs("use_gui", optional=True, default=False, **kwargs)
        verbose = utils.get_var_from_kwargs("verbose", optional=True, default=False, **kwargs)
        log_file = utils.get_var_from_kwargs("log_file", optional=True, default=False, **kwargs)
        use_real_time_simulation = utils.get_var_from_kwargs("use_real_time_simulation", optional=True,
                                                             default=False, **kwargs)
        fixed_time_step = utils.get_var_from_kwargs("fixed_time_step", optional=True,  default=0.01, **kwargs)
        logger_name = utils.get_var_from_kwargs("logger_name", optional=True,  default=__name__, **kwargs)

        self.CYLINDER = sim.GEOM_CYLINDER
        self.BOX = sim.GEOM_BOX
        self.MESH = sim.GEOM_MESH

        self.logger = logging.getLogger(logger_name + __name__)
        utils.setup_logger(self.logger, logger_name, verbose, log_file)

        if use_gui:
            self.gui = sim.connect(sim.GUI_SERVER)
            # self.gui = sim.connect(sim.GUI)
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
        self.joint_name_to_jac_id = OrderedDict()
        self.ignored_collisions = DefaultOrderedDict(bool)
        self.link_pairs = DefaultOrderedDict(list)
        self.robot_info = DefaultOrderedDict(list)
        self.planning_group = []
        self.planning_group_ids = []
        self.joint_ids = []
        self.planning_samples = 0
        self.collision_safe_distance = 0.4
        self.collision_check_distance = 0.2
        self.collision_check_time = 0
        self.robot = None

        if use_real_time_simulation:
            sim.setRealTimeSimulation(use_real_time_simulation)
        else:
            sim.setTimeStep(fixed_time_step)

        self.collision_constraints = []

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

    def create_constraint(self, name, shape, mass, position, orientation=None,
                          size=None, radius=None, height=None):
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if position is not None:
            if radius is not None:
                col_id = sim.createCollisionShape(shape, radius=radius, height=height)
                vis_id = sim.createCollisionShape(shape, radius=radius, height=height)
            if size is not None:
                col_id = sim.createCollisionShape(shape, halfExtents=size)
                vis_id = sim.createCollisionShape(shape, halfExtents=size)

        shape_id = sim.createMultiBody(mass, col_id, vis_id, basePosition=position, baseOrientation=orientation)
        self.scene_items[shape_id] = name

        return shape_id

    def create_constraint_from_mesh(self, name, file_name, mass=1, position=None, orientation=None,
                                    mesh_scale=None,
                                    visual_frame_shift=None, collision_frame_shift=None,
                                    specularColor=None,
                                    rgba_color=None, use_maximalcoordinates=True):
        if position is None:
            position = [0, 0, 0]
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if mesh_scale is None:
            mesh_scale = [1, 1, 1]
        if rgba_color is None:
            rgba_color = [1, 1, 1, 1]
        if visual_frame_shift is None:
            visual_frame_shift = [0, 0, 0]
        if collision_frame_shift is None:
            collision_frame_shift = [0, 0, 0]
        if specularColor is None:
            specularColor = [0, 0, 0]

        if file_name is not None:
            vis_id = sim.createVisualShape(shapeType=sim.GEOM_MESH, fileName=file_name,
                                                rgbaColor=rgba_color,
                                                specularColor=specularColor,
                                                visualFramePosition=visual_frame_shift,
                                                meshScale=mesh_scale
                                                )
            col_id = sim.createCollisionShape(shapeType=sim.GEOM_MESH, fileName=file_name,
                                                      collisionFramePosition=collision_frame_shift,
                                                      meshScale=mesh_scale
                                                      )
        shape_id = sim.createMultiBody(baseMass=mass, basePosition=position, baseOrientation=orientation,
                                       baseCollisionShapeIndex=col_id,
                                        baseVisualShapeIndex=vis_id, useMaximalCoordinates=use_maximalcoordinates)
        self.scene_items[shape_id] = name

        return shape_id

    def add_collision_constraints(self, constraint_id):
        self.collision_constraints.append(constraint_id)

    def load_urdf(self, name, urdf_file, position, orientation=None, use_fixed_base=False):

        if orientation is None:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position, useFixedBase=use_fixed_base)
        else:
            urdf_id = sim.loadURDF(urdf_file, basePosition=position,
                                   baseOrientation=orientation, useFixedBase=use_fixed_base)
        self.scene_items[urdf_id] = name

        return urdf_id

    def load_robot(self, urdf_file, position, orientation=None, use_fixed_base=False):
        # utils.replace_paths(urdf_file)
        # urdf_file = "/tmp/robot.urdf"
        if orientation is None:
            orientation = [0, 0, 0, 1]
        if len(orientation) == 3:
            orientation = sim.getQuaternionFromEuler(orientation)
        robot_id = sim.loadURDF(urdf_file, basePosition=position, baseOrientation=orientation,
                                useFixedBase=use_fixed_base,
                                    # flags=sim.URDF_USE_SELF_COLLISION
                                )

        self.setup_joint_id_to_joint_name(robot_id)
        self.joint_ids = [i for i in range(sim.getNumJoints(robot_id))]

        self.init_js_info(robot_id)
        self.robot_info["id"] = robot_id
        self.robot_info["joint_infos"] = self.joint_id_to_info

        return robot_id

    def init_js_info(self, robot_id):
        self.joint_name_to_info['base'] = JointInfo(*([-1, 'base'] + [None] * 15))
        self.joint_id_to_info[-1] = JointInfo(*([-1, 'base'] + [None] * 15))
        dof_count = 0
        for joint_index in range(sim.getNumJoints(robot_id)):
            joint_info = JointInfo(*sim.getJointInfo(robot_id, joint_index))
            self.joint_name_to_info[joint_info.joint_name] = joint_info
            self.joint_id_to_info[joint_info.joint_index] = joint_info
            if joint_info.q_index > -1:
                self.joint_name_to_jac_id[joint_info.joint_name] = dof_count
                dof_count += 1

        for body_a, body_b in itertools.combinations(self.joint_ids, 2):
            self.link_pairs[body_a].append(body_b)
            self.link_pairs[body_b].append(body_a)
        # initial_distances = self.check_self_collision(robot_id, distance=0.05)
        # for (link_a, link_b) in initial_distances:
        #     self.ignored_collisions[link_a, link_b] = True
        #     self.ignored_collisions[link_b, link_a] = True

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
        self.reset_joint_states_to(robot_id, trajectory, group)
        joint_states = sim.getJointStates(robot_id, range(sim.getNumJoints(robot_id)))
        joint_infos = [sim.getJointInfo(robot_id, i) for i in range(sim.getNumJoints(robot_id))]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def get_joint_and_link_states_at(self, robot_id, trajectory, group):
        link_states = []
        joint_positions = []
        joint_velocities = []
        joint_torques = []
        # print "start . .. . . . ............................."
        self.reset_joint_states_to(robot_id, trajectory, group)

        for i in range(len(self.joint_ids)):
            if self.joint_id_to_name[i] in group:
                state = sim.getLinkState(robot_id, self.joint_ids[i], computeLinkVelocity=1,
                                     computeForwardKinematics=1)

                link_states.append(state)

            joint_state = sim.getJointState(robot_id, self.joint_ids[i])
            joint_info = sim.getJointInfo(robot_id, i)
            inf = JointInfo(*joint_info)
            if joint_info[3] > -1:
                # print inf

                joint_positions.append(joint_state[0])
                joint_velocities.append(joint_state[1])
                joint_torques.append(joint_state[3])
        # print "end ******************************************"
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
        jacobian_matrix = None
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
            jacobian_matrix = np.hstack([jaco1, np.asarray(position_jacobian)])
        elif len(jaco1) == 0 and len(jaco2) > 0:
            jacobian_matrix = np.vstack(
                [np.asarray(position_jacobian).reshape(1, 3, planning_group_length), jaco2])
            jacobian_matrix = np.hstack(jacobian_matrix)
        return jacobian_matrix

    def print_contact_points(self, cp, time, index):

        link_a = self.joint_id_to_info[cp.link_index_a].link_name
        if cp.body_unique_id_a == cp.body_unique_id_b:
            link_b = self.joint_id_to_info[cp.link_index_b].link_name
        else:
            # link_b = JointInfo(*sim.getJointInfo(cp.body_unique_id_b, cp.link_index_b)).link_name
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

    def get_convex_sweep_closest_points(self, body_a, body_b, link_index_a, current_state, next_state, distance=0.1):
        start = time.time()
        cast_closest_points = [CastClosestPointInfo(*x) for x in
                               sim.getConvexSweepClosestPoints(body_a,
                                                               bodyB=body_b,
                                                               linkIndexA=link_index_a,
                                                               distance=distance,
                                                               bodyAfromPosition=current_state[0],
                                                               bodyAfromOrientation=current_state[1],
                                                               bodyAtoPosition=next_state[0],
                                                               bodyAtoOrientation=next_state[1]
                                                               )]
        # if type(body_b) is long:
        #     cast_closest_points = [CastClosestPointInfo(*x) for x in
        #                            sim.getConvexSweepClosestPoints(body_a,
        #                                                            # bodyB=constraint,
        #                                                            bodyB=body_b,
        #                                                            linkIndexA=link_index_a,
        #                                                            # linkIndexB=link_index_B,
        #                                                            distance=distance,
        #                                                            bodyAfromPosition=current_state[0],
        #                                                            bodyAfromOrientation=current_state[1],
        #                                                            # bodyAfromOrientation=[0, 0, 0, 1],
        #                                                            bodyAtoPosition=next_state[0],
        #                                                            bodyAtoOrientation=next_state[1],
        #                                                            # bodyAtoOrientation=[0, 0, 0, 1],
        #                                                            # bodyUniqueIdBIndices=[2, 3, 4, 5],
        #                                                            #  bodyUniqueIdBIndices=constraint,
        #                                                            # linkIndexBIndices=[2, 3, 4, 5]
        #                                                            )]
        # elif type(body_b) is list or type(body_b) is tuple:
        #     cast_closest_points = [CastClosestPointInfo(*x) for x in
        #                            sim.getConvexSweepClosestPoints(body_a,
        #                                                            bodyB=-1,
        #                                                            linkIndexA=link_index_a,
        #                                                            # linkIndexB=link_index_B,
        #                                                            distance=distance,
        #                                                            bodyAfromPosition=current_state[0],
        #                                                            bodyAfromOrientation=current_state[1],
        #                                                            # bodyAfromOrientation=[0, 0, 0, 1],
        #                                                            bodyAtoPosition=next_state[0],
        #                                                            bodyAtoOrientation=next_state[1],
        #                                                            # bodyAtoOrientation=[0, 0, 0, 1],
        #                                                             bodyUniqueIdBIndices=body_b,
        #                                                            )]

        end = time.time()
        self.collision_check_time += end - start

        return cast_closest_points

    def formulate_jacbian_matrix(self, robot_id, link_index, robot_state, link_state, cp, trajectory, group,
                                 time_step_count, zero_vec):
        link_position_in_world_frame = link_state[4]
        link_orentation_in_world_frame = link_state[5]
        closest_point_on_link_in_link_frame, _ = self.get_point_in_local_frame(
            link_position_in_world_frame, link_orentation_in_world_frame,
            cp)

        position_jacobian, _ = sim.calculateJacobian( robot_id, link_index,
            closest_point_on_link_in_link_frame,
            robot_state, zero_vec, zero_vec)
        position_jacobian = [jac[-len(robot_state):] for jac in position_jacobian]

        current_position_jacobian1 = []
        # if len(group) < len(robot_state):
        #     current_position_jacobian1.append(
        #         # [jac[3:9] for jac in position_jacobian])
        #         [jac[3:] for jac in position_jacobian])
        # else:
        #     current_position_jacobian1.append(
        #         [jac[-len(robot_state):] for jac in position_jacobian])

        group_jacobian = [[x[self.joint_name_to_jac_id[g]] for g in group] for x in position_jacobian]

        jacobian_matrix = self.get_jacobian_matrix(group_jacobian, len(trajectory), len(group), time_step_count)

        return jacobian_matrix

    def get_robot_self_collision_infos(self, robot_id, link_index, current_link_state, next_link_state, distance,
                                       current_robot_state, next_robot_state,
                                       zero_vec, trajectory, group, time_step_count,
                                       initial_signed_distance,
                                       current_normal_T_times_jacobian,
                                       next_normal_T_times_jacobian):

        cast_closest_points = self.get_convex_sweep_closest_points(robot_id, robot_id, link_index,
                                                                   current_link_state, next_link_state, distance)
        for cp in cast_closest_points:
            if cp.link_index_a > -1 and cp.link_index_b > -1:
                link_a = self.joint_id_to_info[cp.link_index_a].link_name
                link_b = self.joint_id_to_info[cp.link_index_b].link_name

                if cp.link_index_a == link_index and cp.link_index_a != cp.link_index_b and not self.ignored_collisions[link_a, link_b]:

                    closest_pt_on_A_at_t = cp.position_on_a
                    closest_pt_on_A_at_t_plus_1 = cp.position_on_a1
                    normal_ = 1 * np.vstack(cp.contact_normal_on_b).reshape(3, 1)
                    normal_ = utils.normalize_vector(normal_)
                    cp._replace(contact_normal_on_b=normal_)
                    dist = cp.contact_distance
                    fraction = cp.contact_fraction

                    if dist < 0:
                        # self.print_contact_points(cp, time_step_count, link_index)
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
                        initial_signed_distance.append(dist)

                        current_normal_T_times_jacobian.append(np.matmul(fraction * normal_.T,
                                                                         current_state_jacobian_matrix))

                        next_normal_T_times_jacobian.append(np.matmul((1 - fraction) * normal_.T,
                                                                      next_state_jacobian_matrix))

    def get_collision_infos_for_constraints(self, robot, link_index, current_link_state, next_link_state, distance,
                                            current_robot_state, next_robot_state,
                                            zero_vec, trajectory, group, time_step_count,
                                            initial_signed_distance_,
                                            current_normal_T_times_jacobian_,
                                            next_normal_T_times_jacobian_):

        for constraint in self.collision_constraints:
            if robot.id != constraint:

                cast_closest_points = self.get_convex_sweep_closest_points(robot.id,
                                                                           # self.collision_constraints,
                                                                           constraint,
                                                                           link_index,
                                                                           current_link_state, next_link_state,
                                                                           distance)

                for closest_point in cast_closest_points:
                    link_a = self.joint_id_to_info[closest_point.link_index_a].link_name
                    link_b = self.joint_id_to_info[closest_point.link_index_b].link_name
                    closest_pt_on_A_at_t = closest_point.position_on_a
                    closest_pt_on_A_at_t_plus_1 = closest_point.position_on_a1
                    normal_ = np.vstack(closest_point.contact_normal_on_b).reshape(3, 1)
                    normal_ = utils.normalize_vector(normal_)
                    closest_point._replace(contact_normal_on_b=normal_)
                    dist = closest_point.contact_distance
                    fraction = closest_point.contact_fraction

                    if closest_point.link_index_a == link_index and dist < 0:
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

                for link_index, current_link_state, next_link_state in itertools.izip(self.planning_group_ids,
                                                                                      current_link_states,
                                                                                      next_link_states):

                    self.get_collision_infos_for_constraints(robot, link_index, current_link_state, next_link_state,
                                                         distance, current_robot_state, next_robot_state,
                                                         zero_vec, trajectory, group, time_step_count,
                                                         initial_signed_distance,
                                                         current_normal_T_times_jacobian,
                                                         next_normal_T_times_jacobian)

                    self.get_robot_self_collision_infos(robot.id, link_index, current_link_state, next_link_state,
                                                        distance, current_robot_state, next_robot_state,
                                                        zero_vec, trajectory, group, time_step_count,
                                                        initial_signed_distance,
                                                        current_normal_T_times_jacobian,
                                                        next_normal_T_times_jacobian)

        if len(initial_signed_distance) > 0:
            initial_signed_distance = np.vstack(initial_signed_distance)
        if len(current_normal_T_times_jacobian) > 0:
            current_normal_T_times_jacobian = np.vstack(current_normal_T_times_jacobian)
        if len(next_normal_T_times_jacobian) > 0:
            next_normal_T_times_jacobian = np.vstack(next_normal_T_times_jacobian)

        self.reset_joint_states(robot.id, start_state, group)

        return initial_signed_distance, current_normal_T_times_jacobian, next_normal_T_times_jacobian

    def get_point_in_local_frame(self, frame_position, frame_orientation, point, orn=None, inverse=True):
        if orn is None:
            orn = [0, 0, 0, 1]
        if inverse:
            inv_trans, inv_orn = sim.invertTransform(frame_position, frame_orientation)
            point_on_frame = sim.multiplyTransforms(inv_trans, inv_orn, point, [0, 0, 0, 1])
        else:
            point_on_frame = sim.multiplyTransforms(frame_position, frame_orientation, point, orn)

        return point_on_frame

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

    def execute_trajectory(self, robot, trajectory, step_time=0.5):
        for each_time_step_trajectory in trajectory.final:
            self.reset_joint_states_to(robot.id, each_time_step_trajectory, trajectory.trajectory_by_name.keys())
            time.sleep(step_time)
        status = "Trajectory execution has finished"
        self.logger.info(status)
        return status

    def plan_and_execute_trajectory(self, group, goal_state, samples, duration, solver_config=None,
                                    collision_safe_distance=0.05, collision_check_distance=0.1):
        status, can_execute_trajectory = self.plan_trajectory(group, goal_state, samples, duration,
                                                              solver_config=solver_config,
                                                              collision_safe_distance=collision_safe_distance,
                                                              collision_check_distance=collision_check_distance)
        status += " and "
        status += self.execute_trajectory()

        return status, can_execute_trajectory

    def is_trajectory_collision_free(self, robot_id, trajectory, group, collision_safe_distance=0.05):
        collision_free = True
        start_state = self.get_current_states_for_given_joints(robot_id, group)
        time_step_count = 0

        for previous_time_step_of_trajectory, current_time_step_of_trajectory, \
            next_time_step_of_trajectory in utils.iterate_with_previous_and_next(trajectory):
            time_step_count += 1
            if next_time_step_of_trajectory is not None:
                next_link_states = self.get_link_states_at(robot_id, next_time_step_of_trajectory, group)
                current_link_states = self.get_link_states_at(robot_id, current_time_step_of_trajectory, group)

                self.reset_joint_states_to(robot_id, current_time_step_of_trajectory, group)

                for link_index, current_link_state, next_link_state in itertools.izip(self.planning_group_ids,
                                                                                      current_link_states,
                                                                                      next_link_states):
                    if next_link_state is not None:
                        for constraint in self.collision_constraints:

                            cast_closest_points = [CastClosestPointInfo(*x) for x in
                                                   sim.getConvexSweepClosestPoints(robot_id, constraint,
                                                                                   linkIndexA=link_index,
                                                                                   distance=collision_safe_distance,
                                                                                   bodyAfromPosition=current_link_state[0],
                                                                                   bodyAfromOrientation=current_link_state[
                                                                                       1],
                                                                                   # bodyAfromOrientation=[0, 0, 0, 1],
                                                                                   bodyAtoPosition=next_link_state[0],
                                                                                   bodyAtoOrientation=next_link_state[1],
                                                                                   # bodyAtoOrientation=[0, 0, 0, 1],
                                                                                   )]


                            for cp in cast_closest_points:
                                dist = cp.contact_distance
                                if dist < 0 and cp.body_unique_id_a != cp.body_unique_id_b:
                                    # self.print_contact_points(cp, time_step_count, link_index)
                                    collision_free = False
                                    break

                        cast_closest_points = [CastClosestPointInfo(*x) for x in
                                               sim.getConvexSweepClosestPoints(robot_id, robot_id,
                                                                               linkIndexA=link_index,
                                                                               # linkIndexB=link_index_B,
                                                                               distance=collision_safe_distance,
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
                                        # self.print_contact_points(cp, time_step_count, link_index)
                                        collision_free = False
                                        break

                    if not collision_free:
                        break
                if not collision_free:
                    break

        self.reset_joint_states(robot_id, start_state, group)

        return collision_free

    def is_given_state_in_collision(self, robot_id, state, group, distance=0.02):
        collision = False
        start_state = self.get_current_states_for_given_joints(robot_id, group)
        time_step_count = 0
        self.reset_joint_states_to(robot_id, state, group)
        for link_index in group:
            link_index = self.joint_name_to_id[link_index]
            for constraint in self.collision_constraints:
                closest_points = [ClosestPointInfo(*x) for x in
                                       sim.getClosestPoints(robot_id, constraint,
                                                            linkIndexA=link_index, distance=distance)]

                for cp in closest_points:
                    dist = cp.contact_distance
                    if dist < 0 and cp.body_unique_id_a != cp.body_unique_id_b:
                        # self.print_contact_points(cp, time_step_count, link_index)
                        collision = True
                        break

            closest_points = [ClosestPointInfo(*x) for x in
                                   sim.getClosestPoints(robot_id, robot_id,
                                                        linkIndexA=link_index, distance=distance)]

            for cp in closest_points:
                if cp.link_index_a > -1 and cp.link_index_b > -1:
                    link_a = self.joint_id_to_info[cp.link_index_a].link_name
                    link_b = self.joint_id_to_info[cp.link_index_b].link_name

                    if cp.link_index_a == link_index and cp.link_index_a != cp.link_index_b and not \
                            self.ignored_collisions[link_a, link_b]:
                        dist = cp.contact_distance
                        if dist < 0:
                            # self.print_contact_points(cp, time_step_count, link_index)
                            collision = True
                            break

            if collision:
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

        motor_dir = np.random.uniform(-2.96, 2.96, size=len(joints))
        half_pi = 1.57079632679
        for j, joint in enumerate(joints):
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

    def manual_control(self, robot_id, group, file_name="./state_end_state.yaml", use_current_state=False):
        wrote = False
        joint_ids = []
        param_ids = []
        current_state = self.get_current_states_for_given_joints(robot_id, group)
        exit_id = sim.addUserDebugParameter("exit".decode("utf-8"), 0, 1, 0)
        print_pose_id = sim.addUserDebugParameter("write_pose".decode("utf-8"), 0, 1, 0)

        for i, j in enumerate(group):
            info = self.joint_name_to_info[j]
            joint_name = info.joint_name
            joint_type = info.joint_type
            l_limit = info.joint_lower_limit
            u_limit = info.joint_upper_limit
            if (joint_type == sim.JOINT_PRISMATIC or joint_type == sim.JOINT_REVOLUTE):
                joint_ids.append(j)
                if use_current_state:
                    state = current_state[i]
                else:
                    state = 0
                if "odom" in joint_name:
                    # state = 0.0
                    l_limit = -0.5
                    u_limit = 0.5
                print joint_name, state, l_limit, u_limit
                param_ids.append(sim.addUserDebugParameter(joint_name.decode("utf-8"),
                                                           l_limit, u_limit, state))
        param_ids.append(exit_id)
        param_ids.append(print_pose_id)

        while (True):
            target_pos = []
            for i in range(len(param_ids)):
                param = param_ids[i]
                param_value = sim.readUserDebugParameter(param)
                if param == print_pose_id:
                    if param_value > 0:
                        data = OrderedDict(zip(group, target_pos))
                        data = {"loc" + str(i): data}
                        if not wrote:
                            ConfigParser.save_to_file(file_name, data, mode="ab")
                            wrote = True
                    if param_value == 0:
                        wrote = False
                elif param == exit_id:
                    if param_value > 0:
                        # print "current pose at exit: ", target_pos
                        data = OrderedDict(zip(group, target_pos))
                        data = {"loc" + str(i): data}
                        ConfigParser.save_to_file(file_name, data, mode="ab")
                        exit(0)
                else:
                    target_pos.append(param_value)
            # print target_pos
            self.reset_joint_states_to(robot_id, target_pos, group)
            time.sleep(0.01)
