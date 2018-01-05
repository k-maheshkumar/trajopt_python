import logging
import pybullet as sim
import time
import numpy as np
from scripts.Robot import Robot
from munch import *
CYLINDER = sim.GEOM_CYLINDER
BOX = sim.GEOM_BOX


class SimulationWorld():
    def __init__(self, urdf_file=None):
        self.gui = sim.connect(sim.GUI)
        # self.gui = sim.connect(sim.DIRECT)
        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 0)
        location_prefix = '/home/mahe/masterThesis/bullet3/data/'
        # location_prefix = '/home/mahesh/libraries/bullet3/data/'
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

        self.end_effector_index = 6
        use_real_time_simulation = 1
        fixed_time_step = 0.001
        if use_real_time_simulation:
            sim.setRealTimeSimulation(use_real_time_simulation)
        else:
            sim.setTimeStep(fixed_time_step)

        sim.setGravity(0, 0, -10)
        self.no_of_joints = sim.getNumJoints(self.robot_id)
        self.setup_joint_id_to_joint_name()

        self.cylinder_id = self.create_constraint(shape=CYLINDER, height=0.23, radius=0.1,
                                                  position=[-0.25, -0.28, 0.9], mass=1)
        self.box_id = self.create_constraint(shape=BOX, size=[0.1, 0.1, 0.23],
                                             position=[0.23, -0.28, 0.9], mass=100)
        self.collision_constraints = None

        sim.configureDebugVisualizer(sim.COV_ENABLE_RENDERING, 1)

        # self.logger = logging.getLogger("Trajectory_Planner." + __name__)

        main_logger_name = "Trajectory_Planner"
        verbose = "DEBUG"
        self.logger = logging.getLogger(main_logger_name)
        self.setup_logger(main_logger_name, verbose)

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
                duration = 6
                samples = 5
                full_arm = group1 + group2
                # full_arm = group1_test
                lower_d_safe = 2
                upper_d_safe = 3
                self.reset_joint_states(start_state)

                self.plan_trajectory(full_arm, goal_state, samples, duration, lower_d_safe, upper_d_safe)
                # self.execute_trajectory()

                import sys
                sys.exit()

    def get_collision_infos(self, group, lower_d_safe_limit, upper_d_safe_limit, distance=2):

        can_execute_trajectory = False
        pos = self.get_joint_states(group)[0]
        zero_vec = [0.0] * len(pos)
        jacobian_by_joint_name = {}
        # for joint_name in group:
        # result = sim.getLinkState(self.robot_id, self.joint_name_to_id[joint_name], computeLinkVelocity=1,
        #                           computeForwardKinematics=1)
        result = sim.getLinkState(self.robot_id, self.end_effector_index, computeLinkVelocity=1,
                                  computeForwardKinematics=1)
        com_trn = result[2]

        # jac_t, jac_r = sim.calculateJacobian(self.robot_id, self.joint_name_to_id[joint_name], com_trn, pos, zero_vec, zero_vec)
        jac_t, jac_r = sim.calculateJacobian(self.robot_id, self.end_effector_index, com_trn, pos, zero_vec, zero_vec)
        jac_t = np.asarray(jac_t)
        # print jac_t
        # print "gfda ", jac_t[:, 2]
        # print jac_t[0]
        for joint_name in group:
            # jacobian_by_joint_name[joint_name] = jac_t[:, self.joint_name_to_id[joint_name]]
            contact_points = sim.getClosestPoints(self.robot_id, self.cylinder_id,
                                                  linkIndexA=self.joint_name_to_id[joint_name],  distance=distance)
            self.collision_constraints[joint_name] = (munchify({
                "jacobian": np.asarray(jac_t[:, self.joint_name_to_id[joint_name]]).reshape(3,1),
                "normal": np.asarray(contact_points[0][7]).reshape(3,1),
                "initial_signed_distance": contact_points[0][8],
                "limits": {"lower": lower_d_safe_limit, "upper": upper_d_safe_limit}
            }))
            # print "self.collision_constraints[joint_name]", self.collision_constraints[joint_name].jacobian.shape
        # print self.collision_constraints

    def plan_trajectory(self, group, goal_state, samples, duration, lower_collision_limit=None, upper_collision_limit=None, solver_config=None):
        if (lower_collision_limit is not None or lower_collision_limit != 0) and (upper_collision_limit is not None or upper_collision_limit != 0):
            self.collision_constraints = {}
            self.get_collision_infos(group, lower_collision_limit, upper_collision_limit)

        # print self.collision_constraints
            # print joint_name, jac_t
            # print self.jacobian_by_joint_name
            # print np.asarray(self.jacobian_by_joint_name).shape
            status, can_execute_trajectory = self.robot.plan_trajectory(group=group,
                                                                    current_state=self.get_current_states_for_given_joints(group),
                                                                    goal_state=goal_state, samples=int(samples),
                                                                    duration=int(duration),
                                                                    collision_constraints=self.collision_constraints,
                                                                    solver_config=solver_config,
                                                                    lower_d_safe=lower_collision_limit,
                                                                    upper_d_safe=upper_collision_limit)
        else:
            status, can_execute_trajectory = self.robot.plan_trajectory(group=group,
                                                                        current_state=self.get_current_states_for_given_joints(group),
                                                                        goal_state=goal_state, samples=int(samples),
                                                                        duration=int(duration),
                                                                        solver_config=solver_config)
        return status, can_execute_trajectory

    def get_current_states_for_given_joints(self, joints):
        current_state = {}
        for joint in joints:
            current_state[joint] = sim.getJointState(bodyUniqueId=self.robot_id, jointIndex=self.joint_name_to_id[joint])[0]
        return current_state

    def execute_trajectory(self):
        trajectories = self.robot.get_trajectory()
        for i in range(int(trajectories.no_of_samples)):
            for joint_name, corresponding_trajectory in trajectories.trajectory.items():
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
            time.sleep(trajectories.duration / float(trajectories.no_of_samples))
            # sim.stepSimulation()

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

    def reset_joint_states(self, joints, motor_dir=None):
        # if motor_dir is None:
        #     # motor_dir = [-1, -1, -1, 1, 1, 1, 1]
        #     motor_dir = np.random.uniform(-1, 1, size=len(joints))
        # half_pi = 1.57079632679
        for joint in joints:
            for j in range(len(joints)):
                sim.resetJointState(self.robot_id, self.joint_name_to_id[joint], joints[joint])
        status = "Reset joints to random pose is complete"
        self.logger.info(status)
        return status

    def get_joint_states(self, group):
        joint_states = sim.getJointStates(self.robot_id, [self.joint_name_to_id[joint_name] for joint_name in group])
        joint_positions = [state[0] for state in joint_states]
        joint_velocities = [state[1] for state in joint_states]
        joint_torques = [state[3] for state in joint_states]
        return joint_positions, joint_velocities, joint_torques

    def get_contact_points(self):
        for i in range(sim.getNumJoints(self.robot_id)):
            contact_points = sim.getClosestPoints(self.robot_id, self.box_id, linkIndexA=i,  distance=4)
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
