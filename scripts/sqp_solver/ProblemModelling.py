import numpy as np
import logging
from scripts.utils.utils import Utils as utils

class ProblemModelling:
    def __init__(self):
        self.duration = -1
        self.samples = -1
        self.no_of_joints = -1
        self.decimals_to_round = -1
        self.joints = {}
        self.cost_matrix_P = []
        self.cost_matrix_q = []
        self.robot_constraints_matrix = []
        self.velocity_lower_limits = []
        self.velocity_upper_limits = []
        self.joints_lower_limits = []
        self.joints_upper_limits = []
        self.constraints_lower_limits = []
        self.constraints_upper_limits = []
        self.start_and_goal_matrix = []
        self.start_and_goal_limits = []
        self.velocity_upper_limits = []
        self.initial_guess = []
        self.collision_constraints = None
        self.lower_safe_distance_threshold = 0.5
        self.upper_safe_distance_threshold = 2

        main_logger_name = "Trajectory_Planner"
        # verbose = "DEBUG"
        verbose = False
        self.logger = logging.getLogger(main_logger_name)
        self.setup_logger(main_logger_name, verbose)

    def init(self, joints, no_of_samples, duration, decimals_to_round=5, lower_safe_distance_threshold=0.5, upper_safe_distance_threshold=2):
        self.samples = no_of_samples
        self.duration = duration
        self.no_of_joints = len(joints)
        self.joints = joints
        self.decimals_to_round = decimals_to_round
        self.lower_safe_distance_threshold = lower_safe_distance_threshold
        self.upper_safe_distance_threshold = upper_safe_distance_threshold
        if "collision_constraints" in self.joints:
            self.collision_constraints = self.joints["collision_constraints"]
        self.fill_cost_matrix()
        # self.fill_velocity_matrix()
        self.fill_start_and_goal_matrix()
        self.fill_robot_constraints_matrix()
        self.fill_velocity_limits()

        main_logger_name = "Trajectory_Planner"
        # verbose = "DEBUG"
        verbose = False
        self.logger = logging.getLogger(main_logger_name)
        self.setup_logger(main_logger_name, verbose)

    def fill_cost_matrix(self):

        shape = self.samples * self.no_of_joints
        self.cost_matrix_P = np.zeros((shape, shape))
        i, j = np.indices(self.cost_matrix_P.shape)
        # np.fill_diagonal(A, [1] * param_a + [2] * (shape - 2 * param_a) + [1] * param_a)
        np.fill_diagonal(self.cost_matrix_P,
                         [1] * self.no_of_joints + [2] * (shape - 2 * self.no_of_joints) + [1] * self.no_of_joints)

        self.cost_matrix_P[i == j - self.no_of_joints] = -2.0
        self.cost_matrix_q = np.zeros(shape)
        # self.cost_matrix_P = 2.0 * self.cost_matrix_P + 1e-08 * np.eye(self.cost_matrix_P.shape[1])

        # Make symmetric and not indefinite
        self.cost_matrix_P = (self.cost_matrix_P + self.cost_matrix_P.T) + 1e-08 * np.eye(self.cost_matrix_P.shape[1])

    def fill_start_and_goal_matrix(self):
        # shape = self.samples * self.no_of_joints
        # self.start_and_goal_matrix = np.zeros((2 * self.no_of_joints, shape))
        # print self.start_and_goal_matrix
        #
        # np.fill_diagonal(self.start_and_goal_matrix, [1] * self.no_of_joints +  [0] * self.no_of_joints)
        # i, j = np.indices(self.start_and_goal_matrix.shape)
        #
        # # self.start_and_goal_matrix[i == j + self.no_of_joints] =  [1] * self.no_of_joints +  [0] * self.no_of_joints
        # self.start_and_goal_matrix[i == j - (self.samples + self.no_of_joints)] = [0] * self.no_of_joints + [1] * self.no_of_joints
        #
        # # np.fill_diagonal(self.start_and_goal_matrix,
        # #                  [1] * self.no_of_joints + [0] * (shape - 2 * self.no_of_joints) + [1] * self.no_of_joints)

        shape = self.samples * self.no_of_joints
        self.start_and_goal_matrix = np.zeros((2 * self.no_of_joints, shape))
        i, j = np.indices(self.start_and_goal_matrix.shape)

        # start = np.zeros((2 * self.no_of_joints, shape))
        # end = np.zeros((2 * self.no_of_joints, shape))
        # np.fill_diagonal(start, [1] * self.no_of_joints + [0] * self.no_of_joints)
        # np.fill_diagonal(end, [1] * self.no_of_joints + [0] * self.no_of_joints)
        # self.start_and_goal_matrix = start + np.fliplr(end)
        self.start_and_goal_matrix[i == j] = [1] * self.no_of_joints + [0] * self.no_of_joints
        self.start_and_goal_matrix[i == j - (shape - 2* self.no_of_joints) ] = [0] * self.no_of_joints + [1] * self.no_of_joints

        self.start_and_goal_matrix = np.vstack([self.start_and_goal_matrix, self.start_and_goal_matrix,
                                                self.start_and_goal_matrix])

    def fill_velocity_matrix(self):

        self.robot_constraints_matrix = np.zeros((self.no_of_joints * self.samples, self.samples * self.no_of_joints))
        np.fill_diagonal(self.robot_constraints_matrix, -1.0)
        i, j = np.indices(self.robot_constraints_matrix.shape)
        self.robot_constraints_matrix[i == j - self.no_of_joints] = -1.0

        # to slice zero last row
        self.robot_constraints_matrix.resize(self.robot_constraints_matrix.shape[0] - self.no_of_joints, self.robot_constraints_matrix.shape[1])

    def fill_velocity_limits(self):
        start_and_goal_lower_limits = []
        start_and_goal_upper_limits = []

        for joint in self.joints:
            if type(joint) is dict:
                max_vel = self.joints[joint]["limit"]["velocity"]
                min_vel = -self.joints[joint]["limit"]["velocity"]
                joint_lower_limit = self.joints[joint]["limit"]["lower"]
                joint_upper_limit = self.joints[joint]["limit"]["upper"]
            else:
                max_vel = self.joints[joint].limit.velocity
                min_vel = -self.joints[joint].limit.velocity
                joint_lower_limit = self.joints[joint].limit.lower
                joint_upper_limit = self.joints[joint].limit.upper


            min_vel = min_vel * self.duration / float(self.samples - 1)
            max_vel = max_vel * self.duration / float(self.samples - 1)


            self.velocity_lower_limits.append(np.full((1, self.samples - 1), min_vel))
            self.velocity_upper_limits.append(np.full((1, self.samples - 1), max_vel))
            self.joints_lower_limits.append(joint_lower_limit)
            self.joints_upper_limits.append(joint_upper_limit)
            start_state = np.round(self.joints[joint]["states"]["start"], self.decimals_to_round)
            end_state = np.round(self.joints[joint]["states"]["end"], self.decimals_to_round)
            start_and_goal_lower_limits.append(np.round(start_state, self.decimals_to_round))
            start_and_goal_upper_limits.append(np.round(end_state, self.decimals_to_round))
            self.initial_guess.append(utils.interpolate(start_state, end_state, self.samples, self.decimals_to_round))

        self.joints_lower_limits = np.hstack([self.joints_lower_limits] * self.samples).reshape(
            (1, len(self.joints_lower_limits) * self.samples))
        self.joints_upper_limits = np.hstack([self.joints_upper_limits] * self.samples).reshape(
            (1, len(self.joints_upper_limits) * self.samples))

        self.velocity_lower_limits = np.hstack(self.velocity_lower_limits)
        self.velocity_upper_limits = np.hstack(self.velocity_upper_limits)

        self.start_and_goal_limits = np.hstack(
            [start_and_goal_lower_limits, start_and_goal_upper_limits, start_and_goal_lower_limits,
             start_and_goal_upper_limits, start_and_goal_lower_limits,
             start_and_goal_upper_limits])

        self.constraints_lower_limits = np.hstack([self.velocity_lower_limits.flatten(),
                                                   self.joints_lower_limits.flatten()])
        self.constraints_upper_limits = np.hstack([self.velocity_upper_limits.flatten(),
                                                   self.joints_upper_limits.flatten()])

        self.initial_guess = (np.asarray(self.initial_guess).T).flatten()


    def update_collision_infos1(self, collision_infos):
        normal_times_jacobian, lower_collision_limit, upper_collision_limit = None, None, None
        if collision_infos is not None:
            if len(collision_infos) > 0:
                initial_signed_distance = collision_infos[0]
                normal = collision_infos[1]
                jacobian = collision_infos[2]
                if len(initial_signed_distance) > 0 and len(normal) > 0 and len(jacobian) > 0:
                    normal_times_jacobian = np.matmul(normal.T, jacobian)
                    # print self.robot_constraints_matrix.shape
                    # self.robot_constraints_matrix = np.vstack([self.robot_constraints_matrix, normal_times_jacobian])

                    np.full((1, initial_signed_distance.shape[0]), self.upper_safe_distance_threshold)
                    lower_collision_limit = np.full((1, initial_signed_distance.shape[0]), self.lower_safe_distance_threshold) - initial_signed_distance
                    upper_collision_limit = np.full((1, initial_signed_distance.shape[0]), self.upper_safe_distance_threshold)
                    lower_collision_limit = lower_collision_limit.flatten()
                    upper_collision_limit = upper_collision_limit.flatten()

                    # self.constraints_lower_limits = np.hstack([self.constraints_lower_limits, lower_collision_limit])
                    # self.constraints_upper_limits = np.hstack([self.constraints_upper_limits, upper_collision_limit])
                    #
                    # print self.robot_constraints_matrix.shape
                    # print self.constraints_lower_limits.shape
                    # print self.constraints_upper_limits.shape
                    # lower_collision_limit = np.vstack([lower_collision_limit])
                    # upper_collision_limit = np.vstack([upper_collision_limit])
        print "normal times jacobian", np.asarray(normal_times_jacobian).shape
        print "lower_collision_limit", np.asarray(lower_collision_limit).shape
        return normal_times_jacobian, lower_collision_limit, upper_collision_limit

    def update_collision_infos(self, collision_infos, safety_limit=0.5):
        self.lower_safe_distance_threshold = safety_limit
        # print "lower limit .. . ", self.lower_safe_distance_threshold
        normal_times_jacobian, lower_collision_limit, upper_collision_limit = None, None, None
        if collision_infos is not None:
            if len(collision_infos) > 0:
                initial_signed_distance = collision_infos[0]
                normal_times_jacobian = collision_infos[1]
                normal_times_jacobian1 = collision_infos[2]
                resoultion_matrix = collision_infos[3]
                if len(resoultion_matrix) > 0:
                    self.update_constraints(resoultion_matrix)
                # print "befr", normal_times_jacobian.shape
                if len(initial_signed_distance) > 0:
                    # print self.robot_constraints_matrix.shape
                    # self.robot_constraints_matrix = np.vstack([self.robot_constraints_matrix, normal_times_jacobian])

                    np.full((1, initial_signed_distance.shape[0]), self.upper_safe_distance_threshold)
                    # lower_collision_limit = self.lower_safe_distance_threshold - initial_signed_distance
                    # upper_collision_limit = np.full((1, initial_signed_distance.shape[0]), self.upper_safe_distance_threshold)
                    # lower_collision_limit = lower_collision_limit.flatten()
                    # upper_collision_limit = upper_collision_limit.flatten()
                    lower_collision_limit = np.full((1, initial_signed_distance.shape[0]), self.upper_safe_distance_threshold)
                    upper_collision_limit = initial_signed_distance - self.lower_safe_distance_threshold
                    lower_collision_limit = lower_collision_limit.flatten()
                    upper_collision_limit = upper_collision_limit.flatten()

                    # self.constraints_lower_limits = np.hstack([self.constraints_lower_limits, lower_collision_limit])
                    # self.constraints_upper_limits = np.hstack([self.constraints_upper_limits, upper_collision_limit])
                    #
                    # print self.robot_constraints_matrix.shape
                    # print self.constraints_lower_limits.shape
                    # print self.constraints_upper_limits.shape
                    # lower_collision_limit = np.vstack([lower_collision_limit])
                    # upper_collision_limit = np.vstack([upper_collision_limit])
        # print "normal times jacobian", normal_times_jacobian.shape
        # print "lower_collision_limit", np.asarray(lower_collision_limit).shape
        return np.asarray([normal_times_jacobian, normal_times_jacobian1]), lower_collision_limit, upper_collision_limit

    def get_collision_matrix(self):
        collision_matrix = []

        row = self.samples * self.no_of_joints
        column = self.samples * (self.no_of_joints + 1)
        collision_matrix = np.zeros((row, column))
        i, j = np.indices(collision_matrix.shape)
        # np.fill_diagonal(A, [1] * param_a + [2] * (shape - 2 * param_a) + [1] * param_a)
        # np.fill_diagonal(collision_matrix,
        #                  [1] * self.no_of_joints + [2] * (row - 2 * self.no_of_joints) + [1] * self.no_of_joints)
        #
        # collision_matrix[i == j - self.no_of_joints] = -2.0
        # self.cost_matrix_P = 2.0 * self.cost_matrix_P + 1e-08 * np.eye(self.cost_matrix_P.shape[1])

        # collision_matrix[::3,:1:] = -1
        # collision_matrix[1:1:3,:2:] = -2

        collision_matrix[i == j - self.samples] = 1.0
        temp = [1] * self.no_of_joints + [0] * (row - self.samples)
        print row, column
        print self.samples, self.no_of_joints
        print temp
        collision_matrix[i == j] = 2
        # collision_matrix[i == j - self.no_of_joints] = [1] * self.samples + [0] * self.samples
        # collision_matrix[i == j + (self.no_of_joints + 1)] = [1] * (self.no_of_joints * self.samples - self.samples)


        # collision_matrix[i == j + (self.samples)] = [-1] * (self.samples)
        collision_matrix[i == j + (self.samples)] = 3

        print collision_matrix



        return collision_matrix

    def update_constraints(self, matrix):
        # print self.constraints_lower_limits.shape, matrix.shape, np.full((1, matrix.shape[0]), -0.2).shape
        self.robot_constraints_matrix = np.vstack([self.robot_constraints_matrix, matrix])
        self.constraints_lower_limits = np.hstack([self.constraints_lower_limits, np.full((1, matrix.shape[0]), -0.02).flatten()])
        self.constraints_upper_limits = np.hstack([self.constraints_upper_limits, np.full((1, matrix.shape[0]), 0.02).flatten()])

    def get_velocity_matrix(self):

        velocity_matrix = np.zeros((self.no_of_joints * self.samples, self.samples * self.no_of_joints))
        np.fill_diagonal(velocity_matrix, -1.0)
        i, j = np.indices(velocity_matrix.shape)
        velocity_matrix[i == j - self.no_of_joints] = 1.0

        # to slice zero last row
        velocity_matrix.resize(velocity_matrix.shape[0] - self.no_of_joints, velocity_matrix.shape[1])
        return velocity_matrix

    def get_joints_matrix(self):

        joints_matrix = np.eye(self.samples * self.no_of_joints)
        return joints_matrix

    def fill_robot_constraints_matrix(self):
        self.velocity_matrix = self.get_velocity_matrix()
        self.joints_matrix = self.get_joints_matrix()
        # self.constraints_matrix = np.vstack([self.velocity_matrix, self.joints_matrix])
        self.robot_constraints_matrix.append(self.velocity_matrix)
        self.robot_constraints_matrix.append(self.joints_matrix)
        self.robot_constraints_matrix = np.vstack(self.robot_constraints_matrix)



    def formulate_collision_constraints(self, initial_singed_distance, normal, jacbian, safe_distance):
        pass

    def __diagonal_block_mat_slicing(self, matrix):
        shape = matrix[0].shape
        length = len(matrix)
        length_range = range(length)
        out = np.zeros((length, shape[0], length, shape[1]), dtype=int)
        out[length_range, :, length_range, :] = matrix
        return out.reshape(np.asarray(shape) * length)

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




