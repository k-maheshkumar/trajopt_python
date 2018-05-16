import numpy as np
import logging
from scripts.utils.utils import Utils as utils
import collections

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
        self.collision_safe_distance = 0.05
        self.collision_check_distance = 0.1


    def init(self, joints, no_of_samples, duration, decimals_to_round=5, collision_safe_distance=0.05, collision_check_distance=0.1):
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
        self.samples = no_of_samples
        self.duration = duration
        self.no_of_joints = len(joints)
        self.joints = joints
        self.decimals_to_round = decimals_to_round
        self.collision_safe_distance = collision_safe_distance
        self.collision_check_distance = collision_check_distance
        if "collision_constraints" in self.joints:
            self.collision_constraints = self.joints["collision_constraints"]
        self.fill_cost_matrix()
        # self.fill_velocity_matrix()
        self.fill_start_and_goal_matrix()
        self.fill_velocity_limits()
        self.fill_robot_constraints_matrix()
        # self.duplicate_matrices()

    def duplicate_matrices(self):
        self.cost_matrix_P1 = np.hstack(
            [self.cost_matrix_P, np.zeros((self.cost_matrix_P.shape[0], self.cost_matrix_P.shape[1]))])
        self.cost_matrix_P2 = np.hstack(
            [np.zeros((self.cost_matrix_P.shape[0], self.cost_matrix_P.shape[1])), self.cost_matrix_P])
        self.cost_matrix_P = np.vstack([self.cost_matrix_P1, self.cost_matrix_P2])
        self.cost_matrix_q = np.hstack([self.cost_matrix_q, self.cost_matrix_q])
        self.start_and_goal_matrix1 = np.hstack([self.start_and_goal_matrix, np.zeros(
            (self.start_and_goal_matrix.shape[0], self.start_and_goal_matrix.shape[1]))])
        self.start_and_goal_matrix2 = np.hstack(
            [np.zeros((self.start_and_goal_matrix.shape[0], self.start_and_goal_matrix.shape[1])),
             self.start_and_goal_matrix])
        self.start_and_goal_matrix = np.vstack([self.start_and_goal_matrix1, self.start_and_goal_matrix2])
        self.robot_constraints_matrix1 = np.hstack([self.robot_constraints_matrix,
                                                    np.zeros((self.robot_constraints_matrix.shape[0],
                                                              self.robot_constraints_matrix.shape[1]))])
        self.robot_constraints_matrix2 = np.hstack(
            [np.zeros((self.robot_constraints_matrix.shape[0], self.robot_constraints_matrix.shape[1])),
             self.robot_constraints_matrix])
        self.robot_constraints_matrix = np.vstack([self.robot_constraints_matrix1, self.robot_constraints_matrix2])
        self.initial_guess = np.hstack([self.initial_guess, self.initial_guess])
        self.constraints_lower_limits = np.hstack([self.constraints_lower_limits, self.constraints_lower_limits])
        self.constraints_upper_limits = np.hstack([self.constraints_upper_limits, self.constraints_upper_limits])
        self.start_and_goal_limits = np.hstack([self.start_and_goal_limits, self.start_and_goal_limits])

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

        shape = self.samples * self.no_of_joints
        self.start_and_goal_matrix = np.zeros((2 * self.no_of_joints, shape))
        i, j = np.indices(self.start_and_goal_matrix.shape)

        self.start_and_goal_matrix[i == j] = [1] * self.no_of_joints + [0] * self.no_of_joints
        self.start_and_goal_matrix[i == j - (shape - 2* self.no_of_joints) ] = [0] * self.no_of_joints + [1] * self.no_of_joints
        self.start_and_goal_matrix = np.vstack([self.start_and_goal_matrix])

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
        for i, joint in enumerate(self.joints):
            if type(joint) is list:
                max_vel = joint[2].velocity
                min_vel = -joint[2].velocity
                joint_lower_limit = joint[2].lower
                joint_upper_limit = joint[2].upper
                start_state = joint[0]
                end_state = joint[1]
                joint_type = joint[3]
            elif type(joint) is dict:
                max_vel = self.joints[joint]["limit"]["velocity"]
                min_vel = -self.joints[joint]["limit"]["velocity"]
                joint_lower_limit = self.joints[joint]["limit"]["lower"]
                joint_upper_limit = self.joints[joint]["limit"]["upper"]
                start_state = self.joints[joint]["states"]["start"]
                end_state = self.joints[joint]["states"]["end"]
            else:
                max_vel = self.joints[joint]["limit"].velocity
                min_vel = -self.joints[joint]["limit"].velocity
                joint_lower_limit = self.joints[joint]["limit"].lower
                joint_upper_limit = self.joints[joint]["limit"].upper
                start_state = self.joints[joint]["states"]["start"]
                end_state = self.joints[joint]["states"]["end"]

            min_vel = min_vel * self.duration / float(self.samples - 1)
            max_vel = max_vel * self.duration / float(self.samples - 1)

            if joint_type == "prismatic" or "continuous":
                self.start_and_goal_matrix[i + len(self.joints), i + self.samples + len(self.joints) + 1] = 0

            self.velocity_lower_limits.append(np.full((1, self.samples - 1), min_vel))
            self.velocity_upper_limits.append(np.full((1, self.samples - 1), max_vel))
            self.joints_lower_limits.append(joint_lower_limit)
            self.joints_upper_limits.append(joint_upper_limit)
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
            [start_and_goal_lower_limits, start_and_goal_upper_limits])

        self.constraints_lower_limits = np.hstack([self.velocity_lower_limits.flatten(),
                                                   self.joints_lower_limits.flatten()])
        self.constraints_upper_limits = np.hstack([self.velocity_upper_limits.flatten(),
                                                   self.joints_upper_limits.flatten()])


        self.initial_guess = (np.asarray(self.initial_guess).T).flatten()

    def update_collision_infos(self, collision_infos, safety_limit=0.5):
        self.collision_safe_distance = safety_limit
        current_state_normal_times_jacobian, lower_collision_limit, upper_collision_limit = None, None, None
        if collision_infos is not None:
            if len(collision_infos) > 0:
                initial_signed_distance = collision_infos[0]
                current_state_normal_times_jacobian = collision_infos[1]
                next_state_normal_times_jacobian = collision_infos[2]

                if len(initial_signed_distance) > 0:
                    upper_collision_limit = None
                    np.full((1, initial_signed_distance.shape[0]), self.collision_check_distance)
                    lower_collision_limit = self.collision_safe_distance - initial_signed_distance
                    lower_collision_limit = lower_collision_limit.flatten()
                    # upper_collision_limit = initial_signed_distance - self.collision_safe_distance
                    # upper_collision_limit = upper_collision_limit.flatten()
                    vel = self.get_p_to_x_converstion_matrix()
                    current_state_normal_times_jacobian = np.matmul(current_state_normal_times_jacobian, vel)
                    next_state_normal_times_jacobian = np.matmul(next_state_normal_times_jacobian, vel)
                    constraint = np.hstack([current_state_normal_times_jacobian, next_state_normal_times_jacobian])

        return constraint, \
               lower_collision_limit, upper_collision_limit

    def get_velocity_matrix(self):

        velocity_matrix = np.zeros((self.no_of_joints * self.samples, self.samples * self.no_of_joints))
        np.fill_diagonal(velocity_matrix, -1.0)
        i, j = np.indices(velocity_matrix.shape)
        velocity_matrix[i == j - self.no_of_joints] = 1.0

        # to slice zero last row
        velocity_matrix.resize(velocity_matrix.shape[0] - self.no_of_joints, velocity_matrix.shape[1])
        return velocity_matrix

    def get_p_to_x_converstion_matrix(self):

        velocity_matrix = np.zeros((self.no_of_joints * self.samples, self.samples * self.no_of_joints))
        np.fill_diagonal(velocity_matrix, -1.0)
        i, j = np.indices(velocity_matrix.shape)
        velocity_matrix[i == j - self.no_of_joints] = 1.0
        return velocity_matrix

    def get_joints_matrix(self):

        joints_matrix = np.eye(self.samples * self.no_of_joints)
        return joints_matrix

    def fill_robot_constraints_matrix(self):
        self.velocity_matrix = self.get_velocity_matrix()
        self.joints_matrix = self.get_joints_matrix()

        self.robot_constraints_matrix.append(self.velocity_matrix)
        self.robot_constraints_matrix.append(self.joints_matrix)
        self.robot_constraints_matrix = np.vstack(self.robot_constraints_matrix)





