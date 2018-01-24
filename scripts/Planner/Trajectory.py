from collections import defaultdict
import numpy as np


class Trajectory:
    def __init__(self):
        self.__trajectory = -1
        self.__no_of_samples = -1
        self.__duration = -1
        self.__trajectory_by_joint_name = defaultdict(list)
        self.__initial = None
        self.__trajectories = []
        self.__final = None
        self.__trajectory_group = None

    def get_single_joint_trajectory(self, joint_index):
        if joint_index == self.__trajectory.shape[0]:
            joint_index = joint_index - 1
        return self.__trajectory[joint_index]

    @property
    def trajectory(self):
        return self.__trajectory
    @property
    def trajectories(self):
        return self.__trajectories

    @property
    def initial(self):
        return self.__initial

    @property
    def final(self):
        return self.__final

    @property
    def duration(self):
        return self.__duration

    @property
    def no_of_samples(self):
        return self.__no_of_samples

    @property
    def trajectory_by_name(self):
        return self.__trajectory_by_joint_name
    @property
    def trajectory_group(self):
        return self.__trajectory_by_joint_name

    def init(self, trajectory, no_of_samples, duration, group):
        self.__no_of_samples = no_of_samples
        self.__duration = duration
        self.__initial = np.array(trajectory)
        self.__trajectory_group = group

    def update(self, trajectory, group):
        self.__trajectory = trajectory
        self.__final = trajectory
        self.extract_trajectory_of_individual_joints(group)

    def extract_trajectory_of_individual_joints(self, group):
        self.__trajectory_by_joint_name = dict(zip(group, np.array(self.final).T))

    def add_trajectory(self, trajectory):
        self.__trajectories.append(dict(zip(self.trajectory_group, np.array(trajectory).T)))

    # def get_trajectory_by_name(self, trajectory):
    #     self.add_trajectory(trajectory)






