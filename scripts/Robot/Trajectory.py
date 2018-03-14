import numpy as np
import matplotlib.pyplot as plt
from pylab import *
from scripts.utils.dict import DefaultOrderedDict

class Trajectory:
    def __init__(self):
        self.__trajectory = -1
        self.__no_of_samples = -1
        self.__duration = -1
        self.__trajectory_by_joint_name = DefaultOrderedDict(list)
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
        return self.__trajectory_group

    def init(self, trajectory, no_of_samples, duration, group):
        self.__no_of_samples = no_of_samples
        self.__duration = duration
        self.__initial = np.array(trajectory)
        self.__trajectory_group = group

    def update(self, trajectory, group):
        self.__trajectory = trajectory
        self.__final = trajectory
        self.extract_trajectory_of_individual_joints(group)
        # self.add_trajectory(trajectory)

    def extract_trajectory_of_individual_joints(self, group):
        self.__trajectory_by_joint_name = dict(zip(group, np.array(self.final).T))

    def add_trajectory(self, trajectory):

        # self.__trajectories.append(dict(zip(self.trajectory_group, np.array(trajectory).T)))
        self.__trajectories.append(dict(zip(self.trajectory_group, np.array(trajectory).T)))

    # def get_trajectory_by_name(self, trajectory):
    #     self.add_trajectory(trajectory)


    def plot_trajectories(self):
        # print self.trajectories[0]
        fig = plt.figure()
        subplots_adjust(hspace=0.000)

        for index, trajectory in enumerate(self.trajectories):
            if (index == 0 or index == len(self.trajectories) - 1):
                count = 0
                for joint_name, traj in trajectory.items():
                    # plt.plot(traj, label=joint_name, marker='x')
                    count += 1

                    # plt.subplot(7, 1, count)
                    if index == 0:
                        label = "initial"
                    elif index == 6:
                        label = "final"
                    # plt.plot(traj, label=label, marker='x')

                    ax = plt.subplot(7, 1, count)
                    ax.plot(traj, label=joint_name, marker='x')

                    # plt.title('A tale of 2 subplots')
                    # plt.ylabel('Damped oscillation')

        # plt.scatter(x, y, c='b', marker='x', label='1')
        # plt.scatter(x, y, c='r', marker='s', label='-1')
        plt.legend(loc='upper left')
        plt.show(block=False)





