

class Trajectory:
    def __init__(self, trajectory, no_of_samples, duration):
        self.__trajectory = trajectory
        self.__no_of_samples = no_of_samples
        self.__duration = duration


    def get_single_joint_trajectory(self, joint_index):
        if joint_index == self.__trajectory.shape[0]:
            joint_index = joint_index -1
        return self.__trajectory[joint_index]

    @property
    def trajectory(self):
        return self.__trajectory

    @property
    def duration(self):
        return self.__duration

    @property
    def no_of_samples(self):
        return self.__no_of_samples


