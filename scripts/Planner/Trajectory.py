

class Trajectory:
    def __init__(self, trajectory):
        self.__trajectory = trajectory


    def get_single_joint_trajectory(self, joint_index):
        if joint_index == self.__trajectory.shape[0]:
            joint_index = joint_index -1
        return self.__trajectory[joint_index]

    def get_trajectory(self):
        return self.__trajectory

