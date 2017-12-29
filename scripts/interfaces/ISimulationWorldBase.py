import abc


class ISimulationWorldBase(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def get_current_states_for_given_joints(self):
        """should return robot's current state from the simulation environment"""
        return

    @abc.abstractmethod
    def execute_trajectory(self, trajectory):
        """should execute the given trajectoy"""
