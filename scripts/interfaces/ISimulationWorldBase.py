import abc


class ISimulationWorldBase(object):
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def toggle_rendering_while_planning(self, enable=True):
        """interface to toggle GUI rendering while planning trajectory"""

    @abc.abstractmethod
    def toggle_rendering(self, enable):
        """interface to toggle GUI rendering"""

    @abc.abstractmethod
    def set_gravity(self, x, y, z):
        """interface to set the simulation world gravity"""
        return

    @abc.abstractmethod
    def create_constraint(self, shape, mass, position, size, radius, height, orientation):
        """interface to create collision constraints"""

    @abc.abstractmethod
    def load_urdf(self, urdf_file, position, orientation, use_fixed_base):
        """interface to load urdf into the scene"""
    @abc.abstractmethod
    def get_link_states_at(self, robot_id, trajectory, group):
        """given a trajectory and planning group along with the robot id, this method should list of link states
        corresponding to each time step of the trajectory"""
        return

    @abc.abstractmethod
    def get_collision_infos(self, robot_id, trajectory, group, distance=0.10):
        """given a trajectory (lists of list), planning group and distance to check for the collision
        along with the robot id, this method should return the collision infos such as initial distance
        between the robot link, array of matmul(current_time_step_normal_from_collision_object.T,
         position_jacobian(current_time_step_closest_point_on_link), and
         array of matmul(next_time_step_normal_from_collision_object.T,
         position_jacobian(next_time_step_closest_point_on_link)"""

    @abc.abstractmethod
    def get_point_in_local_frame(self, frame_position, frame_orientation, point):
        """Given a frame position and orientation, also the point to be transformed in world frame, this method
        should return the given point in the local frame"""


    @abc.abstractmethod
    def update_collsion_infos(self, planning_samples, new_trajectory, delta_trajectory=None):
        """This a callback function from the solver which takes a new trajectory from the solver and
        returns solver constraints along with upper and lower bounds"""


    @abc.abstractmethod
    def plan_trajectory(self, group, goal_state, samples, duration, solver_config=None, collision_safe_distance=None,
                        collision_check_distance=0.2):
        """given planning group, goal state, samples, duration, solver_config, collsion safe and check distance,
        this method should plan the trajectory"""

    @abc.abstractmethod
    def get_current_states_for_given_joints(self):
        """should return robot's current state from the simulation environment"""
        return

    @abc.abstractmethod
    def execute_trajectory(self, robot, trajectory, step_time):
        """should execute the given trajectoy for a given robot"""

    @abc.abstractmethod
    def execute_trajectories(self, robot, group, trajectories):
        """same as executetrajectories, but input is list of trajectory"""

    @abc.abstractmethod
    def plan_and_execute_trajectory(self, group, goal_state, samples, duration, solver_config=None,
                                   collision_safe_distance=0.05, collision_check_distance=0.1):
        """method plan and execute trajectory clubbed together"""


    @abc.abstractmethod
    def is_trajectory_collision_free(self, robot_id, trajectory, group, collision_safe_distance=0.05):
        """given a trajectory, planning group and collision safe distance  along with the robot id,
        this method return true if there is a collision in the trajectory else returns false"""


    @abc.abstractmethod
    def get_link_states_at(self, trajectory, group):
        """given a trajectory and planning group, this method should list of link states
        corresponding to each time step of the trajectory"""
        return

    @abc.abstractmethod
    def reset_joint_states_to(self, trajectory, joints):
        """joints are reset to the  given a trajectory position"""


    @abc.abstractmethod
    def reset_joints_to_random_states(self, robot_id, joints):
        """interface to reset given joints to the random valid pose"""
        return

    @abc.abstractmethod
    def get_joint_states(self, robot_id, group):
        """given a planning group  along with the robot id, this method should return the current joint states"""


    @abc.abstractmethod
    def step_simulation_for(self, seconds):
        """this method is to run (step) the simulation for the given seconds"""
        return