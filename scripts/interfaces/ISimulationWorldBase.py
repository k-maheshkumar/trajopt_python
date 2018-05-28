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
    def create_constraint_from_mesh(self, name, file_name, mass=1, position=None, orientation=None,
                          mesh_scale=None,
                          visual_frame_shift=None, collision_frame_shift=None,
                          specularColor=None,
                          rgba_color=None, use_maximalcoordinates=True):
        """interface to create collision constraints from mesh"""

    @abc.abstractmethod
    def load_urdf(self, name, urdf_file, position, orientation=None, use_fixed_base=False):
        """interface to load urdf into the scene"""

    @abc.abstractmethod
    def load_robot(self, urdf_file, position, orientation=None, use_fixed_base=False):
        """interface to load robot into the scene"""

    @abc.abstractmethod
    def get_link_states_at(self, robot_id, trajectory, group):
        """given a trajectory and planning group along with the robot id, this method should list of link states
        corresponding to each time step of the trajectory"""

    @abc.abstractmethod
    def get_joint_and_link_states_at(self, robot_id, trajectory, group):
        """given a trajectory and planning group along with the robot id, this method should list of joints and link
        states corresponding to each time step of the trajectory"""

    @abc.abstractmethod
    def get_joint_states_at(self, robot_id, trajectory, group):
        """given a trajectory and planning group along with the robot id, this method should list of joints states
        corresponding to each time step of the trajectory"""

    @abc.abstractmethod
    def get_collision_infos(self, robot_id, trajectory, group, distance=0.10):
        """given a trajectory (lists of list), planning group and distance to check for the collision
        along with the robot id, this method should return the collision infos such as initial distance
        between the robot link, array of matmul(current_time_step_normal_from_collision_object.T,
         position_jacobian(current_time_step_closest_point_on_link), and
         array of matmul(next_time_step_normal_from_collision_object.T,
         position_jacobian(next_time_step_closest_point_on_link)"""

    @abc.abstractmethod
    def get_point_in_local_frame(self, frame_position, frame_orientation, point, orn=None, inverse=True):
        """Given a frame position and orientation, also the point to be transformed in world frame, this method
        should return the given point in the local frame"""


    @abc.abstractmethod
    def update_collsion_infos(self, planning_samples, new_trajectory, delta_trajectory=None):
        """This a callback function from the solver which takes a new trajectory from the solver and
        returns solver constraints along with upper and lower bounds"""

    @abc.abstractmethod
    def get_current_states_for_given_joints(self, robot_id, group):
        """given planning group, this method should return robot's current state from the simulation environment"""
        return

    @abc.abstractmethod
    def execute_trajectory(self, robot, trajectory, step_time):
        """should execute the given trajectoy for a given robot"""

    @abc.abstractmethod
    def is_trajectory_collision_free(self, robot_id, trajectory, group, collision_safe_distance=0.05):
        """given a trajectory, planning group and collision safe distance  along with the robot id,
        this method return true if there is a collision in the trajectory else returns false"""

    @abc.abstractmethod
    def is_given_state_in_collision(self, robot_id, state, group, distance=0.02):
        """given a trajectory state, planning group and collision safe distance  along with the robot id,
        this method return true if there is a collision else returns false"""

    @abc.abstractmethod
    def get_link_states_at(self, trajectory, group):
        """given a trajectory and planning group, this method should list of link states
        corresponding to each time step of the trajectory"""
        return

    @abc.abstractmethod
    def reset_joint_states_to(self, robot_id, trajectory, joints):
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