import kdl_parser_py.urdf
import os
import PyKDL as kdl
import numpy as np
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from collections import OrderedDict

class RobotTree:
    def __init__(self, robot_model, base_link, end_link):

        status, self.tree = kdl_parser_py.urdf.treeFromUrdfModel(robot_model, quiet=True)

        self.kdl_kin = KDLKinematics(robot_model, base_link, end_link, kdl_tree=self.tree)
        self.chain = self.tree.getChain(base_link, end_link)

    def get_jacobian_of_a_chain(self, q, position=None):

        # print "\n \n"
        # print "---------------------------"
        # print "joint angles:", q, len(q), type(q)

        if position is not None:
            J = self.kdl_kin.jacobian(q, position)
        else:
            J = self.kdl_kin.jacobian(q)

        return J[:3, :], J[3:, :]
        # return J
        # q = kdl_kin.random_joint_angles()
        # print "Random angles:", q, len(q), type(q)
        # pose = self.kdl_kin.forward(q)
        # print "FK:", pose
        # q_new = self.kdl_kin.inverse(pose)
        # print "IK (not necessarily the same):", q_new
        # if q_new is not None:
        #     pose_new = kdl_kin.forward(q_new)
        #     print "FK on IK:", pose_new
        #     print "Error:", np.linalg.norm(pose_new * pose ** -1 - np.mat(np.eye(4)))
        # else:
        #     print "IK failure"


        # M = kdl_kin.inertia(q)
        # print "Inertia matrix:", M
        # if False:
        #     M_cart = self.kdl_kin.cart_inertia(q)
        #     # print "Cartesian inertia matrix:", M_cart

    def get_ee_points_jacobians(self, ref_jacobian, ee_points, ref_rot):
        """
        Get the jacobians of the points on a link given the jacobian for that link's origin
        :param ref_jacobian: 6 x 6 numpy array, jacobian for the link's origin
        :param ee_points: N x 3 numpy array, points' coordinates on the link's coordinate system
        :param ref_rot: 3 x 3 numpy array, rotational matrix for the link's coordinate system
        :return: 3N x 6 Jac_trans, each 3 x 6 numpy array is the Jacobian[:3, :] for that point
                 3N x 6 Jac_rot, each 3 x 6 numpy array is the Jacobian[3:, :] for that point
        """
        ee_points = np.asarray(ee_points)
        ref_jacobians_trans = ref_jacobian[:3, :]
        ref_jacobians_rot = ref_jacobian[3:, :]
        end_effector_points_rot = np.expand_dims(ref_rot.dot(ee_points.T).T, axis=1)
        ee_points_jac_trans = np.tile(ref_jacobians_trans, (ee_points.shape[0], 1)) + \
                              np.cross(ref_jacobians_rot.T, end_effector_points_rot).transpose(
                                  (0, 2, 1)).reshape(-1, self.chain.getNrOfJoints())
        ee_points_jac_rot = np.tile(ref_jacobians_rot, (ee_points.shape[0], 1))
        return ee_points_jac_trans, ee_points_jac_rot


if __name__ == '__main__':
    home = os.path.expanduser('~')

    location_prefix = home + '/masterThesis/bullet3/data/'

    urdf_file = location_prefix + "kuka_iiwa/model.urdf"
    base_link, end_link = "lbr_iiwa_link_0", "lbr_iiwa_link_7"

    location_prefix = home
    urdf_file = location_prefix + "/catkin_ws/src/iai_robots/iai_donbot_description/robots/" + "don_bot.urdf"
    print urdf_file
    base_link, end_link = "ur5_base_link", "ur5_ee_link"
    start_state = [-2.4823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103,
                 # 1.5855963769735366, 1.5770985888989753, 1.5704531145724918]
                 1.5855963769735366, 1.5770985888989753]

    start_state = [0.0, 0.0, 0.0, -2.4823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103, 1.5855963769735366, 1.5770985888989753, 0.0, 0.0]

    ur5_ee_link = 14
    zero_vec = [0] * len(start_state)
    import pybullet as p

    p.connect(p.DIRECT)
    robot = p.loadURDF(urdf_file)

    current_position_jacobian, _ = p.calculateJacobian(robot, ur5_ee_link,
                                                           # closest_pt_on_A_at_t,
                                                           [0, 0, 0],
                                                       start_state,
                                                           zero_vec, zero_vec)

    print current_position_jacobian

    # location_prefix = home + "/catkin_ws/src/iai_robots/"
    #
    # urdf_file = location_prefix + "iai_donbot_description/robots/don_bot.urdf"
    #
    #
    # base_link, end_link = "ur5_base_link", "ur5_wrist_3_link"
    # base_link, end_link = "odom", "ur5_wrist_3_link"

    # start_state = [2.3823357809267462, 2.3823357809267462, 2.3823357809267462, 2.3823357809267462, 2.929997551699614,
    #                -1.9762726255540712, 0.8666279970481103, -1.5855963769735366, -1.5770985888989753]

    robot_model = URDF.from_xml_file(urdf_file)

    tree = RobotTree(robot_model, base_link, end_link)




    jacobian = tree.get_jacobian_of_a_chain(start_state)
    print jacobian

