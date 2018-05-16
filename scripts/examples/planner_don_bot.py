import os
import time
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from scripts.utils.dict import DefaultOrderedDict
from srdfdom.srdf import SRDF
import pybullet as p
from scripts.simulation.bulletTypes import *
from random import randint
from collections import OrderedDict

home = os.path.expanduser('~')

class PlannerExample:
    def __init__(self):

        location_prefix = home + "/catkin_ws/src/iai_robots/"

        urdf_file = location_prefix + "iai_donbot_description/robots/don_bot.urdf"
        # urdf_file = location_prefix + "iai_donbot_description/robots/robot.urdf"

        shelf_file = home + "/catkin_ws/src/iai_shelf_description/urdf/shelf.urdf"
        srdf_file = home + "/catkin_ws/src/iai_robots/iai_donbot_description/ur5_moveit_config/config/ur5.srdf"

        config = {
            "use_gui": True,
            "verbose": "INFO",
            "log_file": True,
            "save_problem": True,
            "robot_config": "robot_config_don_bot.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(use_gui=False)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_gravity(0, 0, -10)
        plane_id = self.planner.add_constraint_from_urdf("plane", "plane.urdf", position=[0, 0, 0.0])
        self.robot_id = self.planner.load_robot(urdf_file,
                                                use_fixed_base=True,
                                                position=[0.6, 0.2, 0],
                                                orientation=p.getQuaternionFromEuler([0, 0, -1.57])
                                                )

        shelf_id = self.planner.add_constraint_from_urdf("shelf", urdf_file=shelf_file, position=[-0.15, 0, 0.0],
                                                         orientation=p.getQuaternionFromEuler([0, 0, 1.57]))

        # table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf",
        #                                                  position=[0, 0, 0.0])
        # #
        # self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=[0.05, 0.05, 0.1],
        #                                           position=[0, 0.3, 0.62], mass=100)
        # self.box_id1 = self.planner.add_constraint("box2", shape=self.planner.world.BOX, size=[0.05, 0.05, 0.2],
        #                                           position=[0.15, -0.2, 0.62], mass=100)
        # self.box_id2 = self.planner.add_constraint("box3", shape=self.planner.world.BOX, size=[0.1, 0.05, 0.1],
        #                                           position=[0.15, 0.4, 1], mass=100)
        # self.box_id2 = self.planner.add_constraint("box3", shape=self.planner.world.BOX, size=[0.1, 1, 0.02],
        #                                           position=[0.15, 0.4, 0.9], mass=100)

        shelf_item_prefix = home + "/catkin_ws/src/shelf_item_descriptions/urdf/"
        salt_urdf = shelf_item_prefix + "salt.urdf"
        salt_urdf = shelf_item_prefix + "duschGel.urdf"
        salt_id = OrderedDict()

        y, z = 0.3, 1.0
        offset = -0.29
        for x in range(4):
            salt_id[x] = self.planner.add_constraint_from_urdf("salt" + str(x), urdf_file=salt_urdf,
                                                               position=[offset + 0.1 * x, y, z])
        gel_urdf = shelf_item_prefix + "duschGel.urdf"
        gel_id = OrderedDict()
        y, z = -0.3, 0.62
        offset = -0.14
        for x in range(1):
            gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
                                                              position=[offset + 0.1 * x, y, z])
        for x in range(1):
            gel_id[x + 4] = self.planner.add_constraint_from_urdf("gel" + str(x + 4), urdf_file=gel_urdf,
                                                                  position=[offset + 0.1 * x, y - 0.14, z])
        lotion_urdf = shelf_item_prefix + "bodyLotion.urdf"
        lotion_id = OrderedDict()
        y, z = -0.4, 0.92
        offset = -0.14
        for x in range(1):
            lotion_id[x] = self.planner.add_constraint_from_urdf("lotion" + str(x), urdf_file=lotion_urdf,
                                                                 position=[offset + 0.1 * x, y, z])

        self.planner.robot.load_srdf(srdf_file)
        self.planner.world.ignored_collisions = self.planner.robot.get_ignored_collsion()
        self.planner.world.toggle_rendering(1)
        # self.planner.world.step_simulation_for(0.2)
        # print self.planner.world.ignored_collisions

    def run(self):
        from collections import OrderedDict

        start_state = OrderedDict()
        goal_state = OrderedDict()

        start_state["odom_x_joint"] = 0.1
        start_state["odom_y_joint"] = 0.3
        start_state["odom_z_joint"] = 0.01

        start_state["ur5_shoulder_pan_joint"] = 1.9823357809267463
        start_state["ur5_shoulder_lift_joint"] = -2.4299975516996142
        start_state["ur5_elbow_joint"] = -1.9762726255540713
        start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state["ur5_wrist_2_joint"] = 1.5855963769735366
        start_state["ur5_wrist_3_joint"] = -1.5770985888989753

        start_state["gripper_joint"] = 0
        start_state["gripper_base_gripper_left_joint"] = 0
        start_state["ur5_ee_fixed_joint"] = 1.5704531145724918

        goal_state["odom_x_joint"] = 0.1
        goal_state["odom_y_joint"] = 0.3
        goal_state["odom_z_joint"] = 0.01

        goal_state["ur5_shoulder_pan_joint"] = 1.9823357809267463
        goal_state["ur5_shoulder_lift_joint"] = -1.8299975516996142
        goal_state["ur5_elbow_joint"] = -1.9762726255540713
        goal_state["ur5_wrist_1_joint"] = 0.8666279970481103
        goal_state["ur5_wrist_2_joint"] = 1.5855963769735366
        goal_state["ur5_wrist_3_joint"] = -1.5770985888989753
        #
        goal_state["gripper_joint"] = 0
        goal_state["gripper_base_gripper_left_joint"] = 0

        # start_state = OrderedDict()
        # goal_state = OrderedDict()
        #
        # start_state["odom_x_joint"] = 0.01
        # start_state["odom_y_joint"] = 0.01
        # start_state["odom_z_joint"] = 0.01
        #
        # # start_state["ur5_shoulder_pan_joint"] = -1.3823357809267463
        # # start_state["ur5_shoulder_lift_joint"] = -1.9299975516996142
        # # start_state["ur5_elbow_joint"] = -2.2762726255540713
        # # start_state["ur5_wrist_1_joint"] = 2.2666279970481103
        # # start_state["ur5_wrist_2_joint"] = -2.8855963769735366
        # # start_state["ur5_wrist_3_joint"] = 2.1770985888989753
        #
        # start_state["ur5_shoulder_pan_joint"] = 0
        # start_state["ur5_shoulder_lift_joint"] = -1.9
        # start_state["ur5_elbow_joint"] = 2.5
        # start_state["ur5_wrist_1_joint"] = 0
        # start_state["ur5_wrist_2_joint"] = 0
        # start_state["ur5_wrist_3_joint"] = 0
        #
        # start_state["gripper_joint"] = 0
        # start_state["gripper_base_gripper_left_joint"] = 0
        #
        # goal_state["odom_x_joint"] = 0.01
        # goal_state["odom_y_joint"] = 0.01
        # goal_state["odom_z_joint"] = 0.01
        #
        # # start_state["ur5_shoulder_pan_joint"] = -1.3823357809267463
        # # start_state["ur5_shoulder_lift_joint"] = -1.9299975516996142
        # # start_state["ur5_elbow_joint"] = -2.2762726255540713
        # # start_state["ur5_wrist_1_joint"] = 2.2666279970481103
        # # start_state["ur5_wrist_2_joint"] = -2.8855963769735366
        # # start_state["ur5_wrist_3_joint"] = 2.1770985888989753
        #
        # goal_state["ur5_shoulder_pan_joint"] = 1.5
        # goal_state["ur5_shoulder_lift_joint"] = -1.9
        # goal_state["ur5_elbow_joint"] = -2.5
        # goal_state["ur5_wrist_1_joint"] = -1.5
        # goal_state["ur5_wrist_2_joint"] = 0
        # goal_state["ur5_wrist_3_joint"] = -1.5
        #
        # goal_state["gripper_joint"] = 0
        # goal_state["gripper_base_gripper_left_joint"] = 0

        # goal_state["odom_x_joint"] = 0.01
        # goal_state["odom_y_joint"] = 0.01
        # goal_state["odom_z_joint"] = 0.01
        #
        # goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        # goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        # goal_state["ur5_elbow_joint"] = 1.5873548294514912
        # goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        # goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        # goal_state["ur5_wrist_3_joint"] = 1.5857854098720727
        #
        # goal_state["gripper_joint"] = 0
        # goal_state["gripper_base_gripper_left_joint"] = 0

        # start_state["ur5_shoulder_pan_joint"] = -1.4823357809267463
        # start_state["ur5_shoulder_lift_joint"] = -2.9299975516996142
        # start_state["ur5_elbow_joint"] = 1.9762726255540713
        # start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        # start_state["ur5_wrist_2_joint"] = -1.5855963769735366
        # start_state["ur5_wrist_3_joint"] = -1.5770985888989753
        # # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918
        #
        # goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        # goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        # goal_state["ur5_elbow_joint"] = -1.5873548294514912
        # goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        # goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        # goal_state["ur5_wrist_3_joint"] = 1.5857854098720727
        # goal_state["ur5_ee_fixed_joint"] = 1.5726221954434347

        # start_state = "place"
        # goal_state = "pick"
        # group = "full_arm"
        group = goal_state.keys()

        duration = 10
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1
        #
        self.planner.world.reset_joint_states(self.robot_id, start_state.values(), start_state.keys())

        start_state1 = OrderedDict()
        start_state1["ur5_shoulder_pan_joint"] = 1.9823357809267463
        start_state1["ur5_shoulder_lift_joint"] = -2.4299975516996142
        start_state1["ur5_elbow_joint"] = -1.9762726255540713
        start_state1["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state1["ur5_wrist_2_joint"] = 1.5855963769735366
        start_state1["ur5_wrist_3_joint"] = -1.5770985888989753
        start_state1["gripper_joint"] = 0
        start_state1["gripper_base_gripper_left_joint"] = 0

        goal_state1 = OrderedDict()
        goal_state1["ur5_shoulder_pan_joint"] = 1.9823357809267463
        goal_state1["ur5_shoulder_lift_joint"] = -1.6299975516996142
        goal_state1["ur5_elbow_joint"] = -1.9762726255540713
        goal_state1["ur5_wrist_1_joint"] = 0.8666279970481103
        goal_state1["ur5_wrist_2_joint"] = 1.5855963769735366
        goal_state1["ur5_wrist_3_joint"] = -1.5770985888989753
        goal_state1["gripper_joint"] = 0
        goal_state1["gripper_base_gripper_left_joint"] = 0
        group1 = goal_state1.keys()
        self.planner.world.reset_joint_states(self.robot_id, start_state1.values(), start_state1.keys())

        _, status, trajectory = self.planner.get_trajectory(group=group,
                                                            goal_state=goal_state, samples=samples, duration=duration,
                                                            collision_safe_distance=collision_safe_distance,
                                                            collision_check_distance=collision_check_distance)
        print("is trajectory free from collision: ", status)
        # # print trajectory.final
        self.planner.execute_trajectory()
        # self.planner.world.step_simulation_for(1)
        # time.sleep(5)
        # import sys
        # sys.exit(0)

    def load_srdf(self, srdf):
        srdf_file = home + "/catkin_ws/src/iai_robots/iai_donbot_description/ur5_moveit_config/config/ur5.srdf"

        stream = open(srdf_file, 'r')
        srdf = SRDF.from_xml_string(stream.read())

        ignored_collisions = DefaultOrderedDict(bool)
        shape = len(self.planner.world.joint_ids)

        # ignored_collisions_matrix = np.zeros((shape, shape))
        # joints = self.planner.world.link_name_to_id

        for collision in srdf.disable_collisionss:
            ignored_collisions[collision.link1, collision.link2] = True
            ignored_collisions[collision.link2, collision.link1] = True
            # if collision.link1 in joints and collision.link2 in joints:
            #     ignored_collisions_matrix[joints[collision.link1], joints[collision.link2]] = 1
            #     ignored_collisions_matrix[joints[collision.link2], joints[collision.link1]] = 1
        # print ignored_collisions
        self.planner.world.ignored_collisions = ignored_collisions

    def cal_jacobian(self):
        import pybullet as p
        p.connect(p.SHARED_MEMORY, "localhost")
        joints = [i for i in range(p.getNumJoints(self.robot_id))]

        ur5_wrist_3_joint = 13
        # print "ghdghrdklhg", p.getNumJoints(self.robot_id)
        # zero_vec = [0] * 11
        # for i in range(p.getNumJoints(self.robot_id)):
        #     print p.getJointInfo(self.robot_id, i)
        current_robot_state = self.planner.world.get_joint_states_at(self.robot_id, start_state.values(),
                                                                     goal_state.keys())
        zero_vec = [0] * len(current_robot_state[0])

        print "current_robot_state*------------------", len(current_robot_state[0])
        print current_robot_state[0]

        current_position_jacobian, _ = p.calculateJacobian(self.robot_id, ur5_wrist_3_joint,
                                                           # closest_pt_on_A_at_t,
                                                           [0, 0, 0],
                                                           current_robot_state[0],
                                                           zero_vec, zero_vec)

        print current_position_jacobian

    def manual_control(self):
        import pybullet as p
        import time

        p.connect(p.SHARED_MEMORY)
        jointIds = []
        paramIds = []
        for j in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, j)
            joint_state = p.getJointState(self.robot_id, j)
            # print(info)
            jointName = info[1]
            jointType = info[2]
            lower_limit = info[8]
            upper_limit = info[9]
            lower_limit = -4
            lower_limit = 4
            print lower_limit, upper_limit
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                jointIds.append(j)
                paramIds.append(
                    p.addUserDebugParameter(jointName.decode("utf-8"), lower_limit, upper_limit, joint_state[0]))

        p.setRealTimeSimulation(1)
        while (1):
            for i in range(len(paramIds)):
                c = paramIds[i]
                targetPos = p.readUserDebugParameter(c)
                p.setJointMotorControl2(self.planner.robot.id, jointIds[i], p.VELOCITY_CONTROL, targetPos,
                                        force=5 * 240.)
            time.sleep(0.01)

    def connect_directly(self):
        import pybullet as p
        import time

        print "bdlnfd"
        # p.connect(p.SHARED_MEMORY, "localhost")
        p.connect(p.DIRECT)
        print "fdlknbldfka ------------"
        # closest_points = [ClosestPointInfo(*x) for x in p.getClosestPoints(self.robot_id, self.robot_id, 0.1)]
        # ignored_collisions = DefaultOrderedDict(bool)
        # joint_info = self.planner.world.joint_id_to_info
        # for cp in closest_points:
        #     link_a = joint_info[cp.link_index_a].link_name
        #     link_b = joint_info[cp.link_index_b].link_name
        #     if cp.contact_distance < 0 and cp.link_index_a != cp.link_index_b and \
        #             not ignored_collisions[link_a, link_b]:
        #         ignored_collisions[link_a, link_b] = True
        #         ignored_collisions[link_b, link_a] = True
        #
        # print ignored_collisions
        from scripts.Robot.ModelandTree import RobotTree
        from collections import OrderedDict
        tree = RobotTree(self.planner.robot.model, "ur5_base_link", "ur5_ee_link")
        start_state = OrderedDict()

        # start_state["odom_x_joint"] = 0.1
        # start_state["odom_y_joint"] = 0.3
        # start_state["odom_z_joint"] = 0.01

        start_state["ur5_shoulder_pan_joint"] = 1.9823357809267463
        start_state["ur5_shoulder_lift_joint"] = -2.4299975516996142
        start_state["ur5_elbow_joint"] = -1.9762726255540713
        start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state["ur5_wrist_2_joint"] = 1.5855963769735366
        start_state["ur5_wrist_3_joint"] = -1.5770985888989753

        # start_state["gripper_joint"] = 0
        # start_state["gripper_base_gripper_left_joint"] = 0
        # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918

        temp = [-2.4823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103,
                1.5855963769735366, 1.5770985888989753]
        self.planner.world.reset_joint_states(self.robot_id, temp, start_state.keys())
        # ur5_wrist_3_joint = 13
        ur5_ee_link = 14
        # print "ghdghrdklhg", p.getNumJoints(self.robot_id)
        # zero_vec = [0] * 11
        # for i in range(p.getNumJoints(self.robot_id)):
        #     print p.getJointInfo(self.robot_id, i)
        current_robot_state = self.planner.world.get_joint_states_at(self.robot_id, start_state.values(),
                                                                     start_state.keys())
        zero_vec = [0] * len(current_robot_state[0])

        print "current_robot_state*------------------", len(current_robot_state[0])
        print current_robot_state[0]
        point = [0, 0, 0]
        current_position_jacobian, _ = p.calculateJacobian(self.robot_id, ur5_ee_link,
                                                           # closest_pt_on_A_at_t,
                                                           point,
                                                           current_robot_state[0],
                                                           zero_vec, zero_vec)

        print current_position_jacobian
        link_state = p.getLinkState(self.robot_id, ur5_ee_link)
        print "rhgdl", link_state
        tem = [-2.4823357809267463, 1.4999975516996142, -1.5762726255540713, -0.8666279970481103, 1.5855963769735366,
               1.5770985888989753]
        point1 = link_state[5]
        jac, _ = tree.get_jacobian_of_a_chain(tem, point1)

        print jac


def main():
    example = PlannerExample()
    # example.load_srdf()
    example.run()
    # example.manual_control()
    # example.connect_directly()
    while True:
        pass


if __name__ == '__main__':
    main()
