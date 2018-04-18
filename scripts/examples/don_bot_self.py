import os
import time
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from srdfdom.srdf import SRDF
from scripts.utils.dict import DefaultOrderedDict
from collections import OrderedDict


class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + "/catkin_ws/src/iai_robots/"

        urdf_file = location_prefix + "iai_donbot_description/robots/don_bot.urdf"

        config = {
            "use_gui": True,
            "verbose": False,
            "log_file": True,
            "robot_config": "robot_config_don_bot.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(use_gui=False)
        self.planner.world.toggle_rendering(0)
        self.planner.world.set_gravity(0, 0, -10)
        plane_id = self.planner.add_constraint_from_urdf("plane", "plane.urdf", position=[0, 0, 0.0])
        self.robot_id = self.planner.load_robot(urdf_file)
        #
        # # table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf",
        # #                                                  position=[0, 0, 0.0])
        # #
        # # self.box_id = self.planner.add_constraint(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        # #                                           position=[0.28, -0.43, 0.9], mass=100)
        #
        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.02)

    def run(self):
        from collections import OrderedDict
        start_state = OrderedDict()
        goal_state = OrderedDict()

        start_state["odom_x_joint"] = 0
        start_state["odom_y_joint"] = 0
        start_state["odom_z_joint"] = 0

        start_state["ur5_shoulder_pan_joint"] = 2.3823357809267463
        start_state["ur5_shoulder_lift_joint"] = 2.9299975516996142
        start_state["ur5_elbow_joint"] = -1.9762726255540713
        start_state["ur5_wrist_1_joint"] = 0.8666279970481103
        start_state["ur5_wrist_2_joint"] = -1.5855963769735366
        start_state["ur5_wrist_3_joint"] = -1.5770985888989753

        start_state["gripper_joint"] = 0
        start_state["gripper_base_gripper_left_joint"] = 0
        # start_state["ur5_ee_fixed_joint"] = 1.5704531145724918

        goal_state["odom_x_joint"] = 0.1
        goal_state["odom_y_joint"] = 0.1
        goal_state["odom_z_joint"] = 0.1

        goal_state["ur5_shoulder_pan_joint"] = 2.08180533826032865
        goal_state["ur5_shoulder_lift_joint"] = -1.5474152457596664
        goal_state["ur5_elbow_joint"] = 1.5873548294514912
        goal_state["ur5_wrist_1_joint"] = -0.5791571346767671
        goal_state["ur5_wrist_2_joint"] = 1.5979105177314896
        goal_state["ur5_wrist_3_joint"] = 1.5857854098720727

        goal_state["gripper_joint"] = 0.1
        goal_state["gripper_base_gripper_left_joint"] = 0.1


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

        start_state = [0.0, 0.0, -0.4684210419654846, 4.629716396331787, -1.2566370964050293, 2.2487189769744873, 5.55565881729126, 4.894270420074463, 4.761992931365967, -0.029125263914465904, 0.06422368437051773]

        self.planner.world.reset_joint_states(self.robot_id, start_state, goal_state.keys())


        import pybullet as p
        p.connect(p.SHARED_MEMORY, "localhost")
        joints = [i for i in range(p.getNumJoints(self.robot_id))]

        ur5_wrist_3_joint = 13
        # print "ghdghrdklhg", p.getNumJoints(self.robot_id)
        # zero_vec = [0] * 11
        zero_vec = [0] * p.getNumJoints(self.robot_id)
        # for i in range(p.getNumJoints(self.robot_id)):
        #     print p.getJointInfo(self.robot_id, i)
        current_robot_state = self.planner.world.get_joint_states_at(self.robot_id, start_state, goal_state.keys())

        print current_robot_state[0]

        current_position_jacobian, _ = p.calculateJacobian(self.robot_id, ur5_wrist_3_joint,
                                                             # closest_pt_on_A_at_t,
                                                             [0, 0, 0],
                                                           current_robot_state[0],
                                                             zero_vec, zero_vec)

        print current_position_jacobian


        # status, _, trajectory = self.planner.get_trajectory(group=group, start_state=start_state,
        #                                                  goal_state=goal_state, samples=samples, duration=duration,
        #                                                  collision_safe_distance=collision_safe_distance,
        #                                                  collision_check_distance=collision_check_distance)
        # print("is trajectory free from collision: ", status)
        # self.planner.execute_trajectory()
        # self.planner.world.step_simulation_for(5)
        # time.sleep(5)
        # import sys
        # sys.exit(0)

    def self_coll(self):
        import pybullet as p
        import time
        p.connect(p.SHARED_MEMORY)

        paramIds = []
        jointIds = []
        jointNames = []
        start_state = [0.0, 0.0, -1.0, 4.629716396331787, -1.2566370964050293, 2.910106897354126,
                       -0.859804630279541, -0.46297168731689453, -0.06613874435424805,
                       -0.0027000010013580322, 0.006500000134110451]

        group = ['odom_x_joint', 'odom_y_joint', 'odom_z_joint', 'ur5_shoulder_pan_joint', 'ur5_shoulder_lift_joint', 'ur5_elbow_joint', 'ur5_wrist_1_joint', 'ur5_wrist_2_joint', 'ur5_wrist_3_joint', 'gripper_joint', 'gripper_base_gripper_left_joint']
        self.planner.world.reset_joint_states(self.robot_id, start_state, group)

        for j in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, j, linearDamping=0, angularDamping=0)
            info = p.getJointInfo(self.robot_id, j)
            # print(info)
            jointName = info[1]
            jointType = info[2]
            lower_limit = info[8]
            upper_limit = info[9]
            if (jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE):
                jointIds.append(j)
                paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), lower_limit, upper_limit, 0))
                # paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), -5, 5, 0))
                jointNames.append(jointName.decode("utf-8"))

        p.setRealTimeSimulation(1)
        while (1):
            jointValues = []

            for i in range(len(paramIds)):
                c = paramIds[i]
                targetPos = start_state[i] if p.readUserDebugParameter(c) == 0 else p.readUserDebugParameter(c)
                jointValues.append(targetPos)
                p.setJointMotorControl2(self.robot_id, jointIds[i], p.POSITION_CONTROL, targetPos, force=5 * 240.)
            time.sleep(0.01)
            # print jointNames
            # print jointValues
        while True:
            p.stepSimulation()
    def check_srdf(self):
        datadir = "/home/mahe/catkin_ws/src/iai_robots/iai_donbot_description/ur5_moveit_config/config/"
        stream = open(datadir + 'ur5.srdf', 'r')
        robot = SRDF.from_xml_string(stream.read())
        # print robot.disable_collisionss
        self.ignored_collisions = DefaultOrderedDict(bool)
        # self.ignored_collisions1 = DefaultOrderedDict(DefaultOrderedDict(bool))
        self.ignored_collisions1 = DefaultOrderedDict(lambda: DefaultOrderedDict(bool))

        for i in robot.disable_collisionss:
            self.ignored_collisions[i.link1, i.link2] = True
            self.ignored_collisions[i.link2, i.link1] = True
            self.ignored_collisions1[i.link1][i.link2] = True
            self.ignored_collisions1[i.link2][i.link1] = True
        print "---------------------------"
        # print self.ignored_collisions
        # print self.ignored_collisions1["adapter_fwa050_wsg50_frame_in"]["adapter_iso50_kms40_frame_in1"]
        print self.ignored_collisions1.keys()

def main():
    example = PlannerExample()
    # example.run()
    # example.self_coll()
    example.check_srdf()
    while True:
        pass


if __name__ == '__main__':
    main()
