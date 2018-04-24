import os
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from scripts.utils.dict import DefaultOrderedDict
from collections import OrderedDict

from srdfdom.srdf import SRDF

home = os.path.expanduser('~')


class PlannerExample:
    def __init__(self):

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/urdf/kuka_husky.urdf"
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/moveit_config/config/kuka_husky.srdf"

        config = {
            "use_gui": True,
            "verbose": True,
            "log_file": False,
            # "save_problem": True,
            "robot_config": "robot_config_kukka_arm.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)

        self.planner.world.robot = self.planner.robot

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        self.robot_id = self.planner.load_robot(urdf_file,
                                                # use_fixed_base=True
                                                )
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        # table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf",
        #                                                  position=[0, 0, 0.0])
        #
        self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[0.68, 0.05, 0.9], mass=100)
        #
        # self.box_id1 = self.planner.add_constraint("box2", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        #                                           position=[-0.48, -0.43, 0.9], mass=100)
        # self.box_id2 = self.planner.add_constraint("box3", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        #                                           position=[-0.48, 0.43, 0.9], mass=100)



        self.planner.robot.load_srdf(srdf_file)
        self.planner.world.ignored_collisions = self.planner.robot.get_ignored_collsion()
        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(1)

    def run(self):
        start_state = "pick"
        # goal_state = "place"
        # group = "full_arm"

        start_state = OrderedDict()

        start_state["front_left_wheel"] = 0.0
        start_state["front_right_wheel"] = 0.0
        start_state["rear_left_wheel"] = 0.0
        start_state["rear_right_wheel"] = 0.0

        start_state["lbr_iiwa_joint_1"] = -1.8933
        start_state["lbr_iiwa_joint_2"] = 1.5694
        start_state["lbr_iiwa_joint_3"] = 0.9404
        start_state["lbr_iiwa_joint_4"] = -1.0499
        start_state["lbr_iiwa_joint_5"] = -0.5409
        start_state["lbr_iiwa_joint_6"] = 1.2149
        start_state["lbr_iiwa_joint_7"] = 0.0

        # start_state["lbr_iiwa_joint_1"] = -1.3933
        # start_state["lbr_iiwa_joint_2"] = 0.9694
        # start_state["lbr_iiwa_joint_3"] = 0.9404
        # start_state["lbr_iiwa_joint_4"] = -1.0499
        # start_state["lbr_iiwa_joint_5"] = -0.5409
        # start_state["lbr_iiwa_joint_6"] = 1.2149
        # start_state["lbr_iiwa_joint_7"] = 0.0

        goal_state = OrderedDict()

        goal_state["front_left_wheel"] = 0.5
        goal_state["front_right_wheel"] = 0.5
        goal_state["rear_left_wheel"] = 0.2
        goal_state["rear_right_wheel"] = 0.6

        goal_state["lbr_iiwa_joint_1"] = 0.8032
        goal_state["lbr_iiwa_joint_2"] = 1.4067
        goal_state["lbr_iiwa_joint_3"] = 0.9404
        goal_state["lbr_iiwa_joint_4"] = -1.0499
        goal_state["lbr_iiwa_joint_5"] = -0.5409
        goal_state["lbr_iiwa_joint_6"] = 1.2149
        goal_state["lbr_iiwa_joint_7"] = 0.0

        # self.planner.world.reset_joint_states(self.planner.robot.id, start_state.values(), start_state.keys())
        # self.planner.world.step_simulation_for(0.2)

        duration = 20
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.10
        # start_state = [1.561610221862793, 2.094395160675049, 2.96705961227417, 1.5873310565948486, 2.96705961227417,
        #                1.1904981136322021, 0.0]
        status, is_collision_free, trajectory = self.planner.get_trajectory(group=start_state.keys(), start_state=start_state.values(),
                                                                            goal_state=goal_state.values(), samples=samples,
                                                                            duration=duration,
                                                                            collision_safe_distance=collision_safe_distance,
                                                                            collision_check_distance=collision_check_distance
                                                                            )
        print("is trajectory free from collision: ", is_collision_free)
        print status
        # print trajectory.final
        self.planner.execute_trajectory()
        self.planner.world.step_simulation_for(2)
        # # # import sys
        # # # sys.exit(0)

    def load_srdf(self):
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_husky_description/moveit_config/config/kuka_husky.srdf"

        stream = open(srdf_file, 'r')
        srdf = SRDF.from_xml_string(stream.read())

        ignored_collisions = DefaultOrderedDict(bool)

        for collision in srdf.disable_collisionss:
            ignored_collisions[collision.link1, collision.link2] = True
            ignored_collisions[collision.link2, collision.link1] = True
        # print ignored_collisions
        self.planner.world.ignored_collisions = ignored_collisions

    def manual_control(self):
        import pybullet as p
        import time

        p.connect(p.SHARED_MEMORY)
        jointIds = []
        paramIds     = []
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
                paramIds.append(p.addUserDebugParameter(jointName.decode("utf-8"), lower_limit, upper_limit, joint_state[0]))

        p.setRealTimeSimulation(1)
        while (1):
            for i in range(len(paramIds)):
                c = paramIds[i]
                targetPos = p.readUserDebugParameter(c)
                p.setJointMotorControl2(self.planner.robot.id, jointIds[i], p.VELOCITY_CONTROL, targetPos, force=5 * 240.)
            time.sleep(0.01)

    def connect_directly(self):

        start_state = OrderedDict()

        start_state["front_left_wheel"] = 0.0
        start_state["front_right_wheel"] = 0.0
        start_state["rear_left_wheel"] = 0.0
        start_state["rear_right_wheel"] = 0.0

        start_state["lbr_iiwa_joint_1"] = -1.8933
        start_state["lbr_iiwa_joint_2"] = 1.5694
        start_state["lbr_iiwa_joint_3"] = 0.9404
        start_state["lbr_iiwa_joint_4"] = -1.0499
        start_state["lbr_iiwa_joint_5"] = -0.5409
        start_state["lbr_iiwa_joint_6"] = 1.2149
        start_state["lbr_iiwa_joint_7"] = 0.0

        import pybullet as p
        p.connect(p.SHARED_MEMORY, "localhost")
        joints = [i for i in range(p.getNumJoints(self.robot_id))]

        index = 17
        # print "ghdghrdklhg", p.getNumJoints(self.robot_id)
        # zero_vec = [0] * 11
        zero_vec = [0] * p.getNumJoints(self.robot_id)
        # for i in range(p.getNumJoints(self.robot_id)):
        #     print p.getJointInfo(self.robot_id, i)
        current_robot_state = self.planner.world.get_joint_states_at(self.robot_id, start_state.values(), start_state.keys())
        zero_vec = [0] * len(current_robot_state[0])

        print "current_robot_state*------------------", len(current_robot_state[0])
        print current_robot_state[0]
        print len(zero_vec), len(current_robot_state[0]), p.getNumJoints(self.robot_id)

        current_position_jacobian, _ = p.calculateJacobian(self.robot_id, index,
                                                             # closest_pt_on_A_at_t,
                                                             [0, 0, 0],
                                                           current_robot_state[0],
                                                             zero_vec, zero_vec)

        print current_position_jacobian
        print len(current_position_jacobian[0])

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
