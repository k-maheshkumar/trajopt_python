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
        self.robot_id = self.planner.load_robot(urdf_file)
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

        start_state["lbr_iiwa_joint_1"] = -1.3933
        start_state["lbr_iiwa_joint_2"] = 0.9694
        start_state["lbr_iiwa_joint_3"] = 0.9404
        start_state["lbr_iiwa_joint_4"] = -1.0499
        start_state["lbr_iiwa_joint_5"] = -0.5409
        start_state["lbr_iiwa_joint_6"] = 1.2149
        start_state["lbr_iiwa_joint_7"] = 0.0

        goal_state = OrderedDict()

        goal_state["front_left_wheel"] = 2.1
        goal_state["front_right_wheel"] = 2.1
        goal_state["rear_left_wheel"] = 2.1
        goal_state["rear_right_wheel"] = 2.1

        goal_state["lbr_iiwa_joint_1"] = 0.8032
        goal_state["lbr_iiwa_joint_2"] = 1.0067
        goal_state["lbr_iiwa_joint_3"] = 0.9404
        goal_state["lbr_iiwa_joint_4"] = -1.0499
        goal_state["lbr_iiwa_joint_5"] = -0.5409
        goal_state["lbr_iiwa_joint_6"] = 1.2149
        goal_state["lbr_iiwa_joint_7"] = 0.0

        # self.planner.world.reset_joint_states(self.planner.robot.id, start_state.values(), goal_state.keys())

        duration = 20
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1
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
        self.planner.execute_trajectory()
        self.planner.world.step_simulation_for(2)
        # import sys
        # sys.exit(0)

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

def main():
    example = PlannerExample()
    example.load_srdf()
    example.run()
    while True:
        pass

if __name__ == '__main__':
    main()
