import os
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from collections import OrderedDict
from scripts.utils.dict import DefaultOrderedDict
from srdfdom.srdf import SRDF

home = os.path.expanduser('~')

class PlannerExample:
    def __init__(self):

        location_prefix = home + '/masterThesis/bullet3/data/'

        # urdf_file = location_prefix + "kuka_iiwa/model.urdf"
        urdf_file = location_prefix + "kuka_iiwa/stomp_model.urdf"
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_iiwa_description/moveit_config/config/lbr_iiwa.srdf"

        config = {
            "use_gui": True,
            # "verbose": "INFO",
            "log_file": False,
            # "save_problem": True,
            "robot_config": "robot_config_kukka_arm.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)

        self.planner.world.robot = self.planner.robot

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        # self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.25],
        #                                           position=[0.28, -0.43, 0.9], mass=100)

        # self.box_id1 = self.planner.add_constraint("box2", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        #                                           position=[-0.48, -0.43, 0.9], mass=100)
        # self.box_id2 = self.planner.add_constraint("box3", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
        #                                           position=[-0.48, 0.43, 0.9], mass=100)

        self.planner.robot.load_srdf(srdf_file)
        self.planner.world.ignored_collisions = self.planner.robot.get_ignored_collsion()

        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)


    def run(self):

        start_state = "pick"
        goal_state = "place"
        group = "full_arm"

        duration = 20
        samples = 20
        collision_check_distance = 0.15
        collision_safe_distance = 0.1

        status, is_collision_free, trajectory = self.planner.get_trajectory(group=group, start_state= goal_state,
                                                        goal_state=start_state, samples=samples, duration=duration,
                                                        collision_safe_distance=collision_safe_distance,
                                                        collision_check_distance=collision_check_distance
                                                        )
        # print("is trajectory free from collision: ", is_collision_free)
        print (status)
        self.planner.execute_trajectory()
        self.planner.world.step_simulation_for(2)
        # import sys
        # sys.exit(0)

    def load_srdf(self):
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_iiwa_description/moveit_config/config/lbr_iiwa.srdf"

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

def main():
    example = PlannerExample()
    # example.load_srdf()
    example.run()
    while True:
        pass


if __name__ == '__main__':
    main()
