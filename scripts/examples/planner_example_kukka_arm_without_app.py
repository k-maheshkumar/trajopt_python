import os
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from random import randrange, uniform

home = os.path.expanduser('~')

class PlannerExample:
    def __init__(self):

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"
        srdf_file = home + "/catkin_ws/src/robot_descriptions/kuka_iiwa_description/moveit_config/config/lbr_iiwa.srdf"

        config = {
             "use_gui": True,
            # "verbose": "DEBUG",
            "log_file": False,
            "save_problem": True,
            # "db_name": "Trajectory_planner_results",
            "db_name": "Trajectory_planner_evaluation",
            "robot_config": "robot_config_kukka_arm.yaml",
            # "plot_trajectory": True
        }

        self.planner = TrajectoryOptimizationPlanner(**config)

        self.planner.world.robot = self.planner.robot

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        x = uniform(0, 0.5)
        y = uniform(0, 0.5)

        self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.25],
                                                  position=[0.28, -0.43, 0.9],
                                                  # position=[x, -y, 0.9],
                                                  mass=100)
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
        # samples = randrange(5, 30) + randrange(1, 10)
        samples = 20

        collision_check_distance = 0.15
        collision_safe_distance = 0.1

        status, is_collision_free, trajectory = self.planner.get_trajectory(group=group, start_state= goal_state,
                                                        goal_state=start_state, samples=samples, duration=duration,
                                                        collision_safe_distance=collision_safe_distance,
                                                        collision_check_distance=collision_check_distance
                                                        )
        print("is trajectory free from collision: ", is_collision_free)
        print (status)
        # if is_collision_free:
        #     self.planner.execute_trajectory()


def main():
    example = PlannerExample()
    example.run()


if __name__ == '__main__':
    main()
