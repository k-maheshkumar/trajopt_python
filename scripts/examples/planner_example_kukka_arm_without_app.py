import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))

from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from random import randrange, uniform, randint
import numpy as np

home = os.path.expanduser('~')

class PlannerExample:
    def __init__(self):

        location_prefix = home + '/masterThesis/bullet3/data/'

        config = {
            "use_gui": True,
            "verbose": "DEBUG",
            "log_file": False,
            # "save_problem": True,
            "db_name": "Trajectory_planner_evaluation",
            # "plot_trajectory": True
            "robot_config": "robot_config_kuka_arm.yaml"
        }

        self.planner = TrajectoryOptimizationPlanner(**config)

        self.planner.world.robot = self.planner.robot

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        box = randint(0, 4)
        loc = randint(0, 4)

        box_loc = [[0.164830659421, -0.464962595183, 0.9], [0.255743925678, -0.373197182129, 0.9],
                   [0.2, -0.4, 0.9], [0.4, -0.2, 0.9], [0.5, -0.3, 0.9], [0.48, -0.43, 0.9]]
        box_size = [[0.05, 0.05, 0.35], [0.03, 0.03, 0.25], [0.03, 0.1, 0.25], [0.03, 0.2, 0.15],
                    [0.06, 0.04, 0.35]]
        # self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])

        self.planner.world.toggle_rendering(0)
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=box_size[box],
                                                  position=box_loc[loc],
                                                  mass=100)

        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)


    def run(self):
        start = randint(1, 11)
        end = randint(1, 11)
        print start, end
        if start != end:
            start_state = "loc" + str(start)
            goal_state = "loc" + str(end)
            group = "full_arm"
            duration = 20
            # samples = randrange(5, 30) + randrange(1, 10)
            samples = 20

            collision_check_distance = 0.15
            collision_safe_distance = 0.1
            self.planner.reset_robot_to(start_state, group)

            status, is_collision_free, trajectory = self.planner.get_trajectory(group=group, start_state=start_state,
                                                                                goal_state=goal_state, samples=samples, duration=duration,
                                                                                collision_safe_distance=collision_safe_distance,
                                                                                collision_check_distance=collision_check_distance
                                                                                )
            print("is trajectory free from collision: ", is_collision_free)
            print (status)
            if is_collision_free:
                self.planner.execute_trajectory()


    def manual_control(self):
        start_state = "loc3"
        goal_state = "pick"
        group = "full_arm"
        group = self.planner.get_group_names(group)

        self.planner.reset_robot_to(start_state, group)

        self.planner.world.manual_control(self.planner.robot.id, group, use_current_state=True)



def main():
    example = PlannerExample()
    example.run()
    # example.manual_control()

if __name__ == '__main__':
    main()
