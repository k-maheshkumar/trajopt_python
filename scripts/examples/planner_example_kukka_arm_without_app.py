import os
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from collections import OrderedDict

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"

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
        self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf", position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[0.28, -0.43, 0.9], mass=100)

        self.box_id1 = self.planner.add_constraint("box2", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[-0.48, -0.43, 0.9], mass=100)
        self.box_id2 = self.planner.add_constraint("box3", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[-0.48, 0.43, 0.9], mass=100)


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
        print("is trajectory free from collision: ", is_collision_free)
        print status
        self.planner.execute_trajectory()
        self.planner.world.step_simulation_for(2)
        import sys
        sys.exit(0)


def main():
    example = PlannerExample()
    example.run()


if __name__ == '__main__':
    main()
