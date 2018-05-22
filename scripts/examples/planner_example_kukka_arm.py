from scripts.simulation.SimulationWorld import SimulationWorld
import os
from scripts.GUI import TrajPlanner
from PyQt4 import QtGui
import sys
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from collections import OrderedDict

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        config = {
            "use_gui": True,
            "verbose": False,
            "log_file": False,
            # "robot_config": "robot_config_kukka_arm.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        # self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf",
                                                         position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint("box", shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[0.28, -0.43, 0.9], mass=100)

        # self.planner.robot.load_srdf(srdf_file)
        # self.planner.world.ignored_collisions = self.planner.robot.get_ignored_collsion()

        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)

    def init(self):
        start_state = "home"
        group = "full_arm"

        self.planner.reset_robot_to(start_state, group)

def start_planner_app():
    example = PlannerExample()
    example.init()
    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner.PlannerGui(verbose="DEBUG", file_log=False, planner=example.planner)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    start_planner_app()
