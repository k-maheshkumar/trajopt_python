import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), "../../"))
from scripts.simulation.SimulationWorld import SimulationWorld
from scripts.GUI import TrajPlanner
from PyQt4 import QtGui
import sys
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner
from collections import OrderedDict
from random import randrange, uniform, randint


class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        config = {
            "use_gui": True,
            "verbose": False,
            "log_file": False,
            "robot_config": "robot_config_kuka_arm.yaml"
        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        box = randint(0, 4)
        loc = randint(0, 4)

        box, loc = 2, 1

        box_loc = [[0.164830659421, -0.464962595183, 0.9], [0.255743925678, -0.373197182129, 0.9],
                   [0.2, -0.4, 0.9], [0.4, -0.2, 0.9], [0.5, -0.3, 0.9], [0.48, -0.43, 0.9]]
        box_size = [[0.05, 0.05, 0.35], [0.03, 0.03, 0.25], [0.03, 0.1, 0.25], [0.03, 0.2, 0.15],
                    [0.06, 0.04, 0.35]]
        # self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])

        self.planner.world.toggle_rendering(0)
        plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf("table", urdf_file=location_prefix + "table/table.urdf",
                                                         position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint("box1", shape=self.planner.world.BOX, size=box_size[box],
                                                  position=box_loc[loc],
                                                  mass=100)

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
