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
        shelf_file = home + "/catkin_ws/src/iai_shelf_description/urdf/shelf.urdf"

        config = {
            "use_gui": True,
            "verbose": False,
            "log_file": False,
            # "robot_config": "robot_config_kukka_arm.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)
        #
        # self.planner.world.set_gravity(0, 0, -10)
        # self.planner.world.toggle_rendering(0)
        # # # self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        # plane_id = self.planner.load_from_urdf("plane", urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])
        # shelf_id = self.planner.add_constraint_from_urdf("shelf", urdf_file=shelf_file, position=[-0.47, 0, 0.0],
        #                                                  orientation=[0, 0, 1.57])
        #
        # self.planner.world.toggle_rendering(1)
        # # self.planner.world.step_simulation_for(0.01)
        #
        # shelf_item_prefix = home + "/catkin_ws/src/shelf_item_descriptions/urdf/"
        # salt_urdf = shelf_item_prefix + "salt.urdf"
        # gel_urdf = shelf_item_prefix + "duschGel.urdf"
        # gel_id = OrderedDict()
        #
        # y, z = 0.3, 1.0
        # offset = -0.58
        # # offset = uniform(-0.7, -0.58)
        # zs = [0.2, 0.6, 1]
        #
        # obj_at_shelf = randint(1, 4)
        # obj_at_shelf = 3
        # y = 0.3
        # z = 2
        # for x in range(obj_at_shelf):
        #     # y = uniform(-0.2, 0.2)
        #     # z = uniform(0.3, 1.5)
        #     # z = randint(0, 2)
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
        #                                                       position=[offset + 0.1 * x, y-0.2, zs[z]])
        #     gel_id[x + 4] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
        #                                                           position=[offset + 0.1 * x, y - 0.38, zs[z]])
        #
        # y, z = -0.3, 0.62
        # offset = -0.59
        # for x in range(2):
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=gel_urdf,
        #                                                       position=[offset + 0.1 * x, y, z])
        # for x in range(2):
        #     gel_id[x] = self.planner.add_constraint_from_urdf("gel" + str(x), urdf_file=salt_urdf,
        #                                                       position=[offset + 0.1 * x, 0.3, z])

    def init(self):
        start_state = "below_shelf"
        group = "full_body"
        start = 3
        start_state = "floc" + str(start)
        self.planner.reset_robot_to(start_state, group)

def start_planner_app():
    example = PlannerExample()
    # example.init()
    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner.PlannerGui(verbose="DEBUG", file_log=False, planner=example.planner)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    start_planner_app()
