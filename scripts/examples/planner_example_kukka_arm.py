import os
from scripts.GUI import TrajPlanner
from PyQt4 import QtGui
import sys
from scripts.TrajectoryOptimizationPlanner.TrajectoryOptimizationPlanner import TrajectoryOptimizationPlanner

class PlannerExample:
    def __init__(self):
        home = os.path.expanduser('~')

        location_prefix = home + '/masterThesis/bullet3/data/'

        urdf_file = location_prefix + "kuka_iiwa/model.urdf"

        config = {
            "use_gui": True,
            "verbose": False,
            "log_file": False,
            "robot_config": "robot_config_kukka_arm.yaml"

        }

        self.planner = TrajectoryOptimizationPlanner(**config)
        # self.planner = TrajectoryOptimizationPlanner(urdf_file, use_gui=False)

        self.planner.world.set_gravity(0, 0, -10)
        self.planner.world.toggle_rendering(0)
        self.robot_id = self.planner.load_robot(urdf_file, position=[0, 0.25, 0.6])
        plane_id = self.planner.load_from_urdf(urdf_file=location_prefix + "plane.urdf", position=[0, 0, 0.0])

        table_id = self.planner.add_constraint_from_urdf(urdf_file=location_prefix + "table/table.urdf",
                                                         position=[0, 0, 0.0])

        self.box_id = self.planner.add_constraint(shape=self.planner.world.BOX, size=[0.1, 0.2, 0.45],
                                                  position=[0.28, -0.43, 0.9], mass=100)

        self.planner.world.toggle_rendering(1)
        self.planner.world.step_simulation_for(0.01)

    def init(self):
        start_state = {}

        start_state["lbr_iiwa_joint_1"] = -2.4823357809267463
        start_state["lbr_iiwa_joint_2"] = 1.4999975516996142
        start_state["lbr_iiwa_joint_3"] = -1.5762726255540713
        start_state["lbr_iiwa_joint_4"] = -0.8666279970481103
        start_state["lbr_iiwa_joint_5"] = 1.5855963769735366
        start_state["lbr_iiwa_joint_6"] = 1.5770985888989753
        start_state["lbr_iiwa_joint_7"] = 1.5704531145724918

        self.planner.world.reset_joint_states(self.planner.robot.id, start_state)


def start_planner_app():
    example = PlannerExample()
    example.init()
    app = QtGui.QApplication(sys.argv)
    window = TrajPlanner.PlannerGui(verbose="DEBUG", file_log=False, planner=example.planner)
    window.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    start_planner_app()
