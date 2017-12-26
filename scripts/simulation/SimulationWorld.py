import pybullet as sim
from scripts.Planner import Robot

class SimulationWorld:
    def __init__(self, file_path=None):
        location_prefix = '/home/mahe/masterThesis/bullet3/data/'
        # location_prefix = '/home/mahesh/libraries/bullet3/data/'
        urdf_file = "kuka_iiwa/model.urdf"
        self.robot = Robot.Robot(location_prefix + urdf_file)
        # self.gui = sim.connect(sim.GUI)

        # self.set_up_world(location_prefix, urdf_file)
        kukaEndEffectorIndex = 6
        useRealTimeSimulation = 1
        # if useRealTimeSimulation:
        #     sim.setRealTimeSimulation(useRealTimeSimulation)
    def set_up_world(self, file_path, urdf_file):
        sim.loadURDF(file_path + "plane.urdf", [0, 0, -0.3], useFixedBase=True)
        sim.loadURDF(file_path + "table/table.urdf", [0, 0, -0.3], useFixedBase=True)

        kukaId = sim.loadURDF(file_path + urdf_file, [0, 0, 0], useFixedBase=True)
        sim.resetBasePositionAndOrientation(kukaId, [0, 0.25, 0.4], [0, 0, 0, 1])

        sim.setGravity(0, 0, -10)

    # def run_simulation(self):
    #     while 1:
    #         pass
    def plan_trajectory(self, group, goal_state, samples, duration):
        print group, goal_state, samples, duration
        self.robot.plan_trajectory(group=group, current_state=self.get_robot_current_state(),
                                   goal_state=goal_state, samples=int(samples), duration=int(duration))

    def get_robot_current_state(self):

        current_state = {
                'lbr_iiwa_joint_1': -0.49197958189616936,
                'lbr_iiwa_joint_2': 1.4223062659337982,
                'lbr_iiwa_joint_3': -1.5688299779644697,
                'lbr_iiwa_joint_4':  -1.3135004031364736,
                'lbr_iiwa_joint_5': 1.5696229411153653,
                'lbr_iiwa_joint_6': 1.5749627479696093,
                'lbr_iiwa_joint_7': 1.5708037563007493,
            }
        return current_state


# sim1 = SimulationWorld(None)
# sim1.run_simulation()
