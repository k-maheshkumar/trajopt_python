import pybullet as sim


class SimulationWorld:
    def __init__(self, file_path):
        location_prefix = '/home/mahesh/libraries/bullet3/data/'

        self.gui = sim.connect(sim.GUI)

        self.set_up_world(location_prefix)
        kukaEndEffectorIndex = 6
        useRealTimeSimulation = 1
        if useRealTimeSimulation:
            sim.setRealTimeSimulation(useRealTimeSimulation)
    def set_up_world(self, file_path):
        sim.loadURDF(file_path + "plane.urdf", [0, 0, -0.3], useFixedBase=True)
        sim.loadURDF(file_path + "table/table.urdf", [0, 0, -0.3], useFixedBase=True)

        kukaId = sim.loadURDF(file_path + "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        sim.resetBasePositionAndOrientation(kukaId, [0, 0.25, 0.4], [0, 0, 0, 1])

        sim.setGravity(0, 0, -10)

    def run_simulation(self):
        while 1:
            pass


sim1 = SimulationWorld(None)
sim1.run_simulation()
