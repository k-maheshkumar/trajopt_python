import pybullet as sim

location_prefix = '/home/mahe/masterThesis/bullet3/data/'
urdf_file = ""
file_path = location_prefix
sim.connect(sim.GUI)
sim.setGravity(0, 0, -10)
planeId = sim.loadURDF(file_path + "plane.urdf", [0, 0, 0.0], useFixedBase=True)
world = sim.loadURDF("../../../world/world.urdf")
# table_id = sim.loadURDF(file_path + "table/table.urdf", [0, 0, -0.3], useFixedBase=True)

# robot_id = sim.loadURDF(urdf_file, [0, 0, 0], useFixedBase=True)
# sim.resetBasePositionAndOrientation(self.robot_id, [0, 0.0, 0.0], [0, 0, 0, 1])
# sim.resetBasePositionAndOrientation(robot_id, [0, 0.25, 0.4], [0, 0, 0, 1])

sim.setGravity(0, 0, -10)
# no_of_joints = sim.getNumJoints(robot_id)


def run_simulation():
    while 1:
        pass


if __name__ == '__main__':
    run_simulation()
