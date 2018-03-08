import sys
# sys.path.append('../')
sys.path.insert(0, '../')
import unittest
import numpy as np
import time
import pybullet as p


class Test_Continous_collision(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        p.connect(p.DIRECT)
        # p.connect(p.GUI)
        p.setGravity(0, 0, -10)
        cls.location_prefix = '/home/mahesh/libraries/bullet3/data/'
        p.loadURDF(cls.location_prefix + "plane.urdf", [0, 0, 0.0], useFixedBase=True)
        useRealTimeSimulation = 0
        if useRealTimeSimulation:
            p.setRealTimeSimulation(useRealTimeSimulation)

    def create_constraint(self, shape, mass, position, size=None, radius=None, height=None, orientation=None):
        if position is not None:
            if radius is not None:
                col_id = p.createCollisionShape(shape, radius=radius)
                vis_id = p.createCollisionShape(shape, radius=radius)
            if radius is not None and height is not None:
                col_id = p.createCollisionShape(shape, radius=radius, height=height)
                vis_id = p.createCollisionShape(shape, radius=radius, height=height)
            if size is not None:
                col_id = p.createCollisionShape(shape, halfExtents=size)
                vis_id = p.createCollisionShape(shape, halfExtents=size)
            shape_id = p.createMultiBody(mass, col_id, vis_id, position)

        return shape_id

    def reset_joint_states_to(self, robot_id, trajectory, joints, joint_name_to_id):

        if len(trajectory) == len(joints):
            for i in range(len(trajectory)):
                p.resetJointState(robot_id, joint_name_to_id[joints[i]], trajectory[i])

    def interpolate(self, start, end, samples=5, decimals_to_round=3):
        data = []
        step_size = (end - start) / (samples - 1)
        intermediate = start
        for i in range(samples):
            data.append(intermediate)
            intermediate += step_size
        return np.round(data, decimals_to_round)

    def step_simulation_for(self, seconds):
        start = time.time()
        while time.time() < start + seconds:
            p.stepSimulation()

    def setUp(self):
        pass

    def tearDown(self):
        pass

    def test_sphere(self):
        print("------------------------------ testing sphere ------------------------------")

        start = [0.28, 1.0, 0.3]
        end = [0.28, 1.8, 0.3]
        sphere = self.create_constraint(shape=p.GEOM_SPHERE, radius=0.3, position=start, mass=1)
        wall = self.create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.5], position=[0, 1.2, 0], mass=1)

        trajectory = []

        for i in range(3):
            traj = self.interpolate(start[i], end[i], 3).tolist()
            trajectory.append(traj)

        trajectory = np.vstack(trajectory).T
        for i in range(len(trajectory)):
            p.resetBasePositionAndOrientation(sphere, trajectory[i], [0.0, 0.0, 0.0, 1])

            if i < len(trajectory) - 1:

                closet_points = p.getClosestPoints(sphere, wall, distance=0.10)
                start = time.time()
                cast_points = p.getConvexSweepClosestPoints(sphere, wall, distance=0.10,
                                                           bodyAfromPosition=trajectory[i],
                                                           bodyAtoPosition=trajectory[i + 1],
                                                           bodyAfromOrientation=[0.0, 0.0, 0.0, 1],
                                                           bodyAtoOrientation=[0.0, 0.0, 0.0, 1]
                                                           )

                for i in range(len(closet_points)):
                    if len(closet_points):
                        if closet_points[i][8] < 0:
                            print("-------------closest points -----------------")
                            print("link A index: ", closet_points[i][3])
                            print("link B index: ", closet_points[i][4])
                            print("point on A: ", closet_points[i][5])
                            print("point on B: ", closet_points[i][6])
                            print("contact normal on B: ", closet_points[i][7])
                            print("contact distance: ", closet_points[i][8])
                            print("-------------************ -----------------")

                for k in range(len(cast_points)):
                    if len(cast_points):
                        if cast_points[i][9] < 0:
                            print("-------------cast_points -----------------")
                            print("link A index: ", cast_points[k][3])
                            print("link B index: ", cast_points[k][4])
                            print("point on A(t): ", cast_points[k][5])
                            print("point on A(t+1): ", cast_points[k][6])
                            print("point on B: ", cast_points[k][7])
                            print("contact normal on B: ", cast_points[k][8])
                            print("contact distance: ", cast_points[k][9])
                            print("contact fraction: ", cast_points[k][10])
                            print("-------------************ -----------------")

                if cast_points[k][10] == 0:
                    self.assertEquals(np.isclose(cast_points[k][9], closet_points[i][8], atol=0.01), True)
                else:
                    self.assertEquals(np.isclose(cast_points[k][10], 0.5, atol=0.01), True)
        print("------------------------------ testing sphere done------------------------------")

    def test_box(self):
        print("------------------------------ testing boxes ------------------------------")
        start = [0.28, 1.0, 0.2]
        start1 = [-0.28, 1.05, 0.2]
        end = [0.28, 2.0, 0.2]
        trajectory = [[0.28, 1.0, 0.3], [0.28, 1.8, 0.3], ]
        trajectory1 = [[-0.28, 0.7, 0.3], [-0.28, 1.8, 0.3], ]
        box = self.create_constraint(shape=p.GEOM_BOX, size=[0.05, 0.35, 0.15], position=start, mass=1)
        box1 = self.create_constraint(shape=p.GEOM_BOX, size=[0.05, 0.35, 0.15], position=start1,
                                 orientation=[0.8, 1.8, 0.2, 1], mass=1)
        wall = self.create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.5], position=[0, 1.4, 0], mass=1)

        print ("----------------------------box 1------------------------------------")

        for i in range(len(trajectory)):

            p.resetBasePositionAndOrientation(box, trajectory[i], [0.0, 0.0, 0.0, 1])
            time.sleep(2)

            closet_points = p.getClosestPoints(box, wall, distance=0.10)

            for j in range(len(closet_points)):
                print("-------------closest points -----------------")
                print("link A index: ", closet_points[j][3])
                print("link B index: ", closet_points[j][4])
                print("point on A: ", closet_points[j][5])
                print("point on B: ", closet_points[j][6])
                print("contact normal on B: ", closet_points[j][7])
                print("contact distance: ", closet_points[j][8])
                print("-------------************ -----------------")

            if i < (len(trajectory) - 1):

                cast_points = p.getConvexSweepClosestPoints(box, wall, distance=0.10,
                                                           bodyAfromPosition=trajectory[i],
                                                           bodyAtoPosition=trajectory[i + 1],
                                                           bodyAfromOrientation=[0.0, 0.0, 0.0, 1],
                                                           bodyAtoOrientation=[0.0, 0.0, 0.0, 1]
                                                           )

                for k in range(len(cast_points)):
                    print("-------------cast_points -----------------")
                    print("link A index: ", cast_points[k][3])
                    print("link B index: ", cast_points[k][4])
                    print("point on A(t): ", cast_points[k][5])
                    print("point on A(t+1): ", cast_points[k][6])
                    print("point on B: ", cast_points[k][7])
                    print("contact normal on B: ", cast_points[k][8])
                    print("contact distance: ", cast_points[k][9])
                    print("contact fraction: ", cast_points[k][10])
                    print("-------------************ -----------------")

                    self.assertEquals(np.isclose(cast_points[k][10], 0.5, atol=0.01), True)

        print ("----------------------------box 2------------------------------------")

        for i in range(len(trajectory1)):

            print ("------------------trajectory[" + str(i) + "]: ", trajectory1[i])
            p.resetBasePositionAndOrientation(box1, trajectory1[i], [0.8, 1.8, 0.2, 1])
            time.sleep(0.5)

            closet_points = p.getClosestPoints(box1, wall, distance=0.10)

            for j in range(len(closet_points)):
                print("-------------closest points -----------------")
                print("link A index: ", closet_points[j][3])
                print("link B index: ", closet_points[j][4])
                print("point on A: ", closet_points[j][5])
                print("point on B: ", closet_points[j][6])
                print("contact normal on B: ", closet_points[j][7])
                print("contact distance: ", closet_points[j][8])
                print("-------------************ -----------------")

            if i < (len(trajectory1) - 1):

                cast_points = p.getConvexSweepClosestPoints(box1, wall, distance=0.10,
                                                           bodyAfromPosition=trajectory1[i],
                                                           bodyAtoPosition=trajectory1[i + 1],
                                                           bodyAfromOrientation=[0.8, 1.8, 0.2, 1],
                                                           bodyAtoOrientation=[0.8, 1.8, 0.2, 1]
                                                           )

                for k in range(len(cast_points)):
                    print("-------------cast_points -----------------")
                    print("link A index: ", cast_points[k][3])
                    print("link B index: ", cast_points[k][4])
                    print("point on A(t): ", cast_points[k][5])
                    print("point on A(t+1): ", cast_points[k][6])
                    print("point on B: ", cast_points[k][7])
                    print("contact normal on B: ", cast_points[k][8])
                    print("contact distance: ", cast_points[k][9])
                    print("contact fraction: ", cast_points[k][10])
                    print("-------------************ -----------------")


                    self.assertEquals(np.isclose(cast_points[k][10], 0.63, atol=0.01), True)
        print("------------------------------ testing boxes done------------------------------")


    def test_kukka_arm(self):
        print("------------------------------ testing kukka arm ------------------------------")
        kukaId = p.loadURDF(self.location_prefix + "kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
        wall = self.create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.18], position=[0.95, 0.3, 0], mass=1)
        self.step_simulation_for(0.05)

        trajectory = []
        # print jointInfo[1], jointNameToId[jointInfo[1]]

        start_state = [1.5708024764979394, 1.5712326366904628, 1.57079632708463, 1.5707855978401637,
                       -1.5701336236638301,
                       1.5719725305810723, 1.570793057646014]
        end_state = [0.6799090616158451, 1.5729501835585853, 1.5704022441549654, 1.6398254945800432,
                     -1.5625212113651783,
                     1.596406342019127, 1.568985184095934]

        for i in range(len(start_state)):
            trajectory.append(self.interpolate(start_state[i], end_state[i], 3, 5))

        trajectory = np.asarray(trajectory).T

        useRealTimeSimulation = 0
        if useRealTimeSimulation:
            p.setRealTimeSimulation(useRealTimeSimulation)

        jointNameToId = {}
        numJoints = p.getNumJoints(kukaId)

        joints = ['lbr_iiwa_joint_1', 'lbr_iiwa_joint_2', 'lbr_iiwa_joint_3', 'lbr_iiwa_joint_4',
                  'lbr_iiwa_joint_5', 'lbr_iiwa_joint_6', 'lbr_iiwa_joint_7']

        for i in range(numJoints):
            jointInfo = p.getJointInfo(kukaId, i)
            # print jointInfo
            jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]


        for i in range(numJoints):
            p.resetJointState(kukaId, jointNameToId[joints[i]], start_state[i])

        for i in range(len(trajectory)):

            next_link_state = None
            if i < (len(trajectory) - 1):
                self.reset_joint_states_to(kukaId, trajectory[i + 1], joints, jointNameToId)
                next_link_state = p.getLinkState(kukaId, 6, computeLinkVelocity=1,
                                                 computeForwardKinematics=1)

            self.reset_joint_states_to(kukaId, trajectory[i], joints, jointNameToId)
            current_link_state = p.getLinkState(kukaId, 6, computeLinkVelocity=1,
                                                computeForwardKinematics=1)
            time.sleep(0.5)

            closest_points = p.getClosestPoints(kukaId, wall, linkIndexA=6, distance=0.10)

            for j in range(len(closest_points)):
                if len(closest_points):
                    if closest_points[j][8] < 0:
                        print("-------------closest points -----------------")
                        print("link A index: ", closest_points[j][3])
                        print("link B index: ", closest_points[j][4])
                        print("point on A: ", closest_points[j][5])
                        print("point on B: ", closest_points[j][6])
                        print("contact normal on B: ", closest_points[j][7])
                        print("contact distance: ", closest_points[j][8])
                        print("-------------************ -----------------")
            if next_link_state is not None:

                cast_points = p.getConvexSweepClosestPoints(kukaId, wall, distance=0.10, linkIndexA=6,
                                                           bodyAfromPosition=current_link_state[0],
                                                           bodyAtoPosition=next_link_state[0],
                                                           bodyAfromOrientation=current_link_state[1],
                                                           bodyAtoOrientation=next_link_state[1]
                                                           )

                for k in range(len(cast_points)):
                    if len(cast_points):
                        if cast_points[k][9] < 0:
                            print("-------------cast_points -----------------")
                            print("link A index: ", cast_points[k][3])
                            print("link B index: ", cast_points[k][4])
                            print("point on A(t): ", cast_points[k][5])
                            print("point on A(t+1): ", cast_points[k][6])
                            print("point on B: ", cast_points[k][7])
                            print("contact normal on B: ", cast_points[k][8])
                            print("contact distance: ", cast_points[k][9])
                            print("contact fraction: ", cast_points[k][10])
                            print("-------------************ -----------------")
                            self.assertEquals(np.isclose(cast_points[k][10], 0.52, atol=0.01), True)
                            self.assertEquals(np.isclose(cast_points[k][9], -0.07, atol=0.01), True)
        print("------------------------------ testing kukka arm done------------------------------")


if __name__ == '__main__':
    unittest.main()


