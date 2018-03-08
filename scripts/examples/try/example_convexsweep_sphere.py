# import time
import math
import pybullet as p
import time

import numpy as np


def create_constraint(shape, mass, position, size=None, radius=None, height=None, orientation=None):
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

def interpolate(start, end, samples=5, decimals_to_round=3):
    data = []
    step_size = (end - start) / (samples - 1)
    intermediate = start
    for i in range(samples):
        data.append(intermediate)
        intermediate += step_size
    return np.round(data, decimals_to_round)

def step_simulation_for(seconds):
    start = time.time()
    while time.time() < start + seconds:
        p.stepSimulation()

# __all__ = [pybullet, math, datetime]

# clid = p.connect(p.SHARED_MEMORY)

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, -10)

location_prefix = '/home/mahesh/libraries/bullet3/data/'
p.loadURDF(location_prefix + "plane.urdf", [0, 0, 0.0], useFixedBase=True)
useRealTimeSimulation = 0
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)

start = [0.28, 0.3, 0.3]
end = [0.28, 1.8, 0.3]
sphere = create_constraint(shape=p.GEOM_SPHERE, radius=0.3, position=start, mass=1)
wall = create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.5], position=[0, 1.2, 0], mass=1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
step_simulation_for(0.05)

trajectory = []


for i in range(3):
    traj = interpolate(start[i], end[i], 10).tolist()
    trajectory.append(traj)

trajectory = np.vstack(trajectory).T

# print(trajectory)
time.sleep(2)

for i in range(len(trajectory)):
    print ("------------------trajectory["+str(i)+"]: ", trajectory[i])
    p.resetBasePositionAndOrientation(sphere, trajectory[i], [0.0, 0.0, 0.0, 1])
    time.sleep(0.5)

    if i < len(trajectory) - 1:

        closest_points = p.getClosestPoints(sphere, wall, distance=0.10)
        start = time.time()
        cast_points = p.getConvexSweepClosestPoints(sphere, wall, distance=0.10,
                                                   bodyAfromPosition=trajectory[i],
                                                   bodyAtoPosition=trajectory[i+1],
                                                   bodyAfromOrientation= [0.0, 0.0, 0.0, 1],
                                                   bodyAtoOrientation=[0.0, 0.0, 0.0, 1]
                                                   )
        # print (len(closet_points))
        # print (len(cast_points))


        print ("time :--------------------- ", time.time() - start)
        for i in range(len(closest_points)):
            print("-------------closest points -----------------")
            print("link A index: ", closest_points[i][3])
            print("link B index: ", closest_points[i][4])
            print("point on A: ", closest_points[i][5])
            print("point on B: ", closest_points[i][6])
            print("contact normal on B: ", closest_points[i][7])
            print("contact distance: ", closest_points[i][8])
            print("-------------************ -----------------")

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
#
# while True:
#     pass

