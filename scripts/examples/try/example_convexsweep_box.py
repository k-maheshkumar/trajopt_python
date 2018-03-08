import pybullet as p
import time
import numpy as np
import sys

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

        if orientation is not None:
            shape_id = p.createMultiBody(mass, col_id, vis_id, position, orientation)
        else:
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

p.connect(p.GUI)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
p.setGravity(0, 0, -10)

location_prefix = '/home/mahesh/libraries/bullet3/data/'
p.loadURDF(location_prefix + "plane.urdf", [0, 0, 0.0], useFixedBase=True)
useRealTimeSimulation = 0
if useRealTimeSimulation:
    p.setRealTimeSimulation(useRealTimeSimulation)
start = [0.28, 1.0, 0.2]
start1 = [-0.28, 1.05, 0.2]
end = [0.28, 2.0, 0.2]

box = create_constraint(shape=p.GEOM_BOX, size=[0.05, 0.35, 0.15], position=start, mass=1)
box1 = create_constraint(shape=p.GEOM_BOX, size=[0.05, 0.35, 0.15], position=start1, orientation=[0.8, 1.8, 0.2, 1], mass=1)
wall = create_constraint(shape=p.GEOM_BOX, size=[0.5, 0.01, 0.5], position=[0, 1.4, 0], mass=1)
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
# step_simulation_for(0.05)

trajectory = []


# for i in range(3):
#     traj = interpolate(start[i], end[i], 10).tolist()
#     trajectory.append(traj)
#
# trajectory = np.vstack(trajectory).T

trajectory = [[0.28, 1.0, 0.3], [0.28, 1.4, 0.3], [0.28, 1.8, 0.3],]
trajectory1 = [[-0.28, 1.0, 0.3], [-0.28, 1.4, 0.3], [-0.28, 1.8, 0.3],]
# trajectory = [[0.28, 0.3, 0.3], [0.28, 0.8, 0.3], [0.28, 1.8, 0.3], [0.28, 2.2, 0.3]]

# print(trajectory)
time.sleep(2)
print ("----------------------------box------------------------------------")

for i in range(len(trajectory)):

    print ("------------------trajectory["+str(i)+"]: ", trajectory[i])
    p.resetBasePositionAndOrientation(box, trajectory[i], [0.0, 0.0, 0.0, 1])
    time.sleep(2)

    closest_points = p.getClosestPoints(box, wall, distance=0.10)

    for j in range(len(closest_points)):
        print("-------------closest points -----------------")
        print("link A index: ", closest_points[j][3])
        print("link B index: ", closest_points[j][4])
        print("point on A: ", closest_points[j][5])
        print("point on B: ", closest_points[j][6])
        print("contact normal on B: ", closest_points[j][7])
        print("contact distance: ", closest_points[j][8])
        print("-------------************ -----------------")

    if i < (len(trajectory) - 1):

        cast_points = p.getConvexSweepClosestPoints(box, wall, distance=0.10,
                                                   bodyAfromPosition=trajectory[i],
                                                   bodyAtoPosition=trajectory[i+1],
                                                   bodyAfromOrientation= [0.0, 0.0, 0.0, 1],
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

print ("----------------------------box1------------------------------------")

for i in range(len(trajectory1)):

    print ("------------------trajectory[" + str(i) + "]: ", trajectory1[i])
    p.resetBasePositionAndOrientation(box1, trajectory1[i], [0.8, 1.8, 0.2, 1])
    time.sleep(0.5)

    closest_points = p.getClosestPoints(box1, wall, distance=0.10)

    for j in range(len(closest_points)):
        print("-------------closest points -----------------")
        print("link A index: ", closest_points[j][3])
        print("link B index: ", closest_points[j][4])
        print("point on A: ", closest_points[j][5])
        print("point on B: ", closest_points[j][6])
        print("contact normal on B: ", closest_points[j][7])
        print("contact distance: ", closest_points[j][8])
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



# sys.exit(0)
# while True:
#     pass

