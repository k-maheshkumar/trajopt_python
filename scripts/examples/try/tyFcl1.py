from fcl import fcl, transform, collision_data

objs = [fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0)),
        fcl.CollisionObject(fcl.Sphere(4.0)),
        fcl.CollisionObject(fcl.Cone(5.0, 6.0))]

# Register objects to DynamicAABBTreeCollisionManager
manager = fcl.DynamicAABBTreeCollisionManager()
print("Before register: ", manager.size())
manager.registerObjects(objs)
print("After register 1 : ", manager.size())
manager.registerObject(fcl.CollisionObject(fcl.Cylinder(7.0, 8.0)))
print("After register 2 : ", manager.size())


# Use Callback function
def cb_func(obj1, obj2, res):
    print("cb_func start")
    ret, res = fcl.collide(obj1, obj2, collision_data.CollisionRequest())
    print("result: ", ret)
    return ret


res = collision_data.CollisionResult()
manager.collide(res, cb_func)

# Collision calculation
ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
                                              transform.Transform(transform.Quaternion(), [10.0, 0.0, 0.0])),
                          fcl.CollisionObject(fcl.Sphere(4.0),
                                              transform.Transform(transform.Quaternion(), [-10.0, 0.0, 0.0])),
                          collision_data.CollisionRequest())

print("-- Collision result: ", ret)
for contact in result.contacts:
    print(contact.o1)
    print(contact.o2)
for cost_source in result.cost_sources:
    print(cost_source)

dis, result = fcl.distance(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
                                               transform.Transform(transform.Quaternion(), [10.0, 0.0, 0.0])),
                           fcl.CollisionObject(fcl.Sphere(4.0),
                                               transform.Transform(transform.Quaternion(), [-10.0, 0.0, 0.0])),
                           collision_data.DistanceRequest(True))

print("-- Distance result: ", dis)
print(result.nearest_points)

ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(0.1, 0.2, 0.0),
                                              transform.Transform(transform.Quaternion())),
                          fcl.CollisionObject(fcl.Box(1.0, 0.2, 0.0),
                                              transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0])),
                          collision_data.CollisionRequest())
print "-- Collision result: ", ret
for contact in result.contacts:
    print contact.normal
    print "sd: ", contact.penetration_depth
for cost_source in result.cost_sources:
    print cost_source

dis, result = fcl.distance(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
                                               transform.Transform(transform.Quaternion())),
                           fcl.CollisionObject(fcl.Sphere(4.0),
                                               transform.Transform(transform.Quaternion(), [0.0, 0.0, 0.0])),
                           collision_data.DistanceRequest(True))

print "-- Distance result: ", dis
print result.nearest_points
print result.min_distance

model = fcl.BVHModel()
print model
