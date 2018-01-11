from fcl import fcl, transform, collision_data

cube = fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0))
rot = [0, 0, 0, 1]
trans = [1, 1, 1]
cube.setTransform(rot, trans)

cylinder = fcl.CollisionObject(fcl.Cylinder(1, 2))
rot1 = [0, 0, 0, 1]
trans1 = [5, 0, 0]
cylinder.setTransform(rot1, trans1)

cylinder1 = fcl.CollisionObject(fcl.Cylinder(1, 2))
rot1 = [0, 0, 0, 1]
trans1 = [-5, -3, 2.2]
cylinder1.setTransform(rot1, trans1)

manager = fcl.DynamicAABBTreeCollisionManager()
manager.registerObject(cube)
manager.registerObject(cylinder)
# manager.registerObject(cylinder1)
print("After register 1 : ", manager.size())
# print "cube", cube.getTranslation(), cube.getNodeType()
# print "cylinder", cylinder.getTranslation(), cylinder.getNodeType()
# print "cylinder1", cylinder1.getTranslation(), cylinder1.getNodeType()

# Collision calculation
ret, result = fcl.collide(cube, cylinder1, collision_data.CollisionRequest())
print("-- Collision result: ", ret)
# for contact in result.contacts:
#     print(contact.o1)
#     print(contact.o2)
# coll = collision_data.CollisionResult()
# coll.contacts
# print "num contacts normal", result.contacts.normal
# print "num contacts depth", result.contacts.penetration_depth

for contact in result.contacts:
    print "normal", contact.normal
    print "depth", contact.penetration_depth

dis, result = fcl.distance(cube, cylinder1, collision_data.DistanceRequest(True))
print("-- Distance result: ", dis)
print(result.nearest_points)
# print(result.normal)
print(result.min_distance)
print(result.o1)
print(result.o2)


# disr = collision_data.DistanceResult()
# disr.
# for cost_source in result.cost_sources:
#     print("cost_source", cost_source)


# # Register objects to DynamicAABBTreeCollisionManager
# manager = fcl.DynamicAABBTreeCollisionManager()
# print("Before register: ", manager.size())
# manager.registerObjects(objs)
# print("After register 1 : ", manager.size())
# manager.registerObject(fcl.CollisionObject(fcl.Cylinder(7.0, 8.0)))
# print("After register 2 : ", manager.size())
#
# # Use Callback function
# def cb_func(obj1, obj2, res):
#     print("cb_func start")
#     ret, res = fcl.collide(obj1, obj2, collision_data.CollisionRequest())
#     print("result: ", ret)
#     return ret
# res = collision_data.CollisionResult()
# manager.collide(res, cb_func)
#
# # Collision calculation
# ret, result = fcl.collide(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
#                                               transform.Transform(transform.Quaternion(), [10.0, 0.0, 0.0])),
#                           fcl.CollisionObject(fcl.Sphere(4.0),
#                                               transform.Transform(transform.Quaternion(), [-10.0, 0.0, 0.0])),
#                           collision_data.CollisionRequest())
#
# print("-- Collision result: ", ret)
# for contact in result.contacts:
#     print(contact.o1)
#     print(contact.o2)
# for cost_source in result.cost_sources:
#     print(cost_source)
#
# dis, result = fcl.distance(fcl.CollisionObject(fcl.Box(1.0, 2.0, 3.0),
#                                                transform.Transform(transform.Quaternion(), [10.0, 0.0, 0.0])),
#                            fcl.CollisionObject(fcl.Sphere(4.0),
#                                                transform.Transform(transform.Quaternion(), [-10.0, 0.0, 0.0])),
#                            collision_data.DistanceRequest(True))
#
# print("-- Distance result: ", dis)
# print(result.nearest_points)
# print result