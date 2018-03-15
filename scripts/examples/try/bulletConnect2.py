import pybullet as p
import os

home = os.path.expanduser('~')
#
p.connect(p.SHARED_MEMORY, "localhost")
# p.connect(p.SHARED_MEMORY)
location_prefix = home + '/masterThesis/bullet3/data/'

urdf_file = location_prefix + "kuka_iiwa/model.urdf"
p.loadURDF(urdf_file)

print "loaded"
# print p.getNumBodies()