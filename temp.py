from numpy import array, dot
import numpy as np


M = array([
    [1., 2., 0.],
    [-8., 3., 2.],
    [0., 1., 1.]])
P = dot(M.T, M)
q = dot(array([3., 2., 3.]), M).reshape((3,))
G = array([
    [1., 2., 1.],
    [2., 0., 1.],
    [-1., 2., -1.]])

print M


# M = np.append(M, [1, 1, 1], axis = 1)
# np.vstack([M, 1])
# print M

import numpy as np
from scipy import sparse

X = sparse.rand(100, 10000)
xt = np.random.random((100, 1))
xt = xt.astype('object') # Comment this to fix the error
print 'X:', X.shape, X.dtype
print 'xt:', xt.shape, xt.dtype
print 'Stacked shape:', sparse.hstack((X,xt)).shape