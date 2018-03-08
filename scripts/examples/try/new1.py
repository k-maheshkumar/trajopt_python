import numpy as np


# D = np.array([1, 2])
#
# A =[]
# A.append(D)
# B = np.array([3, 4])
#
# C = []
# C.append(np.asarray(A))
# C.append(B)
# print "C", C
#
# # A *= 4
#
# print "C", C
# print "a", A
# A.append(np.asarray(B))
# print "C", C
# print "a", A

A = np.asarray([
    [0.,  0.,  0.,  0., - 1.,  1.,  0.,  0.,  0.,],
    [0.,  0.,  0.,  0., - 2.,  2.,  0.,  0.,  0.,],
    [0.,  0.,  0.,  0., - 3.,  3.,  0.,  0.,  0.,],
    [0.,  0.,  0.,  0., - 4.,  4.,  0.,  0.,  0.,],
    [0.,  0.,  0.,  0., - 5.,  5.,  0.,  0.,  0.,],
    [0.,  0.,  0.,  0., - 6.,  6.,  0.,  0.,  0.,],
              ])

# print A

# B = A[::2,-5:]
B = A[:,-5:]
# print B
E = B[:6:2,:]
print E

# F = B[-3:, :]
#
# # print F
#
# G = np.vstack([E, F])
# print G

# C = np.asarray([
#     [1., 2., 3., 4., 5., 6., 7., 8., 9., ],
#
# ]).reshape((9,))
#
# # print A
# D = C[::2]
# print D