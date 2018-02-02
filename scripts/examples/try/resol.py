import numpy as np


planning_samples = 8
joints =  2
group = [1] * joints

mat = np.zeros((5, planning_samples * joints))
i, j = np.indices(mat.shape)

link_index = 1
time_step_count = 2
# mat[i == j - (((time_step_count - 1) * planning_samples) + link_index)] = -1
# mat[i == j - ((link_index + (time_step_count ) * planning_samples))] = 1
# # A[2,:] = 0
# # mat = np.split(mat, planning_samples)
# print mat


# mat[i == j - (((time_step_count - 1) * planning_samples) + link_index)] = -1
# mat[i == i * time_step_count] = 1
# for i in range(5):
#     # print (i, (link_index * (time_step_count - i)) + planning_samples)
#     mat[i, (link_index + (i * planning_samples)) - 8] = -(-1) ** i
# print mat

# for i in range(1, 3):
#     mat1 = [0] * (link_index - 1) + [1] + [0] * (joints - link_index)
#     mat2 = [0] * (link_index - 1) + [-1] + [0] * (joints - link_index)
#     mat = [[0] * joints] * (time_step_count-(i+1)) + [mat2] + [mat1] + [[0] * joints] * ((planning_samples* joints) - (time_step_count) + (i-1))
#
# print (mat)

#
# mat[i == j - (((time_step_count - 1) * planning_samples) + link_index)] = -1
# mat[i == j - ((link_index + (time_step_count ) * planning_samples))] = 1
# # A[2,:] = 0
# # mat = np.split(mat, planning_samples)
# print mat
# for k in range(planning_samples):
#     mat[i, ((time_step_count * k) + link_index * (k) + planning_samples)] = -(-1) ** k

# mat = [0] * link_index + [-1] + [0] * (len(group) - link_index - 1)
# mat += [0] * link_index + [1] + [0] * (len(group) - link_index - 1)
#
# # mat = mat + [0] * (len(group) - len(mat))
# # mat = [0] * (len(group) * time_step_count-1)+ mat
# # mat = [0] * (len(group) * planning_samples - len(mat)) + mat
# print mat
# print np.asarray(mat).shape

# mat_ = []
# for k in range(5):
#
#     mat[k, (len(group) * (time_step_count - (k+1))) + ((time_step_count-2)) + link_index] = -1
#     mat[k, (len(group) * (time_step_count - (k + 1))) + (len(group)) + link_index] = 1
#
#     mat_.append(mat.tolist())
#     # print mat
#
#     mat[:,:] = 0
#
#
# print np.vstack(mat_)

velocity_matrix = np.zeros((len(group) * planning_samples, planning_samples * len(group)))
np.fill_diagonal(velocity_matrix, -1.0)
i, j = np.indices(velocity_matrix.shape)
velocity_matrix[i == j - len(group)] = 1.0

# to slice zero last row
velocity_matrix.resize(velocity_matrix.shape[0] - len(group), velocity_matrix.shape[1])

# print velocity_matrix

mat = velocity_matrix[((time_step_count - 2) * len(group)):, :]
# print mat

mat = mat[link_index::(len(group)), :]
# print mat

mat = mat[:5:, :]
print mat

# A = np.asarray([
#     [0., 0., 0., 0., - 1., 1., 0., 0., 0., ],
#     [0., 0., 0., 0., - 2., 2., 0., 0., 0., ],
#     [0., 0., 0., 0., - 3., 3., 0., 0., 0., ],
#     [0., 0., 0., 0., - 4., 4., 0., 0., 0., ],
#     [0., 0., 0., 0., - 5., 5., 0., 0., 0., ],
#     [0., 0., 0., 0., - 6., 6., 0., 0., 0., ],
# ])
# i, j = np.indices(A.shape)
# E = A[:5:, :]
# print E