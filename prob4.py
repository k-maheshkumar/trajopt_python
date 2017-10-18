#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Copyright (C) 2016-2017 Stephane Caron <stephane.caron@normalesup.org>
#
# This file is part of qpsolvers.
#
# qpsolvers is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# qpsolvers is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# qpsolvers. If not, see <http://www.gnu.org/licenses/>.

import sys

from numpy import array, dot
from numpy.linalg import norm
from os.path import basename, dirname, realpath
from scipy.sparse import csc_matrix

try:
    from qpsolvers import dense_solvers, sparse_solvers
    from qpsolvers import solve_qp
except ImportError:  # run locally if not installed
    sys.path.append(dirname(realpath(__file__)) + '/..')
    from qpsolvers import dense_solvers, sparse_solvers
    from qpsolvers import solve_qp


# QP matrices
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
h = array([3., 2., -2.]).reshape((3,))
P_csc = csc_matrix(P)
G_csc = csc_matrix(G)


if __name__ == "__main__":

    sol = solve_qp(P, q, G, h, solver="qpoases")
    sol1 = solve_qp(P_csc, q, G_csc, h, solver="osqp")

    print "qpoases"
    print sol
    print "osqp"

    print sol1


