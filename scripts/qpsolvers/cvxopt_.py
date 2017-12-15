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

from cvxopt import matrix, spmatrix
from cvxopt.solvers import options, qp
from numpy import array, ndarray
import numpy as np
import warnings as warning

options['show_progress'] = False  # disable cvxopt output


def cvxopt_matrix(M):
    if type(M) is ndarray:
        return matrix(M)
    elif type(M) is spmatrix or type(M) is matrix:
        return M
    coo = M.tocoo()
    return spmatrix(
        coo.data.tolist(), coo.row.tolist(), coo.col.tolist(), size=M.shape)


def cvxopt_solve_qp(P, q, G=None, h=None, A=None, b=None, solver=None,
                    initvals=None):
    """
    Solve a Quadratic Program defined as:

        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            G * x <= h
            A * x == b

    using CVXOPT <http://cvxopt.org/>.

    Parameters
    ----------
    P : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Symmetric quadratic-cost matrix.
    q : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Quadratic-cost vector.
    G : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear inequality matrix.
    h : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear inequality vector.
    A : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear equality constraint matrix.
    b : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear equality constraint vector.
    solver : string, optional
        Set to 'mosek' to run MOSEK rather than CVXOPT.
    initvals : numpy.array, optional
        Warm-start guess vector.

    Returns
    -------
    x : array, shape=(n,)
        Solution to the QP, if found, otherwise ``None``.

    Note
    ----
    CVXOPT only considers the lower entries of `P`, therefore it will use a
    wrong cost function if a non-symmetric matrix is provided.
    """
    args = [cvxopt_matrix(P), cvxopt_matrix(q)]
    if G is not None:
        args.extend([cvxopt_matrix(G), cvxopt_matrix(h)])
        if A is not None:
            args.extend([cvxopt_matrix(A), cvxopt_matrix(b)])
    sol = qp(*args, solver=solver, initvals=initvals)
    if 'optimal' not in sol['status']:
        return None
    return array(sol['x']).reshape((q.shape[0],))

def cvxopt_solve_qp1(P, q, G=None, lb=None, ub=None, lbG=None, ubG=None, A=None, b=None,
                    initvals=None, solver=None):
    """
    Solve a Quadratic Program defined as:

        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            G * x <= h
            A * x == b

    using CVXOPT <http://cvxopt.org/>.

    Parameters
    ----------
    P : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Symmetric quadratic-cost matrix.
    q : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Quadratic-cost vector.
    G : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear inequality matrix.
    h : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear inequality vector.
    A : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear equality constraint matrix.
    b : numpy.array, cvxopt.matrix or cvxopt.spmatrix
        Linear equality constraint vector.
    solver : string, optional
        Set to 'mosek' to run MOSEK rather than CVXOPT.
    initvals : numpy.array, optional
        Warm-start guess vector.

    Returns
    -------
    x : array, shape=(n,)
        Solution to the QP, if found, otherwise ``None``.

    Note
    ----
    CVXOPT only considers the lower entries of `P`, therefore it will use a
    wrong cost function if a non-symmetric matrix is provided.
    """
    # q = np.transpose(q)
    # h = np.transpose(ubA)
    # C = G
    # # lbA = np.hstack([lbA, b, lb])
    # # ubA = np.hstack([ubA, b, ub])
    # # C = np.vstack([G, A])
    # lb_C = np.hstack([lbA, b])
    # ub_C = np.hstack([ubA, b])
    # display(P, q, C, lbA, ubA)
    #
    #
    #
    #
    # lb = np.array([-0.3, -0.3, -0.3])
    # ub = np.array([1.1, 1.1, 1.1])
    # lbA = np.array([-0.3, -0.3, 0.2, 0.7])
    # ubA = np.array([0.3, 0.3, 0.2, 0.7])
    # q = np.transpose(q)
    __infty = 1e10

    if G is not None and A is None and lbG is None:
        C = G
        lb_C = None  # NB:
        ub_C = ubG

    elif G is None and A is not None:
        C = np.vstack([A, A])
        lb_C = ubG
        ub_C = ubG
    elif G is not None and A is not None and lbG is None:

        C = np.vstack([G, A, A])
        lb_C = np.hstack([-__infty * np.ones(lb.shape[0]), b])
        ub_C = np.hstack([ubG, b, b])
    elif G is not None and A is None and lbG is not None and ubG is not None:
        C = G
        C = np.hstack([G, A])

        lb_C = np.hstack([lbG, b])
        ub_C = np.hstack([ubG, b])
    elif G is not None and A is not None and lbG is not None and ubG is not None:
        # C = -G
        C = np.vstack([-G, G])
        print C.shape
        # lb_C = np.hstack([-lb, -lb])
        # ub_C = np.hstack([-ub, ub])
        lb_C = lb
        ub_C = ub
        print lb_C.shape

        h = np.vstack([-lb_C, ub_C]).flatten()
        print h
        # h = np.hstack([h, -lb.flatten(), ub.flatten()])
        # C = np.vstack([C, -np.identity(lb.shape[1]), np.identity(ub.shape[1])])



        # ub_C = np.hstack([-lbG, ubG, -b, b]).flatten()

    # print np.identity(lb.shape[1]), np.identity(ub.shape[1]),
    # print C.shape
    # print lb_C
    # print ub_C
    # print h.shape
    # print lb.shape[1]
    # print ub
    # display(P, q, C, lb_C, h)
    #
    q = np.transpose(q)
    h = np.transpose(h)

    P = .5 * (P + P.transpose())

    display(P, q, C, lb_C, ub_C)
    args = [cvxopt_matrix(P), cvxopt_matrix(q)]
    if C is not None:
        args.extend([cvxopt_matrix(C), cvxopt_matrix(h)])
        # if A is not None:
            # print "test"
            # args.extend([cvxopt_matrix(A), cvxopt_matrix(b)])
    sol = qp(*args, solver=solver, initvals=initvals)
    print sol['x']
    # print sol['status']
    # print zip(sol['x'])
    if 'optimal' not in sol['status']:
        return None
    return array(sol['x']).reshape((q.shape[0],))


def display(P, q, A, lbG, ubG, max_wsr=10):
    print "P"
    print P
    print "q"
    print q
    print "C"
    print A
    # print "lb"
    # print lb
    # print "ub"
    # print ub
    print "lbC"
    print lbG
    print "ubC"
    print ubG
    print "max_wsr"
    print max_wsr