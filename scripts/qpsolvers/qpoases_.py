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

from numpy import array, hstack, ones, vstack, zeros, full
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel
from qpoases import PyQProblem as QProblem
from qpoases import PyQProblemB as QProblemB
from qpoases import PyReturnValue as ReturnValue
from warnings import warn


__infty = 1e10
options = Options()
options.setToDefault()

options.printLevel = PrintLevel.LOW

def display(P, q, C, lb, ub, lb_C, ub_C, max_wsr):
    print "P"
    print P
    print "q"
    print q
    print "C"
    print C
    print "lb"
    print lb
    print "ub"
    print ub
    print "lb_C"
    print lb_C
    print "ub_C"
    print ub_C
    print "max_wsr"
    print max_wsr


def qpoases_solve_qp(P, q, G=None, lb=None, ub=None, lbG=None, ubG=None, A=None, b=None, initvals=None,
                     max_wsr=1000):
    """
    Solve a Quadratic Program defined as:

        minimize
            (1/2) * x.T * P * x + q.T * x

        subject to
            lbG <= G * x <= ubG
            lb <= x <= ub
            A * x == b



    using qpOASES <https://projects.coin-or.org/qpOASES>.

    Parameters
    ----------
    P : numpy.array
        Symmetric quadratic-cost matrix.
    q : numpy.array
        Quadratic-cost vector.
    G : numpy.array
        Linear inequality constraint matrix.
    ubG : numpy.array
        Linear inequality constraint vector.
    A : numpy.array, optional
        Linear equality constraint matrix.
    b : numpy.array, optional
        Linear equality constraint vector.
    initvals : numpy.array, optional
        Warm-start guess vector.
    max_wsr : integer, optional
        Maximum number of Working-Set Recalculations given to qpOASES.

    Returns
    -------
    x : numpy.array
        Solution to the QP, if found, otherwise ``None``.

    Note
    ----
    This function relies on some updates from the standard distribution of
    qpOASES (details below). A fully compatible repository is published at
    <https://github.com/stephane-caron/qpOASES>.

    Note
    ----
    This function allows empty bounds (lb, ub, lbA or ubA). This was provisioned
    by the C++ API but not by the Python API of qpOASES (as of version 3.2.0).
    Be sure to update the Cython file (qpoases.pyx) to convert ``None`` to the
    null pointer.
    """
    if initvals is not None:
        warn("warm-start values ignored by qpOASES wrapper")
    n = P.shape[0]
    # lb, ub = None, None
    # lb = full((1, n), lb)
    # ub = full((1, n), ub)

    has_cons = G is not None or A is not None
    if G is not None and A is None and lbG is None:
        C = G
        lb_C = None  # NB:
        ub_C = ubG

    elif G is None and A is not None:
        C = vstack([A, A])
        lb_C = ubG
        ub_C = ubG
    elif G is not None and A is not None and lbG is None :

        C = vstack([G, A, A])
        lb_C = hstack([-__infty * ones(h.shape[0]), b])
        ub_C = hstack([ubG, b, b])
    elif G is not None and A is None and lbG is not None and ubG is not None:
        C = G
        # C = hstack([G, A])

        lb_C = hstack([lbG, b])
        ub_C = hstack([ubG, b])
    elif G is not None and A is not None and lbG is not None and ubG is not None:
        C = G
        # C = vstack([G, A])
        lb_C = hstack([lbG, b])
        ub_C = hstack([ubG, b])
    if has_cons:
        qp = QProblem(n, C.shape[0])
        qp.setOptions(options)
        display(P, q.flatten(), C, lb.flatten(), ub.flatten(), lb_C.flatten(), ub_C.flatten(), max_wsr)
        return_value = qp.init(P, q.flatten(), C, lb.flatten(), ub.flatten(), lb_C.flatten(), ub_C.flatten(), array([max_wsr]))
        # print("init status", return_value)
        status = qp.hotstart(q.flatten(), lb.flatten(), ub.flatten(), lb_C.flatten(), ub_C.flatten(), array([max_wsr]))
        # print("hotstart status", status)
        if return_value == ReturnValue.MAX_NWSR_REACHED:
            warn("qpOASES reached the maximum number of WSR (%d)" % max_wsr)
    else:
        qp = QProblemB(n)
        qp.setOptions(options)
        qp.init(P, q, lb, ub, max_wsr)
    x_opt = zeros(n)
    ret = qp.getPrimalSolution(x_opt)
    if ret != 0:  # 0 == SUCCESSFUL_RETURN code of qpOASES
        warn("qpOASES failed with return code %d" % ret)
    # return x_opt
    return qp