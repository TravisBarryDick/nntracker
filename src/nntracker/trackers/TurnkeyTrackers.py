"""
This module contains functions for constructing trackers with various
parameter settings that seem to work well in typical situations.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from BMICTracker import BMICTracker
from ESMTracker import ESMTracker
from NNTracker import NNTracker

from ThreadedCascadeTracker import ThreadedCascadeTracker
from CascadeTracker import CascadeTracker

def make_nn_GN(use_scv=False, res=(40,40), threaded=False):
    t1 = NNTracker(10, 1000, res[0], res[1], 0.06, 0.04, use_scv)
    t2 = NNTracker(10, 1000, res[0], res[1], 0.03, 0.02, use_scv)
    t3 = NNTracker(10, 1000, res[0], res[1], 0.015, 0.01, use_scv)
    t4 = BMICTracker(5, 0.001, res[0], res[1], use_scv)

    if threaded: return ThreadedCascadeTracker([t1, t2, t3, t4])
    else: return CascadeTracker([t1, t2, t3, t4], use_scv=use_scv)

def make_esm(use_scv=False, res=(40,40)):
    return ESMTracker(20, 0.01, res[0], res[1], use_scv)
