"""
This module contains functions for constructing trackers with various
parameter settings that seem to work well in typical situations.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from BMICTracker import BMICTracker
from ESMTracker import ESMTracker
from NNTracker import NNTracker

from CascadeTracker import CascadeTracker
from Homography import random_homography

_coarse_warps = lambda: random_homography(0.07, 0.06)
_fine_warps = lambda: random_homography(0.005, 0.0001)
_finer_warps = lambda: random_homography(0.0001, 0.0001)

def make_pure_nn(use_scv=False, res=(40,40)):
    t1 = NNTracker(1, 8000, res[0], res[1], 0.07, 0.06)
    t2 = NNTracker(1, 5000, res[0], res[1], 0.005, 0.0001)
    t3 = NNTracker(1, 2000, res[0], res[1], 0.0001, 0.0001)
    tracker = CascadeTracker([t1, t2, t3])
    return tracker

def make_nn_GN(use_scv=False, res=(40,40)):
    t1 = NNTracker(1, 8000, res[0], res[1], 0.07, 0.06)
    t2 = NNTracker(1, 5000, res[0], res[1], 0.005, 0.0001)
    t3 = BMICTracker(15, 0.001, res[0], res[1])
    return CascadeTracker([t1, t2, t3])

def make_nn_esm(use_scv=False, res=(40,40)):
    t1 = NNTracker(1, 8000, res[0], res[1], 0.07, 0.06)
    t2 = ESMTracker(15, 0.01, res[0], res[1])
    tracker = CascadeTracker([t1, t2])
    return tracker

def make_esm(use_scv=False, res=(40,40)):
    return ESMTracker(20, 0.01, res[0], res[1])