"""
This module contains functions for constructing trackers with various
parameter settings that seem to work well in typical situations.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from BakerMatthewsICTracker import BakerMatthewsICTracker
from CascadeTracker import CascadeTracker
from ESMTracker import ESMTracker
from Homography import random_homography
from NNTracker import NNTracker

_coarse_warps = lambda: random_homography(0.07, 0.06)
_fine_warps = lambda: random_homography(0.005, 0.0001)
_finer_warps = lambda: random_homography(0.0001, 0.0001)


def make_pure_nn(use_scv=False, res=(40,40)):
    coarse_tracker = NNTracker(8000, 1, res, _coarse_warps, use_scv)
    fine_tracker = NNTracker(5000, 1, res, _fine_warps, use_scv)
    finer_tracker = NNTracker(2000, 1, res, _finer_warps, use_scv)
    
    tracker = CascadeTracker([coarse_tracker, fine_tracker, finer_tracker])
    return tracker

def make_nn_GN(use_scv=False, res=(40,40)):
    coarse_tracker = NNTracker(8000, 1, res, _coarse_warps, use_scv)
    fine_tracker = NNTracker(2000, 2, res, _fine_warps, use_scv)
    finer_tracker = BakerMatthewsICTracker(15, 0.001, res, use_scv)
    return CascadeTracker([coarse_tracker, fine_tracker, finer_tracker])

def make_nn_esm(use_scv=False, res=(40,40)):
    coarse_tracker = NNTracker(8000, 1, res, _coarse_warps, use_scv)
    fine_tracker = ESMTracker(15, 0.01, res, use_scv)
    tracker = CascadeTracker([coarse_tracker, fine_tracker])
    return tracker

def make_esm(use_scv=False, res=(40,40)):
    return ESMTracker(20, 0.01, res, use_scv)
