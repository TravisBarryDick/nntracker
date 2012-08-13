"""
Implementation of the Nearest Neighbour Tracking Algorithm.
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import os
import shelve
import threading

cimport numpy as np
import numpy as np

import pyflann

from nntracker.utility import apply_to_pts, square_to_corners_warp
from nntracker.utility cimport *

_stored_warps_lock = threading.Lock()
_stored_warps = shelve.open(os.path.expanduser("~/warp_cache"))

cdef class _WarpIndex_Flann:
    cdef:
        double[:,:,:] warps
        double[:,:] images
        object flann

    def __init__(self, double[:,:] img, double[:,:] warp, 
                 int n_samples, int resx, int resy, 
                 double sigma_t, double sigma_d):

        # --- Sampling Warps --- #
        print "Sampling Warps..."
        warp_key = "%d %.5g %.5g" % (n_samples, sigma_t, sigma_d)
        print "Warp Key =", warp_key
        if not _stored_warps.has_key(warp_key):
            with _stored_warps_lock:
                warps = np.empty((n_samples,3,3), dtype=np.float64)
                for i in range(n_samples):
                    warps[i,:,:] = _random_homography(sigma_t, sigma_d)
                _stored_warps[warp_key] = warps
        self.warps = _stored_warps[warp_key]

         # --- Sampling Images --- #
        print "Sampling Images..."
        cdef int n_pts = resx * resy
        self.images = np.empty((n_pts, n_samples), dtype=np.float64)
        for i in range(n_samples):
            inv_warp = np.asmatrix(self.warps[i,:,:]).I
            self.images[:,i] = sample_pts(img, resx, resy, mat_mul(warp, inv_warp))

        # --- Building Flann Index --- #
        print "Building Flann Index..."
        self.flann = pyflann.FLANN()
        self.flann.build_index(np.asarray(self.images).T, algorithm='kdtree', trees=3)
        print "Done!"         

    cpdef best_match(self, img):
        results, dists = self.flann.nn_index(np.asarray(img))
        return self.warps[<int>results[0],:,:]

cdef class NNTracker:

    cdef:
        _WarpIndex_Flann warp_index
        int max_iters
        int resx, resy
        np.ndarray template
        int n_samples
        double sigma_t, sigma_d
        double[:,:] current_warp
        double[:] intensity_map
        bint use_scv
        bint initialized

    def __init__(self, int max_iters, int n_samples, int resx, int resy, double sigma_t, 
                 double sigma_d, bint use_scv):
        self.max_iters = max_iters
        self.n_samples = n_samples
        self.resx = resx
        self.resy = resy
        self.sigma_t = sigma_t
        self.sigma_d = sigma_d
        self.use_scv = use_scv
        self.initialized = False

    cpdef initialize(self, double[:,:] img, double[:,:] region_corners):
        self.initialized = False
        self.current_warp = square_to_corners_warp(np.asarray(region_corners))
        self.template = np.asarray(sample_pts(img, self.resx, self.resy, self.current_warp))
        self.warp_index = _WarpIndex_Flann(img, self.current_warp,
                                           self.n_samples, self.resx, self.resy,
                                           self.sigma_t, self.sigma_d)
        if self.use_scv:
            self.intensity_map = np.arange(256, dtype=np.float64)
        self.initialized = True

    cpdef initialize_with_rectangle(self, double[:,:] img, ul, lr):
        cpdef double[:,:] region_corners = \
            np.array([[ul[0], ul[1]],
                      [lr[0], ul[1]],
                      [lr[0], lr[1]],
                      [ul[0], lr[1]]], dtype=np.float64).T
        self.initialize(img, region_corners)

    cpdef update(self, double[:,:] img):
        if not self.initialized: return
        cdef int i
        cdef double[:] sampled_img
        for i in range(self.max_iters):
            sampled_img = sample_pts(img, self.resx, self.resy, self.current_warp)
            if self.use_scv:
                if self.intensity_map == None: self.intensity_map = scv_intensity_map(sampled_img, self.template)
                sampled_img = scv_expected_img(sampled_img, self.intensity_map)
            update = self.warp_index.best_match(sampled_img)
            self.current_warp = mat_mul(self.current_warp, update)
            normalize_hom(self.current_warp)
        if self.use_scv:
            sampled_img = sample_pts(img, self.resx, self.resy, self.current_warp)
            self.intensity_map = scv_intensity_map(sampled_img, self.template)

    cpdef is_initialized(self):
        return self.initialized

    cpdef set_warp(self, double[:,:] warp):
        self.current_warp = warp
        self.intensity_map = None

    cpdef double[:,:] get_warp(self):
        return np.asmatrix(self.current_warp)

    cpdef set_region(self, double[:,:] corners):
        self.current_warp = square_to_corners_warp(corners)
        self.intensity_map = None

    cpdef get_region(self):
        return apply_to_pts(self.get_warp(), np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)

cdef double[:,:] _square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
cdef double[:,:] _random_homography(double sigma_t, double sigma_d):
    cdef double[:,:] disturbed = np.random.normal(0,sigma_d, (2,4)) + np.random.normal(0, sigma_t, (2,1)) + _square
    cdef double[:,:] H = compute_homography(_square, disturbed)
    return H
