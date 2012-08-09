""" 
Implementation of the Baker+Matthews Inverse Compositional Tracker

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import cv2

cimport numpy as np
import numpy as np

from utility import apply_to_pts, square_to_corners_warp
from utility cimport *

cdef class BMICTracker:

    cdef:
        int max_iters
        double threshold
        int resx
        int resy
        np.ndarray template, H_inv, J
        double[:,:] current_warp
        bint initialized

    def __init__(self, int max_iters, double threshold, int resx, int resy):
        self.max_iters = max_iters
        self.threshold = threshold
        self.resx = resx
        self.resy = resy
        self.initialized = False

    cpdef initialize(self, double[:,:] img, double[:,:] region_corners):
        self.current_warp = square_to_corners_warp(np.asarray(region_corners))
        self.template = np.asarray(sample_pts(img, self.resx, self.resy, self.current_warp))
        self.J = np.asmatrix(sample_pts_grad_sl3(img, self.resx, self.resy, self.current_warp))
        self.H_inv = (self.J.T * self.J).I
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
        cdef double[:,:] Jpc
        cdef double[:] sampled_img
        for i in range(self.max_iters):
            sampled_img = sample_pts(img, self.resx, self.resy, self.current_warp)
            error = np.asarray(self.template - sampled_img).reshape(-1,1)
            update = self.J.T * error
            update = self.H_inv * update
            update = np.asarray(update).squeeze()
            self.current_warp = mat_mul(self.current_warp, make_hom_sl3(update))
            normalize_hom(self.current_warp)
            if np.sum(np.abs(update)) < self.threshold: break

    cpdef is_initialized(self):
        return self.initialized

    cpdef set_warp(self, double[:,:] warp):
        self.current_warp = warp

    cpdef double[:,:] get_warp(self):
        return np.asmatrix(self.current_warp)

    cpdef set_region(self, double[:,:] corners):
        self.proposal = square_to_corners_warp(corners)

    cpdef get_region(self):
        return apply_to_pts(self.get_warp(), np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)
        
