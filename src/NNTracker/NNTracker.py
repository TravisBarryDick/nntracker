"""
Implementation of the Nearest Neighbour Tracking Algorithm.
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import itertools
import random

import numpy as np
import pyflann
from scipy import weave
from scipy.weave import converters

from Homography import *
from ImageUtils import *
from TrackerBase import *

class NNTracker(TrackerBase):

    def __init__(self, n_samples, n_iterations=1, res=(20,20),
                 warp_generator=lambda:random_homography(0.05, 0.1)):
        """ An implemetation of the Nearest Neighbour Tracker. 

        Parameters:
        -----------
        n_samples : integer
          The number of sample motions to generate. Higher values will improve tracking
          accuracy but increase running time.
        
        n_iterations : integer
          The number of times to update the tracker state per frame. Larger numbers
          may improve convergence but will increase running time.
        
        res : (integer, integer)
          The desired resolution of the template image. Higher values allow for more
          precise tracking but increase running time.

        warp_generator : () -> (3,3) numpy matrix.
          A function that randomly generates a homography. The distribution should
          roughly mimic the types of motions that you expect to observe in the 
          tracking sequence. random_homography seems to work well in most applications.
        """
        self.n_samples = n_samples
        self.n_iterations = n_iterations
        self.res = res
        self.warp_generator = warp_generator
        self.n_points = np.prod(res)
        self.initialized = False
        self.pts = np.array(list(itertools.product(np.linspace(-.5, .5, self.res[0]), 
                                                   np.linspace(-.5, .5, self.res[1])))).T
    
    def set_region(self, corners):
        self.proposal = square_to_corners_warp(corners)

    def initialize(self, img, region):
        self.set_region(region)
        self.template = sample_and_normalize(img, apply_to_pts(self.get_warp(), self.pts))
        self.warp_index = _WarpIndex(self.n_samples, self.warp_generator, img, self.pts, self.get_warp())
        self.initialized = True

    def is_initialized(self):
        return self.initialized

    def update(self, img):
        if not self.is_initialized(): return None
        for i in xrange(self.n_iterations):
            warped_pts = apply_to_pts(self.proposal, self.pts)
            sampled_img = sample_and_normalize(img, warped_pts)
            self.proposal = self.proposal * self.warp_index.best_match(sampled_img)
            self.proposal /= self.proposal[2,2]
        return self.proposal

    def get_warp(self):
        return self.proposal

    def get_region(self):
        return apply_to_pts(self.get_warp(), np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)

class _WarpIndex:
    """ Utility class for building and querying the set of reference images/warps. """
    def __init__(self, n_samples, warp_generator, img, pts, initial_warp):
        n_points = pts.shape[1]
        print "Sampling Warps..."
        self.warps = [np.asmatrix(np.eye(3))] + [warp_generator() for i in xrange(n_samples - 1)]
        print "Sampling Images..."
        self.images = np.empty((n_points, n_samples))
        for i,w in enumerate(self.warps):
            self.images[:,i] = sample_and_normalize(img, apply_to_pts(initial_warp * w.I, pts))
        print "Building FLANN Index..."
        pyflann.set_distance_type("manhattan")
        self.flann = pyflann.FLANN()
        self.flann.build_index(self.images.T, algorithm='kdtree', trees=10)
        print "Done!"

    def best_match(self, img):
        results, dists = self.flann.nn_index(img)
        return self.warps[results[0]]
