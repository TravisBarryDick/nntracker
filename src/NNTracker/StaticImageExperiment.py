"""
Implementation of the Static Image experiment found in Baker + Matthews 2004
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import math

import cv2
import numpy as np

from Homography import *
from ImageUtils import *

class StaticImageMotion:

    def __init__(self, image, target_region, sigmas, n_trials, max_spatial_sigma, single_template=False):

        self.image = image
        self.target_region = target_region
        self.sigmas = sigmas
        self.n_trials = n_trials
        self.max_spatial_sigma = max_spatial_sigma
        self.single_template = single_template

    def run(self, tracker):
        if self.single_template:
            tracker.initialize(self.image, self.target_region)
            initial_warp = tracker.get_warp()

        results = []
        for sigma in self.sigmas:
            n_converged = 0
            for i in xrange(self.n_trials):
                point_noise = np.random.normal(0, sigma, (2,4))
                disturbed_region = self.target_region + point_noise

                if not self.single_template:
                    tracker.initialize(self.image, disturbed_region)
                    tracker.set_warp(initial_warp)
                else:
                    tracker.set_region(disturbed_region)

                tracker.update(self.image)

                if self.single_template:
                    true_region = self.target_region
                else:
                    true_region = disturbed_region

                if _point_rmse(true_region, tracker.get_region()) < self.max_spatial_sigma:
                    n_converged += 1

            convergence_rate = n_converged / float(self.n_trials)
            print "Sigma %g : %g" % (sigma, convergence_rate)
            results.append(convergence_rate)
        return results

def _point_rmse(a,b):
    return  math.sqrt(np.sum((a-b)**2)/a.shape[1])
