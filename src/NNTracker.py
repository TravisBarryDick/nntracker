import itertools
import random

import numpy as np
import pyflann
from scipy import weave
from scipy.weave import converters

from CameraMotionHomography import *
from Homography import *
from ImageUtils import *

class WarpIndex:
    def __init__(self, n_samples, warp_generator):
        self.n_samples = n_samples
        self.warp_generator = warp_generator

    def build_index(self, img, pts, initial_warp):
        n_points = pts.shape[1]
        print "Sampling Warps..."
        self.warps = [np.asmatrix(np.eye(3))] + [self.warp_generator() for i in xrange(self.n_samples - 1)]
        print "Sampling Images..."
        self.images = np.empty((n_points, self.n_samples))
        for i,w in enumerate(self.warps):
            self.images[:,i] = sample_region(img, apply_to_pts(initial_warp * w.I, pts))
        print "Building FLANN Index..."
        pyflann.set_distance_type("manhattan")
        self.flann = pyflann.FLANN()
        self.flann.build_index(self.images.T, algorithm='kdtree', trees=10)
        print "Done!"

    def best_index_weave(self, img): # This is much slower than the FLANN function
        images = self.images
        (image_size, num_images) = images.shape
        code = \
            """
            int best_index = -1;
            double best_loss = std::numeric_limits<double>::max();
            for (int j = 0; j < num_images; j++) {
              double loss = 0;
                for (int i = 0; i < image_size; i++) {
                  loss += std::abs(images(i,j) - img(i));
                }
              if (loss < best_loss) {
            best_index = j;
            best_loss = loss;
          }
        }
        return_val = best_index;
        """
        best_index = weave.inline(code, ['images', 'img', 'image_size', 'num_images'],
                                  headers = ['<limits>', '<cmath>'],
                                  type_converters=converters.blitz,
                                  compiler='gcc')
        return self.warps[best_index]

    def best_match(self, img):
        results, dists = self.flann.nn_index(img)
        return self.warps[results[0]]

class NNTracker:

    def __init__(self, n_samples, n_iterations=1, res=(16,16),
                 n_proposals=1, proposal_recycle_rate=0,
                 warp_generator=lambda: random_homography(0.05, 0.04)):
        self.n_samples = n_samples
        self.n_iterations = n_iterations
        self.n_points = np.prod(res)
        self.res = res
        self.n_proposals = n_proposals
        self.proposal_recycle_rate = proposal_recycle_rate
        self.warp_generator = warp_generator
    
    def set_region(self, corners):
        warp = square_to_corners_warp(corners)
        self.proposals = [warp.copy() for i in xrange(self.n_proposals)]
        self.best_proposal = 0
        
    def set_region_with_rectangle(self, ul, lr):
        corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
        self.set_region(corners)

    def initialize(self, img, region):
        self.pts = np.array(list(itertools.product(np.linspace(-.5, .5, self.res[0]), 
                                                   np.linspace(-.5, .5, self.res[1])))).T
        self.set_region(region)
        print "Main warp index:"
        self.template = sample_region(img, apply_to_pts(self.get_warp(), self.pts))
        self.warp_index = WarpIndex(self.n_samples, self.warp_generator)
        self.warp_index.build_index(img, self.pts, self.get_warp())
        print "Small warp index:"
        self.small_warp_index = WarpIndex(2000, lambda: random_homography(0.002, 0.0002))
        self.small_warp_index.build_index(img, self.pts, self.get_warp())

    def initialize_with_rectangle(self, img, ul, lr):
        corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
        self.initialize(img, corners)

    def update_proposal(self, img, warp, warp_index):
        warped_pts = apply_to_pts(warp, self.pts)
        sampled_img = sample_region(img, warped_pts)
        updated_warp = warp * warp_index.best_match(sampled_img)
        updated_warp /= updated_warp[2,2]
        return updated_warp

    def update(self, img):
        losses = np.empty(self.n_proposals)
        for i in xrange(self.n_proposals):
            for j in xrange(self.n_iterations):
                self.proposals[i] = self.update_proposal(img, self.proposals[i], self.warp_index)
            losses[i] = np.sum(np.abs(sample_region(img, apply_to_pts(self.proposals[i], self.pts)) - self.template))
        order = np.argsort(losses)
        self.best_proposal = order[0]
        self.loss = losses[order[0]]

        for i in xrange(6):
            self.proposals[order[0]] = self.update_proposal(img, self.proposals[order[0]], self.small_warp_index)

        for i in xrange(self.proposal_recycle_rate):
            self.proposals[order[-i-1]] = self.proposals[order[0]] * self.warp_generator()

    def get_loss(self, img, warp = None):
        if warp == None: warp = self.get_warp()
        return np.sum(np.abs(sample_region(img, apply_to_pts(warp, self.pts)) 
                             - self.template))

    def get_warp(self, proposal=None):
        if proposal == None: proposal = self.best_proposal
        return self.proposals[proposal]

    def get_corners(self, proposal=None):
        return apply_to_pts(self.get_warp(proposal), 
                            np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)

    def error_image(self, img):
        current = sample_region(img, apply_to_pts(self.get_warp(), self.pts))
        return np.abs(current - self.template).reshape(self.res)



