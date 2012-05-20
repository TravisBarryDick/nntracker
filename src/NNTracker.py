import itertools
import random

import numpy as np
from scipy import weave
from scipy.weave import converters

from CameraMotionHomography import *
from Homography import *
from ImageUtils import *

def warp_generator():
    if random.random() < 0.5:
        return random_camera_homography()
    else:
        return random_homography(0.02, 0.02)

class NNTracker:

    def __init__(self, n_samples, n_iterations=1, res=(16,16),
                 n_proposals=1, proposal_recycle_rate=0,
                 warp_generator=lambda: warp_generator()):
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

    def initialize(self, img, region):
        self.pts = np.array(list(itertools.product(np.linspace(-.5, .5, self.res[0]), 
                                                   np.linspace(-.5, .5, self.res[1])))).T
        self.set_region(region)
        self.template = sample_region(img, apply_to_pts(self.get_warp(), self.pts))
        self.ref_warps = [np.asmatrix(np.eye(3))] + [self.warp_generator() for i in xrange(self.n_samples-1)]
        self.ref_images = np.empty((self.n_points, self.n_samples))
        for i,w in enumerate(self.ref_warps):
            self.ref_images[:,i] = sample_region(img, apply_to_pts(self.get_warp() * w.I, self.pts))

    def initialize_with_rectangle(self, img, ul, lr):
        corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
        self.initialize(img, corners)

    def update(self, img):
        losses = np.empty(self.n_proposals)
        for i in xrange(self.n_proposals):
            count = 0
            best_ref = -1
            while best_ref != 0 and count < self.n_iterations:
                warped_pts = apply_to_pts(self.proposals[i], self.pts)
                sampled_img = sample_region(img, warped_pts).reshape(-1,1)
                best_ref = self.best_match(sampled_img)
                self.proposals[i] = self.proposals[i] * self.ref_warps[best_ref]
                self.proposals[i] /= self.proposals[i][2,2]
                count += 1
            losses[i] = np.sum(np.abs(sample_region(img, apply_to_pts(self.proposals[i], self.pts)) - self.template))
        order = np.argsort(losses)
        self.best_proposal = order[0]
        self.loss = losses[order[0]]
        for i in xrange(self.proposal_recycle_rate):
            self.proposals[order[-i-1]] = self.proposals[order[0]] * (self.warp_generator()*2)

    def best_match(self, reference): 
        images = self.ref_images
        (image_size, num_images) = images.shape
        code = \
        """
          int best_index = -1;
          double best_loss = std::numeric_limits<double>::max();
          for (int j = 0; j < num_images; j++) {
            double loss = 0;
            for (int i = 0; i < image_size; i++) {
              loss += std::abs(images(i,j) - reference(i));
            }
            if (loss < best_loss) {
              best_index = j;
              best_loss = loss;
            }
          }
          return_val = best_index;
        """
        return weave.inline(code, ['images', 'reference', 'image_size', 'num_images'],
                            headers=['<limits>', '<cmath>'],
                            type_converters=converters.blitz)

    def get_loss(self, img):
        return np.sum(np.abs(sample_region(img, apply_to_pts(self.get_warp(), self.pts)) 
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



