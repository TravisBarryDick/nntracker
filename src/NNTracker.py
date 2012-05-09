import itertools

import numpy as np
from scipy import weave
from scipy.weave import converters

from Homography import *
from ImageUtils import *

class NNTracker:

    def __init__(self, n_samples, n_iterations=1, sigma_d=0.1, sigma_t=0.1, 
                 n_proposals=1, recycle_rate=0):
        self.n_samples = n_samples
        self.n_iterations = n_iterations
        self.sigma_d = sigma_d
        self.sigma_t = sigma_t
        self.n_proposals = n_proposals
        self.recycle_rate = recycle_rate
    
    def set_region(self, corners):
        square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
        warp = compute_homography(square, corners)
        self.proposals = [warp.copy() for i in xrange(self.n_proposals)]
        return warp

    def initialize(self, img, ul, lr, res):
        self.pts = np.array(list(itertools.product(np.linspace(-.5, .5, res[0]), 
                                                   np.linspace(-.5, .5, res[1])))).T
        corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
        initial_warp = self.set_region(corners)
        self.template = sample_region(img, apply_to_pts(initial_warp, self.pts))
        self.ref_warps = [np.asmatrix(np.eye(3))] + [random_homography(self.sigma_d, self.sigma_t)
                                                 for i in xrange(self.n_samples-1)]
        self.ref_images = np.empty((res[0]*res[1], self.n_samples))
        for i,w in enumerate(self.ref_warps):
            self.ref_images[:,i] = sample_region(img, apply_to_pts(initial_warp * w.I, self.pts))

    def update(self, img):
        # update each of the proposals and compute resulting loss:
        losses = np.empty(self.n_proposals)
        for i in xrange(self.n_proposals):
            for j in xrange(self.n_iterations):
                warped_pts = apply_to_pts(self.proposals[i], self.pts)
                sampled_img = sample_region(img, warped_pts).reshape(-1,1)
                best_ref = best_match(self.ref_images, sampled_img)
                self.proposals[i] = self.proposals[i] * self.ref_warps[best_ref]
                self.proposals[i] /= self.proposals[i][2,2]
            losses[i] = np.sum(np.abs(sample_region(img, apply_to_pts(self.proposals[i], self.pts)) - self.template))
        # Sort indices by their losses and select the best one
        order = np.argsort(losses)
        self.best_proposal = order[0]
        # Replace bad proposals with random ones:
        for i in xrange(self.recycle_rate):
            self.proposals[order[-i-1]] = self.proposals[order[0]] * random_homography(self.sigma_d, self.sigma_t)

    def get_corners(self, proposal=None):
        if proposal == None: proposal = self.best_proposal
        return apply_to_pts(self.proposals[proposal], np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)

def best_match(images, reference): # Try to write this as a GLSL shader
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
