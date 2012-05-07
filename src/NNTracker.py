import itertools

import numpy as np
from scipy import weave
from scipy.weave import converters

from Homography import *
from ImageUtils import *

def random_warp(distortion_sigma=0.02, translation_sigma=0.04):
    return random_homography(distortion_sigma, translation_sigma)

class NNTracker:

    def __init__(self, n_samples, n_iterations, sigma=0.1):
        self.n_samples = n_samples
        self.n_iterations = n_iterations
        self.sigma = sigma
    
    def initialize(self, img, ul, lr, res):
        self.pts = np.array(list(itertools.product(np.linspace(-.5, .5, res[0]), 
                                                   np.linspace(-.5, .5, res[1])))).T
        scalex = lr[0] - ul[0]
        scaley = lr[1] - ul[1]
        tx = ul[0] + scalex*0.5
        ty = ul[1] + scaley*0.5
        self.warp = np.matrix([[scalex, 0     , tx],
                               [0     , scaley, ty],
                               [0     , 0     , 1 ]])
        
        self.warps = [np.asmatrix(np.eye(3))] + [random_warp() for i in xrange(self.n_samples-1)]
        self.images = np.empty((np.prod(res), self.n_samples))
        for i,w in enumerate(self.warps):
            self.images[:,i] = sample_region(img, apply_to_pts(self.warp*w.I, self.pts))

    def update(self, img):
        for i in xrange(self.n_iterations):
            warped_pts = apply_to_pts(self.warp, self.pts)
            sampled_img = sample_region(img, warped_pts).reshape(-1,1)
            i = best_match(self.images, sampled_img)
            self.warp = self.warp * self.warps[i]
            self.warp /= self.warp[2,2]

    def get_corners(self):
        return apply_to_pts(self.warp, np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)

def best_match(images, reference):
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
