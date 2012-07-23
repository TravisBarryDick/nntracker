"""
Adapation of the NN Tracker algorithm to use something like
the Sum of Conditional Variances (SCV) as opposed to the 
sum of absolute differences.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from math import floor

import numpy as np
from scipy import weave
from scipy.weave import converters

from ImageUtils import *
from NNTracker import NNTracker

def _get_intensity_map(src, dst):
    conditional_probability = np.zeros((256,256))
    intensity_map = np.arange(256, dtype=np.float64)
    n = len(src)
    code = \
    """
    for (int k = 0; k < n; k++) {
      int i = int(src(k));
      int j = int(dst(k));
      conditional_probability(i,j) += 1;
    }
    for (int i = 0; i < 256; i++) {
      double normalizer = 0;
      double total = 0;
      for (int j = 0; j < 256; j++) {
        total += j * conditional_probability(i,j);
        normalizer += conditional_probability(i,j);
      }
      if (normalizer > 0) {
        intensity_map(i) = total / normalizer;
      }
    }
    """
    weave.inline(code, ['conditional_probability', 'intensity_map', 'n', 'src', 'dst'],
                 type_converters=converters.blitz,
                 compiler='gcc')
    return intensity_map

def _expected_template(intensity_map, img):
    return intensity_map[np.floor(img).astype(np.int)]

class SCVNNTracker(NNTracker):
    """ An implementation of the Nearest Neighbour Tracker with the
    sum of conditional variances similarity measure. Interface is 
    identical to the class NNTracker. """

    def __init__(self, *args, **kwargs):
        NNTracker.__init__(self, *args, **kwargs)
        self.intensity_map = np.arange(256)

    def initialize(self, img, region):
        NNTracker.initialize(self, img, region)
        self.update_intensity_map(img)

    def update_intensity_map(self, img):
        sampled_img = sample_and_normalize(img, self.pts, warp=self.proposal)
        new_map = _get_intensity_map(sampled_img, self.template)
        self.intensity_map = self.intensity_map + 0.1*(new_map - self.intensity_map)

    def update(self, img):
        if not self.is_initialized(): return None
        self.update_intensity_map(img)
        for _ in xrange(self.n_iterations):
            sampled_img = sample_and_normalize(img, self.pts, warp=self.proposal)
            update = self.warp_index.best_match(_expected_template(self.intensity_map, sampled_img))
            self.proposal = self.proposal * update
            self.proposal /= self.proposal[2,2]
        return self.proposal

    
