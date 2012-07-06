"""
Implementation of the Baker-Matthews Inverse Compositional Tracking Algorithm.

S. Baker and I. Matthews, "Equivalence and efficiency of image alignment algorithms", Computer Vision and Pattern Recognition, 2001. CVPR 2001. Proceedings of the 2001 IEEE Computer Society Conference on, vol. 1, pp. I-1090-I-1097 vol. 1, 2001.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import numpy as np

from Homography import *
from ImageUtils import *
from TrackerBase import *

class BakerMatthewsICTracker(TrackerBase):

    def __init__(self, max_iters, threshold=0.01, res=(20,20)):
        """ An implementation of the inverse composititionl tracker from Baker and Matthews

        Parameters:
        -----------
        max_iters : integer
          The maximum number of iterations per frame

        threshold : real
          If the decrease in error is smaller than the threshold then the per-frame
          update terminates.

        res : (integer, integer)
          The desired resolution of the template image. Higher values allow for more
          precise tracking but increase running time.
        
        See Also:
        ---------
        TrackerBase
        NNTracker
        """
        
        self.max_iters = max_iters
        self.res = res
        self.pts = res_to_pts(self.res)
        self.initialized = False

    def set_region(self, corners):
        self.proposal = square_to_corners_warp(corners)
        
    def initialize(self, img, region):
        n_pts = self.pts.shape[1]
        self.set_region(region)
        self.template = sample_region(img, self.pts, self.get_warp())
        GT = np.asmatrix(image_gradient(img, self.pts, self.get_warp()))
        self.JIT = np.empty((n_pts, 8))
        for i in xrange(n_pts):
            self.JIT[i,:] = GT[i,:] * _make_hom_jacobian(self.pts[:,i])
        self.JIT = np.asmatrix(self.JIT)
        self.initialized = True

    def update(self, img):
        if not self.is_initialized(): return None
        for i in xrange(self.max_iters):
            error_image = (sample_region(img, self.pts, self.get_warp()) - self.template).reshape((-1,1))
            update_parameters = np.linalg.lstsq(self.JIT, error_image)[0]
            self.proposal = self.proposal * _make_hom(update_parameters).I
            
    def is_initialized(self):
        return self.initialized

    def get_warp(self):
        return self.proposal

    def get_region(self):
        return apply_to_pts(self.get_warp(), np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T)


def _make_hom(p):
    return np.matrix([[1 + p[0],   p[1]  , p[2]],
                      [  p[3]  , 1 + p[4], p[5]],
                      [  p[6]  ,   p[7]  ,   1]], dtype=np.float64)

def _make_hom_jacobian(pt, p=np.zeros(8)):
    x = pt[0]
    y = pt[1]
    jacobian = np.zeros((2,8))
    d = (p[6]*x + p[7]*y + 1)
    d2 = d*d
    jacobian[0,0] = x / d
    jacobian[0,1] = y / d
    jacobian[0,2] = 1 / d
    jacobian[1,3] = x / d
    jacobian[1,4] = y / d
    jacobian[1,5] = 1 / d
    jacobian[0,6] = -x * (p[0]*x + x + p[1]*y + p[2]) / d2
    jacobian[0,7] = -y * (p[0]*x + x + p[1]*y + p[2]) / d2
    jacobian[1,6] = -x * (p[3]*x + p[4]*y + y + p[5]) / d2
    jacobian[1,7] = -y * (p[3]*x + p[4]*y + y + p[5]) / d2
    return np.asmatrix(jacobian)
