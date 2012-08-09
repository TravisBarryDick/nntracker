""" 
Paramaterization of the set of homography using the lie algebra sl(3)
associated to the special linear group SL(3). This has the advantage of
only producing homographie matrices with det(H) = 1.

For details, see

S. Benhimane and E. Malis, "Real-time image-based tracking of planes
using efficient second-order minimization," Intelligent Robots and Systems, 2004.
(IROS 2004). Proceedings. 2004 IEEE/RSJ International Conference on, vol. 1, 
pp. 943-948 vol. 1, 2004.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import numpy as np
from scipy.linalg import expm

from ImageUtils import sample_region
from WeaveWrapper import *

def make_hom_sl3(p):
    A = np.matrix([[p[0],     p[1]   , p[2]],
                   [p[3], p[4] - p[0], p[5]],
                   [p[6],     p[7]   , -p[4]]], dtype=np.float64)
    return np.asmatrix(expm(A))

class JacobianEstimator:

    def __init__(self, eps = 1e-8):
        self.eps = eps
        self.ows = []
        for i in xrange(8):
            p = np.zeros(8)
            p[i] = eps
            self.ows.append( (make_hom_sl3(-p), make_hom_sl3(p)) )

    def estimate(self, img, pts, initial_warp, result):
        if result == None: result = np.empty((pts.shape[1], 8))
        code = \
        """
        for (int i = 0; i < num_pts; i++) {
          double d = w1(2,0)*pts(0,i) + w1(2,1)*pts(1,i) + w1(2,2);
          double x1 = (w1(0,0)*pts(0,i) + w1(0,1)*pts(1,i) + w1(0,2)) / d;
          double y1 = (w1(1,0)*pts(0,i) + w1(1,1)*pts(1,i) + w1(1,2)) / d;

          d = w2(2,0)*pts(0,i) + w2(2,1)*pts(1,i) + w2(2,2);
          double x2 = (w2(0,0)*pts(0,i) + w2(0,1)*pts(1,i) + w2(0,2)) / d;
          double y2 = (w2(1,0)*pts(0,i) + w2(1,1)*pts(1,i) + w2(1,2)) / d;

          double f2 = bilinear_interp(img, width, height, x2, y2);
          double f1 = bilinear_interp(img, width, height, x1, y1);
          result(i,j) = (f2-f1) / (2*eps);
        }
        """
        for j in xrange(8):
            vars = {
                "w1"      : np.array(initial_warp * self.ows[j][0]),
                "w2"      : np.array(initial_warp * self.ows[j][1]),
                "pts"     : pts,
                "num_pts" : pts.shape[1],
                "img"     : img,
                "height"  : img.shape[0],
                "width"   : img.shape[1],
                "result"  : result,
                "eps"     : self.eps,
                "j"       : j
                }
            weave_cpp(code, vars)
        return result
        
