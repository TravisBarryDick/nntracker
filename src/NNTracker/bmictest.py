import cPickle

import cv2
import numpy as np

from BakerMatthewsICTracker import *
from Homography import *
from ImageUtils import *

def estimate_jacobian(img, ul, lr, eps=1e-10):
    def _rectangle_to_region(ul, lr):
        return np.array([ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
    def _make_hom(p):
        return np.matrix([[1 + p[0],   p[1]  , p[2]],
                          [  p[3]  , 1 + p[4], p[5]],
                          [  p[6]  ,   p[7]  ,   1]], dtype=np.float64)    
    res = (lr[0]-ul[0], lr[1]-ul[1])
    pts = res_to_pts(res)
    n_pts = np.prod(res)
    initial_warp = square_to_corners_warp(_rectangle_to_region(ul,lr))
    def f(p):
        H = _make_hom(p)
        W = initial_warp * H
        return sample_region(img, pts, W)
    jacobian = np.empty((n_pts,8))
    for i in xrange(0,8):
        o = np.zeros(8)
        o[i] = eps
        jacobian[:,i] = (f(o) - f(-o)) / (2*eps)
    return jacobian 

if __name__=="__main__":
    data = cPickle.load(file("/Users/travisdick/Desktop/takeo.pkl", "r"))
    ul, lr, img = data["ul"], data["lr"], data["img"]
    res = (lr[0] - ul[0], lr[1] - ul[1])
    tracker = BakerMatthewsICTracker(15, 1e-6, res)
    tracker.initialize_with_rectangle(img, ul, lr)
    
    est_jacobian = estimate_jacobian(img, ul, lr)
    for i in xrange(8):
        est_sd_img = est_jacobian[:,i].reshape(res)
        est_sd_img -= np.min(est_sd_img)
        est_sd_img /= np.max(est_sd_img)
        sd_img = tracker.VT_dW_dp[:,i].reshape(tracker.res)
        sd_img -= np.min(sd_img)
        sd_img /= np.max(sd_img)
        cv2.imshow("img%d"%i, np.hstack([sd_img, est_sd_img]).astype(np.float64))
        cv2.imshow("diff%d"%i, np.abs(sd_img/2 - est_sd_img/2).astype(np.float64))



    

