import cv2
import numpy as np

import sys
sys.path.append('../../src/NNTracker')
from CascadeTracker import *
from Homography import *
from MultiProposalTracker import *
from NNTracker import *


def random_warp(width, height, sigma):
    corners = np.array([ (0,0), (width,0), (width, height), (0, height)],
                       dtype=np.float64).T
    perturbations = np.random.randn(2,4) * sigma
    return compute_homography(corners, corners + perturbations)
    
# def warp_image(img, pts, w, result):
#     result.shape = (-1,)
#     sample_region(img, pts, np.array(w.I), result)
#     result.shape = (height, width)

def warp_image(img, w, result):
    if result == None: result = np.empty_like(img)
    return cv2.warpPerspective(img, w, img.shape, dst=result)

lenna = to_grayscale(cv2.imread("lenna.png"))
(height, width) = lenna.shape
pts = np.array(list(map(lambda (x,y): (y,x), itertools.product(xrange(0,width),xrange(0,height))))).T

cv2.imshow("before", lenna)
warped_lenna = np.empty_like(lenna)

coarse_tracker = NNTracker(12000, 1, res=(20,20))
fine_tracker = NNTracker(2000, 5, res=(100,100), warp_generator = lambda:random_homography(0.005, 0.0001))
tracker = CascadeTracker([coarse_tracker, fine_tracker])

centerx = lenna.shape[0]/2
centery = lenna.shape[1]/2
target_corners = np.array([(centerx - 50, centery - 50),
                           (centerx + 50, centery - 50),
                           (centerx + 50, centery + 50),
                           (centerx - 50, centery + 50)]).T

tracker.initialize(lenna, target_corners)

for i in xrange(1000):
    w = random_warp(lenna.shape[0], lenna.shape[1], 5)
    warp_image(lenna,np.asarray(w).astype(np.float64), warped_lenna)
    tracker.set_region(target_corners)
    tracker.update(warped_lenna)
    draw_region(warped_lenna, target_corners, 0.3, 3)
    draw_region(warped_lenna, apply_to_pts(w, target_corners), 1, 3)
    draw_region(warped_lenna, tracker.get_region(), 0, 1)
    cv2.imshow("after", warped_lenna)
    cv2.waitKey(1)
