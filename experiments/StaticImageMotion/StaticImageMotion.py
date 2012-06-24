import cv2
import numpy as np

import sys
sys.path.append('../../src/NNTracker')
from Homography import *
from NNTracker import *

lenna = to_grayscale(cv2.imread("lenna.png"))

def random_warp(width, height, sigma):
    corners = np.array([ (0,0), (width,0), (width, height), (0, height)],
                       dtype=np.float64).T
    perturbations = np.random.randn(2,4) * sigma
    return compute_homography(corners, corners + perturbations)
    
def warp_image(img, w, result):
    (height, width) = img.shape
    pts = np.array(list(map(lambda (x,y): (y,x), itertools.product(xrange(0,width),xrange(0,height))))).T
    result.shape = (-1,)
    warped_pts = apply_to_pts(w.I, pts)
    sample_region(img, warped_pts, result)
    result.shape = (height, width)

cv2.imshow("before", lenna)
warped_lenna = np.empty_like(lenna)

tracker = NNTracker(20000, 5, res=(25,25),
                    warp_generator = lambda: random_homography(0.1,1e-10))
centerx = lenna.shape[0]/2
centery = lenna.shape[1]/2
target_corners = np.array([(centerx - 100, centery - 100),
                           (centerx + 100, centery - 100),
                           (centerx + 100, centery + 100),
                           (centerx - 100, centery + 100)]).T

tracker.initialize(lenna, target_corners)

for i in xrange(1000):
    w = random_warp(lenna.shape[0], lenna.shape[1], 10)
    warp_image(lenna, w, warped_lenna)
    tracker.set_region(target_corners)
    tracker.update(warped_lenna)
    draw_region(warped_lenna, target_corners, 0.3, 3)
    draw_region(warped_lenna, apply_to_pts(w, target_corners), 1, 3)
    draw_region(warped_lenna, tracker.get_corners(), 0, 1)
    cv2.imshow("after", warped_lenna)
    cv2.waitKey(1)
