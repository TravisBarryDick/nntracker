from utility import *
from ESMTracker import *
from BMICTracker import *
from NNTracker import *
import cv2
import numpy as np

import sys
sys.path.append("/Users/travisdick/Dropbox/School/Research/VisualTracking/ROSPackages/NNTracker/src/NNTracker")
import Main
import ImageUtils
import SL3HomParam
import CascadeTracker


img = np.asarray(to_grayscale(cv2.imread("/Users/travisdick/Desktop/Lenna.png")))
w = np.matrix([[200,0,512/2.0+0.01],[0,200,512/2.0],[0,0,1]])
res = (200,200)
pts = ImageUtils.res_to_pts(res)
je = SL3HomParam.JacobianEstimator()

if __name__ == "__main__":
    vc = cv2.VideoCapture(0)

    t1 = NNTracker(1, 8000, 50, 50, 0.06, 0.07)
    t2 = NNTracker(1, 5000, 50, 50, 0.0001, 0.005)
    t3 = BMICTracker(15, 0.001, 50, 50)
    tracker = CascadeTracker.CascadeTracker([t1,t2,t3])
    
    app = Main.StandaloneTrackingApp(vc, tracker, "test")
    app.run()
