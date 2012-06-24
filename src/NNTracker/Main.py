import cv
import cv2
import numpy as np

from CascadeTracker import *
from Homography import *
from InteractiveTracking import *
from NNTracker import *

class StandaloneTrackingApp(InteractiveTrackingApp):
    """ A demo program that uses OpenCV to grab frames. """
    
    def __init__(self, vc, tracker, name="vis"):
        InteractiveTrackingApp.__init__(self, tracker, name)
        self.vc = vc
    
    def run(self):
        while True:
            (succ, img) = self.vc.read()
            if not succ: break
            if not self.on_frame(img): break
        self.cleanup()

if __name__ == '__main__':
    coarse_tracker = NNTracker(12000, 2, res=(20,20))
    fine_tracker = NNTracker(2000, 3, res=(50,50), warp_generator = lambda:random_homography(0.005, 0.0001))
    tracker = CascadeTracker([coarse_tracker, fine_tracker])
    app = StandaloneTrackingApp(cv2.VideoCapture(0), tracker)
    app.run()
