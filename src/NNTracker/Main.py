"""
A small standalone application for tracker demonstration. Depends
on OpenCV VideoCapture to grab frames from the camera.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""

from BakerMatthewsICTracker import *
from CascadeTracker import *
from Homography import *
from InteractiveTracking import *
from MultiProposalTracker import *
from NNTracker import *
from ParallelTracker import *
from SCVNNTracker import *

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
    coarse_tracker = NNTracker(12000, 1, res=(40,40))
    fine_tracker = SCVNNTracker(2000, 3, res=(40,40), warp_generator = lambda:random_homography(0.001, 0.0001))
    tracker = BakerMatthewsICTracker(20, res=(100,100))
    cascade_tracker = CascadeTracker([coarse_tracker, fine_tracker])
    app = StandaloneTrackingApp(cv2.VideoCapture(0), tracker)
    app.run()
