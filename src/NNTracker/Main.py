"""
A small standalone application for tracker demonstration. Depends
on OpenCV VideoCapture to grab frames from the camera.

Author: Travis Dick (travis.barry.dick@gmail.com)
"""
from Homography import *
from InteractiveTracking import *
from TurnkeyTrackers import make_esm, make_nn_GN, make_nn_esm

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
    app = StandaloneTrackingApp(cv2.VideoCapture(0), make_nn_GN(use_scv=True))
    app.run()
    app.cleanup()
