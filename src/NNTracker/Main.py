"""
A small standalone application for tracker demonstration. Depends
on OpenCV VideoCapture to grab frames from the camera.

Author: Travis Dick (travis.barry.dick@gmail.com)

"""

from utility import *
from InteractiveTracking import *
from TurnkeyTrackers import make_esm, make_nn_GN, make_nn_esm

class StandaloneTrackingApp(InteractiveTrackingApp):
    """ A demo program that uses OpenCV to grab frames. """
    
    def __init__(self, vc, tracker, name="vis", start_paused=False):
        InteractiveTrackingApp.__init__(self, tracker, name, init_with_rectangle=False)
        self.vc = vc
        self.start_paused = start_paused
    
    def run(self):
        if self.start_paused:
            succ, img = self.vc.read()
            self.on_frame(img)
            self.paused = True
        while True:
            if not self.paused:
                (succ, img) = self.vc.read()
                if not succ: break
            if not self.on_frame(img): break
            cv2.waitKey(1)
        self.cleanup()

if __name__ == '__main__':
    app = StandaloneTrackingApp(cv2.VideoCapture(0), make_nn_GN(use_scv=True))
    app.run()
    app.cleanup()
