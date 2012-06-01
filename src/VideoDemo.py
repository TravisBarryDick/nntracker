import cv2

from ImageUtils import *
from NNTracker import *

class GrabInitsTracker:

    def __init__(self):
        self.inits = {}
        self.frame = 0

    def initialize(self, img, corners):
        self.inits[self.frame] = corners

    def initialize_with_rectangle(self, img, ul, lr):
        corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T
        self.initialize(img, corners)

    def is_initialized(self):
        return False;
        
    def update(self, img):
        self.frame += 1

def video_demo(vc, inits, title="NN Tracking Demo"):
    tracker = NNTracker(10000, 5, res=(40,40), n_proposals=12, proposal_recycle_rate=6)
    frame = 0
    corners = []
    cv2.namedWindow(title)
    try:
        while True:
            succ, img = vc.read()
            if not succ: break
            gray_img = to_grayscale(img)
            if frame in inits.keys():
                tracker.initialize(gray_img, inits[frame])
            else:
                tracker.update(gray_img)
            if tracker.is_initialized():
                draw_region(img, tracker.get_corners(), (255,0,0), 1)
                corners.append(tracker.get_corners())
            cv2.imshow(title, img)
            frame += 1
            cv2.waitKey(1)
    finally:
        cv2.destroyWindow(title)
    return corners
