import cv
import cv2
import numpy as np

from CameraMotionHomography import *
from Homography import *
from ImageUtils import *
from NNTracker import *

class CameraTrackingApp:
    
    def __init__(self):
        self.m_start = None
        self.m_end = None
        self.tracker = None
        self.gray_img = None

    def run(self):
        def mouse_handler(evt,x,y,arg,extra):
            if self.gray_img == None: return # Make sure at least one image has been captured
            if evt == cv2.EVENT_LBUTTONDOWN and self.m_start == None:
                self.m_start = (x,y)
                self.m_end = (x,y)
            elif evt == cv2.EVENT_MOUSEMOVE and self.m_start != None:
                self.m_end = (x,y)
            elif evt == cv2.EVENT_LBUTTONUP:
                self.m_end = (x,y)
                self.tracker = NNTracker(500,1,res=(16,16), n_proposals=10, proposal_recycle_rate=5)
                ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
                lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))
                self.tracker.initialize_with_rectangle(self.gray_img, ul, lr)
                self.m_start, self.m_end = None, None

        vc = cv2.VideoCapture(0)

        cv2.namedWindow('vis')
        cv2.setMouseCallback('vis', mouse_handler)
        
        while cv2.waitKey(1) <= 0:
            (succ, img) = vc.read()
            if not succ: break
            gray = to_grayscale(img)
            self.gray_img = cv2.GaussianBlur(gray, (5,5), 3.0)

            if self.tracker != None:
                self.tracker.update(self.gray_img)
                for i in xrange(self.tracker.n_proposals):
                    draw_region(img, self.tracker.get_corners(i), (0,190,0))
                draw_region(img, self.tracker.get_corners(), (255,0,0), 2)
            cv2.imshow('vis', img)
        cv2.destroyWindow('vis')
        vc.release()

def track_camera():
    app = CameraTrackingApp()
    app.run()

if __name__ == '__main__':
    track_camera()
