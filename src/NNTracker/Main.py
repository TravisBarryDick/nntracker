import cv
import cv2
import numpy as np

from Homography import *
from ImageUtils import *
from NNTracker import *

class InteractiveTrackingApp:
    def __init__(self, vc, tracker):
        self.vc = vc
        self.tracker = tracker
        self.m_start = None
        self.m_end = None
        self.gray_img = None
        self.paused = False

    def display(self, img):
        annotated_img = img.copy()
        if self.tracker.is_initialized():
            draw_region(annotated_img, self.tracker.get_corners(), (255,0,0), 2)
        if self.m_start != None and self.m_end != None:
            ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
            lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))            
            corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T            
            draw_region(annotated_img, corners, (0,255,0), 1)
        cv2.imshow('vis', annotated_img)

    def run(self):
        def mouse_handler(evt,x,y,arg,extra):
            if self.gray_img == None: return 
            if evt == cv2.EVENT_LBUTTONDOWN and self.m_start == None:
                self.m_start = (x,y)
                self.m_end = (x,y)
                self.paused = True
            elif evt == cv2.EVENT_MOUSEMOVE and self.m_start != None:
                self.m_end = (x,y)
            elif evt == cv2.EVENT_LBUTTONUP:
                self.m_end = (x,y)
                ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
                lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))
                self.tracker.initialize_with_rectangle(self.gray_img, ul, lr)
                self.m_start, self.m_end = None, None
                self.paused = False
                self.inited = True

        cv2.namedWindow('vis')
        cv2.setMouseCallback('vis', mouse_handler)
        self.corners = []
        self.initeds = []
        self.inited = False
        while True:
            key = cv2.waitKey(1)
            if key == ord(' '): self.paused = not self.paused
            elif key > 0: break
            if not self.paused:
                (succ, img) = self.vc.read()
                if not succ: break
                gray = to_grayscale(img)
                self.gray_img = cv2.GaussianBlur(gray, (5,5), 3.0)
                self.tracker.update(self.gray_img)
                if self.tracker.is_initialized(): self.corners.append(self.tracker.get_corners())
                else: self.corners.append(None)
                self.initeds.append(self.inited)
                self.inited = False
            self.display(img)
        cv2.destroyWindow('vis')
        self.vc.release()


def track_camera():
    app = InteractiveTrackingApp(cv2.VideoCapture(0), 
                                 NNTracker(10000, 4, res=(20,20),n_proposals=12, proposal_recycle_rate=6))
    app.run()

if __name__ == '__main__':
    track_camera()
