import cv2
import numpy as np

from Homography import *
from ImageUtils import *
from NNTracker import *

def draw_region(img, corners, color, thickness=1):
    for i in xrange(4):
        p1 = (int(corners[0,i]), int(corners[1,i]))
        p2 = (int(corners[0,(i+1)%4]), int(corners[1,(i+1)%4]))
        cv2.line(img, p1, p2, color, thickness)
        #cv2.line(img, corners[:,i], corners[:,(i+1)%4], color, thickness)
        

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
                self.tracker = NNTracker(5000,1)
                ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
                lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))
                res = (32,32)
                self.tracker.initialize(self.gray_img, ul, lr, res)
                self.m_start, self.m_end = None, None
        vc = cv2.VideoCapture(0)
        cv2.namedWindow('vis')
        cv2.setMouseCallback('vis', mouse_handler)
        
        while cv2.waitKey(1) <= 0:
            (succ, img) = vc.read()
            self.gray_img = to_grayscale(img)
            if self.tracker != None:
                self.tracker.update(self.gray_img)
                draw_region(img, self.tracker.get_corners(), (0,255,0))
            cv2.imshow('vis', img)
        cv2.destroyWindow('vis')
        vc.release()

def track_camera():
    app = CameraTrackingApp()
    app.run()

if __name__ == '__main__':
    track_camera()
