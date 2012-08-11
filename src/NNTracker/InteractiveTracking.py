"""
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import cv
import cv2
import numpy as np

from utility import *

class InteractiveTrackingApp:
    def __init__(self, tracker, name="vis", init_with_rectangle=True):
        """ An interactive window for initializing and visualizing tracker state.

        The on_frame method should be called for each new frame. Typically real
        applications subclass InteractiveTrackingApp and build in some application
        loop that captures frames and calls on_frame.
        
        Parameters:
        -----------
        tracker : TrackerBase
          Any class implementing the interface of TrackerBase. 

        name : string
          The name of the window. Due to some silliness in OpenCV this must
          be unique (in the set of all OpenCV window names).

        See Also:
        ---------
        StandaloneTrackingApp
        RosInteractiveTrackingApp
        """

        self.tracking = False
        self.tracker = tracker
        self.name = name
        self.init_with_rectangle = init_with_rectangle

        self.m_start = None
        self.m_end = None
        self.corner_pts = []

        self.gray_img = None
        self.paused = False
        self.img = None
        cv2.namedWindow(self.name)
        cv2.setMouseCallback(self.name, self.mouse_handler)

    def display(self, img):
        annotated_img = img.copy()
        if self.tracker.is_initialized() and self.tracking:
            draw_region(annotated_img, self.tracker.get_region(), (0,255,0), 2)
        if self.init_with_rectangle and self.m_start != None and self.m_end != None:
            ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
            lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))             
            corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T            
            draw_region(annotated_img, corners, (255,0,0), 1)
        if not self.init_with_rectangle:
            for pt in self.corner_pts:
                cv2.circle(annotated_img, pt, 2, (255,0,0), 1)

        cv2.imshow(self.name, annotated_img)

    def mouse_handler(self, evt,x,y,arg,extra):
        if self.gray_img == None: return 
        if evt == cv2.EVENT_LBUTTONDOWN and self.m_start == None:
            if self.init_with_rectangle:
                self.m_start = (x,y)
                self.m_end = (x,y)
                self.paused = True
            else:
                self.corner_pts.append( (x,y) )
                if len(self.corner_pts) == 4:
                    self.display(self.img)
                    cv2.waitKey(1)
                    corners = np.array(self.corner_pts, dtype=np.float64).T
                    if not self.tracking:
                        self.tracker.initialize(self.gray_img, corners)
                        self.tracking = True
                    else:
                        self.tracker.set_region(corners)
                    self.corner_pts = []


        elif evt == cv2.EVENT_MOUSEMOVE and self.m_start != None:
            if self.init_with_rectangle:
                self.m_end = (x,y)
        elif evt == cv2.EVENT_LBUTTONUP:
            if self.init_with_rectangle:
                self.m_end = (x,y)
                ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
                lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))
                if not self.tracking:
                    self.tracker.initialize_with_rectangle(self.gray_img, ul, lr)
                else: 
                    self.tracker.set_region(np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T)
                self.m_start, self.m_end = None, None
                self.tracking = True
                self.paused = False

    

    def on_frame(self, img):
        if not self.paused:
            self.img = img
            self.gray_img = cv2.GaussianBlur(np.asarray(to_grayscale(img)), (5,5), 3)
            #self.gray_img = to_grayscale(img)
            self.tracker.update(self.gray_img)
        if self.img != None: self.display(self.img)
        key = cv.WaitKey(7)
        if key == ord(' '): self.paused = not self.paused
        elif key == ord('c'): self.tracking = False
        elif key > 0: return False
        return True

    def cleanup(self):
        cv2.destroyWindow(self.name)
