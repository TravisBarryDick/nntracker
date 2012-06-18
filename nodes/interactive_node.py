#!/usr/bin/env python

import roslib
roslib.load_manifest('NNTracker')

import sys

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from NNTracker.msg import NNTrackerROI
from cv_bridge import CvBridge, CvBridgeError

import cv
import cv2
import numpy as np

from NNTracker.Homography import *
from NNTracker.ImageUtils import *
from NNTracker.NNTracker import *
from NNTracker.Polygons import *

class InteractiveTrackingApp:
    def __init__(self, tracker, image_topic="/image_raw", roi_topic="/roi_NNTracker"):
        rospy.init_node('travis_tracker')
        self.tracker = tracker
        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.roi_pub = rospy.Publisher(roi_topic, NNTrackerROI)
        self.bridge = CvBridge()

        self.m_start = None
        self.m_end = None
        self.gray_img = None
        self.img = None
        self.paused = False

        cv2.namedWindow("Travis Tracker", 1)
        cv2.setMouseCallback("Travis Tracker", self.mouse_handler)

    def cleanup(self):
        cv2.destroyWindow('Travis Tracker')

    def display(self, img):
        annotated_img = img.copy()
        if self.tracker.is_initialized():
            draw_region(annotated_img, self.tracker.get_corners(), (255,0,0), 2)
        if self.m_start != None and self.m_end != None:
            ul = (min(self.m_start[0],self.m_end[0]), min(self.m_start[1],self.m_end[1]))
            lr = (max(self.m_start[0],self.m_end[0]), max(self.m_start[1],self.m_end[1]))            
            corners = np.array([ ul, [lr[0],ul[1]], lr, [ul[0],lr[1]]]).T            
            draw_region(annotated_img, corners, (0,255,0), 1)
        cv2.imshow('Travis Tracker', annotated_img)

    def mouse_handler(self, evt,x,y,arg,extra):
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

    def callback(self, data):
        if self.img != None: self.display(self.img)
        if not self.paused:
            self.img = np.array(self.bridge.imgmsg_to_cv(data, "bgr8"))
            gray = to_grayscale(self.img)
            self.gray_img = cv2.GaussianBlur(gray, (5,5), 3.0)
            self.tracker.update(self.gray_img)
        if self.tracker.is_initialized():
            message = NNTrackerROI()
            corners = self.tracker.get_corners()
            message.ulx, message.uly = corners[:,0]
            message.urx, message.ury = corners[:,1]
            message.lrx, message.lry = corners[:,2]
            message.llx, message.lly = corners[:,3]
            message.perimeter, message.area, (message.cmx, message.cmy) = polygon_descriptors(corners)
            self.roi_pub.publish(message)
        #cv.WaitKey(7) 
        cv2.waitKey(1)

def main(args):
    tracker =  NNTracker(10000, 4, res=(35,35),n_proposals=12, proposal_recycle_rate=6)
    app = InteractiveTrackingApp(tracker)
    rospy.spin()
    app.cleanup()

if __name__ == '__main__':
    main(sys.argv)
