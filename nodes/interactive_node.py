#!/usr/bin/env python

import roslib
roslib.load_manifest('NNTracker')

import sys

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo

from NNTracker.msg import NNTrackerROI
from NNTracker.CascadeTracker import *
from NNTracker.Homography import *
from NNTracker.InteractiveTracking import *
from NNTracker.NNTracker import *
from NNTracker.Polygons import *

class RosInteractiveTrackingApp(InteractiveTrackingApp):
    
    def __init__(self, tracker):
        InteractiveTrackingApp.__init__(self, tracker, "ROS Interactive NN Tracker")
        rospy.init_node("NNTracker")
        self.image_sub = rospy.Subscriber("/image_raw", Image, self.callback, queue_size=1)
        self.roi_pub = rospy.Publisher("/roi_NNTracker", NNTrackerROI)
        self.bridge = CvBridge()

    def callback(self, data):
        img = np.array(self.bridge.imgmsg_to_cv(data, "bgr8"))
        self.on_frame(img)
        if self.tracker.is_initialized() and not self.paused:
            message = NNTrackerROI()
            region = self.tracker.get_region()
            message.ulx, message.uly = region[:,0]
            message.urx, message.ury = region[:,1]
            message.lrx, message.lry = region[:,2]
            message.llx, message.lly = region[:,3]
            message.perimeter, message.area, (message.cmx, message.cmy) = polygon_descriptors(region)
            self.roi_pub.publish(message)
        cv.WaitKey(7) 
        #cv2.waitKey(1)

if __name__ == '__main__':
    coarse_tracker = NNTracker(12000, 2, res=(20,20))
    fine_tracker = NNTracker(2000, 3, res=(50,50), warp_generator = 
                             lambda:random_homography(0.005, 0.0001))
    tracker = CascadeTracker([coarse_tracker, fine_tracker])
    app = RosInteractiveTrackingApp(tracker)
    rospy.spin()
    app.cleanup()


