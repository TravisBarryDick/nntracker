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
from NNTracker.BakerMatthewsICTracker import *
from NNTracker.CascadeTracker import *
from NNTracker.Homography import *
from NNTracker.InteractiveTracking import *
from NNTracker.NNTracker import *
from NNTracker.Polygons import *

class RosInteractiveTrackingApp(InteractiveTrackingApp):
    
    def __init__(self, tracker):
        InteractiveTrackingApp.__init__(self, tracker, "ROS Interactive NN Tracker")
        rospy.init_node("NNTracker")
        
        image_topic = rospy.get_param("~image", "/camera/image_raw")
        roi_topic = rospy.get_param("~roi_topic", "roi_NNTracker")

        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.roi_pub = rospy.Publisher(roi_topic, NNTrackerROI)
        self.bridge = CvBridge()

    def run(self):
        rospy.spin()

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
    coarse_tracker = NNTracker(12000, 2, res=(40,40), use_scv=True)
    fine_tracker = NNTracker(2000, 3, res=(40,40), 
                             warp_generator = lambda:random_homography(0.005, 0.0001),
                             use_scv=True)
    finer_tracker = BakerMatthewsICTracker(20, res=(40,40), use_scv=True)
    tracker = CascadeTracker([coarse_tracker, fine_tracker, finer_tracker])
    app = RosInteractiveTrackingApp(tracker)
    app.run()
    app.cleanup()


