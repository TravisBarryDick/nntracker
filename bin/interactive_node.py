#!/usr/bin/env python

import roslib
roslib.load_manifest('nntracker')

import sys

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo

from nntracker.trackers.TurnkeyTrackers import *
from nntracker.msg import NNTrackerROI
from nntracker.InteractiveTracking import *
from nntracker.utility import *

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
    tracker = make_nn_GN()
    app = RosInteractiveTrackingApp(tracker)
    app.run()
    app.cleanup()


