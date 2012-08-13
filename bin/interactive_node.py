#!/usr/bin/env python

import roslib
roslib.load_manifest('nntracker')

import sys
import threading

from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo

from nntracker.trackers.TurnkeyTrackers import *
from nntracker.msg import NNTrackerROI
from nntracker.InteractiveTracking import *
from nntracker.utility import *

class TrackingThread(threading.Thread):
    def __init__(self, tracking_app):
        self.tracking_app = tracking_app

    def run(self):
        with self.tracking_app.next_frame_lock:
            if self.tracking_app.next_frame != None:
                tracking_app.onframe(tracking_app.next_frame)
            tracking_app.next_frame = None
        if self.tracking_app.tracker.is_initialized() and not self.tracking_app.paused:
            message = NNTrackerROI()
            region = self.tracking_app
            message.ulx, message.uly = region[:,0]
            message.urx, message.ury = region[:,1]
            message.lrx, message.lry = region[:,2]
            message.llx, message.lly = region[:,3]
            message.perimeter, message.area, (message.cmx, message.cmy) = polygon_descriptors(region)
            self.roi_pub.publish(message)
        cv.waitKey(1)

class RosInteractiveTrackingApp(InteractiveTrackingApp):
    
    def __init__(self, tracker):
        InteractiveTrackingApp.__init__(self, tracker, "ROS Interactive NN Tracker",
                                        init_with_rectangle=False)
        rospy.init_node("NNTracker")
        
        image_topic = rospy.get_param("~image", "/camera/image_raw")
        roi_topic = rospy.get_param("~roi_topic", "roi_NNTracker")

        self.image_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1)
        self.roi_pub = rospy.Publisher(roi_topic, NNTrackerROI)
        self.bridge = CvBridge()
        
        self.next_frame = None
        self.next_frame_lock = threading.Lock()

    def run(self):
        thread = TrackingThread(self)
        thread.start()
        rospy.spin()

    def callback(self, data):
        img = np.array(self.bridge.imgmsg_to_cv(data, "bgr8"))
        with self.next_frame_lock:
            self.next_frame = img

if __name__ == '__main__':
    tracker = make_nn_GN()
    app = RosInteractiveTrackingApp(tracker)
    app.run()
    app.cleanup()


