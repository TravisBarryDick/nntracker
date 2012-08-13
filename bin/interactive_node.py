#!/usr/bin/env python

import roslib
roslib.load_manifest('nntracker')

import sys
import threading
from Queue import LifoQueue

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
        threading.Thread.__init__(self)
        self.tracking_app = tracking_app
        self.daemon = True

    def run(self):
        while True:
            while not self.tracking_app.frames.empty():
                frame = self.tracking_app.frames.get()
                self.tracking_app.on_frame(frame)
            if self.tracking_app.tracker.is_initialized() and not self.tracking_app.paused:
                message = NNTrackerROI()
                region = self.tracking_app.tracker.get_region()
                message.ulx, message.uly = region[:,0]
                message.urx, message.ury = region[:,1]
                message.lrx, message.lry = region[:,2]
                message.llx, message.lly = region[:,3]
                message.perimeter, message.area, (message.cmx, message.cmy) = polygon_descriptors(region)
                self.tracking_app.roi_pub.publish(message)
            cv2.waitKey(1)

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
        
        self.frames = LifoQueue(5)

        # self.next_frame = None
        # self.next_frame_lock = threading.Lock()

    def run(self):
        thread = TrackingThread(self)
        thread.start()
        rospy.spin()

    def callback(self, data):
        self.frames.put(np.array(self.bridge.imgmsg_to_cv(data, "bgr8")), False)
        # with self.next_frame_lock:
        #     self.next_frame = np.array(self.bridge.imgmsg_to_cv(data, "bgr8"))

if __name__ == '__main__':
    tracker = make_nn_GN(use_scv = True)
    app = RosInteractiveTrackingApp(tracker)
    app.run()
    app.cleanup()


