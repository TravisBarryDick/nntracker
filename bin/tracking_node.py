#!/user/bin/env python

import roslib
roslib.load_manifest('nntracker')

import threading

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from nntracker.trackers.TurnkeyTrackers import *
from nntracker.msg import NNTrackerROI
from nntracker.utility import *

rospy.init_node("nntracker")

# Read the algorithm name argument and construct appropriate tracker
algorithm_name = rospy.get_param("~algorithm", "nn_bmic")
if algorithm_name == "nn_bmic":
    algorithm = make_nn_GN(use_scv=True)
else if algorithm_name == "esm":
    algorithm = make_esm(use_scv=True)

# Set up the OpenCV Bridge
cvbridge = CvBridge()

# Make a subscriber to grab images
image_sub = rospy.Subscriber("image", Image, image_callback, queue_size = 1)

current_rgb_img = None
current_gray_img = None

def image_callback(self, data):
    global current_rgb_img, current_gray_img
    # Read image from message
    current_img = np.array(cvbridge.imgmsg_to_cv(data, "bgr8"))
    # Compute the gray image
    current_gray_img = cv2.GaussianBlur(np.asarray(to_grayscale(current_img)), (5,5), 3)
    # If the tracker is running, ask it to update
    if algorithm.is_initialized(): algorithm.update(current_gray_img)
        
