Nearest Neighbour Tracking Algorithm
====================================

ROS Example:
------------

    roscore
    rosrun some_camera_driver
    rosrun NNTracker interactive_node.py _image:=the_image_topic _roi_topic:=/roi
    rxplot /roi/cmx /roi/cmy

Python Example:
---------------
    from NNTracker import *
    
    # Initialization:
    first_frame = grab_frame()
	ul, lr = grab_initialization_rectangle()
    tracker = NNTracker( ... some parameters ... )
    tracker.initialize_with_rectangle(first_frame, ul, lr)

	# Tracking:
    while True:
        frame = grab_frame()
        tracker.update(frame)
        draw_region(frame, tracker.get_region())
        imshow(frame)

The functions <code> grab_frame() </code>, <code>
grab_initialization_rectangle() </code> and <code> imshow() </code> are
just placeholders to simplify the example.

For a complete example, look at src/NNTracker/Main.py.

Installation:
-------------

Place this directory in your ros_workspace. 

You must also have the following dependencies installed:

- Python
- Numpy
- Scipy
- OpenCV (with python bindings)
- FLANN (with python bindings)

Note: The binary distributions of flann that I can find do not come
with the python binding. If you build from source they are installed
by defualt. To check to make sure you have them, in a python shell
type: <code> import pyflann </code>
