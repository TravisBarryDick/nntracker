Nearest Neighbour Tracking Algorithm
====================================

Example:
--------
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
        draw_region(frame, tracker.get_corners())
        imshow(frame)

The functions <code> grab_frame() </code>, <code>
grab_initialization_rectangle() </code> and <code> imshow </code> are
just placeholders to simplify the example.

Installation:
-------------

There is no special installation procedure. You can obtain a copy of
the source code by cloning this git repository:

    git clone git://github.com/TravisBarryDick/NNTracker.git

You must also install the following dependencies:

- Python
- Numpy
- Scipy (scipy.weave in particular)
- OpenCV with python bindings
- FLANN with python bindings
