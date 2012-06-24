"""
Some tools for efficiently manipulating images.
Author: Travis Dick (travis.barry.dick@gmail.com)
"""

import itertools

import cv2
import numpy as np
from scipy import weave
from scipy.weave import converters

from Homography import *

def draw_region(img, corners, color, thickness=1):
    """ Draws a warped rectangle in an image.

    Parameters:
    -----------
    img : (n,m) numpy array
      The image to be drawn into
      
    corners : (2,4) numpy array
      An array whose columns are the corners of the warped
      rectangle. Points should be in (counter-)clockwise order.

    color: scalar or (b,g,r) triple
      The color to make the region. In black and white images
      a scalar between 0 and 1 specifies intensity, and in 
      color images, a (b,g,r) triple (0 <= b,g,r <= 255) specifies
      the color.

    thickness: integer
      The width in pixels of the lines used.
    """
    for i in xrange(4):
        p1 = (int(corners[0,i]), int(corners[1,i]))
        p2 = (int(corners[0,(i+1)%4]), int(corners[1,(i+1)%4]))
        cv2.line(img, p1, p2, color, thickness)
    for i in xrange(4):
        p1 = (int(corners[0,i]), int(corners[1,i]))
        p2 = (int(corners[0,(i+2)%4]), int(corners[1,(i+2)%4]))
        cv2.line(img, p1, p2, color, thickness)

def to_grayscale(img):
    """ Converts an bgr8 image into a grayscale image.

    Parameters:
    -----------
    img : (n,m,3) numpy array
      The input image, with 3 channels. Entries should take
      values in the range (0,256).
    
    Returns:
    --------
    An (n,m) numpy array with entries between 0 and 1.
    """
    (height, width, depth) = img.shape
    grayscale = np.empty((height,width), dtype=np.float64)
    code = \
    """
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        double mean = 0;
        for (int k = 0; k < depth; k++) mean += img(i,j,k);
        grayscale(i,j) = mean / depth / 256;
      }
    }
    """
    weave.inline(code, ['height', 'width', 'depth', 'grayscale', 'img'],
                 type_converters=converters.blitz,
                 compiler='gcc')
    return grayscale

def sample_region(img, pts, result=None):
    """ Samples the image intenisty at a collection of points.

    Notes: 
    ------
      - Only works with grayscale images. 
      - All points outside the bounds of the image have intensity 0.5.

    Parameters:
    -----------
    img : (n,m) numpy array
      The image to be sampled from.
     
    pts : (2,k) numpy array
      The points to be sampled out of the image. These may be sub-pixel
      coordinates, in which case bilinear interpolation is used.

    result : (k) numpy array (optional)
      Optionally you can pass in a results vector which will store the 
      sampled vector. If you do not supply one, this function will allocate
      one and return a reference.
    
    Returns:
    --------
    Returns a (k) numpy array containing the intensities of the given 
    sub-pixel coordinates in the provided image.
    """
    num_pts = pts.shape[1]
    (height, width) = img.shape
    if result == None: result = np.empty(num_pts)
    support_code = \
    """
    double bilinear_interp(blitz::Array<double,2> img, int width, int height, double x, double y) {
      using std::floor;
      using std::ceil;
      const int lx = floor(x);
      const int ux = ceil(x);
      const int ly = floor(y);
      const int uy = ceil(y);
      if (lx < 0 || ux >= width || ly < 0 || uy >= height) return 0.5;
      const double ulv = img(ly,lx);
      const double urv = img(ly,ux);
      const double lrv = img(uy,ux);
      const double llv = img(uy,lx);
      const double dx = x - lx;
      const double dy = y - ly;
      return ulv*(1-dx)*(1-dy) + urv*dx*(1-dy) + llv*(1-dx)*dy + lrv*dx*dy;
    }
    """
    code = \
    """
    for (int i = 0; i < num_pts; i++) {
      result(i) = bilinear_interp(img, width, height, pts(0,i), pts(1,i));
    }
    """
    weave.inline(code, ["img", "result", "pts", "num_pts", "width", "height"],
                 support_code=support_code, headers=["<cmath>"],
                 type_converters=converters.blitz,
                 compiler='gcc')
    return result

def sample_and_normalize(img, pts):
    """ Samples the image intensity at a collection of points 
    and normalizes the result.

    Identical to sample_region, except the result is shifted
    so that it's components have mean 0.

    See Also:
    ---------
    sample_region
    """
    result = sample_region(img, pts);
    result -= result.mean()
    return result
