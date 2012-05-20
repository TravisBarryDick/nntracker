import math

import numpy as np

from Homography import *

def x_rotation(angle):
    s = math.sin(angle)
    c = math.cos(angle)
    return np.matrix([[1,0, 0,0],
                      [0,c,-s,0],
                      [0,s, c,0],
                      [0,0, 0,1]])

def y_rotation(angle):
    s = math.sin(angle)
    c = math.cos(angle)
    return np.matrix([[ c,0,s,0],
                      [ 0,1,0,0],
                      [-s,0,c,0],
                      [ 0,0,0,1]])

def z_rotation(angle):
    s = math.sin(angle)
    c = math.cos(angle)
    return np.matrix([[c,-s,0,0],
                      [s, c,0,0],
                      [0, 0,1,0],
                      [0, 0,0,1]])

def homography_from_camera(x, y, z, rx, ry, rz):
    translation = np.matrix([[1,0,0,x],
                             [0,1,0,y],
                             [0,0,1,z],
                             [0,0,0,1]], dtype=np.float64)
    projection = np.matrix([[1,0,0,0],
                            [0,1,0,0],
                            [0,0,1,0]], dtype=np.float64)
    camera_matrix = projection * translation * x_rotation(rx) * y_rotation(ry) * z_rotation(rz)

    corners3d = np.array([ [-.5,-.5,1,1],
                            [ .5,-.5,1,1],
                            [ .5, .5,1,1],
                            [-.5, .5,1,1] ]).T
    projected_corners = camera_matrix * corners3d
    projected_corners[:-1] /= projected_corners[-1]
    
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
    return compute_homography(square, projected_corners[:-1])

def random_camera_homography():
    p = np.random.normal(0,1,6) * [0.1, 0.1, 0.1, math.pi/16, math.pi/16, math.pi/16]
    return homography_from_camera(*p)
