import math

import numpy as np

def homogenize(pts):
    (h,w) = pts.shape
    results = np.empty((h+1,w))
    results[:h] = pts
    results[-1].fill(1)
    return results

def dehomogenize(pts):
    (h,w) = pts.shape
    results = np.empty((h-1,w))
    results[:h-1] = pts[:h-1]/pts[h-1]
    return results

def apply_to_pts(homography, pts):
    (h,w) = pts.shape
    result = np.empty((h+1,w))
    result[:h] = pts
    result[-1].fill(1)
    result = np.asmatrix(homography) * result
    result[:h] /= result[-1]
    return np.asarray(result[:h])

def square_to_corners_warp(corners):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T    
    return compute_homography(square, corners)

# Note: This is not doing the preconditioning step! Only important
# if we ever want to estimate homographies from more than 4 point
# correspondences.
def compute_homography(in_pts, out_pts):
    num_pts = in_pts.shape[1]
    in_pts = homogenize(in_pts)
    out_pts = homogenize(out_pts)
    constraint_matrix = np.empty((num_pts*2, 9))
    for i in xrange(num_pts):
        p = in_pts[:,i]
        q = out_pts[:,i]
        constraint_matrix[2*i,:] = np.concatenate([[0,0,0], -p, q[1]*p], axis=1)
        constraint_matrix[2*i+1,:] = np.concatenate([p, [0,0,0], -q[0]*p], axis=1)
    U,S,V = np.linalg.svd(constraint_matrix)
    homography = V[8].reshape((3,3))
    homography /= homography[2,2]
    return np.asmatrix(homography)

def random_homography(sigma_d, sigma_t):
    square = np.array([[-.5,-.5],[.5,-.5],[.5,.5],[-.5,.5]]).T
    disturbance = np.random.normal(0,sigma_d,(2,4)) + np.random.normal(0,sigma_t,(2,1))
    return compute_homography(square, square+disturbance)

def random_translation_and_scale(sigma_t, sigma_s):
    tx = np.random.normal(0, sigma_t)
    ty = np.random.normal(0, sigma_t)
    s = np.random.normal(1, sigma_s)
    return np.matrix([[1,0,tx],[0,1,ty],[0,0,1/s]])

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
